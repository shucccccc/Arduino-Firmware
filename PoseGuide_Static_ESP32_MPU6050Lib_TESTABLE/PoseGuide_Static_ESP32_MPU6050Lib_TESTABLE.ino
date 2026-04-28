// ============================================================
// PoseGuide — Static Comparator Firmware
// ESP32 + MPU6050 library + Grove Vibration Motor
// ============================================================
//
// Purpose:
//   Static (non-adaptive) comparator firmware for the revised
//   PoseGuide thesis.
//
// Static rule (revised comparator):
//   deviation = ema_angle - baseline_angle
//   IF deviation <= -10 deg for 3 seconds
//   THEN buzz every 2 seconds until deviation recovers
//
// Hardware assumed from your working bench test:
//   - ESP32 board
//   - MPU6050 IMU using the MPU6050 library
//   - Grove vibration motor (active HIGH)
//   - I2C pins: SDA=8, SCL=9
//   - Motor pin: GPIO0 (works, but is a boot-strapping pin; move later if needed)
//
// Notes:
//   1. This file is Arduino IDE compatible as an .ino sketch.
//   2. It uses the same style of MPU6050 library access as your simple test code:
//        mpu.initialize(), mpu.testConnection(), getAcceleration(), getRotation()
//   3. Angle is computed in the same plane as your test sketch:
//        angle = atan2(ay, az) * 180 / PI + 90
//      Then PoseGuide uses baseline-relative deviation instead of a hard absolute range.
//   4. Gyro bias is estimated on the matched rotational axis (gx) during calibration.
//      Static mode does not use gyro for alerting, but it is used for calibration QC and BLE.
//   5. BLE is included for app compatibility and fault/status reporting.
//   6. TESTABLE version adds safe Serial fault-injection commands for alpha testing.
//
// Requires in Arduino IDE:
//   - ESP32 board support (Arduino-ESP32)
//   - MPU6050 library matching your current test sketch
//
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MPU6050.h>
#include <math.h>

// ======================== USER CONFIG ========================
static const int I2C_SDA_PIN = 8;
static const int I2C_SCL_PIN = 9;
static const int VIBRATION_PIN = 0;   // your current working test pin

// If your slouch direction is inverted, set this to -1.0f.
static const float ANGLE_SIGN = 1.0f;

// ======================== SAMPLING / FILTER ========================
static const unsigned long SAMPLE_INTERVAL_MS = 100;   // 10 Hz
static const float EMA_ALPHA = 0.15f;

// ======================== STATIC COMPARATOR ========================
static const float STATIC_SLOUCH_THRESHOLD_DEG = -10.0f;
static const unsigned long STATIC_HOLD_MS = 3000;
static const unsigned long STATIC_REPEAT_MS = 2000;
static const unsigned long ALERT_PULSE_MS = 200;

// ======================== FAILSAFE ========================
static const uint8_t MAX_CONSEC_READ_FAILS = 3;
static const uint16_t STUCK_IDENTICAL_SAMPLES = 25;      // 2.5 s
static const uint16_t IMPLAUSIBLE_SAMPLES = 10;          // 1.0 s
static const float ACCEL_MAG_MIN_G = 0.50f;
static const float ACCEL_MAG_MAX_G = 1.50f;
static const float MAX_ABS_GYRO_BIAS_DPS = 20.0f;
static const float MAX_ABS_BASELINE_DEG = 180.0f;
static const float MAX_CAL_GYRO_STD_DPS = 2.0f;
static const float MAX_CAL_ANGLE_STD_DEG = 3.0f;

// ======================== SAFE FAULT-INJECTION TEST HARNESS ========================
// Keep this enabled during alpha/bench testing. Set to 0 for participant/live testing.
#define POSEGUIDE_ENABLE_FAULT_INJECTION 1

// ======================== BLE UUIDS ========================
static const char *DEVICE_NAME = "PoseGuide-Static";
static const char *SERVICE_UUID = "00001810-0000-1000-8000-00805f9b34fb";
static const char *POSTURE_CHAR_UUID = "00002a35-0000-1000-8000-00805f9b34fb";
static const char *CALIBRATION_CHAR_UUID = "00002a36-0000-1000-8000-00805f9b34fb";
static const char *STATUS_CHAR_UUID = "00002a37-0000-1000-8000-00805f9b34fb";

// ======================== CONTROL COMMANDS ========================
static const uint8_t CMD_ENTER_CALIBRATION = 0x01;
static const uint8_t CMD_WRITE_CALIBRATION = 0x02;
static const uint8_t CMD_RECALIBRATE       = 0x03;
static const uint8_t CMD_START_STATIC      = 0x10;
static const uint8_t CMD_STOP_IDLE         = 0x11;
static const uint8_t CMD_CLEAR_FAULT       = 0x12;
static const uint8_t CMD_START_SILENT      = 0x13;

// ======================== TYPES ========================
enum DeviceMode : uint8_t {
  MODE_IDLE = 0,
  MODE_CALIBRATING = 1,
  MODE_MONITOR_SILENT = 2,
  MODE_MONITOR_STATIC = 3
};

enum StaticState : uint8_t {
  STATIC_UPRIGHT = 0,
  STATIC_CANDIDATE = 1,
  STATIC_ALERTING = 2
};

enum ErrorCode : uint8_t {
  ERR_NONE = 0,
  ERR_IMU_INIT_FAIL = 1,
  ERR_IMU_READ_FAIL = 2,
  ERR_CALIBRATION_REQUIRED = 3,
  ERR_CALIBRATION_UNSTABLE = 4,
  ERR_SENSOR_STUCK = 5,
  ERR_IMPLAUSIBLE_DATA = 6,
  ERR_TIMING_OVERRUN = 7,
  ERR_STORAGE_FAIL = 8
};

#if POSEGUIDE_ENABLE_FAULT_INJECTION
enum InjectedFaultMode : uint8_t {
  TEST_FAULT_NONE = 0,
  TEST_FAULT_READ_FAIL = 1,
  TEST_FAULT_STUCK_SENSOR = 2,
  TEST_FAULT_IMPLAUSIBLE_ACCEL = 3,
  TEST_FAULT_TIMING_OVERRUN = 4,
  TEST_FAULT_STORAGE_FAIL = 5
};
#endif

struct CalibrationData {
  bool valid;
  float gyro_bias_dps;
  float baseline_angle_deg;
};

// ======================== GLOBALS ========================
MPU6050 mpu;
Preferences prefs;

DeviceMode currentMode = MODE_IDLE;
StaticState staticState = STATIC_UPRIGHT;

bool imuAvailable = false;
bool calibrated = false;

float gyroBiasDps = 0.0f;      // from gx axis, matched to your angle plane
float baselineAngleDeg = 0.0f;

float emaAngleDeg = 0.0f;
float emaGyroDps = 0.0f;
bool firstReading = true;

float lastDeviationDeg = 0.0f;
float lastAngleDeg = 0.0f;
float lastGyroXDps = 0.0f;
float lastAccelMagG = 1.0f;

unsigned long lastSampleMs = 0;
unsigned long lastStatusNotifyMs = 0;

unsigned long staticSlouchStartMs = 0;
unsigned long lastBuzzMs = 0;
uint8_t lastAlertCode = 0;

bool motorActive = false;
unsigned long motorOffAtMs = 0;

ErrorCode transientError = ERR_NONE;
ErrorCode latchedError = ERR_NONE;
bool hardFaultLatched = false;
uint8_t consecutiveReadFails = 0;
uint16_t identicalSampleCount = 0;
uint16_t implausibleSampleCount = 0;

int16_t prevRawAx = 0, prevRawAy = 0, prevRawAz = 0;
int16_t prevRawGx = 0, prevRawGy = 0, prevRawGz = 0;
bool havePrevRawSample = false;

#if POSEGUIDE_ENABLE_FAULT_INJECTION
InjectedFaultMode injectedFaultMode = TEST_FAULT_NONE;
bool injectedStuckHaveSample = false;
int16_t injectedStuckAx = 0, injectedStuckAy = 0, injectedStuckAz = 0;
int16_t injectedStuckGx = 0, injectedStuckGy = 0, injectedStuckGz = 0;
#endif

bool serialSelfCalActive = false;
unsigned long selfCalStartMs = 0;
uint32_t selfCalCount = 0;
double selfCalAngleSum = 0.0;
double selfCalAngleSqSum = 0.0;
double selfCalGyroSum = 0.0;
double selfCalGyroSqSum = 0.0;

BLEServer *bleServer = nullptr;
BLECharacteristic *postureChar = nullptr;
BLECharacteristic *calibrationChar = nullptr;
BLECharacteristic *statusChar = nullptr;
bool bleDeviceConnected = false;

// ======================== HELPERS ========================
float clampf_local(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

ErrorCode activeError() {
  if (hardFaultLatched) return latchedError;
  if (!calibrated) return ERR_CALIBRATION_REQUIRED;
  return transientError;
}

bool unsafeForFeedback(ErrorCode err) {
  return err == ERR_IMU_INIT_FAIL ||
         err == ERR_IMU_READ_FAIL ||
         err == ERR_CALIBRATION_REQUIRED ||
         err == ERR_CALIBRATION_UNSTABLE ||
         err == ERR_SENSOR_STUCK ||
         err == ERR_IMPLAUSIBLE_DATA ||
         err == ERR_TIMING_OVERRUN;
}

bool alertsAllowed() {
  ErrorCode err = activeError();
  return calibrated &&
         imuAvailable &&
         !hardFaultLatched &&
         !unsafeForFeedback(err) &&
         currentMode == MODE_MONITOR_STATIC;
}

void setSoftError(ErrorCode code) {
  if (!hardFaultLatched) transientError = code;
}

void latchHardFault(ErrorCode code) {
  hardFaultLatched = true;
  latchedError = code;
  digitalWrite(VIBRATION_PIN, LOW);
  motorActive = false;
  staticState = STATIC_UPRIGHT;
  staticSlouchStartMs = 0;
  lastBuzzMs = 0;
}

void clearRecoverableFaults() {
  if (latchedError != ERR_IMU_INIT_FAIL) {
    hardFaultLatched = false;
    latchedError = ERR_NONE;
  }
  transientError = calibrated ? ERR_NONE : ERR_CALIBRATION_REQUIRED;
  consecutiveReadFails = 0;
  identicalSampleCount = 0;
  implausibleSampleCount = 0;
  havePrevRawSample = false;
}

void startMotorPulse(uint8_t alertCode) {
  digitalWrite(VIBRATION_PIN, HIGH);
  motorActive = true;
  motorOffAtMs = millis() + ALERT_PULSE_MS;
  lastAlertCode = alertCode;
}

void serviceMotorPulse(unsigned long nowMs) {
  if (motorActive && nowMs >= motorOffAtMs) {
    digitalWrite(VIBRATION_PIN, LOW);
    motorActive = false;
  }
}

// ======================== SENSOR ========================
bool initMPU6050() {
  mpu.initialize();
  return mpu.testConnection();
}

bool readMPU6050Raw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Basic sanity: if everything is exactly zero, treat as a read failure.
  if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
    return false;
  }
  return true;
}

// ======================== CALIBRATION PERSISTENCE ========================
bool saveCalibrationToPrefs(float gyroBiasDegPerSec, float baselineAngleDegValue) {
#if POSEGUIDE_ENABLE_FAULT_INJECTION
  if (injectedFaultMode == TEST_FAULT_STORAGE_FAIL) {
    setSoftError(ERR_STORAGE_FAIL);
    Serial.println(F("Injected storage fault: calibration intentionally not saved."));
    return false;
  }
#endif
  size_t w0 = prefs.putBool("valid", true);
  size_t w1 = prefs.putFloat("gBias", gyroBiasDegPerSec);
  size_t w2 = prefs.putFloat("bAngle", baselineAngleDegValue);
  if (w0 == 0 || w1 == 0 || w2 == 0) {
    setSoftError(ERR_STORAGE_FAIL);
    return false;
  }
  return true;
}

CalibrationData loadCalibrationFromPrefs() {
  CalibrationData data;
  data.valid = prefs.getBool("valid", false);
  data.gyro_bias_dps = prefs.getFloat("gBias", 0.0f);
  data.baseline_angle_deg = prefs.getFloat("bAngle", 0.0f);
  return data;
}

bool validateCalibrationValues(float gyroBiasDegPerSec, float baselineAngleDegValue) {
  if (!isfinite(gyroBiasDegPerSec) || !isfinite(baselineAngleDegValue)) return false;
  if (fabs(gyroBiasDegPerSec) > MAX_ABS_GYRO_BIAS_DPS) return false;
  if (fabs(baselineAngleDegValue) > MAX_ABS_BASELINE_DEG) return false;
  return true;
}

void applyCalibration(float gyroBiasDegPerSec, float baselineAngleDegValue, bool persist) {
  gyroBiasDps = gyroBiasDegPerSec;
  baselineAngleDeg = baselineAngleDegValue;
  calibrated = true;
  firstReading = true;
  emaAngleDeg = baselineAngleDeg;
  emaGyroDps = 0.0f;
  staticState = STATIC_UPRIGHT;
  staticSlouchStartMs = 0;
  lastBuzzMs = 0;
  clearRecoverableFaults();
  if (persist) saveCalibrationToPrefs(gyroBiasDegPerSec, baselineAngleDegValue);
}

void invalidateCalibration() {
  calibrated = false;
  prefs.putBool("valid", false);
  currentMode = MODE_IDLE;
  staticState = STATIC_UPRIGHT;
  staticSlouchStartMs = 0;
  lastBuzzMs = 0;
}

// ======================== BLE ========================
class PoseGuideServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    bleDeviceConnected = true;
  }
  void onDisconnect(BLEServer *server) override {
    (void)server;
    bleDeviceConnected = false;
    delay(500);
    BLEDevice::startAdvertising();
    Serial.println(F("Device disconnected. Restarting BLE advertising..."));
  }
};

void sendCalibrationPacket(float angleDeg, float gyroXDps) {
  if (!postureChar || !bleDeviceConnected) return;
  uint8_t packet[9];
  memcpy(&packet[0], &angleDeg, 4);
  memcpy(&packet[4], &gyroXDps, 4);
  packet[8] = 0x01;
  postureChar->setValue(packet, sizeof(packet));
  postureChar->notify();
}

void sendMonitoringPackets(unsigned long nowMs, uint8_t alertFlag, uint8_t alertCode) {
  if (!bleDeviceConnected || !postureChar || !statusChar) return;

  uint8_t posturePacket[5];
  memcpy(&posturePacket[0], &emaAngleDeg, 4);
  posturePacket[4] = alertFlag;
  postureChar->setValue(posturePacket, sizeof(posturePacket));
  postureChar->notify();

  uint8_t statusPacket[12];
  memcpy(&statusPacket[0], &lastDeviationDeg, 4);
  statusPacket[4] = (uint8_t)activeError();
  statusPacket[5] = alertCode;
  statusPacket[6] = (uint8_t)currentMode;
  statusPacket[7] = (uint8_t)staticState;
  uint32_t ts = (uint32_t)nowMs;
  memcpy(&statusPacket[8], &ts, 4);
  statusChar->setValue(statusPacket, sizeof(statusPacket));
  statusChar->notify();
}

void sendStatusHeartbeat(unsigned long nowMs) {
  if (!bleDeviceConnected || !statusChar) return;
  if (nowMs - lastStatusNotifyMs < 500) return;
  lastStatusNotifyMs = nowMs;

  uint8_t statusPacket[12];
  memcpy(&statusPacket[0], &lastDeviationDeg, 4);
  statusPacket[4] = (uint8_t)activeError();
  statusPacket[5] = lastAlertCode;
  statusPacket[6] = (uint8_t)currentMode;
  statusPacket[7] = (uint8_t)staticState;
  uint32_t ts = (uint32_t)nowMs;
  memcpy(&statusPacket[8], &ts, 4);
  statusChar->setValue(statusPacket, sizeof(statusPacket));
  statusChar->notify();
}

class PoseGuideControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    size_t len = characteristic->getLength();
    if (len == 0) return;

    uint8_t *data = characteristic->getData();
    if (!data) return;
    uint8_t command = data[0];

    switch (command) {
      case CMD_ENTER_CALIBRATION:
        serialSelfCalActive = false;
        currentMode = MODE_CALIBRATING;
        firstReading = true;
        break;

      case CMD_WRITE_CALIBRATION:
      case CMD_RECALIBRATE:
        if (len >= 9) {
          float incomingGyroBias = 0.0f;
          float incomingBaseline = 0.0f;
          memcpy(&incomingGyroBias, &data[1], 4);
          memcpy(&incomingBaseline, &data[5], 4);
          if (validateCalibrationValues(incomingGyroBias, incomingBaseline)) {
            applyCalibration(incomingGyroBias, incomingBaseline, true);
            currentMode = MODE_MONITOR_STATIC;
          } else {
            setSoftError(ERR_CALIBRATION_UNSTABLE);
            currentMode = MODE_IDLE;
          }
        }
        break;

      case CMD_START_STATIC:
        if (calibrated) currentMode = MODE_MONITOR_STATIC;
        break;

      case CMD_START_SILENT:
        if (calibrated) currentMode = MODE_MONITOR_SILENT;
        break;

      case CMD_STOP_IDLE:
        currentMode = MODE_IDLE;
        motorActive = false;
        digitalWrite(VIBRATION_PIN, LOW);
        break;

      case CMD_CLEAR_FAULT:
        clearRecoverableFaults();
        break;

      default:
        break;
    }
  }
};

void setupBLE() {
  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new PoseGuideServerCallbacks());

  BLEService *service = bleServer->createService(SERVICE_UUID);

  postureChar = service->createCharacteristic(
    POSTURE_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  postureChar->addDescriptor(new BLE2902());

  calibrationChar = service->createCharacteristic(
    CALIBRATION_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  calibrationChar->setCallbacks(new PoseGuideControlCallbacks());

  statusChar = service->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  statusChar->addDescriptor(new BLE2902());

  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}


#if POSEGUIDE_ENABLE_FAULT_INJECTION
const __FlashStringHelper *injectedFaultName() {
  switch (injectedFaultMode) {
    case TEST_FAULT_NONE: return F("NONE");
    case TEST_FAULT_READ_FAIL: return F("READ_FAIL");
    case TEST_FAULT_STUCK_SENSOR: return F("STUCK_SENSOR");
    case TEST_FAULT_IMPLAUSIBLE_ACCEL: return F("IMPLAUSIBLE_ACCEL");
    case TEST_FAULT_TIMING_OVERRUN: return F("TIMING_OVERRUN");
    case TEST_FAULT_STORAGE_FAIL: return F("STORAGE_FAIL");
    default: return F("UNKNOWN");
  }
}

void setInjectedFault(InjectedFaultMode mode) {
  injectedFaultMode = mode;
  injectedStuckHaveSample = false;
  clearRecoverableFaults();
  Serial.print(F("Injected fault mode = "));
  Serial.println(injectedFaultName());
}

void clearInjectedFaultMode() {
  injectedFaultMode = TEST_FAULT_NONE;
  injectedStuckHaveSample = false;
  clearRecoverableFaults();
  Serial.println(F("Injected fault mode cleared."));
}

bool applyInjectedFaults(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  switch (injectedFaultMode) {
    case TEST_FAULT_READ_FAIL:
      // Simulates repeated bad I2C/MPU reads without unplugging anything.
      return true;

    case TEST_FAULT_STUCK_SENSOR:
      // Freezes raw samples so the normal stuck-sensor detector trips after 25 samples.
      if (!injectedStuckHaveSample) {
        injectedStuckAx = ax; injectedStuckAy = ay; injectedStuckAz = az;
        injectedStuckGx = gx; injectedStuckGy = gy; injectedStuckGz = gz;
        injectedStuckHaveSample = true;
      } else {
        ax = injectedStuckAx; ay = injectedStuckAy; az = injectedStuckAz;
        gx = injectedStuckGx; gy = injectedStuckGy; gz = injectedStuckGz;
      }
      return false;

    case TEST_FAULT_IMPLAUSIBLE_ACCEL:
      // Makes accel magnitude about 2 g so the implausible-data detector trips after 10 samples.
      ax = 32767;
      ay = 0;
      az = 0;
      // keep gyro nonzero-ish so it does not look like an all-zero read failure
      gx = 10;
      gy = 20;
      gz = 30;
      return false;

    case TEST_FAULT_TIMING_OVERRUN:
      // Forces the 100 ms loop-budget check to fail.
      delay(SAMPLE_INTERVAL_MS + 30);
      return false;

    case TEST_FAULT_STORAGE_FAIL:
      // Report-only unless you recalibrate; saveCalibrationToPrefs() also catches this.
      setSoftError(ERR_STORAGE_FAIL);
      return false;

    case TEST_FAULT_NONE:
    default:
      return false;
  }
}
#endif

// ======================== SERIAL ========================
void printHelp() {
  Serial.println(F("\nSerial commands:"));
  Serial.println(F("  c = self-calibrate (3 s still, upright)"));
  Serial.println(F("  s = start static monitoring"));
  Serial.println(F("  n = start silent monitoring"));
  Serial.println(F("  p = pause / idle"));
  Serial.println(F("  e = clear recoverable fault"));
  Serial.println(F("  i = print current status"));
  Serial.println(F("  h = help"));
#if POSEGUIDE_ENABLE_FAULT_INJECTION
  Serial.println(F(""));
  Serial.println(F("Safe fault-injection test commands:"));
  Serial.println(F("  0 = clear injected fault + clear recoverable faults"));
  Serial.println(F("  1 = inject IMU read failure"));
  Serial.println(F("  2 = inject frozen/stuck sensor stream"));
  Serial.println(F("  3 = inject implausible accelerometer magnitude"));
  Serial.println(F("  4 = inject timing overrun"));
  Serial.println(F("  5 = inject storage-save fault"));
  Serial.println(F("  6 = invalidate calibration, then require recalibration"));
#endif
}

void printStatus() {
  Serial.println(F("\n===== PoseGuide Static Status ====="));
  Serial.print(F("IMU available: ")); Serial.println(imuAvailable ? F("YES") : F("NO"));
  Serial.print(F("Calibrated:    ")); Serial.println(calibrated ? F("YES") : F("NO"));
  Serial.print(F("Mode:          ")); Serial.println((int)currentMode);
  Serial.print(F("Static state:  ")); Serial.println((int)staticState);
  Serial.print(F("Error code:    ")); Serial.println((int)activeError());
  Serial.print(F("Hard fault:    ")); Serial.println(hardFaultLatched ? F("YES") : F("NO"));
#if POSEGUIDE_ENABLE_FAULT_INJECTION
  Serial.print(F("Injected mode: ")); Serial.println(injectedFaultName());
#endif
  Serial.print(F("gyroBiasDps:   ")); Serial.println(gyroBiasDps, 4);
  Serial.print(F("baselineAngle: ")); Serial.println(baselineAngleDeg, 2);
  Serial.print(F("emaAngleDeg:   ")); Serial.println(emaAngleDeg, 2);
  Serial.print(F("deviationDeg:  ")); Serial.println(lastDeviationDeg, 2);
  Serial.println(F("===================================\n"));
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = (char)Serial.read();
  while (Serial.available()) Serial.read();

  switch (c) {
    case 'c':
    case 'C':
      if (!imuAvailable) {
        Serial.println(F("Cannot self-calibrate: IMU unavailable."));
        return;
      }
      serialSelfCalActive = true;
      currentMode = MODE_CALIBRATING;
      selfCalStartMs = millis();
      selfCalCount = 0;
      selfCalAngleSum = 0.0;
      selfCalAngleSqSum = 0.0;
      selfCalGyroSum = 0.0;
      selfCalGyroSqSum = 0.0;
      firstReading = true;
      Serial.println(F("Self-calibration started. Sit upright and hold still for 3 s."));
      break;

    case 's':
    case 'S':
      if (!calibrated) {
        Serial.println(F("Cannot start static monitoring: calibration required."));
        return;
      }
      currentMode = MODE_MONITOR_STATIC;
      Serial.println(F("Static monitoring started."));
      break;

    case 'n':
    case 'N':
      if (!calibrated) {
        Serial.println(F("Cannot start silent monitoring: calibration required."));
        return;
      }
      currentMode = MODE_MONITOR_SILENT;
      Serial.println(F("Silent monitoring started."));
      break;

    case 'p':
    case 'P':
      currentMode = MODE_IDLE;
      motorActive = false;
      digitalWrite(VIBRATION_PIN, LOW);
      Serial.println(F("Monitoring paused. Device is idle."));
      break;

    case 'e':
    case 'E':
      clearRecoverableFaults();
      Serial.println(F("Recoverable faults cleared."));
      break;

#if POSEGUIDE_ENABLE_FAULT_INJECTION
    case '0':
      clearInjectedFaultMode();
      break;

    case '1':
      setInjectedFault(TEST_FAULT_READ_FAIL);
      break;

    case '2':
      setInjectedFault(TEST_FAULT_STUCK_SENSOR);
      break;

    case '3':
      setInjectedFault(TEST_FAULT_IMPLAUSIBLE_ACCEL);
      break;

    case '4':
      setInjectedFault(TEST_FAULT_TIMING_OVERRUN);
      break;

    case '5':
      setInjectedFault(TEST_FAULT_STORAGE_FAIL);
      setSoftError(ERR_STORAGE_FAIL);
      break;

    case '6':
      clearInjectedFaultMode();
      invalidateCalibration();
      setSoftError(ERR_CALIBRATION_REQUIRED);
      Serial.println(F("Calibration invalidated. This safely tests calibration-required behavior."));
      break;
#endif

    case 'i':
    case 'I':
      printStatus();
      break;

    case 'h':
    case 'H':
    default:
      printHelp();
      break;
  }
}

// ======================== SIGNAL PROCESSING ========================
float computeAngleDegFromAccel(int16_t ay, int16_t az) {
  // Same geometry as your simple working test sketch.
  float angle = atan2((float)ay, (float)az) * 180.0f / PI;
  angle += 90.0f;
  angle *= ANGLE_SIGN;
  return angle;
}

float computeGyroXDps(int16_t gxRaw) {
  return ((float)gxRaw) / 131.0f; // default +/-250 dps scale
}

void updateFailsafeSampleChecks(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  float axg = (float)ax / 16384.0f;
  float ayg = (float)ay / 16384.0f;
  float azg = (float)az / 16384.0f;
  lastAccelMagG = sqrtf(axg * axg + ayg * ayg + azg * azg);

  bool implausible = !isfinite(lastAccelMagG) || lastAccelMagG < ACCEL_MAG_MIN_G || lastAccelMagG > ACCEL_MAG_MAX_G;
  if (implausible) {
    implausibleSampleCount++;
  } else {
    if (implausibleSampleCount > 0) implausibleSampleCount--;
  }

  if (havePrevRawSample &&
      ax == prevRawAx && ay == prevRawAy && az == prevRawAz &&
      gx == prevRawGx && gy == prevRawGy && gz == prevRawGz) {
    identicalSampleCount++;
  } else {
    identicalSampleCount = 0;
  }

  prevRawAx = ax; prevRawAy = ay; prevRawAz = az;
  prevRawGx = gx; prevRawGy = gy; prevRawGz = gz;
  havePrevRawSample = true;

  if (implausibleSampleCount >= IMPLAUSIBLE_SAMPLES) {
    setSoftError(ERR_IMPLAUSIBLE_DATA);
  }
  if (identicalSampleCount >= STUCK_IDENTICAL_SAMPLES) {
    setSoftError(ERR_SENSOR_STUCK);
  }
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  prefs.begin("poseguide", false);

  imuAvailable = initMPU6050();
  if (!imuAvailable) {
    latchHardFault(ERR_IMU_INIT_FAIL);
  }

  setupBLE();

  CalibrationData cal = loadCalibrationFromPrefs();
  if (cal.valid && validateCalibrationValues(cal.gyro_bias_dps, cal.baseline_angle_deg)) {
    applyCalibration(cal.gyro_bias_dps, cal.baseline_angle_deg, false);
    currentMode = MODE_IDLE;
  } else {
    calibrated = false;
    setSoftError(ERR_CALIBRATION_REQUIRED);
    currentMode = MODE_IDLE;
  }

  Serial.println(F("\nPoseGuide Static Comparator (ESP32 + MPU6050 library)"));
  Serial.println(F("Arduino IDE compatible sketch."));
  Serial.print(F("I2C SDA=")); Serial.print(I2C_SDA_PIN);
  Serial.print(F(", SCL=")); Serial.println(I2C_SCL_PIN);
  Serial.print(F("Vibration pin=")); Serial.println(VIBRATION_PIN);
  printHelp();
  printStatus();
}

// ======================== LOOP ========================
void loop() {
  handleSerial();
  serviceMotorPulse(millis());
  sendStatusHeartbeat(millis());

  if (bleDeviceConnected == false) {
    // keep advertising after disconnect
    delay(5);
  }

  unsigned long nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  unsigned long loopStartMs = millis();
  lastSampleMs = nowMs;

  if (!imuAvailable) {
    latchHardFault(ERR_IMU_INIT_FAIL);
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;
  bool readOk = readMPU6050Raw(ax, ay, az, gx, gy, gz);

#if POSEGUIDE_ENABLE_FAULT_INJECTION
  if (readOk && applyInjectedFaults(ax, ay, az, gx, gy, gz)) {
    readOk = false;
  }
#endif

  if (!readOk) {
    consecutiveReadFails++;
    if (consecutiveReadFails >= MAX_CONSEC_READ_FAILS) {
      setSoftError(ERR_IMU_READ_FAIL);
    }
    sendStatusHeartbeat(nowMs);
    return;
  }
  consecutiveReadFails = 0;

  updateFailsafeSampleChecks(ax, ay, az, gx, gy, gz);

  float rawAngleDeg = computeAngleDegFromAccel(ay, az);
  float rawGyroXDps = computeGyroXDps(gx);
  lastAngleDeg = rawAngleDeg;
  lastGyroXDps = rawGyroXDps;

  if (firstReading) {
    emaAngleDeg = rawAngleDeg;
    emaGyroDps = rawGyroXDps;
    firstReading = false;
  } else {
    emaAngleDeg = EMA_ALPHA * rawAngleDeg + (1.0f - EMA_ALPHA) * emaAngleDeg;
    emaGyroDps = EMA_ALPHA * rawGyroXDps + (1.0f - EMA_ALPHA) * emaGyroDps;
  }

  if (currentMode == MODE_CALIBRATING) {
    sendCalibrationPacket(emaAngleDeg, rawGyroXDps);

    if (serialSelfCalActive) {
      selfCalAngleSum += emaAngleDeg;
      selfCalAngleSqSum += (double)emaAngleDeg * (double)emaAngleDeg;
      selfCalGyroSum += rawGyroXDps;
      selfCalGyroSqSum += (double)rawGyroXDps * (double)rawGyroXDps;
      selfCalCount++;

      if (nowMs - selfCalStartMs >= 3000 && selfCalCount > 5) {
        double n = (double)selfCalCount;
        float meanAngle = (float)(selfCalAngleSum / n);
        float meanGyro = (float)(selfCalGyroSum / n);
        float varAngle = (float)((selfCalAngleSqSum / n) - ((selfCalAngleSum / n) * (selfCalAngleSum / n)));
        float varGyro = (float)((selfCalGyroSqSum / n) - ((selfCalGyroSum / n) * (selfCalGyroSum / n)));
        float stdAngle = sqrtf(varAngle < 0.0f ? 0.0f : varAngle);
        float stdGyro = sqrtf(varGyro < 0.0f ? 0.0f : varGyro);

        if (stdAngle > MAX_CAL_ANGLE_STD_DEG || stdGyro > MAX_CAL_GYRO_STD_DPS ||
            !validateCalibrationValues(meanGyro, meanAngle)) {
          setSoftError(ERR_CALIBRATION_UNSTABLE);
          currentMode = MODE_IDLE;
          serialSelfCalActive = false;
          Serial.println(F("Calibration failed: unstable or implausible calibration window."));
        } else {
          applyCalibration(meanGyro, meanAngle, true);
          currentMode = MODE_IDLE;
          serialSelfCalActive = false;
          Serial.print(F("Calibration OK. baseline=")); Serial.print(meanAngle, 2);
          Serial.print(F(", gyroBias=")); Serial.println(meanGyro, 4);
        }
      }
    }
    return;
  }

  if (!calibrated) {
    setSoftError(ERR_CALIBRATION_REQUIRED);
    return;
  }

  lastDeviationDeg = emaAngleDeg - baselineAngleDeg;

  uint8_t alertFlag = 0;
  uint8_t alertCode = 0;

  if (alertsAllowed()) {
    if (lastDeviationDeg <= STATIC_SLOUCH_THRESHOLD_DEG) {
      if (staticState == STATIC_UPRIGHT) {
        staticState = STATIC_CANDIDATE;
        staticSlouchStartMs = nowMs;
      }

      if ((nowMs - staticSlouchStartMs) >= STATIC_HOLD_MS) {
        staticState = STATIC_ALERTING;
        if (lastBuzzMs == 0 || (nowMs - lastBuzzMs) >= STATIC_REPEAT_MS) {
          startMotorPulse(1);
          lastBuzzMs = nowMs;
          alertFlag = 1;
          alertCode = 1;
        }
      }
    } else {
      staticState = STATIC_UPRIGHT;
      staticSlouchStartMs = 0;
      // keep lastBuzzMs for spacing history? no, reset for a clean new episode
      lastBuzzMs = 0;
    }
  } else {
    staticState = STATIC_UPRIGHT;
    staticSlouchStartMs = 0;
    if (currentMode != MODE_MONITOR_STATIC) {
      // silent / idle modes do not alert
      lastBuzzMs = 0;
    }
  }

  sendMonitoringPackets(nowMs, alertFlag, alertCode);

  // Serial debug output for testing.
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(emaAngleDeg, 3);
  Serial.print(',');
  Serial.print(lastDeviationDeg, 3);
  Serial.print(',');
  Serial.print(emaGyroDps, 3);
  Serial.print(',');
  Serial.print((int)activeError());
  Serial.print(',');
  Serial.print((int)currentMode);
  Serial.print(',');
  Serial.print((int)staticState);
  Serial.print(',');
  Serial.println((int)alertFlag);

  unsigned long loopElapsed = millis() - loopStartMs;
  if (loopElapsed > SAMPLE_INTERVAL_MS) {
    setSoftError(ERR_TIMING_OVERRUN);
  }
}
