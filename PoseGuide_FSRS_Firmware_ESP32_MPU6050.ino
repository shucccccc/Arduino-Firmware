// ============================================================
// PoseGuide - FSRS-Inspired Adaptive Haptic Scheduler Firmware
// ESP32 + MPU6050 library + Grove Vibration Motor
// ============================================================
//
// Purpose:
//   Full FSRS-inspired PoseGuide firmware for the revised thesis.
//   This sketch keeps the working static firmware style, MPU6050
//   access pattern, ESP32 BLE stack, Preferences storage, serial
//   controls, and failsafe checks, but replaces the static alert
//   rule with the revised pipeline:
//
//     IMU -> EMA signal conditioning -> rolling-window features
//         -> logistic-regression p_slouch
//         -> motion gate for transient suppression
//         -> episode manager
//         -> FSRS-inspired D/S/R haptic scheduler
//         -> Grove vibration motor + BLE logging
//
// Hardware assumed from the current PoseGuide bench setup:
//   - ESP32 board
//   - MPU-6050 IMU using the MPU6050 library
//   - Grove vibration motor, active HIGH
//   - I2C pins: SDA=8, SCL=9
//   - Motor pin: GPIO0 by default
//
// Notes:
//   1. This file is Arduino IDE compatible as a single .ino sketch.
//   2. It intentionally uses the same MPU6050 library access as the
//      static comparator firmware:
//        mpu.initialize(), mpu.testConnection(), getAcceleration(),
//        getRotation().
//   3. The posture angle is computed in the same plane as the static
//      comparator:
//        angle = atan2(ay, az) * 180 / PI + 90
//      PoseGuide then uses baseline-relative deviation.
//   4. This implementation assumes forward slouch is NEGATIVE
//      baseline-relative deviation. If your wiring/mounting makes
//      forward slouch positive, set ANGLE_SIGN to -1.0f.
//   5. The logistic-regression coefficients below are engineering
//      starter coefficients so the firmware is immediately testable.
//      For thesis data collection, replace LR_B0..LR_B4 with the
//      exported coefficients from the cleaned/annotated training set.
//   6. BLE keeps backward-compatible posture and status packets and
//      adds a richer runtime packet for thesis logging.
//
// Requires in Arduino IDE:
//   - ESP32 board support (Arduino-ESP32)
//   - MPU6050 library matching the working static sketch
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
static const int VIBRATION_PIN = 0;   // Current working bench-test pin.

// If forward slouch becomes positive deviation, set this to -1.0f.
static const float ANGLE_SIGN = 1.0f;

// ======================== SAMPLING / FILTER ========================
static const unsigned long SAMPLE_INTERVAL_MS = 100;   // 10 Hz.
static const float EMA_ALPHA = 0.15f;

// ======================== ROLLING WINDOW ========================
static const uint8_t DEV_WINDOW_SIZE = 5;               // 0.5 s at 10 Hz.
static const float DEV_PERSIST_THRESHOLD_DEG = -8.0f;   // Provisional slouch-oriented reference.
static const float SAMPLE_DT_SEC = 0.1f;

// ======================== LOGISTIC-REGRESSION DETECTOR ========================
// Starter coefficients only. Replace with values exported from train_pose_lr.py.
// Features: x1=deviation, x2=window_mean_dev, x3=dev_persist, x4=window_std_dev.
// These coefficients assume slouch deviation is negative.
static const float LR_B0 = -4.50f;
static const float LR_B1 = -0.22f;
static const float LR_B2 = -0.12f;
static const float LR_B3 =  2.10f;
static const float LR_B4 = -0.08f;
static const float P_SLOUCH_ALPHA = 0.30f;

// Episode thresholds using smoothed p_slouch.
static const float THETA_ENTER = 0.65f;
static const float THETA_EXIT = 0.45f;
static const unsigned long T_CONFIRM_MS = 1000;
static const unsigned long T_RESET_MS = 1500;

// ======================== MOTION GATE ========================
static const float MOTION_ALPHA = 0.40f;
static const float MOTION_W_GYRO = 0.50f;
static const float MOTION_W_SLOPE = 0.30f;
static const float MOTION_W_STD = 0.20f;

// Normalization ceilings. Tune from development data if needed.
static const float NORM_ABS_GYRO_MAX_DPS = 45.0f;
static const float NORM_ABS_SLOPE_MAX_DPS = 40.0f;
static const float NORM_STD_MAX_DEG = 8.0f;

static const float THETA_MOVE_ON = 0.60f;
static const float THETA_MOVE_OFF = 0.40f;
static const uint8_t MOTION_ON_SAMPLES = 2;    // 0.2 s.
static const uint8_t MOTION_OFF_SAMPLES = 4;   // 0.4 s.

// ======================== FSRS-INSPIRED SCHEDULER ========================
// D: difficulty, S: stability seconds, R: retrievability.
static const float FSRS_D_INIT = 5.0f;
static const float FSRS_S_INIT_SEC = 3.0f;
static const float FSRS_RHO_TARGET = 0.90f;
static const float FSRS_KD = 0.25f;

static const float FSRS_D_MIN = 1.0f;
static const float FSRS_D_MAX = 10.0f;
static const float FSRS_S_MIN_SEC = 1.5f;
static const float FSRS_S_MAX_SEC = 12.0f;
static const float FSRS_I_MIN_SEC = 1.5f;
static const float FSRS_I_MAX_SEC = 12.0f;

static const unsigned long T_COOLDOWN_MS = 2000;
static const unsigned long T_RESPONSE_MS = 4000;
static const unsigned long T_MAX_RESPONSE_MS = 12000;
static const unsigned long T_FORCE_ALERT_MS = 15000;
static const unsigned long T_RELAPSE_WINDOW_MS = 2000;

static const float SEVERE_DEV_THRESHOLD_DEG = -15.0f;
static const float SEVERE_PROB_THRESHOLD = 0.90f;
static const unsigned long T_SEVERE_HOLD_MS = 1000;

// ======================== HAPTIC OUTPUT ========================
static const unsigned long ALERT_SINGLE_PULSE_MS = 200;
static const unsigned long ALERT_DOUBLE_PULSE_MS = 150;
static const unsigned long ALERT_DOUBLE_GAP_MS = 100;

// ======================== FAILSAFE ========================
static const uint8_t MAX_CONSEC_READ_FAILS = 3;
static const uint16_t STUCK_IDENTICAL_SAMPLES = 25;      // 2.5 s.
static const uint16_t IMPLAUSIBLE_SAMPLES = 10;          // 1.0 s.
static const float ACCEL_MAG_MIN_G = 0.50f;
static const float ACCEL_MAG_MAX_G = 1.50f;
static const float MAX_ABS_GYRO_BIAS_DPS = 20.0f;
static const float MAX_ABS_BASELINE_DEG = 180.0f;
static const float MAX_CAL_GYRO_STD_DPS = 2.0f;
static const float MAX_CAL_ANGLE_STD_DEG = 3.0f;
static const unsigned long SELF_CAL_DURATION_MS = 3000;  // Change to 10000 for research QC if desired.

// ======================== BLE UUIDS ========================
static const char *DEVICE_NAME = "PoseGuide-FSRS";
static const char *SERVICE_UUID = "00001810-0000-1000-8000-00805f9b34fb";
static const char *POSTURE_CHAR_UUID = "00002a35-0000-1000-8000-00805f9b34fb";
static const char *CALIBRATION_CHAR_UUID = "00002a36-0000-1000-8000-00805f9b34fb";
static const char *STATUS_CHAR_UUID = "00002a37-0000-1000-8000-00805f9b34fb";
static const char *RUNTIME_CHAR_UUID = "0000ffaa-0000-1000-8000-00805f9b34fb";

// ======================== CONTROL COMMANDS ========================
static const uint8_t CMD_ENTER_CALIBRATION = 0x01;
static const uint8_t CMD_WRITE_CALIBRATION = 0x02;
static const uint8_t CMD_RECALIBRATE       = 0x03;
static const uint8_t CMD_START_FSRS        = 0x10; // Same value as old START_STATIC for app compatibility.
static const uint8_t CMD_STOP_IDLE         = 0x11;
static const uint8_t CMD_CLEAR_FAULT       = 0x12;
static const uint8_t CMD_START_SILENT      = 0x13;
static const uint8_t CMD_RESET_SCHEDULER   = 0x14;

// ======================== TYPES ========================
enum DeviceMode : uint8_t {
  MODE_IDLE = 0,
  MODE_CALIBRATING = 1,
  MODE_MONITOR_SILENT = 2,
  MODE_MONITOR_FSRS = 3
};

enum MotionGateState : uint8_t {
  MOTION_CLOSED = 0,
  MOTION_OPEN = 1
};

enum EpisodeState : uint8_t {
  EP_UPRIGHT_STABLE = 0,
  EP_SLOUCH_CANDIDATE = 1,
  EP_SLOUCH_STABLE = 2,
  EP_RECOVERING = 3,
  EP_TRANSIENT_PAUSED = 4
};

enum EventMarker : uint8_t {
  EVENT_NONE = 0,
  EVENT_EPISODE_STARTED = 1,
  EVENT_EPISODE_RECOVERED = 2,
  EVENT_TRANSIENT_PAUSE = 3,
  EVENT_TRANSIENT_RESUME = 4,
  EVENT_FORCED_FAIL = 5,
  EVENT_SEVERE_OVERRIDE = 6,
  EVENT_RELAPSE_DOWNGRADE = 7
};

enum OutcomeGrade : uint8_t {
  OUTCOME_NONE = 0,
  OUTCOME_EASY = 1,
  OUTCOME_GOOD = 2,
  OUTCOME_HARD = 3,
  OUTCOME_FAIL = 4
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
  ERR_STORAGE_FAIL = 8,
  ERR_MODEL_NUMERIC = 9
};

struct CalibrationData {
  bool valid;
  float gyro_bias_dps;
  float baseline_angle_deg;
};

// ======================== GLOBALS ========================
MPU6050 mpu;
Preferences prefs;

DeviceMode currentMode = MODE_IDLE;
EpisodeState episodeState = EP_UPRIGHT_STABLE;
EpisodeState preTransientEpisodeState = EP_UPRIGHT_STABLE;
MotionGateState motionGate = MOTION_CLOSED;
EventMarker lastEventMarker = EVENT_NONE;
OutcomeGrade lastOutcome = OUTCOME_NONE;

bool imuAvailable = false;
bool calibrated = false;

float gyroBiasDps = 0.0f;       // Raw gx bias from calibration.
float baselineAngleDeg = 0.0f;

float emaAngleDeg = 0.0f;
float emaGyroDps = 0.0f;        // Bias-corrected gx axis after EMA.
bool firstReading = true;

float lastDeviationDeg = 0.0f;
float lastAngleDeg = 0.0f;
float lastRawGyroXDps = 0.0f;
float lastCorrectedGyroDps = 0.0f;
float lastAbsGyroDps = 0.0f;
float lastAccelMagG = 1.0f;

float devWindow[DEV_WINDOW_SIZE];
uint8_t devWindowIndex = 0;
uint8_t devWindowCount = 0;
float windowMeanDev = 0.0f;
float windowStdDev = 0.0f;
float devSlope = 0.0f;
float devPersist = 0.0f;

float pSlouchRaw = 0.0f;
float pSlouch = 0.0f;
float motionScoreRaw = 0.0f;
float motionScore = 0.0f;
uint8_t motionOnCount = 0;
uint8_t motionOffCount = 0;

unsigned long lastSampleMs = 0;
unsigned long lastStatusNotifyMs = 0;
unsigned long sessionStartMs = 0;

unsigned long candidateStartMs = 0;
unsigned long episodeStartMs = 0;
unsigned long recoveryStartMs = 0;
unsigned long lastRecoveryMs = 0;
bool hasSuccessfulRecovery = false;
unsigned long severeStartMs = 0;

float fsrsDifficultyD = FSRS_D_INIT;
float fsrsStabilityS = FSRS_S_INIT_SEC;
float fsrsRetrievabilityR = 1.0f;
float currentAlertIntervalSec = FSRS_S_INIT_SEC;
unsigned long nextAlertTimeMs = 0;

bool firstAlertSent = false;
bool outcomeAppliedThisEpisode = false;
unsigned long firstAlertTimeMs = 0;
unsigned long lastBuzzMs = 0;
uint8_t repeatCount = 0;
uint8_t lastAlertCode = 0;

bool pendingOutcomeActive = false;
OutcomeGrade pendingOutcomeGrade = OUTCOME_NONE;
unsigned long pendingOutcomeTimeMs = 0;

// Non-blocking vibration pattern state.
bool motorPatternActive = false;
uint8_t motorPatternCode = 0;
uint8_t motorPatternStage = 0;
unsigned long motorStageUntilMs = 0;

ErrorCode transientError = ERR_NONE;
ErrorCode latchedError = ERR_NONE;
bool hardFaultLatched = false;
uint8_t consecutiveReadFails = 0;
uint16_t identicalSampleCount = 0;
uint16_t implausibleSampleCount = 0;

int16_t prevRawAx = 0, prevRawAy = 0, prevRawAz = 0;
int16_t prevRawGx = 0, prevRawGy = 0, prevRawGz = 0;
bool havePrevRawSample = false;

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
BLECharacteristic *runtimeChar = nullptr;
bool bleDeviceConnected = false;

// ======================== HELPERS ========================
float clampf_local(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

float safeExpf(float x) {
  if (x < -30.0f) x = -30.0f;
  if (x > 30.0f) x = 30.0f;
  return expf(x);
}

float sigmoidf_local(float z) {
  z = clampf_local(z, -30.0f, 30.0f);
  return 1.0f / (1.0f + expf(-z));
}

float norm01(float value, float maxValue) {
  if (maxValue <= 0.0f) return 0.0f;
  return clampf_local(value / maxValue, 0.0f, 1.0f);
}

unsigned long secondsToMs(float secondsValue) {
  if (secondsValue <= 0.0f) return 0;
  return (unsigned long)(secondsValue * 1000.0f + 0.5f);
}

float elapsedSec(unsigned long nowMs, unsigned long originMs) {
  return ((float)(nowMs - originMs)) / 1000.0f;
}

const char *outcomeName(OutcomeGrade grade) {
  switch (grade) {
    case OUTCOME_EASY: return "EASY";
    case OUTCOME_GOOD: return "GOOD";
    case OUTCOME_HARD: return "HARD";
    case OUTCOME_FAIL: return "FAIL";
    default: return "NONE";
  }
}

ErrorCode activeError() {
  if (hardFaultLatched) return latchedError;
  if (!calibrated) return ERR_CALIBRATION_REQUIRED;
  return transientError;
}

bool sensorEvidenceHealthy() {
  if (!calibrated || !imuAvailable || hardFaultLatched) return false;
  if (transientError == ERR_IMU_READ_FAIL) return false;
  if (transientError == ERR_SENSOR_STUCK) return false;
  if (transientError == ERR_IMPLAUSIBLE_DATA) return false;
  if (transientError == ERR_MODEL_NUMERIC) return false;
  return true;
}

bool alertsAllowed() {
  return sensorEvidenceHealthy() && currentMode == MODE_MONITOR_FSRS;
}

void setSoftError(ErrorCode code) {
  if (!hardFaultLatched) transientError = code;
}

void clearSoftErrorIf(ErrorCode code) {
  if (!hardFaultLatched && transientError == code) {
    transientError = calibrated ? ERR_NONE : ERR_CALIBRATION_REQUIRED;
  }
}

void latchHardFault(ErrorCode code) {
  hardFaultLatched = true;
  latchedError = code;
  digitalWrite(VIBRATION_PIN, LOW);
  motorPatternActive = false;
  episodeState = EP_UPRIGHT_STABLE;
  candidateStartMs = 0;
  episodeStartMs = 0;
  recoveryStartMs = 0;
  nextAlertTimeMs = 0;
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

// ======================== VIBRATION MOTOR ========================
void startMotorPattern(uint8_t alertCode) {
  if (alertCode == 0) return;
  if (motorPatternActive) return;

  unsigned long nowMs = millis();
  lastAlertCode = alertCode;
  motorPatternCode = alertCode;
  motorPatternActive = true;
  motorPatternStage = 1;

  digitalWrite(VIBRATION_PIN, HIGH);
  if (alertCode == 1) {
    motorStageUntilMs = nowMs + ALERT_SINGLE_PULSE_MS;
  } else {
    motorStageUntilMs = nowMs + ALERT_DOUBLE_PULSE_MS;
  }
}

void serviceMotorPattern(unsigned long nowMs) {
  if (!motorPatternActive) return;
  if (nowMs < motorStageUntilMs) return;

  if (motorPatternCode == 1) {
    digitalWrite(VIBRATION_PIN, LOW);
    motorPatternActive = false;
    motorPatternStage = 0;
    return;
  }

  if (motorPatternCode == 2) {
    if (motorPatternStage == 1) {
      digitalWrite(VIBRATION_PIN, LOW);
      motorPatternStage = 2;
      motorStageUntilMs = nowMs + ALERT_DOUBLE_GAP_MS;
    } else if (motorPatternStage == 2) {
      digitalWrite(VIBRATION_PIN, HIGH);
      motorPatternStage = 3;
      motorStageUntilMs = nowMs + ALERT_DOUBLE_PULSE_MS;
    } else {
      digitalWrite(VIBRATION_PIN, LOW);
      motorPatternActive = false;
      motorPatternStage = 0;
    }
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

float computeAngleDegFromAccel(int16_t ay, int16_t az) {
  // Same geometry as the working static comparator sketch.
  float angle = atan2((float)ay, (float)az) * 180.0f / PI;
  angle += 90.0f;
  angle *= ANGLE_SIGN;
  return angle;
}

float computeGyroXDps(int16_t gxRaw) {
  return ((float)gxRaw) / 131.0f; // Default +/-250 dps scale for MPU-6050.
}

// ======================== RUNTIME RESET ========================
void resetRollingWindow() {
  for (uint8_t i = 0; i < DEV_WINDOW_SIZE; i++) {
    devWindow[i] = 0.0f;
  }
  devWindowIndex = 0;
  devWindowCount = 0;
  windowMeanDev = 0.0f;
  windowStdDev = 0.0f;
  devSlope = 0.0f;
  devPersist = 0.0f;
}

void resetDetectorAndGate() {
  pSlouchRaw = 0.0f;
  pSlouch = 0.0f;
  motionScoreRaw = 0.0f;
  motionScore = 0.0f;
  motionGate = MOTION_CLOSED;
  motionOnCount = 0;
  motionOffCount = 0;
}

void resetEpisodeState() {
  episodeState = EP_UPRIGHT_STABLE;
  preTransientEpisodeState = EP_UPRIGHT_STABLE;
  lastEventMarker = EVENT_NONE;
  candidateStartMs = 0;
  episodeStartMs = 0;
  recoveryStartMs = 0;
  severeStartMs = 0;
}

void resetSchedulerState(unsigned long nowMs) {
  fsrsDifficultyD = FSRS_D_INIT;
  fsrsStabilityS = FSRS_S_INIT_SEC;
  fsrsRetrievabilityR = 1.0f;
  currentAlertIntervalSec = FSRS_S_INIT_SEC;
  nextAlertTimeMs = 0;
  firstAlertSent = false;
  outcomeAppliedThisEpisode = false;
  firstAlertTimeMs = 0;
  lastBuzzMs = 0;
  repeatCount = 0;
  pendingOutcomeActive = false;
  pendingOutcomeGrade = OUTCOME_NONE;
  pendingOutcomeTimeMs = 0;
  lastOutcome = OUTCOME_NONE;
  sessionStartMs = nowMs;
  lastRecoveryMs = nowMs;
  hasSuccessfulRecovery = false;
}

void resetRuntimeState(unsigned long nowMs, bool resetScheduler) {
  firstReading = true;
  emaAngleDeg = baselineAngleDeg;
  emaGyroDps = 0.0f;
  lastDeviationDeg = 0.0f;
  lastAbsGyroDps = 0.0f;
  lastAlertCode = 0;
  resetRollingWindow();
  resetDetectorAndGate();
  resetEpisodeState();
  if (resetScheduler) {
    resetSchedulerState(nowMs);
  }
}

// ======================== CALIBRATION PERSISTENCE ========================
bool saveCalibrationToPrefs(float gyroBiasDegPerSec, float baselineAngleDegValue) {
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
  clearRecoverableFaults();
  resetRuntimeState(millis(), true);
  if (persist) saveCalibrationToPrefs(gyroBiasDegPerSec, baselineAngleDegValue);
}

void invalidateCalibration() {
  calibrated = false;
  prefs.putBool("valid", false);
  currentMode = MODE_IDLE;
  resetRuntimeState(millis(), true);
  setSoftError(ERR_CALIBRATION_REQUIRED);
}

// ======================== SIGNAL CONDITIONING ========================
void pushDeviation(float value) {
  devWindow[devWindowIndex] = value;
  devWindowIndex = (uint8_t)((devWindowIndex + 1) % DEV_WINDOW_SIZE);
  if (devWindowCount < DEV_WINDOW_SIZE) devWindowCount++;
}

float getOldestDeviation() {
  if (devWindowCount == 0) return 0.0f;
  if (devWindowCount < DEV_WINDOW_SIZE) return devWindow[0];
  return devWindow[devWindowIndex];
}

float getLatestDeviation() {
  if (devWindowCount == 0) return 0.0f;
  uint8_t latestIndex = (uint8_t)((devWindowIndex + DEV_WINDOW_SIZE - 1) % DEV_WINDOW_SIZE);
  return devWindow[latestIndex];
}

void computeRollingFeatures() {
  if (devWindowCount == 0) {
    windowMeanDev = 0.0f;
    windowStdDev = 0.0f;
    devSlope = 0.0f;
    devPersist = 0.0f;
    return;
  }

  float sum = 0.0f;
  uint8_t persistCount = 0;
  for (uint8_t i = 0; i < devWindowCount; i++) {
    sum += devWindow[i];
    if (devWindow[i] <= DEV_PERSIST_THRESHOLD_DEG) persistCount++;
  }
  windowMeanDev = sum / (float)devWindowCount;

  float varSum = 0.0f;
  for (uint8_t i = 0; i < devWindowCount; i++) {
    float diff = devWindow[i] - windowMeanDev;
    varSum += diff * diff;
  }
  windowStdDev = sqrtf(varSum / (float)devWindowCount);

  if (devWindowCount >= 2) {
    float oldest = getOldestDeviation();
    float latest = getLatestDeviation();
    float timeSpan = ((float)(devWindowCount - 1)) * SAMPLE_DT_SEC;
    devSlope = (latest - oldest) / timeSpan;
  } else {
    devSlope = 0.0f;
  }

  devPersist = (float)persistCount / (float)devWindowCount;
}

void updateSignalConditioning(float rawAngleDeg, float rawGyroXDps) {
  float correctedGyroDps = rawGyroXDps - gyroBiasDps;
  lastCorrectedGyroDps = correctedGyroDps;

  if (firstReading) {
    emaAngleDeg = rawAngleDeg;
    emaGyroDps = correctedGyroDps;
    firstReading = false;
  } else {
    emaAngleDeg = EMA_ALPHA * rawAngleDeg + (1.0f - EMA_ALPHA) * emaAngleDeg;
    emaGyroDps = EMA_ALPHA * correctedGyroDps + (1.0f - EMA_ALPHA) * emaGyroDps;
  }

  lastDeviationDeg = emaAngleDeg - baselineAngleDeg;
  lastAbsGyroDps = fabs(emaGyroDps);

  pushDeviation(lastDeviationDeg);
  computeRollingFeatures();
}

// ======================== POSE LR ========================
void updatePoseLR() {
  float z = LR_B0
            + LR_B1 * lastDeviationDeg
            + LR_B2 * windowMeanDev
            + LR_B3 * devPersist
            + LR_B4 * windowStdDev;

  if (!isfinite(z)) {
    setSoftError(ERR_MODEL_NUMERIC);
    z = -30.0f;
  } else {
    clearSoftErrorIf(ERR_MODEL_NUMERIC);
  }

  pSlouchRaw = sigmoidf_local(z);
  pSlouch = P_SLOUCH_ALPHA * pSlouchRaw + (1.0f - P_SLOUCH_ALPHA) * pSlouch;
  pSlouch = clampf_local(pSlouch, 0.0f, 1.0f);
}

// ======================== MOTION GATE ========================
void updateMotionGate() {
  float g1 = norm01(lastAbsGyroDps, NORM_ABS_GYRO_MAX_DPS);
  float g2 = norm01(fabs(devSlope), NORM_ABS_SLOPE_MAX_DPS);
  float g3 = norm01(windowStdDev, NORM_STD_MAX_DEG);

  motionScoreRaw = MOTION_W_GYRO * g1 + MOTION_W_SLOPE * g2 + MOTION_W_STD * g3;
  motionScoreRaw = clampf_local(motionScoreRaw, 0.0f, 1.0f);
  motionScore = MOTION_ALPHA * motionScoreRaw + (1.0f - MOTION_ALPHA) * motionScore;
  motionScore = clampf_local(motionScore, 0.0f, 1.0f);

  if (motionScore >= THETA_MOVE_ON) {
    if (motionOnCount < 255) motionOnCount++;
    motionOffCount = 0;
  } else if (motionScore <= THETA_MOVE_OFF) {
    if (motionOffCount < 255) motionOffCount++;
    motionOnCount = 0;
  } else {
    motionOnCount = 0;
    motionOffCount = 0;
  }

  if (motionOnCount >= MOTION_ON_SAMPLES) {
    motionGate = MOTION_OPEN;
  }
  if (motionOffCount >= MOTION_OFF_SAMPLES) {
    motionGate = MOTION_CLOSED;
  }
}

// ======================== EPISODE MANAGER ========================
EventMarker updateEpisodeManager(unsigned long nowMs) {
  EventMarker event = EVENT_NONE;

  if (motionGate == MOTION_OPEN) {
    if (episodeState != EP_TRANSIENT_PAUSED) {
      preTransientEpisodeState = episodeState;
      episodeState = EP_TRANSIENT_PAUSED;
      event = EVENT_TRANSIENT_PAUSE;
    }
    return event;
  }

  if (episodeState == EP_TRANSIENT_PAUSED) {
    event = EVENT_TRANSIENT_RESUME;
    if (pSlouch >= THETA_ENTER) {
      episodeState = EP_SLOUCH_CANDIDATE;
      candidateStartMs = nowMs;
    } else {
      episodeState = EP_UPRIGHT_STABLE;
    }
    return event;
  }

  switch (episodeState) {
    case EP_UPRIGHT_STABLE:
      if (pSlouch >= THETA_ENTER) {
        episodeState = EP_SLOUCH_CANDIDATE;
        candidateStartMs = nowMs;
      }
      break;

    case EP_SLOUCH_CANDIDATE:
      if (pSlouch < THETA_ENTER) {
        episodeState = EP_UPRIGHT_STABLE;
        candidateStartMs = 0;
      } else if ((nowMs - candidateStartMs) >= T_CONFIRM_MS) {
        episodeState = EP_SLOUCH_STABLE;
        episodeStartMs = nowMs;
        severeStartMs = 0;
        event = EVENT_EPISODE_STARTED;
      }
      break;

    case EP_SLOUCH_STABLE:
      if (pSlouch <= THETA_EXIT) {
        episodeState = EP_RECOVERING;
        recoveryStartMs = nowMs;
      }
      break;

    case EP_RECOVERING:
      if (pSlouch > THETA_EXIT) {
        episodeState = EP_SLOUCH_STABLE;
        recoveryStartMs = 0;
      } else if ((nowMs - recoveryStartMs) >= T_RESET_MS) {
        episodeState = EP_UPRIGHT_STABLE;
        lastRecoveryMs = nowMs;
        hasSuccessfulRecovery = true;
        event = EVENT_EPISODE_RECOVERED;
      }
      break;

    case EP_TRANSIENT_PAUSED:
    default:
      episodeState = EP_UPRIGHT_STABLE;
      break;
  }

  return event;
}

// ======================== FSRS SCHEDULER ========================
float computeRetrievability(unsigned long nowMs) {
  unsigned long originMs = hasSuccessfulRecovery ? lastRecoveryMs : sessionStartMs;
  float elapsed = elapsedSec(nowMs, originMs);
  float stableS = clampf_local(fsrsStabilityS, FSRS_S_MIN_SEC, FSRS_S_MAX_SEC);
  float r = safeExpf(logf(0.9f) * elapsed / stableS);
  return clampf_local(r, 0.0f, 1.0f);
}

float computeFSRSIntervalSec() {
  float d = clampf_local(fsrsDifficultyD, FSRS_D_MIN, FSRS_D_MAX);
  float s = clampf_local(fsrsStabilityS, FSRS_S_MIN_SEC, FSRS_S_MAX_SEC);
  float difficultyScale = clampf_local(1.0f - FSRS_KD * (d - 5.0f) / 5.0f, 0.75f, 1.25f);
  float baseInterval = s * difficultyScale * (logf(FSRS_RHO_TARGET) / logf(0.9f));
  return clampf_local(baseInterval, FSRS_I_MIN_SEC, FSRS_I_MAX_SEC);
}

void applyFSRSOutcome(OutcomeGrade grade) {
  if (grade == OUTCOME_NONE) return;

  if (grade == OUTCOME_EASY) {
    fsrsStabilityS = clampf_local(fsrsStabilityS * 1.25f, FSRS_S_MIN_SEC, FSRS_S_MAX_SEC);
    fsrsDifficultyD = clampf_local(fsrsDifficultyD - 0.15f, FSRS_D_MIN, FSRS_D_MAX);
  } else if (grade == OUTCOME_GOOD) {
    fsrsStabilityS = clampf_local(fsrsStabilityS * 1.10f, FSRS_S_MIN_SEC, FSRS_S_MAX_SEC);
    fsrsDifficultyD = clampf_local(fsrsDifficultyD - 0.05f, FSRS_D_MIN, FSRS_D_MAX);
  } else if (grade == OUTCOME_HARD) {
    fsrsStabilityS = clampf_local(fsrsStabilityS * 0.95f, FSRS_S_MIN_SEC, FSRS_S_MAX_SEC);
    fsrsDifficultyD = clampf_local(fsrsDifficultyD + 0.10f, FSRS_D_MIN, FSRS_D_MAX);
  } else {
    fsrsStabilityS = clampf_local(fsrsStabilityS * 0.70f, FSRS_S_MIN_SEC, FSRS_S_MAX_SEC);
    fsrsDifficultyD = clampf_local(fsrsDifficultyD + 0.25f, FSRS_D_MIN, FSRS_D_MAX);
  }

  lastOutcome = grade;
  currentAlertIntervalSec = computeFSRSIntervalSec();
}

OutcomeGrade downgradeForRelapse(OutcomeGrade grade) {
  if (grade == OUTCOME_EASY || grade == OUTCOME_GOOD) return OUTCOME_HARD;
  if (grade == OUTCOME_HARD) return OUTCOME_FAIL;
  return OUTCOME_FAIL;
}

void queuePendingOutcome(OutcomeGrade grade, unsigned long nowMs) {
  if (grade == OUTCOME_NONE) return;
  pendingOutcomeActive = true;
  pendingOutcomeGrade = grade;
  pendingOutcomeTimeMs = nowMs;
}

void servicePendingOutcome(unsigned long nowMs) {
  if (!pendingOutcomeActive) return;
  if ((nowMs - pendingOutcomeTimeMs) >= T_RELAPSE_WINDOW_MS) {
    applyFSRSOutcome(pendingOutcomeGrade);
    pendingOutcomeActive = false;
    pendingOutcomeGrade = OUTCOME_NONE;
  }
}

void applyRelapseDowngradeIfPending(unsigned long nowMs) {
  if (!pendingOutcomeActive) return;
  if ((nowMs - pendingOutcomeTimeMs) <= T_RELAPSE_WINDOW_MS) {
    OutcomeGrade downgraded = downgradeForRelapse(pendingOutcomeGrade);
    applyFSRSOutcome(downgraded);
    pendingOutcomeActive = false;
    pendingOutcomeGrade = OUTCOME_NONE;
    lastEventMarker = EVENT_RELAPSE_DOWNGRADE;
  }
}

OutcomeGrade gradeRecovery(unsigned long nowMs) {
  if (outcomeAppliedThisEpisode) return OUTCOME_NONE;
  if (!firstAlertSent) return OUTCOME_EASY;

  unsigned long correctionLatency = nowMs - firstAlertTimeMs;
  if (repeatCount == 0 && correctionLatency <= T_RESPONSE_MS) return OUTCOME_GOOD;
  if (correctionLatency <= T_MAX_RESPONSE_MS) return OUTCOME_HARD;
  return OUTCOME_FAIL;
}

bool severeSlouchHeld(unsigned long nowMs) {
  bool severeNow = (lastDeviationDeg <= SEVERE_DEV_THRESHOLD_DEG) &&
                   (pSlouch >= SEVERE_PROB_THRESHOLD) &&
                   (motionGate == MOTION_CLOSED) &&
                   (episodeState == EP_SLOUCH_STABLE);

  if (severeNow) {
    if (severeStartMs == 0) severeStartMs = nowMs;
    return (nowMs - severeStartMs) >= T_SEVERE_HOLD_MS;
  }

  severeStartMs = 0;
  return false;
}

void markAlertSent(unsigned long nowMs, uint8_t alertCode) {
  if (!firstAlertSent) {
    firstAlertSent = true;
    firstAlertTimeMs = nowMs;
    repeatCount = 0;
  } else {
    if (repeatCount < 255) repeatCount++;
  }
  lastBuzzMs = nowMs;
  lastAlertCode = alertCode;
}

void fsrsSchedulerTick(unsigned long nowMs, EventMarker eventMarker, uint8_t &alertFlag, uint8_t &alertCode) {
  alertFlag = 0;
  alertCode = 0;

  servicePendingOutcome(nowMs);
  fsrsRetrievabilityR = computeRetrievability(nowMs);

  if (eventMarker == EVENT_EPISODE_STARTED) {
    applyRelapseDowngradeIfPending(nowMs);
    currentAlertIntervalSec = computeFSRSIntervalSec();
    nextAlertTimeMs = nowMs + secondsToMs(currentAlertIntervalSec);
    firstAlertSent = false;
    outcomeAppliedThisEpisode = false;
    firstAlertTimeMs = 0;
    repeatCount = 0;
    severeStartMs = 0;
  }

  if (eventMarker == EVENT_EPISODE_RECOVERED) {
    fsrsRetrievabilityR = 1.0f;
    OutcomeGrade grade = gradeRecovery(nowMs);
    if (grade != OUTCOME_NONE) {
      queuePendingOutcome(grade, nowMs);
    }
    firstAlertSent = false;
    outcomeAppliedThisEpisode = false;
    firstAlertTimeMs = 0;
    nextAlertTimeMs = 0;
    repeatCount = 0;
    return;
  }

  if (!alertsAllowed()) return;
  if (episodeState != EP_SLOUCH_STABLE) return;
  if (motionGate == MOTION_OPEN) return;

  // Severe deviation override: early first alert is allowed if stable and extreme.
  if (severeSlouchHeld(nowMs) && (nowMs - lastBuzzMs) >= T_COOLDOWN_MS) {
    alertFlag = 1;
    alertCode = firstAlertSent ? 2 : 1;
    markAlertSent(nowMs, alertCode);
    lastEventMarker = EVENT_SEVERE_OVERRIDE;
    return;
  }

  // First alert after the adaptive FSRS interval.
  if (!firstAlertSent) {
    if (nextAlertTimeMs == 0) {
      currentAlertIntervalSec = computeFSRSIntervalSec();
      nextAlertTimeMs = nowMs + secondsToMs(currentAlertIntervalSec);
    }

    bool scheduledAlertDue = nowMs >= nextAlertTimeMs;
    bool forceAlertDue = (episodeStartMs > 0) && ((nowMs - episodeStartMs) >= T_FORCE_ALERT_MS);

    if ((scheduledAlertDue || forceAlertDue) && (nowMs - lastBuzzMs) >= T_COOLDOWN_MS) {
      alertFlag = 1;
      alertCode = 1;
      markAlertSent(nowMs, alertCode);
      return;
    }
  } else {
    // If no correction occurs within the maximum response window, apply FAIL once.
    if (!outcomeAppliedThisEpisode && (nowMs - firstAlertTimeMs) >= T_MAX_RESPONSE_MS) {
      applyFSRSOutcome(OUTCOME_FAIL);
      outcomeAppliedThisEpisode = true;
      lastEventMarker = EVENT_FORCED_FAIL;
    }

    // Repeat alert after the acceptable response window, bounded by cooldown.
    if ((nowMs - firstAlertTimeMs) >= T_RESPONSE_MS && (nowMs - lastBuzzMs) >= T_COOLDOWN_MS) {
      alertFlag = 1;
      alertCode = 2;
      markAlertSent(nowMs, alertCode);
      return;
    }
  }
}

// ======================== FAILSAFE SAMPLE CHECKS ========================
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
  } else {
    clearSoftErrorIf(ERR_IMPLAUSIBLE_DATA);
  }

  if (identicalSampleCount >= STUCK_IDENTICAL_SAMPLES) {
    setSoftError(ERR_SENSOR_STUCK);
  } else {
    clearSoftErrorIf(ERR_SENSOR_STUCK);
  }
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

  // Backward-compatible posture packet: 5 bytes.
  // [0..3] emaAngleDeg float LE, [4] alert flag.
  uint8_t posturePacket[5];
  memcpy(&posturePacket[0], &emaAngleDeg, 4);
  posturePacket[4] = alertFlag;
  postureChar->setValue(posturePacket, sizeof(posturePacket));
  postureChar->notify();

  // Backward-compatible status packet: 12 bytes.
  // [0..3] deviation float LE, [4] error, [5] alertCode,
  // [6] mode, [7] episodeState, [8..11] timestamp uint32 LE.
  uint8_t statusPacket[12];
  memcpy(&statusPacket[0], &lastDeviationDeg, 4);
  statusPacket[4] = (uint8_t)activeError();
  statusPacket[5] = alertCode;
  statusPacket[6] = (uint8_t)currentMode;
  statusPacket[7] = (uint8_t)episodeState;
  uint32_t ts = (uint32_t)nowMs;
  memcpy(&statusPacket[8], &ts, 4);
  statusChar->setValue(statusPacket, sizeof(statusPacket));
  statusChar->notify();

  // Revised thesis runtime packet: 32 bytes.
  // [0..3] deviation, [4..7] p_slouch, [8..11] motion_score,
  // [12..15] R, [16..19] D, [20..23] S,
  // [24] motionGate, [25] episodeState, [26] lastOutcome,
  // [27] alertCode, [28..31] timestamp.
  if (runtimeChar) {
    uint8_t runtimePacket[32];
    memcpy(&runtimePacket[0], &lastDeviationDeg, 4);
    memcpy(&runtimePacket[4], &pSlouch, 4);
    memcpy(&runtimePacket[8], &motionScore, 4);
    memcpy(&runtimePacket[12], &fsrsRetrievabilityR, 4);
    memcpy(&runtimePacket[16], &fsrsDifficultyD, 4);
    memcpy(&runtimePacket[20], &fsrsStabilityS, 4);
    runtimePacket[24] = (uint8_t)motionGate;
    runtimePacket[25] = (uint8_t)episodeState;
    runtimePacket[26] = (uint8_t)lastOutcome;
    runtimePacket[27] = alertCode;
    memcpy(&runtimePacket[28], &ts, 4);
    runtimeChar->setValue(runtimePacket, sizeof(runtimePacket));
    runtimeChar->notify();
  }
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
  statusPacket[7] = (uint8_t)episodeState;
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
        resetDetectorAndGate();
        resetEpisodeState();
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
            currentMode = MODE_MONITOR_FSRS;
            sessionStartMs = millis();
          } else {
            setSoftError(ERR_CALIBRATION_UNSTABLE);
            currentMode = MODE_IDLE;
          }
        }
        break;

      case CMD_START_FSRS:
        if (calibrated) {
          resetRuntimeState(millis(), true);
          currentMode = MODE_MONITOR_FSRS;
        }
        break;

      case CMD_START_SILENT:
        if (calibrated) {
          resetRuntimeState(millis(), true);
          currentMode = MODE_MONITOR_SILENT;
        }
        break;

      case CMD_STOP_IDLE:
        currentMode = MODE_IDLE;
        motorPatternActive = false;
        digitalWrite(VIBRATION_PIN, LOW);
        break;

      case CMD_CLEAR_FAULT:
        clearRecoverableFaults();
        break;

      case CMD_RESET_SCHEDULER:
        resetSchedulerState(millis());
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

  runtimeChar = service->createCharacteristic(
    RUNTIME_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  runtimeChar->addDescriptor(new BLE2902());

  service->start();
  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

// ======================== SERIAL ========================
void printCsvHeader() {
  Serial.println(F("timestamp_ms,ema_angle_deg,deviation_deg,ema_gyro_dps,abs_gyro_dps,window_mean_dev,window_std_dev,dev_slope,dev_persist,p_slouch_raw,p_slouch,motion_score_raw,motion_score,motion_gate,episode_state,D,S,R,alert_interval_sec,alert_code,outcome,error,mode,event"));
}

void printHelp() {
  Serial.println(F("\nSerial commands:"));
  Serial.println(F("  c = self-calibrate (3 s still, upright)"));
  Serial.println(F("  s = start FSRS monitoring"));
  Serial.println(F("  n = start silent monitoring/logging"));
  Serial.println(F("  p = pause / idle"));
  Serial.println(F("  r = reset FSRS scheduler state"));
  Serial.println(F("  e = clear recoverable fault"));
  Serial.println(F("  i = print current status"));
  Serial.println(F("  h = help"));
}

void printStatus() {
  Serial.println(F("\n===== PoseGuide FSRS Status ====="));
  Serial.print(F("IMU available: " )); Serial.println(imuAvailable ? F("YES") : F("NO"));
  Serial.print(F("Calibrated:    " )); Serial.println(calibrated ? F("YES") : F("NO"));
  Serial.print(F("Mode:          " )); Serial.println((int)currentMode);
  Serial.print(F("Episode state: " )); Serial.println((int)episodeState);
  Serial.print(F("Motion gate:   " )); Serial.println((int)motionGate);
  Serial.print(F("Error code:    " )); Serial.println((int)activeError());
  Serial.print(F("Hard fault:    " )); Serial.println(hardFaultLatched ? F("YES") : F("NO"));
  Serial.print(F("gyroBiasDps:   " )); Serial.println(gyroBiasDps, 4);
  Serial.print(F("baselineAngle: " )); Serial.println(baselineAngleDeg, 2);
  Serial.print(F("emaAngleDeg:   " )); Serial.println(emaAngleDeg, 2);
  Serial.print(F("deviationDeg:  " )); Serial.println(lastDeviationDeg, 2);
  Serial.print(F("pSlouch:       " )); Serial.println(pSlouch, 4);
  Serial.print(F("motionScore:   " )); Serial.println(motionScore, 4);
  Serial.print(F("D:             " )); Serial.println(fsrsDifficultyD, 3);
  Serial.print(F("S:             " )); Serial.println(fsrsStabilityS, 3);
  Serial.print(F("R:             " )); Serial.println(fsrsRetrievabilityR, 3);
  Serial.print(F("intervalSec:   " )); Serial.println(currentAlertIntervalSec, 3);
  Serial.print(F("lastOutcome:   " )); Serial.println(outcomeName(lastOutcome));
  Serial.println(F("=================================\n"));
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
      resetDetectorAndGate();
      resetEpisodeState();
      Serial.println(F("Self-calibration started. Sit upright and hold still for 3 s."));
      break;

    case 's':
    case 'S':
      if (!calibrated) {
        Serial.println(F("Cannot start FSRS monitoring: calibration required."));
        return;
      }
      resetRuntimeState(millis(), true);
      currentMode = MODE_MONITOR_FSRS;
      Serial.println(F("FSRS monitoring started."));
      printCsvHeader();
      break;

    case 'n':
    case 'N':
      if (!calibrated) {
        Serial.println(F("Cannot start silent monitoring: calibration required."));
        return;
      }
      resetRuntimeState(millis(), true);
      currentMode = MODE_MONITOR_SILENT;
      Serial.println(F("Silent monitoring started."));
      printCsvHeader();
      break;

    case 'p':
    case 'P':
      currentMode = MODE_IDLE;
      motorPatternActive = false;
      digitalWrite(VIBRATION_PIN, LOW);
      Serial.println(F("Monitoring paused. Device is idle."));
      break;

    case 'r':
    case 'R':
      resetSchedulerState(millis());
      Serial.println(F("FSRS scheduler state reset."));
      break;

    case 'e':
    case 'E':
      clearRecoverableFaults();
      Serial.println(F("Recoverable faults cleared."));
      break;

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

  Serial.println(F("\nPoseGuide FSRS-Inspired Firmware (ESP32 + MPU6050 library)"));
  Serial.println(F("Arduino IDE compatible sketch."));
  Serial.print(F("I2C SDA=")); Serial.print(I2C_SDA_PIN);
  Serial.print(F(", SCL=")); Serial.println(I2C_SCL_PIN);
  Serial.print(F("Vibration pin=")); Serial.println(VIBRATION_PIN);
  Serial.println(F("Forward slouch is assumed to be negative baseline-relative deviation."));
  printHelp();
  printStatus();
}

// ======================== LOOP ========================
void loop() {
  handleSerial();
  serviceMotorPattern(millis());
  sendStatusHeartbeat(millis());

  if (bleDeviceConnected == false) {
    // Keep CPU relaxed while advertising after disconnect.
    delay(5);
  }

  unsigned long nowMs = millis();
  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }

  unsigned long loopStartMs = millis();
  lastSampleMs = nowMs;
  lastEventMarker = EVENT_NONE;

  if (!imuAvailable) {
    latchHardFault(ERR_IMU_INIT_FAIL);
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (!readMPU6050Raw(ax, ay, az, gx, gy, gz)) {
    consecutiveReadFails++;
    if (consecutiveReadFails >= MAX_CONSEC_READ_FAILS) {
      setSoftError(ERR_IMU_READ_FAIL);
    }
    return;
  }
  consecutiveReadFails = 0;
  clearSoftErrorIf(ERR_IMU_READ_FAIL);

  updateFailsafeSampleChecks(ax, ay, az, gx, gy, gz);

  float rawAngleDeg = computeAngleDegFromAccel(ay, az);
  float rawGyroXDps = computeGyroXDps(gx);
  lastAngleDeg = rawAngleDeg;
  lastRawGyroXDps = rawGyroXDps;

  // During calibration, stream the angle and raw gyro so the app can compute
  // baseline_angle and gyro_bias exactly like the static firmware.
  if (currentMode == MODE_CALIBRATING) {
    if (firstReading) {
      emaAngleDeg = rawAngleDeg;
      emaGyroDps = rawGyroXDps;
      firstReading = false;
    } else {
      emaAngleDeg = EMA_ALPHA * rawAngleDeg + (1.0f - EMA_ALPHA) * emaAngleDeg;
      emaGyroDps = EMA_ALPHA * rawGyroXDps + (1.0f - EMA_ALPHA) * emaGyroDps;
    }

    sendCalibrationPacket(emaAngleDeg, rawGyroXDps);

    if (serialSelfCalActive) {
      selfCalAngleSum += emaAngleDeg;
      selfCalAngleSqSum += (double)emaAngleDeg * (double)emaAngleDeg;
      selfCalGyroSum += rawGyroXDps;
      selfCalGyroSqSum += (double)rawGyroXDps * (double)rawGyroXDps;
      selfCalCount++;

      if (nowMs - selfCalStartMs >= SELF_CAL_DURATION_MS && selfCalCount > 5) {
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

  updateSignalConditioning(rawAngleDeg, rawGyroXDps);
  updatePoseLR();
  updateMotionGate();

  EventMarker event = updateEpisodeManager(nowMs);
  lastEventMarker = event;

  uint8_t alertFlag = 0;
  uint8_t alertCode = 0;

  if (currentMode == MODE_MONITOR_FSRS) {
    fsrsSchedulerTick(nowMs, event, alertFlag, alertCode);
    if (alertFlag == 1 && alertCode != 0) {
      startMotorPattern(alertCode);
    }
  } else {
    // Silent and idle modes compute and log state, but do not alert.
    alertFlag = 0;
    alertCode = 0;
  }

  if (!alertsAllowed() && motorPatternActive) {
    digitalWrite(VIBRATION_PIN, LOW);
    motorPatternActive = false;
  }

  fsrsRetrievabilityR = computeRetrievability(nowMs);
  sendMonitoringPackets(nowMs, alertFlag, alertCode);

  // Serial CSV for thesis debugging/logging.
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(emaAngleDeg, 3);
  Serial.print(',');
  Serial.print(lastDeviationDeg, 3);
  Serial.print(',');
  Serial.print(emaGyroDps, 3);
  Serial.print(',');
  Serial.print(lastAbsGyroDps, 3);
  Serial.print(',');
  Serial.print(windowMeanDev, 3);
  Serial.print(',');
  Serial.print(windowStdDev, 3);
  Serial.print(',');
  Serial.print(devSlope, 3);
  Serial.print(',');
  Serial.print(devPersist, 3);
  Serial.print(',');
  Serial.print(pSlouchRaw, 4);
  Serial.print(',');
  Serial.print(pSlouch, 4);
  Serial.print(',');
  Serial.print(motionScoreRaw, 4);
  Serial.print(',');
  Serial.print(motionScore, 4);
  Serial.print(',');
  Serial.print((int)motionGate);
  Serial.print(',');
  Serial.print((int)episodeState);
  Serial.print(',');
  Serial.print(fsrsDifficultyD, 3);
  Serial.print(',');
  Serial.print(fsrsStabilityS, 3);
  Serial.print(',');
  Serial.print(fsrsRetrievabilityR, 4);
  Serial.print(',');
  Serial.print(currentAlertIntervalSec, 3);
  Serial.print(',');
  Serial.print((int)alertCode);
  Serial.print(',');
  Serial.print((int)lastOutcome);
  Serial.print(',');
  Serial.print((int)activeError());
  Serial.print(',');
  Serial.print((int)currentMode);
  Serial.print(',');
  Serial.println((int)lastEventMarker);

  unsigned long loopElapsed = millis() - loopStartMs;
  if (loopElapsed > SAMPLE_INTERVAL_MS) {
    setSoftError(ERR_TIMING_OVERRUN);
  } else {
    clearSoftErrorIf(ERR_TIMING_OVERRUN);
  }
}
