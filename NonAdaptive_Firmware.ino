// ============================================================
// PoseGuide — Non-Adaptive (Threshold) Firmware
// For thesis comparison: standard "if angle > threshold, buzz"
// logic typical of commercial posture wearables.
//
// Same hardware, same EMA filter, same BLE protocol, same
// calibration flow — only the decision logic differs.
//
// NO HMM, NO Bayesian belief, NO dual-gating, NO escalation.
// Just: deviation > threshold for N seconds → buzz.
// ============================================================

#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
#include <FlashStorage.h>

// ======================== PIN CONFIG ========================
const int VIBRATION_PIN = 2;

// ============ PERSISTENT CALIBRATION ============
typedef struct {
  boolean valid;
  float   gyro_bias;
  float   baseline_angle;
} CalibrationData;

FlashStorage(flash_calibration, CalibrationData);

float gyro_bias       = 0.0;
float baseline_angle  = 0.0;
bool  calibrated      = false;

// ====================== IMU / EMA (same as adaptive) ======================
float ema_pitch = 0.0;
float ema_gyro  = 0.0;
const float EMA_ALPHA = 0.15;
bool first_reading    = true;

// =================== SAMPLING =================== 
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 Hz
static unsigned long last_sample = 0;

// ===================== DEVICE MODE =====================
enum DeviceMode { MODE_CALIBRATING, MODE_MONITORING };
DeviceMode currentMode = MODE_MONITORING;

// ============ THRESHOLD CONTROLLER (the whole "algorithm") ============
// This is the standard approach used by most commercial posture devices.
//
// Rule: if |deviation| > SLOUCH_THRESHOLD for HOLD_TIME, buzz.
//       After buzzing, wait COOLDOWN before buzzing again.
//       If |deviation| < SLOUCH_THRESHOLD, reset.
//
// SLOUCH_THRESHOLD: midpoint between HMM upright mean (-0.99°)
//   and slouch mean (-19.66°) = ~10°. This catches the user
//   before they reach full slouch. Adjustable for testing.

const float SLOUCH_THRESHOLD      = 10.0;      // degrees deviation from baseline
const unsigned long HOLD_TIME_MS  = 3000;       // must exceed threshold for 3 seconds
const unsigned long COOLDOWN_MS   = 5000;       // 5 seconds between buzzes

unsigned long slouch_start_ms = 0;
bool          slouch_timing   = false;
unsigned long last_buzz_ms    = 0;
bool          alert_active    = false;

// ===================== BLE SERVICE =====================
BLEService postureService("00001810-0000-1000-8000-00805f9b34fb");

BLECharacteristic postureChar(
  "00002a35-0000-1000-8000-00805f9b34fb",
  BLERead | BLENotify, 20
);

BLECharacteristic calibrationChar(
  "00002a36-0000-1000-8000-00805f9b34fb",
  BLEWrite, 9
);

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);

  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    for (int i = 0; i < 5; i++) {
      digitalWrite(VIBRATION_PIN, HIGH); delay(200);
      digitalWrite(VIBRATION_PIN, LOW);  delay(200);
    }
    while (1);
  }

  // Load calibration from flash
  CalibrationData stored = flash_calibration.read();
  if (stored.valid) {
    gyro_bias      = stored.gyro_bias;
    baseline_angle = stored.baseline_angle;
    calibrated     = true;
    currentMode    = MODE_MONITORING;
    Serial.println("Loaded calibration from flash:");
    Serial.print("  gyro_bias: ");      Serial.println(gyro_bias, 4);
    Serial.print("  baseline_angle: "); Serial.println(baseline_angle, 2);
  } else {
    Serial.println("No stored calibration. Awaiting calibration via app.");
    calibrated  = false;
    currentMode = MODE_CALIBRATING;
  }

  // BLE Init
  if (BLE.begin()) {
    BLE.setLocalName("PoseGuide");
    BLE.setAdvertisedService(postureService);
    postureService.addCharacteristic(postureChar);
    postureService.addCharacteristic(calibrationChar);
    BLE.addService(postureService);
    calibrationChar.setEventHandler(BLEWritten, onCalibrationWritten);
    BLE.advertise();
    Serial.println("BLE advertising as PoseGuide");
  } else {
    Serial.println("BLE init failed — standalone mode only");
  }

  digitalWrite(VIBRATION_PIN, HIGH); delay(150);
  digitalWrite(VIBRATION_PIN, LOW);
  Serial.println("PoseGuide (Non-Adaptive) ready.");
}

// ============ BLE CALLBACK: Same calibration protocol ============
void onCalibrationWritten(BLEDevice central, BLECharacteristic characteristic) {
  const uint8_t* data = characteristic.value();
  int len = characteristic.valueLength();
  if (len < 1) return;

  uint8_t command = data[0];

  switch (command) {
    case 0x01: {
      currentMode   = MODE_CALIBRATING;
      first_reading = true;
      Serial.println("Entered CALIBRATION mode");
      break;
    }
    case 0x02:
    case 0x03: {
      if (len >= 9) {
        memcpy(&gyro_bias,      &data[1], 4);
        memcpy(&baseline_angle, &data[5], 4);
        calibrated = true;

        CalibrationData toStore;
        toStore.valid          = true;
        toStore.gyro_bias      = gyro_bias;
        toStore.baseline_angle = baseline_angle;
        flash_calibration.write(toStore);

        Serial.println("Calibration received and saved:");
        Serial.print("  gyro_bias: ");      Serial.println(gyro_bias, 4);
        Serial.print("  baseline_angle: "); Serial.println(baseline_angle, 2);

        // Reset EMA and threshold state
        ema_pitch      = baseline_angle;
        ema_gyro       = 0.0;
        first_reading  = false;
        slouch_timing  = false;
        alert_active   = false;

        currentMode = MODE_MONITORING;
        Serial.println("Entered MONITORING mode");

        digitalWrite(VIBRATION_PIN, HIGH); delay(100);
        digitalWrite(VIBRATION_PIN, LOW);  delay(80);
        digitalWrite(VIBRATION_PIN, HIGH); delay(100);
        digitalWrite(VIBRATION_PIN, LOW);
      }
      break;
    }
  }
}

// ======================== MAIN LOOP ========================
void loop() {
  BLE.poll();

  unsigned long now = millis();
  if (now - last_sample < SAMPLE_INTERVAL_MS) return;
  last_sample = now;

  // ---- Read IMU + EMA (identical to adaptive) ----
  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  float pitch_rad = atan2(ax, sqrt(ay * ay + az * az));
  float pitch_deg = pitch_rad * 180.0 / PI;
  float gyro_rate = gy - gyro_bias;

  if (first_reading) {
    ema_pitch     = pitch_deg;
    ema_gyro      = gyro_rate;
    first_reading = false;
  } else {
    ema_pitch = EMA_ALPHA * pitch_deg  + (1.0 - EMA_ALPHA) * ema_pitch;
    ema_gyro  = EMA_ALPHA * gyro_rate  + (1.0 - EMA_ALPHA) * ema_gyro;
  }

  float abs_gyro = fabs(ema_gyro);

  // ---- CALIBRATING ----
  if (currentMode == MODE_CALIBRATING) {
    uint8_t calPacket[9];
    memcpy(&calPacket[0], &ema_pitch, 4);
    memcpy(&calPacket[4], &gyro_rate, 4);
    calPacket[8] = 0x01;
    postureChar.writeValue(calPacket, 9);

    Serial.print("[CAL] ");
    Serial.print(now);          Serial.print(",");
    Serial.print(ema_pitch, 2); Serial.print(",");
    Serial.println(gyro_rate, 2);
    return;
  }

  // ---- MONITORING: Simple threshold logic ----
  float deviation = ema_pitch - baseline_angle;
  // deviation is negative when leaning forward (slouching)
  // so we check if deviation < -SLOUCH_THRESHOLD
  bool slouching = (deviation < -SLOUCH_THRESHOLD);

  int buzz_command = 0;

  if (slouching) {
    if (!slouch_timing) {
      // Start timing how long they've been past threshold
      slouch_timing   = true;
      slouch_start_ms = now;
    } else if (now - slouch_start_ms >= HOLD_TIME_MS) {
      // Been slouching long enough — fire alert
      alert_active = true;
      if (now - last_buzz_ms >= COOLDOWN_MS || last_buzz_ms == 0) {
        buzz_command = 1;  // Single buzz (no escalation in non-adaptive)
        last_buzz_ms = now;
      }
    }
  } else {
    // Back under threshold — reset everything immediately
    slouch_timing = false;
    alert_active  = false;
  }

  // Actuate
  if (buzz_command > 0) {
    digitalWrite(VIBRATION_PIN, HIGH); delay(200);
    digitalWrite(VIBRATION_PIN, LOW);
  }

  // Send BLE (same format as adaptive — app doesn't need to know)
  uint8_t packet[5];
  memcpy(packet, &ema_pitch, 4);
  packet[4] = (buzz_command > 0) ? 1 : 0;
  postureChar.writeValue(packet, 5);

  // Serial logging
  // Format: timestamp, angle, deviation, slouching, alert_active, buzz
  Serial.print(now);            Serial.print(",");
  Serial.print(ema_pitch, 2);   Serial.print(",");
  Serial.print(deviation, 2);   Serial.print(",");
  Serial.print(slouching);      Serial.print(",");
  Serial.print(alert_active);   Serial.print(",");
  Serial.println(buzz_command);
}