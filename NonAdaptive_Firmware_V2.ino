// ============================================================
// PoseGuide — Non-Adaptive (Static Threshold) Firmware
// For thesis comparison against the adaptive dual-gated system.
//
// Replicates the standard commercial posture wearable approach:
//   - Single angular deviation threshold
//   - Fixed hold duration before alerting
//   - Repeating buzz until posture is corrected
//   - No gyroscope data, no probabilistic inference
//   - No transient suppression, no escalation
//
// Same hardware, same EMA filter, same BLE protocol, same
// calibration flow — only the decision logic differs.
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

// ====================== IMU / EMA ======================
// Same EMA filter as adaptive — ensures differences in results
// are attributable to the decision logic, not signal conditioning.
float ema_pitch = 0.0;
float ema_gyro  = 0.0;              // Filtered but NOT used for decisions
const float EMA_ALPHA = 0.15;
bool first_reading    = true;

// =================== SAMPLING ===================
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 Hz
static unsigned long last_sample = 0;

// ===================== DEVICE MODE =====================
enum DeviceMode { MODE_CALIBRATING, MODE_MONITORING };
DeviceMode currentMode = MODE_MONITORING;

// ============ STATIC THRESHOLD CONTROLLER ============
// Replicates the standard commercial approach:
//
// 1. Compute angular deviation from calibrated baseline
// 2. If |deviation| > threshold for HOLD_TIME → start buzzing
// 3. Keep buzzing (every BUZZ_REPEAT_INTERVAL) until fixed
// 4. If |deviation| < threshold → immediately stop buzzing
//
// SLOUCH_THRESHOLD: derived from the midpoint between the
//   HMM-learned upright centroid (-0.99°) and slouch centroid
//   (-19.66°). The midpoint is approximately -10.3°.
//   Rounded to 10° for implementation.
//
// HOLD_TIME: 3-second delay before first alert. Comparable to
//   the default "sensitivity" setting on commercial devices
//   such as Upright GO (which offers 1s, 3s, 5s, 15s, 60s).
//
// BUZZ_REPEAT_INTERVAL: 2 seconds between repeated buzzes.
//   Keeps reminding the user until they correct posture.
//   Aggressive by design — this is how static systems work,
//   and is the source of user fatigue that the adaptive
//   system aims to address.

const float SLOUCH_THRESHOLD            = 10.0;    // degrees from baseline
const unsigned long HOLD_TIME_MS        = 3000;    // 3 seconds before first buzz
const unsigned long BUZZ_REPEAT_MS      = 2000;    // buzz every 2s while slouching
const unsigned long BUZZ_DURATION_MS    = 200;     // each buzz lasts 200ms

// State tracking
unsigned long slouch_start_ms = 0;
bool          slouch_timing   = false;    // Are we counting the hold time?
bool          alert_active    = false;    // Has the hold time been met?
unsigned long last_buzz_ms    = 0;

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

// ============ BLE CALLBACK ============
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

  // ---- Read IMU + EMA (identical to adaptive firmware) ----
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

  // ================================================================
  // MONITORING: Static Threshold Logic
  //
  // This is the ENTIRE decision algorithm. Compare to the adaptive
  // firmware which runs HMM + Bayesian + dual-gating at this point.
  //
  // Rule: deviation < -THRESHOLD for HOLD_TIME → buzz repeatedly
  //       deviation ≥ -THRESHOLD → stop immediately
  // ================================================================

  float deviation = ema_pitch - baseline_angle;
  bool slouching = (deviation < -SLOUCH_THRESHOLD);
  int buzz_command = 0;

  if (slouching) {
    if (!slouch_timing) {
      // Just crossed threshold — start the hold timer
      slouch_timing   = true;
      slouch_start_ms = now;
    }
    
    if (slouch_timing && (now - slouch_start_ms >= HOLD_TIME_MS)) {
      // Hold time met — activate persistent alerting
      alert_active = true;
    }

    if (alert_active) {
      // Buzz repeatedly every BUZZ_REPEAT_MS until posture is fixed
      if (now - last_buzz_ms >= BUZZ_REPEAT_MS) {
        buzz_command = 1;
        last_buzz_ms = now;
      }
    }
  } else {
    // Back under threshold — IMMEDIATELY stop everything
    // No hysteresis, no sustained upright requirement, no grace period.
    // This is a key behavioral difference from the adaptive system:
    // the user gets instant relief when they sit up, but also gets
    // instant re-triggering if they slouch again momentarily.
    slouch_timing = false;
    alert_active  = false;
  }

  // Actuate
  if (buzz_command > 0) {
    digitalWrite(VIBRATION_PIN, HIGH);
    delay(BUZZ_DURATION_MS);
    digitalWrite(VIBRATION_PIN, LOW);
  }

  // BLE — same 5-byte packet as adaptive firmware
  // App doesn't need to know which firmware is running
  uint8_t packet[5];
  memcpy(packet, &ema_pitch, 4);
  packet[4] = (alert_active) ? 1 : 0;  // Report alert STATE, not just buzz events
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