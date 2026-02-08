// ============================================================
// PoseGuide Firmware — Main Sketch
// Integrates: IMU+EMA (Albert) → HMM + Bayesian + Dual-Gate
//             (Denise) → Vibration + BLE (Tavi)
// ============================================================

#include <Arduino_LSM6DS3.h>
#include <ArduinoBLE.h>
#include <FlashStorage.h>

#include "hmm_forward.h"
#include "bayesian_belief.h"
#include "dual_gate_controller.h"

// ======================== PIN CONFIG ========================
const int VIBRATION_PIN = 2;

// ============ PERSISTENT CALIBRATION (survives power off) ============
typedef struct {
  boolean valid;
  float   gyro_bias;
  float   baseline_angle;
} CalibrationData;

FlashStorage(flash_calibration, CalibrationData);

float gyro_bias       = 0.0;
float baseline_angle  = 0.0;
bool  calibrated      = false;

// ====================== IMU / EMA (Albert's updated code) ======================
float ema_pitch = 0.0;
float ema_gyro  = 0.0;
const float EMA_ALPHA = 0.15;       
bool first_reading    = true;

// =================== SAMPLING CONTROL ===================
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 Hz
static unsigned long last_sample = 0;

// ===================== DEVICE MODE =====================
enum DeviceMode { MODE_CALIBRATING, MODE_MONITORING };
DeviceMode currentMode = MODE_MONITORING;

// ===================== BLE SERVICE =====================
// UUIDs matching Tavi's BluetoothSensorManager.kt
BLEService postureService("00001810-0000-1000-8000-00805f9b34fb");

// Device → App: posture data (notifications)
// During CALIBRATING: [4B angle][4B raw_gyro][1B mode=0x01] = 9 bytes
// During MONITORING:  [4B angle][1B alert_flag]              = 5 bytes
BLECharacteristic postureChar(
  "00002a35-0000-1000-8000-00805f9b34fb",
  BLERead | BLENotify, 20
);

// App → Device: calibration commands (write)
// [1B command][4B gyro_bias][4B baseline_angle] = 9 bytes
// Commands: 0x01 = enter calibration mode (device streams raw data)
//           0x02 = write calibration + start monitoring
//           0x03 = recalibrate (resets session)
BLECharacteristic calibrationChar(
  "00002a36-0000-1000-8000-00805f9b34fb",
  BLEWrite, 9
);

// ===================== MODULE INSTANCES =====================
HMMForward         hmm;
BayesianBelief     bayesian;
DualGateController gate;

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  // No while(!Serial) — device must work standalone
  
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);

  // --- IMU Init ---
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
    for (int i = 0; i < 5; i++) {
      digitalWrite(VIBRATION_PIN, HIGH); delay(200);
      digitalWrite(VIBRATION_PIN, LOW);  delay(200);
    }
    while (1);
  }
  
  // NOTE: No gyro bias calibration here anymore!
  // The app handles it during the calibration flow and sends it back.

  // --- Load calibration from flash ---
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

  // --- Init modules ---
  hmm.begin();
  bayesian.begin();
  gate.begin();

  // --- BLE Init ---
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

  // Confirmation buzz
  digitalWrite(VIBRATION_PIN, HIGH); delay(150);
  digitalWrite(VIBRATION_PIN, LOW);
  Serial.println("PoseGuide ready.");
}

// ============ BLE CALLBACK: App writes calibration data ============
void onCalibrationWritten(BLEDevice central, BLECharacteristic characteristic) {
  const uint8_t* data = characteristic.value();
  int len = characteristic.valueLength();
  if (len < 1) return;

  uint8_t command = data[0];

  switch (command) {
    case 0x01: {
      // Enter calibration mode — stream angle+gyro for app to collect
      currentMode   = MODE_CALIBRATING;
      first_reading = true;   // Reset EMA so it re-initializes cleanly
      Serial.println("Entered CALIBRATION mode");
      break;
    }
    case 0x02:
    case 0x03: {
      // Receive computed calibration values from app
      if (len >= 9) {
        memcpy(&gyro_bias,      &data[1], 4);
        memcpy(&baseline_angle, &data[5], 4);
        calibrated = true;

        // Persist to flash
        CalibrationData toStore;
        toStore.valid          = true;
        toStore.gyro_bias      = gyro_bias;
        toStore.baseline_angle = baseline_angle;
        flash_calibration.write(toStore);

        Serial.println("Calibration received and saved:");
        Serial.print("  gyro_bias: ");      Serial.println(gyro_bias, 4);
        Serial.print("  baseline_angle: "); Serial.println(baseline_angle, 2);

        // Reset all modules for fresh start
        hmm.begin();
        bayesian.begin();
        gate.begin();
        
        // Reset EMA — seed with baseline so no ramp-up artifact
        ema_pitch     = baseline_angle;
        ema_gyro      = 0.0;
        first_reading = false;  // Already seeded, skip auto-init

        currentMode = MODE_MONITORING;
        Serial.println("Entered MONITORING mode");

        // Confirmation double-buzz
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

  // ---- STEP 1: Read IMU + EMA Filter ----
  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  float pitch_rad = atan2(ax, sqrt(ay * ay + az * az));
  float pitch_deg = pitch_rad * 180.0 / PI;
  float gyro_rate = gy - gyro_bias;

  // Seed EMA with actual value, not 0
  if (first_reading) {
    ema_pitch     = pitch_deg;
    ema_gyro      = gyro_rate;
    first_reading = false;
  } else {
    ema_pitch = EMA_ALPHA * pitch_deg  + (1.0 - EMA_ALPHA) * ema_pitch;
    ema_gyro  = EMA_ALPHA * gyro_rate  + (1.0 - EMA_ALPHA) * ema_gyro;
  }

  // Take absolute value of filtered gyro
  float abs_gyro = fabs(ema_gyro);

  // ======== MODE: CALIBRATING ========
  // Stream angle + raw gyro to app. App collects 3 seconds of readings.
  // Packet: [4B ema_pitch float][4B gyro_rate float][1B 0x01 mode flag]
  // The mode flag byte lets the app distinguish calibration packets from
  // monitoring packets on the same characteristic.
  if (currentMode == MODE_CALIBRATING) {
    uint8_t calPacket[9];
    memcpy(&calPacket[0], &ema_pitch, 4);    // Filtered angle
    memcpy(&calPacket[4], &gyro_rate, 4);    // Raw gyro (pre-abs, pre-bias for app to compute bias)
    calPacket[8] = 0x01;                     // Mode flag: calibrating
    postureChar.writeValue(calPacket, 9);

    Serial.print("[CAL] ");
    Serial.print(now);          Serial.print(",");
    Serial.print(ema_pitch, 2); Serial.print(",");
    Serial.println(gyro_rate, 2);
    return;
  }

  // ======== MODE: MONITORING (full pipeline) ========
  float deviation = ema_pitch - baseline_angle;

  // STEP 2: HMM Forward Algorithm (Denise)
  hmm.update(deviation, abs_gyro);
  float p_upright   = hmm.getProb(HMM_UPRIGHT);
  float p_slouch    = hmm.getProb(HMM_SLOUCHING);
  float p_transient = hmm.getProb(HMM_TRANSIENT);

  // STEP 3: Bayesian Belief Update (Denise)
  bayesian.update(deviation, abs_gyro);
  float belief = bayesian.getBelief();

  // STEP 4: Dual-Gating Controller (Denise)
  gate.update(belief, p_slouch, p_transient, p_upright, now);
  int buzz_command = gate.getBuzzCommand();

  // STEP 5: Actuate Vibration Motor
  executeVibration(buzz_command);

  // STEP 6: Send monitoring data via BLE
  // Packet: [4B ema_pitch float][1B alert_flag]
  // Matches Tavi's parsePostureData() in BluetoothSensorManager.kt
  sendBLEMonitoring(ema_pitch, buzz_command > 0);

  // STEP 7: Serial logging (debug / thesis data collection)
  Serial.print(now);            Serial.print(",");
  Serial.print(ema_pitch, 2);   Serial.print(",");
  Serial.print(deviation, 2);   Serial.print(",");
  Serial.print(abs_gyro, 2);    Serial.print(",");
  Serial.print(belief, 4);      Serial.print(",");
  Serial.print(p_upright, 4);   Serial.print(",");
  Serial.print(p_slouch, 4);    Serial.print(",");
  Serial.print(p_transient, 4); Serial.print(",");
  Serial.println(buzz_command);
  Serial.print(gate.getPostureHistory(), 4);      Serial.print(","); 
  Serial.println(gate.getAdaptiveThreshold(), 4);                    
}

// ======================== VIBRATION PATTERNS ========================
void executeVibration(int command) {
  switch (command) {
    case 0:
      digitalWrite(VIBRATION_PIN, LOW);
      break;
    case 1:  // Single pulse — initial alert
      digitalWrite(VIBRATION_PIN, HIGH); delay(200);
      digitalWrite(VIBRATION_PIN, LOW);
      break;
    case 2:  // Double pulse — escalated alert
      digitalWrite(VIBRATION_PIN, HIGH); delay(150);
      digitalWrite(VIBRATION_PIN, LOW);  delay(100);
      digitalWrite(VIBRATION_PIN, HIGH); delay(150);
      digitalWrite(VIBRATION_PIN, LOW);
      break;
  }
}

// ======================== BLE MONITORING PACKET ========================
void sendBLEMonitoring(float angle, bool alert) {
  uint8_t packet[5];
  memcpy(packet, &angle, 4);
  packet[4] = alert ? 1 : 0;
  postureChar.writeValue(packet, 5);
}
