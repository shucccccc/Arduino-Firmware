#include <Arduino_LSM6DS3.h>

float kal_pitch = 0.0;
float gyro_bias = 0.0;
const float alpha = 0.10;

void setup(){
  Serial.begin(115200);  // Fast baud rate
  while (!Serial);

  if(!IMU.begin()){
    Serial.println("Failed to initialize IMU");
    while(1);
  }

  // Calibrate gyro bias (hold still!)
  float sum_gyro = 0;
  int samples = 0;
  for(int i = 0; i < 100; i++) {
    float gx, gy, gz;
    if(IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      sum_gyro += gy;
      samples++;
    }
    delay(20);
  }
  gyro_bias = sum_gyro / samples;
  
  delay(1000);  // Wait before starting data stream
}

void loop() {
  static unsigned long last_sample = 0;
  unsigned long now = millis();
  
  // Sample every 100ms (10Hz)
  if (now - last_sample < 100) {
    return;
  }
  last_sample = now;
  
  float ax, ay, az, gx, gy, gz;
  
  // Read IMU directly
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  
  float pitch_rad = atan2(ax, sqrt(ay*ay + az*az));
  float pitch_deg = pitch_rad * 180.0 / PI;
  
  // Low-pass filter for stable angle
  kal_pitch = alpha * pitch_deg + (1.0 - alpha) * kal_pitch;
  float gyro_rate = gy - gyro_bias;
  
  // CSV output: timestamp,angle,gyro
  Serial.print(now);
  Serial.print(",");
  Serial.print(kal_pitch, 2);
  Serial.print(",");
  Serial.println(gyro_rate, 2);
}
