#include "network_untils.h"
#include "kalman.h"
#include "mpu6050.h"
#include "pid_control.h"

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  for (int i = 0; i < 2000; i++) {
    gyro_signals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }
  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;

  // ⭐ Set ค่าเริ่มต้นให้ Kalman จากค่ามุมที่วัดได้ขณะนิ่ง
  KalmanAngleRoll = AngleRoll;
  KalmanAnglePitch = AnglePitch;
  motor.attach(32); // กำหนดขา PWM ควบคุม ESC/Servo

  xTaskCreatePinnedToCore(TaskMPU6050, "MPU6050", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskPublish, "Publish", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskPIDControl, "PID", 4096, NULL, 1, NULL, 1);
}

// -------------------- Loop --------------------
void loop() {

}
