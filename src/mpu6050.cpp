#include <math.h>
#include <Arduino.h>
#include "mpu6050.h"

// MPU6050
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;

// -------------------- อ่านค่า MPU6050 --------------------
void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
}

// -------------------- Task: Read MPU6050 --------------------
void TaskMPU6050(void *pvParameters) {
  for (;;) {
    gyro_signals();
    RateRoll  -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw   -= RateCalibrationYaw;

    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    vTaskDelay(4 / portTICK_PERIOD_MS);
  }
}