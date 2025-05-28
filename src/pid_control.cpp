#include "pid_control.h"

Servo motor;
float Kp = 1.5, Ki = 0.3, Kd = 0.1;
float error = 0, integral = 0, derivative = 0, prev_error = 0;
unsigned long lastPID = 0;

// -------------------- Task: PID Control --------------------
void TaskPIDControl(void *pvParameters) {
  for (;;) {
    unsigned long now = millis();
    float dt = (now - lastPID) / 1000.0;
    lastPID = now;

    error = setDegree - KalmanAngleRoll;
    integral += error * dt;
    derivative = (error - prev_error) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;

    int pwm = constrain(map(output, -45, 45, 1000, 2000), 1000, 2000);
    motor.writeMicroseconds(pwm);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}