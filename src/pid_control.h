#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <ESP32Servo.h> 

extern Servo motor;
extern float Kp , Ki , Kd;
extern float error , integral , derivative , prev_error ;
extern unsigned long lastPID ;
extern float setDegree;
extern float KalmanAngleRoll;

void TaskPIDControl(void *pvParameters);

#endif