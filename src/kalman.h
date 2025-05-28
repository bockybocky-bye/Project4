#ifndef KALMAN_H
#define KALMAN_H

void kalman_1d(float &state_estimate, float &uncertainty, float rate_input, float measurement);

extern float Kalman1DOutput[2];
#endif