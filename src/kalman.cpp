#include "kalman.h"

float Kalman1DOutput[2] = {0, 0};
// -------------------- Kalman Filter --------------------
void kalman_1d(float &state_estimate, float &uncertainty, float rate_input, float measurement) {
  float dt = 0.004;             // Sampling time [s]
  float Q = dt * dt * 4.0 * 4.0;     // Process noise covariance (gyro)
  float R = 3.0 * 3.0;                // Measurement noise covariance (acc)

  // Prediction step
  state_estimate += dt * rate_input;  // x̂⁻ = x̂ + dt·u
  uncertainty += Q;                   // P⁻ = P + Q

  // Correction step
  float kalman_gain = uncertainty / (uncertainty + R);       // K = P⁻ / (P⁻ + R)
  state_estimate += kalman_gain * (measurement - state_estimate);  // x̂ = x̂⁻ + K(z - x̂⁻)
  uncertainty *= (1.0 - kalman_gain);                        // P = (1 - K)P⁻

  Kalman1DOutput[0] = state_estimate;
  Kalman1DOutput[1] = uncertainty;
}