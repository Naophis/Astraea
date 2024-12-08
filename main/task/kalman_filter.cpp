
#include "include/kalman_filter.hpp"

KalmanFilter::KalmanFilter() {}
void KalmanFilter::init(float initial_x, float initial_P, float process_noise,
                        float measurement_noise) {
  x = initial_x;
  P = P2 = init_P = initial_P;
  Q = process_noise;
  R = R2 = measurement_noise;
}

void KalmanFilter::print_state() {
  printf("  P:%f, Q:%f, R:%f\n", P, Q, R); //
}

void KalmanFilter::predict(float u) {
  // 予測ステップ
  x = x + u * dt;
  P = P + Q;
  P2 = P2 + Q;
}

void KalmanFilter::update(float z) {
  // 更新ステップ
  float K = P / (P + R);
  x = x + K * (z - x);
  P = (1 - K) * P;
}

void KalmanFilter::update2(float z_gyro, float z_wheel) {
  // 重み計算
  float weight_gyro = P / (P + R);
  float weight_wheel = P2 / (P2 + R2);

  // 状態統合
  x = (weight_gyro * z_gyro + weight_wheel * z_wheel) /
      (weight_gyro + weight_wheel);

  // 共分散統合
  P = (P * R2) / (R + R2);
  P2 = (P2 * R) / (R + R2);
}

float KalmanFilter::get_state() {
  return x; //
}

void KalmanFilter::reset(float reset_val) {
  x = reset_val;
  P = init_P;
}

void KalmanFilter::offset(float offset) {
  x = x + offset; //
}