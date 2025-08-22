
#include "include/yaw_estimator.hpp"
#include "esp_system.h"
#include <algorithm>

namespace estimator {

IRAM_ATTR YawEstimator::YawEstimator() { setIdentity(); }

void IRAM_ATTR YawEstimator::setCalibration(const Calibration &c) {
  calib_ = c;
}
void IRAM_ATTR YawEstimator::enableAccelFusion(bool on) {
  use_acc_fusion_ = on;
}
void IRAM_ATTR YawEstimator::setAccelGate(float g0, float gate_width,
                                          float alpha_max) {
  g_ref_ = g0;
  gate_width_ = std::max(1e-3f, gate_width);
  alpha_max_ = std::clamp(alpha_max, 0.0f, 1.0f);
}

void IRAM_ATTR YawEstimator::resetYaw(float yaw_rad) {
  float roll, pitch, yaw_old;
  eulerRPY(roll, pitch, yaw_old);
  q_ = quatFromEuler(roll, pitch, yaw_rad);
}

void IRAM_ATTR YawEstimator::resetAll(float roll_rad, float pitch_rad,
                                      float yaw_rad) {
  q_ = quatFromEuler(roll_rad, pitch_rad, yaw_rad);
  prev_w_.fill(0.0f);
  prev_valid_ = false;
}

void IRAM_ATTR YawEstimator::update(const ImuSample &s) {
  float r_corr = s.r - calib_.eps_zx * s.p - calib_.eps_zy * s.q -
                 (calib_.kgzx * s.ax + calib_.kgzy * s.ay + calib_.kgzz * s.az);

  std::array<float, 3> w{{s.p, s.q, r_corr}};

  std::array<float, 3> w_avg = w;
  if (prev_valid_) {
    w_avg[0] = 0.5f * (prev_w_[0] + w[0]);
    w_avg[1] = 0.5f * (prev_w_[1] + w[1]);
    w_avg[2] = 0.5f * (prev_w_[2] + w[2]);
  }
  integrateQuat(w_avg, s.dt);
  prev_w_ = w;
  prev_valid_ = true;

  if (use_acc_fusion_) {
    float a_norm = std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    float d = std::fabs(a_norm - g_ref_);
    float w_acc = 1.0f - std::min(d / gate_width_, 1.0f);
    if (w_acc > 0.0f) {
      float roll_acc = std::atan2(s.ay, s.az);
      float pitch_acc = std::atan2(-s.ax, std::sqrt(s.ay * s.ay + s.az * s.az));

      float roll, pitch, yaw_now;
      eulerRPY(roll, pitch, yaw_now);
      float alpha = alpha_max_ * w_acc;
      roll = lerpAngle(roll, roll_acc, alpha);
      pitch = lerpAngle(pitch, pitch_acc, alpha);
      q_ = quatFromEuler(roll, pitch, yaw_now);
    }
  }
}

float IRAM_ATTR YawEstimator::yaw() const {
  float r, p, y;
  eulerRPY(r, p, y);
  return y;
}
float IRAM_ATTR YawEstimator::pitch() const {
  float r, p, y;
  eulerRPY(r, p, y);
  return p;
}
float IRAM_ATTR YawEstimator::roll() const {
  float r, p, y;
  eulerRPY(r, p, y);
  return r;
}

float IRAM_ATTR YawEstimator::yaw_rate_kinematic(const ImuSample &s) const {
  float r_corr = s.r - calib_.eps_zx * s.p - calib_.eps_zy * s.q -
                 (calib_.kgzx * s.ax + calib_.kgzy * s.ay + calib_.kgzz * s.az);
  float roll, pitch, yaw_now;
  eulerRPY(roll, pitch, yaw_now);
  float ct = std::max(1e-4f, std::cos(pitch));
  return (std::sin(roll) / ct) * s.p + (std::cos(roll) / ct) * s.q + r_corr;
}

std::array<float, 4> IRAM_ATTR YawEstimator::quat() const { return q_; }

// --- private utils ---

std::array<float, 4>
    IRAM_ATTR YawEstimator::quatMul(const std::array<float, 4> &a,
                                    const std::array<float, 4> &b) {
  return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
          a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
          a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
          a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

std::array<float, 4>
    IRAM_ATTR YawEstimator::quatFromEuler(float roll, float pitch, float yaw) {
  float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
  float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
  float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
  return {cy * cp * cr + sy * sp * sr, cy * cp * sr - sy * sp * cr,
          cy * sp * cr + sy * cp * sr, sy * cp * cr - cy * sp * sr};
}

void IRAM_ATTR YawEstimator::eulerRPY(float &roll, float &pitch,
                                      float &yaw) const {
  const float w = q_[0], x = q_[1], y_ = q_[2], z = q_[3];
  float sinr_cosp = 2.0f * (w * x + y_ * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y_ * y_);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  float sinp = 2.0f * (w * y_ - z * x);
  if (std::fabs(sinp) >= 1.0f)
    pitch = std::copysign(M_PI / 2.0f, sinp);
  else
    pitch = std::asin(sinp);
  float siny_cosp = 2.0f * (w * z + x * y_);
  float cosy_cosp = 1.0f - 2.0f * (y_ * y_ + z * z);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

float IRAM_ATTR YawEstimator::norm3(const std::array<float, 3> &v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::array<float, 4>
    IRAM_ATTR YawEstimator::quatNormalize(const std::array<float, 4> &q) {
  float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n <= 0.0f)
    return {1, 0, 0, 0};
  float inv = 1.0f / n;
  return {q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv};
}

float IRAM_ATTR YawEstimator::sinc_over_x(float x) {
  if (std::fabs(x) < 1e-6f)
    return 1.0f - x * x / 6.0f;
  return std::sin(x) / x;
}

void IRAM_ATTR YawEstimator::integrateQuat(const std::array<float, 3> &w_avg,
                                           float dt) {
  const float wx = w_avg[0], wy = w_avg[1], wz = w_avg[2];
  float half_dt = 0.5f * dt;
  float rvx = wx * half_dt, rvy = wy * half_dt, rvz = wz * half_dt;
  float theta = std::sqrt(rvx * rvx + rvy * rvy + rvz * rvz);
  float s_over = sinc_over_x(theta);
  std::array<float, 4> dq{std::cos(theta), rvx * s_over, rvy * s_over,
                          rvz * s_over};
  q_ = quatNormalize(quatMul(q_, dq));
}

float IRAM_ATTR YawEstimator::wrapPi(float a) {
  while (a > M_PI)
    a -= 2.0f * M_PI;
  while (a <= -M_PI)
    a += 2.0f * M_PI;
  return a;
}

float IRAM_ATTR YawEstimator::lerpAngle(float a, float b, float t) {
  float d = wrapPi(b - a);
  return a + t * d;
}

void IRAM_ATTR YawEstimator::setIdentity() {
  q_ = {1.0f, 0.0f, 0.0f, 0.0f};
  prev_w_.fill(0.0f);
  prev_valid_ = false;
}

} // namespace mmouse
