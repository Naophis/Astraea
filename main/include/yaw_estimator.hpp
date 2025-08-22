#include <array>
#include <cmath>

namespace estimator {

struct ImuSample {
  float p, q, r;   // [rad/s]
  float ax, ay, az; // [m/s^2]
  float dt;         // [s]
};

struct Calibration {
  float eps_zx = 0.0f;
  float eps_zy = 0.0f;
  float kgzx = 0.0f;
  float kgzy = 0.0f;
  float kgzz = 0.0f;
};

class YawEstimator {
public:
  YawEstimator();

  void setCalibration(const Calibration& c);
  void enableAccelFusion(bool on);
  void setAccelGate(float g0 = 9.80665f, float gate_width = 3.0f, float alpha_max = 0.1f);

  void resetYaw(float yaw_rad);
  void resetAll(float roll_rad = 0.0f, float pitch_rad = 0.0f, float yaw_rad = 0.0f);

  void update(const ImuSample& s);

  float yaw()   const;
  float pitch() const;
  float roll()  const;

  float yaw_rate_kinematic(const ImuSample& s) const;

  std::array<float,4> quat() const;

private:
  static std::array<float,4> quatMul(const std::array<float,4>& a,
                                     const std::array<float,4>& b);
  static std::array<float,4> quatFromEuler(float roll, float pitch, float yaw);
  void eulerRPY(float& roll, float& pitch, float& yaw) const;
  static float norm3(const std::array<float,3>& v);
  static std::array<float,4> quatNormalize(const std::array<float,4>& q);
  static float sinc_over_x(float x);
  void integrateQuat(const std::array<float,3>& w_avg, float dt);
  static float wrapPi(float a);
  static float lerpAngle(float a, float b, float t);
  void setIdentity();

private:
  Calibration calib_{};
  std::array<float,4> q_ {1.0f,0.0f,0.0f,0.0f};
  std::array<float,3> prev_w_ {0.0f,0.0f,0.0f};
  bool prev_valid_ = false;

  bool  use_acc_fusion_ = false;
  float g_ref_ = 9.80665f;
  float gate_width_ = 3.0f;
  float alpha_max_ = 0.1f;
};

} // namespace mmouse