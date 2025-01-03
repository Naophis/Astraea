#include <iostream>

class KalmanFilter {
private:
  float x; // 状態の推定値
  float P; // 状態の誤差共分散行列
  float P2; // 状態の誤差共分散行列
  float Q; // プロセスノイズの共分散
  float R; // 観測ノイズの共分散
  float R2; // 観測ノイズの共分散

  float init_P;

public:
  float dt = 0.001;
  KalmanFilter();
  void init(float initial_x, float initial_P, float process_noise,
            float measurement_noise);
  void predict(float u);

  void update(float z);

  void update2(float z, float z2);

  float get_state();

  void reset(float reset_val);

  void offset(float offset);

  void print_state();
};