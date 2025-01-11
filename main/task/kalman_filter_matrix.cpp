#include "include/kalman_filter_matrix.hpp"
KalmanFilterMatrix::KalmanFilterMatrix() {
  // 状態ベクトル [x, y, θ]
  x = {0.0, 0.0, 0.0};

  // 状態遷移行列 (初期値: 単位行列)
  F = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

  // 制御入力モデル行列
  B = {{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}}};

  // 観測モデル行列
  H = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

  // 状態共分散行列
  P = {{{0.05, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}}};

  // プロセスノイズ共分散行列
  Q = {{{0.01, 0.0, 0.0}, {0.0, 0.01, 0.0}, {0.0, 0.0, 0.01}}};

  // 観測ノイズ共分散行列
  R = {{{0.1, 0.0, 0.0}, {0.0, 0.1, 0.0}, {0.0, 0.0, 0.1}}};
}

void IRAM_ATTR KalmanFilterMatrix::predict(float v, float w, float dt) {
  // 状態遷移モデル更新
  float theta = x[2];
  F[0][2] = -v * sinf(theta) * dt;
  F[1][2] = v * cosf(theta) * dt;

  // 制御入力モデル更新
  B[0][0] = cosf(theta) * dt;
  B[1][0] = sinf(theta) * dt;
  B[2][1] = dt;

  // 制御入力
  std::array<float, 2> u = {v, w};

  // 状態推定
  x[0] += B[0][0] * u[0];
  x[1] += B[1][0] * u[0];
  x[2] += B[2][1] * u[1];

  // 共分散推定
  auto FP = matrixMultiply(F, P);
  auto FPFt = matrixMultiply(FP, transpose(F));
  P = matrixAdd(FPFt, Q);
}
void IRAM_ATTR KalmanFilterMatrix::init(float init_x, float init_y,
                                        float init_theta, float init_cov,
                                        float p_noise, float m_noise) {
  // 座標の初期化
  x = {init_x, init_y, init_theta};
  ang = init_theta;
  reset_cov(init_cov, p_noise, m_noise);
}

void IRAM_ATTR KalmanFilterMatrix::reset_cov(float init_cov, float p_noise,
                                             float m_noise) {
  // 状態共分散行列を初期化
  P = {{{init_cov, 0.0, 0.0}, {0.0, init_cov, 0.0}, {0.0, 0.0, init_cov}}};
  Q = {{{p_noise, 0.0, 0.0}, {0.0, p_noise, 0.0}, {0.0, 0.0, p_noise}}};
  // 観測ノイズ共分散行列
  R = {{{m_noise, 0.0, 0.0}, {0.0, m_noise, 0.0}, {0.0, 0.0, m_noise}}};
}
void IRAM_ATTR KalmanFilterMatrix::update(const std::array<float, 3> &z) {
  // カルマンゲイン計算
  auto HP = matrixMultiply(H, P);
  auto HPHt = matrixMultiply(HP, transpose(H));
  auto S = matrixAdd(HPHt, R);
  auto S_inv = invertMatrix(S);
  auto K = matrixMultiply(P, matrixMultiply(transpose(H), S_inv));

  // 状態更新
  std::array<float, 3> y = {z[0] - x[0], z[1] - x[1], z[2] - x[2]};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      x[i] += K[i][j] * y[j];
    }
  }

  // 共分散更新
  auto KH = matrixMultiply(K, H);
  auto I_KH = identityMatrix(3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      I_KH[i][j] -= KH[i][j];
    }
  }
  P = matrixMultiply(I_KH, P);
}