#include "esp_timer.h"
#include <array>
#include <cmath>
#include <iostream>

#include "esp_system.h"

class KalmanFilterMatrix {

public:
  KalmanFilterMatrix();

  void init(float init_x, float init_y, float init_theta, float init_cov,
            float p_noise, float m_noise);

  void predict(float v, float w, float dt);

  void update(const std::array<float, 3> &z);

  const std::array<float, 3> &get_state() const { return x; }

  float ang = 0;
private:
  std::array<float, 3> x; // 状態ベクトル [x, y, θ]
  std::array<std::array<float, 3>, 3> F; // 状態遷移行列
  std::array<std::array<float, 2>, 3> B; // 制御入力モデル行列
  std::array<std::array<float, 3>, 3> H; // 観測モデル行列
  std::array<std::array<float, 3>, 3> P; // 状態共分散行列
  std::array<std::array<float, 3>, 3> Q; // プロセスノイズ共分散行列
  std::array<std::array<float, 3>, 3> R; // 観測ノイズ共分散行列

  // ヘルパー関数群
  std::array<std::array<float, 3>, 3>
      IRAM_ATTR matrixMultiply(const std::array<std::array<float, 3>, 3> &A,
                               const std::array<std::array<float, 3>, 3> &B) {
    std::array<std::array<float, 3>, 3> result = {0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }
    return result;
  }

  std::array<std::array<float, 3>, 3>
      IRAM_ATTR transpose(const std::array<std::array<float, 3>, 3> &A) {
    std::array<std::array<float, 3>, 3> result = {0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i][j] = A[j][i];
      }
    }
    return result;
  }

  std::array<std::array<float, 3>, 3>
      IRAM_ATTR matrixAdd(const std::array<std::array<float, 3>, 3> &A,
                          const std::array<std::array<float, 3>, 3> &B) {
    std::array<std::array<float, 3>, 3> result = {0};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i][j] = A[i][j] + B[i][j];
      }
    }
    return result;
  }

  std::array<std::array<float, 3>, 3>
      IRAM_ATTR invertMatrix(const std::array<std::array<float, 3>, 3> &A) {
    std::array<std::array<float, 3>, 3> result = {0};
    // 必要な小行列式を事前計算
    float m00 = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    float m01 = A[1][0] * A[2][2] - A[1][2] * A[2][0];
    float m02 = A[1][0] * A[2][1] - A[1][1] * A[2][0];

    float m10 = A[0][1] * A[2][2] - A[0][2] * A[2][1];
    float m11 = A[0][0] * A[2][2] - A[0][2] * A[2][0];
    float m12 = A[0][0] * A[2][1] - A[0][1] * A[2][0];

    float m20 = A[0][1] * A[1][2] - A[0][2] * A[1][1];
    float m21 = A[0][0] * A[1][2] - A[0][2] * A[1][0];
    float m22 = A[0][0] * A[1][1] - A[0][1] * A[1][0];

    // 行列式の計算
    float det = A[0][0] * m00 - A[0][1] * m01 + A[0][2] * m02;

    // 行列が非可逆の場合
    if (det == 0.0) {
      // throw std::runtime_error("Matrix is not invertible");
    }

    // 逆行列の計算 (1 / det を掛ける)
    float inv_det = 1.0 / det;

    result[0][0] = m00 * inv_det;
    result[0][1] = -m10 * inv_det;
    result[0][2] = m20 * inv_det;

    result[1][0] = -m01 * inv_det;
    result[1][1] = m11 * inv_det;
    result[1][2] = -m21 * inv_det;

    result[2][0] = m02 * inv_det;
    result[2][1] = -m12 * inv_det;
    result[2][2] = m22 * inv_det;
    return result;
  }

  std::array<std::array<float, 3>, 3> IRAM_ATTR identityMatrix(int size) {
    std::array<std::array<float, 3>, 3> result = {0};
    for (int i = 0; i < size; ++i) {
      result[i][i] = 1.0;
    }
    return result;
  }
};
