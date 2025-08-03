#pragma once

#include <cstdint>
#include <cmath>

/**
 * @brief 最適化された数学関数ユーティリティ
 * 
 * 元のSimulinkコードから数学関数を分離し、
 * パフォーマンスを最適化した実装を提供
 */
namespace MathUtils {

/**
 * @brief 高速平方根計算（逆数平方根を使用）
 * @param x 入力値
 * @return 平方根
 */
float IRAM_ATTR fastSqrt(float x);

/**
 * @brief 高速べき乗計算（整数べき）
 * @param x 底
 * @param n 指数
 * @return x^n
 */
float IRAM_ATTR fastPow(float x, int n);

/**
 * @brief 高精度指数関数
 * @param x 指数
 * @return e^x
 */
float IRAM_ATTR exactExp(float x);

/**
 * @brief 安全なべき乗関数（NaN/Inf処理付き）
 * @param base 底
 * @param exponent 指数  
 * @return base^exponent
 */
float IRAM_ATTR safePow(float base, float exponent);

/**
 * @brief 値のクランピング
 * @param value 値
 * @param min_val 最小値
 * @param max_val 最大値
 * @return クランプされた値
 */
template<typename T>
constexpr T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/**
 * @brief NaN/Inf チェック
 * @param value チェックする値
 * @return true if NaN or Inf
 */
bool isNanOrInf(float value);

} // namespace MathUtils