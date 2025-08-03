#include "math_utils.hpp"
#include <cstring>
#include <cmath>

namespace MathUtils {

// bit_cast用のユニオン（C++20のstd::bit_cast代替）
template<class To, class From>
To IRAM_ATTR bit_cast(const From& from) noexcept {
    To to;
    static_assert(sizeof(to) == sizeof(from), "Size mismatch in bit_cast");
    std::memcpy(&to, &from, sizeof(to));
    return to;
}

// 高速平方根計算（Newton-Raphson法による逆数平方根）
float IRAM_ATTR fastSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    
    union {
        float f;
        uint32_t i;
    } conv;
    
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);  // マジックナンバー
    conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);  // 1回のNewton-Raphson
    return 1.0f / conv.f;
}

// 高速べき乗計算（バイナリべき乗法）
float IRAM_ATTR fastPow(float x, int n) {
    if (n == 0) {
        return 1.0f;
    }
    if (n < 0) {
        x = 1.0f / x;
        n = -n;
    }
    
    float result = 1.0f;
    while (n > 0) {
        if (n % 2 == 1) {
            result *= x;
        }
        x *= x;
        n /= 2;
    }
    return result;
}

// 高精度指数関数（テーブル補間 + Taylor展開）
namespace {
    double expm1_taylor3(double t1) noexcept {
        constexpr double C2 = 1.0 / 2.0;
        constexpr double C3 = 1.0 / 6.0;
        const double s1 = std::fma(C3, t1, C2);
        const double t2 = t1 * t1;
        return std::fma(s1, t2, t1);
    }

    double exp_table(uint64_t s) noexcept {
        constexpr double b1table[32]{
            0x1.0000000000000p+0, 0x1.059b0d3158574p+0, 0x1.0b5586cf9890fp+0, 0x1.11301d0125b51p+0,
            0x1.172b83c7d517bp+0, 0x1.1d4873168b9aap+0, 0x1.2387a6e756238p+0, 0x1.29e9df51fdee1p+0,
            0x1.306fe0a31b715p+0, 0x1.371a7373aa9cbp+0, 0x1.3dea64c123422p+0, 0x1.44e086061892dp+0,
            0x1.4bfdad5362a27p+0, 0x1.5342b569d4f82p+0, 0x1.5ab07dd485429p+0, 0x1.6247eb03a5585p+0,
            0x1.6a09e667f3bcdp+0, 0x1.71f75e8ec5f74p+0, 0x1.7a11473eb0187p+0, 0x1.82589994cce13p+0,
            0x1.8ace5422aa0dbp+0, 0x1.93737b0cdc5e5p+0, 0x1.9c49182a3f090p+0, 0x1.a5503b23e255dp+0,
            0x1.ae89f995ad3adp+0, 0x1.b7f76f2fb5e47p+0, 0x1.c199bdd85529cp+0, 0x1.cb720dcef9069p+0,
            0x1.d5818dcfba487p+0, 0x1.dfc97337b9b5fp+0, 0x1.ea4afa2a490dap+0, 0x1.f50765b6e4540p+0,
        };

        constexpr double b2table[32]{
            0x1.0000000000000p+0, 0x1.002c605e2e8cfp+0, 0x1.0058c86da1c0ap+0, 0x1.0085382faef83p+0,
            0x1.00b1afa5abcbfp+0, 0x1.00de2ed0ee0f5p+0, 0x1.010ab5b2cbd11p+0, 0x1.0137444c9b5b5p+0,
            0x1.0163da9fb3335p+0, 0x1.019078ad6a19fp+0, 0x1.01bd1e77170b4p+0, 0x1.01e9cbfe113efp+0,
            0x1.02168143b0281p+0, 0x1.02433e494b755p+0, 0x1.027003103b10ep+0, 0x1.029ccf99d720ap+0,
            0x1.02c9a3e778061p+0, 0x1.02f67ffa765e6p+0, 0x1.032363d42b027p+0, 0x1.03504f75ef071p+0,
            0x1.037d42e11bbccp+0, 0x1.03aa3e170aafep+0, 0x1.03d7411915a8ap+0, 0x1.04044be896ab6p+0,
            0x1.04315e86e7f85p+0, 0x1.045e78f5640b9p+0, 0x1.048b9b35659d8p+0, 0x1.04b8c54847a28p+0,
            0x1.04e5f72f654b1p+0, 0x1.051330ec1a03fp+0, 0x1.0540727fc1762p+0, 0x1.056dbbebb786bp+0,
        };

        const double b1 = b1table[s >> 5 & 31];
        const double b2 = b2table[s & 31];
        const uint64_t exponent = (s >> 10) << 52;
        return bit_cast<double>(bit_cast<uint64_t>(b1 * b2) + exponent);
    }
}

float IRAM_ATTR exactExp(float x) {
    if (x < -104.0f) { return 0.0f; }
    if (x > 0x1.62e42ep+6f) { return HUGE_VALF; }

    constexpr double R = 0x3.p+51f;
    constexpr double iln2 = 0x1.71547652b82fep+10;
    constexpr double ln2h = 0x1.62e42fefc0000p-11;
    constexpr double ln2l = -0x1.c610ca86c3899p-47;

    const double k_R = std::fma(static_cast<double>(x), iln2, R);
    const double k = k_R - R;
    const double t = std::fma(k, -ln2l, std::fma(k, -ln2h, static_cast<double>(x)));
    const double exp_s = exp_table(bit_cast<uint64_t>(k_R));
    const double expm1_t = expm1_taylor3(t);
    const double exp_x = std::fma(exp_s, expm1_t, exp_s);
    return static_cast<float>(exp_x);
}

// 安全なべき乗関数（NaN/Inf処理付き）
float IRAM_ATTR safePow(float base, float exponent) {
    if (std::isnan(base) || std::isnan(exponent)) {
        return NAN;
    }
    
    float abs_base = std::abs(base);
    float abs_exp = std::abs(exponent);
    
    if (std::isinf(exponent)) {
        if (abs_base == 1.0f) {
            return 1.0f;
        } else if (abs_base > 1.0f) {
            return (exponent > 0.0f) ? HUGE_VALF : 0.0f;
        } else {
            return (exponent > 0.0f) ? 0.0f : HUGE_VALF;
        }
    }
    
    if (abs_exp == 0.0f) {
        return 1.0f;
    }
    
    if (abs_exp == 1.0f) {
        return (exponent > 0.0f) ? base : 1.0f / base;
    }
    
    if (exponent == 2.0f) {
        return base * base;
    }
    
    if (exponent == 0.5f && base >= 0.0f) {
        return fastSqrt(base);
    }
    
    if (base < 0.0f && exponent > std::floor(exponent)) {
        return NAN;  // 負数の非整数べき乗
    }
    
    // 整数べき乗の場合は高速計算を使用
    if (exponent == std::floor(exponent)) {
        return fastPow(base, static_cast<int>(exponent));
    }
    
    // それ以外は標準ライブラリを使用
    return std::pow(base, exponent);
}

// NaN/Inf チェック
bool isNanOrInf(float value) {
    return std::isnan(value) || std::isinf(value);
}

} // namespace MathUtils