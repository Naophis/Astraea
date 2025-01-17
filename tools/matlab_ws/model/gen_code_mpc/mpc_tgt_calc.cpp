#include "mpc_tgt_calc.h"
#include "rtwtypes.h"
#include "bus.h"
#include <cmath>
#include "mpc_tgt_calc_private.h"
#include "cmath"
#include <cstdint>
#include <cstring>
#include <iostream>

void mpc_tgt_calcModelClass::mpc_tgt_calc_IfActionSubsystem(real32_T rty_Out1[2],
  P_IfActionSubsystem_mpc_tgt_c_T *localP)
{
  rty_Out1[0] = localP->Constant1_Value;
  rty_Out1[1] = localP->Constant2_Value;
}

// void mpc_tgt_calcModelClass::mpc_tgt_calc_keep(real32_T *rty_accl_out, int32_T
//   *rty_state_out, P_keep_mpc_tgt_calc_T *localP)
// {
//   *rty_accl_out = static_cast<real32_T>(localP->Constant_Value);
//   *rty_state_out = localP->Constant2_Value;
// }

union {
  float f;
  uint32_t i;
} conv;

real32_T t_sqrtF(const real32_T &x) {
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
  return 1.0f / conv.f;
}
real32_T fast_pow(real32_T x, int n) {
  if (n == 0) {
    return 1.0;
  }
  if (n < 0) {
    x = 1.0 / x;
    n = -n;
  }
  real32_T result = 1.0;
  while (n > 0) {
    if (n % 2 == 1) {
      result *= x;
    }
    x *= x;
    n /= 2;
  }
  return result;
}

template<class To, class From>
To bit_cast(const From &from) noexcept {
    To to;
    static_assert(sizeof to == sizeof from);
    std::memcpy(&to, &from, sizeof to);
    return to;
}

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
                0x1.0000000000000p+0,
                0x1.059b0d3158574p+0,
                0x1.0b5586cf9890fp+0,
                0x1.11301d0125b51p+0,
                0x1.172b83c7d517bp+0,
                0x1.1d4873168b9aap+0,
                0x1.2387a6e756238p+0,
                0x1.29e9df51fdee1p+0,
                0x1.306fe0a31b715p+0,
                0x1.371a7373aa9cbp+0,
                0x1.3dea64c123422p+0,
                0x1.44e086061892dp+0,
                0x1.4bfdad5362a27p+0,
                0x1.5342b569d4f82p+0,
                0x1.5ab07dd485429p+0,
                0x1.6247eb03a5585p+0,
                0x1.6a09e667f3bcdp+0,
                0x1.71f75e8ec5f74p+0,
                0x1.7a11473eb0187p+0,
                0x1.82589994cce13p+0,
                0x1.8ace5422aa0dbp+0,
                0x1.93737b0cdc5e5p+0,
                0x1.9c49182a3f090p+0,
                0x1.a5503b23e255dp+0,
                0x1.ae89f995ad3adp+0,
                0x1.b7f76f2fb5e47p+0,
                0x1.c199bdd85529cp+0,
                0x1.cb720dcef9069p+0,
                0x1.d5818dcfba487p+0,
                0x1.dfc97337b9b5fp+0,
                0x1.ea4afa2a490dap+0,
                0x1.f50765b6e4540p+0,
        };

        constexpr double b2table[32]{
                0x1.0000000000000p+0,
                0x1.002c605e2e8cfp+0,
                0x1.0058c86da1c0ap+0,
                0x1.0085382faef83p+0,
                0x1.00b1afa5abcbfp+0,
                0x1.00de2ed0ee0f5p+0,
                0x1.010ab5b2cbd11p+0,
                0x1.0137444c9b5b5p+0,
                0x1.0163da9fb3335p+0,
                0x1.019078ad6a19fp+0,
                0x1.01bd1e77170b4p+0,
                0x1.01e9cbfe113efp+0,
                0x1.02168143b0281p+0,
                0x1.02433e494b755p+0,
                0x1.027003103b10ep+0,
                0x1.029ccf99d720ap+0,
                0x1.02c9a3e778061p+0,
                0x1.02f67ffa765e6p+0,
                0x1.032363d42b027p+0,
                0x1.03504f75ef071p+0,
                0x1.037d42e11bbccp+0,
                0x1.03aa3e170aafep+0,
                0x1.03d7411915a8ap+0,
                0x1.04044be896ab6p+0,
                0x1.04315e86e7f85p+0,
                0x1.045e78f5640b9p+0,
                0x1.048b9b35659d8p+0,
                0x1.04b8c54847a28p+0,
                0x1.04e5f72f654b1p+0,
                0x1.051330ec1a03fp+0,
                0x1.0540727fc1762p+0,
                0x1.056dbbebb786bp+0,
        };

        const double b1 = b1table[s >> 5 & 31];
        const double b2 = b2table[s & 31];
        const uint64_t exponent = (s >> 10) << 52;
        return bit_cast<double>(bit_cast<uint64_t>(b1 * b2) + exponent);
    }
}

float exact_expf(float x) noexcept {
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
    return static_cast<float>( exp_x );
}
real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaNF);
  } else {
    real32_T tmp;
    real32_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (std::isinf(u1)) {
      if (tmp == 1.0F) {
        y = 1.0F;
      } else if (tmp > 1.0F) {
        if (u1 > 0.0F) {
          y = (rtInfF);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = (rtInfF);
      }
    } else if (tmp_0 == 0.0F) {
      y = 1.0F;
    } else if (tmp_0 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if (u1 == 0.5F && u0 >= 0.0F) {
      // y = std::sqrt(u0);
      y = t_sqrtF(u0);
    } else if (u0 < 0.0F && u1 > std::floor(u1)) {
      y = (rtNaNF);
    } else {
      // y = std::pow(u0, u1);
      y = fast_pow(u0, u1);
    }
  }

  return y;
}

void mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics
  *arg_ego1)
{
  t_ego rtb_BusAssignment1_o;
  t_ego rtb_BusAssignment_m;
  real_T rtb_Switch_e;
  int32_T rtb_Merge1;
  int32_T rtb_Merge2;
  int32_T rtb_pivot_state;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Divide_o;
  real32_T rtb_Gain2_ht;
  real32_T rtb_Gain4;
  real32_T rtb_Subtract2_k;
  real32_T rtb_Switch1_n_idx_1;
  real32_T rtb_resist;
  boolean_T rtb_RelationalOperator_a;
  if (arg_tgt->v_max >= 0.0F) {
    rtb_Abs7 = arg_tgt->tgt_dist - arg_ego->dist;
    if (arg_ego->v - arg_tgt->end_v > mpc_tgt_calc_P.Constant3_Value_a &&
        (arg_ego->state == mpc_tgt_calc_P.Constant1_Value_a || std::abs
         (arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v) /
         (mpc_tgt_calc_P.Gain1_Gain_o0 * std::abs(arg_tgt->decel)) +
         arg_ego->dist >= arg_tgt->tgt_dist)) {
      if (arg_ego->v > arg_tgt->end_v) {
        if (rtb_Abs7 > mpc_tgt_calc_P.Constant3_Value) {
          rtb_Switch_e = mpc_tgt_calc_P.Gain_Gain_n * rtb_Abs7;
        } else {
          rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_k;
        }

        rtb_Switch_e = std::abs((arg_ego->v * arg_ego->v - arg_tgt->end_v *
          arg_tgt->end_v) / rtb_Switch_e) * mpc_tgt_calc_P.Gain1_Gain;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_nl;
      }

      rtb_Merge2 = arg_ego->decel_delay_cnt + mpc_tgt_calc_P.Constant5_Value;
      if (arg_ego->v < arg_tgt->accl_param.limit) {
        rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
      } else {
        rtb_Abs6 = (static_cast<real32_T>(mpc_tgt_calc_P.Constant4_Value) -
                    rt_powf_snf(static_cast<real32_T>
          (mpc_tgt_calc_P.Constant4_Value) - std::fmin(static_cast<real32_T>
          (rtb_Merge2) / static_cast<real32_T>
          (arg_tgt->accl_param.decel_delay_cnt), static_cast<real32_T>
          (mpc_tgt_calc_P.Constant6_Value)), arg_tgt->accl_param.decel_delay_n))
          * static_cast<real32_T>(rtb_Switch_e);
      }

      if (!(mpc_tgt_calc_P.dt * rtb_Abs6 * static_cast<real32_T>(arg_time_step)
            + arg_ego->v > arg_tgt->end_v)) {
        rtb_Abs6 = (arg_tgt->end_v - arg_ego->v) / (mpc_tgt_calc_P.dt *
          static_cast<real32_T>(arg_time_step));
      }

      if (rtb_Merge2 > arg_tgt->accl_param.decel_delay_cnt) {
        rtb_pivot_state = arg_tgt->accl_param.decel_delay_cnt;
      } else {
        rtb_pivot_state = rtb_Merge2;
      }

      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_p;
    } else if (arg_ego->state == mpc_tgt_calc_P.Constant2_Value) {
      if (arg_ego->v < arg_tgt->v_max) {
        if (arg_ego->v > arg_tgt->accl_param.limit) {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_c - fast_pow(arg_ego->v /
            arg_tgt->v_max, arg_tgt->accl_param.n)) * (arg_tgt->accl *
            arg_tgt->axel_degenerate_gain);
        } else {
          rtb_Abs7 = arg_tgt->accl * arg_tgt->axel_degenerate_gain;
        }

        rtb_Switch_e = rtb_Abs7;
        rtb_Merge2 = mpc_tgt_calc_P.Constant1_Value_lj;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_c;
        rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_gw;
      }

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant4_Value_f;
    } else {
      rtb_Abs6 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_cm);
      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_pg;
      rtb_pivot_state = mpc_tgt_calc_P.Constant4_Value_o;
    }

    rtb_resist = std::fmax(std::fmin(arg_tgt->v_max, mpc_tgt_calc_P.dt *
      rtb_Abs6 * static_cast<real32_T>(arg_time_step) + arg_ego->v),
      mpc_tgt_calc_P.Constant_Value_nex);
  } else {
    rtb_Abs7 = std::abs(arg_ego->dist);
    rtb_Abs6 = std::abs(arg_tgt->tgt_dist);
    if (arg_ego->state == 2.0F || std::abs(std::abs(arg_ego->v * arg_ego->v -
          arg_tgt->end_v * arg_tgt->end_v) / (mpc_tgt_calc_P.Gain1_Gain_nz * std::
          abs(arg_tgt->decel))) + rtb_Abs7 >= rtb_Abs6) {
      if (arg_ego->v < arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_e == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          rtb_Switch_e = std::abs((arg_ego->v * arg_ego->v - arg_tgt->end_v *
            arg_tgt->end_v) / (std::fmax(mpc_tgt_calc_P.Constant1_Value_h,
            static_cast<real_T>(rtb_Abs6 - rtb_Abs7)) *
                               mpc_tgt_calc_P.Gain_Gain_i)) *
            mpc_tgt_calc_P.Gain1_Gain_c;
        }
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cx;
      }

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_i;
    } else if (arg_ego->state == 0.0F) {
      if (arg_ego->v > arg_tgt->v_max) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_b == 1) {
          rtb_Abs7 = arg_tgt->accl;
        } else {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_cn - fast_pow(arg_ego->v
            / arg_tgt->v_max, mpc_tgt_calc_P.Constant4_Value_c)) * arg_tgt->accl;
        }

        rtb_Switch_e = rtb_Abs7;
        rtb_Merge2 = mpc_tgt_calc_P.Constant1_Value_kv;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cf;
        rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_d;
      }

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Abs6 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_e);
      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_dw;
    }

    rtb_resist = std::fmin(std::fmax(arg_tgt->v_max, mpc_tgt_calc_P.dt *
      rtb_Abs6 * static_cast<real32_T>(arg_time_step) + arg_ego->v),
      mpc_tgt_calc_P.Constant_Value_hs);
    rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_n;
  }

  rtb_Abs7 = rtb_resist * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.dt;
  if (arg_mode == 1) {
    rtb_Merge1 = arg_time_step + arg_ego->sla_param.counter;
    if (rtb_Merge1 > arg_ego->sla_param.limit_time_count) {
      mpc_tgt_calc_IfActionSubsystem(mpc_tgt_calc_B.Merge_p,
        &mpc_tgt_calc_P.IfActionSubsystem_g);
    } else {
      rtb_Divide_o = static_cast<real32_T>(rtb_Merge1) * mpc_tgt_calc_P.dt /
        arg_ego->sla_param.base_time;
      rtb_Subtract2_k = rtb_Divide_o - mpc_tgt_calc_P.Constant_Value_ne;
      rtb_Gain2_ht = fast_pow(rtb_Subtract2_k, arg_ego->sla_param.pow_n -
        mpc_tgt_calc_P.Constant1_Value_mn);
      rtb_Subtract2_k *= rtb_Gain2_ht;
      rtb_Gain4 = mpc_tgt_calc_P.Constant6_Value_o / (rtb_Subtract2_k -
        mpc_tgt_calc_P.Constant5_Value_i);
      rtb_Subtract2_k -= mpc_tgt_calc_P.Constant2_Value_e;
      mpc_tgt_calc_B.Merge_p[0] = mpc_tgt_calc_P.Gain1_Gain_l *
        arg_ego->sla_param.pow_n * rtb_Gain2_ht / (rtb_Subtract2_k *
        rtb_Subtract2_k) * std::exp(mpc_tgt_calc_P.Constant4_Value_b + rtb_Gain4)
        / arg_ego->sla_param.base_time;
      mpc_tgt_calc_B.Merge_p[1] = std::exp(mpc_tgt_calc_P.Constant8_Value /
        (mpc_tgt_calc_P.Constant7_Value - fast_pow(rtb_Divide_o -
        mpc_tgt_calc_P.Constant10_Value, arg_ego->sla_param.pow_n))) *
        mpc_tgt_calc_P.Constant3_Value_j;
    }

    if (arg_ego->sla_param.state != 0) {
      rtb_Divide_o = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Divide_o = arg_ego->sla_param.base_alpha * mpc_tgt_calc_B.Merge_p[0];
    }

    if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting == 1) {
      if (arg_ego->sla_param.counter >= arg_ego->sla_param.limit_time_count) {
        rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.dt * rtb_Divide_o * static_cast<real32_T>
          (arg_time_step) + arg_ego->w;
      }
    } else if (arg_ego->sla_param.state != 0) {
      rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Switch_e = arg_ego->sla_param.base_alpha * mpc_tgt_calc_B.Merge_p[1];
    }

    rtb_Switch1_n_idx_1 = static_cast<real32_T>(arg_tgt->time_step2 +
      arg_ego->sla_param.counter);
    if (rtb_Switch1_n_idx_1 > arg_ego->sla_param.limit_time_count) {
      mpc_tgt_calc_IfActionSubsystem(mpc_tgt_calc_B.Merge1,
        &mpc_tgt_calc_P.IfActionSubsystem2);
    } else {
      rtb_Subtract2_k = rtb_Switch1_n_idx_1 * mpc_tgt_calc_P.dt /
        arg_ego->sla_param.base_time;
      rtb_Gain2_ht = rtb_Subtract2_k - mpc_tgt_calc_P.Constant_Value_f;
      rtb_Gain4 = fast_pow(rtb_Gain2_ht, arg_ego->sla_param.pow_n -
        mpc_tgt_calc_P.Constant1_Value_i);
      rtb_Gain2_ht *= rtb_Gain4;
      rtb_Switch1_n_idx_1 = mpc_tgt_calc_P.Constant6_Value_oh / (rtb_Gain2_ht -
        mpc_tgt_calc_P.Constant5_Value_j);
      rtb_Gain2_ht -= mpc_tgt_calc_P.Constant2_Value_m;
      mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Gain1_Gain_d *
        arg_ego->sla_param.pow_n * rtb_Gain4 / (rtb_Gain2_ht * rtb_Gain2_ht) *
        std::exp(mpc_tgt_calc_P.Constant4_Value_d + rtb_Switch1_n_idx_1) /
        arg_ego->sla_param.base_time;
      mpc_tgt_calc_B.Merge1[1] = std::exp(mpc_tgt_calc_P.Constant8_Value_k /
        (mpc_tgt_calc_P.Constant7_Value_a - fast_pow(rtb_Subtract2_k -
        mpc_tgt_calc_P.Constant10_Value_f, arg_ego->sla_param.pow_n))) *
        mpc_tgt_calc_P.Constant3_Value_k;
    }

    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = static_cast<real32_T>(rtb_Switch_e);
    rtb_BusAssignment1_o.alpha = rtb_Divide_o;
    rtb_BusAssignment1_o.sla_param.counter = rtb_Merge1;
    if (arg_ego->sla_param.state != 0) {
      rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_BusAssignment1_o.alpha2 = arg_ego->sla_param.base_alpha *
        mpc_tgt_calc_B.Merge1[0];
    }
  } else if (arg_mode == 2) {
    rtb_Divide_o = std::abs(arg_ego->ang);
    rtb_Gain2_ht = mpc_tgt_calc_P.Gain2_Gain * arg_tgt->alpha;
    rtb_Subtract2_k = std::abs(arg_tgt->tgt_angle);
    if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Gain4 = arg_tgt->w_max;
    } else {
      rtb_Gain4 = mpc_tgt_calc_P.Gain3_Gain * arg_tgt->w_max;
    }

    rtb_Switch1_n_idx_1 = arg_ego->w * arg_ego->w - arg_tgt->end_w *
      arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Switch1_n_idx_1) /
        (mpc_tgt_calc_P.Gain1_Gain_m * std::abs(rtb_Gain2_ht)) + rtb_Divide_o >=
        rtb_Subtract2_k) {
      if (rtb_Gain2_ht > mpc_tgt_calc_P.Switch2_Threshold) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_o != 1) {
          if (rtb_Gain2_ht > mpc_tgt_calc_P.Switch1_Threshold) {
            rtb_Gain2_ht = static_cast<real32_T>(std::abs((arg_ego->w *
              arg_ego->w - arg_tgt->end_w * arg_tgt->end_w) / (std::fmax
              (mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>
               (rtb_Subtract2_k - rtb_Divide_o)) * mpc_tgt_calc_P.Gain_Gain)));
          } else {
            rtb_Gain2_ht = static_cast<real32_T>(std::abs(rtb_Switch1_n_idx_1 /
              (std::fmax(mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>
                         (rtb_Subtract2_k - rtb_Divide_o)) *
               mpc_tgt_calc_P.Gain_Gain))) * mpc_tgt_calc_P.Gain1_Gain_h;
          }
        }
      } else {
        rtb_Gain2_ht = mpc_tgt_calc_P.Constant_Value_ph;
      }

      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_g;
    } else if (arg_ego->pivot_state == 0.0F) {
      if (std::abs(arg_ego->w) < std::abs(rtb_Gain4)) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_c == 1) {
          rtb_Gain2_ht = arg_tgt->alpha;
        } else {
          rtb_Gain2_ht = (mpc_tgt_calc_P.Constant3_Value_f - fast_pow
                          (arg_ego->w / rtb_Gain4,
                           mpc_tgt_calc_P.Constant4_Value_i)) * arg_tgt->alpha;
        }

        rtb_Switch_e = rtb_Gain2_ht;
        rtb_Merge1 = mpc_tgt_calc_P.Constant1_Value_p;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value;
        rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_f;
      }

      rtb_Gain2_ht = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Gain2_ht = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_n);
      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_g3;
    }

    rtb_BusAssignment_m = *arg_ego;
    rtb_BusAssignment_m.w = mpc_tgt_calc_P.dt * rtb_Gain2_ht *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_m.alpha = rtb_Gain2_ht;
    rtb_BusAssignment_m.alpha2 = rtb_Gain2_ht;
    rtb_BusAssignment_m.pivot_state = rtb_Merge1;
    rtb_BusAssignment1_o = rtb_BusAssignment_m;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_c;
    if (rtb_Divide_o < rtb_Subtract2_k) {
      rtb_BusAssignment1_o = rtb_BusAssignment_m;
    }
  } else if (arg_mode == 4) {
    rtb_Divide_o = std::abs(arg_ego->img_ang);
    rtb_Gain2_ht = mpc_tgt_calc_P.Gain2_Gain_i * arg_tgt->alpha;
    rtb_Subtract2_k = std::abs(arg_tgt->tgt_angle);
    if (arg_ego->sla_param.pow_n != mpc_tgt_calc_P.Constant_Value_g) {
      rtb_Gain4 = mpc_tgt_calc_P.Gain6_Gain * rtb_Subtract2_k;
      rtb_Switch1_n_idx_1 = mpc_tgt_calc_P.Gain7_Gain * rtb_Subtract2_k;
    } else {
      rtb_Gain4 = mpc_tgt_calc_P.Gain4_Gain * rtb_Subtract2_k;
      rtb_Switch1_n_idx_1 = rtb_Gain4;
    }

    if (arg_ego->pivot_state == 2.0F || std::abs(arg_ego->w * arg_ego->w -
         arg_tgt->end_w * arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_hy * std::
         abs(rtb_Gain2_ht)) + rtb_Divide_o >= mpc_tgt_calc_P.Gain_Gain_m *
        rtb_Subtract2_k || rtb_Divide_o >= rtb_Gain4) {
      if (rtb_Gain2_ht > mpc_tgt_calc_P.Switch2_Threshold_m) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (rtb_Gain2_ht > mpc_tgt_calc_P.Switch1_Threshold_i) {
          rtb_Gain2_ht = static_cast<real32_T>(std::abs((arg_ego->w * arg_ego->w
            - arg_tgt->end_w * arg_tgt->end_w) / (std::fmax(static_cast<real_T>
            (rtb_Subtract2_k - rtb_Divide_o), mpc_tgt_calc_P.Constant1_Value_m) *
            mpc_tgt_calc_P.Gain_Gain_g)));
        } else {
          rtb_Gain2_ht = static_cast<real32_T>(std::abs((arg_ego->w * arg_ego->w
            - arg_tgt->end_w * arg_tgt->end_w) / (std::fmax(static_cast<real_T>
            (rtb_Subtract2_k - rtb_Divide_o), mpc_tgt_calc_P.Constant1_Value_m) *
            mpc_tgt_calc_P.Gain_Gain_g))) * mpc_tgt_calc_P.Gain1_Gain_n;
        }
      } else {
        rtb_Gain2_ht = mpc_tgt_calc_P.Constant_Value_h;
      }

      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_f2;
    } else if (arg_ego->pivot_state == 0.0F && rtb_Divide_o <
               rtb_Switch1_n_idx_1) {
      rtb_Gain2_ht = arg_tgt->alpha;
      rtb_Merge1 = mpc_tgt_calc_P.Constant1_Value_o;
    } else {
      rtb_Gain2_ht = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_p);
      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_j;
    }

    rtb_BusAssignment_m = *arg_ego;
    rtb_BusAssignment_m.w = mpc_tgt_calc_P.dt * rtb_Gain2_ht *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_m.alpha = rtb_Gain2_ht;
    rtb_BusAssignment_m.pivot_state = rtb_Merge1;
    rtb_BusAssignment1_o = rtb_BusAssignment_m;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_e;
    if (rtb_Divide_o < rtb_Subtract2_k) {
      rtb_BusAssignment1_o = rtb_BusAssignment_m;
    }
  } else {
    rtb_Divide_o = (mpc_tgt_calc_P.Constant_Value_eh - arg_ego->w) /
      (mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step));
    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_eh;
    rtb_BusAssignment1_o.alpha = rtb_Divide_o;
    rtb_BusAssignment1_o.alpha2 = rtb_Divide_o;
  }

  rtb_Divide_o = rtb_BusAssignment1_o.w * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.dt;
  if (mpc_tgt_calc_P.Constant4_Value_m != 0 && arg_tgt->enable_slip_decel != 0)
  {
    rtb_Abs6 = mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step);
    rtb_resist = (mpc_tgt_calc_P.Constant_Value_o / arg_ego1->mass +
                  rtb_BusAssignment1_o.w * arg_ego->slip.vy) * rtb_Abs6 +
      arg_ego->slip.vx;
    rtb_Subtract2_k = (mpc_tgt_calc_P.Gain3_Gain_d * arg_tgt->slip_gain_K1 *
                       arg_ego->slip.beta / arg_ego1->mass -
                       rtb_BusAssignment1_o.w * arg_ego->slip.vx) * rtb_Abs6 +
      arg_ego->slip.vy;
    rtb_Gain2_ht = t_sqrtF(rtb_resist * rtb_resist + rtb_Subtract2_k *
      rtb_Subtract2_k);
    rtb_Gain4 = mpc_tgt_calc_P.Gain1_Gain_o * rtb_Gain2_ht;
    rtb_Switch1_n_idx_1 = (rtb_Gain4 - arg_ego->v) / rtb_Abs6;
    rtb_BusAssignment1_o.v_r = arg_ego->v_r;
    rtb_BusAssignment1_o.v_l = arg_ego->v_l;
    rtb_BusAssignment1_o.dist = arg_ego->dist + rtb_Abs7;
    rtb_BusAssignment1_o.ang = arg_ego->ang + rtb_Divide_o;
    rtb_BusAssignment1_o.img_dist = arg_ego->img_dist + rtb_Abs7;
    rtb_BusAssignment1_o.img_ang = arg_ego->img_ang + rtb_Divide_o;
    rtb_BusAssignment1_o.sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    rtb_BusAssignment1_o.sla_param.base_time = arg_ego->sla_param.base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    rtb_BusAssignment1_o.sla_param.pow_n = arg_ego->sla_param.pow_n;
    rtb_BusAssignment1_o.sla_param.state = arg_ego->sla_param.state;
    rtb_BusAssignment1_o.state = rtb_Merge2;
    rtb_BusAssignment1_o.ideal_point = arg_ego->ideal_point;
    rtb_BusAssignment1_o.slip_point = arg_ego->slip_point;
    rtb_BusAssignment1_o.kanayama_point = arg_ego->kanayama_point;
    rtb_BusAssignment1_o.trj_diff = arg_ego->trj_diff;
    rtb_BusAssignment1_o.delay_accl = arg_ego->delay_accl;
    rtb_BusAssignment1_o.delay_v = arg_ego->delay_v;
    rtb_BusAssignment1_o.cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
    rtb_BusAssignment1_o.cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;
    rtb_BusAssignment1_o.ff_duty_l = arg_ego->ff_duty_l;
    rtb_BusAssignment1_o.ff_duty_r = arg_ego->ff_duty_r;
    rtb_BusAssignment1_o.ff_duty_low_th = arg_ego->ff_duty_low_th;
    rtb_BusAssignment1_o.ff_duty_low_v_th = arg_ego->ff_duty_low_v_th;
    rtb_BusAssignment1_o.ff_duty_front = arg_ego->ff_duty_front;
    rtb_BusAssignment1_o.ff_duty_roll = arg_ego->ff_duty_roll;
    rtb_BusAssignment1_o.ff_duty_rpm_r = arg_ego->ff_duty_rpm_r;
    rtb_BusAssignment1_o.ff_duty_rpm_l = arg_ego->ff_duty_rpm_l;
    rtb_BusAssignment1_o.ff_front_torque = arg_ego->ff_front_torque;
    rtb_BusAssignment1_o.ff_roll_torque = arg_ego->ff_roll_torque;
    rtb_BusAssignment1_o.decel_delay_cnt = rtb_pivot_state;
    rtb_BusAssignment1_o.slip.beta = (arg_ego->slip.beta / rtb_Abs6 -
      rtb_BusAssignment1_o.w) / (mpc_tgt_calc_P.Constant1_Value_ih / rtb_Abs6 +
      arg_tgt->slip_gain_K2 / rtb_Gain2_ht);
    rtb_BusAssignment1_o.slip.vx = rtb_resist;
    rtb_BusAssignment1_o.slip.vy = rtb_Subtract2_k;
    rtb_BusAssignment1_o.slip.v = rtb_Gain2_ht;
    rtb_BusAssignment1_o.slip.accl = rtb_Switch1_n_idx_1;
    rtb_BusAssignment1_o.v = rtb_Gain4;
    rtb_BusAssignment1_o.accl = rtb_Switch1_n_idx_1;
  } else {
    rtb_BusAssignment1_o.v = rtb_resist;
    rtb_BusAssignment1_o.v_r = arg_ego->v_r;
    rtb_BusAssignment1_o.v_l = arg_ego->v_l;
    rtb_BusAssignment1_o.accl = rtb_Abs6;
    rtb_BusAssignment1_o.dist = arg_ego->dist + rtb_Abs7;
    rtb_BusAssignment1_o.ang = arg_ego->ang + rtb_Divide_o;
    rtb_BusAssignment1_o.img_dist = arg_ego->img_dist + rtb_Abs7;
    rtb_BusAssignment1_o.img_ang = arg_ego->img_ang + rtb_Divide_o;
    rtb_BusAssignment1_o.sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    rtb_BusAssignment1_o.sla_param.base_time = arg_ego->sla_param.base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    rtb_BusAssignment1_o.sla_param.pow_n = arg_ego->sla_param.pow_n;
    rtb_BusAssignment1_o.sla_param.state = arg_ego->sla_param.state;
    rtb_BusAssignment1_o.state = rtb_Merge2;
    rtb_BusAssignment1_o.ideal_point = arg_ego->ideal_point;
    rtb_BusAssignment1_o.slip_point = arg_ego->slip_point;
    rtb_BusAssignment1_o.kanayama_point = arg_ego->kanayama_point;
    rtb_BusAssignment1_o.trj_diff = arg_ego->trj_diff;
    rtb_BusAssignment1_o.delay_accl = arg_ego->delay_accl;
    rtb_BusAssignment1_o.delay_v = arg_ego->delay_v;
    rtb_BusAssignment1_o.cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
    rtb_BusAssignment1_o.cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;
    rtb_BusAssignment1_o.ff_duty_l = arg_ego->ff_duty_l;
    rtb_BusAssignment1_o.ff_duty_r = arg_ego->ff_duty_r;
    rtb_BusAssignment1_o.ff_duty_low_th = arg_ego->ff_duty_low_th;
    rtb_BusAssignment1_o.ff_duty_low_v_th = arg_ego->ff_duty_low_v_th;
    rtb_BusAssignment1_o.ff_duty_front = arg_ego->ff_duty_front;
    rtb_BusAssignment1_o.ff_duty_roll = arg_ego->ff_duty_roll;
    rtb_BusAssignment1_o.ff_duty_rpm_r = arg_ego->ff_duty_rpm_r;
    rtb_BusAssignment1_o.ff_duty_rpm_l = arg_ego->ff_duty_rpm_l;
    rtb_BusAssignment1_o.ff_front_torque = arg_ego->ff_front_torque;
    rtb_BusAssignment1_o.ff_roll_torque = arg_ego->ff_roll_torque;
    rtb_BusAssignment1_o.decel_delay_cnt = rtb_pivot_state;
    rtb_BusAssignment1_o.slip.beta = mpc_tgt_calc_P.Constant_Value_nu;
    rtb_BusAssignment1_o.slip.vx = mpc_tgt_calc_P.Gain1_Gain_i * rtb_resist;
    rtb_BusAssignment1_o.slip.vy = mpc_tgt_calc_P.Constant_Value_nu;
    rtb_BusAssignment1_o.slip.v = mpc_tgt_calc_P.Gain_Gain_h * rtb_resist;
    rtb_BusAssignment1_o.slip.accl = arg_ego->slip.accl;
  }

  if (std::isnan(rtb_BusAssignment1_o.accl) || std::isinf
      (rtb_BusAssignment1_o.accl)) {
    rtb_resist = mpc_tgt_calc_P.Constant_Value_h5;
    rtb_Divide_o = arg_ego->v;
    rtb_Abs6 = arg_ego->img_dist;
  } else {
    rtb_resist = rtb_BusAssignment1_o.accl;
    rtb_Divide_o = rtb_BusAssignment1_o.v;
    rtb_Abs6 = rtb_BusAssignment1_o.img_dist;
  }

  rtb_BusAssignment1_o.accl = rtb_resist;
  rtb_BusAssignment1_o.v = rtb_Divide_o;
  rtb_BusAssignment1_o.img_dist = rtb_Abs6;
  if (std::isnan(rtb_BusAssignment1_o.alpha) || std::isinf
      (rtb_BusAssignment1_o.alpha)) {
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant1_Value_e1;
    rtb_BusAssignment1_o.w = arg_ego->w;
    rtb_BusAssignment1_o.img_ang = arg_ego->img_ang;
  }

  if (std::isnan(arg_ego->alpha2) || std::isinf(arg_ego->alpha2)) {
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant4_Value_l;
  }

  rtb_Gain2_ht = arg_ego1->gear_ratio * arg_ego1->km;
  rtb_resist = mpc_tgt_calc_P.Gain4_Gain_c * rtb_resist * arg_ego1->mass *
    (mpc_tgt_calc_P.Gain1_Gain_b * arg_ego1->tire);
  rtb_Abs7 = rtb_resist * arg_ego1->resist / rtb_Gain2_ht;
  if ((arg_mode == 0 || arg_mode == 3) && rtb_BusAssignment1_o.state == 0) {
    if (rtb_Divide_o > rtb_BusAssignment1_o.ff_duty_low_v_th) {
      rtb_Divide_o = rtb_Abs7;
    } else {
      rtb_Divide_o = std::fmax(rtb_Abs7, rtb_BusAssignment1_o.ff_duty_low_th);
    }
  } else {
    rtb_Divide_o = rtb_Abs7;
  }

  rtb_Abs6 = rtb_BusAssignment1_o.alpha2 * arg_ego1->lm;
  rtb_Abs7 = mpc_tgt_calc_P.Gain1_Gain_it * arg_ego1->tire * rtb_Abs6 *
    arg_ego1->resist / rtb_Gain2_ht / (mpc_tgt_calc_P.Gain2_Gain_j *
    arg_ego1->tread);
  rtb_Subtract2_k = mpc_tgt_calc_P.Gain4_Gain_m * rtb_BusAssignment1_o.v;
  rtb_Gain2_ht = mpc_tgt_calc_P.Gain5_Gain * arg_ego1->tread *
    mpc_tgt_calc_P.Gain_Gain_k * rtb_BusAssignment1_o.w;
  rtb_Gain4 = mpc_tgt_calc_P.Gain6_Gain_f * arg_ego1->tire *
    mpc_tgt_calc_P.Gain1_Gain_i5;
  rtb_Switch1_n_idx_1 = (rtb_Subtract2_k - rtb_Gain2_ht) * arg_ego1->ke *
    mpc_tgt_calc_P.Gain2_Gain_d / rtb_Gain4;
  rtb_Gain4 = (rtb_Subtract2_k + rtb_Gain2_ht) * arg_ego1->ke *
    mpc_tgt_calc_P.Gain3_Gain_k / rtb_Gain4;
  *arg_next_ego = rtb_BusAssignment1_o;
  arg_next_ego->ff_duty_front = rtb_Divide_o;
  arg_next_ego->ff_duty_l = rtb_Divide_o - rtb_Abs7 + rtb_Switch1_n_idx_1;
  arg_next_ego->ff_duty_r = rtb_Divide_o + rtb_Abs7 + rtb_Gain4;
  arg_next_ego->ff_duty_roll = rtb_Abs7;
  arg_next_ego->ff_duty_rpm_r = rtb_Gain4;
  arg_next_ego->ff_duty_rpm_l = rtb_Switch1_n_idx_1;
  arg_next_ego->ff_front_torque = rtb_resist * mpc_tgt_calc_P.Gain_Gain_f;
  arg_next_ego->ff_roll_torque = rtb_Abs6 * arg_ego1->resist *
    mpc_tgt_calc_P.Gain3_Gain_i;
}

void mpc_tgt_calcModelClass::initialize()
{
  mpc_tgt_calc_B.Merge_p[0] = mpc_tgt_calc_P.Merge_InitialOutput;
  mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Merge1_InitialOutput;
  mpc_tgt_calc_B.Merge_p[1] = mpc_tgt_calc_P.Merge_InitialOutput;
  mpc_tgt_calc_B.Merge1[1] = mpc_tgt_calc_P.Merge1_InitialOutput;
}

void mpc_tgt_calcModelClass::terminate()
{
}

const char_T* mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T::getErrorStatus()
  const
{
  return (errorStatus);
}

void mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T::setErrorStatus(const
  char_T* const volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

mpc_tgt_calcModelClass::mpc_tgt_calcModelClass() :
  mpc_tgt_calc_B(),
  mpc_tgt_calc_M()
{
}

mpc_tgt_calcModelClass::~mpc_tgt_calcModelClass() = default;
mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * mpc_tgt_calcModelClass::getRTM
  ()
{
  return (&mpc_tgt_calc_M);
}
