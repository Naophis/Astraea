#include "mpc_tgt_calc.h"
#include "rtwtypes.h"
#include "bus.h"
#include "rtwtypes.h"
#include <cmath>
#include "mpc_tgt_calc_private.h"
#include "cmath"
#include <cstdint>
#include <cstring>
#include <iostream>
#include "esp_system.h"

union {
  float f;
  uint32_t i;
} conv;

real32_T IRAM_ATTR t_sqrtF(const real32_T &x) {
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f = conv.f * (1.5f - 0.5f * x * conv.f * conv.f);
  return 1.0f / conv.f;
}
real32_T IRAM_ATTR fast_pow(real32_T x, int n) {
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
To IRAM_ATTR bit_cast(const From &from) noexcept {
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

float IRAM_ATTR exact_expf(float x) noexcept {
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
real32_T IRAM_ATTR rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  return fast_pow(u0, u1);
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
      y = t_sqrtF(u0);
    } else if (u0 < 0.0F && u1 > std::floor(u1)) {
      y = (rtNaNF);
    } else {
      y = fast_pow(u0, u1);
    }
  }

  return y;
}

void IRAM_ATTR mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics
  *arg_ego1, int32_T *arg_In1)
{
  t_ego rtb_BusAssignment1;
  t_ego rtb_BusAssignment1_o;
  t_ego rtb_BusAssignment_b;
  real_T rtb_Switch_e;
  int32_T rtb_Merge1_decel_delay_cnt;
  int32_T rtb_Merge2;
  int32_T rtb_pivot_state;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Abs7_tmp;
  real32_T rtb_Add1_eh;
  real32_T rtb_Add2;
  real32_T rtb_Add2_i;
  real32_T rtb_Add3_kh;
  real32_T rtb_BusAssignment1_tmp;
  real32_T rtb_Divide1_a;
  real32_T rtb_Divide2;
  real32_T rtb_Divide_jy_tmp;
  real32_T rtb_Divide_l;
  real32_T rtb_Divide_n;
  real32_T rtb_Divide_o;
  real32_T rtb_FF_Right;
  real32_T rtb_Gain3_m;
  real32_T rtb_Merge1_alpha;
  real32_T rtb_Merge1_alpha2;
  real32_T rtb_Merge1_cnt_delay_accl_ratio;
  real32_T rtb_Merge1_cnt_delay_decel_rati;
  real32_T rtb_Merge1_delay_accl;
  real32_T rtb_Merge1_delay_v;
  real32_T rtb_Merge1_dist;
  real32_T rtb_Merge1_ff_duty_low_th;
  real32_T rtb_Merge1_ff_duty_low_v_th;
  real32_T rtb_Merge1_ideal_point_slip_ang;
  real32_T rtb_Merge1_ideal_point_theta;
  real32_T rtb_Merge1_ideal_point_v;
  real32_T rtb_Merge1_ideal_point_w;
  real32_T rtb_Merge1_ideal_point_x;
  real32_T rtb_Merge1_ideal_point_y;
  real32_T rtb_Merge1_ideal_px;
  real32_T rtb_Merge1_ideal_py;
  real32_T rtb_Merge1_kanayama_point_theta;
  real32_T rtb_Merge1_kanayama_point_v;
  real32_T rtb_Merge1_kanayama_point_w;
  real32_T rtb_Merge1_kanayama_point_x;
  real32_T rtb_Merge1_kanayama_point_y;
  real32_T rtb_Merge1_sla_param_base_time;
  real32_T rtb_Merge1_sla_param_limit_time;
  real32_T rtb_Merge1_sla_param_pow_n;
  real32_T rtb_Merge1_slip_beta;
  real32_T rtb_Merge1_slip_point_slip_angl;
  real32_T rtb_Merge1_slip_point_theta;
  real32_T rtb_Merge1_slip_point_v;
  real32_T rtb_Merge1_slip_point_w;
  real32_T rtb_Merge1_slip_point_x;
  real32_T rtb_Merge1_slip_point_y;
  real32_T rtb_Merge1_trj_diff_theta;
  real32_T rtb_Merge1_trj_diff_x;
  real32_T rtb_Merge1_trj_diff_y;
  real32_T rtb_Merge1_w;
  real32_T rtb_Power_f;
  real32_T rtb_Product2_l;
  real32_T rtb_Sqrt;
  real32_T rtb_Subtract2;
  real32_T rtb_Subtract2_k;
  real32_T rtb_Switch1_n_idx_1;
  boolean_T rtb_RelationalOperator_a;
  if (arg_tgt->v_max >= 0.0F) {
    rtb_Add2_i = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;
    if (arg_ego->v - arg_tgt->end_v > mpc_tgt_calc_P.Constant3_Value &&
        (arg_ego->state == mpc_tgt_calc_P.Constant1_Value || std::abs(rtb_Add2_i)
         / (mpc_tgt_calc_P.Gain1_Gain_o * std::abs(arg_tgt->decel)) +
         arg_ego->dist >= arg_tgt->tgt_dist)) {
      if (arg_ego->v > arg_tgt->end_v) {
        rtb_Abs7 = (arg_tgt->tgt_dist - arg_ego->dist) *
          mpc_tgt_calc_P.Gain_Gain_n;
        if (rtb_Abs7 > mpc_tgt_calc_P.Saturation_UpperSat) {
          rtb_Abs7 = mpc_tgt_calc_P.Saturation_UpperSat;
        } else if (rtb_Abs7 < mpc_tgt_calc_P.Saturation_LowerSat) {
          rtb_Abs7 = mpc_tgt_calc_P.Saturation_LowerSat;
        }

        rtb_Switch_e = std::abs(rtb_Add2_i / rtb_Abs7) *
          mpc_tgt_calc_P.Gain1_Gain_i;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value;
      }

      rtb_pivot_state = mpc_tgt_calc_DW.UnitDelay_DSTATE +
        mpc_tgt_calc_P.Constant5_Value;
      if (arg_ego->v < arg_tgt->accl_param.limit) {
        rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
      } else {
        rtb_Abs6 = (static_cast<real32_T>(mpc_tgt_calc_P.Constant4_Value) -
                    rt_powf_snf(static_cast<real32_T>
          (mpc_tgt_calc_P.Constant4_Value) - std::fmin(static_cast<real32_T>
          (rtb_pivot_state) / static_cast<real32_T>
          (arg_tgt->accl_param.decel_delay_cnt), static_cast<real32_T>
          (mpc_tgt_calc_P.Constant6_Value)), arg_tgt->accl_param.decel_delay_n))
          * static_cast<real32_T>(rtb_Switch_e);
      }

      if (!(mpc_tgt_calc_P.dt * rtb_Abs6 * static_cast<real32_T>(arg_time_step)
            + arg_ego->v > arg_tgt->end_v)) {
        rtb_Abs6 = (arg_tgt->end_v - arg_ego->v) / (mpc_tgt_calc_P.dt *
          static_cast<real32_T>(arg_time_step));
      }

      if (rtb_pivot_state > arg_tgt->accl_param.decel_delay_cnt) {
        rtb_Merge2 = arg_tgt->accl_param.decel_delay_cnt;
      } else {
        rtb_Merge2 = rtb_pivot_state;
      }

      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_p;
    } else if (arg_ego->state == mpc_tgt_calc_P.Constant2_Value) {
      if (arg_ego->v < arg_tgt->v_max) {
        if (arg_ego->v > arg_tgt->accl_param.limit) {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_c - rt_powf_snf(arg_ego->v /
            arg_tgt->v_max, arg_tgt->accl_param.n)) * (arg_tgt->accl *
            arg_tgt->axel_degenerate_gain);
        } else {
          rtb_Abs7 = arg_tgt->accl * arg_tgt->axel_degenerate_gain;
        }

        rtb_Switch_e = rtb_Abs7;
        rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_lj;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_c;
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_g;
      }

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
      rtb_Merge2 = mpc_tgt_calc_P.Constant4_Value_f;
    } else {
      rtb_Abs6 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_cm);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_pg;
      rtb_Merge2 = mpc_tgt_calc_P.Constant4_Value_o;
    }

    rtb_Abs7 = std::fmax(std::fmin(arg_tgt->v_max, mpc_tgt_calc_P.dt * rtb_Abs6 *
      static_cast<real32_T>(arg_time_step) + arg_ego->v),
                         mpc_tgt_calc_P.Constant_Value_ne);
    mpc_tgt_calc_DW.UnitDelay_DSTATE = rtb_Merge2;
  } else {
    rtb_Abs7 = std::abs(arg_ego->dist);
    rtb_Abs6 = std::abs(arg_tgt->tgt_dist);
    rtb_Add2_i = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;
    if (arg_ego->state == 2.0F || std::abs(std::abs(rtb_Add2_i) /
         (mpc_tgt_calc_P.Gain1_Gain_n * std::abs(arg_tgt->decel))) + rtb_Abs7 >=
        rtb_Abs6) {
      if (arg_ego->v < arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          rtb_Switch_e = std::abs(rtb_Add2_i / (std::fmax
            (mpc_tgt_calc_P.Constant1_Value_h, static_cast<real_T>(rtb_Abs6 -
            rtb_Abs7)) * mpc_tgt_calc_P.Gain_Gain)) * mpc_tgt_calc_P.Gain1_Gain;
        }
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cx;
      }

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_i;
    } else if (arg_ego->state == 0.0F) {
      if (arg_ego->v > arg_tgt->v_max) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_b == 1) {
          rtb_Abs7 = arg_tgt->accl;
        } else {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_cn - rt_powf_snf(arg_ego->v
            / arg_tgt->v_max, mpc_tgt_calc_P.Constant4_Value_c)) * arg_tgt->accl;
        }

        rtb_Switch_e = rtb_Abs7;
        rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_k;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cf;
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_d;
      }

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Abs6 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_dw;
    }

    rtb_Abs7 = std::fmin(std::fmax(arg_tgt->v_max, mpc_tgt_calc_P.dt * rtb_Abs6 *
      static_cast<real32_T>(arg_time_step) + arg_ego->v),
                         mpc_tgt_calc_P.Constant_Value_h);
  }

  rtb_Add2 = rtb_Abs7 * static_cast<real32_T>(arg_time_step) * mpc_tgt_calc_P.dt;
  rtb_BusAssignment1 = *arg_ego;
  rtb_BusAssignment1.v = rtb_Abs7;
  rtb_BusAssignment1.dist = arg_ego->dist + rtb_Add2;
  rtb_BusAssignment1.accl = rtb_Abs6;
  rtb_BusAssignment1.state = rtb_pivot_state;
  switch (arg_mode) {
   case 1:
    rtb_Merge2 = arg_time_step + arg_ego->sla_param.counter;
    if (rtb_Merge2 > arg_ego->sla_param.limit_time_count) {
      mpc_tgt_calc_B.Merge[0] = mpc_tgt_calc_P.Constant1_Value_dt;
      mpc_tgt_calc_B.Merge[1] = mpc_tgt_calc_P.Constant2_Value_ib;
    } else {
      rtb_Divide_o = static_cast<real32_T>(rtb_Merge2) * mpc_tgt_calc_P.dt /
        arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_ne4;
      rtb_Power_f = rt_powf_snf(rtb_Divide_o, arg_ego->sla_param.pow_n -
        mpc_tgt_calc_P.Constant1_Value_mn);
      rtb_Divide_o *= rtb_Power_f;
      rtb_Subtract2_k = rtb_Divide_o - mpc_tgt_calc_P.Constant2_Value_e;
      mpc_tgt_calc_B.Merge[0] = mpc_tgt_calc_P.Gain1_Gain_l *
        arg_ego->sla_param.pow_n * rtb_Power_f / (rtb_Subtract2_k *
        rtb_Subtract2_k) * std::exp(mpc_tgt_calc_P.Constant6_Value_o /
        rtb_Subtract2_k + mpc_tgt_calc_P.Constant4_Value_b) /
        arg_ego->sla_param.base_time;
      mpc_tgt_calc_B.Merge[1] = std::exp(mpc_tgt_calc_P.Constant8_Value /
        (mpc_tgt_calc_P.Constant7_Value - rtb_Divide_o)) *
        mpc_tgt_calc_P.Constant3_Value_j;
    }

    if (arg_ego->sla_param.state != 0) {
      rtb_Divide_o = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Divide_o = arg_ego->sla_param.base_alpha * mpc_tgt_calc_B.Merge[0];
    }

    if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_l == 1) {
      if (arg_ego->sla_param.counter >= arg_ego->sla_param.limit_time_count) {
        rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_d;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.dt * rtb_Divide_o * static_cast<real32_T>
          (arg_time_step) + arg_ego->w;
      }
    } else if (arg_ego->sla_param.state != 0) {
      rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Switch_e = arg_ego->sla_param.base_alpha * mpc_tgt_calc_B.Merge[1];
    }

    rtb_Add2_i = static_cast<real32_T>(arg_tgt->time_step2 +
      arg_ego->sla_param.counter);
    if (rtb_Add2_i > arg_ego->sla_param.limit_time_count) {
      mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Constant1_Value_pl;
      mpc_tgt_calc_B.Merge1[1] = mpc_tgt_calc_P.Constant2_Value_k;
    } else {
      rtb_Power_f = rtb_Add2_i * mpc_tgt_calc_P.dt /
        arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_f;
      rtb_Subtract2_k = rt_powf_snf(rtb_Power_f, arg_ego->sla_param.pow_n -
        mpc_tgt_calc_P.Constant1_Value_i);
      rtb_Power_f *= rtb_Subtract2_k;
      rtb_Subtract2 = rtb_Power_f - mpc_tgt_calc_P.Constant2_Value_m;
      mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Gain1_Gain_d5 *
        arg_ego->sla_param.pow_n * rtb_Subtract2_k / (rtb_Subtract2 *
        rtb_Subtract2) * std::exp(mpc_tgt_calc_P.Constant6_Value_oh /
        rtb_Subtract2 + mpc_tgt_calc_P.Constant4_Value_d) /
        arg_ego->sla_param.base_time;
      mpc_tgt_calc_B.Merge1[1] = std::exp(mpc_tgt_calc_P.Constant8_Value_k /
        (mpc_tgt_calc_P.Constant7_Value_a - rtb_Power_f)) *
        mpc_tgt_calc_P.Constant3_Value_k;
    }

    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = static_cast<real32_T>(rtb_Switch_e);
    rtb_BusAssignment1_o.alpha = rtb_Divide_o;
    rtb_BusAssignment1_o.sla_param.counter = rtb_Merge2;
    if (arg_ego->sla_param.state != 0) {
      rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_BusAssignment1_o.alpha2 = arg_ego->sla_param.base_alpha *
        mpc_tgt_calc_B.Merge1[0];
    }
    break;

   case 2:
    rtb_Divide_o = std::abs(arg_ego->ang);
    rtb_Subtract2_k = mpc_tgt_calc_P.Gain2_Gain_g * arg_tgt->alpha;
    rtb_Power_f = std::abs(arg_tgt->tgt_angle);
    if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Subtract2 = arg_tgt->w_max;
    } else {
      rtb_Subtract2 = mpc_tgt_calc_P.Gain3_Gain_p * arg_tgt->w_max;
    }

    rtb_Add2_i = arg_ego->w * arg_ego->w - arg_tgt->end_w * arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Add2_i) /
        (mpc_tgt_calc_P.Gain1_Gain_m * std::abs(rtb_Subtract2_k)) + rtb_Divide_o
        >= rtb_Power_f) {
      if (rtb_Subtract2_k > mpc_tgt_calc_P.Switch2_Threshold) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_o != 1) {
          if (rtb_Subtract2_k > mpc_tgt_calc_P.Switch1_Threshold) {
            rtb_Subtract2_k = static_cast<real32_T>(std::abs(rtb_Add2_i / (std::
              fmax(mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>
                   (rtb_Power_f - rtb_Divide_o)) * mpc_tgt_calc_P.Gain_Gain_j)));
          } else {
            rtb_Subtract2_k = static_cast<real32_T>(std::abs(rtb_Add2_i / (std::
              fmax(mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>
                   (rtb_Power_f - rtb_Divide_o)) * mpc_tgt_calc_P.Gain_Gain_j)))
              * mpc_tgt_calc_P.Gain1_Gain_h;
          }
        }
      } else {
        rtb_Subtract2_k = mpc_tgt_calc_P.Constant_Value_ph;
      }

      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_gm;
    } else if (arg_ego->pivot_state == 0.0F) {
      if (std::abs(arg_ego->w) < std::abs(rtb_Subtract2)) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_c == 1) {
          rtb_Subtract2_k = arg_tgt->alpha;
        } else {
          rtb_Subtract2_k = (mpc_tgt_calc_P.Constant3_Value_f - rt_powf_snf
                             (arg_ego->w / rtb_Subtract2,
                              mpc_tgt_calc_P.Constant4_Value_i)) *
            arg_tgt->alpha;
        }

        rtb_Switch_e = rtb_Subtract2_k;
        rtb_Merge2 = mpc_tgt_calc_P.Constant1_Value_p;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_k;
        rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_f;
      }

      rtb_Subtract2_k = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Subtract2_k = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_n);
      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_g3;
    }

    rtb_BusAssignment_b = *arg_ego;
    rtb_BusAssignment_b.w = mpc_tgt_calc_P.dt * rtb_Subtract2_k *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_b.alpha = rtb_Subtract2_k;
    rtb_BusAssignment_b.alpha2 = rtb_Subtract2_k;
    rtb_BusAssignment_b.pivot_state = rtb_Merge2;
    rtb_BusAssignment1_o = rtb_BusAssignment_b;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_c;
    if (rtb_Divide_o < rtb_Power_f) {
      rtb_BusAssignment1_o = rtb_BusAssignment_b;
    }
    break;

   case 4:
    rtb_Divide_o = std::abs(arg_ego->img_ang);
    rtb_Subtract2_k = mpc_tgt_calc_P.Gain2_Gain_i * arg_tgt->alpha;
    rtb_Power_f = std::abs(arg_tgt->tgt_angle);
    if (arg_ego->sla_param.pow_n != mpc_tgt_calc_P.Constant_Value_g) {
      rtb_Subtract2 = mpc_tgt_calc_P.Gain6_Gain_e * rtb_Power_f;
      rtb_Switch1_n_idx_1 = mpc_tgt_calc_P.Gain7_Gain * rtb_Power_f;
    } else {
      rtb_Subtract2 = mpc_tgt_calc_P.Gain4_Gain_b * rtb_Power_f;
      rtb_Switch1_n_idx_1 = rtb_Subtract2;
    }

    rtb_Add2_i = arg_ego->w * arg_ego->w - arg_tgt->end_w * arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Add2_i) /
        (mpc_tgt_calc_P.Gain1_Gain_hy * std::abs(rtb_Subtract2_k)) +
        rtb_Divide_o >= mpc_tgt_calc_P.Gain_Gain_mm * rtb_Power_f ||
        rtb_Divide_o >= rtb_Subtract2) {
      if (rtb_Subtract2_k > mpc_tgt_calc_P.Switch2_Threshold_m) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (rtb_Subtract2_k > mpc_tgt_calc_P.Switch1_Threshold_i) {
          rtb_Subtract2_k = static_cast<real32_T>(std::abs(rtb_Add2_i / (std::
            fmax(static_cast<real_T>(rtb_Power_f - rtb_Divide_o),
                 mpc_tgt_calc_P.Constant1_Value_m) * mpc_tgt_calc_P.Gain_Gain_g)));
        } else {
          rtb_Subtract2_k = static_cast<real32_T>(std::abs(rtb_Add2_i / (std::
            fmax(static_cast<real_T>(rtb_Power_f - rtb_Divide_o),
                 mpc_tgt_calc_P.Constant1_Value_m) * mpc_tgt_calc_P.Gain_Gain_g)))
            * mpc_tgt_calc_P.Gain1_Gain_nc;
        }
      } else {
        rtb_Subtract2_k = mpc_tgt_calc_P.Constant_Value_hr;
      }

      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_f2;
    } else if (arg_ego->pivot_state == 0.0F && rtb_Divide_o <
               rtb_Switch1_n_idx_1) {
      rtb_Subtract2_k = arg_tgt->alpha;
      rtb_Merge2 = mpc_tgt_calc_P.Constant1_Value_o;
    } else {
      rtb_Subtract2_k = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_p);
      rtb_Merge2 = mpc_tgt_calc_P.Constant2_Value_j;
    }

    rtb_BusAssignment_b = *arg_ego;
    rtb_BusAssignment_b.w = mpc_tgt_calc_P.dt * rtb_Subtract2_k *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_b.alpha = rtb_Subtract2_k;
    rtb_BusAssignment_b.pivot_state = rtb_Merge2;
    rtb_BusAssignment1_o = rtb_BusAssignment_b;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_e;
    if (rtb_Divide_o < rtb_Power_f) {
      rtb_BusAssignment1_o = rtb_BusAssignment_b;
    }
    break;

   default:
    rtb_Divide_o = (mpc_tgt_calc_P.Constant_Value_eh - arg_ego->w) /
      (mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step));
    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_eh;
    rtb_BusAssignment1_o.alpha = rtb_Divide_o;
    rtb_BusAssignment1_o.alpha2 = rtb_Divide_o;
    break;
  }

  rtb_Add2_i = rtb_BusAssignment1_o.w * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.dt;
  rtb_BusAssignment1.w = rtb_BusAssignment1_o.w;
  rtb_Switch1_n_idx_1 = arg_ego->ang + rtb_Add2_i;
  rtb_BusAssignment1.ang = rtb_Switch1_n_idx_1;
  rtb_BusAssignment1.sla_param.counter = rtb_BusAssignment1_o.sla_param.counter;
  rtb_BusAssignment1.pivot_state = rtb_BusAssignment1_o.pivot_state;
  rtb_BusAssignment1_tmp = arg_ego->img_dist + rtb_Add2;
  rtb_BusAssignment1.img_dist = rtb_BusAssignment1_tmp;
  rtb_Add2 = arg_ego->img_ang + rtb_Add2_i;
  rtb_BusAssignment1.img_ang = rtb_Add2;
  rtb_BusAssignment1.alpha = rtb_BusAssignment1_o.alpha;
  rtb_BusAssignment1.alpha2 = rtb_BusAssignment1_o.alpha2;
  if (*arg_In1 == 1) {
    if (mpc_tgt_calc_P.Constant4_Value_k != 0 && arg_tgt->enable_slip_decel != 0)
    {
      rtb_Abs6 = mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step);
      rtb_Add1_eh = (mpc_tgt_calc_P.Constant_Value_nl / arg_ego1->mass +
                     rtb_BusAssignment1_o.w * rtb_BusAssignment1.slip.vy) *
        rtb_Abs6 + rtb_BusAssignment1.slip.vx;
      rtb_Add3_kh = (mpc_tgt_calc_P.Gain3_Gain * arg_tgt->slip_gain_K1 *
                     rtb_BusAssignment1.slip.beta / arg_ego1->mass -
                     rtb_BusAssignment1_o.w * rtb_BusAssignment1.slip.vx) *
        rtb_Abs6 + rtb_BusAssignment1.slip.vy;
      rtb_Sqrt = t_sqrtF(rtb_Add1_eh * rtb_Add1_eh + rtb_Add3_kh * rtb_Add3_kh);
      rtb_Abs7 = mpc_tgt_calc_P.Gain1_Gain_e * rtb_Sqrt;
      rtb_Divide2 = (rtb_Abs7 - arg_ego->v) / rtb_Abs6;
      rtb_Divide_o = rtb_BusAssignment1.v_r;
      rtb_Power_f = rtb_BusAssignment1.v_l;
      // rtb_Subtract2_k = rtb_BusAssignment1.pos_x;
      // rtb_Subtract2 = rtb_BusAssignment1.pos_y;
      rtb_Merge1_ideal_px = rtb_BusAssignment1.ideal_px;
      rtb_Merge1_ideal_py = rtb_BusAssignment1.ideal_py;
      rtb_Merge1_w = rtb_BusAssignment1_o.w;
      rtb_Merge1_alpha = rtb_BusAssignment1_o.alpha;
      rtb_Merge1_alpha2 = rtb_BusAssignment1_o.alpha2;
      rtb_Merge1_dist = rtb_BusAssignment1.dist;
      rtb_Add2_i = rtb_BusAssignment1.sla_param.base_alpha;
      rtb_Merge1_sla_param_base_time = rtb_BusAssignment1.sla_param.base_time;
      rtb_Merge1_sla_param_limit_time =
        rtb_BusAssignment1.sla_param.limit_time_count;
      rtb_Merge1_sla_param_pow_n = rtb_BusAssignment1.sla_param.pow_n;
      rtb_Merge2 = rtb_BusAssignment1.sla_param.state;
      rtb_Merge1_ideal_point_x = rtb_BusAssignment1.ideal_point.x;
      rtb_Merge1_ideal_point_y = rtb_BusAssignment1.ideal_point.y;
      rtb_Merge1_ideal_point_theta = rtb_BusAssignment1.ideal_point.theta;
      rtb_Merge1_ideal_point_v = rtb_BusAssignment1.ideal_point.v;
      rtb_Merge1_ideal_point_w = rtb_BusAssignment1.ideal_point.w;
      rtb_Merge1_ideal_point_slip_ang =
        rtb_BusAssignment1.ideal_point.slip_angle;
      rtb_Merge1_slip_point_x = rtb_BusAssignment1.slip_point.x;
      rtb_Merge1_slip_point_y = rtb_BusAssignment1.slip_point.y;
      rtb_Merge1_slip_point_theta = rtb_BusAssignment1.slip_point.theta;
      rtb_Merge1_slip_point_v = rtb_BusAssignment1.slip_point.v;
      rtb_Merge1_slip_point_w = rtb_BusAssignment1.slip_point.w;
      rtb_Merge1_slip_point_slip_angl = rtb_BusAssignment1.slip_point.slip_angle;
      rtb_Merge1_kanayama_point_x = rtb_BusAssignment1.kanayama_point.x;
      rtb_Merge1_kanayama_point_y = rtb_BusAssignment1.kanayama_point.y;
      rtb_Merge1_kanayama_point_theta = rtb_BusAssignment1.kanayama_point.theta;
      rtb_Merge1_kanayama_point_v = rtb_BusAssignment1.kanayama_point.v;
      rtb_Merge1_kanayama_point_w = rtb_BusAssignment1.kanayama_point.w;
      rtb_Merge1_trj_diff_x = rtb_BusAssignment1.trj_diff.x;
      rtb_Merge1_trj_diff_y = rtb_BusAssignment1.trj_diff.y;
      rtb_Merge1_trj_diff_theta = rtb_BusAssignment1.trj_diff.theta;
      rtb_Merge1_delay_accl = rtb_BusAssignment1.delay_accl;
      rtb_Merge1_delay_v = rtb_BusAssignment1.delay_v;
      rtb_Merge1_cnt_delay_accl_ratio = rtb_BusAssignment1.cnt_delay_accl_ratio;
      rtb_Merge1_cnt_delay_decel_rati = rtb_BusAssignment1.cnt_delay_decel_ratio;
      rtb_Merge1_ff_duty_low_th = rtb_BusAssignment1.ff_duty_low_th;
      rtb_Merge1_ff_duty_low_v_th = rtb_BusAssignment1.ff_duty_low_v_th;
      rtb_Merge1_decel_delay_cnt = rtb_BusAssignment1.decel_delay_cnt;
      rtb_Merge1_slip_beta = (rtb_BusAssignment1.slip.beta / rtb_Abs6 -
        rtb_BusAssignment1_o.w) / (mpc_tgt_calc_P.Constant1_Value_g / rtb_Abs6 +
        arg_tgt->slip_gain_K2 / rtb_Sqrt);
      rtb_Abs6 = rtb_Divide2;
    } else {
      rtb_Divide_o = rtb_BusAssignment1.v_r;
      rtb_Power_f = rtb_BusAssignment1.v_l;
      // rtb_Subtract2_k = rtb_BusAssignment1.pos_x;
      // rtb_Subtract2 = rtb_BusAssignment1.pos_y;
      rtb_Merge1_ideal_px = rtb_BusAssignment1.ideal_px;
      rtb_Merge1_ideal_py = rtb_BusAssignment1.ideal_py;
      rtb_Merge1_w = rtb_BusAssignment1_o.w;
      rtb_Merge1_alpha = rtb_BusAssignment1_o.alpha;
      rtb_Merge1_alpha2 = rtb_BusAssignment1_o.alpha2;
      rtb_Merge1_dist = rtb_BusAssignment1.dist;
      rtb_Add2_i = rtb_BusAssignment1.sla_param.base_alpha;
      rtb_Merge1_sla_param_base_time = rtb_BusAssignment1.sla_param.base_time;
      rtb_Merge1_sla_param_limit_time =
        rtb_BusAssignment1.sla_param.limit_time_count;
      rtb_Merge1_sla_param_pow_n = rtb_BusAssignment1.sla_param.pow_n;
      rtb_Merge2 = rtb_BusAssignment1.sla_param.state;
      rtb_Merge1_ideal_point_x = rtb_BusAssignment1.ideal_point.x;
      rtb_Merge1_ideal_point_y = rtb_BusAssignment1.ideal_point.y;
      rtb_Merge1_ideal_point_theta = rtb_BusAssignment1.ideal_point.theta;
      rtb_Merge1_ideal_point_v = rtb_BusAssignment1.ideal_point.v;
      rtb_Merge1_ideal_point_w = rtb_BusAssignment1.ideal_point.w;
      rtb_Merge1_ideal_point_slip_ang =
        rtb_BusAssignment1.ideal_point.slip_angle;
      rtb_Merge1_slip_point_x = rtb_BusAssignment1.slip_point.x;
      rtb_Merge1_slip_point_y = rtb_BusAssignment1.slip_point.y;
      rtb_Merge1_slip_point_theta = rtb_BusAssignment1.slip_point.theta;
      rtb_Merge1_slip_point_v = rtb_BusAssignment1.slip_point.v;
      rtb_Merge1_slip_point_w = rtb_BusAssignment1.slip_point.w;
      rtb_Merge1_slip_point_slip_angl = rtb_BusAssignment1.slip_point.slip_angle;
      rtb_Merge1_kanayama_point_x = rtb_BusAssignment1.kanayama_point.x;
      rtb_Merge1_kanayama_point_y = rtb_BusAssignment1.kanayama_point.y;
      rtb_Merge1_kanayama_point_theta = rtb_BusAssignment1.kanayama_point.theta;
      rtb_Merge1_kanayama_point_v = rtb_BusAssignment1.kanayama_point.v;
      rtb_Merge1_kanayama_point_w = rtb_BusAssignment1.kanayama_point.w;
      rtb_Merge1_trj_diff_x = rtb_BusAssignment1.trj_diff.x;
      rtb_Merge1_trj_diff_y = rtb_BusAssignment1.trj_diff.y;
      rtb_Merge1_trj_diff_theta = rtb_BusAssignment1.trj_diff.theta;
      rtb_Merge1_delay_accl = rtb_BusAssignment1.delay_accl;
      rtb_Merge1_delay_v = rtb_BusAssignment1.delay_v;
      rtb_Merge1_cnt_delay_accl_ratio = rtb_BusAssignment1.cnt_delay_accl_ratio;
      rtb_Merge1_cnt_delay_decel_rati = rtb_BusAssignment1.cnt_delay_decel_ratio;
      rtb_Merge1_ff_duty_low_th = rtb_BusAssignment1.ff_duty_low_th;
      rtb_Merge1_ff_duty_low_v_th = rtb_BusAssignment1.ff_duty_low_v_th;
      rtb_Merge1_decel_delay_cnt = rtb_BusAssignment1.decel_delay_cnt;
      rtb_Merge1_slip_beta = mpc_tgt_calc_P.Constant_Value_pi;
      rtb_Add1_eh = mpc_tgt_calc_P.Gain1_Gain_if * rtb_Abs7;
      rtb_Add3_kh = mpc_tgt_calc_P.Constant_Value_pi;
      rtb_Sqrt = mpc_tgt_calc_P.Gain_Gain_m * rtb_Abs7;
      rtb_Divide2 = rtb_BusAssignment1.slip.accl;
    }

    if (std::isnan(rtb_Abs6) || std::isinf(rtb_Abs6)) {
      rtb_Abs6 = mpc_tgt_calc_P.Constant_Value_o;
      rtb_Abs7 = arg_ego->v;
      rtb_BusAssignment1_o.img_dist = arg_ego->img_dist;
    } else {
      rtb_BusAssignment1_o.img_dist = rtb_BusAssignment1_tmp;
    }

    rtb_RelationalOperator_a = std::isnan(rtb_Merge1_alpha) || std::isinf
      (rtb_Merge1_alpha);
    rtb_BusAssignment1_tmp = rtb_Abs7;
    if (rtb_RelationalOperator_a) {
      rtb_Merge1_w = arg_ego->w;
    }

    if (std::isnan(arg_ego->alpha2) || std::isinf(arg_ego->alpha2)) {
      rtb_Merge1_alpha2 = mpc_tgt_calc_P.Constant4_Value_j;
    }

    rtb_FF_Right = arg_ego1->gear_ratio * arg_ego1->km;
    rtb_Divide_jy_tmp = mpc_tgt_calc_P.Gain4_Gain * rtb_Abs6 * arg_ego1->mass *
      (mpc_tgt_calc_P.Gain1_Gain_f * arg_ego1->tire);
    rtb_Divide_n = rtb_Divide_jy_tmp * arg_ego1->resist / rtb_FF_Right;
    if ((arg_mode == 0 || arg_mode == 3) && rtb_pivot_state == 0 && !(rtb_Abs7 >
         rtb_Merge1_ff_duty_low_v_th)) {
      rtb_Divide_n = std::fmax(rtb_Divide_n, rtb_Merge1_ff_duty_low_th);
    }

    rtb_Abs7_tmp = rtb_Merge1_alpha2 * arg_ego1->lm;
    rtb_Abs7 = mpc_tgt_calc_P.Gain1_Gain_d * arg_ego1->tire * rtb_Abs7_tmp *
      arg_ego1->resist / rtb_FF_Right / (mpc_tgt_calc_P.Gain2_Gain *
      arg_ego1->tread);
    rtb_Gain3_m = mpc_tgt_calc_P.Gain4_Gain_d * rtb_BusAssignment1_tmp;
    rtb_FF_Right = mpc_tgt_calc_P.Gain5_Gain * arg_ego1->tread *
      mpc_tgt_calc_P.Gain_Gain_l * rtb_Merge1_w;
    rtb_Divide1_a = mpc_tgt_calc_P.Gain6_Gain * arg_ego1->tire *
      mpc_tgt_calc_P.Gain1_Gain_nu;
    rtb_Divide_l = (rtb_Gain3_m - rtb_FF_Right) * arg_ego1->ke *
      mpc_tgt_calc_P.Gain2_Gain_o / rtb_Divide1_a;
    rtb_Divide1_a = (rtb_Gain3_m + rtb_FF_Right) * arg_ego1->ke *
      mpc_tgt_calc_P.Gain3_Gain_e / rtb_Divide1_a;
    rtb_Product2_l = mpc_tgt_calc_P.Gain4_Gain_a * rtb_BusAssignment1_tmp;
    rtb_Gain3_m = mpc_tgt_calc_P.Gain5_Gain_k * arg_ego1->tread *
      mpc_tgt_calc_P.Gain_Gain_jr * rtb_Merge1_w;
    rtb_FF_Right = rtb_Product2_l - rtb_Gain3_m;
    rtb_Product2_l += rtb_Gain3_m;
    rtb_BusAssignment1_o.v = rtb_BusAssignment1_tmp;
    rtb_BusAssignment1_o.v_r = rtb_Divide_o;
    rtb_BusAssignment1_o.v_l = rtb_Power_f;
    // rtb_BusAssignment1_o.pos_x = rtb_Subtract2_k;
    // rtb_BusAssignment1_o.pos_y = rtb_Subtract2;
    rtb_BusAssignment1_o.ideal_px = rtb_Merge1_ideal_px;
    rtb_BusAssignment1_o.ideal_py = rtb_Merge1_ideal_py;
    rtb_BusAssignment1_o.accl = rtb_Abs6;
    rtb_BusAssignment1_o.w = rtb_Merge1_w;
    if (rtb_RelationalOperator_a) {
      rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant1_Value_e0;
    } else {
      rtb_BusAssignment1_o.alpha = rtb_Merge1_alpha;
    }

    rtb_BusAssignment1_o.alpha2 = rtb_Merge1_alpha2;
    rtb_BusAssignment1_o.dist = rtb_Merge1_dist;
    rtb_BusAssignment1_o.ang = rtb_Switch1_n_idx_1;
    if (rtb_RelationalOperator_a) {
      rtb_BusAssignment1_o.img_ang = arg_ego->img_ang;
    } else {
      rtb_BusAssignment1_o.img_ang = rtb_Add2;
    }

    rtb_BusAssignment1_o.sla_param.base_alpha = rtb_Add2_i;
    rtb_BusAssignment1_o.sla_param.base_time = rtb_Merge1_sla_param_base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      rtb_Merge1_sla_param_limit_time;
    rtb_BusAssignment1_o.sla_param.pow_n = rtb_Merge1_sla_param_pow_n;
    rtb_BusAssignment1_o.sla_param.state = rtb_Merge2;
    rtb_BusAssignment1_o.state = rtb_pivot_state;
    rtb_BusAssignment1_o.ideal_point.x = rtb_Merge1_ideal_point_x;
    rtb_BusAssignment1_o.ideal_point.y = rtb_Merge1_ideal_point_y;
    rtb_BusAssignment1_o.ideal_point.theta = rtb_Merge1_ideal_point_theta;
    rtb_BusAssignment1_o.ideal_point.v = rtb_Merge1_ideal_point_v;
    rtb_BusAssignment1_o.ideal_point.w = rtb_Merge1_ideal_point_w;
    rtb_BusAssignment1_o.ideal_point.slip_angle =
      rtb_Merge1_ideal_point_slip_ang;
    rtb_BusAssignment1_o.slip_point.x = rtb_Merge1_slip_point_x;
    rtb_BusAssignment1_o.slip_point.y = rtb_Merge1_slip_point_y;
    rtb_BusAssignment1_o.slip_point.theta = rtb_Merge1_slip_point_theta;
    rtb_BusAssignment1_o.slip_point.v = rtb_Merge1_slip_point_v;
    rtb_BusAssignment1_o.slip_point.w = rtb_Merge1_slip_point_w;
    rtb_BusAssignment1_o.slip_point.slip_angle = rtb_Merge1_slip_point_slip_angl;
    rtb_BusAssignment1_o.kanayama_point.x = rtb_Merge1_kanayama_point_x;
    rtb_BusAssignment1_o.kanayama_point.y = rtb_Merge1_kanayama_point_y;
    rtb_BusAssignment1_o.kanayama_point.theta = rtb_Merge1_kanayama_point_theta;
    rtb_BusAssignment1_o.kanayama_point.v = rtb_Merge1_kanayama_point_v;
    rtb_BusAssignment1_o.kanayama_point.w = rtb_Merge1_kanayama_point_w;
    rtb_BusAssignment1_o.trj_diff.x = rtb_Merge1_trj_diff_x;
    rtb_BusAssignment1_o.trj_diff.y = rtb_Merge1_trj_diff_y;
    rtb_BusAssignment1_o.trj_diff.theta = rtb_Merge1_trj_diff_theta;
    rtb_BusAssignment1_o.delay_accl = rtb_Merge1_delay_accl;
    rtb_BusAssignment1_o.delay_v = rtb_Merge1_delay_v;
    rtb_BusAssignment1_o.cnt_delay_accl_ratio = rtb_Merge1_cnt_delay_accl_ratio;
    rtb_BusAssignment1_o.cnt_delay_decel_ratio = rtb_Merge1_cnt_delay_decel_rati;
    rtb_BusAssignment1_o.slip.beta = rtb_Merge1_slip_beta;
    rtb_BusAssignment1_o.slip.vx = rtb_Add1_eh;
    rtb_BusAssignment1_o.slip.vy = rtb_Add3_kh;
    rtb_BusAssignment1_o.slip.v = rtb_Sqrt;
    rtb_BusAssignment1_o.slip.accl = rtb_Divide2;
    rtb_BusAssignment1_o.ff_duty_low_th = rtb_Merge1_ff_duty_low_th;
    rtb_BusAssignment1_o.ff_duty_low_v_th = rtb_Merge1_ff_duty_low_v_th;
    rtb_BusAssignment1_o.decel_delay_cnt = rtb_Merge1_decel_delay_cnt;
    rtb_BusAssignment1_o.ff_duty_front = rtb_Divide_n;
    rtb_BusAssignment1_o.ff_duty_l = rtb_Divide_n - rtb_Abs7 + rtb_Divide_l;
    rtb_BusAssignment1_o.ff_duty_r = rtb_Divide_n + rtb_Abs7 + rtb_Divide1_a;
    rtb_BusAssignment1_o.ff_duty_roll = rtb_Abs7;
    rtb_BusAssignment1_o.ff_duty_rpm_r = rtb_Divide1_a;
    rtb_BusAssignment1_o.ff_duty_rpm_l = rtb_Divide_l;
    rtb_BusAssignment1_o.ff_front_torque = mpc_tgt_calc_P.Gain_Gain_l3 *
      rtb_Divide_jy_tmp;
    rtb_BusAssignment1_o.ff_roll_torque = rtb_Abs7_tmp * arg_ego1->resist *
      mpc_tgt_calc_P.Gain3_Gain_g;
    if (std::isnan(rtb_FF_Right)) {
      rtb_Add2_i = (rtNaNF);
    } else if (rtb_FF_Right < 0.0F) {
      rtb_Add2_i = -1.0F;
    } else {
      rtb_Add2_i = rtb_FF_Right > 0.0F;
    }

    rtb_BusAssignment1_o.ff_friction_torque_l = rtb_Add2_i *
      arg_ego1->coulomb_friction + rtb_FF_Right * arg_ego1->viscous_friction;
    if (std::isnan(rtb_Product2_l)) {
      rtb_Add2_i = (rtNaNF);
    } else if (rtb_Product2_l < 0.0F) {
      rtb_Add2_i = -1.0F;
    } else {
      rtb_Add2_i = rtb_Product2_l > 0.0F;
    }

    rtb_BusAssignment1_o.ff_friction_torque_r = rtb_Add2_i *
      arg_ego1->coulomb_friction + rtb_Product2_l * arg_ego1->viscous_friction;
  } else {
    rtb_BusAssignment1_o = rtb_BusAssignment1;
  }

  rtb_Abs7 = mpc_tgt_calc_P.dt * rtb_BusAssignment1_o.v;
  *arg_next_ego = rtb_BusAssignment1_o;
  arg_next_ego->ideal_px = rtb_Abs7 * std::cos(rtb_BusAssignment1_o.img_ang) +
    rtb_BusAssignment1_o.ideal_px;
  arg_next_ego->ideal_py = rtb_Abs7 * std::sin(rtb_BusAssignment1_o.img_ang) +
    rtb_BusAssignment1_o.ideal_py;
  // arg_next_ego->pos_x = rtb_BusAssignment1_o.pos_x;
  // arg_next_ego->pos_y = rtb_BusAssignment1_o.pos_y;
}

void mpc_tgt_calcModelClass::initialize()
{
  mpc_tgt_calc_DW.UnitDelay_DSTATE = mpc_tgt_calc_P.UnitDelay_InitialCondition;
  mpc_tgt_calc_B.Merge[0] = mpc_tgt_calc_P.Merge_InitialOutput_h;
  mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Merge1_InitialOutput;
  mpc_tgt_calc_B.Merge[1] = mpc_tgt_calc_P.Merge_InitialOutput_h;
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
  mpc_tgt_calc_DW(),
  mpc_tgt_calc_M()
{
}

mpc_tgt_calcModelClass::~mpc_tgt_calcModelClass() = default;
mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * mpc_tgt_calcModelClass::getRTM
  ()
{
  return (&mpc_tgt_calc_M);
}
