#include "mpc_tgt_calc.h"
#include "rtwtypes.h"
#include "bus.h"
#include <cmath>
#include "mpc_tgt_calc_private.h"
#include "cmath"
#include <cstdint>
#include <cstring>
#include <iostream>
#include "esp_system.h"

void mpc_tgt_calcModelClass::mpc_tgt_calc_IfActionSubsystem(real32_T rty_Out1[2],
  P_IfActionSubsystem_mpc_tgt_c_T *localP)
{
  rty_Out1[0] = localP->Constant1_Value;
  rty_Out1[1] = localP->Constant2_Value;
}


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

void IRAM_ATTR mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego arg_next_ego[50], t_dynamics
  *arg_ego1, int32_T *arg_In1)
{
  t_ego expl_temp;
  real_T rtb_Switch_h;
  int32_T i;
  int32_T rtb_Merge1_l;
  int32_T rtb_Switch_decel_delay_cnt;
  int32_T rtb_Switch_sla_param_counter;
  int32_T rtb_Switch_sla_param_state;
  int32_T rtb_pivot_state;
  int32_T s1_iter;
  int32_T tmp;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Add3_c;
  real32_T rtb_BusAssignment1_n_v;
  real32_T rtb_Divide1_j;
  real32_T rtb_Divide1_m;
  real32_T rtb_Divide2;
  real32_T rtb_Divide3;
  real32_T rtb_Divide_j;
  real32_T rtb_FF_Left;
  real32_T rtb_FF_Right;
  real32_T rtb_Gain2_j;
  real32_T rtb_Gain3_k;
  real32_T rtb_Gain4;
  real32_T rtb_Merge1_img_dist;
  real32_T rtb_Merge_o;
  real32_T rtb_Power1_b;
  real32_T rtb_Power_c;
  real32_T rtb_Sqrt;
  real32_T rtb_Switch_alpha2;
  real32_T rtb_Switch_ang;
  real32_T rtb_Switch_cnt_delay_accl_ratio;
  real32_T rtb_Switch_cnt_delay_decel_rati;
  real32_T rtb_Switch_delay_accl;
  real32_T rtb_Switch_delay_v;
  real32_T rtb_Switch_dist;
  real32_T rtb_Switch_ff_duty_low_th;
  real32_T rtb_Switch_ff_duty_low_v_th;
  real32_T rtb_Switch_ideal_point_slip_ang;
  real32_T rtb_Switch_ideal_point_theta;
  real32_T rtb_Switch_ideal_point_v;
  real32_T rtb_Switch_ideal_point_w;
  real32_T rtb_Switch_ideal_point_x;
  real32_T rtb_Switch_ideal_point_y;
  real32_T rtb_Switch_img_ang;
  real32_T rtb_Switch_img_dist;
  real32_T rtb_Switch_kanayama_point_theta;
  real32_T rtb_Switch_kanayama_point_v;
  real32_T rtb_Switch_kanayama_point_w;
  real32_T rtb_Switch_kanayama_point_x;
  real32_T rtb_Switch_kanayama_point_y;
  real32_T rtb_Switch_pos_x;
  real32_T rtb_Switch_pos_y;
  real32_T rtb_Switch_sla_param_base_alpha;
  real32_T rtb_Switch_sla_param_base_time;
  real32_T rtb_Switch_sla_param_limit_time;
  real32_T rtb_Switch_sla_param_pow_n;
  real32_T rtb_Switch_slip_beta;
  real32_T rtb_Switch_slip_point_slip_angl;
  real32_T rtb_Switch_slip_point_theta;
  real32_T rtb_Switch_slip_point_v;
  real32_T rtb_Switch_slip_point_w;
  real32_T rtb_Switch_slip_point_x;
  real32_T rtb_Switch_slip_point_y;
  real32_T rtb_Switch_trj_diff_theta;
  real32_T rtb_Switch_trj_diff_x;
  real32_T rtb_Switch_trj_diff_y;
  real32_T rtb_Switch_v_l;
  real32_T rtb_Switch_v_r;
  real32_T rtb_Switch_w;
  real32_T tmp_0;
  boolean_T rtb_RelationalOperator_g;
  tmp = *arg_In1;
  if (*arg_In1 > 2147483646) {
    tmp = 2147483646;
  } else if (*arg_In1 < 0) {
    tmp = 0;
  }

  for (s1_iter = 1; s1_iter <= tmp; s1_iter++) {
    if (s1_iter > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Merge_o = mpc_tgt_calc_DW.UnitDelay_DSTATE.v;
      rtb_Switch_v_r = mpc_tgt_calc_DW.UnitDelay_DSTATE.v_r;
      rtb_Switch_v_l = mpc_tgt_calc_DW.UnitDelay_DSTATE.v_l;
      rtb_Switch_pos_x = mpc_tgt_calc_DW.UnitDelay_DSTATE.pos_x;
      rtb_Switch_pos_y = mpc_tgt_calc_DW.UnitDelay_DSTATE.pos_y;
      rtb_Switch_w = mpc_tgt_calc_DW.UnitDelay_DSTATE.w;
      rtb_Switch_alpha2 = mpc_tgt_calc_DW.UnitDelay_DSTATE.alpha2;
      rtb_Switch_dist = mpc_tgt_calc_DW.UnitDelay_DSTATE.dist;
      rtb_Switch_ang = mpc_tgt_calc_DW.UnitDelay_DSTATE.ang;
      rtb_Switch_img_dist = mpc_tgt_calc_DW.UnitDelay_DSTATE.img_dist;
      rtb_Switch_img_ang = mpc_tgt_calc_DW.UnitDelay_DSTATE.img_ang;
      rtb_Switch_sla_param_base_alpha =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.base_alpha;
      rtb_Switch_sla_param_base_time =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.base_time;
      rtb_Switch_sla_param_limit_time =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.limit_time_count;
      rtb_Switch_sla_param_pow_n =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.pow_n;
      rtb_Switch_sla_param_state =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.state;
      rtb_Switch_sla_param_counter =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.counter;
      rtb_pivot_state = mpc_tgt_calc_DW.UnitDelay_DSTATE.state;
      rtb_Merge1_l = mpc_tgt_calc_DW.UnitDelay_DSTATE.pivot_state;
      rtb_Switch_ideal_point_x = mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.x;
      rtb_Switch_ideal_point_y = mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.y;
      rtb_Switch_ideal_point_theta =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.theta;
      rtb_Switch_ideal_point_v = mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.v;
      rtb_Switch_ideal_point_w = mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.w;
      rtb_Switch_ideal_point_slip_ang =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.slip_angle;
      rtb_Switch_slip_point_x = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.x;
      rtb_Switch_slip_point_y = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.y;
      rtb_Switch_slip_point_theta =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.theta;
      rtb_Switch_slip_point_v = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.v;
      rtb_Switch_slip_point_w = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.w;
      rtb_Switch_slip_point_slip_angl =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.slip_angle;
      rtb_Switch_kanayama_point_x =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.x;
      rtb_Switch_kanayama_point_y =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.y;
      rtb_Switch_kanayama_point_theta =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.theta;
      rtb_Switch_kanayama_point_v =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.v;
      rtb_Switch_kanayama_point_w =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.w;
      rtb_Switch_trj_diff_x = mpc_tgt_calc_DW.UnitDelay_DSTATE.trj_diff.x;
      rtb_Switch_trj_diff_y = mpc_tgt_calc_DW.UnitDelay_DSTATE.trj_diff.y;
      rtb_Switch_trj_diff_theta =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.trj_diff.theta;
      rtb_Switch_delay_accl = mpc_tgt_calc_DW.UnitDelay_DSTATE.delay_accl;
      rtb_Switch_delay_v = mpc_tgt_calc_DW.UnitDelay_DSTATE.delay_v;
      rtb_Switch_cnt_delay_accl_ratio =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.cnt_delay_accl_ratio;
      rtb_Switch_cnt_delay_decel_rati =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.cnt_delay_decel_ratio;
      rtb_Switch_slip_beta = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.beta;
      rtb_Add3_c = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.vx;
      rtb_Sqrt = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.vy;
      rtb_Divide2 = mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.accl;
      rtb_Switch_ff_duty_low_th =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_low_th;
      rtb_Switch_ff_duty_low_v_th =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_low_v_th;
      rtb_Switch_decel_delay_cnt =
        mpc_tgt_calc_DW.UnitDelay_DSTATE.decel_delay_cnt;
    } else {
      rtb_Merge_o = arg_ego->v;
      rtb_Switch_v_r = arg_ego->v_r;
      rtb_Switch_v_l = arg_ego->v_l;
      rtb_Switch_pos_x = arg_ego->pos_x;
      rtb_Switch_pos_y = arg_ego->pos_y;
      rtb_Switch_w = arg_ego->w;
      rtb_Switch_alpha2 = arg_ego->alpha2;
      rtb_Switch_dist = arg_ego->dist;
      rtb_Switch_ang = arg_ego->ang;
      rtb_Switch_img_dist = arg_ego->img_dist;
      rtb_Switch_img_ang = arg_ego->img_ang;
      rtb_Switch_sla_param_base_alpha = arg_ego->sla_param.base_alpha;
      rtb_Switch_sla_param_base_time = arg_ego->sla_param.base_time;
      rtb_Switch_sla_param_limit_time = arg_ego->sla_param.limit_time_count;
      rtb_Switch_sla_param_pow_n = arg_ego->sla_param.pow_n;
      rtb_Switch_sla_param_state = arg_ego->sla_param.state;
      rtb_Switch_sla_param_counter = arg_ego->sla_param.counter;
      rtb_pivot_state = arg_ego->state;
      rtb_Merge1_l = arg_ego->pivot_state;
      rtb_Switch_ideal_point_x = arg_ego->ideal_point.x;
      rtb_Switch_ideal_point_y = arg_ego->ideal_point.y;
      rtb_Switch_ideal_point_theta = arg_ego->ideal_point.theta;
      rtb_Switch_ideal_point_v = arg_ego->ideal_point.v;
      rtb_Switch_ideal_point_w = arg_ego->ideal_point.w;
      rtb_Switch_ideal_point_slip_ang = arg_ego->ideal_point.slip_angle;
      rtb_Switch_slip_point_x = arg_ego->slip_point.x;
      rtb_Switch_slip_point_y = arg_ego->slip_point.y;
      rtb_Switch_slip_point_theta = arg_ego->slip_point.theta;
      rtb_Switch_slip_point_v = arg_ego->slip_point.v;
      rtb_Switch_slip_point_w = arg_ego->slip_point.w;
      rtb_Switch_slip_point_slip_angl = arg_ego->slip_point.slip_angle;
      rtb_Switch_kanayama_point_x = arg_ego->kanayama_point.x;
      rtb_Switch_kanayama_point_y = arg_ego->kanayama_point.y;
      rtb_Switch_kanayama_point_theta = arg_ego->kanayama_point.theta;
      rtb_Switch_kanayama_point_v = arg_ego->kanayama_point.v;
      rtb_Switch_kanayama_point_w = arg_ego->kanayama_point.w;
      rtb_Switch_trj_diff_x = arg_ego->trj_diff.x;
      rtb_Switch_trj_diff_y = arg_ego->trj_diff.y;
      rtb_Switch_trj_diff_theta = arg_ego->trj_diff.theta;
      rtb_Switch_delay_accl = arg_ego->delay_accl;
      rtb_Switch_delay_v = arg_ego->delay_v;
      rtb_Switch_cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
      rtb_Switch_cnt_delay_decel_rati = arg_ego->cnt_delay_decel_ratio;
      rtb_Switch_slip_beta = arg_ego->slip.beta;
      rtb_Add3_c = arg_ego->slip.vx;
      rtb_Sqrt = arg_ego->slip.vy;
      rtb_Divide2 = arg_ego->slip.accl;
      rtb_Switch_ff_duty_low_th = arg_ego->ff_duty_low_th;
      rtb_Switch_ff_duty_low_v_th = arg_ego->ff_duty_low_v_th;
      rtb_Switch_decel_delay_cnt = arg_ego->decel_delay_cnt;
    }

    if (arg_tgt->v_max >= 0.0F) {
      rtb_Abs7 = arg_tgt->tgt_dist - rtb_Switch_dist;
      tmp_0 = rtb_Merge_o * rtb_Merge_o;
      if (rtb_Merge_o - arg_tgt->end_v > mpc_tgt_calc_P.Constant3_Value_m &&
          (rtb_pivot_state == mpc_tgt_calc_P.Constant1_Value_o || std::abs(tmp_0
            - arg_tgt->end_v * arg_tgt->end_v) / (mpc_tgt_calc_P.Gain1_Gain_k *
            std::abs(arg_tgt->decel)) + rtb_Switch_dist >= arg_tgt->tgt_dist)) {
        if (rtb_Merge_o > arg_tgt->end_v) {
          if (rtb_Abs7 > mpc_tgt_calc_P.Constant3_Value) {
            rtb_Switch_h = mpc_tgt_calc_P.Gain_Gain_i * rtb_Abs7;
          } else {
            rtb_Switch_h = mpc_tgt_calc_P.Constant1_Value_g;
          }

          rtb_Switch_h = std::abs((tmp_0 - arg_tgt->end_v * arg_tgt->end_v) /
            rtb_Switch_h) * mpc_tgt_calc_P.Gain1_Gain;
        } else {
          rtb_Switch_h = mpc_tgt_calc_P.Constant_Value_l;
        }

        rtb_pivot_state = mpc_tgt_calc_DW.UnitDelay_DSTATE_k +
          mpc_tgt_calc_P.Constant5_Value;
        if (rtb_Merge_o < arg_tgt->accl_param.limit) {
          rtb_Abs6 = static_cast<real32_T>(rtb_Switch_h);
        } else {
          rtb_Abs6 = (static_cast<real32_T>(mpc_tgt_calc_P.Constant4_Value) -
                      rt_powf_snf(static_cast<real32_T>
            (mpc_tgt_calc_P.Constant4_Value) - std::fmin(static_cast<real32_T>
            (rtb_pivot_state) / static_cast<real32_T>
            (arg_tgt->accl_param.decel_delay_cnt), static_cast<real32_T>
            (mpc_tgt_calc_P.Constant6_Value)), arg_tgt->accl_param.decel_delay_n))
            * static_cast<real32_T>(rtb_Switch_h);
        }

        if (!(mpc_tgt_calc_P.dt * rtb_Abs6 * static_cast<real32_T>(arg_time_step)
              + rtb_Merge_o > arg_tgt->end_v)) {
          rtb_Abs6 = (arg_tgt->end_v - rtb_Merge_o) / (mpc_tgt_calc_P.dt *
            static_cast<real32_T>(arg_time_step));
        }

        if (rtb_pivot_state > arg_tgt->accl_param.decel_delay_cnt) {
          i = arg_tgt->accl_param.decel_delay_cnt;
        } else {
          i = rtb_pivot_state;
        }

        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_m;
      } else if (rtb_pivot_state == mpc_tgt_calc_P.Constant2_Value) {
        if (rtb_Merge_o < arg_tgt->v_max) {
          if (rtb_Merge_o > arg_tgt->accl_param.limit) {
            rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_b1 - rt_powf_snf
                        (rtb_Merge_o / arg_tgt->v_max, arg_tgt->accl_param.n)) *
              (arg_tgt->accl * arg_tgt->axel_degenerate_gain);
          } else {
            rtb_Abs7 = arg_tgt->accl * arg_tgt->axel_degenerate_gain;
          }

          rtb_Switch_h = rtb_Abs7;
          rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_e;
        } else {
          rtb_Switch_h = mpc_tgt_calc_P.Constant_Value_ii;
          rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_lk;
        }

        rtb_Abs6 = static_cast<real32_T>(rtb_Switch_h);
        i = mpc_tgt_calc_P.Constant4_Value_l;
      } else {
        rtb_Abs6 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_c);
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_l5;
        i = mpc_tgt_calc_P.Constant4_Value_p;
      }

      rtb_Abs7 = std::fmax(std::fmin(arg_tgt->v_max, mpc_tgt_calc_P.dt *
        rtb_Abs6 * static_cast<real32_T>(arg_time_step) + rtb_Merge_o),
                           mpc_tgt_calc_P.Constant_Value_a);
      mpc_tgt_calc_DW.UnitDelay_DSTATE_k = i;
    } else {
      rtb_Abs7 = std::abs(rtb_Switch_dist);
      rtb_Abs6 = std::abs(arg_tgt->tgt_dist);
      tmp_0 = rtb_Merge_o * rtb_Merge_o;
      if (rtb_pivot_state == 2.0F || std::abs(std::abs(tmp_0 - arg_tgt->end_v *
            arg_tgt->end_v) / (mpc_tgt_calc_P.Gain1_Gain_f * std::abs
            (arg_tgt->decel))) + rtb_Abs7 >= rtb_Abs6) {
        if (rtb_Merge_o < arg_tgt->end_v) {
          if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_m == 1) {
            rtb_Switch_h = arg_tgt->decel;
          } else {
            rtb_Switch_h = std::abs((tmp_0 - arg_tgt->end_v * arg_tgt->end_v) /
              (std::fmax(mpc_tgt_calc_P.Constant1_Value_p, static_cast<real_T>
                         (rtb_Abs6 - rtb_Abs7)) * mpc_tgt_calc_P.Gain_Gain_m)) *
              mpc_tgt_calc_P.Gain1_Gain_p;
          }
        } else {
          rtb_Switch_h = mpc_tgt_calc_P.Constant_Value_d;
        }

        rtb_Abs6 = static_cast<real32_T>(rtb_Switch_h);
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_ly;
      } else if (rtb_pivot_state == 0.0F) {
        if (rtb_Merge_o > arg_tgt->v_max) {
          if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_i == 1) {
            rtb_Abs7 = arg_tgt->accl;
          } else {
            rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_mn - rt_powf_snf
                        (rtb_Merge_o / arg_tgt->v_max,
                         mpc_tgt_calc_P.Constant4_Value_pv)) * arg_tgt->accl;
          }

          rtb_Switch_h = rtb_Abs7;
          rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_n;
        } else {
          rtb_Switch_h = mpc_tgt_calc_P.Constant_Value_bn;
          rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_f;
        }

        rtb_Abs6 = static_cast<real32_T>(rtb_Switch_h);
      } else {
        rtb_Abs6 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_g);
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_lu;
      }

      rtb_Abs7 = std::fmin(std::fmax(arg_tgt->v_max, mpc_tgt_calc_P.dt *
        rtb_Abs6 * static_cast<real32_T>(arg_time_step) + rtb_Merge_o),
                           mpc_tgt_calc_P.Constant_Value_cp);
    }

    rtb_Divide1_m = rtb_Abs7 * static_cast<real32_T>(arg_time_step) *
      mpc_tgt_calc_P.dt;
    if (arg_mode == 1) {
      i = arg_time_step + rtb_Switch_sla_param_counter;
      if (i > rtb_Switch_sla_param_limit_time) {
        mpc_tgt_calc_IfActionSubsystem(mpc_tgt_calc_B.Merge_i,
          &mpc_tgt_calc_P.IfActionSubsystem_d);
      } else {
        rtb_Power1_b = static_cast<real32_T>(i) * mpc_tgt_calc_P.dt /
          rtb_Switch_sla_param_base_time;
        rtb_Gain2_j = rtb_Power1_b - mpc_tgt_calc_P.Constant_Value_cx;
        rtb_Power_c = rt_powf_snf(rtb_Gain2_j, rtb_Switch_sla_param_pow_n -
          mpc_tgt_calc_P.Constant1_Value_n0);
        rtb_Gain2_j *= rtb_Power_c;
        rtb_Gain4 = mpc_tgt_calc_P.Constant6_Value_p / (rtb_Gain2_j -
          mpc_tgt_calc_P.Constant5_Value_n);
        rtb_Gain2_j -= mpc_tgt_calc_P.Constant2_Value_i;
        mpc_tgt_calc_B.Merge_i[0] = mpc_tgt_calc_P.Gain1_Gain_h *
          rtb_Switch_sla_param_pow_n * rtb_Power_c / (rtb_Gain2_j * rtb_Gain2_j)
          * exact_expf(mpc_tgt_calc_P.Constant4_Value_n + rtb_Gain4) /
          rtb_Switch_sla_param_base_time;
        mpc_tgt_calc_B.Merge_i[1] = exact_expf(mpc_tgt_calc_P.Constant8_Value /
          (mpc_tgt_calc_P.Constant7_Value - rt_powf_snf(rtb_Power1_b -
          mpc_tgt_calc_P.Constant10_Value, rtb_Switch_sla_param_pow_n))) *
          mpc_tgt_calc_P.Constant3_Value_f;
      }

      if (rtb_Switch_sla_param_state != 0) {
        rtb_Gain2_j = mpc_tgt_calc_P.Constant_Value_ci;
      } else {
        rtb_Gain2_j = rtb_Switch_sla_param_base_alpha * mpc_tgt_calc_B.Merge_i[0];
      }

      if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting == 1) {
        if (rtb_Switch_sla_param_counter >= rtb_Switch_sla_param_limit_time) {
          rtb_Switch_h = mpc_tgt_calc_P.Constant1_Value;
        } else {
          rtb_Switch_h = mpc_tgt_calc_P.dt * rtb_Gain2_j * static_cast<real32_T>
            (arg_time_step) + rtb_Switch_w;
        }
      } else if (rtb_Switch_sla_param_state != 0) {
        rtb_Switch_h = mpc_tgt_calc_P.Constant_Value_ci;
      } else {
        rtb_Switch_h = rtb_Switch_sla_param_base_alpha * mpc_tgt_calc_B.Merge_i
          [1];
      }

      tmp_0 = static_cast<real32_T>(arg_tgt->time_step2 +
        rtb_Switch_sla_param_counter);
      if (tmp_0 > rtb_Switch_sla_param_limit_time) {
        mpc_tgt_calc_IfActionSubsystem(mpc_tgt_calc_B.Merge1,
          &mpc_tgt_calc_P.IfActionSubsystem2);
      } else {
        rtb_Power1_b = tmp_0 * mpc_tgt_calc_P.dt /
          rtb_Switch_sla_param_base_time;
        rtb_Power_c = rtb_Power1_b - mpc_tgt_calc_P.Constant_Value_id;
        rtb_Gain4 = rt_powf_snf(rtb_Power_c, rtb_Switch_sla_param_pow_n -
          mpc_tgt_calc_P.Constant1_Value_a);
        rtb_Power_c *= rtb_Gain4;
        rtb_Divide3 = mpc_tgt_calc_P.Constant6_Value_e / (rtb_Power_c -
          mpc_tgt_calc_P.Constant5_Value_k);
        rtb_Power_c -= mpc_tgt_calc_P.Constant2_Value_f0;
        mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Gain1_Gain_d *
          rtb_Switch_sla_param_pow_n * rtb_Gain4 / (rtb_Power_c * rtb_Power_c) *
          exact_expf(mpc_tgt_calc_P.Constant4_Value_pq + rtb_Divide3) /
          rtb_Switch_sla_param_base_time;
        mpc_tgt_calc_B.Merge1[1] = exact_expf(mpc_tgt_calc_P.Constant8_Value_f /
          (mpc_tgt_calc_P.Constant7_Value_j - rt_powf_snf(rtb_Power1_b -
          mpc_tgt_calc_P.Constant10_Value_k, rtb_Switch_sla_param_pow_n))) *
          mpc_tgt_calc_P.Constant3_Value_b;
      }

      if (rtb_Switch_sla_param_state != 0) {
        rtb_Divide3 = mpc_tgt_calc_P.Constant_Value_ci;
      } else {
        rtb_Divide3 = rtb_Switch_sla_param_base_alpha * mpc_tgt_calc_B.Merge1[0];
      }

      rtb_Gain3_k = static_cast<real32_T>(rtb_Switch_h);
      rtb_Switch_sla_param_counter = i;
    } else if (arg_mode == 2) {
      rtb_Power1_b = std::abs(rtb_Switch_ang);
      rtb_Gain2_j = mpc_tgt_calc_P.Gain2_Gain * arg_tgt->alpha;
      rtb_Power_c = std::abs(arg_tgt->tgt_angle);
      if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold_f) {
        rtb_Gain4 = arg_tgt->w_max;
      } else {
        rtb_Gain4 = mpc_tgt_calc_P.Gain3_Gain * arg_tgt->w_max;
      }

      tmp_0 = rtb_Switch_w * rtb_Switch_w;
      if (rtb_Merge1_l == 2.0F || std::abs(tmp_0 - arg_tgt->end_w *
           arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_a * std::abs(rtb_Gain2_j))
          + rtb_Power1_b >= rtb_Power_c) {
        if (rtb_Gain2_j > mpc_tgt_calc_P.Switch2_Threshold) {
          rtb_RelationalOperator_g = rtb_Switch_w < arg_tgt->end_w;
        } else {
          rtb_RelationalOperator_g = rtb_Switch_w > arg_tgt->end_w;
        }

        if (rtb_RelationalOperator_g) {
          if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_n != 1) {
            if (rtb_Gain2_j > mpc_tgt_calc_P.Switch1_Threshold) {
              rtb_Gain2_j = static_cast<real32_T>(std::abs((tmp_0 -
                arg_tgt->end_w * arg_tgt->end_w) / (std::fmax
                (mpc_tgt_calc_P.Constant1_Value_d, static_cast<real_T>
                 (rtb_Power_c - rtb_Power1_b)) * mpc_tgt_calc_P.Gain_Gain)));
            } else {
              rtb_Gain2_j = static_cast<real32_T>(std::abs((tmp_0 -
                arg_tgt->end_w * arg_tgt->end_w) / (std::fmax
                (mpc_tgt_calc_P.Constant1_Value_d, static_cast<real_T>
                 (rtb_Power_c - rtb_Power1_b)) * mpc_tgt_calc_P.Gain_Gain))) *
                mpc_tgt_calc_P.Gain1_Gain_n;
            }
          }
        } else {
          rtb_Gain2_j = mpc_tgt_calc_P.Constant_Value_n;
        }

        rtb_Merge1_l = mpc_tgt_calc_P.Constant2_Value_b;
      } else if (rtb_Merge1_l == 0.0F) {
        if (std::abs(rtb_Switch_w) < std::abs(rtb_Gain4)) {
          if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_g == 1) {
            rtb_Gain2_j = arg_tgt->alpha;
          } else {
            rtb_Gain2_j = (mpc_tgt_calc_P.Constant3_Value_i - rt_powf_snf
                           (rtb_Switch_w / rtb_Gain4,
                            mpc_tgt_calc_P.Constant4_Value_o)) * arg_tgt->alpha;
          }

          rtb_Switch_h = rtb_Gain2_j;
          rtb_Merge1_l = mpc_tgt_calc_P.Constant1_Value_d3;
        } else {
          rtb_Switch_h = mpc_tgt_calc_P.Constant_Value_i;
          rtb_Merge1_l = mpc_tgt_calc_P.Constant2_Value_l;
        }

        rtb_Gain2_j = static_cast<real32_T>(rtb_Switch_h);
      } else {
        rtb_Gain2_j = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_b);
        rtb_Merge1_l = mpc_tgt_calc_P.Constant2_Value_p;
      }

      if (rtb_Power1_b < rtb_Power_c) {
        rtb_Gain3_k = mpc_tgt_calc_P.dt * rtb_Gain2_j * static_cast<real32_T>
          (arg_time_step) + rtb_Switch_w;
        rtb_Divide3 = rtb_Gain2_j;
      } else {
        rtb_Gain3_k = mpc_tgt_calc_P.Constant_Value_h;
        rtb_Gain2_j = mpc_tgt_calc_P.Constant_Value_h;
        rtb_Divide3 = mpc_tgt_calc_P.Constant_Value_h;
        rtb_Merge1_l = mpc_tgt_calc_P.Constant1_Value_j;
      }
    } else if (arg_mode == 4) {
      rtb_Power1_b = std::abs(rtb_Switch_img_ang);
      rtb_Gain2_j = mpc_tgt_calc_P.Gain2_Gain_d * arg_tgt->alpha;
      rtb_Power_c = std::abs(arg_tgt->tgt_angle);
      if (rtb_Switch_sla_param_pow_n != mpc_tgt_calc_P.Constant_Value_bv) {
        rtb_Gain4 = mpc_tgt_calc_P.Gain6_Gain * rtb_Power_c;
        rtb_Gain3_k = mpc_tgt_calc_P.Gain7_Gain * rtb_Power_c;
      } else {
        rtb_Gain4 = mpc_tgt_calc_P.Gain4_Gain * rtb_Power_c;
        rtb_Gain3_k = rtb_Gain4;
      }

      tmp_0 = rtb_Switch_w * rtb_Switch_w;
      if (rtb_Merge1_l == 2.0F || std::abs(tmp_0 - arg_tgt->end_w *
           arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_nj * std::abs
                              (rtb_Gain2_j)) + rtb_Power1_b >=
          mpc_tgt_calc_P.Gain_Gain_n * rtb_Power_c || rtb_Power1_b >= rtb_Gain4)
      {
        if (rtb_Gain2_j > mpc_tgt_calc_P.Switch2_Threshold_i) {
          rtb_RelationalOperator_g = rtb_Switch_w < arg_tgt->end_w;
        } else {
          rtb_RelationalOperator_g = rtb_Switch_w > arg_tgt->end_w;
        }

        if (rtb_RelationalOperator_g) {
          if (rtb_Gain2_j > mpc_tgt_calc_P.Switch1_Threshold_c) {
            rtb_Gain2_j = static_cast<real32_T>(std::abs((tmp_0 - arg_tgt->end_w
              * arg_tgt->end_w) / (std::fmax(static_cast<real_T>(rtb_Power_c -
              rtb_Power1_b), mpc_tgt_calc_P.Constant1_Value_m) *
              mpc_tgt_calc_P.Gain_Gain_f)));
          } else {
            rtb_Gain2_j = static_cast<real32_T>(std::abs((tmp_0 - arg_tgt->end_w
              * arg_tgt->end_w) / (std::fmax(static_cast<real_T>(rtb_Power_c -
              rtb_Power1_b), mpc_tgt_calc_P.Constant1_Value_m) *
              mpc_tgt_calc_P.Gain_Gain_f))) * mpc_tgt_calc_P.Gain1_Gain_c;
          }
        } else {
          rtb_Gain2_j = mpc_tgt_calc_P.Constant_Value_j;
        }

        rtb_Merge1_l = mpc_tgt_calc_P.Constant2_Value_p0;
      } else if (rtb_Merge1_l == 0.0F && rtb_Power1_b < rtb_Gain3_k) {
        rtb_Gain2_j = arg_tgt->alpha;
        rtb_Merge1_l = mpc_tgt_calc_P.Constant1_Value_c;
      } else {
        rtb_Gain2_j = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_k);
        rtb_Merge1_l = mpc_tgt_calc_P.Constant2_Value_pj;
      }

      if (rtb_Power1_b < rtb_Power_c) {
        rtb_Gain3_k = mpc_tgt_calc_P.dt * rtb_Gain2_j * static_cast<real32_T>
          (arg_time_step) + rtb_Switch_w;
        rtb_Divide3 = rtb_Switch_alpha2;
      } else {
        rtb_Gain3_k = mpc_tgt_calc_P.Constant_Value_h4;
        rtb_Gain2_j = mpc_tgt_calc_P.Constant_Value_h4;
        rtb_Divide3 = mpc_tgt_calc_P.Constant_Value_h4;
        rtb_Merge1_l = mpc_tgt_calc_P.Constant1_Value_b;
      }
    } else {
      rtb_Gain2_j = (mpc_tgt_calc_P.Constant_Value_dx - rtb_Switch_w) /
        (mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step));
      rtb_Gain3_k = mpc_tgt_calc_P.Constant_Value_dx;
      rtb_Divide3 = rtb_Gain2_j;
    }

    rtb_Power_c = rtb_Gain3_k * static_cast<real32_T>(arg_time_step) *
      mpc_tgt_calc_P.dt;
    if (mpc_tgt_calc_P.Constant4_Value_h != 0 && arg_tgt->enable_slip_decel != 0)
    {
      rtb_Abs6 = mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step);
      rtb_Power1_b = (mpc_tgt_calc_P.Constant_Value_da / arg_ego1->mass +
                      rtb_Gain3_k * rtb_Sqrt) * rtb_Abs6 + rtb_Add3_c;
      rtb_Add3_c = (mpc_tgt_calc_P.Gain3_Gain_e * arg_tgt->slip_gain_K1 *
                    rtb_Switch_slip_beta / arg_ego1->mass - rtb_Gain3_k *
                    rtb_Add3_c) * rtb_Abs6 + rtb_Sqrt;
      rtb_Sqrt = t_sqrtF(rtb_Power1_b * rtb_Power1_b + rtb_Add3_c * rtb_Add3_c);
      rtb_Abs7 = mpc_tgt_calc_P.Gain1_Gain_b * rtb_Sqrt;
      rtb_Divide2 = (rtb_Abs7 - rtb_Merge_o) / rtb_Abs6;
      rtb_Switch_dist += rtb_Divide1_m;
      rtb_Switch_ang += rtb_Power_c;
      rtb_Merge1_img_dist = rtb_Switch_img_dist + rtb_Divide1_m;
      rtb_Divide1_m = rtb_Switch_img_ang + rtb_Power_c;
      rtb_Switch_slip_beta = (rtb_Switch_slip_beta / rtb_Abs6 - rtb_Gain3_k) /
        (mpc_tgt_calc_P.Constant1_Value_mu / rtb_Abs6 + arg_tgt->slip_gain_K2 /
         rtb_Sqrt);
      rtb_Abs6 = rtb_Divide2;
    } else {
      rtb_Switch_dist += rtb_Divide1_m;
      rtb_Switch_ang += rtb_Power_c;
      rtb_Merge1_img_dist = rtb_Switch_img_dist + rtb_Divide1_m;
      rtb_Divide1_m = rtb_Switch_img_ang + rtb_Power_c;
      rtb_Switch_slip_beta = mpc_tgt_calc_P.Constant_Value_nc;
      rtb_Power1_b = mpc_tgt_calc_P.Gain1_Gain_o * rtb_Abs7;
      rtb_Add3_c = mpc_tgt_calc_P.Constant_Value_nc;
      rtb_Sqrt = mpc_tgt_calc_P.Gain_Gain_b * rtb_Abs7;
    }

    if (std::isnan(rtb_Abs6) || std::isinf(rtb_Abs6)) {
      rtb_Gain4 = mpc_tgt_calc_P.Constant_Value_p;
    } else {
      rtb_Gain4 = rtb_Abs6;
      rtb_Merge_o = rtb_Abs7;
      rtb_Switch_img_dist = rtb_Merge1_img_dist;
    }

    rtb_Merge1_img_dist = rtb_Gain4;
    rtb_BusAssignment1_n_v = rtb_Merge_o;
    if (std::isnan(rtb_Gain2_j) || std::isinf(rtb_Gain2_j)) {
      rtb_Gain2_j = mpc_tgt_calc_P.Constant1_Value_e0;
    } else {
      rtb_Switch_w = rtb_Gain3_k;
      rtb_Switch_img_ang = rtb_Divide1_m;
    }

    if (std::isnan(rtb_Switch_alpha2) || std::isinf(rtb_Switch_alpha2)) {
      rtb_Switch_alpha2 = mpc_tgt_calc_P.Constant4_Value_ps;
    } else {
      rtb_Switch_alpha2 = rtb_Divide3;
    }

    rtb_Divide1_m = arg_ego1->gear_ratio * arg_ego1->km;
    rtb_Divide3 = mpc_tgt_calc_P.Gain4_Gain_a * rtb_Gain4 * arg_ego1->mass *
      (mpc_tgt_calc_P.Gain1_Gain_i * arg_ego1->tire);
    rtb_Abs7 = rtb_Divide3 * arg_ego1->resist / rtb_Divide1_m;
    if ((arg_mode == 0 || arg_mode == 3) && rtb_pivot_state == 0) {
      if (rtb_Merge_o > rtb_Switch_ff_duty_low_v_th) {
        rtb_Merge_o = rtb_Abs7;
      } else {
        rtb_Merge_o = std::fmax(rtb_Abs7, rtb_Switch_ff_duty_low_th);
      }
    } else {
      rtb_Merge_o = rtb_Abs7;
    }

    rtb_Abs7 = rtb_Switch_alpha2 * arg_ego1->lm;
    rtb_Divide1_m = mpc_tgt_calc_P.Gain1_Gain_hw * arg_ego1->tire * rtb_Abs7 *
      arg_ego1->resist / rtb_Divide1_m / (mpc_tgt_calc_P.Gain2_Gain_n *
      arg_ego1->tread);
    rtb_Gain3_k = mpc_tgt_calc_P.Gain4_Gain_f * rtb_BusAssignment1_n_v;
    rtb_FF_Right = mpc_tgt_calc_P.Gain5_Gain * arg_ego1->tread *
      mpc_tgt_calc_P.Gain_Gain_g * rtb_Switch_w;
    rtb_Divide1_j = mpc_tgt_calc_P.Gain6_Gain_n * arg_ego1->tire *
      mpc_tgt_calc_P.Gain1_Gain_j;
    rtb_Divide_j = (rtb_Gain3_k - rtb_FF_Right) * arg_ego1->ke *
      mpc_tgt_calc_P.Gain2_Gain_n3 / rtb_Divide1_j;
    rtb_FF_Left = rtb_Merge_o - rtb_Divide1_m + rtb_Divide_j;
    rtb_Divide1_j = (rtb_Gain3_k + rtb_FF_Right) * arg_ego1->ke *
      mpc_tgt_calc_P.Gain3_Gain_n / rtb_Divide1_j;
    rtb_FF_Right = rtb_Merge_o + rtb_Divide1_m + rtb_Divide1_j;
    rtb_Divide3 *= mpc_tgt_calc_P.Gain_Gain_k;
    rtb_Gain3_k = rtb_Abs7 * arg_ego1->resist * mpc_tgt_calc_P.Gain3_Gain_a;
    rtb_Gain4 = mpc_tgt_calc_P.Gain4_Gain_d * rtb_BusAssignment1_n_v;
    rtb_Abs6 = mpc_tgt_calc_P.Gain5_Gain_d * arg_ego1->tread *
      mpc_tgt_calc_P.Gain_Gain_p * rtb_Switch_w;
    rtb_Power_c = rtb_Gain4 - rtb_Abs6;
    if (std::isnan(rtb_Power_c)) {
      tmp_0 = (rtNaNF);
    } else if (rtb_Power_c < 0.0F) {
      tmp_0 = -1.0F;
    } else {
      tmp_0 = rtb_Power_c > 0.0F;
    }

    rtb_Abs7 = tmp_0 * arg_ego1->coulomb_friction + rtb_Power_c *
      arg_ego1->viscous_friction;
    rtb_Gain4 += rtb_Abs6;
    if (std::isnan(rtb_Gain4)) {
      tmp_0 = (rtNaNF);
    } else if (rtb_Gain4 < 0.0F) {
      tmp_0 = -1.0F;
    } else {
      tmp_0 = rtb_Gain4 > 0.0F;
    }

    rtb_Abs6 = tmp_0 * arg_ego1->coulomb_friction + rtb_Gain4 *
      arg_ego1->viscous_friction;
    if (s1_iter == 1) {
      std::memcpy(&mpc_tgt_calc_B.Assignment[0], &mpc_tgt_calc_P.Constant_Value
                  [0], 50U * sizeof(t_ego));
    }

    expl_temp.v = rtb_BusAssignment1_n_v;
    expl_temp.v_r = rtb_Switch_v_r;
    expl_temp.v_l = rtb_Switch_v_l;
    expl_temp.pos_x = rtb_Switch_pos_x;
    expl_temp.pos_y = rtb_Switch_pos_y;
    expl_temp.accl = rtb_Merge1_img_dist;
    expl_temp.w = rtb_Switch_w;
    expl_temp.alpha = rtb_Gain2_j;
    expl_temp.alpha2 = rtb_Switch_alpha2;
    expl_temp.dist = rtb_Switch_dist;
    expl_temp.ang = rtb_Switch_ang;
    expl_temp.img_dist = rtb_Switch_img_dist;
    expl_temp.img_ang = rtb_Switch_img_ang;
    expl_temp.sla_param.base_alpha = rtb_Switch_sla_param_base_alpha;
    expl_temp.sla_param.base_time = rtb_Switch_sla_param_base_time;
    expl_temp.sla_param.limit_time_count = rtb_Switch_sla_param_limit_time;
    expl_temp.sla_param.pow_n = rtb_Switch_sla_param_pow_n;
    expl_temp.sla_param.state = rtb_Switch_sla_param_state;
    expl_temp.sla_param.counter = rtb_Switch_sla_param_counter;
    expl_temp.state = rtb_pivot_state;
    expl_temp.pivot_state = rtb_Merge1_l;
    expl_temp.ideal_point.x = rtb_Switch_ideal_point_x;
    expl_temp.ideal_point.y = rtb_Switch_ideal_point_y;
    expl_temp.ideal_point.theta = rtb_Switch_ideal_point_theta;
    expl_temp.ideal_point.v = rtb_Switch_ideal_point_v;
    expl_temp.ideal_point.w = rtb_Switch_ideal_point_w;
    expl_temp.ideal_point.slip_angle = rtb_Switch_ideal_point_slip_ang;
    expl_temp.slip_point.x = rtb_Switch_slip_point_x;
    expl_temp.slip_point.y = rtb_Switch_slip_point_y;
    expl_temp.slip_point.theta = rtb_Switch_slip_point_theta;
    expl_temp.slip_point.v = rtb_Switch_slip_point_v;
    expl_temp.slip_point.w = rtb_Switch_slip_point_w;
    expl_temp.slip_point.slip_angle = rtb_Switch_slip_point_slip_angl;
    expl_temp.kanayama_point.x = rtb_Switch_kanayama_point_x;
    expl_temp.kanayama_point.y = rtb_Switch_kanayama_point_y;
    expl_temp.kanayama_point.theta = rtb_Switch_kanayama_point_theta;
    expl_temp.kanayama_point.v = rtb_Switch_kanayama_point_v;
    expl_temp.kanayama_point.w = rtb_Switch_kanayama_point_w;
    expl_temp.trj_diff.x = rtb_Switch_trj_diff_x;
    expl_temp.trj_diff.y = rtb_Switch_trj_diff_y;
    expl_temp.trj_diff.theta = rtb_Switch_trj_diff_theta;
    expl_temp.delay_accl = rtb_Switch_delay_accl;
    expl_temp.delay_v = rtb_Switch_delay_v;
    expl_temp.cnt_delay_accl_ratio = rtb_Switch_cnt_delay_accl_ratio;
    expl_temp.cnt_delay_decel_ratio = rtb_Switch_cnt_delay_decel_rati;
    expl_temp.slip.beta = rtb_Switch_slip_beta;
    expl_temp.slip.vx = rtb_Power1_b;
    expl_temp.slip.vy = rtb_Add3_c;
    expl_temp.slip.v = rtb_Sqrt;
    expl_temp.slip.accl = rtb_Divide2;
    expl_temp.ff_duty_l = rtb_FF_Left;
    expl_temp.ff_duty_r = rtb_FF_Right;
    expl_temp.ff_duty_low_th = rtb_Switch_ff_duty_low_th;
    expl_temp.ff_duty_low_v_th = rtb_Switch_ff_duty_low_v_th;
    expl_temp.ff_duty_front = rtb_Merge_o;
    expl_temp.ff_duty_roll = rtb_Divide1_m;
    expl_temp.ff_duty_rpm_r = rtb_Divide1_j;
    expl_temp.ff_duty_rpm_l = rtb_Divide_j;
    expl_temp.ff_front_torque = rtb_Divide3;
    expl_temp.ff_roll_torque = rtb_Gain3_k;
    expl_temp.ff_friction_torque_l = rtb_Abs7;
    expl_temp.ff_friction_torque_r = rtb_Abs6;
    expl_temp.decel_delay_cnt = rtb_Switch_decel_delay_cnt;
    mpc_tgt_calc_B.Assignment[s1_iter - 1] = expl_temp;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.v = rtb_BusAssignment1_n_v;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.v_r = rtb_Switch_v_r;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.v_l = rtb_Switch_v_l;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.pos_x = rtb_Switch_pos_x;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.pos_y = rtb_Switch_pos_y;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.accl = rtb_Merge1_img_dist;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.w = rtb_Switch_w;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.alpha = rtb_Gain2_j;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.alpha2 = rtb_Switch_alpha2;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.dist = rtb_Switch_dist;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ang = rtb_Switch_ang;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.img_dist = rtb_Switch_img_dist;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.img_ang = rtb_Switch_img_ang;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.base_alpha =
      rtb_Switch_sla_param_base_alpha;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.base_time =
      rtb_Switch_sla_param_base_time;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.limit_time_count =
      rtb_Switch_sla_param_limit_time;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.pow_n =
      rtb_Switch_sla_param_pow_n;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.state =
      rtb_Switch_sla_param_state;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.sla_param.counter =
      rtb_Switch_sla_param_counter;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.state = rtb_pivot_state;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.pivot_state = rtb_Merge1_l;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.x = rtb_Switch_ideal_point_x;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.y = rtb_Switch_ideal_point_y;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.theta =
      rtb_Switch_ideal_point_theta;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.v = rtb_Switch_ideal_point_v;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.w = rtb_Switch_ideal_point_w;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ideal_point.slip_angle =
      rtb_Switch_ideal_point_slip_ang;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.x = rtb_Switch_slip_point_x;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.y = rtb_Switch_slip_point_y;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.theta =
      rtb_Switch_slip_point_theta;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.v = rtb_Switch_slip_point_v;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.w = rtb_Switch_slip_point_w;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip_point.slip_angle =
      rtb_Switch_slip_point_slip_angl;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.x =
      rtb_Switch_kanayama_point_x;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.y =
      rtb_Switch_kanayama_point_y;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.theta =
      rtb_Switch_kanayama_point_theta;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.v =
      rtb_Switch_kanayama_point_v;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.kanayama_point.w =
      rtb_Switch_kanayama_point_w;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.trj_diff.x = rtb_Switch_trj_diff_x;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.trj_diff.y = rtb_Switch_trj_diff_y;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.trj_diff.theta = rtb_Switch_trj_diff_theta;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.delay_accl = rtb_Switch_delay_accl;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.delay_v = rtb_Switch_delay_v;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.cnt_delay_accl_ratio =
      rtb_Switch_cnt_delay_accl_ratio;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.cnt_delay_decel_ratio =
      rtb_Switch_cnt_delay_decel_rati;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.beta = rtb_Switch_slip_beta;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.vx = rtb_Power1_b;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.vy = rtb_Add3_c;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.v = rtb_Sqrt;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.slip.accl = rtb_Divide2;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_l = rtb_FF_Left;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_r = rtb_FF_Right;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_low_th = rtb_Switch_ff_duty_low_th;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_low_v_th =
      rtb_Switch_ff_duty_low_v_th;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_front = rtb_Merge_o;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_roll = rtb_Divide1_m;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_rpm_r = rtb_Divide1_j;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_duty_rpm_l = rtb_Divide_j;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_front_torque = rtb_Divide3;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_roll_torque = rtb_Gain3_k;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_friction_torque_l = rtb_Abs7;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.ff_friction_torque_r = rtb_Abs6;
    mpc_tgt_calc_DW.UnitDelay_DSTATE.decel_delay_cnt =
      rtb_Switch_decel_delay_cnt;
  }

  std::memcpy(&arg_next_ego[0], &mpc_tgt_calc_B.Assignment[0], 50U * sizeof
              (t_ego));
}

void mpc_tgt_calcModelClass::initialize()
{
  {
    int32_T i;
    mpc_tgt_calc_DW.UnitDelay_DSTATE = mpc_tgt_calc_P.UnitDelay_InitialCondition;
    mpc_tgt_calc_DW.UnitDelay_DSTATE_k =
      mpc_tgt_calc_P.UnitDelay_InitialCondition_o;
    mpc_tgt_calc_B.Merge_i[0] = mpc_tgt_calc_P.Merge_InitialOutput;
    mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Merge1_InitialOutput;
    mpc_tgt_calc_B.Merge_i[1] = mpc_tgt_calc_P.Merge_InitialOutput;
    mpc_tgt_calc_B.Merge1[1] = mpc_tgt_calc_P.Merge1_InitialOutput;
    for (i = 0; i < 50; i++) {
      mpc_tgt_calc_B.Assignment[i] = mpc_tgt_calc_P.next_ego_Y0;
    }
  }
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
