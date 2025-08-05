#include "mpc_tgt_calc.h"
#include "bus.h"
#include "rtwtypes.h"
#include <cmath>
#include "mpc_tgt_calc_private.h"
#include "cmath"

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
      y = std::sqrt(u0);
    } else if (u0 < 0.0F && u1 > std::floor(u1)) {
      y = (rtNaNF);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

void mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics
  *arg_ego1, int32_T *arg_In1)
{
  t_ego rtb_BusAssignment1_o;
  t_ego rtb_BusAssignment_b;
  real_T rtb_Switch_e;
  real_T tmp;
  int32_T rtb_Merge1_decel_delay_cnt;
  int32_T rtb_Merge1_n;
  int32_T rtb_pivot_state;
  real32_T Merge_k;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Add1_e;
  real32_T rtb_Add1_m;
  real32_T rtb_Add3;
  real32_T rtb_Divide1_a;
  real32_T rtb_Divide2;
  real32_T rtb_Divide_jy_tmp;
  real32_T rtb_Divide_l;
  real32_T rtb_Divide_m4;
  real32_T rtb_FF_Right;
  real32_T rtb_Gain1_cy;
  real32_T rtb_Gain1_mo_tmp;
  real32_T rtb_Gain3_m;
  real32_T rtb_Merge1_alpha;
  real32_T rtb_Merge1_alpha2;
  real32_T rtb_Merge1_cnt_delay_accl_ratio;
  real32_T rtb_Merge1_cnt_delay_decel_rati;
  real32_T rtb_Merge1_delay_accl;
  real32_T rtb_Merge1_delay_v;
  real32_T rtb_Merge1_ff_duty_low_th;
  real32_T rtb_Merge1_ff_duty_low_v_th;
  real32_T rtb_Merge1_ideal_point_slip_ang;
  real32_T rtb_Merge1_ideal_point_theta;
  real32_T rtb_Merge1_ideal_point_v;
  real32_T rtb_Merge1_ideal_point_w;
  real32_T rtb_Merge1_ideal_point_x;
  real32_T rtb_Merge1_ideal_point_y;
  real32_T rtb_Merge1_kanayama_point_theta;
  real32_T rtb_Merge1_kanayama_point_v;
  real32_T rtb_Merge1_kanayama_point_w;
  real32_T rtb_Merge1_kanayama_point_x;
  real32_T rtb_Merge1_kanayama_point_y;
  real32_T rtb_Merge1_pos_x;
  real32_T rtb_Merge1_sla_param_base_alpha;
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
  real32_T rtb_Power;
  real32_T rtb_Product2_l;
  real32_T rtb_Sqrt;
  real32_T rtb_Subtract2;
  real32_T rtb_Subtract2_n;
  real32_T rtb_Switch1_n_idx_1;
  boolean_T rtb_RelationalOperator_a;
  if (arg_tgt->v_max >= 0.0F) {
    rtb_Abs7 = arg_tgt->tgt_dist - arg_ego->dist;
    rtb_Abs6 = std::abs(arg_tgt->decel);
    rtb_Merge1_pos_x = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;
    if (arg_ego->v - arg_tgt->end_v > mpc_tgt_calc_P.Constant3_Value_a &&
        (arg_ego->state == mpc_tgt_calc_P.Constant1_Value_a || std::abs
         (rtb_Merge1_pos_x) / (mpc_tgt_calc_P.Gain1_Gain_o * rtb_Abs6) +
         arg_ego->dist >= arg_tgt->tgt_dist)) {
      if (arg_ego->v > arg_tgt->end_v) {
        if (rtb_Abs7 > mpc_tgt_calc_P.Constant3_Value) {
          tmp = mpc_tgt_calc_P.Gain_Gain_n * rtb_Abs7;
        } else {
          tmp = mpc_tgt_calc_P.Constant1_Value;
        }

        rtb_Switch_e = std::abs(rtb_Merge1_pos_x / tmp) *
          mpc_tgt_calc_P.Gain1_Gain;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value;
      }

      if (std::abs(mpc_tgt_calc_P.dt * static_cast<real32_T>(rtb_Switch_e) *
                   static_cast<real32_T>(arg_time_step) + arg_ego->v) > rtb_Abs6)
      {
        Merge_k = static_cast<real32_T>(rtb_Switch_e);
      } else {
        Merge_k = arg_tgt->decel;
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

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
    } else {
      Merge_k = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_cm);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_pg;
    }

    rtb_Add1_e = std::fmax(std::fmin(arg_tgt->v_max, mpc_tgt_calc_P.dt * Merge_k
      * static_cast<real32_T>(arg_time_step) + arg_ego->v),
      mpc_tgt_calc_P.Constant_Value_ne);
  } else {
    rtb_Abs7 = std::abs(arg_ego->dist);
    rtb_Abs6 = std::abs(arg_tgt->tgt_dist);
    rtb_Merge1_pos_x = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;
    if (arg_ego->state == 2.0F || std::abs(std::abs(rtb_Merge1_pos_x) /
         (mpc_tgt_calc_P.Gain1_Gain_n * std::abs(arg_tgt->decel))) + rtb_Abs7 >=
        rtb_Abs6) {
      if (arg_ego->v < arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          rtb_Switch_e = std::abs(rtb_Merge1_pos_x / (std::fmax
            (mpc_tgt_calc_P.Constant1_Value_h, static_cast<real_T>(rtb_Abs6 -
            rtb_Abs7)) * mpc_tgt_calc_P.Gain_Gain)) *
            mpc_tgt_calc_P.Gain1_Gain_c;
        }
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cx;
      }

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_id;
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

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
    } else {
      Merge_k = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_dw;
    }

    rtb_Add1_e = std::fmin(std::fmax(arg_tgt->v_max, mpc_tgt_calc_P.dt * Merge_k
      * static_cast<real32_T>(arg_time_step) + arg_ego->v),
      mpc_tgt_calc_P.Constant_Value_h);
  }

  rtb_Abs6 = rtb_Add1_e * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.dt;
  rtb_Gain1_cy = rtb_Add1_e;
  rtb_Abs7 = arg_ego->dist + rtb_Abs6;
  switch (arg_mode) {
   case 1:
    rtb_Merge1_n = arg_time_step + arg_ego->sla_param.counter;
    if (rtb_Merge1_n > arg_ego->sla_param.limit_time_count) {
      tmp = mpc_tgt_calc_P.Constant1_Value_m;
    } else {
      tmp = mpc_tgt_calc_P.Constant2_Value_i;
    }

    tmp = std::trunc(tmp);
    if (std::isnan(tmp) || std::isinf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    if ((tmp < 0.0 ? -static_cast<int32_T>(static_cast<uint32_T>(-tmp)) :
         static_cast<int32_T>(static_cast<uint32_T>(tmp))) == 1) {
      mpc_tgt_calc_B.Merge[0] = mpc_tgt_calc_P.Constant1_Value_dt;
      mpc_tgt_calc_B.Merge[1] = mpc_tgt_calc_P.Constant2_Value_ib;
    } else {
      rtb_Add1_e = static_cast<real32_T>(rtb_Merge1_n) * mpc_tgt_calc_P.dt /
        arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_ne4;
      rtb_Power = rt_powf_snf(rtb_Add1_e, arg_ego->sla_param.pow_n -
        mpc_tgt_calc_P.Constant1_Value_mn);
      rtb_Add1_e *= rtb_Power;
      rtb_Subtract2 = rtb_Add1_e - mpc_tgt_calc_P.Constant2_Value_e;
      mpc_tgt_calc_B.Merge[0] = mpc_tgt_calc_P.Gain1_Gain_l *
        arg_ego->sla_param.pow_n * rtb_Power / (rtb_Subtract2 * rtb_Subtract2) *
        std::exp(mpc_tgt_calc_P.Constant6_Value_o / rtb_Subtract2 +
                 mpc_tgt_calc_P.Constant4_Value_b) /
        arg_ego->sla_param.base_time;
      mpc_tgt_calc_B.Merge[1] = std::exp(mpc_tgt_calc_P.Constant8_Value_e /
        (mpc_tgt_calc_P.Constant7_Value_m - rtb_Add1_e)) *
        mpc_tgt_calc_P.Constant3_Value_j;
    }

    if (arg_ego->sla_param.state != 0) {
      rtb_Add1_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Add1_e = arg_ego->sla_param.base_alpha * mpc_tgt_calc_B.Merge[0];
    }

    if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_l == 1) {
      if (arg_ego->sla_param.counter >= arg_ego->sla_param.limit_time_count) {
        rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_d;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.dt * rtb_Add1_e * static_cast<real32_T>
          (arg_time_step) + arg_ego->w;
      }
    } else if (arg_ego->sla_param.state != 0) {
      rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Switch_e = arg_ego->sla_param.base_alpha * mpc_tgt_calc_B.Merge[1];
    }

    rtb_Merge1_pos_x = static_cast<real32_T>(arg_tgt->time_step2 +
      arg_ego->sla_param.counter);
    if (rtb_Merge1_pos_x > arg_ego->sla_param.limit_time_count) {
      tmp = mpc_tgt_calc_P.Constant3_Value_o;
    } else {
      tmp = mpc_tgt_calc_P.Constant4_Value;
    }

    tmp = std::trunc(tmp);
    if (std::isnan(tmp) || std::isinf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    if ((tmp < 0.0 ? -static_cast<int32_T>(static_cast<uint32_T>(-tmp)) :
         static_cast<int32_T>(static_cast<uint32_T>(tmp))) == 1) {
      mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Constant1_Value_pl;
      mpc_tgt_calc_B.Merge1[1] = mpc_tgt_calc_P.Constant2_Value_k;
    } else {
      rtb_Power = rtb_Merge1_pos_x * mpc_tgt_calc_P.dt /
        arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_f;
      rtb_Subtract2 = rt_powf_snf(rtb_Power, arg_ego->sla_param.pow_n -
        mpc_tgt_calc_P.Constant1_Value_i);
      rtb_Power *= rtb_Subtract2;
      rtb_Subtract2_n = rtb_Power - mpc_tgt_calc_P.Constant2_Value_m;
      mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Gain1_Gain_d5 *
        arg_ego->sla_param.pow_n * rtb_Subtract2 / (rtb_Subtract2_n *
        rtb_Subtract2_n) * std::exp(mpc_tgt_calc_P.Constant6_Value /
        rtb_Subtract2_n + mpc_tgt_calc_P.Constant4_Value_d) /
        arg_ego->sla_param.base_time;
      mpc_tgt_calc_B.Merge1[1] = std::exp(mpc_tgt_calc_P.Constant8_Value /
        (mpc_tgt_calc_P.Constant7_Value - rtb_Power)) *
        mpc_tgt_calc_P.Constant3_Value_k;
    }

    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = static_cast<real32_T>(rtb_Switch_e);
    rtb_BusAssignment1_o.alpha = rtb_Add1_e;
    rtb_BusAssignment1_o.sla_param.counter = rtb_Merge1_n;
    if (arg_ego->sla_param.state != 0) {
      rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_BusAssignment1_o.alpha2 = arg_ego->sla_param.base_alpha *
        mpc_tgt_calc_B.Merge1[0];
    }
    break;

   case 2:
    rtb_Add1_e = std::abs(arg_ego->ang);
    rtb_Subtract2 = mpc_tgt_calc_P.Gain2_Gain_g * arg_tgt->alpha;
    rtb_Power = std::abs(arg_tgt->tgt_angle);
    if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Subtract2_n = arg_tgt->w_max;
    } else {
      rtb_Subtract2_n = mpc_tgt_calc_P.Gain3_Gain_p * arg_tgt->w_max;
    }

    rtb_Merge1_pos_x = arg_ego->w * arg_ego->w - arg_tgt->end_w * arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Merge1_pos_x) /
        (mpc_tgt_calc_P.Gain1_Gain_m * std::abs(rtb_Subtract2)) + rtb_Add1_e >=
        rtb_Power) {
      if (rtb_Subtract2 > mpc_tgt_calc_P.Switch2_Threshold) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_o != 1) {
          if (rtb_Subtract2 > mpc_tgt_calc_P.Switch1_Threshold) {
            rtb_Subtract2 = static_cast<real32_T>(std::abs(rtb_Merge1_pos_x /
              (std::fmax(mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>
                         (rtb_Power - rtb_Add1_e)) * mpc_tgt_calc_P.Gain_Gain_j)));
          } else {
            rtb_Subtract2 = static_cast<real32_T>(std::abs(rtb_Merge1_pos_x /
              (std::fmax(mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>
                         (rtb_Power - rtb_Add1_e)) * mpc_tgt_calc_P.Gain_Gain_j)))
              * mpc_tgt_calc_P.Gain1_Gain_h;
          }
        }
      } else {
        rtb_Subtract2 = mpc_tgt_calc_P.Constant_Value_ph;
      }

      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_gm;
    } else if (arg_ego->pivot_state == 0.0F) {
      if (std::abs(arg_ego->w) < std::abs(rtb_Subtract2_n)) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_c == 1) {
          rtb_Subtract2 = arg_tgt->alpha;
        } else {
          rtb_Subtract2 = (mpc_tgt_calc_P.Constant3_Value_f - rt_powf_snf
                           (arg_ego->w / rtb_Subtract2_n,
                            mpc_tgt_calc_P.Constant4_Value_i)) * arg_tgt->alpha;
        }

        rtb_Switch_e = rtb_Subtract2;
        rtb_Merge1_n = mpc_tgt_calc_P.Constant1_Value_p;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_k;
        rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_f;
      }

      rtb_Subtract2 = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Subtract2 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_n);
      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_g3;
    }

    rtb_BusAssignment_b = *arg_ego;
    rtb_BusAssignment_b.w = mpc_tgt_calc_P.dt * rtb_Subtract2 *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_b.alpha = rtb_Subtract2;
    rtb_BusAssignment_b.alpha2 = rtb_Subtract2;
    rtb_BusAssignment_b.pivot_state = rtb_Merge1_n;
    rtb_BusAssignment1_o = rtb_BusAssignment_b;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_c;
    if (rtb_Add1_e < rtb_Power) {
      rtb_BusAssignment1_o = rtb_BusAssignment_b;
    }
    break;

   case 4:
    rtb_Add1_e = std::abs(arg_ego->img_ang);
    rtb_Subtract2 = mpc_tgt_calc_P.Gain2_Gain_i * arg_tgt->alpha;
    rtb_Power = std::abs(arg_tgt->tgt_angle);
    if (arg_ego->sla_param.pow_n != mpc_tgt_calc_P.Constant_Value_g) {
      rtb_Subtract2_n = mpc_tgt_calc_P.Gain6_Gain_e * rtb_Power;
      rtb_Switch1_n_idx_1 = mpc_tgt_calc_P.Gain7_Gain * rtb_Power;
    } else {
      rtb_Subtract2_n = mpc_tgt_calc_P.Gain4_Gain_b * rtb_Power;
      rtb_Switch1_n_idx_1 = rtb_Subtract2_n;
    }

    rtb_Merge1_pos_x = arg_ego->w * arg_ego->w - arg_tgt->end_w * arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Merge1_pos_x) /
        (mpc_tgt_calc_P.Gain1_Gain_hy * std::abs(rtb_Subtract2)) + rtb_Add1_e >=
        mpc_tgt_calc_P.Gain_Gain_mm * rtb_Power || rtb_Add1_e >= rtb_Subtract2_n)
    {
      if (rtb_Subtract2 > mpc_tgt_calc_P.Switch2_Threshold_m) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (rtb_Subtract2 > mpc_tgt_calc_P.Switch1_Threshold_i) {
          rtb_Subtract2 = static_cast<real32_T>(std::abs(rtb_Merge1_pos_x / (std::
            fmax(static_cast<real_T>(rtb_Power - rtb_Add1_e),
                 mpc_tgt_calc_P.Constant1_Value_mw) * mpc_tgt_calc_P.Gain_Gain_g)));
        } else {
          rtb_Subtract2 = static_cast<real32_T>(std::abs(rtb_Merge1_pos_x / (std::
            fmax(static_cast<real_T>(rtb_Power - rtb_Add1_e),
                 mpc_tgt_calc_P.Constant1_Value_mw) * mpc_tgt_calc_P.Gain_Gain_g)))
            * mpc_tgt_calc_P.Gain1_Gain_nc;
        }
      } else {
        rtb_Subtract2 = mpc_tgt_calc_P.Constant_Value_hr;
      }

      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_f2;
    } else if (arg_ego->pivot_state == 0.0F && rtb_Add1_e < rtb_Switch1_n_idx_1)
    {
      rtb_Subtract2 = arg_tgt->alpha;
      rtb_Merge1_n = mpc_tgt_calc_P.Constant1_Value_o;
    } else {
      rtb_Subtract2 = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_p);
      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_j;
    }

    rtb_BusAssignment_b = *arg_ego;
    rtb_BusAssignment_b.w = mpc_tgt_calc_P.dt * rtb_Subtract2 *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_b.alpha = rtb_Subtract2;
    rtb_BusAssignment_b.pivot_state = rtb_Merge1_n;
    rtb_BusAssignment1_o = rtb_BusAssignment_b;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha2 = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_e;
    if (rtb_Add1_e < rtb_Power) {
      rtb_BusAssignment1_o = rtb_BusAssignment_b;
    }
    break;

   default:
    rtb_Add1_e = (mpc_tgt_calc_P.Constant_Value_eh - arg_ego->w) /
      (mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step));
    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_eh;
    rtb_BusAssignment1_o.alpha = rtb_Add1_e;
    rtb_BusAssignment1_o.alpha2 = rtb_Add1_e;
    break;
  }

  rtb_Power = rtb_BusAssignment1_o.w * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.dt;
  rtb_Add1_e = arg_ego->ang + rtb_Power;
  rtb_Abs6 += arg_ego->img_dist;
  rtb_Power += arg_ego->img_ang;
  if (*arg_In1 == 1) {
    if (mpc_tgt_calc_P.Constant4_Value_k != 0 && arg_tgt->enable_slip_decel != 0)
    {
      Merge_k = mpc_tgt_calc_P.dt * static_cast<real32_T>(arg_time_step);
      rtb_Add1_m = (mpc_tgt_calc_P.Constant_Value_nl / arg_ego1->mass +
                    rtb_BusAssignment1_o.w * arg_ego->slip.vy) * Merge_k +
        arg_ego->slip.vx;
      rtb_Add3 = (mpc_tgt_calc_P.Gain3_Gain * arg_tgt->slip_gain_K1 *
                  arg_ego->slip.beta / arg_ego1->mass - rtb_BusAssignment1_o.w *
                  arg_ego->slip.vx) * Merge_k + arg_ego->slip.vy;
      rtb_Sqrt = std::sqrt(rtb_Add1_m * rtb_Add1_m + rtb_Add3 * rtb_Add3);
      rtb_Gain1_cy = mpc_tgt_calc_P.Gain1_Gain_e * rtb_Sqrt;
      rtb_Divide2 = (rtb_Gain1_cy - arg_ego->v) / Merge_k;
      rtb_Subtract2 = arg_ego->v_r;
      rtb_Subtract2_n = arg_ego->v_l;
      rtb_Merge1_pos_x = arg_ego->pos_x;
      rtb_Switch1_n_idx_1 = arg_ego->pos_y;
      rtb_Merge1_w = rtb_BusAssignment1_o.w;
      rtb_Merge1_alpha = rtb_BusAssignment1_o.alpha;
      rtb_Merge1_alpha2 = rtb_BusAssignment1_o.alpha2;
      rtb_Merge1_sla_param_base_alpha = arg_ego->sla_param.base_alpha;
      rtb_Merge1_sla_param_base_time = arg_ego->sla_param.base_time;
      rtb_Merge1_sla_param_limit_time = arg_ego->sla_param.limit_time_count;
      rtb_Merge1_sla_param_pow_n = arg_ego->sla_param.pow_n;
      rtb_Merge1_n = arg_ego->sla_param.state;
      rtb_Merge1_ideal_point_x = arg_ego->ideal_point.x;
      rtb_Merge1_ideal_point_y = arg_ego->ideal_point.y;
      rtb_Merge1_ideal_point_theta = arg_ego->ideal_point.theta;
      rtb_Merge1_ideal_point_v = arg_ego->ideal_point.v;
      rtb_Merge1_ideal_point_w = arg_ego->ideal_point.w;
      rtb_Merge1_ideal_point_slip_ang = arg_ego->ideal_point.slip_angle;
      rtb_Merge1_slip_point_x = arg_ego->slip_point.x;
      rtb_Merge1_slip_point_y = arg_ego->slip_point.y;
      rtb_Merge1_slip_point_theta = arg_ego->slip_point.theta;
      rtb_Merge1_slip_point_v = arg_ego->slip_point.v;
      rtb_Merge1_slip_point_w = arg_ego->slip_point.w;
      rtb_Merge1_slip_point_slip_angl = arg_ego->slip_point.slip_angle;
      rtb_Merge1_kanayama_point_x = arg_ego->kanayama_point.x;
      rtb_Merge1_kanayama_point_y = arg_ego->kanayama_point.y;
      rtb_Merge1_kanayama_point_theta = arg_ego->kanayama_point.theta;
      rtb_Merge1_kanayama_point_v = arg_ego->kanayama_point.v;
      rtb_Merge1_kanayama_point_w = arg_ego->kanayama_point.w;
      rtb_Merge1_trj_diff_x = arg_ego->trj_diff.x;
      rtb_Merge1_trj_diff_y = arg_ego->trj_diff.y;
      rtb_Merge1_trj_diff_theta = arg_ego->trj_diff.theta;
      rtb_Merge1_delay_accl = arg_ego->delay_accl;
      rtb_Merge1_delay_v = arg_ego->delay_v;
      rtb_Merge1_cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
      rtb_Merge1_cnt_delay_decel_rati = arg_ego->cnt_delay_decel_ratio;
      rtb_Merge1_ff_duty_low_th = arg_ego->ff_duty_low_th;
      rtb_Merge1_ff_duty_low_v_th = arg_ego->ff_duty_low_v_th;
      rtb_Merge1_decel_delay_cnt = arg_ego->decel_delay_cnt;
      rtb_Merge1_slip_beta = (arg_ego->slip.beta / Merge_k -
        rtb_BusAssignment1_o.w) / (mpc_tgt_calc_P.Constant1_Value_g / Merge_k +
        arg_tgt->slip_gain_K2 / rtb_Sqrt);
      Merge_k = rtb_Divide2;
    } else {
      rtb_Subtract2 = arg_ego->v_r;
      rtb_Subtract2_n = arg_ego->v_l;
      rtb_Merge1_pos_x = arg_ego->pos_x;
      rtb_Switch1_n_idx_1 = arg_ego->pos_y;
      rtb_Merge1_w = rtb_BusAssignment1_o.w;
      rtb_Merge1_alpha = rtb_BusAssignment1_o.alpha;
      rtb_Merge1_alpha2 = rtb_BusAssignment1_o.alpha2;
      rtb_Merge1_sla_param_base_alpha = arg_ego->sla_param.base_alpha;
      rtb_Merge1_sla_param_base_time = arg_ego->sla_param.base_time;
      rtb_Merge1_sla_param_limit_time = arg_ego->sla_param.limit_time_count;
      rtb_Merge1_sla_param_pow_n = arg_ego->sla_param.pow_n;
      rtb_Merge1_n = arg_ego->sla_param.state;
      rtb_Merge1_ideal_point_x = arg_ego->ideal_point.x;
      rtb_Merge1_ideal_point_y = arg_ego->ideal_point.y;
      rtb_Merge1_ideal_point_theta = arg_ego->ideal_point.theta;
      rtb_Merge1_ideal_point_v = arg_ego->ideal_point.v;
      rtb_Merge1_ideal_point_w = arg_ego->ideal_point.w;
      rtb_Merge1_ideal_point_slip_ang = arg_ego->ideal_point.slip_angle;
      rtb_Merge1_slip_point_x = arg_ego->slip_point.x;
      rtb_Merge1_slip_point_y = arg_ego->slip_point.y;
      rtb_Merge1_slip_point_theta = arg_ego->slip_point.theta;
      rtb_Merge1_slip_point_v = arg_ego->slip_point.v;
      rtb_Merge1_slip_point_w = arg_ego->slip_point.w;
      rtb_Merge1_slip_point_slip_angl = arg_ego->slip_point.slip_angle;
      rtb_Merge1_kanayama_point_x = arg_ego->kanayama_point.x;
      rtb_Merge1_kanayama_point_y = arg_ego->kanayama_point.y;
      rtb_Merge1_kanayama_point_theta = arg_ego->kanayama_point.theta;
      rtb_Merge1_kanayama_point_v = arg_ego->kanayama_point.v;
      rtb_Merge1_kanayama_point_w = arg_ego->kanayama_point.w;
      rtb_Merge1_trj_diff_x = arg_ego->trj_diff.x;
      rtb_Merge1_trj_diff_y = arg_ego->trj_diff.y;
      rtb_Merge1_trj_diff_theta = arg_ego->trj_diff.theta;
      rtb_Merge1_delay_accl = arg_ego->delay_accl;
      rtb_Merge1_delay_v = arg_ego->delay_v;
      rtb_Merge1_cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
      rtb_Merge1_cnt_delay_decel_rati = arg_ego->cnt_delay_decel_ratio;
      rtb_Merge1_ff_duty_low_th = arg_ego->ff_duty_low_th;
      rtb_Merge1_ff_duty_low_v_th = arg_ego->ff_duty_low_v_th;
      rtb_Merge1_decel_delay_cnt = arg_ego->decel_delay_cnt;
      rtb_Merge1_slip_beta = mpc_tgt_calc_P.Constant_Value_pi;
      rtb_Add1_m = mpc_tgt_calc_P.Gain1_Gain_i * rtb_Gain1_cy;
      rtb_Add3 = mpc_tgt_calc_P.Constant_Value_pi;
      rtb_Sqrt = mpc_tgt_calc_P.Gain_Gain_m * rtb_Gain1_cy;
      rtb_Divide2 = arg_ego->slip.accl;
    }

    if (std::isnan(Merge_k) || std::isinf(Merge_k)) {
      Merge_k = mpc_tgt_calc_P.Constant_Value_o;
      rtb_Gain1_cy = arg_ego->v;
      rtb_BusAssignment1_o.img_dist = arg_ego->img_dist;
    } else {
      rtb_BusAssignment1_o.img_dist = rtb_Abs6;
    }

    rtb_RelationalOperator_a = std::isnan(rtb_Merge1_alpha) || std::isinf
      (rtb_Merge1_alpha);
    rtb_Abs6 = rtb_Gain1_cy;
    if (rtb_RelationalOperator_a) {
      rtb_Merge1_w = arg_ego->w;
    }

    if (std::isnan(arg_ego->alpha2) || std::isinf(arg_ego->alpha2)) {
      rtb_Merge1_alpha2 = mpc_tgt_calc_P.Constant4_Value_j;
    }

    rtb_FF_Right = arg_ego1->gear_ratio * arg_ego1->km;
    rtb_Divide_jy_tmp = mpc_tgt_calc_P.Gain4_Gain * Merge_k * arg_ego1->mass *
      (mpc_tgt_calc_P.Gain1_Gain_f * arg_ego1->tire);
    rtb_Divide_m4 = rtb_Divide_jy_tmp * arg_ego1->resist / rtb_FF_Right;
    if ((arg_mode == 0 || arg_mode == 3) && rtb_pivot_state == 0 &&
        !(rtb_Gain1_cy > rtb_Merge1_ff_duty_low_v_th)) {
      rtb_Divide_m4 = std::fmax(rtb_Divide_m4, rtb_Merge1_ff_duty_low_th);
    }

    rtb_Gain1_mo_tmp = rtb_Merge1_alpha2 * arg_ego1->lm;
    rtb_Gain1_cy = mpc_tgt_calc_P.Gain1_Gain_d * arg_ego1->tire *
      rtb_Gain1_mo_tmp * arg_ego1->resist / rtb_FF_Right /
      (mpc_tgt_calc_P.Gain2_Gain * arg_ego1->tread);
    rtb_Gain3_m = mpc_tgt_calc_P.Gain4_Gain_d * rtb_Abs6;
    rtb_FF_Right = mpc_tgt_calc_P.Gain5_Gain * arg_ego1->tread *
      mpc_tgt_calc_P.Gain_Gain_l * rtb_Merge1_w;
    rtb_Divide1_a = mpc_tgt_calc_P.Gain6_Gain * arg_ego1->tire *
      mpc_tgt_calc_P.Gain1_Gain_nu;
    rtb_Divide_l = (rtb_Gain3_m - rtb_FF_Right) * arg_ego1->ke *
      mpc_tgt_calc_P.Gain2_Gain_o / rtb_Divide1_a;
    rtb_Divide1_a = (rtb_Gain3_m + rtb_FF_Right) * arg_ego1->ke *
      mpc_tgt_calc_P.Gain3_Gain_e / rtb_Divide1_a;
    rtb_Product2_l = mpc_tgt_calc_P.Gain4_Gain_a * rtb_Abs6;
    rtb_Gain3_m = mpc_tgt_calc_P.Gain5_Gain_k * arg_ego1->tread *
      mpc_tgt_calc_P.Gain_Gain_jr * rtb_Merge1_w;
    rtb_FF_Right = rtb_Product2_l - rtb_Gain3_m;
    rtb_Product2_l += rtb_Gain3_m;
    rtb_BusAssignment1_o.v = rtb_Abs6;
    rtb_BusAssignment1_o.v_r = rtb_Subtract2;
    rtb_BusAssignment1_o.v_l = rtb_Subtract2_n;
    rtb_BusAssignment1_o.pos_x = rtb_Merge1_pos_x;
    rtb_BusAssignment1_o.pos_y = rtb_Switch1_n_idx_1;
    rtb_BusAssignment1_o.accl = Merge_k;
    rtb_BusAssignment1_o.w = rtb_Merge1_w;
    if (rtb_RelationalOperator_a) {
      rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant1_Value_e0;
    } else {
      rtb_BusAssignment1_o.alpha = rtb_Merge1_alpha;
    }

    rtb_BusAssignment1_o.alpha2 = rtb_Merge1_alpha2;
    rtb_BusAssignment1_o.dist = rtb_Abs7;
    rtb_BusAssignment1_o.ang = rtb_Add1_e;
    if (rtb_RelationalOperator_a) {
      rtb_BusAssignment1_o.img_ang = arg_ego->img_ang;
    } else {
      rtb_BusAssignment1_o.img_ang = rtb_Power;
    }

    rtb_BusAssignment1_o.sla_param.base_alpha = rtb_Merge1_sla_param_base_alpha;
    rtb_BusAssignment1_o.sla_param.base_time = rtb_Merge1_sla_param_base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      rtb_Merge1_sla_param_limit_time;
    rtb_BusAssignment1_o.sla_param.pow_n = rtb_Merge1_sla_param_pow_n;
    rtb_BusAssignment1_o.sla_param.state = rtb_Merge1_n;
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
    rtb_BusAssignment1_o.slip.vx = rtb_Add1_m;
    rtb_BusAssignment1_o.slip.vy = rtb_Add3;
    rtb_BusAssignment1_o.slip.v = rtb_Sqrt;
    rtb_BusAssignment1_o.slip.accl = rtb_Divide2;
    rtb_BusAssignment1_o.ff_duty_low_th = rtb_Merge1_ff_duty_low_th;
    rtb_BusAssignment1_o.ff_duty_low_v_th = rtb_Merge1_ff_duty_low_v_th;
    rtb_BusAssignment1_o.decel_delay_cnt = rtb_Merge1_decel_delay_cnt;
    rtb_BusAssignment1_o.ff_duty_front = rtb_Divide_m4;
    rtb_BusAssignment1_o.ff_duty_l = rtb_Divide_m4 - rtb_Gain1_cy + rtb_Divide_l;
    rtb_BusAssignment1_o.ff_duty_r = rtb_Divide_m4 + rtb_Gain1_cy +
      rtb_Divide1_a;
    rtb_BusAssignment1_o.ff_duty_roll = rtb_Gain1_cy;
    rtb_BusAssignment1_o.ff_duty_rpm_r = rtb_Divide1_a;
    rtb_BusAssignment1_o.ff_duty_rpm_l = rtb_Divide_l;
    rtb_BusAssignment1_o.ff_front_torque = mpc_tgt_calc_P.Gain_Gain_l3 *
      rtb_Divide_jy_tmp;
    rtb_BusAssignment1_o.ff_roll_torque = rtb_Gain1_mo_tmp * arg_ego1->resist *
      mpc_tgt_calc_P.Gain3_Gain_g;
    if (std::isnan(rtb_FF_Right)) {
      rtb_Merge1_pos_x = (rtNaNF);
    } else if (rtb_FF_Right < 0.0F) {
      rtb_Merge1_pos_x = -1.0F;
    } else {
      rtb_Merge1_pos_x = rtb_FF_Right > 0.0F;
    }

    rtb_BusAssignment1_o.ff_friction_torque_l = rtb_Merge1_pos_x *
      arg_ego1->coulomb_friction + rtb_FF_Right * arg_ego1->viscous_friction;
    if (std::isnan(rtb_Product2_l)) {
      rtb_Merge1_pos_x = (rtNaNF);
    } else if (rtb_Product2_l < 0.0F) {
      rtb_Merge1_pos_x = -1.0F;
    } else {
      rtb_Merge1_pos_x = rtb_Product2_l > 0.0F;
    }

    rtb_BusAssignment1_o.ff_friction_torque_r = rtb_Merge1_pos_x *
      arg_ego1->coulomb_friction + rtb_Product2_l * arg_ego1->viscous_friction;
  } else {
    rtb_BusAssignment1_o.v = rtb_Gain1_cy;
    rtb_BusAssignment1_o.v_r = arg_ego->v_r;
    rtb_BusAssignment1_o.v_l = arg_ego->v_l;
    rtb_BusAssignment1_o.pos_x = arg_ego->pos_x;
    rtb_BusAssignment1_o.pos_y = arg_ego->pos_y;
    rtb_BusAssignment1_o.accl = Merge_k;
    rtb_BusAssignment1_o.dist = rtb_Abs7;
    rtb_BusAssignment1_o.ang = rtb_Add1_e;
    rtb_BusAssignment1_o.img_dist = rtb_Abs6;
    rtb_BusAssignment1_o.img_ang = rtb_Power;
    rtb_BusAssignment1_o.sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    rtb_BusAssignment1_o.sla_param.base_time = arg_ego->sla_param.base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    rtb_BusAssignment1_o.sla_param.pow_n = arg_ego->sla_param.pow_n;
    rtb_BusAssignment1_o.sla_param.state = arg_ego->sla_param.state;
    rtb_BusAssignment1_o.state = rtb_pivot_state;
    rtb_BusAssignment1_o.ideal_point = arg_ego->ideal_point;
    rtb_BusAssignment1_o.slip_point = arg_ego->slip_point;
    rtb_BusAssignment1_o.kanayama_point = arg_ego->kanayama_point;
    rtb_BusAssignment1_o.trj_diff = arg_ego->trj_diff;
    rtb_BusAssignment1_o.delay_accl = arg_ego->delay_accl;
    rtb_BusAssignment1_o.delay_v = arg_ego->delay_v;
    rtb_BusAssignment1_o.cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
    rtb_BusAssignment1_o.cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;
    rtb_BusAssignment1_o.slip = arg_ego->slip;
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
    rtb_BusAssignment1_o.ff_friction_torque_l = arg_ego->ff_friction_torque_l;
    rtb_BusAssignment1_o.ff_friction_torque_r = arg_ego->ff_friction_torque_r;
    rtb_BusAssignment1_o.decel_delay_cnt = arg_ego->decel_delay_cnt;
  }

  rtb_Abs7 = mpc_tgt_calc_P.dt * rtb_BusAssignment1_o.v;
  rtb_BusAssignment1_o.pos_x += rtb_Abs7 * std::cos(rtb_BusAssignment1_o.img_ang);
  rtb_BusAssignment1_o.pos_y += rtb_Abs7 * std::sin(rtb_BusAssignment1_o.img_ang);
  *arg_next_ego = rtb_BusAssignment1_o;
}

void mpc_tgt_calcModelClass::initialize()
{
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
  mpc_tgt_calc_M()
{
}

mpc_tgt_calcModelClass::~mpc_tgt_calcModelClass() = default;
mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * mpc_tgt_calcModelClass::getRTM
  ()
{
  return (&mpc_tgt_calc_M);
}
