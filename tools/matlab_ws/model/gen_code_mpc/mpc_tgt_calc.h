#ifndef mpc_tgt_calc_h_
#define mpc_tgt_calc_h_
#include <cmath>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "bus.h"
#include "mpc_tgt_calc_types.h"

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include "bus.h"

class mpc_tgt_calcModelClass final
{
 public:
  struct B_mpc_tgt_calc_T {
    real32_T Merge[2];
    real32_T Merge1[2];
  };

  struct DW_mpc_tgt_calc_T {
    int32_T UnitDelay_DSTATE;
  };

  struct P_mpc_tgt_calc_T {
    real32_T dt;
    real_T Constant_Value;
    real_T Constant_Value_c;
    real_T Constant_Value_cm;
    real_T Constant3_Value;
    real_T Constant1_Value;
    real_T Constant2_Value;
    real_T Constant1_Value_h;
    real_T Gain_Gain;
    real_T Gain1_Gain;
    real_T Constant_Value_cx;
    real_T Constant_Value_cf;
    real_T Constant_Value_e;
    real_T Constant1_Value_d;
    real_T Constant1_Value_l;
    real_T Gain_Gain_j;
    real_T Constant_Value_k;
    real_T Constant_Value_n;
    real_T Constant1_Value_m;
    real_T Gain_Gain_g;
    real_T Constant_Value_p;
    real_T Constant_Value_g;
    int32_T Constant6_Value;
    int32_T Constant4_Value;
    int32_T Constant5_Value;
    int32_T Constant2_Value_p;
    int32_T Constant1_Value_lj;
    int32_T Constant2_Value_g;
    int32_T Constant4_Value_f;
    int32_T Constant2_Value_pg;
    int32_T Constant4_Value_o;
    int32_T UnitDelay_InitialCondition;
    int32_T Constant2_Value_i;
    int32_T Constant1_Value_k;
    int32_T Constant2_Value_d;
    int32_T Constant2_Value_dw;
    int32_T Constant2_Value_gm;
    int32_T Constant1_Value_p;
    int32_T Constant2_Value_f;
    int32_T Constant2_Value_g3;
    int32_T Constant1_Value_c;
    int32_T Constant2_Value_f2;
    int32_T Constant1_Value_o;
    int32_T Constant2_Value_j;
    int32_T Constant1_Value_e;
    int32_T DataStoreMemory_InitialValue;
    int32_T DataStoreMemory1_InitialValue;
    real32_T Gain_Gain_n;
    real32_T Saturation_UpperSat;
    real32_T Saturation_LowerSat;
    real32_T Gain1_Gain_i;
    real32_T Constant3_Value_c;
    real32_T Gain1_Gain_o;
    real32_T Merge_InitialOutput;
    real32_T Constant_Value_ne;
    real32_T Constant4_Value_c;
    real32_T Constant3_Value_cn;
    real32_T Gain1_Gain_n;
    real32_T Merge_InitialOutput_m;
    real32_T Constant_Value_h;
    real32_T Constant_Value_nl;
    real32_T Gain3_Gain;
    real32_T Constant1_Value_g;
    real32_T Gain1_Gain_e;
    real32_T Constant_Value_pi;
    real32_T Gain1_Gain_if;
    real32_T Gain_Gain_m;
    real32_T Constant1_Value_e0;
    real32_T Constant4_Value_j;
    real32_T Constant_Value_o;
    real32_T Gain4_Gain;
    real32_T Gain1_Gain_f;
    real32_T Gain1_Gain_d;
    real32_T Gain2_Gain;
    real32_T Gain4_Gain_d;
    real32_T Gain5_Gain;
    real32_T Gain_Gain_l;
    real32_T Gain2_Gain_o;
    real32_T Gain6_Gain;
    real32_T Gain1_Gain_nu;
    real32_T Gain3_Gain_e;
    real32_T Gain_Gain_l3;
    real32_T Gain3_Gain_g;
    real32_T Gain4_Gain_a;
    real32_T Gain5_Gain_k;
    real32_T Gain_Gain_jr;
    real32_T Constant1_Value_dt;
    real32_T Constant2_Value_ib;
    real32_T Constant4_Value_b;
    real32_T Constant6_Value_o;
    real32_T Constant_Value_ne4;
    real32_T Constant1_Value_mn;
    real32_T Constant2_Value_e;
    real32_T Constant3_Value_j;
    real32_T Constant7_Value;
    real32_T Constant8_Value;
    real32_T Gain1_Gain_l;
    real32_T Constant1_Value_pl;
    real32_T Constant2_Value_k;
    real32_T Constant4_Value_d;
    real32_T Constant6_Value_oh;
    real32_T Constant_Value_f;
    real32_T Constant1_Value_i;
    real32_T Constant2_Value_m;
    real32_T Constant3_Value_k;
    real32_T Constant7_Value_a;
    real32_T Constant8_Value_k;
    real32_T Gain1_Gain_d5;
    real32_T Constant_Value_j;
    real32_T Merge_InitialOutput_h;
    real32_T Merge1_InitialOutput;
    real32_T Gain1_Gain_h;
    real32_T Switch1_Threshold;
    real32_T Constant_Value_ph;
    real32_T Switch2_Threshold;
    real32_T Constant4_Value_i;
    real32_T Constant3_Value_f;
    real32_T Gain3_Gain_p;
    real32_T Gain2_Gain_g;
    real32_T Gain1_Gain_m;
    real32_T Switch_Threshold;
    real32_T Constant_Value_pr;
    real32_T Gain1_Gain_nc;
    real32_T Switch1_Threshold_i;
    real32_T Constant_Value_hr;
    real32_T Switch2_Threshold_m;
    real32_T Gain6_Gain_e;
    real32_T Gain7_Gain;
    real32_T Gain4_Gain_b;
    real32_T Gain2_Gain_i;
    real32_T Gain1_Gain_hy;
    real32_T Gain_Gain_mm;
    real32_T Constant_Value_m;
    real32_T Constant_Value_eh;
    int8_T Constant4_Value_k;
    uint8_T ManualSwitch_CurrentSetting;
    uint8_T ManualSwitch_CurrentSetting_b;
    uint8_T ManualSwitch_CurrentSetting_l;
    uint8_T ManualSwitch_CurrentSetting_o;
    uint8_T ManualSwitch_CurrentSetting_c;
  };

  struct RT_MODEL_mpc_tgt_calc_T {
    const char_T * volatile errorStatus;
    const char_T* getErrorStatus() const;
    void setErrorStatus(const char_T* const volatile aErrorStatus);
  };

  mpc_tgt_calcModelClass(mpc_tgt_calcModelClass const&) = delete;
  mpc_tgt_calcModelClass& operator= (mpc_tgt_calcModelClass const&) & = delete;
  mpc_tgt_calcModelClass(mpc_tgt_calcModelClass &&) = delete;
  mpc_tgt_calcModelClass& operator= (mpc_tgt_calcModelClass &&) = delete;
  mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * getRTM();
  void initialize();
  void step(const t_tgt *arg_tgt, const t_ego *arg_ego, int32_T arg_mode,
            int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics *arg_ego1,
            int32_T *arg_In1);
  static void terminate();
  mpc_tgt_calcModelClass();
  ~mpc_tgt_calcModelClass();
 private:
  B_mpc_tgt_calc_T mpc_tgt_calc_B;
  DW_mpc_tgt_calc_T mpc_tgt_calc_DW;
  static P_mpc_tgt_calc_T mpc_tgt_calc_P;
  RT_MODEL_mpc_tgt_calc_T mpc_tgt_calc_M;
};

#endif

