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
    t_ego Assignment[50];
    real32_T Merge_i[2];
    real32_T Merge1[2];
  };

  struct DW_mpc_tgt_calc_T {
    t_ego UnitDelay_DSTATE;
    int32_T UnitDelay_DSTATE_k;
  };

  struct P_IfActionSubsystem_mpc_tgt_c_T {
    real32_T Constant1_Value;
    real32_T Constant2_Value;
  };

  struct P_mpc_tgt_calc_T {
    real32_T dt;
    t_ego next_ego_Y0;
    t_ego UnitDelay_InitialCondition;
    t_ego Constant_Value[50];
    real_T Constant1_Value;
    real_T Constant1_Value_d;
    real_T Gain_Gain;
    real_T Constant_Value_i;
    real_T Constant_Value_b;
    real_T Constant1_Value_m;
    real_T Gain_Gain_f;
    real_T Constant_Value_k;
    real_T Constant_Value_bv;
    real_T Constant1_Value_g;
    real_T Constant3_Value;
    real_T Gain1_Gain;
    real_T Constant_Value_l;
    real_T Constant_Value_ii;
    real_T Constant_Value_c;
    real_T Constant3_Value_m;
    real_T Constant1_Value_o;
    real_T Constant2_Value;
    real_T Constant1_Value_p;
    real_T Gain_Gain_m;
    real_T Gain1_Gain_p;
    real_T Constant_Value_d;
    real_T Constant_Value_bn;
    real_T Constant_Value_g;
    int32_T Constant2_Value_b;
    int32_T Constant1_Value_d3;
    int32_T Constant2_Value_l;
    int32_T Constant2_Value_p;
    int32_T Constant1_Value_j;
    int32_T Constant2_Value_p0;
    int32_T Constant1_Value_c;
    int32_T Constant2_Value_pj;
    int32_T Constant1_Value_b;
    int32_T Constant6_Value;
    int32_T Constant4_Value;
    int32_T Constant5_Value;
    int32_T Constant2_Value_m;
    int32_T Constant1_Value_e;
    int32_T Constant2_Value_lk;
    int32_T Constant4_Value_l;
    int32_T Constant2_Value_l5;
    int32_T Constant4_Value_p;
    int32_T UnitDelay_InitialCondition_o;
    int32_T Constant2_Value_ly;
    int32_T Constant1_Value_n;
    int32_T Constant2_Value_f;
    int32_T Constant2_Value_lu;
    int32_T Switch_Threshold;
    int32_T DataStoreMemory_InitialValue;
    int32_T DataStoreMemory1_InitialValue;
    real32_T Constant4_Value_n;
    real32_T Constant6_Value_p;
    real32_T Constant_Value_cx;
    real32_T Constant1_Value_n0;
    real32_T Constant5_Value_n;
    real32_T Constant10_Value;
    real32_T Constant2_Value_i;
    real32_T Constant3_Value_f;
    real32_T Constant7_Value;
    real32_T Constant8_Value;
    real32_T Gain1_Gain_h;
    real32_T Constant4_Value_pq;
    real32_T Constant6_Value_e;
    real32_T Constant_Value_id;
    real32_T Constant1_Value_a;
    real32_T Constant5_Value_k;
    real32_T Constant10_Value_k;
    real32_T Constant2_Value_f0;
    real32_T Constant3_Value_b;
    real32_T Constant7_Value_j;
    real32_T Constant8_Value_f;
    real32_T Gain1_Gain_d;
    real32_T Constant_Value_ci;
    real32_T Merge_InitialOutput;
    real32_T Merge1_InitialOutput;
    real32_T Gain1_Gain_n;
    real32_T Switch1_Threshold;
    real32_T Constant_Value_n;
    real32_T Switch2_Threshold;
    real32_T Constant4_Value_o;
    real32_T Constant3_Value_i;
    real32_T Gain3_Gain;
    real32_T Gain2_Gain;
    real32_T Gain1_Gain_a;
    real32_T Switch_Threshold_f;
    real32_T Constant_Value_h;
    real32_T Gain1_Gain_c;
    real32_T Switch1_Threshold_c;
    real32_T Constant_Value_j;
    real32_T Switch2_Threshold_i;
    real32_T Gain6_Gain;
    real32_T Gain7_Gain;
    real32_T Gain4_Gain;
    real32_T Gain2_Gain_d;
    real32_T Gain1_Gain_nj;
    real32_T Gain_Gain_n;
    real32_T Constant_Value_h4;
    real32_T Constant_Value_dx;
    real32_T Constant_Value_da;
    real32_T Gain3_Gain_e;
    real32_T Constant1_Value_mu;
    real32_T Gain1_Gain_b;
    real32_T Constant_Value_nc;
    real32_T Gain1_Gain_o;
    real32_T Gain_Gain_b;
    real32_T Gain_Gain_i;
    real32_T Constant3_Value_b1;
    real32_T Gain1_Gain_k;
    real32_T Merge_InitialOutput_a;
    real32_T Constant_Value_a;
    real32_T Constant4_Value_pv;
    real32_T Constant3_Value_mn;
    real32_T Gain1_Gain_f;
    real32_T Merge_InitialOutput_k;
    real32_T Constant_Value_cp;
    real32_T Constant1_Value_e0;
    real32_T Constant4_Value_ps;
    real32_T Constant_Value_p;
    real32_T Gain4_Gain_a;
    real32_T Gain1_Gain_i;
    real32_T Gain1_Gain_hw;
    real32_T Gain2_Gain_n;
    real32_T Gain4_Gain_f;
    real32_T Gain5_Gain;
    real32_T Gain_Gain_g;
    real32_T Gain2_Gain_n3;
    real32_T Gain6_Gain_n;
    real32_T Gain1_Gain_j;
    real32_T Gain3_Gain_n;
    real32_T Gain_Gain_k;
    real32_T Gain3_Gain_a;
    real32_T Gain4_Gain_d;
    real32_T Gain5_Gain_d;
    real32_T Gain_Gain_p;
    int8_T Constant4_Value_h;
    uint8_T ManualSwitch_CurrentSetting;
    uint8_T ManualSwitch_CurrentSetting_n;
    uint8_T ManualSwitch_CurrentSetting_g;
    uint8_T ManualSwitch_CurrentSetting_m;
    uint8_T ManualSwitch_CurrentSetting_i;
    P_IfActionSubsystem_mpc_tgt_c_T IfActionSubsystem2;
    P_IfActionSubsystem_mpc_tgt_c_T IfActionSubsystem_d;
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
            int32_T arg_time_step, t_ego arg_next_ego[50], t_dynamics *arg_ego1,
            int32_T *arg_In1);
  static void terminate();
  mpc_tgt_calcModelClass();
  ~mpc_tgt_calcModelClass();
 private:
  B_mpc_tgt_calc_T mpc_tgt_calc_B;
  DW_mpc_tgt_calc_T mpc_tgt_calc_DW;
  static P_mpc_tgt_calc_T mpc_tgt_calc_P;
  static void mpc_tgt_calc_IfActionSubsystem(real32_T rty_Out1[2],
    P_IfActionSubsystem_mpc_tgt_c_T *localP);
  RT_MODEL_mpc_tgt_calc_T mpc_tgt_calc_M;
};

#endif

