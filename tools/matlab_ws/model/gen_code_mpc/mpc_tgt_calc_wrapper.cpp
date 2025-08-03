#include "mpc_tgt_calc.h"
#include "motion_planner_refactored.hpp"

// グローバルなリファクタリング済みプランナーインスタンス
static MotionPlannerRefactored g_motion_planner;

void mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics
  *arg_ego1)
{
    // 新しいリファクタリング済み実装を呼び出し
    g_motion_planner.calculateMotionPlan(
        *arg_tgt,           // 目標パラメータ
        *arg_ego,           // 現在のエゴ状態
        arg_mode,           // 動作モード
        arg_time_step,      // タイムステップ
        *arg_next_ego,      // 次のエゴ状態（出力）
        *arg_ego1           // 車両動力学パラメータ
    );
}

void mpc_tgt_calcModelClass::initialize()
{
    // 新しい実装の初期化
    g_motion_planner.initialize();
    
    // 既存の初期化コード（互換性のため）
    mpc_tgt_calc_DW.UnitDelay_DSTATE = mpc_tgt_calc_P.UnitDelay_InitialCondition;
    mpc_tgt_calc_B.Merge_p[0] = mpc_tgt_calc_P.Merge_InitialOutput;
    mpc_tgt_calc_B.Merge1[0] = mpc_tgt_calc_P.Merge1_InitialOutput;
    mpc_tgt_calc_B.Merge_p[1] = mpc_tgt_calc_P.Merge_InitialOutput;
    mpc_tgt_calc_B.Merge1[1] = mpc_tgt_calc_P.Merge1_InitialOutput;
}

void mpc_tgt_calcModelClass::terminate()
{
    // 終了処理（必要に応じて追加）
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

// ヘルパー関数（IfActionSubsystem）
void IRAM_ATTR mpc_tgt_calcModelClass::mpc_tgt_calc_IfActionSubsystem(real32_T rty_Out1[2],
  P_IfActionSubsystem_mpc_tgt_c_T *localP)
{
  rty_Out1[0] = localP->Constant1_Value;
  rty_Out1[1] = localP->Constant2_Value;
}