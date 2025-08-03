#pragma once

#include "structs.hpp"
#include "enums.hpp"
#include "velocity_planner.hpp"
#include "angular_planner.hpp"
#include "slip_dynamics.hpp"
#include "feedforward_calculator.hpp"

/**
 * @brief リファクタリングされた動作計画クラス
 * 
 * 元のmpc_tgt_calcを可読性とメンテナンス性を考慮して再実装
 */
class MotionPlannerRefactored {
public:
    /**
     * @brief コンストラクタ
     */
    MotionPlannerRefactored();
    
    /**
     * @brief デストラクタ
     */
    ~MotionPlannerRefactored() = default;

    /**
     * @brief 動作計画のメイン計算
     * @param tgt 目標パラメータ
     * @param ego 現在のエゴ状態
     * @param mode 動作モード（0:直線, 1:スラローム, 2:旋回, 3:停止, 4:斜め）
     * @param time_step タイムステップ
     * @param next_ego 次のエゴ状態（出力）
     * @param dynamics 車両動力学パラメータ
     */
    void calculateMotionPlan(
        const t_tgt& tgt,
        const t_ego& ego,
        int mode,
        int time_step,
        t_ego& next_ego,
        const t_dynamics& dynamics
    );

    /**
     * @brief 初期化
     */
    void initialize();

private:
    // 各種プランナーのインスタンス
    VelocityPlanner velocity_planner_;
    AngularPlanner angular_planner_;
    SlipDynamics slip_dynamics_;
    FeedforwardCalculator feedforward_calculator_;
    
    // 内部状態
    int decel_delay_counter_;
    
    /**
     * @brief 直線移動の計算
     */
    void calculateStraightMotion(
        const t_tgt& tgt, 
        const t_ego& ego, 
        int time_step,
        t_ego& next_ego,
        const t_dynamics& dynamics
    );
    
    /**
     * @brief スラローム動作の計算
     */
    void calculateSlalomMotion(
        const t_tgt& tgt,
        const t_ego& ego, 
        int time_step,
        t_ego& next_ego,
        const t_dynamics& dynamics
    );
    
    /**
     * @brief 旋回動作の計算
     */
    void calculatePivotMotion(
        const t_tgt& tgt,
        const t_ego& ego,
        int time_step, 
        t_ego& next_ego,
        const t_dynamics& dynamics
    );
    
    /**
     * @brief 停止動作の計算
     */
    void calculateStopMotion(
        const t_ego& ego,
        int time_step,
        t_ego& next_ego,
        const t_dynamics& dynamics
    );
    
    /**
     * @brief 斜め移動の計算
     */
    void calculateDiagonalMotion(
        const t_tgt& tgt,
        const t_ego& ego,
        int time_step,
        t_ego& next_ego, 
        const t_dynamics& dynamics
    );
    
    /**
     * @brief 基本的なエゴ状態の更新
     */
    void updateBasicEgoState(
        const t_ego& ego,
        float velocity,
        float angular_velocity,
        int time_step,
        t_ego& next_ego
    );
    
    /**
     * @brief NaN/Inf の安全性チェック
     */
    void sanitizeOutputs(t_ego& next_ego, const t_ego& ego);
};