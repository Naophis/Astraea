#pragma once

#include "structs.hpp"
#include "enums.hpp"

/**
 * @brief 角速度計画クラス
 */
class AngularPlanner {
public:
    /**
     * @brief 旋回動作の角速度計画計算
     */
    struct AngularVelocityResult {
        float angular_velocity;
        float angular_acceleration;
        int pivot_state;
    };

    /**
     * @brief 通常旋回の角速度計画
     * @param tgt 目標パラメータ
     * @param ego 現在のエゴ状態  
     * @param time_step タイムステップ
     * @return 角速度計画結果
     */
    AngularVelocityResult calculatePivotAngularVelocity(
        const t_tgt& tgt,
        const t_ego& ego, 
        int time_step
    );

    /**
     * @brief 斜め移動の角速度計画
     * @param tgt 目標パラメータ
     * @param ego 現在のエゴ状態
     * @param time_step タイムステップ  
     * @return 角速度計画結果
     */
    AngularVelocityResult calculateDiagonalAngularVelocity(
        const t_tgt& tgt,
        const t_ego& ego,
        int time_step
    );

private:
    /**
     * @brief 加速フェーズでの角加速度計算
     */
    float calculateAngularAcceleration(const t_tgt& tgt, const t_ego& ego, float w_max);
    
    /**
     * @brief 減速フェーズでの角加速度計算
     */
    float calculateAngularDeceleration(const t_tgt& tgt, const t_ego& ego, float remaining_angle);
    
    /**
     * @brief 残り角度の計算
     */
    float calculateRemainingAngle(const t_tgt& tgt, const t_ego& ego, bool use_img_angle = false);
};