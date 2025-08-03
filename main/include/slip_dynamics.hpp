#pragma once

#include "structs.hpp"

/**
 * @brief タイヤスリップ動力学計算クラス
 */
class SlipDynamics {
public:
    /**
     * @brief スリップ動力学の計算結果
     */
    struct SlipResult {
        float vx;           // X方向速度
        float vy;           // Y方向速度  
        float beta;         // スリップ角
        float velocity;     // 合成速度
        float acceleration; // 加速度
    };

    /**
     * @brief スリップ動力学の更新計算
     * @param ego 現在のエゴ状態
     * @param dynamics 車両動力学パラメータ
     * @param tgt 目標パラメータ  
     * @param angular_velocity 角速度
     * @param dt 時間刻み
     * @return スリップ動力学結果
     */
    SlipResult updateSlipDynamics(
        const t_ego& ego,
        const t_dynamics& dynamics,
        const t_tgt& tgt,
        float angular_velocity,
        float dt
    );

private:
    /**
     * @brief X方向速度の更新
     */
    float updateVx(const t_ego& ego, const t_dynamics& dynamics, float angular_velocity, float dt);
    
    /**
     * @brief Y方向速度の更新  
     */
    float updateVy(const t_ego& ego, const t_dynamics& dynamics, const t_tgt& tgt, float angular_velocity, float dt);
    
    /**
     * @brief スリップ角の更新
     */
    float updateSlipAngle(const t_ego& ego, const t_tgt& tgt, float velocity, float angular_velocity, float dt);
};