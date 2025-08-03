#pragma once

#include "structs.hpp"
#include "enums.hpp"

/**
 * @brief マイクロマウス速度計画クラス
 * 
 * 生成されたMPCコードを可読性の高いモジュールに分離
 */
class VelocityPlanner {
public:
    /**
     * @brief 速度計画のモード
     */
    enum class MotionMode {
        STRAIGHT = 0,   // 直線移動
        SLALOM = 1,     // スラローム
        PIVOT = 2,      // 旋回
        STOP = 3,       // 停止
        DIAGONAL = 4    // 斜め移動
    };

    /**
     * @brief 直線移動の速度計画計算
     * @param tgt 目標パラメータ
     * @param ego 現在のエゴ状態
     * @param time_step タイムステップ
     * @return 次の速度と加速度
     */
    struct LinearVelocityResult {
        float velocity;
        float acceleration;
        int state;
        int decel_delay_count;
    };
    
    LinearVelocityResult calculateLinearVelocity(
        const t_tgt& tgt, 
        const t_ego& ego, 
        int time_step
    );

private:
    int decel_delay_counter = 0;

    /**
     * @brief 加速フェーズの計算
     */
    float calculateAcceleration(const t_tgt& tgt, const t_ego& ego);
    
    /**
     * @brief 減速フェーズの計算  
     */
    float calculateDeceleration(const t_tgt& tgt, const t_ego& ego, int time_step);
    
    /**
     * @brief 減速遅延の適用
     */
    float applyDecelerationDelay(
        float base_decel, 
        const t_tgt& tgt, 
        const t_ego& ego, 
        int delay_count
    );
};