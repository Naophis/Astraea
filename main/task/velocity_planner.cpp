#include "velocity_planner.hpp"
#include "math_utils.hpp"
#include "defines.hpp"

VelocityPlanner::LinearVelocityResult VelocityPlanner::calculateLinearVelocity(
    const t_tgt& tgt, 
    const t_ego& ego, 
    int time_step
) {
    LinearVelocityResult result;
    
    float remaining_dist = tgt.tgt_dist - ego.dist;
    
    // 減速開始条件のチェック
    bool should_decelerate = (ego.v - tgt.end_v > 0.1f) && 
        (ego.state == 1.0f || 
         (std::abs(ego.v * ego.v - tgt.end_v * tgt.end_v) / (2.0f * std::abs(tgt.decel)) + ego.dist >= tgt.tgt_dist));
    
    if (should_decelerate) {
        // 減速フェーズ
        result.acceleration = calculateDeceleration(tgt, ego, time_step);
        decel_delay_counter += 1;
        
        // 減速遅延の適用
        if (ego.v < tgt.accl_param.limit) {
            // 通常の減速
        } else {
            // 減速遅延の適用
            result.acceleration = applyDecelerationDelay(result.acceleration, tgt, ego, decel_delay_counter);
        }
        
        // 目標速度を超えないようにクランプ
        float next_velocity = ego.v + result.acceleration * DELTA_T * time_step;
        if (next_velocity < tgt.end_v) {
            result.acceleration = (tgt.end_v - ego.v) / (DELTA_T * time_step);
        }
        
        result.state = 2;  // 減速状態
        result.decel_delay_count = MathUtils::clamp(decel_delay_counter, 0, tgt.accl_param.decel_delay_cnt);
        
    } else if (ego.state == 0.0f) {  // 加速フェーズ
        if (ego.v < tgt.v_max) {
            result.acceleration = calculateAcceleration(tgt, ego);
            result.state = 1;  // 加速状態
        } else {
            result.acceleration = 0.0f;
            result.state = 2;  // 等速状態  
        }
        result.decel_delay_count = 0;
        decel_delay_counter = 0;
        
    } else {
        // その他の状態（停止など）
        result.acceleration = 0.0f;
        result.state = 2;
        result.decel_delay_count = 0;
    }
    
    // 速度の計算と制限
    result.velocity = MathUtils::clamp(
        ego.v + result.acceleration * DELTA_T * time_step,
        0.0f,
        tgt.v_max
    );
    
    return result;
}

float VelocityPlanner::calculateAcceleration(const t_tgt& tgt, const t_ego& ego) {
    if (ego.v > tgt.accl_param.limit) {
        // 高速時の加速度調整（速度に応じて減衰）
        float velocity_ratio = ego.v / tgt.v_max;
        float degeneration_factor = 1.0f - MathUtils::safePow(velocity_ratio, tgt.accl_param.n);
        return tgt.accl * tgt.axel_degenerate_gain * degeneration_factor;
    } else {
        // 低速時の最大加速度
        return tgt.accl * tgt.axel_degenerate_gain;
    }
}

float VelocityPlanner::calculateDeceleration(const t_tgt& tgt, const t_ego& ego, int time_step) {
    float remaining_dist = tgt.tgt_dist - ego.dist;
    
    if (ego.v > tgt.end_v) {
        float distance_factor;
        if (remaining_dist > 0.001f) {  // 十分な距離がある場合
            distance_factor = 2.0f * remaining_dist;
        } else {
            distance_factor = 0.001f;  // 最小値でクランプ
        }
        
        // 運動方程式に基づく減速度計算: a = (v_end² - v_current²) / (2 * distance)
        float decel_magnitude = std::abs((ego.v * ego.v - tgt.end_v * tgt.end_v) / distance_factor);
        return -decel_magnitude;  // 減速なので負の値
    } else {
        return 0.0f;
    }
}

float VelocityPlanner::applyDecelerationDelay(
    float base_decel, 
    const t_tgt& tgt, 
    const t_ego& ego, 
    int delay_count
) {
    float delay_ratio = MathUtils::clamp(
        static_cast<float>(delay_count) / static_cast<float>(tgt.accl_param.decel_delay_cnt),
        0.0f,
        1.0f
    );
    
    // 減速遅延による減速度の調整（徐々に減速度を増加）
    float delay_factor = 1.0f - MathUtils::safePow(1.0f - delay_ratio, tgt.accl_param.decel_delay_n);
    
    return base_decel * delay_factor;
}