#include "angular_planner.hpp"
#include "math_utils.hpp"
#include "defines.hpp"

AngularPlanner::AngularVelocityResult AngularPlanner::calculatePivotAngularVelocity(
    const t_tgt& tgt,
    const t_ego& ego, 
    int time_step
) {
    AngularVelocityResult result;
    
    float remaining_angle = calculateRemainingAngle(tgt, ego);
    float target_alpha = 2.0f * tgt.alpha;  // 目標角加速度
    
    // 最大角速度の決定
    float w_max = (tgt.alpha > 0.0f) ? tgt.w_max : (0.5f * tgt.w_max);
    
    // 減速開始条件のチェック
    float angular_velocity_diff = ego.w * ego.w - tgt.end_w * tgt.end_w;
    float decel_distance = std::abs(angular_velocity_diff) / (2.0f * std::abs(target_alpha)) + std::abs(ego.ang);
    
    bool should_decelerate = (ego.pivot_state == 2.0f) || (decel_distance >= std::abs(tgt.tgt_angle));
    
    if (should_decelerate) {
        // 減速フェーズ
        bool should_apply_decel = (target_alpha > 0.0f) ? (ego.w < tgt.end_w) : (ego.w > tgt.end_w);
        
        if (should_apply_decel) {
            result.angular_acceleration = calculateAngularDeceleration(tgt, ego, remaining_angle);
        } else {
            result.angular_acceleration = 0.0f;
        }
        
        result.pivot_state = 2;  // 減速状態
        
    } else if (ego.pivot_state == 0.0f) {
        // 加速フェーズ
        if (std::abs(ego.w) < std::abs(w_max)) {
            result.angular_acceleration = calculateAngularAcceleration(tgt, ego, w_max);
            result.pivot_state = 1;  // 加速状態
        } else {
            result.angular_acceleration = 0.0f;
            result.pivot_state = 2;  // 等角速度状態
        }
        
    } else {
        // その他の状態
        result.angular_acceleration = 0.0f;
        result.pivot_state = 2;
    }
    
    // 角速度の更新
    result.angular_velocity = ego.w + result.angular_acceleration * DELTA_T * time_step;
    
    return result;
}

AngularPlanner::AngularVelocityResult AngularPlanner::calculateDiagonalAngularVelocity(
    const t_tgt& tgt,
    const t_ego& ego,
    int time_step
) {
    AngularVelocityResult result;
    
    float remaining_angle = calculateRemainingAngle(tgt, ego, true);  // 画像角度を使用
    float target_alpha = 2.0f * tgt.alpha;
    
    // 斜め移動用の角度閾値
    float angle_threshold1, angle_threshold2;
    if (ego.sla_param.pow_n != 0.0f) {
        angle_threshold1 = 0.6f * std::abs(tgt.tgt_angle);
        angle_threshold2 = 0.7f * std::abs(tgt.tgt_angle);
    } else {
        angle_threshold1 = 0.4f * std::abs(tgt.tgt_angle);
        angle_threshold2 = angle_threshold1;
    }
    
    // 減速開始条件（斜め移動特有の条件）
    float angular_velocity_diff = ego.w * ego.w - tgt.end_w * tgt.end_w;
    float decel_distance = std::abs(angular_velocity_diff) / (2.0f * std::abs(target_alpha)) + std::abs(ego.img_ang);
    float angle_factor = 0.9f * std::abs(tgt.tgt_angle);
    
    bool should_decelerate = (ego.pivot_state == 2.0f) || 
                            (decel_distance >= angle_factor) ||
                            (std::abs(ego.img_ang) >= angle_threshold1);
    
    if (should_decelerate) {
        // 減速フェーズ
        bool should_apply_decel = (target_alpha > 0.0f) ? (ego.w < tgt.end_w) : (ego.w > tgt.end_w);
        
        if (should_apply_decel) {
            if (target_alpha > 0.0f) {
                result.angular_acceleration = std::abs((ego.w * ego.w - tgt.end_w * tgt.end_w) / 
                    (2.0f * std::fmax(std::abs(tgt.tgt_angle) - std::abs(ego.img_ang), 0.001f)));
            } else {
                result.angular_acceleration = -std::abs((ego.w * ego.w - tgt.end_w * tgt.end_w) / 
                    (2.0f * std::fmax(std::abs(tgt.tgt_angle) - std::abs(ego.img_ang), 0.001f)));
            }
        } else {
            result.angular_acceleration = 0.0f;
        }
        
        result.pivot_state = 2;  // 減速状態
        
    } else if (ego.pivot_state == 0.0f && std::abs(ego.img_ang) < angle_threshold2) {
        // 加速フェーズ（角度が閾値内）
        result.angular_acceleration = tgt.alpha;
        result.pivot_state = 1;  // 加速状態
        
    } else {
        // その他の状態
        result.angular_acceleration = 0.0f;
        result.pivot_state = 2;
    }
    
    // 角速度の更新
    result.angular_velocity = ego.w + result.angular_acceleration * DELTA_T * time_step;
    
    return result;
}

float AngularPlanner::calculateAngularAcceleration(const t_tgt& tgt, const t_ego& ego, float w_max) {
    // 角速度に応じた加速度調整
    float velocity_ratio = ego.w / w_max;
    float degeneration_factor = 1.0f - MathUtils::safePow(velocity_ratio, 4.0f);  // 4乗で調整
    
    return tgt.alpha * degeneration_factor;
}

float AngularPlanner::calculateAngularDeceleration(const t_tgt& tgt, const t_ego& ego, float remaining_angle) {
    if (remaining_angle > 0.001f) {
        // 運動方程式に基づく角減速度計算
        float angular_decel = std::abs((ego.w * ego.w - tgt.end_w * tgt.end_w) / (2.0f * remaining_angle));
        
        // 符号の決定
        return (tgt.alpha > 0.0f) ? angular_decel : -angular_decel;
    } else {
        return 0.0f;
    }
}

float AngularPlanner::calculateRemainingAngle(const t_tgt& tgt, const t_ego& ego, bool use_img_angle) {
    float current_angle = use_img_angle ? std::abs(ego.img_ang) : std::abs(ego.ang);
    return std::fmax(std::abs(tgt.tgt_angle) - current_angle, 0.0f);
}