#include "motion_planner_refactored.hpp"
#include "math_utils.hpp"
#include "defines.hpp"

MotionPlannerRefactored::MotionPlannerRefactored() 
    : decel_delay_counter_(0) {
}

void MotionPlannerRefactored::initialize() {
    decel_delay_counter_ = 0;
}

void MotionPlannerRefactored::calculateMotionPlan(
    const t_tgt& tgt,
    const t_ego& ego,
    int mode,
    int time_step,
    t_ego& next_ego,
    const t_dynamics& dynamics
) {
    // 基本的なエゴ状態のコピー
    next_ego = ego;
    
    switch (mode) {
        case 0:  // 直線移動
            calculateStraightMotion(tgt, ego, time_step, next_ego, dynamics);
            break;
            
        case 1:  // スラローム
            calculateSlalomMotion(tgt, ego, time_step, next_ego, dynamics);
            break;
            
        case 2:  // 旋回
            calculatePivotMotion(tgt, ego, time_step, next_ego, dynamics);
            break;
            
        case 3:  // 停止
            calculateStopMotion(ego, time_step, next_ego, dynamics);
            break;
            
        case 4:  // 斜め移動
            calculateDiagonalMotion(tgt, ego, time_step, next_ego, dynamics);
            break;
            
        default:
            // 不明なモード - 停止処理
            calculateStopMotion(ego, time_step, next_ego, dynamics);
            break;
    }
    
    // スリップ動力学の計算（有効な場合）
    if (tgt.enable_slip_decel != 0) {
        auto slip_result = slip_dynamics_.updateSlipDynamics(ego, dynamics, tgt, next_ego.w, DELTA_T);
        
        next_ego.slip.vx = slip_result.vx;
        next_ego.slip.vy = slip_result.vy;
        next_ego.slip.v = slip_result.velocity;
        next_ego.slip.beta = slip_result.beta;
        next_ego.slip.accl = slip_result.acceleration;
        next_ego.v = slip_result.velocity;
        next_ego.accl = slip_result.acceleration;
    }
    
    // フィードフォワード制御の計算
    auto ff_result = feedforward_calculator_.calculateFeedforward(next_ego, dynamics, mode);
    
    next_ego.ff_duty_front = ff_result.front_duty;
    next_ego.ff_duty_l = ff_result.left_duty;
    next_ego.ff_duty_r = ff_result.right_duty;
    next_ego.ff_duty_roll = ff_result.roll_duty;
    next_ego.ff_duty_rpm_l = ff_result.left_rpm_duty;
    next_ego.ff_duty_rpm_r = ff_result.right_rpm_duty;
    next_ego.ff_front_torque = ff_result.front_torque;
    next_ego.ff_roll_torque = ff_result.roll_torque;
    next_ego.ff_friction_torque_l = ff_result.left_friction_torque;
    next_ego.ff_friction_torque_r = ff_result.right_friction_torque;
    
    // 安全性チェック
    sanitizeOutputs(next_ego, ego);
}

void MotionPlannerRefactored::calculateStraightMotion(
    const t_tgt& tgt, 
    const t_ego& ego, 
    int time_step,
    t_ego& next_ego,
    const t_dynamics& dynamics
) {
    // 直線移動の速度計画
    auto velocity_result = velocity_planner_.calculateLinearVelocity(tgt, ego, time_step);
    
    next_ego.v = velocity_result.velocity;
    next_ego.accl = velocity_result.acceleration;
    next_ego.state = velocity_result.state;
    
    // 角速度は0に設定
    next_ego.w = 0.0f;
    next_ego.alpha = 0.0f;
    next_ego.alpha2 = 0.0f;
    
    // 位置の更新
    updateBasicEgoState(ego, velocity_result.velocity, 0.0f, time_step, next_ego);
}

void MotionPlannerRefactored::calculateSlalomMotion(
    const t_tgt& tgt,
    const t_ego& ego, 
    int time_step,
    t_ego& next_ego,
    const t_dynamics& dynamics
) {
    // スラローム動作の計算（簡略化）
    // 実際のスラローム計算は複雑な数式を含むため、ここでは基本的な処理のみ
    
    int counter = time_step + ego.sla_param.counter;
    next_ego.sla_param.counter = counter;
    
    if (counter > ego.sla_param.limit_time_count) {
        // スラローム終了
        next_ego.w = 0.0f;
        next_ego.alpha = 0.0f;
        next_ego.alpha2 = 0.0f;
    } else {
        // スラローム実行中
        float time_ratio = static_cast<float>(counter) * DELTA_T / ego.sla_param.base_time;
        
        // 簡略化されたスラローム計算
        next_ego.alpha = ego.sla_param.base_alpha * 0.5f;  // 簡略化
        next_ego.w = ego.w + next_ego.alpha * DELTA_T * time_step;
        next_ego.alpha2 = next_ego.alpha;
    }
    
    // 速度の計算
    auto velocity_result = velocity_planner_.calculateLinearVelocity(tgt, ego, time_step);
    next_ego.v = velocity_result.velocity;
    next_ego.accl = velocity_result.acceleration;
    
    // 位置の更新
    updateBasicEgoState(ego, next_ego.v, next_ego.w, time_step, next_ego);
}

void MotionPlannerRefactored::calculatePivotMotion(
    const t_tgt& tgt,
    const t_ego& ego,
    int time_step, 
    t_ego& next_ego,
    const t_dynamics& dynamics
) {
    // 旋回動作の角速度計画
    auto angular_result = angular_planner_.calculatePivotAngularVelocity(tgt, ego, time_step);
    
    next_ego.w = angular_result.angular_velocity;
    next_ego.alpha = angular_result.angular_acceleration;
    next_ego.alpha2 = angular_result.angular_acceleration;
    next_ego.pivot_state = angular_result.pivot_state;
    
    // 直線速度は0に設定
    next_ego.v = 0.0f;
    next_ego.accl = 0.0f;
    
    // 角度が目標に達していない場合のみ更新
    if (std::abs(ego.ang) < std::abs(tgt.tgt_angle)) {
        updateBasicEgoState(ego, 0.0f, next_ego.w, time_step, next_ego);
    } else {
        // 目標角度に達した場合は停止
        next_ego.w = 0.0f;
        next_ego.alpha = 0.0f;
        next_ego.alpha2 = 0.0f;
        next_ego.pivot_state = 1;  // 完了状態
    }
}

void MotionPlannerRefactored::calculateStopMotion(
    const t_ego& ego,
    int time_step,
    t_ego& next_ego,
    const t_dynamics& dynamics
) {
    // 停止処理 - 0に向かって減速
    float decel_rate = (0.0f - ego.w) / (DELTA_T * time_step);
    
    next_ego.w = 0.0f;
    next_ego.alpha = decel_rate;
    next_ego.alpha2 = decel_rate;
    next_ego.v = ego.v;  // 速度は維持
    next_ego.accl = 0.0f;
    
    // 位置は更新しない（停止）
}

void MotionPlannerRefactored::calculateDiagonalMotion(
    const t_tgt& tgt,
    const t_ego& ego,
    int time_step,
    t_ego& next_ego, 
    const t_dynamics& dynamics
) {
    // 斜め移動の角速度計画
    auto angular_result = angular_planner_.calculateDiagonalAngularVelocity(tgt, ego, time_step);
    
    next_ego.w = angular_result.angular_velocity;
    next_ego.alpha = angular_result.angular_acceleration;
    next_ego.alpha2 = 0.0f;  // 斜め移動では alpha2 は別計算
    next_ego.pivot_state = angular_result.pivot_state;
    
    // 直線速度は0に設定
    next_ego.v = 0.0f;
    next_ego.accl = 0.0f;
    
    // 角度が目標に達していない場合のみ更新
    if (std::abs(ego.img_ang) < std::abs(tgt.tgt_angle)) {
        updateBasicEgoState(ego, 0.0f, next_ego.w, time_step, next_ego);
    } else {
        // 目標角度に達した場合は停止
        next_ego.w = 0.0f;
        next_ego.alpha = 0.0f;
        next_ego.pivot_state = 1;  // 完了状態
    }
}

void MotionPlannerRefactored::updateBasicEgoState(
    const t_ego& ego,
    float velocity,
    float angular_velocity,
    int time_step,
    t_ego& next_ego
) {
    float dt_step = DELTA_T * time_step;
    
    // 位置の更新
    float distance_delta = velocity * dt_step;
    float angle_delta = angular_velocity * dt_step;
    
    next_ego.dist = ego.dist + distance_delta;
    next_ego.ang = ego.ang + angle_delta;
    next_ego.img_dist = ego.img_dist + distance_delta;
    next_ego.img_ang = ego.img_ang + angle_delta;
    
    // その他の状態をコピー（変更されない値）
    next_ego.v_r = ego.v_r;
    next_ego.v_l = ego.v_l;
    next_ego.pos_x = ego.pos_x;
    next_ego.pos_y = ego.pos_y;
}

void MotionPlannerRefactored::sanitizeOutputs(t_ego& next_ego, const t_ego& ego) {
    // 加速度のNaN/Infチェック
    if (MathUtils::isNanOrInf(next_ego.accl)) {
        next_ego.accl = 0.0f;
        next_ego.v = ego.v;
        next_ego.img_dist = ego.img_dist;
    }
    
    // 角加速度のNaN/Infチェック
    if (MathUtils::isNanOrInf(next_ego.alpha)) {
        next_ego.alpha = 0.0f;
        next_ego.w = ego.w;
        next_ego.img_ang = ego.img_ang;
    }
    
    // alpha2のNaN/Infチェック
    if (MathUtils::isNanOrInf(next_ego.alpha2)) {
        next_ego.alpha2 = 0.0f;
    }
}