#include "feedforward_calculator.hpp"
#include "math_utils.hpp"
#include "defines.hpp"

FeedforwardCalculator::FeedforwardResult FeedforwardCalculator::calculateFeedforward(
    const t_ego& ego,
    const t_dynamics& dynamics,
    int mode
) {
    FeedforwardResult result;
    
    // 基本的なフィードフォワード計算
    result.front_duty = calculateFrontDuty(ego, dynamics, mode);
    result.roll_duty = calculateRollDuty(ego, dynamics);
    
    // モーターRPMデューティの計算
    auto motor_rpm = calculateMotorRpmDuty(ego, dynamics);
    result.left_rpm_duty = motor_rpm.left;
    result.right_rpm_duty = motor_rpm.right;
    
    // 左右モーターデューティの計算
    result.left_duty = result.front_duty - result.roll_duty + result.left_rpm_duty;
    result.right_duty = result.front_duty + result.roll_duty + result.right_rpm_duty;
    
    // トルクの計算
    float gear_ratio_km = dynamics.gear_ratio * dynamics.km;
    result.front_torque = 0.25f * ego.accl * dynamics.mass * (0.5f * dynamics.tire);
    result.roll_torque = ego.alpha2 * dynamics.lm * dynamics.resist;
    
    // 摩擦トルクの計算
    auto friction = calculateFrictionTorque(ego, dynamics);
    result.left_friction_torque = friction.left;
    result.right_friction_torque = friction.right;
    
    return result;
}

float FeedforwardCalculator::calculateFrontDuty(const t_ego& ego, const t_dynamics& dynamics, int mode) {
    float gear_ratio_km = dynamics.gear_ratio * dynamics.km;
    float base_torque = 0.25f * ego.accl * dynamics.mass * (0.5f * dynamics.tire);
    float base_duty = base_torque * dynamics.resist / gear_ratio_km;
    
    // モードに応じた調整
    if ((mode == 0 || mode == 3) && ego.state == 0) {  // 直線移動または停止で加速状態
        if (ego.v > ego.ff_duty_low_v_th) {
            return base_duty;
        } else {
            return MathUtils::clamp(base_duty, ego.ff_duty_low_th, base_duty);
        }
    } else {
        return base_duty;
    }
}

float FeedforwardCalculator::calculateRollDuty(const t_ego& ego, const t_dynamics& dynamics) {
    float gear_ratio_km = dynamics.gear_ratio * dynamics.km;
    float roll_torque = ego.alpha2 * dynamics.lm;
    float tread_factor = 0.5f * dynamics.tire * dynamics.resist / gear_ratio_km / (2.0f * dynamics.tread);
    
    return roll_torque * tread_factor;
}

FeedforwardCalculator::MotorRpmDuty FeedforwardCalculator::calculateMotorRpmDuty(const t_ego& ego, const t_dynamics& dynamics) {
    MotorRpmDuty result;
    
    float linear_velocity = 0.25f * ego.v;
    float angular_component = 0.2f * dynamics.tread * 0.5f * ego.w;
    float tire_factor = 0.166f * dynamics.tire * 1.0f;  // 簡略化
    
    float left_velocity = linear_velocity - angular_component;
    float right_velocity = linear_velocity + angular_component;
    
    result.left = left_velocity * dynamics.ke * 2.0f / tire_factor;
    result.right = right_velocity * dynamics.ke * 3.0f / tire_factor;
    
    return result;
}

FeedforwardCalculator::FrictionTorque FeedforwardCalculator::calculateFrictionTorque(const t_ego& ego, const t_dynamics& dynamics) {
    FrictionTorque result;
    
    float linear_velocity = 0.25f * ego.v;
    float angular_component = 0.2f * dynamics.tread * 0.5f * ego.w;
    
    float left_velocity = linear_velocity - angular_component;
    float right_velocity = linear_velocity + angular_component;
    
    // 符号関数の適用
    float left_sign = getSign(left_velocity);
    float right_sign = getSign(right_velocity);
    
    // 摩擦トルクの計算：クーロン摩擦 + 粘性摩擦
    result.left = left_sign * dynamics.coulomb_friction + left_velocity * dynamics.viscous_friction;
    result.right = right_sign * dynamics.coulomb_friction + right_velocity * dynamics.viscous_friction;
    
    return result;
}

float FeedforwardCalculator::getSign(float value) {
    if (MathUtils::isNanOrInf(value)) {
        return 0.0f;  // NaN
    } else if (value < 0.0f) {
        return -1.0f;
    } else {
        return (value > 0.0f) ? 1.0f : 0.0f;
    }
}