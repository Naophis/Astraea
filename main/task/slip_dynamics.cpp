#include "slip_dynamics.hpp"
#include "math_utils.hpp"
#include "defines.hpp"

SlipDynamics::SlipResult SlipDynamics::updateSlipDynamics(
    const t_ego& ego,
    const t_dynamics& dynamics,
    const t_tgt& tgt,
    float angular_velocity,
    float dt
) {
    SlipResult result;
    
    // X方向速度の更新
    result.vx = updateVx(ego, dynamics, angular_velocity, dt);
    
    // Y方向速度の更新
    result.vy = updateVy(ego, dynamics, tgt, angular_velocity, dt);
    
    // 合成速度の計算
    result.velocity = MathUtils::fastSqrt(result.vx * result.vx + result.vy * result.vy);
    
    // 加速度の計算
    result.acceleration = (result.velocity - ego.v) / dt;
    
    // スリップ角の更新
    result.beta = updateSlipAngle(ego, tgt, result.velocity, angular_velocity, dt);
    
    return result;
}

float SlipDynamics::updateVx(const t_ego& ego, const t_dynamics& dynamics, float angular_velocity, float dt) {
    // X方向の運動方程式: dvx/dt = force_x/mass + w*vy
    float force_x = 0.0f;  // 外力（ここでは簡略化）
    return (force_x / dynamics.mass + angular_velocity * ego.slip.vy) * dt + ego.slip.vx;
}

float SlipDynamics::updateVy(const t_ego& ego, const t_dynamics& dynamics, const t_tgt& tgt, float angular_velocity, float dt) {
    // Y方向の運動方程式: dvy/dt = -3*K1*beta/mass - w*vx
    float slip_force = -3.0f * tgt.slip_gain_K1 * ego.slip.beta / dynamics.mass;
    return (slip_force - angular_velocity * ego.slip.vx) * dt + ego.slip.vy;
}

float SlipDynamics::updateSlipAngle(const t_ego& ego, const t_tgt& tgt, float velocity, float angular_velocity, float dt) {
    // スリップ角の更新式: dbeta/dt = (beta/dt - w) / (1/dt + K2/v)
    if (velocity > 0.001f) {  // ゼロ除算回避
        float beta_rate = ego.slip.beta / dt - angular_velocity;
        float denominator = 1.0f / dt + tgt.slip_gain_K2 / velocity;
        return beta_rate / denominator;
    } else {
        return 0.0f;
    }
}