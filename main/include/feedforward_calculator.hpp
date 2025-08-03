#pragma once

#include "structs.hpp"

/**
 * @brief フィードフォワード制御計算クラス
 */
class FeedforwardCalculator {
public:
    /**
     * @brief フィードフォワード制御の計算結果
     */
    struct FeedforwardResult {
        float front_duty;           // 前進デューティ
        float left_duty;            // 左モーターデューティ
        float right_duty;           // 右モーターデューティ  
        float roll_duty;            // ロールデューティ
        float left_rpm_duty;        // 左RPMデューティ
        float right_rpm_duty;       // 右RPMデューティ
        float front_torque;         // 前進トルク
        float roll_torque;          // ロールトルク
        float left_friction_torque; // 左摩擦トルク
        float right_friction_torque;// 右摩擦トルク
    };

    /**
     * @brief フィードフォワード制御の計算
     * @param ego エゴ状態
     * @param dynamics 車両動力学パラメータ
     * @param mode 動作モード
     * @return フィードフォワード制御結果
     */
    FeedforwardResult calculateFeedforward(
        const t_ego& ego,
        const t_dynamics& dynamics,
        int mode
    );

private:
    /**
     * @brief 前進デューティの計算
     */
    float calculateFrontDuty(const t_ego& ego, const t_dynamics& dynamics, int mode);
    
    /**
     * @brief ロールデューティの計算
     */
    float calculateRollDuty(const t_ego& ego, const t_dynamics& dynamics);
    
    /**
     * @brief モーターRPMデューティの計算
     */
    struct MotorRpmDuty {
        float left;
        float right;
    };
    MotorRpmDuty calculateMotorRpmDuty(const t_ego& ego, const t_dynamics& dynamics);
    
    /**
     * @brief 摩擦トルクの計算
     */
    struct FrictionTorque {
        float left;
        float right;
    };
    FrictionTorque calculateFrictionTorque(const t_ego& ego, const t_dynamics& dynamics);
    
    /**
     * @brief 速度の符号を取得
     */
    float getSign(float value);
};