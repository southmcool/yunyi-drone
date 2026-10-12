#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "type/stabilizer_types.h"
#include <Arduino.h>

// 安全等级定义
enum SafetyLevel
{
    SAFE_NORMAL,
    SAFE_ATTENTION,
    SAFE_WARNING,
    SAFE_EMERGENCY
};

class SafetyMonitor
{
private:
    // 安全阈值
    float max_safe_angle_;
    float max_safe_angle_rate_;
    float max_safe_output_;

    // 状态
    bool is_safe_;

public:
    SafetyMonitor();

    // 核心接口
    bool checkSafety(const state_t &state, control_t &output);

    // 配置接口
    void setSafetyLimits(float max_angle, float max_angle_rate, float max_output);

    // 状态获取
    bool isSafe() const { return is_safe_; }

    // 评估当前安全等级
    SafetyLevel evaluateSafety(const state_t &state);

    // 根据安全等级削弱控制输出（不直接停机）
    void applySafetyOutput(SafetyLevel level, control_t &output);

    // 紧急停机专用函数（直接清零所有输出）
    void emergencyStop(control_t &output);

private:
    bool checkAngleSafety(float roll, float pitch);
    bool checkRateSafety(float roll_rate, float pitch_rate, float yaw_rate);
    void applySafetyLimits(control_t &output);
    float applyLimits(float value, float min, float max);
};

#endif