#include "safety_monitor.h"
#include <math.h>

SafetyMonitor::SafetyMonitor()
    : max_safe_angle_(0.52f), max_safe_angle_rate_(2.09f),
      max_safe_output_(1.0f), is_safe_(true)
{
}

SafetyLevel SafetyMonitor::evaluateSafety(const state_t &state)
{
    float abs_roll = fabs(state.attitude_rad.roll);
    float abs_pitch = fabs(state.attitude_rad.pitch);
    float max_angle = fmax(abs_roll, abs_pitch);
    float max_rate = fmax(fabs(state.gyro_rate.x), fmax(fabs(state.gyro_rate.y), fabs(state.gyro_rate.z)));

    // 紧急等级：估计质量极低 或 角度>50° 或 角速率>180°/s
    if (state.estimate_quality < 20 || max_angle > 0.87f || max_rate > 3.14f)
        return SAFE_EMERGENCY;
    // 警告等级：角度>40° 或 角速率>150°/s
    if (max_angle > 0.70f || max_rate > 2.62f)
        return SAFE_WARNING;
    // 注意等级：角度>30° 或 角速率>120°/s
    if (max_angle > 0.52f || max_rate > 2.09f)
        return SAFE_ATTENTION;
    return SAFE_NORMAL;
}

void SafetyMonitor::applySafetyOutput(SafetyLevel level, control_t &output)
{
    switch (level)
    {
    case SAFE_ATTENTION:
        output.roll *= 0.5f;
        output.pitch *= 0.5f;
        output.yaw *= 0.5f;
        // output.thrust_output = constrain(output.thrust_output, 0.0f, 0.7f);
        break;
    case SAFE_WARNING:
        output.roll *= 0.2f;
        output.pitch *= 0.2f;
        output.yaw *= 0.2f;
        // output.thrust_output = constrain(output.thrust_output, 0.0f, 0.5f);
        break;
    case SAFE_EMERGENCY:
        emergencyStop(output);
        break;
    default:
        break;
    }
}

void SafetyMonitor::emergencyStop(control_t &output)
{
    output.roll = 0.0f;
    output.pitch = 0.0f;
    output.yaw = 0.0f;
    output.thrust = 0.0f;
    
}

void SafetyMonitor::setSafetyLimits(float max_angle, float max_angle_rate, float max_output)
{
    max_safe_angle_ = max_angle;
    max_safe_angle_rate_ = max_angle_rate;
    max_safe_output_ = max_output;
}
bool SafetyMonitor::checkSafety(const state_t &state, control_t &output)
{
    bool angle_ok = checkAngleSafety(state.gyro_rate.x, state.gyro_rate.y);
    bool rate_ok = checkRateSafety(state.gyro_rate.x, state.gyro_rate.y, state.gyro_rate.z);

    if (!angle_ok || !rate_ok)
    {
        static uint32_t last_stop_time = 0;
        uint32_t now = millis();
        // 角度或角速度超出安全阈值 1.5 倍以上 -> 立即停机
        if (fabs(state.gyro_rate.x) > max_safe_angle_ * 1.5f ||
            fabs(state.gyro_rate.y) > max_safe_angle_ * 1.5f ||
            fabs(state.gyro_rate.x) > max_safe_angle_rate_ * 1.5f ||
            fabs(state.gyro_rate.y) > max_safe_angle_rate_ * 1.5f)
        {
            // 强制油门置零
            output.thrust = 0.0f;
            Serial.println("SAFETY: EMERGENCY STOP (severe limit exceeded)");
            return false;
        }
        // 一般超限：降低输出
        applySafetyLimits(output);
        is_safe_ = false; // 添加这一行
        return false;
    }
    is_safe_ = true;
    return true;
}

bool SafetyMonitor::checkAngleSafety(float roll, float pitch)
{
    return (fabs(roll) <= max_safe_angle_ && fabs(pitch) <= max_safe_angle_);
}

bool SafetyMonitor::checkRateSafety(float roll_rate, float pitch_rate, float yaw_rate)
{
    return (fabs(roll_rate) <= max_safe_angle_rate_ &&
            fabs(pitch_rate) <= max_safe_angle_rate_ &&
            fabs(yaw_rate) <= max_safe_angle_rate_);
}

void SafetyMonitor::applySafetyLimits(control_t &output)
{
    // 角度或角速度超限时，大幅降低控制输出
    float safety_factor = 0.3f;
    output.roll *= safety_factor;
    output.pitch *= safety_factor;
    output.yaw *= safety_factor;

    // 确保输出在安全范围内
    output.roll = constrain(output.roll, -max_safe_output_, max_safe_output_);
    output.pitch = constrain(output.pitch, -max_safe_output_, max_safe_output_);
    output.yaw = constrain(output.yaw, -max_safe_output_, max_safe_output_);
    output.thrust = constrain(output.thrust, 0.0f, 1.0f);
}

float SafetyMonitor::applyLimits(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}