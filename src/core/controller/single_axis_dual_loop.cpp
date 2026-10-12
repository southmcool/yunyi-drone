#include "single_axis_dual_loop.h"
#include <math.h>

SingleAxisDualLoop::SingleAxisDualLoop()
    : max_angle_rad_(0.5f), max_angle_rate_rad_s_(3.0f), max_output_(1.0f),
      outer_output_(0.0f), inner_output_(0.0f), filtered_rate_measurement_(0.0f), rate_measurement_filter_alpha_(0.0f)
{
}

float SingleAxisDualLoop::update(float angle_setpoint, float angle_measurement,
                                 float rate_measurement, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    // 对测量角速度进行低通滤波（抑制高频振动噪声）
    filtered_rate_measurement_ = rate_measurement_filter_alpha_ * filtered_rate_measurement_ + (1.0f - rate_measurement_filter_alpha_) * rate_measurement;

    // 外环：角度控制 → 目标角速度
    outer_output_ = outer_loop_.update(angle_setpoint, angle_measurement, dt);
    outer_output_ = applyLimits(outer_output_, -max_angle_rate_rad_s_, max_angle_rate_rad_s_);

    // ✅ 内环：只调用一次
    float raw_output = inner_loop_.update(outer_output_, filtered_rate_measurement_, dt);
    float limited_output = applyLimits(raw_output, -max_output_, max_output_);

    return limited_output;
}

float SingleAxisDualLoop::updateSafe(float angle_error_rad, float body_rate_feedforward,
                                     float rate_measurement, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    // 角速度低通滤波
    filtered_rate_measurement_ = rate_measurement_filter_alpha_ * filtered_rate_measurement_ + (1.0f - rate_measurement_filter_alpha_) * rate_measurement;

    // ✅ 外环：基于无奇点的几何误差计算目标角速度
    // measurement=0 因为 angle_error_rad 已经是精确差值
    outer_output_ = outer_loop_.update(angle_error_rad, 0.0f, dt);
    outer_output_ = applyLimits(outer_output_, -max_angle_rate_rad_s_, max_angle_rate_rad_s_);

    // ✅ 内环：外环输出即为机体系目标角速度
    // body_rate_feedforward 可用于前馈补偿，默认为0时退化为纯串级PID
    float final_rate_sp = outer_output_ + body_rate_feedforward;

    float raw_output = inner_loop_.update(final_rate_sp, filtered_rate_measurement_, dt);
    float limited_output = applyLimits(raw_output, -max_output_, max_output_);

    inner_output_ = limited_output;
    return limited_output;
}

void SingleAxisDualLoop::setOuterGains(float kp, float ki, float kd)
{
    outer_loop_.setGains(kp, ki, kd);
}

void SingleAxisDualLoop::setInnerGains(float kp, float ki, float kd)
{
    inner_loop_.setGains(kp, ki, kd);
}

void SingleAxisDualLoop::setLimits(float max_angle, float max_angle_rate, float max_output)
{
    max_angle_rad_ = max_angle;
    max_angle_rate_rad_s_ = max_angle_rate;
    max_output_ = max_output;
}

void SingleAxisDualLoop::reset()
{
    outer_loop_.reset();
    inner_loop_.reset();
    outer_output_ = 0.0f;
    inner_output_ = 0.0f;
}

float SingleAxisDualLoop::applyLimits(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}