#include "pid_controller.h"
#include <math.h>

PIDController::PIDController()
    : kp_(0), ki_(0), kd_(0), integral_(0), last_error_(0), last_measurement(0),
      output_min_(-1.0f), output_max_(1.0f),
      integral_min_(-0.01f), integral_max_(0.01f),
      derivative_filter_alpha_(0.8f), filtered_derivative_(0.0f),
      integral_separation_threshold_(INFINITY), was_saturated_(false)
{
}

float PIDController::update(float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    float error = setpoint - measurement;
    float proportional = kp_ * error;

    // ✅ 积分分离：误差大于阈值时不积分
    float integral_term = 0.0f;
    // ★R77e 地面积分保持：每拍清零（在分离/饱和逻辑之前，最高优先级）
    if (integral_hold_)
    {
        integral_ = 0.0f;
    }
    else if (ki_ != 0.0f && fabs(error) < integral_separation_threshold_)
    {
        // 只有在未饱和时才累加积分（反饱和）
        if (!was_saturated_)
        {
            integral_ += ki_ * error * dt;
            integral_ = applyLimits(integral_, integral_min_, integral_max_);
        }
        integral_term = integral_;
    }
    else if (fabs(error) > integral_separation_threshold_ * 2.0f)
    {
        integral_ = 0.0f; // 大误差时清零积分
    }

    // 微分项
    float derivative = 0.0f;
    if (kd_ != 0.0f)
    {
        float raw_derivative = -(measurement - last_measurement) / dt;
        last_measurement = measurement;
        filtered_derivative_ = lowPassFilter(raw_derivative, filtered_derivative_, derivative_filter_alpha_);
        derivative = kd_ * filtered_derivative_;
    }

    last_error_ = error;
    float raw_output = proportional + integral_term + derivative;
    float limited_output = applyLimits(raw_output, output_min_, output_max_);
    // 更新饱和标志（用于下一周期的积分冻结）
    was_saturated_ = (raw_output != limited_output);
    return limited_output;
}

void PIDController::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setLimits(float min, float max)
{
    output_min_ = min;
    output_max_ = max;
}

void PIDController::setIntegralLimits(float min, float max)
{
    integral_min_ = min;
    integral_max_ = max;
}

void PIDController::setDerivativeFilter(float alpha)
{
    derivative_filter_alpha_ = alpha;
}

void PIDController::reset()
{
    integral_ = 0.0f;
    last_error_ = 0.0f;
    filtered_derivative_ = 0.0f;
    last_measurement = 0.0f;
    was_saturated_ = false;
}

float PIDController::applyLimits(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

float PIDController::lowPassFilter(float current, float last, float alpha)
{
    return alpha * last + (1.0f - alpha) * current;
}