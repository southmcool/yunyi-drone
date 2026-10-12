#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "type/stabilizer_types.h"
#include <Arduino.h>

class PIDController
{
private:
    // PID参数
    float kp_, ki_, kd_;

    // 状态变量
    float integral_;
    float last_error_;
    float output_min_, output_max_;
    float integral_min_, integral_max_;

    // 滤波器
    float derivative_filter_alpha_;
    float filtered_derivative_;

    float last_measurement;

    float integral_separation_threshold_; // 新增：积分分离阈值

    // 新增：饱和标志（用于反饱和逻辑）
    bool was_saturated_;

    // ★R77e 地面积分保持：置位时每拍清零积分（P/D 照常），防地面静止误差
    //   造成差动慢速积累（flight7 离地翻扣根因）
    bool integral_hold_ = false;

public:
    PIDController();
    // ✅ 添加 const 限定符
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
    // 核心接口
    float update(float setpoint, float measurement, float dt);

    // 配置接口
    void setGains(float kp, float ki, float kd);
    void setLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDerivativeFilter(float alpha);
    void setIntegralSeparation(float threshold) { integral_separation_threshold_ = threshold; }
    // ★R77e：地面积分保持开关（true=每拍清零积分，P/D 不受影响）
    void setIntegralHold(bool hold)
    {
        integral_hold_ = hold;
        if (hold)
            integral_ = 0.0f;
    }
    // 状态管理
    void reset();

    // 状态获取
    float getLastError() const { return last_error_; }
    float getIntegral() const { return integral_; }
    float applyLimits(float value, float min, float max);

private:
    float lowPassFilter(float current, float last, float alpha);
};

#endif