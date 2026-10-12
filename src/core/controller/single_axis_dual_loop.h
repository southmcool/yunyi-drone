#ifndef SINGLE_AXIS_DUAL_LOOP_H
#define SINGLE_AXIS_DUAL_LOOP_H

// 单轴双环控制器
#include "pid_controller.h"

class SingleAxisDualLoop
{
private:
    // 限制参数 (全部使用弧度单位)
    float max_angle_rad_;        // 最大角度 (rad)
    float max_angle_rate_rad_s_; // 最大角速度 (rad/s) - 注意命名一致性
    float max_output_;           // 最大控制输出

    // 状态
    float outer_output_;
    float inner_output_;

    float filtered_rate_measurement_;     // 滤波后的角速度测量值
    float rate_measurement_filter_alpha_; // 滤波系数，可在构造函数中设置默认值 0.7

public:
    PIDController outer_loop_; // 角度环
    PIDController inner_loop_; // 角速度环
public:
    SingleAxisDualLoop();

    void setRateFilterAlpha(float alpha) { rate_measurement_filter_alpha_ = alpha; }

    // 核心接口
    float update(float angle_setpoint, float angle_measurement,
                 float rate_measurement, float dt);
    // 新增：接受外部计算好的无奇点角度误差和机体系目标角速度
    float updateSafe(float angle_error_rad, float body_rate_setpoint_rad_s,
                     float rate_measurement, float dt);
    // 配置接口
    void setOuterGains(float kp, float ki, float kd);
    void setInnerGains(float kp, float ki, float kd);
    void setLimits(float max_angle, float max_angle_rate, float max_output);
    // ★R77e：地面积分保持透传（内外环同时生效）
    void setIntegralHold(bool hold)
    {
        outer_loop_.setIntegralHold(hold);
        inner_loop_.setIntegralHold(hold);
    }

    // 状态管理
    void reset();

    // 调试接口
    float getOuterOutput() const { return outer_output_; }
    float getInnerOutput() const { return inner_output_; }
    float getAngleError() const { return outer_loop_.getLastError(); }
    float getRateError() const { return inner_loop_.getLastError(); }
    float applyLimits(float value, float min, float max);
};

#endif