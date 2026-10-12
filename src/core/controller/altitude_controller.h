// 高度控制
#ifndef ALTITUDE_CONTROLLER_H
#define ALTITUDE_CONTROLLER_H

#include "type/stabilizer_types.h"
#include "pid_controller.h"

class AltitudeController
{
public:
    AltitudeController();

    // 核心更新函数
    // 输入: 目标高度, 当前状态, 时间步长
    // 输出: 油门修正量 (范围 -1.0 ~ 1.0)
    float update(float target_altitude, const state_t &state, float dt);

    // 配置接口
    void setGains(const float pos_gains[3], const float vel_gains[3]);
    void setLimits(float max_velocity, float max_correction);
    void reset();
    // ★R77k：推力修正斜率限幅（单位 1/s），防止贴地大误差时位置/速度环双重
    //   饱和导致 th_out 由 0.069 瞬间跳到 0.575（flight(2) 实测），进而弹起-砸下
    //   bang-bang 并带歪姿态。设为 0 表示不限幅。
    void setSlewRate(float max_correction_per_s) { _max_correction_rate = max_correction_per_s; }
    // 串级 PID 控制器
    PIDController _pos_controller; // 位置环 (P)
    PIDController _vel_controller; // 速度环 (PID)

private:
    // 限制参数
    float _max_velocity;
    float _max_correction;
    float _max_correction_rate = 2.0f; // 每秒钟推力修正最大变化量（默认 2.0/s）

    // 状态
    float _target_velocity;
    float _correction_output;
};
// 定义全局电机对象
extern AltitudeController altitudeController;
#endif