/**
 * @file position_controller.h
 * @brief 水平位置串级控制器（地球坐标系解算）
 *
 * 控制架构:
 * [目标位置] -> (P) -> [目标速度] -> (PID) -> [目标加速度] -> (逆动力学) -> [目标Roll/Pitch]
 */

#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include "pid_controller.h"
#include "safety_monitor.h"
#include <cmath>

// ===== 调试信息 =====
struct position_debug_info_t
{
    float pos_error[2];    // X/Y 位置误差 (m)
    float vel_error[2];    // X/Y 速度误差 (m/s)
    float target_vel[2];   // X/Y 目标速度 (m/s)
    float target_accel[2]; // X/Y 目标加速度 (m/s^2)
    float target_roll;     // 输出目标横滚 (rad)
    float target_pitch;    // 输出目标俯仰 (rad)
};

// ===== 位置环增益配置 =====
// ★★ 2026-10-03：pos_p 1.8→0.9、vel_p 3.2→1.8（SIL 闭环定标，n=3 复现）★★
//   旧值 (1.8, 3.2) 的闭环 ωn = √(pos_p·vel_p) = 0.382 Hz，而它所串接的姿态环
//   闭环带宽实测只有 ≈0.29 Hz（指令俯仰→实测俯仰在 0.416Hz 处相位滞后 55.3°，
//   等效一阶 τ≈0.55s）。串级要求内环 ≥5× 外环，此处只有 0.75× ⇒ 环路在 ωn
//   处相位余度 ≈0 ⇒ 任何扰动都放大成**自持极限环**：
//     hold_novib 实测 gt_E 包络 10–38s 恒 1.13~1.32m 不收敛、俯仰 ±22.5°、
//     加速度指令 46.5% 时间撞 20° 限幅（需求峰值 8.35 vs 限幅 3.57 m/s²）。
//   佐证（离线重放控制器，逐级换真值）：把 pos/vel 反馈换成真值后撞限幅率
//     46.6%→48.7% **不降** ⇒ 估计器无罪；把 max_tilt 从 20° 扫到 45° 结果
//     不变（0.084m）⇒ 不是限幅不足，是相位不足。
//   新值 (0.9, 1.8)：ωn = 0.20 Hz（= 0.7×内环带宽）、ζ=0.71。
//     实测 3 连跑：包络收敛、俯仰 p90 0.23~0.32°、撞限幅 0%。
//   ⚠ 代价：位置环带宽减半 ⇒ 抗风/抗扰动的位置保持刚度下降、对外部位置
//     指令的响应变慢。若要提速，正解是**先抬姿态环带宽**（外环 P=6.0 /
//     内环 P=0.09 偏低），但需单独评估姿态环自身稳定性。
struct position_gains_t
{
    // 外环：位置 -> 速度（仅 P）
    float pos_p = 0.9f;

    // 内环：速度 -> 加速度（PID）
    float vel_p = 1.8f;
    float vel_i = 0.1f;
    float vel_d = 0.0f;

    // 前馈
    float vel_ff = 0.95f; // 速度前馈系数
};

// ===== 位置环限幅配置 =====
struct position_limits_t
{
    float max_vel_xy = 1.5f;                               // 最大水平速度 (m/s)
    float max_tilt_deg = 20.0f;                            // 最大倾斜角 (deg)
    float max_accel = 9.81f * tanf(20.0f * M_PI / 180.0f); // 推导最大加速度
};

class PositionController
{
public:
    PositionController();

    // 设置增益
    void setGains(const position_gains_t &gains);
    position_gains_t getGains() const { return gains_; }

    // 设置限幅
    void setLimits(const position_limits_t &limits);
    position_limits_t getLimits() const { return limits_; }

    // 核心更新
    // 输入：当前位置/速度(地球系)、目标位置/速度(地球系)、当前偏航角
    // 输出：目标 roll 和 pitch (rad)
    void update(float current_x, float current_y,
                float current_vx, float current_vy,
                float target_x, float target_y,
                float target_vx, float target_vy,
                float current_yaw, float dt);

    // 获取输出
    float getTargetRoll() const { return target_roll_; }
    float getTargetPitch() const { return target_pitch_; }

    // 状态管理
    void reset();
    // ★ 目标跳变时同步前馈基准（2026-10-01）：update() 用 (target−prev_target_)/dt
    //   差分估计目标速度前馈；目标位置一旦跳变（如自稳重捕获），该差分会产生
    //   raw_ff = 跳变量/dt 的尖峰（1m/0.002s=500 ⇒ max_vel/max_accel 连环饱和，
    //   且 alpha≈0.01 衰减 ~1s）。重捕获/换目标时必须调用本函数对齐基准。
    void syncFeedforward(float target_x, float target_y)
    {
        prev_target_x_ = target_x;
        prev_target_y_ = target_y;
        ff_vx_ = 0.0f;
        ff_vy_ = 0.0f;
    }
    void enable() { enabled_ = true; }
    void disable()
    {
        enabled_ = false;
        reset();
    }
    bool isEnabled() const { return enabled_; }

    // 调试
    const position_debug_info_t &getDebugInfo() const { return debug_info_; }

private:
    // 辅助：加速度矢量限幅
    void constrainVector(float &x, float &y, float max_magnitude);

    position_gains_t gains_;
    position_limits_t limits_;

    // 内部速度环 PID（X 和 Y 各一个）
    PIDController pid_vel_x_;
    PIDController pid_vel_y_;

    // 输出
    float target_roll_ = 0.0f;
    float target_pitch_ = 0.0f;

    float prev_target_x_ = 0, prev_target_y_ = 0;
    float ff_vx_ = 0, ff_vy_ = 0;

    // 状态
    bool enabled_ = true;

    // 调试
    position_debug_info_t debug_info_;
};

#endif // POSITION_CONTROLLER_H