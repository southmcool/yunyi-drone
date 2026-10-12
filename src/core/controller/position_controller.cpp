/**
 * @file position_controller.cpp
 * @brief 水平位置串级控制器实现
 */

#include "position_controller.h"

#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float DEG_TO_RAD_LOCAL = M_PI / 180.0f;
static constexpr float GRAVITY = 9.81f;

PositionController::PositionController()
{
    reset();
    setGains(gains_);
    setLimits(limits_);
}

// ===== 配置函数 =====

void PositionController::setGains(const position_gains_t &gains)
{
    gains_ = gains;
    pid_vel_x_.setGains(gains_.vel_p, gains_.vel_i, gains_.vel_d);
    pid_vel_y_.setGains(gains_.vel_p, gains_.vel_i, gains_.vel_d);

    Serial.println("✅ 位置环PID参数设置成功");

    Serial.printf("位置环: KP=%.5f\n",
                  gains.pos_p);
    Serial.printf("速度环: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  gains_.vel_p, gains_.vel_i, gains_.vel_d);
    Serial.printf("vel_ff: %.5f\n",
                  gains_.vel_ff);
}

void PositionController::setLimits(const position_limits_t &limits)
{
    limits_ = limits;
    // 同步更新最大加速度（由最大倾斜角推导）
    float max_tilt_rad = limits_.max_tilt_deg * DEG_TO_RAD_LOCAL;
    // ✅ 不再自动覆盖！使用传入的 max_accel
    // 仅当 max_accel 未显式设置时（为0或负数），才从 tilt 推导
    if (limits_.max_accel <= 0.0f)
    {
        float max_tilt_rad = limits_.max_tilt_deg * DEG_TO_RAD_LOCAL;
        limits_.max_accel = GRAVITY * tanf(max_tilt_rad);
    }

    // ✅ 关键：设置速度 PID 的输出限幅（加速度命令范围）
    pid_vel_x_.setLimits(-limits_.max_accel, limits_.max_accel);
    pid_vel_y_.setLimits(-limits_.max_accel, limits_.max_accel);
    // 更新 PID 输出限幅
    pid_vel_x_.setIntegralLimits(-limits_.max_accel, limits_.max_accel);
    pid_vel_y_.setIntegralLimits(-limits_.max_accel, limits_.max_accel);
    // ★ 2026-09-25 定点保持整改：积分分离 2.0→0.5 m/s。
    //   原阈值 2.0 m/s 远大于悬停时的速度误差（<0.3 m/s），积分永远激活，
    //   抗积分饱和形同虚设；0.5 m/s 下小误差才积、大误差靠 P 兜底。
    pid_vel_x_.setIntegralSeparation(0.5f);
    pid_vel_y_.setIntegralSeparation(0.5f);
    pid_vel_x_.setDerivativeFilter(0.1f);
    pid_vel_y_.setDerivativeFilter(0.1f);
    // 添加调试打印确认实际生效值
    Serial.printf("✅ Limits Applied: max_accel=%.3f, max_tilt=%.1f°, max_vel=%.1f\n",
                  limits_.max_accel, limits_.max_tilt_deg, limits_.max_vel_xy);
}

// ===== 核心更新逻辑 =====

void PositionController::update(
    float current_x, float current_y,
    float current_vx, float current_vy,
    float target_x, float target_y,
    float target_vx, float target_vy, // ⚠️ 注意：这个参数现在被内部前馈替代，但保留接口兼容性
    float current_yaw, float dt)
{
    if (dt <= 1e-6f)
        return; // 防止除零和异常dt

    if (!enabled_)
    {
        target_roll_ = 0.0f;
        target_pitch_ = 0.0f;
        return;
    }

    // ===== ✅ 修复1: 将 static 改为成员变量（在头文件中声明）=====
    // 在 position_controller.h 的 private 中添加:
    // float prev_target_x_ = 0, prev_target_y_ = 0;
    // float ff_vx_ = 0, ff_vy_ = 0;

    // 自适应平滑系数（限制在合理范围，避免dt过大时alpha=1导致噪声直通）
    float alpha = fminf(fmaxf(gains_.pos_p * dt, 0.01f), 0.9f);

    // 目标速度差分估计（带除零保护）
    float raw_ff_vx = (target_x - prev_target_x_) / dt;
    float raw_ff_vy = (target_y - prev_target_y_) / dt;

    // 一阶低通滤波
    ff_vx_ = ff_vx_ * (1.0f - alpha) + raw_ff_vx * alpha;
    ff_vy_ = ff_vy_ * (1.0f - alpha) + raw_ff_vy * alpha;

    prev_target_x_ = target_x;
    prev_target_y_ = target_y;

    // ===== 1. 外环：位置 P + 前馈 =====
    float pos_error_x = target_x - current_x;
    float pos_error_y = target_y - current_y;

    float desired_vx = gains_.pos_p * pos_error_x + gains_.vel_ff * ff_vx_;
    float desired_vy = gains_.pos_p * pos_error_y + gains_.vel_ff * ff_vy_;

    constrainVector(desired_vx, desired_vy, limits_.max_vel_xy);

    // ===== 2. 内环：速度 PID =====
    float accel_x = pid_vel_x_.update(desired_vx, current_vx, dt);
    float accel_y = pid_vel_y_.update(desired_vy, current_vy, dt);

    constrainVector(accel_x, accel_y, limits_.max_accel);

    // ===== 3. 精确逆动力学 =====
    float cos_yaw = cosf(current_yaw);
    float sin_yaw = sinf(current_yaw);

    float body_accel_x = accel_x * cos_yaw + accel_y * sin_yaw;
    float body_accel_y = -accel_x * sin_yaw + accel_y * cos_yaw;

    float out_pitch, out_roll;
    float accel_mag_sq = body_accel_x * body_accel_x + body_accel_y * body_accel_y;

    // ✅ 修复2: 用平方比较代替sqrt，减少计算量；仅在必要时调用atan2
    constexpr float ACCEL_THRESHOLD_SQ = 1e-4f;
    if (accel_mag_sq > ACCEL_THRESHOLD_SQ)
    {
        float accel_mag = sqrtf(accel_mag_sq);
        float tilt_angle = atan2f(accel_mag, GRAVITY);
        float scale = tilt_angle / accel_mag;
        out_pitch = -body_accel_x * scale; // 前加速→低头
        out_roll = +body_accel_y * scale;  // 右加速→右压杆
    }
    else
    {
        out_pitch = -body_accel_x / GRAVITY;
out_roll  = +body_accel_y / GRAVITY;
    }

    // ===== 4. 姿态限幅 =====
    float max_tilt_rad = limits_.max_tilt_deg * DEG_TO_RAD_LOCAL;
    // out_pitch = constrain(out_pitch, -max_tilt_rad, max_tilt_rad);
    // out_roll = constrain(out_roll, -max_tilt_rad, max_tilt_rad);

    target_roll_ = out_roll;
    target_pitch_ = out_pitch;

    // ===== 5. 调试信息（保持不变）=====
    debug_info_.pos_error[0] = pos_error_x;
    debug_info_.pos_error[1] = pos_error_y;
    debug_info_.vel_error[0] = desired_vx - current_vx;
    debug_info_.vel_error[1] = desired_vy - current_vy;
    debug_info_.target_vel[0] = desired_vx;
    debug_info_.target_vel[1] = desired_vy;
    debug_info_.target_accel[0] = accel_x;
    debug_info_.target_accel[1] = accel_y;
    debug_info_.target_roll = out_roll;
    debug_info_.target_pitch = out_pitch;
}

// ===== 状态管理 =====

void PositionController::reset()
{
    pid_vel_x_.reset();
    pid_vel_y_.reset();
    target_roll_ = 0.0f;
    target_pitch_ = 0.0f;
    debug_info_ = position_debug_info_t{};
    // ★ 前馈基准一并清零（2026-10-01）：原实现遗漏，prev_target_/ff_ 残留旧值，
    //   reset 后首个 update 的目标差分会把残留值当"目标在移动"产生假前馈。
    prev_target_x_ = 0.0f;
    prev_target_y_ = 0.0f;
    ff_vx_ = 0.0f;
    ff_vy_ = 0.0f;
}

// ===== 辅助函数 =====

void PositionController::constrainVector(float &x, float &y, float max_magnitude)
{
    float magnitude = sqrtf(x * x + y * y);
    if (magnitude > max_magnitude && magnitude > 1e-6f)
    {
        float scale = max_magnitude / magnitude;
        x *= scale;
        y *= scale;
    }
}
