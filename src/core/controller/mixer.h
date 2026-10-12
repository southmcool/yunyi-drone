/**
 * @title  混控器
 * @author: vaie
 * @date: 2026-10-12
 *
 * mixer.h — 把「归一化推力 + 三轴力矩」分配到 4 个电机。
 *
 * 职责边界：
 *   · 输入：thrust ∈ [0,1]，roll/pitch/yaw 为无量纲力矩
 *   · 输出：motor[4] ∈ [0,1]（未乘 DShot 满值）
 *   · 饱和处理：上溢缩姿态分量；下溢保 MOTOR_FLOOR
 *   · **不做**：DShot 转换、arming 状态、写全局变量 —— 那些属于 task 层
 *
 * 换机架 / 换电机布局只改本文件，控制器零改动。
 *
 * 无平台依赖，SIL 与实机共用。
 */
#pragma once

#include <cstdint>

#include "type/stabilizer_types.h"

class Mixer
{
public:
    Mixer() = delete; // 纯静态

    // 混控
    //   thrust / roll / pitch / yaw 均来自 AttitudeController::update 的输出
    //   out->timestamp 填 0，由调用方填（这里不引 esp_timer）
    static void mix(control_t output,
                    motor_mix_t *out);

    // 直通（未上锁时用）：4 个电机输出同一个归一化值
    static void allSame(float normalized, motor_mix_t *out);

private:
    // X 型四轴 NED（与 attitude_controller.h 的原表一致）
    //   行 = 电机 [FL, FR, BL, BR]
    //   列 = 系数 [roll, pitch, yaw]
    static constexpr float kMix[4][3] = {
        {+1.0f, +1.0f, +1.0f}, // FL (CCW) FL:
        {-1.0f, +1.0f, -1.0f}, // FR (CW) FR:
        {+1.0f, -1.0f, -1.0f}, // BL (CW) BL:
        {-1.0f, -1.0f, +1.0f}  // BR (CCW) BR:
    };

    // 电机最小值（防单桨熄火——你的 R77j 经验）
    static constexpr float kMotorFloor = 0.03f;
};