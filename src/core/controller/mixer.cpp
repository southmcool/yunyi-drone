#include "core/controller/mixer.h"
#include <cmath>

void Mixer::mix(control_t output,
                motor_mix_t *out)
{
    if (!out)
        return;

    // ---- 1. 计算未限幅的 4 电机 ----
    float raw[4];
    float min_v = 1e9f, max_v = -1e9f;
    for (int i = 0; i < 4; ++i)
    {
        raw[i] = output.thrust + kMix[i][0] * output.roll + kMix[i][1] * output.pitch + kMix[i][2] * output.yaw;
        if (raw[i] < min_v)
            min_v = raw[i];
        if (raw[i] > max_v)
            max_v = raw[i];
    }

    // ---- 2. 无越界：直接输出 ----
    if (min_v >= 0.0f && max_v <= 1.0f)
    {
        for (int i = 0; i < 4; ++i)
            out->motor[i] = raw[i];
        return;
    }

    // ---- 3. 有越界：只缩放姿态分量，推力基准不变 ----
    float attitude_scale = 1.0f;

    // 上溢：把最大姿态贡献缩到 (1 - thrust)
    if (max_v > 1.0f)
    {
        float max_attitude = 0.0f;
        for (int i = 0; i < 4; ++i)
        {
            const float a = raw[i] - output.thrust;
            if (a > max_attitude)
                max_attitude = a;
        }
        if (max_attitude > 1e-3f)
        {
            const float cap = 1.0f - output.thrust;
            const float s = cap / max_attitude;
            if (s < attitude_scale)
                attitude_scale = s;
        }
    }

    // 下溢：把最负姿态贡献缩到 (thrust - MOTOR_FLOOR)
    if (min_v < 0.0f)
    {
        float min_attitude = 0.0f;
        for (int i = 0; i < 4; ++i)
        {
            const float a = raw[i] - output.thrust;
            if (a < min_attitude)
                min_attitude = a;
        }
        if (min_attitude < -1e-3f)
        {
            const float floor_target =
                (output.thrust > kMotorFloor) ? (output.thrust - kMotorFloor) : 0.0f;
            const float s = floor_target / (-min_attitude);
            if (s < attitude_scale)
                attitude_scale = s;
        }
    }

    // 保护：极端情况下 scale 可能为负，钳到 [0, 1]
    if (attitude_scale < 0.0f)
        attitude_scale = 0.0f;
    if (attitude_scale > 1.0f)
        attitude_scale = 1.0f;

    // ---- 4. 应用缩放 ----
    for (int i = 0; i < 4; ++i)
    {
        const float attitude_part = raw[i] - output.thrust;
        out->motor[i] = output.thrust + attitude_part * attitude_scale;
    }
}

void Mixer::allSame(float normalized, motor_mix_t *out)
{
    if (!out)
        return;
    for (int i = 0; i < 4; ++i)
        out->motor[i] = normalized;
}