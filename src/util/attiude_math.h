#ifndef ATTIUDE_MATH_H
#define ATTIUDE_MATH_H
#include <math.h>
#include <Arduino.h>
// 四元数结构体
struct Quaternion
{
    float w, x, y, z;
};

// 欧拉角(rad) → 四元数 (顺序: ZYX - yaw, pitch, roll)
inline Quaternion eulerToQuat(float roll, float pitch, float yaw)
{
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

// 四元数共轭
inline Quaternion quatConjugate(const Quaternion &q)
{
    return {q.w, -q.x, -q.y, -q.z};
}

// 四元数乘法 (q1 * q2)
inline Quaternion quatMultiply(const Quaternion &q1, const Quaternion &q2)
{
    return {
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w};
}

inline void quatToAxisError(const Quaternion &q_err, float &err_x, float &err_y, float &err_z)
{
    Quaternion q = q_err;
    // 确保是最短路径旋转：若 w < 0，对整个四元数取反（旋转角变 2π-θ，轴反向）
    if (q.w < 0.0f)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    float w = constrain(q.w, -1.0f, 1.0f);
    if (w > 0.9999f)
    {
        err_x = 2.0f * q.x;
        err_y = 2.0f * q.y;
        err_z = 2.0f * q.z;
    }
    else
    {
        float angle = 2.0f * acosf(w); // 此时 angle ∈ [0, π]
        float sin_half = sinf(angle / 2.0f);
        if (fabs(sin_half) > 1e-6f)
        {
            float factor = angle / sin_half;
            err_x = q.x * factor;
            err_y = q.y * factor;
            err_z = q.z * factor;
        }
        else
        {
            // 退化情况（angle≈0）
            err_x = 2.0f * q.x;
            err_y = 2.0f * q.y;
            err_z = 2.0f * q.z;
        }
    }
}

#endif