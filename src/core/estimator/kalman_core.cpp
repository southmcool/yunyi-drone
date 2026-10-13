/**
 * @title  Kalman 核心实现（NED 适配版）
 * @author: vaie
 * @date: 2026-10-12
 *
 * 参考 Crazyflie kalman_core.c（Mueller 2015 / Mueller 2016）。
 * 本文件与 Crazyflie 原版的差异 —— 仅 NED 世界系适配 17 处：
 *   · Crazyflie 世界系 = NWU（Z 向上）；本项目 = NED（Z 向下）
 *   · 重力方向：NWU 下 g_world = (0,0,-g)；NED 下 = (0,0,+g)
 *   · 所有"含重力"的项符号翻转（见下方代码注释 ★ NED）
 *   · 其余（状态方程、四元数、Rodrigues 近似、协方差旋转）完全一致
 *
 * 无平台依赖：只用 <cmath> / <cstring>。
 */

#include "core/estimator/kalman_core.h"
#include "core/estimator/kalman_matrix.h"

#include <cmath>
#include <cstring>

// ============================================================================
//  内部辅助
// ============================================================================

namespace
{

    void quatMult(const float a[4], const float b[4], float r[4])
    {
        const float a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
        const float b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
        r[0] = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
        r[1] = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2;
        r[2] = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1;
        r[3] = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
    }

    void quatNormalize(float q[4])
    {
        const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (n2 < 1e-20f)
        {
            q[0] = 1.0f;
            q[1] = q[2] = q[3] = 0.0f;
            return;
        }
        const float inv = 1.0f / std::sqrt(n2);
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    }

    // R = body→world（Hamilton，标量在前，与 Crazyflie quatToR 一致）
    void quatToR(const float q[4], float R[3][3])
    {
        R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        R[0][1] = 2.0f * q[1] * q[2] - 2.0f * q[0] * q[3];
        R[0][2] = 2.0f * q[1] * q[3] + 2.0f * q[0] * q[2];

        R[1][0] = 2.0f * q[1] * q[2] + 2.0f * q[0] * q[3];
        R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
        R[1][2] = 2.0f * q[2] * q[3] - 2.0f * q[0] * q[1];

        R[2][0] = 2.0f * q[1] * q[3] - 2.0f * q[0] * q[2];
        R[2][1] = 2.0f * q[2] * q[3] + 2.0f * q[0] * q[1];
        R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    }

    void enforceCovarianceBounds(kalmanCoreData_t *kf)
    {
        using namespace kalman;
        for (int i = 0; i < KC_STATE_DIM; ++i)
        {
            for (int j = i; j < KC_STATE_DIM; ++j)
            {
                float p = 0.5f * kf->P[i][j] + 0.5f * kf->P[j][i];
                if (std::isnan(p) || p > kMaxCovariance)
                    p = kMaxCovariance;
                else if (i == j && p < kMinCovariance)
                    p = kMinCovariance;
                kf->P[i][j] = kf->P[j][i] = p;
            }
        }
    }

    constexpr const char *kMod = "kalman_core";

} // namespace

// ============================================================================
//  噪声参数（Crazyflie 默认值）
// ============================================================================

namespace
{
    constexpr float kProcNoiseAccXY = 0.5f;
    constexpr float kProcNoiseAccZ = 1.0f;
    constexpr float kProcNoiseVel = 0.0f;
    constexpr float kProcNoisePos = 0.0f;
    constexpr float kProcNoiseAtt = 0.0f;

    constexpr float kMeasNoiseBaro = 2.0f;
    constexpr float kMeasNoiseGyroRP = 0.1f;
    constexpr float kMeasNoiseGyroYaw = 0.1f;
} // namespace

// ============================================================================
//  scalarUpdate
// ============================================================================

namespace
{

    void scalarUpdate(kalmanCoreData_t *kf,
                      const float H[KC_STATE_DIM],
                      float error,
                      float stdMeasNoise)
    {
        static float K[KC_STATE_DIM];
        static float tmp1[KC_STATE_DIM][KC_STATE_DIM];
        static float tmp2[KC_STATE_DIM][KC_STATE_DIM];
        static float tmp3[KC_STATE_DIM][KC_STATE_DIM];
        static float P_new[KC_STATE_DIM][KC_STATE_DIM];

        // PHT = P · H^T
        float PHT[KC_STATE_DIM];
        for (int i = 0; i < KC_STATE_DIM; ++i)
        {
            float s = 0.0f;
            for (int j = 0; j < KC_STATE_DIM; ++j)
                s += kf->P[i][j] * H[j];
            PHT[i] = s;
        }
        const float R_var = stdMeasNoise * stdMeasNoise;
        float HPHR = R_var;
        for (int i = 0; i < KC_STATE_DIM; ++i)
            HPHR += H[i] * PHT[i];
        if (HPHR < 1e-12f)
            HPHR = 1e-12f;

        for (int i = 0; i < KC_STATE_DIM; ++i)
        {
            K[i] = PHT[i] / HPHR;
            kf->S[i] += K[i] * error;
        }

        for (int i = 0; i < KC_STATE_DIM; ++i)
        {
            for (int j = 0; j < KC_STATE_DIM; ++j)
                tmp1[i][j] = K[i] * H[j];
            tmp1[i][i] -= 1.0f;
        }
        for (int i = 0; i < KC_STATE_DIM; ++i)
            for (int j = 0; j < KC_STATE_DIM; ++j)
                tmp2[i][j] = tmp1[j][i];
        kmat::mult9(tmp1, kf->P, tmp3);
        kmat::mult9(tmp3, tmp2, P_new);

        for (int i = 0; i < KC_STATE_DIM; ++i)
        {
            for (int j = i; j < KC_STATE_DIM; ++j)
            {
                float p = 0.5f * P_new[i][j] + 0.5f * P_new[j][i] + K[i] * R_var * K[j];
                if (std::isnan(p) || p > kalman::kMaxCovariance)
                    p = kalman::kMaxCovariance;
                else if (i == j && p < kalman::kMinCovariance)
                    p = kalman::kMinCovariance;
                kf->P[i][j] = kf->P[j][i] = p;
            }
        }
    }

} // namespace

// ============================================================================
//  kalmanCorePredict
// ============================================================================

void kalmanCorePredict(kalmanCoreData_t *kf,
                       float thrust_norm,
                       const Axis3f *acc,
                       const Axis3f *gyro,
                       float dt,
                       bool quadIsFlying)
{
    if (!kf || !acc || !gyro)
        return;
    (void)thrust_norm;

    static float A[KC_STATE_DIM][KC_STATE_DIM];
    static float At[KC_STATE_DIM][KC_STATE_DIM];
    static float tmp[KC_STATE_DIM][KC_STATE_DIM];

    const float dt2 = dt * dt;

    // 每轮清零 A（防御式：非零项后面都会显式写入，但显式清零更安全）
    std::memset(A, 0, sizeof(A));

    // ---- 单位矩阵项 ----
    A[KC_STATE_X][KC_STATE_X] = 1.0f;
    A[KC_STATE_Y][KC_STATE_Y] = 1.0f;
    A[KC_STATE_Z][KC_STATE_Z] = 1.0f;

    A[KC_STATE_PX][KC_STATE_PX] = 1.0f;
    A[KC_STATE_PY][KC_STATE_PY] = 1.0f;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1.0f;

    A[KC_STATE_D0][KC_STATE_D0] = 1.0f;
    A[KC_STATE_D1][KC_STATE_D1] = 1.0f;
    A[KC_STATE_D2][KC_STATE_D2] = 1.0f;

    // ---- 位置 ← 机体系速度 ----
    A[KC_STATE_X][KC_STATE_PX] = kf->R[0][0] * dt;
    A[KC_STATE_Y][KC_STATE_PX] = kf->R[1][0] * dt;
    A[KC_STATE_Z][KC_STATE_PX] = kf->R[2][0] * dt;

    A[KC_STATE_X][KC_STATE_PY] = kf->R[0][1] * dt;
    A[KC_STATE_Y][KC_STATE_PY] = kf->R[1][1] * dt;
    A[KC_STATE_Z][KC_STATE_PY] = kf->R[2][1] * dt;

    A[KC_STATE_X][KC_STATE_PZ] = kf->R[0][2] * dt;
    A[KC_STATE_Y][KC_STATE_PZ] = kf->R[1][2] * dt;
    A[KC_STATE_Z][KC_STATE_PZ] = kf->R[2][2] * dt;

    // ---- 位置 ← 姿态误差（与重力无关，两系一致）----
    A[KC_STATE_X][KC_STATE_D0] = (kf->S[KC_STATE_PY] * kf->R[0][2] - kf->S[KC_STATE_PZ] * kf->R[0][1]) * dt;
    A[KC_STATE_Y][KC_STATE_D0] = (kf->S[KC_STATE_PY] * kf->R[1][2] - kf->S[KC_STATE_PZ] * kf->R[1][1]) * dt;
    A[KC_STATE_Z][KC_STATE_D0] = (kf->S[KC_STATE_PY] * kf->R[2][2] - kf->S[KC_STATE_PZ] * kf->R[2][1]) * dt;

    A[KC_STATE_X][KC_STATE_D1] = (-kf->S[KC_STATE_PX] * kf->R[0][2] + kf->S[KC_STATE_PZ] * kf->R[0][0]) * dt;
    A[KC_STATE_Y][KC_STATE_D1] = (-kf->S[KC_STATE_PX] * kf->R[1][2] + kf->S[KC_STATE_PZ] * kf->R[1][0]) * dt;
    A[KC_STATE_Z][KC_STATE_D1] = (-kf->S[KC_STATE_PX] * kf->R[2][2] + kf->S[KC_STATE_PZ] * kf->R[2][0]) * dt;

    A[KC_STATE_X][KC_STATE_D2] = (kf->S[KC_STATE_PX] * kf->R[0][1] - kf->S[KC_STATE_PY] * kf->R[0][0]) * dt;
    A[KC_STATE_Y][KC_STATE_D2] = (kf->S[KC_STATE_PX] * kf->R[1][1] - kf->S[KC_STATE_PY] * kf->R[1][0]) * dt;
    A[KC_STATE_Z][KC_STATE_D2] = (kf->S[KC_STATE_PX] * kf->R[2][1] - kf->S[KC_STATE_PY] * kf->R[2][0]) * dt;

    // ---- 机体系速度 ← 机体系速度（陀螺叉乘 -[[ω]]，两系一致）----
    A[KC_STATE_PX][KC_STATE_PX] = 1.0f;
    A[KC_STATE_PY][KC_STATE_PX] = -gyro->z * dt;
    A[KC_STATE_PZ][KC_STATE_PX] = gyro->y * dt;

    A[KC_STATE_PX][KC_STATE_PY] = gyro->z * dt;
    A[KC_STATE_PY][KC_STATE_PY] = 1.0f;
    A[KC_STATE_PZ][KC_STATE_PY] = -gyro->x * dt;

    A[KC_STATE_PX][KC_STATE_PZ] = -gyro->y * dt;
    A[KC_STATE_PY][KC_STATE_PZ] = gyro->x * dt;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1.0f;

    // ---- 机体系速度 ← 姿态误差（重力对 d 的偏导）----
    //   ★ NED：相对 Crazyflie 原版全部翻符号
    A[KC_STATE_PX][KC_STATE_D0] = 0.0f;
    A[KC_STATE_PY][KC_STATE_D0] = kalman::kGravity * kf->R[2][2] * dt;
    A[KC_STATE_PZ][KC_STATE_D0] = -kalman::kGravity * kf->R[2][1] * dt;

    A[KC_STATE_PX][KC_STATE_D1] = -kalman::kGravity * kf->R[2][2] * dt;
    A[KC_STATE_PY][KC_STATE_D1] = 0.0f;
    A[KC_STATE_PZ][KC_STATE_D1] = kalman::kGravity * kf->R[2][0] * dt;

    A[KC_STATE_PX][KC_STATE_D2] = kalman::kGravity * kf->R[2][1] * dt;
    A[KC_STATE_PY][KC_STATE_D2] = -kalman::kGravity * kf->R[2][0] * dt;
    A[KC_STATE_PZ][KC_STATE_D2] = 0.0f;

    // ---- 姿态误差 ← 姿态误差（Rodrigues 二阶近似，与两系无关）----
    const float d0 = gyro->x * dt * 0.5f;
    const float d1 = gyro->y * dt * 0.5f;
    const float d2 = gyro->z * dt * 0.5f;

    A[KC_STATE_D0][KC_STATE_D0] = 1.0f - d1 * d1 * 0.5f - d2 * d2 * 0.5f;
    A[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 * 0.5f;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 * 0.5f;

    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 * 0.5f;
    A[KC_STATE_D1][KC_STATE_D1] = 1.0f - d0 * d0 * 0.5f - d2 * d2 * 0.5f;
    A[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 * 0.5f;

    A[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 * 0.5f;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 * 0.5f;
    A[KC_STATE_D2][KC_STATE_D2] = 1.0f - d0 * d0 * 0.5f - d1 * d1 * 0.5f;

    // ---- 协方差预测 P = A·P·A^T ----
    kmat::mult9(A, kf->P, tmp);
    kmat::trans9(A, At);
    kmat::mult9(tmp, At, kf->P);

    // ---- 状态预测 ----
    float dx, dy, dz;
    float tmpSPX, tmpSPY, tmpSPZ;
    float zacc;

    if (quadIsFlying)
    {
        zacc = acc->z;

        dx = kf->S[KC_STATE_PX] * dt;
        dy = kf->S[KC_STATE_PY] * dt;
        dz = kf->S[KC_STATE_PZ] * dt + zacc * dt2 * 0.5f;

        // ★ NED：位置项 `+ g*dt2/2`（NWU 是 `- g*dt2/2`）
        kf->S[KC_STATE_X] += kf->R[0][0] * dx + kf->R[0][1] * dy + kf->R[0][2] * dz;
        kf->S[KC_STATE_Y] += kf->R[1][0] * dx + kf->R[1][1] * dy + kf->R[1][2] * dz;
        kf->S[KC_STATE_Z] += kf->R[2][0] * dx + kf->R[2][1] * dy + kf->R[2][2] * dz + kalman::kGravity * dt2 * 0.5f;

        tmpSPX = kf->S[KC_STATE_PX];
        tmpSPY = kf->S[KC_STATE_PY];
        tmpSPZ = kf->S[KC_STATE_PZ];

        // ★ NED：`+ g*R[2][x]`（NWU 是 `- g*R[2][x]`）
        kf->S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ + kalman::kGravity * kf->R[2][0]);
        kf->S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ + kalman::kGravity * kf->R[2][1]);
        kf->S[KC_STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY + kalman::kGravity * kf->R[2][2]);
    }
    else
    {
        dx = kf->S[KC_STATE_PX] * dt + acc->x * dt2 * 0.5f;
        dy = kf->S[KC_STATE_PY] * dt + acc->y * dt2 * 0.5f;
        dz = kf->S[KC_STATE_PZ] * dt + acc->z * dt2 * 0.5f;

        // ★ NED：`+ g*dt2/2`
        kf->S[KC_STATE_X] += kf->R[0][0] * dx + kf->R[0][1] * dy + kf->R[0][2] * dz;
        kf->S[KC_STATE_Y] += kf->R[1][0] * dx + kf->R[1][1] * dy + kf->R[1][2] * dz;
        kf->S[KC_STATE_Z] += kf->R[2][0] * dx + kf->R[2][1] * dy + kf->R[2][2] * dz + kalman::kGravity * dt2 * 0.5f;

        tmpSPX = kf->S[KC_STATE_PX];
        tmpSPY = kf->S[KC_STATE_PY];
        tmpSPZ = kf->S[KC_STATE_PZ];

        // ★ NED：`+ g*R[2][x]`
        kf->S[KC_STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ + kalman::kGravity * kf->R[2][0]);
        kf->S[KC_STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ + kalman::kGravity * kf->R[2][1]);
        kf->S[KC_STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY + kalman::kGravity * kf->R[2][2]);
    }

    // ---- 参考四元数积分（陀螺）----
    const float dtwx = dt * gyro->x;
    const float dtwy = dt * gyro->y;
    const float dtwz = dt * gyro->z;

    const float angle = std::sqrt(dtwx * dtwx + dtwy * dtwy + dtwz * dtwz);
    float dq[4];
    if (angle > 1e-6f)
    {
        const float ca = std::cos(angle * 0.5f);
        const float sa = std::sin(angle * 0.5f);
        dq[0] = ca;
        dq[1] = sa * dtwx / angle;
        dq[2] = sa * dtwy / angle;
        dq[3] = sa * dtwz / angle;
    }
    else
    {
        dq[0] = 1.0f;
        dq[1] = dq[2] = dq[3] = 0.0f;
    }

    float qNew[4];
    qNew[0] = dq[0] * kf->q[0] - dq[1] * kf->q[1] - dq[2] * kf->q[2] - dq[3] * kf->q[3];
    qNew[1] = dq[1] * kf->q[0] + dq[0] * kf->q[1] + dq[3] * kf->q[2] - dq[2] * kf->q[3];
    qNew[2] = dq[2] * kf->q[0] - dq[3] * kf->q[1] + dq[0] * kf->q[2] + dq[1] * kf->q[3];
    qNew[3] = dq[3] * kf->q[0] + dq[2] * kf->q[1] - dq[1] * kf->q[2] + dq[0] * kf->q[3];

    // // 地面缓拉回
    // if (!quadIsFlying)
    // {
    //     const float keep = 1.0f - kalman::kRollPitchZeroReversion;
    //     qNew[0] = keep * qNew[0] + kalman::kRollPitchZeroReversion * 1.0f;
    //     qNew[1] = keep * qNew[1];
    //     qNew[2] = keep * qNew[2];
    //     qNew[3] = keep * qNew[3];
    // }

    const float norm = std::sqrt(qNew[0] * qNew[0] + qNew[1] * qNew[1] + qNew[2] * qNew[2] + qNew[3] * qNew[3]);
    if (norm > 1e-12f)
    {
        kf->q[0] = qNew[0] / norm;
        kf->q[1] = qNew[1] / norm;
        kf->q[2] = qNew[2] / norm;
        kf->q[3] = qNew[3] / norm;
    }
}

// ============================================================================
//  kalmanCoreAddProcessNoise
// ============================================================================

void kalmanCoreAddProcessNoise(kalmanCoreData_t *kf, float dt)
{
    if (!kf)
        return;

    if (dt > 0.0f)
    {
        const float pn_xy_pos = kProcNoiseAccXY * dt * dt + kProcNoiseVel * dt + kProcNoisePos;
        const float pn_z_pos = kProcNoiseAccZ * dt * dt + kProcNoiseVel * dt + kProcNoisePos;
        kf->P[KC_STATE_X][KC_STATE_X] += pn_xy_pos * pn_xy_pos;
        kf->P[KC_STATE_Y][KC_STATE_Y] += pn_xy_pos * pn_xy_pos;
        kf->P[KC_STATE_Z][KC_STATE_Z] += pn_z_pos * pn_z_pos;

        const float pn_xy_vel = kProcNoiseAccXY * dt + kProcNoiseVel;
        const float pn_z_vel = kProcNoiseAccZ * dt + kProcNoiseVel;
        kf->P[KC_STATE_PX][KC_STATE_PX] += pn_xy_vel * pn_xy_vel;
        kf->P[KC_STATE_PY][KC_STATE_PY] += pn_xy_vel * pn_xy_vel;
        kf->P[KC_STATE_PZ][KC_STATE_PZ] += pn_z_vel * pn_z_vel;

        const float pn_rp = kMeasNoiseGyroRP * dt + kProcNoiseAtt;
        const float pn_y = kMeasNoiseGyroYaw * dt + kProcNoiseAtt;
        kf->P[KC_STATE_D0][KC_STATE_D0] += pn_rp * pn_rp;
        kf->P[KC_STATE_D1][KC_STATE_D1] += pn_rp * pn_rp;
        kf->P[KC_STATE_D2][KC_STATE_D2] += pn_y * pn_y;
    }

    enforceCovarianceBounds(kf);
}

// ============================================================================
//  kalmanCoreUpdateWithBaro
//   ★ 前提：调用方传入 baroAsl 是 NED 深度（向下为正），
//     即 estimator_task 传 `-altitude_m`。
// ============================================================================

void kalmanCoreUpdateWithBaro(kalmanCoreData_t *kf,
                              float baroAsl,
                              bool quadIsFlying)
{
    if (!kf)
        return;

    float H[KC_STATE_DIM] = {0};
    H[KC_STATE_Z] = 1.0f;

    // ★ NED：地面持续刷新基准（去掉原 `baroReferenceHeight < 1` 判据）
    if (!quadIsFlying)
    {
        kf->baroReferenceHeight = baroAsl;
    }

    const float meas = baroAsl - kf->baroReferenceHeight;
    scalarUpdate(kf, H, meas - kf->S[KC_STATE_Z], kMeasNoiseBaro);
}

// ============================================================================
//  kalmanCoreInit
// ============================================================================

void kalmanCoreInit(kalmanCoreData_t *kf)
{
    if (!kf)
        return;

    std::memset(kf, 0, sizeof(kalmanCoreData_t));

    kf->S[KC_STATE_X] = 0.0f;
    kf->S[KC_STATE_Y] = 0.0f;
    kf->S[KC_STATE_Z] = 0.0f;

    kf->q[0] = 1.0f;
    kf->q[1] = 0.0f;
    kf->q[2] = 0.0f;
    kf->q[3] = 0.0f;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            kf->R[i][j] = (i == j) ? 1.0f : 0.0f;

    kf->P[KC_STATE_X][KC_STATE_X] = 100.0f * 100.0f;
    kf->P[KC_STATE_Y][KC_STATE_Y] = 100.0f * 100.0f;
    kf->P[KC_STATE_Z][KC_STATE_Z] = 1.0f;

    kf->P[KC_STATE_PX][KC_STATE_PX] = 0.01f * 0.01f;
    kf->P[KC_STATE_PY][KC_STATE_PY] = 0.01f * 0.01f;
    kf->P[KC_STATE_PZ][KC_STATE_PZ] = 0.01f * 0.01f;

    kf->P[KC_STATE_D0][KC_STATE_D0] = 0.01f * 0.01f;
    kf->P[KC_STATE_D1][KC_STATE_D1] = 0.01f * 0.01f;
    kf->P[KC_STATE_D2][KC_STATE_D2] = 0.01f * 0.01f;

    kf->resetEstimation = false;
    kf->baroReferenceHeight = 0.0f;
    kf->lastUpdateUs = 0;

    kf->mag_initialized = false;
    kf->mag_ref_yaw = 0.0f;

    kf->tof_initialized = false;
}

// ============================================================================
//  kalmanCoreFinalize（与坐标系无关，与 Crazyflie 完全一致）
// ============================================================================

void kalmanCoreFinalize(kalmanCoreData_t *kf, uint64_t osTickUs)
{
    if (!kf)
        return;

    const float v0 = kf->S[KC_STATE_D0];
    const float v1 = kf->S[KC_STATE_D1];
    const float v2 = kf->S[KC_STATE_D2];

    const bool meaningful = (std::fabs(v0) > 0.1e-3f) ||
                            (std::fabs(v1) > 0.1e-3f) ||
                            (std::fabs(v2) > 0.1e-3f);
    const bool sane = (std::fabs(v0) < 10.0f) &&
                      (std::fabs(v1) < 10.0f) &&
                      (std::fabs(v2) < 10.0f);

    if (meaningful && sane)
    {
        const float angle = std::sqrt(v0 * v0 + v1 * v1 + v2 * v2);
        const float ca = std::cos(angle * 0.5f);
        const float sa = std::sin(angle * 0.5f);
        float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

        float qNew[4];
        quatMult(dq, kf->q, qNew);
        quatNormalize(qNew);
        kf->q[0] = qNew[0];
        kf->q[1] = qNew[1];
        kf->q[2] = qNew[2];
        kf->q[3] = qNew[3];

        const float d0 = v0 * 0.5f, d1 = v1 * 0.5f, d2 = v2 * 0.5f;

        static float A[9][9];
        std::memset(A, 0, sizeof(A));
        for (int i = 0; i < 9; ++i)
            A[i][i] = 1.0f;

        A[KC_STATE_D0][KC_STATE_D0] = 1.0f - d1 * d1 * 0.5f - d2 * d2 * 0.5f;
        A[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 * 0.5f;
        A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 * 0.5f;

        A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 * 0.5f;
        A[KC_STATE_D1][KC_STATE_D1] = 1.0f - d0 * d0 * 0.5f - d2 * d2 * 0.5f;
        A[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 * 0.5f;

        A[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 * 0.5f;
        A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 * 0.5f;
        A[KC_STATE_D2][KC_STATE_D2] = 1.0f - d0 * d0 * 0.5f - d1 * d1 * 0.5f;

        static float tmp[9][9];
        kmat::mult9(A, kf->P, tmp);

        static float At[9][9];
        kmat::trans9(A, At);
        kmat::mult9(tmp, At, kf->P);
    }

    kf->S[KC_STATE_D0] = 0.0f;
    kf->S[KC_STATE_D1] = 0.0f;
    kf->S[KC_STATE_D2] = 0.0f;

    quatToR(kf->q, kf->R);
    enforceCovarianceBounds(kf);
    kf->lastUpdateUs = osTickUs;
}

// ============================================================================
//  kalmanCoreExternalizeState
//   ★ 输出 state_t 的语义：
//       position：世界系 NED（Z 向下为深度）
//       velocity：世界系 NED
//       acc     ：世界系 NED，去重力后（Gs）
//       attitude：roll/pitch/yaw (deg)，NED 直接给出
// ============================================================================

void kalmanCoreExternalizeState(const kalmanCoreData_t *kf,
                                state_t *state,
                                const Axis3f *acc,
                                const Axis3f *gyro_rads, // ★ 新增参数
                                uint64_t osTickUs)
{
    if (!kf || !state)
        return;

    state->attitude_deg.timestamp = osTickUs;
    state->attitude_rad.timestamp = osTickUs;
    state->attitudeQuaternion.timestamp = osTickUs;
    state->position.timestamp = osTickUs;
    state->velocity.timestamp = osTickUs;
    state->acc.timestamp = osTickUs;

    // 位置
    state->position.x = kf->S[KC_STATE_X];
    state->position.y = kf->S[KC_STATE_Y];
    state->position.z = kf->S[KC_STATE_Z];

    // 速度
    state->velocity.x = kf->R[0][0] * kf->S[KC_STATE_PX] + kf->R[0][1] * kf->S[KC_STATE_PY] + kf->R[0][2] * kf->S[KC_STATE_PZ];
    state->velocity.y = kf->R[1][0] * kf->S[KC_STATE_PX] + kf->R[1][1] * kf->S[KC_STATE_PY] + kf->R[1][2] * kf->S[KC_STATE_PZ];
    state->velocity.z = kf->R[2][0] * kf->S[KC_STATE_PX] + kf->R[2][1] * kf->S[KC_STATE_PY] + kf->R[2][2] * kf->S[KC_STATE_PZ];

    // ★ 辅助字段：向上为正
    state->altitude = -state->position.z;
    state->vertical_velocity = -state->velocity.z;
    // acc（世界系，去重力）
    if (acc)
    {
        state->acc.x = kf->R[0][0] * acc->x + kf->R[0][1] * acc->y + kf->R[0][2] * acc->z;
        state->acc.y = kf->R[1][0] * acc->x + kf->R[1][1] * acc->y + kf->R[1][2] * acc->z;
        state->acc.z = kf->R[2][0] * acc->x + kf->R[2][1] * acc->y + kf->R[2][2] * acc->z + kalman::kGravity; // ← 不是 1.0f
        // state->acc.z = kf->R[2][0] * acc->x + kf->R[2][1] * acc->y + kf->R[2][2] * acc->z + 1.0f; // ← 不是 1.0f
        // BOOT_LOG_W(kMod, "state->acc.z%.4f", state->acc.z);
    }
    else
    {
        state->acc.x = state->acc.y = state->acc.z = 0.0f;
    }

    // 四元数
    state->attitudeQuaternion.q0 = kf->q[0];
    state->attitudeQuaternion.q1 = kf->q[1];
    state->attitudeQuaternion.q2 = kf->q[2];
    state->attitudeQuaternion.q3 = kf->q[3];

    // Euler
    constexpr float kRad2Deg = 57.2957795f;
    const float q0 = kf->q[0], q1 = kf->q[1], q2 = kf->q[2], q3 = kf->q[3];

    const float yaw = std::atan2(2.0f * (q1 * q2 + q0 * q3),
                                 q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    const float pitch = std::asin(-2.0f * (q1 * q3 - q0 * q2));
    const float roll = std::atan2(2.0f * (q2 * q3 + q0 * q1),
                                  q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

    // ★ rad：直接
    state->attitude_rad.roll = roll;
    state->attitude_rad.pitch = pitch;
    state->attitude_rad.yaw = yaw;

    // ★ deg：乘 57.3
    state->attitude_deg.roll = roll * kRad2Deg;
    state->attitude_deg.pitch = pitch * kRad2Deg;
    state->attitude_deg.yaw = yaw * kRad2Deg;

    // ★ 角速率（透传自 IMU gyro，机体系 rad/s）
    if (gyro_rads)
    {
        state->gyro_rate = *gyro_rads;
    }
    else
    {
        state->gyro_rate = {0.0f, 0.0f, 0.0f};
    }

    // ★ 估计质量：当前恒 100
    state->estimate_quality = 100;
}

// ============================================================================
//  kalmanCoreIsWithinBounds
// ============================================================================

bool kalmanCoreIsWithinBounds(const kalmanCoreData_t *kf)
{
    if (!kf)
        return false;

    // 与 kalman_supervisor.c 一致
    constexpr float kMaxPosition = 100.0f; // m
    constexpr float kMaxVelocity = 10.0f;  // m/s

    for (int i = 0; i < 3; ++i)
    {
        const float pos = kf->S[KC_STATE_X + i];
        const float vel = kf->S[KC_STATE_PX + i];

        if (pos > kMaxPosition || pos < -kMaxPosition)
            return false;
        if (vel > kMaxVelocity || vel < -kMaxVelocity)
            return false;
    }

    return true;
}

void kalmanCoreUpdateWithTof(
    kalmanCoreData_t *kf,
    const tofMeasurement_t *tof)
{
    if (!kf || !tof)
        return;

    // 先检查测量与姿态几何条件，避免除以接近零的 R[2][2]。
    if (!std::isfinite(tof->distance) ||
        !std::isfinite(tof->stdDev) ||
        tof->distance <= 0.0f ||
        tof->stdDev <= 0.0f)
        return;

    if (!std::isfinite(kf->R[2][2]) ||
        kf->R[2][2] <= 0.1f)
        return;

    const float predictedDistance =
        -kf->S[KC_STATE_Z] / kf->R[2][2];

    // 只有建立过有效高度估计后，才使用残差门限。
    if (kf->tof_initialized &&
        std::fabs(tof->distance - predictedDistance) > 0.5f)
        return;

    float H[KC_STATE_DIM] = {0};
    H[KC_STATE_Z] = -1.0f / kf->R[2][2];

    const float error =
        tof->distance - predictedDistance;

    scalarUpdate(kf, H, error, tof->stdDev);

    kf->tof_initialized = true;
}

// ============================================================================
//  kalmanCoreUpdateWithMag
//    磁力计 yaw 观测。只更新 D2（yaw 误差），不碰 roll/pitch。
//
//    原理：
//      世界系磁场恒定（地磁）；机体系磁场随飞机旋转。
//      正确姿态下：R · m_body = m_world_ref（上电瞬间的世界系磁场）
//      yaw 估计漂了 Δ ⇒ atan2(m_world) 也偏 ⇒ 用这个差修正 D2。
//
//    只在水平时用（倾斜 > 30° 时投影不可靠）。
// ============================================================================

void kalmanCoreUpdateWithMag(kalmanCoreData_t *kf, const magMeasurement_t *mag)
{
    if (!kf || !mag)
        return;

    // 只在水平时用（R[2][2] = cos(tilt)）
    if (kf->R[2][2] < 0.866f)
        return; // cos(30°)

    // 机体系磁场 → 世界系
    const float mx = kf->R[0][0] * mag->mag_body.x + kf->R[0][1] * mag->mag_body.y + kf->R[0][2] * mag->mag_body.z;
    const float my = kf->R[1][0] * mag->mag_body.x + kf->R[1][1] * mag->mag_body.y + kf->R[1][2] * mag->mag_body.z;

    // 强场保护：水平分量太小时跳过（磁力计无效 / 强干扰）
    if (sqrtf(mx * mx + my * my) < 1.0f)
        return;

    const float yaw_now = atan2f(my, mx);

    if (!kf->mag_initialized)
    {
        if (kf->R[2][2] > 0.95f)
        { // ← 只在姿态基本收敛时锁定
            kf->mag_ref_yaw = yaw_now;
            kf->mag_initialized = true;
        }
        return;
    }

    // yaw 误差：期望=ref，实际=now
    float err = kf->mag_ref_yaw - yaw_now;
    // 归一化到 [-π, π]
    while (err > (float)M_PI)
        err -= 2.0f * (float)M_PI;
    while (err < -(float)M_PI)
        err += 2.0f * (float)M_PI;

    float H[KC_STATE_DIM] = {0};
    H[KC_STATE_D2] = 1.0f;

    scalarUpdate(kf, H, err, mag->stdDev);
}

// ============================================================================
//  kalmanCoreUpdateWithFlow
//    二维水平速度观测（光流）。Crazyflie 公式 + NED 3 处适配。
//      · z_g 由 S[Z] 改为 -S[Z]（离地高度，正数）
//      · hx[Z] / hy[Z] 的符号翻转
//      · 其余（Npix / thetapix / omegaFactor）与 Crazyflie 一致
// ============================================================================

void kalmanCoreUpdateWithFlow(kalmanCoreData_t *kf,
                              const flowMeasurement_t *flow,
                              const Axis3f *gyro)
{
    if (!kf || !flow || !gyro)
        return;
    if (flow->dt <= 0.0f)
        return;

    // 相机常数（与 Crazyflie 一致，按 PMW3901 视场角可后续标定）
    constexpr float kNpix = 30.0f;
    constexpr float kThetapix = 0.0174532925f * 4.2f; // rad
    constexpr float kOmegaFactor = 1.25f;

    // ★ NED：离地高度（正数）
    float z_g = -kf->S[KC_STATE_Z];
    if (z_g < 0.1f)
        z_g = 0.1f; // 地面附近饱和，避免除零

    const float omega_x = gyro->x; // rad/s
    const float omega_y = gyro->y;

    // 机体系水平速度（Crazyflie 里 S[PX/PY] 是机体系）
    const float dx_b = kf->S[KC_STATE_PX];
    const float dy_b = kf->S[KC_STATE_PY];

    const float scale = kNpix * flow->dt / kThetapix;

    // ===== X 轴 =====
    {
        float H[KC_STATE_DIM] = {0};

        const float predictedNX =
            scale * ((dx_b * kf->R[2][2] / z_g) - kOmegaFactor * omega_y);
        const float measuredNX = flow->dpixelx;

        // ★ NED：hx[Z] 正号（Crazyflie 是负号）
        H[KC_STATE_Z] = scale * (kf->R[2][2] * dx_b) / (z_g * z_g);
        H[KC_STATE_PX] = scale * (kf->R[2][2] / z_g);

        scalarUpdate(kf, H, measuredNX - predictedNX, flow->stdDevX);
    }

    // ===== Y 轴 =====
    {
        float H[KC_STATE_DIM] = {0};

        const float predictedNY =
            scale * ((dy_b * kf->R[2][2] / z_g) + kOmegaFactor * omega_x);
        const float measuredNY = flow->dpixely;

        // ★ NED：hy[Z] 正号
        H[KC_STATE_Z] = scale * (kf->R[2][2] * dy_b) / (z_g * z_g);
        H[KC_STATE_PY] = scale * (kf->R[2][2] / z_g);

        scalarUpdate(kf, H, measuredNY - predictedNY, flow->stdDevY);
    }
}

// ============================================================================
//  kalmanCoreResetHorizontal
//    解锁瞬间调用：把水平位置/速度清零，姿态与高度不动。
//
//    为什么只清零不重置全部：EKF 水平状态本身没有绝对参考（无 GPS），
//    上电到解锁期间会因加计零偏残差、光流噪声缓慢漂移。清零让位置环
//    的 (0,0) 与"解锁瞬间的物理位置"对齐。
//
//    协方差处理：X/Y 设为中度不确定（1 m²，光流几秒内收紧到 cm 级）；
//    X/Y 与其他状态的交叉协方差清零，避免旧的错误耦合。
// ============================================================================

void kalmanCoreResetHorizontal(kalmanCoreData_t *kf)
{
    if (!kf)
        return;

    // ---- 状态清零 ----
    kf->S[KC_STATE_X] = 0.0f;
    kf->S[KC_STATE_Y] = 0.0f;
    kf->S[KC_STATE_PX] = 0.0f;
    kf->S[KC_STATE_PY] = 0.0f;

    // ---- 交叉协方差清零（X/Y/PX/PY 与其它所有状态）----
    const int horiz_idx[4] = {KC_STATE_X, KC_STATE_Y, KC_STATE_PX, KC_STATE_PY};
    for (int k = 0; k < 4; ++k)
    {
        const int i = horiz_idx[k];
        for (int j = 0; j < KC_STATE_DIM; ++j)
        {
            if (j == KC_STATE_X || j == KC_STATE_Y ||
                j == KC_STATE_PX || j == KC_STATE_PY)
                continue;
            kf->P[i][j] = 0.0f;
            kf->P[j][i] = 0.0f;
        }
    }

    // ---- 水平状态自协方差：中度不确定 ----
    kf->P[KC_STATE_X][KC_STATE_X] = 1.0f; // 1 m² (std 1m)
    kf->P[KC_STATE_Y][KC_STATE_Y] = 1.0f;
    kf->P[KC_STATE_PX][KC_STATE_PX] = 0.01f;
    kf->P[KC_STATE_PY][KC_STATE_PY] = 0.01f;

    // 水平位置-速度交叉：清（新原点，二者无先验相关性）
    kf->P[KC_STATE_X][KC_STATE_PX] = 0.0f;
    kf->P[KC_STATE_PX][KC_STATE_X] = 0.0f;
    kf->P[KC_STATE_X][KC_STATE_PY] = 0.0f;
    kf->P[KC_STATE_PY][KC_STATE_X] = 0.0f;
    kf->P[KC_STATE_Y][KC_STATE_PX] = 0.0f;
    kf->P[KC_STATE_PX][KC_STATE_Y] = 0.0f;
    kf->P[KC_STATE_Y][KC_STATE_PY] = 0.0f;
    kf->P[KC_STATE_PY][KC_STATE_Y] = 0.0f;
}