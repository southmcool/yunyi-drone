/**
 * @title  Kalman 核心（9 维 EKF）
 * @author: vaie
 * @date: 2026-10-12
 *
 * kalman_core.h — 基于 Crazyflie kalman_core 的全状态 EKF（9 维）。
 *
 * 论文：
 *   Mueller 2015 (ICRA) "Fusing UWB range measurements with accelerometers
 *     and rate gyroscopes for quadrocopter state estimation"
 *   Mueller 2016 (AIAA) "Covariance Correction Step for Kalman Filtering
 *     with an Attitude"
 *
 * 状态向量（9 维，NED，SI 单位）—— 与 Crazyflie 一致：
 *   S[KC_STATE_X  .. Z ]   位置 [m]，世界系 NED
 *   S[KC_STATE_PX .. PZ]   速度 [m/s]，**机体系**
 *   S[KC_STATE_D0 .. D2]   姿态误差 [rad]（参考四元数 → 真实姿态）
 *
 * 说明：
 *   · 加计/陀螺零偏不在此 EKF 状态里 —— 由 IMU 校准层（ImuCalibration）
 *     静态扣除。Crazyflie 的 9 维 kalman_core 也是这么做的。
 *   · 姿态用「参考四元数 q + 误差角 D」的写法，不用四元数直接进 EKF。
 *     每轮 finalize 把 D 搬进 q，D 归零（Mueller 2016）。
 *   · 一期只接 IMU + 气压。TOF/Flow/Position 等测量接口后续分期加入。
 *   · 无平台依赖（无 Arduino / FreeRTOS / esp_timer），SIL 与实机共用。
 *
 * 依赖：type/imu_types.h（Axis3f）+ type/stabilizer_types.h（state_t）
 */
#pragma once

#include <cstdint>
#include "type/imu_types.h"        // Axis3f
#include "type/stabilizer_types.h" // state_t
#include "util/boot_log.h"
// ============================================================================
//  状态向量索引（与 Crazyflie 一致）
// ============================================================================

enum
{
    KC_STATE_X = 0, // 位置 [m]，世界系 NED
    KC_STATE_Y = 1,
    KC_STATE_Z = 2,

    KC_STATE_PX = 3, // 速度 [m/s]，机体系
    KC_STATE_PY = 4,
    KC_STATE_PZ = 5,

    KC_STATE_D0 = 6, // 姿态误差 [rad]
    KC_STATE_D1 = 7,
    KC_STATE_D2 = 8,

    KC_STATE_DIM = 9,
};

// ============================================================================
//  常量（与 Crazyflie physicalConstants.h / kalman_core.c 一致）
// ============================================================================

namespace kalman
{
    constexpr float kGravity = 9.81f; // m/s²
    constexpr float kMaxCovariance = 100.0f;
    constexpr float kMinCovariance = 1e-6f;

    // // 地面时姿态向初始姿态缓拉回，防止陀螺零偏累积
    // constexpr float kRollPitchZeroReversion = 0.001f;
}

// ============================================================================
//  内部状态（9 维）
// ============================================================================

struct kalmanCoreData_t
{
    // ---- 状态向量与协方差 ----
    float S[KC_STATE_DIM];
    float P[KC_STATE_DIM][KC_STATE_DIM];

    // ---- 参考姿态 ----
    float q[4];    // (w, x, y, z)，由 D 扰动
    float R[3][3]; // 由 q 算出的旋转矩阵（body→world）

    // ---- 标志 ----
    bool resetEstimation;      // 请求重初始化
    float baroReferenceHeight; // 气压零基准 [m]
    uint64_t lastUpdateUs;     // 上次 finalize 时刻 [µs]
    // ★ 新增：磁力计参考
    bool mag_initialized; // 是否已记录"上电瞬间的世界系磁力方向"
    float mag_ref_yaw;    // 参考方向 [rad]
    bool tof_initialized;
};

// ============================================================================
//  核心接口（一期）
// ============================================================================

void kalmanCoreInit(kalmanCoreData_t *kf);

// ---- 预测：用 IMU 数据把状态推进 dt ----
void kalmanCorePredict(kalmanCoreData_t *kf,
                       float thrust_norm,
                       const Axis3f *acc,
                       const Axis3f *gyro,
                       float dt,
                       bool quadIsFlying);

// ---- 过程噪声累积（与 Predict 同频或更高频调用）----
void kalmanCoreAddProcessNoise(kalmanCoreData_t *kf, float dt);

// ---- 气压校正（一维高度观测）----
void kalmanCoreUpdateWithBaro(kalmanCoreData_t *kf,
                              float baroAsl,
                              bool quadIsFlying);

// ---- 收尾：把姿态误差 D 搬进参考四元数（Mueller 2016 协方差旋转）----
void kalmanCoreFinalize(kalmanCoreData_t *kf, uint64_t osTickUs);

// ---- 外部化：内部状态 → state_t（速度转世界系、Euler 输出、acc 转世界系）----
void kalmanCoreExternalizeState(const kalmanCoreData_t *kf,
                                state_t *state,
                                const Axis3f *acc,
                                const Axis3f *gyro_rads, // ★ 新增
                                uint64_t osTickUs);

// ---- 边界检查：S / P 是否合理（不符则调用方重初始化）----
bool kalmanCoreIsWithinBounds(const kalmanCoreData_t *kf);

// TOF（一维高度观测）
void kalmanCoreUpdateWithTof(kalmanCoreData_t *kf,
                             const tofMeasurement_t *tof);

// 磁力计 yaw 观测
void kalmanCoreUpdateWithMag(kalmanCoreData_t *kf, const magMeasurement_t *mag);

// 光流（二维速度观测）
void kalmanCoreUpdateWithFlow(kalmanCoreData_t *kf,
                              const flowMeasurement_t *flow,
                              const Axis3f *gyro);

void kalmanCoreResetHorizontal(kalmanCoreData_t *kf);