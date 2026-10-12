/**
 * @title  Kalman 估计器（单例封装）
 * @author: vaie
 * @date: 2026-10-12
 *
 * estimator_kalman.h — 包装 kalmanCore，提供：
 *   · 单例内部状态（唯一 kalmanCoreData_t）
 *   · 单写者：estimator_task（250Hz 主循环）
 *   · 多读者：controller / telemetry（无锁 Slot）
 *
 * 与 kalmanCore 的分工：
 *   · kalmanCore    —— 纯算法，无状态封装，无并发
 *   · 本文件        —— 生命周期 + 并发 + 对外接口
 *
 * 线程模型：
 *   · predict / updateBaro / finalize / publishState → estimator_task 独调
 *   · readState                                     → 任意任务，无锁
 *   · requestReset                                  → 任意任务，置标志
 *     estimator_task 在下一拍消费
 *
 * 依赖：kalman_core.h + type/stabilizer_types.h
 */
#pragma once

#include <cstdint>
#include "type/imu_types.h"        // Axis3f
#include "type/stabilizer_types.h" // state_t

class EstimatorKalman
{
public:
    EstimatorKalman() = delete;

    // ---- 生命周期 ----
    //   清零 g_kf，置 P 初值，重置 Slot。
    //   ★ 只在 estimator_task 里调（或 setup 时单线程调）。
    static void init();

    // ---- 单步操作（只在 estimator_task 里调）----

    // 预测：用 IMU 数据把状态推进 dt
    static void predict(float dt,
                        const Axis3f &acc,
                        const Axis3f &gyro,
                        bool quadIsFlying);

    // 气压校正（低频率调，如 30Hz）
    static void updateBaro(float baroAsl, bool quadIsFlying);

    // 收尾：把姿态误差搬进参考四元数（每拍 predict 后调一次）
    static void finalize(uint64_t osTickUs);

    // 外部化 + 发布到 Slot（供 readState 读）
    static void publishState(const Axis3f &acc, const Axis3f &gyro_rads, uint64_t osTickUs);

    // ---- 状态读取（多读者）----
    //   返回 true 表示拿到未撕裂的快照
    static bool readState(state_t *out);

    // ---- 诊断 / 复位 ----
    static bool isWithinBounds();

    //   请求复位（任意任务可调）
    static void requestReset();
    //   消费复位请求（estimator_task 每拍调一次）
    //   返回 true 表示本拍执行了复位
    static bool consumeResetRequest();

    //   当前参考四元数（诊断用）
    static bool readQuaternion(float q[4]);

    // 测量更新（只在 estimator_task 里调）
    static void updateTof(const tofMeasurement_t &tof);
    static void updateFlow(const flowMeasurement_t &flow, const Axis3f &gyro_rads);
    // 解锁瞬间调用：水平位置清零（姿态/高度不动）
    static void resetHorizontal();

    static void updateMag(const magMeasurement_t &mag);

private:
    static void internalReset();
};
