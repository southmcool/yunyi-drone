/**
 * @title  IMU 静态校准（累加器）
 * @author: vaie
 * @date: 2026-10-12
 *
 * imu_calibration.h — 静止采集 → 计算陀螺/加计零偏。
 *
 * 使用（阻塞采 3s）：
 *     ImuCalibrator cal;
 *     cal.reset();
 *     while (采样未结束) {
 *         if (mpu.readRaw(&raw)) cal.addSample(raw);
 *     }
 *     ImuCalibration calib{};
 *     float stddev[3];
 *     cal.finish(&calib, stddev);
 *     chain.setCalibration(calib);
 *
 * 一期只校陀螺零偏；加计零偏暂置 0（真机有安装倾角，
 * 简单静止平均不可靠——留给后续六面校准或 EKF 融合）。
 *
 * 纯算法，无平台依赖（<cmath>/<cstdint>），SIL 与实机共用。
 */
#pragma once

#include <cstdint>
#include "type/imu_types.h" // ImuRawSample / ImuCalibration / Axis3i16

class ImuCalibrator
{
public:
    ImuCalibrator() = default;

    // 清零累加器
    void reset();

    // 累加一个原始采样
    void addSample(const ImuRawSample &raw);

    // 已累加样本数
    uint32_t sampleCount() const { return n_; }

    // 是否够用（默认 1000 个样本 ≈ 1s @1kHz）
    bool hasEnough(uint32_t min_samples = 1000) const { return n_ >= min_samples; }

    // 计算最终结果
    //   calib             ：输出，填 gyro_offset（int16 LSB 域）
    //   gyro_stddev_dps   ：可选诊断，输出陀螺噪声 stddev [°/s]
    //   accel_stddev_g    ：可选诊断，输出加计噪声 stddev [g]
    void finish(ImuCalibration *calib,
                float *gyro_stddev_dps = nullptr,
                float *accel_stddev_g = nullptr) const;


private:
    // 累加器（int64 防溢出：int16 × 10⁵ 采样）
    Axis3i64 sum_gyro_[3] = {0};
    Axis3i64 sum_accel_[3] = {0};
    Axis3i64 sum_gyro_sq_[3] = {0};
    Axis3i64 sum_accel_sq_[3] = {0};
    uint32_t n_ = 0;
};