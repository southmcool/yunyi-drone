#include "core/imu/imu_calibration.h"
#include <cmath>

void ImuCalibrator::reset()
{
    for (int i = 0; i < 3; ++i)
    {
        sum_gyro_->axis[i] = 0;
        sum_accel_->axis[i] = 0;
        sum_gyro_sq_->axis[i] = 0;
        sum_accel_sq_->axis[i] = 0;
    }
    n_ = 0;
}

void ImuCalibrator::addSample(const ImuRawSample &raw)
{
    for (int i = 0; i < 3; ++i)
    {
        const int64_t g = raw.gyro.axis[i];
        const int64_t a = raw.acc.axis[i];
        sum_gyro_->axis[i] += g;
        sum_accel_->axis[i] += a;
        sum_gyro_sq_->axis[i] += g * g;
        sum_accel_sq_->axis[i] += a * a;
    }
    ++n_;
}

void ImuCalibrator::finish(ImuCalibration *calib,
                           float *gyro_stddev_dps,
                           float *accel_stddev_g) const
{
    if (!calib)
        return;

    if (n_ == 0)
    {
        calib->gyro_offset = Axis3i16{};
        calib->accel_offset = Axis3i16{};
        if (gyro_stddev_dps)
            gyro_stddev_dps[0] = gyro_stddev_dps[1] = gyro_stddev_dps[2] = 0.0f;
        if (accel_stddev_g)
            accel_stddev_g[0] = accel_stddev_g[1] = accel_stddev_g[2] = 0.0f;
        return;
    }

    // ---- 陀螺零偏 = 均值（int16 LSB 域）----
    //   ★ 与 ImuSignalChain 的换算一致：
    //     gyro_物理值 = (raw.gyro - calib.gyro_offset) / 16.384
    //   ⇒ calib.gyro_offset = mean(raw.gyro)
    for (int i = 0; i < 3; ++i)
    {
        calib->gyro_offset.axis[i] = (int16_t)(sum_gyro_->axis[i] / (int64_t)n_);
    }

    // ---- 加计零偏（★ 新增）----
    //   假设校准期间机身**水平静止**：
    //     理论 NED 机体系读数 = (0, 0, -1) g
    //     LSB 域 = (0, 0, -4096)
    //   offset = 实测均值 - 理论值
    //   → chain 里 `(raw - offset) / 4096` 就得到 (0, 0, -1)
    //
    //   ★ 前提：校准时机身水平。否则会"吃掉"安装倾角（短期可接受，
    //     长期若接光流需要更精细的六面校准）。
    constexpr int32_t kTarget[3] = {0, 0, -4096};
    for (int i = 0; i < 3; ++i)
    {
        const int32_t mean = (int32_t)(sum_accel_->axis[i] / (int64_t)n_);
        int32_t off = mean - kTarget[i];
        if (off > 32000)
            off = 32000;
        if (off < -32000)
            off = -32000;
        calib->accel_offset.axis[i] = (int16_t)off;
    }

    // ---- 诊断：stddev ----
    //   var = E[x²] - (E[x])²
    if (gyro_stddev_dps)
    {
        for (int i = 0; i < 3; ++i)
        {
            const double mean = (double)sum_gyro_->axis[i] / (double)n_;
            const double var = (double)sum_gyro_sq_->axis[i] / (double)n_ - mean * mean;
            const double sd = (var > 0.0) ? std::sqrt(var) : 0.0;
            gyro_stddev_dps[i] = (float)(sd / 16.384); // ±2000dps 档
        }
    }
    if (accel_stddev_g)
    {
        for (int i = 0; i < 3; ++i)
        {
            const double mean = (double)sum_accel_->axis[i] / (double)n_;
            const double var = (double)sum_accel_sq_->axis[i] / (double)n_ - mean * mean;
            const double sd = (var > 0.0) ? std::sqrt(var) : 0.0;
            accel_stddev_g[i] = (float)(sd / 4096.0);
        }
    }
}