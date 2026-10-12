/**
 * @title  IMU 信号链
 * @author: vaie
 * @date: 2026-10-12
 *
 * imu_signal_chain.h — 从原始 int16 采样到物理 ImuData 的全部处理。
 *
 * 职责：
 *   校准换算 → 坐标变换 → VRE预测量 → 自适应测频 → 谐波陷波
 *   → 加计低通 → 陀螺低通 → VRE补偿 → 幅值
 *
 * 不负责：
 *   SPI/寄存器（Mpu6500Device）、FreeRTOS 任务（task 层）、
 *   持久化（param 层）、启动横幅打印（task 层）。
 *
 * 依赖：
 *   仅 type/imu_types.h + core/imu/imu_orientation.h + 算法组件 + ImuVibConfig。
 *   不依赖任何驱动 / HAL / 平台头。
 *
 * SIL 与实机共用本类；SIL 注入走 process() 的同一路径。
 */
#pragma once

#include <cstdint>

#include "core/imu/imu_orientation.h"    // imu_orientation_t / toBodyFrame
#include "config/imu_vibration_config.h" // ImuVibConfig

// 信号链算法组件（路径按项目实际调整）
#include "filter/SensorFilter.h"
#include "filter/ButterworthFilter3D.h"
#include "filter/NotchFilter3D.h"
#include "filter/HarmonicNotchFilter.h"
#include "filter/VibDcCompensator.h"
#include "filter/SpectralPeakTracker.h"

class ImuSignalChain
{
public:
    ImuSignalChain() = default;
    ImuSignalChain(const ImuSignalChain &) = delete;
    ImuSignalChain &operator=(const ImuSignalChain &) = delete;

    // ---- 生命周期 ----
    //   cfg    ：振动治理配置（真机默认值；SIL env 覆盖由 .cpp 内部处理）
    //   calib  ：当前校准数据
    //   scale  ：当前量程换算因子（通常来自 Mpu6500Device::scale()）
    //   fs_hz  ：名义采样率。真机 1000.0f；SIL 之后由 setActualSampleRate() 矫正。
    void init(const ImuVibConfig &cfg,
              const ImuCalibration &calib,
              const ImuScale &scale,
              float fs_hz);

    // ---- 运行中切换（低频事件）----
    void setScale(const ImuScale &scale);             // 切量程
    void setCalibration(const ImuCalibration &calib); // 地面站下发新校准

    // ---- 主处理 ----
    //   raw ：驱动层的原始 int16（int16 LSB + 温度 + 时间戳）
    //   out ：物理量输出，必填
    //   返回 false 仅当 out == nullptr。
    bool process(const ImuRawSample &raw, ImuData *out);

    // ---- 采样率对齐（R37）----
    //   SIL 里由 sim_main.cc 按实测 fs 调用；真机 fs 恒 1kHz ⇒ 本函数等价空操作。
    //   只重算系数，保留所有滤波器状态（不打瞬态）。
    void setActualSampleRate(float fs_hz);

    // ---- 自适应测频两段式（R58）----
    //   process() 内只做 stage()（O(1)，IMU 上下文）；
    //   processPendingFft() 由外部决定谁调：
    //     SIL  ：process() 内联（非硬实时）
    //     真机 ：低优先级任务 imu_fft 调用（避免 1ms 硬周期里跑 FFT 丢帧）
    void processPendingFft();

    // ---- 查询 ----
    float sampleRate() const { return fs_hz_; }
    const ImuVibConfig &config() const { return cfg_; }
    const ImuCalibration &calibration() const { return calib_; }
    const ImuScale &scale() const { return scale_; }

    // ---- 自适应陷波诊断 ----
    bool fftReady() const;
    float notchTrackFreq() const;
    float notchTrackRatio() const;
    bool notchAdaptive() const { return notch_adapt_; }
    float notchAppliedFreq() const { return notch_applied_f_; }

    // ---- 朝向 ----
    void setOrientation(imu_orientation_t o) { orientation_ = o; }
    imu_orientation_t orientation() const { return orientation_; }

private:
    // 自适应重调：把测频器给出的新基频写进 6 条谐波陷波（2Hz 滞回）。
    //   只在 IMU 上下文调用 ⇒ 与陷波器使用同上下文，无竞态。
    void applyAdaptiveFreq(float f_new);

    // ---- 配置与状态 ----
    ImuVibConfig cfg_;
    ImuCalibration calib_;
    ImuScale scale_;
    float fs_hz_ = 1000.0f;

    // 预计算：避免 1kHz 循环里做除法
    float inv_accel_scale_ = 1.0f / 4096.0f;
    float inv_gyro_scale_ = 1.0f / 16.384f;

    imu_orientation_t orientation_ = ORIENTATION_DEFAULT;

    // ---- 信号链组件 ----
    SensorFilter filter_;                    // 加计 / 陀螺低通
    ButterworthFilter3D butterworth_filter_; // 未接入（保留）
    NotchFilter3D motor_notch_filter_;       // 未接入（保留）
    HarmonicNotchFilter harmonic_notch_accel_[3];
    HarmonicNotchFilter harmonic_notch_gyro_[3];

    VibDcCompensator vibdc_;           // VRE 前馈
    SpectralPeakTracker spec_tracker_; // 自适应测频
    bool notch_adapt_ = false;
    float notch_applied_f_ = 0.0f;
};