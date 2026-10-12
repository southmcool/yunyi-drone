/**
 * @title  IMU 信号链实现
 * @author: vaie
 * @date: 2026-10-12
 *
 * 从 Mpu6500::initSignalChain + Mpu6500::processRawSample 机械搬迁，
 * 保持所有滤波/时序行为不变。原 R37/R58/R76 的注释一并保留。
 *
 * 本文件不依赖 Arduino / ESP-IDF / FreeRTOS 头（除诊断打印分支外），
 * SIL 与实机共用同一份实现。
 */

#include "core/imu/imu_signal_chain.h"

#include <cmath>
#include <cstring>

#if defined(SIL_HOST)
#include <cstdio>
#include <cstdlib> // getenv / atof / atoi
#else
#include <Arduino.h>
#endif

// =========================================================================
//  init —— 配置解析与滤波器初始化（原 Mpu6500::initSignalChain）
// =========================================================================
void ImuSignalChain::init(const ImuVibConfig &cfg,
                          const ImuCalibration &calib,
                          const ImuScale &scale,
                          float fs_hz)
{
    calib_ = calib;
    fs_hz_ = (fs_hz > 1.0f) ? fs_hz : 1000.0f;

    // ---- 配置解析：三层优先级（后写覆盖前写）----
    //   ① 传入 cfg（真机默认值来自 ImuVibConfig.h）
    //   ② SIL：显式重置为"历史默认"（保证已注册基线逐位不变）
    //   ③ SIL_* env 覆盖（A/B 注入）
    ImuVibConfig resolved = cfg;
#ifdef SIL_HOST
    // ★ R58：SIL 历史默认 —— 必须逐项对齐 R57 及之前的硬编码值。
    //   五个字段与 ImuVibConfig 真机默认不同，必须显式覆盖：
    //     · notch_base_hz ：真机 300 → SIL 历史 473（旧注释称"写死的猜测值"）
    //     · notch_adapt   ：真机 true → SIL 历史 false
    //     · vdc.tau_s     ：真机 0.03 → SIL 历史 1.0（★ 见下）
    //     · vdc.fnorm     ：真机 true → SIL 历史 false
    //     · vdc.fref_hz   ：真机 200 → SIL 历史 300
    //   其余字段真机与 SIL 历史一致，但仍逐项写出以防将来漂移。
    //
    //   ⚠ vdc.tau_s 的坑：历史 A/B 臂里有只给 SIL_VDC_K 不给 SIL_VDC_TAU 的，
    //     不回填 1.0f 就会把它们的 a_rms EMA 从 1.0s 悄悄改成 30ms。
    resolved.notch_base_hz = 473.0f;
    resolved.notch_q = 3.5f;
    resolved.notch_harmonics = 3;
    resolved.accel_lp_hz = 30.0f;
    resolved.gyro_lp_hz = 0.0f;
    resolved.notch_adapt = false;
    resolved.vdc.kx = resolved.vdc.ky = resolved.vdc.kz = 0.0f;
    resolved.vdc.tau_s = 1.0f;
    resolved.vdc.fnorm = false;
    resolved.vdc.fref_hz = 300.0f;
    resolved.vdc.dlpf_fc_hz = 460.0f;

    // env 覆盖
    if (const char *v = std::getenv("SIL_NOTCH_BASE"))
        resolved.notch_base_hz = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_NOTCH_Q"))
        resolved.notch_q = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_NOTCH_HARMONICS"))
        resolved.notch_harmonics = std::atoi(v);
    if (const char *v = std::getenv("SIL_ACCEL_LP"))
        resolved.accel_lp_hz = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_GYRO_LP"))
        resolved.gyro_lp_hz = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_NOTCH_ADAPT"))
        resolved.notch_adapt = (std::atoi(v) != 0);
    if (const char *v = std::getenv("SIL_ADAPT_FMIN"))
        resolved.adapt_fmin_hz = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_ADAPT_FMAX"))
        resolved.adapt_fmax_ratio = (float)std::atof(v) / fs_hz_;
    if (const char *v = std::getenv("SIL_ADAPT_TAU"))
        resolved.adapt_tau_s = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_VDC_K"))
    {
        const float k = (float)std::atof(v);
        resolved.vdc.kx = k;
        resolved.vdc.ky = k;
    }
    if (const char *v = std::getenv("SIL_VDC_KX"))
        resolved.vdc.kx = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_VDC_KY"))
        resolved.vdc.ky = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_VDC_KZ"))
        resolved.vdc.kz = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_VDC_TAU"))
        resolved.vdc.tau_s = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_VDC_FNORM"))
        resolved.vdc.fnorm = (std::atoi(v) != 0);
    if (const char *v = std::getenv("SIL_VDC_FREF"))
        resolved.vdc.fref_hz = (float)std::atof(v);
    if (const char *v = std::getenv("SIL_VDC_DLPF_FC"))
        resolved.vdc.dlpf_fc_hz = (float)std::atof(v);
#endif
    cfg_ = resolved;

    // ---- 量程换算因子 + 预计算 ----
    setScale(scale);

    // ---- 滤波器初始化 ----
    filter_.init(fs_hz_);

    // VRE 前馈（加计振动整流 DC 补偿）
    vibdc_.init(cfg_.vdc, fs_hz_);

    // 加计低通（30Hz）：进 EKF 前压噪，避免二次积分漂移
    filter_.initAccelLowPass(fs_hz_, cfg_.accel_lp_hz > 0.0f ? cfg_.accel_lp_hz : 30.0f);
    filter_.setAccelLowPassEnabled(cfg_.accel_lp_hz > 0.0f);

    // 陀螺低通（默认关：会给角速度内环引入相位滞后）
    if (cfg_.gyro_lp_hz > 0.0f)
        filter_.initGyroLowPass(fs_hz_, cfg_.gyro_lp_hz);
    filter_.setGyroLowPassEnabled(cfg_.gyro_lp_hz > 0.0f);

    // 分轴陷波（当前未接入主链，保留与旧代码一致的初始化）
    float accel_freq[3] = {235, 235, 247};
    float gyro_freq[3] = {235, 235, 247};
    float Q_arr[3] = {2.5, 2.5, 2.5};
    motor_notch_filter_.initPerAxis(fs_hz_, accel_freq, gyro_freq);
    motor_notch_filter_.updatePerAxisQ(Q_arr, Q_arr);

    // 谐波陷波（真机主力）
    const float base_freq = cfg_.notch_base_hz;
    const float harmonic_q = cfg_.notch_q;
    const int harmonics = cfg_.notch_harmonics;
    if (base_freq > 0.0f && harmonics > 0)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            harmonic_notch_accel_[axis].init(fs_hz_, base_freq, harmonics, harmonic_q);
            harmonic_notch_gyro_[axis].init(fs_hz_, base_freq, harmonics, harmonic_q);
        }
    }

    // 自适应陷波（在线测频 → 重调基频）
    notch_adapt_ = cfg_.notch_adapt;
    notch_applied_f_ = base_freq;
    if (notch_adapt_)
    {
        const float fmin = cfg_.adapt_fmin_hz;
        const float fmax = cfg_.adapt_fmax_ratio * fs_hz_;
        const float tau = cfg_.adapt_tau_s;
        spec_tracker_.init(fs_hz_, 1024, fmin, fmax, tau);
    }
}

// =========================================================================
//  setScale —— 运行时切换量程（低频事件）
// =========================================================================
void ImuSignalChain::setScale(const ImuScale &scale)
{
    scale_ = scale;
    // 预计算乘法因子：1kHz 循环里避免做除法
    inv_accel_scale_ = 1.0f / scale.accel_lsb_per_g;
    inv_gyro_scale_ = 1.0f / scale.gyro_lsb_per_dps;
}

void ImuSignalChain::setCalibration(const ImuCalibration &calib)
{
    calib_ = calib;
}

// =========================================================================
//  process —— 主信号链（原 Mpu6500::processRawSample）
// =========================================================================
bool ImuSignalChain::process(const ImuRawSample &raw, ImuData *out)
{
    if (!out)
        return false;

    // ---- 1. 校准换算 ----
    //   顺序与旧 getAccelX/getGyroX 完全一致：先扣偏移，再乘 1/scale。
    for (int i = 0; i < 3; ++i)
    {
        // 改成
        out->acc.axis[i] = (float)(raw.acc.axis[i] - calib_.accel_offset.axis[i]) * inv_accel_scale_ // 1/4096，LSB → g
                           * calib_.accel_scale.axis[i];                                             // 灵敏度修正
        out->gyro.axis[i] = (float)(raw.gyro.axis[i] - calib_.gyro_offset.axis[i]) * inv_gyro_scale_;
    }
    out->temperature = raw.temperature / scale_.temp_lsb_per_deg + scale_.temp_offset_deg;
    out->timestamp_us = raw.timestamp_us;

    // ---- 2. 保存校准后但未做坐标变换的值（诊断用）----
    out->accRaw = out->acc;
    out->gyroRaw = out->gyro;

    // ---- 3. 坐标变换（芯片朝向 → 机体系 NED）----
    //   纯函数：加速度与陀螺走同一变换矩阵，调用两次。
    out->acc = toBodyFrame(orientation_, out->acc);
    out->gyro = toBodyFrame(orientation_, out->gyro);

    // ---- 4. VRE 前馈 ①：陷波/低通前测加计交流 RMS ----
    //   陷波会把激励频率削掉，放到后面测就测不到 a_rms 了。
    vibdc_.updateRaw(out->acc.axis);

    // ---- 5. VDCCAL 真机台架标定出口（默认关，cfg_.vdc.calib_log 控制）----
    if (cfg_.vdc.calib_log)
    {
#if !defined(SIL_HOST)
        static uint32_t cal_tick = 0;
        if ((cal_tick++ % 200u) == 0u)
        {
            Serial.printf("[VDCCAL] mean=(%.4f,%.4f,%.4f)g arms=%.4f arms2=%.5f k_scale=%.4f\n",
                          vibdc_.mean(0), vibdc_.mean(1), vibdc_.mean(2),
                          vibdc_.arms(), vibdc_.arms() * vibdc_.arms(), vibdc_.kScale());
        }
#endif
    }

    // ---- 6. 自适应测频：在陷波前喂测频器（PX4 gyro_fft 同构）----
    //   ★ 必须在陷波前——若喂陷波后的信号，窄带陷波把激励频率削掉 ⇒ 自反馈失锁。
    //   ★ R58 两段式：本处只做 O(1) 入窗；FFT 由 processPendingFft() 跑。
    //     SIL 侧在 process() 尾部内联；真机由 imu_fft 任务轮询。
    if (notch_adapt_)
    {
        spec_tracker_.stage(out->gyro.axis);

        if (spec_tracker_.ready())
        {
            const float f_new = spec_tracker_.freq();
            vibdc_.setFrequency(f_new);

            // ★ R76 重调持续性门：单帧过门即重调会被地面假峰拖走。
            const bool retune_ok =
                spec_tracker_.histLen() >= (uint32_t)cfg_.adapt_min_hist &&
                spec_tracker_.recentValidFrac() >= cfg_.adapt_min_valid_frac;
            if (retune_ok)
                applyAdaptiveFreq(f_new); // 内含 2Hz 滞回
        }
    }

    // ---- 7. 谐波陷波（2×3 = 6 路）----
    for (int i = 0; i < 3; ++i)
    {
        out->acc.axis[i] = harmonic_notch_accel_[i].apply(out->acc.axis[i]);
        out->gyro.axis[i] = harmonic_notch_gyro_[i].apply(out->gyro.axis[i]);
    }

    // ---- 8. 加计低通（30Hz，陷波之后）----
    filter_.applyAccelLowPass(out);

    // ---- 9. 陀螺低通（默认关）----
    filter_.applyGyroLowPass(out);

    // ---- 10. VRE 前馈 ②：滤波后扣掉前馈 DC ----
    vibdc_.compensate(out->acc.axis);

    // ---- 11. 幅值 ----
    out->accel_magnitude = std::sqrt(out->acc.x * out->acc.x +
                                     out->acc.y * out->acc.y +
                                     out->acc.z * out->acc.z);

#if defined(SIL_HOST)
    // SIL 侧：FFT 就地跑（非硬实时）
    processPendingFft();
#endif

    return true;
}

// =========================================================================
//  采样率对齐（R37）
// =========================================================================
void ImuSignalChain::setActualSampleRate(float hz)
{
    if (!(hz > 1.0f))
        return;
    // 死区 0.1Hz：避免每拍做 6×3 组三角函数
    if (fs_hz_ > 1.0f && hz > fs_hz_ - 0.1f && hz < fs_hz_ + 0.1f)
        return;
    fs_hz_ = hz;

    filter_.setSampleRate(hz);
    motor_notch_filter_.updateSampleFrequency(hz);
    for (int axis = 0; axis < 3; ++axis)
    {
        harmonic_notch_accel_[axis].updateSampleFrequency(hz);
        harmonic_notch_gyro_[axis].updateSampleFrequency(hz);
    }
    vibdc_.setSampleRate(hz);
    spec_tracker_.setSampleRate(hz);
}

// =========================================================================
//  自适应重调（R58）
// =========================================================================
void ImuSignalChain::applyAdaptiveFreq(float f_new)
{
    if (std::fabs(f_new - notch_applied_f_) <= 2.0f)
        return; // 2Hz 滞回
    for (int axis = 0; axis < 3; ++axis)
    {
        harmonic_notch_gyro_[axis].updateBaseFrequency(f_new);
        harmonic_notch_accel_[axis].updateBaseFrequency(f_new);
    }
    notch_applied_f_ = f_new;
}

// =========================================================================
//  自适应测频——FFT 执行入口（两段式接口）
// =========================================================================
void ImuSignalChain::processPendingFft()
{
    if (notch_adapt_)
        spec_tracker_.processPending();
}

bool ImuSignalChain::fftReady() const
{
    return notch_adapt_ && spec_tracker_.ready();
}

float ImuSignalChain::notchTrackFreq() const { return spec_tracker_.freq(); }
float ImuSignalChain::notchTrackRatio() const { return spec_tracker_.peakRatio(); }