#pragma once
// =============================================================================
// VibDcCompensator —— 加计振动整流误差（Vibration Rectification Error, VRE）
//                      前馈补偿（传感器信号调理级，不是估计器机制）
//
// 机理（外部证据）：
//   · ADI《Vibration Rectification in MEMS Accelerometers》：VRE 由敏感元件内部
//     非线性整流出直流偏置，量级  VRE = VRC · Grms²  （与我们的 a_dc = k_rect·a_rms²
//     同式）。该 DC 在**敏感元件内**产生，物理不可逆 —— 后置模拟/数字滤波只能治
//     交流、治不了这个直流。
//   · US EP1930691「Vibration rectification error correction」= VRE 前馈补偿专利；
//     ADI 给出的缓解优先级里，「VRE 前馈」位列算法类手段之首。
//
// 为什么必须在**传感器级**补偿而不是让 EKF 去学：
//   EKF2 无加计倾角直接观测，水平加计偏置只能经**速度新息**间接观测；而 DC 使姿态
//   估计偏 Δθ ⇒ 光流去旋转失配 ⇒ 新息超门被拒 ⇒ 偏置学不动 ⇒ Δθ 更大 —— 结构性
//   死锁（R36j n=9 路径实测）。而 a_rms 是**直接可观测量**（加计交流分量），
//   不需要任何姿态/可观测性前提 ⇒ 前馈可解。
//
// 做法：
//   ① 在【陷波/低通之前】的机体轴加计上在线测交流 RMS：
//        m = EMA(a, τ)，s = EMA(a², τ)，var = s − m²（逐轴）
//        a_rms = sqrt((var_x + var_y + var_z) / 3)      [g]
//   ② 前馈 DC 估计： dc[ax] = k_eff[ax] · a_rms²      [g]
//   ③ 在【滤波之后】就地扣掉 dc —— 送进 EKF / 控制器的是"已被修好的"加计。
//
// ---------------------------------------------------------------------------
// ★★ 为什么 k 会"随频率变"—— 以及 k 其实**不需要标定**（R56，2026-10-08）
// ---------------------------------------------------------------------------
//   真实整流 DC 是  a_dc = k_rect · a_true²，其中 a_true 是**敏感元件处**的交流 RMS。
//   但固件能测到的 a_rms 在**片内 DLPF 之后**（DLPF 属于传感器芯片，位于 SPI 读出
//   之前），于是
//        a_meas = |H_dlpf(f_blade)| · a_true
//        ⇒ 自洽系数  K(f) = k_rect / |H_dlpf(f_blade)|²
//   即 K **不是**常数，而是刀叶通过频率的函数（真机换桨/换转速都会移动它）。
//
//   实测验证（全部用同一份 SIL 二进制，drone_sim）：
//     · 加计片内 DLPF：固件从未写 ACCEL_CONFIG2 ⇒ A_DLPF_CFG=0 ⇒ 3dB 带宽 460Hz，
//       datasheet 群延迟列反推为**一阶**单极点（见 drone_sim/src/sensor_chip_dlpf.h）。
//     · 一阶数字（双线性，fs≈992Hz）幅频：|H(300)|²=0.5207、|H(200)|²=0.7980。
//       ⇒ 预测 K(300)=0.080/0.5207=0.1536、K(200)=0.080/0.7980=0.1003。
//     · 实测最优 VDC_K：300Hz 工况 −0.150（R36n 标定区间 −0.13…−0.16）；200Hz 工况
//       实测 −0.098 使 θ_dc 由 15.2° 掉到 0.8°、e_att 16.1°→3.0°、位移 5.24→1.35 m
//       （R56）。⇒ 解析预测与实测吻合到 2~3%。
//   ⇒ 结论：k 不必台架标定；用**真实整流常数 k_rect**（注入模型 = 0.08，可由器件
//     手册或单次台架推出）配 **DLPF 频率归一化**即可。这也是本项目"自适应"路线
//     在 VRE 侧的落点（与陷波基频由 SpectralPeakTracker 在线跟踪同源）。
//
// ★ 默认全关（k=0）⇒ 不启用时整条信号链行为与改动前逐位一致。
// ★ SIL 环境变量（与其它 SIL_* 注入口同风格）：
//     SIL_VDC_KX / SIL_VDC_KY / SIL_VDC_KZ   [g/g²] 逐轴标定常数（默认 0）
//     SIL_VDC_K                              [g/g²] 便捷：同时设 X 与 Y（Z 保持 0）
//     SIL_VDC_TAU                            滑动 EMA 时间常数 [s]（默认 1.0）
//     SIL_VDC_FNORM=1                        打开频率归一化（默认 0=逐位不变）
//     SIL_VDC_FREF                            k 的标定频率 f_ref [Hz]（默认 300）
//     SIL_VDC_DLPF_FC                         加计片内 DLPF 3dB 带宽 [Hz]（默认 460）
// =============================================================================

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "config/imu_vibration_config.h" // VibDcParams（参数由统一配置传入，见 R58 说明）

class VibDcCompensator
{
public:
    // 采样率已知后再初始化（与其它滤波器同一处调用）。
    //   ★ R58：参数由 ImuVibConfig 传入（不再自己读 env）——
    //     真机上 getenv 恒 NULL 会静默把整条前馈关掉；把"默认值从哪来"上移到
    //     Mpu6500::initSignalChain() 一处，SIL 与真机的差异只在那一层可见。
    void init(const VibDcParams &p, float sample_rate_hz)
    {
        k_[0] = p.kx;
        k_[1] = p.ky;
        k_[2] = p.kz;
        tau_ = (p.tau_s > 1e-3f) ? p.tau_s : 1e-3f;
        fnorm_ = p.fnorm;
        f_ref_ = (p.fref_hz > 1.0f) ? p.fref_hz : 300.0f;
        dlpf_fc_ = p.dlpf_fc_hz;
        k_scale_ = 1.0f;

        enabled_ = (k_[0] != 0.0f) || (k_[1] != 0.0f) || (k_[2] != 0.0f);

        fs_ = (sample_rate_hz > 1.0f) ? sample_rate_hz : 1000.0f;
        dt_ = 1.0f / fs_;
        const float denom = dt_ + tau_;
        alpha_ = (denom > 1e-9f) ? (dt_ / denom) : 1.0f;

        // 归一化的起点：f_ref 处 scale≡1 ⇒ 只设 k 时行为与改动前逐位一致。
        f_last_ = f_ref_;

        reset();

#ifdef SIL_HOST
        fprintf(stderr,
                "[SIL] VibDcCompensator: %s k=(%.5f,%.5f,%.5f) [g/g^2] tau=%.2fs (alpha=%.5f)"
                " | f_norm=%s f_ref=%.1fHz dlpf_fc=%.1fHz\n",
                enabled_ ? "ON" : "OFF", k_[0], k_[1], k_[2], tau_, alpha_,
                fnorm_ ? "ON" : "OFF", f_ref_, dlpf_fc_);
#endif
    }

    // ★ 自适应采样率（R37）：tau 是**物理时间常数**，alpha 只是它的离散化。
    //   SIL 实测处理速率 729~965Hz（名义 1000）⇒ 不重算的话 a_rms 的等效
    //   时间常数会按 fs 比例漂移（tau=0.03s 名义 → 0.031~0.041s 实际）。
    //   保留 m_/s_/primed_ 状态，只改 alpha。
    void setSampleRate(float sample_rate_hz)
    {
        if (!(sample_rate_hz > 1.0f))
            return;
        fs_ = sample_rate_hz;
        dt_ = 1.0f / fs_;
        const float denom = dt_ + tau_;
        alpha_ = (denom > 1e-9f) ? (dt_ / denom) : 1.0f;
    }

    void reset()
    {
        primed_ = false;
        for (int i = 0; i < 3; ++i)
        {
            m_[i] = 0.0;
            s_[i] = 0.0;
            dc_[i] = 0.0f;
        }
        arms_ = 0.0f;
    }

    // ★ 频率归一化（R56）：把由跟踪器给出的当前刀叶通过频率喂进来，
    //   实时把在 f_ref 处定义的标定系数折算到当前频率：
    //        k_eff = k_ · |H_dlpf(f_ref)|² / |H_dlpf(f)|²
    //   ⇒ 一份 k（= k_rect，物理常数）在任何刀叶频率下都成立，不必按转速重标定。
    //   f<=1Hz 或未开 fnorm 时不动作（保持上一次的 scale）。
    void setFrequency(float f_hz)
    {
        if (!fnorm_ || !(f_hz > 1.0f))
            return;
        if (fabsf(f_hz - f_last_) < 0.25f) // 死区：省三角函数，也防抖
            return;
        const float g_ref = dlpfMag2(f_ref_);
        const float g_now = dlpfMag2(f_hz);
        if (!(g_ref > 1e-6f) || !(g_now > 1e-6f))
            return;
        k_scale_ = g_ref / g_now;
        f_last_ = f_hz;
    }

    // ① 在陷波/低通**之前**的机体轴加计上调用（单位 g）
    void updateRaw(const float a_g[3])
    {
        if (!primed_)
        {
            for (int i = 0; i < 3; ++i)
            {
                m_[i] = a_g[i];
                s_[i] = (double)a_g[i] * a_g[i];
            }
            primed_ = true;
            return;
        }
        for (int i = 0; i < 3; ++i)
        {
            m_[i] += alpha_ * ((double)a_g[i] - m_[i]);
            s_[i] += alpha_ * ((double)a_g[i] * a_g[i] - s_[i]);
        }

        double var = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            double v = s_[i] - m_[i] * m_[i];
            if (v < 0.0)
                v = 0.0;
            var += v;
        }
        arms_ = (float)sqrt(var / 3.0); // 逐轴 RMS 的平均（与注入模型 a_rms 同口径）

        if (!enabled_)
            return;
        for (int i = 0; i < 3; ++i)
            dc_[i] = k_[i] * k_scale_ * arms_ * arms_;
    }

    // ② 在滤波**之后**的机体轴加计上调用，就地扣掉前馈 DC
    void compensate(float a_g[3]) const
    {
        if (!enabled_)
            return;
        for (int i = 0; i < 3; ++i)
            a_g[i] -= dc_[i];
    }

    // ---- 诊断 ----
    bool enabled() const { return enabled_; }
    float arms() const { return arms_; } // 当前振动交流 RMS [g]
    // 原始（陷波前）加计的 EMA 均值 [g] —— 即 DC 分量本身，台架标定 k 用。
    //   标定差分式：k_axis = Δmean_axis / Δ(arms²)（消重力与零偏，见 Mpu6500 [VDCCAL]）。
    float mean(int ax) const { return (ax >= 0 && ax < 3) ? (float)m_[ax] : 0.0f; }
    float dc(int ax) const { return dc_[ax]; } // 当前前馈 DC [g]
    float kScale() const { return k_scale_; }  // 频率归一化当前缩放（无归一化=1）

private:
    // |H_dlpf(f)|²：与 drone_sim/src/sensor_chip_dlpf.h 的 design() 同构的
    // 一阶双线性低通幅频平方（fc=dlpf_fc_，fs=实测采样率）。
    //   H(z) = (b0 + b1 z^-1)/(1 + a1 z^-1)，b0=b1=1/(1+K)，a1=(1-K)/(1+K)，
    //   K = fs/(π·fc)  （双线性无预畸变，与片区模型一致）。
    float dlpfMag2(float f_hz) const
    {
        if (!(dlpf_fc_ > 0.0f) || !(fs_ > 1.0f))
            return 1.0f;
        const float K = fs_ / (K_PI * dlpf_fc_);
        const float b0 = 1.0f / (1.0f + K);
        const float a1 = (1.0f - K) / (1.0f + K);
        const float w = 2.0f * K_PI * f_hz / fs_;
        const float cw = cosf(w), sw = sinf(w);
        const float nr = b0 * (1.0f + cw), ni = -b0 * sw;
        const float dr = 1.0f + a1 * cw, di = -a1 * sw;
        const float den = dr * dr + di * di;
        return (den > 1e-12f) ? (nr * nr + ni * ni) / den : 0.0f;
    }

    static constexpr float K_PI = 3.14159265358979323846f;

    bool enabled_ = false;
    float k_[3] = {0.0f, 0.0f, 0.0f};
    float tau_ = 1.0f;
    float dt_ = 0.001f;
    float alpha_ = 0.001f;

    // ---- 频率归一化（R56）----
    bool fnorm_ = false;
    float f_ref_ = 300.0f;   // k_ 的标定频率 [Hz]
    float dlpf_fc_ = 460.0f; // 加计片内 DLPF 3dB 带宽 [Hz]
    float fs_ = 1000.0f;
    float k_scale_ = 1.0f; // 当前 k_eff/k_
    float f_last_ = 0.0f;

    bool primed_ = false;
    double m_[3] = {0.0, 0.0, 0.0};
    double s_[3] = {0.0, 0.0, 0.0};

    float dc_[3] = {0.0f, 0.0f, 0.0f};
    float arms_ = 0.0f;
};
