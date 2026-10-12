#pragma once
#include <Arduino.h>
#include <complex>

// 陷波滤波器
class NotchFilter
{
private:
    // 滤波器系数
    float b0, b1, b2, a1, a2;

    // 滤波器状态
    float x1 = 0, x2 = 0; // 输入历史
    float y1 = 0, y2 = 0; // 输出历史

    // 滤波器参数
    float center_freq = 250.0f;  // 中心频率 (Hz)
    float sample_freq = 1000.0f; // 采样频率 (Hz)
    float Q = 10.0f;             // 品质因数
    float bandwidth = 0.0f;      // 带宽 (Hz)

    bool initialized = false;

    // ★★ R37 稳定性守卫：中心频率 ≥ fs/2 ⇒ ω0 ≥ π ⇒ sin(ω0) ≤ 0 ⇒ α ≤ 0
    //    ⇒ a2 = (1−α)/(1+α) ≥ 1 ⇒ **极点在单位圆外**（指数发散！）。
    //    旧代码只在 init() 里用 `f0 < fs/2.05` 判过一次；而 fs 现在是自适应的、
    //    `updateCenterFrequency()/updateBaseFrequency()` 也能改 f0 —— 任一入口都
    //    可能把陷波推过 Nyquist。**必须每次重算系数后重新判定**，不可实现就旁路
    //    （返回输入），绝不允许静默发散。
    //    实证（R37，fs 由实测喂入）：f0=473Hz 在 fs=940Hz 时 |z|=1.0029（+0.29%/拍，
    //    ⇒ 每秒 15 倍增长）；fs=946Hz 是临界点（|z|=1.0000）。
    bool active_ = true;

public:
    NotchFilter() = default;

    // 初始化滤波器（使用中心频率和Q值）
    void init(float sample_freq, float center_freq, float Q = 10.0f)
    {
        this->sample_freq = sample_freq;
        this->center_freq = center_freq;
        this->Q = Q;

        calculateCoefficientsFromQ();
        reset();
        initialized = true;

        Serial.printf("✅ 陷波滤波器初始化: 中心频率=%.1fHz, Q=%.1f, 带宽=%.1fHz\n",
                      center_freq, Q, bandwidth);
    }

    // 初始化滤波器（使用中心频率和带宽）
    void initWithBandwidth(float sample_freq, float center_freq, float bandwidth)
    {
        this->sample_freq = sample_freq;
        this->center_freq = center_freq;
        this->bandwidth = bandwidth;
        this->Q = center_freq / bandwidth; // 计算Q值

        calculateCoefficientsFromQ();
        reset();
        initialized = true;

        Serial.printf("✅ 陷波滤波器初始化: 中心频率=%.1fHz, 带宽=%.1fHz, Q=%.1f\n",
                      center_freq, bandwidth, Q);
    }

    // 应用滤波器
    float apply(float input)
    {

        // ★ R37：不可实现（f0 ≥ fs/2）时**旁路**。见 active_ 的说明。
        if (!initialized || !active_)
            return input;

        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

        // 数值保护：防止 NaN 或 Inf
        if (isnan(output) || isinf(output))
        {
            Serial.println("⚠️ 滤波器输出异常，已重置");
            reset();
            return input; // 返回原始值作为 fallback
        }

        // 更新状态
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;

        return output;
    }

    // ★ 自适应采样率（用于自适应滤波）——只重算系数，**保留滤波器状态**（不 reset）。
    //   动机：center_freq 存的是**物理频率 [Hz]**，而 b/a 系数由 omega=2π·fc/fs 得出。
    //   若 fs 与算系数时用的名义值不一致（SIL 主机循环达不到 1kHz，实测 730~965Hz
    //   随并行负载漂移），中心频率就会按 fs 比例整体漂移
    //   （300Hz 名义 → 219~290Hz 实际）⇒ 窄带陷波**完全错过**激励频率、形同虚设。
    //   本方法把 fs 改成实测值并重算系数 ⇒ 物理中心频率不变。
    //   实机 fs 恒为 1kHz，调用它与不调用等价（< 1e-3Hz 阈值直接返回）。
    void setSampleFrequency(float new_sample_freq)
    {
        if (new_sample_freq <= 0.0f)
            return;
        if (fabs(sample_freq - new_sample_freq) < 1e-3f)
            return;
        sample_freq = new_sample_freq;
        applyCoefficientUpdate("采样频率");
    }

    // 更新中心频率（用于自适应滤波）
    void updateCenterFrequency(float new_center_freq)
    {
        if (fabs(center_freq - new_center_freq) > 1.0f)
        { // 频率变化超过1Hz才更新
            center_freq = new_center_freq;
            applyCoefficientUpdate("中心频率");

            Serial.printf("🔄 更新陷波频率: %.1fHz\n", center_freq);
        }
    }

    // 更新Q值
    void updateQ(float new_Q)
    {
        if (fabs(Q - new_Q) > 0.5f)
        {
            Q = new_Q;
            applyCoefficientUpdate("Q值");

            Serial.printf("🔄 更新Q值: %.1f\n", Q);
        }
    }

    // 重置滤波器状态
    void reset()
    {
        x1 = x2 = 0;
        y1 = y2 = 0;
    }

    // 获取滤波器信息
    bool isActive() const { return initialized && active_; }
    float centerFrequency() const { return center_freq; }
    float sampleFrequency() const { return sample_freq; }

    void printInfo()
    {
        if (!initialized)
            return;

        Serial.println("📊 陷波滤波器信息:");
        Serial.printf("  中心频率: %.1fHz\n", center_freq);
        Serial.printf("  采样频率: %.0fHz\n", sample_freq);
        Serial.printf("  品质因数Q: %.1f\n", Q);
        Serial.printf("  带宽: %.1fHz\n", bandwidth);
        Serial.printf("  系数: b0=%.6f, b1=%.6f, b2=%.6f\n", b0, b1, b2);
        Serial.printf("        a1=%.6f, a2=%.6f\n", a1, a2);
    }

    // 计算频率响应（用于调试）
    void calculateFrequencyResponse()
    {
        Serial.println("📈 陷波滤波器频率响应:");
        Serial.println("  频率(Hz)   增益(dB)   相位(度)");

        float test_freqs[] = {center_freq / 2, center_freq - 5, center_freq, center_freq + 5, center_freq * 2};
        for (float freq : test_freqs)
        {
            complexResponse(freq);
        }
    }

private:
    // ★ R37：所有"改系数但保留滤波器状态"的入口统一走这里，保证 active_ 状态机对称：
    //   不可实现→可实现 时 reset()（停留期间 y1/y2 陈旧，直接复活会打瞬态）；
    //   可实现→不可实现 时告警一次（否则只是静默旁路，现场无从察觉）。
    void applyCoefficientUpdate(const char *what)
    {
        const bool was_active = active_;
        calculateCoefficientsFromQ();
        if (active_ && !was_active)
        {
            reset();
            Serial.printf("✅ 陷波恢复: %s变更后中心频率%.1fHz < Nyquist(%.1fHz)\n",
                          what, center_freq, 0.5f * sample_freq);
        }
        else if (!active_ && was_active)
        {
            Serial.printf("⚠️ 陷波旁路: %s变更后中心频率%.1fHz ≥ Nyquist(%.1fHz), fs=%.1fHz\n",
                          what, center_freq, 0.5f * sample_freq, sample_freq);
        }
    }

    // 根据Q值计算系数
    void calculateCoefficientsFromQ()
    {
        float omega = 2.0f * PI * center_freq / sample_freq;
        float cos_w = cos(omega);
        float sin_w = sin(omega);
        float alpha = sin_w / (2.0f * Q);
        bandwidth = center_freq / Q;

        // 分子系数（零点）
        float b0_unorm = 1.0f;
        float b1_unorm = -2.0f * cos_w;
        float b2_unorm = 1.0f;

        // 分母系数（极点）
        float a0_unorm = 1.0f + alpha;
        float a1_unorm = -2.0f * cos_w;
        float a2_unorm = 1.0f - alpha;

        // 归一化
        float a0_inv = 1.0f / a0_unorm;
        b0 = b0_unorm * a0_inv;
        b1 = b1_unorm * a0_inv;
        b2 = b2_unorm * a0_inv;
        a1 = a1_unorm * a0_inv;
        a2 = a2_unorm * a0_inv;

        // ★ R37 可实现性/稳定性判定（每个入口都会走到这里 ⇒ 单点把关）
        //   α > 0 ⇔ 0 < ω0 < π ⇔ 0 < f0 < fs/2 ⇔ |a2| = (1−α)/(1+α) < 1。
        //   任一条件不满足即判不可实现：apply() 会旁路，不再输出发散序列。
        active_ = (center_freq > 0.0f) &&
                  (center_freq < 0.5f * sample_freq) &&
                  (alpha > 0.0f) &&
                  (a2 < 1.0f) && (a2 > -1.0f);
    }

    // 计算复频率响应
    void complexResponse(float freq)
    {
        float omega = 2.0f * PI * freq / sample_freq;
        std::complex<float> z = std::polar(1.0f, omega);
        std::complex<float> numerator = b0 + b1 * std::pow(z, -1) + b2 * std::pow(z, -2);
        std::complex<float> denominator = 1.0f + a1 * std::pow(z, -1) + a2 * std::pow(z, -2);
        std::complex<float> H = numerator / denominator;

        float gain_db = 20.0f * log10(std::abs(H));
        float phase_deg = std::arg(H) * 180.0f / PI;

        Serial.printf("  %6.1f      %6.1f      %6.1f\n", freq, gain_db, phase_deg);
    }
};