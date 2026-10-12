#pragma once
// =============================================================================
// SpectralPeakTracker —— 在线振动基频跟踪器（自适应陷波的「测频」环节）
//
// 外部证据 / 架构对标（PX4-Autopilot-main）：
//   · PX4 的自适应陷波由两部分组成——
//       ① `src/modules/sensors/gyro_fft/`：对陀螺做加窗 FFT，输出
//          `sensor_gyro_fft_s.peak_frequencies_{x,y,z}`（峰值频率）；
//       ② `VehicleAngularVelocity::UpdateDynamicNotchFFT()`：把峰值喂给
//          `NotchFilter::setParameters(fs, notch_freq, bandwidth)` **在线重调**中心频率
//          （参数 `IMU_GYRO_DNF_EN/MIN/BW/HMC`）。
//    本类承担 ① 的角色（只「测频」，不碰滤波）；重调由
//    `HarmonicNotchFilter::updateBaseFrequency()` 完成（= ② 的角色）。
//    ⇒ 分工与 PX4 同构，不自创机制。
//
// 为什么自包含（不依赖 ESP-DSP / Arduino）：
//   仓库既有的 `FFTAnalyzer` 依赖 `dsps_*`（ESP-DSP）。SIL 宿主的
//   `tools/sil/mock/esp_dsp.h` 里这些接口是**空桩**（其注释原文：「固件内的
//   FFTAnalyzer 不参与 SIL 计算」）⇒ 在线测频在 SIL 里从来跑不起来，频谱分析被
//   搬到**离线 Python**，测出的 300Hz 再手工写回常量 —— 这就是"自适应"退化成
//   "固定参数"的确切原因（闭环被切成两半，用人替代）。
//   本类只用 <math.h>/<stdint.h> ⇒ SIL 宿主与 ESP32 跑**同一份代码**（真源不分裂），
//   闭环可在线成立、也可被 SIL 直接验证。
//
// 算法（每帧）：
//   滑动 N 点窗（非重叠，N=1024 → fs=1kHz 时约 1 帧/s）
//     → 逐轴 Hanning 加窗 → 就地 radix-2 FFT
//     → 三轴功率谱求和 P[k]
//     → [fmin,fmax] 带内取最大峰 kpk
//     → 峰 / 带内中位数 ≥ min_ratio 才认（否则保持旧估计，抗"跟踪噪声"）
//     → 抛物线插值取亚 bin 精度
//     → EMA 平滑（时间常数 tau，按实测 fs 重算）
//   输出 f_est_ = 平滑后的基频估计 [Hz]
//
// ★ 默认不启用：由 Mpu6500 读 SIL_NOTCH_ADAPT=1 打开 ⇒ 不注入时整链逐位不变。
//
// =============================================================================
// ★★ R59：SIL 上测到的频率会**系统性偏低**，且这不是本类的缺陷（实测澄清）
// =============================================================================
//   现象：SIL 里自适应臂长期给出 ~272–288Hz，而注入的刀叶频率是 299.85Hz。
//   离线裁决（对同一条 CSV 做两种 FFT，见 out/_fbasechk.py）：
//     · 把样本重采样到**均匀 1ms 仿真栅格**再 FFT → 峰值 299.8~300.2Hz（ratio 232~419）
//     · 直接按**原始索引栅格**、用实测速率做 FFT → 峰值 213~291Hz（ratio 25~86）
//   两者唯一的差别是采样间隔是否均匀 ⇒ 偏低的根因是 **SIL 的采样栅格抖动**
//   （dt_sim 均值 1.09ms、SD 0.39ms ⇒ 约 9~12% 的 IMU 帧被丢），不是算法。
//   机理与 R37 对「陷波」的结论同源：非均匀栅格上 300Hz 激励的相位被打乱 ⇒
//   谱展宽、峰位偏移、带内中位数抬高（ratio 掉到门限附近）。
//   ★ 推论（重要，别误读 SIL 结论）：**SIL 无法验证本类的测频精度**。实机
//     imu_task 是硬件节拍的均匀 1kHz 且不丢帧（FFT 已挪到低优先级任务，
//     不再占用 1ms 周期）⇒ 该失效模式在真机上不存在。反过来说：本类在 SIL 上
//     表现差**不能**用来否定它，在 SIL 上表现好也**不能**用来担保真机。
//   ⇒ 真机验收只能用 Mpu6500::adaptiveFreqTaskLoop() 里那行 [NOTCHADAPT] 串口日志。
// =============================================================================

#include <math.h>
#include <stdint.h>

class SpectralPeakTracker
{
public:
    static const int MAX_N = 1024;

    // fs: 实测采样率[Hz]；nfft: 窗长（规整为 2 的幂，上限 MAX_N）
    // fmin/fmax: 搜索带[Hz]（上限须 < fs/2）；tau_s: 频率 EMA 时间常数[s]
    void init(float fs, int nfft, float fmin, float fmax, float tau_s)
    {
        fs_ = (fs > 1.0f) ? fs : 1000.0f;

        int want = (nfft > 32) ? nfft : MAX_N;
        if (want > MAX_N)
            want = MAX_N;
        n_ = 1;
        while (n_ * 2 <= want)
            n_ *= 2;

        fmin_ = fmin;
        fmax_ = fmax;
        tau_ = (tau_s > 1e-3f) ? tau_s : 1.0f;
        const float dt_frame = (float)n_ / fs_;
        alpha_ = dt_frame / (dt_frame + tau_);

        for (int i = 0; i < n_; ++i)
            window_[i] = 0.5f * (1.0f - cosf(K_TWO_PI * (float)i / (float)(n_ - 1)));
        for (int a = 0; a < 3; ++a)
            for (int i = 0; i < n_; ++i)
                buf_[a][i] = 0.0f;

        idx_ = 0;
        frames_ = 0;
        n_tot_ = 0;
        n_valid_ = 0;
        hist_ = 0; // ★ R76 近窗有效性位图一并复位
        n_hist_ = 0;
        peak_ratio_ = 0.0f;
        f_meas_ = 0.0f;
        f_est_ = 0.0f;
        have_est_ = false;
        pending_ = false;
        initialized_ = true;
    }

    // fs 变化时重算 EMA 系数（窗长与状态保留）——与整条链的自适应 fs 保持一致
    void setSampleRate(float fs)
    {
        if (!initialized_ || !(fs > 1.0f))
            return;
        fs_ = fs;
        const float dt_frame = (float)n_ / fs_;
        alpha_ = dt_frame / (dt_frame + tau_);
    }

    // -------------------------------------------------------------------------
    // ★ 两段式接口（R58）：把"入窗"(O(1)) 与"跑 FFT"(重活) 分开，以便把 FFT 搬到
    //   低优先级任务去 ——
    //
    //   为什么必须分（真机，SIL 从未暴露）：
    //     真机 imu_task 是 **1ms 硬周期**（imu_task.cpp:57 `pdMS_TO_TICKS(1)` +
    //     `vTaskDelayUntil`）。而 runFrame() 要做 3×(512×10)=15360 个 butterfly
    //     ≈ 246k flops；ESP32-S3 @240MHz 只有单精度 FPU（无硬件 double）⇒ 约
    //     1~2ms。**同步跑会让那一拍超期 ⇒ 周期性丢帧**，正是 R37 实证过的陷阱：
    //     丢帧 1% ⇒ 陷波抑制度从 −39.8dB 掉到 −9.3dB（out/_sweep.py 剂量-响应）。
    //     ⇒ 自适应测频会**反噬陷波本身**。必须离开 IMU 上下文。
    //
    //   SIL 的用法：紧接着 stage() 调 processPending()（非硬实时，语义等价于旧
    //     push()，保证已注册基线与 A/B 臂逐位不变）。
    //   真机的用法：stage() 在 IMU 任务；processPending() 在低优先级任务轮询。
    // -------------------------------------------------------------------------

    // 阶段①：入窗。只做 3 次 float 拷贝 + 一次索引比较 ⇒ 可在硬实时上下文调用。
    //   返回 true = 本拍窗满（产生了一整帧待处理数据）。
    bool stage(const float gyro[3])
    {
        if (!initialized_)
            return false;
        for (int a = 0; a < 3; ++a)
            buf_[a][idx_] = gyro[a];
        if (++idx_ < n_)
            return false;
        idx_ = 0;
        frames_++;
        pending_ = true; // 置标志；FFT 交给 processPending()
        return true;
    }

    // 阶段②：若有待处理帧就把它算掉（重活）。返回 true = 本次真的跑了一帧。
    //   单写者（IMU 任务, core 0）+ 单读者（imu_fft 任务, core 1）⇒ 无锁。
    //   ⚠ 竞态是**良性**的，别为此加双缓冲（会多 12KB RAM）：
    //     runFrame() 逐轴先整段拷贝 buf_[a][*] 再算 FFT，而同时 IMU 任务正以
    //     1 样本/ms 覆写 buf_[a][idx_]。故可能被"踩到"的只是**最前面 1~4 个**样本；
    //     而窗函数是 Hann（window_[0]=0，头部权重按 0.5·(1−cos(2πi/(n−1))) 缓升）
    //     ⇒ 对 1024 点功率谱和的贡献可忽略。频率再经 τ=1.5s 的 EMA 平滑。
    //   另外 pending_ 的 32-bit 读写天然原子，最坏读到上一帧的频率（≤1 拍陈旧）。
    bool processPending()
    {
        if (!pending_)
            return false;
        pending_ = false;
        runFrame();
        return true;
    }

    // 兼容旧接口 = 入窗 + 立即算（同拍完成一帧，即旧 push 的同步语义）。
    bool push(const float gyro[3])
    {
        if (!stage(gyro))
            return false;
        processPending();
        return true;
    }

    bool ready() const { return have_est_; }
    float freq() const { return f_est_; }       // 平滑后基频估计 [Hz]
    float lastMeas() const { return f_meas_; }  // 本帧原始测量（未平滑）
    float peakRatio() const { return peak_ratio_; }
    float validFrac() const
    {
        return (n_tot_ > 0) ? ((float)n_valid_ / (float)n_tot_) : 0.0f;
    }
    // ★ R76：近窗（≤32 帧 ≈ 33s @1kHz/1024 点）有效帧占比与已积累帧数 ——
    //   供 Mpu6500 的「重调持续性门」。单写者 runFrame()（真机 imu_fft 任务 /
    //   SIL 同步调用），读者在 IMU 上下文原子读 32-bit ⇒ 无锁，最坏 ≤1 帧陈旧。
    float recentValidFrac() const
    {
        return (n_hist_ > 0)
                   ? ((float)__builtin_popcount(hist_) / (float)n_hist_)
                   : 0.0f;
    }
    uint32_t histLen() const { return (uint32_t)n_hist_; }
    uint32_t frames() const { return frames_; }
    int window() const { return n_; }
    float sampleRate() const { return fs_; }

private:
    static constexpr float K_TWO_PI = 6.283185307179586f;
    // 峰/带内中位数门限。★ R53 由 4.0 提到 50：R52 实测"无振动段"（地面/起飞）会出现
    //   84~93Hz 的假峰、比值仅 8.6~14.8，刚过 4 就把陷波重调到 84Hz（虽自愈但属误调）；
    //   而真实振动段比值 ≥299（多数 1e3~9e3）⇒ 取 50 有 6× 余量，干净分离。
    //   达不到门限时**不更新**（保持既有基频）⇒ 降级安全，绝不锁到噪声上。
    static constexpr float K_MIN_RATIO = 50.0f;

    void runFrame()
    {
        const int half = n_ / 2;
        for (int k = 0; k <= half; ++k)
            powsum_[k] = 0.0f;

        // 逐轴 FFT，功率谱求和（三轴一起找"共同"的模态，抗单轴瞬态）
        for (int a = 0; a < 3; ++a)
        {
            for (int i = 0; i < n_; ++i)
            {
                re_[i] = buf_[a][i] * window_[i];
                im_[i] = 0.0f;
            }
            fftRadix2(re_, im_, n_);
            for (int k = 0; k <= half; ++k)
                powsum_[k] += re_[k] * re_[k] + im_[k] * im_[k];
        }

        int kmin = (int)ceilf(fmin_ * (float)n_ / fs_);
        int kmax = (int)floorf(fmax_ * (float)n_ / fs_);
        if (kmin < 1)
            kmin = 1;
        if (kmax > half - 1)
            kmax = half - 1;
        if (kmax <= kmin + 2)
            return;

        int kpk = kmin;
        float ppk = powsum_[kmin];
        for (int k = kmin + 1; k <= kmax; ++k)
            if (powsum_[k] > ppk)
            {
                ppk = powsum_[k];
                kpk = k;
            }

        // 峰/带内中位数门控：峰不显著（无振动 / 宽带噪声）⇒ 本帧不更新，保持旧值
        const int m = kmax - kmin + 1;
        for (int i = 0; i < m; ++i)
            tmp_[i] = powsum_[kmin + i];
        insertionSort(tmp_, m);
        const float med = tmp_[m / 2];
        n_tot_++;
        // ★ R76：滚动 32 帧有效性历史（先移入 0=无效；有效分支再置位）。
        //   位置在门判定之前 ⇒ 有效/无效两分支都被记录。
        hist_ = (uint32_t)(hist_ << 1);
        if (n_hist_ < 32)
            ++n_hist_;
        peak_ratio_ = (med > 1e-12f) ? (ppk / med) : 0.0f;
        if (peak_ratio_ < K_MIN_RATIO)
            return;

        // 抛物线插值（亚 bin 精度）
        float d = 0.0f;
        if (kpk > kmin && kpk < kmax)
        {
            const float y0 = powsum_[kpk - 1], y1 = powsum_[kpk], y2 = powsum_[kpk + 1];
            const float den = y0 - 2.0f * y1 + y2;
            if (fabsf(den) > 1e-12f)
                d = 0.5f * (y0 - y2) / den;
            if (d > 0.5f)
                d = 0.5f;
            if (d < -0.5f)
                d = -0.5f;
        }
        f_meas_ = ((float)kpk + d) * fs_ / (float)n_;
        n_valid_++;
        hist_ |= 1u; // ★ R76：本帧有效 ⇒ 位置 1

        if (!have_est_)
        {
            f_est_ = f_meas_;
            have_est_ = true;
        }
        else
        {
            f_est_ += alpha_ * (f_meas_ - f_est_);
        }
    }

    // 就地 radix-2（迭代 Cooley-Tukey），要求 n 为 2 的幂
    static void fftRadix2(float *re, float *im, int n)
    {
        for (int i = 1, j = 0; i < n; ++i)
        {
            int bit = n >> 1;
            for (; j & bit; bit >>= 1)
                j ^= bit;
            j ^= bit;
            if (i < j)
            {
                float t = re[i]; re[i] = re[j]; re[j] = t;
                t = im[i]; im[i] = im[j]; im[j] = t;
            }
        }
        for (int len = 2; len <= n; len <<= 1)
        {
            const float ang = -K_TWO_PI / (float)len;
            const float wr = cosf(ang), wi = sinf(ang);
            const int half = len >> 1;
            for (int i = 0; i < n; i += len)
            {
                float cwr = 1.0f, cwi = 0.0f;
                for (int j = 0; j < half; ++j)
                {
                    const int i1 = i + j, i2 = i + j + half;
                    const float ur = re[i1], ui = im[i1];
                    const float vr = re[i2] * cwr - im[i2] * cwi;
                    const float vi = re[i2] * cwi + im[i2] * cwr;
                    re[i1] = ur + vr; im[i1] = ui + vi;
                    re[i2] = ur - vr; im[i2] = ui - vi;
                    const float ncwr = cwr * wr - cwi * wi;
                    cwi = cwr * wi + cwi * wr;
                    cwr = ncwr;
                }
            }
        }
    }

    static void insertionSort(float *a, int n)
    {
        for (int i = 1; i < n; ++i)
        {
            const float key = a[i];
            int j = i - 1;
            while (j >= 0 && a[j] > key)
            {
                a[j + 1] = a[j];
                --j;
            }
            a[j + 1] = key;
        }
    }

    bool initialized_ = false;
    int n_ = MAX_N;
    float fs_ = 1000.0f;
    float fmin_ = 80.0f;
    float fmax_ = 450.0f;
    float tau_ = 1.5f;
    float alpha_ = 0.4f;

    int idx_ = 0;
    uint32_t frames_ = 0;
    uint32_t n_tot_ = 0;
    uint32_t n_valid_ = 0;
    uint32_t hist_ = 0;  // ★ R76：bit[i]=最近第 i 帧是否过 K_MIN_RATIO 门（1=有效）
    uint8_t n_hist_ = 0; // 已积累帧数（饱和 32；单写者更新，读者原子读）
    float peak_ratio_ = 0.0f;
    float f_meas_ = 0.0f;
    float f_est_ = 0.0f;
    bool have_est_ = false;
    // ★ volatile 是必需的（不是保险）：processPending() 在低优先级任务的循环里被
    //   反复轮询，若不 volatile 编译器会把读提升到循环外 ⇒ 循环永不退出。
    volatile bool pending_ = false;

    float buf_[3][MAX_N];
    float window_[MAX_N];
    float re_[MAX_N];
    float im_[MAX_N];
    float powsum_[MAX_N / 2 + 1];
    float tmp_[MAX_N / 2 + 1];
};
