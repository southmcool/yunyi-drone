
#ifndef SENSORFILTER_H
#define SENSORFILTER_H

#include "type/imu_types.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

class SensorFilter
{
private:
    static constexpr float ACCEL_CUTOFF_X = 15.0f; // Hz
    static constexpr float ACCEL_CUTOFF_Y = 15.0f; // Hz
    static constexpr float ACCEL_CUTOFF_Z = 10.0f; // Hz - Z轴更激进

    static constexpr Axis3f ACCEL_CUTOFF = {15.0f, 15.0f, 10.0f}; // Hz - Z轴更激进

    static constexpr float GYRO_CUTOFF = 30.0f; // Hz

    float alpha_gyro;
    Axis3f alpha_accel;
    Axis3f filteredAccel;
    Axis3f filteredGyro;

public:
    void init(float sampleFreq)
    {
        float dt = 1.0f / sampleFreq;

        for (int i = 0; i < 3; ++i)
        {
            alpha_accel.axis[i] = 1.0f / (1.0f + 1.0f / (2 * PI * ACCEL_CUTOFF.axis[i] * dt));
        }

        alpha_gyro = 1.0f / (1.0f + 1.0f / (2 * PI * GYRO_CUTOFF * dt));
#if defined(LOG_INFO)
        Serial.printf("分轴滤波 ");
        for (int i = 0; i < 3; ++i)
        {
            Serial.printf("Accel[%d]:%.0fHz(α=%.3f)", i, ACCEL_CUTOFF[i], ACCEL_CUTOFF.axis[i]);
        }
        Serial.println("");
#endif
    }

    void applyFilter(ImuData *data)
    {
        // 简单的一阶滤波
        // 分轴应用不同的滤波强度

        for (int i = 0; i < 3; ++i)
        {
            filteredAccel.axis[i] = alpha_accel.axis[i] * data->acc.axis[i] + (1.0f - alpha_accel.axis[i]) * filteredAccel.axis[i];
            filteredGyro.axis[i] = alpha_gyro * data->gyro.axis[i] + (1.0f - alpha_gyro) * filteredGyro.axis[i];

            data->acc.axis[i] = filteredAccel.axis[i];
            data->gyro.axis[i] = filteredGyro.axis[i];
        }
    }

    // -----------------------------------------------------------------------
    // ★ 仅加速度计的低通（不碰陀螺！）
    //
    // 为什么不能直接用上面的 applyFilter()：那条路径会把陀螺也低通到 30Hz，
    // 给角速度内环硬塞进相位滞后 —— 角速度环靠的就是陀螺的"快"，滤掉就没意义了。
    //
    // 为什么加计需要它：加计同时喂给两处，而这两处都对高频噪声敏感——
    //   1) Fusion AHRS 的重力方向修正（噪声会让静止时的姿态抖动）
    //   2) PositionEKF 的 predict()（加计噪声被二次积分 ⇒ 位置随 t² 漂）
    // 参考实现都做了这件事：Crazyflie 加计 30Hz 二阶 LPF
    //   （esp-drone sensors_bmi088_bmp388.c:136-137，陀螺 80Hz/加计 30Hz）。
    // -----------------------------------------------------------------------
    void initAccelLowPass(float sampleFreq, float cutoffHz = 30.0f)
    {
        fs_ = sampleFreq;
        accel_lp_cutoff_hz_ = cutoffHz; // ★ 保存物理截止频率，供 setSampleRate() 重算
        accel_lp_alpha = 1.0f / (1.0f + 1.0f / (2.0f * PI * cutoffHz * (1.0f / sampleFreq)));
        accel_lp_ready = false;

#if defined(LOG_INFO)
        Serial.printf("加计低通: %.0fHz (fs=%.0fHz, alpha=%.4f)\n",
                      cutoffHz, sampleFreq, accel_lp_alpha);
#endif
    }

    // ★ SIL：旁路/启用加计低通（A/B 对照用）。固件默认启用。
    void setAccelLowPassEnabled(bool en) { accel_lp_enabled = en; }

    void applyAccelLowPass(ImuData *data)
    {
        // ★ SIL 可通过 setAccelLowPassEnabled(false) 旁路，用于 A/B 对照实验。
        //   固件默认 true，行为不变。
        if (!accel_lp_enabled)
            return;

        for (int i = 0; i < 3; ++i)
        {
            if (!accel_lp_ready)
            {
                // 首帧直接灌入，避免从 0 缓慢爬升造成的启动瞬态
                filteredAccelLP = data->acc;
                accel_lp_ready = true;
                return;
            }
            else
            {
                filteredAccelLP.axis[i] = accel_lp_alpha * data->acc.axis[i] + (1.0f - accel_lp_alpha) * filteredAccelLP.axis[i];
            }

            data->acc.axis[i] = filteredAccelLP.axis[i];
        }
    }

    // -----------------------------------------------------------------------
    // ★ 陀螺低通（**默认关闭**）
    //
    // 依据：tools/sil/imu_probe 实测陀螺通道 50~450Hz 抑制度 0.0dB —— 唯一保护点
    //   是 473Hz 单点陷波（-66dB）。机架刚性不足时电机转频激励（63~350Hz）会
    //   原封不动进入两处：Fusion AHRS 的陀螺积分、角速度内环的微分路径。
    //
    // 与 accel 低通的关键区别：陀螺低通会**直接给角速度内环加相位滞后**，
    //   所以截止频率必须显著高于姿态环带宽，否则内环失稳。
    //   此处默认 80Hz，对齐 Crazyflie 的实现
    //   （esp-drone sensors_bmi088_bmp388.c:136-137 陀螺 80Hz / 加计 30Hz）。
    //
    // ★ enabled 默认 false：不在未经确认的情况下改变固件飞行行为。
    //   SIL 用环境变量 SIL_GYRO_LP=<Hz> 打开做 A/B，量化收益后再决定是否改为默认开。
    // -----------------------------------------------------------------------
    void initGyroLowPass(float sampleFreq, float cutoffHz = 80.0f)
    {
        fs_ = sampleFreq;
        gyro_lp_cutoff_hz_ = cutoffHz; // ★ 保存物理截止频率，供 setSampleRate() 重算
        gyro_lp_alpha = 1.0f / (1.0f + 1.0f / (2.0f * PI * cutoffHz * (1.0f / sampleFreq)));
        gyro_lp_ready = false;

#if defined(LOG_INFO)
        Serial.printf("陀螺低通: %.0fHz (fs=%.0fHz, alpha=%.4f)\n",
                      cutoffHz, sampleFreq, gyro_lp_alpha);
#endif
    }

    void setGyroLowPassEnabled(bool en) { gyro_lp_enabled = en; }
    bool gyroLowPassEnabled() const { return gyro_lp_enabled; }

    void applyGyroLowPass(ImuData *data)
    {
        if (!gyro_lp_enabled)
            return;

        for (int i = 0; i < 3; ++i)
        {

            if (!gyro_lp_ready)
            {
                // 首帧直接灌入，避免从 0 缓慢爬升造成的启动瞬态
                filteredGyroLP = data->gyro;
                gyro_lp_ready = true;
                return;
            }
            else
            {
                filteredGyroLP.axis[i] = gyro_lp_alpha * data->gyro.axis[i] + (1.0f - gyro_lp_alpha) * filteredGyroLP.axis[i];
            }

            data->gyro.axis[i] = filteredGyroLP.axis[i];
        }
    }

    // -----------------------------------------------------------------------
    // ★ 自适应采样率（R37）：把 fs 从「名义值」改成「实测值」并重算所有 alpha。
    //
    //   动机（SIL 实测，R36ah）：sim_main.cc 把整条数字滤波链按 FS=1000Hz 设计，
    //   但 SIL 的 IMU 循环是「事件驱动 + 100µs 轮询 + 100µs 粒度 sleep」，在 CPU
    //   争抢下漏采样（--jobs 6 实测 dt 中 12% 为 2ms、4% 为 3ms），**实际处理速率
    //   只有 729~965Hz 且随并发负载逐批漂移**。后果：
    //     ① 一阶 LPF 的物理截止频率按 fs 比例漂移（30Hz 名义 → 21.9~29Hz 实际）；
    //     ② 陷波中心漂移（300Hz 名义 → 219~290Hz 实际）⇒ 完全错过激励频率；
    //     ③ 各 A/B 臂实际 fs 不同 ⇒ 滤波器带宽不同 ⇒「单变量」前提被破坏。
    //   ⇒ 把真实 fs 喂进来即可同时解决①②③。实机 fs 恒定 ⇒ 调用它与不调用等价。
    //
    //   只重算 alpha、**保留滤波状态**（filteredAccelLP_*/gyro_lp_ready 不动），
    //   否则每次调用都会在输出上打一个「从 0 爬升」的瞬态（比 fs 误差本身更糟）。
    // -----------------------------------------------------------------------
    void setSampleRate(float new_hz)
    {
        if (new_hz <= 1.0f)
            return;
        // 0.1Hz 以内的抖动不重算（避免每拍都做除法和 printf 级的开销）
        if (new_hz > fs_ - 0.1f && new_hz < fs_ + 0.1f)
            return;
        fs_ = new_hz;
        const float dt = 1.0f / fs_;

        // 通用一阶滤波（applyFilter 路径，当前未接入信号链，一并保持一致）
        for (int i = 0; i < 3; i++)
        {
            alpha_accel.axis[i] = 1.0f / (1.0f + 1.0f / (2.0f * PI * ACCEL_CUTOFF.axis[i] * dt));
        }

        alpha_gyro = 1.0f / (1.0f + 1.0f / (2.0f * PI * GYRO_CUTOFF * dt));

        // 加计/陀螺低通（★ 信号链在用，物理截止频率必须与 fs 解耦）
        accel_lp_alpha = 1.0f / (1.0f + 1.0f / (2.0f * PI * accel_lp_cutoff_hz_ * dt));
        gyro_lp_alpha = 1.0f / (1.0f + 1.0f / (2.0f * PI * gyro_lp_cutoff_hz_ * dt));
    }

    float sampleRate() const { return fs_; }
    float accelLowPassAlpha() const { return accel_lp_alpha; }
    float gyroLowPassAlpha() const { return gyro_lp_alpha; }

private:
    float accel_lp_alpha = 0.0f;
    bool accel_lp_ready = false;
    bool accel_lp_enabled = true; // ★ SIL 可旁路，固件默认启用
    Axis3f filteredAccelLP = {0};
    Axis3f filteredGyroLP = {0};

    float gyro_lp_alpha = 1.0f;
    bool gyro_lp_ready = false;
    bool gyro_lp_enabled = false; // ★ 默认关闭（不改固件飞行行为）

    // ★ R37 自适应采样率用的记忆量：物理频率与采样率分离存储，
    //   这样 setSampleRate() 才能只改 fs 而不动截止频率。
    float fs_ = 1000.0f;
    float accel_lp_cutoff_hz_ = 30.0f;
    float gyro_lp_cutoff_hz_ = 80.0f;
};
#endif // SENSOR_DATA_H