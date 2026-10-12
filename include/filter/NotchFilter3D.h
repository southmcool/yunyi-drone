#pragma once
#include "NotchFilter.h"
#include "type/imu_types.h"
class NotchFilter3D
{
private:
    NotchFilter filters[6]; // 0-2:加速度计, 3-5:陀螺仪
    bool initialized = false;

    // 🆕 分轴频率配置
    float accel_freq[3] = {250.0f, 250.0f, 250.0f}; // Accel X,Y,Z
    float gyro_freq[3] = {250.0f, 250.0f, 250.0f};  // Gyro X,Y,Z

    // 🆕 分轴Q值配置
    float accel_Q[3] = {8.0f, 8.0f, 8.0f};
    float gyro_Q[3] = {12.0f, 12.0f, 12.0f};

public:
    // 🆕 初始化分轴陷波滤波器
    void initPerAxis(float sample_freq,
                     const float accel_freq[3], const float gyro_freq[3],
                     const float accel_Q[3] = nullptr, const float gyro_Q[3] = nullptr)
    {
        // 初始化加速度计陷波滤波器
        for (int i = 0; i < 3; i++)
        {
            float q = (accel_Q != nullptr) ? accel_Q[i] : 8.0f;
            filters[i].init(sample_freq, accel_freq[i], q);
            this->accel_freq[i] = accel_freq[i];
            this->accel_Q[i] = q;
        }

        // 初始化陀螺仪陷波滤波器
        for (int i = 0; i < 3; i++)
        {
            float q = (gyro_Q != nullptr) ? gyro_Q[i] : 12.0f;
            filters[i + 3].init(sample_freq, gyro_freq[i], q);
            this->gyro_freq[i] = gyro_freq[i];
            this->gyro_Q[i] = q;
        }

        initialized = true;

        Serial.println("✅ 分轴陷波滤波器初始化完成");
        printPerAxisConfig();
    }

    // 🆕 基于FFT分析结果自动配置
    void initFromFFTAnalysis(float sample_freq, const float fft_freq[6])
    {
        // fft_freq[0-2]: Gyro X,Y,Z 主频
        // fft_freq[3-5]: Accel X,Y,Z 主频

        float default_accel_Q[3] = {10.0f, 10.0f, 15.0f}; // Z轴Q值更高（重力分量）
        float default_gyro_Q[3] = {12.0f, 12.0f, 12.0f};

        initPerAxis(sample_freq,
                    &fft_freq[3], // 加速度计频率
                    &fft_freq[0], // 陀螺仪频率
                    default_accel_Q, default_gyro_Q);
    }

    // 🆕 分轴更新频率
    void updatePerAxisFrequency(const float new_accel_freq[3], const float new_gyro_freq[3])
    {
        for (int i = 0; i < 3; i++)
        {
            if (fabs(accel_freq[i] - new_accel_freq[i]) > 0.5f)
            {
                filters[i].updateCenterFrequency(new_accel_freq[i]);
                accel_freq[i] = new_accel_freq[i];
            }
        }

        for (int i = 0; i < 3; i++)
        {
            if (fabs(gyro_freq[i] - new_gyro_freq[i]) > 0.5f)
            {
                filters[i + 3].updateCenterFrequency(new_gyro_freq[i]);
                gyro_freq[i] = new_gyro_freq[i];
            }
        }

        static unsigned long last_print = 0;
        if (millis() - last_print > 2000)
        {
            last_print = millis();
            Serial.println("🔄 分轴陷波频率已更新");
            printPerAxisConfig();
        }
    }

    // 🆕 分轴更新Q值
    void updatePerAxisQ(const float new_accel_Q[3], const float new_gyro_Q[3])
    {
        for (int i = 0; i < 3; i++)
        {
            if (fabs(accel_Q[i] - new_accel_Q[i]) > 0.1f)
            {
                filters[i].updateQ(new_accel_Q[i]);
                accel_Q[i] = new_accel_Q[i];
            }
        }

        for (int i = 0; i < 3; i++)
        {
            if (fabs(gyro_Q[i] - new_gyro_Q[i]) > 0.1f)
            {
                filters[i + 3].updateQ(new_gyro_Q[i]);
                gyro_Q[i] = new_gyro_Q[i];
            }
        }
    }

    // ★ 自适应采样率（R37）：把六条陷波的 fs 改成实测值并重算系数（物理中心频率
    //   不变、滤波器状态保留）。见 NotchFilter::setSampleFrequency 的说明。
    //   注：本滤波器当前**未接入**信号链（processRawSample 里的调用被注释），
    //   此处一并支持，以免日后启用时踩同一个 fs 失配坑。
    void updateSampleFrequency(float new_sample_freq)
    {
        if (!initialized || new_sample_freq <= 0.0f)
            return;
        for (auto &f : filters)
            f.setSampleFrequency(new_sample_freq);
    }

    // ★ R37 说明：本类**不需要**自己写 Nyquist 门限——`initPerAxis / updatePerAxisFrequency
    //   / updatePerAxisQ / updateSampleFrequency` 四条入口最终都落到
    //   `NotchFilter::calculateCoefficientsFromQ()`，那里做了单点可实现性判定
    //   （f0 ≥ fs/2 ⇒ 旁路），并且 `applyCoefficientUpdate()` 保证失活/复活时状态对称。
    //   此处保留本注释，避免日后有人在 3D 层再加一层重复门限造成口径分叉。

    // 应用滤波器（保持不变）
    void apply(ImuData *data)
    {
        if (!initialized)
            return;

        // 滤波加速度计数据
        data->acc.x = filters[0].apply(data->acc.x);
        data->acc.y = filters[1].apply(data->acc.y);
        data->acc.z = filters[2].apply(data->acc.z);

        // 滤波陀螺仪数据
        data->gyro.x = filters[3].apply(data->gyro.x);
        data->gyro.y = filters[4].apply(data->gyro.y);
        data->gyro.z = filters[5].apply(data->gyro.z);
    }

    // 🆕 打印分轴配置
    void printPerAxisConfig()
    {
        Serial.println("\n📊 分轴陷波滤波器配置:");
        Serial.println("  轴   类型      频率(Hz)   Q值");
        Serial.println("  --- --------- ---------- -----");

        const char *axis_names[] = {"X", "Y", "Z"};
        for (int i = 0; i < 3; i++)
        {
            Serial.printf("  %s   Accel     %8.1f   %5.1f\n",
                          axis_names[i], accel_freq[i], accel_Q[i]);
        }
        for (int i = 0; i < 3; i++)
        {
            Serial.printf("  %s   Gyro      %8.1f   %5.1f\n",
                          axis_names[i], gyro_freq[i], gyro_Q[i]);
        }
    }

    // 🆕 获取当前配置
    void getCurrentConfig(float accel_freq_out[3], float gyro_freq_out[3],
                          float accel_Q_out[3], float gyro_Q_out[3])
    {
        memcpy(accel_freq_out, accel_freq, sizeof(accel_freq));
        memcpy(gyro_freq_out, gyro_freq, sizeof(gyro_freq));
        memcpy(accel_Q_out, accel_Q, sizeof(accel_Q));
        memcpy(gyro_Q_out, gyro_Q, sizeof(gyro_Q));
    }

    void reset()
    {
        for (auto &filter : filters)
        {
            filter.reset();
        }
    }
};