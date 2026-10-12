#pragma once
#include <Arduino.h>
#include <cmath>

class ButterworthFilter2nd
{
private:
    // 滤波器系数
    float b0, b1, b2, a1, a2;

    // 滤波器状态
    float x1 = 0, x2 = 0; // 输入历史
    float y1 = 0, y2 = 0; // 输出历史

    // 滤波器参数
    float cutoff_freq = 8.0f;
    float sample_freq = 1000.0f;
    bool initialized = false;

public:
    ButterworthFilter2nd() = default;

    // 初始化滤波器
    void init(float sample_freq, float cutoff_freq = 8.0f)
    {
        this->sample_freq = sample_freq;
        this->cutoff_freq = cutoff_freq;

        calculateCoefficients();
        reset();
        initialized = true;

        Serial.printf("✅ 二阶巴特沃斯滤波器初始化: 截止频率=%.1fHz, 采样率=%.0fHz\n",
                      cutoff_freq, sample_freq);
    }

    // 设置截止频率
    void setCutoffFrequency(float cutoff_freq)
    {
        if (fabs(this->cutoff_freq - cutoff_freq) > 0.1f)
        {
            this->cutoff_freq = cutoff_freq;
            calculateCoefficients();
            Serial.printf("🔄 更新滤波器截止频率: %.1fHz\n", cutoff_freq);
        }
    }

    // 应用滤波器
    float apply(float input)
    {
        if (!initialized)
        {
            Serial.println("❌ 滤波器未初始化!");
            return input;
        }

        // 直接形式II实现
        float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

        // 更新状态
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;

        return output;
    }

    // 重置滤波器状态
    void reset()
    {
        x1 = x2 = 0;
        y1 = y2 = 0;
        Serial.println("🔄 滤波器状态重置");
    }

    // 获取滤波器信息
    void printInfo()
    {
        if (!initialized)
            return;

        Serial.println("📊 二阶巴特沃斯滤波器信息:");
        Serial.printf("  截止频率: %.1fHz\n", cutoff_freq);
        Serial.printf("  采样频率: %.0fHz\n", sample_freq);
        Serial.printf("  系数: b0=%.6f, b1=%.6f, b2=%.6f\n", b0, b1, b2);
        Serial.printf("        a1=%.6f, a2=%.6f\n", a1, a2);
    }

private:
    // 计算巴特沃斯滤波器系数
    void calculateCoefficients()
    {
        // 二阶巴特沃斯滤波器系数计算
        float omega = 2.0f * PI * cutoff_freq / sample_freq;
        float sin_omega = sin(omega);
        float cos_omega = cos(omega);

        // 巴特沃斯特征参数
        float alpha = sin_omega / (2.0f * 0.7071f); // Q = 1/√2 = 0.7071

        // 双二阶滤波器系数
        float a0 = 1.0f + alpha;

        b0 = (1.0f - cos_omega) / (2.0f * a0);
        b1 = (1.0f - cos_omega) / a0;
        b2 = (1.0f - cos_omega) / (2.0f * a0);

        a1 = -2.0f * cos_omega / a0;
        a2 = (1.0f - alpha) / a0;

        // 归一化，使直流增益为1
        float dc_gain = b0 + b1 + b2;
        b0 /= dc_gain;
        b1 /= dc_gain;
        b2 /= dc_gain;
    }
};