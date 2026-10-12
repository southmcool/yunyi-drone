#pragma once
#include <Arduino.h>
#include "NotchFilter.h"

class HarmonicNotchFilter {
private:
    static const int MAX_HARMONICS = 5; // 最大谐波数量
    NotchFilter harmonic_filters[MAX_HARMONICS];
    int num_harmonics = 0;
    float base_frequency = 250.0f;
    float sample_freq = 1000.0f;
    bool initialized = false;

public:
    // 初始化谐波陷波滤波器
    void init(float sample_freq, float base_freq, int harmonics = 3, float Q = 8.0f) {
        this->sample_freq = sample_freq;
        this->base_frequency = base_freq;
        this->num_harmonics = constrain(harmonics, 1, MAX_HARMONICS);
        
        // 创建基频和各次谐波的陷波滤波器
        for (int i = 0; i < num_harmonics; i++) {
            float harmonic_freq = base_freq * (i + 1);
            if (harmonic_freq < sample_freq / 2.05f) { // 避免混叠，留有余量
                harmonic_filters[i].init(sample_freq, harmonic_freq, Q);
                Serial.printf("  谐波%d: %.1fHz\n", i+1, harmonic_freq);
            } else {
                num_harmonics = i; // 调整实际谐波数量
                break;
            }
        }
        
        initialized = true;
        Serial.printf("✅ 谐波陷波滤波器初始化: 基频=%.1fHz, %d次谐波\n", 
                     base_freq, num_harmonics);
    }
    
    // 应用所有谐波滤波
    float apply(float input) {
        if (!initialized || num_harmonics == 0) return input;
        
        float output = input;
        for (int i = 0; i < num_harmonics; i++) {
            output = harmonic_filters[i].apply(output);
        }
        return output;
    }
    
    // ★ 自适应采样率：把每条谐波的 fs 改成实测值并重算系数（物理中心频率不变、
    //   滤波器状态保留）。见 NotchFilter::setSampleFrequency 的说明。
    void updateSampleFrequency(float new_sample_freq)
    {
        if (!initialized || new_sample_freq <= 0.0f)
            return;
        sample_freq = new_sample_freq;
        for (int i = 0; i < num_harmonics; i++) {
            harmonic_filters[i].setSampleFrequency(new_sample_freq);
        }
    }

    // 更新基频（用于自适应）
    void updateBaseFrequency(float new_base_freq) {
        if (!initialized) return;
        
        base_frequency = new_base_freq;
        
        for (int i = 0; i < num_harmonics; i++) {
            float harmonic_freq = base_frequency * (i + 1);
            if (harmonic_freq < sample_freq / 2.2f) {
                harmonic_filters[i].updateCenterFrequency(harmonic_freq);
            }
        }
        
        static unsigned long last_print = 0;
        if (millis() - last_print > 1000) {
            last_print = millis();
            Serial.printf("🔄 谐波陷波更新: 基频=%.1fHz\n", base_frequency);
        }
    }
    
    // 获取基频
    float getBaseFrequency() const {
        return base_frequency;
    }
    
    // 获取谐波数量
    int getHarmonicCount() const {
        return num_harmonics;
    }
    
    // 重置所有滤波器
    void reset() {
        for (int i = 0; i < num_harmonics; i++) {
            harmonic_filters[i].reset();
        }
    }
    
    // 打印谐波信息
    void printHarmonics() {
        if (!initialized) return;
        
        Serial.println("🎵 谐波陷波滤波器信息:");
        Serial.printf("  基频: %.1fHz\n", base_frequency);
        Serial.printf("  谐波数量: %d\n", num_harmonics);
        for (int i = 0; i < num_harmonics; i++) {
            Serial.printf("  谐波%d: %.1fHz\n", i+1, base_frequency * (i+1));
        }
    }
};