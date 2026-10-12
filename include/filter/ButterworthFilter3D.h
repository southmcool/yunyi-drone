#pragma once
#include "ButterworthFilter.h"
#include "type/imu_types.h"
class ButterworthFilter3D
{
private:
    ButterworthFilter2nd filters[6]; // 0-2:加速度计, 3-5:陀螺仪
    bool initialized = false;

public:
    // 初始化所有轴的滤波器
    void init(float sample_freq, float accel_cutoff = 8.0f, float gyro_cutoff = 8.0f)
    {
        for (int i = 0; i < 3; i++)
        {
            filters[i].init(sample_freq, accel_cutoff);    // 加速度计
            filters[i + 3].init(sample_freq, gyro_cutoff); // 陀螺仪
        }
        initialized = true;

        Serial.printf("✅ 3D巴特沃斯滤波器初始化完成\n");
        Serial.printf("   加速度计截止: %.1fHz, 陀螺仪截止: %.1fHz\n",
                      accel_cutoff, gyro_cutoff);
    }

    // 应用滤波器到传感器数据
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

    // 设置截止频率
    void setCutoffFrequencies(float accel_cutoff, float gyro_cutoff)
    {
        for (int i = 0; i < 3; i++)
        {
            filters[i].setCutoffFrequency(accel_cutoff);
            filters[i + 3].setCutoffFrequency(gyro_cutoff);
        }
    }

    // 重置所有滤波器
    void reset()
    {
        for (auto &filter : filters)
        {
            filter.reset();
        }
    }

    // 打印所有滤波器信息
    void printInfo()
    {
        Serial.println("\n📊 3D巴特沃斯滤波器组信息:");
        for (int i = 0; i < 6; i++)
        {
            const char *axis_name[] = {"Accel-X", "Accel-Y", "Accel-Z",
                                       "Gyro-X", "Gyro-Y", "Gyro-Z"};
            Serial.printf("  %s: ", axis_name[i]);
            // 这里可以添加各滤波器的详细信息
        }
    }
};