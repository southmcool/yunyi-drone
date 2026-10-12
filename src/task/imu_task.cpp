/**
 * @title  imu_task —— MPU6500 采样 + 信号链 1kHz 硬实时任务
 * @author: vaie
 * @date: 2026-10-12
 *
 * 职责：
 *   · 起 SPI 总线（BUS2）+ MPU6500
 *   · 加载 ImuCalibration（缺失则零校准）
 *   · 起 ImuSignalChain
 *   · 1kHz 主循环：readRaw → chain.process → publishImu
 *   · 就绪后置位 INIT_BIT_IMU_READY
 *
 * 不做：
 *   · 自适应测频的 FFT（→ imu_fft_task）
 *   · 磁力计 / 气压计（→ magbaro_task）
 *   · 任何 Serial 打印进 1kHz 循环
 *   · 校准命令处理（→ sensor_calib_task）
 *
 * 采样触发：vTaskDelayUntil 1kHz（DRDY 中断后续再加）
 *
 * 错误处理：
 *   · SPI / MPU 初始化失败 → FATAL，进 ERROR 状态
 *   · 校准文件缺失 → WARN，零校准继续（可后补标定）
 *   · 运行期读/处理失败 → 计数（供 imu_fft_task 诊断）
 *
 * 依赖：hal / drivers / core / param / tasks，只在实机编译。
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "hal/spi_bus.h"
#include "drivers/imu/mpu6500_device.h"
#include "core/imu/imu_signal_chain.h"
#include "core/sensors/sensors_hub.h"
#include "core/system/system_status.h"
#include "param/imu_calib_store.h"
#include "task_scheduler.h"
#include "util/boot_log.h"
#include "config/gpio_config.h"
#include "core/imu/imu_calibration.h"

#if defined(SIL_HOST)
// SIL 下本任务不编译（sim_main.cc 直接注入 raw 到 chain）
#else

// ============================================================================
//  全局状态
//   g_chain 非 static —— imu_fft_task 需要 extern 它调 processPendingFft
//   错误计数非 static —— imu_fft_task 30s 诊断时读
// ============================================================================

ImuSignalChain g_chain;

volatile uint32_t g_imu_read_err = 0;
volatile uint32_t g_imu_process_err = 0;

// 仅本任务可见的资源
static SpiBus s_spi_imu;
static Mpu6500Device s_mpu(s_spi_imu, IMU_CSPIN);

// ============================================================================
//  常量
// ============================================================================

namespace
{
    constexpr TickType_t kTaskPeriodTicks = pdMS_TO_TICKS(1); // 1 kHz
    constexpr const char *kMod = "IMU";
}

// ============================================================================
//  任务入口
// ============================================================================

void imu_task(void *pvParameters)
{
    (void)pvParameters;
    BOOT_LOG_I(kMod, "启动");

    // ---- 1. 起 SPI 总线 ----
    if (!s_spi_imu.begin({
            .host = SpiHost::BUS2,
            .sck = IMU_CLKPIN,
            .miso = IMU_MISOPIN,
            .mosi = IMU_MOSIPIN,
            .frequency = 10'000'000,
            .mode = 0,
        }))
    {
        BOOT_LOG_E(kMod, "SPI 总线初始化失败");
        SystemStatus::raiseError(ERR_IMU_INIT_FAIL);
        SystemStatus::setState(SystemState::ERROR);
        vTaskDelete(nullptr);
        return;
    }
    BOOT_LOG_S(kMod, "SPI 总线就绪 (%d Hz, mode 0)", 10000000);

    // ---- 2. 起 MPU6500 ----
    if (!s_mpu.begin())
    {
        BOOT_LOG_E(kMod, "MPU6500 初始化失败（检查接线 / WHO_AM_I）");
        SystemStatus::raiseError(ERR_IMU_INIT_FAIL);
        SystemStatus::setState(SystemState::ERROR);
        vTaskDelete(nullptr);
        return;
    }
    s_mpu.printConfig();

    // ---- 3. 加载校准（零校准兜底）----
    ImuCalibration calib{};

    if (ImuCalibStore::load(&calib))
    {
        BOOT_LOG_S(kMod, "校准已加载 gyro=(%d,%d,%d) accel=(%d,%d,%d)",
                   calib.gyro_offset.x, calib.gyro_offset.y, calib.gyro_offset.z,
                   calib.accel_offset.x, calib.accel_offset.y, calib.accel_offset.z);
    }
    else
    {
        BOOT_LOG_I(kMod, "无有效校准，静止采集 3s（保持机身不动）...");

        ImuCalibrator cal;
        const uint32_t t0 = millis();
        uint32_t read_err = 0;

        while (millis() - t0 < 3000)
        {
            ImuRawSample raw;
            if (s_mpu.readRaw(&raw))
            {
                cal.addSample(raw);
            }
            else
            {
                ++read_err;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (cal.hasEnough(1000))
        {
            float gyro_sd[3];
            float accel_sd[3];
            cal.finish(&calib, gyro_sd, accel_sd);

            BOOT_LOG_S(kMod,
                       "校准完成 n=%u gyro_off=(%d,%d,%d) accel_off=(%d,%d,%d) | gyro_sd=(%.3f,%.3f,%.3f)°/s",
                       cal.sampleCount(),
                       calib.gyro_offset.x, calib.gyro_offset.y, calib.gyro_offset.z,
                       calib.accel_offset.x, calib.accel_offset.y, calib.accel_offset.z,
                       gyro_sd[0], gyro_sd[1], gyro_sd[2]);

            if (ImuCalibStore::save(calib))
            {
                BOOT_LOG_S(kMod, "校准已保存到 Flash");
            }
            else
            {
                BOOT_LOG_W(kMod, "校准保存失败（本次有效，掉电丢失）");
                SystemStatus::raiseError(ERR_IMU_NO_CALIB);
            }
        }
        else
        {
            BOOT_LOG_W(kMod, "校准样本不足 (%u)，使用零校准", cal.sampleCount());
            SystemStatus::raiseError(ERR_IMU_NO_CALIB);
        }
    }

    // ---- 4. 起信号链 ----
    ImuVibConfig vibcfg; // 真机默认值（notch_adapt=true / vdc.k=0 等）
    g_chain.setOrientation(ORIENTATION_FUSIONCONVENTIONNED_YAW_90);
    g_chain.init(vibcfg, calib, s_mpu.scale(), 1000.0f);

    BOOT_LOG_S(kMod, "信号链就绪 fs=%.0fHz notch=%.1fHz adapt=%s VRE=%s",
               g_chain.sampleRate(),
               vibcfg.notch_base_hz,
               vibcfg.notch_adapt ? "ON" : "OFF",
               (vibcfg.vdc.kx || vibcfg.vdc.ky || vibcfg.vdc.kz) ? "ON" : "OFF");

    // ---- 5. 置位 IMU_READY（稳定器 / ToF / 光流的启动依赖）----
    xEventGroupSetBits(xSystemInitEventGroup, INIT_BIT_IMU_READY);
    BOOT_LOG_S(kMod, "就绪（INIT_BIT_IMU_READY）");

    // ---- 6. 主循环 @1kHz ----
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, kTaskPeriodTicks);

        // 6.1 读 raw
        ImuRawSample raw;
        if (!s_mpu.readRaw(&raw))
        {
            g_imu_read_err++;
            continue;
        }
        // ---- ★ 加计质量门：模长偏离 1g 超过 ±15% 丢弃 ----
        //   理由：SPI 偶发坏帧 → accel 突跳 → EKF 姿态被拉到 ±5° → 下一帧拉回
        //   表现：静止时 rpy 出现 ±5° 瞬态尖峰（你的"波动范围 10.6°"）
        // const float ax = raw.acc.x / 4096.0f;
        // const float ay = raw.acc.y / 4096.0f;
        // const float az = raw.acc.z / 4096.0f;
        // const float a_mag = sqrtf(ax * ax + ay * ay + az * az);

        // if (fabsf(a_mag - 1.0f) > 0.15f)
        // {
        //     static uint32_t s_drop = 0;
        //     if ((++s_drop % 20) == 1)
        //     {
        //         Serial.printf("[ACC_BAD] |a|=%.3f  raw=(%d,%d,%d)\n",
        //                       a_mag, raw.acc.x, raw.acc.y, raw.acc.z);
        //     }
        //     continue; // ★ 丢弃本帧
        // }

        // 6.2 信号链（含谐波陷波 + 低通 + VRE + stage 入窗）
        ImuData sd;
        if (!g_chain.process(raw, &sd))
        {
            g_imu_process_err++;
            continue;
        }

        // 6.3 发布
        SensorsHub::publishImu(sd);

        // ★ 无打印、无阻塞 —— 诊断由 imu_fft_task 承担
    }
}

#endif // !SIL_HOST