/**
 * @title  magbaro_task —— ATK-MS901M 磁力计/气压计采集任务
 * @author: vaie
 * @date: 2026-10-12
 *
 * 频率 125Hz（8ms），非阻塞 update()：
 *   模块上报 250Hz（帧间隔 4ms），任务 125Hz。
 *   UART 缓冲 8ms 内积累 ≤30B，远小于默认 256B ⇒ 永不溢出。
 *   8ms 内可能到达 2 帧，后一帧覆盖前一帧（update() 内部语义）
 *   ⇒ 跳帧但不丢帧。mag/baro 的时间常数（100ms / 秒级）远大于 8ms
 *   ⇒ 对 EKF 无影响。
 *
 * 校准：
 *   本任务**不做**校准 —— 45s 阻塞操作属于 sensor_calib_task。
 *   提供全局 g_atk901 供其访问。
 *
 * 不依赖 IMU：
 *   走 UART2，与 IMU 的 SPI 无耦合 ⇒ 与 imu_task 并行启动。
 *   完成后置位 INIT_BIT_MAGBARO_READY（WARN 级 —— 缺失不阻塞起飞）。
 *
 * 依赖：drivers / core / tasks，只在实机编译。
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include "drivers/mag_baro/atk_imu901.h"
#include "core/sensors/sensors_hub.h"
#include "core/system/system_status.h"
#include "task_scheduler.h"
#include "util/boot_log.h"
#include "config/gpio_config.h"

#if defined(SIL_HOST)
// SIL 下不编译（sim_main.cc 直接注入 mag/baro）
#else

// ============================================================================
//  全局实例（sensor_calib_task 也需要访问）
// ============================================================================

AtkImu901 g_atk901(Serial2, ATK901_RX_PIN, ATK901_TX_PIN);

// ============================================================================
//  常量
// ============================================================================

namespace
{
    constexpr TickType_t kTaskPeriodTicks = pdMS_TO_TICKS(20); //50 Hz
    constexpr uint32_t kDiagPeriodMs = 5000;                  // 5s 一行
    constexpr const char *kMod = "MAGBARO";
}

// ============================================================================
//  任务入口
// ============================================================================
static uint32_t loop_count = 0;
void magbaro_task(void *pvParameters)
{
    (void)pvParameters;
    BOOT_LOG_I(kMod, "启动");

    // ---- 1. 起 ATK-MS901M（不等 IMU，独立启动）----
    if (!g_atk901.begin())
    {
        BOOT_LOG_E(kMod, "ATK-MS901M 通信失败，降级运行（无 mag/baro）");
        SystemStatus::raiseError(ERR_MAGBARO_INIT_FAIL); // WARN，不阻塞
        vTaskDelete(nullptr);
        return;
    }

    // ---- 2. 加载气压校准（缺失用零基准）----
    if (g_atk901.loadCalibration())
    {
        BOOT_LOG_S(kMod, "气压校准已加载");
    }
    else
    {
        BOOT_LOG_W(kMod, "无气压校准，使用零基准（需标定）");
        SystemStatus::raiseError(ERR_MAGBARO_NO_CALIB);
    }
    g_atk901.printCalibration();

    // ---- 3. 置位 MAGBARO_READY（可选传感器）----
    xEventGroupSetBits(xSystemInitEventGroup, INIT_BIT_MAGBARO_READY);
    BOOT_LOG_S(kMod, "就绪（125Hz 主循环）");

    // ---- 4. 主循环 @125Hz ----
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_diag_ms = millis();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, kTaskPeriodTicks);
        loop_count++;
        // 4.1 非阻塞扫一遍 UART（内部拿不到锁会跳过）
        g_atk901.update();

        // 4.2 有帧就发布
        mag_s m;
        if (g_atk901.readMag(&m))
        {
            SensorsHub::publishMag(m);
        }
        baro_s b;
        if (g_atk901.readBaro(&b))
        {
            SensorsHub::publishBaro(b);
        }

        // 4.3 诊断（5s 一行）
        const uint32_t now_ms = millis();
        if (now_ms - last_diag_ms < kDiagPeriodMs)
            continue;
        last_diag_ms = now_ms;

        const auto mh = SensorsHub::magHealth();
        const auto bh = SensorsHub::baroHealth();
        const uint64_t now_us = (uint64_t)esp_timer_get_time();

        const bool mag_offline = (!mh.valid) || (now_us - mh.last_update_us > SensorsHub::kMagTimeoutUs);
        const bool baro_offline = (!bh.valid) || (now_us - bh.last_update_us > SensorsHub::kBaroTimeoutUs);

        // ★ 加：实测帧率
        const auto rate = g_atk901.sampleRateStats();

        Serial.printf(
            "[MAGBARO]task_loop=%u Hz (期望50) | rate=%u/%u Hz | mag total=%u offline=%d | baro total=%u offline=%d | err=%u(+%u)\n",
            loop_count * 1000 / kDiagPeriodMs,
            rate.mag_hz, rate.baro_hz,
            g_atk901.magFrameCount(), (int)mag_offline,
            g_atk901.baroFrameCount(), (int)baro_offline,
            g_atk901.frameErrorCount(), rate.frame_errors);
        // 上报超时（WARN）
        if (mag_offline && mh.valid)
            SystemStatus::raiseError(ERR_MAGBARO_NO_DATA);
        if (baro_offline && bh.valid)
            SystemStatus::raiseError(ERR_MAGBARO_NO_DATA);
        loop_count = 0;
    }
}

#endif // !SIL_HOST