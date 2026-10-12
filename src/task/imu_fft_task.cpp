/**
 * @title  imu_fft_task —— 自适应测频 FFT 执行者 + 低频诊断
 * @author: vaie
 * @date: 2026-10-12
 *
 * 为什么必须独立任务：
 *   SpectralPeakTracker::processPending() 做 3×(512×10) butterfly
 *   ≈ 246k flops；ESP32-S3 @240MHz 仅单精度 FPU ⇒ 单次 ~1~2ms。
 *   若塞进 1kHz imu_task 会周期性丢帧（R37 实证：丢帧 1% ⇒ 陷波
 *   抑制度从 −39.8dB 崩到 −9.3dB）。本任务优先级 2、core 1 ⇒ 绝不
 *   抢占 imu_task（core 0）；5ms 轮询、1Hz 有效计算 ⇒ CPU 占用可忽略。
 *
 * 顺带承担 30s 一轮的诊断打印：
 *   · [NOTCHADAPT] 自适应陷波状态
 *   · [HUB]        SensorsHub 健康
 *   · [STATUS]     系统错误
 *   · [STACK]      任务栈余量
 *   · [ERR]        IMU 读/处理错误计数
 *
 * ★ 这些打印若放 1kHz 主循环，一行 ~100 字符 @115200 ≈ 8ms 阻塞
 *   ⇒ 每打一次丢 7~8 个采样点。本任务优先级低 ⇒ 打印零影响。
 *
 * 依赖：core / util，只在实机编译。
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include "core/imu/imu_signal_chain.h"
#include "core/sensors/sensors_hub.h"
#include "core/system/system_status.h"

#if defined(SIL_HOST)
// SIL 下不编译（SIL 侧 processPendingFft 在 chain.process() 尾部内联）
#else

// imu_task.cpp 定义
extern ImuSignalChain g_chain;
extern volatile uint32_t g_imu_read_err;
extern volatile uint32_t g_imu_process_err;

// ============================================================================
//  常量
// ============================================================================

namespace
{

    constexpr uint32_t kFftPollMs = 5;
    constexpr uint32_t kDiagMs = 30000;
    constexpr const char *kMod = "FFT";

    // FFT 停滞判据：连续 N 次诊断（= N×30s）都没跟踪到频率
    constexpr uint32_t kFftStallDiagThreshold = 3;

    // 任务栈 HWM 检查列表（任务名 + 友好名）
    struct TaskEntry
    {
        const char *name;
    };
    constexpr TaskEntry kTasksToCheck[] = {
        {"imu"},
        {"imu_fft"},
        {"magbaro"},
        {"vl53l1xs"},
        {"pmw3901"},
        {"stabilizer"},
        {"motor_out"},
        {"telemetry"},
        {"sensor_calib"},
    };

} // namespace

// ============================================================================
//  内部辅助
// ============================================================================

namespace
{

    void printStackHwm(const char *name)
    {
        TaskHandle_t h = xTaskGetHandle(name); // 不存在返回 NULL
        if (!h)
        {
            Serial.printf("  %-14s n/a\n", name);
            return;
        }
        Serial.printf("  %-14s %5u B free%s\n",
                      name,
                      (unsigned)uxTaskGetStackHighWaterMark(h),
                      uxTaskGetStackHighWaterMark(h) < 1024 ? "  ⚠" : "");
    }

    void runDiag(uint32_t &last_err_r, uint32_t &last_err_p,
                 uint32_t &fft_stall_count)
    {
        // ---- 1. 自适应陷波状态 ----
        if (g_chain.notchAdaptive())
        {
            const float f = g_chain.notchTrackFreq();
            Serial.printf("[NOTCHADAPT] f=%.1fHz ratio=%.1f applied=%.1fHz\n",
                          f, g_chain.notchTrackRatio(), g_chain.notchAppliedFreq());

            // FFT 停滞检测
            if (f < 1.0f)
            {
                if (++fft_stall_count >= kFftStallDiagThreshold)
                {
                    SystemStatus::raiseError(ERR_FFT_STALLED);
                }
            }
            else
            {
                fft_stall_count = 0;
                SystemStatus::clearError(ERR_FFT_STALLED);
            }
        }
        else
        {
            Serial.println("[NOTCHADAPT] adaptive=OFF");
        }

        // ---- 2. SensorsHub 健康 ----
        {
            const auto ih = SensorsHub::imuHealth();
            const uint64_t now_us = (uint64_t)esp_timer_get_time();
            const bool offline = (!ih.valid) ||
                                 (now_us - ih.last_update_us > SensorsHub::kImuTimeoutUs);

            Serial.printf("[HUB] imu n=%u offline=%d\n",
                          ih.update_count, (int)offline);

            if (offline && ih.valid)
            {
                SystemStatus::raiseError(ERR_IMU_NO_DATA);
            }
        }

        // ---- 3. IMU 读/处理错误计数（增量）----
        {
            const uint32_t r = g_imu_read_err;
            const uint32_t p = g_imu_process_err;
            const uint32_t dr = r - last_err_r;
            const uint32_t dp = p - last_err_p;
            last_err_r = r;
            last_err_p = p;

            if (dr || dp)
            {
                Serial.printf("[ERR] imu read=%u(+%u) process=%u(+%u)\n",
                              r, dr, p, dp);
            }
        }

        // ---- 4. 系统状态与错误 ----
        SystemStatus::dumpErrors();

        // ---- 5. 任务栈余量 ----
        Serial.println("[STACK] --- 任务栈余量（字节；历史最小值）---");
        for (const auto &t : kTasksToCheck)
        {
            printStackHwm(t.name);
        }
    }

} // namespace

// ============================================================================
//  任务入口
// ============================================================================

void imu_fft_task(void *pvParameters)
{
    (void)pvParameters;

    uint32_t last_diag_ms = millis();
    uint32_t last_err_r = 0;
    uint32_t last_err_p = 0;
    uint32_t fft_stall_count = 0;

    for (;;)
    {
        // ---- 1. FFT 主职：无待处理帧时 O(1) 早退 ----
        g_chain.processPendingFft();

        vTaskDelay(pdMS_TO_TICKS(kFftPollMs));

        // ---- 2. 30s 诊断 ----
        const uint32_t now_ms = millis();
        if (now_ms - last_diag_ms < kDiagMs)
            continue;
        last_diag_ms = now_ms;

        runDiag(last_err_r, last_err_p, fft_stall_count);
    }
}

#endif // !SIL_HOST