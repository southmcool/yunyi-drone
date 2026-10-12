/**
 * @title  stabilizer_task —— 姿态控制 + 混控 + 写电机油门
 * @author: vaie
 * @date: 2026-10-12
 *
 * 1kHz 主循环：
 *   读 EKF state（EstimatorKalman::readState）
 *   读全局 setpoint（xGlobalSetpoint / system_armed）
 *   AttitudeController::update → control_t
 *   Mixer::mix → motor_mix_t
 *   [0,1] → DShot [48, 2047]
 *   motor_throttle_update（临界区）
 *
 * 未上锁时写 0（触发 EscDShot::vStopMotor）。
 *
 * 依赖：core/estimator / core/controller / util / tasks，只在实机编译。
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include "core/estimator/estimator_kalman.h"
#include "core/controller/attitude_controller.h"
#include "core/controller/mixer.h"
#include "core/system/system_status.h"
#include "task/task_scheduler.h"
#include "util/boot_log.h"
#include "core/system/shared_data.h" // motor_throttle_update
#include "core/telemetry/telemetry_data.h"

#if defined(SIL_HOST)
// SIL 下不编译
#else

// ---- 外部（receiver.cpp 提供）----
extern bool system_armed;
extern setpoint_t xGlobalSetpoint;
extern portMUX_TYPE g_receiver_lock;
extern AttitudeController attitudeController;
// ============================================================================
//  常量
// ============================================================================

namespace
{

    constexpr TickType_t kTaskPeriodTicks = pdMS_TO_TICKS(1); // 1 kHz
    constexpr uint32_t kDiagPeriodMs = 30000;
    constexpr uint32_t kInitWaitMs = 10000;
    constexpr const char *kMod = "STAB";

    // DShot300 编码范围
    constexpr uint16_t kDshotMin = 48;
    constexpr uint16_t kDshotMax = 2047;
    constexpr float kDshotRange = (float)(kDshotMax - kDshotMin);

    inline uint64_t nowMicros()
    {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

    inline uint16_t toDshot(float v)
    {
        if (v < 0.0f)
            v = 0.0f;
        if (v > 1.0f)
            v = 1.0f;
        return (uint16_t)(kDshotMin + v * kDshotRange + 0.5f);
    }

} // namespace

// ============================================================================
//  任务入口
// ============================================================================

void stabilizer_task(void *pvParameters)
{
    (void)pvParameters;
    BOOT_LOG_I(kMod, "启动");

    // ---- 1. 等 IMU 就绪 ----
    const EventBits_t bits = xEventGroupWaitBits(
        xSystemInitEventGroup,
        INIT_BIT_IMU_READY,
        pdFALSE, pdTRUE,
        pdMS_TO_TICKS(kInitWaitMs));

    if ((bits & INIT_BIT_IMU_READY) != INIT_BIT_IMU_READY)
    {
        BOOT_LOG_E(kMod, "等 IMU 就绪超时，任务退出");
        SystemStatus::raiseError(ERR_STABILIZER_FAIL);
        vTaskDelete(nullptr);
        return;
    }

    // ---- 2. 置位 STABILIZER_READY ----
    xEventGroupSetBits(xSystemInitEventGroup, INIT_BIT_STABILIZER_READY);
    BOOT_LOG_S(kMod, "就绪（1kHz）");

    // ---- 3. 主循环 ----
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_us = nowMicros();
    uint32_t last_diag_ms = millis();
    uint32_t ctrl_count = 0;
    uint32_t skip_count = 0;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, kTaskPeriodTicks);

        // 3.1 dt
        const uint64_t now_us = nowMicros();
        float dt = (float)(now_us - last_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.001f;
        last_us = now_us;

        // 3.2 读 EKF 状态
        state_t st;
        if (!EstimatorKalman::readState(&st))
        {
            ++skip_count;
            continue;
        }
        telemetry_update_attitude(st);
        // 3.3 读全局 setpoint（临界区）
        setpoint_t sp;
        bool armed;
        portENTER_CRITICAL_SAFE(&g_receiver_lock);
        sp = xGlobalSetpoint;
        armed = system_armed;
        portEXIT_CRITICAL_SAFE(&g_receiver_lock);

        telemetry_update_setpoint(xGlobalSetpoint);
        // 3.4 控制器
        control_t out;
        attitudeController.update(st, sp, out, dt);
        ++ctrl_count;

        // 3.5 混控
        motor_mix_t mix;
        if (armed)
        {
            Mixer::mix(out, &mix);
        }
        else
        {
            Mixer::allSame(0.0f, &mix);
        }

        // 3.6 [0,1] → DShot
        uint16_t throttle[4];
        if (!armed)
        {
            for (int i = 0; i < 4; ++i)
                throttle[i] = 0; // → vStopMotor
        }
        else
        {
            for (int i = 0; i < 4; ++i)
                throttle[i] = toDshot(mix.motor[i]);
        }

        // 3.7 写全局（临界区）
        motor_throttle_update(throttle);

        // 3.8 诊断（30s 一行）
        const uint32_t now_ms = millis();
        if (now_ms - last_diag_ms >= kDiagPeriodMs)
        {
            last_diag_ms = now_ms;
            BOOT_LOG_I(kMod,
                       "armed=%d ctrl=%u skip=%u | "
                       "rpy=(%.1f,%.1f,%.1f)° | "
                       "sp=(%.2f,%.2f,%.2f) thr=%.2f | "
                       "out=(%.2f,%.2f,%.2f) | "
                       "m=(%.2f,%.2f,%.2f,%.2f)",
                       (int)armed, ctrl_count, skip_count,
                       st.attitude_deg.roll, st.attitude_deg.pitch, st.attitude_deg.yaw,
                       sp.attitudeRate.roll, sp.attitudeRate.pitch, sp.attitudeRate.yaw,
                       sp.thrust,
                       out.roll, out.pitch, out.yaw,
                       mix.motor[0], mix.motor[1], mix.motor[2], mix.motor[3]);
            ctrl_count = 0;
            skip_count = 0;
        }
    }
}

#endif // !SIL_HOST