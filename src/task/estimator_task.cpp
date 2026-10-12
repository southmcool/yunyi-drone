/**
 * @title  estimator_task —— 250Hz EKF 主循环
 * @author: vaie
 * @date: 2026-10-12
 *
 * 职责：
 *   · 等 IMU_READY
 *   · EstimatorKalman::init()
 *   · 250Hz 主循环：
 *       readImu → predict
 *       每 8 拍（≈31Hz）：readBaro → updateBaro
 *       finalize + publishState
 *       边界检查 → requestReset
 *   · 30s 诊断打印
 *
 * ---- ★★★ 前置条件：kalman_core.cpp 必须改 14 处符号 ★★★ ----
 *   Crazyflie 内部世界系是"Z 向上"，我们 state_t 是 NED（Z 向下）。
 *   若不做转换，EKF 会把重力算反，静止时状态立即发散。
 *
 *   两种做法（二选一）：
 *
 *   做法 A（推荐）：改 kalman_core.cpp，内部直接用 NED
 *     1) A 矩阵 6 处重力项翻符号：
 *        A[PY][D0]: -g*R[2][2]*dt → +g*R[2][2]*dt
 *        A[PZ][D0]: +g*R[2][1]*dt → -g*R[2][1]*dt
 *        A[PX][D1]: +g*R[2][2]*dt → -g*R[2][2]*dt
 *        A[PZ][D1]: -g*R[2][0]*dt → +g*R[2][0]*dt
 *        A[PX][D2]: -g*R[2][1]*dt → +g*R[2][1]*dt
 *        A[PY][D2]: +g*R[2][0]*dt → -g*R[2][0]*dt
 *     2) 状态更新 4 处重力项翻符号（飞行 3 + 地面 3，位置项 2）：
 *        所有 `- kalman::kGravity * kf->R[2][x]` → `+ kalman::kGravity * kf->R[2][x]`
 *        所有 `- kalman::kGravity * dt2 * 0.5f`  → `+ kalman::kGravity * dt2 * 0.5f`
 *     3) kalmanCoreExternalizeState 里：
 *        `state->acc.z = ... - 1.0f` → `state->acc.z = ... + 1.0f`
 *        `state->attitude.pitch = -pitch * kRad2Deg` → 去掉负号
 *     4) kalmanCoreUpdateWithBaro 里参考更新简化：
 *        `if (!quadIsFlying || baroReferenceHeight < 1.0f)`
 *        → `if (!quadIsFlying)`
 *     5) estimator_task 里 baro 传 `-altitude_m`（NED 深度）
 *
 *   做法 B：estimator_task 里做输入/输出转换
 *     · 输入翻 Y 和 Z（acc + gyro）
 *     · 输出 state.position.z / velocity.z / acc.z 翻符号
 *     更复杂，不推荐。
 *
 *   本文件按**做法 A** 编写。
 *
 * 依赖：core/estimator / core/sensors / tasks / util，只在实机编译。
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>

#include "core/estimator/estimator_kalman.h"
#include "core/estimator/kalman_core.h" // 只为 kalman::kGravity
#include "core/sensors/sensors_hub.h"
#include "core/system/system_status.h"
#include "task/task_scheduler.h"
#include "util/boot_log.h"
#include "core/telemetry/receiver.h"
#if defined(SIL_HOST)
// SIL 下不编译（sim_main.cc 直接驱动 estimator）
#else

// ============================================================================
//  常量
// ============================================================================

namespace
{

    constexpr TickType_t kTaskPeriodTicks = pdMS_TO_TICKS(2); // 500 Hz
    constexpr uint32_t kBaroDividerMax = 16;                  // 250/8 ≈ 31 Hz
    constexpr uint32_t kDiagPeriodMs = 30000;
    constexpr uint32_t kInitWaitMs = 10000;
    constexpr float kDegToRad = 0.0174532925f;
    constexpr const char *kMod = "EST";

    inline uint64_t nowMicros()
    {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

} // namespace

// ============================================================================
//  任务入口
// ============================================================================

extern setpoint_t xGlobalSetpoint;
extern bool system_armed; // 全局安全状态锁
void estimator_task(void *pvParameters)
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
        SystemStatus::raiseError(ERR_IMU_INIT_FAIL);
        vTaskDelete(nullptr);
        return;
    }

    // ---- 2. 起 EKF ----
    EstimatorKalman::init();
    BOOT_LOG_S(kMod, "EKF 就绪（250Hz）");

    // ---- 3. 主循环 ----
    TickType_t last_wake = xTaskGetTickCount();
    uint64_t last_us = nowMicros();
    uint32_t baro_div = 0;
    uint32_t last_diag_ms = millis();
    uint32_t predict_count = 0;
    uint32_t baro_count = 0;
    uint32_t mag_count = 0;
    uint32_t tof_count = 0;
    uint32_t flow_count = 0;
    uint32_t reset_count = 0;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, kTaskPeriodTicks);

        // ---- 3.1 dt ----
        const uint64_t now_us = nowMicros();
        float dt = static_cast<float>(now_us - last_us) * 1e-6f;
        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.004f; // 防跳变/回绕
        last_us = now_us;

        // ---- 3.2 处理复位请求（外部命令或越界触发）----
        if (EstimatorKalman::consumeResetRequest())
        {
            ++reset_count;
            BOOT_LOG_W(kMod, "EKF 复位");
        }

        // ---- 3.3 读 IMU（NED 机体系）----
        ImuData sd;
        if (!SensorsHub::readImu(&sd))
            continue; // 无新帧跳过

        // 单位换算：g → m/s²，°/s → rad/s
        // ★ 不做坐标系转换（kalman_core 已改符号适配 NED）
        Axis3f acc_ms2 = {
            sd.acc.x * kalman::kGravity,
            sd.acc.y * kalman::kGravity,
            sd.acc.z * kalman::kGravity,
        };
        Axis3f gyro_rads = {
            sd.gyro.x * kDegToRad,
            sd.gyro.y * kDegToRad,
            sd.gyro.z * kDegToRad,
        };

        // ---- 3.4 在飞判据 ----
        //   TODO：接入 control_t.thrust 后改为 thrust > 0.1f
        //   当前固定 false（地面/调试模式）

        const bool flying = system_armed || (xGlobalSetpoint.thrust > 0.05f);

        // ---- 3.5 预测 ----
        EstimatorKalman::predict(dt, acc_ms2, gyro_rads, flying);
        ++predict_count;

        // ---- 3.6 气压更新（≈31Hz）----
        if (++baro_div >= kBaroDividerMax)
        {
            baro_div = 0;
            baro_s ba;
            if (SensorsHub::readBaro(&ba))
            {
                // NED: S[Z] 是向下深度；baro altitude 是向上高度
                // ⇒ 传负值
                const float asl_ned = -ba.altitude_cm * 0.01f;
                EstimatorKalman::updateBaro(asl_ned, flying);
                ++baro_count;
            }
        }

        // 3.6d 磁力计（yaw 观测，10Hz）
        // {
        //     mag_s mg;
        //     if (SensorsHub::readMag(&mg))
        //     {
        //         magMeasurement_t mm;
        //         mm.timestamp = mg.timestamp_us;
        //         // ★ 直接用 raw LSB —— yaw 只看方向，不需要 gauss 换算
        //         mm.mag_body.x = (float)mg.mag.x;
        //         mm.mag_body.y = (float)mg.mag.y;
        //         mm.mag_body.z = (float)mg.mag.z;
        //         // 噪声：LSB 单位。你的 raw 值在 ±300 左右
        //         //   0.05 倍的水平分量 = 15 LSB 噪声，可接受
        //         mm.stdDev = 0.5f;
        //         EstimatorKalman::updateMag(mm);
        //         ++mag_count;
        //     }
        // }

        // 3.6b TOF
        {
            tofMeasurement_t tof;
            while (xQueueReceive(xTofQueue, &tof, 0) == pdTRUE)
            {
                EstimatorKalman::updateTof(tof);
                ++tof_count;
            }
        }

        // 3.6c 光流
        {
            flowMeasurement_t flow;
            while (xQueueReceive(xFlowDataQueue, &flow, 0) == pdTRUE)
            {
                EstimatorKalman::updateFlow(flow, gyro_rads);
                ++flow_count;
            }
        }

        // ---- 3.7 收尾 + 发布 ----
        EstimatorKalman::finalize(now_us);
        EstimatorKalman::publishState(acc_ms2, gyro_rads, now_us);

        // ---- 3.8 边界检查 ----
        if (!EstimatorKalman::isWithinBounds())
        {
            BOOT_LOG_W(kMod, "EKF 越界，请求复位");
            EstimatorKalman::requestReset();
        }

        // ---- 3.9 诊断（30s）----
        const uint32_t now_ms = millis();
        if (now_ms - last_diag_ms >= kDiagPeriodMs)
        {
            last_diag_ms = now_ms;
            state_t s;

            if (EstimatorKalman::readState(&s))
            {
                // 改成：
                BOOT_LOG_I(kMod,
                           "rpy=(%.1f,%.1f,%.1f)°"
                           "  | vel=(%.2f,%.2f,%.2f)m/s"
                           "  | pos=(%.2f,%.2f,%.2f)m"
                           "  | predict=%u reset=%u"
                           "  | tof=%u flow=%u mag=%u",
                           s.attitude_deg.roll, s.attitude_deg.pitch, s.attitude_deg.yaw,
                           s.velocity.x, s.velocity.y, s.velocity.z,
                           s.position.x, s.position.y, s.position.z,
                           predict_count, reset_count, tof_count, flow_count, mag_count);
            }
            else
            {
                BOOT_LOG_W(kMod, "readState 失败（Slot 空）");
            }
            predict_count = 0;
            baro_count = 0;
            tof_count = 0;
            flow_count = 0;
        }
    }
}

#endif // !SIL_HOST