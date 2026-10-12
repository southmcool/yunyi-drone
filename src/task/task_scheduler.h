/**
 * @title  任务调度器
 * @author: vaie
 * @date: 2026-10-12
 *
 * 集中创建所有任务 + 管理全局事件位。
 *
 * ---- 依赖图（无环，必须） ----
 *
 *   必需路径：
 *     imu_task ──→ IMU_READY ──→ stabilizer_task ──→ STABILIZER_READY
 *                                     │
 *                                     └──→ motor_output_task ──→ MOTOR_READY
 *
 *   可选传感器（不阻塞稳定器启动）：
 *     magbaro_task  ──→ MAGBARO_READY
 *     vl53l1x_task  ──→ TOF_READY
 *     pmw3901_task  ──→ FLOW_READY
 *
 *   后台任务（自启动，不置位）：
 *     imu_fft_task / sensor_calib_task / telemetry_task
 *
 * ---- 设计原则 ----
 *   · 稳定器**只等 IMU**；MagBaro / ToF / Flow 是可选设备，
 *     缺失不应阻塞起飞（降级运行：无磁力计 yaw 会漂，无 ToF/Flow 无定点）。
 *   · 任务入口不持任何全局句柄 —— 各任务自己在内部创建资源
 *     （SpiBus、AtkImu901 等），scheduler 只负责"创建任务"。
 *   · 任务创建失败时通过 SystemStatus 上报告警，不静默继续。
 */
#pragma once
#include "core/system/shared_data.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "util/boot_log.h"
#include "core/system/system_status.h"
#include "type/stabilizer_types.h" // state_t
// ---------------------------------------------------------------------------
//  初始化完成位
// ---------------------------------------------------------------------------
#define INIT_BIT_IMU_READY (1 << 0)        // imu_task 置位
#define INIT_BIT_MAGBARO_READY (1 << 1)    // magbaro_task 置位
#define INIT_BIT_TOF_READY (1 << 2)        // vl53l1x_task 置位
#define INIT_BIT_FLOW_READY (1 << 3)       // pmw3901_task 置位
#define INIT_BIT_STABILIZER_READY (1 << 4) // stabilizer_task 置位
#define INIT_BIT_MOTOR_READY (1 << 5)      // motor_output_task 置位

// ---------------------------------------------------------------------------
//  稳定器启动所需的最小传感器集合
//   ★ 只等 IMU —— 可选传感器不进必需位，避免"MS901M 坏了就飞不起来"。
//   ★ 不得包含 STABILIZER_READY / MOTOR_READY（会构成依赖环）。
// ---------------------------------------------------------------------------
#define INIT_BIT_SENSORS_REQUIRED (INIT_BIT_IMU_READY)

extern SemaphoreHandle_t xSPIMutex;
extern SemaphoreHandle_t xIICMutex;
extern SemaphoreHandle_t xParamMutex;
extern SemaphoreHandle_t xUartDataReadySem;

extern QueueHandle_t xTofQueue;      // 深度=2，覆写模式
extern QueueHandle_t xFlowDataQueue; // ★ 新增

// ---------------------------------------------------------------------------
//  全局事件组
// ---------------------------------------------------------------------------
extern EventGroupHandle_t xSystemInitEventGroup;

// ---------------------------------------------------------------------------
//  初始化：创建事件组 + 创建所有任务
//   在 Arduino setup() 里调用一次。返回后所有任务已就绪（不一定已完成初始化）。
// ---------------------------------------------------------------------------
void vTaskSchedulerInit(void);