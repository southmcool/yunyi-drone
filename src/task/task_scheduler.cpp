/**
 * @title  任务调度器实现
 * @author: vaie
 * @date: 2026-10-12
 *
 * 所有任务在 setup() 阶段一次性创建。
 */

#include "task_scheduler.h"

// ============================================================================
//  全局句柄定义
// ============================================================================

EventGroupHandle_t xSystemInitEventGroup = nullptr;

SemaphoreHandle_t xSPIMutex = NULL;
SemaphoreHandle_t xUartDataReadySem = NULL;
SemaphoreHandle_t xIICMutex = NULL;
SemaphoreHandle_t xParamMutex = NULL;

QueueHandle_t xTofQueue = NULL;
QueueHandle_t xFlowDataQueue = NULL;
// ============================================================================
//  任务入口（各 .cpp 定义）
// ============================================================================

extern void imu_task(void *pv);
extern void imu_fft_task(void *pv);
extern void magbaro_task(void *pv);
extern void estimator_task(void *pv); // ★ 新增
// extern void sensor_calib_task(void *pv);
extern void stabilizer_task(void *pv);
extern void motor_output_task(void *pv);
extern void vl53l1x_task(void *pv);
extern void pmw3901_task(void *pv);
extern void telemetry_task(void *pv);

// ============================================================================
//  内部辅助
// ============================================================================

namespace
{

    // 统一的任务创建：失败时打日志 + 上报错误
    bool spawn(TaskFunction_t fn, const char *name,
               uint32_t stack, UBaseType_t prio, BaseType_t core)
    {
        const BaseType_t ok =
            xTaskCreatePinnedToCore(fn, name, stack, nullptr, prio, nullptr, core);
        if (ok != pdPASS)
        {
            BOOT_LOG_E("TASK", "创建失败: %s (stack=%u prio=%u core=%d)",
                       name, (unsigned)stack, (unsigned)prio, (int)core);
            return false;
        }
        return true;
    }

} // namespace

// ============================================================================
//  初始化
// ============================================================================

void vTaskSchedulerInit(void)
{
    // ---- 事件组 ----
    xSystemInitEventGroup = xEventGroupCreate();
    if (!xSystemInitEventGroup)
    {
        BOOT_LOG_E("BOOT", "事件组创建失败，系统无法启动");
        SystemStatus::raiseError(ERR_STABILIZER_FAIL); // 借用：整体启动失败
        SystemStatus::setState(SystemState::ERROR);
        return;
    }

    xTofQueue = xQueueCreate(1, sizeof(tofMeasurement_t));
    xFlowDataQueue = xQueueCreate(16, sizeof(flowMeasurement_t));

    xSPIMutex = xSemaphoreCreateMutex();
    xUartDataReadySem = xSemaphoreCreateBinary();
    xIICMutex = xSemaphoreCreateMutex();
    xParamMutex = xSemaphoreCreateMutex();

    BOOT_LOG_I("BOOT", "创建任务...");

    // ---- Core 0：飞控硬实时 ----
    //   优先级由高到低：stabilizer(6) > imu(5) > motor_out(4)
    //   · stabilizer 需要最快响应融合结果
    //   · imu 次之（1kHz 采样与信号链）
    //   · motor_out 再次（DShot 输出，硬件外设驱动，1kHz 喂值即可）

    if (!spawn(imu_task, "imu", 8192, 5, 0))
    {
        SystemStatus::raiseError(ERR_IMU_INIT_FAIL);
    }
    // Core 0：飞控硬实时
    if (!spawn(stabilizer_task, "stabilizer", 8192, 6, 0))
    {
        SystemStatus::raiseError(ERR_STABILIZER_FAIL);
    }
    if (!spawn(motor_output_task, "motor_out", 6144, 4, 0))
    {
        SystemStatus::raiseError(ERR_MOTOR_INIT_FAIL);
    }

    // ---- Core 1：传感器 + 遥测 + 后台 ----
    //   · magbaro(4)：UART 状态机，任务轻但频率高
    //   · tof/flow(3)：中频
    //   · sensor_calib(3)：低频，但校准期间会阻塞 45s
    //   · telemetry(1)：低频，允许被抢占
    //   · imu_fft(2)：FFT 后台，必须高于 telemetry 否则可能被饿死

    spawn(magbaro_task, "magbaro", 5120, 4, 1);
    spawn(estimator_task, "estimator", 8192, 3, 1); // ★ 新增
    spawn(vl53l1x_task, "vl53l1xs", 6144, 3, 1);
    spawn(pmw3901_task, "pmw3901", 6144, 3, 1);
    // spawn(sensor_calib_task, "sensor_calib", 4096, 3, 1);
    spawn(imu_fft_task, "imu_fft", 8192, 2, 1);
    spawn(telemetry_task, "telemetry", 6144, 1, 1);

    BOOT_LOG_S("BOOT", "所有任务已创建");
}