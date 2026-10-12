#include "task/task_scheduler.h"
#include <VL53L1X.h> // 确保已安装此库[reference:10]
#include <Wire.h>
#include "config/gpio_config.h"

// 外部声明（由主程序提供）

// 任务句柄（可选）
TaskHandle_t xVL53L1XTaskHandle = NULL;

// 传感器对象（静态或全局）
static VL53L1X vl53l1x;

namespace
{

    // VL53L1X 噪声模型（Crazyflie zRanger2.c 同款）
    constexpr float kExpPointA = 2.5f;
    constexpr float kExpStdA = 0.0025f; // [m] @ 2.5m
    constexpr float kExpPointB = 4.0f;
    constexpr float kExpStdB = 0.2f; // [m] @ 4m
    const float kExpCoeff = logf(kExpStdB / kExpStdA) / (kExpPointB - kExpPointA);

    inline float tofStdDev(float distance_m)
    {
        return kExpStdA * (1.0f + expf(kExpCoeff * (distance_m - kExpPointA)));
    }

} //

// 任务函数
void vl53l1x_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTaskPeriod = pdMS_TO_TICKS(33); // ~29.4Hz` 周期，与连续测量匹配

    // 🎯 关键：阻塞等待 IMU 初始化完成，超时 10 秒
    Serial.println("tof: 等待 IMU 初始化完成...");
    // ★ 等待位修正：ToF 只依赖 IMU（共用 I2C 总线与初始化时序），
    //   与气压计/磁力计模块（PA_IMU_READY）没有依赖关系，等它属于语义错配。
    //   两个位目前在 imu_task 中同批置位，所以行为等价、零风险，但依赖图变正确了。
    EventBits_t uxBits = xEventGroupWaitBits(
        xSystemInitEventGroup,
        INIT_BIT_IMU_READY,  // 等待的位
        pdFALSE,             // 不清除该位（其他任务可能也需要）
        pdTRUE,              // 必须全部置位才返回
        pdMS_TO_TICKS(10000) // 超时 10s
    );

    if ((uxBits & INIT_BIT_IMU_READY) != INIT_BIT_IMU_READY)
    {
        Serial.println("❌ tof: 等待 IMU 就绪超时，删除任务");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("✅ tof: IMU 已就绪，开始初始化传感器");

    // ---- 1. 初始化 I2C 总线（注意：如果多个设备共用同一总线，此操作应只执行一次） ----
    // 建议将 Wire.begin() 放在系统初始化阶段，这里只做检查
    // 若需要在此处初始化，请确保使用互斥体
    if (xSemaphoreTake(xIICMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Wire.begin(VL53L1X_SDA, VL53L1X_SCL); // SDA=37, SCL=38（根据您的硬件修改）
        Wire.setClock(400000);
        xSemaphoreGive(xIICMutex);
    }
    else
    {
        Serial.println("❌ VL53L1X: 无法获取 I2C 互斥体进行初始化，删除任务");
        vTaskDelete(NULL);
    }

    // ---- 2. 初始化 VL53L1X 传感器 ----
    vl53l1x.setTimeout(500);
    bool initOk = false;
    if (xSemaphoreTake(xIICMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        initOk = vl53l1x.init();
        xSemaphoreGive(xIICMutex);
    }

    if (!initOk)
    {
        Serial.println("❌ VL53L1X: 检测/初始化失败！");
        vTaskDelete(NULL);
    }

    xEventGroupSetBits(xSystemInitEventGroup, INIT_BIT_TOF_READY);

    // 配置传感器
    vl53l1x.setDistanceMode(VL53L1X::Long);
    vl53l1x.setMeasurementTimingBudget(33000); // 33ms，Medium模式下足够覆盖2-3m
    vl53l1x.startContinuous(25);               // 匹配新周期，预期可达~30Hz

    Serial.println("✅ VL53L1X 初始化完成，开始连续测量");

    // ---- 3. 统计与监控变量 ----
    uint32_t ulLoopCount = 0;        // 任务循环次数
    uint32_t ulDataReadyCount = 0;   // dataReady 为真的次数
    uint32_t ulI2CFailCount = 0;     // 获取 I2C 锁失败次数
    uint32_t ulQualityFailCount = 0; // 质量不合格次数
    uint32_t ulStepRejectCount = 0;  // 跳变剔除次数
    uint32_t ulSuccessCount = 0;     // 成功入队次数

    uint32_t distanceCount = 0; // 平均值

    uint32_t ulLastPrintTime = millis();
    uint32_t ulLastDataTime = millis();
    const uint32_t TIMEOUT_MS = 3000; // 3秒无数据视为离线

    // ---- 4. 主循环 ----
    for (;;)
    {
        // 严格保持 34ms (~29.4Hz) 唤醒周期
        vTaskDelayUntil(&xLastWakeTime, xTaskPeriod);
        ulLoopCount++;

        bool readOk = false;
        uint16_t distance = 0;

        // 1. 尝试获取 I2C 锁，仅保护必要的硬件读取操作
        if (xSemaphoreTake(xIICMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            // 原子化检查并读取
            if (vl53l1x.dataReady())
            {
                distance = vl53l1x.read();
                ulDataReadyCount++; // ★ 补上
                // 使用库底层的超时标志判断，而非简单的非零判断
                readOk = !vl53l1x.timeoutOccurred();
            }

            // 立即释放锁！绝不在锁内进行串口打印或队列发送
            xSemaphoreGive(xIICMutex);
        }
        else
        {
            ulI2CFailCount++;
        }

        // 2. 数据处理与分发（在锁外部执行，不阻塞 I2C）
        if (readOk)
        {
            // ★ 修复：status 类型显式转换
            auto rangeStatus = static_cast<VL53L1X::RangeStatus>(vl53l1x.ranging_data.range_status);

            // ---- 基于 RangeStatus 的质量分级 ----
            uint8_t quality = 0;
            switch (rangeStatus)
            {
            case VL53L1X::RangeValid:
                quality = 3;
                break;
            case VL53L1X::RangeValidMinRangeClipped:
            case VL53L1X::RangeValidNoWrapCheckFail:
                quality = 2;
                break;
            case VL53L1X::SigmaFail:
            case VL53L1X::SignalFail:
                quality = 1;
                break;
            default:
                quality = 0;
                break;
            }

            // ---- 异常值剔除（仅在有效数据上执行）----
            if (quality > 0)
            {
                if (distance < 1 || distance > 4000)
                {
                    ulQualityFailCount++;
                    quality = 0;
                }
                else
                {
                    // ★ 只保留这一个 static 变量
                    // ★ B-02（R13）：跳变剔除加"连续 N 帧解锁"。
                    //   旧逻辑缺陷：lastValidDist 只在被接受时更新。若真实高度
                    //   在一个采样周期(50ms)内阶跃 >0.5m（飞过台阶/家具边、
                    //   快速下坠），新读数与旧参考永远差 >500mm → 此后每一帧
                    //   （即使完全有效）都被拒绝，lastValidDist 永不更新，
                    //   ToF 链路永久失效且无告警（latch-up）。
                    //   修复：连续 5 帧（250ms）被拒后判定为真实阶跃，
                    //   解除锁存——以当前读数为新参考并放行本帧。
                    //   参考：PX4 对测距类量测用可恢复的创新门控（连续失败→
                    //   临时抑制→超时重试），不做传感器级单向锁存。
                    static uint16_t lastValidDist = 0;
                    static uint8_t step_reject_cnt = 0;
                    if (lastValidDist > 0 && abs((int)distance - (int)lastValidDist) > 500)
                    {
                        if (++step_reject_cnt >= 5)
                        {
                            // 真实阶跃：解除锁存，接受当前值
                            lastValidDist = distance;
                            step_reject_cnt = 0;
                        }
                        else
                        {
                            quality = 0;
                            ulStepRejectCount++;
                        }
                    }
                    else
                    {
                        lastValidDist = distance;
                        step_reject_cnt = 0;
                    }
                }
            }

            // ---- 仅有效数据才入队 ----
            if (quality > 0 && xTofQueue != NULL)
            {
                tofMeasurement_t tofMsg = {};
                tofMsg.timestamp = (uint32_t)micros();
                tofMsg.distance = (float)distance * 0.001f; // mm → m
                tofMsg.stdDev = tofStdDev(tofMsg.distance); // ★ 指数模型

                xQueueOverwrite(xTofQueue, &tofMsg);
                distanceCount += distance;
                ulSuccessCount++;
                ulLastDataTime = millis();
            }
            else
            {
                //  如果 quality == 0 但前面没计过，可以在这里补一个
            }
        }

        // ---- 每秒统计一次 ----
        uint32_t now = millis();
        if (now - ulLastPrintTime >= 30000)
        {
            uint32_t dt = now - ulLastPrintTime;
            float hz = ulSuccessCount * 1000.0f / dt;
            float avgDist = (ulSuccessCount > 0) ? ((float)distanceCount / ulSuccessCount) : 0.0f;
            Serial.printf(
                "ToF统计: 循环=%lu, dataReady=%lu, I2C失败=%lu, 质量失败=%lu, 跳变拒绝=%lu, 入队=%lu, 有效频率=%.1f Hz, 离线=%s,平均高度=%.2f\n",
                ulLoopCount,
                ulDataReadyCount,
                ulI2CFailCount,
                ulQualityFailCount,
                ulStepRejectCount,
                ulSuccessCount,
                hz,
                (now - ulLastDataTime > TIMEOUT_MS) ? "是" : "否",
                avgDist);

            // 清零本秒增量，下一轮重新统计
            ulLoopCount = 0;
            ulDataReadyCount = 0;
            ulI2CFailCount = 0;
            ulQualityFailCount = 0;
            ulStepRejectCount = 0;
            ulSuccessCount = 0;
            distanceCount = 0;
            ulLastPrintTime = now;
        }
    }
}