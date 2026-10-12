// #include "modules/esc/pwm/MotorPwm.h"
#include "core/esc/dshot/EscDShot.h"
#include "task/task_scheduler.h"

// 假设你的上锁/解锁由遥控器或状态机控制，这里模拟外部变量
extern bool system_armed; // 应由安全状态机统一管理

void motor_output_task(void *pvParameters)
{
    // 建议使用 vTaskDelayUntil 以保证严格周期（如 4kHz）
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500hz

    Serial.println("电机任务: 等待 传感器 初始化完成...");
    EventBits_t uxBits = xEventGroupWaitBits(
        xSystemInitEventGroup,
        INIT_BIT_STABILIZER_READY, // 等待的位
        pdFALSE,                   // 不清除该位（其他任务可能也需要）
        pdTRUE,                    // 只需匹配任一位即可
        pdMS_TO_TICKS(20000)       // 超时 20s
    );

    if (!(uxBits & INIT_BIT_STABILIZER_READY))
    {
        Serial.println("❌ 电机任务: 等待 传感器 就绪超时，删除任务");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("✅ 电机任务: 传感器 已就绪，开始初始化传感器");

    if (!escDShot.vInitEsc())
    {
        // 初始化失败，停止任务
        Serial.println("❌ 电机初始化失败! 删除任务");
        vTaskDelete(NULL);
        return;
    }
    Serial.println("✅ 电机初始化和校准完成! ");

    // ★ 新增：ESC 真正就绪后才置位。
    //   旧代码全工程没有任何一处置过 INIT_BIT_MOTOR_READY，导致该位形同虚设；
    //   现在它是"ESC 已初始化成功"的真实观测点，可供遥测/自检使用。
    xEventGroupSetBits(xSystemInitEventGroup, INIT_BIT_MOTOR_READY);
    uint16_t throttle[4] = {0, 0, 0, 0};
    uint16_t last_throttle[4] = {0, 0, 0, 0}; // 用于比较变化

    // 统计变量
    uint32_t ulChangeCount = 0; // 油门变化次数
    uint32_t ulLastPrintTime = 0;
    for (;;)
    {
        // 安全状态同步
        if (system_armed && escDShot.getArmState() == ArmState::DISARMED)
        {
            escDShot.startArming(); // 启动解锁流程（非阻塞）
        }
        else if (!system_armed && escDShot.getArmState() != ArmState::DISARMED)
        {
            escDShot.vDisarmMotors();
        }

        // 驱动解锁状态机（每个周期调用，推进解锁流程）
        if (escDShot.getArmState() == ArmState::ARMING)
        {
            escDShot.updateArming();
        }

        // 2. 获取 PID 控制任务写入的最新油门
        motor_throttle_get_latest(throttle);

        // 3. 统计油门是否发生变化
        // bool bChanged = false;
        // for (int i = 0; i < 4; i++)
        // {
        //     if (throttle[i] > 48)
        //     {
        //         Serial.printf("📊电机%d 油门 %d\n", i, throttle[i]);
        //     }
        // }
        // if (bChanged)
        // {
        //     ulChangeCount++;
        // }

        // 5. 每秒输出一次统计信息
        // uint32_t ulNow = millis();
        // if (ulNow - ulLastPrintTime >= 1000)
        // {
        //     for (int i = 0; i < 4; i++)
        //     {
        //         Serial.printf("📊 油门 %d :%d", i, throttle[i]);
        //         Serial.printf("\n");
        //     }
        //     // Serial.printf("📊 油门更新频率: %d Hz (期望接近 PID 更新频率)\n", ulChangeCount);
        //     ulChangeCount = 0;
        //     ulLastPrintTime = ulNow;
        // }

        escDShot.vSetAllMotors(throttle);

        // 等待下一个周期（精确节拍）
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}