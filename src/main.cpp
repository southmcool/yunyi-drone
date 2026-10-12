/**
 * @title  主入口
 * @author: vaie
 * @date: 2026-10-12
 *
 * Arduino setup() / loop() —— 极薄一层，只负责：
 *   1. 串口初始化（BOOT_LOG 依赖）
 *   2. 打印启动横幅
 *   3. 调 vTaskSchedulerInit() 创建所有任务
 *   4. 让 loop() 停摆（FreeRTOS 任务接管一切）
 *
 * 所有实际初始化都在各任务内部 —— main 不持有任何传感器/总线/信号链。
 *
 * 依赖：tasks / util / core/system，只在实机编译。
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "application/main_app.h"
#include "task/task_scheduler.h"

// ============================================================================
//  setup —— Arduino 入口，在 loopTask 里调用
// ============================================================================

MainApplication mainA;
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000);

  mainA.initialize();

  Serial.println("ESP32-S3 位置开发分支 -v1 已启动！");

  // ---- 3. 初始状态 ----
  SystemStatus::setState(SystemState::BOOT);

  // ---- 4. 创建所有任务 ----
  vTaskSchedulerInit();

  // ---- 5. 状态转 READY（系统进入就绪，等待解锁）----
  //   注意：这里只是"任务已创建"，不代表所有传感器都就绪。
  //   稳定器任务会在传感器就绪后自行推进状态。
  SystemStatus::setState(SystemState::READY);
  BOOT_LOG_S("BOOT", "系统进入 READY 状态222");

  // ---- 6. loopTask 自杀 ----
  //   所有功能都在独立任务里，Arduino 的 loop 任务没用了。
  //   保持它空转也会消耗 CPU + 栈，直接删除释放 8KB。
  //   （Arduino 框架允许在 setup() 里删 loopTask，参考 ESP-IDF 官方例程）
  vTaskDelete(nullptr);
}

// ============================================================================
//  loop —— 不会执行到（setup 已删除 loopTask）
// ============================================================================

void loop()
{
  // 空壳：setup() 里 vTaskDelete(nullptr) 已经删除当前任务，
  // 这个函数永远不被调度器执行。保留它是为了 Arduino 框架编译通过。
  vTaskDelay(pdMS_TO_TICKS(1000));
}
