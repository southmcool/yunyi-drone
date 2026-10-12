#pragma once
#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"

// 全局油门缓冲区
extern uint16_t g_motor_throttle[4];
// 保护油门缓冲区的自旋锁
extern portMUX_TYPE g_motor_throttle_lock;

// 控制任务调用：写入最新混控值（线程安全）
void motor_throttle_update(const uint16_t throttle[4]);

// 电机任务调用：读取最新值（线程安全）
void motor_throttle_get_latest(uint16_t out[4]);