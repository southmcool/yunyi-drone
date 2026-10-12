#include "shared_data.h"

uint16_t g_motor_throttle[4] = {0};
portMUX_TYPE g_motor_throttle_lock = portMUX_INITIALIZER_UNLOCKED; // 静态初始化器

void motor_throttle_update(const uint16_t throttle[4])
{
    portENTER_CRITICAL_SAFE(&g_motor_throttle_lock);
    memcpy(g_motor_throttle, throttle, sizeof(g_motor_throttle));
    portEXIT_CRITICAL_SAFE(&g_motor_throttle_lock);
}

void motor_throttle_get_latest(uint16_t out[4])
{
    portENTER_CRITICAL_SAFE(&g_motor_throttle_lock);
    memcpy(out, g_motor_throttle, sizeof(g_motor_throttle));
    portEXIT_CRITICAL_SAFE(&g_motor_throttle_lock);
}