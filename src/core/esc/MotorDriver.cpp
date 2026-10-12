#include "MotorDriver.h"

// 外部依赖（根据你的项目保留或注释）
extern void motor_throttle_update(const uint16_t throttles[4]);
extern bool system_armed;

bool MotorDriver::init()
{
    if (_initialized)
        return true;
    if (hwInit())
    {
        _initialized = true;
        return true;
    }
    return false;
}

bool MotorDriver::startArming()
{
    // 只允许从 DISARMED 状态启动解锁流程
    if (!_initialized || _armState != ArmState::DISARMED)
        return false;

    _armState = ArmState::ARMING;
    _armingStartTime = millis();
    Serial.println("开始解锁");
    onArmingStart();
    for (int i = 0; i < 4; i++)
    {
        clearStop(i);
    }
    return true;
}

void MotorDriver::updateArming()
{
    if (!_initialized || _armState != ArmState::ARMING)
        return;

    uint32_t elapsed = millis() - _armingStartTime;
    if (elapsed < 1000)
    {
        // 解锁第一阶段：持续发送停止命令
        for (int i = 0; i < 4; i++)
        {
            vStopMotor(i);
        }
    }
    else if (elapsed >= 1150)
    {
        _armState = ArmState::ARMED;
        Serial.println("解锁完成");
        onArmed();
    }
}

void MotorDriver::vDisarmMotors()
{
    if (!_initialized)
        return;

    // 严格遵循原始顺序：先同步外部油门归零（此时可能还处于 ARMED）
    uint16_t throttle[4] = {0};
    motor_throttle_update(throttle);

    // 再改变内部状态
    _armState = ArmState::DISARMED;

    // 最后执行硬件停止
    for (int i = 0; i < 4; i++)
    {
        vStopMotor(i);
    }
    Serial.println("上锁");
    onDisarm();
}

void MotorDriver::emergencyStop()
{
    if (!_initialized)
        return;
    system_armed = false;
    vDisarmMotors();
}

void MotorDriver::vStopAllMotors()
{
    if (!_initialized)
        return;

    uint16_t throttle[4] = {0, 0, 0, 0};
    motor_throttle_update(throttle);

    for (int i = 0; i < 4; i++)
    {
        requestStop(i); // ← 显式置位，与调用方意图一致
        vStopMotor(i);
    }
}

void MotorDriver::vSetAllMotors(const uint16_t throttles[4])
{
    if (!_initialized)
        return;
    for (int i = 0; i < 4; i++)
    {
        vSetMotorOutput(i, throttles[i]);
    }
}