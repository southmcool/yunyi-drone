#include "MotorPwm.h"

MotorPwm motorPwm;

// 根据你的实际硬件连接修改此数组（每个电机一个 INA 引脚）
const uint8_t MotorPwm::motorPins[4] = {13, 12, 10, 11}; // fl,fr,bl,br

bool MotorPwm::hwInit()
{
    for (int i = 0; i < 4; i++)
    {
        ledcAttach(motorPins[i], pwmFreq, pwmRes);
        ledcWrite(motorPins[i], 0);
    }
    return true;
}

uint8_t MotorPwm::throttleToPwm(uint16_t throttle) const
{
    // 映射 48~2047 → minPwmDuty~255，避免低速死区
    if (throttle < 48)
        throttle = 48;
    if (throttle > 2047)
        throttle = 2047;

    uint32_t range = 255 - minPwmDuty;
    return minPwmDuty + (uint8_t)((uint32_t)(throttle - 48) * range / (2047 - 48));
}

void MotorPwm::vSetMotorOutput(uint8_t index, uint16_t throttle)
{
    if (!isInitialized() || index >= 4)
        return;

    // 未解锁时强制停止
    if (_armState != ArmState::ARMED)
    {
        vStopMotor(index);
        return;
    }

    // 限幅
    if (throttle < 48)
        throttle = 48;
    else if (throttle > 2047)
        throttle = 2047;

    ledcWrite(motorPins[index], throttleToPwm(throttle));
}

void MotorPwm::vStopMotor(uint8_t index)
{
    if (index >= 4)
        return;
    ledcWrite(motorPins[index], 0);
}