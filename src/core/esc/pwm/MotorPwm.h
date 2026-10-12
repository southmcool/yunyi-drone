
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include "core/esc/MotorDriver.h"

class MotorPwm : public MotorDriver
{
public:
    // 可配置参数（在调用 init() 之前设置）
    uint8_t pwmRes = 8;      // PWM 分辨率（8 位，0~255）
    uint32_t pwmFreq = 20000; // PWM 频率，建议 5~20kHz
    uint8_t minPwmDuty = 10; // 最低有效占空比（避免低速死区），10/255≈4%

    bool hwInit() override;
    void vSetMotorOutput(uint8_t index, uint16_t throttle) override;
    void vStopMotor(uint8_t index) override;

private:
    static const uint8_t motorPins[4]; // 引脚定义，见 cpp 文件
    uint8_t throttleToPwm(uint16_t throttle) const;
};

// 全局对象
extern MotorPwm motorPwm;

// 兼容旧代码：所有 escDShot.xxx() 调用将自动变为 motorPwm.xxx()
// #define escDShot motorPwm

#endif