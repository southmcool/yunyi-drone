#ifndef ESC_DSHOT_H
#define ESC_DSHOT_H

#include "core/esc/MotorDriver.h"
#include <DShotRMT.h>

class EscDShot : public MotorDriver
{
public:
    bool hwInit() override;
    void vSetMotorOutput(uint8_t index, uint16_t throttle) override;
    void vStopMotor(uint8_t index) override;

protected:
    void onArmed() override;       // 实现在 .cpp
    void onDisarm() override;      // 实现在 .cpp
    void onArmingStart() override; // 实现在 .cpp

private:
    DShotRMT *motors[4] = {nullptr};
    static const gpio_num_t motor_pins[4];
};

extern EscDShot escDShot;

#endif