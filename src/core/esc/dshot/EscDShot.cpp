#include "EscDShot.h"
#include "driver/rmt_tx.h" // 或对应头文件
#include "esp_rom_sys.h"
EscDShot escDShot;

const gpio_num_t EscDShot::motor_pins[4] = {GPIO_NUM_11, GPIO_NUM_38, GPIO_NUM_10, GPIO_NUM_39}; // fl,fr,bl,br

bool EscDShot::hwInit()
{
    Serial.println("=== RMT 通道诊断 ===");
    Serial.printf("SOC_RMT_TX_CANDIDATES_PER_GROUP = %d\n", SOC_RMT_TX_CANDIDATES_PER_GROUP);

    for (int i = 0; i < 4; i++)
    {
        motors[i] = new DShotRMT(motor_pins[i], DSHOT300, false);
        dshot_result_t res = motors[i]->begin();
        if (!res.success)
        {

            Serial.printf("❌ 电机 %d 号初始化失败, code=%d\n", i, res.result_code);
            // 回滚已创建的实例
            for (int j = 0; j <= i; j++)
            {
                delete motors[j];
                motors[j] = nullptr;
            }
            return false;
        }
    }
    return true;
}

void EscDShot::vSetMotorOutput(uint8_t index, uint16_t throttle)
{
    // ✅ 边界检查放最前
    if (!isInitialized() || index >= 4)
        return;

    // ✅ stop 标志优先（不再自我置位）
    if (_stop_requested[index])
    {
        return;
    }

    if (throttle < 48)
    {
        vStopMotor(index);
        return;
    }

    if (_armState != ArmState::ARMED)
    {
        vStopMotor(index);
        return;
    }

    if (throttle < 48)
        throttle = 48;
    else if (throttle > 2047)
        throttle = 2047;

    motors[index]->sendThrottle(throttle);
}

void EscDShot::vStopMotor(uint8_t index)
{
    if (index >= 4 || !motors[index])
        return;
    // ✅ 删掉 _stop_requested[index] = true;
    motors[index]->sendCustomCommand(DSHOT_CMD_MOTOR_STOP, 1, 0);
}

void EscDShot::onArmed()
{
    for (int i = 0; i < 4; i++)
        _stop_requested[i] = false;
}

void EscDShot::onDisarm()
{
    for (int i = 0; i < 4; i++)
        _stop_requested[i] = true;
}

void EscDShot::onArmingStart()
{
    for (int i = 0; i < 4; i++)
        _stop_requested[i] = false;
}