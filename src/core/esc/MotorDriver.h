#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

enum class ArmState
{
    DISARMED,
    ARMING,
    ARMED
};

class MotorDriver
{
public:
    virtual ~MotorDriver() = default;

    // ---- 兼容旧接口，调用此函数等同于 init() ----
    bool vInitEsc() { return init(); }

    // 初始化（硬件 + 状态）
    bool init();

    // 纯虚函数：由派生类实现硬件相关操作
    virtual bool hwInit() = 0;
    virtual void vSetMotorOutput(uint8_t index, uint16_t throttle) = 0; // 设置油门
    virtual void vStopMotor(uint8_t index) = 0;                         // 立即停止单个电机

    // 通用流程（基类实现，无需重写）
    bool startArming();
    void updateArming();
    void vDisarmMotors();
    void vStopAllMotors();
    void emergencyStop();
    void vSetAllMotors(const uint16_t throttles[4]);

    // 状态查询
    ArmState getArmState() const { return _armState; }
    bool isArmed() const { return _armState == ArmState::ARMED; }
    bool isInitialized() const { return _initialized; }

    void requestStop(uint8_t index)
    {
        if (index < 4)
            _stop_requested[index] = true;
    }
    void clearStop(uint8_t index)
    {
        if (index < 4)
            _stop_requested[index] = false;
    }

protected:
    // 可选钩子（派生类可重写）
    virtual void onArmingStart() {}
    virtual void onArmed() {}
    virtual void onDisarm() {}
    bool _stop_requested[4] = {false, false, false, false};
    ArmState _armState = ArmState::ARMED;

private:
    bool _initialized = false;
    uint32_t _armingStartTime = 0;
};

#endif