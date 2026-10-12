#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "type/stabilizer_types.h"
#include "single_axis_dual_loop.h"
#include "safety_monitor.h"
#include <ArduinoJson.h>
#include "util/attiude_math.h"
#include "altitude_controller.h"
#include "position_controller.h"
class AttitudeController
{
private:
    // 三轴控制器
    SingleAxisDualLoop roll_controller_;
    SingleAxisDualLoop pitch_controller_;
    SingleAxisDualLoop yaw_controller_;

    // 安全监控
    SafetyMonitor safety_monitor_;

    // 状态
    bool enabled_;
    uint32_t last_update_time_;
    uint32_t ulLastPrintTime;
    // debug_info_t debug_info_;

    // // FusionConventionNwu, /* North (X), West (Y), Up (Z) */
    // // CW(顺时针)Yaw -	CCW(逆时针)Yaw +
    // const float mix[4][3] = {
    //     {+1.0f, -1.0f, -1.0f}, // FL (CW) FL:
    //     {-1.0f, -1.0f, +1.0f}, // FR (CCW) FR:
    //     {+1.0f, +1.0f, +1.0f}, // BL (CCW) BL:
    //     {-1.0f, +1.0f, -1.0f}  // BR (CW) BR:
    // };

    // FusionConventionNed
    // CW(顺时针)Yaw -	CCW(逆时针)Yaw +
    const float mix[4][3] = {
        {+1.0f, +1.0f, +1.0f}, // FL (CCW) FL:
        {-1.0f, +1.0f, -1.0f}, // FR (CW) FR:
        {+1.0f, -1.0f, -1.0f}, // BL (CW) BL:
        {-1.0f, -1.0f, +1.0f}  // BR (CCW) BR:
    };

public:
    AttitudeController();
    PositionController positionController_; // ← 必须是类成员
    // PID JSON 接口
    StaticJsonDocument<200> getPIDJson() const;
    StaticJsonDocument<200> getPIDJsonCompact() const;   // 添加这行
    void getPIDJson(StaticJsonDocument<200> &doc) const; // 添加这行（推荐）
    void printPIDJson() const;                           // 添加这行

    void getCurrentGains(float outer_gains[3][3], float inner_gains[3][3]) const;

    // 主流程接口
    bool update(const state_t &state, const setpoint_t &setpoint, control_t &output, float dt);

    // ★R77e 地面积分保持：true 时三轴 PID 积分器每拍清零（P/D 照常运行），
    //   防止地面静止误差造成差动慢速积累、离地瞬间满舵翻扣
    void setGroundHold(bool hold)
    {
        roll_controller_.setIntegralHold(hold);
        pitch_controller_.setIntegralHold(hold);
        yaw_controller_.setIntegralHold(hold);
    }

    // 配置接口
    void setGains(const float outer_gains[3][3], const float inner_gains[3][3]);
    void setLimits(float max_angle_deg, float max_angle_rate_deg, float max_output);

    // 状态管理
    void reset();
    void enable() { enabled_ = true; }
    void disable() { enabled_ = false; }
    bool isEnabled() const { return enabled_; }

    // 调试接口
    // const debug_info_t &getDebugInfo() const { return debug_info_; }

private:
    float calculateDeltaTime(uint32_t current_time);
    void updateDebugInfo();
    bool validateInputs(const state_t &state, const setpoint_t &setpoint);
};
// 定义全局电机对象
extern AttitudeController attitudeController;
#endif