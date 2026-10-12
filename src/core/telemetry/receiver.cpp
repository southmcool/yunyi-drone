#include "receiver.h"
#include "task/task_scheduler.h" // 引入 requestIMUCalibration 声明
#include <math.h>                // for M_PI

// 最大姿态角配置（根据机型调整）
static constexpr float MAX_ROLL_ANGLE_RAD = 45.0f * M_PI / 180.0f;  // 0.7854 rad
static constexpr float MAX_PITCH_ANGLE_RAD = 45.0f * M_PI / 180.0f; // 0.7854 rad
static constexpr float MAX_YAW_RATE_RADS = 120.0f * M_PI / 180.0f;  // 2.094 rad/s ⚠️ Yaw是角速度！

// ===== 高度控制状态 =====
float _altitude_setpoint = 0.0f;      // 目标高度 (米)
float _altitude_offset = 0.0f;        // 累积高度偏移 (米)
float _last_altitude_update_time = 0; // 上次更新时间 (ms)
bool _altitude_initialized = false;
// 高度控制参数
float MAX_ALTITUDE_CHANGE_RATE = 0.2f; // 最大升降速度 (m/s)
float ALT_DEADZONE = 0.05f;            // 摇杆死区

// ✅ 适用于 [-1.0, +1.0] 输入的映射函数
static inline float mapNormalizedToRange(float value, float max_range)
{
    // value: -1.0~+1.0 → output: -max_range ~ +max_range
    // 无需再乘2减1，直接线性缩放即可
    return value * max_range;
}

portMUX_TYPE g_receiver_lock = portMUX_INITIALIZER_UNLOCKED;

bool system_armed = false; // 全局安全状态锁

setpoint_t xGlobalSetpoint = {};

bool serial_send_flag = false;
bool serial_print_flag = false;
bool calib_mode = false;
bool arm()
{
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    system_armed = true;
    setThrust(0);
    resetAltitudeState(); // ✅ 重置高度状态
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);

    // ★ 解锁后立即重置 EKF 水平位置（以当前物理位置为原点）
    //   放在临界区外——不进 EKF 的并发路径，避免抢锁
    EstimatorKalman::resetHorizontal();
    return true;
}

bool stop()
{
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    system_armed = false;
    setThrust(0);
    resetAltitudeState(); // ✅ 重置高度状态
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    return true;
}

bool setThrust(float throttle)
{
    if (throttle < 0.0f || throttle > 1.0f)
        return false;
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.thrust = throttle;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    return true;
}

bool setROLL(float value)
{
    if (value < -1.0f || value > 1.0f)
        return false;
    float roll_rad = mapNormalizedToRange(value, MAX_ROLL_ANGLE_RAD);
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.attitudeRate.roll = roll_rad;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    return true;
}

bool setPITCH(float value)
{
    if (value < -1.0f || value > 1.0f)
        return false;
    float pitch_rad = mapNormalizedToRange(value, MAX_PITCH_ANGLE_RAD);
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.attitudeRate.pitch = pitch_rad;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    return true;
}

bool setYAW(float value)
{
    if (value < -1.0f || value > 1.0f)
        return false;
    // ⚠️ Yaw 通常是角速度设定值(rad/s)，不是角度！
    float yaw_rate_rads = mapNormalizedToRange(value, MAX_YAW_RATE_RADS);
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.attitudeRate.yaw = yaw_rate_rads;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    return true;
}

bool setHeigth(int16_t cm)
{
    if (cm < 0 || cm > 300)
        return false;
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.position.z = cm / 100.0f;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    return true;
}

bool setPosX(float pos_xcm)
{

    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.position.x = pos_xcm / 100.0f;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    Serial.printf("设置位置 pos_x=%.2f pos_y=%.2f", xGlobalSetpoint.position.x, xGlobalSetpoint.position.y);
    return true;
}
bool setPosY(float pos_ycm)
{

    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.position.y = pos_ycm / 100.0f;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    Serial.printf("设置位置 pos_x=%.2f pos_y=%.2f", xGlobalSetpoint.position.x, xGlobalSetpoint.position.y);
    return true;
}

bool setControlMode(int16_t mode)
{
    if (mode < 0 || mode > 2)
        return false;
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    xGlobalSetpoint.flight_mode = (flight_mode_t)mode;
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);

    switch (xGlobalSetpoint.flight_mode)
    {
    case 0:
        Serial.println("控制模式：手动");
        break;
    case 1:
        Serial.println("控制模式：自动");
        break;
    case 2:
        Serial.println("控制模式：自稳");
        break;

    default:
        Serial.println("控制模式：设置失败");
        break;
    }
    return true;
}

void setSerialSend(float f)
{
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    serial_send_flag = (f == 1.0);
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    // 打印必须在临界区外
    if (serial_send_flag)
    {
        Serial.println("开启串口数据传输");
    }
    else
    {
        Serial.println("关闭串口数据传输");
    }
}

void setSerialPrintSend(float f)
{
    portENTER_CRITICAL_SAFE(&g_receiver_lock);
    serial_print_flag = (f == 1.0);
    portEXIT_CRITICAL_SAFE(&g_receiver_lock);
    // 打印必须在临界区外
    if (serial_print_flag)
    {
        Serial.println("开启串口数据打印");
    }
    else
    {
        Serial.println("关闭串口数据打印");
    }
}

// void setCalibModeSend(float f)
// {
//     portENTER_CRITICAL_SAFE(&g_receiver_lock);
//     calib_mode = (f == 1.0);
//     portEXIT_CRITICAL_SAFE(&g_receiver_lock);
//     // 打印必须在临界区外
//     if (calib_mode)
//     {
//         Serial.println("开启标定");
//     }
//     else
//     {
//         Serial.println("关闭标定");
//     }
// }

// 在 receiver.cpp 的 stop() 或 arm() 中添加
void resetAltitudeState()
{
    _altitude_initialized = false;
    _altitude_setpoint = 0.0f;
}

static void (*g_pIMUCalibrationCallback)(bool success, void *userData) = nullptr;
static void *g_pIMUCalibrationUserData = nullptr;

void setIMUCalibrationCallback(void (*callback)(bool success, void *userData), void *userData)
{
    g_pIMUCalibrationCallback = callback;
    g_pIMUCalibrationUserData = userData;
}

// bool checkIMUCalibrationTrigger()
// {
//     if (system_armed)
//     {
//         Serial.println("⚠️ IMU校准必须在锁定状态下进行！");
//         return false;
//     }
//     // 一切就绪，发送校准请求
//     Serial.println("🔧 触发IMU校准...");
//     bool reqOk = requestIMUCalibration(
//         [](bool success, void *userData)
//         {
//             Serial.println(success ? "✅ 校准成功" : "❌ 校准失败");
//             // 🆕 调用回调函数
//             if (g_pIMUCalibrationCallback)
//             {
//                 g_pIMUCalibrationCallback(success, g_pIMUCalibrationUserData);
//             }
//         },
//         NULL);

//     if (!reqOk)
//     {
//         Serial.println("⚠️ 校准请求发送失败（可能已在校准中）");
//         return false;
//     }
//     return true;
// }