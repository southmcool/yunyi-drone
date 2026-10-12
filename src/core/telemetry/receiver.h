#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include "type/stabilizer_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h" // portMUX_TYPE
#include "core/estimator/estimator_kalman.h"

// 全局设定值（由接收机任务写入，姿态控制任务只读）
extern setpoint_t xGlobalSetpoint;
extern bool system_armed;
extern bool serial_send_flag;

// ===== 高度控制状态 =====
extern float _altitude_setpoint;         // 目标高度 (米)
extern float _altitude_offset;           // 累积高度偏移 (米)
extern float _last_altitude_update_time; // 上次更新时间 (ms)
extern bool _altitude_initialized;

// 高度控制参数
extern float MAX_ALTITUDE_CHANGE_RATE; // 最大升降速度 (m/s)
extern float ALT_DEADZONE;             // 摇杆死区

extern bool arm();
extern bool stop();
extern bool setThrust(float throttle);
extern void setSerialSend(float f);
extern void setSerialPrintSend(float f);
extern void setCalibModeSend(float f);
extern bool setROLL(float f);
extern bool setPITCH(float f);
extern bool setYAW(float f);
extern bool setHeigth(int16_t cm);
extern bool setPosX(float pos_cm);
extern bool setPosY(float pos_cm);
extern bool setControlMode(int16_t mode);
extern void resetAltitudeState();

// receiver.h 中新增函数声明
extern bool setAltitude(float value);
extern float getAltitudeOffset(); // 获取当前高度偏移量（用于调试）

bool checkIMUCalibrationTrigger();
void setIMUCalibrationCallback(void (*callback)(bool success, void *userData), void *userData = nullptr);

#endif