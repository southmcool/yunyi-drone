#pragma once
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h" // portMUX_TYPE
#include "type/stabilizer_types.h"

// 全局遥测变量（在 .cpp 中定义）
extern stabilizer_log_entry_t stabilizer_data;
extern ImuData pa_sensorData;
extern portMUX_TYPE g_telemetry_lock;

// 各数据生产者调用，更新自己的字段（已内置临界区保护）
void telemetry_update_setpoint(const setpoint_t &setpoint);
// void telemetry_update_controller(const controller_log_data_t &controller);
// void telemetry_update_output(const control_output_t &output);
void telemetry_update_sensor_data(const ImuData &sensorData);
void telemetry_update_mag_sensor_data(const mag_s &mag);
void telemetry_update_baro_sensor_data(const baro_s &baro);
void telemetry_update_attitude(const state_t &state);
void telemetry_update_system_armed(bool armed);

// 采集器调用，一次性安全拷贝全量数据
void telemetry_snapshot(stabilizer_log_entry_t &out);