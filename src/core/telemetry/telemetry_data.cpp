#include "telemetry_data.h"
#include <cstring>

stabilizer_log_entry_t stabilizer_data;
ImuData pa_sensorData;
portMUX_TYPE g_telemetry_lock = portMUX_INITIALIZER_UNLOCKED;

void telemetry_update_setpoint(const setpoint_t &setpoint)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.setpoint = setpoint;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}

void telemetry_update_sensor_data(const ImuData &sensorData)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.sensor_raw = sensorData;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}

void telemetry_update_mag_sensor_data(const mag_s &mag)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.mag = mag;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}

void telemetry_update_baro_sensor_data(const baro_s &baro)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.baro = baro;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}
void telemetry_update_attitude(const state_t &state_t)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.state_estimated = state_t;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}
void telemetry_update_system_armed(bool armed)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.is_armed = armed;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}

void telemetry_update_motor_pwd(bool armed)
{
    portENTER_CRITICAL_SAFE(&g_telemetry_lock);
    stabilizer_data.is_armed = armed;
    portEXIT_CRITICAL_SAFE(&g_telemetry_lock);
}