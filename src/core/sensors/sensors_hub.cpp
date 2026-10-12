/**
 * @title  SensorsHub 实现
 * @author: vaie
 * @date: 2026-10-12
 *
 * 三个静态 Slot 的定义 + 9 个接口转发。
 * 无平台依赖（<atomic> 是标准库），SIL 与实机共用。
 */

#include "core/sensors/sensors_hub.h"

// ============================================================================
//  静态 Slot 定义
//   Slot 是 private 嵌套模板，此处定义于 SensorsHub 作用域内 —— C++ 允许
//   在类的 .cpp 里访问自己 private 成员。
// ============================================================================

SensorsHub::Slot<ImuData> SensorsHub::s_imu;
SensorsHub::Slot<mag_s> SensorsHub::s_mag;
SensorsHub::Slot<baro_s> SensorsHub::s_baro;

// ============================================================================
//  publish
// ============================================================================

void SensorsHub::publishImu(const ImuData &d)
{
     s_imu.write(d);
     telemetry_update_sensor_data(d);
}
void SensorsHub::publishMag(const mag_s &d)
{
     s_mag.write(d);
     telemetry_update_mag_sensor_data(d);
}
void SensorsHub::publishBaro(const baro_s &d)
{
     s_baro.write(d);
     telemetry_update_baro_sensor_data(d);
}

// ============================================================================
//  read
//   ★ out == nullptr 时直接返回 false —— 避免调用方崩溃
// ============================================================================

bool SensorsHub::readImu(ImuData *out) { return out && s_imu.read(out); }
bool SensorsHub::readMag(mag_s *out) { return out && s_mag.read(out); }
bool SensorsHub::readBaro(baro_s *out) { return out && s_baro.read(out); }

// ============================================================================
//  health
// ============================================================================

SensorsHub::Health SensorsHub::imuHealth() { return s_imu.health(); }
SensorsHub::Health SensorsHub::magHealth() { return s_mag.health(); }
SensorsHub::Health SensorsHub::baroHealth() { return s_baro.health(); }