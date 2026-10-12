
#ifndef IMU_TYPES_H_
#define IMU_TYPES_H_
/**
 * title  IMU 数据结构
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * imu_types.h - Types used by IMU and releted functions.
 */
#include <cstdint>

typedef union
{
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };
  int16_t axis[3];
} Axis3i16;

typedef union
{
  struct
  {
    int32_t x;
    int32_t y;
    int32_t z;
  };
  int32_t axis[3];
} Axis3i32;

typedef union
{
  struct
  {
    int64_t x;
    int64_t y;
    int64_t z;
  };
  int64_t axis[3];
} Axis3i64;

typedef union
{
  struct
  {
    float x;
    float y;
    float z;
  };
  float axis[3];
} Axis3f;

// ---- IMU 原始采样（int16 LSB 域）----
struct ImuRawSample
{
  Axis3i16 gyro;
  Axis3i16 acc;
  int16_t temperature;   // LSB
  uint64_t timestamp_us; // µs
};

// ---- 信号链输出（物理单位，机体系）----
struct ImuData
{
  // ---- 主数据（校准 + 坐标变换 + 滤波后）----
  Axis3f acc;        // g（机体系 NED）
  Axis3f gyro;       // °/s（机体系 NED）
  float temperature; // °C

  // ---- 派生量 ----
  float accel_magnitude; // g

  // ---- 诊断（坐标变换前的校准值）----
  Axis3f accRaw;
  Axis3f gyroRaw;

  uint64_t timestamp_us;
};

// ---- 校准数据（int16 域，与旧 Mpu6500 的 accelOffsetX 等语义一致）----
struct ImuCalibration
{
  Axis3i16 accel_offset;
  Axis3i16 gyro_offset;
  // 可选：灵敏度归一化因子（旧代码 accScale 默认 1.0）
  Axis3f accel_scale = {1.0f, 1.0f, 1.0f}; // 每轴独立
};

// ---- 量程换算因子（任何 IMU 都用这个结构，具体数值由驱动填）----
struct ImuScale
{
  float accel_lsb_per_g;  // LSB/g
  float gyro_lsb_per_dps; // LSB/(°/s)
  float temp_lsb_per_deg; // LSB/°C
  float temp_offset_deg;  // °C
};

#endif /* IMU_TYPES_H_ */
