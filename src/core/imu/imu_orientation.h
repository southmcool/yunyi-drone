/**
 * @title  IMU 朝向与坐标变换
 * @author: vaie
 * @date: 2026-10-12
 *
 * imu_orientation.h — 芯片物理朝向 → 机体系（NED）的坐标变换。
 *
 * 纯函数：无状态、无副作用、不依赖任何平台头（Arduino/ESP-IDF/FreeRTOS）。
 * 可被 ImuSignalChain 调用，也可单独单测。
 *
 * 依赖：
 *   仅 type/imu_types.h（Axis3f）。
 */
#pragma once

#include "type/imu_types.h" // Axis3f

// ---------------------------------------------------------------------------
// 芯片物理朝向
//
//   枚举的语义 = "芯片坐标系相对机体系（NED）的姿态"。
//   DEFAULT 表示芯片系与机体系一致，无需变换。
//
//   ★ 真机实际值 = ORIENTATION_FUSIONCONVENTIONNED_YAW_90
//     芯片 = X东, Y北, Z上；变换后 = 机体系 NED (X北, Y东, Z下)。
//     判据见 imu_orientation.cpp 里该分支的注释。
// ---------------------------------------------------------------------------
enum imu_orientation_t
{
    ORIENTATION_DEFAULT = 0,                // 芯片系 = 机体系，无需变换
    ORIENTATION_FUSIONCONVENTIONNWU,        // 芯片 = NWU（X北, Y西, Z上）
    ORIENTATION_FUSIONCONVENTIONENU,        // 芯片 = ENU（X东, Y北, Z上）
    ORIENTATION_FUSIONCONVENTIONNED,        // 芯片 = NED（X北, Y东, Z下）
    ORIENTATION_FUSIONCONVENTIONNED_YAW_90, // ★ 真机：芯片绕 Z 转 +90°
};

// ---------------------------------------------------------------------------
// 芯片坐标系 → 机体系 NED
//
//   纯函数：给定朝向 orient 和芯片系下的向量 v，返回机体系下的向量。
//   只做轴映射与符号翻转，不改变单位 —— 加速度（g）、角速度（°/s）、
//   磁场（Gauss）都可用同一函数。
//
//   加速度计与陀螺仪的变换矩阵相同，所以调用方分别调用两次：
//       data.acc  = toBodyFrame(orient, data.acc);
//       data.gyro = toBodyFrame(orient, data.gyro);
//
//   orient 非法（枚举越界）时原样返回 v。
// ---------------------------------------------------------------------------
Axis3f toBodyFrame(imu_orientation_t orient, const Axis3f &v);