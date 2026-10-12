/**
 * title  MPU6500 寄存器常量
 * @author: vaie
 * @date: 2026-10-11
 * @description: 纯常量，无依赖，可被任意编译单元 include（含 SIL）。
 */
#pragma once
#include <cstdint>

namespace mpu6500
{

    // ---------- 身份 ----------
    constexpr uint8_t WHO_AM_I_ADDR = 0x75;
    constexpr uint8_t WHO_AM_I_VALUE = 0x70;

    // ---------- 寄存器地址 ----------
    constexpr uint8_t SMPLRT_DIV = 0x19;
    constexpr uint8_t CONFIG = 0x1A;
    constexpr uint8_t GYRO_CONFIG = 0x1B;
    constexpr uint8_t ACCEL_CONFIG = 0x1C;
    constexpr uint8_t ACCEL_CONFIG2 = 0x1D;
    constexpr uint8_t INT_PIN_CFG = 0x37;
    constexpr uint8_t INT_ENABLE = 0x38;
    constexpr uint8_t INT_STATUS = 0x3A;
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t GYRO_XOUT_H = 0x43;
    constexpr uint8_t PWR_MGMT_1 = 0x6B;
    constexpr uint8_t PWR_MGMT_2 = 0x6C;

    // ---------- PWR_MGMT_1 ----------
    constexpr uint8_t PWR_RESET = 0x80;
    constexpr uint8_t PWR_WAKE = 0x00;

    // ---------- INT_PIN_CFG (0x37) ----------
    //   bit7 ACTL         0=active high
    //   bit6 OPEN         0=push-pull
    //   bit5 LATCH_INT_EN 0=50µs 脉冲
    //   bit4 INT_RD_CLEAR 0=不读清（走 ISR 显式清）
    constexpr uint8_t INT_PULSE_HIGH = 0x00;

    // ---------- INT_ENABLE (0x38) ----------
    constexpr uint8_t INT_RAW_RDY_EN = 0x01;

    // ---------- INT_STATUS (0x3A) ----------
    constexpr uint8_t INT_RAW_RDY_ST = 0x01;

    // ---------- GYRO_CONFIG FS_SEL[4:3] ----------
    constexpr uint8_t GYRO_FS_250 = 0x00;
    constexpr uint8_t GYRO_FS_500 = 0x08;
    constexpr uint8_t GYRO_FS_1000 = 0x10;
    constexpr uint8_t GYRO_FS_2000 = 0x18;

    // ---------- ACCEL_CONFIG AFS_SEL[4:3] ----------
    constexpr uint8_t ACCEL_FS_2G = 0x00;
    constexpr uint8_t ACCEL_FS_4G = 0x08;
    constexpr uint8_t ACCEL_FS_8G = 0x10;
    constexpr uint8_t ACCEL_FS_16G = 0x18;

    // ---------- CONFIG DLPF_CFG[2:0]（陀螺）----------
    constexpr uint8_t DLPF_BW_250HZ = 0x00;
    constexpr uint8_t DLPF_BW_184HZ = 0x01; // ← 本仓库当前使用
    constexpr uint8_t DLPF_BW_92HZ = 0x02;
    constexpr uint8_t DLPF_BW_41HZ = 0x03;
    constexpr uint8_t DLPF_BW_20HZ = 0x04;
    constexpr uint8_t DLPF_BW_10HZ = 0x05;
    constexpr uint8_t DLPF_BW_5HZ = 0x06;

    // ---------- 换算因子（LSB / 物理单位）----------
    constexpr float ACCEL_LSB_PER_G_2G = 16384.0f;
    constexpr float ACCEL_LSB_PER_G_4G = 8192.0f;
    constexpr float ACCEL_LSB_PER_G_8G = 4096.0f; // ← 本仓库当前使用
    constexpr float ACCEL_LSB_PER_G_16G = 2048.0f;

    constexpr float GYRO_LSB_PER_DPS_250 = 131.0f;
    constexpr float GYRO_LSB_PER_DPS_500 = 65.5f;
    constexpr float GYRO_LSB_PER_DPS_1000 = 32.8f;
    constexpr float GYRO_LSB_PER_DPS_2000 = 16.384f; // ← 本仓库当前使用

    constexpr float TEMP_LSB_PER_DEG = 340.0f;
    constexpr float TEMP_ROOM_OFFSET = 25.0f;

} // namespace mpu6500