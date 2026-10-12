/**
 * @title: mpu6500 驱动实现
 * @author: vaie
 * @date: 2026-10-11
 * @description: 寄存器级 IO + 原始采样。不含校准、换算、坐标变换、滤波、任务。
 */

#include "drivers/imu/mpu6500_device.h"
#include "drivers/imu/mpu6500_regs.h"

#if !defined(SIL_HOST)
#include <Arduino.h>
#include <cstdio>
#include <esp_timer.h> // ← 加这行
#endif

namespace
{

    inline uint64_t nowMicros()
    {
#if defined(SIL_HOST)
        return 0;
#else
        return static_cast<uint64_t>(esp_timer_get_time()); // int64 µs，永不回绕
#endif
    }

    inline void delayMs(uint32_t ms)
    {
#if defined(SIL_HOST)
        (void)ms;
#else
        delay(ms);
#endif
    }

} // namespace

// ============================ 构造 ============================

Mpu6500Device::Mpu6500Device(SpiBus &bus, int8_t cs_pin)
    : bus_(bus), cs_(cs_pin)
{
    // 默认与旧 Mpu6500 一致：±8g / ±2000dps
    scale_.accel_lsb_per_g = mpu6500::ACCEL_LSB_PER_G_8G;
    scale_.gyro_lsb_per_dps = mpu6500::GYRO_LSB_PER_DPS_2000;
    scale_.temp_lsb_per_deg = mpu6500::TEMP_LSB_PER_DEG;
    scale_.temp_offset_deg = mpu6500::TEMP_ROOM_OFFSET;
}

// ============================ 生命周期 ============================

bool Mpu6500Device::begin()
{
    SpiBus::csInit(cs_); // 拉高，之后由 CsGuard 每次事务接管

    reset();
    if (!initRegisters())
        return false;
    if (!isConnected())
        return false;

    return true;
}

void Mpu6500Device::reset()
{
    writeReg(mpu6500::PWR_MGMT_1, mpu6500::PWR_RESET);
    delayMs(150);
    writeReg(mpu6500::PWR_MGMT_1, mpu6500::PWR_WAKE);
    delayMs(200);
}

bool Mpu6500Device::initRegisters()
{
    // ★ 顺序与旧 Mpu6500::initializeSensor 完全一致，逐条对照：
    setSampleRateDivider(0);               // SMPLRT_DIV = 0 → 1kHz
    setDLPFConfig(mpu6500::DLPF_BW_184HZ); // CONFIG = 0x01（相位裕度最优）

    setGyroRange(mpu6500::GYRO_FS_2000); // GYRO_CONFIG  = 0x18
    setAccelRange(mpu6500::ACCEL_FS_8G); // ACCEL_CONFIG = 0x10

    // INT：默认关，脉冲模式 active-high push-pull；上层 attachInterrupt 生效后
    //   再调 enableDataReadyInt(true)。
    enableDataReadyInt(false);                // INT_ENABLE = 0x00
    configureIntPin(mpu6500::INT_PULSE_HIGH); // INT_PIN_CFG = 0x00

    delayMs(50);

    // ★ 不写 ACCEL_CONFIG2 —— 与旧代码一致，依赖复位值 0x00（A_DLPF_CFG=0 ⇒ 460Hz）。
    //   这是 VRE 频率归一化 K(f) = k/|H_dlpf(f)|² 的物理前提，SIL 与实机共享假设。
    //   printConfig() 会回读该寄存器，实机可据此确认假设未漂。
    return true;
}

bool Mpu6500Device::isConnected()
{
    return readReg(mpu6500::WHO_AM_I_ADDR) == mpu6500::WHO_AM_I_VALUE;
}

// ============================ 采样 ============================

bool Mpu6500Device::readRaw(ImuRawSample *out)
{
    if (!out)
        return false;

    // 时间戳取"读取起始时刻" —— 与旧 Mpu6500::readSensor 语义一致
    const uint64_t  t0 = nowMicros();

    // 14 字节连读：ACCEL_XOUT_H..GYRO_ZOUT_L
    //   [0..5] accel, [6..7] temp, [8..13] gyro
    uint8_t buf[14];
    readRegs(mpu6500::ACCEL_XOUT_H, buf, 14);

    out->acc.x = static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
    out->acc.y = static_cast<int16_t>((static_cast<uint16_t>(buf[2]) << 8) | buf[3]);
    out->acc.z = static_cast<int16_t>((static_cast<uint16_t>(buf[4]) << 8) | buf[5]);

    out->temperature = static_cast<int16_t>((static_cast<uint16_t>(buf[6]) << 8) | buf[7]);

    out->gyro.x = static_cast<int16_t>((static_cast<uint16_t>(buf[8]) << 8) | buf[9]);
    out->gyro.y = static_cast<int16_t>((static_cast<uint16_t>(buf[10]) << 8) | buf[11]);
    out->gyro.z = static_cast<int16_t>((static_cast<uint16_t>(buf[12]) << 8) | buf[13]);

    out->timestamp_us = t0;
    return true;
}

// ============================ 寄存器原语 ============================
//   全部走 CsGuard：早返回 / 异常路径也保证 CS 回高。

uint8_t Mpu6500Device::readReg(uint8_t reg)
{
    SpiBus::CsGuard g(cs_);
    bus_.transfer(reg | 0x80); // 读：最高位置 1
    return bus_.transfer(0x00);
}

void Mpu6500Device::readRegs(uint8_t reg, uint8_t *buf, size_t n)
{
    SpiBus::CsGuard g(cs_);
    bus_.transfer(reg | 0x80);
    bus_.transferBuffer(nullptr, buf, n);
}

void Mpu6500Device::writeReg(uint8_t reg, uint8_t val)
{
    SpiBus::CsGuard g(cs_);
    bus_.transfer(reg & 0x7F); // 写：最高位清 0
    bus_.transfer(val);
}

// 兼容旧命名（如果外部代码还在用 readRegister/writeRegister）
uint8_t Mpu6500Device::readRegister(uint8_t reg) { return readReg(reg); }
void Mpu6500Device::writeRegister(uint8_t reg, uint8_t value) { writeReg(reg, value); }

// ============================ 配置 ============================

void Mpu6500Device::setSampleRateDivider(uint8_t div)
{
    writeReg(mpu6500::SMPLRT_DIV, div);
}

void Mpu6500Device::setDLPFConfig(uint8_t cfg)
{
    writeReg(mpu6500::CONFIG, cfg & 0x07);
}

void Mpu6500Device::setGyroRange(uint8_t fs_sel)
{
    fs_sel &= 0x18;
    writeReg(mpu6500::GYRO_CONFIG, fs_sel);
    gyro_fs_ = fs_sel;

    switch (fs_sel)
    {
    case mpu6500::GYRO_FS_250:
        scale_.gyro_lsb_per_dps = mpu6500::GYRO_LSB_PER_DPS_250;
        break;
    case mpu6500::GYRO_FS_500:
        scale_.gyro_lsb_per_dps = mpu6500::GYRO_LSB_PER_DPS_500;
        break;
    case mpu6500::GYRO_FS_1000:
        scale_.gyro_lsb_per_dps = mpu6500::GYRO_LSB_PER_DPS_1000;
        break;
    case mpu6500::GYRO_FS_2000:
        scale_.gyro_lsb_per_dps = mpu6500::GYRO_LSB_PER_DPS_2000;
        break;
    default:
        break;
    }
}

void Mpu6500Device::setAccelRange(uint8_t afs_sel)
{
    afs_sel &= 0x18;
    writeReg(mpu6500::ACCEL_CONFIG, afs_sel);
    accel_afs_ = afs_sel;

    switch (afs_sel)
    {
    case mpu6500::ACCEL_FS_2G:
        scale_.accel_lsb_per_g = mpu6500::ACCEL_LSB_PER_G_2G;
        break;
    case mpu6500::ACCEL_FS_4G:
        scale_.accel_lsb_per_g = mpu6500::ACCEL_LSB_PER_G_4G;
        break;
    case mpu6500::ACCEL_FS_8G:
        scale_.accel_lsb_per_g = mpu6500::ACCEL_LSB_PER_G_8G;
        break;
    case mpu6500::ACCEL_FS_16G:
        scale_.accel_lsb_per_g = mpu6500::ACCEL_LSB_PER_G_16G;
        break;
    default:
        break;
    }
}

// ============================ DRDY 中断 ============================

void Mpu6500Device::enableDataReadyInt(bool enable)
{
    writeReg(mpu6500::INT_ENABLE,
             enable ? mpu6500::INT_RAW_RDY_EN : uint8_t{0x00});
}

void Mpu6500Device::configureIntPin(uint8_t cfg)
{
    writeReg(mpu6500::INT_PIN_CFG, cfg);
}

bool Mpu6500Device::dataReadyFlag()
{
    // INT_STATUS 读即清 —— 这一读顺带清标志
    return (readReg(mpu6500::INT_STATUS) & mpu6500::INT_RAW_RDY_ST) != 0;
}

void Mpu6500Device::clearDataReadyFlag()
{
    (void)readReg(mpu6500::INT_STATUS);
}

// ============================ 诊断 ============================

void Mpu6500Device::printConfig()
{
#if !defined(SIL_HOST)
    // 与旧 Mpu6500::begin() 里那段"芯片事实"打印等价（R58 证据链）：
    //   ACCEL_CONFIG2 是 VRE 频率归一化的物理前提，本仓库从不写它，
    //   此处回读以确认复位值仍为 0x00（A_DLPF_CFG=0 ⇒ 460Hz）。
    const uint8_t who = readReg(mpu6500::WHO_AM_I_ADDR);
    const uint8_t div = readReg(mpu6500::SMPLRT_DIV);
    const uint8_t gcfg = readReg(mpu6500::GYRO_CONFIG);
    const uint8_t acfg = readReg(mpu6500::ACCEL_CONFIG);
    const uint8_t cfg1a = readReg(mpu6500::CONFIG);
    const uint8_t cfg1d = readReg(mpu6500::ACCEL_CONFIG2);

    Serial.printf(
        "[IMU] WHO=0x%02X SMPLRT_DIV=%u GYRO_CFG=0x%02X ACCEL_CFG=0x%02X "
        "CONFIG(0x1A)=0x%02X ACCEL_CONFIG2(0x1D)=0x%02X "
        "=> 陀螺DLPF=%u 加计A_DLPF=%u %s\n",
        who, div, gcfg, acfg, cfg1a, cfg1d,
        (unsigned)(cfg1a & 0x07), (unsigned)(cfg1d & 0x07),
        ((cfg1d & 0x07) == 0) ? "(=460Hz，与 ImuVibConfig::vdc.dlpf_fc_hz 一致)"
                              : "(★ 与代码假设不符 ⇒ 必须改 dlpf_fc_hz)");
#endif
}