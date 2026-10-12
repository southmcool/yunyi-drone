/**
 * @title: mpu驱动层
 * @author: vaie
 * @date: 2026-10-11
 * @description:
 */

// src/drivers/imu/mpu6500_device.h
#pragma once

#include <cstdint>
#include <cstddef>
#include "hal/spi_bus.h"
#include "type/imu_types.h"

class Mpu6500Device
{
public:
    Mpu6500Device(SpiBus &bus, int8_t cs_pin);

    Mpu6500Device(const Mpu6500Device &) = delete;
    Mpu6500Device &operator=(const Mpu6500Device &) = delete;

    // ---- 生命周期 ----
    //   begin() 内部不做任何 Serial 打印、不建任务、不加载校准。
    //   频率由 SpiBus::begin() 决定；此处的 spi_hz 仅作为文档/断言用途。
    bool begin();
    bool isConnected();

    // ---- 采样 ----
    bool readRaw(ImuRawSample *out);

    // ---- 裸寄存器访问（诊断/自检）----
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

    // ---- 配置 ----
    void setSampleRateDivider(uint8_t div);
    void setDLPFConfig(uint8_t cfg);
    void setGyroRange(uint8_t fs_sel);
    void setAccelRange(uint8_t afs_sel);

    // ---- DRDY 中断 ----
    //   enable=false 时 INT_ENABLE 清 0（默认）。
    //   上层在 attachInterrupt() 生效后再调 enable=true。
    void enableDataReadyInt(bool enable);
    void configureIntPin(uint8_t cfg); // 见 mpu6500_regs.h INT_*
    bool dataReadyFlag();              // 读 INT_STATUS（读即清）
    void clearDataReadyFlag();         // 显式再读一次清

    // ---- 量程查询 ----
    ImuScale scale() const { return scale_; }

    // ---- 诊断 ----
    void printConfig();

private:
    void reset();
    bool initRegisters();

    // 私有寄存器原语（全部走 SpiBus::CsGuard，RAII 保 CS）
    uint8_t readReg(uint8_t reg);
    void readRegs(uint8_t reg, uint8_t *buf, size_t n);
    void writeReg(uint8_t reg, uint8_t val);

    SpiBus &bus_;
    int8_t cs_;
    ImuScale scale_;

    uint8_t gyro_fs_ = 0x18;   // GYRO_FS_2000
    uint8_t accel_afs_ = 0x10; // ACCEL_FS_8G
};