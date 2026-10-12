/**
 * @title: spi总线
 * @author: vaie
 * @date: 2026-10-11
 * @description:
 */
// hal/spi_bus.h
#pragma once

#include <stdint.h>
#include <stddef.h>

// 抽象总线编号 —— 与具体芯片变体无关，cpp 里映射到 Arduino 常量。
//   ESP32   : BUS1=HSPI  BUS2=VSPI
//   ESP32-S : BUS1=FSPI  BUS2=HSPI
enum class SpiHost : uint8_t
{
    BUS1 = 0,
    BUS2 = 1,
    BUS3 = 2, // S3 独有
};

struct SpiConfig
{
    SpiHost host = SpiHost::BUS2;
    int8_t sck = -1;
    int8_t miso = -1;
    int8_t mosi = -1;
    uint32_t frequency = 10'000'000;
    uint8_t mode = 0; // SPI_MODE0
    bool msb_first = true;
};

class SpiBus
{
public:
    SpiBus() = default;
    ~SpiBus();

    // 独占资源，禁拷贝
    SpiBus(const SpiBus &) = delete;
    SpiBus &operator=(const SpiBus &) = delete;

    bool begin(const SpiConfig &cfg);
    void end();

    bool ready() const;
    uint32_t frequency() const;

    // ---- 事务原语：调用方负责 CS（见 CsGuard） ----
    uint8_t transfer(uint8_t out);
    void transferBuffer(const uint8_t *tx, uint8_t *rx, size_t n);

    // ---- CS 管理（静态：CS 是纯 GPIO，不是总线状态） ----
    static void csInit(int8_t pin);
    static void csAssert(int8_t pin);
    static void csRelease(int8_t pin);

    // RAII：早返回 / 异常也保证 CS 回高
    struct CsGuard
    {
        int8_t pin;
        explicit CsGuard(int8_t p) : pin(p) { csAssert(p); }
        ~CsGuard() { csRelease(pin); }
        CsGuard(const CsGuard &) = delete;
        CsGuard &operator=(const CsGuard &) = delete;
    };

private:
    struct Impl; // PIMPL：头文件不暴露 SPIClass
    Impl *impl_ = nullptr;
};