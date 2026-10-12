/**
 * @title: spi总线
 * @author: vaie
 * @date: 2026-10-11
 * @description:
 */
// hal/spi_bus.cpp
#include "hal/spi_bus.h"

#include <cstdio>

#if defined(SIL_HOST)
// SIL：无硬件 SPI。以下所有方法走桩，谁调到就是 bug（驱动走 injectRawSample）。
#else
#include <Arduino.h>
#include <SPI.h>
#endif

// ============================ 内部实现 ============================

struct SpiBus::Impl
{
#if !defined(SIL_HOST)
    SPIClass spi;
    explicit Impl(uint8_t host_num) : spi(host_num) {}
#else
    explicit Impl(uint8_t) {}
#endif
    uint32_t freq = 0;
    bool initialized = false;
};

#if !defined(SIL_HOST)
// SpiHost 抽象编号 → Arduino 常量
static uint8_t toArduinoHost(SpiHost h)
{
    switch (h)
    {
#if defined(CONFIG_IDF_TARGET_ESP32)
    case SpiHost::BUS1:
        return HSPI;
    case SpiHost::BUS2:
        return VSPI;
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    case SpiHost::BUS1:
        return FSPI;
    case SpiHost::BUS2:
        return HSPI;
#endif
    default:
        return HSPI;
    }
}
#endif

// ============================ CS 管理 ============================

void SpiBus::csInit(int8_t pin)
{
#if defined(SIL_HOST)
    (void)pin;
#else
    if (pin < 0)
        return;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
#endif
}

void SpiBus::csAssert(int8_t pin)
{
#if defined(SIL_HOST)
    (void)pin;
#else
    if (pin < 0)
        return;
    digitalWrite(pin, LOW);
#endif
}

void SpiBus::csRelease(int8_t pin)
{
#if defined(SIL_HOST)
    (void)pin;
#else
    if (pin < 0)
        return;
    digitalWrite(pin, HIGH);
#endif
}

// ============================ 生命周期 ============================

SpiBus::~SpiBus()
{
    end();
    delete impl_;
    impl_ = nullptr;
}

bool SpiBus::begin(const SpiConfig &cfg)
{
    if (impl_)
    {
        end();
        delete impl_;
        impl_ = nullptr;
    }

#if defined(SIL_HOST)
    impl_ = new Impl(0);
#else
    impl_ = new Impl(toArduinoHost(cfg.host));
#endif
    if (!impl_)
        return false;

#if !defined(SIL_HOST)
    csInit(cfg.sck);
    csInit(cfg.miso);
    csInit(cfg.mosi);

    // 传 -1：不把 CS 交给 SPIClass 托管，CS 由设备自己拉
    impl_->spi.begin(cfg.sck, cfg.miso, cfg.mosi, -1);
    impl_->spi.setFrequency(cfg.frequency);
    impl_->spi.setBitOrder(cfg.msb_first ? MSBFIRST : LSBFIRST);
    impl_->spi.setDataMode(cfg.mode);
#endif

    impl_->freq = cfg.frequency;
    impl_->initialized = true;
    return true;
}

void SpiBus::end()
{
    if (!impl_ || !impl_->initialized)
        return;
#if !defined(SIL_HOST)
    impl_->spi.end();
#endif
    impl_->initialized = false;
    impl_->freq = 0;
}

bool SpiBus::ready() const
{
    return impl_ && impl_->initialized;
}

uint32_t SpiBus::frequency() const
{
    return impl_ ? impl_->freq : 0;
}

// ============================ 传输 ============================

uint8_t SpiBus::transfer(uint8_t out)
{
#if defined(SIL_HOST)
    (void)out;
    std::fprintf(stderr, "[SpiBus] SIL 下不应调用 transfer()\n");
    return 0;
#else
    return impl_ ? impl_->spi.transfer(out) : 0;
#endif
}

void SpiBus::transferBuffer(const uint8_t *tx, uint8_t *rx, size_t n)
{
#if defined(SIL_HOST)
    (void)tx;
    (void)rx;
    (void)n;
    std::fprintf(stderr, "[SpiBus] SIL 下不应调用 transferBuffer()\n");
#else
    if (!impl_)
        return;
    for (size_t i = 0; i < n; ++i)
    {
        const uint8_t o = tx ? tx[i] : 0x00;
        const uint8_t v = impl_->spi.transfer(o);
        if (rx)
            rx[i] = v;
    }
#endif
}