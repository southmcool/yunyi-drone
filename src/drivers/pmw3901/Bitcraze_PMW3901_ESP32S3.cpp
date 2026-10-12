#include "Bitcraze_PMW3901_ESP32S3.h"

Bitcraze_PMW3901::Bitcraze_PMW3901(
    uint8_t cspin,
    uint8_t sck,
    uint8_t miso,
    uint8_t mosi)
    : _cs(cspin),
      _sck(sck),
      _miso(miso),
      _mosi(mosi),
      _healthy(false),
      _consecutiveErrorCount(0)
{
}

bool Bitcraze_PMW3901::begin(void)
{
    // 初始化 ESP32-S3 SPI
    SPI.begin(_sck, _miso, _mosi, _cs);

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);

    delay(10);

    // PMW3901 上电复位
    if (!registerWrite(0x3A, 0x5A))
    {
        markError();
        return false;
    }

    delay(50);

    // 验证 Chip ID
    if (!verifyChipId())
    {
        Serial.println("[PMW3901] Chip ID verification failed");
        markError();
        return false;
    }

    Serial.println("[PMW3901] Chip ID verified");

    // 清除可能残留的运动数据
    uint8_t dummy = 0;

    registerRead(0x02, dummy);
    registerRead(0x03, dummy);
    registerRead(0x04, dummy);
    registerRead(0x05, dummy);
    registerRead(0x06, dummy);

    delay(1);

    // 官方/原始驱动初始化寄存器序列
    initRegisters();

    // 初始化完成后再次验证一次 ID
    if (!verifyChipId())
    {
        Serial.println("[PMW3901] Chip lost after initialization");
        markError();
        return false;
    }

    _healthy = true;
    _consecutiveErrorCount = 0;

    return true;
}

bool Bitcraze_PMW3901::verifyChipId()
{
    uint8_t chipId = 0;
    uint8_t invId = 0;

    if (!registerRead(0x00, chipId))
    {
        return false;
    }

    if (!registerRead(0x5F, invId))
    {
        return false;
    }

    Serial.printf(
        "[PMW3901] ChipID=0x%02X InvID=0x%02X\n",
        chipId,
        invId);

    // 保留你当前驱动已经验证成功的兼容 ID 判断
    if (chipId != 0x49)
    {
        return false;
    }

    if (invId != 0xB8 && invId != 0xB6)
    {
        return false;
    }

    return true;
}

bool Bitcraze_PMW3901::readMotionBurst(
    Pmw3901MotionBurst &burst)
{
    constexpr uint8_t MOTION_BURST_ADDRESS = 0x16;

    uint8_t rxBuffer[sizeof(Pmw3901MotionBurst)] = {0};

    SPI.beginTransaction(
        SPISettings(
            2000000,
            MSBFIRST,
            SPI_MODE3));

    digitalWrite(_cs, LOW);

    delayMicroseconds(50);

    // Motion Burst 地址必须是读操作
    SPI.transfer(MOTION_BURST_ADDRESS);

    delayMicroseconds(50);

    // 连续读取完整 Burst
    for (size_t i = 0;
         i < sizeof(Pmw3901MotionBurst);
         ++i)
    {
        rxBuffer[i] = SPI.transfer(0x00);
    }

    delayMicroseconds(50);

    digitalWrite(_cs, HIGH);

    SPI.endTransaction();

    // PMW3901 Motion Burst 数据采用 little-endian
    burst.motion = rxBuffer[0];
    burst.observation = rxBuffer[1];

    burst.deltaX =
        static_cast<int16_t>(
            static_cast<uint16_t>(rxBuffer[2]) |
            (static_cast<uint16_t>(rxBuffer[3]) << 8));

    burst.deltaY =
        static_cast<int16_t>(
            static_cast<uint16_t>(rxBuffer[4]) |
            (static_cast<uint16_t>(rxBuffer[5]) << 8));

    burst.squal = rxBuffer[6];

    burst.rawDataSum = rxBuffer[7];
    burst.maxRawData = rxBuffer[8];
    burst.minRawData = rxBuffer[9];

    burst.shutter =
        static_cast<uint16_t>(
            (static_cast<uint16_t>(rxBuffer[10]) << 8) |
            static_cast<uint16_t>(rxBuffer[11]));

    // Serial.printf(
    //     "Shutter bytes: %02X %02X -> shutter: %u\n",
    //     rxBuffer[10],
    //     rxBuffer[11],
    //     burst.shutter);

    return true;
}

bool Bitcraze_PMW3901::validateBurst(
    const Pmw3901MotionBurst &burst) const
{
    // 防止明显异常的 Delta 数据进入飞控
    if (abs(burst.deltaX) > MAX_ABSOLUTE_DELTA)
    {
        return false;
    }

    if (abs(burst.deltaY) > MAX_ABSOLUTE_DELTA)
    {
        return false;
    }

    /*
     * 注意：
     *
     * 这里不能因为 deltaX == 0 && deltaY == 0
     * 就判定数据无效。
     *
     * 静止状态的合法输出就是：
     *
     * deltaX = 0
     * deltaY = 0
     */

    return true;
}

bool Bitcraze_PMW3901::readMeasurement(
    Pmw3901Measurement &measurement)
{
    Pmw3901MotionBurst burst{};

    measurement.deltaX = 0;
    measurement.deltaY = 0;

    measurement.motionDetected = false;
    measurement.valid = false;
    measurement.healthy = _healthy;

    measurement.squal = 0;
    measurement.rawDataSum = 0;
    measurement.maxRawData = 0;
    measurement.minRawData = 0;
    measurement.shutter = 0;

    if (!readMotionBurst(burst))
    {
        markError();

        measurement.healthy = _healthy;

        return false;
    }

    if (!validateBurst(burst))
    {
        markError();

        measurement.healthy = _healthy;

        return false;
    }

    // Motion bit 7 表示是否检测到运动
    measurement.motionDetected =
        (burst.motion & 0x80) != 0;

    /*
     * ★ R66：不再用 motion bit 门控 deltaX/deltaY —— 对齐官方 Bitcraze 库
     *   （Bitcraze_PMW3901.cpp::readMotionCount 无条件使用 delta 寄存器，
     *     motion 寄存器只读一次做清零，从不参与数据有效性判定）。
     *
     * 原实现危害（本仓库 pmw3901_task.cpp:155-157 注释已自证）：
     *   项目日志出现过「有效帧累加出 -21 个 count，但 motionDetected 门限下的
     *   和为 0」⇒ 该位在真实运动下【长期不置位】。原代码在
     *   motionDetected=false 时把 deltaX/deltaY 强制归零 ⇒ 真实运动被静默
     *   丢弃 ⇒ 光流速度观测恒 0 ⇒ 位置恒 0（真机台架现象）。
     *
     * 有效性仍由两道门保证（未削弱）：
     *   1) validateBurst()：|delta| <= MAX_ABSOLUTE_DELTA (3000)
     *   2) pmw3901_task：squal > 30
     * motionDetected 保留为【纯诊断】字段，不再做数据门控。
     */
    measurement.deltaX = burst.deltaX;
    measurement.deltaY = burst.deltaY;

    measurement.squal = burst.squal;
    measurement.rawDataSum = burst.rawDataSum;
    measurement.maxRawData = burst.maxRawData;
    measurement.minRawData = burst.minRawData;
    measurement.shutter = burst.shutter;

    measurement.valid = true;

    markSuccess();

    measurement.healthy = _healthy;

    return true;
}

bool Bitcraze_PMW3901::readMotionCount(
    int16_t *deltaX,
    int16_t *deltaY)
{
    if (deltaX == nullptr ||
        deltaY == nullptr)
    {
        return false;
    }

    Pmw3901Measurement measurement{};

    if (!readMeasurement(measurement))
    {
        *deltaX = 0;
        *deltaY = 0;

        return false;
    }

    *deltaX = measurement.deltaX;
    *deltaY = measurement.deltaY;

    return true;
}

bool Bitcraze_PMW3901::isHealthy() const
{
    return _healthy;
}

void Bitcraze_PMW3901::markSuccess()
{
    _consecutiveErrorCount = 0;

    _healthy = true;
}

void Bitcraze_PMW3901::markError()
{
    if (_consecutiveErrorCount <
        MAX_CONSECUTIVE_ERRORS)
    {
        _consecutiveErrorCount++;
    }

    if (_consecutiveErrorCount >=
        MAX_CONSECUTIVE_ERRORS)
    {
        _healthy = false;
    }
}

bool Bitcraze_PMW3901::registerWrite(
    uint8_t reg,
    uint8_t value)
{
    reg |= 0x80u;

    SPI.beginTransaction(
        SPISettings(
            2000000,
            MSBFIRST,
            SPI_MODE3));

    digitalWrite(_cs, LOW);

    delayMicroseconds(50);

    SPI.transfer(reg);
    SPI.transfer(value);

    delayMicroseconds(50);

    digitalWrite(_cs, HIGH);

    SPI.endTransaction();

    delayMicroseconds(200);

    return true;
}

bool Bitcraze_PMW3901::registerRead(
    uint8_t reg,
    uint8_t &value)
{
    reg &= ~0x80u;

    SPI.beginTransaction(
        SPISettings(
            2000000,
            MSBFIRST,
            SPI_MODE3));

    digitalWrite(_cs, LOW);

    delayMicroseconds(50);

    SPI.transfer(reg);

    delayMicroseconds(100);

    value = SPI.transfer(0x00);

    delayMicroseconds(50);

    digitalWrite(_cs, HIGH);

    SPI.endTransaction();

    return true;
}

// Performance optimisation registers
void Bitcraze_PMW3901::initRegisters()
{
    registerWrite(0x7F, 0x00);
    registerWrite(0x61, 0xAD);

    registerWrite(0x7F, 0x03);
    registerWrite(0x40, 0x00);

    registerWrite(0x7F, 0x05);
    registerWrite(0x41, 0xB3);
    registerWrite(0x43, 0xF1);
    registerWrite(0x45, 0x14);
    registerWrite(0x5B, 0x32);
    registerWrite(0x5F, 0x34);
    registerWrite(0x7B, 0x08);

    registerWrite(0x7F, 0x06);
    registerWrite(0x44, 0x1B);
    registerWrite(0x40, 0xBF);
    registerWrite(0x4E, 0x3F);

    registerWrite(0x7F, 0x08);
    registerWrite(0x65, 0x20);
    registerWrite(0x6A, 0x18);

    registerWrite(0x7F, 0x09);
    registerWrite(0x4F, 0xAF);
    registerWrite(0x5F, 0x40);
    registerWrite(0x48, 0x80);
    registerWrite(0x49, 0x80);
    registerWrite(0x57, 0x77);
    registerWrite(0x60, 0x78);
    registerWrite(0x61, 0x78);
    registerWrite(0x62, 0x08);
    registerWrite(0x63, 0x50);

    registerWrite(0x7F, 0x0A);
    registerWrite(0x45, 0x60);

    registerWrite(0x7F, 0x00);
    registerWrite(0x4D, 0x11);
    registerWrite(0x55, 0x80);
    registerWrite(0x74, 0x1F);
    registerWrite(0x75, 0x1F);
    registerWrite(0x4A, 0x78);
    registerWrite(0x4B, 0x78);
    registerWrite(0x44, 0x08);
    registerWrite(0x45, 0x50);
    registerWrite(0x64, 0xFF);
    registerWrite(0x65, 0x1F);

    registerWrite(0x7F, 0x14);
    registerWrite(0x65, 0x60);
    registerWrite(0x66, 0x08);
    registerWrite(0x63, 0x78);

    registerWrite(0x7F, 0x15);
    registerWrite(0x48, 0x58);

    registerWrite(0x7F, 0x07);
    registerWrite(0x41, 0x0D);
    registerWrite(0x43, 0x14);
    registerWrite(0x4B, 0x0E);
    registerWrite(0x45, 0x0F);
    registerWrite(0x44, 0x42);
    registerWrite(0x4C, 0x80);

    registerWrite(0x7F, 0x10);
    registerWrite(0x5B, 0x02);

    registerWrite(0x7F, 0x07);
    registerWrite(0x40, 0x41);
    registerWrite(0x70, 0x00);

    // 等待内部校准
    delay(100);

    registerWrite(0x32, 0x44);

    registerWrite(0x7F, 0x07);
    registerWrite(0x40, 0x40);

    registerWrite(0x7F, 0x06);
    registerWrite(0x62, 0xF0);
    registerWrite(0x63, 0x00);

    registerWrite(0x7F, 0x0D);
    registerWrite(0x48, 0xC0);
    registerWrite(0x6F, 0xD5);

    registerWrite(0x7F, 0x00);
    registerWrite(0x5B, 0xA0);
    registerWrite(0x4E, 0xA8);
    registerWrite(0x5A, 0x50);
    registerWrite(0x40, 0x80);
}
