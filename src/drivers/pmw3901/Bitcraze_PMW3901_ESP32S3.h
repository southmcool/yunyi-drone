#ifndef BITCRAZE_PMW3901_ESP32S3_H
#define BITCRAZE_PMW3901_ESP32S3_H

#include <Arduino.h>
#include <SPI.h>

#pragma pack(push, 1)

/**

* PMW3901 Motion Burst 数据。
*
* 对应 Motion_Burst 寄存器读取数据。
* 驱动层保持 PMW3901 原始坐标系，不进行飞控坐标转换。
  */
struct Pmw3901MotionBurst
{
    uint8_t motion;
    uint8_t observation;

    int16_t deltaX;
    int16_t deltaY;

    uint8_t squal;

    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
};

#pragma pack(pop)

/**

* PMW3901 一帧处理后的原始测量结果。
  */
struct Pmw3901Measurement
{
    int16_t deltaX;
    int16_t deltaY;

    bool motionDetected;

    // 本次 Motion Burst 是否成功读取并通过基本合理性检查
    bool valid;

    // 传感器整体健康状态
    bool healthy;

    // Surface Quality
    uint8_t squal;

    // 原始图像统计
    uint8_t rawDataSum;
    uint8_t maxRawData;
    uint8_t minRawData;

    uint16_t shutter;
};

class Bitcraze_PMW3901
{
public:
    Bitcraze_PMW3901(
        uint8_t cspin,
        uint8_t sck,
        uint8_t miso,
        uint8_t mosi);

     bool begin(void);

    /**
     * 读取完整 Motion Burst。
     *
     * 注意：
     * 本函数不负责获取 FreeRTOS SPI Mutex。
     * 调用者必须确保 SPI 总线已经安全获取。
     */
    bool readMotionBurst(Pmw3901MotionBurst &burst);

    /**
     * 读取并进行基本数据有效性判断。
     */
    bool readMeasurement(Pmw3901Measurement &measurement);

    /**
     * 兼容旧接口。
     *
     * 返回 true 表示本次读取有效。
     * deltaX / deltaY 保持 PMW3901 原始坐标。
     */
    bool readMotionCount(
        int16_t *deltaX,
        int16_t *deltaY);

    bool isHealthy() const;


    private : uint8_t _cs;
    uint8_t _sck;
    uint8_t _miso;
    uint8_t _mosi;

     bool _healthy;

    uint16_t _consecutiveErrorCount;

    static constexpr uint16_t MAX_CONSECUTIVE_ERRORS = 10;

    // 单帧原始 Delta 极限保护。
    // 当前是驱动层防止明显异常数据的宽松保护，
    // 后续应根据实际采样率和飞行速度进一步标定。
    static constexpr int16_t MAX_ABSOLUTE_DELTA = 3000;

    bool registerWrite(uint8_t reg, uint8_t value);

    bool registerRead(
        uint8_t reg,
        uint8_t &value);

    bool verifyChipId();

    void initRegisters();

    void markSuccess();

    void markError();

    bool validateBurst(
        const Pmw3901MotionBurst &burst) const;
    
};

#endif
