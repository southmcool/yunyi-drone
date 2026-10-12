/**
 * @title  ATK-MS901M (IMU901) 十轴模块驱动实现
 * @author: vaie
 * @date: 2026-10-12
 *
 * 从 SensorLib 迁移而来，改动：
 *   · 只保留磁力计 / 气压计解析（删除姿态角 / 陀螺 / 加计 / 四元数 / 端口）
 *   · update() 非阻塞（try_take(0)），不再在 1kHz 里打印
 *   · 时间戳 64 位（esp_timer_get_time）
 *   · 校准流程封装为 calibrateMagBlocking() / calibrateBaroBlocking()
 *   · 校准文件与 ImuCalibStore 分离（/mag_baro_calib.dat）
 *
 * 依赖：HardwareSerial / esp_timer / FreeRTOS / LittleFS，只在实机环境编译。
 */

#include "drivers/mag_baro/atk_imu901.h"

#if !defined(SIL_HOST)

#include <cstring>
#include <Arduino.h>
#include <LittleFS.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ============================================================================
//  常量
// ============================================================================

namespace
{

    // 上报帧 ID（模块主动发送）
    constexpr uint8_t kFrameIdMag = 0x04;  // 磁力计：dataLen 0x08
    constexpr uint8_t kFrameIdBaro = 0x05; // 气压计：dataLen 0x0A

    // 上报帧 dataLen
    constexpr uint8_t kMagDataLen = 0x08;
    constexpr uint8_t kBaroDataLen = 0x0A;

    // 指令帧 ID（上位机发送）
    constexpr uint8_t kCmdSencal = 0x01;         // 设置传感器校准
    constexpr uint8_t kCmdSenstaR = 0x02 | 0x80; // 读校准状态（bit7=读）
    constexpr uint8_t kCmdBaucal = 0x0E;         // 气压计自校准开关

    // SENCAL 指令的 DATA 参数
    constexpr uint8_t kSencalMag = 0x01;
    constexpr uint8_t kSencalBaro = 0x02;

    // SENSTA 返回的状态位
    constexpr uint8_t kStaMagCal = 0x02; // bit1 = 磁力计已校准

    // 校准文件
    constexpr const char *kCalibPath = "/mag_baro_calib.dat";
    constexpr uint32_t kCalibMagic = 0x4D424152; // 'MBAR'
    constexpr uint16_t kCalibVersion = 1;

    // 指令帧收发
    constexpr uint32_t kInstructTimeoutMs = 3000;
    constexpr uint32_t kInstructSendGapMs = 100;

    inline uint64_t nowMicros()
    {
        return static_cast<uint64_t>(esp_timer_get_time());
    }

} // namespace

// ============================================================================
//  构造 / 析构
// ============================================================================

AtkImu901::AtkImu901(HardwareSerial &serial, int rx_pin, int tx_pin)
    : serial_(serial), rx_pin_(rx_pin), tx_pin_(tx_pin)
{
    serial_mutex_ = xSemaphoreCreateMutex();
}

AtkImu901::~AtkImu901()
{
    if (serial_mutex_)
    {
        vSemaphoreDelete(serial_mutex_);
        serial_mutex_ = nullptr;
    }
}

// ============================================================================
//  begin
// ============================================================================

bool AtkImu901::begin()
{
    // 起串口（模块默认 115200 8N1；若改过需同步）
    // ★ 加大 RX 缓冲（默认 256B）
    //    必须在 begin() 之前调，begin 之后无效
    serial_.setRxBufferSize(4096);
    serial_.begin(115200, SERIAL_8N1, rx_pin_, tx_pin_);
    delay(1000); // 等模块稳定（模块自身也有上电流程）

    // 按 250Hz 计算高度低通 α
    const float dt = 1.0f / (float)kReportHz;
    altitude_lpf_alpha_ = 1.0f / (1.0f + 1.0f / (2.0f * PI * kAltLpfHz * dt));

    // 探测：2 秒内有字节到达即视为在线
    while (serial_.available())
        serial_.read();
    const uint32_t start = millis();
    bool detected = false;
    while (millis() - start < 2000)
    {
        if (serial_.available() > 0)
        {
            detected = true;
            break;
        }
        delay(10);
    }

    if (detected)
    {
        // ★ 查询模块配置的上报速率
        const uint16_t cfg_rate = queryConfiguredRate();
        if (cfg_rate > 0)
        {
            Serial.printf("[ATK901] 模块配置上报速率: %u Hz\n", cfg_rate);
            if (cfg_rate < kReportHz)
            {
                Serial.printf("[ATK901] ⚠ 配置速率低于预期（%u < %u）"
                              " —— 上位机检查 RETURNRATE 并点\"保存配置\"\n",
                              cfg_rate, kReportHz);
            }
        }
        else
        {
            Serial.println("[ATK901] ⚠ 无法读取模块配置速率（通讯异常？）");
        }
        // ---- ★ 新加：查上报内容 ----
        const uint8_t content = queryConfiguredContent();
        if (content > 0)
        {
            Serial.printf("[ATK901] 上报内容: 0x%02X | ", content);
            if (content & 0x01)
                Serial.print("姿态 ");
            if (content & 0x02)
                Serial.print("四元数 ");
            if (content & 0x04)
                Serial.print("陀螺加计 ");
            if (content & 0x08)
                Serial.print("磁力 ");
            if (content & 0x10)
                Serial.print("气压 ");
            if (content & 0x20)
                Serial.print("端口 ");
            if (content & 0x40)
                Serial.print("匿名 ");
            Serial.println();

            // 期望：只勾 bit3(磁力) + bit4(气压) = 0x18
            constexpr uint8_t kExpected = 0x18;
            const uint8_t extra = content & ~kExpected;
            if (extra != 0)
            {
                Serial.printf("[ATK901] ⚠ 冗余帧启用 (0x%02X) —— 这是降速的直接原因！\n"
                              "[ATK901] ⚠ 上位机\"内容\"区只勾【磁力】+【气压】，其余全关，然后点保存\n",
                              extra);
            }
            else if (content == kExpected)
            {
                Serial.println("[ATK901] ✓ 上报内容正确（仅 磁力 + 气压）");
            }
        }
        else
        {
            Serial.println("[ATK901] ⚠ 无法读取上报内容");
        }
        last_rate_ms_ = millis(); // ★ 初始化统计窗口
    }

    // configureMagBaroOnly(250);
    // Serial.println("[ATK901] 请断电重启模块，让新配置生效");

    return detected;
}

// ============================================================================
//  update —— 非阻塞状态机
// ============================================================================

void AtkImu901::update()
{
    // ★ 拿不到锁就跳过本拍。绝不阻塞 1kHz 主循环。
    if (xSemaphoreTake(serial_mutex_, 0) != pdTRUE)
        return;

    while (serial_.available() > 0)
    {
        const uint8_t b = (uint8_t)serial_.read();
        rx_last_byte_ms_ = millis();

        switch (rx_state_)
        {
        case RxState::WaitHead1:
            if (b == 0x55)
            {
                rx_buf_[0] = b;
                rx_idx_ = 1;
                rx_state_ = RxState::WaitHead2;
            }
            break;

        case RxState::WaitHead2:
            if (b == 0x55)
            {
                rx_buf_[1] = b;
                rx_idx_ = 2;
                rx_state_ = RxState::WaitId;
            }
            else if (b == 0x55)
            { /* unreachable */
            }
            else
            {
                rx_state_ = RxState::WaitHead1;
                rx_idx_ = 0;
            }
            break;

        case RxState::WaitId:
            rx_buf_[rx_idx_++] = b;
            rx_state_ = RxState::WaitLen;
            break;

        case RxState::WaitLen:
        {
            rx_buf_[rx_idx_++] = b;
            const uint8_t data_len = b;
            rx_expected_len_ = (uint8_t)(4 + data_len + 1); // 头4 + 数据N + 校验1
            if (rx_expected_len_ > kMaxFrameLen)
            {
                rx_state_ = RxState::WaitHead1;
                rx_idx_ = 0;
            }
            else
            {
                rx_state_ = RxState::WaitData;
            }
            break;
        }

        case RxState::WaitData:
            rx_buf_[rx_idx_++] = b;
            if (rx_idx_ >= rx_expected_len_)
            {
                if (!parseFrame_(rx_buf_, rx_expected_len_))
                    frame_err_count_++;
                rx_state_ = RxState::WaitHead1;
                rx_idx_ = 0;
            }
            break;
        }
    }

    // 帧内超时：状态机停在中间太久 → 重置
    if (rx_state_ != RxState::WaitHead1 && (millis() - rx_last_byte_ms_) > kRxTimeoutMs)
    {
        rx_state_ = RxState::WaitHead1;
        rx_idx_ = 0;
    }

    xSemaphoreGive(serial_mutex_);
}

// ============================================================================
//  parseFrame_ —— 上报帧解析
// ============================================================================

uint8_t AtkImu901::checksum_(const uint8_t *data, size_t n)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i)
        sum = (uint8_t)(sum + data[i]);
    return sum;
}

bool AtkImu901::parseFrame_(const uint8_t *buf, uint8_t len)
{
    // 校验：前 len-1 字节之和 == 最后一字节
    if (len < 5)
        return false;
    if (checksum_(buf, len - 1) != buf[len - 1])
        return false;

    const uint8_t frame_id = buf[2];
    const uint8_t data_len = buf[3];
    const uint8_t *d = buf + 4; // 数据段起点
    const uint64_t ts = nowMicros();

    if (frame_id == kFrameIdMag && data_len == kMagDataLen)
    {
        mag_.mag.x = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
        mag_.mag.y = (int16_t)((uint16_t)d[2] | ((uint16_t)d[3] << 8));
        mag_.mag.z = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
        mag_.temperature_c = (float)(int16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8)) / 100.0f;
        mag_.timestamp_us = ts;
        new_mag_ = true;
        mag_count_total_++;  // ★ 加
        mag_count_window_++; // ★ 加
        return true;
    }

    if (frame_id == kFrameIdBaro && data_len == kBaroDataLen)
    {
        // 32 位有符号小端
        int32_t p = (int32_t)((uint32_t)d[0] | ((uint32_t)d[1] << 8) |
                              ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24));
        int32_t a = (int32_t)((uint32_t)d[4] | ((uint32_t)d[5] << 8) |
                              ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24));
        const float t = (float)(int16_t)((uint16_t)d[8] | ((uint16_t)d[9] << 8)) / 100.0f;

        // 扣固件侧校准偏移
        const int32_t p_adj = p - offset_p_;
        const int32_t a_adj = a - offset_a_;

        // 高度一阶低通
        if (!altitude_lpf_ready_)
        {
            altitude_lpf_ = (float)a_adj;
            altitude_lpf_ready_ = true;
        }
        else
        {
            altitude_lpf_ += altitude_lpf_alpha_ * ((float)a_adj - altitude_lpf_);
        }

        baro_.pressure_pa = p_adj;
        baro_.altitude_cm = (int32_t)(altitude_lpf_ + 0.5f);
        baro_.temperature_c = t;
        baro_.timestamp_us = ts;
        new_baro_ = true;
        baro_count_total_++;  // ★ 加
        baro_count_window_++; // ★ 加
        return true;
    }

    // 其余帧 ID 一律忽略（配置为只上报 mag + baro 后不应出现）
    return true;
}

// ============================================================================
//  readMag / readBaro —— 单读者，读后清标志
// ============================================================================

bool AtkImu901::readMag(mag_s *out)
{
    if (!out || !new_mag_)
        return false;
    *out = mag_;
    new_mag_ = false;
    return true;
}

bool AtkImu901::readBaro(baro_s *out)
{
    if (!out || !new_baro_)
        return false;
    *out = baro_;
    new_baro_ = false;
    return true;
}

// ============================================================================
//  指令帧交互
// ============================================================================

uint8_t AtkImu901::sendInstruct_(uint8_t frame_id, uint8_t data, bool wait_ack)
{
    if (xSemaphoreTake(serial_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        if (verbose_)
            Serial.println("[ATK901] sendInstruct: 拿不到锁");
        return 255;
    }

    // 构造 6 字节指令帧
    uint8_t frame[6];
    frame[0] = 0x55;
    frame[1] = 0xAF;
    frame[2] = frame_id;
    frame[3] = 0x01;
    frame[4] = data;
    frame[5] = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] + frame[4]);

    // 清 buffer，发送
    while (serial_.available())
        serial_.read();
    serial_.write(frame, sizeof(frame));
    delay(kInstructSendGapMs);

    if (!wait_ack)
    {
        xSemaphoreGive(serial_mutex_);
        return 200;
    }

    // 等响应：0x55 0xAF <id> <len> <data> <sum>
    uint8_t resp[8];
    uint8_t pos = 0;
    uint8_t expected_len = 0;
    const uint32_t start = millis();

    while (millis() - start < kInstructTimeoutMs)
    {
        if (serial_.available() == 0)
        {
            delay(1);
            continue;
        }
        const uint8_t b = (uint8_t)serial_.read();

        if (pos == 0 && b != 0x55)
            continue;
        if (pos == 1 && b != 0xAF)
        {
            pos = 0;
            continue;
        }

        resp[pos++] = b;

        if (pos == 4)
            expected_len = (uint8_t)(4 + resp[3] + 1);
        if (expected_len > 0 && pos >= expected_len)
        {
            const uint8_t sum = checksum_(resp, expected_len - 1);
            if (sum != resp[expected_len - 1])
            {
                if (verbose_)
                    Serial.println("[ATK901] 指令响应校验失败");
                xSemaphoreGive(serial_mutex_);
                return 255;
            }
            const uint8_t status = resp[4]; // data 段首字节
            xSemaphoreGive(serial_mutex_);
            return status;
        }
        if (pos >= sizeof(resp))
        {
            pos = 0;
            expected_len = 0;
        }
    }

    if (verbose_)
        Serial.println("[ATK901] 指令响应超时");
    xSemaphoreGive(serial_mutex_);
    return 255;
}

bool AtkImu901::readRawFrameBlocking_(uint8_t frame_id,
                                      uint8_t *out, uint8_t expected_len,
                                      uint32_t timeout_ms)
{
    // ★ 调用方已持 serial_mutex_，这里不再 take
    uint8_t buf[16];
    uint8_t pos = 0;
    const uint32_t start = millis();

    while (millis() - start < timeout_ms)
    {
        if (serial_.available() == 0)
        {
            delay(1);
            continue;
        }
        const uint8_t b = (uint8_t)serial_.read();

        if (pos == 0 && b != 0x55)
            continue;
        if (pos == 1 && b != 0x55)
        {
            pos = 0;
            continue;
        }
        if (pos == 2 && b != frame_id)
        {
            pos = 0;
            continue;
        }

        buf[pos++] = b;
        if (pos >= 4 + expected_len + 1)
        {
            const uint8_t total = (uint8_t)(4 + expected_len + 1);
            if (checksum_(buf, total - 1) != buf[total - 1])
            {
                pos = 0;
                continue;
            }
            std::memcpy(out, buf + 4, expected_len);
            return true;
        }
    }
    return false;
}

// ============================================================================
//  磁力计校准（45s，阻塞）
// ============================================================================

bool AtkImu901::calibrateMagBlocking()
{
    Serial.println("[ATK901] 磁力计校准开始：请在 45s 内缓慢旋转 X/Y/Z 各一圈");

    // ① 发 SENCAL(0x01) DATA=0x01，不等响应
    const uint8_t r = sendInstruct_(kCmdSencal, kSencalMag, false);
    if (r != 200)
    {
        Serial.printf("[ATK901] 磁力计校准指令发送失败: %u\n", r);
        return false;
    }

    // ② 等 45s（手册 §2.4.7）
    //    ★ 这是阻塞操作，绝不能在 1kHz 任务里调
    delay(kMagCalMs);

    // ③ 读 SENSTA(0x82) 检查 bit1
    const uint8_t sta = sendInstruct_(kCmdSenstaR, 0x00, true);
    if (sta == 255)
    {
        Serial.println("[ATK901] 磁力计校准状态读取失败");
        return false;
    }

    const bool ok = (sta & kStaMagCal) != 0;
    Serial.printf("[ATK901] 磁力计校准%s (SENSTA=0x%02X)\n", ok ? "成功" : "失败", sta);
    return ok;
}

// ============================================================================
//  气压计校准（固件侧读 100 帧，0.4s，阻塞）
// ============================================================================

bool AtkImu901::calibrateBaroBlocking()
{
    Serial.println("[ATK901] 气压零基准校准：读 100 帧（0.4s）取平均");

    if (xSemaphoreTake(serial_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        Serial.println("[ATK901] 气压校准: 拿不到锁");
        return false;
    }

    int64_t sum_p = 0;
    int64_t sum_a = 0;
    int valid = 0;
    uint8_t frame[16];

    for (int i = 0; i < (int)kBaroCalFrames; ++i)
    {
        if (!readRawFrameBlocking_(kFrameIdBaro, frame, kBaroDataLen, 200))
            continue;

        const int32_t p = (int32_t)((uint32_t)frame[0] | ((uint32_t)frame[1] << 8) |
                                    ((uint32_t)frame[2] << 16) | ((uint32_t)frame[3] << 24));
        const int32_t a = (int32_t)((uint32_t)frame[4] | ((uint32_t)frame[5] << 8) |
                                    ((uint32_t)frame[6] << 16) | ((uint32_t)frame[7] << 24));
        sum_p += p;
        sum_a += a;
        ++valid;
    }

    xSemaphoreGive(serial_mutex_);

    if (valid == 0)
    {
        Serial.println("[ATK901] 气压校准失败：无有效帧");
        return false;
    }

    offset_p_ = (int32_t)(sum_p / valid);
    offset_a_ = (int32_t)(sum_a / valid);

    // 复位高度低通，避免旧值污染
    altitude_lpf_ready_ = false;
    altitude_lpf_ = 0.0f;

    Serial.printf("[ATK901] 气压校准完成：offset_p=%ld Pa  offset_a=%ld cm  (n=%d)\n",
                  (long)offset_p_, (long)offset_a_, valid);
    return true;
}

// ============================================================================
//  校准持久化（只存气压偏移）
// ============================================================================

namespace
{

    struct CalibFile
    {
        uint32_t magic;
        uint16_t version;
        uint16_t reserved;
        int32_t offset_p;
        int32_t offset_a;
        uint32_t checksum; // 前 16 字节之和
    };

    static_assert(sizeof(CalibFile) == 20, "CalibFile 应为 20 字节");

    uint32_t calibChecksum(const CalibFile &f)
    {
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&f);
        uint32_t sum = 0;
        for (size_t i = 0; i < sizeof(CalibFile) - sizeof(uint32_t); ++i)
            sum += p[i];
        return sum;
    }

} // namespace

bool AtkImu901::saveCalibration(const char *path)
{
    const char *p = path ? path : kCalibPath;

    if (LittleFS.exists(p))
        LittleFS.remove(p);

    CalibFile f{};
    f.magic = kCalibMagic;
    f.version = kCalibVersion;
    f.offset_p = offset_p_;
    f.offset_a = offset_a_;
    f.checksum = calibChecksum(f);

    File file = LittleFS.open(p, "w");
    if (!file)
    {
        Serial.printf("[ATK901] 无法写文件: %s\n", p);
        return false;
    }
    const size_t w = file.write(reinterpret_cast<const uint8_t *>(&f), sizeof(f));
    file.close();
    if (w != sizeof(f))
    {
        Serial.printf("[ATK901] 写入不完整: %u/%u\n", (unsigned)w, (unsigned)sizeof(f));
        LittleFS.remove(p);
        return false;
    }

    // 读回校验
    File vf = LittleFS.open(p, "r");
    if (!vf)
    {
        LittleFS.remove(p);
        return false;
    }
    CalibFile rb{};
    const size_t r = vf.read(reinterpret_cast<uint8_t *>(&rb), sizeof(rb));
    vf.close();
    if (r != sizeof(rb) ||
        rb.magic != kCalibMagic ||
        rb.checksum != calibChecksum(rb) ||
        std::memcmp(&f, &rb, sizeof(f)) != 0)
    {
        Serial.println("[ATK901] 读回校验失败");
        LittleFS.remove(p);
        return false;
    }

    Serial.printf("[ATK901] 校准已保存: %s\n", p);
    return true;
}

bool AtkImu901::loadCalibration(const char *path)
{
    const char *p = path ? path : kCalibPath;
    if (!LittleFS.exists(p))
        return false;

    File file = LittleFS.open(p, "r");
    if (!file)
        return false;

    CalibFile f{};
    const size_t r = file.read(reinterpret_cast<uint8_t *>(&f), sizeof(f));
    file.close();

    if (r != sizeof(f))
        return false;
    if (f.magic != kCalibMagic)
        return false;
    if (f.version != kCalibVersion)
        return false;
    if (f.checksum != calibChecksum(f))
        return false;

    offset_p_ = f.offset_p;
    offset_a_ = f.offset_a;

    // 复位高度低通
    altitude_lpf_ready_ = false;
    altitude_lpf_ = 0.0f;

    Serial.printf("[ATK901] 校准已加载: offset_p=%ld Pa  offset_a=%ld cm\n",
                  (long)offset_p_, (long)offset_a_);
    return true;
}

bool AtkImu901::calibrationValid() const
{
    return (offset_p_ != 0) || (offset_a_ != 0);
}

void AtkImu901::printCalibration() const
{
    Serial.println("=== ATK901 气压校准 ===");
    Serial.printf("  offset_p = %ld Pa\n", (long)offset_p_);
    Serial.printf("  offset_a = %ld cm\n", (long)offset_a_);
    Serial.printf("  有效性   = %s\n", calibrationValid() ? "有效" : "无效");
}

// ============================================================================
//  诊断：实测帧率
// ============================================================================

AtkImu901::RateStats AtkImu901::sampleRateStats()
{
    const uint32_t now = millis();
    const uint32_t dt = now - last_rate_ms_;

    RateStats s{};
    if (dt > 0)
    {
        s.mag_hz = (uint16_t)((uint64_t)mag_count_window_ * 1000ULL / dt);
        s.baro_hz = (uint16_t)((uint64_t)baro_count_window_ * 1000ULL / dt);
    }
    s.frame_errors = frame_err_count_ - last_err_count_;
    s.window_ms = dt;

    // 重置窗口
    mag_count_window_ = 0;
    baro_count_window_ = 0;
    last_err_count_ = frame_err_count_;
    last_rate_ms_ = now;
    return s;
}

// ============================================================================
//  诊断：读模块配置的上报速率
// ============================================================================

uint16_t AtkImu901::queryConfiguredRate()
{
    // 读 RETURNRATE (0x0A | 0x80 = 0x8A)，DATA 段为空
    const uint8_t r = sendInstruct_(0x0A | 0x80, 0x00, /*wait_ack=*/true);
    if (r == 255)
        return 0; // 超时/校验失败

    switch (r)
    {
    case 0x00:
        return 250;
    case 0x01:
        return 200;
    case 0x02:
        return 125;
    case 0x03:
        return 100;
    case 0x04:
        return 50;
    case 0x05:
        return 20;
    case 0x06:
        return 10;
    case 0x07:
        return 5;
    case 0x08:
        return 2;
    case 0x09:
        return 1;
    default:
        return 0; // 未识别编码
    }
}

// ============================================================================
//  诊断：读模块配置的上报内容
// ============================================================================

uint8_t AtkImu901::queryConfiguredContent()
{
    // 读 RETURNSET (0x08 | 0x80 = 0x88)，DATA 段为空
    const uint8_t r = sendInstruct_(0x08 | 0x80, 0x00, /*wait_ack=*/true);
    if (r == 255)
        return 0; // 超时/校验失败
    return r;     // 1 字节掩码
}

bool AtkImu901::configureMagBaroOnly(uint16_t rate_hz)
{
    Serial.println("[ATK901] 强制配置：内容=mag+baro, 速率=..., 保存 Flash");

    // ① 设置上报内容 = 0x18（bit3 磁力 + bit4 气压）
    //    RETURNSET (0x08) 写操作：帧ID bit7 = 0
    {
        const uint8_t r = sendInstruct_(0x08, 0x18, /*wait_ack=*/false);
        if (r != 200)
        {
            Serial.printf("[ATK901] 设置上报内容失败: %u\n", r);
            return false;
        }
    }

    // ② 设置上报速率（RETURNRATE 0x0A）
    //    编码：0=250, 1=200, 2=125, 3=100, 4=50, 5=20, 6=10, 7=5, 8=2, 9=1
    uint8_t rate_code = 0;
    switch (rate_hz)
    {
    case 250:
        rate_code = 0x00;
        break;
    case 200:
        rate_code = 0x01;
        break;
    case 125:
        rate_code = 0x02;
        break;
    case 100:
        rate_code = 0x03;
        break;
    case 50:
        rate_code = 0x04;
        break;
    case 20:
        rate_code = 0x05;
        break;
    case 10:
        rate_code = 0x06;
        break;
    default:
        Serial.printf("[ATK901] 不支持的速率 %u，用 250\n", rate_hz);
        rate_code = 0x00;
        break;
    }
    {
        const uint8_t r = sendInstruct_(0x0A, rate_code, false);
        if (r != 200)
        {
            Serial.printf("[ATK901] 设置上报速率失败: %u\n", r);
            return false;
        }
    }

    // ③ 保存到 Flash（SAVE 0x00）
    {
        const uint8_t r = sendInstruct_(0x00, 0x00, false);
        if (r != 200)
        {
            Serial.printf("[ATK901] 保存配置失败: %u\n", r);
            return false;
        }
    }

    Serial.println("[ATK901] ✓ 配置已写入，复位模块后生效");
    Serial.println("[ATK901]   建议：拔电重上，或执行 module reboot");
    return true;
}

#endif // !SIL_HOST