/**
 * @title  ATK-MS901M (IMU901) 十轴模块驱动
 * @author: vaie
 * @date: 2026-10-12
 *
 * atk_imu901.h — 正点原子 ATK-MS901M 模块的 UART 驱动。
 *
 * 该模块通过 UART 主动上报 5 类数据；本项目**只使用磁力计 + 气压计**。
 *
 * ★★ 上位机前置配置（必须，否则带宽不够）★★
 *   115200 bps 下若 5 种帧全开，250Hz 需要 172.5 kbps > 波特率上限，
 *   模块会自动降速。**必须**用正点原子上位机配置：
 *     RETURNSET (0x08) DATA = 0x18    // bit3=磁力计, bit4=气压计
 *   只发 mag + baro ⇒ 每周期 28B ⇒ 70kbps，250Hz 满载无压力。
 *
 * 模块上报频率已配置为 250Hz（磁力计与气压计同周期，帧间隔 4ms）。
 *
 * 职责：
 *   · 起串口、探测模块
 *   · 非阻塞状态机解析上报帧（update()），提取磁力计 / 气压计
 *   · 磁力计校准（发 SENCAL 指令 + 45s 等待 + 读 SENSTA 状态）
 *   · 气压零基准校准（固件侧读 100 帧取平均，本地保存 offset）
 *   · 气压校准持久化（LittleFS）
 *
 * 不负责：
 *   · 磁力计校准值的持久化（模块自己存 Flash）
 *   · 磁力计 / 气压计的上层使用（estimator / controller）
 *   · 与 IMU（MPU6500）的融合
 *
 * ★ 编译约束：依赖 HardwareSerial / LittleFS，只在**实机环境**编译。
 *   SIL 环境下本驱动不参与（sim_main.cc 直接注入 mag / baro 数据）。
 *
 * ★ 并发约束：
 *   update() 由 sensors_task（1kHz）调用；calibrateXxxBlocking() 由校准任务调用。
 *   两者共享 serial_ ⇒ 内部用 serial_mutex_ 互斥。
 *   update() 用 try_take(0)：拿不到锁直接跳过本拍，绝不阻塞 1kHz 主循环。
 */
#pragma once

#include <cstdint>
#include <HardwareSerial.h>
#include "type/stabilizer_types.h" // Axis3i16

class AtkImu901
{
public:
    AtkImu901(HardwareSerial &serial, int rx_pin, int tx_pin);
    ~AtkImu901();

    AtkImu901(const AtkImu901 &) = delete;
    AtkImu901 &operator=(const AtkImu901 &) = delete;

    // ---- 生命周期 ----
    //   起串口 + 探测模块是否在发送数据（最长阻塞 ~2s）。
    //   内部按 250Hz 上报频率计算高度低通 alpha。
    //   ★ 前置条件：模块已被上位机配置为只上报 mag + baro（RETURNSET=0x18）。
    //   ★ 不做校准：校准耗时 45s，由独立任务显式调 calibrateXxxBlocking()。
    bool begin();

    // ---- 非阻塞更新 ----
    //   逐字节喂状态机。1kHz 主循环里可每拍调用。
    //   ★ 绝不阻塞（serial_mutex_ 用 try_take(0)）、绝不打日志
    //     （除 verbose_ 模式下的帧校验错误）。
    void update();

    // ---- 数据查询 ----
    //   内部有新数据时返回 true 并填充 out，同时清除"新"标志。
    //   返回 false 表示无新帧（out 内容未定义）。
    //   单读者语义：sensors_task 独占调用。
    bool readMag(mag_s *out);
    bool readBaro(baro_s *out);

    // ---- 校准（阻塞，最长 45s，**绝不能**在 1kHz 任务里调）----
    //
    // 磁力计校准流程（手册 §2.4.7）：
    //   ① 发 SENCAL(0x01) DATA=0x01（写操作指令）
    //   ② 等待 45s（用户缓慢旋转 X/Y/Z 各一圈）
    //   ③ 读 SENSTA(0x82) 检查 bit1（磁力计已校准位）
    //   ④ 校准值由**模块内部 Flash 保存**，固件侧无需持久化
    //
    // 气压计校准流程（固件侧）：
    //   ① 读 100 帧（0.4s @250Hz）取平均，作为 offset_p / offset_a
    //   ② 调用方负责调 saveCalibration() 持久化
    //
    // ★ 两者成功后**不自动保存**，由调用方显式调 saveCalibration()。
    bool calibrateMagBlocking();
    bool calibrateBaroBlocking();

    // ---- 气压校准数据持久化 ----
    //   只保存 offset_p / offset_a；磁力计校准在模块内部 Flash，与本接口无关。
    //   path == nullptr 时使用默认路径 /calibration.dat。
    bool saveCalibration(const char *path = nullptr);
    bool loadCalibration(const char *path = nullptr);
    bool calibrationValid() const;
    void printCalibration() const;

    // ---- 诊断 ----
    void setVerbose(bool on) { verbose_ = on; }
    uint32_t frameErrorCount() const { return frame_err_count_; }

    // ---- 诊断：实测帧率（最近一个统计窗口）----
    //   调用后窗口计数清零。窗口时长 = 距上次调用（或 begin()）的时间。
    struct RateStats
    {
        uint16_t mag_hz;       // 实测 mag 帧率
        uint16_t baro_hz;      // 实测 baro 帧率
        uint32_t frame_errors; // 窗口内错误增量
        uint32_t window_ms;    // 本次窗口时长
    };
    RateStats sampleRateStats();

    // ---- 诊断：读模块配置的上报速率（阻塞 ~100ms，只应在启动期调）----
    //   返回 Hz；0 = 读取失败。
    //   编码：0=250, 1=200, 2=125, 3=100, 4=50, 5=20, 6=10, 7=5, 8=2, 9=1
    uint16_t queryConfiguredRate();

    // ---- 累计帧计数（供遥测/日志长期观察）----
    uint32_t magFrameCount() const { return mag_count_total_; }
    uint32_t baroFrameCount() const { return baro_count_total_; }
    // ---- 诊断：读模块配置的上报内容（RETURNSET 0x08）----
    //   返回位掩码；0 = 读取失败。
    //   位定义（手册 §2.3.2 第 9 节）：
    //     bit0 = 姿态角      bit1 = 四元数
    //     bit2 = 陀螺加计    bit3 = 磁力计    ★ 需要
    //     bit4 = 气压计      ★ 需要          bit5 = 端口状态
    //     bit6 = 匿名上位机
    uint8_t queryConfiguredContent();

    // ---- 强制配置：只上报 mag + baro，速率 250Hz，保存到模块 Flash ----
    //   ★ 会覆盖模块现有配置。正常只在标定时调一次。
    //   返回 true = 三条指令全部发送成功。
    bool configureMagBaroOnly(uint16_t rate_hz = 250);

private:
    // 帧率统计
    uint32_t mag_count_total_ = 0;
    uint32_t baro_count_total_ = 0;
    uint32_t mag_count_window_ = 0;
    uint32_t baro_count_window_ = 0;
    uint32_t last_rate_ms_ = 0;
    uint32_t last_err_count_ = 0;

    // ---- 常量 ----
    //   模块上报频率已配置为 250Hz（磁力计 / 气压计同周期，4ms 一帧）。
    static constexpr uint32_t kReportHz = 250;
    static constexpr uint32_t kRxTimeoutMs = 100;   // 帧内 100ms 无字节则重置状态机
    static constexpr uint32_t kBaroCalFrames = 100; // 气压零基准取 100 帧（0.4s @250Hz）
    static constexpr uint32_t kMagCalMs = 45000;    // 磁力计校准等待时间（手册 §2.4.7）
    static constexpr float kAltLpfHz = 5.0f;        // 高度一阶低通截止频率
    static constexpr uint8_t kMaxFrameLen = 32;     // 最长帧 15B（气压），32 有余量

    // ---- 帧接收状态机 ----
    //   上报帧：0x55 0x55 <id> <len> <data...> <checksum>
    //   指令帧：0x55 0xAF <id> <len> <data...> <checksum>
    //   checksum = 前 4+N 字节和
    enum class RxState : uint8_t
    {
        WaitHead1,
        WaitHead2,
        WaitId,
        WaitLen,
        WaitData,
    };

    RxState rx_state_ = RxState::WaitHead1;
    uint8_t rx_buf_[kMaxFrameLen];
    uint8_t rx_idx_ = 0;
    uint8_t rx_expected_len_ = 0;
    uint32_t rx_last_byte_ms_ = 0;

    // ---- 帧解析 ----
    //   磁力计 0x04：dataLen == 0x08
    //   气压计 0x05：dataLen == 0x0A
    //   其余帧 ID 一律忽略（配置为只上报 mag + baro 后不应出现）
    bool parseFrame_(const uint8_t *buf, uint8_t len);
    static uint8_t checksum_(const uint8_t *data, size_t n);

    // ---- 指令帧交互（阻塞）----
    //   wait_ack=false：发完即返回（返回 200）。
    //   wait_ack=true ：等模块响应（3s 超时）；成功返回状态字节，
    //                   超时/校验失败返回 255。
    //   ★ 内部会 take serial_mutex_；调用方**不要**再持锁。
    //   ★ 响应帧的 frameID 在 byte[2]（不是 byte[3]，那是 dataLength）。
    uint8_t sendInstruct_(uint8_t frame_id, uint8_t data, bool wait_ack);

    // ---- 阻塞读指定帧（校准内部用）----
    //   返回 true 表示在 timeout_ms 内收到完整帧且校验通过；
    //   帧数据拷贝到 out[0..expected_len-1]。
    //   ★ 调用方必须已持 serial_mutex_。
    bool readRawFrameBlocking_(uint8_t frame_id,
                               uint8_t *out, uint8_t expected_len,
                               uint32_t timeout_ms);

    // ---- 最新数据缓存 ----
    mag_s mag_;
    baro_s baro_;
    bool new_mag_ = false;
    bool new_baro_ = false;

    // ---- 气压零基准（固件侧校准）----
    int32_t offset_p_ = 0; // Pa
    int32_t offset_a_ = 0; // cm

    // ---- 高度一阶低通 ----
    float altitude_lpf_ = 0.0f;
    bool altitude_lpf_ready_ = false;

    // ---- 串口 ----
    HardwareSerial &serial_;
    int rx_pin_;
    int tx_pin_;

    // ---- 并发保护 ----
    //   update()（1kHz sensors_task）与校准接口（校准任务）共享 serial_，
    //   必须互斥。update() 用 try_take(0)，拿不到就跳过本拍。
    SemaphoreHandle_t serial_mutex_ = nullptr;

    // ---- 运行时状态 ----
    uint32_t frame_err_count_ = 0;
    float altitude_lpf_alpha_ = 0.0f; // begin() 里按 kReportHz 计算（≈ 0.1116）

    // ---- 诊断 ----
    bool verbose_ = false;
};