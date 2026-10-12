/**
 * @title  系统状态与错误码
 * @author: vaie
 * @date: 2026-10-12
 *
 * system_status.h — 全局状态机 + 错误码通道。
 *
 * 动机：
 *   · 各任务自行决定"删任务"或"继续"，没有统一策略
 *   · 没有"能否解锁"的判据
 *   · LED / 蜂鸣器 / 地面站无法一致地反映系统状态
 *
 * 设计：
 *   · 状态机（单值）：BOOT → READY → ARMED → FLYING → FAILSAFE
 *   · 错误码（位图）：每个错误一个 bit，可多个同时存在
 *   · 严重级别：FATAL 阻止解锁；WARN 只记录
 *
 * 并发：
 *   · 单写多读
 *   · 状态与错误位用 std::atomic；写入频率低，无需加锁
 *
 * 依赖：仅 <cstdint> + <atomic>，SIL 与实机共用。
 */
#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
//  系统状态
// ---------------------------------------------------------------------------
enum class SystemState : uint8_t
{
    BOOT = 0,     // 启动中，任务正在初始化
    READY = 1,    // 初始化完成，等待解锁
    ARMED = 2,    // 已解锁，电机怠速
    FLYING = 3,   // 飞行中
    FAILSAFE = 4, // 失控保护触发
    ERROR = 5,    // 致命错误，无法继续
};

// ---------------------------------------------------------------------------
//  错误码（位图，可多选）
//   FATAL_*  阻止解锁
//   WARN_*   只记录
// ---------------------------------------------------------------------------
enum SystemError : uint32_t
{
    ERR_NONE = 0,

    // ---- FATAL ----
    ERR_IMU_INIT_FAIL = 1u << 0,   // MPU6500 SPI / WHO_AM_I 失败
    ERR_IMU_NO_DATA = 1u << 1,     // IMU 连续无数据（超时）
    ERR_STABILIZER_FAIL = 1u << 2, // 稳定器任务启动失败
    ERR_MOTOR_INIT_FAIL = 1u << 3, // DShot 初始化失败

    // ---- WARN ----
    ERR_MAGBARO_INIT_FAIL = 1u << 8, // ATK-MS901M 通信失败（可降级飞）
    ERR_MAGBARO_NO_DATA = 1u << 9,   // magbaro 超时
    ERR_MAGBARO_NO_CALIB = 1u << 10, // 未标定气压零基准
    ERR_TOF_INIT_FAIL = 1u << 11,    // VL53L1X 失败（可选）
    ERR_TOF_NO_DATA = 1u << 12,
    ERR_FLOW_INIT_FAIL = 1u << 13, // PMW3901 失败（可选）
    ERR_FLOW_NO_DATA = 1u << 14,
    ERR_IMU_NO_CALIB = 1u << 15, // 未标定 IMU 零偏
    ERR_FFT_STALLED = 1u << 16,  // FFT 长时间无输出（自适应陷波失效）
};

// ---------------------------------------------------------------------------
//  统一接口（纯静态）
// ---------------------------------------------------------------------------
class SystemStatus
{
public:
    SystemStatus() = delete;

    // ---- 状态机 ----
    static SystemState state();
    static void setState(SystemState s);
    static const char *stateStr();

    // ---- 错误位 ----
    static void raiseError(SystemError e);
    static void clearError(SystemError e);
    static uint32_t errors();
    static bool hasFatalError();
    static bool hasError(SystemError e);
    static const char *errorStr(SystemError e);

    // ---- 解锁判据 ----
    //   仅当状态 ∈ {READY, ARMED} 且无 FATAL 错误时返回 true。
    static bool canArm();

    // ---- 诊断 ----
    static void dumpErrors();
};

namespace system_error
{
    constexpr uint32_t FATAL_MASK = ERR_IMU_INIT_FAIL | ERR_IMU_NO_DATA | ERR_STABILIZER_FAIL | ERR_MOTOR_INIT_FAIL;
    constexpr uint32_t WARN_MASK = ERR_MAGBARO_INIT_FAIL | ERR_MAGBARO_NO_DATA | ERR_MAGBARO_NO_CALIB | ERR_TOF_INIT_FAIL | ERR_TOF_NO_DATA | ERR_FLOW_INIT_FAIL | ERR_FLOW_NO_DATA | ERR_IMU_NO_CALIB | ERR_FFT_STALLED;
}