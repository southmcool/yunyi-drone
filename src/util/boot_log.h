/**
 * @title  启动日志
 * @author: vaie
 * @date: 2026-10-12
 *
 * boot_log.h — 统一的启动日志格式。
 *
 * 动机：各任务 printf 格式不一，上机排障时串口日志是乱的。统一为：
 *     [BOOT] [模块名] · 消息      信息
 *     [BOOT] [模块名] ✓ 成功
 *     [BOOT] [模块名] ⚠ 警告
 *     [BOOT] [模块名] ✗ 失败
 *
 * 使用（仅启动期，不进 1kHz 主循环）：
 *     BOOT_LOG_I("IMU", "SPI bus ready, fs=%.0fHz", 1000.0f);
 *     BOOT_LOG_W("MAGBARO", "无气压校准，使用零基准");
 *     BOOT_LOG_E("IMU", "MPU6500 WHO_AM_I 校验失败");
 *
 * ★ 1kHz 循环内禁止调用（printf 阻塞 5~10ms）。
 *   运行期诊断走 imu_fft_task 的低频打印。
 *
 * 依赖：仅 <cstdio>，SIL 与实机共用（SIL 走 stderr）。
 */
#pragma once

namespace bootlog
{
    void info(const char *module, const char *fmt, ...);
    void warn(const char *module, const char *fmt, ...);
    void error(const char *module, const char *fmt, ...);
    void success(const char *module, const char *fmt, ...);
}

#define BOOT_LOG_I(mod, ...) ::bootlog::info(mod, __VA_ARGS__)
#define BOOT_LOG_W(mod, ...) ::bootlog::warn(mod, __VA_ARGS__)
#define BOOT_LOG_E(mod, ...) ::bootlog::error(mod, __VA_ARGS__)
#define BOOT_LOG_S(mod, ...) ::bootlog::success(mod, __VA_ARGS__)