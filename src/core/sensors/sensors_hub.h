/**
 * @title  SensorsHub —— 传感器数据汇聚
 * @author: vaie
 * @date: 2026-10-12
 *
 * sensors_hub.h — IMU / 磁力计 / 气压计三类数据的最新值快照。
 *
 * 定位（与 task 层的分工）：
 *   · 各传感器任务独写自己那一份（publishXxx）
 *   · EKF / 控制器 / 遥测从 hub 读最新值（readXxx）
 *   · 不含频率、不含任务、不含调度 —— 那些属于 task 层
 *
 * 数据范围（第一批）：
 *   · IMU  —— ImuSignalChain 的输出（1kHz）
 *   · Mag  —— AtkImu901::readMag（250Hz）
 *   · Baro —— AtkImu901::readBaro（250Hz）
 *   ★ ToF / 光流**不**进本 hub：它们保留现有队列，
 *     避免"为了统一而统一"。等 hub 稳定跑通后再考虑是否迁移。
 *
 * 并发模型：
 *   单写者（每类数据一个传感器任务）/ 多读者（EKF、控制器、遥测）
 *   写者：data = v; fence(release); version++
 *   读者：v1 = version; copy data; fence(acquire); v2 = version;
 *         校验 v1 == v2（相等 ⇒ 未撕裂）
 *   ★ 无锁、不阻塞。读者若读到撕裂版本返回 false，调用方重试一次即可。
 *
 * 时间戳：
 *   每类数据自带 64 位 timestamp_us（esp_timer_get_time，永不回绕）。
 *   hub 不重新打时间戳 —— 时间戳由采集时刻决定，搬运时补是错的。
 *   ★ 三个传感器的时间戳**不同步**：EKF 按各自时间戳融合，不假设同一瞬间。
 *
 * 依赖：仅 type/imu_types.h（纯 C++ 类型），SIL 与实机共用。
 */
#pragma once

#include <cstdint>
#include <atomic>
#include "type/stabilizer_types.h" // ImuData / AtkImu901Mag / AtkImu901Baro
#include "core/telemetry/telemetry_data.h"
class SensorsHub
{
public:
    SensorsHub() = delete; // 纯静态接口，禁止实例化

    // ---- 写：每个传感器任务独占一个 publish ----
    static void publishImu(const ImuData &d);
    static void publishMag(const mag_s &d);
    static void publishBaro(const baro_s &d);

    // ---- 读：多读者 ----
    //   返回 true  表示拿到了未撕裂的快照
    //   返回 false 表示 (a) 从未 publish 过 或 (b) 读取期间被写者插入
    //   ★ 调用方对 (b) 可重试一次；对 (a) 应视为"传感器未就绪"。
    static bool readImu(ImuData *out);
    static bool readMag(mag_s *out);
    static bool readBaro(baro_s *out);

    // ---- 健康查询（30s 诊断用）----
    struct Health
    {
        bool valid;              // 至少 publish 过一次
        uint64_t last_update_us; // 最近一次 publish 的时间戳（0=从未）
        uint32_t update_count;   // 累计 publish 次数
    };
    static Health imuHealth();
    static Health magHealth();
    static Health baroHealth();

    // ---- 离线检测阈值 ----
    //   供诊断任务判断"这个传感器是不是挂了"。
    static constexpr uint64_t kImuTimeoutUs = 100000; // 100ms ≈ 100 拍 @1kHz
    static constexpr uint64_t kMagTimeoutUs = 200000; // 200ms ≈  50 帧 @250Hz
    static constexpr uint64_t kBaroTimeoutUs = 200000;

private:
    // ---- 无锁槽 ----
    //   write：先写 data 与元数据，release fence 保证对读者可见，
    //          最后 version++ 作为"提交"信号。
    //   read ：v1 = version; 拷 data; acquire fence; v2 = version;
    //          v1 == v2 ⇒ 无撕裂。v1 == 0 ⇒ 从未写过。
    //
    //   ★ volatile 是必要的：防编译器把 version 的读提升到调用外
    //     （读者可能在循环里反复 read，会被优化成只读一次）。
    //   ★ fence 用 std::atomic_thread_fence（<atomic>），
    //     GCC / Clang 均支持，SIL 与实机行为一致。
    template <typename T>
    struct Slot
    {
        T data{};
        volatile uint32_t version = 0; // 0 = 从未写过
        volatile uint64_t last_update_us = 0;
        volatile uint32_t update_count = 0;

        void write(const T &v)
        {
            data = v;
            last_update_us = v.timestamp_us; // 所有 T 都有 timestamp_us
            std::atomic_thread_fence(std::memory_order_release);
            version++;
            update_count++;
        }

        bool read(T *out) const
        {
            const uint32_t v1 = version;
            if (v1 == 0)
                return false;
            *out = data;
            std::atomic_thread_fence(std::memory_order_acquire);
            return v1 == version;
        }

        Health health() const
        {
            Health h;
            h.valid = (version != 0);
            h.last_update_us = last_update_us;
            h.update_count = update_count;
            return h;
        }
    };

    static Slot<ImuData> s_imu;
    static Slot<mag_s> s_mag;
    static Slot<baro_s> s_baro;
};