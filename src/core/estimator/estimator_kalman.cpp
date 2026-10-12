/**
 * @title  Kalman 估计器封装实现
 * @author: vaie
 * @date: 2026-10-12
 */

#include "core/estimator/estimator_kalman.h"
#include "core/estimator/kalman_core.h"

#include <atomic>
#include <cstring>

// ============================================================================
//  内部状态
// ============================================================================

namespace
{

    // ---- 唯一 kalman 实例（POD 风格，零初始化）----
    kalmanCoreData_t g_kf;

    // ---- 复位请求（原子，避免读撕裂）----
    std::atomic<bool> g_reset_request{false};

    // ---- state_t 的无锁 Slot（与 SensorsHub 同模型）----
    struct StateSlot
    {
        state_t data{};
        volatile uint32_t version = 0;

        void write(const state_t &v)
        {
            data = v;
            std::atomic_thread_fence(std::memory_order_release);
            version++;
        }

        bool read(state_t *out) const
        {
            const uint32_t v1 = version;
            if (v1 == 0)
                return false;
            *out = data;
            std::atomic_thread_fence(std::memory_order_acquire);
            return v1 == version;
        }
    };

    StateSlot g_state_slot;

} // namespace

// ============================================================================
//  生命周期
// ============================================================================

void EstimatorKalman::init()
{
    internalReset();
    g_reset_request.store(false, std::memory_order_release);
    // Slot 也清（用 version=0 表示"从未发布"）
    g_state_slot.version = 0;
    std::memset(&g_state_slot.data, 0, sizeof(g_state_slot.data));
}

void EstimatorKalman::internalReset()
{
    kalmanCoreInit(&g_kf);
}

void EstimatorKalman::requestReset()
{
    g_reset_request.store(true, std::memory_order_release);
}

bool EstimatorKalman::consumeResetRequest()
{
    // exchange：原子取走标志（置 false）
    const bool was = g_reset_request.exchange(false, std::memory_order_acq_rel);
    if (was)
    {
        internalReset();
    }
    return was;
}

// ============================================================================
//  单步操作
// ============================================================================

void EstimatorKalman::predict(float dt,
                              const Axis3f &acc,
                              const Axis3f &gyro,
                              bool quadIsFlying)
{
    if (dt <= 0.0f)
        return;

    // thrust_norm 当前不用，传 0（kalmanCore 会 fallback 到 acc->z）
    kalmanCorePredict(&g_kf, 0.0f, &acc, &gyro, dt, quadIsFlying);
    kalmanCoreAddProcessNoise(&g_kf, dt);
}

void EstimatorKalman::updateBaro(float baroAsl, bool quadIsFlying)
{
    kalmanCoreUpdateWithBaro(&g_kf, baroAsl, quadIsFlying);
}

void EstimatorKalman::finalize(uint64_t osTickUs)
{
    kalmanCoreFinalize(&g_kf, osTickUs);
}

void EstimatorKalman::publishState(const Axis3f &acc, const Axis3f &gyro_rads, uint64_t osTickUs)
{
    state_t snap;
    kalmanCoreExternalizeState(&g_kf, &snap, &acc, &gyro_rads, osTickUs);
    g_state_slot.write(snap);
}

// ============================================================================
//  状态读取
// ============================================================================

bool EstimatorKalman::readState(state_t *out)
{
    return out && g_state_slot.read(out);
}

bool EstimatorKalman::readQuaternion(float q[4])
{
    if (!q)
        return false;
    // 只读一次快照 —— 由版本号判断是否撕裂
    const uint32_t v1 = g_state_slot.version;
    if (v1 == 0)
        return false;
    const state_t &s = g_state_slot.data;
    q[0] = s.attitudeQuaternion.q0;
    q[1] = s.attitudeQuaternion.q1;
    q[2] = s.attitudeQuaternion.q2;
    q[3] = s.attitudeQuaternion.q3;
    std::atomic_thread_fence(std::memory_order_acquire);
    return v1 == g_state_slot.version;
}

// ============================================================================
//  诊断
// ============================================================================

bool EstimatorKalman::isWithinBounds()
{
    return kalmanCoreIsWithinBounds(&g_kf);
}

void EstimatorKalman::updateTof(const tofMeasurement_t &tof)
{
    kalmanCoreUpdateWithTof(&g_kf, &tof);
}

void EstimatorKalman::updateFlow(const flowMeasurement_t &flow,
                                 const Axis3f &gyro_rads)
{
    kalmanCoreUpdateWithFlow(&g_kf, &flow, &gyro_rads);
}

void EstimatorKalman::resetHorizontal()
{
    kalmanCoreResetHorizontal(&g_kf);
}
void EstimatorKalman::updateMag(const magMeasurement_t &mag)
{
    kalmanCoreUpdateWithMag(&g_kf, &mag);
}