#include "core/system/system_status.h"
#include <atomic>
#include <cstdio>

#if !defined(SIL_HOST)
#include <Arduino.h>
#endif

// ============================================================================
//  内部状态
// ============================================================================

namespace
{
    std::atomic<SystemState> g_state{SystemState::BOOT};
    std::atomic<uint32_t> g_errors{0};
}

// ============================================================================
//  状态机
// ============================================================================

SystemState SystemStatus::state()
{
    return g_state.load(std::memory_order_acquire);
}

void SystemStatus::setState(SystemState s)
{
    g_state.store(s, std::memory_order_release);
}

const char *SystemStatus::stateStr()
{
    switch (state())
    {
    case SystemState::BOOT:
        return "BOOT";
    case SystemState::READY:
        return "READY";
    case SystemState::ARMED:
        return "ARMED";
    case SystemState::FLYING:
        return "FLYING";
    case SystemState::FAILSAFE:
        return "FAILSAFE";
    case SystemState::ERROR:
        return "ERROR";
    }
    return "UNKNOWN";
}

// ============================================================================
//  错误位
// ============================================================================

void SystemStatus::raiseError(SystemError e)
{
    g_errors.fetch_or((uint32_t)e, std::memory_order_release);
}

void SystemStatus::clearError(SystemError e)
{
    g_errors.fetch_and(~(uint32_t)e, std::memory_order_release);
}

uint32_t SystemStatus::errors()
{
    return g_errors.load(std::memory_order_acquire);
}

bool SystemStatus::hasFatalError()
{
    return (errors() & system_error::FATAL_MASK) != 0;
}

bool SystemStatus::hasError(SystemError e)
{
    return (errors() & (uint32_t)e) != 0;
}

// ============================================================================
//  解锁判据
// ============================================================================

bool SystemStatus::canArm()
{
    const SystemState s = state();
    if (s != SystemState::READY && s != SystemState::ARMED)
        return false;
    return !hasFatalError();
}

// ============================================================================
//  诊断
// ============================================================================

const char *SystemStatus::errorStr(SystemError e)
{
    switch (e)
    {
    case ERR_NONE:
        return "NONE";
    case ERR_IMU_INIT_FAIL:
        return "IMU_INIT_FAIL";
    case ERR_IMU_NO_DATA:
        return "IMU_NO_DATA";
    case ERR_STABILIZER_FAIL:
        return "STABILIZER_FAIL";
    case ERR_MOTOR_INIT_FAIL:
        return "MOTOR_INIT_FAIL";
    case ERR_MAGBARO_INIT_FAIL:
        return "MAGBARO_INIT_FAIL";
    case ERR_MAGBARO_NO_DATA:
        return "MAGBARO_NO_DATA";
    case ERR_MAGBARO_NO_CALIB:
        return "MAGBARO_NO_CALIB";
    case ERR_TOF_INIT_FAIL:
        return "TOF_INIT_FAIL";
    case ERR_TOF_NO_DATA:
        return "TOF_NO_DATA";
    case ERR_FLOW_INIT_FAIL:
        return "FLOW_INIT_FAIL";
    case ERR_FLOW_NO_DATA:
        return "FLOW_NO_DATA";
    case ERR_IMU_NO_CALIB:
        return "IMU_NO_CALIB";
    case ERR_FFT_STALLED:
        return "FFT_STALLED";
    }
    return "UNKNOWN";
}

void SystemStatus::dumpErrors()
{
    const uint32_t e = errors();
    if (e == 0)
    {
#if defined(SIL_HOST)
        std::fprintf(stderr, "[STATUS] state=%s errors=NONE\n", stateStr());
#else
        Serial.printf("[STATUS] state=%s errors=NONE\n", stateStr());
#endif
        return;
    }

#if defined(SIL_HOST)
    std::fprintf(stderr, "[STATUS] state=%s errors=0x%08X\n", stateStr(), e);
#else
    Serial.printf("[STATUS] state=%s errors=0x%08X\n", stateStr(), e);
#endif

    for (int i = 0; i < 32; ++i)
    {
        const uint32_t bit = 1u << i;
        if ((e & bit) == 0)
            continue;
#if defined(SIL_HOST)
        std::fprintf(stderr, "  - %s\n", errorStr((SystemError)bit));
#else
        Serial.printf("  - %s\n", errorStr((SystemError)bit));
#endif
    }
}