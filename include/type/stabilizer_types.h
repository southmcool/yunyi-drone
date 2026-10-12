/**
 * @title: 稳定器数据结构
 * @author: vaie
 * @date: 2026-10-11
 * @description:
 */
#pragma once
#include "imu_types.h"

// ---------------------------------------------------------------------------
//  磁力计采样（模块原始 LSB 值，未换算成 gauss）
//    灵敏度由模块手册给出；上层使用时自行换算。
// ---------------------------------------------------------------------------
struct mag_s
{
    Axis3i16 mag;          // LSB（模块原始 int16）
    float temperature_c;   // °C（模块内部温度，= 原始 int16 / 100）
    uint64_t timestamp_us; // µs（esp_timer_get_time）
};

// ---------------------------------------------------------------------------
//  气压计采样（已扣固件侧校准偏移 + 一阶低通）
//    pressure_pa  = 原始 Pa   − offset_p
//    altitude_cm  = 原始 cm   − offset_a，再经一阶低通
// ---------------------------------------------------------------------------
struct baro_s
{
    int32_t pressure_pa;   // Pa
    int32_t altitude_cm;   // cm
    float temperature_c;   // °C
    uint64_t timestamp_us; // µs
};

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/** Attitude in euler angle form */
typedef struct attitude_s
{
    uint64_t timestamp; // Timestamp when the data was computed

    float roll;
    float pitch;
    float yaw;
} attitude_t;

/* Orientation as a quaternion */
typedef struct quaternion_s
{
    uint64_t timestamp;

    union
    {
        struct
        {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

/* x,y,z vector */
struct vec3_s
{
    uint64_t timestamp; // Timestamp when the data was computed

    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct state_s
{
    attitude_t attitude_deg;         // deg, ts = µs
    attitude_t attitude_rad;         // rad, ts = µs
    Axis3f gyro_rate;                // 机体系角速率（rad/s），直通自 IMU gyro
    quaternion_t attitudeQuaternion; // 单位四元数, ts = µs
    point_t position;                // m (NED), ts = µs
    velocity_t velocity;             // m/s (NED), ts = µs
    acc_t acc;                       // Gs, 机体轴非重力分量（Crazyflie 语义）

    // ---- ★ 辅助：向上为正（控制器 / 遥测直接用）----
    float altitude;          // [m]    = -position.z
    float vertical_velocity; // [m/s]  = -velocity.z

    // ★ 新增：估计质量 [0, 100]；当前恒 100
    uint8_t estimate_quality;
} state_t;

typedef struct control_s
{
    float roll;   // 横滚力矩分量（无量纲）
    float pitch;  // 俯仰力矩分量（无量纲）
    float yaw;    // 偏航力矩分量（无量纲）
    float thrust; // 归一化推力 [0, 1]
} control_t;

typedef enum mode_e
{
    modeDisable = 0,
    modeAbs,
    modeVelocity
} stab_mode_t;

typedef enum flight_mode_e
{

    manual = 0,
    position_hold,
    stabilize
} flight_mode_t;

typedef struct setpoint_s
{
    uint32_t timestamp;

    attitude_t attitude;     // deg
    attitude_t attitudeRate; // rad /s
    quaternion_t attitudeQuaternion;
    float thrust;
    point_t position;    // m
    velocity_t velocity; // m/s
    acc_t acceleration;  // m/s^2
    bool velocity_body;  // true if velocity is given in body frame; false if velocity is given in world frame
    struct
    {
        stab_mode_t x;
        stab_mode_t y;
        stab_mode_t z;
        stab_mode_t roll;
        stab_mode_t pitch;
        stab_mode_t yaw;
        stab_mode_t quat;
    } mode;
    flight_mode_t flight_mode;
} setpoint_t;

/** Setpoint for althold */
typedef struct setpointZ_s
{
    float z;
    bool isUpdate; // True = small update of setpoint, false = completely new
} setpointZ_t;

struct motor_mix_t
{
    float motor[4]; // [0, 1]
    uint64_t timestamp;
};

/** Flow measurement**/
typedef struct flowMeasurement_s
{
    uint32_t timestamp;
    union
    {
        struct
        {
            float dpixelx; // Accumulated pixel count x
            float dpixely; // Accumulated pixel count y
        };
        float dpixel[2]; // Accumulated pixel count
    };
    float stdDevX; // Measurement standard deviation
    float stdDevY; // Measurement standard deviation
    float dt;      // Time during which pixels were accumulated
} flowMeasurement_t;

/** TOF measurement**/
typedef struct tofMeasurement_s
{
    uint32_t timestamp;
    float distance;
    float stdDev;
} tofMeasurement_t;

/** Mag measurement（EKF 输入格式）**/
typedef struct magMeasurement_s
{
    uint64_t timestamp;
    Axis3f mag_body; // 机体系磁场（已扣硬磁偏移，LSB 即可——yaw 只看方向）
    float stdDev;    // 观测噪声（LSB）
} magMeasurement_t;

typedef struct
{
    // 时间信息
    uint32_t timestamp_us;
    uint32_t loop_iteration;

    // 传感器原始数据
    ImuData sensor_raw;

    // 估计器状态
    state_t state_estimated;

    // 设定值
    setpoint_t setpoint;

    baro_s baro;
    mag_s mag;

    // 系统状态
    bool is_armed;
    bool system_healthy;

} stabilizer_log_entry_t;