#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#include <Arduino.h>
#include "type/stabilizer_types.h"

struct StabilizerPacket
{
    uint8_t header[3];       // 0xAA, 0xBB, 类型
    uint32_t timestamp;      // 时间戳
    uint32_t loop_iteration; // 循环计数
    int16_t gyro[3];         // 陀螺仪 (0.1 dps精度)
    int16_t accel[3];        // 加速度计 (0.01g精度)
    int16_t mag[3];          // 磁力计
    int32_t zv[2];           // 气压计
    int16_t position[4];     // vx,vy,px,py
    uint32_t pa;             // 海拔
    uint32_t pa_altitude;    // 海拔高度
    int16_t pos_sp[2];
    uint8_t flow_act;             // 光流活跃
    uint8_t estimate_quality;     // 质量
    int16_t attitude[3];          // 姿态角 (0.1度精度)
    int16_t setpoint_attitude[3]; // 设定值 (0.1度精度)
    int16_t thrust;               // 推力(0.01度精度)设定推力
    int16_t othrust;              // 推力(0.01度精度)最终推力
    int16_t rates[3];             // 角速度 (0.1 dps精度)
    int16_t att_error[3];         // roll, pitch, yaw误差(0.1 dps精度)
    int16_t rate_target[3];       // 角速度目标值(0.1 dps精度)
    int16_t rate_error[3];        // 角速度误差(0.1 dps精度)
    int16_t torque_output[3];     // 力矩输出
    // PID内部状态
    int16_t att_pid_p[3]; // 角度环P分量(0.001精度)
    int16_t att_pid_i[3]; // 角度环I分量(0.001精度)
    int16_t att_pid_d[3]; // 角度环D分量(0.001精度)

    int16_t rate_pid_p[3]; // 角速度环P分量(0.001精度)
    int16_t rate_pid_i[3]; // 角速度环I分量(0.001精度)
    int16_t rate_pid_d[3]; // 角速度环D分量(0.001精度)
    uint16_t motor[4];     // 电机输出 (0-1000)
    uint16_t motor_pwd[4]; // 电机输出 (0-1000)
    uint8_t system_status; // 系统状态
    uint8_t checksum;      // 校验和
} __attribute__((packed));

struct packet_data
{
    uint8_t *p;
    size_t length;
};

packet_data getStabilizerData(const stabilizer_log_entry_t &data);

uint8_t calculateChecksum(const uint8_t *data, size_t length);

#endif // SENSOR_DATA_H