#include "data_send.h"

packet_data getStabilizerData(const stabilizer_log_entry_t &data)
{

    // 使用更紧凑的数据格式

    static StabilizerPacket packet;

    // 填充数据
    packet.header[0] = 0xAA;
    packet.header[1] = 0xBB;
    packet.header[2] = 0x01;
    packet.timestamp = data.timestamp_us;
    packet.loop_iteration = data.loop_iteration;

    // 转换传感器数据
    packet.gyro[0] = (int16_t)(data.sensor_raw.gyro.x * 10.0f);
    packet.gyro[1] = (int16_t)(data.sensor_raw.gyro.y * 10.0f);
    packet.gyro[2] = (int16_t)(data.sensor_raw.gyro.z * 10.0f);

    packet.accel[0] = (int16_t)(data.sensor_raw.acc.x * 100.0f);
    packet.accel[1] = (int16_t)(data.sensor_raw.acc.y * 100.0f);
    packet.accel[2] = (int16_t)(data.sensor_raw.acc.z * 100.0f);

    packet.mag[0] = data.mag.mag.x;
    packet.mag[1] = data.mag.mag.y;
    packet.mag[2] = data.mag.mag.z;

    packet.zv[0] = (int32_t)(data.state_estimated.vertical_velocity * 100.0f);
    packet.zv[1] = (int32_t)(data.state_estimated.altitude * 100.0f);

    packet.position[0] = (int16_t)(data.state_estimated.velocity.x * 100.0f);
    packet.position[1] = (int16_t)(data.state_estimated.velocity.y * 100.0f);
    packet.position[2] = (int16_t)(data.state_estimated.position.x * 100.0f);
    packet.position[3] = (int16_t)(data.state_estimated.position.y * 100.0f);

    packet.pa = data.baro.pressure_pa;          // rad→0.1°，57.3*10
    packet.pa_altitude = data.baro.altitude_cm; // rad→0.1°，57.3*10

    packet.pos_sp[0] = 0; // rad→0.1°，57.3*10
    packet.pos_sp[1] = 0;

    packet.flow_act = 0;

    packet.estimate_quality = data.state_estimated.estimate_quality;
    packet.attitude[0] = (int16_t)(data.state_estimated.attitude_deg.roll * 10.0f);
    packet.attitude[1] = (int16_t)(data.state_estimated.attitude_deg.pitch * 10.0f);
    packet.attitude[2] = (int16_t)(data.state_estimated.attitude_deg.yaw * 10.0f);

    packet.setpoint_attitude[0] = (int16_t)(data.setpoint.attitude.roll * 10.0f);
    packet.setpoint_attitude[1] = (int16_t)(data.setpoint.attitude.pitch * 10.0f);
    packet.setpoint_attitude[2] = (int16_t)(data.setpoint.attitude.yaw * 10.0f);

    packet.thrust = (int16_t)(data.setpoint.thrust * 100.0f);
    packet.othrust = 0;

    packet.rates[0] = (int16_t)(data.state_estimated.attitude_rad.roll * 10.0f);
    packet.rates[1] = (int16_t)(data.state_estimated.attitude_rad.pitch * 10.0f);
    packet.rates[2] = (int16_t)(data.state_estimated.attitude_rad.yaw * 10.0f);

    packet.att_error[0] = 0;
    packet.att_error[1] = 0;
    packet.att_error[2] = 0;

    packet.rate_target[0] = 0;
    packet.rate_target[1] = 0;
    packet.rate_target[2] = 0;

    packet.rate_error[0] = 0;
    packet.rate_error[1] = 0;
    packet.rate_error[2] = 0;

    packet.torque_output[0] = 0;
    packet.torque_output[1] = 0;
    packet.torque_output[2] = 0;

    // 编码（飞控端）
    packet.att_pid_p[0] = 0;// 乘以1000转换为0.001精度
    packet.att_pid_p[1] = 0;// 乘以1000转换为0.001精度
    packet.att_pid_p[2] = 0;// 乘以1000转换为0.001精度

    packet.att_pid_i[0] = 0;// 乘以1000转换为0.001精度
    packet.att_pid_i[1] = 0;
    packet.att_pid_i[2] = 0;

    packet.att_pid_d[0] = 0;
    packet.att_pid_d[1] = 0;
    packet.att_pid_d[2] = 0;

    packet.rate_pid_p[0] =0; // 乘以1000转换为0.001精度
    packet.rate_pid_p[1] =0; // 乘以1000转换为0.001精度
    packet.rate_pid_p[2] =0; // 乘以1000转换为0.001精度

    packet.rate_pid_i[0] =0; // 乘以1000转换为0.001精度
    packet.rate_pid_i[1] =0;
    packet.rate_pid_i[2] =0;

    packet.rate_pid_d[0] =0;
    packet.rate_pid_d[1] =0;
    packet.rate_pid_d[2] =0;

    // 电机输出
    for (int i = 0; i < 4; i++)
    {
        packet.motor[i] = 0;
    }

    // 电机输出
    for (int i = 0; i < 4; i++)
    {
        packet.motor_pwd[i] =0;
    }

    // 系统状态
    packet.system_status = (data.is_armed ? 0x01 : 0x00);

    // 计算校验和
    packet.checksum = calculateChecksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 1);

    packet_data p;
    p.length = sizeof(packet);
    p.p = reinterpret_cast<uint8_t *>(&packet);

    // 发送数据包
    return p;
}

uint8_t calculateChecksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}