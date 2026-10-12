#include "task_scheduler.h"
#include "util/data_send.h"
#include "core/telemetry/telemetry_data.h"
#include "network/wifi_transport.h"  // 引入 WiFiTransport 头文件
#include "core/system/shared_data.h" // motor_throttle_update
#include "core/telemetry/receiver.h"

extern bool serial_send_flag;  // 应由安全状态机统一管理
extern bool serial_print_flag; // 应由安全状态机统一管理
extern bool system_armed;      // 应由安全状态机统一管理
uint8_t net_send_counter = 0;  // 用于控制网络发送频率的计数器

void telemetry_task(void *pvParameters)
{

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
    uint16_t throttle[4] = {0, 0, 0, 0};
    Serial.println("📡 遥测任务启动 - 核心 " + String(xPortGetCoreID()));

    uint32_t ulSerialCounter = 0;
    for (;;)
    {

        motor_throttle_get_latest(throttle);
        // memcpy(stabilizer_data.motor_pwm, throttle, sizeof(throttle));
        // memcpy(stabilizer_data.motor_out, throttle, sizeof(throttle));
        stabilizer_data.is_armed = system_armed;
        if (serial_send_flag)
        {
            packet_data p = getStabilizerData(stabilizer_data);

            // // 使用非阻塞串口发送
            // Serial.write(p.p, p.length);

            // // 不要每次都 flush，定期 flush 即可
            // if (++ulSerialCounter % 20 == 0)
            // {
            //     Serial.flush();
            //     ulSerialCounter = 0;
            // }

            // 3. 🆕 新增：WebSocket 网络发送
            // 建议降频发送：每5帧发一次 (即 100Hz / 5 = 20Hz)，避免占用过多 WiFi 带宽
            if (++net_send_counter >= 5)
            {
                net_send_counter = 0;

                // 检查是否有客户端连接
                if (wifiTransport.isDeviceConnected())
                {
                    // 直接调用你的 sendBinary 发送原始二进制包
                    wifiTransport.sendBinary((const char *)p.p, p.length);
                }
            }
        }
        if (serial_print_flag)
        {
            const state_t &st = stabilizer_data.state_estimated;
            const ImuData &sr = stabilizer_data.sensor_raw;
            const setpoint_t &sp = stabilizer_data.setpoint;

            // ---- NED → UP ----
            const float altitude_up = -st.position.z; // 高度（向上为正）[m]
            const float vv_up = -st.velocity.z;       // 垂直速度（向上为正）[m/s]

            // ---- 世界系加速度（Gs → m/s²）----
            constexpr float kG = 9.80665f;
            const float aw_x = st.acc.x * kG;
            const float aw_y = st.acc.y * kG;
            const float aw_z = st.acc.z;

            // ---- 机体系加速度（IMU chain 输出，g）----
            const float ab_x = sr.acc.x;
            const float ab_y = sr.acc.y;
            const float ab_z = sr.acc.z;

            // ---- 姿态 ----
            const float roll = st.attitude_deg.roll;
            const float pitch = st.attitude_deg.pitch;
            const float yaw = st.attitude_deg.yaw;

            // ---- stable 判据（新 state 无此字段，就地算）----
            const bool stable =
                (fabsf(roll) < 10.0f) &&
                (fabsf(pitch) < 10.0f) &&
                (fabsf(vv_up) < 0.5f);

            Serial.printf(
                "alt=%.2fm vv=%+.2fm/s | "
                "att_b=(%+.3f,%+.3f,%+.3f)g  "
                "att_w_NED=(%+.2f,%+.2f,%+.2f)m/s2 | "
                "roll/pitch/yaw=%+.2f/%+.2f/%+.2f° | "
                "vel_xy=(%+.2f,%+.2f)m/s  pos_xy=(%+.2f,%+.2f)m | "
                "baro=%d cm / %u Pa  | "
                "armed=%d  stable=%d\n",

                altitude_up, vv_up,

                ab_x, ab_y, ab_z,
                aw_x, aw_y, aw_z,
                roll, pitch, yaw,
                st.velocity.x, st.velocity.y,
                st.position.x, st.position.y,
                stabilizer_data.baro.altitude_cm,
                (unsigned)stabilizer_data.baro.pressure_pa,
                (int)stabilizer_data.is_armed,
                (int)stable);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}