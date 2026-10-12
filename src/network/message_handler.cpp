#include "message_handler.h"
#include "core/telemetry/receiver.h"
#include "core/controller/attitude_controller.h"
// #include "drivers/pa_sensor/sensor.h"
#include "core/controller/altitude_controller.h"
#include "param/imu_calib_store.h"
// #include "core//estimator/ekf2_joint.h"

// extern Ekf2Joint ekf2_estimator;
extern AttitudeController attitudeController;
extern AltitudeController altitudeController;
MessageHandler messageHandler;

static void onIMUCalibrationComplete(bool success, void *userData)
{
    MessageHandler *handler = static_cast<MessageHandler *>(userData);
    if (handler)
    {
        handler->sendIMUCalibrationResponse(success);
    }
}

MessageHandler::MessageHandler()
{
    printf("MessageHandler:MessageHandlerInti");
    throttle = 0.0f;
    udpSender = nullptr;
    webSocketSender = nullptr;
}

void MessageHandler::sendIMUCalibrationResponse(bool success)
{
    StaticJsonDocument<200> doc;
    doc["type"] = "IMU_VERIFICATION";
    doc["flag"] = success;
    doc["message"] = success ? "IMU calibration completed successfully" : "IMU calibration failed";

    String jsonString;
    serializeJson(doc, jsonString);
    sendResponse(jsonString);

    Serial.printf("📤 IMU校准结果已发送: %s\n", jsonString.c_str());
}

// 🆕 发送响应（自动选择协议）
void MessageHandler::sendResponse(const String &response)
{
    // Serial.printf("📤 发送响应 (使用%s): %s\n",
    //               useWebSocketOnly ? "WebSocket" : "UDP",
    //               response.c_str());

    if (useWebSocketOnly && webSocketSender != nullptr)
    {
        webSocketSender(response);
    }
    else if (!useWebSocketOnly && udpSender != nullptr)
    {
        udpSender(response);
    }
    else if (webSocketSender != nullptr)
    {
        // 如果指定了WebSocket但不可用，尝试UDP
        webSocketSender(response);
    }
    else if (udpSender != nullptr)
    {
        udpSender(response);
    }
    else
    {
        Serial.printf("⚠️ 没有可用的消息发送器，无法发送响应: %s\n", response.c_str());
    }
}

void MessageHandler::processMessage(const String &message)
{
    // 1.消息校验
    // 🆕 添加空消息检查
    if (message.isEmpty())
    {
        Serial.println("⚠️ 收到空消息，跳过处理");
        return;
    }
    // 1.判断消息类别
    if (message.startsWith("{") && message.endsWith("}"))
    {
        // 判断是否JSON数据
        parseJSONCommand(message);
        return;
    }

    String direction;
    float strength = 0.0f;
    if (parseDirectionStrength(message, direction, strength))
    {
        handleControlCommand(direction, strength);
        return;
    }

    Serial.printf("📥 未知消息格式: %s\n", message.c_str());
}

void MessageHandler::parseJSONCommand(String jsonMessage)
{
    // 添加详细的调试信息
    Serial.printf("收到原始消息长度: %d\n", jsonMessage.length());
    Serial.printf("收到原始消息: ");
    Serial.println(jsonMessage);

    // 检查是否为空
    if (jsonMessage.length() == 0)
    {
        Serial.println("错误: 接收到的消息为空");
        return;
    }

    // 检查是否是有效的JSON开头
    if (jsonMessage.charAt(0) != '{')
    {
        Serial.println("错误: 消息不是有效的JSON格式（不以{开头）");
        return;
    }
    // 使用 ArduinoJson 库解析
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonMessage);

    if (error)
    {
        Serial.printf("JSON解析失败: %s\n", error.c_str());
        return;
    }
    if (doc.containsKey("MODELNAME") && doc["MODELNAME"] == "bicyclopid")
    {
        // 🆕 正确的方式提取数组数据
        JsonObject inner_loop = doc["inner_loop"];
        JsonObject outer_loop = doc["outer_loop"];
        JsonArray velocity = doc["velocity"];
        JsonArray location = doc["location"];

        // 提取内环参数
        JsonArray roll_inner = inner_loop["roll"];
        JsonArray pitch_inner = inner_loop["pitch"];
        JsonArray yaw_inner = inner_loop["yaw"];

        // 提取外环参数
        JsonArray roll_outer = outer_loop["roll"];
        JsonArray pitch_outer = outer_loop["pitch"];
        JsonArray yaw_outer = outer_loop["yaw"];

        // 🆕 创建增益数组
        float outer_gains[3][3] = {
            {roll_outer[0], roll_outer[1], roll_outer[2]},    // roll外环
            {pitch_outer[0], pitch_outer[1], pitch_outer[2]}, // pitch外环
            {yaw_outer[0], yaw_outer[1], yaw_outer[2]}        // yaw外环
        };

        float inner_gains[3][3] = {
            {roll_inner[0], roll_inner[1], roll_inner[2]},    // roll内环
            {pitch_inner[0], pitch_inner[1], pitch_inner[2]}, // pitch内环
            {yaw_inner[0], yaw_inner[1], yaw_inner[2]}        // yaw内环
        };

        float velocity_gains[3] = {velocity[0], velocity[1], velocity[2]};
        float location_gains[3] = {location[0], location[1], location[2]};

        // 设置PID增益
        attitudeController.setGains(outer_gains, inner_gains);
        altitudeController.setGains(location_gains, velocity_gains);
    }
    // 可以在这里添加其他JSON命令的处理
}

void MessageHandler::handleControlCommand(const String &direction, float strength)
{
    if (direction == "START")
    {
        arm();
        // ★R77c：GUI 的 START 自带当前油门（main_win.py unlock() 发 START&throttle），
        //   旧行为 arm() 内部 setThrust(0) 且滑条不动 GUI 永不重发 ⇒ 解锁后永远零油门。
        //   现接住随解锁送来的油门（安全口径：仅在 (0,1] 区间生效，0 维持原清零行为）。
        if (strength > 0.0f && strength <= 1.0f)
            setThrust(strength);
        Serial.printf("启动/解锁esc, throttle=%.3f\n", strength);
    }
    else if (direction == "STOP")
    {
        stop();
        Serial.println("停止");
    }
    else if (direction == "CALIBRATION_MAG")
    {
        // sensor.calibrationMag();
        Serial.println("校准磁力计");
    }
    else if (direction == "CALIBRATION_PA")
    {
        // sensor.calibrationPa();
        Serial.println("校准气压");
    }
    else if (direction == "CHECKAGM")
    {
        // sensor.checkagm();
        Serial.println("检查校准");
    }
    else if (direction == "CHECKASM")
    {
        // sensor.checkASM();
        Serial.println("检查安装");
    }
    else if (direction == "CHECKPAUCAL")
    {
        // sensor.checkPAUCAL();
        Serial.println("检查气压计");
    }
    else if (direction == "SETAUPA")
    {
        // sensor.setAuPa();
        Serial.println("设置自交易气压为0");
    }
    else if (direction == "syncFlag")
    {
        Serial.printf("同步BLDC: %f\n", strength);
    }
    else if (direction == "recordLogFlag")
    {
        Serial.printf("记录日志: %f\n", strength);
    }
    else if (direction == "TEST_M1")
    {
        Serial.println("测试电机1");
    }
    else if (direction == "TEST_M2")
    {
        Serial.println("测试电机2");
    }
    else if (direction == "TEST_M3")
    {
        Serial.println("测试电机3");
    }
    else if (direction == "TEST_M4")
    {
        Serial.println("测试电机4");
    }
    else if (direction == "TEST_M5")
    {
        Serial.println("测试电机5");
    }
    else if (direction == "ADJUST_FL")
    {
        Serial.printf("调整 FL: %f\n", strength);
    }
    else if (direction == "ADJUST_FR")
    {
        Serial.printf("调整 FR: %f\n", strength);
    }
    else if (direction == "ADJUST_BL")
    {
        Serial.printf("调整 BL: %f\n", strength);
    }
    else if (direction == "ADJUST_BR")
    {
        Serial.printf("调整 BR: %f\n", strength);
    }
    else if (direction == "ADJUST_CAT")
    {
        // adjustType response = xMotor.catADJUST();

        // // 构建JSON响应
        // StaticJsonDocument<200> doc;
        // doc["type"] = "ADJUST";
        // doc["FL"] = response.FL_ADJUST;
        // doc["FR"] = response.FR_ADJUST;
        // doc["BL"] = response.BL_ADJUST;
        // doc["BR"] = response.BR_ADJUST;
        // doc["syncFlag"] = catSyncBldc(); // 或者使用某个状态值

        // String jsonString;
        // serializeJson(doc, jsonString);

        // sendResponse(jsonString);
        // Serial.printf("📤 ADJUST_CAT 响应: %s\n", jsonString.c_str());
    }
    else if (direction == "PID_CAT")
    {
        StaticJsonDocument<200> doc = attitudeController.getPIDJson();
        String jsonString;
        serializeJson(doc, jsonString);

        sendResponse(jsonString);

        Serial.printf("📤 查看PID 响应: %s\n", jsonString.c_str());
    }
    else if (direction == "IMU_VERIFICATION")
    {
        Serial.printf("正在校验对准: %f\n", strength);
        ImuCalibStore::clear();
        // 注册回调（传入 this 指针）
        // setIMUCalibrationCallback(onIMUCalibrationComplete, this);

        // // 启动校准
        // bool started = checkIMUCalibrationTrigger();

        // if (started)
        // {
        //     // 立即返回开始响应
        //     StaticJsonDocument<200> startDoc;
        //     startDoc["type"] = "IMU_VERIFICATION";
        //     startDoc["status"] = "started";
        //     startDoc["message"] = "IMU calibration started, please wait...";

        //     String jsonString;
        //     serializeJson(startDoc, jsonString);
        //     sendResponse(jsonString);
        // }
        // else
        // {
        //     // 返回错误响应
        //     StaticJsonDocument<200> errorDoc;
        //     errorDoc["type"] = "IMU_VERIFICATION";
        //     errorDoc["flag"] = false;
        //     errorDoc["message"] = "Failed to start calibration";

        //     String jsonString;
        //     serializeJson(errorDoc, jsonString);
        //     sendResponse(jsonString);
        // }
    }

    else if (direction == "UP")
    {
        // 使用较小的增量，确保在 0-1 范围内
        throttle += strength * 0.01f; // 缩小增量
        throttle = constrain(throttle, 0.0f, 1.0f);
        setThrust(throttle);
        Serial.printf("11油门增加: %f, 当前: %f\n", strength * 0.01f, throttle);
    }
    else if (direction == "DOWN")
    {
        // 使用较小的减量，确保在 0-1 范围内
        throttle -= strength * 0.01f; // 缩小减量
        throttle = constrain(throttle, 0.0f, 1.0f);
        setThrust(throttle);
        Serial.printf("11油门减少: %f, 当前: %f\n", strength * 0.01f, throttle);
    }
    else if (direction == "ROLL")
    {
        setROLL(constrain(strength, -1.0f, 1.0f));
        Serial.printf("设置姿态翻滚: %f\n", strength);
    }
    else if (direction == "PITCH")
    {
        setPITCH(constrain(strength, -1.0f, 1.0f));
        Serial.printf("设置姿态俯仰: %f \n", strength);
    }
    else if (direction == "YWA")
    {
        setYAW(constrain(strength, -1.0f, 1.0f));
        Serial.printf("设置姿态横滚: %f \n", strength);
    }
    else if (direction == "HEIGTH")
    {
        int16_t height_cm = (int16_t)(constrain(strength, 0.0f, 3.0f) * 100.0f);
        setHeigth(height_cm);
        Serial.printf("设置高度: %d cm \n", height_cm);
    }
    else if (direction == "SET_POS_X")
    {
        // setPosX(strength);
    }
    else if (direction == "SET_POS_Y")
    {
        // setPosY(strength);
    }
    else if (direction == "SET_CONTROL_MODE")
    {
        int16_t CONTROL_MODE = (int16_t)(constrain(strength, 0.0f, 2.0f) * 1.0f);
        Serial.printf("设置控制模式: %d cm \n", CONTROL_MODE);
        if (!setControlMode(CONTROL_MODE))
        {
            Serial.println("设置控制模式: 失败");
        }
    }
    else if (direction == "SET_THROTTLE")
    {
        // 使用较小的减量，确保在 0-1 范围内
        throttle = constrain(strength, 0.0f, 1.0f);
        setThrust(throttle);
        Serial.printf("油门: %f, 当前: %f\n", strength * 0.01f, throttle);
    }
    else if (direction == "STABILIZER_LOG")
    {
        // 使用较小的减量，确保在 0-1 范围内
        // vSetUdpSend(strength);
        Serial.printf("STABILIZER_LOG: %f\n", strength);
    }
    else if (direction == "CLEAN_LOGFILE")
    {
        // 使用较小的减量，确保在 0-1 范围内
        // vcleanLogFile();
        Serial.printf("清除当前飞行日志: %f\n", strength);
    }
    else if (direction == "SERIAL_PRINT")
    {
        setSerialPrintSend(strength);
        Serial.printf("SERIAL_PRINT: %f\n", strength);
    }
    else if (direction == "SERIAL_LOG")
    {
        setSerialSend(strength);
        Serial.printf("SERIAL_LOG: %f\n", strength);
    }
    else if (direction == "CALIB_MODE")
    {
        // setCalibModeSend(strength);
        Serial.printf("CALIB_MODE: %f\n", strength);
    }
    else if (direction == "RESTART")
    {
        Serial.printf("重启\n");
        delay(100); // 给串口输出一点时间
        ESP.restart();
    }
    else if (direction == "RESET_ESTIMATOR")
    {
        Serial.printf("重置估计器\n");
        delay(100); // 给串口输出一点时间
        // ekf2_estimator.reset();
    }
}

bool MessageHandler::parseDirectionStrength(const String &input, String &direction, float &strength)
{
    if (input.isEmpty())
        return false;

    int ampPos = input.indexOf('&');
    if (ampPos == -1)
        return false;

    direction = input.substring(0, ampPos);
    String strengthStr = input.substring(ampPos + 1);

    char *endPtr;
    float value = strtof(strengthStr.c_str(), &endPtr);

    if (endPtr == strengthStr.c_str() || !std::isfinite(value))
    {
        return false;
    }

    strength = value;
    return true;
}