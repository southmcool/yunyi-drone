#pragma once
#include <Arduino.h>
#include "util/littlefs_tool.h"
#include <LittleFS.h>
#include "network/wifi_connect.h"
#include "network/wifi_transport.h"
#include "config/wifi_config.h"

class MainApplication
{
public:
    MainApplication(); // 构造函数声明
    void initialize(); // 初始化方法声明
    // 主循环更新
    void update();
    static void onWiFiConnectedStatic(bool connected);
    static void onProgressStatic(int progress, int total);
    static void onStatusStatic(const String &status);

private:
    bool loadPIDConfigFromFile();

    void applyPIDConfig();
    void loadPIDConfig();
    void _initHardware();
    void _startServices();
    void _handleConnectionError();
    // ✅ 添加实例指针用于静态函数访问
    static MainApplication *instance;

    // 原有的成员函数保持不变
    void onWiFiConnected(bool connected);
    void onProgress(int progress, int total);
    void onStatus(const String &status);

    bool isArmed = false;
    uint32_t lastCommandTime = 0;

    void emergencyStop();

    // PID 配置结构体
    struct PIDConfig
    {
        float roll_kp, roll_ki, roll_kd;
        float pitch_kp, pitch_ki, pitch_kd;
        float yaw_kp, yaw_ki, yaw_kd;
        bool yaw_enabled;
    };

    PIDConfig currentPIDConfig;
};