#include "main_app.h" // 包含对应的头文件

MainApplication *MainApplication::instance = nullptr;
MainApplication::MainApplication()
{
    // 构造函数实现（如果需要初始化成员变量）
    instance = this;
    Serial.println("MainApplication构造函数");
}

// ✅ 静态成员函数实现
void MainApplication::onWiFiConnectedStatic(bool connected)
{
    if (instance)
    {
        instance->onWiFiConnected(connected);
    }
}
// ✅ 静态成员函数实现
void MainApplication::onProgressStatic(int progress, int total)
{
    if (instance)
    {
        instance->onProgress(progress, total);
    }
}
// ✅ 静态成员函数实现
void MainApplication::onStatusStatic(const String &status)
{
    if (instance)
    {
        instance->onStatus(status);
    }
}

// WiFi 连接状态回调
void MainApplication::onWiFiConnected(bool connected)
{
    if (connected)
    {
        Serial.println("[WiFi] Connected to: " + String(WiFi.SSID()));
        Serial.println("[WiFi] IP address: " + WiFi.localIP().toString());
        // 等待网络稳定
        delay(2000);

        // // 初始化WiFi传输
        if (wifiTransport.initialize())
        {
            Serial.println("✓ WiFi传输初始化成功");
            Serial.println("Web服务器地址: http://" + WiFi.localIP().toString());
        }
        else
        {
            Serial.println("✗ WiFi传输初始化失败");
        }
        // wifiConnector.diagnoseWiFi();
    }
    else
    {
        Serial.println("[WiFi] Disconnected");
    }
}

void MainApplication::initialize()
{
    Serial.println("MainApplication：：initialize执行");
    // 设置 WiFi 回调
    wifiConnector.setConnectionCallback(MainApplication::onWiFiConnectedStatic);
    wifiConnector.connect(WIFI_SSID, WIFI_PASSWORD, 50000);
    // 初始化文件系统
    if (!LittleFSHelper.initialize("littlefs", "/littlefs"))
    {
        Serial.println("❌ 文件系统初始化失败");
    }

    Serial.println("主应用初始化完成");
}

// 主循环更新
void MainApplication::update()
{

    // 处理WebSocket事件
    // wifiTransport.loop();
    // 保持稳定的更新频率
}

void MainApplication::_initHardware()
{
    // 具体硬件初始化代码
    // 例如：pinMode(LED_PIN, OUTPUT);
}

void MainApplication::_startServices()
{
    // 启动其他服务
    // 例如：_sensorService.begin();
}

void MainApplication::_handleConnectionError()
{
    // 错误处理逻辑
    // 例如：闪烁LED报警
}
