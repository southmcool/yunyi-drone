#include "wifi_connect.h"
WiFiConnector wifiConnector;
WiFiConnector::WiFiConnector()
{
}

bool WiFiConnector::connect(const char *ssid, const char *password, uint32_t timeout)
{

    Serial.printf("WiFiConnector1 connect ssid=%s password=%s \n", ssid, password);
    // 先断开之前的连接
    // WiFi.disconnect(true);
    delay(1000);
    WiFi.onEvent([this](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
                    Serial.println("WiFiConnector1 onEvent");
                     this->_WiFiEvent(event); });

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    _connectStartTime = millis();

    // 等待连接完成或超时
    while (!isConnected() && (millis() - _connectStartTime < timeout))
    {
        // 这里只等待，事件回调会设置 _isConnecting = false
        delay(100);

        // 可选：打印等待状态
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 2000)
        {
            lastPrint = millis();
            Serial.printf("等待连接... 已等待 %lu ms\n", millis() - _connectStartTime);
        }
    }

    if (isConnected())
    {
        Serial.println("✅ WiFi连接成功 (通过事件回调)");
        return true;
    }
    else
    {
        Serial.println("❌ WiFi连接超时");
        WiFi.disconnect(true);
        return false;
    }
}

// 🆕 详细的WiFi诊断
void WiFiConnector::diagnoseWiFi()
{
    Serial.println("\n═══════════════════════════════════════");
    Serial.println("🔍 WiFi 详细诊断");
    Serial.println("═══════════════════════════════════════");

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("❌ WiFi未连接！");
        return;
    }

    // 基本信息
    Serial.printf("📡 SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("📡 RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("📡 MAC: %s\n", WiFi.BSSIDstr().c_str());
    Serial.printf("📡 信道: %d\n", WiFi.channel());
    Serial.printf("📡 IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("📡 网关: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("📡 DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("📡 子网掩码: %s\n", WiFi.subnetMask().toString().c_str());

    // 🆕 检查WiFi协议和模式
    wifi_mode_t mode;
    esp_wifi_get_mode(&mode);
    Serial.printf("📡 模式: %d (1=STA, 2=AP, 3=AP+STA)\n", mode);

    uint8_t protocol;
    esp_wifi_get_protocol(WIFI_IF_STA, &protocol);
    Serial.printf("📡 协议: ");
    if (protocol & WIFI_PROTOCOL_11B)
        Serial.printf("11b ");
    if (protocol & WIFI_PROTOCOL_11G)
        Serial.printf("11g ");
    if (protocol & WIFI_PROTOCOL_11N)
        Serial.printf("11n ");
    if (protocol == 0)
        Serial.printf("未知");
    Serial.println();

    // 🆕 检查PHY模式（这个很关键！）
    wifi_phy_mode_t phyMode;
    esp_wifi_sta_get_negotiated_phymode(&phyMode);
    Serial.printf("📡 PHY模式: ");
    switch (phyMode)
    {
    case WIFI_PHY_MODE_LR:
        Serial.println("LR (长距离/低速!)");
        break;
    case WIFI_PHY_MODE_11B:
        Serial.println("802.11b (最大11Mbps)");
        break;
    case WIFI_PHY_MODE_11G:
        Serial.println("802.11g (最大54Mbps)");
        break;
    case WIFI_PHY_MODE_HT20:
        Serial.println("802.11n HT20");
        break;
    case WIFI_PHY_MODE_HT40:
        Serial.println("802.11n HT40");
        break;
    default:
        Serial.printf("未知 (%d)\n", phyMode);
        break;
    }

    if (phyMode == WIFI_PHY_MODE_LR || phyMode == WIFI_PHY_MODE_11B)
    {
        Serial.println("⚠️⚠️⚠️ WiFi工作在低速模式！这是速度慢的原因！");
    }

    // 🆕 测试到网关的延迟
    IPAddress gateway = WiFi.gatewayIP();
    Serial.printf("\n📡 Ping网关 %s:\n", gateway.toString().c_str());

    for (int i = 0; i < 5; i++)
    {
        uint32_t startTime = micros();
        WiFiClient client;
        if (client.connect(gateway, 80, 500))
        {
            uint32_t latency = (micros() - startTime) / 1000;
            Serial.printf("  第%d次: %lums\n", i + 1, latency);
            client.stop();
        }
        else
        {
            Serial.printf("  第%d次: 超时\n", i + 1);
        }
        delay(50);
    }

    Serial.println("═══════════════════════════════════════\n");
}

bool WiFiConnector::isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

void WiFiConnector::disconnect()
{
    WiFi.disconnect();
    if (_connectionCallback)
    {
        _connectionCallback(false);
    }
}

String WiFiConnector::getIPAddress()
{
    return WiFi.localIP().toString();
}

int WiFiConnector::getRSSI()
{
    return WiFi.RSSI();
}

void WiFiConnector::setConnectionCallback(void (*callback)(bool))
{
    _connectionCallback = callback;
}

void WiFiConnector::_WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("[WiFi] Station connected");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.println("[WiFi] Got IP address");
        if (_connectionCallback)
        {
            _connectionCallback(true);
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("[WiFi] Station disconnected");
        if (_connectionCallback)
        {
            _connectionCallback(false);
        }
        break;
    default:
        Serial.printf("[_WiFiEvent_default] %d", event);
        break;
    }
}