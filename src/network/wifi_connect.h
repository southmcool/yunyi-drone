#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include <WiFi.h>
#include <Arduino.h>
#include <esp_wifi.h>        // 🆕 添加这个
#include <esp_wifi_types.h>  // 🆕 添加这个

class WiFiConnector
{
public:
    WiFiConnector();

    bool connect(const char *ssid, const char *password, uint32_t timeout = 30000);
    bool isConnected();
    void disconnect();
    String getIPAddress();
    int getRSSI();
    void diagnoseWiFi();

    void setConnectionCallback(void (*callback)(bool));

private:
    void (*_connectionCallback)(bool) = nullptr;
    unsigned long _connectStartTime;

    void _WiFiEvent(WiFiEvent_t event);
};

extern WiFiConnector wifiConnector;

#endif // WIFI_CONNECT_H