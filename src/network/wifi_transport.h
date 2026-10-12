#ifndef WIFI_TRANSPORT_H
#define WIFI_TRANSPORT_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Arduino.h>
#include "util/littlefs_tool.h"
#include "util/fast_file_server.h"

// 在 WiFiTransport.h 中添加
#include <vector>

class WiFiTransport
{
public:
    WiFiTransport();

    bool initialize();

    bool sendMessage(const String &message);
    bool isDeviceConnected();
    bool sendText(const char *data, size_t length); // 🆕 添加
    void loop();
    bool sendBinary(const char *data, size_t length);

private:
    FastFileServer fastFileServer;
    std::vector<AsyncWebSocketClient *> _clients;
    void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                          AwsEventType type, void *arg, uint8_t *data, size_t len);
    bool parseDirectionStrength(const String &input, String &direction, float &strength);
    void handleControlCommand(const String &direction, float strength);
    void parseJSONCommand(String jsonMessage);
    void handlePIDSetting(JsonDocument &doc);
    AsyncWebServer *server;
    AsyncWebSocket *webSocket;
    uint16_t _connectedClients = 0;
    bool deviceConnected;
    bool pageFlag;
    float throttle;
    float pitchStrength;
    float rollStrength;

    static const uint16_t WEB_PORT = 80;
};

extern WiFiTransport wifiTransport;

#endif