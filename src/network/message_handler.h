#ifndef MESSAGE_HANDLER_H
#define MESSAGE_HANDLER_H

#include <Arduino.h>

// 静态成员函数作为回调

// 消息处理器
class MessageHandler
{
public:
    void sendIMUCalibrationResponse(bool success);
    MessageHandler();
    // 处理消息
    void processMessage(const String &message);
    // 🆕 设置UDP消息发送回调
    void setUdpSender(std::function<bool(const String &)> sender)
    {
        udpSender = sender;
    }

    // 🆕 设置WebSocket消息发送回调
    void setWebSocketSender(std::function<void(const String &)> sender)
    {
        webSocketSender = sender;
    }

    // 🆕 选择发送方式
    void setSendMethod(bool useWebSocket)
    {
        useWebSocketOnly = useWebSocket;
    }
    void sendResponse(const String &response);

private:
    bool parseDirectionStrength(const String &input, String &direction, float &strength);
    void handleControlCommand(const String &direction, float strength);
    void parseJSONCommand(String jsonMessage);
    // 🆕 发送响应（自动选择协议）

public:
private:
    float throttle;
    std::function<bool(const String &)> udpSender; // UDP发送回调
    std::function<void(const String &)> webSocketSender;
    bool useWebSocketOnly = true; // 默认使用WebSocket
};

extern MessageHandler messageHandler;

#endif // MESSAGE_HANDLER_H