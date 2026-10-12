#include "wifi_transport.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "message_handler.h"

WiFiTransport wifiTransport;
extern MessageHandler messageHandler;
WiFiTransport::WiFiTransport()
    : server(nullptr), webSocket(nullptr),
      deviceConnected(false), pageFlag(false), throttle(0.0f),
      pitchStrength(0.0f), rollStrength(0.0f)
{
}

bool WiFiTransport::initialize()
{
    // 先检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("❌ 错误: WiFi未连接，无法初始化WebSocket服务器");
        return false;
    }

    // 🆕 设置MessageHandler的WebSocket发送回调
    messageHandler.setWebSocketSender([this](const String &msg)
                                      { return this->sendMessage(msg); });

    // 设置使用WebSocket
    messageHandler.setSendMethod(true);

    Serial.print("✅ WiFi状态: 已连接, IP: ");
    Serial.println(WiFi.localIP());

    // 初始化状态LED

    Serial.println("🚀 初始化WiFi传输控制器...");

    // 确保网络堆栈就绪
    delay(1000);

    // 创建Web服务器
    server = new AsyncWebServer(WEB_PORT);

    // 创建WebSocket
    webSocket = new AsyncWebSocket("/ws");
#ifdef WS_MAX_QUEUED_MESSAGES
    // 部分 fork 版本支持自定义 allocator
#endif
    // 设置WebSocket事件处理
    webSocket->onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client,
                              AwsEventType type, void *arg, uint8_t *data, size_t len)
                       { this->onWebSocketEvent(server, client, type, arg, data, len); });

    // 添加WebSocket到服务器
    server->addHandler(webSocket);

    // 设置简单的根路径响应
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        String html = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>耘翼无人机管理平台</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
    <h1>Drone Control Ready</h1>
    <p>Use WebSocket client to connect to /ws</p>
    <p>IP: )rawliteral" + WiFi.localIP().toString() + R"rawliteral(</p>
    <a href="/files" class="refresh"> 文件管理</a>
</body>
</html>
        )rawliteral";
        request->send(200, "text/html", html); });

    // 修改 /files 路由
    server->on("/files", HTTP_GET, [this](AsyncWebServerRequest *request) // ✅ 添加 this
               {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>飞控文件管理</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body{font-family:Arial;margin:20px;background:#1a1a2e;color:#eee;}
        h2{color:#00d4ff;}
        a{display:block;padding:12px;margin:8px 0;background:#16213e;color:#00d4ff;
          text-decoration:none;border-radius:8px;border:1px solid #0f3460;}
        a:hover{background:#0f3460;}
        .size{color:#888;font-size:0.9em;}
        .dir{background:#0f3460;}
        .back{background:#e94560;color:white;text-align:center;}
        .refresh{background:#00d4ff;color:#1a1a2e;text-align:center;}
        .progress-bar {
            width: 100%;
            height: 10px;
            background: #1a1a2e;
            border-radius: 5px;
            overflow: hidden;
            margin-bottom: 8px;
        }
        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #00d4ff, #00ff88);
            border-radius: 5px;
            width: 0%;
            transition: width 0.3s;
        }
    </style>
</head>
<body>
    <h2>📁 飞控文件管理</h2>
    <a href="/files" class="refresh">🔄 刷新列表</a>
)rawliteral";

    // 文件系统空间信息
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    size_t free = total - used;
    float percent = (float)used / total * 100.0f;
    
    String totalStr, usedStr, freeStr;
    if (total < 1048576) {
        totalStr = String(total / 1024.0f, 1) + " KB";
        usedStr = String(used / 1024.0f, 1) + " KB";
        freeStr = String(free / 1024.0f, 1) + " KB";
    } else {
        totalStr = String(total / 1048576.0f, 2) + " MB";
        usedStr = String(used / 1048576.0f, 2) + " MB";
        freeStr = String(free / 1048576.0f, 2) + " MB";
    }
    
    html += "<div class='fsinfo'>";
    html += "💾 总空间: " + totalStr + " | 已用: " + usedStr + " | 剩余: " + freeStr;
    html += "<div class='bar'><div class='bar-fill' style='width:" + String(percent) + "%'></div></div>";
    html += "<small>" + String(percent, 1) + "% 已使用</small>";
    html += "</div>";
    
    // 获取当前路径
    String currentPath = "/";
    String parentPath = "";
    if (request->hasParam("dir"))
    {
        currentPath = "/" + request->getParam("dir")->value();
        if (!currentPath.endsWith("/")) currentPath += "/";
        
        String dirParam = request->getParam("dir")->value();
        int lastSlash = dirParam.lastIndexOf('/');
        if (lastSlash > 0)
        {
            parentPath = dirParam.substring(0, lastSlash);
        }
        
        html += "<a href='/files" + (parentPath.length() > 0 ? "?dir=" + parentPath : "") + "' class='back'>📂 返回上级目录</a>";
        html += "<p>📂 当前: " + currentPath + "</p>";
    }
    
    File root = LittleFS.open(currentPath);
    if (root && root.isDirectory())
    {
        // 先列出文件夹
        File file = root.openNextFile();
        while (file)
        {
            if (file.isDirectory())
            {
                String fullPath = String(file.name());
                String dirName = fullPath;
                if (dirName.startsWith(currentPath)) dirName = dirName.substring(currentPath.length());
                if (dirName.endsWith("/")) dirName = dirName.substring(0, dirName.length() - 1);
                
                String dirParam = fullPath;
                if (dirParam.startsWith("/")) dirParam = dirParam.substring(1);
                
                html += "<a href='/files?dir=" + dirParam + "' class='dir'>";
                html += "📁 " + dirName + "/";
                html += "</a>";
            }
            file.close();
            file = root.openNextFile();
        }
        root.close();
        
        // 重新扫描列出文件
        root = LittleFS.open(currentPath);
        file = root.openNextFile();
        while (file)
        {
            if (!file.isDirectory())
            {
                String fullPath = String(file.name());
                String displayName = fullPath;
                if (displayName.startsWith(currentPath)) displayName = displayName.substring(currentPath.length());
                
                String downloadPath = currentPath + fullPath;
                if (downloadPath.startsWith("/")) downloadPath = downloadPath.substring(1);
                
                size_t fileSize = file.size();
                String sizeStr;
                if (fileSize < 1024) sizeStr = String(fileSize) + " B";
                else if (fileSize < 1048576) sizeStr = String(fileSize / 1024.0f, 1) + " KB";
                else sizeStr = String(fileSize / 1048576.0f, 2) + " MB";
                
                html += "<a href='/download?file=" + downloadPath + "'>";
                html += "📄 " + displayName;
                html += " <span class='size'>(" + sizeStr + ")</span>";
                html += "</a>";
            }
            file.close();
            file = root.openNextFile();
        }
        root.close();
    }
    else
    {
        html += "<p>无法访问: " + currentPath + "</p>";
    }
    
    // 🆕 预加载BBL文件到PSRAM缓存
    String loadedFile = this->fastFileServer.preloadBBLFile("/");
    if (!loadedFile.isEmpty())
    {
        html += "<p>📦 BBL日志已预加载到缓存: " + loadedFile + "</p>";
    }
    
    // 🆕 显示缓存统计
    html += "<p>" + this->fastFileServer.getStats() + "</p>";
    
    html += R"rawliteral(
    <a href="/download_log" class="refresh">📥 一键下载BBL日志(快速)</a>
    <a href="/clear_cache" class="back">🗑️ 清除缓存</a>
    <a href="/" class="back">🏠 返回首页</a>
</body>
</html>
)rawliteral";
    
    request->send(200, "text/html", html); });

    // 修改 /download_log 路由 - 使用FastFileServer
    server->on("/download_log", HTTP_GET, [this](AsyncWebServerRequest *request)
               {
    // ★R77 飞行日志（CSV）优先：存在则直接下载
    if (LittleFS.exists("/flight.csv"))
    {
        Serial.println("📥 下载飞行日志 flight.csv");
        if (this->fastFileServer.loadToPSRAM("/flight.csv"))
        {
            if (this->fastFileServer.sendFromCache(request))
            {
                return;
            }
        }
        request->send(LittleFS, "/flight.csv", "application/octet-stream", true);
        return;
    }

    // 🆕 先尝试从缓存发送
    if (this->fastFileServer.hasCache())
    {
        if (this->fastFileServer.sendFromCache(request))
        {
            return;
        }
    }
    
    // 扫描查找BBL文件
    File root = LittleFS.open("/");
    if (!root)
    {
        request->send(500, "text/plain", "无法访问文件系统");
        return;
    }
    
    String bblFile = "";
    size_t bblSize = 0;
    File file = root.openNextFile();
    
    while (file)
    {
        String name = String(file.name());
        if (name.endsWith(".bbl") || name.endsWith(".BBL"))
        {
            bblFile = name;
            bblSize = file.size();
            file.close();
            break;
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    
    if (bblFile.isEmpty())
    {
        request->send(404, "text/plain", "未找到BBL日志文件");
        return;
    }
    
    // 🆕 加载到PSRAM并发送
    if (this->fastFileServer.loadToPSRAM(bblFile))
    {
        if (this->fastFileServer.sendFromCache(request))
        {
            return;
        }
    }
    
    // 回退到直接发送
    Serial.printf("📥 一键下载日志: %s (%u 字节)\n", bblFile.c_str(), bblSize);
    request->send(LittleFS, bblFile, "application/octet-stream", true); });

    // 🆕 添加清除缓存路由
    server->on("/clear_cache", HTTP_GET, [this](AsyncWebServerRequest *request)
               {
    this->fastFileServer.clearCache();
    request->send(200, "text/html", 
        "<html><body><h2>✅ 缓存已清除</h2><a href='/files'>返回文件列表</a></body></html>"); });

    // 修复 /download 路由中的file对象问题
    server->on("/download", HTTP_GET, [this](AsyncWebServerRequest *request)
               {
    if (!request->hasParam("file"))
    {
        request->send(400, "text/plain", "缺少文件名参数");
        return;
    }
    
    String fileName = "/" + request->getParam("file")->value();

    // 尝试从PSRAM缓存发送
    if (this->fastFileServer.isCached(fileName))
    {
        if (this->fastFileServer.sendFromCache(request))
        {
            return;
        }
    }

    // 加载到PSRAM缓存
    if (this->fastFileServer.loadToPSRAM(fileName))
    {
        if (this->fastFileServer.sendFromCache(request))
        {
            return;
        }
    }
    
    // 检查文件是否存在
    if (!LittleFS.exists(fileName))
    {
        request->send(404, "text/plain", "文件不存在: " + fileName);
        return;
    }
    
    File file = LittleFS.open(fileName, "r");
    if (!file)
    {
        request->send(500, "text/plain", "无法打开文件");
        return;
    }
    
    size_t fileSize = file.size();
    file.close();  // ✅ 先关闭文件
    
    Serial.printf("📥 下载请求: %s (%u 字节)\n", fileName.c_str(), fileSize);
    
    // ✅ 在lambda中重新打开文件
    AsyncWebServerResponse *response = request->beginResponse(
        "application/octet-stream",
        fileSize,
        [fileName](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
        {
            static File dlFile;
            
            if (index == 0)
            {
                dlFile = LittleFS.open(fileName, "r");
            }
            
            if (!dlFile)
            {
                return 0;
            }
            
            size_t bytesRead = dlFile.read(buffer, maxLen);
            
            if (bytesRead == 0)
            {
                dlFile.close();
            }
            
            return bytesRead;
        }
    );
    
    response->addHeader("Content-Disposition", "attachment; filename=\"" + fileName.substring(1) + "\"");
    response->addHeader("Cache-Control", "no-cache");
    response->addHeader("Connection", "keep-alive");
    
    request->send(response); });

    // 🆕 文件系统信息 - 实时数据
    server->on("/fs_info", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    // ✅ 每次请求实时获取
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    size_t free = total - used;
    
    String json = "{";
    json += "\"total\":" + String(total) + ",";
    json += "\"used\":" + String(used) + ",";
    json += "\"free\":" + String(free) + ",";
    json += "\"percent\":" + String((float)used / total * 100.0f, 1);
    json += "}";
    
    request->send(200, "application/json", json); });

    // 在wifi_transport.cpp中添加诊断端点

    // 🆕 诊断：测试纯TCP速度（不经过文件系统）
    server->on("/speed_test_tcp", HTTP_GET, [](AsyncWebServerRequest *request)
               {
    const size_t TEST_SIZE = 524288; // 512KB测试
    static uint8_t* testBuffer = nullptr;
    
    if (!testBuffer)
    {
        testBuffer = (uint8_t*)malloc(TEST_SIZE);
        if (testBuffer)
        {
            memset(testBuffer, 0xAA, TEST_SIZE);
        }
    }
    
    if (!testBuffer)
    {
        request->send(500, "text/plain", "内存不足");
        return;
    }
    
    uint32_t startTime = millis();
    
    AsyncWebServerResponse *response = request->beginResponse_P(
        200, "application/octet-stream", testBuffer, TEST_SIZE);
    
    response->addHeader("X-Start-Time", String(startTime));
    response->addHeader("Cache-Control", "no-store");
    
    request->send(response);
    
    Serial.printf("🚀 发送512KB测试数据 (内存)\n"); });

    // 🆕 诊断：测试LittleFS读取速度
    server->on("/speed_test_fs", HTTP_GET, [](AsyncWebServerRequest *request)
               {
                   // 创建临时测试文件
                   const char *testFile = "/speed_test.tmp";
                   const size_t TEST_SIZE = 524288; // 512KB

                   // 创建测试文件
                   File f = LittleFS.open(testFile, "w");
                   if (f)
                   {
                       uint8_t *buf = (uint8_t *)malloc(4096);
                       if (buf)
                       {
                           memset(buf, 0xBB, 4096);
                           for (size_t i = 0; i < TEST_SIZE; i += 4096)
                           {
                               f.write(buf, 4096);
                           }
                           free(buf);
                       }
                       f.close();
                   }

                   if (!LittleFS.exists(testFile))
                   {
                       request->send(500, "text/plain", "无法创建测试文件");
                       return;
                   }

                   // 测试读取速度
                   uint32_t startRead = micros();
                   File readFile = LittleFS.open(testFile, "r");
                   size_t bytesRead = 0;
                   uint8_t readBuf[512];

                   while (readFile.available())
                   {
                       bytesRead += readFile.read(readBuf, sizeof(readBuf));
                   }
                   readFile.close();

                   uint32_t readTime = micros() - startRead;
                   float readSpeed = (bytesRead / 1024.0f) / (readTime / 1000000.0f);

                   Serial.printf("📁 文件系统读取速度: %.1f KB/s (读取%u字节耗时%u us)\n",
                                 readSpeed, bytesRead, readTime);

                   // ✅ 修复：使用lambda方式发送文件
                   File dlFile = LittleFS.open(testFile, "r");
                   if (!dlFile)
                   {
                       request->send(500, "text/plain", "无法打开测试文件");
                       return;
                   }

                   AsyncWebServerResponse *response = request->beginResponse(
                       "application/octet-stream",
                       TEST_SIZE,
                       [dlFile](uint8_t *buffer, size_t maxLen, size_t index) mutable -> size_t
                       {
                           if (!dlFile || !dlFile.available())
                           {
                               return 0;
                           }

                           size_t toRead = min(maxLen, (size_t)dlFile.available());
                           return dlFile.read(buffer, toRead);
                       });

                   response->addHeader("X-Read-Speed", String(readSpeed, 1) + " KB/s");
                   response->addHeader("X-Read-Time", String(readTime / 1000) + " ms");
                   response->addHeader("Cache-Control", "no-store");

                   request->send(response);

                   // 注意：不删除测试文件，以便后续测试
               });
    // 启动服务器
    server->begin();

    Serial.printf("✅ Web服务器启动在端口: %d\n", WEB_PORT);
    Serial.printf("✅ 设备IP地址: %s\n", WiFi.localIP().toString().c_str());

    return true;
}

// 🆕 添加文本发送函数
bool WiFiTransport::sendText(const char *data, size_t length)
{
    if (!webSocket || !data || length == 0)
    {
        Serial.println("❌ sendText 失败: 参数无效");
        return false;
    }

    // 检查堆内存
    if (ESP.getFreeHeap() < 10000)
    {
        Serial.printf("⚠️ 内存不足: %u，跳过发送\n", ESP.getFreeHeap());
        return false;
    }

    // 限制数据长度
    if (length > 4096)
    {
        Serial.printf("⚠️ 数据过长: %zu，截断到4096\n", length);
        length = 4096;
    }

    size_t sent = 0;
    for (auto client : _clients)
    {
        if (!client || client->status() != WS_CONNECTED)
        {
            continue;
        }

        if (client->queueIsFull())
        {
            Serial.printf("⚠️ 客户端 #%u 发送队列已满\n", client->id());
            continue;
        }

        // 🆕 使用文本发送（不是二进制）
        client->text(data, length);
        sent++;

        Serial.printf("📤 发送文本数据到客户端 #%u, 长度: %zu, 内容: %.100s\n", client->id(), length, data);
    }

    if (sent == 0)
    {
        Serial.println("⚠️ 没有可用的WebSocket客户端");
    }

    return sent > 0;
}

bool WiFiTransport::sendMessage(const String &message)
{
    // 🆕 使用文本发送，不是二进制
    return sendText(message.c_str(), message.length());
}

// bool WiFiTransport::sendMessage(const String &message)
// {

//     return sendBinary(message.c_str(), message.length());
// }

bool WiFiTransport::sendBinary(const char *data, size_t length)
{
    if (!webSocket || !data || length == 0)
    {
        Serial.println("❌ sendBinary 失败: 参数无效");
        return false;
    }

    // 检查堆内存
    if (ESP.getFreeHeap() < 10000) // 稍微提高阈值
    {
        Serial.printf("⚠️ 内存不足: %u，跳过发送\n", ESP.getFreeHeap());
        return false;
    }

    // 限制数据长度
    if (length > 1024)
    {
        Serial.printf("⚠️ 数据过长: %zu，截断到1024\n", length);
        length = 1024;
    }

    size_t sent = 0;
    for (auto client : _clients)
    {
        if (!client || client->status() != WS_CONNECTED)
        {
            continue;
        }

        if (client->queueIsFull())
        {
            Serial.printf("⚠️ 客户端 #%u 发送队列已满\n", client->id());
            continue;
        }

        // 二进制发送
        client->binary((const uint8_t *)data, length);
        sent++;

        // Serial.printf("📤 websocket发送二进制数据到客户端 #%u, 长度: %zu\n", client->id(), length);
    }

    return sent > 0;
}

bool WiFiTransport::isDeviceConnected()
{
    return _clients.size() > 0;
}

void WiFiTransport::loop()
{
    // 清理断开连接的客户端
    if (webSocket)
    {
        webSocket->cleanupClients();
    }
}

void WiFiTransport::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                     AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
    {
        Serial.printf("WebSocket客户端 #%u 连接来自 %s\n", client->id(), client->remoteIP().toString().c_str());
        _clients.push_back(client);
        deviceConnected = true;
        // 发送连接确认消息
        String connectMsg = "CONNECTED";
        // 发送连接确认消息
        client->text(connectMsg);
        Serial.printf("🔴 连接后客户端数: %zu\n", _clients.size());
        break;
    }

    case WS_EVT_DISCONNECT:
    {
        Serial.printf("WebSocket客户端 #%u 断开连接\n", client->id());
        // 从连接客户端列表中移除
        _clients.erase(
            std::remove(_clients.begin(), _clients.end(), client),
            _clients.end());
        if ((_clients.size() <= 0))
        {
            deviceConnected = false;
        }

        pageFlag = false;

        Serial.printf("🔴 断开后客户端数: %zu\n", _clients.size());
        break;
    }

    case WS_EVT_DATA:
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
        {
            data[len] = 0; // 确保以null结尾
            String message = String((char *)data);
            if (message == "CONNECT_REQUEST")
            {
                // 发送连接确认消息
                String connectMsg = "CONNECT_CONFIRMED";
                // 发送连接确认消息
                client->text(connectMsg);
                Serial.printf("发送确认消息\n");
                return;
            }
            // 消息处理
            messageHandler.processMessage(message);
        }
        break;
    }

    case WS_EVT_ERROR:
    case WS_EVT_PONG:
        break;
    }
}
