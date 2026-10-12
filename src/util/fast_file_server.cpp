#include "fast_file_server.h"

FastFileServer::FastFileServer()
    : psramCache(nullptr), cacheSize(0), isLoaded(false), cacheHits(0), cacheMisses(0), totalBytesServed(0), loadTimeMs(0), lastAccessTime(0), cacheTimeout(DEFAULT_TIMEOUT)
{
    Serial.println("⚡ FastFileServer 初始化");

    if (isPSRAMAvailable())
    {
        Serial.printf("💾 PSRAM 可用: %u 字节\n", ESP.getPsramSize());
        Serial.printf("💾 PSRAM 空闲: %u 字节\n", ESP.getFreePsram());
    }
    else
    {
        Serial.println("⚠️ PSRAM 不可用，将使用内部RAM");
    }
}

FastFileServer::~FastFileServer()
{
    clearCache();
}

bool FastFileServer::loadToPSRAM(const String &filename, bool forceReload)
{
    // 检查是否已经缓存了同一个文件
    if (isLoaded && cachedFile == filename && !forceReload)
    {
        Serial.printf("📦 文件已在缓存中: %s\n", filename.c_str());
        lastAccessTime = millis(); // 更新访问时间
        return true;
    }

    // 检查文件是否存在
    if (!LittleFS.exists(filename))
    {
        Serial.printf("❌ 文件不存在: %s\n", filename.c_str());
        return false;
    }

    // 打开文件
    File file = LittleFS.open(filename, "r");
    if (!file)
    {
        Serial.printf("❌ 无法打开文件: %s\n", filename.c_str());
        return false;
    }

    size_t fileSize = file.size();

    // 检查文件大小是否合理
    if (fileSize == 0)
    {
        Serial.printf("⚠️ 文件为空: %s\n", filename.c_str());
        file.close();
        return false;
    }

    // 检查是否有足够的内存
    size_t requiredMemory = fileSize + 4096;
    size_t freeMemory = getFreeMemory();

    if (freeMemory < requiredMemory)
    {
        Serial.printf("❌ 内存不足: 需要 %u 字节, 可用 %u 字节\n",
                      requiredMemory, freeMemory);
        file.close();
        return false;
    }

    // 清除旧缓存
    if (psramCache)
    {
        free(psramCache);
        psramCache = nullptr;
        cacheSize = 0;
        isLoaded = false;
    }

    // 在PSRAM中分配内存
    if (isPSRAMAvailable())
    {
        psramCache = (uint8_t *)ps_malloc(fileSize);
        if (psramCache)
        {
            Serial.printf("💾 在PSRAM中分配 %u 字节\n", fileSize);
        }
    }

    // PSRAM分配失败或不可用，尝试内部RAM
    if (!psramCache)
    {
        psramCache = (uint8_t *)malloc(fileSize);
        if (psramCache)
        {
            Serial.printf("💾 在内部RAM中分配 %u 字节\n", fileSize);
        }
    }

    if (!psramCache)
    {
        Serial.printf("❌ 无法分配 %u 字节内存\n", fileSize);
        file.close();
        return false;
    }

    // 读取文件到内存
    uint32_t startTime = micros();
    size_t bytesRead = file.read(psramCache, fileSize);
    file.close();

    uint32_t readTime = (micros() - startTime) / 1000;
    loadTimeMs = readTime;

    if (bytesRead != fileSize)
    {
        Serial.printf("❌ 读取不完整: %u/%u 字节\n", bytesRead, fileSize);
        free(psramCache);
        psramCache = nullptr;
        return false;
    }

    // 计算读取速度
    float readSpeed = (fileSize / 1024.0f) / (readTime / 1000.0f);

    cachedFile = filename;
    cacheSize = fileSize;
    isLoaded = true;
    lastAccessTime = millis();
    cacheMisses++;

    Serial.printf("✅ 文件加载成功: %s\n", filename.c_str());
    Serial.printf("   大小: %u 字节 (%.1f KB)\n", fileSize, fileSize / 1024.0f);
    Serial.printf("   耗时: %u ms\n", readTime);
    Serial.printf("   速度: %.1f KB/s\n", readSpeed);
    Serial.printf("   剩余PSRAM: %u 字节\n", ESP.getFreePsram());

    return true;
}

bool FastFileServer::sendFromCache(AsyncWebServerRequest *request)
{
    if (!isLoaded || !psramCache || cacheSize == 0)
    {
        cacheMisses++;
        return false;
    }

    // 更新访问时间
    lastAccessTime = millis();

    // 从PSRAM创建响应
    AsyncWebServerResponse *response = request->beginResponse_P(
        200,
        "application/octet-stream",
        psramCache,
        cacheSize);

    // 设置响应头
    String filename = cachedFile;
    if (filename.startsWith("/"))
    {
        filename = filename.substring(1);
    }

    response->addHeader("Content-Disposition",
                        "attachment; filename=\"" + filename + "\"");
    response->addHeader("Content-Length", String(cacheSize));
    response->addHeader("X-Cache", "PSRAM-HIT");
    response->addHeader("X-Cache-Size", String(cacheSize));
    response->addHeader("X-Load-Time-Ms", String(loadTimeMs));
    response->addHeader("Accept-Ranges", "bytes");
    response->addHeader("Cache-Control", "no-cache");

    // 发送响应
    request->send(response);

    cacheHits++;
    totalBytesServed += cacheSize;

    Serial.printf("⚡ 从PSRAM发送: %s (%u 字节)\n", cachedFile.c_str(), cacheSize);

    return true;
}

bool FastFileServer::isCached(const String &filename)
{
    return isLoaded && cachedFile == filename && psramCache != nullptr;
}

// ✅ 新增公共方法
bool FastFileServer::hasCache() const
{
    return isLoaded && psramCache != nullptr && cacheSize > 0;
}

size_t FastFileServer::getCachedSize()
{
    return cacheSize;
}

String FastFileServer::getCachedFilename()
{
    return cachedFile;
}

void FastFileServer::clearCache()
{
    if (psramCache)
    {
        free(psramCache);
        psramCache = nullptr;
    }

    cachedFile = "";
    cacheSize = 0;
    isLoaded = false;

    Serial.println("🗑️ 缓存已清除");
}

void FastFileServer::setCacheTimeout(uint32_t timeoutMs)
{
    cacheTimeout = timeoutMs;
    Serial.printf("⏱️ 缓存超时设置为: %u ms\n", timeoutMs);
}

void FastFileServer::autoCleanup()
{
    if (!isLoaded)
        return;

    uint32_t currentTime = millis();
    uint32_t elapsed = currentTime - lastAccessTime;

    // 检查是否过期（考虑millis溢出）
    if (elapsed > cacheTimeout || currentTime < lastAccessTime)
    {
        Serial.printf("🗑️ 自动清理过期缓存: %s (空闲 %u ms)\n",
                      cachedFile.c_str(), elapsed);
        clearCache();
    }
}

String FastFileServer::getStats()
{
    String stats;
    stats += "📊 FastFileServer 统计:\n";
    stats += "   缓存命中: " + String(cacheHits) + "\n";
    stats += "   缓存未命中: " + String(cacheMisses) + "\n";
    stats += "   总传输字节: " + String(totalBytesServed) + " (" +
             String(totalBytesServed / 1024.0f / 1024.0f, 2) + " MB)\n";

    if (isLoaded)
    {
        stats += "   当前缓存: " + cachedFile + " (" +
                 String(cacheSize / 1024.0f, 1) + " KB)\n";
        stats += "   加载耗时: " + String(loadTimeMs) + " ms\n";
        stats += "   空闲时间: " + String((millis() - lastAccessTime) / 1000) + "s\n";
    }
    else
    {
        stats += "   当前缓存: 无\n";
    }

    stats += "   PSRAM可用: " + String(isPSRAMAvailable() ? "是" : "否") + "\n";
    stats += "   可用内存: " + String(getFreeMemory() / 1024) + " KB\n";

    return stats;
}

bool FastFileServer::isPSRAMAvailable()
{
    return psramFound() && ESP.getPsramSize() > 0;
}

size_t FastFileServer::getFreeMemory()
{
    if (isPSRAMAvailable())
    {
        return ESP.getFreePsram();
    }
    return ESP.getFreeHeap();
}

String FastFileServer::preloadBBLFile(const String &directory)
{
    File root = LittleFS.open(directory);
    if (!root || !root.isDirectory())
    {
        Serial.printf("❌ 无法打开目录: %s\n", directory.c_str());
        return "";
    }

    File file = root.openNextFile();
    String loadedFile = "";

    while (file)
    {
        String name = String(file.name());

        // 跳过隐藏文件和系统文件
        if (name.startsWith("/."))
        {
            file.close();
            file = root.openNextFile();
            continue;
        }

        // 查找BBL文件
        if (name.endsWith(".bbl") || name.endsWith(".BBL"))
        {
            Serial.printf("🔍 发现BBL文件: %s (%u 字节)\n",
                          name.c_str(), file.size());

            file.close();
            root.close();

            // 加载到PSRAM
            if (loadToPSRAM(name))
            {
                loadedFile = name;
            }
            return loadedFile;
        }

        file.close();
        file = root.openNextFile();
    }

    root.close();

    if (loadedFile.isEmpty())
    {
        Serial.println("⚠️ 未找到BBL文件");
    }

    return loadedFile;
}