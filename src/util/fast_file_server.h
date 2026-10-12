#ifndef FAST_FILE_SERVER_H
#define FAST_FILE_SERVER_H

#include <Arduino.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>

class FastFileServer {
private:
    uint8_t* psramCache;
    size_t cacheSize;
    String cachedFile;
    bool isLoaded;
    
    // 统计信息
    uint32_t cacheHits;
    uint32_t cacheMisses;
    uint32_t totalBytesServed;
    uint32_t loadTimeMs;
    
    uint32_t lastAccessTime;
    uint32_t cacheTimeout;
    static const uint32_t DEFAULT_TIMEOUT = 300000; // 5分钟
    
public:
    FastFileServer();
    ~FastFileServer();
    
    /**
     * @brief 将文件加载到PSRAM缓存
     */
    bool loadToPSRAM(const String& filename, bool forceReload = false);
    
    /**
     * @brief 从PSRAM缓存发送文件
     */
    bool sendFromCache(AsyncWebServerRequest *request);
    
    /**
     * @brief 检查文件是否已缓存
     */
    bool isCached(const String& filename);
    
    /**
     * @brief 检查是否有文件在缓存中
     */
    bool hasCache() const;  // ✅ 替代 isLoaded()
    
    /**
     * @brief 获取缓存文件大小
     */
    size_t getCachedSize();
    
    /**
     * @brief 获取缓存文件名
     */
    String getCachedFilename();
    
    /**
     * @brief 清除缓存
     */
    void clearCache();
    
    /**
     * @brief 获取缓存统计信息
     */
    String getStats();
    
    /**
     * @brief 设置缓存超时时间
     */
    void setCacheTimeout(uint32_t timeoutMs);
    
    /**
     * @brief 自动清理过期缓存
     */
    void autoCleanup();
    
    /**
     * @brief 检查PSRAM是否可用
     */
    static bool isPSRAMAvailable();
    
    /**
     * @brief 获取可用内存
     */
    static size_t getFreeMemory();
    
    /**
     * @brief 预加载目录中的BBL文件
     */
    String preloadBBLFile(const String& directory = "/");
};

#endif // FAST_FILE_SERVER_H