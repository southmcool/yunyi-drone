#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

class LittleFSTool
{
private:
    bool _fsMounted;
    String _basePath;

public:
    LittleFSTool();
    ~LittleFSTool();

    // 文件系统管理
    bool initialize(const char *partitionLabel = "littlefs", const char *basePath = "/");
    bool format();
    void unmount();
    bool isMounted() const { return _fsMounted; }

    // 文件信息
    void printFileSystemInfo();
    void listFiles(const String &path = "/", int depth = 0);
    size_t getFileSize(const String &filepath);
    bool fileExists(const String &filepath);
    bool dirExists(const String &path);
    time_t getFileTime(const String &filepath);

    // 文件操作
    bool writeFile(const String &filepath, const String &content);
    bool writeFile(const String &filepath, const uint8_t *data, size_t length);
    String readFile(const String &filepath);
    bool readFile(const String &filepath, uint8_t *buffer, size_t bufferSize);
    bool appendFile(const String &filepath, const String &content);
    bool deleteFile(const String &filepath);
    bool copyFile(const String &sourcePath, const String &destPath);
    bool moveFile(const String &sourcePath, const String &destPath);

    // 目录操作
    bool createDir(const String &path);
    bool deleteDir(const String &path);
    bool deleteDirRecursive(const String &path);

    // JSON 文件操作
    bool writeJSON(const String &filepath, const JsonDocument &doc);
    bool readJSON(const String &filepath, JsonDocument &doc);
    DynamicJsonDocument readJSON(const String &filepath, size_t docSize = 1024);

    // 配置文件操作
    bool writeConfig(const String &filename, const String &key, const String &value);
    String readConfig(const String &filename, const String &key, const String &defaultValue = "");
    bool deleteConfig(const String &filename, const String &key);

    // 日志操作
    bool writeLog(const String &logname, const String &message);
    String readLog(const String &logname, int maxLines = 100);
    bool clearLog(const String &logname);
    size_t getLogSize(const String &logname);

    // 备份和恢复
    bool backupFile(const String &filepath, const String &backupSuffix = ".bak");
    bool restoreBackup(const String &filepath, const String &backupSuffix = ".bak");
    bool createSnapshot(const String &snapshotName);

    // 工具函数
    String getFullPath(const String &filepath);
    size_t getTotalSpace();
    size_t getUsedSpace();
    size_t getFreeSpace();
    float getUsagePercent();

    // 批量操作
    bool backupAllConfigs(const String &backupDir = "/backups");
    bool cleanOldBackups(int keepDays = 7);

private:
    bool ensureDirExists(const String &filepath);
    String getFileExtension(const String &filename);
    String getFileName(const String &filepath);
    String getDirPath(const String &filepath);
    bool isValidFilename(const String &filename);
};

// 全局实例
extern LittleFSTool LittleFSHelper;