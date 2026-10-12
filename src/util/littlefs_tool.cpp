#include "littlefs_tool.h"

LittleFSTool LittleFSHelper;

LittleFSTool::LittleFSTool()
    : _fsMounted(false), _basePath("/")
{
}

LittleFSTool::~LittleFSTool()
{
    if (_fsMounted)
    {
        LittleFS.end();
    }
}

bool LittleFSTool::initialize(const char *partitionLabel, const char *basePath)
{
    if(_fsMounted){
        return _fsMounted;
    }
    Serial.printf("🚀 初始化 LittleFS (分区: %s)\n", partitionLabel);

    // 尝试挂载文件系统
    _fsMounted = LittleFS.begin(true, basePath, 10, partitionLabel);

    if (!_fsMounted)
    {
        Serial.printf("❌ LittleFS 挂载失败 (分区: %s)\n", partitionLabel);
        return false;
    }

    _basePath = basePath;
    Serial.println("✅ LittleFS 初始化成功");

    // 打印文件系统信息
    printFileSystemInfo();

    // 创建必要的目录结构
    createDir("/configs");
    createDir("/logs");
    createDir("/backups");
    createDir("/data");

    return true;
}

bool LittleFSTool::format()
{
    Serial.println("⚠️  格式化 LittleFS 文件系统...");

    if (_fsMounted)
    {
        LittleFS.end();
    }

    bool result = LittleFS.format();

    if (result)
    {
        Serial.println("✅ 文件系统格式化成功");
        _fsMounted = LittleFS.begin(true, _basePath.c_str(), 10, "littlefs");
    }
    else
    {
        Serial.println("❌ 文件系统格式化失败");
    }

    return result;
}

void LittleFSTool::unmount()
{
    if (_fsMounted)
    {
        LittleFS.end();
        _fsMounted = false;
        Serial.println("✅ LittleFS 已卸载");
    }
}

void LittleFSTool::printFileSystemInfo()
{
    if (!_fsMounted)
    {
        Serial.println("❌ 文件系统未挂载");
        return;
    }

    Serial.println("=== LittleFS 文件系统信息 ===");
    Serial.printf("总空间: %.2f KB\n", getTotalSpace() / 1024.0);
    Serial.printf("已使用: %.2f KB\n", getUsedSpace() / 1024.0);
    Serial.printf("空闲空间: %.2f KB\n", getFreeSpace() / 1024.0);
    Serial.printf("使用率: %.1f%%\n", getUsagePercent());
    Serial.println("============================\n");
}

void LittleFSTool::listFiles(const String &path, int depth)
{
    if (!_fsMounted)
        return;

    File root = LittleFS.open(path);
    if (!root)
    {
        Serial.printf("❌ 无法打开目录: %s\n", path.c_str());
        return;
    }

    if (!root.isDirectory())
    {
        Serial.printf("❌ 不是目录: %s\n", path.c_str());
        root.close();
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        // 缩进显示目录结构
        for (int i = 0; i < depth; i++)
        {
            Serial.print("  ");
        }

        if (file.isDirectory())
        {
            Serial.printf("📁 %s/\n", file.name());
            listFiles(file.name(), depth + 1);
        }
        else
        {
            Serial.printf("📄 %s (%d bytes)\n", file.name(), file.size());
        }

        file = root.openNextFile();
    }

    root.close();
}

size_t LittleFSTool::getFileSize(const String &filepath)
{
    if (!_fsMounted)
        return 0;

    File file = LittleFS.open(filepath, "r");
    if (!file)
        return 0;

    size_t size = file.size();
    file.close();
    return size;
}

bool LittleFSTool::fileExists(const String &filepath)
{
    if (!_fsMounted)
        return false;
    return LittleFS.exists(filepath);
}

bool LittleFSTool::dirExists(const String &path)
{
    if (!_fsMounted)
        return false;

    File dir = LittleFS.open(path);
    if (!dir)
        return false;

    bool exists = dir.isDirectory();
    dir.close();
    return exists;
}

time_t LittleFSTool::getFileTime(const String &filepath)
{
    // LittleFS 不直接支持文件时间戳
    // 这里返回当前时间作为占位符
    return time(nullptr);
}

bool LittleFSTool::writeFile(const String &filepath, const String &content)
{
    if (!_fsMounted)
        return false;

    if (!ensureDirExists(filepath))
    {
        Serial.printf("❌ 无法创建目录: %s\n", getDirPath(filepath).c_str());
        return false;
    }

    File file = LittleFS.open(filepath, "w");
    if (!file)
    {
        Serial.printf("❌ 无法创建文件: %s\n", filepath.c_str());
        return false;
    }

    size_t bytesWritten = file.print(content);
    file.close();

    if (bytesWritten == content.length())
    {
        Serial.printf("✅ 写入文件: %s (%d bytes)\n", filepath.c_str(), bytesWritten);
        return true;
    }
    else
    {
        Serial.printf("❌ 文件写入不完整: %s (%d/%d bytes)\n",
                      filepath.c_str(), bytesWritten, content.length());
        return false;
    }
}

bool LittleFSTool::writeFile(const String &filepath, const uint8_t *data, size_t length)
{
    if (!_fsMounted)
        return false;

    File file = LittleFS.open(filepath, "w");
    if (!file)
        return false;

    size_t bytesWritten = file.write(data, length);
    file.close();

    return bytesWritten == length;
}

String LittleFSTool::readFile(const String &filepath)
{
    if (!_fsMounted)
        return "";

    File file = LittleFS.open(filepath, "r");
    if (!file)
    {
        Serial.printf("❌ 无法读取文件: %s\n", filepath.c_str());
        return "";
    }

    String content = file.readString();
    file.close();

    return content;
}

bool LittleFSTool::readFile(const String &filepath, uint8_t *buffer, size_t bufferSize)
{
    if (!_fsMounted)
        return false;

    File file = LittleFS.open(filepath, "r");
    if (!file)
        return false;

    size_t bytesRead = file.read(buffer, bufferSize);
    file.close();

    return bytesRead > 0;
}

bool LittleFSTool::appendFile(const String &filepath, const String &content)
{
    if (!_fsMounted)
        return false;

    File file = LittleFS.open(filepath, "a");
    if (!file)
        return false;

    size_t bytesWritten = file.print(content);
    file.close();

    return bytesWritten == content.length();
}

bool LittleFSTool::deleteFile(const String &filepath)
{
    if (!_fsMounted)
        return false;

    if (!LittleFS.exists(filepath))
    {
        Serial.printf("⚠️  文件不存在: %s\n", filepath.c_str());
        return true; // 文件不存在，视为删除成功
    }

    bool result = LittleFS.remove(filepath);
    if (result)
    {
        Serial.printf("✅ 删除文件: %s\n", filepath.c_str());
    }
    else
    {
        Serial.printf("❌ 删除文件失败: %s\n", filepath.c_str());
    }

    return result;
}

bool LittleFSTool::copyFile(const String &sourcePath, const String &destPath)
{
    if (!_fsMounted)
        return false;

    // 读取源文件
    String content = readFile(sourcePath);
    if (content.length() == 0)
    {
        return false;
    }

    // 写入目标文件
    return writeFile(destPath, content);
}

bool LittleFSTool::moveFile(const String &sourcePath, const String &destPath)
{
    if (!_fsMounted)
        return false;

    if (copyFile(sourcePath, destPath))
    {
        return deleteFile(sourcePath);
    }

    return false;
}

bool LittleFSTool::createDir(const String &path)
{
    if (!_fsMounted)
        return false;

    if (dirExists(path))
    {
        return true; // 目录已存在
    }

    bool result = LittleFS.mkdir(path);
    if (result)
    {
        Serial.printf("✅ 创建目录: %s\n", path.c_str());
    }
    else
    {
        Serial.printf("❌ 创建目录失败: %s\n", path.c_str());
    }

    return result;
}

bool LittleFSTool::deleteDir(const String &path)
{
    if (!_fsMounted)
        return false;

    if (!dirExists(path))
    {
        return true; // 目录不存在，视为删除成功
    }

    bool result = LittleFS.rmdir(path);
    if (result)
    {
        Serial.printf("✅ 删除目录: %s\n", path.c_str());
    }
    else
    {
        Serial.printf("❌ 删除目录失败: %s (目录可能不为空)\n", path.c_str());
    }

    return result;
}

bool LittleFSTool::deleteDirRecursive(const String &path)
{
    if (!_fsMounted)
        return false;

    File root = LittleFS.open(path);
    if (!root || !root.isDirectory())
    {
        return false;
    }

    File file = root.openNextFile();
    while (file)
    {
        String filepath = file.name();
        if (file.isDirectory())
        {
            deleteDirRecursive(filepath);
        }
        else
        {
            deleteFile(filepath);
        }
        file = root.openNextFile();
    }
    root.close();

    return deleteDir(path);
}

// JSON 文件操作
bool LittleFSTool::writeJSON(const String &filepath, const JsonDocument &doc)
{
    String jsonStr;
    serializeJson(doc, jsonStr);
    return writeFile(filepath, jsonStr);
}

bool LittleFSTool::readJSON(const String &filepath, JsonDocument &doc)
{
    String jsonStr = readFile(filepath);
    if (jsonStr.length() == 0)
    {
        return false;
    }

    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error)
    {
        Serial.printf("❌ JSON解析失败: %s (%s)\n", filepath.c_str(), error.c_str());
        return false;
    }

    return true;
}

DynamicJsonDocument LittleFSTool::readJSON(const String &filepath, size_t docSize)
{
    DynamicJsonDocument doc(docSize);
    readJSON(filepath, doc);
    return doc;
}

// 配置文件操作
bool LittleFSTool::writeConfig(const String &filename, const String &key, const String &value)
{
    String filepath = "/configs/" + filename + ".json";

    // 读取现有配置
    DynamicJsonDocument doc(1024);
    readJSON(filepath, doc);

    // 更新配置
    doc[key] = value;

    // 写回文件
    return writeJSON(filepath, doc);
}

String LittleFSTool::readConfig(const String &filename, const String &key, const String &defaultValue)
{
    String filepath = "/configs/" + filename + ".json";

    DynamicJsonDocument doc(1024);
    if (!readJSON(filepath, doc))
    {
        return defaultValue;
    }

    return doc[key] | defaultValue;
}

bool LittleFSTool::deleteConfig(const String &filename, const String &key)
{
    String filepath = "/configs/" + filename + ".json";

    DynamicJsonDocument doc(1024);
    if (!readJSON(filepath, doc))
    {
        return false;
    }

    doc.remove(key);
    return writeJSON(filepath, doc);
}

// 日志操作
bool LittleFSTool::writeLog(const String &logname, const String &message)
{
    String filepath = "/logs/" + logname + ".log";
    String timestamp = String(millis());
    String logEntry = "[" + timestamp + "] " + message + "\n";

    return appendFile(filepath, logEntry);
}

String LittleFSTool::readLog(const String &logname, int maxLines)
{
    String filepath = "/logs/" + logname + ".log";
    String content = readFile(filepath);

    if (maxLines > 0)
    {
        // 简单实现：返回最后 maxLines 行
        int lineCount = 0;
        int startPos = content.length() - 1;

        while (startPos >= 0 && lineCount < maxLines)
        {
            if (content.charAt(startPos) == '\n')
            {
                lineCount++;
            }
            startPos--;
        }

        if (startPos >= 0)
        {
            return content.substring(startPos + 2); // +2 跳过\n和下一个字符
        }
    }

    return content;
}

bool LittleFSTool::clearLog(const String &logname)
{
    String filepath = "/logs/" + logname + ".log";
    return deleteFile(filepath);
}

size_t LittleFSTool::getLogSize(const String &logname)
{
    String filepath = "/logs/" + logname + ".log";
    return getFileSize(filepath);
}

// 备份和恢复
bool LittleFSTool::backupFile(const String &filepath, const String &backupSuffix)
{
    if (!fileExists(filepath))
    {
        return false;
    }

    String backupPath = filepath + backupSuffix;
    return copyFile(filepath, backupPath);
}

bool LittleFSTool::restoreBackup(const String &filepath, const String &backupSuffix)
{
    String backupPath = filepath + backupSuffix;
    if (!fileExists(backupPath))
    {
        return false;
    }

    return copyFile(backupPath, filepath);
}

bool LittleFSTool::createSnapshot(const String &snapshotName)
{
    String snapshotDir = "/backups/" + snapshotName;
    if (!createDir(snapshotDir))
    {
        return false;
    }

    // 备份所有配置文件
    bool success = true;
    success &= backupFile("/configs/pid_config.json", "_" + snapshotName + ".bak");
    // 可以添加其他需要备份的文件

    return success;
}

// 工具函数
String LittleFSTool::getFullPath(const String &filepath)
{
    return _basePath + filepath;
}

size_t LittleFSTool::getTotalSpace()
{
    if (!_fsMounted)
        return 0;
    return LittleFS.totalBytes();
}

size_t LittleFSTool::getUsedSpace()
{
    if (!_fsMounted)
        return 0;
    return LittleFS.usedBytes();
}

size_t LittleFSTool::getFreeSpace()
{
    if (!_fsMounted)
        return 0;
    return getTotalSpace() - getUsedSpace();
}

float LittleFSTool::getUsagePercent()
{
    if (!_fsMounted)
        return 0.0f;
    return (getUsedSpace() * 100.0f) / getTotalSpace();
}

// 批量操作
bool LittleFSTool::backupAllConfigs(const String &backupDir)
{
    if (!createDir(backupDir))
    {
        return false;
    }

    String timestamp = String(millis());
    String backupSubDir = backupDir + "/config_backup_" + timestamp;

    if (!createDir(backupSubDir))
    {
        return false;
    }

    // 这里可以实现具体的备份逻辑
    // 例如复制 /configs 目录下的所有文件到备份目录

    return true;
}

bool LittleFSTool::cleanOldBackups(int keepDays)
{
    // 简单的实现：删除所有 .bak 文件
    // 实际应用中可以根据时间戳来删除旧备份
    Serial.println("🗑️  清理旧备份文件...");

    File root = LittleFS.open("/");
    File file = root.openNextFile();
    int deletedCount = 0;

    while (file)
    {
        String filename = file.name();
        if (filename.endsWith(".bak"))
        {
            if (deleteFile(filename))
            {
                deletedCount++;
            }
        }
        file = root.openNextFile();
    }

    root.close();
    Serial.printf("✅ 清理完成，删除了 %d 个备份文件\n", deletedCount);
    return deletedCount > 0;
}

// 私有方法
bool LittleFSTool::ensureDirExists(const String &filepath)
{
    String dirPath = getDirPath(filepath);
    if (dirPath.length() > 0 && dirPath != "/")
    {
        return createDir(dirPath);
    }
    return true;
}

String LittleFSTool::getFileExtension(const String &filename)
{
    int dotIndex = filename.lastIndexOf('.');
    if (dotIndex == -1)
        return "";
    return filename.substring(dotIndex + 1);
}

String LittleFSTool::getFileName(const String &filepath)
{
    int slashIndex = filepath.lastIndexOf('/');
    if (slashIndex == -1)
        return filepath;
    return filepath.substring(slashIndex + 1);
}

String LittleFSTool::getDirPath(const String &filepath)
{
    int slashIndex = filepath.lastIndexOf('/');
    if (slashIndex == -1)
        return "";
    return filepath.substring(0, slashIndex);
}

bool LittleFSTool::isValidFilename(const String &filename)
{
    // 简单的文件名验证
    if (filename.length() == 0)
        return false;
    if (filename.indexOf('/') != -1)
        return false;
    if (filename == "." || filename == "..")
        return false;
    return true;
}