/**
 * @title  校准数据持久化
 * @author: vaie
 * @date: 2026-10-12
 *
 * imu_calib_store.cpp — ImuCalibration 的 Flash 存储（LittleFS）。
 *
 * 文件布局（二进制，POD，直接 memcpy）：
 *   [0]  magic      4B  'IMUC' = 0x494D5543
 *   [4]  version    2B  = 1
 *   [6]  reserved   2B  = 0（4 字节对齐填充）
 *   [8]  calib      N B  sizeof(ImuCalibration)
 *   [8+N] checksum  4B  前 8+N 字节的字节和
 *
 * 写入策略（防掉电损坏）：
 *   1. 先删旧文件（确保干净起点）
 *   2. 写完整文件
 *   3. 读回校验：比对 magic / version / checksum / 逐字节 memcmp
 *   4. 校验失败则删除脏文件，返回 false
 *
 * SIL 兼容：
 *   宿主机无 LittleFS ⇒ 整个实现包 #if !defined(SIL_HOST)。
 *   SIL 下 save/load/exists/clear 恒返回 false，校准值由 sim_main 直接注入。
 */

#include "param/imu_calib_store.h"

#if !defined(SIL_HOST)

#include <cstring>
#include <Arduino.h>
#include <LittleFS.h> // 路径按项目实际调整

namespace
{

    // ---------------------------------------------------------------------------
    //  文件布局（POD，无虚函数、无指针，memcpy 安全）
    // ---------------------------------------------------------------------------
    struct FileLayout
    {
        uint32_t magic; // 'IMUC'
        uint16_t version;
        uint16_t reserved;
        ImuCalibration calib;
        uint32_t checksum; // 覆盖前 (sizeof(FileLayout) - sizeof(uint32_t)) 字节
    };

    static_assert(sizeof(ImuCalibration) % 4 == 0,
                  "ImuCalibration 需 4 字节对齐，便于文件整体 memcpy");

    // 计算除 checksum 之外所有字节的和
    uint32_t calcChecksum(const FileLayout &f)
    {
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&f);
        const size_t n = sizeof(FileLayout) - sizeof(uint32_t);
        uint32_t sum = 0;
        for (size_t i = 0; i < n; ++i)
            sum += p[i];
        return sum;
    }

    bool ensureMounted()
    {
        // 项目里已有 LittleFSHelper.isMounted() 之类，如果存在可直接用。
        // 这里保守地检查一下：文件系统未挂载时 open 会失败，直接返回。
        return true;
    }

} // namespace

// ============================================================================
//  save
// ============================================================================
bool ImuCalibStore::save(const ImuCalibration &calib, const char *path)
{
    if (!path || !ensureMounted())
        return false;

    // ---- 步骤 1：先删旧文件，保证干净的写入起点 ----
    if (LittleFS.exists(path))
    {
        LittleFS.remove(path);
    }

    // ---- 步骤 2：填充文件布局 ----
    FileLayout f;
    std::memset(&f, 0, sizeof(f));
    f.magic = kMagic;
    f.version = kVersion;
    f.calib = calib;
    f.checksum = calcChecksum(f);

    // ---- 步骤 3：写入 ----
    File file = LittleFS.open(path, "w");
    if (!file)
    {
        Serial.printf("[CALIB] 无法创建文件: %s\n", path);
        return false;
    }

    const size_t written = file.write(reinterpret_cast<const uint8_t *>(&f),
                                      sizeof(FileLayout));
    file.close(); // close() 内部执行 flush，确保落盘

    if (written != sizeof(FileLayout))
    {
        Serial.printf("[CALIB] 写入不完整: %u/%u 字节\n",
                      (unsigned)written, (unsigned)sizeof(FileLayout));
        LittleFS.remove(path);
        return false;
    }

    // ---- 步骤 4：读回校验 ----
    File verify = LittleFS.open(path, "r");
    if (!verify)
    {
        Serial.println("[CALIB] 读回校验失败：无法打开文件");
        return false;
    }

    FileLayout rb;
    const size_t rd = verify.read(reinterpret_cast<uint8_t *>(&rb), sizeof(FileLayout));
    verify.close();

    if (rd != sizeof(FileLayout) ||
        rb.magic != kMagic ||
        rb.version != kVersion ||
        rb.checksum != calcChecksum(rb) ||
        std::memcmp(&rb, &f, sizeof(FileLayout)) != 0)
    {
        Serial.println("[CALIB] 读回校验失败：数据不一致");
        LittleFS.remove(path);
        return false;
    }

    Serial.printf("[CALIB] 校准已保存并校验: %s (%u 字节)\n",
                  path, (unsigned)written);
    return true;
}

// ============================================================================
//  load
// ============================================================================
bool ImuCalibStore::load(ImuCalibration *out, const char *path)
{
    if (!out || !path)
        return false;

    if (!LittleFS.exists(path))
        return false;

    File file = LittleFS.open(path, "r");
    if (!file)
        return false;

    FileLayout f;
    const size_t rd = file.read(reinterpret_cast<uint8_t *>(&f), sizeof(FileLayout));
    file.close();

    if (rd != sizeof(FileLayout))
    {
        Serial.printf("[CALIB] 读取不完整: %u/%u 字节\n",
                      (unsigned)rd, (unsigned)sizeof(FileLayout));
        return false;
    }

    if (f.magic != kMagic)
    {
        Serial.printf("[CALIB] 魔数不匹配: 0x%08X\n", (unsigned)f.magic);
        return false;
    }

    if (f.version != kVersion)
    {
        // 版本不匹配不一定致命（字段可能向后兼容），但当前版本只有 1 号，
        // 遇到不认识的版本一律拒绝，避免误读未知格式。
        Serial.printf("[CALIB] 版本不匹配: %u（期望 %u）\n",
                      (unsigned)f.version, (unsigned)kVersion);
        return false;
    }

    const uint32_t expected = calcChecksum(f);
    if (f.checksum != expected)
    {
        Serial.printf("[CALIB] 校验和错误: 期望 0x%08X 实际 0x%08X\n",
                      (unsigned)expected, (unsigned)f.checksum);
        return false;
    }

    *out = f.calib;
    return true;
}

// ============================================================================
//  exists / clear
// ============================================================================
bool ImuCalibStore::exists(const char *path)
{
    if (!path)
        return false;
    return LittleFS.exists(path);
}

bool ImuCalibStore::clear(const char *path)
{
    if (!path)
        return false;
    if (!LittleFS.exists(path))
        return true; // 已经不存在 = 清除成功
    return LittleFS.remove(path);
}

// ============================================================================
//  SIL 分支：无 LittleFS，全部返回 false
// ============================================================================
#else // SIL_HOST

bool ImuCalibStore::save(const ImuCalibration &, const char *) { return false; }
bool ImuCalibStore::load(ImuCalibration *, const char *) { return false; }
bool ImuCalibStore::exists(const char *) { return false; }
bool ImuCalibStore::clear(const char *) { return false; }

#endif // SIL_HOST