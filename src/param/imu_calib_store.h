// param/imu_calib_store.h
#pragma once
#include "type/imu_types.h"

namespace imu_calib
{
    inline constexpr const char *kDefaultPath = "/calibration.dat";
}

class ImuCalibStore
{
public:
    // 保存校准到文件。返回 true 表示写入且读回校验通过。
    static bool save(const ImuCalibration &calib,
                     const char *path = imu_calib::kDefaultPath);

    // 从文件加载校准。返回 true 表示文件存在、魔数/checksum 通过。
    //   out 会被填充；返回 false 时 out 内容未定义（调用方自己决定是否用零校准）。
    static bool load(ImuCalibration *out,
                     const char *path = imu_calib::kDefaultPath);

    // 文件是否存在（不做格式检查，仅 stat）。
    static bool exists(const char *path = imu_calib::kDefaultPath);

    // 删除校准文件（用于强制重新校准）。
    static bool clear(const char *path = imu_calib::kDefaultPath);

private:
    // 文件头：魔数 + 版本 + checksum，防掉电写入损坏
    static constexpr uint32_t kMagic = 0x494D5543; // 'IMUC'
    static constexpr uint16_t kVersion = 1;
};