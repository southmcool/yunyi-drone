// 固件版本信息
#ifndef VERSION_H
#define VERSION_H

// 固件版本信息
#define FIRMWARE_VERSION "0.01"
#define FIRMWARE_BUILD_DATE __DATE__ " " __TIME__

// OTA 服务器配置
#ifndef OTA_SERVER_URL
#define OTA_SERVER_URL "http://192.168.1.4:3000/g-drive/yyds/firmware/firmware.bin"
#endif

#ifndef OTA_VERSION_URL
#define OTA_VERSION_URL "http://192.168.1.4:3000/g-drive/yyds/firmware/version.txt"
#endif



#endif // VERSION_H