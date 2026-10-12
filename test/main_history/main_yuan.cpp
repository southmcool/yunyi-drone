/**
 * @title: 程序主入口
 * @author: vaie
 * @date: 2026-10-11
 * @description:
 */

#include <Arduino.h>
#include <esp_partition.h>
#include <esp_ota_ops.h> // ⭐ 必须添加这个头文件

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("ESP32-S3 位置开发分支 -v1 已启动！");
}

void loop()
{
    // put your main code here, to run repeatedly:
}
