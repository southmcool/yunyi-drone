/**
 * @title  IMU 朝向与坐标变换实现
 * @author: vaie
 * @date: 2026-10-12
 *
 * 从旧 Mpu6500::transformToBodyFrame 机械搬迁。
 * 保持所有分支的轴映射与符号翻转行为不变；原注释一并保留。
 *
 * 本文件不依赖 Arduino / ESP-IDF / FreeRTOS，可被 SIL 直接编译。
 */

#include "core/imu/imu_orientation.h"

Axis3f toBodyFrame(imu_orientation_t orient, const Axis3f &v)
{
    Axis3f out = v;

    switch (orient)
    {
    case ORIENTATION_DEFAULT:
        // X=前, Y=左, Z=上：芯片系与机体系一致，无需转换
        break;

    case ORIENTATION_FUSIONCONVENTIONNWU:
        // 与 DEFAULT 等价：旧代码里此枚举落到 switch 的 default 分支，
        // 行为就是恒等变换 —— 保留该行为以对齐已注册基线。
        break;

    case ORIENTATION_FUSIONCONVENTIONNED:
        // 目标：机体系 NED（X=北, Y=东, Z=下）
        // 假设芯片物理朝向 = X=北, Y=西, Z=上（标准 NWU）
        //   轴关系：X_ned =  X_nwu   (北=北)
        //           Y_ned = -Y_nwu   (东=-西)
        //           Z_ned = -Z_nwu   (下=-上)
        // ⚠ 本机（真机）不是这个朝向 ⇒ 真机走下面的 YAW_90 分支。
        out.x = v.x;
        out.y = -v.y;
        out.z = -v.z;
        break;

    case ORIENTATION_FUSIONCONVENTIONNED_YAW_90:
        // ★ 真机实际朝向（imu_task.cpp:58 设为此值）。
        //   芯片绕 Z 轴相对 NWU 转了 +90°：X=东, Y=北, Z=上
        //   （即标准 NWU 的 X=北,Y=西 被换成 Y=北,X=东）
        //
        //   目标机体系 NED：X=北, Y=东, Z=下
        //     X_ned(北) <- 芯片 Y(北)   => out.x = v.y
        //     Y_ned(东) <- 芯片 X(东)   => out.y = v.x
        //     Z_ned(下) <- -芯片 Z(上)  => out.z = -v.z
        //
        //   ★ 判据（flight(4) 静止段 n=169，机体完全静止 std<0.026g）：
        //     芯片原始 rax=-0.00284 ray=+0.04517 raz=+1.00883 (|a|=1.0098g)
        //     本分支输出 ax=+0.04517 ay=-0.00284 az=-1.00883
        //       => 反算 roll=+0.161°, pitch=+2.564°
        //     与 EKF 实测 roll=+0.211°, pitch=+2.535° 吻合 ✅
        //     （说明真机静态安装倾角 2.56° 是绕机体右轴的前后仰，非左右滚）
        //   对比：NED 分支 (x,-y,-z) 会给出 roll=+2.564/pitch=-0.161，与实测不符。
        //   证据脚本 tools/an_flight4o.py。
        out.x = v.y;
        out.y = v.x;
        out.z = -v.z;
        break;

    case ORIENTATION_FUSIONCONVENTIONENU:
        // 目标 ENU：X=东, Y=北, Z=上
        // 输入 NWU：X=北, Y=西, Z=上
        //   变换：原 X(北) -> 新 Y(北)
        //         原 Y(西) -> 新 X(东) 需要反转
        //         原 Z(上) -> 新 Z(上) 不变
        out.x = -v.y;
        out.y = v.x;
        out.z = v.z;
        break;

    default:
        // 未识别的枚举：原样返回
        break;
    }

    return out;
}