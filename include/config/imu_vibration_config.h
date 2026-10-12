#pragma once
// =============================================================================
// ImuVibConfig —— IMU 侧振动治理的统一配置（谐波陷波 / 自适应测频 / VRE 前馈）
//
// 【为什么需要它（R58）】
//   此前这三项治理的参数全部由 SIL_* 环境变量驱动，且读取点散落在
//   `#ifdef SIL_HOST` 里。真机上 getenv() 恒返回 NULL ⇒ 三项**全部落回"关"**：
//     · 谐波陷波基频 = 硬编码 473Hz（实际激励 299.85Hz，Q=3.5 ⇒ 带宽仅 86Hz
//       ⇒ 打偏 173Hz = 零衰减）；
//     · VRE 前馈 k=0（加计整流 DC ~0.35g 全量进 EKF ⇒ 幽灵倾角 ~20°）；
//     · 无 tilt 绝对锚（EKF2 无加计倾角观测，见 ekf2_joint.cpp::fuseGravity）。
//   SIL 对照臂 Z0_bare（= 真机等价配置）满剂振动下 23s 炸机、位移 31.6m、θ_dc 20.2°。
//   ⇒ 本结构把"真机默认值"显式化，env 降级为 SIL 覆盖层。
//
// 【默认值策略 —— 关键，别误改】
//   · 真机：默认值 = SIL 实测最优（能开的都开）⇒ 开箱可飞；
//   · SIL ：由 Mpu6500::initSignalChain() **显式重置为历史默认（全关）**，
//           再由 SIL_* env 覆盖 ⇒ 所有已注册基线与 A/B 臂**逐位不变**。
//   即：SIL 是实验台（默认全关、一切靠显式注入），真机是部署目标（默认开）。
//
// 【哪些默认开、哪些不开（有意为之）】
//   开  ：① 自适应测频（无自由参数，靠在线跟踪，R53/R57 验收）
//          ② 重力锚（σ_grav=0.2，R42 验收；对应 EKF2 侧 grav_fusion_mode）
//   不开：VRE 前馈 k —— 它依赖**器件级整流常数 k_rect**，SIL 里的 0.08 是振动注入
//         模型的常数；真机器件必须台架回归实测（k_rect = a_dc / a_rms²，扫转速取
//         斜率）后再填 kx/ky/kz。盲目抄 −0.150 有**过补偿**风险：若器件 k_rect 远
//         小于模型值，扣掉的 dc 会超过真值 ⇒ 引入反向幽灵倾角，比不补偿更糟
//         （R54 实测过 60% 过补偿 ⇒ 残差 0.21g ≈ 12°）。⇒ 默认 0，标定后一行即开。
//
// =============================================================================
// ★★ 上机首飞检查表（R58，必读）—— 逐条对应串口会打出的日志
// =============================================================================
//   开机即打的两行（Mpu6500::begin()）：
//     [IMU] 芯片事实: WHO_AM_I=0x.. CONFIG(0x1A)=0x.. ACCEL_CONFIG2(0x1D)=0x.. =>
//           陀螺DLPF_CFG=n 加计A_DLPF_CFG=m (...)
//       ✔ 期望 加计 A_DLPF_CFG=0 ⇒ 460Hz。若不然，必须把 vdc.dlpf_fc_hz 改成
//         实际带宽（否则 VRE 频率归一化 |H_dlpf|² 整体系数就错了）。
//     [VIB] 陷波基频=.. 自适应测频=ON/OFF VRE前馈=OFF(未标定) 加计低通=30Hz ...
//       ✔ 确认 自适应测频=ON；VRE 在台架标定前**应该是** OFF(未标定)。
//   起飞后每 ~2s 打一行（低优先级任务里打，不占用 1ms 采样环）：
//     [NOTCHADAPT] f=..Hz(meas ..) ratio=.. valid=..% applied=..Hz ...
//       ✔ 期望：ratio 显著大于门限 50（SIL 实测真实振动段 ≥300），f 稳定。
//       ✘ 若 f 乱跳 / ratio 贴着 50 / valid 很低 ⇒ **测频不可信**：
//         立刻把 notch_adapt 改 false，并把 notch_base_hz 改成台架实测的刀叶通过
//         频率（电机带桨地面转，加速度计谱峰 × 桨叶数），重新烧录。
//   为什么默认 300Hz 而不是实测值：300Hz 来自 SIL 机架模型（f_n=300），真机机架
//   必然不同 ⇒ 固定值本来就是猜的，**由在线测频去找才是正解**，这也是把
//   notch_adapt 默认打开的根本理由。
// =============================================================================

#include <stdint.h>

// VRE 前馈参数（VibDcCompensator 消费）
struct VibDcParams
{
    // 前馈系数 [g/g²]：dc[i] = k[i] · k_scale · a_rms²
    //   物理含义 = k_rect / |H_dlpf(f_ref)|²（推导见 VibDcCompensator.h 头注释）
    //   ⚠ 真机默认 0（未标定）；台架回归出 k_rect 后填 kx/ky/kz
    float kx = 0.0f;
    float ky = 0.0f;
    float kz = 0.0f;           // 安装不对称通常只出现在与激励垂直的两轴 ⇒ 默认 0
    float tau_s = 0.03f;       // a_rms 的 EMA 时间常数 [s]（30ms：介于振动包络与姿态带宽之间）
    bool fnorm = true;         // 频率归一化 k_eff = k·|H(f_ref)|²/|H(f)|²（k=0 时无副作用）
    float fref_hz = 200.0f;    // k 的标定频率 [Hz]
    float dlpf_fc_hz = 460.0f; // 加计片内 DLPF 3dB 带宽 —— 真机 A_DLPF_CFG=0 ⇒ 460Hz
    // 真机台架标定打印 [VDCCAL]（每 0.2s 一行：原始加计 EMA 均值 + a_rms）。
    //   默认关；标定时改 true 重烧一次，标完改回。SIL 不受影响。
    bool calib_log = false;
};

struct ImuVibConfig
{
    // ── 谐波陷波（HarmonicNotchFilter，三轴 accel/gyro 各一个）──
    float notch_base_hz = 300.0f; // 起始基频；自适应开启后由在线测频接管
    float notch_q = 3.5f;
    int notch_harmonics = 3; // 300/600/900Hz；后两个在 1kHz 采样下高于 Nyquist

    // ── 自适应测频（SpectralPeakTracker；与 PX4 gyro_fft + IMU_GYRO_DNF 同构）──
    bool notch_adapt = true;
    float adapt_fmin_hz = 80.0f;    // 搜索带下限（避开控制带与桨叶低频）
    float adapt_fmax_ratio = 0.45f; // 搜索带上限 = ratio × fs（须 < Nyquist）
    float adapt_tau_s = 1.5f;       // 频率 EMA 时间常数 [s]

    // ★ R76 重调持续性门：单帧过 K_MIN_RATIO 门即重调 ⇒ 长期静止会被少数垃圾峰
    //   拖走（2026-10-10 真机实测：静止 valid=3%（449 帧 ~13 帧过门）把陷波从
    //   300Hz 拉到 98.7Hz；meas 80.1Hz ratio 15.2 正是 R53 记录的地面假峰族）。
    //   真实振动段逐帧过门（R53 实测比值 ≥299，多数 1e3~9e3）⇒ recent≈100%；
    //   静止假峰段 ≈3% ⇒ 冻结。两个参数即此门的阈值。
    float adapt_min_valid_frac = 0.25f; // 近窗（≤32 帧）有效帧占比低于此 ⇒ 冻结重调
    int adapt_min_hist = 16;            // 门生效前至少积累的帧数（≈16s @1kHz/1024 点）

    // ── 低通（SensorFilter）──
    //   加计 30Hz：真机与 SIL 都开（`accel_lp_enabled` 默认 true）；
    //   陀螺 80Hz：默认关——会给角速度内环引入相位滞后，SIL 用 SIL_GYRO_LP 做 A/B。
    float accel_lp_hz = 30.0f; // 0 = 旁路
    float gyro_lp_hz = 0.0f;

    VibDcParams vdc;
};
