#include "attitude_controller.h"
#include <stdio.h>

extern AltitudeController altitudeController;
AttitudeController attitudeController;
AttitudeController::AttitudeController()
    : enabled_(true), last_update_time_(0)
{
    // 初始化调试信息
    // debug_info_ = {0};

    // 外环（角度环）增益
    float default_outer_gains[3][3] = {
        {10.0f, 0.005f, 0.0f}, // roll:  P=3.5, I=0.15, D=0.6
        {10.0f, 0.005f, 0.0f}, // pitch
        {2.0f, 0.002f, 0.0f}   // yaw
    };

    // 内环（角速度环）增益 - 大幅降低 D 值，适当提高 P 值
    float default_inner_gains[3][3] = {
        {0.090f, 0.1f, 0.003f}, // roll:  P=0.25, I=0.03, D=0.0015
        {0.090f, 0.1f, 0.003f}, // pitch
        {0.15f, 0.01f, 0.001f}  // yaw:   P=0.12, I=0.012, D=0.001
    };

    setGains(default_outer_gains, default_inner_gains);
    setLimits(30.0f, 120.0f, 0.6f);

    // 积分分离阈值（弧度）
    roll_controller_.outer_loop_.setIntegralSeparation(0.35f); // 20°
    roll_controller_.inner_loop_.setIntegralSeparation(1.05f); // 60°/s
    pitch_controller_.outer_loop_.setIntegralSeparation(0.35f);
    pitch_controller_.inner_loop_.setIntegralSeparation(1.05f);
    yaw_controller_.outer_loop_.setIntegralSeparation(0.44f); // 25°
    yaw_controller_.inner_loop_.setIntegralSeparation(1.40f); // 80°/s

    // 在构造函数末尾添加，替换原来的 mix[3];
    // printf("=== CF2X Mixer Table ===\n");
    // printf("Idx | Pos | Roll   | Pitch  | Yaw\n");
    // printf("------------------------------------\n");
    // const char *pos_names[4] = {"FR", "BR", "BL", "FL"};
    // for (int i = 0; i < 4; i++)
    // {
    //     printf("[%d]  | %s  | %+6.2f | %+6.2f | %+6.2f\n",
    //            i, pos_names[i],
    //            mix[i][0], mix[i][1], mix[i][2]);
    // }
    // printf("====================================\n");
}

// === 2. 低推力安全处理（新增核心逻辑）===
static constexpr float THRUST_PID_ENABLE_THRESHOLD = 0.07f; // PID启用阈值
static constexpr float THRUST_RESET_THRESHOLD = 0.005f;     // 完全复位阈值（带滞回）
static bool pid_was_active = false;
// 自稳模式的位置保持目标。
// ★ 2026-10-01 修复初值陷阱：原实现用 -1 作哨兵判断「是否已捕获目标」
//   （见 case 2 的 == -1 判据），但这四个静态量的实际初值全是 0
//   （原写法 `static float a, b, c, d = 0;` 只有 d 有显式初值，其余零初始化，
//   四个都不是 -1）⇒ 上电后首次进入自稳模式的重捕获判据 0==-1 恒假，
//   目标被钉死在 (0,0) 上电原点而非起飞点。实飞 20261001_004022/005903 证实。
//   现改用显式 bool 捕获标志；标志在 mode 0/1 与「离开空中」时清除，
//   保证每次起飞都重捕获当前位置为保持目标。
static float stabilize_pos_x = 0, stabilize_pos_y = 0, stabilize_vel_x = 0, stabilize_vel_y = 0;
static bool stabilize_target_captured = false;
static float targetRoll, targetPitch = 0;

bool AttitudeController::update(const state_t &state, const setpoint_t &setpoint, control_t &output, float dt)
{
    if (dt <= 0.0001f || dt > 0.1f)
        dt = 0.002f;
    // 推力低于复位阈值 → 彻底重置所有控制器状态
    if (setpoint.thrust < THRUST_RESET_THRESHOLD)
    {
        if (pid_was_active)
        { // 仅从有推力切换到无推力时复位一次
            roll_controller_.reset();
            pitch_controller_.reset();
            yaw_controller_.reset();
            pid_was_active = false;
            Serial.println("🔒 Low thrust: Controllers RESET");
        }
        // 强制输出为零（直通安全状态）
        output.roll = output.pitch = output.yaw = 0.0f;
        output.thrust = setpoint.thrust;

        return true; // 返回true表示"安全地处理了"，而非故障
    }

    // 推力在 [RESET, ENABLE) 区间 → 保持复位状态，不执行PID
    if (setpoint.thrust < THRUST_PID_ENABLE_THRESHOLD)
    {
        output.roll = output.pitch = output.yaw = 0.0f;
        output.thrust = setpoint.thrust;

        return true;
    }

    // 推力 >= ENABLE_THRESHOLD → 正常PID控制
    pid_was_active = true;

    // ---- 高度环诊断（无条件打印，1s 一次）----

    // ===== 新增：高度控制 =====
    // ===== 新增：高度控制 =====
    float altitude_correction = 0.0f;
    // ★R77f：高度环还要求目标高度已被显式设定（>0.05m）。GUI 高度滑条默认 0，
    //   忘设高度就切自动/自稳时，高度环会把目标 0m 当真、把机体往地上砸
    //   （flight(8) 实测：th_out 0.05↔0.575 振荡，离地即失控）。
    if (setpoint.thrust > 0.05f && setpoint.position.z > 0.05f)
    { // 仅在电机有基础推力时启用高度环
        altitude_correction = altitudeController.update(setpoint.position.z, state, dt);
        // Serial.printf("altitude_correction::::%.2", altitude_correction);
    }
    else
    {
        // 无推力时重置高度控制器，防止积分饱和
        altitudeController.reset();
    }

    targetRoll = setpoint.attitudeRate.roll;
    targetPitch = setpoint.attitudeRate.pitch;

    // ===== 位置控制（仅在空中启用）=====
    // ★ 2026-10-01 实飞证据（telemetry_20261001_004022 / 005903）：原判据
    //   `setpoint.thrust > THRUST_PID_ENABLE_THRESHOLD(0.07)` 把位置环使能绑在
    //   油门上，而悬停油门恰好 ≈ 0.07 ⇒ `0.07 > 0.07` 恒假，位置环整场关闭
    //   （第一场：pos_sp 全程绝对值之和 = 0，水平开环漂移 0.41m）。更糟的是
    //   `thrust` 遥测是 int16×100（0.065~0.0749 全显示 0.07），使能状态不可观测
    //   ——同一杆量两次飞行一次开一次关（第二场 pos_sp 有实时输出）。
    //   故改为纯高度滞回判据，与油门解耦：
    //   进入 >0.10 m，退出 <0.05 m（滞回防贴地抖振反复复位）。
    static bool pos_loop_in_air = false;
    if (state.altitude > 0.10f)
        pos_loop_in_air = true;
    else if (state.altitude < 0.05f)
        pos_loop_in_air = false;
    if (pos_loop_in_air)
    {
        switch (setpoint.flight_mode)
        {
        case flight_mode_e::manual:
            stabilize_target_captured = false;
            positionController_.disable();

            break;
        case flight_mode_e::position_hold:
            stabilize_target_captured = false;
            positionController_.enable();
            positionController_.update(state.position.x, state.position.y, state.velocity.x, state.velocity.y,
                                       setpoint.position.x, setpoint.position.y, setpoint.velocity.x, setpoint.velocity.y,
                                       state.attitude_rad.yaw, dt);
            targetRoll = positionController_.getTargetRoll();
            targetPitch = positionController_.getTargetPitch();
            break;
        case flight_mode_e::stabilize:
            if (!stabilize_target_captured)
            {
                stabilize_pos_x = state.position.x;
                stabilize_pos_y = state.position.y;
                stabilize_vel_x = 0;
                stabilize_vel_y = 0;
                positionController_.reset();
                // 目标从旧值跳到当前位置 ⇒ 必须同步前馈基准，否则
                // (target−prev_target)/dt 差分给出 ~500 的假目标速度 ⇒ 饱和尖峰
                positionController_.syncFeedforward(state.position.x, state.position.y);
                stabilize_target_captured = true;
            }
            positionController_.enable();
            positionController_.update(state.position.x, state.position.y, state.velocity.x, state.velocity.y,
                                       setpoint.position.x, setpoint.position.y, setpoint.velocity.x, setpoint.velocity.y,
                                       state.attitude_rad.yaw, dt);
            targetRoll = positionController_.getTargetRoll();
            targetPitch = positionController_.getTargetPitch();
            break;
        default:
            stabilize_target_captured = false;
            positionController_.disable();
            break;
        }
    }
    else
    {
        // 落地（高度 <0.05m）：清除捕获标志 ⇒ 下次起飞重捕获当前位置为保持目标
        stabilize_target_captured = false;
        positionController_.disable(); // 内部会调用 reset()
    }
    // 1. 获取当前姿态四元数（优先直接从状态读取，若无则临时转换）
    Quaternion q_cur;
    q_cur = {state.attitudeQuaternion.q0, state.attitudeQuaternion.q1, state.attitudeQuaternion.q2, state.attitudeQuaternion.q3};
    // 过渡方案：
    // q_cur = eulerToQuat(state.roll, state.pitch, state.yaw);

    // 2. 目标姿态：roll/pitch 来自位置环或手动，yaw 直接用 setpoint.yaw
    float target_yaw = setpoint.attitudeRate.yaw;
    Quaternion q_tar = eulerToQuat(targetRoll, targetPitch, target_yaw);

    // 3. 计算误差四元数 (机体坐标系下)
    Quaternion q_cur_conj = quatConjugate(q_cur);
    Quaternion q_err = quatMultiply(q_cur_conj, q_tar); // q_cur^-1 * q_tar

    // 4. 提取三轴角度误差 (rad)
    float err_roll, err_pitch, err_yaw;
    quatToAxisError(q_err, err_roll, err_pitch, err_yaw);

    // 5. 可选：对误差做限幅（安全）
    const float MAX_ERR = 0.5236f; // 30°
    err_roll = constrain(err_roll, -MAX_ERR, MAX_ERR);
    err_pitch = constrain(err_pitch, -MAX_ERR, MAX_ERR);
    err_yaw = constrain(err_yaw, -MAX_ERR, MAX_ERR);
    // 6. 送入双环控制器的 updateSafe 接口
    output.roll = roll_controller_.updateSafe(err_roll, 0.0f, state.gyro_rate.x, dt);
    output.pitch = pitch_controller_.updateSafe(err_pitch, 0.0f, state.gyro_rate.y, dt);
    output.yaw = yaw_controller_.updateSafe(err_yaw, 0.0f, state.gyro_rate.z, dt);
    // // 三轴独立控制
    // output.roll_output = roll_controller_.update(targetRoll, state.roll, state.roll_rate, dt);
    // output.pitch_output = pitch_controller_.update(targetPitch, state.pitch, state.pitch_rate, dt);
    // output.yaw_output = yaw_controller_.update(target_yaw, state.yaw, state.yaw_rate, dt);

    output.thrust = setpoint.thrust + altitude_correction;
    if (output.thrust < 0.0f)
        output.thrust = 0.0f;
    if (output.thrust > 1.0f)
        output.thrust = 1.0f;
    // output.timestamp = state.attitude_rad.timestamp;

    // 更新调试信息
    // updateDebugInfo();
    return true;
}

void AttitudeController::setGains(const float outer_gains[3][3], const float inner_gains[3][3])
{
    // 横滚轴
    roll_controller_.setOuterGains(outer_gains[0][0], outer_gains[0][1], outer_gains[0][2]);
    roll_controller_.setInnerGains(inner_gains[0][0], inner_gains[0][1], inner_gains[0][2]);

    // 俯仰轴
    pitch_controller_.setOuterGains(outer_gains[1][0], outer_gains[1][1], outer_gains[1][2]);
    pitch_controller_.setInnerGains(inner_gains[1][0], inner_gains[1][1], inner_gains[1][2]);

    // 偏航轴
    yaw_controller_.setOuterGains(outer_gains[2][0], outer_gains[2][1], outer_gains[2][2]);
    yaw_controller_.setInnerGains(inner_gains[2][0], inner_gains[2][1], inner_gains[2][2]);

    // 设置更强的微分滤波
    // roll_controller_.inner_loop_.setDerivativeFilter(0.8f); // 低alpha值
    // pitch_controller_.inner_loop_.setDerivativeFilter(0.8f);
    // yaw_controller_.inner_loop_.setDerivativeFilter(0.85f);
    roll_controller_.inner_loop_.setDerivativeFilter(0.0f); // 低alpha值
    pitch_controller_.inner_loop_.setDerivativeFilter(0.0f);
    yaw_controller_.inner_loop_.setDerivativeFilter(0.0f);

    // roll_controller_.setRateFilterAlpha(0.45f);
    // pitch_controller_.setRateFilterAlpha(0.45f);
    // yaw_controller_.setRateFilterAlpha(0.45f);
    roll_controller_.setRateFilterAlpha(0.7f);
    pitch_controller_.setRateFilterAlpha(0.7f);
    yaw_controller_.setRateFilterAlpha(0.7f);

    float ol = 10.0f;
    float il = 0.1f;

    // 设置合理的积分限
    roll_controller_.outer_loop_.setIntegralLimits(-ol, ol);
    pitch_controller_.outer_loop_.setIntegralLimits(-ol, ol);
    yaw_controller_.outer_loop_.setIntegralLimits(-0.15f, 0.15f);

    roll_controller_.inner_loop_.setIntegralLimits(-il, il);
    pitch_controller_.inner_loop_.setIntegralLimits(-il, il);
    yaw_controller_.inner_loop_.setIntegralLimits(-0.5f, 0.5f);

    Serial.println("✅姿态 PID参数设置成功");

    Serial.printf("内环Roll: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  inner_gains[0][0], inner_gains[0][1], inner_gains[0][2]);
    Serial.printf("内环pitch: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  inner_gains[1][0], inner_gains[1][1], inner_gains[1][2]);
    Serial.printf("内环yaw: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  inner_gains[2][0], inner_gains[2][1], inner_gains[2][2]);

    Serial.printf("外环Roll: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  outer_gains[0][0], outer_gains[0][1], outer_gains[0][2]);
    Serial.printf("外环pitch: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  outer_gains[1][0], outer_gains[1][1], outer_gains[1][2]);
    Serial.printf("外环yaw: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  outer_gains[2][0], outer_gains[2][1], outer_gains[2][2]);
}

StaticJsonDocument<200> AttitudeController::getPIDJson() const
{
    StaticJsonDocument<200> doc;

    doc["MODELNAME"] = "bicyclopid";
    doc["type"] = "PID";

    float outer_gains[3][3];
    float inner_gains[3][3];
    getCurrentGains(outer_gains, inner_gains);

    // 内环
    doc["inner_loop"]["roll"][0] = inner_gains[0][0];
    doc["inner_loop"]["roll"][1] = inner_gains[0][1];
    doc["inner_loop"]["roll"][2] = inner_gains[0][2];

    doc["inner_loop"]["pitch"][0] = inner_gains[1][0];
    doc["inner_loop"]["pitch"][1] = inner_gains[1][1];
    doc["inner_loop"]["pitch"][2] = inner_gains[1][2];

    doc["inner_loop"]["yaw"][0] = inner_gains[2][0];
    doc["inner_loop"]["yaw"][1] = inner_gains[2][1];
    doc["inner_loop"]["yaw"][2] = inner_gains[2][2];

    // 外环
    doc["outer_loop"]["roll"][0] = outer_gains[0][0];
    doc["outer_loop"]["roll"][1] = outer_gains[0][1];
    doc["outer_loop"]["roll"][2] = outer_gains[0][2];

    doc["outer_loop"]["pitch"][0] = outer_gains[1][0];
    doc["outer_loop"]["pitch"][1] = outer_gains[1][1];
    doc["outer_loop"]["pitch"][2] = outer_gains[1][2];

    doc["outer_loop"]["yaw"][0] = outer_gains[2][0];
    doc["outer_loop"]["yaw"][1] = outer_gains[2][1];
    doc["outer_loop"]["yaw"][2] = outer_gains[2][2];

    doc["velocity"][0] = altitudeController._vel_controller.getKp();
    doc["velocity"][1] = altitudeController._vel_controller.getKi();
    doc["velocity"][2] = altitudeController._vel_controller.getKd();

    doc["location"][0] = altitudeController._pos_controller.getKp();
    doc["location"][1] = altitudeController._pos_controller.getKi();
    doc["location"][2] = altitudeController._pos_controller.getKd();
    return doc;
}

// 获取当前所有PID参数的实现
void AttitudeController::getCurrentGains(float outer_gains[3][3], float inner_gains[3][3]) const
{
    // 获取外环参数
    outer_gains[0][0] = roll_controller_.outer_loop_.getKp();
    outer_gains[0][1] = roll_controller_.outer_loop_.getKi();
    outer_gains[0][2] = roll_controller_.outer_loop_.getKd();

    outer_gains[1][0] = pitch_controller_.outer_loop_.getKp();
    outer_gains[1][1] = pitch_controller_.outer_loop_.getKi();
    outer_gains[1][2] = pitch_controller_.outer_loop_.getKd();

    outer_gains[2][0] = yaw_controller_.outer_loop_.getKp();
    outer_gains[2][1] = yaw_controller_.outer_loop_.getKi();
    outer_gains[2][2] = yaw_controller_.outer_loop_.getKd();

    // 获取内环参数
    inner_gains[0][0] = roll_controller_.inner_loop_.getKp();
    inner_gains[0][1] = roll_controller_.inner_loop_.getKi();
    inner_gains[0][2] = roll_controller_.inner_loop_.getKd();

    inner_gains[1][0] = pitch_controller_.inner_loop_.getKp();
    inner_gains[1][1] = pitch_controller_.inner_loop_.getKi();
    inner_gains[1][2] = pitch_controller_.inner_loop_.getKd();

    inner_gains[2][0] = yaw_controller_.inner_loop_.getKp();
    inner_gains[2][1] = yaw_controller_.inner_loop_.getKi();
    inner_gains[2][2] = yaw_controller_.inner_loop_.getKd();
}

void AttitudeController::setLimits(float max_angle_deg, float max_angle_rate_deg, float max_output)
{
    float max_angle_rad = max_angle_deg * DEG_TO_RAD;
    float max_angle_rate_rad = max_angle_rate_deg * DEG_TO_RAD;

    // 设置控制器限制
    roll_controller_.setLimits(max_angle_rad, max_angle_rate_rad, max_output);
    pitch_controller_.setLimits(max_angle_rad, max_angle_rate_rad, max_output);
    yaw_controller_.setLimits(max_angle_rad, max_angle_rate_rad, max_output);

    // 设置安全监控限制（稍微宽松一些）
    safety_monitor_.setSafetyLimits(max_angle_rad * 1.2f, max_angle_rate_rad * 1.5f, max_output);
}

void AttitudeController::reset()
{
    roll_controller_.reset();
    pitch_controller_.reset();
    yaw_controller_.reset();
    last_update_time_ = 0;
    // debug_info_ = {0};
}

float AttitudeController::calculateDeltaTime(uint32_t current_time)
{
    if (last_update_time_ == 0)
    {
        last_update_time_ = current_time;
        return 0.01f; // 默认10ms
    }

    float dt = (current_time - last_update_time_) / 1000.0f;
    last_update_time_ = current_time;

    // 限制在合理范围内
    return (dt > 0.0f && dt <= 0.1f) ? dt : 0.01f;
}

// void AttitudeController::updateDebugInfo()
// {
//     debug_info_.outer_roll = roll_controller_.getOuterOutput();
//     debug_info_.outer_pitch = pitch_controller_.getOuterOutput();
//     debug_info_.outer_yaw = yaw_controller_.getOuterOutput();

//     debug_info_.inner_roll = roll_controller_.getInnerOutput();
//     debug_info_.inner_pitch = pitch_controller_.getInnerOutput();
//     debug_info_.inner_yaw = yaw_controller_.getInnerOutput();

//     debug_info_.angle_errors[0] = roll_controller_.getAngleError();
//     debug_info_.angle_errors[1] = pitch_controller_.getAngleError();
//     debug_info_.angle_errors[2] = yaw_controller_.getAngleError();

//     debug_info_.rate_errors[0] = roll_controller_.getRateError();
//     debug_info_.rate_errors[1] = pitch_controller_.getRateError();
//     debug_info_.rate_errors[2] = yaw_controller_.getRateError();
//     debug_info_.target_state[0] = targetRoll;
//     debug_info_.target_state[1] = targetPitch;
//     debug_info_.target_state[2] = 0;
// }

// bool AttitudeController::validateInputs(const state_t &state, const setpoint_t &setpoint)
// {
//     // 检查时间戳有效性
//     // if (state.timestamp == 0)
//     // {
//     //     return false;
//     // }

//     // 检查姿态估计质量
//     if (state.estimate_quality < 30)
//     {
//         return false;
//     }

//     if (isnan(state.roll) || isnan(state.pitch) || isnan(state.yaw))
//         return false;
//     if (isnan(setpoint.roll) || isnan(setpoint.pitch) || isnan(setpoint.yaw))
//         return false;

//     return true;
// }
