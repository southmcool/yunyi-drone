#include "altitude_controller.h"
#include <math.h>

AltitudeController altitudeController;

// ★R77k：起飞抢高治理（对齐 PX4 的爬升率限幅 MPC_TKO_SPEED/MPC_Z_VEL_MAX_UP）
//   旧值 _max_velocity=3.0 使贴地 0.3m 目标高度就被转成 0.59m/s 目标速度，
//   速度环 Kp=1.5 再转成 0.89 推力修正（被 _max_correction=0.5 钳满）
//   ⇒ th_out = 0.075 + 0.5 = 0.575 满油门蹿起（flight(2) 实测 2219273→2219306
//   一拍内 th_out 0.069→0.575），随后回落再蹿的 bang-bang 把机体带歪。
//   现：位置环速度上限降到 0.5m/s（本机体积小，爬升 0.5m/s 已很快），
//   并对推力修正加 2.0/s 斜率限幅（见 update）。
AltitudeController::AltitudeController()
    : _max_velocity(0.5f), _max_correction(0.5f),
      _target_velocity(0.0f), _correction_output(0.0f)
{
    // 设置默认增益 (需后续调参)
    // 位置环 P: 将高度误差 (米) 映射到目标速度 (米/秒)
    _pos_controller.setGains(2.0, 0.0, 0.2);
    _pos_controller.setLimits(-_max_velocity, _max_velocity);

    // 速度环 PID: 将速度误差 (米/秒) 映射到油门修正 (0~1)
    _vel_controller.setGains(1.5, 0.2, 0.0);
    _vel_controller.setLimits(-_max_correction, _max_correction);
    _vel_controller.setIntegralLimits(-0.2f, 0.2f);
    _vel_controller.setDerivativeFilter(0.5f);
    _vel_controller.setIntegralSeparation(0.5f); // 速度误差小于0.5m/s时才积分
}

float AltitudeController::update(float target_altitude, const state_t &state, float dt)
{
    if (dt <= 0.0f)
        return _correction_output;

    float altitude_error = target_altitude - state.altitude;
    float abs_error = fabsf(altitude_error);

    // ★ 2026-10-05：软死区（smoothstep 权重）取代原「3cm 进 / 5cm 出的硬迟滞
    //    + _target_velocity *= 0.8 陡断」——去掉硬切换与逐拍衰减的指令陡断，
    //    位置环修正随 |误差| 平滑升起，消除「滑行-猛纠」的不连续。
    //    ⚠ SIL A/B（R28 hold180b n=3）实测高度 gt_z 摆动**无显著变化**（std/p2p
    //      持平，频谱 0.05-0.5Hz 占比不变）⇒ SIL 里那 ~0.14Hz 高度摆动**并非死区
    //      所致**（另见 R29 关水平无效、R30 降位置环 P 反而更差）。故本改动
    //      SIL 中性，保留小死区以抑制真机 ToF 噪声；是否改善真机"顺畅"待台架 A/B。
    const float DZ_ZERO = 0.012f; // ≤1.2cm：不修正（仅速度环稳速）
    const float DZ_FULL = 0.040f; // ≥4.0cm：全权重修正
    float w = (abs_error - DZ_ZERO) * (1.0f / (DZ_FULL - DZ_ZERO));
    w = fminf(fmaxf(w, 0.0f), 1.0f);
    w = w * w * (3.0f - 2.0f * w); // smoothstep，C1 连续

    // 位置环每拍都跑（不再冻结内部 D 滤波状态），输出按软权重缩放
    _target_velocity = w * _pos_controller.update(target_altitude, state.altitude, dt);
    // 位置环输出再显式钳一次（PIDController.setLimits 已设，这里双保险对齐 PX4
    // 的爬升/下降速度限制语义：最大爬升率 _max_velocity）。
    _target_velocity = fminf(fmaxf(_target_velocity, -_max_velocity), _max_velocity);
    float raw = _vel_controller.update(_target_velocity, state.vertical_velocity, dt);

    // ★R77k：推力修正斜率限幅。贴地大误差时位置/速度环会同时饱和，raw 触及
    //   _max_correction(0.5)；直接下发会造成 th_out 阶跃（flight(2) 实测 0.069→
    //   0.575 一拍）→ 机体弹起、随即误差反向又猛收 → bang-bang 带歪姿态。
    //   限幅后修正量按 _max_correction_rate 每秒的斜率逼近目标，起飞平缓。
    if (_max_correction_rate > 0.0f)
    {
        const float max_step = _max_correction_rate * dt;
        float delta = raw - _correction_output;
        if (delta > max_step)
            delta = max_step;
        else if (delta < -max_step)
            delta = -max_step;
        _correction_output += delta;
    }
    else
    {
        _correction_output = raw;
    }

    return _correction_output;
}

void AltitudeController::setGains(const float pos_gains[3], const float vel_gains[3])
{
    _pos_controller.setGains(pos_gains[0], pos_gains[1], pos_gains[2]);
    _vel_controller.setGains(vel_gains[0], vel_gains[1], vel_gains[2]);

    Serial.println("✅ 高度PID参数设置成功");

    Serial.printf("位置环: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  pos_gains[0], pos_gains[1], pos_gains[2]);
    Serial.printf("速度环: KP=%.5f, KI=%.5f, KD=%.5f\n",
                  vel_gains[0], vel_gains[1], vel_gains[2]);
}

void AltitudeController::setLimits(float max_velocity, float max_correction)
{
    _max_velocity = max_velocity;
    _max_correction = max_correction;
    _pos_controller.setLimits(-_max_velocity, _max_velocity);
    _vel_controller.setLimits(-_max_correction, _max_correction);
}

void AltitudeController::reset()
{
    _pos_controller.reset();
    _vel_controller.reset();
    _target_velocity = 0.0f;
    _correction_output = 0.0f;
}