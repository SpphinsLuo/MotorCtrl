// biped_hw — pid_track.hpp
// P6 跟踪打包：q_des / dq_des → 电机指令。不是 plugin / AppBase / 虚工厂。
// 曲线形状在 quintic_traj.hpp，这里只跟踪调用方给的 q_des。
//
// 内环（不要再套一层主机 PID）：
//   Damiao MIT：包内 PD — pack_mit(q_des, dq_des, kp, kd, tau_ff)。
//   HA60 servo：固件位置/速度环 — pack_motorevo_servo(θ_ref=q_des, pos kp/kd, V_ref)。
// 髋并联用 hip_ik 的调用在 sendCommand，不在这里。
#pragma once

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/types.hpp"

#include <cmath>
#include <cstdint>

namespace biped_hw
{

inline constexpr float kPidTrackKp = 8.f;
inline constexpr float kPidTrackKd = 0.5f;

// HA60 firmware servo (p3 hold). V_ref=0 or vel_kp=0 cannot track θ_ref.
inline constexpr float kPidTrackHa60VelLimit = 0.5f;
inline constexpr float kPidTrackHa60VelKp = 20.f;
inline constexpr float kPidTrackHa60VelKd = 0.f;
inline constexpr float kPidTrackHa60VelKi = 0.001f;

struct PidTrackGains
{
  float kp = kPidTrackKp;
  float kd = kPidTrackKd;
  float tau_ff = 0.f;  // MIT only
  float ha60_vel_limit = kPidTrackHa60VelLimit;
  float ha60_vel_kp = kPidTrackHa60VelKp;
  float ha60_vel_kd = kPidTrackHa60VelKd;
  float ha60_vel_ki = kPidTrackHa60VelKi;
};

// Damiao: MIT PD on the wire. No host outer PID.
inline void pid_track_pack_mit(uint8_t out[8], float q_des, float dq_des, float kp, float kd,
                               const LimitParam &lim, float tau_ff = 0.f)
{
  pack_mit(out, kp, kd, q_des, dq_des, tau_ff, lim);
}

// HA60: firmware servo toward θ_ref. No extra host outer PID.
inline void pid_track_pack_ha60(uint8_t out[8], float q_des, float dq_des,
                                const PidTrackGains &g)
{
  float vref = std::fabs(dq_des);
  if (vref < g.ha60_vel_limit)
  {
    vref = g.ha60_vel_limit;
  }
  pack_motorevo_servo(out, q_des, g.kp, g.kd, vref, g.ha60_vel_kp, g.ha60_vel_kd,
                      g.ha60_vel_ki);
}

inline void pid_track_pack(uint8_t out[8], MotorType model, float q_des, float dq_des,
                           const PidTrackGains &g, const LimitParam &lim)
{
  if (model == MotorType::HA60)
  {
    pid_track_pack_ha60(out, q_des, dq_des, g);
    return;
  }
  pid_track_pack_mit(out, q_des, dq_des, g.kp, g.kd, lim, g.tau_ff);
}

// 0 = pass. No hardware.
inline int pid_track_self_test()
{
  const float q0 = 5.f;  // right-leg ID14-like; pack whatever q_des the caller gave
  const float q_des = q0 + 0.35f;
  const LimitParam lim = kLimitDm10422;
  uint8_t mit[8] = {};
  PidTrackGains g{};
  pid_track_pack(mit, MotorType::DM10422, q_des, 0.f, g, lim);
  float kp_w = 0.f, kd_w = 0.f, q_w = 0.f, dq_w = 0.f, tau_w = 0.f;
  unpack_mit_cmd(mit, lim, kp_w, kd_w, q_w, dq_w, tau_w);
  if (!std::isfinite(kp_w) || !std::isfinite(q_w) || std::fabs(kp_w - g.kp) > 1.f ||
      std::fabs(q_w - q_des) > 0.01f || mit[7] == kDmEnableCmd)
  {
    return 1;
  }

  uint8_t evo[8] = {};
  pid_track_pack(evo, MotorType::HA60, q0, 0.1f, g, kLimitHa60);
  float eq = 0.f, pkp = 0.f, pkd = 0.f, vlim = 0.f, vkp = 0.f, vkd = 0.f, vki = 0.f;
  unpack_motorevo_servo_cmd(evo, eq, pkp, pkd, vlim, vkp, vkd, vki);
  if (!std::isfinite(eq) || std::fabs(eq - q0) > 0.02f || std::fabs(pkp - g.kp) > 1.5f ||
      evo[7] == kMotorevoEnableCmd)
  {
    return 2;
  }
  return 0;
}

}  // namespace biped_hw
