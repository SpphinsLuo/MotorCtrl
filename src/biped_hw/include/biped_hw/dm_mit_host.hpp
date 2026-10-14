// biped_hw — dm_mit_host.hpp
// Damiao host helpers: 0xFC/0xFD/0xFE, RID 21/22/23, CTRL_MODE MIT/POS_VEL/VEL/POS_FORCE,
// hold / host dq / home-to-zero (MIT host-dq toward 0, not 0xFE). Functions only — not a class
// hierarchy. Apps keep test motion profiles. MIT remains the default enable path
// (dm_enable_mit). Evo analog lives here because it shares sendCan.
#pragma once

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/types.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <cstdint>

namespace biped_hw
{

inline constexpr int kDmDisableGapUs = 10000;
inline constexpr int kDmEnableSettleUs = 100000;
inline constexpr int kDmRidTimeoutMs = 200;
inline constexpr int kDmRidRetries = 3;
inline constexpr int kDmCtrlModeSettleUs = 20000;

// Home-before-motion: probe |q|, skip if already at zero, else speed-limited
// MIT host-dq toward 0 (kp=0). Not a kp=80 step. Timeout scales with |q|/vel.
inline constexpr float kDmHomeEpsRad = 0.02f;
inline constexpr float kDmHomeTimeoutS = 4.f;       // minimum
inline constexpr float kDmHomeTimeoutSlackS = 2.f;  // |q|/vel + slack
inline constexpr float kDmHomeTimeoutMaxS = 30.f;
inline constexpr float kDmHomeVel = 1.f;            // rad/s default
inline constexpr float kDmHomeVelMax = 3.f;
inline constexpr float kDmHomeKd = 2.f;             // host-dq damping; stall ~ kd*vel
inline constexpr int kDmHomePeriodMs = 10;
inline constexpr float kEvoHomeKp = 8.f;  // p2_evo pos default
inline constexpr float kEvoHomeKd = 0.5f;
inline constexpr float kEvoHomeVelLimit = 0.5f;
inline constexpr float kEvoHomeVelKp = 20.f;
inline constexpr float kEvoHomeVelKd = 0.f;
inline constexpr float kEvoHomeVelKi = 0.001f;

struct DmXactHook
{
  void (*on_tx)(void *ctx, uint32_t can_id, const uint8_t *d, uint8_t n) = nullptr;
  void (*on_rx)(void *ctx, const CanRxFrame &rx) = nullptr;
  void (*on_reject)(void *ctx, const UsbCanRxStats &s) = nullptr;
  void (*on_send_fail)(void *ctx) = nullptr;
  void *ctx = nullptr;
};

struct DmMitEnableStatus
{
  LimitParam lim{};
  float pmax = 0.f;
  float vmax = 0.f;
  float tmax = 0.f;
  bool pmax_ok = false;
  bool vmax_ok = false;
  bool tmax_ok = false;
  bool rid_ok = false;
  bool ctrl_before_ok = false;
  bool wrote_ack = false;
  bool ctrl_after_ok = false;
  uint32_t ctrl_before = 0;
  uint32_t ctrl_after = 0;
  bool enable_sent = false;
};

enum class DmMitEnableErr : uint8_t
{
  Ok = 0,
  Rid,
  CtrlMode,
  EnableTx,
};

void dm_send_disable(UsbCanAdapter &ad, uint8_t id);
bool dm_send_enable(UsbCanAdapter &ad, uint8_t id);
// Writes firmware zero (current pos). MIT 0xFE. Does not enable. True if TX ok.
bool dm_send_set_zero(UsbCanAdapter &ad, uint8_t id);

bool dm_wait_register(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint8_t rid,
                      DmRegisterReply &reply, int timeout_ms, const DmXactHook *hook = nullptr);
bool dm_read_rid(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint8_t rid, float &value,
                 const DmXactHook *hook = nullptr);
bool dm_write_rid_u32(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint8_t rid,
                      uint32_t value, const DmXactHook *hook = nullptr);

bool dm_read_pvt_limits(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, LimitParam &lim,
                        DmMitEnableStatus *st = nullptr, const DmXactHook *hook = nullptr);
bool dm_set_ctrl_mode(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint32_t mode,
                      DmMitEnableStatus &st, const DmXactHook *hook = nullptr);
bool dm_set_ctrl_mode_mit(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id,
                          DmMitEnableStatus &st, const DmXactHook *hook = nullptr);

// require_pvt: knee/DM10422 path — refuse enable if RID 21/22/23 fail.
// Disable, write CTRL_MODE, verify, then 0xFC. Damiao needs disable before mode change.
DmMitEnableErr dm_enable_mode(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, uint32_t mode,
                              bool require_pvt, DmMitEnableStatus &st,
                              const DmXactHook *hook = nullptr);
DmMitEnableErr dm_enable_mit(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, bool require_pvt,
                             DmMitEnableStatus &st, const DmXactHook *hook = nullptr);

bool dm_probe_q0(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, const LimitParam &lim, float &q0,
                 int ticks = 20, int wait_ms = 40, uint32_t ctrl_mode = kDmMitMode);
bool evo_probe_q0(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, float &q0, int ticks = 20,
                  int wait_ms = 40);
bool dm_send_mit_hold(UsbCanAdapter &ad, uint8_t id, float kp, float kd, float q,
                      const LimitParam &lim);

enum class DmHomeResult : uint8_t
{
  Skipped = 0,  // |q| <= eps
  Homed = 1,
  ProbeFail = 2,
  Timeout = 3,
  AbortTau = 4,
};

struct DmHomeStatus
{
  float q_before = 0.f;
  float q_after = 0.f;
  float tau_peak = 0.f;
  DmHomeResult result = DmHomeResult::ProbeFail;
};

inline bool dm_home_ok(DmHomeResult r)
{
  return r == DmHomeResult::Skipped || r == DmHomeResult::Homed;
}

inline const char *dm_home_result_name(DmHomeResult r)
{
  switch (r)
  {
  case DmHomeResult::Skipped:
    return "skip";
  case DmHomeResult::Homed:
    return "homed";
  case DmHomeResult::ProbeFail:
    return "probe-fail";
  case DmHomeResult::Timeout:
    return "timeout";
  case DmHomeResult::AbortTau:
    return "abort-tau";
  }
  return "?";
}

// After 0xFC. Probe q (not 0xFE). If |q|>eps, MIT host-dq toward 0: kp=0,
// kd=home_kd, dq_des=-sign(q)*home_vel, q_des=0, tau_ff=0 until |q|<=eps,
// timeout, or |tau|>abort_tau. Timeout = max(timeout_s, |q|/min(vel,1)+slack)
// capped at kDmHomeTimeoutMaxS (plant may track slower than commanded vel).
// Does not 0xFD.
DmHomeResult dm_home_to_zero(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id,
                             const LimitParam &lim, float abort_tau,
                             float timeout_s = kDmHomeTimeoutS, float eps = kDmHomeEpsRad,
                             DmHomeStatus *st = nullptr, float home_vel = kDmHomeVel,
                             float home_kd = kDmHomeKd);
// Motorevo analog: servo θ_ref=0 with V_ref=home_vel (speed limit, not a step).
DmHomeResult evo_home_to_zero(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, float abort_tau,
                              float timeout_s = kDmHomeTimeoutS, float eps = kDmHomeEpsRad,
                              DmHomeStatus *st = nullptr,
                              float home_vel = kEvoHomeVelLimit);
bool dm_send_pos_vel(UsbCanAdapter &ad, uint8_t id, float pos, float vel);
bool dm_send_vel(UsbCanAdapter &ad, uint8_t id, float vel);
bool dm_send_pos_force(UsbCanAdapter &ad, uint8_t id, float pos, float vel_limit,
                       float torque_ratio);

inline void dm_pack_hold(uint8_t out[8], float kp, float kd, float q, const LimitParam &lim)
{
  pack_mit(out, kp, kd, q, 0.f, 0.f, lim);
}

// Host-like velocity packet: kp=0, q_des=0, dq_des=vel (caller already VMAX-capped).
inline void dm_pack_host_dq(uint8_t out[8], float kd, float dq_des, float tau,
                            const LimitParam &lim)
{
  pack_mit(out, 0.f, kd, 0.f, dq_des, tau, lim);
}

}  // namespace biped_hw
