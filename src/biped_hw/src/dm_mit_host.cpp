// biped_hw — dm_mit_host.cpp
// 0xFC/0xFD/0xFE, RID read/write, CTRL_MODE, q0 probe, MIT hold / host-dq home /
// POS_VEL / VEL / POS_FORCE. Evo probe/home share sendCan.
#include "biped_hw/dm_mit_host.hpp"

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/motorevo_codec.hpp"

#include <chrono>
#include <cmath>
#include <unistd.h>

namespace biped_hw
{
namespace
{

void hook_tx(const DmXactHook *hook, uint32_t can_id, const uint8_t *d, uint8_t n)
{
  if (hook != nullptr && hook->on_tx != nullptr)
  {
    hook->on_tx(hook->ctx, can_id, d, n);
  }
}

void hook_rx(const DmXactHook *hook, const CanRxFrame &rx)
{
  if (hook != nullptr && hook->on_rx != nullptr)
  {
    hook->on_rx(hook->ctx, rx);
  }
}

void hook_reject(const DmXactHook *hook, const UsbCanRxStats &s)
{
  if (hook != nullptr && hook->on_reject != nullptr)
  {
    hook->on_reject(hook->ctx, s);
  }
}

void hook_send_fail(const DmXactHook *hook)
{
  if (hook != nullptr && hook->on_send_fail != nullptr)
  {
    hook->on_send_fail(hook->ctx);
  }
}

float clamp_home_vel(float home_vel, float vmax)
{
  float vel = home_vel > 0.f ? home_vel : kDmHomeVel;
  if (vel > kDmHomeVelMax)
  {
    vel = kDmHomeVelMax;
  }
  if (vmax > 0.1f && vel > vmax)
  {
    vel = vmax;
  }
  return vel;
}

float home_timeout_s(float q_abs, float vel, float timeout_s)
{
  // Commanded vel may be 1–3; the plant often tracks nearer kDmHomeVel.
  // Size the deadline from min(vel, kDmHomeVel) so a 12 rad rewind at ~1 rad/s
  // is not cut off by |q|/3 + slack.
  float v_time = vel > 0.f ? vel : kDmHomeVel;
  if (v_time > kDmHomeVel)
  {
    v_time = kDmHomeVel;
  }
  const float need = q_abs / v_time + kDmHomeTimeoutSlackS;
  float t = timeout_s > kDmHomeTimeoutS ? timeout_s : kDmHomeTimeoutS;
  if (need > t)
  {
    t = need;
  }
  if (t > kDmHomeTimeoutMaxS)
  {
    t = kDmHomeTimeoutMaxS;
  }
  return t;
}

}  // namespace

void dm_send_disable(UsbCanAdapter &ad, uint8_t id)
{
  if (!ad.is_open())
  {
    return;
  }
  uint8_t buf[8] = {};
  pack_disable(buf);
  ad.sendCan(id, buf, 8);
  usleep(kDmDisableGapUs);
  ad.sendCan(id, buf, 8);
}

bool dm_send_enable(UsbCanAdapter &ad, uint8_t id)
{
  if (!ad.is_open())
  {
    return false;
  }
  uint8_t buf[8] = {};
  pack_enable(buf);
  return ad.sendCan(id, buf, 8);
}

bool dm_send_set_zero(UsbCanAdapter &ad, uint8_t id)
{
  if (!ad.is_open())
  {
    return false;
  }
  uint8_t buf[8] = {};
  pack_set_zero(buf);
  const bool ok = ad.sendCan(id, buf, 8);
  usleep(kDmDisableGapUs);
  return ok;
}

bool dm_wait_register(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint8_t rid,
                      DmRegisterReply &reply, int timeout_ms, const DmXactHook *hook)
{
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    const uint32_t bad0 = ad.rx_stats().bad_cmd;
    CanRxFrame rx;
    if (ad.pollCan(rx))
    {
      hook_rx(hook, rx);
      dm.push(rx);
    }
    else if (ad.rx_stats().bad_cmd > bad0)
    {
      hook_reject(hook, ad.rx_stats());
    }
    if (dm.take_register(esc_id, rid, reply))
    {
      return true;
    }
  }
  return false;
}

bool dm_read_rid(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint8_t rid, float &value,
                 const DmXactHook *hook)
{
  for (int i = 0; i < kDmRidRetries; ++i)
  {
    dm.clear();
    uint8_t buf[8] = {};
    pack_register_read(buf, esc_id, rid);
    hook_tx(hook, kDmRegCanId, buf, 8);
    if (!ad.sendCan(kDmRegCanId, buf, 8))
    {
      hook_send_fail(hook);
      continue;
    }
    DmRegisterReply reply{};
    if (dm_wait_register(ad, dm, esc_id, rid, reply, kDmRidTimeoutMs, hook))
    {
      value = reply.f;
      return true;
    }
  }
  return false;
}

bool dm_write_rid_u32(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint8_t rid,
                      uint32_t value, const DmXactHook *hook)
{
  for (int i = 0; i < kDmRidRetries; ++i)
  {
    dm.clear();
    uint8_t buf[8] = {};
    pack_register_write_u32(buf, esc_id, rid, value);
    hook_tx(hook, kDmRegCanId, buf, 8);
    if (!ad.sendCan(kDmRegCanId, buf, 8))
    {
      hook_send_fail(hook);
      continue;
    }
    DmRegisterReply reply{};
    if (dm_wait_register(ad, dm, esc_id, rid, reply, kDmRidTimeoutMs, hook))
    {
      if (reply.u == value || static_cast<uint32_t>(reply.f + 0.5f) == value)
      {
        return true;
      }
    }
  }
  return false;
}

bool dm_read_pvt_limits(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, LimitParam &lim,
                        DmMitEnableStatus *st, const DmXactHook *hook)
{
  float pmax = 0.f, vmax = 0.f, tmax = 0.f;
  const bool pok = dm_read_rid(ad, dm, esc_id, static_cast<uint8_t>(DmRid::Pmax), pmax, hook);
  const bool vok = dm_read_rid(ad, dm, esc_id, static_cast<uint8_t>(DmRid::Vmax), vmax, hook);
  const bool tok = dm_read_rid(ad, dm, esc_id, static_cast<uint8_t>(DmRid::Tmax), tmax, hook);
  const bool ok = pok && vok && tok && pmax > 0.1f && vmax > 0.1f && tmax > 0.1f;
  if (st != nullptr)
  {
    st->pmax = pmax;
    st->vmax = vmax;
    st->tmax = tmax;
    st->pmax_ok = pok;
    st->vmax_ok = vok;
    st->tmax_ok = tok;
    st->rid_ok = ok;
  }
  if (!ok)
  {
    return false;
  }
  lim = LimitParam{pmax, vmax, tmax};
  if (st != nullptr)
  {
    st->lim = lim;
  }
  return true;
}

bool dm_set_ctrl_mode(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id, uint32_t mode,
                      DmMitEnableStatus &st, const DmXactHook *hook)
{
  if (!dm_ctrl_mode_ok(mode))
  {
    return false;
  }
  float mode_f = 0.f;
  st.ctrl_before_ok =
      dm_read_rid(ad, dm, esc_id, static_cast<uint8_t>(DmRid::CtrlMode), mode_f, hook);
  st.ctrl_before = static_cast<uint32_t>(mode_f + 0.5f);
  st.wrote_ack = dm_write_rid_u32(ad, dm, esc_id, static_cast<uint8_t>(DmRid::CtrlMode),
                                  mode, hook);
  usleep(kDmCtrlModeSettleUs);
  mode_f = 0.f;
  st.ctrl_after_ok =
      dm_read_rid(ad, dm, esc_id, static_cast<uint8_t>(DmRid::CtrlMode), mode_f, hook);
  st.ctrl_after = static_cast<uint32_t>(mode_f + 0.5f);
  return st.ctrl_after_ok && st.ctrl_after == mode;
}

bool dm_set_ctrl_mode_mit(UsbCanAdapter &ad, FrameDemux &dm, uint8_t esc_id,
                          DmMitEnableStatus &st, const DmXactHook *hook)
{
  return dm_set_ctrl_mode(ad, dm, esc_id, kDmMitMode, st, hook);
}

DmMitEnableErr dm_enable_mode(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, uint32_t mode,
                              bool require_pvt, DmMitEnableStatus &st, const DmXactHook *hook)
{
  st = {};
  if (require_pvt)
  {
    if (!dm_read_pvt_limits(ad, dm, id, st.lim, &st, hook))
    {
      return DmMitEnableErr::Rid;
    }
  }
  dm_send_disable(ad, id);
  usleep(kDmDisableGapUs);
  ad.drain();
  dm.clear();
  if (!dm_set_ctrl_mode(ad, dm, id, mode, st, hook))
  {
    return DmMitEnableErr::CtrlMode;
  }
  st.enable_sent = dm_send_enable(ad, id);
  if (!st.enable_sent)
  {
    return DmMitEnableErr::EnableTx;
  }
  return DmMitEnableErr::Ok;
}

DmMitEnableErr dm_enable_mit(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, bool require_pvt,
                             DmMitEnableStatus &st, const DmXactHook *hook)
{
  return dm_enable_mode(ad, dm, id, kDmMitMode, require_pvt, st, hook);
}

bool dm_probe_q0(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, const LimitParam &lim, float &q0,
                 int ticks, int wait_ms, uint32_t ctrl_mode)
{
  uint8_t cmd[8] = {};
  const uint32_t tx_id = (ctrl_mode == kDmVelMode) ? dm_cmd_can_id(kDmVelMode, id)
                                                   : static_cast<uint32_t>(id);
  if (ctrl_mode == kDmVelMode)
  {
    pack_vel(cmd, 0.f);
  }
  else if (ctrl_mode == kDmPosVelMode || ctrl_mode == kDmPosForceMode)
  {
    pack_enable(cmd);
  }
  else
  {
    pack_mit(cmd, 0.f, 0.f, 0.f, 0.f, 0.f, lim);
  }
  for (int i = 0; i < ticks; ++i)
  {
    dm.clear();
    if (!ad.sendCan(tx_id, cmd, 8))
    {
      continue;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
      CanRxFrame rx;
      if (ad.pollCan(rx))
      {
        dm.push(rx);
      }
      DmFeedback fb{};
      if (dm.take_mit(id, lim, fb))
      {
        q0 = fb.q;
        return true;
      }
    }
  }
  return false;
}

bool evo_probe_q0(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, float &q0, int ticks,
                  int wait_ms)
{
  uint8_t cmd[8] = {};
  pack_motorevo_servo(cmd, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
  for (int i = 0; i < ticks; ++i)
  {
    dm.clear();
    if (!ad.sendCan(id, cmd, 8))
    {
      continue;
    }
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
      CanRxFrame rx;
      if (ad.pollCan(rx))
      {
        dm.push(rx);
      }
      MotorevoFeedback fb{};
      if (dm.take_motorevo(id, fb))
      {
        q0 = fb.q;
        return true;
      }
    }
  }
  return false;
}

DmHomeResult dm_home_to_zero(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id,
                             const LimitParam &lim, float abort_tau, float timeout_s,
                             float eps, DmHomeStatus *st, float home_vel, float home_kd)
{
  if (st != nullptr)
  {
    *st = {};
  }
  float q0 = 0.f;
  if (!dm_probe_q0(ad, dm, id, lim, q0))
  {
    if (st != nullptr)
    {
      st->result = DmHomeResult::ProbeFail;
    }
    return DmHomeResult::ProbeFail;
  }
  if (st != nullptr)
  {
    st->q_before = q0;
    st->q_after = q0;
  }
  if (std::fabs(q0) <= eps)
  {
    if (st != nullptr)
    {
      st->result = DmHomeResult::Skipped;
    }
    return DmHomeResult::Skipped;
  }

  const float vel = clamp_home_vel(home_vel, lim.dq_max);
  const float kd = home_kd > 0.f ? home_kd : kDmHomeKd;
  float q = q0;
  float tau_peak = 0.f;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<float>(home_timeout_s(std::fabs(q0), vel, timeout_s));
  while (std::chrono::steady_clock::now() < deadline)
  {
    uint8_t cmd[8] = {};
    dm_pack_host_dq(cmd, kd, -std::copysign(vel, q), 0.f, lim);
    (void)ad.sendCan(id, cmd, 8);
    const auto tick_end = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kDmHomePeriodMs);
    while (std::chrono::steady_clock::now() < tick_end)
    {
      CanRxFrame rx;
      if (ad.pollCan(rx))
      {
        dm.push(rx);
      }
      DmFeedback fb{};
      if (!dm.take_mit(id, lim, fb))
      {
        continue;
      }
      q = fb.q;
      const float at = std::fabs(fb.tau);
      if (at > tau_peak)
      {
        tau_peak = at;
      }
      if (st != nullptr)
      {
        st->q_after = q;
        st->tau_peak = tau_peak;
      }
      if (at > abort_tau)
      {
        if (st != nullptr)
        {
          st->result = DmHomeResult::AbortTau;
        }
        return DmHomeResult::AbortTau;
      }
      if (std::fabs(q) <= eps)
      {
        uint8_t stop[8] = {};
        dm_pack_host_dq(stop, kd, 0.f, 0.f, lim);
        for (int i = 0; i < 5; ++i)
        {
          (void)ad.sendCan(id, stop, 8);
          usleep(static_cast<useconds_t>(kDmHomePeriodMs * 1000));
        }
        if (st != nullptr)
        {
          st->result = DmHomeResult::Homed;
        }
        return DmHomeResult::Homed;
      }
    }
  }
  if (st != nullptr)
  {
    st->q_after = q;
    st->tau_peak = tau_peak;
    st->result = DmHomeResult::Timeout;
  }
  return DmHomeResult::Timeout;
}

DmHomeResult evo_home_to_zero(UsbCanAdapter &ad, FrameDemux &dm, uint8_t id, float abort_tau,
                              float timeout_s, float eps, DmHomeStatus *st, float home_vel)
{
  if (st != nullptr)
  {
    *st = {};
  }
  float q0 = 0.f;
  if (!evo_probe_q0(ad, dm, id, q0))
  {
    if (st != nullptr)
    {
      st->result = DmHomeResult::ProbeFail;
    }
    return DmHomeResult::ProbeFail;
  }
  if (st != nullptr)
  {
    st->q_before = q0;
    st->q_after = q0;
  }
  if (std::fabs(q0) <= eps)
  {
    if (st != nullptr)
    {
      st->result = DmHomeResult::Skipped;
    }
    return DmHomeResult::Skipped;
  }

  const float vel = clamp_home_vel(home_vel > 0.f ? home_vel : kEvoHomeVelLimit, kMotorevoDqMax);
  uint8_t cmd[8] = {};
  pack_motorevo_servo(cmd, 0.f, kEvoHomeKp, kEvoHomeKd, vel, kEvoHomeVelKp, kEvoHomeVelKd,
                      kEvoHomeVelKi);
  float q = q0;
  float tau_peak = 0.f;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<float>(home_timeout_s(std::fabs(q0), vel, timeout_s));
  while (std::chrono::steady_clock::now() < deadline)
  {
    (void)ad.sendCan(id, cmd, 8);
    const auto tick_end = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kDmHomePeriodMs);
    while (std::chrono::steady_clock::now() < tick_end)
    {
      CanRxFrame rx;
      if (ad.pollCan(rx))
      {
        dm.push(rx);
      }
      MotorevoFeedback fb{};
      if (!dm.take_motorevo(id, fb))
      {
        continue;
      }
      q = fb.q;
      const float at = std::fabs(fb.tau);
      if (at > tau_peak)
      {
        tau_peak = at;
      }
      if (st != nullptr)
      {
        st->q_after = q;
        st->tau_peak = tau_peak;
      }
      if (at > abort_tau)
      {
        if (st != nullptr)
        {
          st->result = DmHomeResult::AbortTau;
        }
        return DmHomeResult::AbortTau;
      }
      if (std::fabs(q) <= eps)
      {
        if (st != nullptr)
        {
          st->result = DmHomeResult::Homed;
        }
        return DmHomeResult::Homed;
      }
    }
  }
  if (st != nullptr)
  {
    st->q_after = q;
    st->tau_peak = tau_peak;
    st->result = DmHomeResult::Timeout;
  }
  return DmHomeResult::Timeout;
}

bool dm_send_mit_hold(UsbCanAdapter &ad, uint8_t id, float kp, float kd, float q,
                      const LimitParam &lim)
{
  if (!ad.is_open())
  {
    return false;
  }
  uint8_t mit[8] = {};
  dm_pack_hold(mit, kp, kd, q, lim);
  return ad.sendCan(id, mit, 8);
}

bool dm_send_pos_vel(UsbCanAdapter &ad, uint8_t id, float pos, float vel)
{
  if (!ad.is_open())
  {
    return false;
  }
  uint8_t buf[8] = {};
  pack_pos_vel(buf, pos, vel);
  return ad.sendCan(dm_cmd_can_id(kDmPosVelMode, id), buf, 8);
}

bool dm_send_vel(UsbCanAdapter &ad, uint8_t id, float vel)
{
  if (!ad.is_open())
  {
    return false;
  }
  uint8_t buf[8] = {};
  pack_vel(buf, vel);
  return ad.sendCan(dm_cmd_can_id(kDmVelMode, id), buf, 8);
}

bool dm_send_pos_force(UsbCanAdapter &ad, uint8_t id, float pos, float vel_limit,
                       float torque_ratio)
{
  if (!ad.is_open())
  {
    return false;
  }
  uint8_t buf[8] = {};
  pack_pos_force(buf, pos, vel_limit, torque_ratio);
  return ad.sendCan(dm_cmd_can_id(kDmPosForceMode, id), buf, 8);
}

}  // namespace biped_hw
