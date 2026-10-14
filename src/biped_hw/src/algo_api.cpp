// biped_hw — algo_api.cpp
// getState：MIT fb / 可选 MBS 8 / 可选 IMU latch。缺则 valid=false。
// sendCommand：电机 q_des，或髋 roll/pitch → hip_ik（CAN 4/5、10/11）。
#include "biped_hw/algo_api.hpp"

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/mbs_codec.hpp"
#include "biped_hw/mbs_host.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/quintic_traj.hpp"
#include "biped_hw/safety.hpp"
#include "biped_hw/serial_port.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <cmath>

namespace biped_hw
{
namespace
{

LimitParam lim_for(const AlgoSources &src, const MotorMapEntry &e)
{
  const LimitParam &slot = src.lims[e.can_id & 0x0F];
  if (slot.q_max > 0.1f)
  {
    return slot;
  }
  return default_limit(e.model);
}

void poll_bus(UsbCanAdapter *ad, FrameDemux *demux, RobotState &st, BusId bus, SteadyNs t_rx,
              const AlgoSources &src)
{
  if (ad == nullptr || demux == nullptr || !ad->is_open())
  {
    return;
  }
  CanRxFrame rx;
  int n = 0;
  while (n < 32 && ad->pollCan(rx))
  {
    demux->push(rx);
    ++n;
  }

  bool wanted[16] = {};
  LimitParam lims[16] = {};
  for (const auto &e : kMotorMap)
  {
    if (e.bus != bus || e.model == MotorType::HA60)
    {
      continue;
    }
    wanted[e.can_id & 0x0F] = true;
    lims[e.can_id & 0x0F] = lim_for(src, e);
  }
  DmFeedback fb{};
  while (demux->take_wanted_mit(wanted, lims, fb))
  {
    (void)apply_motor(st, fb, t_rx);
    wanted[fb.esc_id & 0x0F] = false;
  }

  for (const auto &e : kMotorMap)
  {
    if (e.bus != bus || e.model != MotorType::HA60)
    {
      continue;
    }
    MotorevoFeedback mf{};
    if (!demux->take_motorevo(e.can_id, mf))
    {
      continue;
    }
    DmFeedback as_dm{};
    as_dm.esc_id = e.can_id;
    as_dm.q = mf.q;
    as_dm.dq = mf.dq;
    as_dm.tau = mf.tau;
    as_dm.err = mf.err_code;
    as_dm.rx_can_id = mf.rx_can_id;
    as_dm.dlc = mf.dlc;
    (void)apply_motor(st, as_dm, t_rx);
  }
}

void apply_hip(RobotCommand &cmd, HipSide side, const HipCommand &hip, bool &ok)
{
  if (!hip.valid)
  {
    return;
  }
  HipMotors q{};
  if (!hip_ik(side, hip.pose, &q))
  {
    ok = false;
    return;
  }
  const uint8_t outer = (side == HipSide::Left) ? 4 : 10;
  const uint8_t inner = (side == HipSide::Left) ? 5 : 11;
  const int io = motor_index_by_can_id(outer);
  const int ii = motor_index_by_can_id(inner);
  if (io < 0 || ii < 0)
  {
    ok = false;
    return;
  }
  auto &mo = cmd.motors[static_cast<std::size_t>(io)];
  auto &mi = cmd.motors[static_cast<std::size_t>(ii)];
  mo.active = true;
  mi.active = true;
  mo.q_des = static_cast<float>(q.q_outer);
  mi.q_des = static_cast<float>(q.q_inner);
}

UsbCanAdapter *adapter_for(AlgoSources *src, BusId bus)
{
  if (src == nullptr)
  {
    return nullptr;
  }
  UsbCanAdapter *ad = (bus == BusId::Right) ? src->right : src->left;
  if (ad == nullptr || !ad->is_open())
  {
    return nullptr;
  }
  return ad;
}

}  // namespace

void getState(AlgoSources &src, RobotState &st)
{
  ++st.cycle;
  const SteadyNs t_rx = now_steady_ns();
  poll_bus(src.left, src.left_demux, st, BusId::Left, t_rx, src);
  poll_bus(src.right, src.right_demux, st, BusId::Right, t_rx, src);

  if (src.poll_mbs && src.mbs != nullptr && src.mbs->is_open())
  {
    for (auto &slot : st.mbs)
    {
      slot.valid = false;
      MbsSample sample{};
      if (!mbs_bus_poll_once(*src.mbs, slot.addr, src.mbs_timeout_ms, src.mbs_rts_de, sample))
      {
        continue;
      }
      (void)apply_mbs(st, slot.addr, sample.angle_rad, sample.total_rad, sample.angle_raw,
                      sample.has_status, sample.status.raw, t_rx);
    }
  }

  if (src.imu_latch != nullptr && src.imu_latch->valid)
  {
    st.imu = *src.imu_latch;
  }

  stamp_cycle(st, t_rx);
}

SendResult sendCommand(AlgoSources *src, const RobotCommand &cmd_in)
{
  SendResult out{};
  RobotCommand cmd = cmd_in;
  apply_hip(cmd, HipSide::Left, cmd.hip_left, out.hip_ik_ok);
  apply_hip(cmd, HipSide::Right, cmd.hip_right, out.hip_ik_ok);

  AlgoSources empty{};
  const AlgoSources &lim_src = (src != nullptr) ? *src : empty;

  for (std::size_t i = 0; i < kMotorMap.size(); ++i)
  {
    const auto &sp = cmd.motors[i];
    if (!sp.active)
    {
      continue;
    }
    const auto &e = kMotorMap[i];
    PackedMotorCmd p{};
    p.can_id = e.can_id;
    p.bus = e.bus;
    p.model = e.model;
    PidTrackGains g{};
    g.kp = sp.kp;
    g.kd = sp.kd;
    g.tau_ff = sp.tau_ff;
    SafetyLimitHit hit = SafetyLimitHit::Ok;
    const float q_cmd = safety_hold_q(e.can_id, sp.q_des, &hit);
    (void)hit;
    pid_track_pack(p.data, e.model, q_cmd, sp.dq_des, g, lim_for(lim_src, e));
    UsbCanAdapter *ad = adapter_for(src, e.bus);
    if (ad != nullptr)
    {
      p.sent = ad->sendCan(e.can_id, p.data, 8);
    }
    if (out.n < kMotorCount)
    {
      out.packed[out.n++] = p;
    }
  }
  return out;
}

int algo_api_self_test()
{
  const int qrc = quintic_traj_self_test();
  if (qrc != 0)
  {
    return 20 + qrc;
  }
  const int ptrc = pid_track_self_test();
  if (ptrc != 0)
  {
    return ptrc;
  }

  RobotState st;
  init_robot_state(st);
  AlgoSources src{};
  getState(src, st);
  if (motors_valid_count(st) != 0 || mbs_valid_count(st) != 0 || st.imu.valid)
  {
    return 10;
  }
  if (st.motors[9].can_id != 12 || st.motors[9].valid)
  {
    return 11;
  }

  RobotCommand cmd{};
  const int i12 = motor_index_by_can_id(12);
  if (i12 < 0)
  {
    return 12;
  }
  auto &s12 = cmd.motors[static_cast<std::size_t>(i12)];
  s12.active = true;
  s12.q_des = 1.25f;
  s12.dq_des = 0.1f;
  s12.kp = 8.f;
  s12.kd = 0.5f;
  const SendResult r = sendCommand(nullptr, cmd);
  if (r.n != 1 || r.packed[0].can_id != 12 || r.packed[0].sent ||
      r.packed[0].data[7] == kDmEnableCmd)
  {
    return 13;
  }
  float kp_w = 0.f, kd_w = 0.f, q_w = 0.f, dq_w = 0.f, tau_w = 0.f;
  unpack_mit_cmd(r.packed[0].data, kLimitDm10422, kp_w, kd_w, q_w, dq_w, tau_w);
  if (!std::isfinite(q_w) || std::fabs(q_w - 1.25f) > 0.01f || std::fabs(kp_w - 8.f) > 1.f)
  {
    return 14;
  }

  HipMotors want{};
  HipPose pose{};
  if (!hip_ik(HipSide::Right, pose, &want))
  {
    return 15;
  }
  RobotCommand hip{};
  hip.hip_right.valid = true;
  hip.hip_right.pose = pose;
  const SendResult rh = sendCommand(nullptr, hip);
  if (!rh.hip_ik_ok || rh.n != 2)
  {
    return 16;
  }
  bool got10 = false;
  bool got11 = false;
  for (std::size_t i = 0; i < rh.n; ++i)
  {
    float kp = 0.f, kd = 0.f, q = 0.f, dq = 0.f, tau = 0.f;
    unpack_mit_cmd(rh.packed[i].data, kLimitDm10010, kp, kd, q, dq, tau);
    if (!std::isfinite(q))
    {
      return 17;
    }
    if (rh.packed[i].can_id == 10)
    {
      got10 = std::fabs(static_cast<double>(q) - want.q_outer) < 0.02;
    }
    if (rh.packed[i].can_id == 11)
    {
      got11 = std::fabs(static_cast<double>(q) - want.q_inner) < 0.02;
    }
  }
  if (!got10 || !got11)
  {
    return 18;
  }
  return 0;
}

}  // namespace biped_hw
