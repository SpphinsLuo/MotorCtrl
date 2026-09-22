// biped_hw — robot_state.hpp
// P4 统一状态：12 电机反馈 + 8 关节 MBS + N100 IMU，同一 steady 周期戳。
// 缺传感器时 valid=false，禁止填假数据。无髋平行机构 FK。
#pragma once

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/mbs_map.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>

namespace biped_hw
{

using SteadyNs = std::int64_t;

inline SteadyNs now_steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

inline double age_ms(SteadyNs t_event, SteadyNs t_cycle)
{
  if (t_event == 0)
  {
    return 0.0;
  }
  return static_cast<double>(t_cycle - t_event) * 1e-6;
}

// One motor slot. Index matches kMotorMap. valid stays false until a real RX.
struct MotorFeedback
{
  uint8_t can_id = 0;
  BusId bus = BusId::Left;
  JointId joint = JointId::LHipYaw;
  MotorType model = MotorType::DM10010;
  bool valid = false;
  float q = 0.f;
  float dq = 0.f;
  float tau = 0.f;
  uint8_t err = 0;
  bool has_temp = false;
  uint8_t t_mos = 0;
  uint8_t t_coil = 0;
  SteadyNs t_rx_ns = 0;
  double rx_age_ms = 0.0;
};

// One joint encoder slot. Index matches kMbsEncoderMap. Hip yaw / ankle
// roll are not in this array. valid stays false until a real RX for addr.
struct MbsFeedback
{
  bool valid = false;
  uint8_t addr = 0;
  const char *name = "";
  uint32_t raw = 0;
  double angle_rad = 0.0;
  double total_rad = 0.0;
  double zero_rad = 0.0;
  int8_t sign = 1;
  bool has_status = false;
  uint8_t status_raw = 0;
  SteadyNs t_rx_ns = 0;
  double rx_age_ms = 0.0;
};

// N100 via ROS /imu. header_* is the message stamp; t_recv_ns is local latch.
struct ImuFeedback
{
  bool valid = false;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  SteadyNs t_recv_ns = 0;
  std::int32_t header_sec = 0;
  std::uint32_t header_nsec = 0;
  char frame_id[32] = {};
  double recv_age_ms = 0.0;
};

struct RobotState
{
  SteadyNs t_cycle_ns = 0;
  std::uint64_t cycle = 0;
  std::array<MotorFeedback, kMotorCount> motors{};
  std::array<MbsFeedback, kMbsEncoderCount> mbs{};
  ImuFeedback imu{};
};

inline void init_robot_state(RobotState &st)
{
  st = {};
  for (std::size_t i = 0; i < kMotorMap.size(); ++i)
  {
    const auto &e = kMotorMap[i];
    auto &m = st.motors[i];
    m.can_id = e.can_id;
    m.bus = e.bus;
    m.joint = e.joint;
    m.model = e.model;
    m.valid = false;
  }
  for (std::size_t i = 0; i < kMbsEncoderMap.size(); ++i)
  {
    const auto &e = kMbsEncoderMap[i];
    auto &s = st.mbs[i];
    s.addr = e.addr;
    s.name = e.name;
    s.zero_rad = e.zero_rad;
    s.sign = e.sign;
    s.valid = false;
  }
}

inline void overlay_mbs_map(RobotState &st, const MbsEncoderConfig &cfg)
{
  for (std::size_t i = 0; i < kMbsEncoderCount; ++i)
  {
    auto &s = st.mbs[i];
    s.addr = cfg.encoders[i].addr;
    s.zero_rad = cfg.encoders[i].zero_rad;
    s.sign = cfg.encoders[i].sign;
  }
}

inline void stamp_cycle(RobotState &st, SteadyNs t_cycle)
{
  st.t_cycle_ns = t_cycle;
  for (auto &m : st.motors)
  {
    m.rx_age_ms = m.valid ? age_ms(m.t_rx_ns, t_cycle) : 0.0;
  }
  for (auto &e : st.mbs)
  {
    e.rx_age_ms = e.valid ? age_ms(e.t_rx_ns, t_cycle) : 0.0;
  }
  st.imu.recv_age_ms = st.imu.valid ? age_ms(st.imu.t_recv_ns, t_cycle) : 0.0;
}

inline int motor_index_by_can_id(uint8_t can_id)
{
  for (std::size_t i = 0; i < kMotorMap.size(); ++i)
  {
    if (kMotorMap[i].can_id == can_id)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

inline bool apply_motor(RobotState &st, const DmFeedback &fb, SteadyNs t_rx)
{
  const int idx = motor_index_by_can_id(fb.esc_id);
  if (idx < 0)
  {
    return false;
  }
  auto &m = st.motors[static_cast<std::size_t>(idx)];
  m.valid = true;
  m.q = fb.q;
  m.dq = fb.dq;
  m.tau = fb.tau;
  m.err = fb.err;
  m.has_temp = fb.has_temp;
  m.t_mos = fb.t_mos;
  m.t_coil = fb.t_coil;
  m.t_rx_ns = t_rx;
  return true;
}

inline int mbs_index_by_addr(const RobotState &st, uint8_t addr)
{
  for (std::size_t i = 0; i < st.mbs.size(); ++i)
  {
    if (st.mbs[i].addr == addr)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// Codec angle/total in; slot stores sign * (value - zero_rad). Defaults are
// identity. Unmapped addr (factory 0x1F, hip yaw, ankle roll) → false.
inline bool apply_mbs(RobotState &st, uint8_t addr, double angle_rad, double total_rad,
                      uint32_t raw, bool has_status, uint8_t status_raw, SteadyNs t_rx)
{
  const int idx = mbs_index_by_addr(st, addr);
  if (idx < 0)
  {
    return false;
  }
  auto &e = st.mbs[static_cast<std::size_t>(idx)];
  const double s = static_cast<double>(e.sign);
  e.valid = true;
  e.raw = raw;
  e.angle_rad = s * (angle_rad - e.zero_rad);
  e.total_rad = s * (total_rad - e.zero_rad);
  e.has_status = has_status;
  e.status_raw = status_raw;
  e.t_rx_ns = t_rx;
  return true;
}

inline void apply_imu(RobotState &st, SteadyNs t_recv, std::int32_t header_sec,
                      std::uint32_t header_nsec, const char *frame_id, double qw,
                      double qx, double qy, double qz, double gx, double gy, double gz,
                      double ax, double ay, double az)
{
  st.imu.valid = true;
  st.imu.t_recv_ns = t_recv;
  st.imu.header_sec = header_sec;
  st.imu.header_nsec = header_nsec;
  st.imu.qw = qw;
  st.imu.qx = qx;
  st.imu.qy = qy;
  st.imu.qz = qz;
  st.imu.gx = gx;
  st.imu.gy = gy;
  st.imu.gz = gz;
  st.imu.ax = ax;
  st.imu.ay = ay;
  st.imu.az = az;
  st.imu.frame_id[0] = '\0';
  if (frame_id != nullptr)
  {
    std::strncpy(st.imu.frame_id, frame_id, sizeof(st.imu.frame_id) - 1);
    st.imu.frame_id[sizeof(st.imu.frame_id) - 1] = '\0';
  }
}

inline std::size_t motors_valid_count(const RobotState &st)
{
  std::size_t n = 0;
  for (const auto &m : st.motors)
  {
    if (m.valid)
    {
      ++n;
    }
  }
  return n;
}

inline std::size_t mbs_valid_count(const RobotState &st)
{
  std::size_t n = 0;
  for (const auto &e : st.mbs)
  {
    if (e.valid)
    {
      ++n;
    }
  }
  return n;
}

// Offline mapper check. Uses constructed codec values, not live hardware.
inline int robot_state_self_test()
{
  RobotState st;
  init_robot_state(st);
  if (st.motors.size() != kMotorCount || st.motors[3].can_id != 6)
  {
    return 1;
  }
  if (st.mbs.size() != kMbsEncoderCount || st.mbs[0].addr != 4 ||
      st.mbs[7].addr != 13 || st.mbs[0].sign != 1)
  {
    return 2;
  }
  if (mbs_index_by_addr(st, 0x1F) >= 0 || mbs_index_by_addr(st, 3) >= 0 ||
      mbs_index_by_addr(st, 8) >= 0)
  {
    return 2;
  }
  if (st.mbs[0].valid || st.imu.valid || st.motors[3].valid ||
      mbs_valid_count(st) != 0)
  {
    return 2;
  }

  DmFeedback fb{};
  fb.esc_id = 6;
  fb.q = 1.25f;
  fb.dq = 0.1f;
  fb.tau = 2.f;
  fb.err = 0;
  fb.has_temp = true;
  fb.t_mos = 33;
  fb.t_coil = 28;
  const SteadyNs t_rx = 1'000'000'000;
  if (!apply_motor(st, fb, t_rx) || !st.motors[3].valid || st.motors[3].q != 1.25f)
  {
    return 3;
  }

  fb.esc_id = 99;
  if (apply_motor(st, fb, t_rx))
  {
    return 4;
  }

  if (apply_mbs(st, 0x1F, 1.5707963267948966, 1.5707963267948966, 0x10000u, true, 0,
                t_rx) ||
      st.mbs[0].valid)
  {
    return 5;
  }
  if (!apply_mbs(st, 4, 1.5707963267948966, 1.5707963267948966, 0x10000u, true, 0,
                 t_rx) ||
      !st.mbs[0].valid || st.mbs[0].addr != 4 || st.mbs[0].raw != 0x10000u ||
      st.mbs[1].valid)
  {
    return 5;
  }
  st.mbs[0].valid = false;
  if (mbs_valid_count(st) != 0)
  {
    return 5;
  }

  apply_imu(st, t_rx + 1000, 1, 2, "imu", 1.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 9.8);
  if (!st.imu.valid || st.imu.header_sec != 1 || std::strcmp(st.imu.frame_id, "imu") != 0)
  {
    return 6;
  }

  stamp_cycle(st, t_rx + 5'000'000);
  if (st.motors[3].rx_age_ms < 4.9 || st.motors[3].rx_age_ms > 5.1)
  {
    return 7;
  }
  if (motors_valid_count(st) != 1)
  {
    return 8;
  }
  return 0;
}

}  // namespace biped_hw
