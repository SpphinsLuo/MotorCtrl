// biped_hw — algo_api.hpp
// P6：同机 getState / sendCommand。函数 + 结构体，不是 plugin registry。
// 以后 MPC/RL 调这两个入口。形状在 quintic_traj；pid_track 只 pack MIT PD / HA60 servo。
#pragma once

#include "biped_hw/hip_parallel_kin.hpp"
#include "biped_hw/pid_track.hpp"
#include "biped_hw/robot_state.hpp"
#include "biped_hw/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace biped_hw
{

class UsbCanAdapter;
class FrameDemux;
class SerialPort;

struct MotorSetpoint
{
  bool active = false;
  float q_des = 0.f;
  float dq_des = 0.f;
  float kp = kPidTrackKp;
  float kd = kPidTrackKd;
  float tau_ff = 0.f;
};

struct HipCommand
{
  bool valid = false;
  HipPose pose{};
};

// Index matches kMotorMap. hip_* valid → hip_ik overwrites CAN 4/5 or 10/11.
struct RobotCommand
{
  std::array<MotorSetpoint, kMotorCount> motors{};
  HipCommand hip_left{};
  HipCommand hip_right{};
};

struct PackedMotorCmd
{
  uint8_t can_id = 0;
  BusId bus = BusId::Left;
  MotorType model = MotorType::DM10010;
  uint8_t data[8] = {};
  bool sent = false;
};

struct SendResult
{
  std::size_t n = 0;
  PackedMotorCmd packed[kMotorCount]{};
  bool hip_ik_ok = true;
};

// Null adapters skip that bus. Missing sensors stay valid=false.
struct AlgoSources
{
  UsbCanAdapter *left = nullptr;
  UsbCanAdapter *right = nullptr;
  FrameDemux *left_demux = nullptr;
  FrameDemux *right_demux = nullptr;
  LimitParam lims[16]{};
  SerialPort *mbs = nullptr;
  int mbs_timeout_ms = 8;
  bool mbs_rts_de = true;
  bool poll_mbs = false;
  const ImuFeedback *imu_latch = nullptr;
};

void getState(AlgoSources &src, RobotState &st);

// Pack MIT / HA60. TX if the matching adapter is open. src==nullptr → pack only.
SendResult sendCommand(AlgoSources *src, const RobotCommand &cmd);

int algo_api_self_test();

}  // namespace biped_hw
