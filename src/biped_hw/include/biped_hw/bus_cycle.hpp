// biped_hw — bus_cycle.hpp
// P3 同周期 write-all / drain-all：先把本总线指令发完，再在超时内收齐期望 ID。
// 缺电机记 drop，不无限等待。双总线：同进程先双发，再对两个 USB-CAN fd 一次
// select() 同收（elapsed = max，不是两次 SerialPort 超时之和）。
#pragma once

#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/types.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace biped_hw
{

struct CycleMotorCmd
{
  uint8_t can_id = 0;
  MotorType model = MotorType::DM10010;
  LimitParam lim{};
  bool send = true;
  bool expect_rx = true;
  uint8_t data[8] = {};
  uint8_t dlc = 8;
};

struct CycleMotorResult
{
  uint8_t can_id = 0;
  MotorType model = MotorType::DM10010;
  bool sent = false;
  bool rx_ok = false;
  bool drop = false;
  bool skipped = false;
  double rtt_ms = 0.0;
  DmFeedback fb{};
  bool have_fb = false;
};

struct BusCycleResult
{
  BusId bus = BusId::Left;
  double elapsed_ms = 0.0;
  bool overrun = false;
  std::size_t expected = 0;
  std::size_t rx_ok = 0;
  std::size_t drop = 0;
  std::size_t skipped = 0;
  std::vector<CycleMotorResult> motors;
};

// Per-bus inflight state so DualBusCycle can send both sides before draining.
struct BusCycleInflight
{
  using Clock = std::chrono::steady_clock;

  std::vector<CycleMotorResult> motors;
  bool wanted[16] = {};
  LimitParam lims[16] = {};
  bool motorevo_pending[16] = {};
  std::size_t pending = 0;
  Clock::time_point t_start{};
  bool active = false;
};

class BusCycle
{
public:
  using Clock = std::chrono::steady_clock;

  BusCycle(UsbCanAdapter &adapter, BusId bus, FrameDemux &demux);

  UsbCanAdapter &adapter() { return *adapter_; }
  BusId bus() const { return bus_; }
  bool is_open() const { return adapter_ != nullptr && adapter_->is_open(); }

  // Send every sendable command. Missing motors are not waited on here.
  void begin_tick(const std::vector<CycleMotorCmd> &cmds, BusCycleInflight &io,
                  CycleLogger *log);

  // One pollCan burst + demux extract. First recv uses the adapter timeout;
  // further recvs in this burst are timeout 0 so complete_rx is not delayed
  // by another SerialPort select quantum (~2 ms).
  void poll_once(BusCycleInflight &io, CycleLogger *log);

  // Mark leftovers as drop. period is only used for overrun vs elapsed.
  BusCycleResult finish(BusCycleInflight &io, Clock::time_point t0,
                        std::chrono::milliseconds period, CycleLogger *log);

  // Single-bus: write-all then drain-all until timeout or all expected IDs arrived.
  BusCycleResult tick(const std::vector<CycleMotorCmd> &cmds,
                      std::chrono::milliseconds timeout,
                      std::chrono::milliseconds period, CycleLogger *log = nullptr);

private:
  int index_of(BusCycleInflight &io, uint8_t can_id) const;
  void complete_rx(BusCycleInflight &io, int idx, const DmFeedback *fb, CycleLogger *log);
  void harvest_rx(BusCycleInflight &io, CycleLogger *log);

  UsbCanAdapter *adapter_ = nullptr;
  BusId bus_ = BusId::Left;
  FrameDemux *demux_ = nullptr;
};

struct DualBusCycleResult
{
  BusCycleResult left;
  BusCycleResult right;
  double elapsed_ms = 0.0;
  bool overrun = false;
  bool left_used = false;
  bool right_used = false;
};

// Same process: send left all, send right all, then one select() on both
// adapter fds until a shared deadline. Whichever fd is readable is drained
// with timeout 0 and stamped immediately. SerialPort::recv select()s one fd,
// so waiting on both in the same call is what actually overlaps the waits.
// Each adapter/logger/demux is used by this thread only. If only one adapter
// is set, that side runs alone (same as BusCycle::tick).
class DualBusCycle
{
public:
  DualBusCycle(BusCycle *left, BusCycle *right);
  ~DualBusCycle() = default;

  DualBusCycle(const DualBusCycle &) = delete;
  DualBusCycle &operator=(const DualBusCycle &) = delete;
  DualBusCycle(DualBusCycle &&) = delete;
  DualBusCycle &operator=(DualBusCycle &&) = delete;

  DualBusCycleResult tick(const std::vector<CycleMotorCmd> *left_cmds,
                          const std::vector<CycleMotorCmd> *right_cmds,
                          std::chrono::milliseconds timeout,
                          std::chrono::milliseconds period, CycleLogger *left_log,
                          CycleLogger *right_log);

private:
  BusCycle *left_ = nullptr;
  BusCycle *right_ = nullptr;
};

bool bus_cycle_self_check();

}  // namespace biped_hw
