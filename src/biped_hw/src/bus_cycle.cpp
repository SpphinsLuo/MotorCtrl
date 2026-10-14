// biped_hw — bus_cycle.cpp
// 实现单总线 write-all/drain-all 与双总线「先双发、再对两 fd 一次 select 同收」。
// 超时内未到的 ESC_ID 记 drop，循环继续；供 P3 三个 app 使用。
// Motorevo 回包用 take_motorevo：从站 CAN 8/14 或 Master 0x02。
#include "biped_hw/bus_cycle.hpp"

#include "biped_hw/motorevo_codec.hpp"

#include <algorithm>
#include <cerrno>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace biped_hw
{

namespace
{

bool is_ha60(MotorType type)
{
  return type == MotorType::HA60;
}

int remaining_timeout_ms(BusCycle::Clock::time_point deadline)
{
  const auto left = deadline - BusCycle::Clock::now();
  const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(left).count();
  if (ms <= 0)
  {
    return 0;
  }
  if (ms > 1000000)
  {
    return 1000000;
  }
  return static_cast<int>(ms);
}

timeval remaining_timeval(BusCycle::Clock::time_point deadline)
{
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(deadline -
                                                                 BusCycle::Clock::now());
  if (us.count() < 0)
  {
    us = std::chrono::microseconds::zero();
  }
  timeval tv{};
  tv.tv_sec = static_cast<time_t>(us.count() / 1000000);
  tv.tv_usec = static_cast<suseconds_t>(us.count() % 1000000);
  return tv;
}

void restore_timeout(BusCycle *cycle)
{
  if (cycle != nullptr)
  {
    cycle->adapter().set_recv_timeout(SerialPort::kDefaultTimeoutMs);
  }
}

void drain_until_deadline(BusCycle *cycle, BusCycleInflight &io, CycleLogger *log,
                          BusCycle::Clock::time_point deadline)
{
  if (cycle == nullptr)
  {
    return;
  }
  while (BusCycle::Clock::now() < deadline && io.pending > 0)
  {
    cycle->adapter().set_recv_timeout(remaining_timeout_ms(deadline));
    cycle->poll_once(io, log);
  }
  restore_timeout(cycle);
}

// One wait on both fds. Ready bits are only valid when return > 0.
int wait_two_fds(int fd_a, int fd_b, bool want_a, bool want_b, timeval timeout,
                 bool *a_ready, bool *b_ready)
{
  if (a_ready != nullptr)
  {
    *a_ready = false;
  }
  if (b_ready != nullptr)
  {
    *b_ready = false;
  }

  fd_set rfds;
  FD_ZERO(&rfds);
  int maxfd = -1;
  if (want_a && fd_a >= 0)
  {
    FD_SET(fd_a, &rfds);
    maxfd = fd_a;
  }
  if (want_b && fd_b >= 0)
  {
    FD_SET(fd_b, &rfds);
    maxfd = std::max(maxfd, fd_b);
  }
  if (maxfd < 0)
  {
    return 0;
  }

  const int n = ::select(maxfd + 1, &rfds, nullptr, nullptr, &timeout);
  if (n > 0)
  {
    if (a_ready != nullptr && want_a && fd_a >= 0)
    {
      *a_ready = FD_ISSET(fd_a, &rfds) != 0;
    }
    if (b_ready != nullptr && want_b && fd_b >= 0)
    {
      *b_ready = FD_ISSET(fd_b, &rfds) != 0;
    }
  }
  return n;
}

void poll_available(BusCycle *cycle, BusCycleInflight &io, CycleLogger *log)
{
  if (cycle == nullptr || io.pending == 0)
  {
    return;
  }
  cycle->adapter().set_recv_timeout(0);
  cycle->poll_once(io, log);
}

void drain_both_until_deadline(BusCycle *left, BusCycleInflight &left_io,
                               CycleLogger *left_log, BusCycle *right,
                               BusCycleInflight &right_io, CycleLogger *right_log,
                               BusCycle::Clock::time_point deadline)
{
  while (BusCycle::Clock::now() < deadline &&
         (left_io.pending > 0 || right_io.pending > 0))
  {
    const bool want_l = left_io.pending > 0 && left != nullptr;
    const bool want_r = right_io.pending > 0 && right != nullptr;
    // Skip select only when a full USB-CAN frame (16 B) is already queued.
    // A leftover byte must not busy-spin and starve the other fd.
    const bool left_queued = want_l && left->adapter().queued_bytes() >= 16;
    const bool right_queued = want_r && right->adapter().queued_bytes() >= 16;

    if (!left_queued && !right_queued)
    {
      const int fd_l = want_l ? left->adapter().native_fd() : -1;
      const int fd_r = want_r ? right->adapter().native_fd() : -1;
      bool l_ready = false;
      bool r_ready = false;
      const int n = wait_two_fds(fd_l, fd_r, want_l, want_r,
                                 remaining_timeval(deadline), &l_ready, &r_ready);
      if (n < 0 && errno == EINTR)
      {
        continue;
      }
      (void)l_ready;
      (void)r_ready;
    }

    // timeout 0: stamp complete_rx as soon as a frame is already readable.
    // Do not issue another 2 ms select on the quiet side.
    poll_available(left, left_io, left_log);
    poll_available(right, right_io, right_log);
  }
  restore_timeout(left);
  restore_timeout(right);
}

}  // namespace

BusCycle::BusCycle(UsbCanAdapter &adapter, BusId bus, FrameDemux &demux)
    : adapter_(&adapter), bus_(bus), demux_(&demux)
{
}

int BusCycle::index_of(BusCycleInflight &io, uint8_t can_id) const
{
  for (std::size_t i = 0; i < io.motors.size(); ++i)
  {
    if (io.motors[i].can_id == can_id)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void BusCycle::complete_rx(BusCycleInflight &io, int idx, const DmFeedback *fb,
                           CycleLogger *log)
{
  if (idx < 0 || static_cast<std::size_t>(idx) >= io.motors.size())
  {
    return;
  }
  auto &r = io.motors[static_cast<std::size_t>(idx)];
  if (r.rx_ok || r.drop || r.skipped)
  {
    return;
  }
  r.rx_ok = true;
  r.drop = false;
  if (fb != nullptr)
  {
    r.fb = *fb;
    r.have_fb = true;
  }
  const auto t_rx = Clock::now();
  if (log != nullptr)
  {
    log->on_rx(r.can_id, fb != nullptr ? fb->rx_can_id : r.can_id, t_rx);
    r.rtt_ms = log->last_rtt_ms();
  }
  else
  {
    r.rtt_ms = std::chrono::duration<double, std::milli>(t_rx - io.t_start).count();
  }
  io.wanted[r.can_id & 0x0F] = false;
  io.motorevo_pending[r.can_id & 0x0F] = false;
  if (io.pending > 0)
  {
    --io.pending;
  }
}

void BusCycle::harvest_rx(BusCycleInflight &io, CycleLogger *log)
{
  if (demux_ == nullptr)
  {
    return;
  }

  DmFeedback fb{};
  while (io.pending > 0 && demux_->take_wanted_mit(io.wanted, io.lims, fb))
  {
    complete_rx(io, index_of(io, fb.esc_id), &fb, log);
  }

  for (uint8_t slot = 0; slot < 16 && io.pending > 0; ++slot)
  {
    if (!io.motorevo_pending[slot])
    {
      continue;
    }
    // Left HA60 is CAN 8, right is CAN 14. Slot is can_id & 0x0F (8 or 14).
    // Feedback may arrive on slave ID or Master 0x02.
    const uint8_t can_id = slot;  // 8 stays 8; 14 stays 14
    MotorevoFeedback mf{};
    if (!demux_->take_motorevo(can_id, mf))
    {
      continue;
    }
    const int idx = index_of(io, can_id);
    DmFeedback as_dm{};
    as_dm.rx_can_id = mf.rx_can_id;
    as_dm.dlc = mf.dlc;
    as_dm.esc_id = can_id;
    as_dm.q = mf.q;
    as_dm.dq = mf.dq;
    as_dm.tau = mf.tau;
    as_dm.err = mf.err_code;
    complete_rx(io, idx, &as_dm, log);
  }
}

void BusCycle::begin_tick(const std::vector<CycleMotorCmd> &cmds, BusCycleInflight &io,
                          CycleLogger *log)
{
  io = {};
  io.active = true;
  io.t_start = Clock::now();
  io.motors.resize(cmds.size());
  if (demux_ != nullptr)
  {
    demux_->clear();
  }

  for (std::size_t i = 0; i < cmds.size(); ++i)
  {
    const auto &c = cmds[i];
    auto &r = io.motors[i];
    r.can_id = c.can_id;
    r.model = c.model;
    const uint8_t slot = static_cast<uint8_t>(c.can_id & 0x0F);
    io.lims[slot] = c.lim;

    if (!c.send)
    {
      r.skipped = true;
      continue;
    }
    const auto t_send = Clock::now();
    if (adapter_ == nullptr || !adapter_->sendCan(c.can_id, c.data, c.dlc))
    {
      r.sent = false;
      if (c.expect_rx)
      {
        r.drop = true;
      }
      continue;
    }
    r.sent = true;
    if (log != nullptr)
    {
      log->on_send(c.can_id, c.can_id, t_send);
    }
    if (c.expect_rx)
    {
      io.wanted[slot] = !is_ha60(c.model);
      io.motorevo_pending[slot] = is_ha60(c.model);
      ++io.pending;
    }
  }
}

void BusCycle::poll_once(BusCycleInflight &io, CycleLogger *log)
{
  if (!io.active || adapter_ == nullptr || demux_ == nullptr || io.pending == 0)
  {
    return;
  }

  const int saved = adapter_->recv_timeout_ms();
  int n = 0;
  while (n < 32 && io.pending > 0)
  {
    CanRxFrame rx;
    if (!adapter_->pollCan(rx))
    {
      break;
    }
    demux_->push(rx);
    ++n;
    harvest_rx(io, log);
    // Extra frames in this burst must not block. The previous pollCan already
    // waited; another 2 ms select would stamp t_rx after a fake quantum.
    adapter_->set_recv_timeout(0);
  }
  adapter_->set_recv_timeout(saved);
}

BusCycleResult BusCycle::finish(BusCycleInflight &io, Clock::time_point t0,
                                std::chrono::milliseconds period, CycleLogger *log)
{
  BusCycleResult out;
  out.bus = bus_;
  if (!io.active)
  {
    return out;
  }

  for (auto &r : io.motors)
  {
    if (r.skipped)
    {
      ++out.skipped;
      continue;
    }
    if (r.rx_ok)
    {
      ++out.rx_ok;
      ++out.expected;
      continue;
    }
    const uint8_t slot = static_cast<uint8_t>(r.can_id & 0x0F);
    if (io.wanted[slot] || io.motorevo_pending[slot])
    {
      r.drop = true;
      if (log != nullptr)
      {
        log->on_timeout(r.can_id);
      }
      io.wanted[slot] = false;
      io.motorevo_pending[slot] = false;
      if (io.pending > 0)
      {
        --io.pending;
      }
    }
    if (r.drop)
    {
      ++out.drop;
      ++out.expected;
    }
  }

  out.motors = std::move(io.motors);
  out.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  out.overrun = out.elapsed_ms > static_cast<double>(period.count());
  io.active = false;
  return out;
}

BusCycleResult BusCycle::tick(const std::vector<CycleMotorCmd> &cmds,
                              std::chrono::milliseconds timeout,
                              std::chrono::milliseconds period, CycleLogger *log)
{
  BusCycleInflight io;
  begin_tick(cmds, io, log);
  const auto deadline = io.t_start + timeout;
  drain_until_deadline(this, io, log, deadline);
  return finish(io, io.t_start, period, log);
}

DualBusCycle::DualBusCycle(BusCycle *left, BusCycle *right) : left_(left), right_(right)
{
}

DualBusCycleResult DualBusCycle::tick(const std::vector<CycleMotorCmd> *left_cmds,
                                      const std::vector<CycleMotorCmd> *right_cmds,
                                      std::chrono::milliseconds timeout,
                                      std::chrono::milliseconds period,
                                      CycleLogger *left_log, CycleLogger *right_log)
{
  DualBusCycleResult out;
  BusCycleInflight left_io;
  BusCycleInflight right_io;
  const auto t0 = BusCycle::Clock::now();

  const bool use_left = left_ != nullptr && left_cmds != nullptr && left_->is_open();
  const bool use_right = right_ != nullptr && right_cmds != nullptr && right_->is_open();
  out.left_used = use_left;
  out.right_used = use_right;

  if (use_left)
  {
    left_->begin_tick(*left_cmds, left_io, left_log);
  }
  if (use_right)
  {
    right_->begin_tick(*right_cmds, right_io, right_log);
  }

  const auto deadline = t0 + timeout;
  if (use_left && use_right)
  {
    drain_both_until_deadline(left_, left_io, left_log, right_, right_io, right_log,
                              deadline);
  }
  else if (use_left)
  {
    drain_until_deadline(left_, left_io, left_log, deadline);
  }
  else if (use_right)
  {
    drain_until_deadline(right_, right_io, right_log, deadline);
  }

  if (use_left)
  {
    out.left = left_->finish(left_io, t0, period, left_log);
  }
  if (use_right)
  {
    out.right = right_->finish(right_io, t0, period, right_log);
  }
  out.elapsed_ms = std::chrono::duration<double, std::milli>(BusCycle::Clock::now() - t0).count();
  out.overrun = out.elapsed_ms > static_cast<double>(period.count());
  return out;
}

bool bus_cycle_self_check()
{
  if (!motorevo_codec_self_check())
  {
    return false;
  }

  CycleMotorCmd c{};
  c.can_id = 6;
  c.model = MotorType::DM10422;
  c.lim = kLimitDm10422;
  c.send = false;
  c.expect_rx = false;
  BusCycleInflight io;
  io.motors.push_back({});
  io.motors[0].can_id = 6;
  io.motors[0].skipped = true;
  BusCycleResult dummy;
  dummy.motors.push_back(io.motors[0]);
  dummy.skipped = 1;
  if (dummy.motors.size() != 1 || !dummy.motors[0].skipped)
  {
    return false;
  }

  DualBusCycleResult dual;
  if (dual.left_used || dual.right_used)
  {
    return false;
  }

  // Right knee: Master-2 + esc nibble 12, and slave-id CAN 12 (nibble may be 0).
  auto mit_zero = [](uint8_t esc_nibble) {
    CanRxFrame rx{};
    rx.dlc = 8;
    rx.data[0] = esc_nibble;
    rx.data[1] = 0x7F;
    rx.data[2] = 0xFF;
    rx.data[3] = 0x7F;
    rx.data[4] = 0xF7;
    rx.data[5] = 0xFF;
    rx.data[6] = 40;
    rx.data[7] = 35;
    return rx;
  };
  bool wanted[16] = {};
  LimitParam lims[16] = {};
  wanted[12] = true;
  lims[12] = kLimitDm10422;
  {
    FrameDemux d;
    CanRxFrame rx = mit_zero(0x0C);
    rx.can_id = kMasterId;
    d.push(rx);
    DmFeedback fb{};
    if (!d.take_wanted_mit(wanted, lims, fb) || fb.esc_id != 12)
    {
      return false;
    }
  }
  {
    FrameDemux d;
    CanRxFrame rx = mit_zero(0x00);
    rx.can_id = 12;
    d.push(rx);
    DmFeedback fb{};
    if (!d.take_wanted_mit(wanted, lims, fb) || fb.esc_id != 12 || fb.rx_can_id != 12)
    {
      return false;
    }
  }
  {
    FrameDemux d;
    CanRxFrame rx = mit_zero(0x06);
    rx.can_id = kMasterId;
    d.push(rx);
    DmFeedback fb{};
    if (d.take_wanted_mit(wanted, lims, fb))
    {
      return false;  // esc=6 on Master 2 must not satisfy wanted[12]
    }
  }

  {
    DualBusCycle empty(nullptr, nullptr);
    std::vector<CycleMotorCmd> none;
    const auto r = empty.tick(&none, &none, std::chrono::milliseconds(1),
                              std::chrono::milliseconds(5), nullptr, nullptr);
    if (r.left_used || r.right_used)
    {
      return false;
    }
  }
  {
    UsbCanAdapter left_ad("/dev/null");
    UsbCanAdapter right_ad("/dev/null");
    FrameDemux ld;
    FrameDemux rd;
    BusCycle lc(left_ad, BusId::Left, ld);
    BusCycle rc(right_ad, BusId::Right, rd);
    DualBusCycle dual_buses(&lc, &rc);
    std::vector<CycleMotorCmd> none;
    const auto r = dual_buses.tick(&none, &none, std::chrono::milliseconds(1),
                                   std::chrono::milliseconds(5), nullptr, nullptr);
    if (r.left_used || r.right_used)
    {
      return false;
    }
  }

  // Dual-fd wait must return when the *second* fd is readable without eating
  // a SerialPort 2 ms quantum on the idle first fd.
  {
    int idle[2] = {-1, -1};
    int live[2] = {-1, -1};
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, idle) != 0 ||
        socketpair(AF_UNIX, SOCK_STREAM, 0, live) != 0)
    {
      return false;
    }
    const uint8_t byte = 0xA5;
    if (::write(live[1], &byte, 1) != 1)
    {
      ::close(idle[0]);
      ::close(idle[1]);
      ::close(live[0]);
      ::close(live[1]);
      return false;
    }
    bool idle_ready = false;
    bool live_ready = false;
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    const auto t0 = BusCycle::Clock::now();
    const int n = wait_two_fds(idle[0], live[0], true, true, tv, &idle_ready, &live_ready);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(BusCycle::Clock::now() - t0).count();
    ::close(idle[0]);
    ::close(idle[1]);
    ::close(live[0]);
    ::close(live[1]);
    if (n <= 0 || idle_ready || !live_ready || elapsed_ms >= 2.0)
    {
      return false;
    }
  }
  return true;
}

}  // namespace biped_hw
