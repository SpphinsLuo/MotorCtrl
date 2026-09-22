// biped_hw — cycle_logger.hpp
// 软件 RTT：为 MIT 发送打时间戳，与同 ESC_ID 的下一帧回包配对。
// P1 使能探测与 P2 阶梯周期日志使用。
#pragma once

// Software RTT: timestamp MIT send, match the next reply for the same ESC_ID.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace biped_hw
{

class CycleLogger
{
public:
  using Clock = std::chrono::steady_clock;

  void on_send(uint8_t esc_id, uint32_t tx_can_id, Clock::time_point t)
  {
    auto &p = pending_[esc_id & 0x0F];
    if (p.valid)
    {
      ++drops_;
      p.valid = false;
    }
    p.valid = true;
    p.tx_can_id = tx_can_id;
    p.t_send = t;
    ++sent_;
  }

  // True if this RX closed a pending send for esc_id.
  bool on_rx(uint8_t esc_id, uint32_t rx_can_id, Clock::time_point t)
  {
    auto &p = pending_[esc_id & 0x0F];
    if (!p.valid)
    {
      return false;
    }
    const double rtt_ms =
        std::chrono::duration<double, std::milli>(t - p.t_send).count();
    last_t_send_ = p.t_send;
    last_t_rx_ = t;
    last_tx_can_id_ = p.tx_can_id;
    last_rx_can_id_ = rx_can_id;
    last_esc_id_ = esc_id;
    last_rtt_ms_ = rtt_ms;
    last_drop_ = false;
    p.valid = false;
    ++rx_;
    rtt_sum_ms_ += rtt_ms;
    if (rtt_ms < rtt_min_ms_)
    {
      rtt_min_ms_ = rtt_ms;
    }
    if (rtt_ms > rtt_max_ms_)
    {
      rtt_max_ms_ = rtt_ms;
    }
    return true;
  }

  void on_timeout(uint8_t esc_id)
  {
    auto &p = pending_[esc_id & 0x0F];
    if (!p.valid)
    {
      return;
    }
    last_t_send_ = p.t_send;
    last_tx_can_id_ = p.tx_can_id;
    last_rx_can_id_ = 0;
    last_esc_id_ = esc_id;
    last_rtt_ms_ = 0.0;
    last_drop_ = true;
    p.valid = false;
    ++drops_;
  }

  std::uint64_t sent() const { return sent_; }
  std::uint64_t rx() const { return rx_; }
  std::uint64_t drops() const { return drops_; }
  double last_rtt_ms() const { return last_rtt_ms_; }
  bool last_drop() const { return last_drop_; }
  uint32_t last_rx_can_id() const { return last_rx_can_id_; }
  uint32_t last_tx_can_id() const { return last_tx_can_id_; }

  double rtt_mean_ms() const { return rx_ == 0 ? 0.0 : rtt_sum_ms_ / static_cast<double>(rx_); }
  double rtt_min_ms() const { return rx_ == 0 ? 0.0 : rtt_min_ms_; }
  double rtt_max_ms() const { return rx_ == 0 ? 0.0 : rtt_max_ms_; }

  void print_cycle(std::FILE *fp, const char *extra) const
  {
    const auto t_send_ms =
        std::chrono::duration<double, std::milli>(last_t_send_.time_since_epoch())
            .count();
    const auto t_rx_ms = last_drop_
                             ? 0.0
                             : std::chrono::duration<double, std::milli>(
                                   last_t_rx_.time_since_epoch())
                                   .count();
    std::fprintf(fp,
                 "t_send=%.3f t_rx=%.3f tx_id=%u rx_id=%u esc=%u drop=%d rtt_ms=%.3f %s\n",
                 t_send_ms, t_rx_ms, last_tx_can_id_, last_rx_can_id_,
                 static_cast<unsigned>(last_esc_id_), last_drop_ ? 1 : 0,
                 last_drop_ ? 0.0 : last_rtt_ms_, extra != nullptr ? extra : "");
  }

  void print_summary(std::FILE *fp) const
  {
    std::fprintf(fp,
                 "summary sent=%llu rx=%llu drop=%llu rtt_ms mean=%.3f min=%.3f max=%.3f\n",
                 static_cast<unsigned long long>(sent_),
                 static_cast<unsigned long long>(rx_),
                 static_cast<unsigned long long>(drops_), rtt_mean_ms(),
                 rtt_min_ms(), rtt_max_ms());
  }

private:
  struct Pending
  {
    bool valid = false;
    uint32_t tx_can_id = 0;
    Clock::time_point t_send{};
  };

  Pending pending_[16]{};
  std::uint64_t sent_ = 0;
  std::uint64_t rx_ = 0;
  std::uint64_t drops_ = 0;
  double rtt_sum_ms_ = 0.0;
  double rtt_min_ms_ = std::numeric_limits<double>::infinity();
  double rtt_max_ms_ = 0.0;

  Clock::time_point last_t_send_{};
  Clock::time_point last_t_rx_{};
  uint32_t last_tx_can_id_ = 0;
  uint32_t last_rx_can_id_ = 0;
  uint8_t last_esc_id_ = 0;
  double last_rtt_ms_ = 0.0;
  bool last_drop_ = false;
};

}  // namespace biped_hw
