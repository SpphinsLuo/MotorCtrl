// biped_hw — metrics.hpp
// P2 闭环阶梯指标：跟踪误差（rms/max）、周期抖动、RTT 汇总，以及 settle 估计。
// 位置档误差为 q_des−q；速度档误差为 dq_des−dq（位置斜坡上的速度跟踪）。
// 被 p2_dm_test 与 p2_evo_test 使用。
#pragma once

// Running metrics for P2 closed-loop ladder tests: tracking error, period
// jitter, and RTT summaries (wraps CycleLogger aggregates).

#include "biped_hw/cycle_logger.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace biped_hw
{

struct ErrorStats
{
  std::uint64_t n = 0;
  double sum_abs = 0.0;
  double sum_sq = 0.0;
  double max_abs = 0.0;

  void push(double err)
  {
    const double a = std::fabs(err);
    ++n;
    sum_abs += a;
    sum_sq += err * err;
    if (a > max_abs)
    {
      max_abs = a;
    }
  }

  double mean_abs() const { return n == 0 ? 0.0 : sum_abs / static_cast<double>(n); }
  double rms() const { return n == 0 ? 0.0 : std::sqrt(sum_sq / static_cast<double>(n)); }
  double max() const { return max_abs; }
};

struct PeriodJitterStats
{
  double nominal_us = 0.0;
  std::uint64_t n = 0;
  double min_us = std::numeric_limits<double>::infinity();
  double max_us = 0.0;
  double sum_us = 0.0;
  double sum_sq_us = 0.0;
  double max_dev_us = 0.0;

  explicit PeriodJitterStats(double nominal_period_us = 0.0) : nominal_us(nominal_period_us) {}

  void push(double period_us)
  {
    ++n;
    sum_us += period_us;
    sum_sq_us += period_us * period_us;
    if (period_us < min_us)
    {
      min_us = period_us;
    }
    if (period_us > max_us)
    {
      max_us = period_us;
    }
    const double dev = std::fabs(period_us - nominal_us);
    if (dev > max_dev_us)
    {
      max_dev_us = dev;
    }
  }

  double mean_us() const { return n == 0 ? 0.0 : sum_us / static_cast<double>(n); }
  double std_us() const
  {
    if (n < 2)
    {
      return 0.0;
    }
    const double m = mean_us();
    const double var = (sum_sq_us / static_cast<double>(n)) - (m * m);
    return var > 0.0 ? std::sqrt(var) : 0.0;
  }
  double min() const { return n == 0 ? 0.0 : min_us; }
  double max() const { return n == 0 ? 0.0 : max_us; }
  double max_deviation_us() const { return max_dev_us; }
};

struct RttStats
{
  std::uint64_t rx = 0;
  double mean_ms = 0.0;
  double min_ms = 0.0;
  double max_ms = 0.0;

  static RttStats from_cycle_logger(const CycleLogger &log)
  {
    RttStats s;
    s.rx = log.rx();
    s.mean_ms = log.rtt_mean_ms();
    s.min_ms = log.rtt_min_ms();
    s.max_ms = log.rtt_max_ms();
    return s;
  }
};

// Estimate settle time: first sample after |err| stays below threshold for
// hold_samples consecutive samples. Returns -1 if never settled.
inline double estimate_settle_s(const double *errs, std::size_t count, double threshold,
                                std::size_t hold_samples, double sample_period_s)
{
  if (count == 0 || hold_samples == 0)
  {
    return -1.0;
  }
  std::size_t streak = 0;
  for (std::size_t i = 0; i < count; ++i)
  {
    if (std::fabs(errs[i]) <= threshold)
    {
      ++streak;
      if (streak >= hold_samples)
      {
        const std::size_t settle_idx = i - hold_samples + 1;
        return static_cast<double>(settle_idx) * sample_period_s;
      }
    }
    else
    {
      streak = 0;
    }
  }
  return -1.0;
}

inline bool metrics_self_check()
{
  ErrorStats es;
  es.push(0.1);
  es.push(-0.3);
  es.push(0.2);
  if (es.n != 3)
  {
    return false;
  }
  if (std::fabs(es.mean_abs() - (0.1 + 0.3 + 0.2) / 3.0) > 1e-9)
  {
    return false;
  }
  const double rms_exp = std::sqrt((0.01 + 0.09 + 0.04) / 3.0);
  if (std::fabs(es.rms() - rms_exp) > 1e-9)
  {
    return false;
  }
  if (std::fabs(es.max() - 0.3) > 1e-9)
  {
    return false;
  }

  PeriodJitterStats pj(10000.0);
  pj.push(10000.0);
  pj.push(10100.0);
  pj.push(9900.0);
  if (pj.n != 3)
  {
    return false;
  }
  if (std::fabs(pj.max_deviation_us() - 100.0) > 1e-9)
  {
    return false;
  }
  if (std::fabs(pj.mean_us() - 10000.0) > 1e-9)
  {
    return false;
  }

  const double errs[] = {1.0, 0.5, 0.08, 0.05, 0.04, 0.03};
  const double settle = estimate_settle_s(errs, 6, 0.05, 3, 0.01);
  if (std::fabs(settle - 0.03) > 1e-9)
  {
    return false;
  }

  return true;
}

}  // namespace biped_hw
