// biped_hw — safety.cpp
// Watchdog / limits / e-stop latch / dual-bus 0xFD. Pack/send via dm_mit_host.
#include "biped_hw/safety.hpp"

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "mbs_encoder/mbs_map.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>

namespace biped_hw
{
namespace
{

std::atomic<bool> g_estop{false};
std::atomic<uint8_t> g_reason{static_cast<uint8_t>(SafetyReason::None)};
UsbCanAdapter *g_left = nullptr;
UsbCanAdapter *g_right = nullptr;
std::atomic<bool> g_armed{false};

float q_soft_for(uint8_t)
{
  return kSafetyQSoftRad;
}

float q_hard_for(uint8_t)
{
  return kSafetyQHardRad;
}

float tau_hard_for(uint8_t can_id)
{
  return peak_tau_nm(can_id);
}

SafetyLimitHit classify_abs(float mag, float soft, float hard)
{
  if (mag > hard)
  {
    return SafetyLimitHit::Hard;
  }
  if (mag > soft)
  {
    return SafetyLimitHit::Soft;
  }
  return SafetyLimitHit::Ok;
}

void on_signal(int)
{
  g_estop.store(true);
  g_reason.store(static_cast<uint8_t>(SafetyReason::Estop));
}

}  // namespace

const char *to_string(SafetyReason reason)
{
  switch (reason)
  {
  case SafetyReason::None:
    return "none";
  case SafetyReason::Watchdog:
    return "watchdog";
  case SafetyReason::Estop:
    return "estop";
  case SafetyReason::HardLimitQ:
    return "hard_q";
  case SafetyReason::HardLimitTau:
    return "hard_tau";
  case SafetyReason::Drop:
    return "drop";
  case SafetyReason::RidFail:
    return "rid";
  case SafetyReason::CommunicationTimeout:
    return "comm_timeout";
  }
  return "?";
}

SafetyLimitHit safety_check_q(uint8_t can_id, float q)
{
  if (find_by_can_id(can_id) == nullptr)
  {
    return SafetyLimitHit::Hard;
  }
  return classify_abs(std::fabs(q), q_soft_for(can_id), q_hard_for(can_id));
}

SafetyLimitHit safety_check_tau(uint8_t can_id, float tau)
{
  if (find_by_can_id(can_id) == nullptr)
  {
    return SafetyLimitHit::Hard;
  }
  if (std::fabs(tau) > tau_hard_for(can_id))
  {
    return SafetyLimitHit::Hard;
  }
  return SafetyLimitHit::Ok;
}

SafetyLimitHit safety_check_mbs_q(uint8_t addr, double q)
{
  if (mbs_map_index_by_addr(addr) < 0)
  {
    return SafetyLimitHit::Ok;
  }
  return classify_abs(static_cast<float>(std::fabs(q)), kSafetyQSoftRad, kSafetyQHardRad);
}

float safety_hold_q(uint8_t can_id, float q_cmd, SafetyLimitHit *hit)
{
  const SafetyLimitHit h = safety_check_q(can_id, q_cmd);
  if (hit != nullptr)
  {
    *hit = h;
  }
  if (h == SafetyLimitHit::Ok)
  {
    return q_cmd;
  }
  const float soft = q_soft_for(can_id);
  if (q_cmd > soft)
  {
    return soft;
  }
  if (q_cmd < -soft)
  {
    return -soft;
  }
  return q_cmd;
}

void safety_watchdog_arm(SafetyWatchdog &wd, int timeout_ms, std::int64_t now_ns)
{
  wd.timeout_ms = timeout_ms > 0 ? timeout_ms : kSafetyWatchdogMs;
  wd.last_pet_ns = now_ns;
  wd.armed = true;
}

void safety_watchdog_pet(SafetyWatchdog &wd, std::int64_t now_ns)
{
  wd.last_pet_ns = now_ns;
}

bool safety_watchdog_expired(const SafetyWatchdog &wd, std::int64_t now_ns)
{
  if (!wd.armed || wd.last_pet_ns == 0)
  {
    return false;
  }
  const std::int64_t dt = now_ns - wd.last_pet_ns;
  const std::int64_t lim = static_cast<std::int64_t>(wd.timeout_ms) * 1000000LL;
  return dt > lim;
}

bool safety_note_rx(SafetyDropWatch &w, uint8_t can_id, bool rx_ok)
{
  const uint8_t slot = static_cast<uint8_t>(can_id & 0x0F);
  if (rx_ok)
  {
    w.streak[slot] = 0;
    return false;
  }
  if (w.streak[slot] < 255)
  {
    ++w.streak[slot];
  }
  return w.streak[slot] >= static_cast<uint8_t>(kSafetyDropStreak);
}

bool safety_communication_timeout(double elapsed_ms, double period_ms)
{
  return std::isfinite(elapsed_ms) && std::isfinite(period_ms) && period_ms > 0.0 &&
         elapsed_ms > period_ms;
}

void safety_estop_trip()
{
  g_estop.store(true);
  g_reason.store(static_cast<uint8_t>(SafetyReason::Estop));
}

bool safety_estop_latched()
{
  return g_estop.load();
}

void safety_estop_clear()
{
  g_estop.store(false);
  if (static_cast<SafetyReason>(g_reason.load()) == SafetyReason::Estop)
  {
    g_reason.store(static_cast<uint8_t>(SafetyReason::None));
  }
}

void safety_install_signals()
{
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
}

SafetyReason safety_last_reason()
{
  return static_cast<SafetyReason>(g_reason.load());
}

void safety_set_reason(SafetyReason reason)
{
  g_reason.store(static_cast<uint8_t>(reason));
}

void safety_bind_buses(UsbCanAdapter *left, UsbCanAdapter *right)
{
  g_left = left;
  g_right = right;
}

void safety_arm()
{
  g_armed.store(true);
}

void safety_disarm()
{
  g_armed.store(false);
}

bool safety_armed()
{
  return g_armed.load();
}

void safety_atexit_disable()
{
  if (!g_armed.load())
  {
    return;
  }
  const SafetyReason r = safety_estop_latched() ? SafetyReason::Estop : safety_last_reason();
  safety_disable_all(g_left, g_right, nullptr, 0,
                     r == SafetyReason::None ? SafetyReason::Estop : r, false, nullptr);
  g_armed.store(false);
}

std::size_t safety_disable_all(UsbCanAdapter *left, UsbCanAdapter *right, const uint8_t *ids,
                               std::size_t n, SafetyReason reason, bool print,
                               SafetyDisableHook *hook)
{
  uint8_t map_ids[kMotorCount];
  if (ids == nullptr || n == 0)
  {
    n = kMotorMap.size();
    for (std::size_t i = 0; i < n; ++i)
    {
      map_ids[i] = kMotorMap[i].can_id;
    }
    ids = map_ids;
  }
  g_reason.store(static_cast<uint8_t>(reason));
  std::size_t sent = 0;
  std::size_t attempted = 0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const auto *e = find_by_can_id(ids[i]);
    if (e == nullptr)
    {
      continue;
    }
    ++attempted;
    UsbCanAdapter *ad = e->bus == BusId::Right ? right : left;
    if (ad != nullptr && ad->is_open())
    {
      dm_send_disable(*ad, e->can_id);
      ++sent;
    }
    if (hook != nullptr && hook->on_disable != nullptr)
    {
      hook->on_disable(hook->ctx, e->bus, e->can_id);
    }
    if (print)
    {
      const bool did = ad != nullptr && ad->is_open();
      std::printf("disable id=%u bus=%s reason=%s %s (0xFD)\n",
                  static_cast<unsigned>(e->can_id), to_string(e->bus), to_string(reason),
                  did ? "sent" : "requested");
    }
  }
  if (print)
  {
    std::printf("disable-all 0xFD reason=%s attempted=%zu sent=%zu (map left 3-8, right 9-14)\n",
                to_string(reason), attempted, sent);
  }
  g_armed.store(false);
  return attempted;
}

namespace
{

struct FdLog
{
  uint8_t ids[16] = {};
  BusId buses[16] = {};
  std::size_t n = 0;
};

void log_fd(void *ctx, BusId bus, uint8_t can_id)
{
  auto *l = static_cast<FdLog *>(ctx);
  if (l->n >= 16)
  {
    return;
  }
  l->ids[l->n] = can_id;
  l->buses[l->n] = bus;
  ++l->n;
}

bool log_has(const FdLog &l, uint8_t id, BusId bus)
{
  for (std::size_t i = 0; i < l.n; ++i)
  {
    if (l.ids[i] == id && l.buses[i] == bus)
    {
      return true;
    }
  }
  return false;
}

int expect_map12(const FdLog &l)
{
  if (l.n != kMotorCount)
  {
    return 10;
  }
  for (uint8_t id = 3; id <= 8; ++id)
  {
    if (!log_has(l, id, BusId::Left))
    {
      return 11;
    }
  }
  for (uint8_t id = 9; id <= 14; ++id)
  {
    if (!log_has(l, id, BusId::Right))
    {
      return 12;
    }
  }
  return 0;
}

}  // namespace

int safety_self_test()
{
  safety_estop_clear();
  safety_disarm();
  safety_set_reason(SafetyReason::None);
  safety_bind_buses(nullptr, nullptr);

  if (kMotorMap.size() != 12 || kMotorMap[0].can_id != 3 || kMotorMap[11].can_id != 14)
  {
    return 1;
  }
  if (safety_check_q(6, 0.f) != SafetyLimitHit::Ok ||
      safety_check_q(6, 10.5f) != SafetyLimitHit::Soft ||
      safety_check_q(6, 12.5f) != SafetyLimitHit::Hard)
  {
    return 2;
  }
  SafetyLimitHit hit = SafetyLimitHit::Ok;
  const float held = safety_hold_q(12, 11.f, &hit);
  if (hit != SafetyLimitHit::Soft || held != kSafetyQSoftRad)
  {
    return 3;
  }
  const float t6 = peak_tau_nm(static_cast<uint8_t>(6));
  const float t8 = peak_tau_nm(static_cast<uint8_t>(8));
  if (t6 < 1.f || t8 < 1.f ||
      safety_check_tau(6, t6) != SafetyLimitHit::Ok ||
      safety_check_tau(6, t6 + 0.1f) != SafetyLimitHit::Hard ||
      safety_check_tau(8, t8) != SafetyLimitHit::Ok ||
      safety_check_tau(8, t8 + 0.1f) != SafetyLimitHit::Hard)
  {
    return 4;
  }
  if (safety_check_mbs_q(6, 10.5) != SafetyLimitHit::Soft ||
      safety_check_mbs_q(6, 12.5) != SafetyLimitHit::Hard ||
      safety_check_mbs_q(0x1F, 99.0) != SafetyLimitHit::Ok)
  {
    return 5;
  }

  SafetyWatchdog wd;
  safety_watchdog_arm(wd, 20, 1'000'000'000);
  safety_watchdog_pet(wd, 1'010'000'000);
  if (safety_watchdog_expired(wd, 1'015'000'000))
  {
    return 6;
  }
  if (!safety_watchdog_expired(wd, 1'040'000'001))
  {
    return 6;
  }
  FdLog wd_log;
  SafetyDisableHook hook{log_fd, &wd_log};
  safety_set_reason(SafetyReason::Watchdog);
  const std::size_t n_wd =
      safety_disable_all(nullptr, nullptr, nullptr, 0, SafetyReason::Watchdog, false, &hook);
  if (n_wd != 12)
  {
    return 6;
  }
  const int wd_map = expect_map12(wd_log);
  if (wd_map != 0)
  {
    return wd_map;
  }

  safety_estop_clear();
  if (safety_estop_latched())
  {
    return 7;
  }
  safety_estop_trip();
  if (!safety_estop_latched() || safety_last_reason() != SafetyReason::Estop)
  {
    return 7;
  }
  FdLog es_log;
  hook.ctx = &es_log;
  if (safety_disable_all(nullptr, nullptr, nullptr, 0, SafetyReason::Estop, false, &hook) != 12 ||
      expect_map12(es_log) != 0)
  {
    return 7;
  }
  safety_estop_clear();

  FdLog q_log;
  hook.ctx = &q_log;
  if (safety_check_q(3, 12.1f) != SafetyLimitHit::Hard)
  {
    return 8;
  }
  if (safety_disable_all(nullptr, nullptr, nullptr, 0, SafetyReason::HardLimitQ, false, &hook) !=
          12 ||
      expect_map12(q_log) != 0)
  {
    return 8;
  }

  SafetyDropWatch drops;
  if (safety_note_rx(drops, 6, false) || safety_note_rx(drops, 6, false))
  {
    return 9;
  }
  if (!safety_note_rx(drops, 6, false))
  {
    return 9;
  }
  if (safety_communication_timeout(10.001, 10.0) == false ||
      safety_communication_timeout(10.0, 10.0) ||
      safety_communication_timeout(9.999, 10.0))
  {
    return 9;
  }
  const uint8_t subset[2] = {6, 12};
  FdLog drop_log;
  hook.ctx = &drop_log;
  if (safety_disable_all(nullptr, nullptr, subset, 2, SafetyReason::Drop, false, &hook) != 2 ||
      drop_log.n != 2 || !log_has(drop_log, 6, BusId::Left) ||
      !log_has(drop_log, 12, BusId::Right))
  {
    return 9;
  }

  FdLog rid_log;
  hook.ctx = &rid_log;
  if (safety_disable_all(nullptr, nullptr, nullptr, 0, SafetyReason::RidFail, false, &hook) !=
          12 ||
      expect_map12(rid_log) != 0)
  {
    return 13;
  }

  uint8_t en[8] = {};
  pack_enable(en);
  uint8_t dis[8] = {};
  pack_disable(dis);
  if (en[7] == kDmDisableCmd || dis[7] != kDmDisableCmd)
  {
    return 14;
  }

  safety_bind_buses(nullptr, nullptr);
  safety_disarm();
  safety_estop_clear();
  safety_set_reason(SafetyReason::None);
  return 0;
}

}  // namespace biped_hw
