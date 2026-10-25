// biped_hw — p3_runtime.cpp
// P3 运行时：探测在线 ID、低 kp hold 或 sine 绕 q0、write-all/drain-all。
// 只使能 --ids（dual12 默认 3–14；right6 默认 9–14；其余默认 6）。退出走 P5 disable-all。
// --comms-only 不发 0xFC：RID + MIT kp=0 / HA60 evo inquire，然后 0xFD。
// --stage sine 不调用 dm_home_to_zero：q_des 绕探测 q0 正弦运动。
// --set-zero：选中 --ids 写固件零点（官方 0xFE），不与 --stage 并用，然后 0xFD。
#include "biped_hw/p3_runtime.hpp"

#include "biped_hw/bus_cycle.hpp"
#include "biped_hw/cli.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/metrics.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/safety.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <unistd.h>
#include <unordered_set>

namespace biped_hw
{
namespace
{

constexpr int kEnableSettleUs = 100000;
constexpr int kDisableGapUs = 10000;
constexpr float kKpCap = kMitKpMax;
constexpr float kKdCap = kMitKdMax;
constexpr int kProbeTicks = 3;
// P2 HA60 conservative servo hold (vel_kp=0 cannot track/hold θ_ref).
constexpr float kHa60HoldPosKp = 8.f;
constexpr float kHa60HoldPosKd = 0.5f;
constexpr float kHa60HoldVelLimit = 0.5f;
constexpr float kHa60HoldVelKp = 20.f;
constexpr float kHa60HoldVelKd = 0.f;
constexpr float kHa60HoldVelKi = 0.001f;
constexpr float kVelDefault = 0.20f;
constexpr float kVelCap = 0.30f;
constexpr float kStrokeDefault = 0.349f;  // ±20 deg
constexpr float kAbortQMargin = 0.10f;

std::atomic<bool> g_run{true};
UsbCanAdapter *g_left = nullptr;
UsbCanAdapter *g_right = nullptr;

void on_signal(int)
{
  g_run.store(false);
  safety_estop_trip();
}

void disable_all_now(bool print)
{
  if (!safety_armed() && !safety_estop_latched() &&
      safety_last_reason() == SafetyReason::None)
  {
    return;
  }
  const auto r = safety_estop_latched() ? SafetyReason::Estop : safety_last_reason();
  safety_disable_all(g_left, g_right, nullptr, 0,
                     r == SafetyReason::None ? SafetyReason::Estop : r, print);
}

void atexit_disable()
{
  safety_atexit_disable();
}

bool contains_id(const std::vector<uint8_t> &ids, uint8_t id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}


bool parse_csv_u8(const char *s, std::vector<uint8_t> &out)
{
  out.clear();
  if (s == nullptr || *s == '\0')
  {
    return false;
  }
  const char *p = s;
  while (*p)
  {
    while (*p == ' ' || *p == '\t' || *p == ',')
    {
      ++p;
    }
    if (*p == '\0')
    {
      break;
    }
    char *end = nullptr;
    const long v = std::strtol(p, &end, 10);
    if (end == p || v < 0 || v > 255)
    {
      return false;
    }
    const uint8_t id = static_cast<uint8_t>(v);
    if (!contains_id(out, id))
    {
      out.push_back(id);
    }
    p = end;
  }
  return !out.empty();
}


UsbCanAdapter *adapter_for(uint8_t can_id)
{
  const auto *e = find_by_can_id(can_id);
  if (e != nullptr && e->bus == BusId::Right)
  {
    return g_right;
  }
  return g_left;
}

bool id12_on_adapter(UsbCanAdapter &ad, float &q0)
{
  if (!ad.is_open())
  {
    return false;
  }
  FrameDemux dm;
  q0 = 0.f;
  return dm_probe_q0(ad, dm, 12, kLimitDm10422, q0, 3, 50);
}

bool open_print_right(UsbCanAdapter &ad, const std::string &path)
{
  if (!ad.open())
  {
    std::fprintf(stderr, "open right %s failed: %s\n", path.c_str(), ad.last_error().c_str());
    return false;
  }
  std::printf("opened right %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n", path.c_str(),
              ad.can_bitrate(), static_cast<unsigned>(ad.can_baud_index()));
  return true;
}

// Right6: prefer --port / ACM1. If that port has no ID12 D[0]=0x0C, use the
// other ACM when it does. MIT kp=0 only (no 0xFC).
UsbCanAdapter *open_right6_port(UsbCanAdapter &primary, UsbCanAdapter &alt,
                                const std::string &primary_path, const std::string &alt_path)
{
  float q0 = 0.f;
  const bool prim_open = open_print_right(primary, primary_path);
  if (prim_open && id12_on_adapter(primary, q0))
  {
    std::printf("identity: ID12 D[0]=0x0C on %s (q=%.4f)\n", primary_path.c_str(),
                static_cast<double>(q0));
    primary.drain();
    return &primary;
  }
  if (prim_open)
  {
    std::printf("identity: ID12 silent on %s; trying %s\n", primary_path.c_str(),
                alt_path.c_str());
    primary.close();
  }
  float q0b = 0.f;
  if (open_print_right(alt, alt_path) && id12_on_adapter(alt, q0b))
  {
    std::printf("identity: ID12 D[0]=0x0C on %s (q=%.4f); using this port\n", alt_path.c_str(),
                static_cast<double>(q0b));
    alt.drain();
    return &alt;
  }
  if (alt.is_open())
  {
    std::printf("identity: ID12 silent on %s too; staying here for the 9-14 probe\n",
                alt_path.c_str());
    return &alt;
  }
  if (prim_open && open_print_right(primary, primary_path))
  {
    std::printf("identity: no ID12 on either ACM; probing requested port %s\n",
                primary_path.c_str());
    return &primary;
  }
  return nullptr;
}

FrameDemux *demux_for(uint8_t can_id, FrameDemux &left, FrameDemux &right)
{
  const auto *e = find_by_can_id(can_id);
  if (e != nullptr && e->bus == BusId::Right)
  {
    return &right;
  }
  return &left;
}


void add_unique(std::vector<uint8_t> &ids, uint8_t id)
{
  if (!contains_id(ids, id))
  {
    ids.push_back(id);
  }
}

struct IdStats
{
  uint8_t can_id = 0;
  MotorType model = MotorType::DM10010;
  std::uint64_t sent = 0;
  std::uint64_t rx = 0;
  std::uint64_t drop = 0;
  std::uint64_t skipped = 0;
  double rtt_sum = 0.0;
  double rtt_min = 1e9;
  double rtt_max = 0.0;
  float last_q = 0.f;
  float last_tau = 0.f;
  float q0 = 0.f;
  float q_des = 0.f;
  float dq_des = 0.f;
  float q_min = 0.f;
  float q_max = 0.f;
  float q_peak_abs = 0.f;
  float tau_peak_abs = 0.f;
  bool have_q = false;
  bool have_q0 = false;
  bool enabled = false;
  bool present_probe = false;
  std::uint64_t err_n = 0;
  uint8_t last_err = 0;
};

IdStats *find_stats(std::vector<IdStats> &st, uint8_t id)
{
  for (auto &s : st)
  {
    if (s.can_id == id)
    {
      return &s;
    }
  }
  return nullptr;
}

void accumulate(std::vector<IdStats> &st, const BusCycleResult &r)
{
  for (const auto &m : r.motors)
  {
    auto *s = find_stats(st, m.can_id);
    if (s == nullptr)
    {
      continue;
    }
    if (m.skipped)
    {
      ++s->skipped;
      continue;
    }
    if (m.sent)
    {
      ++s->sent;
    }
    if (m.rx_ok)
    {
      ++s->rx;
      s->rtt_sum += m.rtt_ms;
      if (m.rtt_ms < s->rtt_min)
      {
        s->rtt_min = m.rtt_ms;
      }
      if (m.rtt_ms > s->rtt_max)
      {
        s->rtt_max = m.rtt_ms;
      }
      if (m.have_fb)
      {
        s->last_q = m.fb.q;
        s->last_tau = m.fb.tau;
        s->have_q = true;
        const float tabs = std::fabs(m.fb.tau);
        if (tabs > s->tau_peak_abs)
        {
          s->tau_peak_abs = tabs;
        }
        if (s->have_q0)
        {
          if (m.fb.q < s->q_min)
          {
            s->q_min = m.fb.q;
          }
          if (m.fb.q > s->q_max)
          {
            s->q_max = m.fb.q;
          }
          const float dev = std::fabs(m.fb.q - s->q0);
          if (dev > s->q_peak_abs)
          {
            s->q_peak_abs = dev;
          }
        }
        if (m.fb.err != 0)
        {
          ++s->err_n;
          s->last_err = m.fb.err;
        }
      }
    }
    else if (m.drop)
    {
      ++s->drop;
    }
  }
}

void pack_cmd_for(CycleMotorCmd &c, const MotorMapEntry &e, const LimitParam &lim, bool send,
                  bool expect, bool enabled, float kp, float kd, float tau_ff, float q_cmd,
                  bool have_q, float dq, float ha60_vref)
{
  c = {};
  c.can_id = e.can_id;
  c.model = e.model;
  c.lim = lim;
  c.send = send;
  c.expect_rx = expect;
  c.dlc = 8;
  if (!send)
  {
    return;
  }
  if (e.model == MotorType::HA60)
  {
    const float q = have_q ? q_cmd : 0.f;
    if (!enabled)
    {
      pack_motorevo_servo(c.data, q, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
      return;
    }
    pack_motorevo_servo(c.data, q, kHa60HoldPosKp, kHa60HoldPosKd, ha60_vref,
                        kHa60HoldVelKp, kHa60HoldVelKd, kHa60HoldVelKi);
    return;
  }
  const float q = (enabled && have_q) ? q_cmd : 0.f;
  const float use_kp = enabled ? kp : 0.f;
  const float use_kd = enabled ? kd : 0.f;
  const float use_dq = enabled ? dq : 0.f;
  const float use_tau = enabled ? tau_ff : 0.f;
  pack_mit(c.data, use_kp, use_kd, q, use_dq, use_tau, lim);
}

void print_sine_table(const std::vector<IdStats> &st, float stroke)
{
  std::printf("\n=== P3 sine q vs ±%.3f rad (probed q0, not firmware 0) ===\n",
              static_cast<double>(stroke));
  std::printf(" id  q0       q_min     q_max     peak_|q-q0|  |tau|peak\n");
  for (const auto &s : st)
  {
    if (!s.have_q0)
    {
      std::printf(" %2u      -        -         -           -          -\n",
                  static_cast<unsigned>(s.can_id));
      continue;
    }
    std::printf(" %2u  % .4f  % .4f  % .4f  % .4f       % .3f\n",
                static_cast<unsigned>(s.can_id), static_cast<double>(s.q0),
                static_cast<double>(s.q_min), static_cast<double>(s.q_max),
                static_cast<double>(s.q_peak_abs), static_cast<double>(s.tau_peak_abs));
  }
}

void print_summary_table(const std::vector<IdStats> &st, std::uint64_t overruns,
                         std::uint64_t cycles)
{
  std::printf("\n=== P3 per-id summary ===\n");
  std::printf(" id  model      enabled  sent   rx     drop   skip   rtt_ms mean/min/max   last_q\n");
  for (const auto &s : st)
  {
    const double mean = s.rx == 0 ? 0.0 : s.rtt_sum / static_cast<double>(s.rx);
    char qbuf[32];
    if (s.have_q)
    {
      std::snprintf(qbuf, sizeof(qbuf), "% .4f", static_cast<double>(s.last_q));
    }
    else
    {
      std::snprintf(qbuf, sizeof(qbuf), "     -");
    }
    std::printf(" %2u  %-9s  %s      %5llu %5llu %6llu %6llu  %6.3f / %6.3f / %6.3f  %s\n",
                static_cast<unsigned>(s.can_id), to_string(s.model), s.enabled ? "Y" : "n",
                static_cast<unsigned long long>(s.sent),
                static_cast<unsigned long long>(s.rx),
                static_cast<unsigned long long>(s.drop),
                static_cast<unsigned long long>(s.skipped), mean,
                s.rx == 0 ? 0.0 : s.rtt_min, s.rx == 0 ? 0.0 : s.rtt_max, qbuf);
  }
  std::printf("cycles=%llu  overrun=%llu\n", static_cast<unsigned long long>(cycles),
              static_cast<unsigned long long>(overruns));
}

}  // namespace

const char *p3_bin_name(P3App app)
{
  switch (app)
  {
  case P3App::Left6:
    return "p3_left6_sync";
  case P3App::Dual12:
    return "p3_dual12_sync";
  case P3App::FaultInject:
    return "p3_fault_inject";
  case P3App::Right6:
    return "p3_right6_sync";
  }
  return "p3";
}

void p3_print_usage(const P3Options &defaults)
{
  const char *bin = p3_bin_name(defaults.app);
  std::string ids_csv;
  for (std::size_t i = 0; i < defaults.ids.size(); ++i)
  {
    if (i != 0)
    {
      ids_csv += ',';
    }
    ids_csv += std::to_string(defaults.ids[i]);
  }
  if (ids_csv.empty())
  {
    ids_csv = "(none)";
  }
  std::printf(
      "Usage: %s [options]\n"
      "  P3 same-cycle write-all / drain-all. Missing motors timeout (drop), loop continues.\n"
      "  --period-ms MS        default %d\n"
      "  --duration SEC        default %.1f\n"
      "  --drain-ms MS         RX wait cap per cycle (default period-2)\n"
      "  --kp --kd             MIT gains (default %.1f / %.2f, cap kp<=%.0f kd<=%.0f)\n"
      "  --tau-ff NM           MIT feedforward torque (default %.1f, sign kept;\n"
      "                        wire saturates at that motor TMAX). HA60 ignores it.\n"
      "  --stage hold|sine default hold (q=last). sine: q_des=q0+stroke*sin(phase),\n"
      "                        dq_des=stroke*omega*cos(phase), same phase,\n"
      "                        same cycle (all selected IDs updated each tick).\n"
      "                        Does not home to firmware 0 (no dm_home_to_zero).\n"
      "                        HA60 uses evo servo θ_ref=q_des around probe q0.\n"
      "                        --comms-only is unchanged (no 0xFC, no motion).\n"
      "  --vel RAD/S           sine peak speed (default %.2f, cap %.2f)\n"
      "  --stroke RAD          sine peak displacement (default %.3f = ±20 deg)\n"
      "  --abort-tau NM        abort if |tau_fb| exceeds. Default = that motor TMAX\n"
      "                        (DM10010/L 200, DM10422 400, HA60 50). Override only.\n"
      "  --ids LIST            motors to operate (default %s). Example: --ids 6,8\n"
      "                        or --ids 3,4,5,6,7,8,9,10,11,12,13,14. Comma list,\n"
      "                        not a range. Only listed IDs are probed/enabled/zeroed.\n"
      "                        Unlisted stay off.\n"
      "  --require-all         exit 1 if any expected id has zero RX\n"
      "  --enable-all-anyway   enable --ids even if probe never replied\n"
      "  --comms-only          no 0xFC, no MIT hold/dq. RID + MIT kp=0 inquire\n"
      "                        / HA60 evo status; 0xFD hygiene after. Right-only\n"
      "                        is enough for p3_right6_sync (does not open left).\n"
      "  --port PATH           left USB-CAN (dual12 / fault); right6: right USB-CAN\n"
      "  --left-port --right-port --yaml PATH\n"
      "  --handshake --verbose --self-test --set-zero\n"
      "  --set-zero            WRITES firmware zero at current shaft on selected\n"
      "                        --ids (official FF..FE SetzeroPosition). Not motion;\n"
      "                        not 0xFC. Mutually exclusive with --stage hold|sine.\n"
      "                        Then 0xFD hygiene on those IDs.\n",
      bin, defaults.period_ms, defaults.duration_s, static_cast<double>(defaults.kp),
      static_cast<double>(defaults.kd), static_cast<double>(kKpCap),
      static_cast<double>(kKdCap), static_cast<double>(defaults.tau_ff),
      static_cast<double>(kVelDefault), static_cast<double>(kVelCap),
      static_cast<double>(kStrokeDefault), ids_csv.c_str());
  if (defaults.app == P3App::Dual12)
  {
    std::printf(
        "  --allow-single-bus    if ttyACM1 missing: error, then run left only\n"
        "  --set-zero zeros the IDs this app selects (default all map 3–14,\n"
        "  including HA60 8/14). Current position → firmware origin.\n"
        "  Not motion; not 0xFC. Exclusive with --stage.\n");
  }
  if (defaults.app == P3App::FaultInject)
  {
    std::printf(
        "  --ignore-id N         simulate drop: do not send and do not wait (repeatable)\n"
        "  P3 records whether other IDs still update and whether the loop blocks.\n"
        "  Exit/SIGINT uses P5 disable-all (0xFD map 3-14 on open buses).\n");
  }
  if (defaults.app == P3App::Right6)
  {
    std::printf(
        "  Right bus only (CAN 9-14). Default --ids 9,10,11,12,13,14 (all right map,\n"
        "  including HA60 14), default port %s.\n"
        "  If ACM swapped: identity looks for ID12 D[0]=0x0C and uses that port.\n"
        "  --set-zero writes firmware zero on those selected IDs at the current\n"
        "  shaft (default all right 9–14). Exclusive with --stage.\n"
        "  HA60 uses official SetzeroPosition FF..FE when listed in --ids.\n",
        kDefaultPortRight);
  }
  std::printf(
      "  Safety: this app default --ids %s. Big warning if enabling more than one motor.\n"
      "  Only --ids are probed/enabled/zeroed. Unlisted stay off.\n"
      "  Exit/SIGINT: P5 disable-all 0xFD on map IDs of open buses.\n",
      ids_csv.c_str());
}

bool p3_parse_args(int argc, char **argv, P3Options &opt, std::string &err)
{
  bool left_port_set = false;
  bool right_port_set = false;
  bool drain_set = false;

  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      p3_print_usage(opt);
      err = "help";
      return false;
    }
    auto need = [&](const char *flag) -> const char * {
      return need_arg(i, argc, argv, flag, err);
    };
    if (std::strcmp(argv[i], "--period-ms") == 0)
    {
      const char *v = need("--period-ms");
      if (v == nullptr)
      {
        return false;
      }
      opt.period_ms = std::atoi(v);
    }
    else if (std::strcmp(argv[i], "--duration") == 0)
    {
      const char *v = need("--duration");
      if (v == nullptr)
      {
        return false;
      }
      opt.duration_s = std::atof(v);
    }
    else if (std::strcmp(argv[i], "--drain-ms") == 0)
    {
      const char *v = need("--drain-ms");
      if (v == nullptr)
      {
        return false;
      }
      opt.drain_ms = std::atoi(v);
      drain_set = true;
    }
    else if (std::strcmp(argv[i], "--kp") == 0)
    {
      const char *v = need("--kp");
      if (v == nullptr)
      {
        return false;
      }
      opt.kp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kd") == 0)
    {
      const char *v = need("--kd");
      if (v == nullptr)
      {
        return false;
      }
      opt.kd = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--tau-ff") == 0)
    {
      const char *v = need("--tau-ff");
      if (v == nullptr)
      {
        return false;
      }
      opt.tau_ff = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--stage") == 0)
    {
      const char *v = need("--stage");
      if (v == nullptr)
      {
        return false;
      }
      if (std::strcmp(v, "hold") == 0)
      {
        opt.stage = P3Stage::Hold;
      }
      else if (std::strcmp(v, "sine") == 0)
      {
        opt.stage = P3Stage::Sine;
      }
      else
      {
        err = "--stage must be hold|sine";
        return false;
      }
      opt.stage_set = true;
    }
    else if (std::strcmp(argv[i], "--set-zero") == 0)
    {
      opt.set_zero = true;
    }
    else if (std::strcmp(argv[i], "--vel") == 0)
    {
      const char *v = need("--vel");
      if (v == nullptr)
      {
        return false;
      }
      opt.vel = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--stroke") == 0)
    {
      const char *v = need("--stroke");
      if (v == nullptr)
      {
        return false;
      }
      opt.stroke = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--abort-tau") == 0)
    {
      const char *v = need("--abort-tau");
      if (v == nullptr)
      {
        return false;
      }
      opt.abort_tau = std::strtof(v, nullptr);
      if (!(opt.abort_tau > 0.f))
      {
        err = "--abort-tau must be > 0";
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--ids") == 0)
    {
      const char *v = need("--ids");
      if (v == nullptr)
      {
        return false;
      }
      if (!parse_csv_u8(v, opt.ids))
      {
        err = "--ids needs comma-separated CAN IDs";
        return false;
      }
      opt.ids_set = true;
    }
    else if (std::strcmp(argv[i], "--ignore-id") == 0)
    {
      const char *v = need("--ignore-id");
      if (v == nullptr)
      {
        return false;
      }
      std::vector<uint8_t> extra;
      if (!parse_csv_u8(v, extra))
      {
        err = "--ignore-id needs an ID";
        return false;
      }
      for (uint8_t id : extra)
      {
        add_unique(opt.ignore_ids, id);
      }
    }
    else if (std::strcmp(argv[i], "--skip-motorevo") == 0 ||
             std::strcmp(argv[i], "--no-skip-motorevo") == 0)
    {
      err = "removed: list HA60 8/14 in --ids to operate them";
      return false;
    }
    else if (std::strcmp(argv[i], "--require-all") == 0)
    {
      opt.require_all = true;
    }
    else if (std::strcmp(argv[i], "--allow-single-bus") == 0)
    {
      opt.allow_single_bus = true;
    }
    else if (std::strcmp(argv[i], "--enable-all-anyway") == 0)
    {
      opt.enable_all_anyway = true;
    }
    else if (std::strcmp(argv[i], "--comms-only") == 0)
    {
      opt.comms_only = true;
    }
    else if (std::strcmp(argv[i], "--port") == 0)
    {
      const char *v = need("--port");
      if (v == nullptr)
      {
        return false;
      }
      if (opt.app == P3App::Right6)
      {
        opt.right_port = v;
        right_port_set = true;
      }
      else
      {
        opt.left_port = v;
        left_port_set = true;
      }
    }
    else if (std::strcmp(argv[i], "--left-port") == 0)
    {
      const char *v = need("--left-port");
      if (v == nullptr)
      {
        return false;
      }
      opt.left_port = v;
      left_port_set = true;
    }
    else if (std::strcmp(argv[i], "--right-port") == 0)
    {
      const char *v = need("--right-port");
      if (v == nullptr)
      {
        return false;
      }
      opt.right_port = v;
      right_port_set = true;
    }
    else if (std::strcmp(argv[i], "--yaml") == 0)
    {
      const char *v = need("--yaml");
      if (v == nullptr)
      {
        return false;
      }
      opt.yaml_path = v;
    }
    else if (std::strcmp(argv[i], "--handshake") == 0)
    {
      opt.handshake = true;
    }
    else if (std::strcmp(argv[i], "--verbose") == 0)
    {
      opt.verbose = true;
    }
    else if (std::strcmp(argv[i], "--self-test") == 0)
    {
      opt.self_test = true;
    }
    else
    {
      err = std::string("unknown argument: ") + argv[i] + " (try --help)";
      return false;
    }
  }

  if (opt.set_zero && opt.stage_set)
  {
    err = "refuse: --set-zero cannot be combined with --stage";
    return false;
  }

  if (opt.left_port.empty())
  {
    opt.left_port = kDefaultPortLeft;
  }
  if (opt.right_port.empty())
  {
    opt.right_port = kDefaultPortRight;
  }
  if (!opt.yaml_path.empty() && (!left_port_set || !right_port_set))
  {
    std::string yl, yr;
    if (!yaml_bus_ports(opt.yaml_path, &yl, &yr))
    {
      err = "yaml overlay missing buses.*.port: " + opt.yaml_path;
      return false;
    }
    if (!left_port_set && !yl.empty())
    {
      opt.left_port = yl;
    }
    if (!right_port_set && !yr.empty())
    {
      opt.right_port = yr;
    }
  }
  if (opt.duration_s < 0.5)
  {
    opt.duration_s = 0.5;
  }
  if (opt.period_ms < 2)
  {
    opt.period_ms = 2;
  }
  if (!drain_set)
  {
    opt.drain_ms = std::max(2, opt.period_ms - 2);
  }
  if (opt.drain_ms < 1)
  {
    opt.drain_ms = 1;
  }
  if (opt.kp < 0.f)
  {
    opt.kp = 0.f;
  }
  if (opt.kp > kKpCap)
  {
    std::printf("WARN: clamping kp %.1f -> %.1f (MIT kp 0-%.0f)\n",
                static_cast<double>(opt.kp), static_cast<double>(kKpCap),
                static_cast<double>(kKpCap));
    opt.kp = kKpCap;
  }
  if (opt.kd < 0.f)
  {
    opt.kd = 0.f;
  }
  if (opt.kd > kKdCap)
  {
    std::printf("WARN: clamping kd %.2f -> %.2f (MIT kd 0-%.0f)\n",
                static_cast<double>(opt.kd), static_cast<double>(kKdCap),
                static_cast<double>(kKdCap));
    opt.kd = kKdCap;
  }
  if (opt.vel < 0.f)
  {
    opt.vel = -opt.vel;
  }
  if (opt.vel < 0.05f)
  {
    opt.vel = 0.05f;
  }
  if (opt.vel > kVelCap)
  {
    std::printf("WARN: clamping vel %.3f -> %.2f rad/s (conservative 6-motor swing)\n",
                static_cast<double>(opt.vel), static_cast<double>(kVelCap));
    opt.vel = kVelCap;
  }
  if (opt.stroke < 0.f)
  {
    opt.stroke = -opt.stroke;
  }
  if (opt.stroke < 0.02f)
  {
    opt.stroke = 0.02f;
  }
  return true;
}

int p3_run(const P3Options &opt_in)
{
  P3Options opt = opt_in;

  if (!dm_codec_self_check() || !bus_cycle_self_check())
  {
    std::fprintf(stderr, "codec/bus_cycle self-check failed\n");
    return 1;
  }
  if (opt.self_test)
  {
    std::printf("p3 self-check OK (no hardware): dm_codec, motorevo_codec, bus_cycle\n");
    return 0;
  }

  const bool want_left = opt.app != P3App::Right6;
  const bool want_right = opt.app == P3App::Dual12 || opt.app == P3App::Right6;
  const bool left_only = want_left && !want_right;
  const bool right_only = want_right && !want_left;

  bool listed_ha60 = false;
  for (uint8_t id : opt.ids)
  {
    const auto *e = find_by_can_id(id);
    if (e == nullptr)
    {
      std::fprintf(stderr, "--ids contains unmapped CAN ID %u\n", static_cast<unsigned>(id));
      return 2;
    }
    if (left_only && e->bus != BusId::Left)
    {
      std::fprintf(stderr, "CAN ID %u is on the right bus; this app is left-only\n",
                   static_cast<unsigned>(id));
      return 2;
    }
    if (right_only && e->bus != BusId::Right)
    {
      std::fprintf(stderr, "CAN ID %u is on the left bus; this app is right-only\n",
                   static_cast<unsigned>(id));
      return 2;
    }
    if (e->model == MotorType::HA60)
    {
      listed_ha60 = true;
    }
  }
  if (listed_ha60)
  {
    std::printf("NOTE: --ids includes Motorevo HA60\n");
  }
  if (opt.comms_only)
  {
    std::printf("comms-only: no 0xFC, no MIT hold/dq; RID + kp=0 inquire / HA60 evo status\n");
    if (opt.stage == P3Stage::Sine)
    {
      std::printf("comms-only: ignoring --stage sine (no motion)\n");
    }
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::atexit(atexit_disable);

  UsbCanAdapter left_ad(opt.left_port);
  UsbCanAdapter right_ad(opt.right_port);
  const std::string right_alt_path =
      opt.right_port == kDefaultPortLeft ? std::string(kDefaultPortRight)
                                        : std::string(kDefaultPortLeft);
  UsbCanAdapter right_alt(right_alt_path);

  bool left_open = false;
  if (want_left)
  {
    if (!left_ad.open())
    {
      std::fprintf(stderr, "open left %s failed: %s\n", opt.left_port.c_str(),
                   left_ad.last_error().c_str());
      return 1;
    }
    left_open = true;
    g_left = &left_ad;
    std::printf("opened left %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n",
                opt.left_port.c_str(), left_ad.can_bitrate(),
                static_cast<unsigned>(left_ad.can_baud_index()));
  }

  bool right_open = false;
  if (opt.app == P3App::Right6)
  {
    UsbCanAdapter *used =
        open_right6_port(right_ad, right_alt, opt.right_port, right_alt_path);
    if (used == nullptr)
    {
      return 1;
    }
    right_open = true;
    g_right = used;
  }
  else if (want_right)
  {
    if (right_ad.open())
    {
      right_open = true;
      g_right = &right_ad;
      std::printf("opened right %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n",
                  opt.right_port.c_str(), right_ad.can_bitrate(),
                  static_cast<unsigned>(right_ad.can_baud_index()));
    }
    else
    {
      std::fprintf(stderr, "open right %s failed: %s\n", opt.right_port.c_str(),
                   right_ad.last_error().c_str());
      if (!opt.allow_single_bus)
      {
        std::fprintf(stderr, "pass --allow-single-bus to continue with left only\n");
        g_left = nullptr;
        return 1;
      }
      std::printf("WARN: --allow-single-bus: continuing with left only\n");
    }
  }
  safety_bind_buses(g_left, g_right);

  if (opt.handshake)
  {
    if (left_open)
    {
      left_ad.handshake();
      usleep(20000);
      left_ad.drain();
    }
    if (right_open && g_right != nullptr)
    {
      g_right->handshake();
      usleep(20000);
      g_right->drain();
    }
    std::printf("handshake sent (CMD 0x02)\n");
  }

  if (opt.set_zero)
  {
    std::printf(
        "\n"
        "**********************************************************************\n"
        "* WARN: --set-zero WRITES firmware zero at current shaft position.  *\n"
        "* Selected --ids only. Official FF..FE SetzeroPosition.             *\n"
        "* Irreversible-ish. Shafts must be at the intended origin.          *\n"
        "**********************************************************************\n");
    bool all_ok = true;
    bool any = false;
    for (uint8_t id : opt.ids)
    {
      const auto *e = find_by_can_id(id);
      if (e == nullptr)
      {
        continue;
      }
      UsbCanAdapter *ad = adapter_for(id);
      if (ad == nullptr || !ad->is_open())
      {
        std::fprintf(stderr, "skip set-zero id=%u (adapter not open)\n",
                     static_cast<unsigned>(id));
        all_ok = false;
        continue;
      }
      any = true;
      bool ok = false;
      if (e->model == MotorType::HA60)
      {
        ok = evo_send_set_zero(*ad, id);
        std::printf("set-zero id=%u %s (HA60 SetzeroPosition+SaveFlash) FE on %u save 0x%X\n",
                    static_cast<unsigned>(id), ok ? "sent" : "TX FAIL",
                    static_cast<unsigned>(id), motorevo_param_can_id(id));
      }
      else
      {
        ok = dm_send_set_zero(*ad, id);
        std::printf("set-zero id=%u %s (0xFE) data=FF FF FF FF FF FF FF FE\n",
                    static_cast<unsigned>(id), ok ? "sent" : "TX FAIL");
      }
      if (ok)
      {
        dm_send_disable(*ad, id);
        std::printf("disable id=%u sent (0xFD)\n", static_cast<unsigned>(id));
      }
      else
      {
        all_ok = false;
      }
    }
    if (!any)
    {
      std::fprintf(stderr, "--set-zero: no selected IDs to write\n");
      all_ok = false;
    }
    if (left_open)
    {
      left_ad.close();
    }
    if (g_right != nullptr)
    {
      g_right->close();
    }
    g_left = nullptr;
    g_right = nullptr;
    safety_bind_buses(nullptr, nullptr);
    return all_ok ? 0 : 1;
  }

  std::vector<uint8_t> left_expected;
  std::vector<uint8_t> right_expected;
  for (uint8_t id : opt.ids)
  {
    const auto *e = find_by_can_id(id);
    if (e == nullptr)
    {
      continue;
    }
    if (e->bus == BusId::Left && left_open)
    {
      add_unique(left_expected, id);
    }
    else if (e->bus == BusId::Right && right_open)
    {
      add_unique(right_expected, id);
    }
  }

  std::vector<uint8_t> all_expected = left_expected;
  all_expected.insert(all_expected.end(), right_expected.begin(), right_expected.end());

  std::vector<IdStats> stats;
  stats.reserve(all_expected.size());
  for (uint8_t id : all_expected)
  {
    const auto *e = find_by_can_id(id);
    IdStats s;
    s.can_id = id;
    s.model = e != nullptr ? e->model : MotorType::DM10010;
    stats.push_back(s);
  }

  LimitParam lims[16]{};
  for (uint8_t id : all_expected)
  {
    const auto *e = find_by_can_id(id);
    lims[id & 0x0F] = e != nullptr ? default_limit(e->model) : kLimitDm10010;
  }

  FrameDemux left_demux;
  FrameDemux right_demux;
  BusCycle left_cycle(left_ad, BusId::Left, left_demux);
  BusCycle right_cycle_primary(right_ad, BusId::Right, right_demux);
  BusCycle right_cycle_alt(right_alt, BusId::Right, right_demux);
  BusCycle *right_cycle = nullptr;
  if (right_open && g_right == &right_alt)
  {
    right_cycle = &right_cycle_alt;
  }
  else if (right_open)
  {
    right_cycle = &right_cycle_primary;
  }
  DualBusCycle dual(left_open ? &left_cycle : nullptr, right_cycle);

  auto build_cmds = [&](const std::vector<uint8_t> &ids, const std::unordered_set<uint8_t> &enable,
                        bool inquiry_only) -> std::vector<CycleMotorCmd> {
    std::vector<CycleMotorCmd> cmds;
    cmds.reserve(ids.size());
    for (uint8_t id : ids)
    {
      const auto *e = find_by_can_id(id);
      if (e == nullptr)
      {
        continue;
      }
      const bool ignore = contains_id(opt.ignore_ids, id);
      const bool en = !inquiry_only && enable.count(id) != 0;
      auto *st = find_stats(stats, id);
      const bool sine = en && opt.stage == P3Stage::Sine;
      const float q_cmd = (st == nullptr) ? 0.f : (sine ? st->q_des : st->last_q);
      const bool have_q =
          st != nullptr && (sine ? st->have_q0 : st->have_q);
      const float dq = (sine && st != nullptr) ? st->dq_des : 0.f;
      const float vref = sine ? std::max(kHa60HoldVelLimit, opt.vel) : kHa60HoldVelLimit;
      CycleMotorCmd c;
      pack_cmd_for(c, *e, lims[id & 0x0F], !ignore, !ignore, en, opt.kp, opt.kd, opt.tau_ff,
                   q_cmd, have_q, dq, vref);
      cmds.push_back(c);
    }
    return cmds;
  };

  const auto period = std::chrono::milliseconds(opt.period_ms);
  const auto drain = std::chrono::milliseconds(opt.drain_ms);
  char abort_tau_buf[32] = "TMAX";
  if (opt.abort_tau > 0.f)
  {
    std::snprintf(abort_tau_buf, sizeof(abort_tau_buf), "%.0f",
                  static_cast<double>(opt.abort_tau));
  }

  std::printf(
      "P3 %s  period=%dms drain=%dms duration=%.1fs kp=%.1f kd=%.2f tau_ff=%.3f\n"
      "stage=%s vel=%.3f stroke=±%.3f abort-tau=%s\n"
      "expected left:",
      p3_bin_name(opt.app), opt.period_ms, opt.drain_ms, opt.duration_s,
      static_cast<double>(opt.kp), static_cast<double>(opt.kd),
      static_cast<double>(opt.tau_ff),
      opt.stage == P3Stage::Sine ? "sine" : "hold", static_cast<double>(opt.vel),
      static_cast<double>(opt.stroke), abort_tau_buf);
  for (uint8_t id : left_expected)
  {
    std::printf(" %u", static_cast<unsigned>(id));
  }
  std::printf("  right:");
  if (right_expected.empty())
  {
    std::printf(" (none)");
  }
  for (uint8_t id : right_expected)
  {
    std::printf(" %u", static_cast<unsigned>(id));
  }
  std::printf("\nselected --ids:");
  for (uint8_t id : opt.ids)
  {
    std::printf(" %u", static_cast<unsigned>(id));
  }
  std::printf("\n");
  if (!opt.ignore_ids.empty())
  {
    std::printf("FAULT INJECT ignore-id:");
    for (uint8_t id : opt.ignore_ids)
    {
      std::printf(" %u", static_cast<unsigned>(id));
    }
    std::printf("  (P3 records only during the loop; exit uses P5 disable-all)\n");
  }

  // Presence probe: inquiry MIT (kp=0) to every expected id. Bounded drain, no hang.
  std::printf("probe %d ticks (kp=0 inquiry, missing IDs timeout)...\n", kProbeTicks);
  std::unordered_set<uint8_t> present;
  for (int p = 0; p < kProbeTicks && g_run.load(); ++p)
  {
    auto lc = build_cmds(left_expected, {}, true);
    auto rc = build_cmds(right_expected, {}, true);
    DualBusCycleResult tick =
        dual.tick(left_open ? &lc : nullptr, right_open ? &rc : nullptr, drain, period, nullptr,
                  nullptr);
    auto mark = [&](const BusCycleResult &br) {
      for (const auto &m : br.motors)
      {
        if (m.rx_ok)
        {
          present.insert(m.can_id);
          auto *st = find_stats(stats, m.can_id);
          if (st != nullptr)
          {
            st->present_probe = true;
            if (m.have_fb)
            {
              st->last_q = m.fb.q;
              st->have_q = true;
            }
          }
        }
      }
    };
    mark(tick.left);
    mark(tick.right);
  }

  std::printf("probe present:");
  if (present.empty())
  {
    std::printf(" (none)");
  }
  for (uint8_t id : all_expected)
  {
    if (present.count(id) != 0)
    {
      auto *st = find_stats(stats, id);
      std::printf(" %u", static_cast<unsigned>(id));
      if (st != nullptr && st->have_q)
      {
        std::printf("(q=%.4f)", static_cast<double>(st->last_q));
      }
    }
  }
  std::printf("\nprobe missing:");
  bool any_missing = false;
  for (uint8_t id : all_expected)
  {
    if (present.count(id) == 0)
    {
      any_missing = true;
      std::printf(" %u", static_cast<unsigned>(id));
    }
  }
  if (!any_missing)
  {
    std::printf(" (none)");
  }
  std::printf("\n");

  std::vector<uint8_t> actually_enabled;
  if (opt.comms_only)
  {
    std::printf("comms-only CTRL_MODE / evo status (read-only; no write, no 0xFC):\n");
    for (uint8_t id : all_expected)
    {
      const auto *e = find_by_can_id(id);
      const bool here = present.count(id) != 0;
      if (e == nullptr)
      {
        continue;
      }
      if (e->model == MotorType::HA60)
      {
        std::printf("id=%u HA60 %s present=%d (no Damiao RID; evo servo kp=0 inquire)\n",
                    static_cast<unsigned>(id), to_string(e->joint), here ? 1 : 0);
        continue;
      }
      if (!here)
      {
        std::printf("id=%u %s %s missing; skip\n", static_cast<unsigned>(id),
                    to_string(e->model), to_string(e->joint));
        continue;
      }
      UsbCanAdapter *ad = adapter_for(id);
      FrameDemux *dm = demux_for(id, left_demux, right_demux);
      if (ad == nullptr || !ad->is_open() || dm == nullptr)
      {
        std::printf("id=%u skip (adapter not open)\n", static_cast<unsigned>(id));
        continue;
      }
      const LimitParam lim = lims[id & 0x0F];
      float mode_f = 0.f;
      const bool cm_ok =
          dm_read_rid(*ad, *dm, id, static_cast<uint8_t>(DmRid::CtrlMode), mode_f);
      const uint32_t cm = static_cast<uint32_t>(mode_f + 0.5f);
      std::printf("id=%u %s %s table PMAX=%.4f VMAX=%.4f TMAX=%.4f "
                  "CTRL_MODE=%s%u (%s)\n",
                  static_cast<unsigned>(id), to_string(e->model), to_string(e->joint),
                  static_cast<double>(lim.q_max), static_cast<double>(lim.dq_max),
                  static_cast<double>(lim.tau_max), cm_ok ? "" : "FAIL ",
                  static_cast<unsigned>(cm), cm_ok ? dm_ctrl_mode_name(cm) : "?");
    }
  }
  else
  {
  std::vector<uint8_t> enable_ids;
  for (uint8_t id : opt.ids)
  {
    if (opt.enable_all_anyway || present.count(id) != 0)
    {
      add_unique(enable_ids, id);
    }
    else
    {
      std::printf("skip enable id=%u (no probe feedback; pass --enable-all-anyway to override)\n",
                  static_cast<unsigned>(id));
    }
  }

  if (enable_ids.size() > 1)
  {
    std::printf(
        "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        "WARN: enabling %zu motors:",
        enable_ids.size());
    for (uint8_t id : enable_ids)
    {
      std::printf(" %u", static_cast<unsigned>(id));
    }
    std::printf(
        "\nOnly do this with joints free, powered, and mechanically safe.\n"
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
  }

  // CTRL_MODE + 0xFC for Damiao; Motorevo HA60 also 0xFC then conservative servo hold.
  for (uint8_t id : enable_ids)
  {
    const auto *e = find_by_can_id(id);
    if (e == nullptr)
    {
      continue;
    }
    UsbCanAdapter *ad = adapter_for(id);
    FrameDemux *dm = demux_for(id, left_demux, right_demux);
    if (ad == nullptr || !ad->is_open() || dm == nullptr)
    {
      std::printf("skip enable id=%u (adapter not open)\n", static_cast<unsigned>(id));
      continue;
    }
    if (e->model == MotorType::HA60)
    {
      uint8_t dis[8] = {};
      pack_motorevo_disable(dis);
      ad->sendCan(id, dis, 8);
      usleep(kDisableGapUs);
      ad->sendCan(id, dis, 8);
      usleep(kDisableGapUs);
      ad->drain();
      dm->clear();
      uint8_t en[8] = {};
      pack_motorevo_enable(en);
      if (!ad->sendCan(id, en, 8))
      {
        std::fprintf(stderr, "enable send failed id=%u\n", static_cast<unsigned>(id));
        continue;
      }
      std::printf("enable id=%u Motorevo HA60 servo (0xFC) hold kp=%.1f vel_kp=%.1f V_ref=%.2f\n",
                  static_cast<unsigned>(id), static_cast<double>(kHa60HoldPosKp),
                  static_cast<double>(kHa60HoldVelKp), static_cast<double>(kHa60HoldVelLimit));
      actually_enabled.push_back(id);
      continue;
    }

    const LimitParam lim = lims[id & 0x0F];
    std::printf("id=%u table PMAX=%.4f VMAX=%.4f TMAX=%.4f\n", static_cast<unsigned>(id),
                static_cast<double>(lim.q_max), static_cast<double>(lim.dq_max),
                static_cast<double>(lim.tau_max));
    DmMitEnableStatus st;
    const DmMitEnableErr en = dm_enable_mit(*ad, *dm, id, st);
    std::printf("id=%u CTRL_MODE before=%s%u write=%s after=%s%u (%s)\n",
                static_cast<unsigned>(id), st.ctrl_before_ok ? "" : "FAIL ",
                static_cast<unsigned>(st.ctrl_before), st.wrote_ack ? "ack" : "NO-ACK",
                st.ctrl_after_ok ? "" : "FAIL ", static_cast<unsigned>(st.ctrl_after),
                dm_ctrl_mode_name(st.ctrl_after));
    if (en == DmMitEnableErr::CtrlMode)
    {
      std::fprintf(stderr, "REFUSE enable id=%u: need CTRL_MODE=MIT(1)\n",
                   static_cast<unsigned>(id));
      continue;
    }
    if (en != DmMitEnableErr::Ok)
    {
      std::fprintf(stderr, "enable send failed id=%u\n", static_cast<unsigned>(id));
      continue;
    }
    std::printf("enable id=%u sent (0xFC)\n", static_cast<unsigned>(id));
    actually_enabled.push_back(id);
  }
  usleep(kEnableSettleUs);
  if (left_open)
  {
    left_ad.drain();
  }
  if (right_open && g_right != nullptr)
  {
    g_right->drain();
  }

  if (!actually_enabled.empty())
  {
    safety_arm();
  }
  }  // !comms_only
  std::unordered_set<uint8_t> enable_set(actually_enabled.begin(), actually_enabled.end());
  for (auto &s : stats)
  {
    s.enabled = enable_set.count(s.can_id) != 0;
    if (s.have_q)
    {
      s.q0 = s.last_q;
      s.q_des = s.last_q;
      s.q_min = s.last_q;
      s.q_max = s.last_q;
      s.have_q0 = true;
      s.dq_des = 0.f;
    }
  }

  const bool do_sine =
      !opt.comms_only && opt.stage == P3Stage::Sine && !actually_enabled.empty();
  if (do_sine)
  {
    std::printf("sine: no firmware-zero home; q_des=q0+%.3f*sin(phase), "
                "dq_des=%.3f*cos(phase)\n"
                "same phase/period: all enabled IDs each %d ms tick  abort-tau=%s\n"
                "q0",
                static_cast<double>(opt.stroke), static_cast<double>(opt.vel),
                opt.period_ms, abort_tau_buf);
    for (const auto &s : stats)
    {
      if (s.enabled && s.have_q0)
      {
        std::printf(" id%u=%.4f", static_cast<unsigned>(s.can_id), static_cast<double>(s.q0));
      }
    }
    std::printf("\n");
  }

  if (opt.comms_only)
  {
    std::printf("comms-only inquire loop (kp=0 MIT / HA60 evo status). 0xFC not sent.\n");
  }
  else if (actually_enabled.empty())
  {
    std::printf("no motors enabled; running inquiry-only (kp=0) for drop accounting\n");
  }

  CycleLogger left_log;
  CycleLogger right_log;
  PeriodJitterStats jitter{static_cast<double>(opt.period_ms) * 1000.0};
  std::uint64_t cycles = 0;
  std::uint64_t overruns = 0;
  bool aborted = false;
  const auto t0 = BusCycle::Clock::now();
  const auto duration = std::chrono::duration<double>(opt.duration_s);
  auto next = t0;
  const float abort_q = opt.stroke + kAbortQMargin;

  if (!opt.comms_only)
  {
    if (do_sine)
    {
      std::printf("sine loop start (q_des=q0+stroke*sin(phase), "
                  "dq_des=stroke*omega*cos(phase)). "
                  "SIGINT disables selected ids.\n");
    }
    else
    {
      std::printf("hold loop start (q=last, tau=0). SIGINT disables selected ids.\n");
    }
  }

  while (g_run.load() && !aborted && (BusCycle::Clock::now() - t0) < duration)
  {
    next += period;
    if (do_sine)
    {
      const double elapsed_s =
          std::chrono::duration<double>(BusCycle::Clock::now() - t0).count();
      const double omega =
          opt.stroke > 1e-6f ? static_cast<double>(opt.vel / opt.stroke) : 0.0;
      const double phase = omega * elapsed_s;
      for (auto &s : stats)
      {
        if (!s.enabled || !s.have_q0)
        {
          continue;
        }
        s.q_des = s.q0 + opt.stroke * std::sin(phase);
        s.dq_des = static_cast<float>(opt.stroke * omega * std::cos(phase));
      }
    }
    auto lc = build_cmds(left_expected, enable_set, opt.comms_only);
    auto rc = build_cmds(right_expected, enable_set, opt.comms_only);
    const auto t_send = BusCycle::Clock::now();
    DualBusCycleResult tick =
        dual.tick(left_open ? &lc : nullptr, right_open ? &rc : nullptr, drain, period,
                  left_open ? &left_log : nullptr, right_open ? &right_log : nullptr);
    accumulate(stats, tick.left);
    accumulate(stats, tick.right);
    ++cycles;
    if (tick.overrun)
    {
      ++overruns;
      std::fprintf(stderr,
                   "ABORT communication timeout: cycle elapsed=%.3fms > period=%dms\n",
                   tick.elapsed_ms, opt.period_ms);
      safety_set_reason(SafetyReason::CommunicationTimeout);
      aborted = true;
    }
    const double dt_us =
        std::chrono::duration<double, std::micro>(BusCycle::Clock::now() - t_send).count();
    jitter.push(dt_us);

    if (do_sine)
    {
      for (const auto &s : stats)
      {
        if (!s.enabled || !s.have_q)
        {
          continue;
        }
        const float tau_lim = abort_tau_nm(opt.abort_tau, s.can_id, lims[s.can_id & 0x0F].tau_max);
        if (std::fabs(s.last_tau) > tau_lim)
        {
          std::fprintf(stderr, "ABORT id=%u: |tau_fb|=%.3f > %.3f Nm\n",
                       static_cast<unsigned>(s.can_id),
                       static_cast<double>(std::fabs(s.last_tau)),
                       static_cast<double>(tau_lim));
          safety_set_reason(SafetyReason::HardLimitTau);
          aborted = true;
        }
        if (s.have_q0 && std::fabs(s.last_q - s.q0) > abort_q)
        {
          std::fprintf(stderr, "ABORT id=%u: |q-q0|=%.4f > stroke+margin %.4f\n",
                       static_cast<unsigned>(s.can_id),
                       static_cast<double>(std::fabs(s.last_q - s.q0)),
                       static_cast<double>(abort_q));
          safety_set_reason(SafetyReason::HardLimitQ);
          aborted = true;
        }
      }
    }

    if (opt.verbose)
    {
      std::printf("cycle %llu elapsed=%.2fms overrun=%d  ",
                  static_cast<unsigned long long>(cycles), tick.elapsed_ms,
                  tick.overrun ? 1 : 0);
      auto dump = [&](const BusCycleResult &br) {
        for (const auto &m : br.motors)
        {
          if (m.skipped)
          {
            std::printf("id%u=IGN ", static_cast<unsigned>(m.can_id));
          }
          else if (m.rx_ok)
          {
            if (do_sine)
            {
              auto *st = find_stats(stats, m.can_id);
              const float qdes = st != nullptr ? st->q_des : 0.f;
              std::printf("id%u=ok(rtt=%.2f q=%.4f qdes=%.4f tau=%.3f) ",
                          static_cast<unsigned>(m.can_id), m.rtt_ms,
                          static_cast<double>(m.have_fb ? m.fb.q : 0.f),
                          static_cast<double>(qdes),
                          static_cast<double>(m.have_fb ? m.fb.tau : 0.f));
            }
            else
            {
              std::printf("id%u=ok(rtt=%.2f%s) ", static_cast<unsigned>(m.can_id), m.rtt_ms,
                          m.have_fb ? "" : "");
            }
          }
          else
          {
            std::printf("id%u=DROP ", static_cast<unsigned>(m.can_id));
          }
        }
      };
      dump(tick.left);
      dump(tick.right);
      std::printf("\n");
    }

    const auto remain = next - BusCycle::Clock::now();
    if (remain > std::chrono::milliseconds(1))
    {
      usleep(static_cast<useconds_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(remain).count()));
    }
  }

  print_summary_table(stats, overruns, cycles);
  if (do_sine)
  {
    print_sine_table(stats, opt.stroke);
    std::printf("sine: %s\n", aborted ? "ABORT" : "clean (duration)");
  }
  std::printf("jitter_us mean=%.1f max_dev=%.1f  (period=%d ms)\n", jitter.mean_us(),
              jitter.max_deviation_us(), opt.period_ms);
  if (opt.comms_only)
  {
    std::printf("comms-only errors (fb.err != 0 during inquire):\n");
    for (const auto &s : stats)
    {
      std::printf(" id=%u present_probe=%d sent=%llu rx=%llu drop=%llu err_n=%llu last_err=%u\n",
                  static_cast<unsigned>(s.can_id), s.present_probe ? 1 : 0,
                  static_cast<unsigned long long>(s.sent),
                  static_cast<unsigned long long>(s.rx),
                  static_cast<unsigned long long>(s.drop),
                  static_cast<unsigned long long>(s.err_n),
                  static_cast<unsigned>(s.last_err));
    }
    std::printf("comms-only: 0xFC sent=0\n");
    for (uint8_t id : all_expected)
    {
      UsbCanAdapter *ad = adapter_for(id);
      if (ad != nullptr && ad->is_open())
      {
        dm_send_disable(*ad, id);
        std::printf("disable id=%u sent (0xFD hygiene)\n", static_cast<unsigned>(id));
      }
    }
  }

  if (opt.app == P3App::FaultInject)
  {
    std::printf(
        "fault-inject note: ignored IDs are not waited on; other IDs should still update\n"
        "if they are on the bus. Loop is bounded by --drain-ms (does not hang).\n"
        "Exit uses P5 disable-all; this loop only recorded drops.\n");
    bool others_rx = false;
    bool ignored_waited = false;
    for (const auto &s : stats)
    {
      if (contains_id(opt.ignore_ids, s.can_id))
      {
        if (s.sent > 0 || s.rx > 0)
        {
          ignored_waited = true;
        }
        continue;
      }
      if (s.rx > 0)
      {
        others_rx = true;
      }
    }
    std::printf("other_ids_got_rx=%d  ignored_id_still_sent_or_rx=%d\n", others_rx ? 1 : 0,
                ignored_waited ? 1 : 0);
  }

  disable_all_now(true);
  if (left_open)
  {
    left_ad.close();
  }
  if (g_right != nullptr)
  {
    g_right->close();
  }
  g_left = nullptr;
  g_right = nullptr;
  safety_bind_buses(nullptr, nullptr);

  int rc = aborted ? 1 : 0;
  if (opt.require_all)
  {
    for (const auto &s : stats)
    {
      if (s.skipped > 0 && s.sent == 0)
      {
        continue;
      }
      if (s.rx == 0)
      {
        std::fprintf(stderr, "--require-all: id=%u had zero RX\n",
                     static_cast<unsigned>(s.can_id));
        rc = 1;
      }
    }
  }
  bool selected_rx = false;
  for (uint8_t id : opt.ids)
  {
    auto *s = find_stats(stats, id);
    if (s != nullptr && s->rx > 0)
    {
      selected_rx = true;
    }
    if (s == nullptr && present.count(id) != 0)
    {
      selected_rx = true;
    }
  }
  if (!opt.ids.empty() && !selected_rx && opt.ignore_ids.empty())
  {
    std::fprintf(stderr, "none of --ids got RX during the loop\n");
    rc = 1;
  }
  return rc;
}

}  // namespace biped_hw
