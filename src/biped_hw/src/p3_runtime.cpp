// biped_hw — p3_runtime.cpp
// P3 运行时：探测在线 ID、低 kp hold、write-all/drain-all，缺电机超时记表。
// 默认只使能 --ids（通常为 6）；--ids 含 HA60 时纳入伺服 hold。P5 才做通信丢失 disable-all。
#include "biped_hw/p3_runtime.hpp"

#include "biped_hw/bus_cycle.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/metrics.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <unistd.h>
#include <unordered_set>

namespace biped_hw
{
namespace
{

constexpr int kEnableSettleUs = 100000;
constexpr int kDisableGapUs = 10000;
constexpr float kKpCap = 20.f;
constexpr float kKdCap = 1.f;
constexpr int kProbeTicks = 3;
// P2 HA60 conservative servo hold (vel_kp=0 cannot track/hold θ_ref).
constexpr float kHa60HoldPosKp = 8.f;
constexpr float kHa60HoldPosKd = 0.5f;
constexpr float kHa60HoldVelLimit = 0.5f;
constexpr float kHa60HoldVelKp = 20.f;
constexpr float kHa60HoldVelKd = 0.f;
constexpr float kHa60HoldVelKi = 0.001f;

std::atomic<bool> g_run{true};
UsbCanAdapter *g_left = nullptr;
UsbCanAdapter *g_right = nullptr;
uint8_t g_disable_ids[16] = {};
std::size_t g_disable_n = 0;

void on_signal(int)
{
  g_run.store(false);
}

bool is_damiao(MotorType t)
{
  return t == MotorType::DM10010L || t == MotorType::DM10010 || t == MotorType::DM10422;
}

bool contains_id(const std::vector<uint8_t> &ids, uint8_t id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

void send_disable(UsbCanAdapter *a, uint8_t id)
{
  if (a == nullptr || !a->is_open())
  {
    return;
  }
  uint8_t buf[8] = {};
  pack_disable(buf);
  a->sendCan(id, buf, 8);
  usleep(kDisableGapUs);
  a->sendCan(id, buf, 8);
}

void disable_all_now(bool print)
{
  for (std::size_t i = 0; i < g_disable_n; ++i)
  {
    const uint8_t id = g_disable_ids[i];
    const auto *e = find_by_can_id(id);
    UsbCanAdapter *a = nullptr;
    if (e != nullptr && e->bus == BusId::Right)
    {
      a = g_right;
    }
    else
    {
      a = g_left;
    }
    send_disable(a, id);
    if (print)
    {
      std::printf("disable id=%u sent (0xFD)\n", static_cast<unsigned>(id));
    }
  }
  g_disable_n = 0;
}

void atexit_disable()
{
  disable_all_now(false);
}

void arm_disable(const std::vector<uint8_t> &ids)
{
  g_disable_n = 0;
  for (uint8_t id : ids)
  {
    if (g_disable_n >= 16)
    {
      break;
    }
    g_disable_ids[g_disable_n++] = id;
  }
}

std::string trim_copy(std::string s)
{
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos)
  {
    return {};
  }
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

bool yaml_ports(const std::string &path, std::string &left, std::string &right)
{
  std::ifstream in(path);
  if (!in)
  {
    return false;
  }
  std::string line;
  int section = 0;  // 1 left, 2 right
  bool got = false;
  while (std::getline(in, line))
  {
    const auto hash = line.find('#');
    if (hash != std::string::npos)
    {
      line = line.substr(0, hash);
    }
    const std::string t = trim_copy(line);
    if (t.empty())
    {
      continue;
    }
    if (t.find("left:") == 0)
    {
      section = 1;
      continue;
    }
    if (t.find("right:") == 0)
    {
      section = 2;
      continue;
    }
    if (t.find("port:") != 0)
    {
      continue;
    }
    const std::string v = trim_copy(t.substr(5));
    if (section == 1)
    {
      left = v;
      got = true;
    }
    else if (section == 2)
    {
      right = v;
      got = true;
    }
  }
  return got;
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

const char *need_arg(int &i, int argc, char **argv, const char *flag, std::string &err)
{
  if (i + 1 >= argc)
  {
    err = std::string(flag) + " needs a value";
    return nullptr;
  }
  return argv[++i];
}

bool parse_bool_flag(const char *v, bool &out)
{
  if (v == nullptr)
  {
    out = true;
    return true;
  }
  if (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0 || std::strcmp(v, "yes") == 0)
  {
    out = true;
    return true;
  }
  if (std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0 || std::strcmp(v, "no") == 0)
  {
    out = false;
    return true;
  }
  return false;
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

FrameDemux *demux_for(uint8_t can_id, FrameDemux &left, FrameDemux &right)
{
  const auto *e = find_by_can_id(can_id);
  if (e != nullptr && e->bus == BusId::Right)
  {
    return &right;
  }
  return &left;
}

bool wait_register(UsbCanAdapter &adapter, FrameDemux &demux, uint8_t esc_id, uint8_t rid,
                   DmRegisterReply &reply, int timeout_ms)
{
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    CanRxFrame rx;
    if (adapter.pollCan(rx))
    {
      demux.push(rx);
    }
    if (demux.take_register(esc_id, rid, reply))
    {
      return true;
    }
  }
  return false;
}

bool read_rid_retries(UsbCanAdapter &adapter, FrameDemux &demux, uint8_t esc_id, uint8_t rid,
                      float &value)
{
  for (int i = 0; i < 3; ++i)
  {
    demux.clear();
    uint8_t buf[8] = {};
    pack_register_read(buf, esc_id, rid);
    if (!adapter.sendCan(kDmRegCanId, buf, 8))
    {
      continue;
    }
    DmRegisterReply reply{};
    if (wait_register(adapter, demux, esc_id, rid, reply, 200))
    {
      value = reply.f;
      return true;
    }
  }
  return false;
}

bool write_rid_u32_retries(UsbCanAdapter &adapter, FrameDemux &demux, uint8_t esc_id,
                           uint8_t rid, uint32_t value)
{
  for (int i = 0; i < 3; ++i)
  {
    demux.clear();
    uint8_t buf[8] = {};
    pack_register_write_u32(buf, esc_id, rid, value);
    if (!adapter.sendCan(kDmRegCanId, buf, 8))
    {
      continue;
    }
    DmRegisterReply reply{};
    if (wait_register(adapter, demux, esc_id, rid, reply, 200))
    {
      if (reply.u == value || static_cast<uint32_t>(reply.f + 0.5f) == value)
      {
        return true;
      }
    }
  }
  return false;
}

std::vector<uint8_t> expected_ids(BusId bus, bool skip_motorevo)
{
  std::vector<uint8_t> ids;
  for (const auto &e : kMotorMap)
  {
    if (e.bus != bus)
    {
      continue;
    }
    if (e.model == MotorType::HA60 && skip_motorevo)
    {
      continue;
    }
    if (e.model == MotorType::HA60)
    {
      continue;  // Motorevo only if later added to enable set
    }
    if (is_damiao(e.model))
    {
      ids.push_back(e.can_id);
    }
  }
  return ids;
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
  bool have_q = false;
  bool enabled = false;
  bool present_probe = false;
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
        s->have_q = true;
      }
    }
    else if (m.drop)
    {
      ++s->drop;
    }
  }
}

void pack_cmd_for(CycleMotorCmd &c, const MotorMapEntry &e, const LimitParam &lim, bool send,
                  bool expect, bool enabled, float kp, float kd, float q_hold, bool have_q)
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
    const float q = have_q ? q_hold : 0.f;
    if (!enabled)
    {
      pack_motorevo_servo(c.data, q, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
      return;
    }
    pack_motorevo_servo(c.data, q, kHa60HoldPosKp, kHa60HoldPosKd, kHa60HoldVelLimit,
                        kHa60HoldVelKp, kHa60HoldVelKd, kHa60HoldVelKi);
    return;
  }
  const float q = (enabled && have_q) ? q_hold : 0.f;
  const float use_kp = enabled ? kp : 0.f;
  const float use_kd = enabled ? kd : 0.f;
  pack_mit(c.data, use_kp, use_kd, q, 0.f, 0.f, lim);
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
  }
  return "p3";
}

void p3_print_usage(const P3Options &defaults)
{
  const char *bin = p3_bin_name(defaults.app);
  std::printf(
      "Usage: %s [options]\n"
      "  P3 same-cycle write-all / drain-all. Missing motors timeout (drop), loop continues.\n"
      "  --period-ms MS        default %d\n"
      "  --duration SEC        default %.1f\n"
      "  --drain-ms MS         RX wait cap per cycle (default period-2)\n"
      "  --kp --kd             MIT hold gains (default %.1f / %.2f, cap kp<=%.0f)\n"
      "  --ids 6               enable set (default 6). Example: --ids 6,8 or 3,4,5,6,7\n"
      "                        If given, cycle/summary is only those IDs (unlisted stay off).\n"
      "  --skip-motorevo [0|1] default 1; listing HA60 in --ids includes it (do not skip)\n"
      "  --no-skip-motorevo    include HA60 only if also in --ids\n"
      "  --require-all         exit 1 if any expected id has zero RX\n"
      "  --enable-all-anyway   enable --ids even if probe never replied\n"
      "  --port PATH           left USB-CAN (left6 / fault)\n"
      "  --left-port --right-port --yaml PATH\n"
      "  --handshake --verbose --self-test\n",
      bin, defaults.period_ms, defaults.duration_s, static_cast<double>(defaults.kp),
      static_cast<double>(defaults.kd), static_cast<double>(kKpCap));
  if (defaults.app == P3App::Dual12)
  {
    std::printf(
        "  --allow-single-bus    if ttyACM1 missing: error, then run left only\n");
  }
  if (defaults.app == P3App::FaultInject)
  {
    std::printf(
        "  --ignore-id N         simulate drop: do not send and do not wait (repeatable)\n"
        "  P3 only records whether other IDs still update and whether the loop blocks.\n"
        "  P5 will disable-all on comm-loss; that policy is NOT implemented here.\n");
  }
  std::printf(
      "  Safety: default --ids 6. Big warning if enabling more than one motor.\n"
      "  Without --ids: probe all Damiao on opened buses (3-7 / 9-13); skip Motorevo.\n"
      "  With --ids: only those CAN IDs are sent/expected/enabled (HA60 if listed).\n"
      "  Disable 0xFD on selected ids at exit/SIGINT.\n");
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
    else if (std::strcmp(argv[i], "--skip-motorevo") == 0)
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        if (!parse_bool_flag(argv[++i], opt.skip_motorevo))
        {
          err = "--skip-motorevo expects 0 or 1";
          return false;
        }
      }
      else
      {
        opt.skip_motorevo = true;
      }
    }
    else if (std::strcmp(argv[i], "--no-skip-motorevo") == 0)
    {
      opt.skip_motorevo = false;
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
    else if (std::strcmp(argv[i], "--port") == 0)
    {
      const char *v = need("--port");
      if (v == nullptr)
      {
        return false;
      }
      opt.left_port = v;
      left_port_set = true;
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
    if (!yaml_ports(opt.yaml_path, yl, yr))
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
    std::printf("WARN: clamping kp %.1f -> %.1f (P3 hold is low-gain)\n",
                static_cast<double>(opt.kp), static_cast<double>(kKpCap));
    opt.kp = kKpCap;
  }
  if (opt.kd < 0.f)
  {
    opt.kd = 0.f;
  }
  if (opt.kd > kKdCap)
  {
    std::printf("WARN: clamping kd %.2f -> %.2f (P3 hold is low-gain)\n",
                static_cast<double>(opt.kd), static_cast<double>(kKdCap));
    opt.kd = kKdCap;
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

  const bool want_right = opt.app == P3App::Dual12;
  const bool left_only = !want_right;

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
    if (e->model == MotorType::HA60)
    {
      listed_ha60 = true;
    }
  }
  if (listed_ha60 && opt.skip_motorevo)
  {
    std::printf("NOTE: --ids includes Motorevo HA60; including servo hold (skip_motorevo=0)\n");
    opt.skip_motorevo = false;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::atexit(atexit_disable);

  UsbCanAdapter left_ad(opt.left_port);
  UsbCanAdapter right_ad(opt.right_port);
  if (!left_ad.open())
  {
    std::fprintf(stderr, "open left %s failed: %s\n", opt.left_port.c_str(),
                 left_ad.last_error().c_str());
    return 1;
  }
  std::printf("opened left %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n",
              opt.left_port.c_str(), left_ad.can_bitrate(),
              static_cast<unsigned>(left_ad.can_baud_index()));
  g_left = &left_ad;

  bool right_open = false;
  if (want_right)
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

  if (opt.handshake)
  {
    left_ad.handshake();
    usleep(20000);
    left_ad.drain();
    if (right_open)
    {
      right_ad.handshake();
      usleep(20000);
      right_ad.drain();
    }
    std::printf("handshake sent (CMD 0x02)\n");
  }

  std::vector<uint8_t> left_expected;
  std::vector<uint8_t> right_expected;
  if (opt.ids_set)
  {
    // Explicit --ids: cycle/summary/enable only those IDs (do not probe unlisted 3-7).
    for (uint8_t id : opt.ids)
    {
      const auto *e = find_by_can_id(id);
      if (e == nullptr)
      {
        continue;
      }
      if (e->model == MotorType::HA60 && opt.skip_motorevo)
      {
        continue;
      }
      if (e->bus == BusId::Left)
      {
        add_unique(left_expected, id);
      }
      else if (right_open)
      {
        add_unique(right_expected, id);
      }
    }
  }
  else
  {
    left_expected = expected_ids(BusId::Left, opt.skip_motorevo);
    right_expected = right_open ? expected_ids(BusId::Right, opt.skip_motorevo)
                                 : std::vector<uint8_t>{};
    // Motorevo enters expected set only when we will actually send to it.
    if (!opt.skip_motorevo)
    {
      for (uint8_t id : opt.ids)
      {
        if (is_motorevo_can_id(id))
        {
          const auto *e = find_by_can_id(id);
          if (e == nullptr)
          {
            continue;
          }
          if (e->bus == BusId::Left)
          {
            add_unique(left_expected, id);
          }
          else if (right_open)
          {
            add_unique(right_expected, id);
          }
        }
      }
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
  BusCycle right_cycle(right_ad, BusId::Right, right_demux);
  DualBusCycle dual(&left_cycle, right_open ? &right_cycle : nullptr);

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
      CycleMotorCmd c;
      pack_cmd_for(c, *e, lims[id & 0x0F], !ignore, !ignore, en, opt.kp, opt.kd,
                   st != nullptr ? st->last_q : 0.f, st != nullptr && st->have_q);
      cmds.push_back(c);
    }
    return cmds;
  };

  const auto period = std::chrono::milliseconds(opt.period_ms);
  const auto drain = std::chrono::milliseconds(opt.drain_ms);

  std::printf(
      "P3 %s  period=%dms drain=%dms duration=%.1fs kp=%.1f kd=%.2f skip_motorevo=%d\n"
      "expected left:",
      p3_bin_name(opt.app), opt.period_ms, opt.drain_ms, opt.duration_s,
      static_cast<double>(opt.kp), static_cast<double>(opt.kd), opt.skip_motorevo ? 1 : 0);
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
    std::printf("  (P3 records only; P5 will disable-all)\n");
  }

  // Presence probe: inquiry MIT (kp=0) to every expected id. Bounded drain, no hang.
  std::printf("probe %d ticks (kp=0 inquiry, missing IDs timeout)...\n", kProbeTicks);
  std::unordered_set<uint8_t> present;
  for (int p = 0; p < kProbeTicks && g_run.load(); ++p)
  {
    auto lc = build_cmds(left_expected, {}, true);
    auto rc = build_cmds(right_expected, {}, true);
    DualBusCycleResult tick;
    if (want_right)
    {
      tick = dual.tick(&lc, right_open ? &rc : nullptr, drain, period, nullptr, nullptr);
    }
    else
    {
      tick.left = left_cycle.tick(lc, drain, period, nullptr);
      tick.left_used = true;
    }
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

  // RID + CTRL_MODE + 0xFC for Damiao; Motorevo HA60 also 0xFC then conservative servo hold.
  std::vector<uint8_t> actually_enabled;
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

    if (e->model == MotorType::DM10422)
    {
      float pmax = 0.f, vmax = 0.f, tmax = 0.f;
      const bool pok =
          read_rid_retries(*ad, *dm, id, static_cast<uint8_t>(DmRid::Pmax), pmax);
      const bool vok =
          read_rid_retries(*ad, *dm, id, static_cast<uint8_t>(DmRid::Vmax), vmax);
      const bool tok =
          read_rid_retries(*ad, *dm, id, static_cast<uint8_t>(DmRid::Tmax), tmax);
      std::printf("id=%u RID 21/22/23 PMAX=%s%.4f VMAX=%s%.4f TMAX=%s%.4f\n",
                  static_cast<unsigned>(id), pok ? "" : "FAIL ", static_cast<double>(pmax),
                  vok ? "" : "FAIL ", static_cast<double>(vmax), tok ? "" : "FAIL ",
                  static_cast<double>(tmax));
      if (!(pok && vok && tok && pmax > 0.1f && vmax > 0.1f && tmax > 0.1f))
      {
        std::fprintf(stderr,
                     "REFUSE enable id=%u: DM10422 limits UNVERIFIED (RID 21/22/23 failed)\n",
                     static_cast<unsigned>(id));
        continue;
      }
      lims[id & 0x0F] = LimitParam{pmax, vmax, tmax};
    }

    uint8_t dis[8] = {};
    pack_disable(dis);
    ad->sendCan(id, dis, 8);
    usleep(kDisableGapUs);
    ad->sendCan(id, dis, 8);
    usleep(kDisableGapUs);
    ad->drain();
    dm->clear();

    float mode_f = 0.f;
    const bool before_ok =
        read_rid_retries(*ad, *dm, id, static_cast<uint8_t>(DmRid::CtrlMode), mode_f);
    const uint32_t before = static_cast<uint32_t>(mode_f + 0.5f);
    const bool wrote = write_rid_u32_retries(*ad, *dm, id, static_cast<uint8_t>(DmRid::CtrlMode),
                                             kDmMitMode);
    usleep(20000);
    mode_f = 0.f;
    const bool after_ok =
        read_rid_retries(*ad, *dm, id, static_cast<uint8_t>(DmRid::CtrlMode), mode_f);
    const uint32_t after = static_cast<uint32_t>(mode_f + 0.5f);
    std::printf("id=%u CTRL_MODE before=%s%u write=%s after=%s%u (%s)\n",
                static_cast<unsigned>(id), before_ok ? "" : "FAIL ",
                static_cast<unsigned>(before), wrote ? "ack" : "NO-ACK",
                after_ok ? "" : "FAIL ", static_cast<unsigned>(after),
                dm_ctrl_mode_name(after));
    if (!after_ok || after != kDmMitMode)
    {
      std::fprintf(stderr, "REFUSE enable id=%u: need CTRL_MODE=MIT(1)\n",
                   static_cast<unsigned>(id));
      continue;
    }

    uint8_t en[8] = {};
    pack_enable(en);
    if (!ad->sendCan(id, en, 8))
    {
      std::fprintf(stderr, "enable send failed id=%u\n", static_cast<unsigned>(id));
      continue;
    }
    std::printf("enable id=%u sent (0xFC)\n", static_cast<unsigned>(id));
    actually_enabled.push_back(id);
  }
  usleep(kEnableSettleUs);
  left_ad.drain();
  if (right_open)
  {
    right_ad.drain();
  }

  arm_disable(actually_enabled);
  std::unordered_set<uint8_t> enable_set(actually_enabled.begin(), actually_enabled.end());
  for (auto &s : stats)
  {
    s.enabled = enable_set.count(s.can_id) != 0;
  }

  if (actually_enabled.empty())
  {
    std::printf("no motors enabled; running inquiry-only (kp=0) for drop accounting\n");
  }

  CycleLogger left_log;
  CycleLogger right_log;
  PeriodJitterStats jitter{static_cast<double>(opt.period_ms) * 1000.0};
  std::uint64_t cycles = 0;
  std::uint64_t overruns = 0;
  const auto t0 = BusCycle::Clock::now();
  const auto duration = std::chrono::duration<double>(opt.duration_s);
  auto next = t0;
  auto last_print = t0;

  std::printf("hold loop start (q=last, tau=0). SIGINT disables selected ids.\n");

  while (g_run.load() && (BusCycle::Clock::now() - t0) < duration)
  {
    next += period;
    auto lc = build_cmds(left_expected, enable_set, false);
    auto rc = build_cmds(right_expected, enable_set, false);
    DualBusCycleResult tick;
    const auto t_send = BusCycle::Clock::now();
    if (want_right)
    {
      tick = dual.tick(&lc, right_open ? &rc : nullptr, drain, period, &left_log, &right_log);
    }
    else
    {
      tick.left = left_cycle.tick(lc, drain, period, &left_log);
      tick.left_used = true;
      tick.elapsed_ms = tick.left.elapsed_ms;
      tick.overrun = tick.left.overrun;
    }
    accumulate(stats, tick.left);
    accumulate(stats, tick.right);
    ++cycles;
    if (tick.overrun)
    {
      ++overruns;
    }
    const double dt_us =
        std::chrono::duration<double, std::micro>(BusCycle::Clock::now() - t_send).count();
    jitter.push(dt_us);

    const auto now = BusCycle::Clock::now();
    const bool print_now =
        opt.verbose || cycles <= 3 ||
        (now - last_print) >= std::chrono::milliseconds(500);
    if (print_now)
    {
      std::printf("cycle %llu elapsed=%.2fms overrun=%d  ",
                  static_cast<unsigned long long>(cycles), tick.elapsed_ms,
                  tick.overrun ? 1 : 0);
      auto dump = [](const BusCycleResult &br) {
        for (const auto &m : br.motors)
        {
          if (m.skipped)
          {
            std::printf("id%u=IGN ", static_cast<unsigned>(m.can_id));
          }
          else if (m.rx_ok)
          {
            std::printf("id%u=ok(rtt=%.2f%s) ", static_cast<unsigned>(m.can_id), m.rtt_ms,
                        m.have_fb ? "" : "");
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
      last_print = now;
    }

    const auto remain = next - BusCycle::Clock::now();
    if (remain > std::chrono::milliseconds(1))
    {
      usleep(static_cast<useconds_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(remain).count()));
    }
  }

  print_summary_table(stats, overruns, cycles);
  std::printf("jitter_us mean=%.1f max_dev=%.1f  (period=%d ms)\n", jitter.mean_us(),
              jitter.max_deviation_us(), opt.period_ms);

  if (opt.app == P3App::FaultInject)
  {
    std::printf(
        "fault-inject note: ignored IDs are not waited on; other IDs should still update\n"
        "if they are on the bus. Loop is bounded by --drain-ms (does not hang).\n"
        "P5 will disable-all on comm-loss; P3 only recorded drops.\n");
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
  g_left = nullptr;
  g_right = nullptr;
  left_ad.close();
  if (right_open)
  {
    right_ad.close();
  }

  int rc = 0;
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
    std::fprintf(stderr, "none of --ids got RX during the hold loop\n");
    rc = 1;
  }
  return rc;
}

}  // namespace biped_hw
