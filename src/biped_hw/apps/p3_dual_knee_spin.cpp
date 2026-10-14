// 双膝电机测试程序（DM10422）
// 双总线 Damiao MIT（默认左 ACM0 CAN 6 / 右 ACM1 CAN 12；--ids / --left-id /--right-id 可改）。map 内 Damiao MIT + Master 2 即可。
// 默认 --stage triangle：q_des += vel*dt，到 ±stroke 反向。 
// 只使能这两路，不碰其它 ID。
// 必须打开两路 USB2CAN；缺右总线直接失败。CTRL_MODE=MIT、RID 21/22/23。退出发 0xFD。
#include "biped_hw/bus_cycle.hpp"
#include "biped_hw/cli.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/metrics.hpp"
#include "biped_hw/motor_map.hpp"
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
#include <vector>

namespace
{

constexpr uint8_t kLeftId = 6;
constexpr uint8_t kRightId = 12;
constexpr int kEnableSettleUs = 100000;
constexpr int kDisableGapUs = 10000;
constexpr int kProbeTicks = 20;          // dual-bus; 3 was too short for ACM1
constexpr float kKpDefault = 80.f;
constexpr float kKdDefault = 1.f;
constexpr float kKpCap = 200.f;  // 达妙 MIT kp 0-500；P3 hold 的 20 会夹死斜坡
constexpr float kKdCap = 5.f;    // Damiao MIT kd pack 0-5
constexpr float kVelDefault = 0.15f;     // rad/s ≈ 8.6 deg/s
constexpr float kStrokeDefault = 0.4f;   // rad；来回摆，避免齿轮/线驱无限转
constexpr float kAbortTauDefault = 30.f; // Nm
constexpr float kAbortQMarginDefault = 0.1f;
constexpr double kDurationDefault = 120.0;

std::atomic<bool> g_run{true};
biped_hw::UsbCanAdapter *g_left = nullptr;
biped_hw::UsbCanAdapter *g_right = nullptr;

void on_signal(int)
{
  g_run.store(false);
  biped_hw::safety_estop_trip();
}

void disable_all_now(bool print)
{
  const auto r = biped_hw::safety_estop_latched() ? biped_hw::SafetyReason::Estop
                                                  : biped_hw::safety_last_reason();
  biped_hw::safety_disable_all(
      g_left, g_right, nullptr, 0,
      r == biped_hw::SafetyReason::None ? biped_hw::SafetyReason::Estop : r, print);
}

void atexit_disable()
{
  biped_hw::safety_atexit_disable();
}


struct Knee
{
  uint8_t can_id = 0;
  biped_hw::MotorType model = biped_hw::MotorType::DM10422;
  biped_hw::LimitParam lim{};
  float q0 = 0.f;
  float q_des = 0.f;
  float vel_sign = 1.f;
  float last_q = 0.f;
  float last_tau = 0.f;
  float last_dq = 0.f;
  float q_peak_abs = 0.f;
  float tau_peak_abs = 0.f;
  bool have_q = false;
  bool present = false;
  bool enabled = false;
  std::uint64_t sent = 0;
  std::uint64_t rx = 0;
  std::uint64_t drop = 0;
  double rtt_sum = 0.0;
  double rtt_min = 1e9;
  double rtt_max = 0.0;
};

Knee *find_knee(Knee *knees, std::size_t n, uint8_t id)
{
  for (std::size_t i = 0; i < n; ++i)
  {
    if (knees[i].can_id == id)
    {
      return &knees[i];
    }
  }
  return nullptr;
}

void accumulate(Knee *knees, std::size_t n, const biped_hw::BusCycleResult &r)
{
  for (const auto &m : r.motors)
  {
    Knee *k = find_knee(knees, n, m.can_id);
    if (k == nullptr || m.skipped)
    {
      continue;
    }
    if (m.sent)
    {
      ++k->sent;
    }
    if (m.rx_ok)
    {
      ++k->rx;
      k->rtt_sum += m.rtt_ms;
      if (m.rtt_ms < k->rtt_min)
      {
        k->rtt_min = m.rtt_ms;
      }
      if (m.rtt_ms > k->rtt_max)
      {
        k->rtt_max = m.rtt_ms;
      }
      if (m.have_fb)
      {
        k->last_q = m.fb.q;
        k->last_tau = m.fb.tau;
        k->last_dq = m.fb.dq;
        k->have_q = true;
        const float dev = std::fabs(m.fb.q - k->q0);
        if (dev > k->q_peak_abs)
        {
          k->q_peak_abs = dev;
        }
        const float tabs = std::fabs(m.fb.tau);
        if (tabs > k->tau_peak_abs)
        {
          k->tau_peak_abs = tabs;
        }
      }
    }
    else if (m.drop)
    {
      ++k->drop;
    }
  }
}

void print_usage()
{
  std::printf(
      "Usage: p3_dual_knee_spin [options]\n"
      "  Dual-bus Damiao MIT (default left /dev/ttyACM0 CAN 6, right /dev/ttyACM1 CAN 12).\n"
      "  Mapped Damiao MIT Master 2; joint names not checked.\n"
      "  Requires both USB2CAN. Enables ONLY the two selected IDs (default 6 and 12).\n"
      "  --stage triangle|dq   default triangle. triangle: q_des += vel*dt, reverse at\n"
      "                        ±stroke (kp from --kp). dq matches Damiao 上位机 MIT:\n"
      "                        kp=0, kd from --kd (e.g. 2), dq_des from --vel (rad/s,\n"
      "                        up to VMAX e.g. 5; not 5%% of VMAX), q_des=0, tau=0.\n"
      "                        Periodic send (--period-ms). Hard stop: --duration OR\n"
      "                        |q-q0|>=--stroke then 0xFD both. Example:\n"
      "                        --stage dq --vel 5 --kd 2 --stroke 8 --duration 3 --abort-tau 15\n"
      "                        Before triangle/dq: each selected ID is probed; if\n"
      "                        |q|>%.3f rad (kDmHomeEpsRad), speed-limited MIT return\n"
      "                        to 0 (kp=0, kd from --kd default %.0f, dq_des=-sign(q)*vel,\n"
      "                        vel from --vel capped %.0f–%.0f rad/s) until |q|<=eps,\n"
      "                        timeout max(%.0f, |q|/min(vel,%.0f)+%.0f) cap %.0f s, or abort-tau.\n"
      "                        Then the loop. Homing fail → P5 disable-all 0xFD, no\n"
      "                        profile. Skipped for --set-zero / --self-test. Not 0xFE.\n"
      "  --ids L,R             left,right CAN IDs (default %u,%u); same as --left-id/--right-id\n"
      "  --left-id N           left-bus ESC (default %u)\n"
      "  --right-id N          right-bus ESC (default %u); may equal left-id (separate buses)\n"
      "  --duration SEC        default %.0f (2 min)\n"
      "  --vel RAD/S           triangle ramp rate; dq: MIT dq_des rad/s up to VMAX\n"
      "                        (default %.2f; host uses 5; not 5%% of VMAX)\n"
      "  --stroke RAD          triangle: reverse at |q_des-q0| (default %.1f).\n"
      "                        dq: |q-q0| travel cap then 0xFD (not reverse).\n"
      "                        dq stroke cap is PMAX (several rad is legal).\n"
      "  --period-ms MS        default 10\n"
      "  --drain-ms MS         RX wait cap (default period-2)\n"
      "  --kp --kd             MIT gains (default %.0f / %.1f, kp<=%.0f kd<=%.0f).\n"
      "                        dq ignores --kp (sends 0). kd pack is 0-5.\n"
      "  --abort-tau NM        abort if |tau_fb| exceeds (default %.0f)\n"
      "  --abort-q-margin RAD  abort if |q-q0| > stroke+margin (default %.2f)\n"
      "  --left-port --right-port --yaml PATH\n"
      "  --handshake --verbose --self-test\n"
      "  --set-zero            WRITES firmware zero on BOTH selected knee IDs\n"
      "                        (MIT 0xFE). Not triangle/dq. Mutually exclusive\n"
      "                        with --stage. Same ports as the run. No 0xFC;\n"
      "                        then 0xFD disable hygiene on those two IDs.\n"
      "  Fault/abort/SIGINT: P5 disable-all 0xFD on map 3-14 (both buses).\n",
      static_cast<double>(biped_hw::kDmHomeEpsRad), static_cast<double>(kKdDefault),
      static_cast<double>(biped_hw::kDmHomeVel), static_cast<double>(biped_hw::kDmHomeVelMax),
      static_cast<double>(biped_hw::kDmHomeTimeoutS), static_cast<double>(biped_hw::kDmHomeVel),
      static_cast<double>(biped_hw::kDmHomeTimeoutSlackS),
      static_cast<double>(biped_hw::kDmHomeTimeoutMaxS),
      static_cast<unsigned>(kLeftId), static_cast<unsigned>(kRightId),
      static_cast<unsigned>(kLeftId), static_cast<unsigned>(kRightId),
      kDurationDefault, static_cast<double>(kVelDefault), static_cast<double>(kStrokeDefault),
      static_cast<double>(kKpDefault), static_cast<double>(kKdDefault),
      static_cast<double>(kKpCap), static_cast<double>(kKdCap),
      static_cast<double>(kAbortTauDefault), static_cast<double>(kAbortQMarginDefault));
}

struct Options
{
  std::string left_port = biped_hw::kDefaultPortLeft;
  std::string right_port = biped_hw::kDefaultPortRight;
  std::string yaml_path;
  double duration_s = kDurationDefault;
  int period_ms = 10;
  int drain_ms = 8;
  float kp = kKpDefault;
  float kd = kKdDefault;
  float vel = kVelDefault;
  float stroke = kStrokeDefault;
  float abort_tau = kAbortTauDefault;
  float abort_q_margin = kAbortQMarginDefault;
  uint8_t left_id = kLeftId;
  uint8_t right_id = kRightId;
  bool handshake = false;
  bool verbose = false;
  bool self_test = false;
  bool stage_dq = false;  // default triangle
  bool stage_set = false;
  bool set_zero = false;
};

bool parse_args(int argc, char **argv, Options &opt, std::string &err)
{
  bool left_port_set = false;
  bool right_port_set = false;
  bool drain_set = false;

  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      print_usage();
      err = "help";
      return false;
    }
    auto need = [&](const char *flag) -> const char * {
      return biped_hw::need_arg(i, argc, argv, flag, err);
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
    }
    else if (std::strcmp(argv[i], "--abort-q-margin") == 0)
    {
      const char *v = need("--abort-q-margin");
      if (v == nullptr)
      {
        return false;
      }
      opt.abort_q_margin = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--left-port") == 0 || std::strcmp(argv[i], "--port") == 0)
    {
      const char *v = need(argv[i]);
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
    else if (std::strcmp(argv[i], "--ids") == 0)
    {
      const char *v = need("--ids");
      if (v == nullptr)
      {
        return false;
      }
      int a = 0;
      int b = 0;
      char sep = 0;
      char extra = 0;
      const int n = std::sscanf(v, "%d %c %d %c", &a, &sep, &b, &extra);
      if (n != 3 || sep != ',' || a < 1 || a > 14 || b < 1 || b > 14)
      {
        err = "--ids needs two CAN IDs 1..14 (e.g. 6,12)";
        return false;
      }
      opt.left_id = static_cast<uint8_t>(a);
      opt.right_id = static_cast<uint8_t>(b);
    }
    else if (std::strcmp(argv[i], "--left-id") == 0)
    {
      const char *v = need("--left-id");
      if (v == nullptr)
      {
        return false;
      }
      const int n = std::atoi(v);
      if (n < 1 || n > 14)
      {
        err = "--left-id must be 1..14";
        return false;
      }
      opt.left_id = static_cast<uint8_t>(n);
    }
    else if (std::strcmp(argv[i], "--right-id") == 0)
    {
      const char *v = need("--right-id");
      if (v == nullptr)
      {
        return false;
      }
      const int n = std::atoi(v);
      if (n < 1 || n > 14)
      {
        err = "--right-id must be 1..14";
        return false;
      }
      opt.right_id = static_cast<uint8_t>(n);
    }
    else if (std::strcmp(argv[i], "--stage") == 0)
    {
      const char *v = need("--stage");
      if (v == nullptr)
      {
        return false;
      }
      if (std::strcmp(v, "dq") == 0)
      {
        opt.stage_dq = true;
      }
      else if (std::strcmp(v, "triangle") == 0)
      {
        opt.stage_dq = false;
      }
      else
      {
        err = "--stage must be triangle|dq";
        return false;
      }
      opt.stage_set = true;
    }
    else if (std::strcmp(argv[i], "--set-zero") == 0)
    {
      opt.set_zero = true;
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

  if (!opt.yaml_path.empty() && (!left_port_set || !right_port_set))
  {
    std::string yl, yr;
    if (!biped_hw::yaml_bus_ports(opt.yaml_path, &yl, &yr))
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
  if (opt.vel < 0.f)
  {
    opt.vel = -opt.vel;
  }
  if (opt.stroke < 0.f)
  {
    opt.stroke = -opt.stroke;
  }
  if (opt.stroke < 0.02f)
  {
    opt.stroke = 0.02f;
  }
  if (opt.kp < 0.f)
  {
    opt.kp = 0.f;
  }
  if (opt.kp > kKpCap)
  {
    std::printf("WARN: clamping kp %.1f -> %.1f\n", static_cast<double>(opt.kp),
                static_cast<double>(kKpCap));
    opt.kp = kKpCap;
  }
  if (opt.kd < 0.f)
  {
    opt.kd = 0.f;
  }
  if (opt.kd > kKdCap)
  {
    std::printf("WARN: clamping kd %.2f -> %.2f\n", static_cast<double>(opt.kd),
                static_cast<double>(kKdCap));
    opt.kd = kKdCap;
  }
  if (opt.abort_tau < 1.f)
  {
    opt.abort_tau = 1.f;
  }
  if (opt.abort_q_margin < 0.f)
  {
    opt.abort_q_margin = -opt.abort_q_margin;
  }
  return true;
}

biped_hw::CycleMotorCmd make_cmd(const Knee &k, bool send, float kp, float kd, float q, float dq)
{
  biped_hw::CycleMotorCmd c{};
  c.can_id = k.can_id;
  c.model = k.model;
  c.lim = k.lim;
  c.send = send;
  c.expect_rx = send;
  c.dlc = 8;
  if (send)
  {
    biped_hw::pack_mit(c.data, kp, kd, q, dq, 0.f, k.lim);
  }
  return c;
}

bool enable_knee(Knee &k, biped_hw::UsbCanAdapter &ad, biped_hw::FrameDemux &dm)
{
  biped_hw::DmMitEnableStatus st;
  const biped_hw::DmMitEnableErr en =
      biped_hw::dm_enable_mit(ad, dm, k.can_id, true, st);
  std::printf("id=%u RID 21/22/23 PMAX=%s%.4f VMAX=%s%.4f TMAX=%s%.4f\n",
              static_cast<unsigned>(k.can_id), st.pmax_ok ? "" : "FAIL ",
              static_cast<double>(st.pmax), st.vmax_ok ? "" : "FAIL ",
              static_cast<double>(st.vmax), st.tmax_ok ? "" : "FAIL ",
              static_cast<double>(st.tmax));
  if (en == biped_hw::DmMitEnableErr::Rid)
  {
    std::fprintf(stderr,
                 "REFUSE enable id=%u: Damiao MIT limits UNVERIFIED (RID 21/22/23 failed)\n",
                 static_cast<unsigned>(k.can_id));
    return false;
  }
  k.lim = st.lim;
  std::printf("id=%u CTRL_MODE before=%s%u write=%s after=%s%u (%s)\n",
              static_cast<unsigned>(k.can_id), st.ctrl_before_ok ? "" : "FAIL ",
              static_cast<unsigned>(st.ctrl_before), st.wrote_ack ? "ack" : "NO-ACK",
              st.ctrl_after_ok ? "" : "FAIL ", static_cast<unsigned>(st.ctrl_after),
              biped_hw::dm_ctrl_mode_name(st.ctrl_after));
  if (en == biped_hw::DmMitEnableErr::CtrlMode)
  {
    std::fprintf(stderr, "REFUSE enable id=%u: need CTRL_MODE=MIT(1)\n",
                 static_cast<unsigned>(k.can_id));
    return false;
  }
  if (en != biped_hw::DmMitEnableErr::Ok)
  {
    std::fprintf(stderr, "enable send failed id=%u\n", static_cast<unsigned>(k.can_id));
    return false;
  }
  std::printf("enable id=%u sent (0xFC)\n", static_cast<unsigned>(k.can_id));
  k.enabled = true;
  return true;
}

void print_summary(const Knee knees[2], std::uint64_t overruns, std::uint64_t cycles)
{
  std::printf("\n=== P3 dual-knee spin per-id summary ===\n");
  std::printf(" id  model      enabled  sent    rx      drop    rtt_ms mean/min/max   "
              "q0      last_q  peak_|q-q0|  |tau|peak\n");
  for (int i = 0; i < 2; ++i)
  {
    const Knee &s = knees[i];
    const double mean = s.rx == 0 ? 0.0 : s.rtt_sum / static_cast<double>(s.rx);
    char qbuf[32];
    char q0buf[32];
    if (s.have_q)
    {
      std::snprintf(qbuf, sizeof(qbuf), "% .4f", static_cast<double>(s.last_q));
    }
    else
    {
      std::snprintf(qbuf, sizeof(qbuf), "     -");
    }
    if (s.have_q || s.enabled)
    {
      std::snprintf(q0buf, sizeof(q0buf), "% .4f", static_cast<double>(s.q0));
    }
    else
    {
      std::snprintf(q0buf, sizeof(q0buf), "     -");
    }
    std::printf(" %2u  %-9s  %s      %6llu %6llu %7llu  %6.3f / %6.3f / %6.3f  %s  %s  %.4f  %.3f\n",
                static_cast<unsigned>(s.can_id), biped_hw::to_string(s.model), s.enabled ? "Y" : "n",
                static_cast<unsigned long long>(s.sent),
                static_cast<unsigned long long>(s.rx),
                static_cast<unsigned long long>(s.drop), mean, s.rx == 0 ? 0.0 : s.rtt_min,
                s.rx == 0 ? 0.0 : s.rtt_max, q0buf, qbuf, static_cast<double>(s.q_peak_abs),
                static_cast<double>(s.tau_peak_abs));
  }
  std::printf("cycles=%llu  overrun=%llu\n", static_cast<unsigned long long>(cycles),
              static_cast<unsigned long long>(overruns));
}

int run(Options opt)
{
  const auto *map_l = biped_hw::find_by_can_id(opt.left_id);
  const auto *map_r = biped_hw::find_by_can_id(opt.right_id);
  // Dual-bus bring-up: accept mapped Damiao MIT Master 2. Do not require
  // knee joint names (ID 12 is right knee DM10422; ID 6 is left knee).
  if (map_l == nullptr || !biped_hw::mapped_damiao_mit_master2(opt.left_id))
  {
    std::fprintf(stderr,
                 "motor map mismatch for CAN ID %u (need mapped Damiao MIT, Master 2)\n",
                 static_cast<unsigned>(opt.left_id));
    return 1;
  }
  if (map_r == nullptr || !biped_hw::mapped_damiao_mit_master2(opt.right_id))
  {
    std::fprintf(stderr,
                 "motor map mismatch for CAN ID %u (need mapped Damiao MIT, Master 2)\n",
                 static_cast<unsigned>(opt.right_id));
    return 1;
  }
  if (opt.left_id == opt.right_id)
  {
    std::printf("NOTE: --left-id and --right-id are both %u (OK: separate ACM adapters)\n",
                static_cast<unsigned>(opt.left_id));
  }

  if (opt.set_zero)
  {
    std::printf(
        "\n"
        "**********************************************************************\n"
        "* WARN: --set-zero WRITES firmware zero at current shaft position.  *\n"
        "* IDs %u and %u only (not 12-motor map). MIT 0xFE. No 0xFC.         *\n"
        "* Irreversible-ish. Shafts must be at the intended origin.          *\n"
        "**********************************************************************\n"
        "left %s id=%u  right %s id=%u\n",
        static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id),
        opt.left_port.c_str(), static_cast<unsigned>(opt.left_id), opt.right_port.c_str(),
        static_cast<unsigned>(opt.right_id));

    biped_hw::UsbCanAdapter left_ad(opt.left_port);
    biped_hw::UsbCanAdapter right_ad(opt.right_port);
    if (!left_ad.open())
    {
      std::fprintf(stderr, "open left %s failed: %s\n", opt.left_port.c_str(),
                   left_ad.last_error().c_str());
      return 1;
    }
    std::printf("opened left %s; USB2CAN CAN baud %u bps\n", opt.left_port.c_str(),
                left_ad.can_bitrate());
    if (!right_ad.open())
    {
      std::fprintf(stderr,
                   "open right %s failed: %s\n"
                   "p3_dual_knee_spin --set-zero needs BOTH buses.\n",
                   opt.right_port.c_str(), right_ad.last_error().c_str());
      left_ad.close();
      return 1;
    }
    std::printf("opened right %s; USB2CAN CAN baud %u bps\n", opt.right_port.c_str(),
                right_ad.can_bitrate());
    if (opt.handshake)
    {
      left_ad.handshake();
      usleep(20000);
      left_ad.drain();
      right_ad.handshake();
      usleep(20000);
      right_ad.drain();
      std::printf("handshake sent (CMD 0x02)\n");
    }
    const bool ok_l = biped_hw::dm_send_set_zero(left_ad, opt.left_id);
    std::printf("set-zero id=%u %s (0xFE) data=FF FF FF FF FF FF FF FE\n",
                static_cast<unsigned>(opt.left_id), ok_l ? "sent" : "TX FAIL");
    const bool ok_r = biped_hw::dm_send_set_zero(right_ad, opt.right_id);
    std::printf("set-zero id=%u %s (0xFE) data=FF FF FF FF FF FF FF FE\n",
                static_cast<unsigned>(opt.right_id), ok_r ? "sent" : "TX FAIL");
    if (ok_l)
    {
      biped_hw::dm_send_disable(left_ad, opt.left_id);
      std::printf("disable id=%u sent (0xFD)\n", static_cast<unsigned>(opt.left_id));
    }
    if (ok_r)
    {
      biped_hw::dm_send_disable(right_ad, opt.right_id);
      std::printf("disable id=%u sent (0xFD)\n", static_cast<unsigned>(opt.right_id));
    }
    left_ad.close();
    right_ad.close();
    return (ok_l && ok_r) ? 0 : 1;
  }

  const float abort_q = opt.stroke + opt.abort_q_margin;
  if (opt.stage_dq)
  {
    std::printf(
        "\n"
        "**********************************************************************\n"
        "* 双总线 --stage dq：左 CAN %u + 右 CAN %u  MIT kp=0 恒速             *\n"
        "**********************************************************************\n"
        "dq_des=+%.3f rad/s (cap VMAX, not 5%%) stroke=±%.3f duration=%.1fs\n"
        "abort-tau=%.0f abort |q-q0|>%.3f  kd=%.2f (MIT pack 0-5) kp=0 q_des=0\n"
        "left %s id=%u  right %s id=%u  period=%dms\n",
        static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id),
        static_cast<double>(opt.vel), static_cast<double>(opt.stroke), opt.duration_s,
        static_cast<double>(opt.abort_tau), static_cast<double>(abort_q),
        static_cast<double>(opt.kd), opt.left_port.c_str(),
        static_cast<unsigned>(opt.left_id), opt.right_port.c_str(),
        static_cast<unsigned>(opt.right_id), opt.period_ms);
  }
  else
  {
    std::printf(
        "\n"
        "**********************************************************************\n"
        "* 双总线慢摆：左 CAN %u + 右 CAN %u  Damiao MIT 位置斜坡              *\n"
        "**********************************************************************\n"
        "vel=%.3f stroke=±%.3f duration=%.0fs abort-tau=%.0f abort |q-q0|>%.3f\n"
        "left %s id=%u  right %s id=%u  period=%dms kp=%.1f kd=%.2f vel=%.3f stroke=±%.3f\n",
        static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id),
        static_cast<double>(opt.vel), static_cast<double>(opt.stroke), opt.duration_s,
        static_cast<double>(opt.abort_tau), static_cast<double>(abort_q), opt.left_port.c_str(),
        static_cast<unsigned>(opt.left_id), opt.right_port.c_str(),
        static_cast<unsigned>(opt.right_id), opt.period_ms, static_cast<double>(opt.kp),
        static_cast<double>(opt.kd), static_cast<double>(opt.vel),
        static_cast<double>(opt.stroke));
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::atexit(atexit_disable);

  biped_hw::UsbCanAdapter left_ad(opt.left_port);
  biped_hw::UsbCanAdapter right_ad(opt.right_port);
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

  if (!right_ad.open())
  {
    std::fprintf(stderr,
                 "open right %s failed: %s\n"
                 "p3_dual_knee_spin needs BOTH buses (left ttyACM0 + right ttyACM1).\n"
                 "Use p2_dm_mit_ladder for left-only.\n",
                 opt.right_port.c_str(), right_ad.last_error().c_str());
    g_left = nullptr;
    return 1;
  }
  std::printf("opened right %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n",
              opt.right_port.c_str(), right_ad.can_bitrate(),
              static_cast<unsigned>(right_ad.can_baud_index()));
  g_right = &right_ad;
  biped_hw::safety_bind_buses(&left_ad, &right_ad);

  if (opt.handshake)
  {
    left_ad.handshake();
    usleep(20000);
    left_ad.drain();
    right_ad.handshake();
    usleep(20000);
    right_ad.drain();
    std::printf("handshake sent (CMD 0x02)\n");
  }

  Knee knees[2]{};
  knees[0].can_id = opt.left_id;
  knees[0].model = map_l->model;
  knees[0].lim = biped_hw::default_limit(map_l->model);
  knees[1].can_id = opt.right_id;
  knees[1].model = map_r->model;
  knees[1].lim = biped_hw::default_limit(map_r->model);

  biped_hw::FrameDemux left_demux;
  biped_hw::FrameDemux right_demux;
  biped_hw::BusCycle left_cycle(left_ad, biped_hw::BusId::Left, left_demux);
  biped_hw::BusCycle right_cycle(right_ad, biped_hw::BusId::Right, right_demux);
  biped_hw::DualBusCycle dual(&left_cycle, &right_cycle);

  const auto period = std::chrono::milliseconds(opt.period_ms);
  const auto drain = std::chrono::milliseconds(opt.drain_ms);
  const auto probe_drain = std::chrono::milliseconds(std::max(opt.drain_ms, 30));
  const float dt = static_cast<float>(opt.period_ms) * 0.001f;

  auto tick_pair = [&](float kp, float kd, bool inquiry,
                       std::chrono::milliseconds rx_wait) -> biped_hw::DualBusCycleResult {
    std::vector<biped_hw::CycleMotorCmd> lc;
    std::vector<biped_hw::CycleMotorCmd> rc;
    const float q0c = inquiry ? 0.f : knees[0].q_des;
    const float q1c = inquiry ? 0.f : knees[1].q_des;
    const float dq0 = inquiry ? 0.f : knees[0].vel_sign * opt.vel;
    const float dq1 = inquiry ? 0.f : knees[1].vel_sign * opt.vel;
    lc.push_back(make_cmd(knees[0], true, kp, kd, q0c, dq0));
    rc.push_back(make_cmd(knees[1], true, kp, kd, q1c, dq1));
    return dual.tick(&lc, &rc, rx_wait, period, nullptr, nullptr);
  };

  auto apply_fb = [&](const biped_hw::DualBusCycleResult &tick, bool set_q0) {
    auto mark = [&](const biped_hw::BusCycleResult &br) {
      for (const auto &m : br.motors)
      {
        Knee *k = find_knee(knees, 2, m.can_id);
        if (k == nullptr)
        {
          continue;
        }
        if (m.rx_ok)
        {
          k->present = true;
          if (m.have_fb)
          {
            k->last_q = m.fb.q;
            k->last_tau = m.fb.tau;
            k->last_dq = m.fb.dq;
            k->have_q = true;
            if (set_q0)
            {
              k->q0 = m.fb.q;
              k->q_des = m.fb.q;
            }
          }
        }
      }
    };
    mark(tick.left);
    mark(tick.right);
  };

  std::printf("probe dual %d ticks drain=%dms (kp=0 inquiry, left id=%u + right id=%u)...\n",
              kProbeTicks, static_cast<int>(probe_drain.count()),
              static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id));
  for (int p = 0; p < kProbeTicks && g_run.load(); ++p)
  {
    apply_fb(tick_pair(0.f, 0.f, true, probe_drain), true);
  }
  std::printf("probe: id%u=%s", static_cast<unsigned>(knees[0].can_id),
              knees[0].present ? "ok" : "MISSING");
  if (knees[0].have_q)
  {
    std::printf("(q=%.4f)", static_cast<double>(knees[0].last_q));
  }
  std::printf("  id%u=%s", static_cast<unsigned>(knees[1].can_id),
              knees[1].present ? "ok" : "MISSING");
  if (knees[1].have_q)
  {
    std::printf("(q=%.4f)", static_cast<double>(knees[1].last_q));
  }
  std::printf("\n");
  if (!knees[0].present || !knees[1].present)
  {
    std::fprintf(stderr,
                 "REFUSE: both motors must reply on probe (left CAN %u + right CAN %u).\n"
                 "未使能任何电机。\n",
                 static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id));
    g_left = nullptr;
    g_right = nullptr;
    return 1;
  }

  if (opt.stage_dq)
  {
    std::printf(
        "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        "WARN: enabling 2 motors: %u %u  (left ACM0 + right ACM1, Damiao MIT dq)\n"
        "kp=0 kd=%.2f dq_des=+%.3f rad/s q_des=0 for %.1f s; stop at |q-q0|>=%.2f then 0xFD.\n"
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n",
        static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id),
        static_cast<double>(opt.kd), static_cast<double>(opt.vel), opt.duration_s,
        static_cast<double>(opt.stroke));
  }
  else
  {
    std::printf(
        "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
        "WARN: enabling 2 motors: %u %u  (left ACM0 + right ACM1, Damiao MIT)\n"
        "Slow oscillation ±%.2f rad at %.2f rad/s for %.0f s. Joints must be free.\n"
        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n",
        static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id),
        static_cast<double>(opt.stroke), static_cast<double>(opt.vel), opt.duration_s);
  }

  if (!enable_knee(knees[0], left_ad, left_demux) ||
      !enable_knee(knees[1], right_ad, right_demux))
  {
    biped_hw::safety_set_reason(biped_hw::SafetyReason::RidFail);
    disable_all_now(true);
    g_left = nullptr;
    g_right = nullptr;
    biped_hw::safety_bind_buses(nullptr, nullptr);
    return 1;
  }
  usleep(kEnableSettleUs);
  left_ad.drain();
  right_ad.drain();
  biped_hw::safety_arm();

  const float home_vel =
      std::min(biped_hw::kDmHomeVelMax, std::max(opt.vel, biped_hw::kDmHomeVel));
  auto home_one = [&](Knee &k, biped_hw::UsbCanAdapter &ad,
                      biped_hw::FrameDemux &dm) -> bool {
    biped_hw::DmHomeStatus hs{};
    const auto hr = biped_hw::dm_home_to_zero(ad, dm, k.can_id, k.lim, opt.abort_tau,
                                              biped_hw::kDmHomeTimeoutS,
                                              biped_hw::kDmHomeEpsRad, &hs, home_vel, opt.kd);
    std::printf("home id=%u q_before=%.4f %s q_after=%.4f eps=%.3f tau_peak=%.3f\n",
                static_cast<unsigned>(k.can_id), static_cast<double>(hs.q_before),
                biped_hw::dm_home_result_name(hr), static_cast<double>(hs.q_after),
                static_cast<double>(biped_hw::kDmHomeEpsRad),
                static_cast<double>(hs.tau_peak));
    if (!biped_hw::dm_home_ok(hr))
    {
      std::fprintf(stderr, "home id=%u FAIL (%s); disable-all 0xFD, no profile\n",
                   static_cast<unsigned>(k.can_id), biped_hw::dm_home_result_name(hr));
      return false;
    }
    k.q0 = hs.q_after;
    k.q_des = hs.q_after;
    k.last_q = hs.q_after;
    k.have_q = true;
    return true;
  };
  if (!home_one(knees[0], left_ad, left_demux) ||
      !home_one(knees[1], right_ad, right_demux))
  {
    disable_all_now(true);
    g_left = nullptr;
    g_right = nullptr;
    return 1;
  }
  std::printf("q0 id%u=%.4f  id%u=%.4f  (after home-to-zero)\n",
              static_cast<unsigned>(knees[0].can_id), static_cast<double>(knees[0].q0),
              static_cast<unsigned>(knees[1].can_id), static_cast<double>(knees[1].q0));

  if (opt.stage_dq)
  {
    const float vmax = std::min(knees[0].lim.dq_max, knees[1].lim.dq_max);
    if (opt.vel > vmax)
    {
      std::printf("WARN: clamping vel %.2f -> %.2f (VMAX)\n", static_cast<double>(opt.vel),
                  static_cast<double>(vmax));
      opt.vel = vmax;
    }
    const float pmax = std::min(knees[0].lim.q_max, knees[1].lim.q_max);
    if (opt.stroke > pmax)
    {
      std::printf("WARN: clamping stroke %.3f -> %.3f (PMAX)\n",
                  static_cast<double>(opt.stroke), static_cast<double>(pmax));
      opt.stroke = pmax;
    }
    std::printf("MIT dq: kp=0 kd=%.3f  dq_des=+%.3f constant  q_des=0  "
                "stroke=±%.3f then 0xFD  duration=%.1fs  stall~kd*|dq_des|=%.2f Nm\n",
                static_cast<double>(opt.kd), static_cast<double>(opt.vel),
                static_cast<double>(opt.stroke), opt.duration_s,
                static_cast<double>(opt.kd * opt.vel));
  }

  knees[0].vel_sign = 1.f;
  knees[1].vel_sign = 1.f;

  biped_hw::CycleLogger left_log;
  biped_hw::CycleLogger right_log;
  biped_hw::PeriodJitterStats jitter{static_cast<double>(opt.period_ms) * 1000.0};
  std::uint64_t cycles = 0;
  std::uint64_t overruns = 0;
  bool aborted = false;
  bool stroke_stop = false;
  const auto t0 = biped_hw::BusCycle::Clock::now();
  const auto duration = std::chrono::duration<double>(opt.duration_s);
  auto next = t0;
  auto last_print = t0;

  if (opt.stage_dq)
  {
    std::printf("dq loop start (kp=0, dq_des=+%.3f, q_des=0; stop duration or |q-q0|>=stroke). "
                "SIGINT disables %u and %u.\n",
                static_cast<double>(opt.vel), static_cast<unsigned>(opt.left_id),
                static_cast<unsigned>(opt.right_id));
  }
  else
  {
    std::printf("ramp loop start (q_des += vel*dt, reverse at ±stroke). SIGINT disables %u and %u.\n",
                static_cast<unsigned>(opt.left_id), static_cast<unsigned>(opt.right_id));
  }

  while (g_run.load() && !aborted && !stroke_stop &&
         (biped_hw::BusCycle::Clock::now() - t0) < duration)
  {
    next += period;
    if (!opt.stage_dq)
    {
      for (int i = 0; i < 2; ++i)
      {
        Knee &k = knees[i];
        k.q_des += k.vel_sign * opt.vel * dt;
        const float q_hi = k.q0 + opt.stroke;
        const float q_lo = k.q0 - opt.stroke;
        if (k.q_des > q_hi)
        {
          k.q_des = q_hi;
          k.vel_sign = -1.f;
        }
        else if (k.q_des < q_lo)
        {
          k.q_des = q_lo;
          k.vel_sign = 1.f;
        }
      }
    }

    const float kp_cmd = opt.stage_dq ? 0.f : opt.kp;
    const float q0c = opt.stage_dq ? 0.f : knees[0].q_des;
    const float q1c = opt.stage_dq ? 0.f : knees[1].q_des;
    const float dq0 = opt.stage_dq ? opt.vel : knees[0].vel_sign * opt.vel;
    const float dq1 = opt.stage_dq ? opt.vel : knees[1].vel_sign * opt.vel;
    std::vector<biped_hw::CycleMotorCmd> lc;
    std::vector<biped_hw::CycleMotorCmd> rc;
    lc.push_back(make_cmd(knees[0], true, kp_cmd, opt.kd, q0c, dq0));
    rc.push_back(make_cmd(knees[1], true, kp_cmd, opt.kd, q1c, dq1));
    if (opt.stage_dq && cycles == 0)
    {
      auto dump_tx = [](uint8_t id, const uint8_t buf[8], const biped_hw::LimitParam &lim) {
        float kp_w = 0.f, kd_w = 0.f, q_w = 0.f, dq_w = 0.f, tau_w = 0.f;
        biped_hw::unpack_mit_cmd(buf, lim, kp_w, kd_w, q_w, dq_w, tau_w);
        std::printf("TX id=%u wire kp=%.2f kd=%.3f q=%.5f dq=%.4f tau=%.4f "
                    "data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                    static_cast<unsigned>(id), static_cast<double>(kp_w),
                    static_cast<double>(kd_w), static_cast<double>(q_w),
                    static_cast<double>(dq_w), static_cast<double>(tau_w), buf[0], buf[1],
                    buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
      };
      dump_tx(knees[0].can_id, lc[0].data, knees[0].lim);
      dump_tx(knees[1].can_id, rc[0].data, knees[1].lim);
    }

    const auto t_send = biped_hw::BusCycle::Clock::now();
    const auto tick = dual.tick(&lc, &rc, drain, period, &left_log, &right_log);
    accumulate(knees, 2, tick.left);
    accumulate(knees, 2, tick.right);
    ++cycles;
    if (tick.overrun)
    {
      ++overruns;
    }
    const double dt_us =
        std::chrono::duration<double, std::micro>(biped_hw::BusCycle::Clock::now() - t_send)
            .count();
    jitter.push(dt_us);

    for (int i = 0; i < 2; ++i)
    {
      const Knee &k = knees[i];
      if (!k.have_q)
      {
        continue;
      }
      if (std::fabs(k.last_tau) > opt.abort_tau)
      {
        std::fprintf(stderr, "ABORT id=%u: |tau_fb|=%.3f > %.3f Nm\n",
                     static_cast<unsigned>(k.can_id), static_cast<double>(std::fabs(k.last_tau)),
                     static_cast<double>(opt.abort_tau));
        biped_hw::safety_set_reason(biped_hw::SafetyReason::HardLimitTau);
        aborted = true;
      }
      if (opt.stage_dq && std::fabs(k.last_q - k.q0) >= opt.stroke)
      {
        std::printf("STOP id=%u: |q-q0|=%.4f >= stroke %.4f; disable 0xFD both\n",
                    static_cast<unsigned>(k.can_id),
                    static_cast<double>(std::fabs(k.last_q - k.q0)),
                    static_cast<double>(opt.stroke));
        stroke_stop = true;
      }
      if (std::fabs(k.last_q - k.q0) > abort_q)
      {
        std::fprintf(stderr, "ABORT id=%u: |q-q0|=%.4f > stroke+margin %.4f\n",
                     static_cast<unsigned>(k.can_id),
                     static_cast<double>(std::fabs(k.last_q - k.q0)),
                     static_cast<double>(abort_q));
        biped_hw::safety_set_reason(biped_hw::SafetyReason::HardLimitQ);
        aborted = true;
      }
    }

    const auto now = biped_hw::BusCycle::Clock::now();
    const bool print_now =
        opt.verbose || cycles <= 3 || (now - last_print) >= std::chrono::milliseconds(500);
    if (print_now)
    {
      std::printf("cycle %llu elapsed=%.2fms overrun=%d  ",
                  static_cast<unsigned long long>(cycles), tick.elapsed_ms,
                  tick.overrun ? 1 : 0);
      auto dump = [&](const biped_hw::BusCycleResult &br) {
        for (const auto &m : br.motors)
        {
          Knee *k = find_knee(knees, 2, m.can_id);
          if (m.rx_ok)
          {
            const float qdes = k == nullptr ? 0.f : (opt.stage_dq ? 0.f : k->q_des);
            const float dqdes =
                k == nullptr ? 0.f : (opt.stage_dq ? opt.vel : k->vel_sign * opt.vel);
            std::printf("id%u=ok(rtt=%.2f q=%.4f dq=%.4f qdes=%.4f dqdes=%.4f tau=%.3f) ",
                        static_cast<unsigned>(m.can_id), m.rtt_ms,
                        static_cast<double>(m.have_fb ? m.fb.q : 0.f),
                        static_cast<double>(m.have_fb ? m.fb.dq : 0.f),
                        static_cast<double>(qdes), static_cast<double>(dqdes),
                        static_cast<double>(m.have_fb ? m.fb.tau : 0.f));
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

    const auto remain = next - biped_hw::BusCycle::Clock::now();
    if (remain > std::chrono::milliseconds(1))
    {
      usleep(static_cast<useconds_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(remain).count()));
    }
  }

  print_summary(knees, overruns, cycles);
  std::printf("jitter_us mean=%.1f max_dev=%.1f  (period=%d ms)\n", jitter.mean_us(),
              jitter.max_deviation_us(), opt.period_ms);

  disable_all_now(true);
  g_left = nullptr;
  g_right = nullptr;
  biped_hw::safety_bind_buses(nullptr, nullptr);
  left_ad.close();
  right_ad.close();

  int rc = aborted ? 1 : 0;
  for (int i = 0; i < 2; ++i)
  {
    if (knees[i].rx == 0)
    {
      std::fprintf(stderr, "id=%u had zero RX during ramp\n",
                   static_cast<unsigned>(knees[i].can_id));
      rc = 1;
    }
  }
  return rc;
}

}  // namespace

int main(int argc, char **argv)
{
  Options opt;
  std::string err;
  if (!parse_args(argc, argv, opt, err))
  {
    if (err == "help")
    {
      return 0;
    }
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }
  if (!biped_hw::dm_codec_self_check() || !biped_hw::bus_cycle_self_check())
  {
    std::fprintf(stderr, "codec/bus_cycle self-check failed\n");
    return 1;
  }
  if (opt.self_test)
  {
    std::printf("p3_dual_knee_spin self-check OK (no hardware): dm_codec, bus_cycle\n");
    return 0;
  }
  return run(opt);
}
