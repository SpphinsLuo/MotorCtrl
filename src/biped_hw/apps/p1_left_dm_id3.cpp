// 达妙电机通信测试程序
// P1 第一关：单电机使能/失能与 RID 限幅读取。同源编成 p1_left_dm_id6（左膝 CAN 6）与 p1_left_dm_id3（髋偏航 CAN 3）。
#include "biped_hw/cli.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <unistd.h>

#ifndef P1_DEFAULT_CAN_ID
#define P1_DEFAULT_CAN_ID 6
#endif
#if P1_DEFAULT_CAN_ID == 6
#define P1_BIN_NAME "p1_left_dm_id6"
#else
#define P1_BIN_NAME "p1_left_dm_id3"
#endif

namespace
{

// Built as p1_left_dm_id6 (primary, left knee DM10422 CAN 6) and p1_left_dm_id3 (left hip yaw DM10010L CAN 3). --can-id remains overridable.
constexpr uint8_t kDefaultEscId = static_cast<uint8_t>(P1_DEFAULT_CAN_ID);
constexpr int kEnableSettleUs = 100000;
constexpr int kDisableGapUs = 10000;

std::atomic<bool> g_run{true};//程序运行标志位
std::atomic<uint8_t> g_esc_id{kDefaultEscId};
biped_hw::UsbCanAdapter *g_exit_adapter = nullptr;

//程序退出时的信号处理函数  
void on_signal(int)
{
  g_run.store(false);
}

//在程序退出时，自动发送失能命令给电机
void atexit_disable()
{
  if (g_exit_adapter == nullptr || !g_exit_adapter->is_open())
  {
    return;
  }
  biped_hw::dm_send_disable(*g_exit_adapter, g_esc_id.load());
}

std::string left_port_from_yaml(const std::string &path)
{
  std::string left;
  std::string right;
  if (!biped_hw::yaml_bus_ports(path, &left, &right))
  {
    return {};
  }
  return left;
}

bool needs_live_limits(biped_hw::MotorType type)
{
  return type == biped_hw::MotorType::DM10422;
}

bool parse_force_limits(const char *s, biped_hw::LimitParam &out)
{
  float p = 0.f;
  float v = 0.f;
  float t = 0.f;
  if (s == nullptr || std::sscanf(s, "%f ,%f ,%f", &p, &v, &t) != 3)
  {
    return false;
  }
  if (p <= 0.1f || v <= 0.1f || t <= 0.1f)
  {
    return false;
  }
  out.q_max = p;
  out.dq_max = v;
  out.tau_max = t;
  return true;
}

void usage()
{
  std::printf(
      "Usage: %s [--duration SEC] [--kp K] [--kd D] [--period-ms MS]\n"
      "                      [--can-id N] [--port PATH] [--yaml PATH] [--handshake]\n"
      "                      [--read-limits] [--force-limits PMAX,VMAX,TMAX]\n"
      "                      [--self-test] [--verbose]\n"
      "Left USB-CAN only. Default CAN ID %u from the compile-time map.\n"
      "Primary P1 gate: p1_left_dm_id6 = left knee DM10422 CAN 6, Master 2.\n"
      "Hip yaw stay-behind: p1_left_dm_id3 = left DM10010L CAN 3 (--can-id 3).\n"
      "Replies on Master CAN ID 2; demux by ESC_ID nibble (not RX canId).\n"
      "DM10422 table default is {12.5, 20, 400}; RID 21/22/23 overrides when read.\n"
      "Knee enable still requires RID 21/22/23, or pass --force-limits.\n"
      "Low-kp MIT hold, tau=0. Knee default kp=1. Motor may twitch. Disable on exit.\n"
      "Never opens the right adapter. Never commands Motorevo ID 8.\n",
      P1_BIN_NAME, static_cast<unsigned>(kDefaultEscId));
}

struct DisableGuard
{
  biped_hw::UsbCanAdapter *adapter = nullptr;
  uint8_t can_id = kDefaultEscId;
  bool armed = false;

  void arm(biped_hw::UsbCanAdapter &a, uint8_t id)
  {
    adapter = &a;
    can_id = id;
    armed = true;
  }

  void disable_now()
  {
    if (!armed || adapter == nullptr || !adapter->is_open())
    {
      return;
    }
    biped_hw::dm_send_disable(*adapter, can_id);
    usleep(kDisableGapUs);
    std::printf("disable id=%u sent (0xFD)\n", static_cast<unsigned>(can_id));
    armed = false;
  }

  ~DisableGuard() { disable_now(); }
};

}  // namespace

int main(int argc, char **argv)
{
  double duration_s = 8.0;//运行时间
  float kp = 0.f;
  float kd = 0.f;
  bool kp_set = false;
  bool kd_set = false;
  int period_ms = 10;//通信周期/100Hz
  uint8_t esc_id = kDefaultEscId;
  std::string port = biped_hw::kDefaultPortLeft;
  std::string yaml_path;
  bool handshake = false;
  bool read_limits = false;
  bool self_test = false;
  bool verbose = false;
  bool port_set = false;
  bool have_force_limits = false;
  biped_hw::LimitParam force_lim{};

  // ---------------------------------Parse command-line arguments---------------------------------
  for (int i = 1; i < argc; ++i)
  {
    auto need = [&](const char *flag) -> const char * {
      if (i + 1 >= argc)
      {
        std::fprintf(stderr, "%s needs a value\n", flag);
        return nullptr;
      }
      return argv[++i];
    };
    if (std::strcmp(argv[i], "--duration") == 0)
    {
      const char *v = need("--duration");
      if (v == nullptr)
      {
        return 2;
      }
      duration_s = std::atof(v);
    }
    else if (std::strcmp(argv[i], "--kp") == 0)
    {
      const char *v = need("--kp");
      if (v == nullptr)
      {
        return 2;
      }
      kp = std::strtof(v, nullptr);
      kp_set = true;
    }
    else if (std::strcmp(argv[i], "--kd") == 0)
    {
      const char *v = need("--kd");
      if (v == nullptr)
      {
        return 2;
      }
      kd = std::strtof(v, nullptr);
      kd_set = true;
    }
    else if (std::strcmp(argv[i], "--period-ms") == 0)
    {
      const char *v = need("--period-ms");
      if (v == nullptr)
      {
        return 2;
      }
      period_ms = std::atoi(v);
    }
    else if (std::strcmp(argv[i], "--can-id") == 0)
    {
      const char *v = need("--can-id");
      if (v == nullptr)
      {
        return 2;
      }
      const int id = std::atoi(v);
      if (id < 0 || id > 255)
      {
        std::fprintf(stderr, "--can-id out of range\n");
        return 2;
      }
      esc_id = static_cast<uint8_t>(id);
    }
    else if (std::strcmp(argv[i], "--port") == 0)
    {
      const char *v = need("--port");
      if (v == nullptr)
      {
        return 2;
      }
      port = v;
      port_set = true;
    }
    else if (std::strcmp(argv[i], "--yaml") == 0)
    {
      const char *v = need("--yaml");
      if (v == nullptr)
      {
        return 2;
      }
      yaml_path = v;
    }
    else if (std::strcmp(argv[i], "--handshake") == 0)
    {
      handshake = true;
    }
    else if (std::strcmp(argv[i], "--read-limits") == 0)
    {
      read_limits = true;
    }
    else if (std::strcmp(argv[i], "--force-limits") == 0)
    {
      const char *v = need("--force-limits");
      if (v == nullptr)
      {
        return 2;
      }
      if (!parse_force_limits(v, force_lim))
      {
        std::fprintf(stderr,
                     "--force-limits needs PMAX,VMAX,TMAX (all > 0.1)\n");
        return 2;
      }
      have_force_limits = true;
    }
    else if (std::strcmp(argv[i], "--self-test") == 0)
    {
      self_test = true;
    }
    else if (std::strcmp(argv[i], "--verbose") == 0)
    {
      verbose = true;
    }
    else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      usage();
      return 0;
    }
    else
    {
      std::fprintf(stderr, "unknown argument: %s (try --help)\n", argv[i]);
      return 2;
    }
  }
  // ----------------------------------------------------------------------------------------------

  // ---------------------------------系统状态检测及阈值保护-----------------------------------------
  if (!biped_hw::dm_codec_self_check())
  {
    std::fprintf(stderr, "dm_codec self-check failed\n");
    return 1;
  }
  if (self_test)
  {
    std::printf("dm_codec self-check OK (no hardware)\n");
    return 0;
  }

  const auto *entry = biped_hw::find_by_can_id(esc_id);
  if (entry == nullptr || entry->bus != biped_hw::BusId::Left ||
      entry->master_id != biped_hw::kMasterId || !biped_hw::is_damiao_mit(entry->model))
  {
    std::fprintf(stderr,
                 "motor map mismatch for CAN ID %u (need left-bus Damiao MIT, Master 2)\n",
                 static_cast<unsigned>(esc_id));
    return 1;
  }

#if P1_DEFAULT_CAN_ID == 3
  std::printf("Note: knee P1 first-gate is p1_left_dm_id6 (DM10422 CAN 6).\n");
#endif

  if (!yaml_path.empty() && !port_set)
  {
    const std::string yport = left_port_from_yaml(yaml_path);
    if (yport.empty())
    {
      std::fprintf(stderr, "yaml overlay missing buses.left.port: %s\n",
                   yaml_path.c_str());
      return 2;
    }
    port = yport;
  }

  if (duration_s < 0.5)
  {
    duration_s = 0.5;
  }
  if (period_ms < 2)
  {
    period_ms = 2;
  }

  const bool knee = needs_live_limits(entry->model);
  if (!kp_set)
  {
    kp = knee ? 1.f : 2.f;
  }
  if (!kd_set)
  {
    kd = knee ? 0.3f : 0.5f;
  }
  const float kp_cap = knee ? 2.f : 5.f;
  const float kd_cap = knee ? 0.5f : 1.f;
  if (kp < 0.f)
  {
    kp = 0.f;
  }
  if (kp > kp_cap)
  {
    std::printf("WARN: clamping kp %.1f -> %.1f (P1 hold is low-gain only)\n",
                static_cast<double>(kp), static_cast<double>(kp_cap));
    kp = kp_cap;
  }
  if (kd < 0.f)
  {
    kd = 0.f;
  }
  if (kd > kd_cap)
  {
    std::printf("WARN: clamping kd %.2f -> %.2f (P1 hold is low-gain only)\n",
                static_cast<double>(kd), static_cast<double>(kd_cap));
    kd = kd_cap;
  }

  std::printf(
      "WARN: %smotor may twitch. Low-kp MIT hold at current/zero position, tau=0.\n"
      "      left only %s  CAN ID %u (%s, %s)  Master %u\n"
      "      feedback on CAN ID %u; demux ESC_ID nibble %u\n"
      "      kp=%.2f kd=%.2f duration=%.1fs period=%dms\n"
      "      right adapter unused; other CAN IDs not commanded.\n",
      knee ? "knee DM10422 CAN 6 — " : "", port.c_str(),
      static_cast<unsigned>(esc_id), biped_hw::to_string(entry->joint),
      biped_hw::to_string(entry->model), static_cast<unsigned>(entry->master_id),
      static_cast<unsigned>(biped_hw::kMasterId), static_cast<unsigned>(esc_id),
      static_cast<double>(kp), static_cast<double>(kd), duration_s, period_ms);

  std::signal(SIGINT, on_signal);//ctrl+c退出
  std::signal(SIGTERM, on_signal);//系统或其他程序请求进程正常终止

  biped_hw::UsbCanAdapter adapter(port);
  if (!adapter.open())
  {
    std::fprintf(stderr, "open %s failed: %s\n", port.c_str(),
                 adapter.last_error().c_str());
    return 1;
  }
  std::printf("opened %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n",
              port.c_str(), adapter.can_bitrate(),
              static_cast<unsigned>(adapter.can_baud_index()));

  DisableGuard guard;
  guard.arm(adapter, esc_id);
  g_esc_id.store(esc_id);
  g_exit_adapter = &adapter;
  std::atexit(atexit_disable);

  if (handshake)
  {
    if (!adapter.handshake())
    {
      std::fprintf(stderr, "handshake send failed\n");
      return 1;
    }
    usleep(20000);
    adapter.drain();
    std::printf("handshake sent (CMD 0x02)\n");
  }
  // ----------------------------------------------------------------------------------------------

  biped_hw::LimitParam lim{};
  const char *lim_src = "unset";
  biped_hw::FrameDemux demux;

  float pmax = 0.f, vmax = 0.f, tmax = 0.f;
  bool pok = false, vok = false, tok = false;
  // ---------------------------------读取电机限幅参数---------------------------------------------
  if (knee || read_limits)
  {
    adapter.drain();
    pok = biped_hw::dm_read_rid(adapter, demux, esc_id,
                           static_cast<uint8_t>(biped_hw::DmRid::Pmax), pmax);
    vok = biped_hw::dm_read_rid(adapter, demux, esc_id,
                           static_cast<uint8_t>(biped_hw::DmRid::Vmax), vmax);
    tok = biped_hw::dm_read_rid(adapter, demux, esc_id,
                           static_cast<uint8_t>(biped_hw::DmRid::Tmax), tmax);
    std::printf("RID 21/22/23 PMAX=%s%.4f VMAX=%s%.4f TMAX=%s%.4f\n", pok ? "" : "FAIL ",
                static_cast<double>(pmax), vok ? "" : "FAIL ",
                static_cast<double>(vmax), tok ? "" : "FAIL ",
                static_cast<double>(tmax));
  }

  const bool rid_ok = pok && vok && tok && pmax > 0.1f && vmax > 0.1f && tmax > 0.1f;
  if (rid_ok)
  {
    lim.q_max = pmax;
    lim.dq_max = vmax;
    lim.tau_max = tmax;
    lim_src = "RID 21/22/23";
  }
  else if (knee && have_force_limits)
  {
    lim = force_lim;
    lim_src = "--force-limits";
    std::printf(
        "WARN: RID 21/22/23 incomplete; using --force-limits (not a guessed table).\n");
  }
  else if (knee)
  {
    std::fprintf(stderr,
                 "REFUSE enable: knee DM10422 CAN %u limits UNVERIFIED and RID 21/22/23 "
                 "read failed.\n"
                 "Do not pack MIT with DM10010 {12.5,20,200}; table default is {12.5,20,400}.\n"
                 "Retry on a live bus, or pass --force-limits PMAX,VMAX,TMAX from the motor.\n",
                 static_cast<unsigned>(esc_id));
    return 1;
  }
  else
  {
    lim = biped_hw::default_limit(entry->model);
    lim_src = "table default";
    if (pok && pmax > 0.1f)
    {
      lim.q_max = pmax;
    }
    if (vok && vmax > 0.1f)
    {
      lim.dq_max = vmax;
    }
    if (tok && tmax > 0.1f)
    {
      lim.tau_max = tmax;
    }
    if (pok || vok || tok)
    {
      lim_src = "table default + RID overlay";
    }
  }

  std::printf("MIT pack/unpack limits: PMAX=%.4f VMAX=%.4f TMAX=%.4f  source=%s\n",
              static_cast<double>(lim.q_max), static_cast<double>(lim.dq_max),
              static_cast<double>(lim.tau_max), lim_src);

  uint8_t cmd[8] = {};
  if (!biped_hw::dm_send_enable(adapter, esc_id))
  {
    std::fprintf(stderr, "enable send failed\n");
    return 1;
  }
  std::printf("enable id=%u sent (0xFC)\n", static_cast<unsigned>(esc_id));
  usleep(kEnableSettleUs);
  adapter.drain();
  // ----------------------------------------------------------------------------------------------

  biped_hw::CycleLogger log;
  float q_hold = 0.f;
  bool have_q = false;
  const auto t0 = biped_hw::CycleLogger::Clock::now();
  const auto duration = std::chrono::duration<double>(duration_s);
  const auto period = std::chrono::milliseconds(period_ms);
  auto next = t0;
  std::uint64_t printed = 0;
  auto last_print = t0;

  // ---------------------------------主循环---------------------------------------------
  while (g_run.load() && (biped_hw::CycleLogger::Clock::now() - t0) < duration)
  {
    next += period;
    const float q_cmd = have_q ? q_hold : 0.f;
    biped_hw::pack_mit(cmd, kp, kd, q_cmd, 0.f, 0.f, lim);
    const auto t_send = biped_hw::CycleLogger::Clock::now(); 
    {
      std::fprintf(stderr, "MIT send failed; disabling\n");
      break;
    }
    log.on_send(esc_id, esc_id, t_send);

    biped_hw::DmFeedback fb{};
    bool got = false;
    while (biped_hw::CycleLogger::Clock::now() < next)
    {
      biped_hw::CanRxFrame rx;
      if (adapter.pollCan(rx))
      {
        demux.push(rx);
        if (demux.take_mit(esc_id, lim, fb))
        {
          log.on_rx(esc_id, fb.rx_can_id, biped_hw::CycleLogger::Clock::now());
          got = true;
          break;
        }
      }
    }
    if (!got)
    {
      log.on_timeout(esc_id);
    }
    else
    {
      q_hold = fb.q;
      have_q = true;
    }

    const auto now = biped_hw::CycleLogger::Clock::now();
    const bool print_now =
        verbose || printed < 5 || log.last_drop() ||
        (now - last_print) >= std::chrono::milliseconds(500);
    if (print_now)
    {
      char extra[160];
      if (got && fb.has_temp)
      {
        std::snprintf(extra, sizeof(extra),
                      "q=%.4f dq=%.4f tau=%.4f err=%u tmos=%u tcoil=%u hold_q=%.4f",
                      static_cast<double>(fb.q), static_cast<double>(fb.dq),
                      static_cast<double>(fb.tau), static_cast<unsigned>(fb.err),
                      static_cast<unsigned>(fb.t_mos),
                      static_cast<unsigned>(fb.t_coil),
                      static_cast<double>(q_hold));
      }
      else if (got)
      {
        std::snprintf(extra, sizeof(extra),
                      "q=%.4f dq=%.4f tau=%.4f err=%u tmos=- tcoil=- hold_q=%.4f",
                      static_cast<double>(fb.q), static_cast<double>(fb.dq),
                      static_cast<double>(fb.tau), static_cast<unsigned>(fb.err),
                      static_cast<double>(q_hold));
      }
      else
      {
        std::snprintf(extra, sizeof(extra), "no ESC_ID %u reply (hold_q=%.4f)",
                      static_cast<unsigned>(esc_id), static_cast<double>(q_hold));
      }
      log.print_cycle(stdout, extra);
      last_print = now;
      ++printed;
    }

    const auto remain = next - biped_hw::CycleLogger::Clock::now();
    if (remain > std::chrono::milliseconds(1))
    {
      usleep(static_cast<useconds_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(remain).count()));
    }
  }

  log.print_summary(stdout);
  std::printf("other_esc_frames_ignored=%zu queue_overflow_drops=%zu have_hold_q=%d\n",
              demux.other_esc_seen(), demux.overflow_drops(), have_q ? 1 : 0);

  guard.disable_now();
  g_exit_adapter = nullptr;
  adapter.close();
  return (log.rx() > 0) ? 0 : 1;
}
