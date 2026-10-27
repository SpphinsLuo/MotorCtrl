// 泉智博电机通信测试程序
// P1：HA60 通信/使能。CAN ID = kCanId，改完重新编译；无 --can-id。
#include "biped_hw/cli.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace
{

// Edit kCanId and rebuild. Must be mapped Motorevo HA60, Master 2.
constexpr uint8_t kCanId = 8;
constexpr int kEnableSettleUs = 100000;
constexpr int kDisableGapUs = 10000;
constexpr float kDefaultPosKp = 8.f;
constexpr float kDefaultPosKd = 0.5f;
constexpr float kPosKpCap = 15.f;
constexpr float kPosKdCap = 2.f;

std::atomic<bool> g_run{true};
biped_hw::UsbCanAdapter *g_exit_adapter = nullptr;

void on_signal(int)
{
  g_run.store(false);
}

void atexit_disable()
{
  if (g_exit_adapter == nullptr || !g_exit_adapter->is_open())
  {
    return;
  }
  uint8_t buf[8] = {};
  biped_hw::pack_motorevo_disable(buf);
  g_exit_adapter->sendCan(kCanId, buf, 8);
  usleep(kDisableGapUs);
  g_exit_adapter->sendCan(kCanId, buf, 8);
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

struct SessionLog
{
  std::string last_path{"/home/ye/00_jfl_26.10/Biped_Hw/log/biped_hw_p1_evo_last.log"};
  std::string stamp_path;
  std::FILE *last_fp = nullptr;
  std::FILE *stamp_fp = nullptr;

  static void emit_one(std::FILE *fp, const char *buf, int n)
  {
    if (fp == nullptr || buf == nullptr || n <= 0)
    {
      return;
    }
    std::fwrite(buf, 1, static_cast<std::size_t>(n), fp);
    std::fflush(fp);
  }

  void emit(std::FILE *console, const char *fmt, va_list ap)
  {
    char stack[2048];
    va_list ap2;
    va_copy(ap2, ap);
    const int n = std::vsnprintf(stack, sizeof(stack), fmt, ap2);
    va_end(ap2);
    if (n < 0)
    {
      return;
    }
    const char *buf = stack;
    std::string heap;
    if (n >= static_cast<int>(sizeof(stack)))
    {
      heap.resize(static_cast<std::size_t>(n) + 1);
      va_copy(ap2, ap);
      std::vsnprintf(heap.data(), heap.size(), fmt, ap2);
      va_end(ap2);
      buf = heap.c_str();
    }
    emit_one(console, buf, n);
    emit_one(last_fp, buf, n);
    emit_one(stamp_fp, buf, n);
  }

  void out(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
  {
    va_list ap;
    va_start(ap, fmt);
    emit(stdout, fmt, ap);
    va_end(ap);
  }

  void err(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
  {
    va_list ap;
    va_start(ap, fmt);
    emit(stderr, fmt, ap);
    va_end(ap);
  }

  bool open()
  {
    std::error_code ec;
    std::filesystem::create_directories("/home/ye/00_jfl_26.10/Biped_Hw/log", ec);

    std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char ts[32] = {};
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);

    last_fp = std::fopen(last_path.c_str(), "w");
    stamp_path = std::string("/home/ye/00_jfl_26.10/Biped_Hw/log/p1_evo_communicate_") + ts + ".log";
    stamp_fp = std::fopen(stamp_path.c_str(), "w");
    return last_fp != nullptr || stamp_fp != nullptr;
  }

  void close()
  {
    if (last_fp != nullptr)
    {
      std::fflush(last_fp);
      std::fclose(last_fp);
      last_fp = nullptr;
    }
    if (stamp_fp != nullptr)
    {
      std::fflush(stamp_fp);
      std::fclose(stamp_fp);
      stamp_fp = nullptr;
    }
  }
};

SessionLog g_slog;

void print_cycle_tee(const biped_hw::CycleLogger &log, const char *extra)
{
  log.print_cycle(stdout, extra);
  std::fflush(stdout);
  if (g_slog.last_fp != nullptr)
  {
    log.print_cycle(g_slog.last_fp, extra);
    std::fflush(g_slog.last_fp);
  }
  if (g_slog.stamp_fp != nullptr)
  {
    log.print_cycle(g_slog.stamp_fp, extra);
    std::fflush(g_slog.stamp_fp);
  }
}

void usage()
{
  std::printf(
      "Usage: p1_evo_communicate [--duration SEC] [--period-ms MS] [--port PATH]\n"
      "                       [--kp POS_KP] [--kd POS_KD] [--yaml PATH]\n"
      "                       [--handshake] [--self-test] [--verbose]\n"
      "CAN ID is kCanId in this file (edit and rebuild). Must be mapped HA60,\n"
      "Master 2. No --can-id. Default port is left USB-CAN.\n"
      "Default servo mode (not mix). Enable 0xFC, disable 0xFD.\n"
      "Feedback may arrive on slave CAN or Master 0x02; both are accepted.\n"
      "Conservative servo hold at current/zero. Motor may twitch. Disable on exit.\n"
      "Never commands Damiao MIT motors.\n");
}

struct DisableGuard
{
  biped_hw::UsbCanAdapter *adapter = nullptr;
  bool armed = false;

  void arm(biped_hw::UsbCanAdapter &a)
  {
    adapter = &a;
    armed = true;
  }

  void disable_now()
  {
    if (!armed || adapter == nullptr || !adapter->is_open())
    {
      return;
    }
    uint8_t buf[8] = {};
    biped_hw::pack_motorevo_disable(buf);
    adapter->sendCan(kCanId, buf, 8);
    usleep(kDisableGapUs);
    adapter->sendCan(kCanId, buf, 8);
    usleep(kDisableGapUs);
    g_slog.out("disable id=%u sent (0xFD)\n", static_cast<unsigned>(kCanId));
    armed = false;
  }

  ~DisableGuard() { disable_now(); }
};

bool demux_self_check()
{
  biped_hw::FrameDemux demux;
  auto fill_mid = [](biped_hw::CanRxFrame &rx, uint32_t can_id) {
    rx = {};
    rx.can_id = can_id;
    rx.dlc = 8;
    rx.data[0] = 0x01;  // servo status
    rx.data[1] = 0x7F;
    rx.data[2] = 0xFF;
    rx.data[3] = 0x7F;
    rx.data[4] = 0xF7;
    rx.data[5] = 0xFF;
  };

  biped_hw::CanRxFrame slave{};
  fill_mid(slave, kCanId);
  demux.push(slave);
  biped_hw::MotorevoFeedback fb{};
  if (!demux.take_motorevo(kCanId, fb) || fb.rx_can_id != kCanId)
  {
    return false;
  }

  biped_hw::CanRxFrame master{};
  fill_mid(master, biped_hw::kMasterId);
  demux.push(master);
  if (!demux.take_motorevo(kCanId, fb) || fb.rx_can_id != biped_hw::kMasterId)
  {
    return false;
  }

  biped_hw::CanRxFrame dm{};
  fill_mid(dm, biped_hw::kMasterId);
  dm.data[0] = 0x16;  // Damiao-looking: ERR=1 ESC=6
  demux.push(dm);
  if (demux.take_motorevo(kCanId, fb))
  {
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char **argv)
{
  double duration_s = 8.0;
  float pos_kp = kDefaultPosKp;
  float pos_kd = kDefaultPosKd;
  int period_ms = 10;
  std::string port = biped_hw::kDefaultPortLeft;
  std::string yaml_path;
  bool handshake = false;
  bool self_test = false;
  bool verbose = false;
  bool port_set = false;

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
      pos_kp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kd") == 0)
    {
      const char *v = need("--kd");
      if (v == nullptr)
      {
        return 2;
      }
      pos_kd = std::strtof(v, nullptr);
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

  if (!biped_hw::motorevo_codec_self_check() || !demux_self_check())
  {
    std::fprintf(stderr, "motorevo_codec/demux self-check failed\n");
    return 1;
  }
  if (self_test)
  {
    std::printf("motorevo_codec + demux self-check OK (no hardware)\n");
    return 0;
  }

  const auto *entry = biped_hw::find_by_can_id(kCanId);
  if (entry == nullptr || entry->master_id != biped_hw::kMasterId ||
      entry->model != biped_hw::MotorType::HA60)
  {
    std::fprintf(stderr, "motor map mismatch for CAN ID %u (need mapped HA60, Master 2)\n",
                 static_cast<unsigned>(kCanId));
    return 1;
  }

  if (!yaml_path.empty() && !port_set)
  {
    const std::string yport = left_port_from_yaml(yaml_path);
    if (yport.empty())
    {
      std::fprintf(stderr, "yaml overlay missing buses.left.port: %s\n", yaml_path.c_str());
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
  if (pos_kp < 0.f)
  {
    pos_kp = 0.f;
  }
  if (pos_kp > kPosKpCap)
  {
    std::printf("WARN: clamping pos_kp %.1f -> %.1f (P1 HA60 hold is low-gain only)\n",
                static_cast<double>(pos_kp), static_cast<double>(kPosKpCap));
    pos_kp = kPosKpCap;
  }
  if (pos_kd < 0.f)
  {
    pos_kd = 0.f;
  }
  if (pos_kd > kPosKdCap)
  {
    std::printf("WARN: clamping pos_kd %.2f -> %.2f (P1 HA60 hold is low-gain only)\n",
                static_cast<double>(pos_kd), static_cast<double>(kPosKdCap));
    pos_kd = kPosKdCap;
  }

  if (!g_slog.open())
  {
    std::fprintf(stderr, "WARN: could not open session log (%s)\n", g_slog.last_path.c_str());
  }

  g_slog.out(
      "WARN: HA60 ankle roll — motor may twitch. Conservative servo hold at current/zero.\n"
      "      left only %s  CAN ID %u (%s, %s)  Master %u\n"
      "      feedback on slave CAN %u OR Master %u; print rx_id each cycle\n"
      "      servo pos_kp=%.2f pos_kd=%.2f vel_kp/kd/ki=0  duration=%.1fs period=%dms\n"
      "      right adapter unused; Damiao CAN 3-7 (including 6) not commanded.\n",
      port.c_str(), static_cast<unsigned>(kCanId), biped_hw::to_string(entry->joint),
      biped_hw::to_string(entry->model), static_cast<unsigned>(entry->master_id),
      static_cast<unsigned>(kCanId), static_cast<unsigned>(biped_hw::kMasterId),
      static_cast<double>(pos_kp), static_cast<double>(pos_kd), duration_s, period_ms);
  g_slog.out("session_log last : %s\n", g_slog.last_path.c_str());
  g_slog.out("session_log stamp: %s\n",
             g_slog.stamp_path.empty() ? "(none)" : g_slog.stamp_path.c_str());

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  biped_hw::UsbCanAdapter adapter(port);
  if (!adapter.open())
  {
    g_slog.err("open %s failed: %s\n", port.c_str(), adapter.last_error().c_str());
    g_slog.close();
    return 1;
  }
  g_slog.out("opened %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n", port.c_str(),
             adapter.can_bitrate(), static_cast<unsigned>(adapter.can_baud_index()));

  DisableGuard guard;
  guard.arm(adapter);
  g_exit_adapter = &adapter;
  std::atexit(atexit_disable);

  if (handshake)
  {
    if (!adapter.handshake())
    {
      g_slog.err("handshake send failed\n");
      g_slog.close();
      return 1;
    }
    usleep(20000);
    adapter.drain();
    g_slog.out("handshake sent (CMD 0x02)\n");
  }

  adapter.drain();
  uint8_t cmd[8] = {};
  biped_hw::pack_motorevo_enable(cmd);
  if (!adapter.sendCan(kCanId, cmd, 8))
  {
    g_slog.err("enable send failed\n");
    g_slog.close();
    return 1;
  }
  g_slog.out("enable id=%u sent (0xFC)\n", static_cast<unsigned>(kCanId));
  usleep(kEnableSettleUs);
  adapter.drain();

  biped_hw::FrameDemux demux;
  biped_hw::CycleLogger log;
  float q_hold = 0.f;
  bool have_q = false;
  const auto t0 = biped_hw::CycleLogger::Clock::now();
  const auto duration = std::chrono::duration<double>(duration_s);
  const auto period = std::chrono::milliseconds(period_ms);
  auto next = t0;
  std::uint64_t printed = 0;
  auto last_print = t0;
  std::size_t rx_on_slave = 0;
  std::size_t rx_on_master = 0;

  while (g_run.load() && (biped_hw::CycleLogger::Clock::now() - t0) < duration)
  {
    next += period;
    const float q_cmd = have_q ? q_hold : 0.f;
    // Servo only. Mix pack lives in motorevo_codec.hpp; this app does not switch.
    biped_hw::pack_motorevo_servo(cmd, q_cmd, pos_kp, pos_kd, 0.f, 0.f, 0.f, 0.f);
    const auto t_send = biped_hw::CycleLogger::Clock::now();
    if (!adapter.sendCan(kCanId, cmd, 8))
    {
      g_slog.err("servo send failed; disabling\n");
      break;
    }
    log.on_send(kCanId, kCanId, t_send);

    biped_hw::MotorevoFeedback fb{};
    bool got = false;
    while (biped_hw::CycleLogger::Clock::now() < next)
    {
      biped_hw::CanRxFrame rx;
      if (adapter.pollCan(rx))
      {
        demux.push(rx);
        if (demux.take_motorevo(kCanId, fb))
        {
          log.on_rx(kCanId, fb.rx_can_id, biped_hw::CycleLogger::Clock::now());
          got = true;
          break;
        }
      }
    }
    if (!got)
    {
      log.on_timeout(kCanId);
    }
    else
    {
      q_hold = fb.q;
      have_q = true;
      if (fb.rx_can_id == kCanId)
      {
        ++rx_on_slave;
      }
      else if (fb.rx_can_id == biped_hw::kMasterId)
      {
        ++rx_on_master;
      }
    }

    const auto now = biped_hw::CycleLogger::Clock::now();
    const bool print_now =
        verbose || printed < 5 || log.last_drop() ||
        (now - last_print) >= std::chrono::milliseconds(500);
    if (print_now)
    {
      char extra[192];
      if (got)
      {
        std::snprintf(extra, sizeof(extra),
                      "q=%.4f dq=%.4f tau=%.4f status=%u(%s) err=%u hold_q=%.4f",
                      static_cast<double>(fb.q), static_cast<double>(fb.dq),
                      static_cast<double>(fb.tau), static_cast<unsigned>(fb.status),
                      biped_hw::motorevo_status_name(fb.status),
                      static_cast<unsigned>(fb.err_code), static_cast<double>(q_hold));
      }
      else
      {
        std::snprintf(extra, sizeof(extra),
                      "no Motorevo reply on id=%u or Master %u (hold_q=%.4f)",
                      static_cast<unsigned>(kCanId),
                      static_cast<unsigned>(biped_hw::kMasterId),
                      static_cast<double>(q_hold));
      }
      print_cycle_tee(log, extra);
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
  if (g_slog.last_fp != nullptr)
  {
    log.print_summary(g_slog.last_fp);
  }
  if (g_slog.stamp_fp != nullptr)
  {
    log.print_summary(g_slog.stamp_fp);
  }
  g_slog.out("rx_id_slave8=%zu rx_id_master2=%zu overflow_drops=%zu have_hold_q=%d\n",
             rx_on_slave, rx_on_master, demux.overflow_drops(), have_q ? 1 : 0);

  guard.disable_now();
  g_exit_adapter = nullptr;
  adapter.close();
  g_slog.close();
  return (log.rx() > 0) ? 0 : 1;
}
