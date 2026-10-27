// 泉智博电机闭环测试。使能/写零/回零在 dm_mit_host；本文件默认 servo 跟踪。
// CAN ID 默认 kCanId，可用 --can-id 覆盖（十进制）。
#include "biped_hw/cli.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/metrics.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
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
#include <vector>

namespace
{

// Default kCanId; --can-id overrides (decimal). Must be mapped Motorevo HA60, Master 2.
constexpr uint8_t kCanId = 8;
constexpr int kDisableGapUs = 10000;
constexpr float kDefaultPosKp = 8.f;
constexpr float kDefaultPosKd = 0.5f;
constexpr float kPosKpCap = 20.f;  // P1 hold 15；P2 跟踪允许略抬，仍保守
constexpr float kPosKdCap = 2.f;
constexpr float kDefaultVelKp = 20.f;    // 指南例 50；踝滚仍保守，5 偏弱
constexpr float kDefaultVelKd = 0.f;
constexpr float kDefaultVelKi = 0.001f;  // 指南伺服例
constexpr float kVelKpCap = 50.f;        
constexpr float kVelKdCap = 2.f;
constexpr float kDefaultVelLimit = 0.5f;  // rad/s 过渡上限；指南例 2.0，踝滚放慢
constexpr float kVelLimitCap = 2.f;
constexpr float kDefaultPosAmp = 0.05f;     // ~3 deg
constexpr float kDefaultVelStroke = 0.15f;  // rad
constexpr float kDefaultVelAmp = 0.15f;     // rad/s
constexpr float kPosAmpCap = 0.20f;
constexpr float kVelStrokeCap = 0.25f;
constexpr float kDefaultAbortTau = biped_hw::kLimitHa60.tau_max;
constexpr float kSoftQStopAbs = 5.f;     // 常见 +5 rad 软限位；CAN 打包仍是 ±12.5
constexpr float kNearStopRad = 0.2f;

std::atomic<bool> g_run{true};
std::atomic<uint8_t> g_esc_id{kCanId};
biped_hw::UsbCanAdapter *g_exit_adapter = nullptr;

struct SessionLog
{
  std::string last_path{"/home/ye/00_jfl_26.10/Biped_Hw/log/biped_hw_p2_evo_last.log"};
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

  void file(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
  {
    va_list ap;
    va_start(ap, fmt);
    emit(nullptr, fmt, ap);
    va_end(ap);
  }

  bool open()
  {
    return true;
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

struct SessionLogGuard
{
  ~SessionLogGuard() { g_slog.close(); }
};

void print_cycle_tee(const biped_hw::CycleLogger &log, const char *extra, bool console)
{
  if (!console)
  {
    return;
  }
  log.print_cycle(stdout, extra);
  std::fflush(stdout);
}

enum class StageKind
{
  Pos,
  Vel,
};

struct StageResult
{
  const char *name = "";
  std::uint64_t samples = 0;
  biped_hw::ErrorStats err{};
  biped_hw::RttStats rtt{};
  biped_hw::PeriodJitterStats jitter{0.0};
  double settle_s = -1.0;
  bool have_fb = false;
  float last_q = 0.f;
  float last_q_des = 0.f;
  float last_dq = 0.f;
  float last_dq_des = 0.f;
  float last_kp = 0.f;
  float last_kd = 0.f;
  float last_tau_fb = 0.f;
  float dq_des_peak = 0.f;
  float q_peak_abs = 0.f;
  std::size_t rx_on_slave = 0;
  std::size_t rx_on_master = 0;
};

//程序退出时的信号处理函数  
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
  g_exit_adapter->sendCan(g_esc_id.load(), buf, 8);
  usleep(kDisableGapUs);
  g_exit_adapter->sendCan(g_esc_id.load(), buf, 8);
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

bool is_motorevo(biped_hw::MotorType type)
{
  return type == biped_hw::MotorType::HA60;
}

void usage()
{
  g_slog.out(
      "Usage: p2_evo_test [options]\n"
      "  P2 Motorevo HA60 test. Default is firmware servo after 0xFC.\n"
      "  Enable / set-zero / home are in dm_mit_host (evo_*).\n"
      "  --port PATH           default socketcan:can0 (use /dev/ttyACM0 for legacy USB2CAN)\n"
      "  --can-id N            decimal slave ID (default kCanId=%u). 8 not 0x08.\n"
      "  --period-ms MS        control period (default 10 = 100 Hz)\n"
      "  --stage pos|vel|all   default pos (no torque/current stage on HA60)\n"
      "                        Before --stage: probe |q|; if |q|>%.3f rad\n"
      "                        (kDmHomeEpsRad), speed-limited servo return to 0\n"
      "                        (θ_ref=0, V_ref=%.1f rad/s) until |q|<=eps, timeout\n"
      "                        max(%.0f, |q|/min(vel,%.0f)+%.0f) cap %.0f s, or abort-tau.\n"
      "                        Then the stage. Fail → 0xFD, no profile. Not\n"
      "                        --set-zero / --self-test. Not 0xFE (firmware zero).\n"
      "  --pos-amp RAD         position step magnitude (default 0.05 ~3 deg)\n"
      "  --pos-sign +1|-1      step q0±pos_amp (default -1 if q0>1 or near +5 stop)\n"
      "  --vel-amp RAD/S       vel-stage sine peak speed (default 0.15)\n"
      "  --vel-stroke RAD      sine peak displacement (default 0.15)\n"
      "  --kp --kd             servo pos gains (default 8 / 0.5, cap 20 / 2)\n"
      "  --vel-kp --vel-kd --vel-ki  cascaded vel loop (default 20 / 0 / 0.001; official 50)\n"
      "  --vel-limit RAD/S     Byte2 V_ref transition speed (default 0.5, cap 2)\n"
      "  --duration-pos SEC    position stage (default 5)\n"
      "  --duration-vel SEC    velocity-as-sine stage (default 6)\n"
      "  --abort-tau NM        abort if |tau_fb| exceeds (default HA60 TMAX 50)\n"
      "  --yaml PATH           overlay buses.left.port\n"
      "  --handshake           USB-CAN adapter handshake\n"
      "  --self-test           codec + metrics + demux, no hardware\n"
      "  --verbose             per-cycle console log\n"
      "  --print-tx            hex dump servo TX (first + q_des edges)\n"
      "  --set-zero            Set-zero 67 06 FE 00 00 00 15 76 on 0x600+ID,\n"
      "                        then SaveFlash 67 00 … 15 76 on the same ID.\n"
      "                        No --stage / --home. Needs --port. Then 0xFD.\n"
      "  --home                Enable servo, then evo_home_to_zero (θ_ref=0, not 0xFE).\n"
      "                        No --stage / --set-zero. Needs --port. Then 0xFD.\n"
      "  After --stage home, commands are around measured q0. Disable 0xFD.\n"
      "  Official servo: θ_ref rad, V_ref=max speed rad/s, vel_kp>0.\n"
      "  Single-motor bring-up. Port is --port / ACM0, not the map bus.\n"
      "  Never commands Damiao MIT motors.\n",
      static_cast<unsigned>(kCanId),
      static_cast<double>(biped_hw::kDmHomeEpsRad),
      static_cast<double>(biped_hw::kEvoHomeVelLimit),
      static_cast<double>(biped_hw::kDmHomeTimeoutS),
      static_cast<double>(biped_hw::kDmHomeVel),
      static_cast<double>(biped_hw::kDmHomeTimeoutSlackS),
      static_cast<double>(biped_hw::kDmHomeTimeoutMaxS));
}

struct DisableGuard
{
  biped_hw::UsbCanAdapter *adapter = nullptr;
  uint8_t can_id = kCanId;
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
    biped_hw::evo_send_disable(*adapter, can_id);
    usleep(kDisableGapUs);
    g_slog.out("disable id=%u sent (0xFD)\n", static_cast<unsigned>(can_id));
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

bool q0_pack_self_check()
{
  // P1 hold sat near q≈5 rad. A q=0 command would be a huge jump.
  // POS_MAX is 12.5 rad (official); 5.0 is not a pack clamp.
  uint8_t buf[8] = {};
  biped_hw::pack_motorevo_servo(buf, 5.0f, 8.f, 0.5f, 0.5f, 20.f, 0.f, 0.001f);
  float q = 0.f, pkp = 0.f, pkd = 0.f, vlim = 0.f, vkp = 0.f, vkd = 0.f, vki = 0.f;
  biped_hw::unpack_motorevo_servo_cmd(buf, q, pkp, pkd, vlim, vkp, vkd, vki);
  if (std::fabs(q - 5.0f) > 0.01f)
  {
    return false;
  }
  if (std::fabs(vlim) < 0.2f || std::fabs(vkp) < 2.f)
  {
    return false;  // tracking pack must put V_ref and vel_kp on the wire
  }
  uint8_t zero[8] = {};
  biped_hw::pack_motorevo_servo(zero, 0.f, 8.f, 0.5f, 0.f, 0.f, 0.f, 0.f);
  float qz = 0.f;
  biped_hw::unpack_motorevo_servo_cmd(zero, qz, pkp, pkd, vlim, vkp, vkd, vki);
  if (std::fabs(qz) > 0.02f || std::fabs(q - qz) < 1.0f)
  {
    return false;
  }
  return true;
}

float clampf(float x, float lo, float hi)
{
  if (x < lo)
  {
    return lo;
  }
  if (x > hi)
  {
    return hi;
  }
  return x;
}

struct ServoCommand
{
  float q_des = 0.f;
  float dq_des = 0.f;
  float kp = 0.f;
  float kd = 0.f;
  float vel_kp = 0.f;
  float vel_kd = 0.f;
  float vel_ki = 0.f;
  float vel_limit = 0.f;
};

int signed_unit(int pos_sign)
{
  return pos_sign < 0 ? -1 : 1;
}

int auto_pos_sign(float q0)
{
  // q0 顶在约 +5 rad 时正阶跃动不了；q0>1 也默认朝 0。负止挡则正向。
  if (q0 <= -kSoftQStopAbs + kNearStopRad)
  {
    return 1;
  }
  if (q0 >= kSoftQStopAbs - kNearStopRad || q0 > 1.f)
  {
    return -1;
  }
  return -1;
}

ServoCommand pos_command(double t, double duration, float q0, float amp, int pos_sign)
{
  ServoCommand c;
  const double step1 = std::min(2.0, duration * 0.4);
  const float signed_amp = static_cast<float>(signed_unit(pos_sign)) * amp;
  if (t < step1)
  {
    c.q_des = q0 + signed_amp;
  }
  else
  {
    c.q_des = q0;
  }
  c.dq_des = 0.f;
  return c;
}

bool pos_sign_self_check()
{
  const ServoCommand neg = pos_command(0.1, 5.0, 4.999f, 0.05f, -1);
  const ServoCommand pos = pos_command(0.1, 5.0, 4.999f, 0.05f, 1);
  if (neg.q_des > 4.96f || pos.q_des < 5.03f)
  {
    return false;
  }
  if (auto_pos_sign(4.999f) != -1 || auto_pos_sign(-4.95f) != 1)
  {
    return false;
  }
  return true;
}

void print_servo_tx(uint32_t can_id, const uint8_t buf[8], float q, float kp, float kd,
                    float vel_limit, float vel_kp, bool console)
{
  float q_w = 0.f, kp_w = 0.f, kd_w = 0.f, vlim_w = 0.f, vkp = 0.f, vkd = 0.f, vki = 0.f;
  biped_hw::unpack_motorevo_servo_cmd(buf, q_w, kp_w, kd_w, vlim_w, vkp, vkd, vki);
  const char *fmt =
      "TX can_id=%u q=%.5f kp=%.2f kd=%.3f vel_limit=%.4f vel_kp=%.2f "
      "wire_q=%.5f wire_kp=%.2f wire_kd=%.3f wire_vlim=%.4f wire_vel_kp=%.2f "
      "wire_vel_kd=%.3f wire_vel_ki=%.4f "
      "data=%02X %02X %02X %02X %02X %02X %02X %02X\n";
  if (console)
  {
    g_slog.out(fmt, can_id, static_cast<double>(q), static_cast<double>(kp),
               static_cast<double>(kd), static_cast<double>(vel_limit),
               static_cast<double>(vel_kp), static_cast<double>(q_w),
               static_cast<double>(kp_w), static_cast<double>(kd_w),
               static_cast<double>(vlim_w), static_cast<double>(vkp),
               static_cast<double>(vkd), static_cast<double>(vki), buf[0], buf[1],
               buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  }
  else
  {
    g_slog.file(fmt, can_id, static_cast<double>(q), static_cast<double>(kp),
                static_cast<double>(kd), static_cast<double>(vel_limit),
                static_cast<double>(vel_kp), static_cast<double>(q_w),
                static_cast<double>(kp_w), static_cast<double>(kd_w),
                static_cast<double>(vlim_w), static_cast<double>(vkp),
                static_cast<double>(vkd), static_cast<double>(vki), buf[0], buf[1],
                buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  }
}

bool run_stage(StageKind kind, biped_hw::UsbCanAdapter &adapter,
               biped_hw::FrameDemux &demux, float q0, float safe_q_margin,
               float abort_tau, double duration_s, int period_ms, float kp, float kd,
               float vel_kp, float vel_kd, float vel_ki, float vel_limit, float pos_amp,
               int pos_sign, float vel_amp, float vel_stroke, bool verbose, bool print_tx,
               StageResult &out)
{
  using Clock = biped_hw::CycleLogger::Clock;
  const auto period = std::chrono::milliseconds(period_ms);
  const double nominal_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(period).count());

  out.name = (kind == StageKind::Pos) ? "position" : "velocity";
  out.jitter = biped_hw::PeriodJitterStats(nominal_us);
  biped_hw::CycleLogger log;
  std::vector<double> pos_errs;
  if (kind == StageKind::Pos)
  {
    pos_errs.reserve(static_cast<std::size_t>(duration_s * 1000.0 / period_ms + 4));
  }

  const auto t0 = Clock::now();
  auto next = t0;
  auto last_tick = t0;
  std::uint64_t printed = 0;
  auto last_print = t0;
  bool aborted = false;
  float last_tx_qdes = 1e9f;
  float last_tx_dqdes = 1e9f;
  float q_des_int = q0;
  const uint8_t esc_id = g_esc_id.load();

  g_slog.out("\n--- stage %s: duration=%.1fs kp=%.2f kd=%.2f vel_kp=%.2f vel_limit=%.3f "
             "q0=%.4f ---\n",
             out.name, duration_s, static_cast<double>(kp), static_cast<double>(kd),
             static_cast<double>(vel_kp), static_cast<double>(vel_limit),
             static_cast<double>(q0));
  if (kind == StageKind::Vel)
  {
    g_slog.out("servo vel: sinusoidal position track  kp=%.1f kd=%.3f  "
               "vel_kp=%.1f V_ref=%.3f rad/s  dq_peak=%.3f  stroke=±%.3f rad\n",
               static_cast<double>(kp), static_cast<double>(kd),
               static_cast<double>(vel_kp), static_cast<double>(vel_limit),
               static_cast<double>(vel_amp), static_cast<double>(vel_stroke));
  }
  else
  {
    g_slog.out("servo pos: step q0%+.4f then back to q0 (pos-sign=%d); "
               "V_ref=%.3f rad/s vel_kp=%.2f (not hold-only)\n",
               static_cast<double>(static_cast<float>(signed_unit(pos_sign)) * pos_amp),
               pos_sign, static_cast<double>(vel_limit), static_cast<double>(vel_kp));
  }

  while (g_run.load() && !aborted &&
         (Clock::now() - t0) < std::chrono::duration<double>(duration_s))
  {
    next += period;
    const auto now_pre = Clock::now();
    const double elapsed = std::chrono::duration<double>(now_pre - t0).count();

    ServoCommand cmd{};
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.vel_kp = vel_kp;
    cmd.vel_kd = vel_kd;
    cmd.vel_ki = vel_ki;
    cmd.vel_limit = vel_limit;
    if (kind == StageKind::Pos)
    {
      cmd = pos_command(elapsed, duration_s, q0, pos_amp, pos_sign);
      cmd.kp = kp;
      cmd.kd = kd;
      cmd.vel_kp = vel_kp;
      cmd.vel_kd = vel_kd;
      cmd.vel_ki = vel_ki;
      cmd.vel_limit = vel_limit;
      const float q_lo = q0 - safe_q_margin;
      const float q_hi = q0 + safe_q_margin;
      cmd.q_des = clampf(cmd.q_des, q_lo, q_hi);
    }
    else
    {
      const float signed_amp = static_cast<float>(signed_unit(pos_sign)) * vel_amp;
      const double omega =
          vel_stroke > 1e-6f ? static_cast<double>(vel_amp / vel_stroke) : 0.0;
      const float phase = static_cast<float>(omega * elapsed);
      float vel_cmd = signed_amp * static_cast<float>(std::cos(phase));
      q_des_int = q0 + static_cast<float>(signed_unit(pos_sign)) * vel_stroke *
                           static_cast<float>(std::sin(phase));
      const float q_lo = q0 - vel_stroke;
      const float q_hi = q0 + vel_stroke;
      q_des_int = clampf(q_des_int, q_lo, q_hi);
      cmd.q_des = q_des_int;
      cmd.dq_des = vel_cmd;
      cmd.vel_limit = std::max(vel_limit, std::fabs(vel_amp));
    }

    uint8_t buf[8] = {};
    // 官方伺服：Byte2=V_ref 过渡上限（正值 rad/s），不是 dq 指令；Byte5=vel_kp。
    biped_hw::pack_motorevo_servo(buf, cmd.q_des, cmd.kp, cmd.kd, cmd.vel_limit,
                                  cmd.vel_kp, cmd.vel_kd, cmd.vel_ki);
    const bool first = out.samples == 0;
    const bool edge = (std::fabs(cmd.q_des - last_tx_qdes) > 1e-6f ||
                       std::fabs(cmd.dq_des - last_tx_dqdes) > 1e-6f);
    // 首包和 q_des 边沿始终解包打印，确认线上是阶跃而不是 hold_q。
    if (first || edge || print_tx)
    {
      print_servo_tx(esc_id, buf, cmd.q_des, cmd.kp, cmd.kd, cmd.vel_limit, cmd.vel_kp,
                     /*console=*/true);
      last_tx_qdes = cmd.q_des;
      last_tx_dqdes = cmd.dq_des;
    }

    const auto t_send = Clock::now();
    if (!adapter.sendCan(esc_id, buf, 8))
    {
      g_slog.err("servo send failed in stage %s\n", out.name);
      break;
    }
    log.on_send(esc_id, esc_id, t_send);

    biped_hw::MotorevoFeedback fb{};
    bool got = false;
    while (Clock::now() < next)
    {
      biped_hw::CanRxFrame rx;
      if (adapter.pollCan(rx))
      {
        demux.push(rx);
        if (demux.take_motorevo(esc_id, fb))
        {
          log.on_rx(esc_id, fb.rx_can_id, Clock::now());
          got = true;
          break;
        }
      }
    }
    if (!got)
    {
      log.on_timeout(esc_id);
    }

    const auto t_rx = Clock::now();
    const double period_us = static_cast<double>(
        std::chrono::duration_cast<std::chrono::microseconds>(t_rx - last_tick).count());
    if (out.samples > 0)
    {
      out.jitter.push(period_us);
    }
    last_tick = t_rx;

    double err = 0.0;
    if (got)
    {
      if (fb.rx_can_id == esc_id)
      {
        ++out.rx_on_slave;
      }
      else if (fb.rx_can_id == biped_hw::kMasterId)
      {
        ++out.rx_on_master;
      }

      if (kind == StageKind::Pos)
      {
        err = static_cast<double>(cmd.q_des - fb.q);
        pos_errs.push_back(err);
      }
      else
      {
        err = static_cast<double>(cmd.dq_des - fb.dq);
      }
      out.err.push(err);
      ++out.samples;

      if (std::fabs(fb.q - q0) > safe_q_margin)
      {
        g_slog.err("ABORT stage %s: |q-q0|=%.4f > margin %.4f\n", out.name,
                   static_cast<double>(std::fabs(fb.q - q0)),
                   static_cast<double>(safe_q_margin));
        aborted = true;
      }
      if (std::fabs(fb.tau) > abort_tau)
      {
        g_slog.err("ABORT stage %s: |tau_fb|=%.3f > %.3f Nm\n", out.name,
                   static_cast<double>(std::fabs(fb.tau)), static_cast<double>(abort_tau));
        aborted = true;
      }
      out.have_fb = true;
      out.last_q = fb.q;
      out.last_q_des = cmd.q_des;
      out.last_dq = fb.dq;
      out.last_dq_des = cmd.dq_des;
      out.last_kp = cmd.kp;
      out.last_kd = cmd.kd;
      out.last_tau_fb = fb.tau;
      if (std::fabs(cmd.dq_des) > std::fabs(out.dq_des_peak))
      {
        out.dq_des_peak = cmd.dq_des;
      }
      const float q_dev = std::fabs(fb.q - q0);
      if (q_dev > out.q_peak_abs)
      {
        out.q_peak_abs = q_dev;
      }
    }

    const bool to_console =
        verbose && (printed < 3 || log.last_drop() ||
                    (t_rx - last_print) >= std::chrono::milliseconds(200));
    if (to_console)
    {
      char extra[384];
      if (got)
      {
        if (kind == StageKind::Vel)
        {
          std::snprintf(
              extra, sizeof(extra),
              "dq_des=%.4f dq=%.4f e=%.4f q_des=%.4f q=%.4f kp=%.1f kd=%.3f "
              "tau=%.4f peak_|q-q0|=%.4f status=%u(%s) err=%u rx_id=%u period_us=%.0f",
              static_cast<double>(cmd.dq_des), static_cast<double>(fb.dq), err,
              static_cast<double>(cmd.q_des), static_cast<double>(fb.q),
              static_cast<double>(cmd.kp), static_cast<double>(cmd.kd),
              static_cast<double>(fb.tau), static_cast<double>(out.q_peak_abs),
              static_cast<unsigned>(fb.status), biped_hw::motorevo_status_name(fb.status),
              static_cast<unsigned>(fb.err_code), fb.rx_can_id, period_us);
        }
        else
        {
          std::snprintf(
              extra, sizeof(extra),
              "q_des=%.4f q=%.4f e=%.4f dq=%.4f tau=%.4f kp=%.1f vel_kp=%.1f "
              "vlim=%.3f status=%u(%s) err=%u rx_id=%u period_us=%.0f",
              static_cast<double>(cmd.q_des), static_cast<double>(fb.q), err,
              static_cast<double>(fb.dq), static_cast<double>(fb.tau),
              static_cast<double>(cmd.kp), static_cast<double>(cmd.vel_kp),
              static_cast<double>(cmd.vel_limit), static_cast<unsigned>(fb.status),
              biped_hw::motorevo_status_name(fb.status),
              static_cast<unsigned>(fb.err_code), fb.rx_can_id, period_us);
        }
      }
      else
      {
        std::snprintf(extra, sizeof(extra),
                      "no Motorevo reply on id=%u or Master %u period_us=%.0f",
                      static_cast<unsigned>(esc_id),
                      static_cast<unsigned>(biped_hw::kMasterId), period_us);
      }
      print_cycle_tee(log, extra, true);
      last_print = t_rx;
      ++printed;
    }

    const auto remain = next - Clock::now();
    if (remain > std::chrono::milliseconds(1))
    {
      usleep(static_cast<useconds_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(remain).count()));
    }
  }

  out.rtt = biped_hw::RttStats::from_cycle_logger(log);
  if (kind == StageKind::Pos && !pos_errs.empty())
  {
    const double sample_s = static_cast<double>(period_ms) / 1000.0;
    out.settle_s =
        biped_hw::estimate_settle_s(pos_errs.data(), pos_errs.size(),
                                    static_cast<double>(pos_amp) * 0.1, 10, sample_s);
  }

  g_slog.out("stage %s done: samples=%llu err_rms=%.5f err_max=%.5f rtt_mean=%.3fms "
             "jitter_max_dev=%.0fus  tmos=n/a (HA60 feedback has no MOS temp)\n",
             out.name, static_cast<unsigned long long>(out.samples), out.err.rms(),
             out.err.max(), out.rtt.mean_ms, out.jitter.max_deviation_us());
  if (kind == StageKind::Vel)
  {
    g_slog.out("  last dq_des=%.4f dq=%.4f q_des=%.4f q=%.4f kp=%.1f kd=%.3f  "
               "peak_|q-q0|=%.4f  peak_dq_des=%.4f\n",
               static_cast<double>(out.last_dq_des), static_cast<double>(out.last_dq),
               static_cast<double>(out.last_q_des), static_cast<double>(out.last_q),
               static_cast<double>(out.last_kp), static_cast<double>(out.last_kd),
               static_cast<double>(out.q_peak_abs),
               static_cast<double>(out.dq_des_peak));
  }
  if (kind == StageKind::Pos && out.settle_s >= 0.0)
  {
    g_slog.out("  settle_time~%.3fs (10%% of amp, 10 samples)\n", out.settle_s);
  }
  return !aborted && out.samples > 0;
}

void print_summary_table(const std::vector<StageResult> &results)
{
  g_slog.out("\n=== P2 evo servo summary ===\n");
  g_slog.out("%-10s %8s %10s %10s %10s %12s %10s %10s\n", "stage", "samples",
             "err_rms", "err_max", "rtt_mean", "jit_max_us", "rx_slave",
             "rx_master2");
  for (const auto &r : results)
  {
    g_slog.out("%-10s %8llu %10.5f %10.5f %10.3f %12.0f %10zu %10zu\n", r.name,
               static_cast<unsigned long long>(r.samples), r.err.rms(), r.err.max(),
               r.rtt.mean_ms, r.jitter.max_deviation_us(), r.rx_on_slave, r.rx_on_master);
  }
  g_slog.out("tmos: skipped (Motorevo HA60 unpack has status/q/dq/tau, no MOS temp)\n");
}

void print_end_banner(float q0, bool have_q0, float pos_amp, int pos_sign, float kp,
                      float vel_kp, const std::vector<StageResult> &results)
{
  g_slog.out("\n========== P2 EVO END BANNER (scroll-safe) ==========\n");
  g_slog.out("mode: Motorevo servo only  CAN ID %u  (HA60 only, no Damiao MIT)\n",
             static_cast<unsigned>(g_esc_id.load()));
  if (have_q0)
  {
    g_slog.out("q0=%.5f rad  (commands must stay near q0, not 0)\n",
               static_cast<double>(q0));
  }
  else
  {
    g_slog.out("q0: n/a\n");
  }

  const StageResult *pos = nullptr;
  const StageResult *vel = nullptr;
  for (const auto &r : results)
  {
    if (r.name != nullptr && std::strcmp(r.name, "position") == 0)
    {
      pos = &r;
    }
    if (r.name != nullptr && std::strcmp(r.name, "velocity") == 0)
    {
      vel = &r;
    }
  }
  if (pos != nullptr && pos->have_fb)
  {
    g_slog.out("last pos: q_des=%.5f q=%.5f e=%.5f tau_fb=%.4f kp=%.1f kd=%.3f\n",
               static_cast<double>(pos->last_q_des), static_cast<double>(pos->last_q),
               static_cast<double>(pos->last_q_des - pos->last_q),
               static_cast<double>(pos->last_tau_fb),
               static_cast<double>(pos->last_kp), static_cast<double>(pos->last_kd));
    g_slog.out("position err_rms=%.5f err_max=%.5f samples=%llu peak_|q-q0|=%.5f\n",
               pos->err.rms(), pos->err.max(),
               static_cast<unsigned long long>(pos->samples),
               static_cast<double>(pos->q_peak_abs));
    const double hold_like = 0.7 * static_cast<double>(pos_amp);
    const bool no_motion = pos->q_peak_abs < 0.3f * pos_amp;
    const bool near_plus_stop =
        have_q0 && q0 >= kSoftQStopAbs - kNearStopRad;
    const bool near_minus_stop =
        have_q0 && q0 <= -kSoftQStopAbs + kNearStopRad;
    if (pos->err.max() >= hold_like && no_motion)
    {
      if (near_plus_stop && signed_unit(pos_sign) > 0)
      {
        g_slog.out("HINT: q0=%.4f sits on a +%.1f rad stop; +pos_amp cannot move "
                   "(peak_|q-q0|=%.5f).\n"
                   "      Do not raise --kp. Retry --pos-sign -1 (q0-pos_amp toward 0).\n"
                   "      Confirm TX wire_vel_kp>0 and wire_vlim>0 (old binary had Byte5=00).\n",
                   static_cast<double>(q0), static_cast<double>(kSoftQStopAbs),
                   static_cast<double>(pos->q_peak_abs));
      }
      else if (near_minus_stop && signed_unit(pos_sign) < 0)
      {
        g_slog.out("HINT: q0=%.4f sits on a -%.1f rad stop; -pos_amp cannot move "
                   "(peak_|q-q0|=%.5f).\n"
                   "      Do not raise --kp. Retry --pos-sign +1 (toward 0).\n",
                   static_cast<double>(q0), static_cast<double>(kSoftQStopAbs),
                   static_cast<double>(pos->q_peak_abs));
      }
      else if (near_plus_stop || pos->q_peak_abs < 0.002f)
      {
        g_slog.out("HINT: peak_|q-q0|=%.5f with q0=%.4f — not a position-kp problem.\n"
                   "      If the step was +pos_amp into a +5 rad stop, use --pos-sign -1.\n"
                   "      Else check TX: wire_vel_kp>0 wire_vlim>0 (vel_kp was %.1f; "
                   "official example 50).\n"
                   "      A log that still prints wire_dq / Byte5=00 is an old binary.\n",
                   static_cast<double>(pos->q_peak_abs), static_cast<double>(q0),
                   static_cast<double>(vel_kp));
      }
      else
      {
        g_slog.out("HINT: q stayed near q0 while q_des stepped (hold-only).\n"
                   "      Official servo needs vel_kp>0 and V_ref>0; raising --kp (was %.1f) "
                   "alone does nothing.\n"
                   "      Defaults now vel_kp=%.1f vel_limit=%.2f rad/s. Retry --stage pos.\n",
                   static_cast<double>(kp), static_cast<double>(kDefaultVelKp),
                   static_cast<double>(kDefaultVelLimit));
      }
    }
    else if (pos->err.max() < 0.5 * static_cast<double>(pos_amp))
    {
      g_slog.out("HINT: q followed q_des (err_max << pos_amp=%.4f). Position track looks OK.\n",
                 static_cast<double>(pos_amp));
    }
  }
  else
  {
    g_slog.out("position err_rms/max: n/a (no position stage samples)\n");
  }
  if (vel != nullptr && vel->have_fb)
  {
    g_slog.out("velocity last: dq_des=%.5f dq=%.5f q_des=%.5f q=%.5f kp=%.1f "
               "tau_fb=%.4f\n",
               static_cast<double>(vel->last_dq_des), static_cast<double>(vel->last_dq),
               static_cast<double>(vel->last_q_des), static_cast<double>(vel->last_q),
               static_cast<double>(vel->last_kp), static_cast<double>(vel->last_tau_fb));
    g_slog.out("velocity peak |q-q0|=%.5f  peak |dq_des|=%.5f  "
               "err_rms=%.5f (dq_des vs dq) err_max=%.5f samples=%llu\n",
               static_cast<double>(vel->q_peak_abs),
               static_cast<double>(std::fabs(vel->dq_des_peak)), vel->err.rms(),
               vel->err.max(), static_cast<unsigned long long>(vel->samples));
  }
  else
  {
    g_slog.out("velocity last: n/a (no velocity stage samples)\n");
  }
  g_slog.out("tmos: skipped\n");
  g_slog.out("=====================================================\n");
}

bool parse_stage(const char *s, bool &do_pos, bool &do_vel)
{
  if (s == nullptr)
  {
    return false;
  }
  if (std::strcmp(s, "all") == 0)
  {
    do_pos = do_vel = true;
    return true;
  }
  do_pos = do_vel = false;
  if (std::strcmp(s, "pos") == 0)
  {
    do_pos = true;
    return true;
  }
  if (std::strcmp(s, "vel") == 0)
  {
    do_vel = true;
    return true;
  }
  return false;
}

}  // namespace

int main(int argc, char **argv)
{
  int period_ms = 10;
  uint8_t esc_id = kCanId;
  std::string port = biped_hw::kDefaultPortLeft;
  std::string yaml_path;
  bool handshake = false;
  bool self_test = false;
  bool verbose = false;
  bool print_tx = false;
  bool port_set = false;
  bool stage_set = false;
  bool set_zero = false;
  bool home = false;

  bool do_pos = true;
  bool do_vel = false;

  float pos_amp = kDefaultPosAmp;
  float vel_amp = kDefaultVelAmp;
  float vel_stroke = kDefaultVelStroke;
  float pos_kp = kDefaultPosKp;
  float pos_kd = kDefaultPosKd;
  float vel_kp = kDefaultVelKp;
  float vel_kd = kDefaultVelKd;
  float vel_ki = kDefaultVelKi;
  float vel_limit = kDefaultVelLimit;
  double duration_pos = 5.0;
  double duration_vel = 6.0;
  float abort_tau = kDefaultAbortTau;
  int pos_sign = 0;
  bool pos_sign_set = false;

  std::vector<StageResult> results;
  float q0 = 0.f;
  bool have_q0 = false;

  // ---------------------------------Parse command-line arguments---------------------------
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
    if (std::strcmp(argv[i], "--period-ms") == 0)
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
      char *end = nullptr;
      const unsigned long id = std::strtoul(v, &end, 10);
      if (end == v || end == nullptr || *end != '\0' || id < 1 || id > 127)
      {
        std::fprintf(stderr, "--can-id is decimal 1–127 (8, not 0x08)\n");
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
    else if (std::strcmp(argv[i], "--stage") == 0)
    {
      const char *v = need("--stage");
      if (v == nullptr || !parse_stage(v, do_pos, do_vel))
      {
        std::fprintf(stderr, "--stage must be pos|vel|all (no torque on HA60)\n");
        return 2;
      }
      stage_set = true;
    }
    else if (std::strcmp(argv[i], "--pos-amp") == 0)
    {
      const char *v = need("--pos-amp");
      if (v == nullptr)
      {
        return 2;
      }
      pos_amp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--pos-sign") == 0)
    {
      const char *v = need("--pos-sign");
      if (v == nullptr)
      {
        return 2;
      }
      if (std::strcmp(v, "+1") == 0 || std::strcmp(v, "1") == 0 ||
          std::strcmp(v, "+") == 0)
      {
        pos_sign = 1;
        pos_sign_set = true;
      }
      else if (std::strcmp(v, "-1") == 0 || std::strcmp(v, "-") == 0)
      {
        pos_sign = -1;
        pos_sign_set = true;
      }
      else
      {
        std::fprintf(stderr, "--pos-sign must be +1 or -1\n");
        return 2;
      }
    }
    else if (std::strcmp(argv[i], "--vel-amp") == 0)
    {
      const char *v = need("--vel-amp");
      if (v == nullptr)
      {
        return 2;
      }
      vel_amp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-stroke") == 0)
    {
      const char *v = need("--vel-stroke");
      if (v == nullptr)
      {
        return 2;
      }
      vel_stroke = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kp") == 0 || std::strcmp(argv[i], "--kp-pos") == 0)
    {
      const char *v = need(argv[i]);
      if (v == nullptr)
      {
        return 2;
      }
      pos_kp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kd") == 0 || std::strcmp(argv[i], "--kd-pos") == 0)
    {
      const char *v = need(argv[i]);
      if (v == nullptr)
      {
        return 2;
      }
      pos_kd = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-kp") == 0)
    {
      const char *v = need("--vel-kp");
      if (v == nullptr)
      {
        return 2;
      }
      vel_kp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-kd") == 0)
    {
      const char *v = need("--vel-kd");
      if (v == nullptr)
      {
        return 2;
      }
      vel_kd = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-ki") == 0)
    {
      const char *v = need("--vel-ki");
      if (v == nullptr)
      {
        return 2;
      }
      vel_ki = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-limit") == 0)
    {
      const char *v = need("--vel-limit");
      if (v == nullptr)
      {
        return 2;
      }
      vel_limit = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--duration-pos") == 0)
    {
      const char *v = need("--duration-pos");
      if (v == nullptr)
      {
        return 2;
      }
      duration_pos = std::atof(v);
    }
    else if (std::strcmp(argv[i], "--duration-vel") == 0)
    {
      const char *v = need("--duration-vel");
      if (v == nullptr)
      {
        return 2;
      }
      duration_vel = std::atof(v);
    }
    else if (std::strcmp(argv[i], "--abort-tau") == 0)
    {
      const char *v = need("--abort-tau");
      if (v == nullptr)
      {
        return 2;
      }
      abort_tau = std::strtof(v, nullptr);
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
    else if (std::strcmp(argv[i], "--print-tx") == 0)
    {
      print_tx = true;
    }
    else if (std::strcmp(argv[i], "--set-zero") == 0)
    {
      set_zero = true;
    }
    else if (std::strcmp(argv[i], "--home") == 0)
    {
      home = true;
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

  if (set_zero && home)
  {
    std::fprintf(stderr, "refuse: --set-zero cannot be combined with --home\n");
    return 2;
  }
  if (set_zero && stage_set)
  {
    std::fprintf(stderr, "refuse: --set-zero cannot be combined with --stage\n");
    return 2;
  }
  if (home && stage_set)
  {
    std::fprintf(stderr, "refuse: --home cannot be combined with --stage\n");
    return 2;
  }
  if ((set_zero || home) && !port_set)
  {
    std::fprintf(stderr, "--set-zero/--home require --port\n");
    return 2;
  }

  // ---------------------------------Initialize session log---------------------------------
  SessionLogGuard slog_guard;
  (void)g_slog.open();

  if (!biped_hw::motorevo_codec_self_check() || !demux_self_check() || !q0_pack_self_check() ||
      !pos_sign_self_check())
  {
    g_slog.err("motorevo_codec/demux/q0-pack/pos-sign self-check failed\n");
    return 1;
  }
  if (!biped_hw::metrics_self_check())
  {
    g_slog.err("metrics self-check failed\n");
    return 1;
  }
  if (self_test)
  {
    g_slog.out("motorevo_codec + demux + metrics + q≈5 pack + PDF servo bytes self-check OK "
               "(no hardware)\n");
    return 0;
  }

  g_esc_id.store(esc_id);
  const auto *entry = biped_hw::find_by_can_id(esc_id);
  // CAN ID is kCanId. Do not switch --port / ACM0 to the map's right adapter.
  if (entry == nullptr || entry->master_id != biped_hw::kMasterId ||
      !is_motorevo(entry->model))
  {
    g_slog.err(
        "motor map mismatch for CAN ID %u (need mapped Motorevo HA60, Master 2)\n",
        static_cast<unsigned>(esc_id));
    return 1;
  }

  if (!yaml_path.empty() && !port_set)
  {
    const std::string yport = left_port_from_yaml(yaml_path);
    if (yport.empty())
    {
      g_slog.err("yaml overlay missing buses.left.port: %s\n", yaml_path.c_str());
      return 2;
    }
    port = yport;
  }

  if (period_ms < 2)
  {
    period_ms = 2;
  }
  if (pos_amp < 0.f)
  {
    pos_amp = -pos_amp;
  }
  if (vel_amp < 0.f)
  {
    vel_amp = -vel_amp;
  }
  if (vel_stroke < 0.f)
  {
    vel_stroke = -vel_stroke;
  }
  if (duration_pos < 0.5)
  {
    duration_pos = 0.5;
  }
  if (duration_vel < 0.5)
  {
    duration_vel = 0.5;
  }
  if (pos_kp < 0.f)
  {
    pos_kp = 0.f;
  }
  if (pos_kd < 0.f)
  {
    pos_kd = 0.f;
  }
  if (pos_kp > kPosKpCap)
  {
    g_slog.out("WARN: clamping pos_kp %.1f -> %.1f (HA60 P2 servo is low-gain)\n",
               static_cast<double>(pos_kp), static_cast<double>(kPosKpCap));
    pos_kp = kPosKpCap;
  }
  if (pos_kd > kPosKdCap)
  {
    g_slog.out("WARN: clamping pos_kd %.2f -> %.2f (HA60 P2 servo is low-gain)\n",
               static_cast<double>(pos_kd), static_cast<double>(kPosKdCap));
    pos_kd = kPosKdCap;
  }
  if (vel_kp < 0.f)
  {
    vel_kp = 0.f;
  }
  if (vel_kp > kVelKpCap)
  {
    g_slog.out("WARN: clamping vel_kp %.1f -> %.1f (ankle roll conservative)\n",
               static_cast<double>(vel_kp), static_cast<double>(kVelKpCap));
    vel_kp = kVelKpCap;
  }
  if (vel_kd < 0.f)
  {
    vel_kd = 0.f;
  }
  if (vel_kd > kVelKdCap)
  {
    vel_kd = kVelKdCap;
  }
  if (vel_ki < 0.f)
  {
    vel_ki = 0.f;
  }
  if (vel_ki > biped_hw::kMotorevoVelKiMax)
  {
    vel_ki = biped_hw::kMotorevoVelKiMax;
  }
  if (vel_limit < 0.f)
  {
    vel_limit = -vel_limit;
  }
  if (vel_limit > kVelLimitCap)
  {
    g_slog.out("WARN: clamping vel_limit %.3f -> %.3f rad/s\n",
               static_cast<double>(vel_limit), static_cast<double>(kVelLimitCap));
    vel_limit = kVelLimitCap;
  }
  if (vel_limit < 0.05f)
  {
    g_slog.out("WARN: vel_limit %.3f too small for tracking; raising to 0.05 rad/s\n",
               static_cast<double>(vel_limit));
    vel_limit = 0.05f;
  }
  if (vel_kp < 0.5f)
  {
    g_slog.out("WARN: vel_kp %.2f too small for cascaded servo; raising to 0.5\n",
               static_cast<double>(vel_kp));
    vel_kp = 0.5f;
  }
  if (vel_kp < 50.f)
  {
    g_slog.out("WARN: vel_kp=%.1f (official SDK servo example is 50; ankle roll "
               "conservative). Pass --vel-kp 50 if a negative step still has no motion.\n",
               static_cast<double>(vel_kp));
  }
  if (pos_amp > kPosAmpCap)
  {
    g_slog.out("WARN: clamping pos_amp %.4f -> %.4f rad (ankle roll safety)\n",
               static_cast<double>(pos_amp), static_cast<double>(kPosAmpCap));
    pos_amp = kPosAmpCap;
  }
  if (vel_stroke > kVelStrokeCap)
  {
    g_slog.out("WARN: clamping vel_stroke %.3f -> %.3f rad\n",
               static_cast<double>(vel_stroke), static_cast<double>(kVelStrokeCap));
    vel_stroke = kVelStrokeCap;
  }
  if (vel_stroke < 0.01f)
  {
    vel_stroke = 0.01f;
  }

  g_slog.out("cmdline:");
  for (int ai = 0; ai < argc; ++ai)
  {
    g_slog.out(" %s", argv[ai]);
  }
  g_slog.out("\n");

  g_slog.out(
      "\n"
      "**********************************************************************\n"
      "* WARNING: Motorevo HA60 servo — motor may twitch. Keep clear.       *\n"
      "* P2 uses SMALL servo steps around measured q0 (~5 rad), not 0.      *\n"
      "* Abort if |q-q0| too large or |tau_fb| > %.1f Nm. Disable 0xFD.     *\n"
      "* Motorevo HA60 only. Port is CLI/default ACM0, not the map bus.     *\n"
      "**********************************************************************\n"
      "port %s  CAN ID %u (%s, %s)  Master %u  period=%dms\n"
      "stages: pos=%d vel=%d  (no torque/current stage)\n"
      "pos_amp=%.3f rad  pos_sign=%s  vel_amp=%.2f rad/s  vel_stroke=%.3f rad\n"
      "servo kp=%.2f kd=%.2f  vel_kp=%.2f vel_kd=%.2f vel_ki=%.4f  "
      "V_ref/vel_limit=%.3f rad/s\n"
      "duration_pos=%.1fs duration_vel=%.1fs\n"
      "feedback on slave CAN %u OR Master %u; print rx_id each cycle\n",
      static_cast<double>(abort_tau), port.c_str(), static_cast<unsigned>(esc_id),
      biped_hw::to_string(entry->joint), biped_hw::to_string(entry->model),
      static_cast<unsigned>(entry->master_id), period_ms, do_pos ? 1 : 0, do_vel ? 1 : 0,
      static_cast<double>(pos_amp), pos_sign_set ? (pos_sign < 0 ? "-1" : "+1") : "auto",
      static_cast<double>(vel_amp),
      static_cast<double>(vel_stroke), static_cast<double>(pos_kp),
      static_cast<double>(pos_kd), static_cast<double>(vel_kp),
      static_cast<double>(vel_kd), static_cast<double>(vel_ki),
      static_cast<double>(vel_limit), duration_pos, duration_vel,
      static_cast<unsigned>(esc_id), static_cast<unsigned>(biped_hw::kMasterId));

  const float safe_q_margin =
      std::max(std::max(pos_amp * 2.f, vel_stroke + 0.05f), 0.15f);
  g_slog.out("safe_q_margin=%.4f rad  abort_tau=%.2f Nm\n",
             static_cast<double>(safe_q_margin), static_cast<double>(abort_tau));

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  biped_hw::UsbCanAdapter adapter(port);
  if (!adapter.open())
  {
    g_slog.err("open %s failed: %s\n", port.c_str(), adapter.last_error().c_str());
    print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);
    return 1;
  }
  g_slog.out("opened %s; USB2CAN CAN baud %u bps cmd=55 05 %02X AA 55\n", port.c_str(),
             adapter.can_bitrate(), static_cast<unsigned>(adapter.can_baud_index()));

  DisableGuard guard;
  guard.arm(adapter, esc_id);
  g_exit_adapter = &adapter;
  std::atexit(atexit_disable);

  if (handshake)
  {
    if (!adapter.handshake())
    {
      g_slog.err("handshake send failed\n");
      print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);
      return 1;
    }
    usleep(20000);
    adapter.drain();
    g_slog.out("handshake sent (CMD 0x02)\n");
  }

  if (set_zero)
  {
    g_slog.out(
        "WARN: --set-zero WRITES firmware zero at current shaft position "
        "(HA60 SetzeroPosition + SaveFlash). Irreversible-ish. Shaft must be at intended origin.\n");
    biped_hw::FrameDemux zdm;
    biped_hw::MotorevoFeedback fb_before{};
    biped_hw::MotorevoFeedback fb_after{};
    if (!biped_hw::evo_send_set_zero(adapter, esc_id, &zdm, &fb_before, &fb_after))
    {
      g_slog.err("set-zero send failed\n");
      print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);
      guard.disable_now();
      g_exit_adapter = nullptr;
      adapter.close();
      return 1;
    }
    char fb_line[256];
    biped_hw::format_motorevo_fb(fb_line, sizeof(fb_line), "set-zero fb before", fb_before);
    g_slog.out("%s\n", fb_line);
    biped_hw::format_motorevo_fb(fb_line, sizeof(fb_line), "set-zero fb after", fb_after);
    g_slog.out("%s\n", fb_line);
    g_slog.out("set-zero id=%u sent (HA60) set-zero+SaveFlash on 0x%X\n",
               static_cast<unsigned>(esc_id),
               biped_hw::motorevo_param_can_id(esc_id));
    const float q_after = fb_after.q;
    if (std::fabs(q_after) > 0.05f)
    {
      g_slog.out("WARN: q_after not near 0 — zero not applied. Do not power-cycle yet.\n");
    }
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return 0;
  }

  if (!biped_hw::evo_enable_servo(adapter, esc_id))
  {
    g_slog.err("enable send failed\n");
    print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);
    return 1;
  }
  g_slog.out("enable id=%u sent (0xFC) mode=servo\n", static_cast<unsigned>(esc_id));

  // Probe |q|; home to 0 if off zero (servo θ_ref=0). Not 0xFE.
  biped_hw::FrameDemux demux;
  biped_hw::DmHomeStatus hs{};
  const auto hr = biped_hw::evo_home_to_zero(adapter, demux, esc_id, abort_tau,
                                             biped_hw::kDmHomeTimeoutS,
                                             biped_hw::kDmHomeEpsRad, &hs);
  if (hr != biped_hw::DmHomeResult::ProbeFail)
  {
    q0 = hs.q_after;
    have_q0 = true;
  }
  g_slog.out("home id=%u q_before=%.4f %s q_after=%.4f eps=%.3f tau_peak=%.3f\n",
             static_cast<unsigned>(esc_id), static_cast<double>(hs.q_before),
             biped_hw::dm_home_result_name(hr), static_cast<double>(hs.q_after),
             static_cast<double>(biped_hw::kDmHomeEpsRad),
             static_cast<double>(hs.tau_peak));
  if (!biped_hw::dm_home_ok(hr))
  {
    g_slog.err("home FAIL (%s); 0xFD, not starting stage\n",
               biped_hw::dm_home_result_name(hr));
    print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return 1;
  }
  if (home)
  {
    print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return 0;
  }

  if (!pos_sign_set)
  {
    pos_sign = auto_pos_sign(q0);
    if (q0 >= kSoftQStopAbs - kNearStopRad)
    {
      g_slog.out("pos-sign auto=-1  q0=%.4f near +%.1f stop; first step q0-%.4f toward 0\n",
                 static_cast<double>(q0), static_cast<double>(kSoftQStopAbs),
                 static_cast<double>(pos_amp));
    }
    else if (q0 <= -kSoftQStopAbs + kNearStopRad)
    {
      g_slog.out("pos-sign auto=+1  q0=%.4f near -%.1f stop; first step q0+%.4f toward 0\n",
                 static_cast<double>(q0), static_cast<double>(kSoftQStopAbs),
                 static_cast<double>(pos_amp));
    }
    else
    {
      g_slog.out("pos-sign auto=%d  first step q0%+.4f\n", pos_sign,
                 static_cast<double>(static_cast<float>(signed_unit(pos_sign)) * pos_amp));
    }
  }
  else
  {
    g_slog.out("pos-sign %d  first step q0%+.4f\n", pos_sign,
               static_cast<double>(static_cast<float>(signed_unit(pos_sign)) * pos_amp));
  }

  int rc = 0;
  if (do_pos && g_run.load())
  {
    StageResult r;
    if (!run_stage(StageKind::Pos, adapter, demux, q0, safe_q_margin, abort_tau,
                   duration_pos, period_ms, pos_kp, pos_kd, vel_kp, vel_kd, vel_ki,
                   vel_limit, pos_amp, pos_sign, vel_amp, vel_stroke, verbose, print_tx,
                   r))
    {
      rc = 1;
    }
    results.push_back(r);
  }
  if (do_vel && g_run.load())
  {
    StageResult r;
    if (!run_stage(StageKind::Vel, adapter, demux, q0, safe_q_margin, abort_tau,
                   duration_vel, period_ms, pos_kp, pos_kd, vel_kp, vel_kd, vel_ki,
                   vel_limit, pos_amp, pos_sign, vel_amp, vel_stroke, verbose, print_tx,
                   r))
    {
      rc = 1;
    }
    results.push_back(r);
  }

  print_summary_table(results);
  print_end_banner(q0, have_q0, pos_amp, pos_sign, pos_kp, vel_kp, results);

  guard.disable_now();
  g_exit_adapter = nullptr;
  adapter.close();
  return rc;
}
