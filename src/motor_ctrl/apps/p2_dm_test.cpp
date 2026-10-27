// 达妙电机闭环测试。CTRL_MODE / 0xFE / 回零在 dm_mit_host；本文件只跑阶梯。
// CAN ID 默认 kCanId，可用 --can-id 覆盖（十进制）。默认 --ctrl-mode mit。
#include "biped_hw/cli.hpp"
#include "biped_hw/cycle_logger.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/metrics.hpp"
#include "biped_hw/motor_map.hpp"
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
#include <string>
#include <unistd.h>
#include <vector>

namespace
{

// Default kCanId; --can-id overrides (decimal). Must be mapped Damiao MIT, Master 2.
constexpr uint8_t kCanId = 9;
constexpr int kEnableSettleUs = 100000;
constexpr int kDisableGapUs = 10000;
constexpr float kLimitFracPos = 0.1f;   // 10% of PMAX
constexpr float kLimitFracVel = 0.1f;   // 10% of VMAX
constexpr float kLimitFracTau = 0.1f;   // 10% of TMAX
constexpr float kSafeQMarginFrac = 0.08f;
constexpr float kKpPosMax = biped_hw::kMitKpMax;
constexpr float kKdMitMax = biped_hw::kMitKdMax;
constexpr float kVelStrokeDefault = 0.2f;  // rad; 24V bench must not integrate forever

std::atomic<bool> g_run{true};
std::atomic<uint8_t> g_esc_id{kCanId};
biped_hw::UsbCanAdapter *g_exit_adapter = nullptr;

// Tee stdout/stderr to Biped_Hw/log last + timestamped copy so terminal scroll
// cannot swallow CTRL_MODE / REG TX / RID dump / MIT_LIVE_CHECK.
struct SessionLog
{
  std::string last_path{"/home/ye/00_jfl_26.10/Biped_Hw/log/biped_hw_p2_last.log"};
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
  Dq,
  Torque,
};

struct StageResult
{
  const char *name = "";
  std::uint64_t samples = 0;
  biped_hw::ErrorStats err{};
  biped_hw::RttStats rtt{};
  biped_hw::PeriodJitterStats jitter{0.0};
  uint8_t tmos_start = 0;
  uint8_t tcoil_start = 0;
  uint8_t tmos_end = 0;
  uint8_t tcoil_end = 0;
  double settle_s = -1.0;
  bool have_fb = false;
  float last_q = 0.f;
  float last_q_des = 0.f;
  float last_dq = 0.f;
  float last_dq_des = 0.f;
  float last_kd = 0.f;
  float last_kp = 0.f;
  float last_wire_dq = 0.f;
  float dq_des_peak = 0.f;  // max |dq_des| actually packed
  float q_peak_abs = 0.f;   // max |q - q0| this stage
  float last_tau_fb = 0.f;
  float last_tau_ff = 0.f;
  bool have_live_check = false;
  float live_q = 0.f;
  float live_q_des = 0.f;
  float live_tau_fb = 0.f;
  float live_tau_ff = 0.f;
  double live_e = 0.0;
  double live_expect = 0.0;
};

struct CtrlModeState
{
  bool write_attempted = false;
  bool before_ok = false;
  uint32_t before = 0;
  bool wrote_ack = false;
  bool after_ok = false;
  uint32_t after = 0;
  bool end_ok = false;
  uint32_t end = 0;
};

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


void usage()
{
  g_slog.out(
      "Usage: p2_dm_test [options]\n"
      "  P2 Damiao closed-loop test. Default --ctrl-mode mit (CTRL_MODE=1).\n"
      "  Mode write / enable / set-zero / home are in dm_mit_host.\n"
      "  --port PATH           default socketcan:can0 (use /dev/ttyACM0 for legacy USB2CAN)\n"
      "  --can-id N            decimal slave ID (default kCanId=%u). 12 not 0x0C.\n"
      "  --period-ms MS        control period (default 10 = 100 Hz)\n"
      "  --ctrl-mode/--mode mit|pos_vel|vel|pos_force|auto\n"
      "                        mit (default): MIT frames on ESC_ID, CTRL_MODE=1.\n"
      "                        pos_vel: CAN 0x100+ID, float pos+vel_limit, CTRL_MODE=2.\n"
      "                        vel: CAN 0x200+ID, float vel, CTRL_MODE=3.\n"
      "                        pos_force: CAN 0x300+ID, pos+vel_u16+i_u16, CTRL_MODE=4.\n"
      "                        auto: pos→POS_VEL, vel/dq→VEL, torque→POS_FORCE.\n"
      "  --i-ratio R           POS_FORCE current limit 0–1 (default max(0.01, tau_amp/TMAX))\n"
      "  --stage all|pos|vel|torque|dq\n"
      "                        all/pos/vel/torque/dq enable the motor (0xFC).\n"
      "                        dump-rids without --stage does not enable.\n"
      "                        all = pos then vel then torque (not dq).\n"
      "                        Stages that do not match --ctrl-mode are skipped.\n"
      "                        Before MIT --stage: same home-to-zero as --home.\n"
      "                        Fail → 0xFD, no profile. Not dump-rids / --set-zero\n"
      "                        / --home / --self-test. Not 0xFE.\n"
      "  --pos-amp RAD         position amplitude (default 0.05 ~3 deg)\n"
      "  --vel-amp RAD/S       vel stage sine peak speed; dq: MIT dq_des rad/s up to VMAX\n"
      "                        (default 0.5; host uses 5; not 5%% of VMAX)\n"
      "  --vel-stroke RAD      vel: |q_des-q0| window then vel_cmd=0 (default 0.2).\n"
      "                        dq: |q-q0| travel limit then disable 0xFD (not reverse).\n"
      "                        dq stroke cap is PMAX (several rad is legal; not 5%% PMAX).\n"
      "  --tau-amp NM          torque amplitude (default 0.5)\n"
      "  --kp-pos --kd-pos     position stage gains (default 80 / 1.0, clamp kp<=500, kd<=5)\n"
      "  --kp-vel --kd-vel     vel stage MIT gains. Default: same as kp_pos / kd_pos.\n"
      "                        --stage vel is a sinusoidal position track (not kd-only):\n"
      "                        q_des=q0+vel-stroke*sin(phase), kp~80.\n"
      "                        --stage dq: host-like MIT: kp=0, kd from --kd-vel\n"
      "                        (else --kd-pos, e.g. 2), dq_des=--vel-amp rad/s up to\n"
      "                        VMAX (e.g. 5; not 5%% VMAX), q_des=0, tau from --tau-ff.\n"
      "                        Hard stop: --duration-vel OR --vel-stroke then 0xFD.\n"
      "  --duration-pos SEC    position stage (default 5)\n"
      "  --duration-vel SEC    velocity / dq stage (default 8)\n"
      "  --duration-tau SEC    torque stage (default 3)\n"
      "  --abort-tau NM        abort if |tau_fb| exceeds (default that motor TMAX)\n"
      "  --tau-ff NM           MIT t_ff on pos, vel, and dq stages (default 0, clamp 1%% TMAX)\n"
      "  --dump-rids           RID hex dump (diagnostic). No enable unless --stage.\n"
      "                        unless --stage. No 0xFE write unless --set-zero.\n"
      "                        IDs>=10: if binary ESC (12=0x0C) gets no\n"
      "                        REG RX, retry BCD 0x12.\n"
      "  --can-baud BPS        USB2CAN CAN bitrate (default 1000000 = 1 Mbps)\n"
      "  --handshake           optional USB2CAN CMD 0x02 after open (off by default)\n"
      "  --self-test           metrics math only, no hardware\n"
      "  --verbose             per-cycle console log\n"
      "  --print-tx            hex dump command TX (first + edges)\n"
      "  --set-zero            WRITES firmware zero (current pos → motor zero),\n"
      "                        MIT 0xFE via dm_send_set_zero. No --stage / --home.\n"
      "                        Needs --port. No 0xFC; then 0xFD.\n"
      "  --home                Enable MIT, then dm_home_to_zero (host-dq toward 0,\n"
      "                        not 0xFE). No --stage / --set-zero. Needs --port.\n"
      "                        Then 0xFD.\n"
      "  Single-motor bring-up. Port is --port / ACM0, not the map bus.\n"
      "  Never commands Motorevo HA60.\n",
      static_cast<unsigned>(kCanId));
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
    biped_hw::dm_send_disable(*adapter, can_id);
    usleep(kDisableGapUs);
    g_slog.out("disable id=%u sent (0xFD)\n", static_cast<unsigned>(can_id));
    armed = false;
  }

  ~DisableGuard() { disable_now(); }
};

void print_usbcan_raw_tx(const char *note, uint32_t can_id, const uint8_t *data, uint8_t len)
{
  uint8_t raw[biped_hw::UsbCanAdapter::kUsbCanTxSize] = {};
  biped_hw::UsbCanAdapter::pack_can_tx(raw, can_id, data, len);
  char line[220];
  int off = std::snprintf(
      line, sizeof(line),
      "USB2CAN TX %s 30B (CMD@3=03 len@2=1E CAN@13=7FF LE payload@21):", note);
  for (std::size_t i = 0; i < biped_hw::UsbCanAdapter::kUsbCanTxSize && off > 0 &&
                          off < static_cast<int>(sizeof(line)) - 4;
       ++i)
  {
    off += std::snprintf(line + off, sizeof(line) - static_cast<std::size_t>(off), " %02X",
                         raw[i]);
  }
  if (off > 0 && off < static_cast<int>(sizeof(line)) - 2)
  {
    line[off] = '\n';
    line[off + 1] = '\0';
  }
  g_slog.out("%s", line);
}

void print_can_payload(const char *tag, uint32_t can_id, const uint8_t *d, uint8_t n)
{
  char line[160];
  int off = std::snprintf(line, sizeof(line), "%s can_id=%u (0x%X) data=", tag, can_id, can_id);
  const uint8_t n8 = n > 8 ? 8 : n;
  for (uint8_t i = 0; i < n8 && off > 0 && off < static_cast<int>(sizeof(line)) - 4; ++i)
  {
    off += std::snprintf(line + off, sizeof(line) - static_cast<std::size_t>(off), "%s%02X",
                         i ? " " : "", d[i]);
  }
  if (off > 0 && off < static_cast<int>(sizeof(line)) - 2)
  {
    line[off] = '\n';
    line[off + 1] = '\0';
  }
  g_slog.out("%s", line);
}

void print_usbcan_rx_stats(const biped_hw::UsbCanAdapter &adapter, const char *tag)
{
  const biped_hw::UsbCanRxStats s = adapter.rx_stats();
  g_slog.out("USB2CAN RX %s: serial_bytes=%llu queued=%zu polls=%u ok=%u "
             "incomplete=%u bad_cmd=%u adapter_status=%u",
             tag, static_cast<unsigned long long>(s.serial_bytes), s.queued_bytes,
             s.polls, s.ok, s.incomplete, s.bad_cmd, s.adapter_status);
  if (s.adapter_status > 0)
  {
    g_slog.out(" last_status_CMD=0x%02X", static_cast<unsigned>(s.last_status_cmd));
  }
  if (s.bad_cmd > 0 && s.last_bad_len > 0)
  {
    g_slog.out(" last_reject CMD=0x%02X END=0x%02X raw=",
               static_cast<unsigned>(s.last_bad_cmd),
               static_cast<unsigned>(s.last_bad_end));
    for (uint8_t i = 0; i < s.last_bad_len; ++i)
    {
      g_slog.out("%s%02X", i ? " " : "", s.last_bad_raw[i]);
    }
  }
  g_slog.out("\n");
  if (s.serial_bytes == 0 && s.ok == 0)
  {
    g_slog.out(
        "USB2CAN RX %s: adapter returned no serial bytes to this process "
        "(close Damiao 上位机 so it does not hold ACM0; open() drops DTR, waits "
        "~2s, then sends 55 05 <idx> AA 55; handshake 55 AA 1E 02 is off "
        "unless --handshake).\n",
        tag);
  }
  else if (s.ok == 0 && s.serial_bytes > 0)
  {
    g_slog.out("USB2CAN RX %s: serial bytes arrived but no CMD 0x11 frame parsed.\n",
               tag);
  }
}

void print_usbcan_reject(const biped_hw::UsbCanRxStats &s)
{
  g_slog.out("USB2CAN reject CMD=0x%02X END=0x%02X raw=",
             static_cast<unsigned>(s.last_bad_cmd),
             static_cast<unsigned>(s.last_bad_end));
  for (uint8_t i = 0; i < s.last_bad_len; ++i)
  {
    g_slog.out("%s%02X", i ? " " : "", s.last_bad_raw[i]);
  }
  g_slog.out("\n");
}

void xact_on_tx(void *, uint32_t can_id, const uint8_t *d, uint8_t n)
{
  print_can_payload("REG TX", can_id, d, n);
  static bool printed_usb = false;
  if (!printed_usb)
  {
    print_usbcan_raw_tx("REG first", can_id, d, n);
    printed_usb = true;
  }
}

void xact_on_rx(void *, const biped_hw::CanRxFrame &rx)
{
  const char *tag =
      biped_hw::is_dm_register_payload(rx.data, rx.dlc) ? "REG RX" : "CAN RX";
  print_can_payload(tag, rx.can_id, rx.data, rx.dlc);
}

void xact_on_reject(void *, const biped_hw::UsbCanRxStats &s)
{
  print_usbcan_reject(s);
}

void xact_on_send_fail(void *)
{
  g_slog.out("REG TX send FAILED\n");
}

biped_hw::DmXactHook xact_hook(bool print)
{
  if (!print)
  {
    return {};
  }
  biped_hw::DmXactHook h;
  h.on_tx = xact_on_tx;
  h.on_rx = xact_on_rx;
  h.on_reject = xact_on_reject;
  h.on_send_fail = xact_on_send_fail;
  return h;
}

bool read_rid_retries(biped_hw::UsbCanAdapter &adapter, biped_hw::FrameDemux &demux,
                      uint8_t esc_id, uint8_t rid, float &value, bool print_xact = false)
{
  const biped_hw::DmXactHook hook = xact_hook(print_xact);
  return biped_hw::dm_read_rid(adapter, demux, esc_id, rid, value, print_xact ? &hook : nullptr);
}

[[maybe_unused]] bool write_rid_u32_retries(biped_hw::UsbCanAdapter &adapter,
                                             biped_hw::FrameDemux &demux, uint8_t esc_id,
                                             uint8_t rid, uint32_t value, bool print_xact)
{
  const biped_hw::DmXactHook hook = xact_hook(print_xact);
  return biped_hw::dm_write_rid_u32(adapter, demux, esc_id, rid, value,
                                    print_xact ? &hook : nullptr);
}

void print_rid_dump_line(bool ok, uint8_t rid, const char *name, float value, bool as_u32)
{
  if (!ok)
  {
    g_slog.out("RID %u %s: FAIL\n", static_cast<unsigned>(rid), name);
    return;
  }
  if (as_u32)
  {
    const uint32_t u = static_cast<uint32_t>(value + 0.5f);
    if (rid == static_cast<uint8_t>(biped_hw::DmRid::CtrlMode))
    {
      g_slog.out("RID %u %s: %u (%s)\n", static_cast<unsigned>(rid), name,
                 static_cast<unsigned>(u), biped_hw::dm_ctrl_mode_name(u));
    }
    else
    {
      g_slog.out("RID %u %s: %u\n", static_cast<unsigned>(rid), name, static_cast<unsigned>(u));
    }
    return;
  }
  g_slog.out("RID %u %s: %.6g\n", static_cast<unsigned>(rid), name, static_cast<double>(value));
}

void dump_diag_rids(biped_hw::UsbCanAdapter &adapter, biped_hw::FrameDemux &demux,
                    uint8_t esc_id, bool print_xact)
{
  g_slog.out("=== register dump (read 0x7FF / 0x33, same as P1 RID 21) ===\n");
  g_slog.out("2EC: dual encoder, datasheet has no brake RID; enable is 0xFC only.\n");
  const uint8_t bcd = biped_hw::dm_esc_id_as_bcd(esc_id);
  g_slog.out("ESC decimal=%u binary D[0]=0x%02X", static_cast<unsigned>(esc_id),
             static_cast<unsigned>(esc_id));
  if (esc_id >= 10)
  {
    g_slog.out(" BCD D[0]=0x%02X (SDK is binary; 上位机 IDs>=10 may store BCD). "
               "dump-rids retries BCD on 0x7FF if binary gets no REG RX.\n",
               static_cast<unsigned>(bcd));
  }
  else
  {
    g_slog.out("\n");
  }
  struct Item
  {
    uint8_t rid;
    const char *name;
    bool as_u32;
  };
  const Item items[] = {
      {static_cast<uint8_t>(biped_hw::DmRid::CtrlMode), "CTRL_MODE", true},
      {static_cast<uint8_t>(biped_hw::DmRid::OcValue), "OC_Value", false},
      {static_cast<uint8_t>(biped_hw::DmRid::KtValue), "KT_Value", false},
      {static_cast<uint8_t>(biped_hw::DmRid::Gref), "GREF", false},
      {static_cast<uint8_t>(biped_hw::DmRid::Vbus), "VBus", false},
      {static_cast<uint8_t>(biped_hw::DmRid::IBase), "IBase", false},
      {static_cast<uint8_t>(biped_hw::DmRid::Timeout), "TIMEOUT", true},
      {static_cast<uint8_t>(biped_hw::DmRid::SwVer), "sw_ver", true},
      {static_cast<uint8_t>(biped_hw::DmRid::Gr), "Gr", false},
  };
  uint8_t wire = esc_id;
  bool tried_bcd = false;
  for (const auto &it : items)
  {
    float v = 0.f;
    bool ok = read_rid_retries(adapter, demux, wire, it.rid, v, print_xact);
    if (!ok && esc_id >= 10 && !tried_bcd && bcd != esc_id)
    {
      g_slog.out("NOTE: binary ESC 0x%02X no REG RX; retry 0x7FF with BCD 0x%02X "
                 "(decimal %u → 0x%02X, not 0x%02X). Look for REG RX D[0]=%02X.\n",
                 static_cast<unsigned>(esc_id), static_cast<unsigned>(bcd),
                 static_cast<unsigned>(esc_id), static_cast<unsigned>(bcd),
                 static_cast<unsigned>(esc_id), static_cast<unsigned>(bcd));
      uint8_t buf[8] = {};
      biped_hw::pack_register_read(buf, bcd, it.rid);
      print_usbcan_raw_tx("REG BCD probe", biped_hw::kDmRegCanId, buf, 8);
      tried_bcd = true;
      ok = read_rid_retries(adapter, demux, bcd, it.rid, v, print_xact);
      if (ok)
      {
        g_slog.out("NOTE: REG RX matched BCD esc-byte 0x%02X (not binary 0x%02X). "
                   "Remaining dump-rids reads use 0x%02X.\n",
                   static_cast<unsigned>(bcd), static_cast<unsigned>(esc_id),
                   static_cast<unsigned>(bcd));
        wire = bcd;
      }
      else
      {
        g_slog.out("NOTE: BCD 0x%02X also no REG RX (not only an ID encoding miss).\n",
                   static_cast<unsigned>(bcd));
      }
    }
    print_rid_dump_line(ok, it.rid, it.name, v, it.as_u32);
  }
}

void print_mit_tx(uint32_t can_id, const uint8_t buf[8], const biped_hw::LimitParam &lim,
                  float kp, float kd, float q, float dq, float tau, bool console)
{
  float kp_w = 0.f, kd_w = 0.f, q_w = 0.f, dq_w = 0.f, tau_w = 0.f;
  biped_hw::unpack_mit_cmd(buf, lim, kp_w, kd_w, q_w, dq_w, tau_w);
  const char *fmt =
      "TX can_id=%u kp=%.2f kd=%.3f q=%.5f dq=%.4f tau=%.4f "
      "wire_kp=%.2f wire_kd=%.3f wire_q=%.5f wire_dq=%.4f wire_tau=%.4f "
      "data=%02X %02X %02X %02X %02X %02X %02X %02X\n";
  if (console)
  {
    g_slog.out(fmt, can_id, static_cast<double>(kp), static_cast<double>(kd),
               static_cast<double>(q), static_cast<double>(dq), static_cast<double>(tau),
               static_cast<double>(kp_w), static_cast<double>(kd_w), static_cast<double>(q_w),
               static_cast<double>(dq_w), static_cast<double>(tau_w), buf[0], buf[1], buf[2],
               buf[3], buf[4], buf[5], buf[6], buf[7]);
  }
  else
  {
    g_slog.file(fmt, can_id, static_cast<double>(kp), static_cast<double>(kd),
                static_cast<double>(q), static_cast<double>(dq), static_cast<double>(tau),
                static_cast<double>(kp_w), static_cast<double>(kd_w), static_cast<double>(q_w),
                static_cast<double>(dq_w), static_cast<double>(tau_w), buf[0], buf[1], buf[2],
                buf[3], buf[4], buf[5], buf[6], buf[7]);
  }
}

void print_mode_tx(uint32_t ctrl_mode, uint32_t can_id, const uint8_t buf[8],
                   const biped_hw::LimitParam &lim, float kp, float kd, float q, float dq,
                   float tau, bool console)
{
  if (ctrl_mode == biped_hw::kDmMitMode)
  {
    print_mit_tx(can_id, buf, lim, kp, kd, q, dq, tau, console);
    return;
  }
  char line[240];
  if (ctrl_mode == biped_hw::kDmPosVelMode)
  {
    float p = 0.f, v = 0.f;
    biped_hw::unpack_pos_vel_cmd(buf, p, v);
    std::snprintf(line, sizeof(line),
                  "TX POS_VEL can_id=%u (0x%X) pos=%.5f vel=%.4f wire_pos=%.5f wire_vel=%.4f "
                  "data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                  can_id, can_id, static_cast<double>(q), static_cast<double>(dq),
                  static_cast<double>(p), static_cast<double>(v), buf[0], buf[1], buf[2],
                  buf[3], buf[4], buf[5], buf[6], buf[7]);
  }
  else if (ctrl_mode == biped_hw::kDmVelMode)
  {
    float v = 0.f;
    biped_hw::unpack_vel_cmd(buf, v);
    std::snprintf(line, sizeof(line),
                  "TX VEL can_id=%u (0x%X) vel=%.4f wire_vel=%.4f "
                  "data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                  can_id, can_id, static_cast<double>(dq), static_cast<double>(v), buf[0],
                  buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
  }
  else
  {
    float p = 0.f, vlim = 0.f, ir = 0.f;
    biped_hw::unpack_pos_force_cmd(buf, p, vlim, ir);
    std::snprintf(line, sizeof(line),
                  "TX POS_FORCE can_id=%u (0x%X) pos=%.5f vel_lim=%.4f i_ratio=%.4f "
                  "wire_pos=%.5f wire_vel=%.4f wire_i=%.4f "
                  "data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                  can_id, can_id, static_cast<double>(q), static_cast<double>(dq),
                  static_cast<double>(tau), static_cast<double>(p), static_cast<double>(vlim),
                  static_cast<double>(ir), buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
                  buf[6], buf[7]);
  }
  if (console)
  {
    g_slog.out("%s", line);
  }
  else
  {
    g_slog.file("%s", line);
  }
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

struct MitCommand
{
  float kp = 0.f;
  float kd = 0.f;
  float q = 0.f;
  float dq = 0.f;
  float tau = 0.f;
  float q_des = 0.f;
  float dq_des = 0.f;
  float tau_cmd = 0.f;
};

MitCommand pos_command(double t, double duration, float q0, float amp)
{
  MitCommand c;
  c.kp = 0.f;  // filled by caller
  c.kd = 0.f;
  // Step: 0-2s to q0+amp, 2-4s back to q0, rest hold q0 (sine if longer).
  const double step1 = std::min(2.0, duration * 0.4);
  const double step2 = std::min(4.0, duration * 0.8);
  if (t < step1)
  {
    c.q_des = q0 + amp;
  }
  else if (t < step2)
  {
    c.q_des = q0;
  }
  else if (duration > step2 + 0.5)
  {
    const double ts = t - step2;
    const double period = std::max(2.0, duration - step2);
    c.q_des = q0 + amp * static_cast<float>(std::sin(2.0 * M_PI * ts / period));
  }
  else
  {
    c.q_des = q0;
  }
  c.q = c.q_des;
  c.dq = 0.f;
  c.tau = 0.f;
  return c;
}

MitCommand vel_command(double t, float amp, float stroke)
{
  // Sinusoidal velocity for a sinusoidal position track.
  MitCommand c;
  const double omega = stroke > 1e-6f ? static_cast<double>(amp / stroke) : 0.0;
  c.q_des = stroke * static_cast<float>(std::sin(omega * t));
  c.dq_des = amp * static_cast<float>(std::cos(omega * t));
  c.q = c.q_des;
  c.dq = c.dq_des;
  return c;
}

MitCommand torque_command(double t, double duration, float amp)
{
  MitCommand c;
  const double seg = duration / 4.0;
  if (t < seg)
  {
    c.tau_cmd = 0.f;
  }
  else if (t < 2.0 * seg)
  {
    c.tau_cmd = amp;
  }
  else if (t < 3.0 * seg)
  {
    c.tau_cmd = -amp;
  }
  else
  {
    c.tau_cmd = 0.f;
  }
  c.tau = c.tau_cmd;
  c.q = 0.f;
  c.dq = 0.f;
  return c;
}

bool run_stage(StageKind kind, biped_hw::UsbCanAdapter &adapter,
               biped_hw::FrameDemux &demux, uint8_t esc_id,
               const biped_hw::LimitParam &lim, float q0, float safe_q_margin,
               float abort_tau, double duration_s, int period_ms, float kp, float kd,
               float pos_amp, float vel_amp, float vel_stroke, float tau_amp,
               float tau_ff, bool verbose, bool print_tx, uint32_t ctrl_mode,
               float i_ratio, StageResult &out)
{
  using Clock = biped_hw::CycleLogger::Clock;
  const auto period = std::chrono::milliseconds(period_ms);
  const double nominal_us =
      static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(period).count());

  switch (kind)
  {
  case StageKind::Pos:
    out.name = "position";
    break;
  case StageKind::Vel:
    out.name = "velocity";
    break;
  case StageKind::Dq:
    out.name = "dq";
    break;
  case StageKind::Torque:
    out.name = "torque";
    break;
  }

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
  bool have_temp = false;
  std::uint64_t printed = 0;
  auto last_print = t0;
  bool aborted = false;
  bool stroke_stop = false;
  float last_tx_qdes = 1e9f;
  float last_tx_dqdes = 1e9f;
  float last_tx_tau = 1e9f;
  bool printed_live_check = false;
  float wire_dq = 0.f;
  float q_des_int = q0;

  const float mit_kp = (kind == StageKind::Torque || kind == StageKind::Dq) ? 0.f : kp;
  const float mit_tau_ff = (kind == StageKind::Torque) ? 0.f : tau_ff;
  const bool native_vel =
      ctrl_mode == biped_hw::kDmVelMode &&
      (kind == StageKind::Vel || kind == StageKind::Dq);
  const bool pos_force_hold =
      ctrl_mode == biped_hw::kDmPosForceMode && kind == StageKind::Torque;
  g_slog.out("\n--- stage %s: duration=%.1fs CTRL_MODE=%s (%u) kp=%.2f kd=%.2f "
             "tau_ff=%.3f i_ratio=%.3f tx_id=0x%X ---\n",
             out.name, duration_s, biped_hw::dm_ctrl_mode_name(ctrl_mode),
             static_cast<unsigned>(ctrl_mode), static_cast<double>(mit_kp),
             static_cast<double>(kd), static_cast<double>(mit_tau_ff),
             static_cast<double>(i_ratio),
             biped_hw::dm_cmd_can_id(ctrl_mode, esc_id));
  if (kind == StageKind::Vel && ctrl_mode == biped_hw::kDmMitMode)
  {
    g_slog.out("MIT vel: sinusoidal position track  kp=%.1f kd=%.3f  "
               "dq_des=±%.3f then 0  stroke=±%.3f rad  t_ff=%.3f\n",
               static_cast<double>(kp), static_cast<double>(kd),
               static_cast<double>(vel_amp), static_cast<double>(vel_stroke),
               static_cast<double>(tau_ff));
  }
  if (kind == StageKind::Vel && ctrl_mode == biped_hw::kDmPosVelMode)
  {
    g_slog.out("POS_VEL vel: sinusoidal velocity profile  v_des=|vel_cmd|  "
               "stroke=±%.3f rad\n",
               static_cast<double>(vel_stroke));
  }
  if (native_vel)
  {
    g_slog.out("VEL: signed vel_cmd on CAN 0x200+ID  stroke=±%.3f then 0xFD  "
               "duration=%.1fs\n",
               static_cast<double>(vel_stroke), duration_s);
  }
  if (kind == StageKind::Dq && ctrl_mode == biped_hw::kDmMitMode)
  {
    g_slog.out("MIT dq: kp=0 kd=%.3f  dq_des=+%.3f constant  q_des=0  "
               "stroke=±%.3f then 0xFD  duration=%.1fs  t_ff=%.3f\n",
               static_cast<double>(kd), static_cast<double>(vel_amp),
               static_cast<double>(vel_stroke), duration_s, static_cast<double>(tau_ff));
  }
  if (kind == StageKind::Pos && ctrl_mode == biped_hw::kDmPosVelMode)
  {
    g_slog.out("POS_VEL pos: p_des step/sine  v_des=%.3f rad/s (trapezoid max)\n",
               static_cast<double>(vel_amp));
  }
  if (ctrl_mode == biped_hw::kDmPosForceMode)
  {
    g_slog.out("POS_FORCE: p_des  v_lim=%.3f rad/s  i_ratio=%.3f (current limit)\n",
               static_cast<double>(vel_amp), static_cast<double>(i_ratio));
  }

  while (g_run.load() && !aborted && !stroke_stop &&
         (Clock::now() - t0) < std::chrono::duration<double>(duration_s))
  {
    next += period;
    const auto now_pre = Clock::now();
    const double elapsed =
        std::chrono::duration<double>(now_pre - t0).count();

    MitCommand cmd{};
    switch (kind)
    {
    case StageKind::Pos:
      cmd = pos_command(elapsed, duration_s, q0, pos_amp);
      cmd.kp = kp;
      cmd.kd = kd;
      cmd.tau = tau_ff;
      break;
    case StageKind::Vel:
      cmd = vel_command(elapsed, vel_amp, vel_stroke);
      cmd.kp = kp;
      cmd.kd = kd;
      cmd.tau = tau_ff;
      cmd.tau_cmd = tau_ff;
      break;
    case StageKind::Dq:
      cmd.kp = 0.f;
      cmd.kd = kd;
      cmd.q_des = 0.f;
      cmd.q = 0.f;
      cmd.dq_des = vel_amp;
      cmd.dq = vel_amp;
      cmd.tau = tau_ff;
      cmd.tau_cmd = tau_ff;
      break;
    case StageKind::Torque:
      if (pos_force_hold)
      {
        cmd.q_des = q0;
        cmd.q = q0;
        cmd.dq_des = vel_amp;
        cmd.dq = vel_amp;
        cmd.tau = i_ratio;
        cmd.tau_cmd = i_ratio;
        cmd.kp = 0.f;
        cmd.kd = 0.f;
      }
      else
      {
        cmd = torque_command(elapsed, duration_s, tau_amp);
        // Datasheet: MIT torque is kp=0, kd=0, t_ff. Unchanged this pass (still q=0).
        cmd.kp = 0.f;
        cmd.kd = kd;
      }
      break;
    }

    // Clamp commands to safe fraction of motor limits.
    if (kind == StageKind::Pos)
    {
      const float q_lo = q0 - safe_q_margin;
      const float q_hi = q0 + safe_q_margin;
      cmd.q = clampf(cmd.q, q_lo, q_hi);
      cmd.q_des = cmd.q;
    }
    if (kind == StageKind::Vel && !native_vel)
    {
      const float vlim = lim.dq_max * kLimitFracVel;
      float vel_cmd = clampf(cmd.dq_des, -vlim, vlim);
      q_des_int = q0 + vel_stroke * static_cast<float>(
                                   std::sin((vel_stroke > 1e-6f ? vel_amp / vel_stroke : 0.f) *
                                            elapsed));
      cmd.q_des = q_des_int;
      cmd.q = q_des_int;
      cmd.dq_des = vel_cmd;
      cmd.dq = vel_cmd;
    }
    if (kind == StageKind::Dq || native_vel)
    {
      const float vel_cmd = clampf(cmd.dq_des, -lim.dq_max, lim.dq_max);
      cmd.dq_des = vel_cmd;
      cmd.dq = vel_cmd;
      cmd.q_des = 0.f;
      cmd.q = 0.f;
      cmd.kp = 0.f;
    }
    if (kind == StageKind::Torque && !pos_force_hold)
    {
      const float tlim = lim.tau_max * kLimitFracTau;
      cmd.tau = clampf(cmd.tau, -tlim, tlim);
      cmd.tau_cmd = cmd.tau;
    }

    uint8_t buf[8] = {};
    const uint32_t tx_id = biped_hw::dm_cmd_can_id(ctrl_mode, esc_id);
    switch (ctrl_mode)
    {
    case biped_hw::kDmPosVelMode:
    {
      float vlim = std::fabs(cmd.dq);
      if (kind == StageKind::Pos && vlim < 1e-6f)
      {
        vlim = vel_amp;
      }
      biped_hw::pack_pos_vel(buf, cmd.q, vlim);
      wire_dq = vlim;
      break;
    }
    case biped_hw::kDmVelMode:
      biped_hw::pack_vel(buf, cmd.dq);
      wire_dq = cmd.dq;
      break;
    case biped_hw::kDmPosForceMode:
      biped_hw::pack_pos_force(buf, cmd.q, vel_amp, i_ratio);
      wire_dq = vel_amp;
      break;
    default:
      biped_hw::pack_mit(buf, cmd.kp, cmd.kd, cmd.q, cmd.dq, cmd.tau, lim);
      {
        float wire_kp = 0.f, wire_kd = 0.f, wire_q = 0.f, wire_tau = 0.f;
        biped_hw::unpack_mit_cmd(buf, lim, wire_kp, wire_kd, wire_q, wire_dq, wire_tau);
        (void)wire_kp;
        (void)wire_kd;
        (void)wire_q;
        (void)wire_tau;
      }
      break;
    }
    if (print_tx)
    {
      const bool first = out.samples < 8;
      const bool edge =
          (kind == StageKind::Vel || kind == StageKind::Dq)
              ? (std::fabs(cmd.dq_des - last_tx_dqdes) > 1e-6f ||
                 std::fabs(cmd.tau_cmd - last_tx_tau) > 1e-6f)
              : (std::fabs(cmd.q_des - last_tx_qdes) > 1e-6f ||
                 std::fabs(cmd.dq_des - last_tx_dqdes) > 1e-6f ||
                 std::fabs(cmd.tau_cmd - last_tx_tau) > 1e-6f);
      if (first || edge || verbose)
      {
        print_mode_tx(ctrl_mode, tx_id, buf, lim, cmd.kp, cmd.kd, cmd.q, cmd.dq, cmd.tau,
                      /*console=*/first || edge);
        last_tx_qdes = cmd.q_des;
        last_tx_dqdes = cmd.dq_des;
        last_tx_tau = cmd.tau_cmd;
      }
    }
    const auto t_send = Clock::now();
    if (!adapter.sendCan(tx_id, buf, 8))
    {
      g_slog.err("%s send failed in stage %s (can_id=0x%X)\n",
                 biped_hw::dm_ctrl_mode_name(ctrl_mode), out.name, tx_id);
      break;
    }
    log.on_send(esc_id, tx_id, t_send);

    biped_hw::DmFeedback fb{};
    bool got = false;
    while (Clock::now() < next)
    {
      biped_hw::CanRxFrame rx;
      if (adapter.pollCan(rx))
      {
        demux.push(rx);
        if (demux.take_mit(esc_id, lim, fb))
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
      if (out.samples == 0)
      {
        if (fb.has_temp)
        {
          out.tmos_start = fb.t_mos;
          out.tcoil_start = fb.t_coil;
          have_temp = true;
        }
      }
      if (fb.has_temp)
      {
        out.tmos_end = fb.t_mos;
        out.tcoil_end = fb.t_coil;
        have_temp = true;
      }

      switch (kind)
      {
      case StageKind::Pos:
        err = static_cast<double>(cmd.q_des - fb.q);
        pos_errs.push_back(err);
        break;
      case StageKind::Vel:
        err = static_cast<double>(cmd.dq_des - fb.dq);
        break;
      case StageKind::Dq:
        err = static_cast<double>(cmd.dq_des - fb.dq);
        break;
      case StageKind::Torque:
        err = pos_force_hold ? static_cast<double>(cmd.q_des - fb.q)
                             : static_cast<double>(cmd.tau_cmd - fb.tau);
        break;
      }
      out.err.push(err);
      ++out.samples;

      if ((kind == StageKind::Dq || native_vel) && std::fabs(fb.q - q0) >= vel_stroke)
      {
        g_slog.out("STOP stage %s: |q-q0|=%.4f >= vel-stroke %.4f; disable 0xFD\n",
                   out.name, static_cast<double>(std::fabs(fb.q - q0)),
                   static_cast<double>(vel_stroke));
        stroke_stop = true;
      }
      if (kind != StageKind::Dq && !native_vel && std::fabs(fb.q - q0) > safe_q_margin)
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
      out.last_kd = cmd.kd;
      out.last_kp = cmd.kp;
      out.last_wire_dq = wire_dq;
      if (std::fabs(cmd.dq_des) > std::fabs(out.dq_des_peak))
      {
        out.dq_des_peak = cmd.dq_des;
      }
      {
        const float q_dev = std::fabs(fb.q - q0);
        if (q_dev > out.q_peak_abs)
        {
          out.q_peak_abs = q_dev;
        }
      }
      out.last_tau_fb = fb.tau;
      out.last_tau_ff = cmd.tau;
      if (kind == StageKind::Pos && ctrl_mode == biped_hw::kDmMitMode && !printed_live_check &&
          std::fabs(err) > 0.5 * static_cast<double>(pos_amp))
      {
        const double expect = static_cast<double>(cmd.kp) * err + static_cast<double>(cmd.tau);
        g_slog.out(
            "MIT_LIVE_CHECK e=%.4f tau_fb=%.4f tau_ff=%.3f expect~%.2f Nm "
            "(kp*e+t_ff). |tau_fb|~0.12 means inner loop idle (LSB).\n",
            err, static_cast<double>(fb.tau), static_cast<double>(cmd.tau), expect);
        printed_live_check = true;
        out.have_live_check = true;
        out.live_q = fb.q;
        out.live_q_des = cmd.q_des;
        out.live_tau_fb = fb.tau;
        out.live_tau_ff = cmd.tau;
        out.live_e = err;
        out.live_expect = expect;
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
        if (kind == StageKind::Vel || kind == StageKind::Dq)
        {
          std::snprintf(
              extra, sizeof(extra),
              "dq_des=%.4f dq=%.4f e=%.4f q_des=%.4f q=%.4f kp=%.1f kd=%.3f "
              "tau_ff=%.3f tau_fb=%.4f peak_|q-q0|=%.4f err=%u period_us=%.0f",
              static_cast<double>(cmd.dq_des), static_cast<double>(fb.dq), err,
              static_cast<double>(cmd.q_des), static_cast<double>(fb.q),
              static_cast<double>(cmd.kp), static_cast<double>(cmd.kd),
              static_cast<double>(cmd.tau), static_cast<double>(fb.tau),
              static_cast<double>(out.q_peak_abs), static_cast<unsigned>(fb.err),
              period_us);
        }
        else
        {
          std::snprintf(
              extra, sizeof(extra),
              "q_des=%.4f q=%.4f e=%.4f dq=%.4f tau=%.4f kp=%.1f err=%u period_us=%.0f tmos=%u tcoil=%u",
              static_cast<double>(cmd.q_des), static_cast<double>(fb.q), err,
              static_cast<double>(fb.dq), static_cast<double>(fb.tau),
              static_cast<double>(cmd.kp), static_cast<unsigned>(fb.err), period_us,
              static_cast<unsigned>(fb.t_mos), static_cast<unsigned>(fb.t_coil));
        }
      }
      else
      {
        std::snprintf(extra, sizeof(extra), "no reply period_us=%.0f", period_us);
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
                                    static_cast<double>(pos_amp) * 0.1, 10,
                                    sample_s);
  }

  g_slog.out("stage %s done: samples=%llu err_rms=%.5f err_max=%.5f rtt_mean=%.3fms "
             "jitter_max_dev=%.0fus%s\n",
             out.name, static_cast<unsigned long long>(out.samples), out.err.rms(),
             out.err.max(), out.rtt.mean_ms, out.jitter.max_deviation_us(),
             have_temp ? "" : " (no temp)");
  if (kind == StageKind::Vel || kind == StageKind::Dq)
  {
    g_slog.out("  last dq_des=%.4f dq=%.4f q_des=%.4f q=%.4f kp=%.1f kd=%.3f  "
               "peak_|q-q0|=%.4f  peak_dq_des=%.4f%s\n",
               static_cast<double>(out.last_dq_des), static_cast<double>(out.last_dq),
               static_cast<double>(out.last_q_des), static_cast<double>(out.last_q),
               static_cast<double>(out.last_kp), static_cast<double>(out.last_kd),
               static_cast<double>(out.q_peak_abs),
               static_cast<double>(out.dq_des_peak),
               stroke_stop ? "  stroke-stop" : "");
  }
  if (kind == StageKind::Dq || native_vel)
  {
    biped_hw::dm_send_disable(adapter, esc_id);
    g_slog.out("disable id=%u sent (0xFD) after stage %s\n",
               static_cast<unsigned>(esc_id), out.name);
  }
  if (kind == StageKind::Pos && out.settle_s >= 0.0)
  {
    g_slog.out("  settle_time~%.3fs (10%% of amp, 10 samples)\n", out.settle_s);
  }
  return !aborted && out.samples > 0;
}

void print_summary_table(const std::vector<StageResult> &results)
{
  g_slog.out("\n=== P2 ladder summary ===\n");
  g_slog.out("%-10s %8s %10s %10s %10s %12s %8s %8s %8s %8s\n", "stage",
             "samples", "err_rms", "err_max", "rtt_mean", "jit_max_us",
             "tmos0", "tmos1", "tcoil0", "tcoil1");
  for (const auto &r : results)
  {
    g_slog.out("%-10s %8llu %10.5f %10.5f %10.3f %12.0f %8u %8u %8u %8u\n", r.name,
               static_cast<unsigned long long>(r.samples), r.err.rms(), r.err.max(),
               r.rtt.mean_ms, r.jitter.max_deviation_us(),
               static_cast<unsigned>(r.tmos_start), static_cast<unsigned>(r.tmos_end),
               static_cast<unsigned>(r.tcoil_start),
               static_cast<unsigned>(r.tcoil_end));
  }
}

const char *mode_ok_str(bool ok, uint32_t mode)
{
  return ok ? biped_hw::dm_ctrl_mode_name(mode) : "FAIL";
}

void print_end_banner(const CtrlModeState &cm, bool tau_ff_set, float tau_ff_user,
                      float tau_ff, float tau_cap, float q0, bool have_q0,
                      const std::vector<StageResult> &results)
{
  const bool mit_confirmed =
      (cm.after_ok && cm.after == biped_hw::kDmMitMode) ||
      (cm.end_ok && cm.end == biped_hw::kDmMitMode);
  g_slog.out("\n========== P2 END BANNER (scroll-safe) ==========\n");
  if (tau_ff_set)
  {
    g_slog.out("tau_ff: USED --tau-ff user=%.3f Nm  applied=%.3f Nm  cap(1%% TMAX)=%.3f Nm\n",
               static_cast<double>(tau_ff_user), static_cast<double>(tau_ff),
               static_cast<double>(tau_cap));
  }
  else
  {
    g_slog.out("tau_ff: not set (applied=%.3f Nm)\n", static_cast<double>(tau_ff));
  }
  g_slog.out("CTRL_MODE before write: %s%u (%s)\n", cm.before_ok ? "" : "FAIL ",
             static_cast<unsigned>(cm.before), mode_ok_str(cm.before_ok, cm.before));
  if (cm.write_attempted)
  {
    g_slog.out("CTRL_MODE write: %s  after: %s%u (%s)\n",
               cm.wrote_ack ? "ack" : "NO-ACK", cm.after_ok ? "" : "FAIL ",
               static_cast<unsigned>(cm.after), mode_ok_str(cm.after_ok, cm.after));
  }
  else
  {
    g_slog.out("CTRL_MODE write: not attempted (dump-rids or early exit)\n");
  }
  g_slog.out("CTRL_MODE last read: %s%u (%s)\n", cm.end_ok ? "" : "FAIL ",
             static_cast<unsigned>(cm.end), mode_ok_str(cm.end_ok, cm.end));
  g_slog.out("MIT confirmed: %s\n", mit_confirmed ? "YES (CTRL_MODE=1)" : "NO");
  if (have_q0)
  {
    g_slog.out("q0=%.5f rad\n", static_cast<double>(q0));
  }
  else
  {
    g_slog.out("q0: n/a\n");
  }
  const StageResult *pos = nullptr;
  for (const auto &r : results)
  {
    if (r.name != nullptr && std::strcmp(r.name, "position") == 0)
    {
      pos = &r;
      break;
    }
  }
  if (pos != nullptr && pos->have_live_check)
  {
    g_slog.out(
        "MIT_LIVE_CHECK snapshot: q_des=%.5f q=%.5f e=%.4f tau_fb=%.4f tau_ff=%.3f "
        "expect~%.2f Nm\n",
        static_cast<double>(pos->live_q_des), static_cast<double>(pos->live_q), pos->live_e,
        static_cast<double>(pos->live_tau_fb), static_cast<double>(pos->live_tau_ff),
        pos->live_expect);
  }
  else
  {
    g_slog.out("MIT_LIVE_CHECK snapshot: (not triggered this run)\n");
  }
  if (pos != nullptr && pos->have_fb)
  {
    g_slog.out("last cycle: q_des=%.5f q=%.5f tau_fb=%.4f tau_ff=%.3f\n",
               static_cast<double>(pos->last_q_des), static_cast<double>(pos->last_q),
               static_cast<double>(pos->last_tau_fb),
               static_cast<double>(pos->last_tau_ff));
    g_slog.out("position err_rms=%.5f err_max=%.5f samples=%llu\n", pos->err.rms(),
               pos->err.max(), static_cast<unsigned long long>(pos->samples));
  }
  else
  {
    g_slog.out("position err_rms/max: n/a (no position stage samples)\n");
  }
  const StageResult *vel = nullptr;
  for (const auto &r : results)
  {
    if (r.name != nullptr && std::strcmp(r.name, "velocity") == 0)
    {
      vel = &r;
      break;
    }
  }
  if (vel != nullptr && vel->have_fb)
  {
    g_slog.out("velocity last: dq_des=%.5f dq=%.5f q_des=%.5f q=%.5f kp=%.1f kd=%.3f "
               "tau_fb=%.4f\n",
               static_cast<double>(vel->last_dq_des), static_cast<double>(vel->last_dq),
               static_cast<double>(vel->last_q_des), static_cast<double>(vel->last_q),
               static_cast<double>(vel->last_kp), static_cast<double>(vel->last_kd),
               static_cast<double>(vel->last_tau_fb));
    g_slog.out("velocity peak |q-q0|=%.5f  peak |dq_des|=%.5f  "
               "err_rms=%.5f (dq_des vs dq) err_max=%.5f samples=%llu\n",
               static_cast<double>(vel->q_peak_abs),
               static_cast<double>(std::fabs(vel->dq_des_peak)),
               vel->err.rms(), vel->err.max(),
               static_cast<unsigned long long>(vel->samples));
  }
  else
  {
    g_slog.out("velocity last: n/a (no velocity stage samples)\n");
  }
  const StageResult *dq = nullptr;
  for (const auto &r : results)
  {
    if (r.name != nullptr && std::strcmp(r.name, "dq") == 0)
    {
      dq = &r;
      break;
    }
  }
  if (dq != nullptr && dq->have_fb)
  {
    g_slog.out("dq last: dq_des=%.5f dq=%.5f q_des=%.5f q=%.5f kp=%.1f kd=%.3f "
               "tau_fb=%.4f\n",
               static_cast<double>(dq->last_dq_des), static_cast<double>(dq->last_dq),
               static_cast<double>(dq->last_q_des), static_cast<double>(dq->last_q),
               static_cast<double>(dq->last_kp), static_cast<double>(dq->last_kd),
               static_cast<double>(dq->last_tau_fb));
    g_slog.out("dq peak |q-q0|=%.5f  peak |dq_des|=%.5f  "
               "err_rms=%.5f (dq_des vs dq) err_max=%.5f samples=%llu\n",
               static_cast<double>(dq->q_peak_abs),
               static_cast<double>(std::fabs(dq->dq_des_peak)),
               dq->err.rms(), dq->err.max(),
               static_cast<unsigned long long>(dq->samples));
  }
  else
  {
    g_slog.out("dq last: n/a (no dq stage samples)\n");
  }
  g_slog.out("=================================================\n");
}

void refresh_ctrl_mode_end(biped_hw::UsbCanAdapter &adapter, biped_hw::FrameDemux &demux,
                           uint8_t esc_id, CtrlModeState &cm)
{
  float f = 0.f;
  cm.end_ok = read_rid_retries(adapter, demux, esc_id,
                               static_cast<uint8_t>(biped_hw::DmRid::CtrlMode), f, false);
  cm.end = static_cast<uint32_t>(f + 0.5f);
  g_slog.out("CTRL_MODE RID10 last: %s%u (%s)\n", cm.end_ok ? "" : "FAIL ",
             static_cast<unsigned>(cm.end), mode_ok_str(cm.end_ok, cm.end));
}

bool parse_stage(const char *s, bool &do_pos, bool &do_vel, bool &do_tau, bool &do_dq)
{
  if (s == nullptr)
  {
    return false;
  }
  do_dq = false;
  if (std::strcmp(s, "all") == 0)
  {
    do_pos = do_vel = do_tau = true;
    return true;
  }
  do_pos = do_vel = do_tau = false;
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
  if (std::strcmp(s, "torque") == 0)
  {
    do_tau = true;
    return true;
  }
  if (std::strcmp(s, "dq") == 0)
  {
    do_dq = true;
    return true;
  }
  return false;
}

bool parse_ctrl_mode_arg(const char *s, uint32_t &mode, bool &is_auto)
{
  if (s == nullptr)
  {
    return false;
  }
  is_auto = false;
  if (std::strcmp(s, "auto") == 0)
  {
    is_auto = true;
    mode = biped_hw::kDmMitMode;
    return true;
  }
  if (std::strcmp(s, "mit") == 0)
  {
    mode = biped_hw::kDmMitMode;
    return true;
  }
  if (std::strcmp(s, "pos_vel") == 0 || std::strcmp(s, "posvel") == 0 ||
      std::strcmp(s, "pos-vel") == 0)
  {
    mode = biped_hw::kDmPosVelMode;
    return true;
  }
  if (std::strcmp(s, "vel") == 0)
  {
    mode = biped_hw::kDmVelMode;
    return true;
  }
  if (std::strcmp(s, "pos_force") == 0 || std::strcmp(s, "posforce") == 0 ||
      std::strcmp(s, "pos-force") == 0 || std::strcmp(s, "force_pos") == 0 ||
      std::strcmp(s, "force-pos") == 0)
  {
    mode = biped_hw::kDmPosForceMode;
    return true;
  }
  return false;
}

uint32_t native_mode_for_stage(StageKind kind)
{
  switch (kind)
  {
  case StageKind::Pos:
    return biped_hw::kDmPosVelMode;
  case StageKind::Vel:
  case StageKind::Dq:
    return biped_hw::kDmVelMode;
  case StageKind::Torque:
    return biped_hw::kDmPosForceMode;
  }
  return biped_hw::kDmMitMode;
}

bool stage_ok_for_mode(StageKind kind, uint32_t mode)
{
  if (mode == biped_hw::kDmMitMode)
  {
    return true;
  }
  if (mode == biped_hw::kDmPosVelMode)
  {
    return kind == StageKind::Pos || kind == StageKind::Vel;
  }
  if (mode == biped_hw::kDmVelMode)
  {
    return kind == StageKind::Vel || kind == StageKind::Dq;
  }
  if (mode == biped_hw::kDmPosForceMode)
  {
    return kind == StageKind::Pos || kind == StageKind::Torque;
  }
  return false;
}

const char *stage_kind_name(StageKind kind)
{
  switch (kind)
  {
  case StageKind::Pos:
    return "position";
  case StageKind::Vel:
    return "velocity";
  case StageKind::Dq:
    return "dq";
  case StageKind::Torque:
    return "torque";
  }
  return "?";
}

bool write_ctrl_mode_and_enable(biped_hw::UsbCanAdapter &adapter, biped_hw::FrameDemux &demux,
                                uint8_t esc_id, uint32_t want, const biped_hw::LimitParam &,
                                CtrlModeState &cm)
{
  biped_hw::DmMitEnableStatus st{};
  const auto err = biped_hw::dm_enable_mode(adapter, demux, esc_id, want, st);
  if (!cm.before_ok)
  {
    cm.before_ok = st.ctrl_before_ok;
    cm.before = st.ctrl_before;
  }
  cm.write_attempted = true;
  cm.wrote_ack = st.wrote_ack;
  cm.after_ok = st.ctrl_after_ok;
  cm.after = st.ctrl_after;
  cm.end_ok = st.ctrl_after_ok;
  cm.end = st.ctrl_after;
  g_slog.out("CTRL_MODE RID10 before write: %s%u (%s)  want %u (%s)\n",
             st.ctrl_before_ok ? "" : "FAIL ", static_cast<unsigned>(st.ctrl_before),
             mode_ok_str(st.ctrl_before_ok, st.ctrl_before), static_cast<unsigned>(want),
             biped_hw::dm_ctrl_mode_name(want));
  g_slog.out("CTRL_MODE RID10 write %s via dm_enable_mode: %s  after: %s%u (%s)\n",
             biped_hw::dm_ctrl_mode_name(want), st.wrote_ack ? "ack" : "NO-ACK",
             st.ctrl_after_ok ? "" : "FAIL ", static_cast<unsigned>(st.ctrl_after),
             mode_ok_str(st.ctrl_after_ok, st.ctrl_after));
  if (err != biped_hw::DmMitEnableErr::Ok)
  {
    if (err == biped_hw::DmMitEnableErr::EnableTx)
    {
      g_slog.err("enable send failed\n");
    }
    else
    {
      g_slog.err(
          "REFUSE enable: CTRL_MODE %s%u (%s), need %u (%s). Command frames for this "
          "mode would be ignored (feedback may still reply).\n",
          st.ctrl_after_ok ? "" : "FAIL ", static_cast<unsigned>(st.ctrl_after),
          mode_ok_str(st.ctrl_after_ok, st.ctrl_after), static_cast<unsigned>(want),
          biped_hw::dm_ctrl_mode_name(want));
    }
    return false;
  }
  g_slog.out("enable id=%u sent (0xFC)  CTRL_MODE=%s\n", static_cast<unsigned>(esc_id),
             biped_hw::dm_ctrl_mode_name(want));
  usleep(kEnableSettleUs);
  adapter.drain();
  return true;
}

void pack_q0_probe(uint8_t cmd[8], uint32_t ctrl_mode, uint8_t esc_id, uint32_t &tx_id,
                   const biped_hw::LimitParam &lim)
{
  tx_id = static_cast<uint32_t>(esc_id);
  if (ctrl_mode == biped_hw::kDmVelMode)
  {
    biped_hw::pack_vel(cmd, 0.f);
    tx_id = biped_hw::dm_cmd_can_id(biped_hw::kDmVelMode, esc_id);
  }
  else if (ctrl_mode == biped_hw::kDmPosVelMode || ctrl_mode == biped_hw::kDmPosForceMode)
  {
    biped_hw::pack_enable(cmd);
  }
  else
  {
    biped_hw::pack_mit(cmd, 0.f, 0.f, 0.f, 0.f, 0.f, lim);
  }
}

struct Options
{
  int period_ms = 10;
  uint8_t esc_id = kCanId;
  std::string port = biped_hw::kDefaultPortLeft;
  std::string yaml_path;
  bool handshake = false;
  bool self_test = false;
  bool verbose = false;
  bool print_tx = false;
  bool dump_rids = false;
  bool set_zero = false;
  bool home = false;
  uint32_t can_bitrate = biped_hw::UsbCanAdapter::kDefaultCanBitrate;
  bool stage_set = false;
  bool port_set = false;
  bool do_pos = true;
  bool do_vel = true;
  bool do_tau = true;
  bool do_dq = false;
  float pos_amp = 0.05f;
  float vel_amp = 0.5f;
  float vel_stroke = kVelStrokeDefault;
  float tau_amp = 0.5f;
  float kp_pos = 80.f;
  float kd_pos = 1.f;
  float kp_vel = 80.f;
  float kd_vel = 1.f;
  bool kp_vel_set = false;
  bool kd_vel_set = false;
  double duration_pos = 5.0;
  double duration_vel = 8.0;
  double duration_tau = 3.0;
  float abort_tau = 0.f;  // 0 = motor TMAX after RID
  float tau_ff = 0.f;
  float tau_ff_user = 0.f;
  bool tau_ff_set = false;
  uint32_t ctrl_mode = biped_hw::kDmMitMode;
  bool ctrl_mode_auto = false;
  float i_ratio = -1.f;
  bool i_ratio_set = false;
};

bool parse_args(int argc, char **argv, Options &opt, std::string &err)
{
  for (int i = 1; i < argc; ++i)
  {
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
    else if (std::strcmp(argv[i], "--can-id") == 0)
    {
      const char *v = need("--can-id");
      if (v == nullptr)
      {
        return false;
      }
      char *end = nullptr;
      const unsigned long id = std::strtoul(v, &end, 10);
      if (end == v || end == nullptr || *end != '\0' || id < 1 || id > 127)
      {
        err = "--can-id is decimal 1–127 (12, not 0x0C)";
        return false;
      }
      opt.esc_id = static_cast<uint8_t>(id);
    }
    else if (std::strcmp(argv[i], "--port") == 0)
    {
      const char *v = need("--port");
      if (v == nullptr)
      {
        return false;
      }
      opt.port = v;
      opt.port_set = true;
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
    else if (std::strcmp(argv[i], "--stage") == 0)
    {
      const char *v = need("--stage");
      if (v == nullptr || !parse_stage(v, opt.do_pos, opt.do_vel, opt.do_tau, opt.do_dq))
      {
        err = "--stage must be all|pos|vel|torque|dq";
        return false;
      }
      opt.stage_set = true;
    }
    else if (std::strcmp(argv[i], "--pos-amp") == 0)
    {
      const char *v = need("--pos-amp");
      if (v == nullptr)
      {
        return false;
      }
      opt.pos_amp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-amp") == 0)
    {
      const char *v = need("--vel-amp");
      if (v == nullptr)
      {
        return false;
      }
      opt.vel_amp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--vel-stroke") == 0)
    {
      const char *v = need("--vel-stroke");
      if (v == nullptr)
      {
        return false;
      }
      opt.vel_stroke = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--tau-amp") == 0)
    {
      const char *v = need("--tau-amp");
      if (v == nullptr)
      {
        return false;
      }
      opt.tau_amp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kp-pos") == 0)
    {
      const char *v = need("--kp-pos");
      if (v == nullptr)
      {
        return false;
      }
      opt.kp_pos = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kd-pos") == 0)
    {
      const char *v = need("--kd-pos");
      if (v == nullptr)
      {
        return false;
      }
      opt.kd_pos = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--kp-vel") == 0)
    {
      const char *v = need("--kp-vel");
      if (v == nullptr)
      {
        return false;
      }
      opt.kp_vel = std::strtof(v, nullptr);
      opt.kp_vel_set = true;
    }
    else if (std::strcmp(argv[i], "--kd-vel") == 0)
    {
      const char *v = need("--kd-vel");
      if (v == nullptr)
      {
        return false;
      }
      opt.kd_vel = std::strtof(v, nullptr);
      opt.kd_vel_set = true;
    }
    else if (std::strcmp(argv[i], "--duration-pos") == 0)
    {
      const char *v = need("--duration-pos");
      if (v == nullptr)
      {
        return false;
      }
      opt.duration_pos = std::atof(v);
    }
    else if (std::strcmp(argv[i], "--duration-vel") == 0)
    {
      const char *v = need("--duration-vel");
      if (v == nullptr)
      {
        return false;
      }
      opt.duration_vel = std::atof(v);
    }
    else if (std::strcmp(argv[i], "--duration-tau") == 0)
    {
      const char *v = need("--duration-tau");
      if (v == nullptr)
      {
        return false;
      }
      opt.duration_tau = std::atof(v);
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
    else if (std::strcmp(argv[i], "--tau-ff") == 0)
    {
      const char *v = need("--tau-ff");
      if (v == nullptr)
      {
        return false;
      }
      opt.tau_ff = std::strtof(v, nullptr);
      opt.tau_ff_user = opt.tau_ff;
      opt.tau_ff_set = true;
    }
    else if (std::strcmp(argv[i], "--ctrl-mode") == 0 || std::strcmp(argv[i], "--mode") == 0)
    {
      const char *v = need(argv[i]);
      if (v == nullptr || !parse_ctrl_mode_arg(v, opt.ctrl_mode, opt.ctrl_mode_auto))
      {
        err = "--ctrl-mode must be mit|pos_vel|vel|pos_force|auto";
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--i-ratio") == 0)
    {
      const char *v = need("--i-ratio");
      if (v == nullptr)
      {
        return false;
      }
      opt.i_ratio = std::strtof(v, nullptr);
      opt.i_ratio_set = true;
    }
    else if (std::strcmp(argv[i], "--dump-rids") == 0)
    {
      opt.dump_rids = true;
    }
    else if (std::strcmp(argv[i], "--set-zero") == 0)
    {
      opt.set_zero = true;
    }
    else if (std::strcmp(argv[i], "--home") == 0)
    {
      opt.home = true;
    }
    else if (std::strcmp(argv[i], "--can-baud") == 0)
    {
      const char *v = need("--can-baud");
      if (v == nullptr)
      {
        return false;
      }
      opt.can_bitrate = static_cast<uint32_t>(std::strtoul(v, nullptr, 10));
    }
    else if (std::strcmp(argv[i], "--handshake") == 0)
    {
      opt.handshake = true;
    }
    else if (std::strcmp(argv[i], "--self-test") == 0)
    {
      opt.self_test = true;
    }
    else if (std::strcmp(argv[i], "--verbose") == 0)
    {
      opt.verbose = true;
    }
    else if (std::strcmp(argv[i], "--print-tx") == 0)
    {
      opt.print_tx = true;
    }
    else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      usage();
      err = "help";
      return false;
    }
    else
    {
      err = std::string("unknown argument: ") + argv[i] + " (try --help)";
      return false;
    }
  }
  if (opt.set_zero && opt.home)
  {
    err = "refuse: --set-zero cannot be combined with --home";
    return false;
  }
  if (opt.set_zero && opt.stage_set)
  {
    err = "refuse: --set-zero cannot be combined with --stage";
    return false;
  }
  if (opt.home && opt.stage_set)
  {
    err = "refuse: --home cannot be combined with --stage";
    return false;
  }
  if ((opt.set_zero || opt.home) && !opt.port_set)
  {
    err = "--set-zero/--home require --port";
    return false;
  }
  return true;
}

int run(Options opt, int argc, char **argv);

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
    g_slog.err("%s\n", err.c_str());
    return 2;
  }
  return run(opt, argc, argv);
}

namespace
{

int run(Options opt, int argc, char **argv)
{
  int period_ms = opt.period_ms;
  uint8_t esc_id = opt.esc_id;
  std::string port = opt.port;
  std::string yaml_path = opt.yaml_path;
  bool handshake = opt.handshake;
  bool self_test = opt.self_test;
  bool verbose = opt.verbose;
  bool print_tx = opt.print_tx;
  bool dump_rids = opt.dump_rids;
  bool set_zero = opt.set_zero;
  bool home = opt.home;
  uint32_t can_bitrate = opt.can_bitrate;
  bool stage_set = opt.stage_set;
  bool port_set = opt.port_set;
  bool do_pos = opt.do_pos;
  bool do_vel = opt.do_vel;
  bool do_tau = opt.do_tau;
  bool do_dq = opt.do_dq;
  float pos_amp = opt.pos_amp;
  float vel_amp = opt.vel_amp;
  float vel_stroke = opt.vel_stroke;
  float tau_amp = opt.tau_amp;
  float kp_pos = opt.kp_pos;
  float kd_pos = opt.kd_pos;
  float kp_vel = opt.kp_vel;
  float kd_vel = opt.kd_vel;
  bool kp_vel_set = opt.kp_vel_set;
  bool kd_vel_set = opt.kd_vel_set;
  double duration_pos = opt.duration_pos;
  double duration_vel = opt.duration_vel;
  double duration_tau = opt.duration_tau;
  float abort_tau = opt.abort_tau;
  float tau_ff = opt.tau_ff;
  float tau_ff_user = opt.tau_ff_user;
  bool tau_ff_set = opt.tau_ff_set;
  uint32_t ctrl_mode = opt.ctrl_mode;
  bool ctrl_mode_auto = opt.ctrl_mode_auto;
  float i_ratio = opt.i_ratio;
  bool i_ratio_set = opt.i_ratio_set;
  CtrlModeState cm;
  std::vector<StageResult> results;
  float q0 = 0.f;
  bool have_q0 = false;
  float tau_cap = 0.f;

  SessionLogGuard slog_guard;
  (void)g_slog.open();
  g_slog.out("cmdline:");
  for (int ai = 0; ai < argc; ++ai)
  {
    g_slog.out(" %s", argv[ai]);
  }
  g_slog.out("\n");

  if (!biped_hw::dm_codec_self_check())
  {
    g_slog.err("dm_codec self-check failed\n");
    return 1;
  }
  if (!biped_hw::metrics_self_check())
  {
    g_slog.err("metrics self-check failed\n");
    return 1;
  }
  if (self_test)
  {
    g_slog.out("dm_codec (MIT/POS_VEL/VEL/POS_FORCE) + metrics self-check OK (no hardware)\n");
    return 0;
  }

  const auto *entry = biped_hw::find_by_can_id(esc_id);
  if (entry == nullptr || entry->master_id != biped_hw::kMasterId ||
      !biped_hw::is_damiao_mit(entry->model))
  {
    g_slog.err(
        "motor map mismatch for CAN ID %u (need mapped Damiao MIT, Master 2)\n",
        static_cast<unsigned>(esc_id));
    return 1;
  }

  if (!yaml_path.empty() && !port_set)
  {
    const std::string yport = left_port_from_yaml(yaml_path);
    if (yport.empty())
    {
      g_slog.err("yaml overlay missing buses.left.port: %s\n",
                   yaml_path.c_str());
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
  if (tau_amp < 0.f)
  {
    tau_amp = -tau_amp;
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
  if (duration_tau < 0.5)
  {
    duration_tau = 0.5;
  }
  if (kp_pos > kKpPosMax)
  {
    g_slog.out("WARN: clamping kp-pos %.1f -> %.1f (Damiao MIT kp max 500)\n",
                static_cast<double>(kp_pos), static_cast<double>(kKpPosMax));
    kp_pos = kKpPosMax;
  }
  if (kp_vel > kKpPosMax)
  {
    g_slog.out("WARN: clamping kp-vel %.1f -> %.1f (same MIT kp cap as kp-pos)\n",
                static_cast<double>(kp_vel), static_cast<double>(kKpPosMax));
    kp_vel = kKpPosMax;
  }
  if (kp_pos < 0.f)
  {
    kp_pos = 0.f;
  }
  if (kp_vel < 0.f)
  {
    kp_vel = 0.f;
  }
  if (kd_pos > kKdMitMax)
  {
    g_slog.out("WARN: clamping kd-pos %.2f -> %.2f (Damiao MIT kd pack 0-5)\n",
                static_cast<double>(kd_pos), static_cast<double>(kKdMitMax));
    kd_pos = kKdMitMax;
  }
  if (kd_vel > kKdMitMax)
  {
    g_slog.out("WARN: clamping kd-vel %.2f -> %.2f (Damiao MIT kd pack 0-5)\n",
                static_cast<double>(kd_vel), static_cast<double>(kKdMitMax));
    kd_vel = kKdMitMax;
  }
  if (kd_pos < 0.f)
  {
    kd_pos = 0.f;
  }
  if (kd_vel < 0.f)
  {
    kd_vel = 0.f;
  }

  const float vel_kp = kp_vel_set ? kp_vel : kp_pos;
  const float vel_kd = kd_vel_set ? kd_vel : kd_pos;

  if (set_zero)
  {
    g_slog.out(
        "\n"
        "**********************************************************************\n"
        "* WARN: --set-zero WRITES firmware zero at current shaft position.  *\n"
        "* MIT 0xFE. No 0xFC. Irreversible-ish. Shaft must be at origin.     *\n"
        "**********************************************************************\n"
        "port %s  CAN ID %u (%s %s)\n",
        port.c_str(), static_cast<unsigned>(esc_id),
        entry != nullptr ? biped_hw::to_string(entry->model) : "unmapped",
        entry != nullptr ? biped_hw::to_string(entry->joint) : "probe");
  }
  else if (home)
  {
    g_slog.out(
        "\n"
        "**********************************************************************\n"
        "* --home: enable MIT, dm_home_to_zero toward firmware 0, then 0xFD. *\n"
        "* Not 0xFE. Abort if |tau_fb| > %.1f Nm.                            *\n"
        "**********************************************************************\n"
        "port %s  CAN ID %u (%s %s)\n",
        static_cast<double>(abort_tau), port.c_str(), static_cast<unsigned>(esc_id),
        entry != nullptr ? biped_hw::to_string(entry->model) : "unmapped",
        entry != nullptr ? biped_hw::to_string(entry->joint) : "probe");
  }
  else
  {
    g_slog.out(
        "\n"
        "**********************************************************************\n"
        "* WARNING: Damiao (DM10422 peak ~400 Nm). Keep clear of joint.       *\n"
        "* P2 uses SMALL amplitudes only. Keep clear of the joint.            *\n"
        "* Abort if |tau_fb| > %.1f Nm. Disable on exit / SIGINT.             *\n"
        "**********************************************************************\n"
        "port %s  CAN ID %u (%s %s)  period=%dms  ctrl_mode=%s%s\n"
        "stages: pos=%d vel=%d torque=%d dq=%d\n"
        "pos_amp=%.3f rad  vel_amp=%.2f rad/s  vel_stroke=%.3f rad  tau_amp=%.2f Nm  "
        "tau_ff=%.3f Nm\n"
        "kp_pos=%.1f kd_pos=%.2f  (MIT tau_est~kp*pos_amp+tau_ff=%.2f Nm)\n"
        "vel MIT/POS_VEL: kp=%.1f kd=%.2f (default=pos gains)  sinusoidal track, stroke=±%.3f rad\n"
        "dq MIT / VEL: kp=0 kd=%.2f  dq_des=+%.3f  stroke=±%.3f then 0xFD\n"
        "MIT formula: tau=kp*(q_des-q)+kd*(dq_des-dq)+t_ff ; kp is Nm/rad, NOT *TMAX/PMAX.\n",
        static_cast<double>(abort_tau), port.c_str(), static_cast<unsigned>(esc_id),
        entry != nullptr ? biped_hw::to_string(entry->model) : "unmapped",
        entry != nullptr ? biped_hw::to_string(entry->joint) : "probe", period_ms,
        ctrl_mode_auto ? "auto" : biped_hw::dm_ctrl_mode_name(ctrl_mode),
        ctrl_mode_auto ? " (pos=POS_VEL vel/dq=VEL torque=POS_FORCE)" : "",
        do_pos ? 1 : 0, do_vel ? 1 : 0, do_tau ? 1 : 0, do_dq ? 1 : 0,
        static_cast<double>(pos_amp), static_cast<double>(vel_amp),
        static_cast<double>(vel_stroke), static_cast<double>(tau_amp),
        static_cast<double>(tau_ff), static_cast<double>(kp_pos),
        static_cast<double>(kd_pos), static_cast<double>(kp_pos * pos_amp + tau_ff),
        static_cast<double>(vel_kp), static_cast<double>(vel_kd),
        static_cast<double>(vel_stroke), static_cast<double>(vel_kd),
        static_cast<double>(vel_amp), static_cast<double>(vel_stroke));
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  biped_hw::UsbCanAdapter adapter(port);
  if (!adapter.set_can_bitrate(can_bitrate))
  {
    g_slog.err("unsupported --can-baud %u (Damiao USB2CAN default is 1000000)\n",
               can_bitrate);
    print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
    return 2;
  }
  if (!adapter.open())
  {
    g_slog.err("open %s failed: %s\n", port.c_str(), adapter.last_error().c_str());
    print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
    return 1;
  }
  {
    const auto &ini = adapter.init_stats();
    if (ini.handshake_sent)
    {
      g_slog.out(
          "opened %s; USB2CAN wait_ms=%u CAN baud %u bps cmd=%02X %02X %02X %02X %02X "
          "handshake=%02X %02X %02X %02X; usb_reply_bytes=%llu last_status_CMD=0x%02X "
          "tx=%s\n",
          port.c_str(), ini.wait_ms, adapter.can_bitrate(),
          static_cast<unsigned>(ini.baud_cmd[0]), static_cast<unsigned>(ini.baud_cmd[1]),
          static_cast<unsigned>(ini.baud_cmd[2]), static_cast<unsigned>(ini.baud_cmd[3]),
          static_cast<unsigned>(ini.baud_cmd[4]),
          static_cast<unsigned>(ini.handshake_prefix[0]),
          static_cast<unsigned>(ini.handshake_prefix[1]),
          static_cast<unsigned>(ini.handshake_prefix[2]),
          static_cast<unsigned>(ini.handshake_prefix[3]),
          static_cast<unsigned long long>(ini.usb_reply_bytes),
          static_cast<unsigned>(ini.last_status_cmd), ini.control_tx_log);
    }
    else
    {
      g_slog.out(
          "opened %s; USB2CAN wait_ms=%u CAN baud %u bps cmd=%02X %02X %02X %02X %02X "
          "handshake=off; usb_reply_bytes=%llu last_status_CMD=0x%02X tx=%s\n",
          port.c_str(), ini.wait_ms, adapter.can_bitrate(),
          static_cast<unsigned>(ini.baud_cmd[0]), static_cast<unsigned>(ini.baud_cmd[1]),
          static_cast<unsigned>(ini.baud_cmd[2]), static_cast<unsigned>(ini.baud_cmd[3]),
          static_cast<unsigned>(ini.baud_cmd[4]),
          static_cast<unsigned long long>(ini.usb_reply_bytes),
          static_cast<unsigned>(ini.last_status_cmd), ini.control_tx_log);
    }
    if (ini.usb_reply_bytes == 0)
    {
      g_slog.out(
          "WARN: USB2CAN silent after dtr=0 + wait_ms=%u + baud (no USB RX). "
          "拔插后不要开上位机; unplug/replug ACM if this repeats.\n",
          ini.wait_ms);
    }
  }

  DisableGuard guard;
  guard.arm(adapter, esc_id);
  g_esc_id.store(esc_id);
  g_exit_adapter = &adapter;
  std::atexit(atexit_disable);

  if (handshake)
  {
    if (!adapter.handshake())
    {
      g_slog.err("handshake send failed\n");
      print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
      return 1;
    }
    usleep(20000);
    adapter.drain();
    g_slog.out("handshake sent (CMD 0x02)\n");
  }

  if (set_zero)
  {
    const bool ok = biped_hw::dm_send_set_zero(adapter, esc_id);
    if (!ok)
    {
      g_slog.err("set-zero id=%u TX failed (0xFE)\n", static_cast<unsigned>(esc_id));
      guard.disable_now();
      g_exit_adapter = nullptr;
      adapter.close();
      return 1;
    }
    g_slog.out("set-zero id=%u sent (0xFE) data=FF FF FF FF FF FF FF FE\n",
               static_cast<unsigned>(esc_id));
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return 0;
  }

  const biped_hw::LimitParam lim = biped_hw::default_limit(entry->model);
  const char *lim_src = "table";
  biped_hw::FrameDemux demux;
  adapter.drain();
  adapter.reset_rx_stats();
  if (abort_tau <= 0.f)
  {
    abort_tau = lim.tau_max > 0.1f ? lim.tau_max : biped_hw::peak_tau_nm(esc_id);
  }

  if (dump_rids)
  {
    dump_diag_rids(adapter, demux, esc_id, /*print_xact=*/true);
    print_usbcan_rx_stats(adapter, "after dump-rids");
  }
  if (dump_rids && !stage_set && !home)
  {
    g_slog.out("--dump-rids done (no enable; pass --stage or --home to continue).\n");
    refresh_ctrl_mode_end(adapter, demux, esc_id, cm);
    print_usbcan_rx_stats(adapter, "end");
    print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return 0;
  }
  if (dump_rids && stage_set)
  {
    g_slog.out("--dump-rids: hex dump done, continuing because --stage is set.\n");
  }

  if (home)
  {
    if (!write_ctrl_mode_and_enable(adapter, demux, esc_id, biped_hw::kDmMitMode, lim, cm))
    {
      refresh_ctrl_mode_end(adapter, demux, esc_id, cm);
      print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
      guard.disable_now();
      g_exit_adapter = nullptr;
      adapter.close();
      return 1;
    }
    biped_hw::DmHomeStatus hs{};
    const auto hr = biped_hw::dm_home_to_zero(
        adapter, demux, esc_id, lim, abort_tau, biped_hw::kDmHomeTimeoutS,
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
    refresh_ctrl_mode_end(adapter, demux, esc_id, cm);
    print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return biped_hw::dm_home_ok(hr) ? 0 : 1;
  }

  // Clamp user amplitudes to safe fractions of the table limits.
  // --stage dq matches Damiao 上位机: dq_des up to VMAX (e.g. 5), not 5% VMAX.
  const float pos_cap = lim.q_max * kLimitFracPos;
  const float vel_cap = do_dq ? lim.dq_max : (lim.dq_max * kLimitFracVel);
  tau_cap = lim.tau_max * kLimitFracTau;
  if (pos_amp > pos_cap)
  {
    g_slog.out("WARN: clamping pos_amp %.4f -> %.4f (5%% PMAX)\n",
                static_cast<double>(pos_amp), static_cast<double>(pos_cap));
    pos_amp = pos_cap;
  }
  if (vel_amp > vel_cap)
  {
    g_slog.out("WARN: clamping vel_amp %.2f -> %.2f (%s)\n",
                static_cast<double>(vel_amp), static_cast<double>(vel_cap),
                do_dq ? "VMAX" : "5% VMAX");
    vel_amp = vel_cap;
  }
  if (tau_amp > tau_cap)
  {
    g_slog.out("WARN: clamping tau_amp %.2f -> %.2f (1%% TMAX)\n",
                static_cast<double>(tau_amp), static_cast<double>(tau_cap));
    tau_amp = tau_cap;
  }
  if (tau_ff < 0.f)
  {
    tau_ff = -tau_ff;
  }
  if (tau_ff > tau_cap)
  {
    g_slog.out("WARN: clamping tau_ff %.2f -> %.2f (1%% TMAX)\n",
                static_cast<double>(tau_ff), static_cast<double>(tau_cap));
    tau_ff = tau_cap;
  }

  const float safe_q_margin = std::max(pos_amp * 2.f, lim.q_max * kSafeQMarginFrac);
  {
    const float stroke_cap = do_dq ? lim.q_max : std::min(pos_cap, safe_q_margin * 0.5f);
    if (vel_stroke > stroke_cap)
    {
      g_slog.out("WARN: clamping vel_stroke %.3f -> %.3f (%s)\n",
                  static_cast<double>(vel_stroke), static_cast<double>(stroke_cap),
                  do_dq ? "PMAX" : "min of 5% PMAX, 0.5*safe_q_margin");
      vel_stroke = stroke_cap;
    }
  }
  if (vel_stroke < 0.01f)
  {
    vel_stroke = 0.01f;
  }

  g_slog.out("MIT limits: PMAX=%.4f VMAX=%.4f TMAX=%.4f source=%s\n",
              static_cast<double>(lim.q_max), static_cast<double>(lim.dq_max),
              static_cast<double>(lim.tau_max), lim_src);
  g_slog.out("safe_q_margin=%.4f rad  command caps: pos<=%.4f vel<=%.2f tau<=%.2f\n",
              static_cast<double>(safe_q_margin), static_cast<double>(pos_cap),
              static_cast<double>(vel_cap), static_cast<double>(tau_cap));
  if (i_ratio_set)
  {
    i_ratio = clampf(i_ratio, 0.f, 1.f);
  }
  else
  {
    const float derived = (lim.tau_max > 0.1f) ? (tau_amp / lim.tau_max) : 0.01f;
    i_ratio = clampf(derived, 0.01f, 1.f);
  }
  g_slog.out("POS_FORCE i_ratio=%.4f (%s)\n", static_cast<double>(i_ratio),
             i_ratio_set ? "--i-ratio" : "max(0.01, tau_amp/TMAX)");
  if (do_dq)
  {
    g_slog.out("MIT dq / VEL: kp=0 kd=%.3f  dq_des=+%.3f rad/s  q_des=0  "
               "stroke=±%.3f then 0xFD  t_ff=%.3f  (vel cap=VMAX, not 5%%)\n",
               static_cast<double>(vel_kd), static_cast<double>(vel_amp),
               static_cast<double>(vel_stroke), static_cast<double>(tau_ff));
  }
  else
  {
    g_slog.out("MIT/POS_VEL vel: sinusoidal track  kp=%.1f kd=%.3f  dq_peak=±%.3f rad/s  "
               "stroke=±%.3f rad  t_ff=%.3f\n",
               static_cast<double>(vel_kp), static_cast<double>(vel_kd),
               static_cast<double>(vel_amp), static_cast<double>(vel_stroke),
               static_cast<double>(tau_ff));
  }

  // Command frames are ignored unless CTRL_MODE matches. Enable 0xFC still
  // works in POS_VEL/VEL/POS_FORCE (feedback OK, shaft still if cmds mismatch).
  // CAN mode write takes effect immediately; flash save is NOT required (and is
  // disabled-only, 30 ms, 10k cycle limit). Do not persist.
  auto exit_fail = [&]() -> int {
    refresh_ctrl_mode_end(adapter, demux, esc_id, cm);
    print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);
    guard.disable_now();
    g_exit_adapter = nullptr;
    adapter.close();
    return 1;
  };

  auto probe_q0 = [&](uint32_t mode) -> bool {
    uint8_t cmd[8] = {};
    uint32_t tx_id = 0;
    pack_q0_probe(cmd, mode, esc_id, tx_id, lim);
    const auto q0_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    bool printed_q0_tx = false;
    while (!have_q0 && std::chrono::steady_clock::now() < q0_deadline)
    {
      if (print_tx && !printed_q0_tx)
      {
        print_mode_tx(mode, tx_id, cmd, lim, 0.f, 0.f, 0.f, 0.f, 0.f, /*console=*/true);
        printed_q0_tx = true;
      }
      adapter.sendCan(tx_id, cmd, 8);
      usleep(static_cast<useconds_t>(period_ms * 1000));
      biped_hw::CanRxFrame rx;
      while (adapter.pollCan(rx))
      {
        demux.push(rx);
        biped_hw::DmFeedback fb{};
        if (demux.take_mit(esc_id, lim, fb))
        {
          q0 = fb.q;
          have_q0 = true;
          g_slog.out("q0=%.4f rad (probe %s)  fb.err=%u (%s)\n", static_cast<double>(q0),
                     biped_hw::dm_ctrl_mode_name(mode), static_cast<unsigned>(fb.err),
                     fb.err == 1 ? "enabled" : (fb.err == 0 ? "DISABLED" : "fault"));
          if (fb.err == 0)
          {
            g_slog.out("WARN: feedback ERR=0 after 0xFC — motor still reports disabled.\n");
          }
          else if (fb.err > 1)
          {
            g_slog.err("WARN: feedback ERR=0x%X (fault). See 10422 datasheet LED table.\n",
                       static_cast<unsigned>(fb.err));
          }
          break;
        }
      }
    }
    return have_q0;
  };

  auto hold_q0 = [&](uint32_t mode) {
    uint8_t buf[8] = {};
    uint32_t tx_id = biped_hw::dm_cmd_can_id(mode, esc_id);
    switch (mode)
    {
    case biped_hw::kDmPosVelMode:
      biped_hw::pack_pos_vel(buf, q0, vel_amp);
      break;
    case biped_hw::kDmVelMode:
      biped_hw::pack_vel(buf, 0.f);
      break;
    case biped_hw::kDmPosForceMode:
      biped_hw::pack_pos_force(buf, q0, vel_amp, i_ratio);
      break;
    default:
      biped_hw::pack_mit(buf, 0.f, 0.f, q0, 0.f, 0.f, lim);
      tx_id = esc_id;
      break;
    }
    for (int i = 0; i < 5; ++i)
    {
      adapter.sendCan(tx_id, buf, 8);
      usleep(static_cast<useconds_t>(period_ms * 1000));
      biped_hw::CanRxFrame rx;
      while (adapter.pollCan(rx))
      {
        demux.push(rx);
        biped_hw::DmFeedback fb{};
        (void)demux.take_mit(esc_id, lim, fb);
      }
    }
  };

  bool enabled_ok = false;
  uint32_t enabled_mode = 0;
  auto ensure_mode = [&](uint32_t mode) -> bool {
    if (enabled_ok && enabled_mode == mode && !ctrl_mode_auto)
    {
      return true;
    }
    if (!write_ctrl_mode_and_enable(adapter, demux, esc_id, mode, lim, cm))
    {
      return false;
    }
    enabled_ok = true;
    enabled_mode = mode;
    if (!have_q0)
    {
      if (mode == biped_hw::kDmMitMode)
      {
        biped_hw::DmHomeStatus hs{};
        const auto hr = biped_hw::dm_home_to_zero(
            adapter, demux, esc_id, lim, abort_tau, biped_hw::kDmHomeTimeoutS,
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
          return false;
        }
      }
      else if (!probe_q0(mode))
      {
        g_slog.err("no feedback for q0; aborting\n");
        return false;
      }
    }
    if (mode != biped_hw::kDmMitMode)
    {
      hold_q0(mode);
    }
    return true;
  };

  int rc = 0;

  auto run_one = [&](StageKind kind, double duration_s, float kp, float kd,
                     float stage_tau_ff) -> bool {
    const uint32_t mode = ctrl_mode_auto ? native_mode_for_stage(kind) : ctrl_mode;
    if (!stage_ok_for_mode(kind, mode))
    {
      g_slog.out("skip stage %s: CTRL_MODE=%s does not use this profile\n",
                 stage_kind_name(kind),
                 ctrl_mode_auto ? "auto" : biped_hw::dm_ctrl_mode_name(ctrl_mode));
      return true;
    }
    if (!ensure_mode(mode))
    {
      return false;
    }
    StageResult r;
    const bool ok =
        run_stage(kind, adapter, demux, esc_id, lim, q0, safe_q_margin, abort_tau,
                  duration_s, period_ms, kp, kd, pos_amp, vel_amp, vel_stroke, tau_amp,
                  stage_tau_ff, verbose, print_tx, mode, i_ratio, r);
    results.push_back(r);
    if (!ok)
    {
      rc = 1;
    }
    return true;
  };

  if (do_pos && g_run.load())
  {
    if (!run_one(StageKind::Pos, duration_pos, kp_pos, kd_pos, tau_ff))
    {
      return exit_fail();
    }
  }
  if (do_vel && g_run.load())
  {
    if (!run_one(StageKind::Vel, duration_vel, vel_kp, vel_kd, tau_ff))
    {
      return exit_fail();
    }
  }
  if (do_dq && g_run.load())
  {
    if (!run_one(StageKind::Dq, duration_vel, 0.f, vel_kd, tau_ff))
    {
      return exit_fail();
    }
  }
  if (do_tau && g_run.load())
  {
    if (!run_one(StageKind::Torque, duration_tau, 0.f, kd_pos * 0.5f, 0.f))
    {
      return exit_fail();
    }
  }

  if (!enabled_ok)
  {
    g_slog.out("no compatible stages for this CTRL_MODE; motor not enabled.\n");
  }

  print_summary_table(results);
  refresh_ctrl_mode_end(adapter, demux, esc_id, cm);
  print_end_banner(cm, tau_ff_set, tau_ff_user, tau_ff, tau_cap, q0, have_q0, results);

  guard.disable_now();
  g_exit_adapter = nullptr;
  adapter.close();
  return rc;
}

}  // namespace
