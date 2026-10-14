// biped_hw — p4_state_dump.cpp
// P4 状态转储：默认证 USB-CAN（不使能、不发 MIT）。
// --enable-knees：只 0xFC CAN 6 与 12，MIT 在探测到的 q0 保持，退出 0xFD。
// 可选 MBS 轮询，可选订阅 /imu（独立 executor 线程；不打开 N100 串口）。
// 缺传感器 valid=false，不填假数。
#include "biped_hw/cli.hpp"
#include "biped_hw/dm_codec.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/mbs_codec.hpp"
#include "biped_hw/mbs_host.hpp"
#include "biped_hw/mbs_map.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/robot_state.hpp"
#include "biped_hw/safety.hpp"
#include "biped_hw/serial_port.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace
{

volatile std::sig_atomic_t g_run = 1;

constexpr uint8_t kKneeLeftId = 6;
constexpr uint8_t kKneeRightId = 12;
constexpr int kEnableSettleUs = 100000;
constexpr int kProbeTicks = 20;
constexpr float kHoldKp = 20.f;
constexpr float kHoldKd = 1.f;
constexpr float kAbortTauNm = 15.f;

biped_hw::UsbCanAdapter *g_left = nullptr;
biped_hw::UsbCanAdapter *g_right = nullptr;

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

void on_sigint(int)
{
  g_run = 0;
  biped_hw::safety_estop_trip();
}

struct Options
{
  bool self_test = false;
  bool no_motors = false;
  bool enable_knees = false;
  double duration_s = 1.0;
  int period_ms = 50;
  std::string left_port = biped_hw::kDefaultPortLeft;
  std::string right_port = biped_hw::kDefaultPortRight;
  bool mbs = false;
  std::string mbs_port;
  uint8_t mbs_addr = biped_hw::kMbsBusAddrDefault;
  bool mbs_addr_set = false;
  int mbs_timeout_ms = 8;
  bool mbs_rts_de = true;
  std::string imu_topic;
  bool verbose = false;
};

void usage()
{
  std::printf(
      "Usage: p4_state_dump [--self-test] [--duration SEC] [--period-ms N]\n"
      "                     [--left-port PATH] [--right-port PATH] [--no-motors]\n"
      "                     [--enable-knees] [--mbs] [--mbs-port PATH]\n"
      "                     [--mbs-addr N] [--imu-topic TOPIC] [--verbose]\n"
      "Unify motor / MBS / N100 timestamps into RobotState.\n"
      "Motors: default listen-only pollCan (no 0xFC, no MIT TX). Idle bus → valid=0.\n"
      "     --enable-knees: 0xFC only decimal CAN 6 (left) and 12 (right).\n"
      "     Probe q0 with kp=0, then MIT hold at that q (kp=20 kd=1, dq=0).\n"
      "     Does not spin 5 rad/s. Does not enable IDs 3–5,7–11,13–14.\n"
      "     abort-tau=15 Nm (not a flag; do not raise). Fault: P5 disable-all 0xFD\n"
      "     on map 3-14 (both buses). No --force-limits. Requires both USB-CAN ports.\n"
      "MBS: --mbs polls 8 joint addrs from config/mbs_encoder.yaml\n"
      "     (decimal 4,5,6,7,10,11,12,13) sequential on one BUS-D.\n"
      "     --mbs-addr N still single-polls that address (0..31 decimal or 0x1F).\n"
      "     Hip yaw / ankle roll have no encoder slot. Timeout → valid=false.\n"
      "     Does not program sensor addresses. Port: yaml by-id, not ttyUSB0.\n"
      "IMU: --imu-topic /imu latches latest sensor_msgs/Imu (orientation, gyro,\n"
      "     accel, header stamp). Spins a ROS executor while the dump loop runs.\n"
      "     Does not open N100 serial; start imu_node separately. No message →\n"
      "     imu valid=false (not faked). Launch port is CP2102 by-id, not ttyUSB.\n"
      "--can-id is not a flag here; map CAN IDs are decimal 3–14.\n");
}

bool parse_args(int argc, char **argv, Options &opt, std::string &err)
{
  for (int i = 1; i < argc; ++i)
  {
    auto need = [&](const char *flag) -> const char * {
      return biped_hw::need_arg(i, argc, argv, flag, err);
    };
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      usage();
      std::exit(0);
    }
    else if (std::strcmp(argv[i], "--self-test") == 0)
    {
      opt.self_test = true;
    }
    else if (std::strcmp(argv[i], "--no-motors") == 0)
    {
      opt.no_motors = true;
    }
    else if (std::strcmp(argv[i], "--enable-knees") == 0)
    {
      opt.enable_knees = true;
    }
    else if (std::strcmp(argv[i], "--verbose") == 0)
    {
      opt.verbose = true;
    }
    else if (std::strcmp(argv[i], "--mbs") == 0)
    {
      opt.mbs = true;
    }
    else if (std::strcmp(argv[i], "--duration") == 0)
    {
      const char *v = need("--duration");
      if (v == nullptr)
      {
        return false;
      }
      char *end = nullptr;
      opt.duration_s = std::strtod(v, &end);
      if (end == v || *end != '\0' || opt.duration_s < 0.0)
      {
        err = "--duration must be >= 0";
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--period-ms") == 0)
    {
      const char *v = need("--period-ms");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      const unsigned n = biped_hw::parse_u(v, ok);
      if (!ok || n == 0)
      {
        err = "--period-ms must be > 0";
        return false;
      }
      opt.period_ms = static_cast<int>(n);
    }
    else if (std::strcmp(argv[i], "--left-port") == 0)
    {
      const char *v = need("--left-port");
      if (v == nullptr)
      {
        return false;
      }
      opt.left_port = v;
    }
    else if (std::strcmp(argv[i], "--right-port") == 0)
    {
      const char *v = need("--right-port");
      if (v == nullptr)
      {
        return false;
      }
      opt.right_port = v;
    }
    else if (std::strcmp(argv[i], "--mbs-port") == 0)
    {
      const char *v = need("--mbs-port");
      if (v == nullptr)
      {
        return false;
      }
      opt.mbs_port = v;
      opt.mbs = true;
    }
    else if (std::strcmp(argv[i], "--mbs-addr") == 0)
    {
      const char *v = need("--mbs-addr");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      const unsigned n = biped_hw::parse_u(v, ok);
      if (!ok || n > biped_hw::kMbsBusAddrMask)
      {
        err = "--mbs-addr is 0..31 decimal (or 0x1F); not a motor --can-id";
        return false;
      }
      opt.mbs_addr = static_cast<uint8_t>(n);
      opt.mbs_addr_set = true;
      opt.mbs = true;
    }
    else if (std::strcmp(argv[i], "--imu-topic") == 0)
    {
      const char *v = need("--imu-topic");
      if (v == nullptr)
      {
        return false;
      }
      opt.imu_topic = v;
    }
    else if (std::strcmp(argv[i], "--abort-tau") == 0 ||
             std::strcmp(argv[i], "--force-limits") == 0)
    {
      err = std::string(argv[i]) + " is not a p4 flag (abort-tau is 15; no --force-limits)";
      return false;
    }
    else
    {
      err = std::string("unknown argument: ") + argv[i] + " (try --help)";
      return false;
    }
  }
  if (opt.enable_knees && opt.no_motors)
  {
    err = "--enable-knees conflicts with --no-motors";
    return false;
  }
  return true;
}

void print_state(const biped_hw::RobotState &st, bool verbose)
{
  std::printf(
      "cycle=%llu t_steady_ns=%lld motors=%zu/%zu mbs=%zu/%zu imu=%d\n",
      static_cast<unsigned long long>(st.cycle),
      static_cast<long long>(st.t_cycle_ns), biped_hw::motors_valid_count(st),
      biped_hw::kMotorCount, biped_hw::mbs_valid_count(st),
      biped_hw::kMbsEncoderCount, st.imu.valid ? 1 : 0);
  if (verbose)
  {
    for (const auto &m : st.motors)
    {
      std::printf(
          "  motor id=%u bus=%s valid=%d q=%.4f dq=%.4f tau=%.3f err=%u "
          "age_ms=%.3f t_rx_ns=%lld\n",
          static_cast<unsigned>(m.can_id), biped_hw::to_string(m.bus),
          m.valid ? 1 : 0, static_cast<double>(m.q), static_cast<double>(m.dq),
          static_cast<double>(m.tau), static_cast<unsigned>(m.err), m.rx_age_ms,
          static_cast<long long>(m.t_rx_ns));
    }
  }
  for (const auto &e : st.mbs)
  {
    std::printf(
        "  mbs %s addr=%u valid=%d angle_rad=%.6f raw=0x%05X sign=%d "
        "zero_rad=%.4f age_ms=%.3f t_rx_ns=%lld\n",
        e.name && e.name[0] ? e.name : "-", static_cast<unsigned>(e.addr),
        e.valid ? 1 : 0, e.angle_rad, e.raw, static_cast<int>(e.sign), e.zero_rad,
        e.rx_age_ms, static_cast<long long>(e.t_rx_ns));
  }
  std::printf(
      "  imu valid=%d frame=%s header=%d.%09u qw=%.4f gx=%.4f ax=%.4f "
      "age_ms=%.3f t_recv_ns=%lld\n",
      st.imu.valid ? 1 : 0, st.imu.frame_id[0] ? st.imu.frame_id : "-",
      st.imu.header_sec, st.imu.header_nsec, st.imu.qw, st.imu.gx, st.imu.ax,
      st.imu.recv_age_ms, static_cast<long long>(st.imu.t_recv_ns));
}

void listen_bus(biped_hw::UsbCanAdapter &ad, biped_hw::FrameDemux &demux,
                biped_hw::RobotState &st, biped_hw::BusId bus, biped_hw::SteadyNs t_rx,
                const biped_hw::LimitParam lim_override[16])
{
  if (!ad.is_open())
  {
    return;
  }
  biped_hw::CanRxFrame rx;
  int n = 0;
  while (n < 32 && ad.pollCan(rx))
  {
    demux.push(rx);
    ++n;
  }

  bool wanted[16] = {};
  biped_hw::LimitParam lims[16] = {};
  for (const auto &e : biped_hw::kMotorMap)
  {
    if (e.bus != bus || e.model == biped_hw::MotorType::HA60)
    {
      continue;
    }
    wanted[e.can_id & 0x0F] = true;
    const uint8_t idn = e.can_id & 0x0F;
    lims[idn] = (lim_override != nullptr && lim_override[idn].q_max > 0.1f)
                    ? lim_override[idn]
                    : biped_hw::default_limit(e.model);
  }
  biped_hw::DmFeedback fb{};
  while (demux.take_wanted_mit(wanted, lims, fb))
  {
    (void)biped_hw::apply_motor(st, fb, t_rx);
    wanted[fb.esc_id & 0x0F] = false;
  }

  for (const auto &e : biped_hw::kMotorMap)
  {
    if (e.bus != bus || e.model != biped_hw::MotorType::HA60)
    {
      continue;
    }
    biped_hw::MotorevoFeedback mf{};
    if (!demux.take_motorevo(e.can_id, mf))
    {
      continue;
    }
    biped_hw::DmFeedback as_dm{};
    as_dm.esc_id = e.can_id;
    as_dm.q = mf.q;
    as_dm.dq = mf.dq;
    as_dm.tau = mf.tau;
    as_dm.err = mf.err_code;
    as_dm.rx_can_id = mf.rx_can_id;
    as_dm.dlc = mf.dlc;
    (void)biped_hw::apply_motor(st, as_dm, t_rx);
  }
}

struct KneeHold
{
  uint8_t can_id = 0;
  biped_hw::LimitParam lim{};
  float q0 = 0.f;
  bool have_q = false;
  bool enabled = false;
};

bool enable_knee(KneeHold &k, biped_hw::UsbCanAdapter &ad, biped_hw::FrameDemux &dm)
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

void print_knee_motor(const biped_hw::RobotState &st, uint8_t can_id)
{
  const int idx = biped_hw::motor_index_by_can_id(can_id);
  if (idx < 0)
  {
    return;
  }
  const auto &m = st.motors[static_cast<std::size_t>(idx)];
  std::printf("  motor id=%u valid=%d q=%.4f dq=%.4f tau=%.3f\n",
              static_cast<unsigned>(m.can_id), m.valid ? 1 : 0,
              static_cast<double>(m.q), static_cast<double>(m.dq),
              static_cast<double>(m.tau));
}

void print_knee_mbs(const biped_hw::RobotState &st, uint8_t addr)
{
  const int idx = biped_hw::mbs_index_by_addr(st, addr);
  if (idx < 0)
  {
    return;
  }
  const auto &e = st.mbs[static_cast<std::size_t>(idx)];
  std::printf("  mbs addr=%u valid=%d angle_rad=%.6f raw=0x%05X\n",
              static_cast<unsigned>(e.addr), e.valid ? 1 : 0, e.angle_rad, e.raw);
}

void print_summary(const biped_hw::RobotState &st, bool abort_tau)
{
  std::printf(
      "\n=== p4 cycle summary ===\n"
      "cycles=%llu t_cycle_ns=%lld motors=%zu/%zu mbs=%zu/%zu imu=%d%s\n",
      static_cast<unsigned long long>(st.cycle),
      static_cast<long long>(st.t_cycle_ns), biped_hw::motors_valid_count(st),
      biped_hw::kMotorCount, biped_hw::mbs_valid_count(st),
      biped_hw::kMbsEncoderCount, st.imu.valid ? 1 : 0,
      abort_tau ? " ABORT |tau|>15" : "");
  print_knee_motor(st, kKneeLeftId);
  print_knee_motor(st, kKneeRightId);
  print_knee_mbs(st, kKneeLeftId);
  print_knee_mbs(st, kKneeRightId);
  std::printf(
      "  imu valid=%d header=%d.%09u vs t_cycle_ns=%lld qw=%.4f gx=%.4f ax=%.4f "
      "age_ms=%.3f t_recv_ns=%lld\n",
      st.imu.valid ? 1 : 0, st.imu.header_sec, st.imu.header_nsec,
      static_cast<long long>(st.t_cycle_ns), st.imu.qw, st.imu.gx, st.imu.ax,
      st.imu.recv_age_ms, static_cast<long long>(st.imu.t_recv_ns));
}

int run(Options opt, int argc, char **argv);

}  // namespace

int main(int argc, char **argv)
{
  Options opt;
  std::string err;
  if (!parse_args(argc, argv, opt, err))
  {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }

  if (opt.self_test)
  {
    const int map_rc = biped_hw::mbs_map_self_test();
    if (map_rc != 0)
    {
      std::fprintf(stderr, "mbs_map self-test FAIL step %d\n", map_rc);
      return 1;
    }
    const int mbs_rc = biped_hw::mbs_codec_self_test();
    if (mbs_rc != 0)
    {
      std::fprintf(stderr, "mbs_codec self-test FAIL step %d\n", mbs_rc);
      return 1;
    }
    const int st_rc = biped_hw::robot_state_self_test();
    if (st_rc != 0)
    {
      std::fprintf(stderr, "robot_state self-test FAIL step %d\n", st_rc);
      return 1;
    }
    biped_hw::MbsEncoderConfig cfg;
    std::string yaml_err;
    if (!biped_hw::load_mbs_encoder_config(cfg, yaml_err))
    {
      std::fprintf(stderr, "mbs yaml self-test FAIL: %s\n", yaml_err.c_str());
      return 1;
    }
    if (!cfg.path.empty())
    {
      const uint8_t want[biped_hw::kMbsEncoderCount] = {4, 5, 6, 7, 10, 11, 12, 13};
      for (std::size_t i = 0; i < biped_hw::kMbsEncoderCount; ++i)
      {
        if (cfg.encoders[i].addr != want[i] || cfg.encoders[i].sign != 1)
        {
          std::fprintf(stderr, "mbs yaml map mismatch at slot %zu\n", i);
          return 1;
        }
      }
    }
    std::printf(
        "p4_state_dump self-test OK (no hardware; 8 MBS slots + mapper + codec; "
        "motors not enabled; missing bus → valid=false)\n");
    return 0;
  }
  return run(opt, argc, argv);
}

namespace
{

int run(Options opt, int argc, char **argv)
{
  std::printf(
      "p4_state_dump duration=%.1f period-ms=%d motors=%s mbs=%s imu=%s "
      "(abort-tau=15, no --force-limits)\n",
      opt.duration_s, opt.period_ms,
      opt.no_motors ? "off" : (opt.enable_knees ? "enable-knees(6,12 hold)" : "listen"),
      opt.mbs ? (opt.mbs_addr_set ? "single" : "map8") : "off",
      opt.imu_topic.empty() ? "off" : opt.imu_topic.c_str());

  biped_hw::MbsEncoderConfig mbs_cfg;
  if (opt.mbs)
  {
    std::string yaml_err;
    if (!biped_hw::load_mbs_encoder_config(mbs_cfg, yaml_err))
    {
      std::fprintf(stderr, "%s\n", yaml_err.c_str());
      return 2;
    }
    if (opt.mbs_port.empty())
    {
      opt.mbs_port = mbs_cfg.port;
    }
  }

  biped_hw::RobotState st;
  biped_hw::init_robot_state(st);
  if (opt.mbs)
  {
    biped_hw::overlay_mbs_map(st, mbs_cfg);
  }

  biped_hw::UsbCanAdapter left_ad(opt.left_port);
  biped_hw::UsbCanAdapter right_ad(opt.right_port);
  biped_hw::FrameDemux left_demux;
  biped_hw::FrameDemux right_demux;
  bool left_open = false;
  bool right_open = false;
  if (!opt.no_motors)
  {
    left_open = left_ad.open();
    right_open = right_ad.open();
    std::printf("  left  %s : %s%s\n", opt.left_port.c_str(),
                left_open ? (opt.enable_knees ? "OK" : "OK (listen)") : "FAIL",
                left_open ? "" : (" (" + left_ad.last_error() + ")").c_str());
    std::printf("  right %s : %s%s\n", opt.right_port.c_str(),
                right_open ? (opt.enable_knees ? "OK" : "OK (listen)") : "FAIL",
                right_open ? "" : (" (" + right_ad.last_error() + ")").c_str());
    if (!left_open && !right_open)
    {
      std::printf(
          "  no USB-CAN (idle/missing is OK); motor slots stay valid=0\n");
    }
  }

  std::atexit(atexit_disable);

  KneeHold knee_l{};
  KneeHold knee_r{};
  knee_l.can_id = kKneeLeftId;
  knee_r.can_id = kKneeRightId;
  biped_hw::LimitParam lim_override[16] = {};
  if (opt.enable_knees)
  {
    if (!left_open || !right_open)
    {
      std::fprintf(stderr, "--enable-knees needs both USB-CAN ports open\n");
      return 2;
    }
    g_left = &left_ad;
    g_right = &right_ad;
    biped_hw::safety_bind_buses(&left_ad, &right_ad);
    if (!enable_knee(knee_l, left_ad, left_demux))
    {
      biped_hw::safety_set_reason(biped_hw::SafetyReason::RidFail);
      disable_all_now(true);
      return 1;
    }
    biped_hw::safety_arm();
    usleep(kEnableSettleUs);
    if (!enable_knee(knee_r, right_ad, right_demux))
    {
      biped_hw::safety_set_reason(biped_hw::SafetyReason::RidFail);
      disable_all_now(true);
      return 1;
    }
    usleep(kEnableSettleUs);
    lim_override[kKneeLeftId] = knee_l.lim;
    lim_override[kKneeRightId] = knee_r.lim;
    const bool q_l = biped_hw::dm_probe_q0(left_ad, left_demux, knee_l.can_id, knee_l.lim,
                                          knee_l.q0, kProbeTicks, 40);
    knee_l.have_q = q_l;
    const bool q_r = biped_hw::dm_probe_q0(right_ad, right_demux, knee_r.can_id, knee_r.lim,
                                          knee_r.q0, kProbeTicks, 40);
    knee_r.have_q = q_r;
    std::printf("q0 id%u=%s%.4f  id%u=%s%.4f  (kp=0 after 0xFC; hold kp=%.0f kd=%.0f)\n",
                static_cast<unsigned>(kKneeLeftId), q_l ? "" : "FAIL ",
                static_cast<double>(knee_l.q0), static_cast<unsigned>(kKneeRightId),
                q_r ? "" : "FAIL ", static_cast<double>(knee_r.q0),
                static_cast<double>(kHoldKp), static_cast<double>(kHoldKd));
    if (!q_l || !q_r)
    {
      std::fprintf(stderr, "no q0 after enable; aborting (P5 disable-all 0xFD)\n");
      disable_all_now(true);
      return 1;
    }
  }

  biped_hw::SerialPort mbs_serial(opt.mbs_port, B2500000, 2, biped_hw::kMbsBusBaudDefault);
  bool mbs_open = false;
  if (opt.mbs)
  {
    if (opt.mbs_port.empty())
    {
      std::fprintf(stderr,
                   "MBS on but no port (pass --mbs-port /dev/serial/by-id/... or "
                   "set port: in config/mbs_encoder.yaml; not ttyUSB0)\n");
    }
    else
    {
      mbs_open = mbs_serial.open();
      const std::string drv = biped_hw::tty_driver_name(opt.mbs_port);
      if (drv != "pl2303")
      {
        opt.mbs_rts_de = false;
      }
      if (opt.mbs_addr_set)
      {
        std::printf("  mbs   %s addr=%u (single) : %s%s\n", opt.mbs_port.c_str(),
                    static_cast<unsigned>(opt.mbs_addr), mbs_open ? "OK" : "FAIL",
                    mbs_open ? "" : (" (" + mbs_serial.last_error() + ")").c_str());
      }
      else
      {
        std::printf("  mbs   %s map8 addrs=4,5,6,7,10,11,12,13 : %s%s\n",
                    opt.mbs_port.c_str(), mbs_open ? "OK" : "FAIL",
                    mbs_open ? "" : (" (" + mbs_serial.last_error() + ")").c_str());
      }
    }
  }

  std::mutex imu_mu;
  sensor_msgs::msg::Imu last_imu;
  biped_hw::SteadyNs imu_t_recv = 0;
  bool have_imu = false;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> imu_exec;
  std::thread imu_spin;
  if (!opt.imu_topic.empty())
  {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("p4_state_dump");
    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        opt.imu_topic, 10,
        [&](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(imu_mu);
          last_imu = *msg;
          imu_t_recv = biped_hw::now_steady_ns();
          have_imu = true;
        });
    imu_exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    imu_exec->add_node(node);
    imu_spin = std::thread([&imu_exec]() { imu_exec->spin(); });
    std::printf(
        "  imu   subscribe %s (executor spinning; imu_node owns N100 serial)\n",
        opt.imu_topic.c_str());
  }

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  bool abort_tau = false;
  const auto t0 = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(opt.period_ms);
  while (g_run)
  {
    if (opt.duration_s > 0.0)
    {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed >= opt.duration_s)
      {
        break;
      }
    }
    const auto cycle_start = std::chrono::steady_clock::now();
    ++st.cycle;
    const auto t_rx = biped_hw::now_steady_ns();

    if (opt.enable_knees)
    {
      if (knee_l.enabled && knee_l.have_q)
      {
        biped_hw::dm_send_mit_hold(left_ad, knee_l.can_id, kHoldKp, kHoldKd, knee_l.q0,
                                   knee_l.lim);
      }
      if (knee_r.enabled && knee_r.have_q)
      {
        biped_hw::dm_send_mit_hold(right_ad, knee_r.can_id, kHoldKp, kHoldKd, knee_r.q0,
                                   knee_r.lim);
      }
    }

    listen_bus(left_ad, left_demux, st, biped_hw::BusId::Left, t_rx, lim_override);
    listen_bus(right_ad, right_demux, st, biped_hw::BusId::Right, t_rx, lim_override);

    if (opt.enable_knees)
    {
      const int i6 = biped_hw::motor_index_by_can_id(kKneeLeftId);
      const int i12 = biped_hw::motor_index_by_can_id(kKneeRightId);
      const float tau6 = i6 >= 0 ? st.motors[static_cast<std::size_t>(i6)].tau : 0.f;
      const float tau12 = i12 >= 0 ? st.motors[static_cast<std::size_t>(i12)].tau : 0.f;
      if ((i6 >= 0 && st.motors[static_cast<std::size_t>(i6)].valid &&
           std::fabs(tau6) > kAbortTauNm) ||
          (i12 >= 0 && st.motors[static_cast<std::size_t>(i12)].valid &&
           std::fabs(tau12) > kAbortTauNm))
      {
        std::fprintf(stderr, "ABORT |tau|>%.1f (id6=%.3f id12=%.3f); P5 disable-all 0xFD\n",
                     static_cast<double>(kAbortTauNm), static_cast<double>(tau6),
                     static_cast<double>(tau12));
        biped_hw::safety_set_reason(biped_hw::SafetyReason::HardLimitTau);
        abort_tau = true;
        g_run = 0;
      }
    }

    if (mbs_open)
    {
      auto poll_addr = [&](uint8_t addr) {
        const int idx = biped_hw::mbs_index_by_addr(st, addr);
        if (idx >= 0)
        {
          st.mbs[static_cast<std::size_t>(idx)].valid = false;
        }
        biped_hw::MbsSample sample{};
        if (!biped_hw::mbs_bus_poll_once(mbs_serial, addr, opt.mbs_timeout_ms, opt.mbs_rts_de,
                           sample))
        {
          return;
        }
        (void)biped_hw::apply_mbs(st, addr, sample.angle_rad, sample.total_rad,
                                  sample.angle_raw, sample.has_status,
                                  sample.status.raw, t_rx);
      };
      if (opt.mbs_addr_set)
      {
        poll_addr(opt.mbs_addr);
      }
      else
      {
        for (const auto &e : st.mbs)
        {
          poll_addr(e.addr);
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(imu_mu);
      if (have_imu)
      {
        biped_hw::apply_imu(
            st, imu_t_recv, static_cast<std::int32_t>(last_imu.header.stamp.sec),
            last_imu.header.stamp.nanosec, last_imu.header.frame_id.c_str(),
            last_imu.orientation.w, last_imu.orientation.x, last_imu.orientation.y,
            last_imu.orientation.z, last_imu.angular_velocity.x,
            last_imu.angular_velocity.y, last_imu.angular_velocity.z,
            last_imu.linear_acceleration.x, last_imu.linear_acceleration.y,
            last_imu.linear_acceleration.z);
      }
    }

    biped_hw::stamp_cycle(st, biped_hw::now_steady_ns());
    print_state(st, opt.verbose);

    if (abort_tau)
    {
      break;
    }

    const auto used = std::chrono::steady_clock::now() - cycle_start;
    if (used < period)
    {
      std::this_thread::sleep_for(period - used);
    }
  }

  print_summary(st, abort_tau);
  if (opt.enable_knees)
  {
    disable_all_now(true);
    g_left = nullptr;
    g_right = nullptr;
    biped_hw::safety_bind_buses(nullptr, nullptr);
  }

  if (imu_exec)
  {
    imu_exec->cancel();
    if (imu_spin.joinable())
    {
      imu_spin.join();
    }
    rclcpp::shutdown();
  }
  return abort_tau ? 1 : 0;
}

}  // namespace
