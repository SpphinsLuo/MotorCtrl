// biped_hw — p6_traj_track.cpp
// P6 相位测试：getState → traj.q_des(i,t)（绕探测 q0）→ sendCommand → 退出 0xFD。
// 不回固件零。默认 --ids 9-14（右腿全电机，含 HA60 14）。薄 main，不 include 其它 app。
// 曲线在 quintic_traj，不在本文件里生成。
#include "biped_hw/algo_api.hpp"
#include "biped_hw/cli.hpp"
#include "biped_hw/dm_mit_host.hpp"
#include "biped_hw/frame_demux.hpp"
#include "biped_hw/mbs_codec.hpp"
#include "biped_hw/mbs_map.hpp"
#include "biped_hw/motor_map.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/quintic_traj.hpp"
#include "biped_hw/safety.hpp"
#include "biped_hw/serial_port.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

constexpr float kAbortTauDefault = 15.f;
constexpr float kKpCap = 20.f;
constexpr float kKdCap = 5.f;
constexpr int kEnableSettleUs = 100000;
constexpr int kProbeTicks = 20;

struct Options
{
  bool self_test = false;
  std::vector<uint8_t> ids = {9, 10, 11, 12, 13, 14};
  bool ids_set = false;
  bool leg_left = false;
  std::string left_port = biped_hw::kDefaultPortLeft;
  std::string right_port = biped_hw::kDefaultPortRight;
  std::string port;
  bool port_set = false;
  float amp = biped_hw::kQuinticAmpRad;
  double seg_time = biped_hw::kQuinticSegTime_s;
  double duration_s = 8.0;
  int period_ms = 10;
  float kp = biped_hw::kPidTrackKp;
  float kd = biped_hw::kPidTrackKd;
  float abort_tau = kAbortTauDefault;
  bool skip_motorevo = true;
  bool mbs = false;
  std::string mbs_port;
  bool mbs_rts_de = true;
  std::string imu_topic;
  bool verbose = false;
};

void usage()
{
  std::printf(
      "Usage: p6_traj_track [options]\n"
      "P6 trajectory track: getState → q_des(t) → sendCommand (MIT PD / HA60 servo).\n"
      "  Piecewise rest-to-rest quintic (zero vel/acc at segment ends):\n"
      "  q0 → q0+A → q0 → q0−A → q0, each --seg-time, then repeats.\n"
      "  Same math as motor_ctrl quinticPolynomial (a3=10,a4=-15,a5=6 at T=1).\n"
      "  Tracks about probe q0 (not firmware 0; right-leg ID14 may be ~5 rad).\n"
      "  Damiao inner loop is MIT PD on the packet (q_des, dq_des, kp, kd).\n"
      "  HA60 inner loop is firmware servo (θ_ref=q_des); no extra host PID.\n"
      "  Motor-space traj (each ID tracks its own q). hip_ik is in sendCommand\n"
      "  if a later caller sets hip roll/pitch (IDs 4/5, 10/11).\n"
      "  --ids LIST            decimal CAN IDs, comma-separated. Default 9,10,11,12,13,14\n"
      "                        (one full right leg, all motors, including HA60 14).\n"
      "                        --ids overrides. Does not default ID12 only or 12 motors.\n"
      "  --leg left|right      default right (9-14). left = 3,4,5,6,7,8. --ids overrides.\n"
      "  --left-port PATH      default %s\n"
      "  --right-port PATH     default %s\n"
      "  --port PATH           single-bus alias: right if only right IDs, left if\n"
      "                        only left IDs. Mixed buses: use --left-port/--right-port.\n"
      "  --amp RAD             quintic swing amplitude about probe q0\n"
      "                        (default %.2f rad ≈ ±20 deg); not sine, not firmware 0.\n"
      "  --seg-time SEC        one quintic segment duration (default %.1f s)\n"
      "  --duration SEC        default %.1f\n"
      "  --period-ms N         control period (default %d)\n"
      "  --kp --kd             MIT PD / HA60 pos gains (default %.1f / %.2f; kp<=%.0f)\n"
      "  --abort-tau NM        abort if |tau_fb| exceeds (default %.0f; do not raise)\n"
      "  --skip-motorevo [0|1] default 1 unless HA60 is in the ID list (then 0).\n"
      "                        Default 9-14 includes ID14, so skip-motorevo is off.\n"
      "  --no-skip-motorevo    include HA60 only if also in --ids\n"
      "  --mbs                 poll MBS 8-map into getState (timeout → valid=false)\n"
      "  --mbs-port PATH       BUS-D by-id (yaml default if omitted)\n"
      "  --imu-topic TOPIC     latch sensor_msgs/Imu into getState (start imu_node)\n"
      "  --verbose\n"
      "  --self-test           no hardware: quintic q(0)=q0 q(T)=q0+A, dq ends ~0,\n"
      "                        pack finite; rc=0. Never 0xFC.\n"
      "  --help\n"
      "No firmware-zero home. Exit/SIGINT/abort/drop: P5 safety_disable_all 0xFD.\n"
      "No --force-limits. Overlay motorctrl/install. --can-id is decimal in --ids.\n",
      biped_hw::kDefaultPortLeft, biped_hw::kDefaultPortRight,
      static_cast<double>(biped_hw::kQuinticAmpRad),
      biped_hw::kQuinticSegTime_s, 8.0, 10,
      static_cast<double>(biped_hw::kPidTrackKp),
      static_cast<double>(biped_hw::kPidTrackKd), static_cast<double>(kKpCap),
      static_cast<double>(kAbortTauDefault));
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
    if (end == p || v < 1 || v > 14)
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

bool parse_bool_flag(const char *v, bool &out)
{
  if (v == nullptr)
  {
    out = true;
    return true;
  }
  if (std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0)
  {
    out = true;
    return true;
  }
  if (std::strcmp(v, "0") == 0 || std::strcmp(v, "false") == 0)
  {
    out = false;
    return true;
  }
  return false;
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
    else if (std::strcmp(argv[i], "--verbose") == 0)
    {
      opt.verbose = true;
    }
    else if (std::strcmp(argv[i], "--mbs") == 0)
    {
      opt.mbs = true;
    }
    else if (std::strcmp(argv[i], "--no-skip-motorevo") == 0)
    {
      opt.skip_motorevo = false;
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
        err = "--ids needs decimal CAN IDs (e.g. 12 or 9,10,11,12,13,14)";
        return false;
      }
      opt.ids_set = true;
    }
    else if (std::strcmp(argv[i], "--leg") == 0)
    {
      const char *v = need("--leg");
      if (v == nullptr)
      {
        return false;
      }
      if (std::strcmp(v, "left") == 0)
      {
        opt.leg_left = true;
      }
      else if (std::strcmp(v, "right") == 0)
      {
        opt.leg_left = false;
      }
      else
      {
        err = "--leg must be left or right";
        return false;
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
    else if (std::strcmp(argv[i], "--amp") == 0)
    {
      const char *v = need("--amp");
      if (v == nullptr)
      {
        return false;
      }
      opt.amp = std::strtof(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--seg-time") == 0)
    {
      const char *v = need("--seg-time");
      if (v == nullptr)
      {
        return false;
      }
      opt.seg_time = std::strtod(v, nullptr);
    }
    else if (std::strcmp(argv[i], "--duration") == 0)
    {
      const char *v = need("--duration");
      if (v == nullptr)
      {
        return false;
      }
      opt.duration_s = std::strtod(v, nullptr);
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
    else if (std::strcmp(argv[i], "--abort-tau") == 0)
    {
      const char *v = need("--abort-tau");
      if (v == nullptr)
      {
        return false;
      }
      opt.abort_tau = std::strtof(v, nullptr);
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
    else if (std::strcmp(argv[i], "--imu-topic") == 0)
    {
      const char *v = need("--imu-topic");
      if (v == nullptr)
      {
        return false;
      }
      opt.imu_topic = v;
    }
    else if (std::strcmp(argv[i], "--force-limits") == 0)
    {
      err = "--force-limits is not a p6 flag";
      return false;
    }
    else
    {
      err = std::string("unknown argument: ") + argv[i] + " (try --help)";
      return false;
    }
  }

  if (!opt.ids_set && opt.leg_left)
  {
    opt.ids = {3, 4, 5, 6, 7, 8};
  }

  if (opt.amp <= 0.f || opt.amp > 1.f)
  {
    err = "--amp must be in (0, 1] rad";
    return false;
  }
  if (!(opt.seg_time > 0.0) || opt.seg_time > 30.0)
  {
    err = "--seg-time must be in (0, 30] s";
    return false;
  }
  if (opt.duration_s < 0.0)
  {
    err = "--duration must be >= 0";
    return false;
  }
  if (opt.kp > kKpCap)
  {
    std::printf("WARN: clamping kp %.1f -> %.0f\n", static_cast<double>(opt.kp),
                static_cast<double>(kKpCap));
    opt.kp = kKpCap;
  }
  if (opt.kp < 0.f)
  {
    opt.kp = 0.f;
  }
  if (opt.kd > kKdCap)
  {
    opt.kd = kKdCap;
  }
  if (opt.kd < 0.f)
  {
    opt.kd = 0.f;
  }
  if (opt.abort_tau < 1.f)
  {
    opt.abort_tau = 1.f;
  }
  if (opt.abort_tau > kAbortTauDefault)
  {
    std::printf("WARN: clamping abort-tau %.1f -> %.0f (do not raise)\n",
                static_cast<double>(opt.abort_tau), static_cast<double>(kAbortTauDefault));
    opt.abort_tau = kAbortTauDefault;
  }

  bool listed_ha60 = false;
  bool need_left = false;
  bool need_right = false;
  for (uint8_t id : opt.ids)
  {
    const auto *e = biped_hw::find_by_can_id(id);
    if (e == nullptr)
    {
      err = "--ids contains unmapped CAN ID " + std::to_string(id);
      return false;
    }
    if (e->model == biped_hw::MotorType::HA60)
    {
      listed_ha60 = true;
    }
    if (e->bus == biped_hw::BusId::Left)
    {
      need_left = true;
    }
    else
    {
      need_right = true;
    }
  }
  if (listed_ha60 && opt.skip_motorevo)
  {
    if (!opt.self_test)
    {
      std::printf("NOTE: --ids includes Motorevo HA60; skip_motorevo=0\n");
    }
    opt.skip_motorevo = false;
  }
  if (opt.port_set)
  {
    if (need_left && need_right)
    {
      err = "--port is single-bus; mixed --ids need --left-port and --right-port";
      return false;
    }
    if (need_left)
    {
      opt.left_port = opt.port;
    }
    else
    {
      opt.right_port = opt.port;
    }
  }
  return true;
}

bool selected(const Options &opt, uint8_t id)
{
  const auto *e = biped_hw::find_by_can_id(id);
  if (e == nullptr || !contains_id(opt.ids, id))
  {
    return false;
  }
  if (e->model == biped_hw::MotorType::HA60 && opt.skip_motorevo)
  {
    return false;
  }
  return true;
}

void disable_now(biped_hw::UsbCanAdapter *left, biped_hw::UsbCanAdapter *right, bool print)
{
  const auto r = biped_hw::safety_estop_latched() ? biped_hw::SafetyReason::Estop
                                                  : biped_hw::safety_last_reason();
  biped_hw::safety_disable_all(
      left, right, nullptr, 0,
      r == biped_hw::SafetyReason::None ? biped_hw::SafetyReason::Estop : r, print);
}

bool enable_one(uint8_t id, biped_hw::UsbCanAdapter &ad, biped_hw::FrameDemux &dm,
                biped_hw::LimitParam &lim)
{
  const auto *e = biped_hw::find_by_can_id(id);
  if (e == nullptr)
  {
    return false;
  }
  if (e->model == biped_hw::MotorType::HA60)
  {
    uint8_t dis[8] = {};
    biped_hw::pack_motorevo_disable(dis);
    ad.sendCan(id, dis, 8);
    usleep(10000);
    uint8_t en[8] = {};
    biped_hw::pack_motorevo_enable(en);
    if (!ad.sendCan(id, en, 8))
    {
      std::fprintf(stderr, "enable send failed id=%u\n", static_cast<unsigned>(id));
      return false;
    }
    lim = biped_hw::kLimitHa60;
    std::printf("enable id=%u Motorevo HA60 servo (0xFC)\n", static_cast<unsigned>(id));
    return true;
  }
  biped_hw::DmMitEnableStatus st;
  const bool require_pvt = e->model == biped_hw::MotorType::DM10422;
  const biped_hw::DmMitEnableErr en = biped_hw::dm_enable_mit(ad, dm, id, require_pvt, st);
  std::printf("id=%u RID 21/22/23 PMAX=%s%.4f VMAX=%s%.4f TMAX=%s%.4f\n",
              static_cast<unsigned>(id), st.pmax_ok ? "" : "FAIL ",
              static_cast<double>(st.pmax), st.vmax_ok ? "" : "FAIL ",
              static_cast<double>(st.vmax), st.tmax_ok ? "" : "FAIL ",
              static_cast<double>(st.tmax));
  if (en != biped_hw::DmMitEnableErr::Ok)
  {
    std::fprintf(stderr, "REFUSE enable id=%u err=%u\n", static_cast<unsigned>(id),
                 static_cast<unsigned>(en));
    if (en == biped_hw::DmMitEnableErr::Rid)
    {
      biped_hw::safety_set_reason(biped_hw::SafetyReason::RidFail);
    }
    return false;
  }
  lim = st.lim;
  std::printf("enable id=%u sent (0xFC) CTRL_MODE=%s\n", static_cast<unsigned>(id),
              biped_hw::dm_ctrl_mode_name(st.ctrl_after));
  return true;
}

int run_live(Options opt, int argc, char **argv)
{
  (void)argc;
  (void)argv;
  std::vector<uint8_t> ids;
  bool need_left = false;
  bool need_right = false;
  for (uint8_t id : opt.ids)
  {
    if (!selected(opt, id))
    {
      std::printf("skip id=%u (Motorevo HA60; list in --ids or --no-skip-motorevo)\n",
                  static_cast<unsigned>(id));
      continue;
    }
    ids.push_back(id);
    const auto *e = biped_hw::find_by_can_id(id);
    if (e->bus == biped_hw::BusId::Left)
    {
      need_left = true;
    }
    else
    {
      need_right = true;
    }
  }
  if (ids.empty())
  {
    std::fprintf(stderr, "no IDs selected\n");
    return 2;
  }
  if (ids.size() > 6)
  {
    std::printf("WARN: enabling %zu motors (default is right-leg 9-14)\n", ids.size());
  }

  std::printf(
      "p6_traj_track ids=");
  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    std::printf("%s%u", i ? "," : "", static_cast<unsigned>(ids[i]));
  }
  std::printf(" amp=%.3f seg-time=%.2f s kp=%.1f kd=%.2f duration=%.1f abort-tau=%.0f\n"
              "traj=quintic about probe q0 (no firmware-zero home); inner=MIT PD / HA60 servo\n",
              static_cast<double>(opt.amp), opt.seg_time,
              static_cast<double>(opt.kp), static_cast<double>(opt.kd), opt.duration_s,
              static_cast<double>(opt.abort_tau));

  biped_hw::UsbCanAdapter left_ad(opt.left_port);
  biped_hw::UsbCanAdapter right_ad(opt.right_port);
  biped_hw::FrameDemux left_dm;
  biped_hw::FrameDemux right_dm;
  bool left_open = false;
  bool right_open = false;
  if (need_left)
  {
    left_open = left_ad.open();
    std::printf("  left  %s : %s%s\n", opt.left_port.c_str(), left_open ? "OK" : "FAIL",
                left_open ? "" : (" (" + left_ad.last_error() + ")").c_str());
  }
  if (need_right)
  {
    right_open = right_ad.open();
    std::printf("  right %s : %s%s\n", opt.right_port.c_str(), right_open ? "OK" : "FAIL",
                right_open ? "" : (" (" + right_ad.last_error() + ")").c_str());
  }
  if ((need_left && !left_open) || (need_right && !right_open))
  {
    std::fprintf(stderr, "required USB-CAN port failed; not enabling\n");
    return 2;
  }

  biped_hw::UsbCanAdapter *left_p = left_open ? &left_ad : nullptr;
  biped_hw::UsbCanAdapter *right_p = right_open ? &right_ad : nullptr;
  std::atexit(biped_hw::safety_atexit_disable);
  biped_hw::safety_bind_buses(left_p, right_p);
  biped_hw::safety_install_signals();

  biped_hw::AlgoSources src{};
  src.left = left_p;
  src.right = right_p;
  src.left_demux = &left_dm;
  src.right_demux = &right_dm;

  std::vector<uint8_t> enabled;
  float q0[16] = {};
  bool have_q0[16] = {};
  for (uint8_t id : ids)
  {
    const auto *e = biped_hw::find_by_can_id(id);
    biped_hw::UsbCanAdapter *ad = (e->bus == biped_hw::BusId::Right) ? right_p : left_p;
    biped_hw::FrameDemux *dm = (e->bus == biped_hw::BusId::Right) ? &right_dm : &left_dm;
    biped_hw::LimitParam lim{};
    if (!enable_one(id, *ad, *dm, lim))
    {
      disable_now(left_p, right_p, true);
      return 1;
    }
    src.lims[id & 0x0F] = lim;
    enabled.push_back(id);
    if (enabled.size() == 1)
    {
      biped_hw::safety_arm();
    }
    usleep(kEnableSettleUs);
    float q = 0.f;
    const bool ok = (e->model == biped_hw::MotorType::HA60)
                        ? biped_hw::evo_probe_q0(*ad, *dm, id, q, kProbeTicks, 40)
                        : biped_hw::dm_probe_q0(*ad, *dm, id, lim, q, kProbeTicks, 40);
    have_q0[id & 0x0F] = ok;
    q0[id & 0x0F] = q;
    std::printf("q0 id%u=%s%.4f (track about this; not firmware 0)\n",
                static_cast<unsigned>(id), ok ? "" : "FAIL ", static_cast<double>(q));
    if (!ok)
    {
      std::fprintf(stderr, "no q0 id=%u; P5 disable-all 0xFD\n", static_cast<unsigned>(id));
      biped_hw::safety_set_reason(biped_hw::SafetyReason::Drop);
      disable_now(left_p, right_p, true);
      return 1;
    }
  }

  biped_hw::QuinticSwingTraj traj;
  traj.amp = opt.amp;
  traj.seg_time = opt.seg_time;
  for (uint8_t id : enabled)
  {
    traj.set_q0(static_cast<int>(id), q0[id & 0x0F]);
  }

  biped_hw::MbsEncoderConfig mbs_cfg;
  if (opt.mbs)
  {
    std::string yaml_err;
    if (!biped_hw::load_mbs_encoder_config(mbs_cfg, yaml_err))
    {
      std::fprintf(stderr, "%s\n", yaml_err.c_str());
      disable_now(left_p, right_p, true);
      return 2;
    }
    if (opt.mbs_port.empty())
    {
      opt.mbs_port = mbs_cfg.port;
    }
  }
  biped_hw::SerialPort mbs_serial(opt.mbs_port, B2500000, 2, biped_hw::kMbsBusBaudDefault);
  if (opt.mbs)
  {
    if (!mbs_serial.open())
    {
      std::printf("  mbs   %s : FAIL (%s); slots stay valid=false\n", opt.mbs_port.c_str(),
                  mbs_serial.last_error().c_str());
    }
    else
    {
      if (biped_hw::tty_driver_name(opt.mbs_port) != "pl2303")
      {
        opt.mbs_rts_de = false;
      }
      src.mbs = &mbs_serial;
      src.poll_mbs = true;
      src.mbs_rts_de = opt.mbs_rts_de;
      std::printf("  mbs   %s map8 : OK\n", opt.mbs_port.c_str());
    }
  }

  std::mutex imu_mu;
  biped_hw::ImuFeedback imu_fb;
  bool have_imu = false;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> imu_exec;
  std::thread imu_spin;
  if (!opt.imu_topic.empty())
  {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("p6_traj_track");
    imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        opt.imu_topic, 10, [&](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(imu_mu);
          imu_fb.valid = true;
          imu_fb.t_recv_ns = biped_hw::now_steady_ns();
          imu_fb.header_sec = static_cast<std::int32_t>(msg->header.stamp.sec);
          imu_fb.header_nsec = msg->header.stamp.nanosec;
          imu_fb.qw = msg->orientation.w;
          imu_fb.qx = msg->orientation.x;
          imu_fb.qy = msg->orientation.y;
          imu_fb.qz = msg->orientation.z;
          imu_fb.gx = msg->angular_velocity.x;
          imu_fb.gy = msg->angular_velocity.y;
          imu_fb.gz = msg->angular_velocity.z;
          imu_fb.ax = msg->linear_acceleration.x;
          imu_fb.ay = msg->linear_acceleration.y;
          imu_fb.az = msg->linear_acceleration.z;
          imu_fb.frame_id[0] = '\0';
          std::strncpy(imu_fb.frame_id, msg->header.frame_id.c_str(),
                       sizeof(imu_fb.frame_id) - 1);
          imu_fb.frame_id[sizeof(imu_fb.frame_id) - 1] = '\0';
          have_imu = true;
        });
    imu_exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    imu_exec->add_node(node);
    imu_spin = std::thread([&imu_exec]() { imu_exec->spin(); });
    std::printf("  imu   subscribe %s\n", opt.imu_topic.c_str());
  }

  biped_hw::RobotState st;
  biped_hw::init_robot_state(st);
  if (opt.mbs)
  {
    biped_hw::overlay_mbs_map(st, mbs_cfg);
  }
  biped_hw::SafetyDropWatch drops{};
  const auto t0 = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(opt.period_ms);
  int rc = 0;

  while (!biped_hw::safety_estop_latched())
  {
    const auto cycle_start = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(cycle_start - t0).count();
    if (opt.duration_s > 0.0 && t >= opt.duration_s)
    {
      break;
    }

    {
      std::lock_guard<std::mutex> lock(imu_mu);
      src.imu_latch = have_imu ? &imu_fb : nullptr;
    }
    biped_hw::getState(src, st);

    bool abort = false;
    for (uint8_t id : enabled)
    {
      const int idx = biped_hw::motor_index_by_can_id(id);
      if (idx < 0)
      {
        continue;
      }
      const auto &m = st.motors[static_cast<std::size_t>(idx)];
      const bool rx_ok = m.valid && m.t_rx_ns == st.t_cycle_ns;
      if (biped_hw::safety_note_rx(drops, id, rx_ok))
      {
        std::fprintf(stderr, "ABORT drop id=%u\n", static_cast<unsigned>(id));
        biped_hw::safety_set_reason(biped_hw::SafetyReason::Drop);
        abort = true;
      }
      if (m.valid && std::fabs(m.tau) > opt.abort_tau)
      {
        std::fprintf(stderr, "ABORT id=%u |tau|=%.3f > %.0f\n", static_cast<unsigned>(id),
                     static_cast<double>(std::fabs(m.tau)), static_cast<double>(opt.abort_tau));
        biped_hw::safety_set_reason(biped_hw::SafetyReason::HardLimitTau);
        abort = true;
      }
      if (m.valid && biped_hw::safety_check_q(id, m.q) == biped_hw::SafetyLimitHit::Hard)
      {
        std::fprintf(stderr, "ABORT id=%u hard |q|\n", static_cast<unsigned>(id));
        biped_hw::safety_set_reason(biped_hw::SafetyReason::HardLimitQ);
        abort = true;
      }
    }
    if (abort)
    {
      rc = 1;
      break;
    }

    biped_hw::RobotCommand cmd{};
    for (uint8_t id : enabled)
    {
      const int idx = biped_hw::motor_index_by_can_id(id);
      if (idx < 0 || !have_q0[id & 0x0F])
      {
        continue;
      }
      auto &sp = cmd.motors[static_cast<std::size_t>(idx)];
      sp.active = true;
      sp.kp = opt.kp;
      sp.kd = opt.kd;
      const float t_s = static_cast<float>(t);
      sp.q_des = traj.q_des(static_cast<int>(id), t_s);
      sp.dq_des = traj.dq_des(static_cast<int>(id), t_s);
    }
    const biped_hw::SendResult sent = biped_hw::sendCommand(&src, cmd);
    if (opt.verbose)
    {
      std::printf("cycle=%llu t=%.3f motors=%zu/%zu mbs=%zu/%zu imu=%d packed=%zu\n",
                  static_cast<unsigned long long>(st.cycle), t,
                  biped_hw::motors_valid_count(st), biped_hw::kMotorCount,
                  biped_hw::mbs_valid_count(st), biped_hw::kMbsEncoderCount,
                  st.imu.valid ? 1 : 0, sent.n);
    }
    (void)sent;

    const auto used = std::chrono::steady_clock::now() - cycle_start;
    if (used < period)
    {
      std::this_thread::sleep_for(period - used);
    }
  }

  disable_now(left_p, right_p, true);
  biped_hw::safety_disarm();
  biped_hw::safety_bind_buses(nullptr, nullptr);

  if (imu_exec)
  {
    imu_exec->cancel();
    if (imu_spin.joinable())
    {
      imu_spin.join();
    }
    rclcpp::shutdown();
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
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }
  if (opt.self_test)
  {
    const int rc = biped_hw::algo_api_self_test();
    if (rc != 0)
    {
      std::fprintf(stderr, "p6_traj_track --self-test FAIL step %d\n", rc);
      return rc;
    }
    std::printf(
        "p6_traj_track self-test OK (no hardware, no 0xFC): quintic q(0)=q0 q(T)=q0+A, "
        "dq ends ~0, MIT PD / HA60 servo pack finite, getState missing→valid=false, "
        "hip_ik IDs 10/11\n");
    return 0;
  }
  return run_live(opt, argc, argv);
}
