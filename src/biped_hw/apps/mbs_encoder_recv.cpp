// 金刚 磁编码器数据接收程序。
// MBS 磁编码器 USB-RS485 轮询。默认口来自 config/mbs_encoder.yaml 的 by-id，
// MBS通信协议采用BUS 高速总线（手册 v2.7），不是RS485,但仍通过485接口实现。
// BUS：TX 1 字节 [奇校验|op|addr]，默认地址 0x1F，UART 8N1 固定 2.5 Mbps，
// 回 B0=请求 + S A0 A1 A2 + XOR CRC。协议 R 的 0x64 在 D 上 op=0b11 → 无应答。
// 附加 A=115200 仅接口 R/A；本 SKU 的 A 是温度档，M 是无附加。不打开 ttyACM*。
#include "biped_hw/cli.hpp"
#include "biped_hw/mbs_codec.hpp"
#include "biped_hw/mbs_host.hpp"
#include "biped_hw/mbs_map.hpp"
#include "biped_hw/serial_port.hpp"

#include <algorithm>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits.h>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{

volatile std::sig_atomic_t g_run = 1;

void on_sigint(int)
{
  g_run = 0;
}

bool baud_to_speed(unsigned baud, speed_t &out)
{
  switch (baud)
  {
  case 115200:
    out = B115200;
    return true;
  case 230400:
    out = B230400;
    return true;
  case 460800:
    out = B460800;
    return true;
  case 921600:
    out = B921600;
    return true;
#ifdef B2000000
  case 2000000:
    out = B2000000;
    return true;
#endif
  case 2500000:
    out = B2500000;
    return true;
#ifdef B3000000
  case 3000000:
    out = B3000000;
    return true;
#endif
  default:
    return false;
  }
}

struct Options
{
  std::string port;
  bool port_set = false;
  unsigned baud = biped_hw::kMbsBusBaudDefault;
  bool baud_set = false;
  biped_hw::MbsIface iface = biped_hw::MbsIface::BusD;
  biped_hw::MbsLayout layout{};
  biped_hw::MbsBusLayout bus{};
  double rate_hz = 100.0;
  double duration_s = 0.0;
  int timeout_ms = 8;  // full 6-byte BUS-D reply; 20ms made 100 Hz polls ~50 Hz
  int turnaround_us = -1;  // <0: auto from baud
  bool verbose = false;
  bool self_test = false;
  bool strip_echo = true;
  bool rts_de = false;
  bool rts_de_set = false;
  bool rts_invert = false;
  bool map = false;
  bool addr_set = false;
  bool set_zero = false;
};

void wait_us(int us)
{
  if (us <= 0)
  {
    return;
  }
  // sleep_for(10us) often becomes 1ms+ and holds RS485 DE through the encoder reply.
  if (us >= 500)
  {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
    return;
  }
  const auto until =
      std::chrono::steady_clock::now() + std::chrono::microseconds(us);
  while (std::chrono::steady_clock::now() < until)
  {
  }
}

int default_turnaround_us(unsigned baud)
{
  // 6 bytes @ 2.5 Mbps is ~24 us. Do not insert a 300 us sleep after TX.
  if (baud >= 2000000u)
  {
    return 0;
  }
  const int char_us = baud > 0 ? static_cast<int>((10 * 1000000u + baud / 2) / baud) : 100;
  const int two_chars = char_us * 2;
  return two_chars + 200 < 300 ? 300 : two_chars + 200;
}

void usage()
{
  std::printf(
      "Usage: mbs_encoder_recv [--port PATH] [--baud N] [--iface d|r] [--addr N]\n"
      "                        [--map] [--bits 16|17] [--multi] [--multi-bits 0|8|16]\n"
      "                        [--cmd 0x31|0x64|0x74]   (iface r only)\n"
      "                        [--rate HZ] [--duration SEC] [--timeout-ms N]\n"
      "                        [--turnaround-us N] [--rts-de] [--no-rts-de]\n"
      "                        [--rts-invert] [--no-strip-echo] [--verbose]\n"
      "                        [--self-test] [--set-zero]\n"
      " sku=%s  --addr single-polls (default 0x1F). --map polls 8 yaml addrs\n"
      " (4,5,6,7,10,11,12,13) sequential BUS-D. Do not combine --map and --addr.\n"
      " --set-zero WRITES encoder firmware/device zero at current shaft (BUS-D\n"
      " SetZero); not a yaml zero_rad edit. One-shot then one GetInfo. Needs\n"
      " --addr N or --map. Not with --duration / --iface r / --self-test.\n",
      biped_hw::kMbsSkuUser);
}

bool parse_iface(const char *v, biped_hw::MbsIface &out, std::string &err)
{
  if (std::strcmp(v, "d") == 0 || std::strcmp(v, "D") == 0 ||
      std::strcmp(v, "bus") == 0 || std::strcmp(v, "BUS") == 0)
  {
    out = biped_hw::MbsIface::BusD;
    return true;
  }
  if (std::strcmp(v, "r") == 0 || std::strcmp(v, "R") == 0 ||
      std::strcmp(v, "rs485") == 0 || std::strcmp(v, "RS485") == 0)
  {
    out = biped_hw::MbsIface::Rs485R;
    return true;
  }
  err = "--iface must be d (BUS) or r (RS485-R)";
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
    else if (std::strcmp(argv[i], "--baud") == 0)
    {
      const char *v = need("--baud");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      opt.baud = biped_hw::parse_u(v, ok);
      if (!ok)
      {
        err = "bad --baud";
        return false;
      }
      opt.baud_set = true;
    }
    else if (std::strcmp(argv[i], "--iface") == 0)
    {
      const char *v = need("--iface");
      if (v == nullptr)
      {
        return false;
      }
      if (!parse_iface(v, opt.iface, err))
      {
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--addr") == 0)
    {
      const char *v = need("--addr");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      const unsigned a = biped_hw::parse_u(v, ok);
      if (!ok || a > biped_hw::kMbsBusAddrMask)
      {
        err = "--addr must be 0..31 (default 0x1F)";
        return false;
      }
      opt.bus.addr = static_cast<uint8_t>(a);
      opt.addr_set = true;
    }
    else if (std::strcmp(argv[i], "--map") == 0)
    {
      opt.map = true;
    }
    else if (std::strcmp(argv[i], "--bits") == 0)
    {
      const char *v = need("--bits");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      const unsigned b = biped_hw::parse_u(v, ok);
      if (!ok || (b != 16 && b != 17))
      {
        err = "--bits must be 16 or 17";
        return false;
      }
      opt.layout.single_bits = static_cast<uint8_t>(b);
      opt.bus.single_bits = static_cast<uint8_t>(b);
    }
    else if (std::strcmp(argv[i], "--multi") == 0)
    {
      opt.layout.multi = true;
      if (opt.bus.multi_bits == 0)
      {
        opt.bus.multi_bits = 16;
      }
    }
    else if (std::strcmp(argv[i], "--multi-bits") == 0)
    {
      const char *v = need("--multi-bits");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      const unsigned m = biped_hw::parse_u(v, ok);
      if (!ok || (m != 0 && m != 8 && m != 16))
      {
        err = "--multi-bits must be 0, 8, or 16";
        return false;
      }
      opt.bus.multi_bits = static_cast<uint8_t>(m);
      opt.layout.multi = m != 0;
    }
    else if (std::strcmp(argv[i], "--cmd") == 0)
    {
      const char *v = need("--cmd");
      if (v == nullptr)
      {
        return false;
      }
      bool ok = false;
      const unsigned c = biped_hw::parse_u(v, ok);
      if (!ok)
      {
        err = "bad --cmd";
        return false;
      }
      if (c == biped_hw::kMbsCmdGetPos)
      {
        opt.layout.cmd = biped_hw::MbsCmd::GetPos;
      }
      else if (c == biped_hw::kMbsCmdGetPosStatus)
      {
        opt.layout.cmd = biped_hw::MbsCmd::GetPosStatus;
      }
      else if (c == biped_hw::kMbsCmdGetPosTemp)
      {
        opt.layout.cmd = biped_hw::MbsCmd::GetPosTemp;
      }
      else
      {
        err = "--cmd must be 0x31, 0x64, or 0x74 (iface r)";
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--rate") == 0)
    {
      const char *v = need("--rate");
      if (v == nullptr)
      {
        return false;
      }
      opt.rate_hz = std::atof(v);
      if (opt.rate_hz <= 0.0)
      {
        err = "--rate must be > 0";
        return false;
      }
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
    else if (std::strcmp(argv[i], "--timeout-ms") == 0)
    {
      const char *v = need("--timeout-ms");
      if (v == nullptr)
      {
        return false;
      }
      opt.timeout_ms = std::atoi(v);
      if (opt.timeout_ms < 1)
      {
        err = "--timeout-ms must be >= 1";
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--turnaround-us") == 0)
    {
      const char *v = need("--turnaround-us");
      if (v == nullptr)
      {
        return false;
      }
      opt.turnaround_us = std::atoi(v);
      if (opt.turnaround_us < 0)
      {
        err = "--turnaround-us must be >= 0";
        return false;
      }
    }
    else if (std::strcmp(argv[i], "--verbose") == 0)
    {
      opt.verbose = true;
    }
    else if (std::strcmp(argv[i], "--self-test") == 0)
    {
      opt.self_test = true;
    }
    else if (std::strcmp(argv[i], "--set-zero") == 0)
    {
      opt.set_zero = true;
    }
    else if (std::strcmp(argv[i], "--no-strip-echo") == 0)
    {
      opt.strip_echo = false;
    }
    else if (std::strcmp(argv[i], "--rts-de") == 0)
    {
      opt.rts_de = true;
      opt.rts_de_set = true;
    }
    else if (std::strcmp(argv[i], "--no-rts-de") == 0)
    {
      opt.rts_de = false;
      opt.rts_de_set = true;
    }
    else if (std::strcmp(argv[i], "--rts-invert") == 0)
    {
      opt.rts_invert = true;
    }
    else
    {
      err = std::string("unknown argument: ") + argv[i] + " (try --help)";
      return false;
    }
  }
  return true;
}

void print_hex(const uint8_t *p, std::size_t n)
{
  for (std::size_t i = 0; i < n; ++i)
  {
    std::printf("%s%02X", i == 0 ? "" : " ", p[i]);
  }
}

void print_sample(const biped_hw::MbsSample &s, bool verbose)
{
  if (s.has_status)
  {
    std::printf(
        "angle_deg=%.6f angle_rad=%.8f total_deg=%.6f total_rad=%.8f "
        "multi=%d raw=0x%05X status=0x%02X err=%d warn=%d",
        s.angle_deg, s.angle_rad, s.total_deg, s.total_rad,
        static_cast<int>(s.multi_turns), s.angle_raw,
        static_cast<unsigned>(s.status.raw), s.status.error ? 1 : 0,
        s.status.warning ? 1 : 0);
  }
  else
  {
    std::printf(
        "angle_deg=%.6f angle_rad=%.8f total_deg=%.6f total_rad=%.8f "
        "multi=%d raw=0x%05X",
        s.angle_deg, s.angle_rad, s.total_deg, s.total_rad,
        static_cast<int>(s.multi_turns), s.angle_raw);
  }
  if (s.has_temp)
  {
    std::printf(" temp_c=%.1f", static_cast<double>(s.temp_c));
  }
  if (verbose)
  {
    std::printf(" hex=");
    print_hex(s.raw.data(), s.raw_len);
  }
  std::printf("\n");
}

bool path_exists(const std::string &p)
{
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0;
}

std::vector<std::string> list_serial_by_id_names()
{
  std::vector<std::string> out;
  DIR *d = ::opendir("/dev/serial/by-id");
  if (d == nullptr)
  {
    return out;
  }
  while (const dirent *e = ::readdir(d))
  {
    if (e->d_name[0] == '.')
    {
      continue;
    }
    out.emplace_back(e->d_name);
  }
  ::closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

int count_tty_usb()
{
  int n = 0;
  DIR *d = ::opendir("/dev");
  if (d == nullptr)
  {
    return 0;
  }
  while (const dirent *e = ::readdir(d))
  {
    if (std::strncmp(e->d_name, "ttyUSB", 6) == 0)
    {
      ++n;
    }
  }
  ::closedir(d);
  return n;
}

const char *describe_by_id_name(const std::string &name)
{
  if (name.find("Prolific") != std::string::npos)
  {
    return "MBS encoder USB-RS485 (pl2303)";
  }
  if (name.find("Silicon_Labs") != std::string::npos ||
      name.find("CP210") != std::string::npos)
  {
    return "Wheeltec N100 IMU (cp210x)";
  }
  return "USB-serial";
}

void print_by_id_candidates(FILE *fp)
{
  const auto names = list_serial_by_id_names();
  std::fprintf(fp, "/dev/serial/by-id candidates:\n");
  if (names.empty())
  {
    std::fprintf(fp, "  (none — plug the Prolific USB-RS485 and the N100 CP210x)\n");
    return;
  }
  for (const auto &name : names)
  {
    const std::string full = "/dev/serial/by-id/" + name;
    char resolved[PATH_MAX];
    const char *tgt = ::realpath(full.c_str(), resolved) ? resolved : "?";
    std::fprintf(fp, "  %s\n    -> %s  (%s)\n", full.c_str(), tgt,
                 describe_by_id_name(name));
  }
}

bool resolve_encoder_port(Options &opt, biped_hw::MbsEncoderConfig &cfg, std::string &err)
{
  if (opt.port_set)
  {
    if (opt.port.empty())
    {
      err = "--port needs a non-empty PATH";
      return false;
    }
    if (!biped_hw::load_mbs_encoder_config(cfg, err))
    {
      return false;
    }
    return true;
  }
  if (!biped_hw::load_mbs_encoder_config(cfg, err))
  {
    return false;
  }
  if (!cfg.port.empty())
  {
    if (!path_exists(cfg.port))
    {
      err = "encoder port from " + cfg.path + " not found: " + cfg.port;
      return false;
    }
    opt.port = cfg.port;
    return true;
  }
  if (!cfg.path.empty())
  {
    err = cfg.path + " has no port: key";
    return false;
  }

  const auto names = list_serial_by_id_names();
  const int usb_n = count_tty_usb();
  if (names.size() == 1 && usb_n <= 1)
  {
    opt.port = "/dev/serial/by-id/" + names[0];
    return true;
  }
  err = "refusing to default to /dev/ttyUSB0 (N100 IMU cp210x and MBS USB-RS485 "
        "pl2303 both enumerate as ttyUSB* and swap with plug order). "
        "Pass --port /dev/serial/by-id/... or set port: in config/mbs_encoder.yaml "
        "(source install/setup.bash so the installed yaml is found)";
  return false;
}

void warn_high_baud(unsigned requested, unsigned linux_hz, const std::string &driver)
{
  if (requested < 2000000u)
  {
    return;
  }
  std::fprintf(stderr,
               "WARNING: BUS/D is datasheet-fixed 2.5 Mbps. "
               "CH340/CP2102/many FT232/some PL2303 cannot do 2500000; termios may "
               "accept B2500000 then run another rate (linux reports %u). "
               "This SKU has no 115200 suffix — 115200 will not match a D encoder.\n",
               linux_hz);
  if (!driver.empty())
  {
    std::fprintf(stderr, "WARNING: tty driver=%s\n", driver.c_str());
  }
  if (linux_hz != 0 && linux_hz != requested)
  {
    std::fprintf(stderr,
                 "WARNING: requested %u baud but driver reports %u.\n",
                 requested, linux_hz);
  }
}

int run(Options opt);

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

  if (opt.set_zero && opt.self_test)
  {
    std::fprintf(stderr, "refuse: --set-zero cannot be combined with --self-test\n");
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
    const int rc = biped_hw::mbs_codec_self_test();
    if (rc != 0)
    {
      std::fprintf(stderr, "mbs_codec self-test FAIL step %d\n", rc);
      return 1;
    }
    std::printf(
        "mbs_codec self-test OK (no hardware; RS485-R 0x64 CRC-8 0x97 + "
        "BUS-D addr 0x1F XOR CRC; SetZero 0xBF / addr 6,12; "
        "8-encoder map 4,5,6,7,10,11,12,13)\n");
    return 0;
  }
  if (opt.map && opt.addr_set)
  {
    std::fprintf(stderr, "use --addr (single) or --map (8 yaml addrs), not both\n");
    return 2;
  }
  if (opt.map && opt.iface != biped_hw::MbsIface::BusD)
  {
    std::fprintf(stderr, "--map is BUS-D only (this SKU is ...-D-...)\n");
    return 2;
  }
  if (opt.set_zero && opt.iface != biped_hw::MbsIface::BusD)
  {
    std::fprintf(stderr, "--set-zero is BUS-D only (RS485-R 0x30 unused on this SKU)\n");
    return 2;
  }
  if (opt.set_zero && !opt.addr_set && !opt.map)
  {
    std::fprintf(stderr, "--set-zero needs --addr N or --map\n");
    return 2;
  }
  if (opt.set_zero && opt.duration_s > 0.0)
  {
    std::fprintf(stderr,
                 "refuse: --set-zero is one-shot then one GetInfo; not with --duration\n");
    return 2;
  }
  return run(opt);
}

namespace
{

int run(Options opt)
{
  std::string err;
  biped_hw::MbsEncoderConfig cfg;
  if (!resolve_encoder_port(opt, cfg, err))
  {
    std::fprintf(stderr, "%s\n", err.c_str());
    print_by_id_candidates(stderr);
    return 2;
  }
  {
    const auto slash = opt.port.find_last_of('/');
    const std::string base =
        slash == std::string::npos ? opt.port : opt.port.substr(slash + 1);
    if (base.rfind("ttyUSB", 0) == 0 || base.rfind("ttyACM", 0) == 0)
    {
      std::fprintf(stderr,
                   "WARNING: %s numbering swaps with plug order; prefer "
                   "/dev/serial/by-id/... (see config/mbs_encoder.yaml)\n",
                   opt.port.c_str());
    }
  }

  if (!opt.baud_set)
  {
    opt.baud = opt.iface == biped_hw::MbsIface::BusD ? biped_hw::kMbsBusBaudDefault
                                                     : biped_hw::kMbsBaudDefault;
  }

  speed_t speed = B2500000;
  if (!baud_to_speed(opt.baud, speed))
  {
    std::fprintf(stderr,
                 "unsupported --baud %u (BUS-D is 2500000; R suffix A is 115200)\n",
                 opt.baud);
    return 2;
  }
  if (opt.turnaround_us < 0)
  {
    opt.turnaround_us = default_turnaround_us(opt.baud);
  }

  const bool bus = opt.iface == biped_hw::MbsIface::BusD;
  struct PollTarget
  {
    uint8_t addr;
    const char *name;
  };
  std::vector<PollTarget> targets;
  if (opt.map)
  {
    for (const auto &e : cfg.encoders)
    {
      targets.push_back({e.addr, e.name});
    }
  }
  else
  {
    targets.push_back({opt.bus.addr, nullptr});
  }
  if (bus)
  {
    opt.bus.addr = targets.front().addr;
  }
  const uint8_t banner_tx =
      bus ? biped_hw::mbs_bus_request(opt.bus)
          : static_cast<uint8_t>(opt.layout.cmd);
  const std::size_t reply_n =
      bus ? biped_hw::mbs_bus_reply_size(opt.bus) : biped_hw::mbs_reply_size(opt.layout);
  const std::string driver = biped_hw::tty_driver_name(opt.port);
  if (!opt.rts_de_set && driver == "pl2303")
  {
    opt.rts_de = true;
  }

  if (bus && opt.set_zero)
  {
    if (opt.map)
    {
      std::printf(
          "MBS BUS-D set-zero sku=%s port=%s baud=%u uart=8N1 map8 addrs=",
          biped_hw::kMbsSkuUser, opt.port.c_str(), opt.baud);
      for (std::size_t i = 0; i < targets.size(); ++i)
      {
        std::printf("%s%u", i == 0 ? "" : ",", static_cast<unsigned>(targets[i].addr));
      }
      std::printf(
          " (device origin at current shaft, not yaml zero_rad) "
          "rts_de=%d driver=%s\n",
          opt.rts_de ? 1 : 0, driver.empty() ? "?" : driver.c_str());
    }
    else
    {
      const uint8_t zero_tx =
          biped_hw::mbs_bus_request(biped_hw::MbsBusOp::SetZero, targets.front().addr);
      std::printf(
          "MBS BUS-D set-zero sku=%s port=%s baud=%u uart=8N1 addr=0x%02X "
          "req=0x%02X (device origin at current shaft, not yaml zero_rad) "
          "rts_de=%d driver=%s\n",
          biped_hw::kMbsSkuUser, opt.port.c_str(), opt.baud,
          static_cast<unsigned>(targets.front().addr),
          static_cast<unsigned>(zero_tx), opt.rts_de ? 1 : 0,
          driver.empty() ? "?" : driver.c_str());
    }
  }
  else if (bus)
  {
    if (opt.map)
    {
      std::printf(
          "MBS BUS-D poll sku=%s port=%s baud=%u uart=8N1 map8 addrs=",
          biped_hw::kMbsSkuUser, opt.port.c_str(), opt.baud);
      for (std::size_t i = 0; i < targets.size(); ++i)
      {
        std::printf("%s%u", i == 0 ? "" : ",", static_cast<unsigned>(targets[i].addr));
      }
      std::printf(
          " bits=%u multi_bits=%u reply=%zu B rx_wait=%d ms rate=%.1f Hz "
          "turnaround=%d us strip_echo=%d rts_de=%d driver=%s\n",
          static_cast<unsigned>(opt.bus.single_bits),
          static_cast<unsigned>(opt.bus.multi_bits), reply_n, opt.timeout_ms,
          opt.rate_hz, opt.turnaround_us, opt.strip_echo ? 1 : 0, opt.rts_de ? 1 : 0,
          driver.empty() ? "?" : driver.c_str());
    }
    else
    {
      std::printf(
          "MBS BUS-D poll sku=%s port=%s baud=%u uart=8N1 (b7 odd-parity in payload, "
          "not termios PARENB) addr=0x%02X req=0x%02X bits=%u multi_bits=%u "
          "reply=%zu B rx_wait=%d ms rate=%.1f Hz turnaround=%d us "
          "strip_echo=%d rts_de=%d driver=%s\n",
          biped_hw::kMbsSkuUser, opt.port.c_str(), opt.baud,
          static_cast<unsigned>(opt.bus.addr), static_cast<unsigned>(banner_tx),
          static_cast<unsigned>(opt.bus.single_bits),
          static_cast<unsigned>(opt.bus.multi_bits), reply_n, opt.timeout_ms,
          opt.rate_hz, opt.turnaround_us, opt.strip_echo ? 1 : 0, opt.rts_de ? 1 : 0,
          driver.empty() ? "?" : driver.c_str());
    }
    std::printf(
        "hint: position read for this model:\n"
        "  ros2 run biped_hw mbs_encoder_recv --iface d --addr 0x1F --baud 2500000 "
        "--bits 17 --verbose --duration 2\n"
        "  ros2 run biped_hw mbs_encoder_recv --map --duration 1 --verbose\n"
        "  (default port is config/mbs_encoder.yaml by-id, not ttyUSB0; "
        "N100 IMU is the CP210x)\n"
        "  (pl2303: RTS DE is on by default; add --rts-invert only if DE is active-low.\n"
        "   --no-rts-de holds RTS off for UART-TX auto-DE; if rx=0 the board may need\n"
        "   RTS to enable the transceiver at all)\n");
  }
  else
  {
    std::printf(
        "MBS RS485-R poll port=%s baud=%u 8N1 cmd=0x%02X bits=%u multi=%d "
        "reply=%zu B rate=%.1f Hz turnaround=%d us strip_echo=%d rts_de=%d "
        "driver=%s\n",
        opt.port.c_str(), opt.baud, static_cast<unsigned>(banner_tx),
        static_cast<unsigned>(opt.layout.single_bits), opt.layout.multi ? 1 : 0,
        reply_n, opt.rate_hz, opt.turnaround_us, opt.strip_echo ? 1 : 0,
        opt.rts_de ? 1 : 0, driver.empty() ? "?" : driver.c_str());
  }

  biped_hw::SerialPort serial(opt.port, speed, 2, opt.baud);
  if (!serial.open())
  {
    std::fprintf(stderr, "open %s failed: %s (USB-RS485, group dialout?)\n",
                 opt.port.c_str(), serial.last_error().c_str());
    return 1;
  }
  warn_high_baud(opt.baud, serial.linux_baud_hz(), driver);
  if (serial.linux_baud_hz() != 0)
  {
    std::printf("linux baud reports %u (requested %u)\n", serial.linux_baud_hz(),
                opt.baud);
  }
  if (!opt.rts_de)
  {
    // Kernel USB-serial open typically asserts DTR+RTS (pl2303
    // set_control_lines 03). --no-rts-de used to skip pulsing but leave RTS
    // stuck on → DE held, encoder never gets the bus, each TX still yields
    // one BREAK/0x00. Clear RTS only; leave DTR. Do not pulse per poll.
    serial.set_rts(false);
    std::printf(
        "RTS held off for auto-DE (TIOCM_RTS cleared, DTR unchanged). "
        "If rx=0 the board may need RTS to enable the transceiver at all.\n");
  }

  if (opt.set_zero)
  {
    std::fprintf(stderr,
                 "WARN: --set-zero WRITES encoder device zero at current shaft "
                 "(BUS-D SetZero). Not yaml zero_rad.\n");
    bool any_info = false;
    for (const auto &tgt : targets)
    {
      const uint8_t tx =
          biped_hw::mbs_bus_request(biped_hw::MbsBusOp::SetZero, tgt.addr);
      if (!biped_hw::mbs_bus_send_set_zero(serial, tgt.addr, opt.rts_de))
      {
        std::fprintf(stderr, "set-zero addr=%u tx=0x%02X failed\n",
                     static_cast<unsigned>(tgt.addr), static_cast<unsigned>(tx));
        return 1;
      }
      if (tgt.name != nullptr)
      {
        std::printf("set-zero addr=%u name=%s tx=0x%02X sent\n",
                    static_cast<unsigned>(tgt.addr), tgt.name,
                    static_cast<unsigned>(tx));
      }
      else
      {
        std::printf("set-zero addr=%u tx=0x%02X sent\n",
                    static_cast<unsigned>(tgt.addr), static_cast<unsigned>(tx));
      }
      biped_hw::MbsSample sample{};
      if (biped_hw::mbs_bus_poll_once(serial, tgt.addr, opt.timeout_ms, opt.rts_de,
                                      sample))
      {
        any_info = true;
        if (tgt.name != nullptr)
        {
          std::printf("addr=%u name=%s valid=1 ", static_cast<unsigned>(tgt.addr),
                      tgt.name);
        }
        print_sample(sample, opt.verbose);
      }
      else if (tgt.name != nullptr)
      {
        std::printf("addr=%u name=%s valid=0 (GetInfo after SetZero)\n",
                    static_cast<unsigned>(tgt.addr), tgt.name);
      }
      else
      {
        std::printf("addr=%u valid=0 (GetInfo after SetZero)\n",
                    static_cast<unsigned>(tgt.addr));
      }
    }
    return any_info ? 0 : 1;
  }

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  biped_hw::MbsRs485Framer framer_r(opt.layout);
  biped_hw::MbsBusFramer framer_d(opt.bus);
  uint8_t raw[128];
  std::uint64_t polls = 0;
  std::uint64_t samples = 0;
  std::uint64_t timeouts = 0;
  std::uint64_t echo_strips = 0;
  const auto t0 = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / opt.rate_hz);

  while (g_run)
  {
    if (opt.duration_s > 0.0)
    {
      const double elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
              .count();
      if (elapsed >= opt.duration_s)
      {
        break;
      }
    }

    const auto cycle_start = std::chrono::steady_clock::now();
    for (const auto &tgt : targets)
    {
      if (!g_run)
      {
        break;
      }
      if (bus)
      {
        opt.bus.addr = tgt.addr;
        framer_d.set_layout(opt.bus);
      }
      const uint8_t tx =
          bus ? biped_hw::mbs_bus_request(opt.bus)
              : static_cast<uint8_t>(opt.layout.cmd);
      ++polls;
      serial.flush_input();
      framer_r.clear();
      framer_d.clear();

      if (opt.rts_de)
      {
        serial.set_rts(!opt.rts_invert);
      }
      if (serial.send(&tx, 1) != 1)
      {
        std::fprintf(stderr, "TX 0x%02X failed\n", static_cast<unsigned>(tx));
        return 1;
      }
      serial.drain_output();
      if (opt.rts_de)
      {
        // tcdrain includes the stop bit. Busy-wait two bit times — sleep_for(10us)
        // can become 1ms+ and hold DE through the entire ~24 us BUS-D reply.
        const int bit_us =
            opt.baud > 0 ? static_cast<int>((1000000u + opt.baud - 1u) / opt.baud) : 1;
        wait_us(bit_us < 1 ? 1 : bit_us * 2);
        serial.set_rts(opt.rts_invert);
      }
      if (opt.turnaround_us > 0)
      {
        wait_us(opt.turnaround_us);
      }

      // Block until a full reply (6 bytes for 17-bit BUS-D) or timeout. Do not
      // finish the poll on the first byte. Leading 0x00 is a DE/break glitch.
      std::size_t raw_n = 0;
      bool stripped = false;
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(opt.timeout_ms);
      biped_hw::MbsSample sample{};
      bool got = false;
      uint8_t view[128];

      auto payload_view = [&](std::size_t &view_n) {
        std::size_t off = 0;
        while (off < raw_n && raw[off] == 0x00)
        {
          ++off;
        }
        view_n = 0;
        if (off >= raw_n)
        {
          return;
        }
        const std::size_t avail = raw_n - off;
        const std::size_t copy_n = avail < sizeof(view) ? avail : sizeof(view);
        std::memcpy(view, raw + off, copy_n);
        view_n = copy_n;
        if (!opt.strip_echo)
        {
          return;
        }
        const std::size_t after =
            bus ? biped_hw::mbs_bus_strip_leading_echo(view, copy_n, tx, reply_n)
                : biped_hw::mbs_strip_leading_echo(view, copy_n, tx, reply_n);
        if (after < copy_n)
        {
          stripped = true;
        }
        view_n = after;
      };

      while (std::chrono::steady_clock::now() < deadline)
      {
        std::size_t view_n = 0;
        payload_view(view_n);
        if (view_n > 0)
        {
          if (bus)
          {
            framer_d.clear();
            framer_d.feed(view, view_n);
            if (framer_d.pop(sample))
            {
              got = true;
              break;
            }
          }
          else
          {
            framer_r.clear();
            framer_r.feed(view, view_n);
            if (framer_r.pop(sample))
            {
              got = true;
              break;
            }
          }
        }

        const std::size_t room = sizeof(raw) - raw_n;
        if (room == 0)
        {
          break;
        }
        const std::size_t want = view_n >= reply_n ? 1u : (reply_n - view_n);
        const std::size_t ask = want < room ? want : room;
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
          break;
        }
        int left_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        if (left_ms < 1)
        {
          left_ms = 1;
        }
        const ssize_t n = serial.recv_n(raw + raw_n, ask, left_ms);
        if (n <= 0)
        {
          break;
        }
        raw_n += static_cast<std::size_t>(n);
      }
      if (stripped)
      {
        ++echo_strips;
      }
      framer_r.clear();
      framer_d.clear();

      if (got)
      {
        ++samples;
        if (tgt.name != nullptr)
        {
          std::printf("addr=%u name=%s valid=1 ", static_cast<unsigned>(tgt.addr),
                      tgt.name);
        }
        print_sample(sample, opt.verbose);
      }
      else
      {
        ++timeouts;
        if (tgt.name != nullptr)
        {
          std::printf("addr=%u name=%s valid=0", static_cast<unsigned>(tgt.addr),
                      tgt.name);
          if (opt.verbose)
          {
            std::printf(" poll=%llu rx=%zu", static_cast<unsigned long long>(polls),
                        raw_n);
          }
          std::printf("\n");
        }
        else if (opt.verbose)
        {
          std::size_t z = 0;
          while (z < raw_n && raw[z] == 0x00)
          {
            ++z;
          }
          std::printf("timeout poll=%llu rx=%zu frame=%zu echo_stripped=%d hex=",
                      static_cast<unsigned long long>(polls), raw_n, raw_n - z,
                      stripped ? 1 : 0);
          if (raw_n == 0)
          {
            std::printf(
                "(empty: encoder silent, baud mismatch, RS485 still in TX, "
                "wrong iface — D ignores 0x64, or RTS held off so the "
                "transceiver never enabled — drop --no-rts-de)");
          }
          else
          {
            print_hex(raw, raw_n);
          }
          std::printf("\n");
        }
      }
    }

    const auto used = std::chrono::steady_clock::now() - cycle_start;
    if (used < period)
    {
      std::this_thread::sleep_for(period - used);
    }
  }

  const std::size_t garbage =
      bus ? framer_d.garbage_drops() : framer_r.garbage_drops();
  std::printf(
      "done polls=%llu samples=%llu timeouts=%llu garbage_drops=%zu echo_strips=%llu\n",
      static_cast<unsigned long long>(polls),
      static_cast<unsigned long long>(samples),
      static_cast<unsigned long long>(timeouts), garbage,
      static_cast<unsigned long long>(echo_strips));
  if (samples == 0)
  {
    std::fprintf(stderr,
                 "no CRC-valid reply. SKU ...-D-... is BUS (TX 0x1F), not protocol R "
                 "(0x64). 0x64 has BUS op=11 and the encoder returns nothing. "
                 "BUS baud is 2.5 Mbps UART 8N1 (odd parity is request b7; not 9-bit). "
                 "rx=1 hex=00 after a full 6-byte wait is a DE/idle glitch or a "
                 "mis-framed 0x1F, not a BUS-D frame. --no-rts-de holds RTS off "
                 "for UART-TX auto-DE; if that yields rx=0 the board may need "
                 "RTS to enable the transceiver (--rts-de). Else try "
                 "--rts-invert, or the USB-UART may not actually run 2500000.\n");
  }
  return samples > 0 ? 0 : 1;
}

}  // namespace
