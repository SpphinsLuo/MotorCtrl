// biped_hw — hw_smoke_dual_bus.cpp
// P0 冒烟：打印编译期 12 电机表，并尝试打开左右 USB-CAN（不使能、不发 MIT）。
#include "biped_hw/motor_map.hpp"
#include "biped_hw/serial_port.hpp"
#include "biped_hw/usb_can_adapter.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace
{

void print_map()
{
  std::printf("=== biped_hw motor map (compile-time, %zu motors) ===\n",
              biped_hw::kMotorCount);
  std::printf(
      " idx  bus    default-port     master  CAN  joint                 model      notes\n");
  for (std::size_t i = 0; i < biped_hw::kMotorMap.size(); ++i)
  {
    const auto &e = biped_hw::kMotorMap[i];
    std::printf(" %3zu  %-5s  %-16s     %u   %2u  %-21s %-10s %s\n", i,
                biped_hw::to_string(e.bus), biped_hw::default_port(e.bus),
                static_cast<unsigned>(e.master_id),
                static_cast<unsigned>(e.can_id), biped_hw::to_string(e.joint),
                biped_hw::to_string(e.model), e.notes);
  }
  std::printf(" master id = %u (0x%02X) on both buses\n",
              static_cast<unsigned>(biped_hw::kMasterId),
              static_cast<unsigned>(biped_hw::kMasterId));
}

bool try_open(const char *label, const std::string &port)
{
  biped_hw::UsbCanAdapter adapter(port);
  const bool ok = adapter.open();
  if (ok)
  {
    const auto &ini = adapter.init_stats();
    if (ini.handshake_sent)
    {
      std::printf("  %-5s  %s : OK (open only, wait_ms=%u CAN baud %u cmd=%02X %02X %02X %02X %02X "
                  "handshake=%02X %02X %02X %02X usb_reply_bytes=%llu tx=%s, motors not enabled)\n",
                  label, port.c_str(), ini.wait_ms, adapter.can_bitrate(),
                  static_cast<unsigned>(ini.baud_cmd[0]),
                  static_cast<unsigned>(ini.baud_cmd[1]),
                  static_cast<unsigned>(ini.baud_cmd[2]),
                  static_cast<unsigned>(ini.baud_cmd[3]),
                  static_cast<unsigned>(ini.baud_cmd[4]),
                  static_cast<unsigned>(ini.handshake_prefix[0]),
                  static_cast<unsigned>(ini.handshake_prefix[1]),
                  static_cast<unsigned>(ini.handshake_prefix[2]),
                  static_cast<unsigned>(ini.handshake_prefix[3]),
                  static_cast<unsigned long long>(ini.usb_reply_bytes),
                  ini.control_tx_log);
    }
    else
    {
      std::printf("  %-5s  %s : OK (open only, wait_ms=%u CAN baud %u cmd=%02X %02X %02X %02X %02X "
                  "handshake=off usb_reply_bytes=%llu tx=%s, motors not enabled)\n",
                  label, port.c_str(), ini.wait_ms, adapter.can_bitrate(),
                  static_cast<unsigned>(ini.baud_cmd[0]),
                  static_cast<unsigned>(ini.baud_cmd[1]),
                  static_cast<unsigned>(ini.baud_cmd[2]),
                  static_cast<unsigned>(ini.baud_cmd[3]),
                  static_cast<unsigned>(ini.baud_cmd[4]),
                  static_cast<unsigned long long>(ini.usb_reply_bytes),
                  ini.control_tx_log);
    }
    adapter.close();
    return true;
  }
  std::printf("  %-5s  %s : FAIL (%s)\n", label, port.c_str(),
              adapter.last_error().c_str());
  return false;
}

int sniff_raw(const std::string &port, int sniff_ms)
{
  biped_hw::SerialPort serial(port);
  if (!serial.open())
  {
    std::printf("sniff open %s failed: %s (close Damiao 上位机 if it holds ACM0)\n",
                port.c_str(), serial.last_error().c_str());
    return 1;
  }
  std::printf("sniff %s for %d ms (raw USB2CAN serial bytes; no CAN TX, no enable)\n",
              port.c_str(), sniff_ms);
  uint8_t buf[256];
  int total = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(sniff_ms);
  while (std::chrono::steady_clock::now() < deadline)
  {
    const ssize_t n = serial.recv(buf, sizeof(buf));
    if (n <= 0)
    {
      continue;
    }
    total += static_cast<int>(n);
    std::printf("raw %zd:", n);
    for (ssize_t i = 0; i < n; ++i)
    {
      std::printf(" %02X", buf[static_cast<std::size_t>(i)]);
    }
    std::printf("\n");
  }
  std::printf(
      "sniff done: %d serial bytes. 0 bytes with 上位机 closed is idle bus; "
      "Permission denied means another process holds the port.\n",
      total);
  serial.close();
  return 0;
}

}  // namespace

int main(int argc, char **argv)
{
  bool require_ports = false;
  int sniff_ms = 0;
  std::string left_port = biped_hw::kDefaultPortLeft;
  std::string right_port = biped_hw::kDefaultPortRight;

  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--require-ports") == 0)
    {
      require_ports = true;
    }
    else if (std::strcmp(argv[i], "--left-port") == 0 && i + 1 < argc)
    {
      left_port = argv[++i];
    }
    else if (std::strcmp(argv[i], "--right-port") == 0 && i + 1 < argc)
    {
      right_port = argv[++i];
    }
    else if (std::strcmp(argv[i], "--sniff-ms") == 0 && i + 1 < argc)
    {
      sniff_ms = std::atoi(argv[++i]);
    }
    else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      std::printf(
          "Usage: hw_smoke_dual_bus [--require-ports] "
          "[--left-port PATH] [--right-port PATH] [--sniff-ms N]\n"
          "Prints the 12-motor map, then tries to open USB-CAN adapters.\n"
          "--sniff-ms N: hex-dump raw serial from --left-port for N ms "
          "(no CAN TX, no enable). Close 上位机 first.\n"
          "Never enables motors. Never sends MIT/servo.\n"
          "Exit 0 after a successful map dump; with --require-ports, exit 1 "
          "if either port fails to open.\n");
      return 0;
    }
    else
    {
      std::fprintf(stderr, "unknown argument: %s (try --help)\n", argv[i]);
      return 2;
    }
  }

  print_map();

  if (sniff_ms > 0)
  {
    std::printf("\n=== raw USB2CAN sniff (no motor enable, no CAN payload) ===\n");
    return sniff_raw(left_port, sniff_ms);
  }

  std::printf("\n=== open adapters (no motor enable, no CAN payload) ===\n");
  const bool left_ok = try_open("left", left_port);
  const bool right_ok = try_open("right", right_port);

  if (!left_ok || !right_ok)
  {
    std::printf(
        "port missing or busy is OK for CI; pass --require-ports to fail.\n");
  }

  if (require_ports && (!left_ok || !right_ok))
  {
    return 1;
  }
  return 0;
}
