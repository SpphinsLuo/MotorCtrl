#include "mbs_encoder/mbs_host.hpp"
#include "mbs_encoder/mbs_map.hpp"
#include "mbs_encoder/serial_port.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct Options
{
  std::vector<int> addrs;
  int timeout_ms = 8;
  std::string port;
};

void print_usage(const char *program)
{
  std::printf(
      "Usage: %s [--addr ADDRESS[,ADDRESS...]] [--port PATH] [--timeout-ms N]\n"
      "\n"
      "Write the current position as the hardware zero for configured MBS encoders.\n"
      "Without --addr, all configured encoders are written.\n"
      "This is a persistent device write; stop mbs_encoder_node first.\n"
      "\n"
      "  --addr ADDRESS   select decimal addresses separated by commas\n"
      "  --port PATH      override the configured MBS serial port\n"
      "  --timeout-ms N   reply timeout in milliseconds (default: 8)\n",
      program);
}

bool parse_int(const char *text, int &value)
{
  if (text == nullptr || *text == '\0')
  {
    return false;
  }
  char *end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed < 0 || parsed > 255)
  {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parse_addr_list(const char *text, std::vector<int> &addrs)
{
  std::string list = text == nullptr ? "" : text;
  std::size_t start = 0;
  while (start <= list.size())
  {
    const std::size_t comma = list.find(',', start);
    const std::string item = list.substr(start, comma == std::string::npos
                                                  ? std::string::npos
                                                  : comma - start);
    int addr = -1;
    if (!parse_int(item.c_str(), addr))
    {
      return false;
    }
    addrs.push_back(addr);
    if (comma == std::string::npos)
    {
      break;
    }
    start = comma + 1;
  }
  return !addrs.empty();
}

bool parse_options(int argc, char **argv, Options &options)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
    {
      print_usage(argv[0]);
      return false;
    }
    if (arg == "--addr" && i + 1 < argc)
    {
      if (!parse_addr_list(argv[++i], options.addrs))
      {
        std::fprintf(stderr, "invalid --addr: %s\n", argv[i]);
        return false;
      }
    }
    else if (arg == "--port" && i + 1 < argc)
    {
      options.port = argv[++i];
    }
    else if (arg == "--timeout-ms" && i + 1 < argc)
    {
      if (!parse_int(argv[++i], options.timeout_ms) || options.timeout_ms <= 0)
      {
        std::fprintf(stderr, "invalid --timeout-ms: %s\n", argv[i]);
        return false;
      }
    }
    else
    {
      std::fprintf(stderr, "unknown or incomplete argument: %s\n", arg.c_str());
      print_usage(argv[0]);
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char **argv)
{
  Options options;
  if (!parse_options(argc, argv, options))
  {
    return 2;
  }

  biped_hw::MbsEncoderConfig config;
  std::string config_error;
  if (!biped_hw::load_mbs_encoder_config(config, config_error))
  {
    std::fprintf(stderr, "cannot load MBS config: %s\n", config_error.c_str());
    return 1;
  }
  if (options.port.empty())
  {
    options.port = config.port;
  }

  if (options.addrs.empty())
  {
    for (const auto &entry : config.encoders)
    {
      options.addrs.push_back(entry.addr);
    }
  }

  for (const int addr : options.addrs)
  {
    bool configured_address = false;
    for (const auto &entry : config.encoders)
    {
      if (entry.addr == static_cast<std::uint8_t>(addr))
      {
        configured_address = true;
        break;
      }
    }
    if (!configured_address)
    {
      std::fprintf(stderr, "refuse: address %d is not in the configured MBS map\n", addr);
      return 2;
    }
  }

  std::printf(
      "MBS hardware zero write\n"
      "  addresses:");
  for (const int addr : options.addrs)
  {
    std::printf(" %d", addr);
  }
  std::printf(
      "\n"
      "  port: %s\n"
      "  timeout: %d ms\n"
      "  WARNING: the current encoder position will be persistently stored as zero.\n",
      options.port.c_str(), options.timeout_ms);

  biped_hw::SerialPort serial(options.port, B2500000, 2, biped_hw::kMbsBusBaudDefault);
  if (!serial.open())
  {
    std::fprintf(stderr, "cannot open MBS port %s: %s\n", options.port.c_str(),
                 serial.last_error().c_str());
    return 1;
  }

  const bool rts_de = biped_hw::tty_driver_name(options.port) == "pl2303";
  for (const int addr : options.addrs)
  {
    biped_hw::MbsSetZeroResult result;
    if (!biped_hw::mbs_bus_set_zero_sequence(
            serial, static_cast<std::uint8_t>(addr), options.timeout_ms, rts_de, result))
    {
      std::fprintf(stderr, "MBS set-zero failed at address %d after %d pair(s), last C=%u S=%u\n",
                   addr, result.pairs, static_cast<unsigned>(result.last_c),
                   static_cast<unsigned>(result.last_s));
      return 1;
    }
    std::printf("MBS set-zero committed: address=%d pairs=%d C=%u S=%u\n", addr, result.pairs,
                static_cast<unsigned>(result.last_c), static_cast<unsigned>(result.last_s));
  }
  return 0;
}
