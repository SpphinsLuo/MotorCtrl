// biped_hw — p5_safety_self_test.cpp
// P5 gate: watchdog / limits / e-stop / dual-bus disable-all. Thin main.
// --self-test injects faults in software and asserts 0xFD was requested.
// Never 0xFC. Does not enable motors.
#include "biped_hw/safety.hpp"

#include <cstdio>
#include <cstring>
#include <string>

namespace
{

void usage()
{
  std::printf(
      "Usage: p5_safety_self_test [--self-test] [--help]\n"
      "P5 safety layer (watchdog, soft/hard limits, e-stop latch, dual-bus disable-all).\n"
      "  --self-test     inject timeout / e-stop / hard-limit / drop in software;\n"
      "                  assert disable-all requested on map IDs 3-14 (left 3-8,\n"
      "                  right 9-14). Never sends 0xFC. No hardware required.\n"
      "  --help          this message (rc=0)\n"
      "Does not enable motors. Live 0xFC is not a flag.\n"
      "Limits are per CAN ID (motor_map 12); q_soft=10 rad hold, q_hard=12 rad\n"
      "disable, tau_hard = that motor TMAX. No hip IK geometry.\n"
      "E-stop is a software latch + SIGINT/SIGTERM; no GPIO.\n"
      "Disable-all reuses dm_send_disable (0xFD) on both buses.\n");
}

}  // namespace

int main(int argc, char **argv)
{
  bool self_test = false;
  std::string err;
  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
    {
      usage();
      return 0;
    }
    if (std::strcmp(argv[i], "--self-test") == 0)
    {
      self_test = true;
      continue;
    }
    if (std::strcmp(argv[i], "--force-limits") == 0 ||
        std::strcmp(argv[i], "--abort-tau") == 0)
    {
      std::fprintf(stderr, "%s is not a p5 flag (tau_hard=TMAX; no --force-limits)\n", argv[i]);
      return 2;
    }
    err = std::string("unknown argument: ") + argv[i] + " (try --help)";
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }

  if (!self_test)
  {
    usage();
    std::printf("pass --self-test (no hardware, no 0xFC)\n");
    return 0;
  }

  const int rc = biped_hw::safety_self_test();
  if (rc != 0)
  {
    std::fprintf(stderr, "p5_safety_self_test FAIL step %d\n", rc);
    return rc;
  }
  std::printf(
      "p5_safety_self_test OK (no hardware, no 0xFC): watchdog/estop/hard_q/drop/rid "
      "requested disable-all on CAN 3-14 (left 3-8, right 9-14); soft q holds\n");
  return 0;
}
