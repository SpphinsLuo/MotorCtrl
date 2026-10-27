// biped_hw — p4_hip_kin_self_test.cpp
// Thin main: hip parallel FK/IK round-trip. No hardware, no 0xFC.
#include "biped_hw/hip_parallel_kin.hpp"

#include <cstdio>
#include <cstring>

namespace
{

void usage()
{
  std::printf(
      "Usage: p4_hip_kin_self_test [--self-test] [--help]\n"
      "Hip parallel FK/IK from 髋关节并联机构的运动耦合分析.doc (2-8)(2-9).\n"
      "  --self-test     polynomial IK + Newton FK round-trip on the doc\n"
      "                  workspace; left/right roll mirror. No hardware.\n"
      "  --help          this message (rc=0)\n"
      "Does not enable motors. Does not send CAN.\n"
      "Upper control: hip_fk on the state path (q_outer/q_inner → roll/pitch);\n"
      "hip_ik on the command path (roll/pitch → q_outer/q_inner).\n"
      "Yaw (CAN 3/9) is independent. P5 limits stay per motor ID.\n");
}

}  // namespace

int main(int argc, char **argv)
{
  bool self_test = false;
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
    std::fprintf(stderr, "unknown argument: %s (try --help)\n", argv[i]);
    return 2;
  }

  if (!self_test)
  {
    usage();
    std::printf("pass --self-test (no hardware, no 0xFC)\n");
    return 0;
  }

  const int rc = biped_hw::hip_parallel_kin_self_test();
  if (rc != 0)
  {
    std::fprintf(stderr, "p4_hip_kin_self_test FAIL step %d\n", rc);
    return rc;
  }

  biped_hw::HipPose p{0.1, -0.2};//这里应该给特值或已知正确的对应值
  biped_hw::HipMotors q{};
  biped_hw::hip_ik(biped_hw::HipSide::Left, p, &q);
  std::printf(
      "p4_hip_kin_self_test OK (no hardware, no 0xFC): "
      "IK/FK round-trip on α∈[-20°,20°] β∈[-45°,15°]; "
      "sample Left roll=0.1 pitch=-0.2 -> q_outer=%.6f q_inner=%.6f rad\n",
      q.q_outer, q.q_inner);
  return 0;
}
