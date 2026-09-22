// biped_hw — p3_runtime.hpp
// P3 三个 app 共用：CLI、探测、仅对选中 ID 写 CTRL_MODE/使能、同周期 hold 与汇总表。
// --ids 列出 HA60 则纳入伺服 hold（不因默认 skip-motorevo 丢掉）。缺电机超时记 drop。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace biped_hw
{

enum class P3App : uint8_t
{
  Left6 = 0,
  Dual12,
  FaultInject,
};

struct P3Options
{
  P3App app = P3App::Left6;
  std::string left_port;
  std::string right_port;
  std::string yaml_path;
  double duration_s = 8.0;
  int period_ms = 10;
  int drain_ms = 8;
  float kp = 8.f;
  float kd = 0.5f;
  std::vector<uint8_t> ids = {6};
  std::vector<uint8_t> ignore_ids;
  bool skip_motorevo = true;
  bool require_all = false;
  bool allow_single_bus = false;
  bool enable_all_anyway = false;
  bool handshake = false;
  bool verbose = false;
  bool self_test = false;
  bool ids_set = false;
};

const char *p3_bin_name(P3App app);
void p3_print_usage(const P3Options &defaults);
bool p3_parse_args(int argc, char **argv, P3Options &opt, std::string &err);
int p3_run(const P3Options &opt);

}  // namespace biped_hw
