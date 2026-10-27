// biped_hw — p3_runtime.hpp
// P3 apps 共用：CLI、探测、仅对选中 ID 写 CTRL_MODE/使能、同周期 hold 或 sine。
// 只操作 --ids（dual12 默认 map 3–14；right6 默认右腿 9–14；其余默认 6）。缺电机超时记 drop。
// --comms-only：只 RID / MIT kp=0 inquire / HA60 evo status，不发 0xFC。
// --stage sine：绕探测 q0 正弦摆，不调用 dm_home_to_zero / 不回固件 0。
// --set-zero：对选中 --ids 写固件零点（Damiao / HA60 官方 0xFE），
// 不与 --stage 并用；随后 0xFD。
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
  Right6,
};

enum class P3Stage : uint8_t
{
  Hold = 0,
  Sine,
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
  float tau_ff = 0.f;
  float vel = 0.2f;
  float stroke = 0.349f;  // ±20 deg
  float abort_tau = 0.f;  // 0 = per-motor TMAX (peak)
  std::vector<uint8_t> ids = {6};
  std::vector<uint8_t> ignore_ids;
  P3Stage stage = P3Stage::Hold;
  bool require_all = false;
  bool allow_single_bus = false;
  bool enable_all_anyway = false;
  bool handshake = false;
  bool verbose = false;
  bool self_test = false;
  bool ids_set = false;
  bool comms_only = false;
  bool set_zero = false;
  bool stage_set = false;
};

const char *p3_bin_name(P3App app);
void p3_print_usage(const P3Options &defaults);
bool p3_parse_args(int argc, char **argv, P3Options &opt, std::string &err);
int p3_run(const P3Options &opt);

}  // namespace biped_hw
