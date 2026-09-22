// biped_hw — p3_fault_inject.cpp
// P3 故障注入：左总线同步循环中 --ignore-id 模拟丢帧（不发、不等待）。
// 只记录其它 ID 是否仍更新、循环是否被拖死。P5 才做 disable-all。
#include "biped_hw/p3_runtime.hpp"

#include <cstdio>
#include <string>

int main(int argc, char **argv)
{
  biped_hw::P3Options opt;
  opt.app = biped_hw::P3App::FaultInject;
  std::string err;
  if (!biped_hw::p3_parse_args(argc, argv, opt, err))
  {
    if (err == "help")
    {
      return 0;
    }
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }
  return biped_hw::p3_run(opt);
}
