// biped_hw — p3_left6_sync.cpp
// P3 左总线同周期同步：默认探测 CAN 3–7（跳过 Motorevo 8），只使能 --ids（默认 6）。
// 显式 --ids 6,8 会带上 HA60 伺服 hold（不跳过），未列出的 3/4/5/7 不发不使能。
// 缺电机超时记 drop，不阻塞；退出对已使能 ID 发 0xFD。
#include "biped_hw/p3_runtime.hpp"

#include <cstdio>
#include <string>

int main(int argc, char **argv)
{
  biped_hw::P3Options opt;
  opt.app = biped_hw::P3App::Left6;
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
