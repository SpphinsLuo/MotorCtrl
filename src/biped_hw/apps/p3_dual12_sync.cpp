// 双腿12电机协同测试程序
// P3 双总线 12 路同周期：先左后右发送，再两路同时收包。ttyACM1 缺失时报错，
// 加 --allow-single-bus 则只跑左腿。默认 --ids 6、跳过 Motorevo 8/14。
#include "biped_hw/p3_runtime.hpp"

#include <cstdio>
#include <string>

int main(int argc, char **argv)
{
  biped_hw::P3Options opt;
  opt.app = biped_hw::P3App::Dual12;
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
