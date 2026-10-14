// 右腿6电机协同测试程序
// 缺电机超时记 drop，不阻塞；退出对已开总线发 P5 disable-all 0xFD。
// 通信测试请加 --comms-only（不发 0xFC）。默认 --ids 12、右口 ACM1。
#include "biped_hw/p3_runtime.hpp"

#include <cstdio>
#include <string>

int main(int argc, char **argv)
{
  biped_hw::P3Options opt;
  opt.app = biped_hw::P3App::Right6;
  opt.ids = {12};
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
