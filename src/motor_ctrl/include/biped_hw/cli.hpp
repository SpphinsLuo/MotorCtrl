// biped_hw — cli.hpp
// Shared CLI helpers used by phase test programs. No app-to-app includes.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace biped_hw
{

inline std::string trim_copy(std::string s)
{
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos)
  {
    return {};
  }
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

inline const char *need_arg(int &i, int argc, char **argv, const char *flag, std::string &err)
{
  if (i + 1 >= argc)
  {
    err = std::string(flag) + " needs a value";
    return nullptr;
  }
  return argv[++i];
}

inline unsigned parse_u(const char *s, bool &ok)
{
  char *end = nullptr;
  const unsigned long v = std::strtoul(s, &end, 0);
  ok = end != s && end != nullptr && *end == '\0';
  return static_cast<unsigned>(v);
}

// Overlay buses.left.port / buses.right.port from motor_map.yaml-style files.
// Returns true if at least one port key was found.
inline bool yaml_bus_ports(const std::string &path, std::string *left, std::string *right)
{
  std::ifstream in(path);
  if (!in)
  {
    return false;
  }
  std::string line;
  int section = 0;
  bool got = false;
  while (std::getline(in, line))
  {
    const auto hash = line.find('#');
    if (hash != std::string::npos)
    {
      line = line.substr(0, hash);
    }
    const std::string t = trim_copy(line);
    if (t.empty())
    {
      continue;
    }
    if (t.find("left:") == 0)
    {
      section = 1;
      continue;
    }
    if (t.find("right:") == 0)
    {
      section = 2;
      continue;
    }
    if (t.find("port:") != 0)
    {
      continue;
    }
    const std::string v = trim_copy(t.substr(5));
    if (section == 1 && left != nullptr)
    {
      *left = v;
      got = true;
    }
    else if (section == 2 && right != nullptr)
    {
      *right = v;
      got = true;
    }
  }
  return got;
}

}  // namespace biped_hw
