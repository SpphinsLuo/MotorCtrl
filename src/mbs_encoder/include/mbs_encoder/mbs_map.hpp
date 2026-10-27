// biped_hw — mbs_map.hpp
// 8 joint MBS encoders on one BUS-D. Decimal addrs from the user map.
// Hip yaw / ankle roll have no slot (those motors exist on the 12-map).
// Do not program new addresses into the sensors; this is software only.
#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace biped_hw
{

inline constexpr std::size_t kMbsEncoderCount = 8;

enum class MbsJoint : uint8_t
{
  LHipRoll = 0,
  LHipPitch,
  LKneePitch,
  LAnklePitch,
  RHipRoll,
  RHipPitch,
  RKneePitch,
  RAnklePitch,
};

struct MbsMapEntry
{
  MbsJoint joint;
  uint8_t addr;       // BUS-D address, decimal 0..31
  const char *key;    // yaml name
  const char *name;   // print name
  double zero_rad;
  int8_t sign;        // +1 or -1; uncalibrated default +1
};

// Authoritative 8-encoder table. Index is stable. Not keyed by JointId
// (hip roll/pitch are joint-space; motors 4/5/10/11 are outer/inner).
inline constexpr std::array<MbsMapEntry, kMbsEncoderCount> kMbsEncoderMap{{
    {MbsJoint::LHipRoll, 4, "left_hip_roll", "left hip roll", 0.0, 1},
    {MbsJoint::LHipPitch, 5, "left_hip_pitch", "left hip pitch", 0.0, 1},
    {MbsJoint::LKneePitch, 6, "left_knee_pitch", "left knee pitch", 0.0, 1},
    {MbsJoint::LAnklePitch, 7, "left_ankle_pitch", "left ankle pitch", 0.0, 1},
    {MbsJoint::RHipRoll, 10, "right_hip_roll", "right hip roll", 0.0, 1},
    {MbsJoint::RHipPitch, 11, "right_hip_pitch", "right hip pitch", 0.0, 1},
    {MbsJoint::RKneePitch, 12, "right_knee_pitch", "right knee pitch", 0.0, 1},
    {MbsJoint::RAnklePitch, 13, "right_ankle_pitch", "right ankle pitch", 0.0, 1},
}};

struct MbsEncoderConfig
{
  std::string path;
  std::string port;
  std::array<MbsMapEntry, kMbsEncoderCount> encoders = kMbsEncoderMap;
};

inline int mbs_map_index_by_addr(uint8_t addr)
{
  for (std::size_t i = 0; i < kMbsEncoderMap.size(); ++i)
  {
    if (kMbsEncoderMap[i].addr == addr)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

inline int mbs_map_index_by_key(const char *key)
{
  if (key == nullptr)
  {
    return -1;
  }
  for (std::size_t i = 0; i < kMbsEncoderMap.size(); ++i)
  {
    if (std::strcmp(kMbsEncoderMap[i].key, key) == 0)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

inline int mbs_map_self_test()
{
  if (kMbsEncoderMap.size() != kMbsEncoderCount)
  {
    return 1;
  }
  const uint8_t want[kMbsEncoderCount] = {4, 5, 6, 7, 10, 11, 12, 13};
  for (std::size_t i = 0; i < kMbsEncoderCount; ++i)
  {
    if (kMbsEncoderMap[i].addr != want[i] || kMbsEncoderMap[i].sign != 1 ||
        kMbsEncoderMap[i].zero_rad != 0.0)
    {
      return 2;
    }
  }
  if (mbs_map_index_by_addr(3) >= 0 || mbs_map_index_by_addr(8) >= 0 ||
      mbs_map_index_by_addr(9) >= 0 || mbs_map_index_by_addr(14) >= 0 ||
      mbs_map_index_by_addr(0x1F) >= 0)
  {
    return 3;
  }
  return 0;
}

namespace mbs_map_detail
{

inline std::string trim(const std::string &s)
{
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos)
  {
    return {};
  }
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

inline std::string unquote(const std::string &s)
{
  if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                        (s.front() == '\'' && s.back() == '\'')))
  {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

inline bool file_is_reg(const std::string &p)
{
  struct stat st {};
  return ::stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

inline void split_colon_paths(const char *env, std::vector<std::string> &out)
{
  if (env == nullptr || env[0] == '\0')
  {
    return;
  }
  const std::string s(env);
  std::size_t start = 0;
  while (start <= s.size())
  {
    const auto sep = s.find(':', start);
    const std::string p =
        s.substr(start, sep == std::string::npos ? std::string::npos : sep - start);
    if (!p.empty())
    {
      out.push_back(p);
    }
    if (sep == std::string::npos)
    {
      break;
    }
    start = sep + 1;
  }
}

inline std::string exe_share_config_dir()
{
  char buf[PATH_MAX];
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n <= 0)
  {
    return {};
  }
  buf[n] = '\0';
  std::string exe(buf);
  for (int i = 0; i < 3; ++i)
  {
    const auto slash = exe.find_last_of('/');
    if (slash == std::string::npos)
    {
      return {};
    }
    exe.resize(slash);
  }
  return exe + "/share/mbs_encoder/config";
}

inline bool yaml_key(const std::string &line, const char *key, std::string &val)
{
  std::string t = line;
  if (t.size() >= 2 && t[0] == '-' && (t[1] == ' ' || t[1] == '\t'))
  {
    t = trim(t.substr(1));
  }
  const std::size_t klen = std::strlen(key);
  if (t.size() < klen + 1 || t.compare(0, klen, key) != 0 || t[klen] != ':')
  {
    return false;
  }
  val = unquote(trim(t.substr(klen + 1)));
  return true;
}

}  // namespace mbs_map_detail

inline std::string find_mbs_encoder_yaml()
{
  constexpr const char *kName = "mbs_encoder.yaml";
  std::vector<std::string> prefixes;
  mbs_map_detail::split_colon_paths(std::getenv("AMENT_PREFIX_PATH"), prefixes);
  for (const auto &p : prefixes)
  {
    const std::string c = p + "/share/mbs_encoder/config/" + kName;
    if (mbs_map_detail::file_is_reg(c))
    {
      return c;
    }
  }
  const std::string share = mbs_map_detail::exe_share_config_dir();
  if (!share.empty())
  {
    const std::string c = share + "/" + kName;
    if (mbs_map_detail::file_is_reg(c))
    {
      return c;
    }
  }
#ifdef MBS_ENCODER_SOURCE_CONFIG_DIR
  {
    const std::string c = std::string(MBS_ENCODER_SOURCE_CONFIG_DIR) + "/" + kName;
    if (mbs_map_detail::file_is_reg(c))
    {
      return c;
    }
  }
#endif
  return {};
}

inline bool load_mbs_encoder_yaml(const std::string &path, MbsEncoderConfig &out,
                                  std::string &err)
{
  std::ifstream in(path);
  if (!in)
  {
    err = "cannot read " + path;
    return false;
  }
  out.path = path;
  out.encoders = kMbsEncoderMap;
  int cur = -1;
  std::string line;
  while (std::getline(in, line))
  {
    const auto hash = line.find('#');
    if (hash != std::string::npos)
    {
      line = line.substr(0, hash);
    }
    const std::string t = mbs_map_detail::trim(line);
    if (t.empty())
    {
      continue;
    }
    std::string val;
    if (mbs_map_detail::yaml_key(t, "port", val) && t.rfind("-", 0) != 0)
    {
      // Top-level port: (encoder list items are "- name:", not "- port:").
      if (!t.empty() && t[0] != '-')
      {
        out.port = val;
      }
      continue;
    }
    if (mbs_map_detail::yaml_key(t, "name", val))
    {
      cur = mbs_map_index_by_key(val.c_str());
      continue;
    }
    if (cur < 0)
    {
      continue;
    }
    auto &e = out.encoders[static_cast<std::size_t>(cur)];
    if (mbs_map_detail::yaml_key(t, "addr", val))
    {
      char *end = nullptr;
      const unsigned long n = std::strtoul(val.c_str(), &end, 0);
      if (end == val.c_str() || *end != '\0' || n > 31u)
      {
        err = path + ": bad addr for " + e.key;
        return false;
      }
      e.addr = static_cast<uint8_t>(n);
    }
    else if (mbs_map_detail::yaml_key(t, "zero_rad", val))
    {
      char *end = nullptr;
      e.zero_rad = std::strtod(val.c_str(), &end);
      if (end == val.c_str() || *end != '\0')
      {
        err = path + ": bad zero_rad for " + e.key;
        return false;
      }
    }
    else if (mbs_map_detail::yaml_key(t, "sign", val))
    {
      char *end = nullptr;
      const long n = std::strtol(val.c_str(), &end, 0);
      if (end == val.c_str() || *end != '\0' || (n != 1 && n != -1))
      {
        err = path + ": sign must be +1 or -1 for " + e.key;
        return false;
      }
      e.sign = static_cast<int8_t>(n);
    }
  }
  return true;
}

inline bool load_mbs_encoder_config(MbsEncoderConfig &out, std::string &err)
{
  out = {};
  out.encoders = kMbsEncoderMap;
  const std::string path = find_mbs_encoder_yaml();
  if (path.empty())
  {
    return true;
  }
  return load_mbs_encoder_yaml(path, out, err);
}

}  // namespace biped_hw
