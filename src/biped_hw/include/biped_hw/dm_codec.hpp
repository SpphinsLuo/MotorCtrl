// biped_hw — dm_codec.hpp
// 达妙 MIT 编解码、使能/失能（0xFC/0xFD）与 RID 读写（含 CTRL_MODE）。
// P1 读限幅、P2 闭环阶梯的协议核心；不要 vendor Motor_Control。
#pragma once

// Damiao MIT + enable/disable + optional RID read.
// Protocol: kit-miao/motor-sdk (damiao.h / DM_CAN.py) and all_motor.h
// dm_control_mit bit packing. Do not vendor Motor_Control.

#include "biped_hw/types.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace biped_hw
{

inline constexpr uint8_t kDmEnableCmd = 0xFC;
inline constexpr uint8_t kDmDisableCmd = 0xFD;
inline constexpr uint32_t kDmRegCanId = 0x7FFu;

enum class DmRid : uint8_t
{
  KtValue = 1,    // float: torque constant
  OcValue = 3,    // float: overcurrent fraction (0,1)
  Timeout = 9,    // uint32: comm-loss counts (50 us)
  CtrlMode = 10,  // uint32: 1=MIT, 2=POS_VEL, 3=VEL, 4=POS_FORCE
  SwVer = 14,     // uint32
  Gr = 20,        // float: gear ratio
  Pmax = 21,
  Vmax = 22,
  Tmax = 23,
  Gref = 30,      // float: gear torque efficiency (0,1]
  IBase = 59,     // float: phase-current peak reference
  Vbus = 60,      // float: bus voltage
};

// Datasheet DM-J10422P-2EC: CTRL_MODE (0x0A) encoding. MIT payloads on ESC_ID
// are ignored unless this is 1.
inline constexpr uint32_t kDmMitMode = 1;
inline constexpr uint32_t kDmPosVelMode = 2;
inline constexpr uint32_t kDmVelMode = 3;
inline constexpr uint32_t kDmPosForceMode = 4;

struct DmFeedback
{
  uint32_t rx_can_id = 0;
  uint8_t dlc = 0;
  uint8_t esc_id = 0;
  uint8_t err = 0;
  float q = 0.f;
  float dq = 0.f;
  float tau = 0.f;
  bool has_temp = false;
  uint8_t t_mos = 0;
  uint8_t t_coil = 0;
};

struct DmRegisterReply
{
  uint16_t esc_id = 0;
  uint8_t rid = 0;
  bool is_uint32 = false;
  float f = 0.f;
  uint32_t u = 0;
};

inline uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits)
{
  if (x < xmin)
  {
    x = xmin;
  }
  if (x > xmax)
  {
    x = xmax;
  }
  const float span = xmax - xmin;
  const float data_norm = (x - xmin) / span;
  return static_cast<uint16_t>(data_norm * static_cast<float>((1u << bits) - 1u));
}

inline float uint_to_float(uint16_t x, float xmin, float xmax, uint8_t bits)
{
  const float span = xmax - xmin;
  const float data_norm = static_cast<float>(x) / static_cast<float>((1u << bits) - 1u);
  return data_norm * span + xmin;
}

inline void pack_enable(uint8_t out[8])
{
  for (int i = 0; i < 7; ++i)
  {
    out[i] = 0xFF;
  }
  out[7] = kDmEnableCmd;
}

inline void pack_disable(uint8_t out[8])
{
  for (int i = 0; i < 7; ++i)
  {
    out[i] = 0xFF;
  }
  out[7] = kDmDisableCmd;
}

// MIT command (sent to motor CAN ID / ESC_ID):
//   q 16, dq 12, kp 12, kd 12, tau 12. kp 0-500, kd 0-5.
inline void pack_mit(uint8_t out[8], float kp, float kd, float q, float dq, float tau,
                     const LimitParam &lim)
{
  const uint16_t kp_uint = float_to_uint(kp, 0.f, 500.f, 12);
  const uint16_t kd_uint = float_to_uint(kd, 0.f, 5.f, 12);
  const uint16_t q_uint = float_to_uint(q, -lim.q_max, lim.q_max, 16);
  const uint16_t dq_uint = float_to_uint(dq, -lim.dq_max, lim.dq_max, 12);
  const uint16_t tau_uint = float_to_uint(tau, -lim.tau_max, lim.tau_max, 12);

  out[0] = static_cast<uint8_t>((q_uint >> 8) & 0xff);
  out[1] = static_cast<uint8_t>(q_uint & 0xff);
  out[2] = static_cast<uint8_t>(dq_uint >> 4);
  out[3] = static_cast<uint8_t>(((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf));
  out[4] = static_cast<uint8_t>(kp_uint & 0xff);
  out[5] = static_cast<uint8_t>(kd_uint >> 4);
  out[6] = static_cast<uint8_t>(((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf));
  out[7] = static_cast<uint8_t>(tau_uint & 0xff);
}

inline bool is_dm_register_payload(const uint8_t data[8], uint8_t dlc)
{
  return dlc >= 8 && (data[2] == 0x33 || data[2] == 0x55);
}

// MIT feedback (CAN ID = MST_ID). D[0] = ESC_ID | (ERR<<4), then pos/vel/tau.
inline bool unpack_mit_feedback(const uint8_t data[8], uint8_t dlc, uint32_t rx_can_id,
                                const LimitParam &lim, DmFeedback &out)
{
  if (dlc < 6)
  {
    return false;
  }
  if (is_dm_register_payload(data, dlc))
  {
    return false;
  }

  out = {};
  out.rx_can_id = rx_can_id;
  out.dlc = dlc;
  out.esc_id = static_cast<uint8_t>(data[0] & 0x0F);
  out.err = static_cast<uint8_t>((data[0] >> 4) & 0x0F);

  const uint16_t q_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);
  const uint16_t dq_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4));
  const uint16_t tau_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5]);

  out.q = uint_to_float(q_uint, -lim.q_max, lim.q_max, 16);
  out.dq = uint_to_float(dq_uint, -lim.dq_max, lim.dq_max, 12);
  out.tau = uint_to_float(tau_uint, -lim.tau_max, lim.tau_max, 12);

  if (dlc >= 8)
  {
    out.has_temp = true;
    out.t_mos = data[6];
    out.t_coil = data[7];
  }
  return true;
}

inline bool dm_rid_is_uint32(uint8_t rid)
{
  return (rid >= 7 && rid <= 10) || (rid >= 13 && rid <= 16) ||
         (rid >= 35 && rid <= 36);
}

// Official SDK (all_motor.h) stores ESC_ID as binary (12 → 0x0C). Some Damiao
// 上位机 UIs type decimal 12 and write 0x12 (BCD). IDs 1–9 are identical;
// 10/11/12 become 0x10/0x11/0x12 vs 0x0A/0x0B/0x0C. Not --can-id 18.
inline constexpr uint8_t dm_esc_id_as_bcd(uint8_t decimal_id)
{
  return static_cast<uint8_t>(static_cast<unsigned>((decimal_id / 10u) << 4) |
                              static_cast<unsigned>(decimal_id % 10u));
}

inline void pack_register_read(uint8_t out[8], uint16_t esc_id, uint8_t rid)
{
  out[0] = static_cast<uint8_t>(esc_id & 0xff);
  out[1] = static_cast<uint8_t>((esc_id >> 8) & 0xff);
  out[2] = 0x33;
  out[3] = rid;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
}

// Write register (CAN ID 0x7FF). uint32 RIDs (incl. CTRL_MODE=10) are LE.
// Matches all_motor.h write_motor_param and the 10422 datasheet 写入参数 frame.
inline void pack_register_write_u32(uint8_t out[8], uint16_t esc_id, uint8_t rid,
                                    uint32_t value)
{
  out[0] = static_cast<uint8_t>(esc_id & 0xff);
  out[1] = static_cast<uint8_t>((esc_id >> 8) & 0xff);
  out[2] = 0x55;
  out[3] = rid;
  out[4] = static_cast<uint8_t>(value & 0xff);
  out[5] = static_cast<uint8_t>((value >> 8) & 0xff);
  out[6] = static_cast<uint8_t>((value >> 16) & 0xff);
  out[7] = static_cast<uint8_t>((value >> 24) & 0xff);
}

inline const char *dm_ctrl_mode_name(uint32_t mode)
{
  switch (mode)
  {
  case kDmMitMode:
    return "MIT";
  case kDmPosVelMode:
    return "POS_VEL";
  case kDmVelMode:
    return "VEL";
  case kDmPosForceMode:
    return "POS_FORCE";
  default:
    return "?";
  }
}

// Inverse of pack_mit — for TX hex dumps / self-check, not used on the bus.
inline void unpack_mit_cmd(const uint8_t in[8], const LimitParam &lim, float &kp,
                           float &kd, float &q, float &dq, float &tau)
{
  const uint16_t q_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
  const uint16_t dq_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(in[2]) << 4) | (in[3] >> 4));
  const uint16_t kp_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(in[3] & 0x0F) << 8) | in[4]);
  const uint16_t kd_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(in[5]) << 4) | (in[6] >> 4));
  const uint16_t tau_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(in[6] & 0x0F) << 8) | in[7]);
  kp = uint_to_float(kp_uint, 0.f, 500.f, 12);
  kd = uint_to_float(kd_uint, 0.f, 5.f, 12);
  q = uint_to_float(q_uint, -lim.q_max, lim.q_max, 16);
  dq = uint_to_float(dq_uint, -lim.dq_max, lim.dq_max, 12);
  tau = uint_to_float(tau_uint, -lim.tau_max, lim.tau_max, 12);
}

inline bool unpack_register(const uint8_t data[8], uint8_t dlc, DmRegisterReply &out)
{
  if (!is_dm_register_payload(data, dlc))
  {
    return false;
  }
  out = {};
  out.esc_id = static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
  out.rid = data[3];
  out.is_uint32 = dm_rid_is_uint32(out.rid);
  if (out.is_uint32)
  {
    out.u = static_cast<uint32_t>(data[4]) | (static_cast<uint32_t>(data[5]) << 8) |
            (static_cast<uint32_t>(data[6]) << 16) | (static_cast<uint32_t>(data[7]) << 24);
    out.f = static_cast<float>(out.u);
  }
  else
  {
    std::memcpy(&out.f, data + 4, sizeof(float));
  }
  return true;
}

inline bool dm_codec_self_check()
{
  uint8_t en[8] = {};
  pack_enable(en);
  if (en[7] != kDmEnableCmd)
  {
    return false;
  }
  uint8_t dis[8] = {};
  pack_disable(dis);
  if (dis[7] != kDmDisableCmd)
  {
    return false;
  }

  const LimitParam lim = kLimitDm10010L;
  uint8_t mit[8] = {};
  pack_mit(mit, 2.f, 0.5f, 0.f, 0.f, 0.f, lim);
  const uint16_t q_cmd =
      static_cast<uint16_t>((static_cast<uint16_t>(mit[0]) << 8) | mit[1]);
  if (q_cmd < 32700 || q_cmd > 32800)
  {
    return false;
  }

  // Synthetic MST_ID feedback: ESC 3, ERR 0, q/dq/tau ~ 0, temps present.
  const uint8_t fb[8] = {0x03, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 40, 35};
  DmFeedback decoded{};
  if (!unpack_mit_feedback(fb, 8, kMasterId, lim, decoded))
  {
    return false;
  }
  if (decoded.esc_id != 3 || decoded.err != 0 || !decoded.has_temp)
  {
    return false;
  }
  // ESC 12 (right knee) must decode from D[0] nibble; also from slave CAN 12.
  const uint8_t fb12[8] = {0x0C, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 40, 35};
  if (!unpack_mit_feedback(fb12, 8, kMasterId, lim, decoded) || decoded.esc_id != 12)
  {
    return false;
  }
  if (!unpack_mit_feedback(fb12, 8, 12, lim, decoded) || decoded.esc_id != 12)
  {
    return false;
  }
  if (decoded.t_mos != 40 || decoded.t_coil != 35)
  {
    return false;
  }
  if (std::fabs(decoded.q) > 0.01f || std::fabs(decoded.dq) > 0.05f ||
      std::fabs(decoded.tau) > 0.2f)
  {
    return false;
  }

  uint8_t reg[8] = {};
  pack_register_read(reg, 3, static_cast<uint8_t>(DmRid::Pmax));
  if (reg[0] != 3 || reg[2] != 0x33 || reg[3] != 21)
  {
    return false;
  }
  pack_register_read(reg, 12, static_cast<uint8_t>(DmRid::CtrlMode));
  if (reg[0] != 0x0C || dm_esc_id_as_bcd(12) != 0x12 || dm_esc_id_as_bcd(9) != 0x09)
  {
    return false;
  }

  uint8_t wr[8] = {};
  pack_register_write_u32(wr, 6, static_cast<uint8_t>(DmRid::CtrlMode), kDmMitMode);
  if (wr[0] != 6 || wr[1] != 0 || wr[2] != 0x55 || wr[3] != 10 || wr[4] != 1 ||
      wr[5] != 0 || wr[6] != 0 || wr[7] != 0)
  {
    return false;
  }

  // kp=80 must not collapse to 0. 80/500 * 4095 ≈ 655.
  uint8_t mit80[8] = {};
  pack_mit(mit80, 80.f, 1.f, 0.f, 0.f, 0.f, lim);
  const uint16_t kp80 =
      static_cast<uint16_t>((static_cast<uint16_t>(mit80[3] & 0x0F) << 8) | mit80[4]);
  if (kp80 < 650 || kp80 > 660)
  {
    return false;
  }
  float kp_w = 0.f, kd_w = 0.f, q_w = 0.f, dq_w = 0.f, tau_w = 0.f;
  unpack_mit_cmd(mit80, lim, kp_w, kd_w, q_w, dq_w, tau_w);
  if (std::fabs(kp_w - 80.f) > 1.f || std::fabs(kd_w - 1.f) > 0.02f)
  {
    return false;
  }

  // Bench TX: kp=80 kd=1 q≈-1.211 tau=0 with PMAX=12.566 VMAX=20 TMAX=500.
  const LimitParam lim10422{12.566f, 20.f, 500.f};
  const uint8_t bench[8] = {0x73, 0xA9, 0x7F, 0xF2, 0x8F, 0x33, 0x37, 0xFF};
  unpack_mit_cmd(bench, lim10422, kp_w, kd_w, q_w, dq_w, tau_w);
  if (std::fabs(kp_w - 80.f) > 1.f || std::fabs(kd_w - 1.f) > 0.02f ||
      std::fabs(q_w + 1.21125f) > 0.002f || std::fabs(tau_w) > 0.25f)
  {
    return false;
  }
  return true;
}

}  // namespace biped_hw
