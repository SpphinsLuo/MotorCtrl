// biped_hw — dm_codec.hpp
// 达妙 MIT / POS_VEL / VEL / POS_FORCE 编解码、使能/失能（0xFC/0xFD）、设零（0xFE）与 RID
// 读写（含 CTRL_MODE）。P1 读限幅、P2 闭环阶梯的协议核心；不要 vendor Motor_Control。
#pragma once

// Damiao MIT + POS_VEL + VEL + POS_FORCE + enable/disable + set-zero + optional RID read.
// Protocol: kit-miao/motor-sdk, datasheet, and all_motor.h dm_control_* packing.
// Do not vendor Motor_Control.

#include "biped_hw/types.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace biped_hw
{

inline constexpr uint8_t kDmEnableCmd = 0xFC;
inline constexpr uint8_t kDmDisableCmd = 0xFD;
inline constexpr uint8_t kDmSetZeroCmd = 0xFE;  // MIT: save current pos as firmware zero
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

// Datasheet DM-J10422P-2EC: CTRL_MODE (0x0A) encoding. Command frames for
// POS_VEL / VEL / POS_FORCE use ESC_ID plus a CAN-ID offset; MIT stays on ESC_ID.
// MIT payloads on ESC_ID are ignored unless CTRL_MODE is 1.
inline constexpr uint32_t kDmMitMode = 1;
inline constexpr uint32_t kDmPosVelMode = 2;
inline constexpr uint32_t kDmVelMode = 3;
inline constexpr uint32_t kDmPosForceMode = 4;  // datasheet FORCE_POS

// all_motor.h POS_MODE / SPEED_MODE / POSI_MODE. Standard 11-bit CAN IDs.
inline constexpr uint32_t kDmPosVelCanOff = 0x100u;
inline constexpr uint32_t kDmVelCanOff = 0x200u;
inline constexpr uint32_t kDmPosForceCanOff = 0x300u;

// POS_FORCE / FORCE_POS: v_des uint16 = vel_limit_rad_s * 100, cap 10000 (0–100 rad/s).
// i_des uint16 = torque_ratio * 10000, cap 10000 (0–1 per-unit current).
inline constexpr float kDmPosForceVelMax = 100.f;
inline constexpr float kDmPosForceVelScale = 100.f;
inline constexpr float kDmPosForceIScale = 10000.f;
inline constexpr uint16_t kDmPosForceU16Max = 10000;

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

// all_motor.h Motor_Control::set_zero_position → control_cmd(id, 0xFE).
// Saves current position as motor firmware zero. Does not enable.
inline void pack_set_zero(uint8_t out[8])
{
  for (int i = 0; i < 7; ++i)
  {
    out[i] = 0xFF;
  }
  out[7] = kDmSetZeroCmd;
}

// Damiao MIT wire range. CLI caps in P2/P3/P6 use these, not a tighter hold limit.
inline constexpr float kMitKpMax = 500.f;
inline constexpr float kMitKdMax = 5.f;

// MIT command (sent to motor CAN ID / ESC_ID):
//   q 16, dq 12, kp 12, kd 12, tau 12. kp 0-500, kd 0-5. tau is ±TMAX.
inline void pack_mit(uint8_t out[8], float kp, float kd, float q, float dq, float tau,
                     const LimitParam &lim)
{
  const uint16_t kp_uint = float_to_uint(kp, 0.f, kMitKpMax, 12);
  const uint16_t kd_uint = float_to_uint(kd, 0.f, kMitKdMax, 12);
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

inline bool dm_ctrl_mode_ok(uint32_t mode)
{
  return mode >= kDmMitMode && mode <= kDmPosForceMode;
}

// Command arbitration ID for CTRL_MODE. Enable/disable/RID stay on ESC_ID / 0x7FF.
inline uint32_t dm_cmd_can_id(uint32_t ctrl_mode, uint8_t esc_id)
{
  const uint32_t id = static_cast<uint32_t>(esc_id);
  switch (ctrl_mode)
  {
  case kDmPosVelMode:
    return kDmPosVelCanOff + id;
  case kDmVelMode:
    return kDmVelCanOff + id;
  case kDmPosForceMode:
    return kDmPosForceCanOff + id;
  default:
    return id;
  }
}

// POS_VEL (CTRL_MODE=2): CAN ID 0x100+ESC_ID.
// D[0..3] pos float32 LE (rad), D[4..7] vel_limit float32 LE (rad/s, trapezoid max).
inline void pack_pos_vel(uint8_t out[8], float pos, float vel)
{
  std::memcpy(out, &pos, sizeof(float));
  std::memcpy(out + 4, &vel, sizeof(float));
}

inline void unpack_pos_vel_cmd(const uint8_t in[8], float &pos, float &vel)
{
  std::memcpy(&pos, in, sizeof(float));
  std::memcpy(&vel, in + 4, sizeof(float));
}

// VEL (CTRL_MODE=3): CAN ID 0x200+ESC_ID.
// D[0..3] vel float32 LE (rad/s, signed). D[4..7] = 0.
inline void pack_vel(uint8_t out[8], float vel)
{
  std::memcpy(out, &vel, sizeof(float));
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
}

inline void unpack_vel_cmd(const uint8_t in[8], float &vel)
{
  std::memcpy(&vel, in, sizeof(float));
}

inline uint16_t dm_pos_force_vel_u16(float vel_limit)
{
  float v = vel_limit;
  if (v < 0.f)
  {
    v = 0.f;
  }
  if (v > kDmPosForceVelMax)
  {
    v = kDmPosForceVelMax;
  }
  const unsigned scaled = static_cast<unsigned>(v * kDmPosForceVelScale + 0.5f);
  if (scaled > kDmPosForceU16Max)
  {
    return kDmPosForceU16Max;
  }
  return static_cast<uint16_t>(scaled);
}

inline uint16_t dm_pos_force_i_u16(float torque_ratio)
{
  float r = torque_ratio;
  if (r < 0.f)
  {
    r = 0.f;
  }
  if (r > 1.f)
  {
    r = 1.f;
  }
  const unsigned scaled = static_cast<unsigned>(r * kDmPosForceIScale + 0.5f);
  if (scaled > kDmPosForceU16Max)
  {
    return kDmPosForceU16Max;
  }
  return static_cast<uint16_t>(scaled);
}

// POS_FORCE / FORCE_POS (CTRL_MODE=4): CAN ID 0x300+ESC_ID.
// D[0..3] pos float32 LE, D[4..5] vel_limit uint16 LE (*100), D[6..7] i_des uint16 LE (*10000).
inline void pack_pos_force_raw(uint8_t out[8], float pos, uint16_t vel_u16, uint16_t i_u16)
{
  uint16_t vu = vel_u16;
  uint16_t iu = i_u16;
  if (vu > kDmPosForceU16Max)
  {
    vu = kDmPosForceU16Max;
  }
  if (iu > kDmPosForceU16Max)
  {
    iu = kDmPosForceU16Max;
  }
  std::memcpy(out, &pos, sizeof(float));
  std::memcpy(out + 4, &vu, sizeof(uint16_t));
  std::memcpy(out + 6, &iu, sizeof(uint16_t));
}

inline void pack_pos_force(uint8_t out[8], float pos, float vel_limit, float torque_ratio)
{
  pack_pos_force_raw(out, pos, dm_pos_force_vel_u16(vel_limit), dm_pos_force_i_u16(torque_ratio));
}

inline void unpack_pos_force_cmd(const uint8_t in[8], float &pos, float &vel_limit,
                                 float &torque_ratio)
{
  uint16_t vu = 0;
  uint16_t iu = 0;
  std::memcpy(&pos, in, sizeof(float));
  std::memcpy(&vu, in + 4, sizeof(uint16_t));
  std::memcpy(&iu, in + 6, sizeof(uint16_t));
  vel_limit = static_cast<float>(vu) / kDmPosForceVelScale;
  torque_ratio = static_cast<float>(iu) / kDmPosForceIScale;
}

inline bool is_dm_register_payload(const uint8_t data[8], uint8_t dlc)
{
  return dlc >= 8 && (data[2] == 0x33 || data[2] == 0x55);
}

// Feedback (CAN ID = MST_ID) is the same in MIT / POS_VEL / VEL / POS_FORCE.
// D[0] = ESC_ID | (ERR<<4), then pos 16 / vel 12 / tau 12. Not a per-mode frame.
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

inline void pack_ctrl_mode_write(uint8_t out[8], uint16_t esc_id, uint32_t mode)
{
  pack_register_write_u32(out, esc_id, static_cast<uint8_t>(DmRid::CtrlMode), mode);
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
  kp = uint_to_float(kp_uint, 0.f, kMitKpMax, 12);
  kd = uint_to_float(kd_uint, 0.f, kMitKdMax, 12);
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
  uint8_t z[8] = {};
  pack_set_zero(z);
  // all_motor.h control_cmd(id, 0xFE) → {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}
  if (z[0] != 0xFF || z[1] != 0xFF || z[2] != 0xFF || z[3] != 0xFF ||
      z[4] != 0xFF || z[5] != 0xFF || z[6] != 0xFF || z[7] != kDmSetZeroCmd)
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

  if (!dm_ctrl_mode_ok(kDmMitMode) || !dm_ctrl_mode_ok(kDmPosForceMode) ||
      dm_ctrl_mode_ok(0) || dm_ctrl_mode_ok(5))
  {
    return false;
  }
  if (dm_cmd_can_id(kDmMitMode, 12) != 12 ||
      dm_cmd_can_id(kDmPosVelMode, 12) != 0x10Cu ||
      dm_cmd_can_id(kDmVelMode, 12) != 0x20Cu ||
      dm_cmd_can_id(kDmPosForceMode, 12) != 0x30Cu)
  {
    return false;
  }

  uint8_t pv[8] = {};
  pack_pos_vel(pv, 1.f, 2.f);
  // 1.0f LE 00 00 80 3F, 2.0f LE 00 00 00 40 (all_motor.h memcpy layout).
  if (pv[0] != 0x00 || pv[1] != 0x00 || pv[2] != 0x80 || pv[3] != 0x3F ||
      pv[4] != 0x00 || pv[5] != 0x00 || pv[6] != 0x00 || pv[7] != 0x40)
  {
    return false;
  }
  float pv_p = 0.f, pv_v = 0.f;
  unpack_pos_vel_cmd(pv, pv_p, pv_v);
  if (std::fabs(pv_p - 1.f) > 1e-6f || std::fabs(pv_v - 2.f) > 1e-6f)
  {
    return false;
  }

  uint8_t velb[8] = {};
  pack_vel(velb, -1.f);
  // -1.0f LE 00 00 80 BF, D[4..7] = 0.
  if (velb[0] != 0x00 || velb[1] != 0x00 || velb[2] != 0x80 || velb[3] != 0xBF ||
      velb[4] != 0 || velb[5] != 0 || velb[6] != 0 || velb[7] != 0)
  {
    return false;
  }
  float vel_w = 0.f;
  unpack_vel_cmd(velb, vel_w);
  if (std::fabs(vel_w + 1.f) > 1e-6f)
  {
    return false;
  }

  uint8_t pf[8] = {};
  pack_pos_force(pf, 1.f, 20.f, 0.5f);
  // pos 1.0f, vel 20 rad/s → 2000 = 0x07D0 LE, i 0.5 → 5000 = 0x1388 LE.
  if (pf[0] != 0x00 || pf[1] != 0x00 || pf[2] != 0x80 || pf[3] != 0x3F ||
      pf[4] != 0xD0 || pf[5] != 0x07 || pf[6] != 0x88 || pf[7] != 0x13)
  {
    return false;
  }
  float pf_p = 0.f, pf_v = 0.f, pf_i = 0.f;
  unpack_pos_force_cmd(pf, pf_p, pf_v, pf_i);
  if (std::fabs(pf_p - 1.f) > 1e-6f || std::fabs(pf_v - 20.f) > 1e-4f ||
      std::fabs(pf_i - 0.5f) > 1e-4f)
  {
    return false;
  }
  pack_pos_force(pf, 0.f, 250.f, 5.f);
  unpack_pos_force_cmd(pf, pf_p, pf_v, pf_i);
  if (std::fabs(pf_v - 100.f) > 1e-4f || std::fabs(pf_i - 1.f) > 1e-4f)
  {
    return false;
  }
  uint8_t pf_raw[8] = {};
  pack_pos_force_raw(pf_raw, 0.f, 2000, 5000);
  pack_pos_force(pf, 0.f, 20.f, 0.5f);
  if (std::memcmp(pf, pf_raw, 8) != 0)
  {
    return false;
  }

  uint8_t wr2[8] = {};
  pack_ctrl_mode_write(wr2, 6, kDmPosVelMode);
  if (wr2[0] != 6 || wr2[2] != 0x55 || wr2[3] != 10 || wr2[4] != 2 || wr2[5] != 0)
  {
    return false;
  }
  pack_ctrl_mode_write(wr2, 12, kDmVelMode);
  if (wr2[0] != 12 || wr2[4] != 3)
  {
    return false;
  }
  pack_ctrl_mode_write(wr2, 3, kDmPosForceMode);
  if (wr2[0] != 3 || wr2[4] != 4)
  {
    return false;
  }
  return true;
}

}  // namespace biped_hw
