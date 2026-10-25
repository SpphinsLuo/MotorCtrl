// biped_hw — motorevo_codec.hpp
// Motorevo HA60（左 CAN 8 / 右 CAN 14）编解码。协议以官方为准：
//   《Motorevo驱动用户指南 V3.5》§6.3.1 伺服帧 + Motorevo_SDK Motorevo_Test/Bsp/bsp_Motorevo.c
// 伺服默认给 P1 hold / P2 track；力位混合打包保留但不在这两个 app 里切换。
// 线上单位是 rad / rad/s（默认 ±12.5 rad、±10 rad/s），不是度。all_motor.h 的 deg_to_rad
// 只是旧调用侧包装，不是 CAN 映射。使能/失能 0xFC / 0xFD。
// 设零：§ SetzeroPosition = 标准帧、ID=MotorID、FF×7+FE（Torque4 才 +100）。
// 掉电保持：§6.9.9 SaveFlash，标准帧 ID=0x600+MotorID，
// 67 00 00 00 00 00 15 76（2024.9.25 后固件）。发送路径不调用 receive()。
#pragma once

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/types.hpp"

#include <cmath>
#include <cstdint>

namespace biped_hw
{

// 打包范围与官方默认 CAN COM Theta/Velocity/Kp/Kd/Ki 一致（指南 §6.3.1 / bsp_Motorevo.h）。
inline constexpr float kMotorevoQMax = 12.5f;
inline constexpr float kMotorevoDqMax = 10.f;
inline constexpr float kMotorevoTauMax = 50.f;
inline constexpr float kMotorevoPosKpMax = 250.f;
inline constexpr float kMotorevoPosKdMax = 50.f;
inline constexpr float kMotorevoVelKpMax = 250.f;
inline constexpr float kMotorevoVelKdMax = 50.f;
inline constexpr float kMotorevoVelKiMax = 0.05f;

// all_motor.h Motor_Control::enable/disable for HA60 used the same opcodes.
inline constexpr uint8_t kMotorevoEnableCmd = kDmEnableCmd;    // 0xFC
inline constexpr uint8_t kMotorevoDisableCmd = kDmDisableCmd;  // 0xFD
inline constexpr uint8_t kMotorevoSetZeroCmd = kDmSetZeroCmd;  // 0xFE V1

// all_motor.h Motorevo_Status. D[0] of receive_evo().
enum class MotorevoStatus : uint8_t
{
  Reset = 0,
  Servo = 1,
  TpMix = 2,
  Velocity = 3,
  Torque = 4,
  Torque4 = 5,
};

struct MotorevoFeedback
{
  uint32_t rx_can_id = 0;
  uint8_t dlc = 0;
  float q = 0.f;
  float dq = 0.f;
  float tau = 0.f;
  uint8_t status = 0;
  uint8_t err_code = 0;
};

// HA60 lab used Motor_Control::enable/disable → control_cmd(slave, 0xFC/0xFD).
inline void pack_motorevo_enable(uint8_t out[8]) { pack_enable(out); }
inline void pack_motorevo_disable(uint8_t out[8]) { pack_disable(out); }

// V1: all_motor.h Motorevo_set_zero_position → {0xFF×7, 0xFE}. Same MIT opcode as Damiao.
inline void pack_motorevo_set_zero_v1(uint8_t out[8]) { pack_set_zero(out); }

// Historical all_motor.h Motorevo_set_zero_position_V2. Wrong CAN ID and
// bytes vs §6.9.9 (must go to 0x600+ID, Byte1=index, Byte6=0x15 write).
inline void pack_motorevo_set_zero_v2(uint8_t out[8])
{
  out[0] = 0x67;
  out[1] = 0x06;
  out[2] = 0xFF;
  out[3] = 0x00;
  out[4] = 0x00;
  out[5] = 0x00;
  out[6] = 0x00;
  out[7] = 0x76;
}

// Official SetzeroPosition on MotorID. Persist is a separate SaveFlash.
inline void pack_motorevo_set_zero(uint8_t out[8]) { pack_motorevo_set_zero_v1(out); }

// §6.9.9 2024.9.25+ : param TX on 0x600+MotorID. MachineAngleOffset index 0x23.
inline constexpr uint32_t kMotorevoParamCanOff = 0x600u;
inline constexpr uint8_t kMotorevoParamHdr = 0x67;
inline constexpr uint8_t kMotorevoParamTail = 0x76;
inline constexpr uint8_t kMotorevoParamRead = 0x04;
inline constexpr uint8_t kMotorevoParamWrite = 0x15;
inline constexpr uint8_t kMotorevoIdxSaveFlash = 0x00;
inline constexpr uint8_t kMotorevoIdxMachineAngleOffset = 0x23;

inline uint32_t motorevo_param_can_id(uint8_t motor_id)
{
  return kMotorevoParamCanOff + static_cast<uint32_t>(motor_id);
}

inline void pack_motorevo_param_write(uint8_t out[8], uint8_t index, const uint8_t data[4])
{
  out[0] = kMotorevoParamHdr;
  out[1] = index;
  out[2] = data != nullptr ? data[0] : 0;
  out[3] = data != nullptr ? data[1] : 0;
  out[4] = data != nullptr ? data[2] : 0;
  out[5] = data != nullptr ? data[3] : 0;
  out[6] = kMotorevoParamWrite;
  out[7] = kMotorevoParamTail;
}

// SaveFlash: index 0x00, data unused, write 0x15. Byte2–5 = 0.
inline void pack_motorevo_save_flash(uint8_t out[8])
{
  const uint8_t z[4] = {};
  pack_motorevo_param_write(out, kMotorevoIdxSaveFlash, z);
}

// 伺服帧（官方 §6.3.1 / Motorevo_motor_servo_control_mode）：
//   Byte0-1 θ_ref 16bit  rad  默认 ±12.5（不是 5.0，q≈5 不是饱和）
//   Byte2   V_ref  8bit  rad/s 过渡速度上限（到达目标的最大转速），不是速度指令
//   Byte3   位置 Kp 8bit  0–250
//   Byte4   位置 Kd 8bit  0–50
//   Byte5   速度 Kp 8bit  0–250  级联速度环；0 则跟不上位置台阶
//   Byte6   速度 Kd 8bit  0–50
//   Byte7   速度 Ki 8bit  0–0.05
// V_ref=0 或 vel_kp=0 → 官方三闭环无法走向 θ_ref。P1 hold 用 0 是故意的。
inline void pack_motorevo_servo(uint8_t out[8], float q_rad, float pos_kp, float pos_kd,
                                float vel_limit_radps, float vel_kp, float vel_kd,
                                float vel_ki)
{
  const uint16_t t_p = float_to_uint(q_rad, -kMotorevoQMax, kMotorevoQMax, 16);
  const uint16_t t_v = float_to_uint(vel_limit_radps, -kMotorevoDqMax, kMotorevoDqMax, 8);
  const uint16_t t_p_kp = float_to_uint(pos_kp, 0.f, kMotorevoPosKpMax, 8);
  const uint16_t t_p_kd = float_to_uint(pos_kd, 0.f, kMotorevoPosKdMax, 8);
  const uint16_t t_v_kp = float_to_uint(vel_kp, 0.f, kMotorevoVelKpMax, 8);
  const uint16_t t_v_kd = float_to_uint(vel_kd, 0.f, kMotorevoVelKdMax, 8);
  const uint16_t t_v_ki = float_to_uint(vel_ki, 0.f, kMotorevoVelKiMax, 8);

  out[0] = static_cast<uint8_t>((t_p >> 8) & 0xff);
  out[1] = static_cast<uint8_t>(t_p & 0xff);
  out[2] = static_cast<uint8_t>(t_v & 0xff);
  out[3] = static_cast<uint8_t>(t_p_kp & 0xff);
  out[4] = static_cast<uint8_t>(t_p_kd & 0xff);
  out[5] = static_cast<uint8_t>(t_v_kp & 0xff);
  out[6] = static_cast<uint8_t>(t_v_kd & 0xff);
  out[7] = static_cast<uint8_t>(t_v_ki & 0xff);
}

inline void unpack_motorevo_servo_cmd(const uint8_t in[8], float &q_rad, float &pos_kp,
                                      float &pos_kd, float &vel_limit_radps, float &vel_kp,
                                      float &vel_kd, float &vel_ki)
{
  const uint16_t t_p =
      static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
  q_rad = uint_to_float(t_p, -kMotorevoQMax, kMotorevoQMax, 16);
  vel_limit_radps = uint_to_float(in[2], -kMotorevoDqMax, kMotorevoDqMax, 8);
  pos_kp = uint_to_float(in[3], 0.f, kMotorevoPosKpMax, 8);
  pos_kd = uint_to_float(in[4], 0.f, kMotorevoPosKdMax, 8);
  vel_kp = uint_to_float(in[5], 0.f, kMotorevoVelKpMax, 8);
  vel_kd = uint_to_float(in[6], 0.f, kMotorevoVelKdMax, 8);
  vel_ki = uint_to_float(in[7], 0.f, kMotorevoVelKiMax, 8);
}

// Motorevo_motor_torque_position_mix_control_mode. Kept for later; P1 stays servo.
inline void pack_motorevo_mix(uint8_t out[8], float q_rad, float dq_radps, float tau,
                              float pos_kp, float pos_kd)
{
  const uint16_t t_p = float_to_uint(q_rad, -kMotorevoQMax, kMotorevoQMax, 16);
  const uint16_t t_v = float_to_uint(dq_radps, -kMotorevoDqMax, kMotorevoDqMax, 12);
  const uint16_t t_t = float_to_uint(tau, -kMotorevoTauMax, kMotorevoTauMax, 12);
  const uint16_t t_p_kp = float_to_uint(pos_kp, 0.f, kMotorevoPosKpMax, 12);
  const uint16_t t_p_kd = float_to_uint(pos_kd, 0.f, kMotorevoPosKdMax, 12);

  out[0] = static_cast<uint8_t>((t_p >> 8) & 0xff);
  out[1] = static_cast<uint8_t>(t_p & 0xff);
  out[2] = static_cast<uint8_t>((t_v >> 4) & 0xff);
  out[3] = static_cast<uint8_t>(static_cast<uint8_t>((t_v & 0x0F) << 4) |
                               static_cast<uint8_t>((t_p_kp >> 8) & 0x0F));
  out[4] = static_cast<uint8_t>(t_p_kp & 0xff);
  out[5] = static_cast<uint8_t>((t_p_kd >> 4) & 0xff);
  out[6] = static_cast<uint8_t>(static_cast<uint8_t>((t_p_kd & 0x0F) << 4) |
                               static_cast<uint8_t>((t_t >> 8) & 0x0F));
  out[7] = static_cast<uint8_t>(t_t & 0xff);
}

inline void unpack_motorevo_mix_cmd(const uint8_t in[8], float &q_rad, float &dq_radps,
                                    float &tau, float &pos_kp, float &pos_kd)
{
  const uint16_t t_p =
      static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
  const uint16_t t_v =
      static_cast<uint16_t>((static_cast<uint16_t>(in[2]) << 4) | (in[3] >> 4));
  const uint16_t t_p_kp =
      static_cast<uint16_t>((static_cast<uint16_t>(in[3] & 0x0F) << 8) | in[4]);
  const uint16_t t_p_kd =
      static_cast<uint16_t>((static_cast<uint16_t>(in[5]) << 4) | (in[6] >> 4));
  const uint16_t t_t =
      static_cast<uint16_t>((static_cast<uint16_t>(in[6] & 0x0F) << 8) | in[7]);
  q_rad = uint_to_float(t_p, -kMotorevoQMax, kMotorevoQMax, 16);
  dq_radps = uint_to_float(t_v, -kMotorevoDqMax, kMotorevoDqMax, 12);
  tau = uint_to_float(t_t, -kMotorevoTauMax, kMotorevoTauMax, 12);
  pos_kp = uint_to_float(t_p_kp, 0.f, kMotorevoPosKpMax, 12);
  pos_kd = uint_to_float(t_p_kd, 0.f, kMotorevoPosKdMax, 12);
}

// Feedback layout follows receive_evo(): q/dq/tau in D[1..5], status in D[0].
inline bool unpack_motorevo_feedback(const CanRxFrame &rx, MotorevoFeedback &out)
{
  if (rx.dlc < 6)
  {
    return false;
  }
  out = {};
  out.rx_can_id = rx.can_id;
  out.dlc = rx.dlc;
  out.status = rx.data[0];
  const uint16_t q_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(rx.data[1]) << 8) | rx.data[2]);
  const uint16_t dq_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(rx.data[3]) << 4) | (rx.data[4] >> 4));
  const uint16_t tau_uint =
      static_cast<uint16_t>((static_cast<uint16_t>(rx.data[4] & 0x0F) << 8) | rx.data[5]);
  out.q = uint_to_float(q_uint, -kMotorevoQMax, kMotorevoQMax, 16);
  out.dq = uint_to_float(dq_uint, -kMotorevoDqMax, kMotorevoDqMax, 12);
  out.tau = uint_to_float(tau_uint, -kMotorevoTauMax, kMotorevoTauMax, 12);
  if (rx.dlc >= 7)
  {
    out.err_code = rx.data[6];
  }
  return true;
}

inline bool is_motorevo_can_id(uint8_t can_id)
{
  return can_id == 8 || can_id == 14;
}

// Motorevo may echo on slave CAN ID 8/14, or on Master 0x02 like Damiao.
inline bool is_motorevo_rx_can_id(uint32_t rx_can_id, uint8_t slave_id)
{
  return rx_can_id == slave_id || rx_can_id == kMasterId;
}

inline const char *motorevo_status_name(uint8_t status)
{
  switch (static_cast<MotorevoStatus>(status))
  {
  case MotorevoStatus::Reset:
    return "reset";
  case MotorevoStatus::Servo:
    return "servo";
  case MotorevoStatus::TpMix:
    return "tpmix";
  case MotorevoStatus::Velocity:
    return "vel";
  case MotorevoStatus::Torque:
    return "tau";
  case MotorevoStatus::Torque4:
    return "tau4";
  }
  return "?";
}

inline bool motorevo_codec_self_check()
{
  uint8_t buf[8] = {};
  pack_motorevo_servo(buf, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
  const uint16_t q0 =
      static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
  if (q0 < 32700 || q0 > 32800)
  {
    return false;
  }
  if (buf[2] < 120 || buf[2] > 135)
  {
    return false;  // dq=0 → mid of 8-bit
  }

  uint8_t hold[8] = {};
  pack_motorevo_servo(hold, 0.1f, 10.f, 1.f, 0.f, 0.f, 0.f, 0.f);
  float q = 0.f, pkp = 0.f, pkd = 0.f, vlim = 0.f, vkp = 0.f, vkd = 0.f, vki = 0.f;
  unpack_motorevo_servo_cmd(hold, q, pkp, pkd, vlim, vkp, vkd, vki);
  if (std::fabs(q - 0.1f) > 0.002f || std::fabs(pkp - 10.f) > 1.5f ||
      std::fabs(pkd - 1.f) > 0.4f)
  {
    return false;
  }

  // 指南 V3.5 伺服例：θ=3.14 rad, V_ref=2 rad/s, pos Kp/Kd=15/4.5, vel Kp/Kd/Ki=50/0/0.001
  // → A0 26 99 0F 16 33 00 05。证明线上是 rad，且 POS_MAX=12.5 不是 5。
  uint8_t pdf[8] = {};
  pack_motorevo_servo(pdf, 3.14f, 15.f, 4.5f, 2.0f, 50.f, 0.f, 0.001f);
  const uint8_t pdf_expect[8] = {0xA0, 0x26, 0x99, 0x0F, 0x16, 0x33, 0x00, 0x05};
  for (int i = 0; i < 8; ++i)
  {
    if (pdf[i] != pdf_expect[i])
    {
      return false;
    }
  }
  float pq = 0.f, ppkp = 0.f, ppkd = 0.f, pvlim = 0.f, pvkp = 0.f, pvkd = 0.f,
        pvki = 0.f;
  unpack_motorevo_servo_cmd(pdf, pq, ppkp, ppkd, pvlim, pvkp, pvkd, pvki);
  if (std::fabs(pq - 3.14f) > 0.002f || std::fabs(pvlim - 2.0f) > 0.08f ||
      std::fabs(pvkp - 50.f) > 1.5f)
  {
    return false;
  }

  CanRxFrame rx{};
  rx.can_id = 8;
  rx.dlc = 8;
  rx.data[0] = 0x01;
  rx.data[1] = 0x7F;
  rx.data[2] = 0xFF;
  rx.data[3] = 0x7F;
  rx.data[4] = 0xF7;
  rx.data[5] = 0xFF;
  MotorevoFeedback fb{};
  if (!unpack_motorevo_feedback(rx, fb))
  {
    return false;
  }
  if (fb.rx_can_id != 8 || std::fabs(fb.q) > 0.02f)
  {
    return false;
  }
  rx.can_id = kMasterId;
  if (!unpack_motorevo_feedback(rx, fb) || fb.rx_can_id != kMasterId)
  {
    return false;
  }
  if (!is_motorevo_rx_can_id(8, 8) || !is_motorevo_rx_can_id(kMasterId, 8) ||
      is_motorevo_rx_can_id(6, 8))
  {
    return false;
  }

  uint8_t mix[8] = {};
  pack_motorevo_mix(mix, 0.f, 0.f, 0.f, 0.f, 0.f);
  const uint16_t mix_q =
      static_cast<uint16_t>((static_cast<uint16_t>(mix[0]) << 8) | mix[1]);
  if (mix_q < 32700 || mix_q > 32800)
  {
    return false;
  }
  uint8_t mix_hold[8] = {};
  pack_motorevo_mix(mix_hold, 0.1f, 0.f, 0.f, 8.f, 0.5f);
  float mq = 0.f, mdq = 0.f, mtau = 0.f, mkp = 0.f, mkd = 0.f;
  unpack_motorevo_mix_cmd(mix_hold, mq, mdq, mtau, mkp, mkd);
  if (std::fabs(mq - 0.1f) > 0.002f || std::fabs(mkp - 8.f) > 0.3f ||
      std::fabs(mkd - 0.5f) > 0.05f)
  {
    return false;
  }

  uint8_t en[8] = {};
  pack_motorevo_enable(en);
  uint8_t dis[8] = {};
  pack_motorevo_disable(dis);
  if (en[7] != kDmEnableCmd || dis[7] != kDmDisableCmd)
  {
    return false;
  }
  uint8_t z1[8] = {};
  pack_motorevo_set_zero_v1(z1);
  if (z1[0] != 0xFF || z1[1] != 0xFF || z1[2] != 0xFF || z1[3] != 0xFF ||
      z1[4] != 0xFF || z1[5] != 0xFF || z1[6] != 0xFF || z1[7] != kMotorevoSetZeroCmd)
  {
    return false;
  }
  uint8_t z2[8] = {};
  pack_motorevo_set_zero_v2(z2);
  uint8_t zdef[8] = {};
  pack_motorevo_set_zero(zdef);
  const uint8_t z2_expect[8] = {0x67, 0x06, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x76};
  uint8_t save[8] = {};
  pack_motorevo_save_flash(save);
  const uint8_t save_expect[8] = {0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x76};
  if (motorevo_param_can_id(8) != 0x608u || motorevo_param_can_id(14) != 0x60Eu)
  {
    return false;
  }
  for (int i = 0; i < 8; ++i)
  {
    if (z2[i] != z2_expect[i] || zdef[i] != z1[i] || save[i] != save_expect[i])
    {
      return false;
    }
  }
  return true;
}

}  // namespace biped_hw
