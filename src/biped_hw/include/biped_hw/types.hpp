// biped_hw — types.hpp
// 公共类型：左右总线、12 关节、电机型号、CAN 帧与 MIT 限幅 LimitParam。
// P0 起被电机表、编解码和全部 apps 共用。
#pragma once

#include <cstddef>
#include <cstdint>

namespace biped_hw
{

enum class BusId : uint8_t
{
  Left = 0,
  Right = 1,
};

inline constexpr std::size_t kBusCount = 2;

// Motor models on this machine. Do not insert a Num_Of_Motor-style sentinel
// in the middle of the enum (that OOB pattern in all_motor.h let extra values
// like ELMO sit past the limit table).
enum class MotorType : uint8_t
{
  DM10010L = 0,
  DM10010,
  DM10422,
  HA60,
};

inline constexpr bool is_damiao_mit(MotorType type)
{
  return type == MotorType::DM10010L || type == MotorType::DM10010 ||
         type == MotorType::DM10422;
}

enum class JointId : uint8_t
{
  LHipYaw = 0,
  LHipParallelOuter,
  LHipParallelInner,
  LKneePitch,
  LAnklePitch,
  LAnkleRoll,
  RHipYaw,
  RHipParallelOuter,
  RHipParallelInner,
  RKneePitch,
  RAnklePitch,
  RAnkleRoll,
};

inline constexpr std::size_t kMotorCount = 12;

// Per-bus USB-CAN destination. Both left and right adapters use Master 2.
inline constexpr uint8_t kMasterId = 0x02;

// Damiao feedback CAN ID is Master 0x02 for every motor on a bus. Demux by
// payload D[0] low nibble (ESC_ID), not by the USB-CAN reported canId.
// Official SDK: https://gitee.com/kit-miao/damiao (motor-sdk). Do not vendor it.
//
// Pack-range defaults (PMAX / VMAX / TMAX). Firmware RID 21/22/23 overrides
// at runtime when successfully read. HA60 ranges are Motorevo (not Damiao MIT).
struct LimitParam
{
  float q_max = 0.f;//单位：rad
  float dq_max = 0.f;//单位：rps  
  float tau_max = 0.f;//单位：Nm  
};

inline constexpr LimitParam kLimitDm10010L{12.5f, 25.f, 200.f};
inline constexpr LimitParam kLimitDm10010{12.5f, 20.f, 200.f};
inline constexpr LimitParam kLimitDm10422{12.5f, 20.f, 400.f};
inline constexpr LimitParam kLimitHa60{12.5f, 10.f, 50.f};

inline LimitParam default_limit(MotorType type)
{
  switch (type)
  {
  case MotorType::DM10010L:
    return kLimitDm10010L;
  case MotorType::DM10010:
    return kLimitDm10010;
  case MotorType::HA60:
    return kLimitHa60;
  case MotorType::DM10422:
    return kLimitDm10422;
  }
  return kLimitDm10010L;
}

struct CanRxFrame
{
  uint32_t can_id = 0;
  uint8_t dlc = 0;
  uint8_t data[8] = {};
  bool extended = false;
  bool remote = false;
};

inline const char *to_string(BusId bus)
{
  switch (bus)
  {
  case BusId::Left:
    return "Left";
  case BusId::Right:
    return "Right";
  }
  return "?";
}

inline const char *to_string(MotorType type)
{
  switch (type)
  {
  case MotorType::DM10010L:
    return "DM10010L";
  case MotorType::DM10010:
    return "DM10010";
  case MotorType::DM10422:
    return "DM10422";
  case MotorType::HA60:
    return "HA60";
  }
  return "?";
}

inline const char *to_string(JointId joint)
{
  switch (joint)
  {
  case JointId::LHipYaw:
    return "hip yaw";
  case JointId::LHipParallelOuter:
    return "hip parallel OUTER";
  case JointId::LHipParallelInner:
    return "hip parallel INNER";
  case JointId::LKneePitch:
    return "knee pitch";
  case JointId::LAnklePitch:
    return "ankle pitch";
  case JointId::LAnkleRoll:
    return "ankle roll";
  case JointId::RHipYaw:
    return "hip yaw";
  case JointId::RHipParallelOuter:
    return "hip parallel OUTER";
  case JointId::RHipParallelInner:
    return "hip parallel INNER";
  case JointId::RKneePitch:
    return "knee pitch";
  case JointId::RAnklePitch:
    return "ankle pitch";
  case JointId::RAnkleRoll:
    return "ankle roll";
  }
  return "?";
}

}  // namespace biped_hw
