// biped_hw — motor_map.hpp
// 编译期权威 12 电机表（CAN ID、主站 0x02、型号、关节角色）。
// P0 映射源；config/motor_map.yaml 只覆盖串口路径，不改 ID。
#pragma once

#include "biped_hw/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace biped_hw
{

// Runtime overlays (serial paths, udev aliases, later zero/dir) live in
// config/motor_map.yaml. CAN IDs, master ID, models, and joint roles stay here.

inline constexpr const char *kDefaultPortLeft = "/dev/ttyACM0";
inline constexpr const char *kDefaultPortRight = "/dev/ttyACM1";
inline constexpr const char *kUdevAliasLeft = "/dev/usb2can_left";
inline constexpr const char *kUdevAliasRight = "/dev/usb2can_right";

struct MotorMapEntry
{
  BusId bus;
  uint8_t master_id;
  uint8_t can_id;
  JointId joint;
  MotorType model;
  const char *notes;
};

// Authoritative 12-motor table. Index is stable; do not reorder without
// updating callers. smaller CAN ID on a parallel hip pair = OUTER.
inline constexpr std::array<MotorMapEntry, kMotorCount> kMotorMap{{
    {BusId::Left, kMasterId, 3, JointId::LHipYaw, MotorType::DM10010L,
     "no reducer 1:1"},
    {BusId::Left, kMasterId, 4, JointId::LHipParallelOuter, MotorType::DM10010,
     "parallel; smaller ID = outer"},
    {BusId::Left, kMasterId, 5, JointId::LHipParallelInner, MotorType::DM10010,
     ""},
    {BusId::Left, kMasterId, 6, JointId::LKneePitch, MotorType::DM10422,
     "cable 1:1; limits UNVERIFIED, read RID 21/22/23 in P1"},
    {BusId::Left, kMasterId, 7, JointId::LAnklePitch, MotorType::DM10010L,
     "cable 1:1"},
    {BusId::Left, kMasterId, 8, JointId::LAnkleRoll, MotorType::HA60,
     "Motorevo HA60-100, servo default later"},
    {BusId::Right, kMasterId, 9, JointId::RHipYaw, MotorType::DM10010L, ""},
    {BusId::Right, kMasterId, 10, JointId::RHipParallelOuter, MotorType::DM10010,
     "parallel; smaller ID = outer"},
    {BusId::Right, kMasterId, 11, JointId::RHipParallelInner, MotorType::DM10010,
     ""},
    {BusId::Right, kMasterId, 12, JointId::RKneePitch, MotorType::DM10422,
     "UNVERIFIED limits"},
    {BusId::Right, kMasterId, 13, JointId::RAnklePitch, MotorType::DM10010L, ""},
    {BusId::Right, kMasterId, 14, JointId::RAnkleRoll, MotorType::HA60,
     "Motorevo HA60-100, servo default later"},
}};

constexpr bool motor_map_ids_unique()
{
  for (std::size_t i = 0; i < kMotorMap.size(); ++i)
  {
    for (std::size_t j = i + 1; j < kMotorMap.size(); ++j)
    {
      if (kMotorMap[i].can_id == kMotorMap[j].can_id)
      {
        return false;
      }
    }
  }
  return true;
}

constexpr bool motor_map_masters_ok()
{
  for (const auto &e : kMotorMap)
  {
    if (e.master_id != kMasterId)
    {
      return false;
    }
  }
  return true;
}

constexpr bool motor_map_bus_can_split_ok()
{
  for (const auto &e : kMotorMap)
  {
    if (e.bus == BusId::Left && (e.can_id < 3 || e.can_id > 8))
    {
      return false;
    }
    if (e.bus == BusId::Right && (e.can_id < 9 || e.can_id > 14))
    {
      return false;
    }
  }
  return true;
}

static_assert(kMotorMap.size() == kMotorCount, "map must list all 12 motors");
static_assert(motor_map_ids_unique(), "CAN IDs must be unique");
static_assert(motor_map_masters_ok(), "master id must be 2 on both buses");
static_assert(motor_map_bus_can_split_ok(), "left CAN 3-8, right CAN 9-14");

inline constexpr const char *default_port(BusId bus)
{
  return bus == BusId::Left ? kDefaultPortLeft : kDefaultPortRight;
}

inline constexpr const MotorMapEntry *find_by_can_id(uint8_t can_id)
{
  for (const auto &e : kMotorMap)
  {
    if (e.can_id == can_id)
    {
      return &e;
    }
  }
  return nullptr;
}

inline constexpr const MotorMapEntry *find_by_joint(JointId joint)
{
  for (const auto &e : kMotorMap)
  {
    if (e.joint == joint)
    {
      return &e;
    }
  }
  return nullptr;
}

}  // namespace biped_hw
