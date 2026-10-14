// biped_hw — safety.hpp
// P5: watchdog, soft/hard per-ID limits, e-stop latch, dual-bus disable-all.
// Functions only. Reuses dm_send_disable (0xFD). No GPIO, no hip IK geometry.
#pragma once

#include "biped_hw/motor_map.hpp"
#include "biped_hw/types.hpp"

#include <cstddef>
#include <cstdint>

namespace biped_hw
{

class UsbCanAdapter;

// q_soft/q_hard are |q| rad on the motor shaft (motor_map CAN ID), not hip FK.
// Joint-space hip roll/pitch limits can later use hip_fk output; P5 stays per ID.
// tau_hard matches existing abort-tau style (15 Nm). Do not raise.
//--------------------------------限位具体值需后续根据实际修改-----------------------------------------
inline constexpr float kSafetyQSoftRad = 10.f;
inline constexpr float kSafetyQHardRad = 12.f;
inline constexpr float kSafetyTauHardNm = 15.f;
inline constexpr int kSafetyWatchdogMs = 40;
inline constexpr int kSafetyDropStreak = 3;
inline constexpr double kSafetyStaleMs = 50.0;

enum class SafetyReason : uint8_t
{
  None = 0,
  Watchdog,
  Estop,
  HardLimitQ,
  HardLimitTau,
  Drop,
  RidFail,
};

enum class SafetyLimitHit : uint8_t
{
  Ok = 0,
  Soft,  // reject/hold command; do not disable
  Hard,  // disable-all
};

struct SafetyWatchdog
{
  std::int64_t last_pet_ns = 0;
  int timeout_ms = kSafetyWatchdogMs;
  bool armed = false;
};

struct SafetyDropWatch
{
  uint8_t streak[16] = {};
};

struct SafetyDisableHook
{
  void (*on_disable)(void *ctx, BusId bus, uint8_t can_id) = nullptr;
  void *ctx = nullptr;
};

const char *to_string(SafetyReason reason);

SafetyLimitHit safety_check_q(uint8_t can_id, float q);
SafetyLimitHit safety_check_tau(uint8_t can_id, float tau);
SafetyLimitHit safety_check_mbs_q(uint8_t addr, double q);

// Clamp command q to ±q_soft. Hard measured |q| is still Hard (caller disables).
float safety_hold_q(uint8_t can_id, float q_cmd, SafetyLimitHit *hit);

void safety_watchdog_arm(SafetyWatchdog &wd, int timeout_ms, std::int64_t now_ns);
void safety_watchdog_pet(SafetyWatchdog &wd, std::int64_t now_ns);
bool safety_watchdog_expired(const SafetyWatchdog &wd, std::int64_t now_ns);

// Consecutive no-RX on a selected CAN ID. Returns true at kSafetyDropStreak.
bool safety_note_rx(SafetyDropWatch &w, uint8_t can_id, bool rx_ok);

void safety_estop_trip();
bool safety_estop_latched();
void safety_estop_clear();
void safety_install_signals();  // SIGINT/SIGTERM → latch (not async CAN TX)

SafetyReason safety_last_reason();
void safety_set_reason(SafetyReason reason);

// Bind adapters for atexit backup. Arm after 0xFC (or partial enable).
void safety_bind_buses(UsbCanAdapter *left, UsbCanAdapter *right);
void safety_arm();
void safety_disarm();
bool safety_armed();
void safety_atexit_disable();

// 0xFD on commanded IDs via dm_send_disable. ids==nullptr / n==0 → motor_map 12
// (left 3–8, right 9–14). Null/closed adapter skipped; the other bus still fires.
std::size_t safety_disable_all(UsbCanAdapter *left, UsbCanAdapter *right,
                               const uint8_t *ids, std::size_t n, SafetyReason reason,
                               bool print, SafetyDisableHook *hook = nullptr);

int safety_self_test();

}  // namespace biped_hw
