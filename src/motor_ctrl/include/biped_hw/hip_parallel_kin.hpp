// biped_hw — hip_parallel_kin.hpp
// 髋并联 FK/IK。公式来自《髋关节并联机构的运动耦合分析》式 (2-8)(2-9)
// 三次拟合 IK；FK 用 (2-12) 雅可比 Newton 反解。下层库，不是 app。
// 不含 hip yaw。不碰 CAN / 0xFC。
#pragma once

#include "biped_hw/types.hpp"

namespace biped_hw
{

// Link lengths from the doc (parallelogram ABCD / OADO1). Kept as constants
// because the production map is the cubic fit, not a reconstructed 5-bar.
inline constexpr double kHipL_m = 0.1725;     // OO1 and BC
inline constexpr double kHipD1_m = 0.0855;    // OA and O1D
inline constexpr double kHipD2_m = 0.100;     // AB and CD
inline constexpr double kHipTheta0_rad = 120.0 * 3.14159265358979323846 / 180.0;

// Doc workspace. α = roll, β = pitch. Polynomial is in radians (fit plots).
inline constexpr double kHipRollMin_rad = -20.0 * 3.14159265358979323846 / 180.0;
inline constexpr double kHipRollMax_rad = 20.0 * 3.14159265358979323846 / 180.0;
inline constexpr double kHipPitchMin_rad = -45.0 * 3.14159265358979323846 / 180.0;
inline constexpr double kHipPitchMax_rad = 15.0 * 3.14159265358979323846 / 180.0;

// Fig 2-3 / 3-4: 电机1 at −Y = inner, 电机2 at +Y (outboard crank) = outer.
// Hardware: outer = smaller CAN ID (left 4, right 10).
// Right hip is a sagittal Y-mirror of the same linkage: α_doc = −roll_body.
enum class HipSide : uint8_t
{
  Left = 0,
  Right = 1,
};

struct HipPose
{
  double roll = 0.0;   // α, rad
  double pitch = 0.0;  // β, rad
};

struct HipMotors
{
  double q_outer = 0.0;  // θ2, rad
  double q_inner = 0.0;  // θ1, rad
};

bool hip_in_workspace(double roll, double pitch);

// Document-frame polynomial (2-8)(2-9): (α,β) → (θ1,θ2). No side, no reject.
void hip_ik_doc(double alpha, double beta, double *theta1, double *theta2);

// Command path: desired roll/pitch → outer/inner motor rad.
// false if (α,β) outside the doc workspace. On false, *q is unchanged.
bool hip_ik(HipSide side, const HipPose &pose, HipMotors *q);

// State path: outer/inner motor rad → roll/pitch.
// false if Newton fails or the recovered pose is outside workspace.
bool hip_fk(HipSide side, const HipMotors &q, HipPose *pose);

// 0 = pass. Nonzero = failed step id. No hardware.
int hip_parallel_kin_self_test();

}  // namespace biped_hw
