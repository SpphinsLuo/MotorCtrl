// biped_hw — hip_parallel_kin.cpp
// Cubic IK (2-8)(2-9) and Newton FK. Geometry constants are documented but
// R1(α)/R2(β) axes are not written; do not substitute a guessed 5-bar.
#include "biped_hw/hip_parallel_kin.hpp"

#include <cmath>

namespace biped_hw
{
namespace
{

constexpr double kIkTol = 1e-10;
constexpr int kFkMaxIter = 16;
constexpr double kFkResTol = 1e-12;
constexpr double kFkDetMin = 1e-12;
constexpr double kWsEps = 1e-9;

// (2-8) θ1
constexpr double kA1 = 1.006;
constexpr double kB1 = 0.921;
constexpr double kC1 = 0.018;
constexpr double kD1 = 0.330;
constexpr double kE1 = -0.098;
constexpr double kF1 = 0.009;
constexpr double kG1 = 0.043;
constexpr double kH1 = 0.326;
constexpr double kI1 = -0.175;

// (2-9) θ2
constexpr double kA2 = 1.006;
constexpr double kB2 = -0.921;
constexpr double kC2 = 0.018;
constexpr double kD2 = -0.330;
constexpr double kE2 = -0.098;
constexpr double kF2 = 0.009;
constexpr double kG2 = -0.043;
constexpr double kH2 = 0.326;
constexpr double kI2 = 0.175;

double poly(double a, double b, double A, double B, double C, double D, double E,
            double F, double G, double H, double I)
{
  const double a2 = a * a;
  const double b2 = b * b;
  return A * a + B * b + C * a2 + D * a * b + E * b2 + F * a2 * a + G * a2 * b +
         H * a * b2 + I * b2 * b;
}

void dpoly(double a, double b, double A, double B, double C, double D, double E,
           double F, double G, double H, double I, double *da, double *db)
{
  const double a2 = a * a;
  const double b2 = b * b;
  *da = A + 2.0 * C * a + D * b + 3.0 * F * a2 + 2.0 * G * a * b + H * b2;
  *db = B + D * a + 2.0 * E * b + G * a2 + 2.0 * H * a * b + 3.0 * I * b2;
}

double alpha_doc(HipSide side, double roll)
{
  return side == HipSide::Right ? -roll : roll;
}

double roll_body(HipSide side, double alpha)
{
  return side == HipSide::Right ? -alpha : alpha;
}

bool in_ws(double alpha, double beta)
{
  return alpha >= kHipRollMin_rad - kWsEps && alpha <= kHipRollMax_rad + kWsEps &&
         beta >= kHipPitchMin_rad - kWsEps && beta <= kHipPitchMax_rad + kWsEps;
}

bool fk_doc(double theta1, double theta2, double *alpha, double *beta)
{
  // Linear inverse of (2-8)(2-9) as Newton start.
  double a = (theta1 + theta2) / (2.0 * kA1);
  double b = (theta1 - theta2) / (2.0 * kB1);
  for (int i = 0; i < kFkMaxIter; ++i)
  {
    const double t1 = poly(a, b, kA1, kB1, kC1, kD1, kE1, kF1, kG1, kH1, kI1);
    const double t2 = poly(a, b, kA2, kB2, kC2, kD2, kE2, kF2, kG2, kH2, kI2);
    const double e1 = theta1 - t1;
    const double e2 = theta2 - t2;
    if (std::fabs(e1) < kFkResTol && std::fabs(e2) < kFkResTol)
    {
      *alpha = a;
      *beta = b;
      return in_ws(a, b);
    }
    double j11, j12, j21, j22;
    dpoly(a, b, kA1, kB1, kC1, kD1, kE1, kF1, kG1, kH1, kI1, &j11, &j12);
    dpoly(a, b, kA2, kB2, kC2, kD2, kE2, kF2, kG2, kH2, kI2, &j21, &j22);
    const double det = j11 * j22 - j12 * j21;
    if (std::fabs(det) < kFkDetMin)
    {
      return false;
    }
    a += (e1 * j22 - e2 * j12) / det;
    b += (j11 * e2 - j21 * e1) / det;
  }
  return false;
}

}  // namespace

bool hip_in_workspace(double roll, double pitch)
{
  return in_ws(roll, pitch);
}

void hip_ik_doc(double alpha, double beta, double *theta1, double *theta2)
{
  *theta1 = poly(alpha, beta, kA1, kB1, kC1, kD1, kE1, kF1, kG1, kH1, kI1);
  *theta2 = poly(alpha, beta, kA2, kB2, kC2, kD2, kE2, kF2, kG2, kH2, kI2);
}

bool hip_ik(HipSide side, const HipPose &pose, HipMotors *q)
{
  if (q == nullptr)
  {
    return false;
  }
  const double a = alpha_doc(side, pose.roll);
  const double b = pose.pitch;
  if (!in_ws(a, b))
  {
    return false;
  }
  double th1 = 0.0;
  double th2 = 0.0;
  hip_ik_doc(a, b, &th1, &th2);
  q->q_inner = th1;
  q->q_outer = th2;
  return true;
}

bool hip_fk(HipSide side, const HipMotors &q, HipPose *pose)
{
  if (pose == nullptr)
  {
    return false;
  }
  double a = 0.0;
  double b = 0.0;
  if (!fk_doc(q.q_inner, q.q_outer, &a, &b))
  {
    return false;
  }
  pose->roll = roll_body(side, a);
  pose->pitch = b;
  return true;
}

int hip_parallel_kin_self_test()
{
  double z1 = 1.0;
  double z2 = 1.0;
  hip_ik_doc(0.0, 0.0, &z1, &z2);
  if (std::fabs(z1) > kIkTol || std::fabs(z2) > kIkTol)
  {
    return 1;
  }

  HipMotors q{};
  HipPose p{0.0, 0.0};
  if (!hip_ik(HipSide::Left, p, &q) || std::fabs(q.q_outer) > kIkTol ||
      std::fabs(q.q_inner) > kIkTol)
  {
    return 2;
  }

  const double roll_lo = kHipRollMin_rad;
  const double roll_hi = kHipRollMax_rad;
  const double pitch_lo = kHipPitchMin_rad;
  const double pitch_hi = kHipPitchMax_rad;
  const int n_a = 9;
  const int n_b = 13;
  for (int ia = 0; ia < n_a; ++ia)
  {
    for (int ib = 0; ib < n_b; ++ib)
    {
      const double a =
          roll_lo + (roll_hi - roll_lo) * static_cast<double>(ia) / (n_a - 1);
      const double b =
          pitch_lo + (pitch_hi - pitch_lo) * static_cast<double>(ib) / (n_b - 1);
      const HipSide sides[2] = {HipSide::Left, HipSide::Right};
      for (int s = 0; s < 2; ++s)
      {
        const HipSide side = sides[s];
        HipPose cmd{roll_body(side, a), b};
        HipMotors mot{};
        if (!hip_ik(side, cmd, &mot))
        {
          return 3;
        }
        HipPose got{};
        if (!hip_fk(side, mot, &got))
        {
          return 4;
        }
        if (std::fabs(got.roll - cmd.roll) > 1e-9 ||
            std::fabs(got.pitch - cmd.pitch) > 1e-9)
        {
          return 5;
        }
        HipMotors mot2{};
        if (!hip_ik(side, got, &mot2))
        {
          return 6;
        }
        if (std::fabs(mot2.q_outer - mot.q_outer) > 1e-9 ||
            std::fabs(mot2.q_inner - mot.q_inner) > 1e-9)
        {
          return 7;
        }
      }
    }
  }

  HipPose oob{kHipRollMax_rad + 0.05, 0.0};
  HipMotors junk{123.0, 456.0};
  if (hip_ik(HipSide::Left, oob, &junk) || junk.q_outer != 123.0 ||
      junk.q_inner != 456.0)
  {
    return 8;
  }
  HipPose oob_p{0.0, kHipPitchMax_rad + 0.05};
  if (hip_ik(HipSide::Left, oob_p, &junk))
  {
    return 9;
  }

  // Pure pitch: sagittal, same motor commands on both legs.
  HipPose pitch_only{0.0, 10.0 * 3.14159265358979323846 / 180.0};
  HipMotors ql{}, qr{};
  if (!hip_ik(HipSide::Left, pitch_only, &ql) ||
      !hip_ik(HipSide::Right, pitch_only, &qr))
  {
    return 10;
  }
  if (std::fabs(ql.q_outer - qr.q_outer) > kIkTol ||
      std::fabs(ql.q_inner - qr.q_inner) > kIkTol)
  {
    return 11;
  }

  // Pure roll: right hip uses α_doc = −roll (Y-mirror). Even polynomial
  // terms mean motors are not a pure sign flip; they match Left(−roll).
  HipPose roll_only{10.0 * 3.14159265358979323846 / 180.0, 0.0};
  HipPose roll_neg{-roll_only.roll, 0.0};
  HipMotors qn{};
  if (!hip_ik(HipSide::Left, roll_only, &ql) ||
      !hip_ik(HipSide::Right, roll_only, &qr) ||
      !hip_ik(HipSide::Left, roll_neg, &qn))
  {
    return 12;
  }
  if (std::fabs(qr.q_outer - qn.q_outer) > 1e-9 ||
      std::fabs(qr.q_inner - qn.q_inner) > 1e-9)
  {
    return 13;
  }

  HipMotors bad{3.0, -3.0};
  HipPose out{};
  if (hip_fk(HipSide::Left, bad, &out))
  {
    return 14;
  }

  return 0;
}

}  // namespace biped_hw
