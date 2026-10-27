// biped_hw — quintic_traj.cpp
// σ(s)=10s³−15s⁴+6s⁵, s=t/T. Same as motor_ctrl quinticPolynomial for T=1.
#include "biped_hw/quintic_traj.hpp"

#include <cmath>

namespace biped_hw
{

void quintic_poly5(double q0, double qf, double T, double t, double *q, double *dq)
{
  double qi = q0;
  double dqi = 0.0;
  if (T <= 1e-12)
  {
    qi = qf;
  }
  else if (t <= 0.0)
  {
    qi = q0;
  }
  else if (t >= T)
  {
    qi = qf;
  }
  else
  {
    const double s = t / T;
    const double s2 = s * s;
    const double s3 = s2 * s;
    const double s4 = s2 * s2;
    const double s5 = s4 * s;
    const double span = qf - q0;
    qi = q0 + span * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
    dqi = span * (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / T;
  }
  if (q != nullptr)
  {
    *q = qi;
  }
  if (dq != nullptr)
  {
    *dq = dqi;
  }
}

void quintic_swing_about(double q0, double amp, double T_seg, double t, double *q,
                         double *dq)
{
  if (T_seg <= 1e-12 || t <= 0.0)
  {
    if (q != nullptr)
    {
      *q = q0;
    }
    if (dq != nullptr)
    {
      *dq = 0.0;
    }
    return;
  }
  const double period = 4.0 * T_seg;
  double tau = t - period * std::floor(t / period);
  if (tau < 0.0)
  {
    tau = 0.0;
  }
  int seg = static_cast<int>(tau / T_seg);
  if (seg > 3)
  {
    seg = 3;
  }
  const double t_local = tau - static_cast<double>(seg) * T_seg;
  const double w[5] = {q0, q0 + amp, q0, q0 - amp, q0};
  quintic_poly5(w[seg], w[seg + 1], T_seg, t_local, q, dq);
}

int quintic_traj_self_test()
{
  const double q0 = 5.0;  // right-leg ID14-like; about probe, not firmware 0
  const double A = 0.35;
  const double T = kQuinticSegTime_s;
  // motor_ctrl T=1 Δ=1: a3=10 a4=-15 a5=6 → q(0.4)=10 t³−15 t⁴+6 t⁵
  {
    double qv = 0.0;
    double dqv = 0.0;
    const double tt = 0.4;
    quintic_poly5(0.0, 1.0, 1.0, tt, &qv, &dqv);
    const double expect = 10.0 * tt * tt * tt - 15.0 * tt * tt * tt * tt +
                          6.0 * tt * tt * tt * tt * tt;
    if (std::fabs(qv - expect) > 1e-12)
    {
      return 11;
    }
  }
  double q = 0.0;
  double dq = 1.0;

  quintic_poly5(q0, q0 + A, T, 0.0, &q, &dq);
  if (!std::isfinite(q) || !std::isfinite(dq) || std::fabs(q - q0) > 1e-9 ||
      std::fabs(dq) > 1e-9)
  {
    return 1;
  }

  quintic_poly5(q0, q0 + A, T, T, &q, &dq);
  if (std::fabs(q - (q0 + A)) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 2;
  }

  quintic_poly5(q0, q0 + A, T, T * 0.5, &q, &dq);
  if (std::fabs(q - (q0 + 0.5 * A)) > 1e-9 || std::fabs(dq - 1.875 * A / T) > 1e-9)
  {
    return 3;
  }

  const double T2 = 2.0;
  quintic_poly5(q0, q0 + A, T2, T2, &q, &dq);
  if (std::fabs(q - (q0 + A)) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 4;
  }

  quintic_swing_about(q0, A, T, 0.0, &q, &dq);
  if (std::fabs(q - q0) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 5;
  }
  quintic_swing_about(q0, A, T, T, &q, &dq);
  if (std::fabs(q - (q0 + A)) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 6;
  }
  quintic_swing_about(q0, A, T, 2.0 * T, &q, &dq);
  if (std::fabs(q - q0) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 7;
  }
  quintic_swing_about(q0, A, T, 3.0 * T, &q, &dq);
  if (std::fabs(q - (q0 - A)) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 8;
  }
  quintic_swing_about(q0, A, T, 4.0 * T, &q, &dq);
  if (std::fabs(q - q0) > 1e-9 || std::fabs(dq) > 1e-9)
  {
    return 9;
  }

  QuinticSwingTraj traj;
  traj.amp = static_cast<float>(A);
  traj.seg_time = T;
  traj.set_q0(14, static_cast<float>(q0));
  traj.set_q0(12, 1.f);
  if (std::fabs(traj.q_des(14, 0.f) - static_cast<float>(q0)) > 1e-5f ||
      std::fabs(traj.dq_des(14, 0.f)) > 1e-5f)
  {
    return 12;
  }
  if (std::fabs(traj.q_des(14, static_cast<float>(T)) - static_cast<float>(q0 + A)) >
          1e-4f ||
      std::fabs(traj.dq_des(14, static_cast<float>(T))) > 1e-4f)
  {
    return 13;
  }
  const float tmid = static_cast<float>(0.3 * T);
  if (std::fabs((traj.q_des(14, tmid) - static_cast<float>(q0)) -
                (traj.q_des(12, tmid) - 1.f)) > 1e-4f)
  {
    return 14;
  }
  return 0;
}

}  // namespace biped_hw
