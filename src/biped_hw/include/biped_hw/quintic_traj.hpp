// biped_hw — quintic_traj.hpp
// Rest-to-rest 五次多项式. Math copied from motor_ctrl trajectoryGenerator
// (single_motor_ctrl.cpp / left_leg_ctrl.cpp quinticPolynomial). Not a plugin.
// Do not #include or link motor_ctrl.
//
// motor_ctrl T=1, Δ=gain: a0=a1=a2=0, a3=10Δ, a4=-15Δ, a5=6Δ
//   q(t)=a0+a1 t+a2 t²+a3 t³+a4 t⁴+a5 t⁵
//   qd(t)=a1+2 a2 t+3 a3 t²+4 a4 t³+5 a5 t⁴
// General T: σ(s)=10s³−15s⁴+6s⁵, s=t/T. q,dq,ddq = 0 vel/acc at both ends.
// Shape lives here — not in pid_track, not generated inline in p6_traj_track.
#pragma once

namespace biped_hw
{

inline constexpr float kQuinticAmpRad = 0.35f;  // ~20 deg
inline constexpr double kQuinticSegTime_s = 1.0;  // motor_ctrl t_end=1

// One point-to-point segment. t clamped to [0, T]. Null q/dq skipped.
void quintic_poly5(double q0, double qf, double T, double t, double *q, double *dq);

// Left-right swing about probe q0:
// q0 → q0+A → q0 → q0−A → q0, each T_seg, then repeats.
void quintic_swing_about(double q0, double amp, double T_seg, double t, double *q,
                         double *dq);

// Per-motor probe q0. i = CAN id (3–14). Same phase, all motors.
struct QuinticSwingTraj
{
  float amp = kQuinticAmpRad;
  double seg_time = kQuinticSegTime_s;

  void set_q0(int i, float q0)
  {
    if (i < 0 || i > 15)
    {
      return;
    }
    q0_[static_cast<unsigned>(i)] = q0;
    have_[static_cast<unsigned>(i)] = true;
  }

  float q_des(int i, float t) const
  {
    double q = 0.0;
    eval_at(i, t, &q, nullptr);
    return static_cast<float>(q);
  }

  float dq_des(int i, float t) const
  {
    double dq = 0.0;
    eval_at(i, t, nullptr, &dq);
    return static_cast<float>(dq);
  }

private:
  float q0_[16]{};
  bool have_[16]{};

  void eval_at(int i, float t, double *q, double *dq) const
  {
    if (i < 0 || i > 15 || !have_[static_cast<unsigned>(i)])
    {
      if (q != nullptr)
      {
        *q = 0.0;
      }
      if (dq != nullptr)
      {
        *dq = 0.0;
      }
      return;
    }
    quintic_swing_about(static_cast<double>(q0_[static_cast<unsigned>(i)]),
                        static_cast<double>(amp), seg_time, static_cast<double>(t), q, dq);
  }
};

int quintic_traj_self_test();

}  // namespace biped_hw
