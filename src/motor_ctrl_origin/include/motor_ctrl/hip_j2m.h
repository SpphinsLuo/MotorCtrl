#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace hip_m2j {

using namespace Eigen;

// 机构参数结构体
struct Params {
    double L;       // B–C 连杆长度
    double A1A2;
    double A1B1;
    double A2B2;
    double C2D2;
    double OpD1;
    Vector3d Op_op; // 全局偏移向量

    // 构造函数：可根据需要调整参数
    Params()
        : L(0.1725), A1A2(0.171), A1B1(0.100), A2B2(0.100), C2D2(0.100),
          OpD1(0.5 * A1A2), Op_op(0, 0, -0.1725) {}
};

// 计算点 C 在全局坐标系的位置（通用函数）
inline Vector3d calc_C_O(double alpha, double beta, const Params& p, bool isFirst) {
    // 根据 isFirst 选择 C1 或 C2 的初始向量
    Vector3d Cp0;
    double sign = isFirst ? -1.0 : 1.0;
    Cp0 << -p.C2D2 * std::cos(M_PI/6),
           sign * p.OpD1,
            p.C2D2 * std::sin(M_PI/6);

    // 绕 Yp 轴旋转 -alpha
    Matrix3d Ry;
    Ry << std::cos(alpha), 0, std::sin(alpha),
          0,                1, 0,
         -std::sin(alpha), 0, std::cos(alpha);
    Vector3d Cprime = Ry * Cp0;

    // 绕 k 轴旋转 -beta
    Vector3d k{std::cos(alpha), 0, std::sin(alpha)};
    Matrix3d I = Matrix3d::Identity();
    Matrix3d K;
    K <<    0, -k.z(),  k.y(),
         k.z(),    0, -k.x(),
        -k.y(), k.x(),    0;
    Matrix3d Rk = std::cos(beta)*I + (1 - std::cos(beta))*(k * k.transpose()) + std::sin(beta)*K;
    Vector3d Cres = Rk * Cprime;

    return p.Op_op + Cres;
}

// 输入 alpha, beta，输出 [theta_a, theta_b]
inline Vector2d inverseKinematics(double alpha, double beta, const Params& p = Params()) {
    // 计算 C1, C2
    Vector3d C1 = calc_C_O(alpha, beta, p, true);
    Vector3d C2 = calc_C_O(alpha, beta, p, false);

    // 常量
    double a1 = p.A1B1, a2 = p.A2B2;
    double L  = p.L;
    double dY = p.A1A2 / 2;
    double yB1 = -dY, yB2 = dY;

    // 链 1
    double x1 = C1.x(), y1 = C1.y(), z1 = C1.z();
    double dy1 = y1 - yB1;
    double K1 = (L*L - dy1*dy1 - (x1*x1 + z1*z1 + a1*a1)) / (2*a1);
    double delta1 = std::atan2(-z1, x1);
    double phi1 = delta1 + std::acos(K1 / std::sqrt(delta1*delta1 + (-z1)*( -z1) ));
    double theta_a = M_PI/6 - phi1;

    // 链 2
    double x2 = C2.x(), y2 = C2.y(), z2 = C2.z();
    double dy2 = y2 - yB2;
    double K2 = (L*L - dy2*dy2 - (x2*x2 + z2*z2 + a2*a2)) / (2*a2);
    double delta2 = std::atan2(-z2, x2);
    double phi2 = delta2 + std::acos(K2 / std::sqrt(delta2*delta2 + (-z2)*(-z2)));
    double theta_b = M_PI/6 - phi2;

    return Vector2d(theta_a, theta_b);
}

} // namespace hip_m2j
