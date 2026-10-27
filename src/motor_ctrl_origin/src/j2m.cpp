#include <iostream>
#include <cmath>
#include <vector>
#include <functional>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

using namespace Eigen;

// 定义常量结构体
struct Const {
    double l1;
    double l2_1;
    double l2_2;
    double h;
    double theta_0;
};

// 旋转矩阵函数
Matrix3d Rot_x(double q) {
    Matrix3d rot;
    rot << 1, 0, 0,
           0, cos(q), -sin(q),
           0, sin(q), cos(q);
    return rot;
}

Matrix3d Rot_y(double q) {
    Matrix3d rot;
    rot << cos(q), 0, sin(q),
           0, 1, 0,
           -sin(q), 0, cos(q);
    return rot;
}

// 计算C1点在O坐标系下的坐标
Vector3d calculate_C1_O(double alpha, double beta, const Const& c) {
    // C1点在O1系中的坐标
    Vector3d C1_O1(-c.l1 * (24.0/25.0),
               c.h,
               c.l1 * (7.0/25.0));

    // 旋转矩阵
    Matrix3d a_1 = Rot_y(alpha) * Rot_x(beta);

    // 齐次变换矩阵
    Matrix4d T_a_1 = Matrix4d::Identity();
    T_a_1.block<3,3>(0,0) = a_1;
    T_a_1(0,3) = 0;
    T_a_1(1,3) = 0;
    T_a_1(2,3) = -c.l2_1;
    T_a_1(3,3) = 1;

    // 齐次坐标变换
    Vector4d C1_O1_homog;
    C1_O1_homog << C1_O1, 1;
    Vector4d C1_O_homog = T_a_1 * C1_O1_homog;

    return C1_O_homog.head<3>();
}

// 计算C2点在O坐标系下的坐标
Vector3d calculate_C2_O(double alpha, double beta, const Const& c) {
    // C2点在O1系中的坐标
    Vector3d C2_O1(-c.l1 * (24.0/25.0),
               -c.h,
               c.l1 * (7.0/25.0));

    // 旋转矩阵
    Matrix3d a_2 = Rot_y(alpha) * Rot_x(beta);

    // 齐次变换矩阵
    Matrix4d T_a_2 = Matrix4d::Identity();
    T_a_2.block<3,3>(0,0) = a_2;
    T_a_2(0,3) = 0;
    T_a_2(1,3) = 0;
    T_a_2(2,3) = -c.l2_1;
    T_a_2(3,3) = 1;

    // 齐次坐标变换
    Vector4d C2_O1_homog;
    C2_O1_homog << C2_O1, 1;
    Vector4d C2_O_homog = T_a_2 * C2_O1_homog;

    return C2_O_homog.head<3>();
}

// 牛顿迭代法求解方程
double newtonSolve(std::function<double(double)> func, 
                   std::function<double(double)> dfunc, 
                   double initial_guess = 0.0, 
                   double tol = 1e-6, 
                   int max_iter = 100) {
    double x = initial_guess;
    for (int i = 0; i < max_iter; ++i) {
        double fx = func(x);
        double dfx = dfunc(x);
        
        if (std::abs(dfx) < tol) {
            std::cout << "牛顿迭代失败: 导数为零" << std::endl;
            return x;
        }
        
        double delta = fx / dfx;
        x -= delta;
        
        if (std::abs(delta) < tol) {
            break;
        }
    }
    return x;
}

int main() {
    // 1. 定义机构几何参数
    Const c;
    c.l1 = 100.0;     // 输出端连杆长
    c.l2_1 = 172.5;   // 长连杆长
    c.l2_2 = 172.5;   // 短连杆长
    c.h = 171.0 / 2.0; // 偏移距离
    c.theta_0 = std::atan(7.0 / 24.0); // 输出端初始角度

    // 2. 定义关节角
    double alpha = 14.48 * M_PI / 180.0; // 俯仰角
    double beta = 4.52 * M_PI / 180.0;   // 滚转角

    // 3. 求解坐标系O下C1与C2的坐标
    Vector3d C1_O = calculate_C1_O(alpha, beta, c);
    Vector3d C2_O = calculate_C2_O(alpha, beta, c);

    // 4. 在坐标系O下B1、B2、C1、C2点的坐标
    // 函数定义
    auto func1 = [&](double theta1) -> double {
        // B1点坐标
        double xB1 = -c.l1 * std::cos(theta1 + c.theta_0);
        double yB1 = c.h;
        double zB1 = c.l1 * std::sin(theta1 + c.theta_0);

        // 距离公式
        double dist_sq = std::pow(xB1 - C1_O[0], 2) + 
                         std::pow(yB1 - C1_O[1], 2) + 
                         std::pow(zB1 - C1_O[2], 2);
        
        return dist_sq - std::pow(c.l2_1, 2);
    };

    // 导数函数（使用中心差分近似）
    auto dfunc1 = [&](double theta1) -> double {
        const double h = 1e-6;
        return (func1(theta1 + h) - func1(theta1 - h)) / (2 * h);
    };

    auto func2 = [&](double theta2) -> double {
        // B2点坐标
        double xB2 = -c.l1 * std::cos(theta2 + c.theta_0);
        double yB2 = -c.h;
        double zB2 = c.l1 * std::sin(theta2 + c.theta_0) - (c.l2_1 - c.l2_2);

        // 距离公式
        double dist_sq = std::pow(xB2 - C2_O[0], 2) + 
                         std::pow(yB2 - C2_O[1], 2) + 
                         std::pow(zB2 - C2_O[2], 2);
        
        return dist_sq - std::pow(c.l2_2, 2);
    };

    // 导数函数（使用中心差分近似）
    auto dfunc2 = [&](double theta2) -> double {
        const double h = 1e-6;
        return (func2(theta2 + h) - func2(theta2 - h)) / (2 * h);
    };

    // 5. 使用牛顿迭代法求解方程
    double sol_theta_1 = newtonSolve(func1, dfunc1);
    double sol_theta_2 = newtonSolve(func2, dfunc2);

    // 转换为角度
    double sol_theta_1_deg = sol_theta_1 * 180.0 / M_PI;
    double sol_theta_2_deg = sol_theta_2 * 180.0 / M_PI;

    // 6. 输出结果
    std::cout << "===== 逆运动学闭式解 =====" << std::endl;
    std::cout << "已知 alpha = " << alpha << " rad (" << alpha*180.0/M_PI 
              << "°)，beta = " << beta << " rad (" << beta*180.0/M_PI 
              << "°)" << std::endl;
    std::cout << "求得：" << std::endl;
    std::cout << "  theta_1 = " << sol_theta_1 << " rad (" 
              << sol_theta_1_deg << "°)" << std::endl;
    std::cout << "  theta_2 = " << sol_theta_2 << " rad (" 
              << sol_theta_2_deg << "°)" << std::endl;

    return 0;
}