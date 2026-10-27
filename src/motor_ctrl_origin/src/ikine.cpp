// #include <iostream>
// #include <Eigen/Dense>
// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/algorithm/kinematics.hpp>
// #include <pinocchio/algorithm/jacobian.hpp>
// #include <pinocchio/algorithm/frames.hpp>
// #include <pinocchio/math/log.hpp>
// #include <pinocchio/algorithm/pseudo-inverse.hpp>
// #include <pinocchio/algorithm/frames.hpp>

// int main()
// {
//   try {
//     // 1. 加载 URDF
//     const std::string urdf_filename = "bipedal_robot.urdf";  // 请将 robot.urdf 放在当前目录
//     pinocchio::Model model;
//     pinocchio::urdf::buildModel( urdf_filename, model);
//     pinocchio::Data data(model);

//     // 2. 初始关节角 (中立位)
//     Eigen::VectorXd q = pinocchio::neutral(model);

//     // 3. 定义目标末端位姿
//     Eigen::Vector3d xyz_des(0.2, 0.1, 0.5);
//     Eigen::Matrix3d R_des = Eigen::Matrix3d::Identity();
//     pinocchio::SE3 M_target(R_des, xyz_des);

//     // 4. IK 参数
//     const int max_iter = 100;
//     const double tol = 1e-6;
//     const double lambda = 1e-2;  // 阻尼系数

//     // 5. 获取末端 frame id
//     const std::string ee_frame = "end_effector";
//     const pinocchio::FrameIndex frame_id = model.getFrameId(ee_frame);
//     if(frame_id == pinocchio::INVALID_INDEX)
//       throw std::runtime_error("未找到末端 Frame: " + ee_frame);

//     // 6. 迭代求解
//     bool success = false;
//     for(int iter = 0; iter < max_iter; ++iter)
//     {
//       // 6.1 正运动学
//       pinocchio::forwardKinematics(model, data, q);
//       pinocchio::updateFramePlacements(model, data);
//       const pinocchio::SE3 & M_curr = data.oMf[frame_id];

//       // 6.2 计算误差 se3
//       const Eigen::Matrix<double,6,1> err = pinocchio::log6(M_curr.inverse() * M_target).toVector();

//       // 检查收敛
//       if(err.norm() < tol)
//       {
//         std::cout << "迭代 " << iter << " 次后收敛，误差 = " << err.norm() << std::endl;
//         success = true;
//         break;
//       }

//       // 6.3 计算雅可比 6xn
//       Eigen::MatrixXd J(6, model.nv);
//       pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::ReferenceFrame::LOCAL, J);

//       // 6.4 阻尼最小二乘求解 d q
//       Eigen::MatrixXd J_pinv = pinocchio::pseudoInverse(J, lambda);
//       Eigen::VectorXd dq = J_pinv * err;

//       // 6.5 更新 q
//       q = pinocchio::integrate(model, q, dq);
//     }

//     if(!success)
//       std::cerr << "警告：未在最大迭代次数内收敛 (" << max_iter << " 次)" << std::endl;

//     // 输出结果
//     std::cout << "最终关节角 (rad):\n" << q.transpose() << std::endl;
//   }
//   catch(const std::exception & ex) {
//     std::cerr << "异常: " << ex.what() << std::endl;
//     return -1;
//   }

//   return 0;
// }