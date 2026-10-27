#ifndef JOINT2MOTOR_H
#define JOINT2MOTOR_H

#include "ros/ros.h"

#include <Eigen/Dense>
#include <iostream>
#include <array>

// 定义向量类型为Eigen的动态大小向量
// using Vector = Eigen::VectorXd;

// 声明函数
// Vector J2M(const Vector& inputVector,int num);
//const 表示该参数是只读的，即在函数内部不能修改这个参数的值。Vector& 表示通过引用传递参数，而不是通过值传递。
std::array<float, 7> J2M(float inputArray[7]);
std::array<float, 3> J2M_S(float inputArray[3]); 
std::array<float, 3> M2J_S(float inputArray[3]); 
std::array<float, 2> J2M_E(float inputArray[2]);
std::array<float, 2> M2J_E(float inputArray[2]);
std::array<float, 2> J2M_W(float inputArray[2]);
std::array<float, 2> M2J_W(float inputArray[2]);
#endif 