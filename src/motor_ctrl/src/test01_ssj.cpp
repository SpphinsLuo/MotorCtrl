#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "motor_ctrl/all_motor.h"

// 全局变量
static std::ofstream file[6];
std::shared_ptr<SerialPort> serial;
constexpr double pi = 3.14159265358979323846;

// 电机定义
motor_head::Motor motors[12] = {
    motor_head::Motor(motor_head::PA105, 0x03, 0x02), //hip pitch
    motor_head::Motor(motor_head::PA105, 0x04, 0x02), //hip roll
    motor_head::Motor(motor_head::PA105, 0x05, 0x02), //hip yaw
    motor_head::Motor(motor_head::PA105, 0x06, 0x02), //knee
    motor_head::Motor(motor_head::HA60, 0x07, 0x02), //ankle pitch
    motor_head::Motor(motor_head::HA60, 0x08, 0x02), // ankle roll
    motor_head::Motor(motor_head::PA105, 0x09, 0x02), //hip pitch
    motor_head::Motor(motor_head::PA105, 0x0A, 0x02), //hip roll
    motor_head::Motor(motor_head::PA105, 0x0B, 0x02), //hip yaw
    motor_head::Motor(motor_head::PA105, 0x0C, 0x02), //knee
    motor_head::Motor(motor_head::HA60, 0x0D, 0x02), //ankle pitch
    motor_head::Motor(motor_head::HA60, 0x0E, 0x02), // ankle roll
};

// 数据写入函数
void dataWriteTimewithSendingTime(int canID, int64_t time, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int whichFileObj, int64_t sendingTime){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + 
                            std::to_string(motor_tor) + " " + std::to_string(time)+" "+ std::to_string(sendingTime);
    std::ofstream& file_ = file[whichFileObj];
    
    if (!file_.is_open())
    {
        std::string save_path = "/home/ding/aaaWsLj/data/motor_ctrl/evo_motor_ctrl_withSendingTime/";
        std::string file_path = save_path + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file_.open(file_path, std::ios::app);
        if (!file_)
        {
            std::cerr << "Failed to open file!";
        }
    }
    file_ << contest << "\n";
}

// 角度转换函数
double degree2rad(double& degree){
    return degree * pi / 180.0;
}

double rad2degree(double& rad){
    return rad * 180.0 / pi;
}

// 五次多项式函数
double quinticPolynomial(double t, double a0, double a1, double a2, double a3, double a4, double a5) {
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
}

double quinticPolynomialVelocity(double t, double a1, double a2, double a3, double a4, double a5) {
    return a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t + 5 * a5 * t * t * t * t;
}

// 轨迹生成函数
void trajectoryGenerator(std::vector<double>& trajectory, std::vector<double>& velocity, double gain_traj){
    double t_start = 0.0;
    double t_end = 1.0;
    double dt = 0.001;

    // 第一部分轨迹系数
    double a0_1 = 0.0 * gain_traj;
    double a1_1 = 0.0 * gain_traj;
    double a2_1 = 0.0 * gain_traj;
    double a3_1 = 10.0 * gain_traj;
    double a4_1 = -15.0 * gain_traj;
    double a5_1 = 6.0 * gain_traj;

    // 第二部分轨迹系数
    double a0_2 = 1.0 * gain_traj;
    double a1_2 = 0.0 * gain_traj;
    double a2_2 = 0.0 * gain_traj;
    double a3_2 = -10.0 * gain_traj;
    double a4_2 = 15.0 * gain_traj;
    double a5_2 = -6.0 * gain_traj;

    // 生成时间序列
    std::vector<double> t;
    for (double time = t_start; time <= t_end; time += dt) {
        t.push_back(time);
    }

    std::vector<double> t2;
    for (double time = t_start + 1.0; time <= t_end + 1.0; time += dt) {
        t2.push_back(time);
    }

    // 生成第一部分轨迹
    for (size_t i = 0; i < t.size(); ++i) {
        double time = t[i];
        trajectory.push_back(quinticPolynomial(time, a0_1, a1_1, a2_1, a3_1, a4_1, a5_1));
        velocity.push_back(quinticPolynomialVelocity(time, a1_1, a2_1, a3_1, a4_1, a5_1));
    }

    // 生成第二部分轨迹
    for (size_t i = 0; i < t2.size(); ++i) {
        double time = t2[i];
        double shifted_time = time - 1.0;
        trajectory.push_back(quinticPolynomial(shifted_time, a0_2, a1_2, a2_2, a3_2, a4_2, a5_2));
        velocity.push_back(quinticPolynomialVelocity(shifted_time, a1_2, a2_2, a3_2, a4_2, a5_2));
    }
}

// 多段轨迹生成函数
void multiTrajGenerator(std::vector<double>& trajectory, double gain_traj, int sectionNum){
    if (sectionNum<=0) return;
    
    for (size_t i = 0; i < sectionNum; i++) {
        std::vector<double> singleTraj, singleVelTraj;
        if (i%2==0) {
            trajectoryGenerator(singleTraj, singleVelTraj, gain_traj);
        } else {
            trajectoryGenerator(singleTraj, singleVelTraj, -gain_traj);
        }
        
        for (size_t j = 0; j < singleTraj.size(); j++) {
            trajectory.push_back(singleTraj[j]);
        }
    }
}

// 主要控制函数
int left_leg_hip_pitch_ctrl(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    int motorID = 0;
    std::string filename_hip_pitch = std::to_string(date)+std::string("left_hip_pitch_") + std::to_string(testNum) + ".txt";

    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch, velocity_hip_pitch;
    multiTrajGenerator(trajectory_hip_pitch, targetpos, 5);

    // 轨迹指令结构体
    struct motorcmd {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    } mc_hip_pitch;

    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 200;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 20;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0;

    // 时间控制变量
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now();
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();
    auto sendingEnd = receiveEnd;
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();
    auto recordComTimeStart = receiveEnd;
    auto recordComTimeEnd = receiveEnd;
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now();

    // 主控制循环
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i++) {
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime);
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        
        if (remainingTime>0) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime)));
        } else {
            std::cout << "通信时间超出控制周期" << std::endl;
        }
    }

    // 稳定状态记录
    for (size_t i = 0; i < 500; i++) {
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime);
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        
        if (remainingTime>0) {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime)));
        } else {
            std::cout << "通信时间超出控制周期" << std::endl;
        }
    }
     
    mc.Motorevo_motor_servo_mode(motors[motorID], 0, mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++) {
        file[i].close();
    }
    
    return 0; 
}

// 测试函数
int test01(){
    left_leg_hip_pitch_ctrl(1, 20, 919);
    return 0;
}

// 主函数
int main(int argc, char const *argv[])
{
    test01();
    return 0;
}