#include <cstdio>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "motor_ctrl/all_motor.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <cctype>
#include <vector>
#include <cmath>
#include <iomanip>
#include <unistd.h>
#include <chrono>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// 一些全局变量
static std::ofstream file;  // 静态变量保持打开状态
static std::ofstream file1;  // 静态变量保持打开状态
static std::ofstream file2;  // 静态变量保持打开状态
static std::ofstream file3;  // 静态变量保持打开状态
const std::string PIPE_PATH = "/home/ye/00_jfl_26.10/MotorCtrl/ros_sync_pipe";

static std::ofstream file_traj;  // 静态变量保持打开状态

std::shared_ptr<SerialPort> serial;
constexpr double pi = 3.14159265358979323846;  // 推荐使用constexpr

// 一些结构体的定义
struct servoPID
{
    float posKp = 0;
    float posKd = 0;
    float velKp = 0;
    float velKd = 0;
    float velKi = 0;
    float velLimit = 0;
};

// 读取状态时，因为读取失败重发的次数
struct reSendNumCanopen
{
    int posSendNum = 0;
    int velSendNum = 0;
    int torSendNum = 0;

};


// elmo驱动器状态的定义
struct elmoState
{
    uint32_t pulseNum = 0;
    double pos = 0;
    double vel = 0;
    double tor = 0; // 力矩
};

 

// one leg
motor_head::Motor motors[12] = {
    // index == canID 参数列表：电机类型 从站ID 主站ID
    motor_head::Motor(motor_head::PA100, 0x03, 0x02), //hip pitch
    motor_head::Motor(motor_head::PA100, 0x04, 0x02), //hip roll
    motor_head::Motor(motor_head::PA100, 0x05, 0x02), //hip yaw
    motor_head::Motor(motor_head::ELMO, 0x06, 0x02), //knee
    motor_head::Motor(motor_head::HA60, 0x07, 0x02), //ankle pitch
    motor_head::Motor(motor_head::HA60, 0x08, 0x02), // ankle roll

    motor_head::Motor(motor_head::PA100, 0x09, 0x02), //hip pitch
    motor_head::Motor(motor_head::PA100, 0x0A, 0x02), //hip roll
    motor_head::Motor(motor_head::PA100, 0x0B, 0x02), //hip yaw
    motor_head::Motor(motor_head::ELMO, 0x0C, 0x02), //knee 12
    motor_head::Motor(motor_head::HA60, 0x0D, 0x02), //ankle pitch
    motor_head::Motor(motor_head::HA60, 0x0E, 0x02) // ankle roll

};


// 一些函数声明
void dataWrite(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename);
void dataWrite2(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename);
std::string execute_command(const char* cmd);
double quinticPolynomial(double t, double a0, double a1, double a2, double a3, double a4, double a5);
double quinticPolynomialVelocity(double t, double a1, double a2, double a3, double a4, double a5);
void trajectoryGenerator(std::vector<double>& trajectory, std::vector<double>& velocity, double gain_traj = 1);
void dataWrite1(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename);
void dataWrite2(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename);
void dataWrite3(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename);
void dataWriteTime(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int64_t time);
void dataWriteTime1(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int64_t time);
void dataWriteTimewithSendingTime(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int64_t state_time, int64_t cmd_send_time);

// void elmo_read_state(elmoState& state_, elmoState& laststate_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall);
void elmo_read_state(elmoState& state_, elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall);
bool elmo_read_init_state(elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_);
void elmo_read_stateV2(elmoState& state_, elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall);

int elmo_read_stateV3(elmoState& state_, elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall, reSendNumCanopen& reSend);
// double rad2degree(double& rad){
//     return rad * 180.0 / pi;
// }
// 轨迹数据结构
struct TrajectoryPoint {
    double time;
    std::vector<double> joint_angles;
};

// 加载轨迹数据函数
std::vector<TrajectoryPoint> loadTrajectory(const std::string& filename) {
    std::vector<TrajectoryPoint> trajectory;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error opening trajectory file: " << filename << std::endl;
        return trajectory;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        TrajectoryPoint point;
        point.joint_angles.resize(12); // 12个关节角度
        
        // 读取时间
        ss >> point.time;
        
        // 读取12个关节角度
        for (int i = 0; i < 12; ++i) {
            ss >> point.joint_angles[i];
        }
        
        trajectory.push_back(point);
    }
    
    file.close();
    return trajectory;
}

double ankleInvKinHighTheta1(double alphy, double beta);
double ankleInvKinLowTheta2(double alphy, double beta);
double ankleInvKinHighTheta1_R(double alphy, double beta);
double ankleInvKinLowTheta2_R(double alphy, double beta);
void anklePosCtrlState(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid);
void anklePosCtrlState_R(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid);
// void anklePosCtrlStateRTest(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid);
void anklePosCtrlStateDoubleCtrl(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid);

int elmo_pos_ctrl_SDO();
int elmo_pos_ctrl_SDO(double posDes);
int elmo_pos_ctrl_SDO(double posDes, elmoState& state);
int elmo_pos_traj_ctrl_SDO(double posDesMax, int testNum);
int elmo_pos_traj_ctrl_SDO_1(double posDesMax, int testNum);
int elmo_pos_traj_ctrl_SDO_2(double posDesMax, int testNum);
int elmo_pos_traj_ctrl_SDO_3(double posDesMax1, double posDesMax2, int testNum1, int testNum2);

// 一些简单函数的定义
double degree2rad(double& degree);
double rad2degree(double& rad);
std::vector<std::string> split(const std::string& s, char delimiter);
std::vector<std::vector<std::string>> readCSV(const std::string& filename);
void CSV_string2double(std::vector<std::vector<double>>* result);
// void printHex(uint8_t value, bool uppercase = false);
template <typename T>
typename std::enable_if<std::is_integral<T>::value>::type
printHex(T value, bool prefix = true, bool uppercase = false);
bool checkSDOCorrect(motor_head::Motor_Control& mc_, motor_head::CAN_Receive_Frame& return_canObj, int& receive_flag);
uint32_t littleToBigEndian(const uint8_t* little_endian_bytes, size_t byte_count);

// 函数功能：将一维数组写入 TXT 文件
// 参数：
//   data: 要写入的一维数组
//   size: 数组元素个数
//   filename: 目标文件名
void writeVectorToTxt(const std::vector<double>& data, const std::string& filename) {
    // 打开文件流，若文件不存在则创建，若存在则覆盖
    std::ofstream outFile(filename);
    
    // 检查文件是否成功打开
    if (!outFile.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }
    
    // 写入数组元素，每个元素占一行
    for (int i = 0; i < data.size(); ++i) {
        outFile << data[i] << std::endl;
    }
    
    // 关闭文件
    outFile.close();
    std::cout << "数组已成功写入文件: " << filename << std::endl;
}


/**
 * @brief applicable to hip pitch & ankle  disapplicable to knee  cause 0->x->0->(-x)->0
*/
void multiTrajGenerator_kneeVersion(std::vector<double>& trajectory, double gain_traj, int sectionNum){
    if (sectionNum<=0)
    {
        return;
    }
    
    for (size_t i = 0; i < sectionNum; i++)
    {
        std::vector<double> singleTraj, singleVelTraj;
        
        trajectoryGenerator(singleTraj, singleVelTraj, gain_traj);
        
        
        for (size_t j = 0; j < singleTraj.size(); j++)
        {
            trajectory.push_back(singleTraj[j]);
        }
    }
    
}

void testTraj(){
    std::vector<double> traj;
    multiTrajGenerator_kneeVersion(traj, 10, 5);
    writeVectorToTxt(traj, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj922.txt");
}

int test01(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    
    dm.addMotor(&motors[0]);
    dm.enable(motors[0]);
    dm.Motorevo_set_zero_position(motors[0]);
    
    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    dm.Motorevo_motor_servo_mode(motors[0],-28.65,10,0,4.7746,10,0,0);
    sleep(5);

    dm.disable(motors[0]);
    return 0;
}

// 通信反馈 未修改完，谨慎使用
int test02(){

    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    int motorID = 1;
    
    dm.addMotor(&motors[motorID]);
    dm.enable(motors[motorID]);
    dm.Motorevo_set_zero_position(motors[motorID]);
    
    
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    std::string filename = std::string("test01") + ".txt";


    dm.Motorevo_motor_servo_mode(motors[motorID],0,0,0,0,0,0,0);
    // while (1)
    // {   
    //     dm.Motorevo_motor_servo_mode(motors[motorID],0,0,0,0,0,0,0);
    //     dm.receive_evo();
    
    //     std::cout << "motor_state.id: "<< 4 << std::endl;
        
    //     std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
    //     std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
    //     std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
    //     // dataWrite(4, motors[3].Get_Position(), motors[3].Get_Velocity(), motors[3].Get_tau(), &filename); // 保存数据到文件中
    // }

    dm.disable(motors[motorID]);
    file.close();
    
    return 0; 

}

// 多电机
int test03(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    for (size_t i = 0; i < 3; i++)
    {
        dm.addMotor(&motors[i]);
        dm.enable(motors[i]);
        dm.Motorevo_set_zero_position(motors[i]);
    }
    
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();


    dm.Motorevo_motor_servo_mode(motors[0],-28.65,10,0,4.7746,10,0,0);
    dm.Motorevo_motor_servo_mode(motors[1],-9.74,10,0,4.7746,10,0,0);
    dm.Motorevo_motor_servo_mode(motors[2], 9.74,10,0,4.7746,10,0,0);
    sleep(5);

    for (size_t i = 0; i < 3; i++)
    {
        dm.disable(motors[i]);
    }
    return 0;

}

// 单电机控制轨迹控制及误差
int test04(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    int motorID = 5;
    int pos_max_desired = -10;
    std::string filename_ankle_pitch = std::string("test714ankleLMotor") + ".txt";
    
    dm.addMotor(&motors[motorID]);
    dm.enable(motors[motorID]);
    dm.Motorevo_set_zero_position(motors[motorID]);
    
    
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    

    // 轨迹产生
    std::vector<double> trajectory,velocity;
    trajectoryGenerator(trajectory, velocity);

    // 轨迹转换为指令 
    // 1.乘以系数，最大期望角度1->30度  2.速度不变
    // 1.轨迹处理
    
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        trajectory[i] = trajectory[i] * pos_max_desired;
    }
    
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_pitch, mc_ankle_roll;
    mc_ankle_pitch.pos_desired = trajectory;
    mc_ankle_pitch.velocity_limit = 10;
    mc_ankle_pitch.posKp = 5;
    mc_ankle_pitch.posKd = 0;
    mc_ankle_pitch.velKp = 5;
    mc_ankle_pitch.velKd = 0;
    mc_ankle_pitch.velKi = 0;


    for (size_t i = 0; i <= trajectory.size(); i++)
    {
        // ankle pitch
        dm.Motorevo_motor_servo_mode(motors[motorID], mc_ankle_pitch.pos_desired[i], mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite(5, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), motors[motorID].Get_tau(), &filename_ankle_pitch); // 保存数据到文件中
        usleep(1000);
    }
    
    dm.disable(motors[motorID]);
    file.close();
    
    return 0; 
}


// 轨迹控制及误差 + 多电机控制 success index已改但未验证
int test05(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[4]);
    dm.addMotor(&motors[5]);
    dm.enable(motors[4]);
    dm.enable(motors[5]);
    dm.Motorevo_set_zero_position(motors[4]);
    dm.Motorevo_set_zero_position(motors[5]);
    
    
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    std::string filename_ankle_pitch = std::string("test01_ankle_pitch") + ".txt";
    std::string filename_ankle_roll = std::string("test01_ankle_roll") + ".txt";

    // 轨迹产生
    std::vector<double> trajectory,velocity;
    trajectoryGenerator(trajectory, velocity);

    // 轨迹转换为指令 
    // 1.乘以系数，最大期望角度1->30度  2.速度不变
    // 1.轨迹处理
    int pos_max_desired = 10;
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        trajectory[i] = trajectory[i] * pos_max_desired;
    }
    
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_pitch, mc_ankle_roll;
    mc_ankle_pitch.pos_desired = trajectory;
    mc_ankle_pitch.velocity_limit = 30;
    mc_ankle_pitch.posKp = 250;
    mc_ankle_pitch.posKd = 0;
    mc_ankle_pitch.velKp = 50;
    mc_ankle_pitch.velKd = 0;
    mc_ankle_pitch.velKi = 0;

    mc_ankle_roll.pos_desired = trajectory;
    mc_ankle_roll.velocity_limit = 30;
    mc_ankle_roll.posKp = 250;
    mc_ankle_roll.posKd = 0;
    mc_ankle_roll.velKp = 50;
    mc_ankle_roll.velKd = 0;
    mc_ankle_roll.velKi = 0;

    for (size_t i = 0; i <= trajectory.size(); i++)
    {
        // ankle pitch
        dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_pitch.pos_desired[i], mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite(4, motors[4].Get_Position(), motors[4].Get_Velocity(), motors[4].Get_tau(), &filename_ankle_pitch); // 保存数据到文件中

        // ankle roll
        dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_roll.pos_desired[i], mc_ankle_roll.posKp, mc_ankle_roll.posKd, mc_ankle_roll.velocity_limit, mc_ankle_roll.velKp, mc_ankle_roll.velKd, mc_ankle_roll.velKi);
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite1(5, motors[5].Get_Position(), motors[5].Get_Velocity(), motors[5].Get_tau(), &filename_ankle_roll); // 保存数据到文件中
        usleep(1000);
    }
     
        
    

    dm.disable(motors[4]);
    dm.disable(motors[5]);
    file.close();
    file1.close();

    
    return 0; 
}

// 轨迹控制及误差 + 髋关节多电机控制 
int test06(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    // 控制的i个电机的canID 
    int motorID[3] = {0, 1, 2};

    std::string filename_hip_pitch = std::string("test715_hip_pitch") + ".txt";
    std::string filename_hip_roll = std::string("test715_hip_roll") + ".txt";
    std::string filename_hip_yaw = std::string("test715_hip_yaw") + ".txt";


    // dm.enable(motors[0]);
    dm.enable(motors[1]);
    // dm.enable(motors[2]);

    for (size_t i = 0; i < 3; i++)
    {
        dm.addMotor(&motors[i]);
        
        dm.Motorevo_set_zero_position(motors[i]);
    }

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);
    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch,
                        trajectory_hip_roll,velocity_hip_roll,
                        trajectory_hip_yaw,velocity_hip_yaw;
    double traj_hip_pitch_max_pos = -105;
    double traj_hip_roll_max_pos = 60;
    double traj_hip_yaw_max_pos = 45;

    trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, traj_hip_pitch_max_pos);
    trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, traj_hip_roll_max_pos);
    trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, traj_hip_yaw_max_pos);


    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch, mc_hip_roll, mc_hip_yaw;

    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 200;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 200;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0.01;

    mc_hip_roll.pos_desired = trajectory_hip_roll;
    mc_hip_roll.velocity_limit = 200;
    mc_hip_roll.posKp = 50;
    mc_hip_roll.posKd = 10;
    mc_hip_roll.velKp = 50;
    mc_hip_roll.velKd = 0;
    mc_hip_roll.velKi = 0.001;

    mc_hip_yaw.pos_desired = trajectory_hip_yaw;
    mc_hip_yaw.velocity_limit = 10;
    mc_hip_yaw.posKp = 30;
    mc_hip_yaw.posKd = 0;
    mc_hip_yaw.velKp = 50;
    mc_hip_yaw.velKd = 0;
    mc_hip_yaw.velKi = 0;

    for (size_t i = 0; i <= trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        dm.Motorevo_motor_servo_mode(motors[0], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        dm.receive_evo();
        dataWrite(motorID[0], motors[0].Get_Position(), motors[0].Get_Velocity(), motors[0].Get_tau(), &filename_hip_pitch); // 保存数据到文件中

        // hip roll
        dm.Motorevo_motor_servo_mode(motors[1], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        dm.receive_evo();
        dataWrite1(motorID[1], motors[1].Get_Position(), motors[1].Get_Velocity(), motors[1].Get_tau(), &filename_hip_roll); // 保存数据到文件中

        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[2], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        dm.receive_evo();
        dataWrite2(motorID[2], motors[2].Get_Position(), motors[2].Get_Velocity(), motors[2].Get_tau(), &filename_hip_yaw); // 保存数据到文件中

        usleep(1000);
    }
     
        
    
    for (size_t i = 0; i < 3; i++)
    {
        dm.disable(motors[i]);
    }
    file.close();
    file1.close();
    file2.close();
    
    return 0; 
}

// 加了逆运动学的左踝关节控制
int test07(){
    double apitch = 10;
    double aroll = 10;
    double posMotorHigh = ankleInvKinHighTheta1(apitch, aroll);
    double posMotorLow = ankleInvKinLowTheta2(apitch, aroll);

    std::cout << "posMotorHigh is:" <<posMotorHigh<< std::endl;
    std::cout << "posMotorHigh is:" <<posMotorLow<< std::endl;


    servoPID highM,lowM;
    // 高位电机PID设置
    highM.velLimit = 10;
    highM.posKp = 50;
    highM.posKd = 0;
    highM.velKp = 30;
    highM.velKd = 0;
    highM.velKi = 0;
    // 地位电机PID设置
    lowM.velLimit = 10;
    lowM.posKp = 50;
    lowM.posKd = 0;
    lowM.velKp = 30;
    lowM.velKd = 0;
    lowM.velKi = 0;
    anklePosCtrlState(apitch, aroll, highM, lowM);
    return 0;
}

// 加了逆运动学的右踝关节控制
int test071(){
    double apitch = 10;
    double aroll = 10;
    double posMotorHigh = ankleInvKinHighTheta1_R(apitch, aroll);
    double posMotorLow = ankleInvKinLowTheta2_R(apitch, aroll);

    std::cout << "posMotorHigh is:" <<posMotorHigh<< std::endl;
    std::cout << "posMotorHigh is:" <<posMotorLow<< std::endl;


    servoPID highM,lowM;
    // 高位电机PID设置
    highM.velLimit = 10;
    highM.posKp = 50;
    highM.posKd = 0;
    highM.velKp = 30;
    highM.velKd = 0;
    highM.velKi = 0;
    // 地位电机PID设置
    lowM.velLimit = 10;
    lowM.posKp = 50;
    lowM.posKd = 0;
    lowM.velKp = 30;
    lowM.velKd = 0;
    lowM.velKi = 0;
    anklePosCtrlState_R(apitch, aroll, highM, lowM);
    return 0;
}


// 加了逆运动学的 左 右 踝关节控制 success
int test072(){
    double apitch = 10;
    double aroll = 0;
    double posMotorHigh = ankleInvKinHighTheta1(apitch, aroll);
    double posMotorLow = ankleInvKinLowTheta2(apitch, aroll);
    double posMotorHigh_R = ankleInvKinHighTheta1_R(apitch, aroll);
    double posMotorLow_R = ankleInvKinLowTheta2_R(apitch, aroll);

    std::cout << "posMotorHigh is:" <<posMotorHigh<< std::endl;
    std::cout << "posMotorLow is:" <<posMotorLow<< std::endl;
    std::cout << "posMotorHigh_R is:" <<posMotorHigh_R<< std::endl;
    std::cout << "posMotorLow_R is:" <<posMotorLow_R<< std::endl;

    servoPID highM,lowM;
    // 高位电机PID设置
    highM.velLimit = 10;
    highM.posKp = 50;
    highM.posKd = 0;
    highM.velKp = 30;
    highM.velKd = 0;
    highM.velKi = 0;
    // 地位电机PID设置
    lowM.velLimit = 10;
    lowM.posKp = 50;
    lowM.posKd = 0;
    lowM.velKp = 30;
    lowM.velKd = 0;
    lowM.velKi = 0;
    anklePosCtrlStateDoubleCtrl(apitch, aroll, highM, lowM);

    apitch = -10;
    aroll = 0;
    anklePosCtrlStateDoubleCtrl(apitch, aroll, highM, lowM);

    return 0;
}

// 状态读取
int test08(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[4]);
    dm.enable(motors[4]);
    dm.Motorevo_set_zero_position(motors[4]);
    
    
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    std::string filename_ankle_pitch = std::string("test01_ankle_pitch") + ".txt";

    // 轨迹产生
    std::vector<double> trajectory,velocity;
    trajectoryGenerator(trajectory, velocity);

    // 轨迹转换为指令 
    // 1.乘以系数，最大期望角度1->30度  2.速度不变
    // 1.轨迹处理
    int pos_max_desired = 10;
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        trajectory[i] = trajectory[i] * pos_max_desired;
    }
    
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_pitch, mc_ankle_roll;
    mc_ankle_pitch.pos_desired = trajectory;
    mc_ankle_pitch.velocity_limit = 0;
    mc_ankle_pitch.posKp = 0;
    mc_ankle_pitch.posKd = 0;
    mc_ankle_pitch.velKp = 0;
    mc_ankle_pitch.velKd = 0;
    mc_ankle_pitch.velKi = 0;


    for (size_t i = 0; i <= trajectory.size(); i++)
    {
        // ankle pitch
        dm.Motorevo_motor_servo_mode(motors[4], 0, mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite(4, motors[4].Get_Position(), motors[4].Get_Velocity(), motors[4].Get_tau(), &filename_ankle_pitch); // 保存数据到文件中
        usleep(1000);
    }
     
        
    

    dm.disable(motors[4]);
    file.close();
    
    return 0; 
}

// 髋关节的多电机控制
int test09(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[0]);
    dm.addMotor(&motors[1]);
    dm.addMotor(&motors[2]);

    dm.enable(motors[0]);
    dm.enable(motors[1]);
    dm.enable(motors[2]);

    dm.Motorevo_set_zero_position(motors[0]);
    dm.Motorevo_set_zero_position(motors[1]);
    dm.Motorevo_set_zero_position(motors[2]);

    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_pitch;
    mc_ankle_pitch.velocity_limit = 50;
    mc_ankle_pitch.posKp = 10;
    mc_ankle_pitch.posKd = 5;
    mc_ankle_pitch.velKp = 10;
    mc_ankle_pitch.velKd = 5;
    mc_ankle_pitch.velKi = 0.01;

    
    dm.Motorevo_motor_servo_mode(motors[0], 30, mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
    dm.Motorevo_motor_servo_mode(motors[1], 30, mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
    dm.Motorevo_motor_servo_mode(motors[2], 30, mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
    
    sleep(3);
    dm.disable(motors[0]);
    dm.disable(motors[1]);
    dm.disable(motors[2]);
    return 0;
}

// 加了逆运动学的踝关节控制
// int test10(){
//     double apitch = 30;
//     double aroll = 0;
//     double posMotorHigh = ankleInvKinHighTheta1(apitch, aroll);
//     double posMotorLow = ankleInvKinLowTheta2(apitch, aroll);

//     std::cout << "posMotorHigh is:" <<posMotorHigh<< std::endl;
//     std::cout << "posMotorHigh is:" <<posMotorLow<< std::endl;


//     servoPID highM,lowM;
//     // 高位电机PID设置
//     highM.velLimit = 10;
//     highM.posKp = 50;
//     highM.posKd = 5;
//     highM.velKp = 30;
//     highM.velKd = 0;
//     highM.velKi = 0;
//     // 地位电机PID设置
//     lowM.velLimit = 10;
//     lowM.posKp = 50;
//     lowM.posKd = 5;
//     lowM.velKp = 30;
//     lowM.velKd = 0;
//     lowM.velKi = 0;
//     anklePosCtrlStateRTest(apitch, aroll, highM, lowM);
//     return 0;
// }

// 右踝关节的多电机控制
int test11(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[10]);
    dm.addMotor(&motors[11]);

    dm.enable(motors[10]);
    dm.enable(motors[11]);

    dm.Motorevo_set_zero_position(motors[10]);
    dm.Motorevo_set_zero_position(motors[11]);


    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_pitch;
    mc_ankle_pitch.velocity_limit = 50;
    mc_ankle_pitch.posKp = 10;
    mc_ankle_pitch.posKd = 5;
    mc_ankle_pitch.velKp = 10;
    mc_ankle_pitch.velKd = 5;
    mc_ankle_pitch.velKi = 0.01;

    
    dm.Motorevo_motor_servo_mode(motors[10], 30, mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);
    dm.Motorevo_motor_servo_mode(motors[11], 30, mc_ankle_pitch.posKp, mc_ankle_pitch.posKd, mc_ankle_pitch.velocity_limit, mc_ankle_pitch.velKp, mc_ankle_pitch.velKd, mc_ankle_pitch.velKi);

    
    sleep(3);
    dm.disable(motors[10]);
    dm.disable(motors[11]);
    return 0;
}


int main()
{  
    // double apitch, aroll;
    // apitch = 10;
    // aroll = 10;
    // double hmotor,lmotor;
    // hmotor = ankleInvKinHighTheta1_R(apitch, aroll);
    // lmotor = ankleInvKinLowTheta2_R(apitch, aroll);
    // std::cout << "high motor angle is:" << hmotor << std::endl;
    // std::cout << "low motor angle is:" << lmotor << std::endl;
    
    // int result = elmo_pos_ctrl_SDO(30);
    // testTraj();


    int result = elmo_pos_traj_ctrl_SDO_2(30, 2);
    // // int result = elmo_pos_traj_ctrl_SDO_3(5, 5, 3, 3);
    if (result == -1)
    {
        std::cout << "SDO正确性检查失败" << std::endl;
    }
    
    return 0;
}

void dataWriteTime(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int64_t time){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor) + " " + std::to_string(time);
    
    if (!file.is_open())
    {
        // std::string package_name = "motor_ctrl";
        // std::string cmd = "rospack find " + package_name;
        // std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出
        
        // ros2 lib find package
        // std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        std::string save_path = "/home/ye/00_jfl_26.10/MotorCtrl/data/motor_ctrl/";
        std::string file_path = save_path + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file.open(file_path, std::ios::app);
        if (!file)
        {
            std::cerr << "Failed to open file!";
            
        }
        
    }
    file << contest << "\n";
}

void dataWriteTimewithSendingTime(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int64_t state_time, int64_t cmd_send_time){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor) + " " + std::to_string(state_time) + " " + std::to_string(cmd_send_time);
    
    if (!file.is_open())
    {
        // std::string package_name = "motor_ctrl";
        // std::string cmd = "rospack find " + package_name;
        // std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出
        
        // ros2 lib find package
        // std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        std::string save_path = "/home/ye/00_jfl_26.10/MotorCtrl/data/motor_ctrl/elmolike_motor/";
        std::string file_path = save_path + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file.open(file_path, std::ios::app);
        if (!file)
        {
            std::cerr << "Failed to open file!";
            
        }
        
    }
    file << contest << "\n";
}

void dataWriteTime1(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int64_t time){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor) + " " + std::to_string(time);
    
    if (!file1.is_open())
    {
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出

        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/record/" + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file1.open(file_path, std::ios::app);
        if (!file1)
        {
            std::cerr << "Failed to open file!";
            
        }
        
    }
    file1 << contest << "\n";
}

void dataWrite(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor);
    
    if (!file.is_open())
    {
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出

        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/record/" + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file.open(file_path, std::ios::app);
        if (!file)
        {
            std::cerr << "Failed to open file!";
            
        }
        
    }
    file << contest << "\n";
}


void dataWrite1(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor);
    
    if (!file1.is_open())
    {
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出

        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/record/" + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file1.open(file_path, std::ios::app);
        if (!file1)
        {
            std::cerr << "Failed to open file1!";
            
        }
        
    }
    file1 << contest << "\n";
}

void dataWrite2(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor);
    
    if (!file2.is_open())
    {
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出

        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/record/" + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file2.open(file_path, std::ios::app);
        if (!file2)
        {
            std::cerr << "Failed to open file1!";
            
        }
        
    }
    file2 << contest << "\n";
}

void dataWrite3(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor);
    
    if (!file3.is_open())
    {
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出

        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/record/" + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file3.open(file_path, std::ios::app);
        if (!file3)
        {
            std::cerr << "Failed to open file1!";
            
        }
        
    }
    file3 << contest << "\n";
}

void dataWrite_traj(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor);
    
    if (!file_traj.is_open())
    {
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出

        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/record/" + *filename;
        std::cout << "file_path is "<< file_path << std::endl;
        file_traj.open(file_path, std::ios::app);
        if (!file_traj)
        {
            std::cerr << "Failed to open file_traj!";
            
        }
        
    }
    file_traj << contest << "\n";
}

std::string execute_command(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    // 打开管道读取命令输出
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() 失败！");

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();  // 逐行读取输出
    }
    return result;  // 返回完整输出字符串
}

void trajectoryGenerator(std::vector<double>& trajectory, std::vector<double>& velocity, double gain_traj){
    // 参数定义
    double t_start = 0.0;  // 起始时间
    double t_end = 1.0;    // 结束时间
    double dt = 0.001;     // 时间间隔

    // 第一部分轨迹的系数
    double a0_1 = 0.0 * gain_traj;
    double a1_1 = 0.0 * gain_traj;
    double a2_1 = 0.0 * gain_traj;
    double a3_1 = 10.0 * gain_traj;
    double a4_1 = -15.0 * gain_traj;
    double a5_1 = 6.0 * gain_traj;

    // 第二部分轨迹的系数
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

    // 生成第二部分的时间序列（偏移 1 秒）
    std::vector<double> t2;
    for (double time = t_start + 1.0; time <= t_end + 1.0; time += dt) {
        t2.push_back(time);
    }

    // 生成完整的轨迹、速度
    // std::vector<double> trajectory;
    // std::vector<double> velocity;


    // 第一部分轨迹
    for (size_t i = 0; i < t.size(); ++i) {
        double time = t[i];
        trajectory.push_back(quinticPolynomial(time, a0_1, a1_1, a2_1, a3_1, a4_1, a5_1));
        velocity.push_back(quinticPolynomialVelocity(time, a1_1, a2_1, a3_1, a4_1, a5_1));
    }

    // 第二部分轨迹
    for (size_t i = 0; i < t2.size(); ++i) {
        double time = t2[i];
        double shifted_time = time - 1.0; // 偏移 1 秒
        trajectory.push_back(quinticPolynomial(shifted_time, a0_2, a1_2, a2_2, a3_2, a4_2, a5_2));
        velocity.push_back(quinticPolynomialVelocity(shifted_time, a1_2, a2_2, a3_2, a4_2, a5_2));
    }

    // 输出结果
    std::cout << std::fixed << std::setprecision(3);
    // std::cout << "时间\t位置\t速度\t加速度\n";
    std::cout << "位置\n";
    // 打印第一部分轨迹
    for (size_t i = 0; i < t.size(); ++i) {
        // std::cout << t[i] << "\t" << trajectory[i] << "\t" << velocity[i] <<"\n";
        std::cout << trajectory[i] << " ";
    }

    // 打印第二部分轨迹
    for (size_t i = 0; i < t2.size(); ++i) {
        // std::cout << t2[i] << "\t" << trajectory[t.size() + i] << "\t" << velocity[t.size() + i] <<"\n";
        std::cout << trajectory[t.size() + i] << " ";

    }
    std::cout << "trajectory.size() is:" << trajectory.size() << "\n";
    std::cout << "velocity.size() is:" << velocity.size() << "\n";
    std::cout << "t.size() is:" << t.size() << "\n";
    std::cout << "t2.size() is:" << t2.size() << "\n";


    
}


// 定义五次多项式函数
double quinticPolynomial(double t, double a0, double a1, double a2, double a3, double a4, double a5) {
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
}

// 定义五次多项式的一阶导数（速度）
double quinticPolynomialVelocity(double t, double a1, double a2, double a3, double a4, double a5) {
    return a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t + 5 * a5 * t * t * t * t;
}

/** 
 * @brief ankle Inverse Kinematic part1 success(1 try)
 * @param alphy ankle pitch (degree
 * @param beta ankle roll   (degree
 * @retval ankle high position motor angle
 */
double ankleInvKinHighTheta1(double alphy, double beta){
    // 角度转换 degree->rad
    // [&alphy,&beta](){
    //     alphy = alphy * pi / 180;
    //     beta = beta * pi / 180;
    // }();
    alphy = degree2rad(alphy);
    beta = degree2rad(beta);
    
    static const double indexVector[15] = {-0.0001,
    1.0002,
    0.8669,
    0.0033,
    0.2580,
    -0.0483,
    0.0065,
    0.0846,
    0.3987,
    -0.1122,
    0.0036,
    0.0951,
    0.1759,
    -0.1427,
    -0.1355};

    double result = indexVector[0] + indexVector[1]*alphy + indexVector[2]*beta + indexVector[3]*std::pow(alphy,2) + indexVector[4]*alphy*beta
    + indexVector[5]*std::pow(beta,2) + indexVector[6]*std::pow(alphy,3) + indexVector[7]*std::pow(alphy,2)*beta + indexVector[8]*alphy*std::pow(beta,2)
    + indexVector[9]*std::pow(beta,3) + indexVector[10]*std::pow(alphy,4) + indexVector[11]*std::pow(alphy,3)*beta + indexVector[12]*std::pow(alphy,2)*std::pow(beta,2)
    + indexVector[13]*alphy*std::pow(beta,3) + indexVector[14]*std::pow(beta,4); 

    result = rad2degree(result);
    
    return result;
}


/** 
 * @brief ankle Inverse Kinematic part2 success(1 try)
 * @param alphy ankle pitch
 * @param beta ankle roll
 * @retval ankle low position motor angle 
 */
double ankleInvKinLowTheta2(double alphy, double beta){
    // 角度转换 degree->rad
    alphy = degree2rad(alphy);
    beta = degree2rad(beta);

    static const double indexVector[15] = {-0.0001,
    1.0003,
    -0.8669,
    0.0036,
    -0.2578,
    -0.0568,
    0.0070,
    -0.0837,
    0.3918,
    0.1361,
    0.0039,
    -0.0943,
    0.1663,
    0.1493,
    -0.1331};
    
    double result = indexVector[0] + indexVector[1]*alphy + indexVector[2]*beta + indexVector[3]*std::pow(alphy,2) + indexVector[4]*alphy*beta
    + indexVector[5]*std::pow(beta,2) + indexVector[6]*std::pow(alphy,3) + indexVector[7]*std::pow(alphy,2)*beta + indexVector[8]*alphy*std::pow(beta,2)
    + indexVector[9]*std::pow(beta,3) + indexVector[10]*std::pow(alphy,4) + indexVector[11]*std::pow(alphy,3)*beta + indexVector[12]*std::pow(alphy,2)*std::pow(beta,2)
    + indexVector[13]*alphy*std::pow(beta,3) + indexVector[14]*std::pow(beta,4); 

    result = -rad2degree(result);
    
    return result;
}


/** 
 * @brief ankle Inverse Kinematic part1 success(1 try)
 * @param alphy ankle pitch (degree
 * @param beta ankle roll   (degree
 * @retval ankle high position motor angle
 */
double ankleInvKinHighTheta1_R(double alphy, double beta){
    // 角度转换 degree->rad
    // [&alphy,&beta](){
    //     alphy = alphy * pi / 180;
    //     beta = beta * pi / 180;
    // }();
    alphy = degree2rad(alphy);
    beta = degree2rad(beta);
    
    static const double indexVector[15] = {-0.0001,
    1.0003,
    0.8669,
    0.0036,
    0.2578,
    -0.0568,
    0.0070,
    0.0837,
    0.3918,
    -0.1361,
    0.0039,
    0.0943,
    0.1663,
    -0.1493,
    -0.1331};

    double result = indexVector[0] + indexVector[1]*alphy + indexVector[2]*beta + indexVector[3]*std::pow(alphy,2) + indexVector[4]*alphy*beta
    + indexVector[5]*std::pow(beta,2) + indexVector[6]*std::pow(alphy,3) + indexVector[7]*std::pow(alphy,2)*beta + indexVector[8]*alphy*std::pow(beta,2)
    + indexVector[9]*std::pow(beta,3) + indexVector[10]*std::pow(alphy,4) + indexVector[11]*std::pow(alphy,3)*beta + indexVector[12]*std::pow(alphy,2)*std::pow(beta,2)
    + indexVector[13]*alphy*std::pow(beta,3) + indexVector[14]*std::pow(beta,4); 

    result = -rad2degree(result);
    
    return result;
}


/** 
 * @brief ankle Inverse Kinematic part2 success(1 try)
 * @param alphy ankle pitch
 * @param beta ankle roll
 * @retval ankle low position motor angle 
 */
double ankleInvKinLowTheta2_R(double alphy, double beta){
    // 角度转换 degree->rad
    alphy = degree2rad(alphy);
    beta = degree2rad(beta);

    static const double indexVector[15] = {-0.0001,
    1.0002,
    -0.8669,
    0.0033,
    -0.2580,
    -0.0483,
    0.0065,
    -0.0846,
    0.3987,
    0.1122,
    0.0036,
    -0.0951,
    0.1759,
    0.1427,
    -0.1355};
    
    double result = indexVector[0] + indexVector[1]*alphy + indexVector[2]*beta + indexVector[3]*std::pow(alphy,2) + indexVector[4]*alphy*beta
    + indexVector[5]*std::pow(beta,2) + indexVector[6]*std::pow(alphy,3) + indexVector[7]*std::pow(alphy,2)*beta + indexVector[8]*alphy*std::pow(beta,2)
    + indexVector[9]*std::pow(beta,3) + indexVector[10]*std::pow(alphy,4) + indexVector[11]*std::pow(alphy,3)*beta + indexVector[12]*std::pow(alphy,2)*std::pow(beta,2)
    + indexVector[13]*alphy*std::pow(beta,3) + indexVector[14]*std::pow(beta,4); 

    result = rad2degree(result);
    
    return result;
}


// 
/**
 * @brief 轨迹控制及误差 + 多电机控制 + 踝关节逆运动学 封装为一个函数  左腿
 * @param anklePitch 踝关节pitch最大角度
 * @param ankleRoll 踝关节roll最大角度
 * @param ankleHighPid 踝关节高位电机PID参数
 * @param ankleLowPid 踝关节低位电机PID参数
 */
void anklePosCtrlState(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[4]);
    dm.addMotor(&motors[5]);
    dm.enable(motors[4]);
    dm.enable(motors[5]);
    dm.Motorevo_set_zero_position(motors[4]);
    dm.Motorevo_set_zero_position(motors[5]);
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    std::string filename_ankle_pitch = std::string("test714_ankle_high") + ".txt";  
    std::string filename_ankle_roll = std::string("test714_ankle_low") + ".txt";

    // 轨迹产生
    // 关节角度到电机角度的映射
    double ankleMotorH = ankleInvKinHighTheta1(anklePitch, ankleRoll);
    double ankleMotorL = ankleInvKinLowTheta2(anklePitch, ankleRoll);
    
    // 高位电机的轨迹
    std::vector<double> trajectoryH,velocityH;
    trajectoryGenerator(trajectoryH, velocityH, ankleMotorH);
    // 低位电机的轨迹
    std::vector<double> trajectoryL,velocityL;
    trajectoryGenerator(trajectoryL, velocityL, ankleMotorL);
    // 轨迹转换为指令 
    // 1.乘以系数，最大期望角度1->30度  2.速度不变
    // 1.轨迹处理
    std::cout << "ankleMotorH is:" << ankleMotorH << std::endl;
    std::cout << "ankleMotorL is:" << ankleMotorL << std::endl;
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_high, mc_ankle_low;
    mc_ankle_high.pos_desired = trajectoryH;
    mc_ankle_high.velocity_limit = ankleHighPid.velLimit;
    mc_ankle_high.posKp = ankleHighPid.posKp;
    mc_ankle_high.posKd = ankleHighPid.posKd;
    mc_ankle_high.velKp = ankleHighPid.velKp;
    mc_ankle_high.velKd = ankleHighPid.velKd;
    mc_ankle_high.velKi = ankleHighPid.velKi;

    mc_ankle_low.pos_desired = trajectoryL;
    mc_ankle_low.velocity_limit = ankleLowPid.velLimit;
    mc_ankle_low.posKp = ankleLowPid.posKp;
    mc_ankle_low.posKd = ankleLowPid.posKd;
    mc_ankle_low.velKp = ankleLowPid.velKp;
    mc_ankle_low.velKd = ankleLowPid.velKd;
    mc_ankle_low.velKi = ankleLowPid.velKi;

    // std::string desired_traj_L = "desired_traj_L";
    // std::string desired_traj_H = "desired_traj_H";


    for (size_t i = 0; i <= trajectoryH.size(); i++)
    {
        // ankle high
        dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        // dataWrite_traj(5, mc_ankle_high.pos_desired[i], 0, 0, &desired_traj_H);
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite(4, motors[4].Get_Position(), motors[4].Get_Velocity(), motors[4].Get_tau(), &filename_ankle_pitch); // 保存数据到文件中

        // ankle low
        dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite1(5, motors[5].Get_Position(), motors[5].Get_Velocity(), motors[5].Get_tau(), &filename_ankle_roll); // 保存数据到文件中
        usleep(1000);
    }
    dm.disable(motors[4]);
    dm.disable(motors[5]);
    file.close();
    file1.close();
}

void anklePosCtrlState_R(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[10]);
    dm.addMotor(&motors[11]);
    dm.enable(motors[10]);
    dm.enable(motors[11]);
    dm.Motorevo_set_zero_position(motors[10]);
    dm.Motorevo_set_zero_position(motors[11]);
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    std::string filename_ankle_pitch = std::string("test719_ankle_high_R") + ".txt";  
    std::string filename_ankle_roll = std::string("test719_ankle_low_R") + ".txt";

    // 轨迹产生
    // 关节角度到电机角度的映射
    double ankleMotorH = ankleInvKinHighTheta1_R(anklePitch, ankleRoll);
    double ankleMotorL = ankleInvKinLowTheta2_R(anklePitch, ankleRoll);
    
    // 高位电机的轨迹
    std::vector<double> trajectoryH,velocityH;
    trajectoryGenerator(trajectoryH, velocityH, ankleMotorH);
    // 低位电机的轨迹
    std::vector<double> trajectoryL,velocityL;
    trajectoryGenerator(trajectoryL, velocityL, ankleMotorL);
    // 轨迹转换为指令 
    // 1.乘以系数，最大期望角度1->30度  2.速度不变
    // 1.轨迹处理
    std::cout << "ankleMotorH is:" << ankleMotorH << std::endl;
    std::cout << "ankleMotorL is:" << ankleMotorL << std::endl;
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    };
    
    motorcmd mc_ankle_high, mc_ankle_low;
    mc_ankle_high.pos_desired = trajectoryH;
    mc_ankle_high.velocity_limit = ankleHighPid.velLimit;
    mc_ankle_high.posKp = ankleHighPid.posKp;
    mc_ankle_high.posKd = ankleHighPid.posKd;
    mc_ankle_high.velKp = ankleHighPid.velKp;
    mc_ankle_high.velKd = ankleHighPid.velKd;
    mc_ankle_high.velKi = ankleHighPid.velKi;

    mc_ankle_low.pos_desired = trajectoryL;
    mc_ankle_low.velocity_limit = ankleLowPid.velLimit;
    mc_ankle_low.posKp = ankleLowPid.posKp;
    mc_ankle_low.posKd = ankleLowPid.posKd;
    mc_ankle_low.velKp = ankleLowPid.velKp;
    mc_ankle_low.velKd = ankleLowPid.velKd;
    mc_ankle_low.velKi = ankleLowPid.velKi;

    // std::string desired_traj_L = "desired_traj_L";
    // std::string desired_traj_H = "desired_traj_H";


    for (size_t i = 0; i <= trajectoryH.size(); i++)
    {
        // ankle high
        dm.Motorevo_motor_servo_mode(motors[10], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        // dataWrite_traj(5, mc_ankle_high.pos_desired[i], 0, 0, &desired_traj_H);
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite(10, motors[10].Get_Position(), motors[10].Get_Velocity(), motors[10].Get_tau(), &filename_ankle_pitch); // 保存数据到文件中

        // ankle low
        dm.Motorevo_motor_servo_mode(motors[11], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        
        dm.receive_evo();
        // std::cout << "motor_state.id: "<< 4 << std::endl;
        
        // std::cout << "motor_state.baseData.position: "<< motors[3].Get_Position() << std::endl;
        // std::cout << "motor_state.baseData.velocity: "<< motors[3].Get_Velocity() << std::endl;
        // std::cout << "motor_state.baseData.torque: "<< motors[3].Get_tau() << std::endl;
        dataWrite1(11, motors[11].Get_Position(), motors[11].Get_Velocity(), motors[11].Get_tau(), &filename_ankle_roll); // 保存数据到文件中
        usleep(1000);
    }
    dm.disable(motors[10]);
    dm.disable(motors[11]);
    file.close();
    file1.close();
}

double degree2rad(double& degree){
    return degree * pi / 180.0;
}
double rad2degree(double& rad){
    return rad * 180.0 / pi;
}

// 
// void evomotorCtrl_servo(int motorID, ){

// }

// 
/**
 * @brief 轨迹控制及误差 + 多电机控制 + 踝关节逆运动学 封装为一个函数  右腿测试版，未修改运动学，只测试电机轨迹
 * @param anklePitch 踝关节pitch最大角度
 * @param ankleRoll 踝关节roll最大角度
 * @param ankleHighPid 踝关节高位电机PID参数
 * @param ankleLowPid 踝关节低位电机PID参数
 */
void anklePosCtrlStateDoubleCtrl(double anklePitch, double ankleRoll, servoPID& ankleHighPid, servoPID& ankleLowPid){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    dm.addMotor(&motors[4]);
    dm.addMotor(&motors[5]);
    dm.enable(motors[4]);
    dm.enable(motors[5]);
    dm.Motorevo_set_zero_position(motors[4]);
    dm.Motorevo_set_zero_position(motors[5]);
    
    dm.addMotor(&motors[10]);
    dm.addMotor(&motors[11]);
    dm.enable(motors[10]);
    dm.enable(motors[11]);
    dm.Motorevo_set_zero_position(motors[10]);
    dm.Motorevo_set_zero_position(motors[11]);
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    std::string filename_ankle_pitch_L = std::string("test719_ankle_high_L_C") + ".txt";  
    std::string filename_ankle_roll_L = std::string("test719_ankle_low_L_C") + ".txt";
    std::string filename_ankle_pitch_R = std::string("test719_ankle_high_R_C") + ".txt";  
    std::string filename_ankle_roll_R = std::string("test719_ankle_low_R_C") + ".txt";

    // 轨迹产生
    // 关节角度到电机角度的映射
    double ankleMotorH = ankleInvKinHighTheta1(anklePitch, ankleRoll);
    double ankleMotorL = ankleInvKinLowTheta2(anklePitch, ankleRoll);
    double ankleMotorH_R = -ankleInvKinHighTheta1_R(anklePitch, ankleRoll);
    double ankleMotorL_R = -ankleInvKinLowTheta2_R(anklePitch, ankleRoll);
    
    // 高位电机的轨迹
    std::vector<double> trajectoryH,velocityH,trajectoryH_R,velocityH_R;
    trajectoryGenerator(trajectoryH, velocityH, ankleMotorH);
    trajectoryGenerator(trajectoryH_R, velocityH_R, ankleMotorH_R);
    // 低位电机的轨迹
    std::vector<double> trajectoryL,velocityL,trajectoryL_R,velocityL_R;
    trajectoryGenerator(trajectoryL, velocityL, ankleMotorL);
    trajectoryGenerator(trajectoryL_R, velocityL_R, ankleMotorL_R);

    // 轨迹转换为指令 
    // 1.乘以系数，最大期望角度1->30度  2.速度不变
    // 1.轨迹处理
    std::cout << "ankleMotorH is:" << ankleMotorH << std::endl;
    std::cout << "ankleMotorL is:" << ankleMotorL << std::endl;
    std::cout << "ankleMotorH_R is:" << ankleMotorH_R << std::endl;
    std::cout << "ankleMotorL_R is:" << ankleMotorL_R << std::endl;
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_ankle_high, mc_ankle_low, mc_ankle_high_R, mc_ankle_low_R;
    
    mc_ankle_high.pos_desired = trajectoryH;
    mc_ankle_high.velocity_limit = ankleHighPid.velLimit;
    mc_ankle_high.posKp = ankleHighPid.posKp;
    mc_ankle_high.posKd = ankleHighPid.posKd;
    mc_ankle_high.velKp = ankleHighPid.velKp;
    mc_ankle_high.velKd = ankleHighPid.velKd;
    mc_ankle_high.velKi = ankleHighPid.velKi;

    mc_ankle_low.pos_desired = trajectoryL;
    mc_ankle_low.velocity_limit = ankleLowPid.velLimit;
    mc_ankle_low.posKp = ankleLowPid.posKp;
    mc_ankle_low.posKd = ankleLowPid.posKd;
    mc_ankle_low.velKp = ankleLowPid.velKp;
    mc_ankle_low.velKd = ankleLowPid.velKd;
    mc_ankle_low.velKi = ankleLowPid.velKi;

    mc_ankle_high_R.pos_desired = trajectoryH_R;
    mc_ankle_high_R.velocity_limit = ankleHighPid.velLimit;
    mc_ankle_high_R.posKp = ankleHighPid.posKp;
    mc_ankle_high_R.posKd = ankleHighPid.posKd;
    mc_ankle_high_R.velKp = ankleHighPid.velKp;
    mc_ankle_high_R.velKd = ankleHighPid.velKd;
    mc_ankle_high_R.velKi = ankleHighPid.velKi;

    mc_ankle_low_R.pos_desired = trajectoryL_R;
    mc_ankle_low_R.velocity_limit = ankleLowPid.velLimit;
    mc_ankle_low_R.posKp = ankleLowPid.posKp;
    mc_ankle_low_R.posKd = ankleLowPid.posKd;
    mc_ankle_low_R.velKp = ankleLowPid.velKp;
    mc_ankle_low_R.velKd = ankleLowPid.velKd;
    mc_ankle_low_R.velKi = ankleLowPid.velKi;

    // std::string desired_traj_L = "desired_traj_L";
    // std::string desired_traj_H = "desired_traj_H";


    for (size_t i = 0; i <= trajectoryH.size(); i++)
    {
        // ankle high leftLeg
        dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        dm.receive_evo();
        dataWrite(4, motors[4].Get_Position(), motors[4].Get_Velocity(), motors[4].Get_tau(), &filename_ankle_pitch_L); // 保存数据到文件中

        // ankle low leftLeg
        dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        dm.receive_evo();
        dataWrite1(5, motors[5].Get_Position(), motors[5].Get_Velocity(), motors[5].Get_tau(), &filename_ankle_roll_L); // 保存数据到文件中
        usleep(1000);
        // ankle high rightLeg
        dm.Motorevo_motor_servo_mode(motors[10], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        dm.receive_evo();
        dataWrite2(10, motors[10].Get_Position(), motors[10].Get_Velocity(), motors[10].Get_tau(), &filename_ankle_roll_R); // 保存数据到文件中

        // ankle low rightLeg
        dm.Motorevo_motor_servo_mode(motors[11], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        dm.receive_evo();
        dataWrite3(11, motors[11].Get_Position(), motors[11].Get_Velocity(), motors[11].Get_tau(), &filename_ankle_roll_R); // 保存数据到文件中

        usleep(1000);
    }
    dm.disable(motors[4]);
    dm.disable(motors[5]);
    dm.disable(motors[10]);
    dm.disable(motors[11]);
    file.close();
    file1.close();
    file2.close();
    file3.close();

}

/**
 * @brief ELMO电机控制测试
 */
int elmo_pos_ctrl_SDO(){
    // 1.初始话串口对象
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;

    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    int motorID = 9;  //右腿膝关节
    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();
    std::cout << "motorCANID is: " << motorCANID << std::endl;
    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    usleep(1000000);
    
    // 4.读取ELMO状态字
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);
    
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"receive failed" << std::endl;
    }
    else
    {
    
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
            }
            
            
        }
        
        
    }
    return 0;
}

/**
 * @brief 重载版本--真正的位置控制
 * @param posDes 期望的位置, 测试无问题
 */
int elmo_pos_ctrl_SDO(double posDes){
    // 1.初始话串口对象
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;

    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    int motorID = 2;
    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();

    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    
    // 4.读取ELMO状态字
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);
    
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
    }
    else
    {
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43，且状态字第0位为1（即准备就绪）
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
            
            // 5. 配置位置模式
            // 使能
            mc.elmo_enable(motorCANID);

            // 设置位置模式
            mc.elmo_set_pos_mode(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);
            // 设置期望位置
            mc.elmo_set_pos(motorCANID, posDes);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置速度限制
            mc.elmo_set_vel(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置加速度限制
            mc.elmo_set_acc(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置减速度限制
            mc.elmo_set_dec(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置相对位移模式
            mc.elmo_config_rel_or_abs(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 6. 执行
            mc.elmo_set_start(motorCANID);

            
            
            // 7. 检测是否达到对应位置
            do
            {
                mc.canopen_sdo_read(motorCANID, 0x6041, 0);
                mc.receive_canopen(return_canObj, receive_flag);
                usleep(500);
                
            } while (!((return_canObj.canData[5]&0x04)==0x04));
            
            
            std::cout << "已经抵达期望位置" << std::endl;
            
            sleep(1);
            
            // 8. 达到则禁用使能
            mc.elmo_disable(motorCANID);


            
        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
                std::cout << "CANFrame's canID is: ";
                printHex(return_canObj.canId);
                std::cout << " CANFrame's canData[0] is: " ;
                printHex(return_canObj.canData[0]);
                std::cout << std::endl;


            }
            
            
        }
        
        
    }
    
    return 0;
}

/**
 * @brief 重载版本--真正的位置控制+通信反馈--测试无问题
 * @param posDes 期望的位置
 * @param state 
 */
int elmo_pos_ctrl_SDO(double posDes, elmoState& state){
    // 1.初始话串口对象 以及一些变量
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;
    elmoState lastState, curState;

    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    int motorID = 9;
    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();

    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    
    // 4.读取ELMO状态字
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);
    
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
    }
    else
    {
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43，且状态字第0位为1（即准备就绪）
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
            
            

            // 5. 配置位置模式
            // 使能
            mc.elmo_enable(motorCANID);
            // 读取一次电机当前的状态
            elmo_read_state(curState, lastState, mc, motorCANID, return_canObj, receive_flag, true);


            // 设置位置模式
            mc.elmo_set_pos_mode(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);
            // 设置期望位置
            mc.elmo_set_pos(motorCANID, posDes);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置速度限制
            mc.elmo_set_vel(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置加速度限制
            mc.elmo_set_acc(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置减速度限制
            mc.elmo_set_dec(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置相对位移模式
            mc.elmo_config_rel_or_abs(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 6. 执行
            mc.elmo_set_start(motorCANID);

            int timeCount = 0;
            motor_head::CAN_Receive_Frame return_canObj_last = return_canObj;
            // 7. 检测是否达到对应位置
            do
            {
                mc.canopen_sdo_read(motorCANID, 0x6041, 0);
                mc.receive_canopen(return_canObj, receive_flag);
                usleep(500);
                
            } while (!((return_canObj.canData[5]&0x04)==0x04));
            
            if (timeCount<=1000)
            {
                std::cout << "已经抵达期望位置" << std::endl;
            }
            else
            {
                std::cout << "未抵达期望位置" << std::endl;
            }

            // 8. 读取电机当点状态
            lastState = curState;
            // sleep(3);
            elmo_read_state(curState, lastState, mc, motorCANID, return_canObj, receive_flag, false);
            sleep(1);
            
            // 9. 达到则禁用使能
            mc.elmo_disable(motorCANID);

        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
                std::cout << "CANFrame's canID is: ";
                printHex(return_canObj.canId);
                std::cout << " CANFrame's canData[0] is: " ;
                printHex(return_canObj.canData[0]);
                std::cout << std::endl;


            }
            
            
        }
        
        
    }
    
    return 0;
}

/**
 * @brief 重载版本--位置轨迹控制+通信反馈--待测试8.1   8.1遇到问题,轨迹控制不动---找到问题,没有给相对位置指令所以没动.且开始时相对位置很小几乎不动, 其次逻辑不对---错误逻辑为想相对于零点发的相对位置命令,实际则为相对于上一次的相对位置命令.明天改
 * @param posDesMax 期望的位置
 * @param testNum 测试次数,用于命名文件
 */
int elmo_pos_traj_ctrl_SDO(double posDesMax, int testNum){
    // 1.初始话串口对象 以及一些变量
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;
    elmoState lastState, curState, zeroState;
    std::string filename_knee = std::string("test84_knee_") + std::to_string(testNum) + ".txt";
    std::vector<double> trajectory_knee, vel_traj_knee;


    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    int motorID = 9;
    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();

    // 建立五次多项式轨迹
    trajectoryGenerator(trajectory_knee, vel_traj_knee, posDesMax);

    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    
    // 4.读取ELMO状态字  判断是否建立起了通信
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);
    
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
    }
    else
    {
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43，且状态字第0位为1（即准备就绪）
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
            
            // 5. 配置位置模式
            // 使能
            mc.elmo_enable(motorCANID);
            // 读取一次电机当前的状态 作为零点
            elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, true);
            zeroState = curState;

            // 设置位置模式
            mc.elmo_set_pos_mode(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);
            

            // 设置速度限制
            mc.elmo_set_vel(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置加速度限制
            mc.elmo_set_acc(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置减速度限制
            mc.elmo_set_dec(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            
            

            for (size_t i = 0; i < trajectory_knee.size(); i = i+10)
            {
                // 设置期望位置
                float deltaPos = trajectory_knee[i] - lastState.pos;
                mc.elmo_set_pos(motorCANID, deltaPos);
                // usleep(1000);
                // 设置相对位移模式  进行了位置模式的修改
                mc.elmo_config_rel_or_abs(motorCANID, 1);
                // usleep(1000);
                // 6. 执行
                mc.elmo_set_start(motorCANID);

                // 7. 检测是否达到对应位置
                do
                {
                    mc.canopen_sdo_read(motorCANID, 0x6041, 0);
                    mc.receive_canopen(return_canObj, receive_flag);
                    usleep(500);
                    
                } while (!((return_canObj.canData[5]&0x04)==0x04));
                
                // 8. 读取电机当点状态
                lastState = curState;
                elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false);
                dataWrite(motorCANID, curState.pos, curState.vel, curState.tor, &filename_knee);
            }
            
            sleep(3);
            // 9. 达到则禁用使能
            mc.elmo_disable(motorCANID);
            file.close();

        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
                std::cout << "CANFrame's canID is: ";
                printHex(return_canObj.canId);
                std::cout << " CANFrame's canData[0] is: " ;
                printHex(return_canObj.canData[0]);
                std::cout << std::endl;


            }
            
            
        }
        
        
    }
    
    return 0;
}


/**
 * @brief 重载版本--位置轨迹控制+通信反馈--待测试8.1   8.1遇到问题,轨迹控制不动---找到问题,没有给相对位置指令所以没动.且开始时相对位置很小几乎不动, 其次逻辑不对---错误逻辑为想相对于零点发的相对位置命令,实际则为相对于上一次的相对位置命令.明天改
 * @param posDesMax 期望的位置
 * @param testNum 测试次数,用于命名文件
 */
int elmo_pos_traj_ctrl_SDO_1(double posDesMax, int testNum){
    // 1.初始话串口对象 以及一些变量
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;
    elmoState curState, zeroState;
    std::string filename_knee = std::string("test85_knee_") + std::to_string(testNum) + ".txt";
    std::vector<double> trajectory_knee, vel_traj_knee;


    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    int motorID = 9;
    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();

    // 建立五次多项式轨迹
    trajectoryGenerator(trajectory_knee, vel_traj_knee, posDesMax);

    

    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    
    // 4.读取ELMO状态字  判断是否建立起了通信
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);
    
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
    }
    else
    {
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43，且状态字第0位为1（即准备就绪）
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
            
            // 5. 配置位置模式
            // 使能
            mc.elmo_enable(motorCANID);
            // 读取一次电机当前的状态 作为零点
            elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, true);
            zeroState = curState;

            // 设置当前脉冲数为0点，将位置轨迹转换为脉冲轨迹
            int zeroPulse = zeroState.pulseNum;
            uint32_t temp = 0;
            std::vector<uint32_t> traj_knee_pulse;
            std::cout << "trajectory_knee's value: ";
            for (auto i : trajectory_knee)
            {
                mc.elmo_angle2pulse_mapping(i,temp);
                temp = temp + zeroPulse; // 偏置
                traj_knee_pulse.push_back(temp);
                std::cout << temp << "  ";
            }
            std::cout << std::endl;
            std::cout << "vel_traj_knee's value: ";

            for (auto i: vel_traj_knee)
            {
                std::cout << i << "  ";
                
            }
            
            std::cout << std::endl;

            

            // 设置位置模式
            mc.elmo_set_pos_mode(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);
            

            // 设置速度限制
            mc.elmo_set_vel(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置加速度限制
            mc.elmo_set_acc(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置减速度限制
            mc.elmo_set_dec(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            
            // if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            // usleep(100000);
            
            // 等待输入继续执行
            std::cout << "Press Enter to continue ..."<<std::endl;
            std::cin.get();
            

            for (size_t i = 0; i < traj_knee_pulse.size(); i= i+10)
            {
                // 设置期望位置
                
                mc.elmo_set_pos_pulse(motorCANID, traj_knee_pulse[i]);
                // usleep(1000);
                // 设置相对位移模式  进行了位置模式的修改
                mc.elmo_config_rel_or_abs(motorCANID, 1, 0);

                // // 设置end速度,也就是在pp模式中,到达目标点的速度 --待测试--有问题晚上继续
                // mc.elmo_set_end_vel(motorCANID, vel_traj_knee[i]);

                usleep(500);
                // 6. 执行
                mc.elmo_set_start(motorCANID);
                // usleep(1000);
                // 8. 读取电机当点状态
                
                elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false);
                dataWrite(motorCANID, curState.pos, curState.vel, curState.tor, &filename_knee);
            }
            

            // 7. 检测是否达到对应位置
            do
            {
                mc.canopen_sdo_read(motorCANID, 0x6041, 0);
                mc.receive_canopen(return_canObj, receive_flag);
                usleep(500);
                
            } while (!((return_canObj.canData[5]&0x04)==0x04));


            sleep(3);
            // 9. 达到则禁用使能
            mc.elmo_disable(motorCANID);
            file.close();

        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
                std::cout << "CANFrame's canID is: ";
                printHex(return_canObj.canId);
                std::cout << " CANFrame's canData[0] is: " ;
                printHex(return_canObj.canData[0]);
                std::cout << std::endl;


            }
            
            
        }
        
        
    }
    
    return 0;
}

/**
 * @brief 重载版本--位置轨迹控制+通信反馈--跑ZMP膝关节轨迹 -- 待测试  待测试 8.18
 * @param posDesMax 期望的位置
 * @param testNum 测试次数,用于命名文件
 */
int elmo_pos_traj_ctrl_SDO_2(double posDesMax, int testNum){
    // 1.初始话串口对象 以及一些变量
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;
    elmoState curState, zeroState;
    std::string filename_knee = std::string("test95_knee_L_final") + std::to_string(testNum) + ".txt";
    std::vector<double> trajectory_knee, vel_traj_knee, acc_traj_knee;
    
    // ZMP
    // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    // std::cout<< "traj_knee's all data: " << std::endl;

    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_knee.push_back(rad2degree(trajectory[i].joint_angles[3]));
    //     std::cout << trajectory_knee[i] << " ";
    // }

    // std::cout << "trajectory_knee.size() is: " << trajectory_knee.size();

    
    // std::cout << std::endl;


    // read csv data
    // std::vector<std::vector<double>> csvdata(9501, std::vector<double>(12));
    // CSV_string2double(&csvdata);
    
    //   二选一
    
    // std::cout << "i th row, 9th colum's value is: ";
    // for (size_t i = 0; i < csvdata.size(); i++)
    // {
    //     for (size_t j = 0; j < (csvdata)[i].size(); j++)
    //     {
    //         if (j==9)
    //         {
    //             // std::cout << rad2degree((csvdata)[i][j]) << "  ";
    //             trajectory_knee.push_back(rad2degree((csvdata)[i][j]));
    //         }
            
    //     }
        
    // }
    
    

    
    
    // 五次多项式轨迹
    trajectoryGenerator(trajectory_knee, vel_traj_knee, posDesMax);

    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    // int motorID = 9; //right knee
    int motorID = 3; //left knee

    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();

    

    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    usleep(200);
    // 4.读取ELMO状态字  判断是否建立起了通信
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);

    mc.receive_canopen_sdo(return_canObj, receive_flag);

    
    
    

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
    }
    else
    {
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43，且状态字第0位为1（即准备就绪）
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
            
            // 5. 配置位置模式
            usleep(1000);
            // 使能
            mc.elmo_enable(motorCANID);
            // 读取一次电机当前的状态 作为零点
            // elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, true);
            // zeroState = curState;
            
            // ensure first read state success
            bool initRes = false;
            do
            {
                initRes = elmo_read_init_state(zeroState, mc, motorCANID, return_canObj, receive_flag);
            } while (!initRes);
            
            
            elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false);
            usleep(1000);
             
            // debug code   桥过载一定有原因，明天继续
            // int count1 = 0;
            // while (1)
            // {
            //     count1++;
            //     std::cout << count1 << "th read state" << std::endl;
            //     elmo_read_stateV2(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false);
            //     usleep(1500);
            // }

            // debug beta 2
            // int count1 = 0;
            // reSendNumCanopen reSendNum;
            // while (1)
            // {
            //     count1++;
            //     std::cout << count1 << "th read state" << std::endl;
            //     elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, reSendNum);
            //     usleep(1500);
            // }


            // debug beta 3
            
            reSendNumCanopen reSendNum;
            // int exceedLimitNum[3]{0,0,0};
            // int resReadState = 0;
            // int noReSendNum = 0;
            // int abnormalNum = 0; //resend >= 4;
            // for (size_t i = 1; i <= 1000; i++)
            // {
            //     std::cout << i << "th read state" << std::endl;
            //     resReadState = elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, reSendNum);
            //     switch (resReadState)
            //     {
            //     case -1:
            //         exceedLimitNum[0]++;
            //         break;
            //     case -2:
            //         exceedLimitNum[1]++;
            //         break;
            //     case -3:
            //         exceedLimitNum[2]++;
            //         break;
            //     default:
            //         break;
            //     }
            //     if (reSendNum.posSendNum==0 && reSendNum.velSendNum==0 && reSendNum.torSendNum==0)
            //     {
            //         noReSendNum++;
            //     }
            //     else if (reSendNum.posSendNum>=4 || reSendNum.velSendNum>=4 || reSendNum.torSendNum>=4)
            //     {
            //         abnormalNum++;
            //     }
                
                
            //     usleep(1500);
            // }

            // std::cout << "In the loop:"<<"\n";
            // std::cout << "total communicate num is: " << 1000 << "\n"
            //             << "due to resend SDO exceeding limit , the num of read state failed is : " 
            //             << exceedLimitNum[0] + exceedLimitNum[1] + exceedLimitNum[2] 
            //             << "the rate of exceeding num / total num is: " << (exceedLimitNum[0] + exceedLimitNum[1] + exceedLimitNum[2]) / 1000.0 << "\n";

            // std::cout << "in this case, due to read pos exceed limit , the num is :" << exceedLimitNum[0] << "\n"
            //             << "in this case, due to read vel exceed limit , the num is :" << exceedLimitNum[1] << "\n"
            //             << "in this case, due to read tor exceed limit , the num is :" << exceedLimitNum[2] << std::endl;
            
            // std::cout << "the num of no reSend receive state is: " << noReSendNum << "\n"
            //             << "the rate of noResendNum / total is: " << noReSendNum/1000.0 << "\n"
            //             << "the num of abnormal receive state is: " << abnormalNum << "\n"
            //             << "the rate of abnormal / total is: " << abnormalNum/1000.0 << std::endl;
            


            // return 0;
            
            

            // 设置当前脉冲数为0点，将位置轨迹转换为脉冲轨迹
            int zeroPulse = zeroState.pulseNum;
            uint32_t temp = 0;
            double vel_temp = 0;
            double acc_temp = 0;
            std::vector<uint32_t> traj_knee_pulse, vel_traj_knee_pulse;
            // print info of traj of pulseNum and pos_traj->pulse_traj
            // std::cout << "trajectory_knee's value: ";
            // for (auto i : trajectory_knee)
            // {
            //     mc.elmo_angle2pulse_mapping(i,temp);
            //     temp = temp + zeroPulse; // 偏置
            //     traj_knee_pulse.push_back(temp);
            //     std::cout << temp << "  ";
            // }

            
            // pos_traj->pulse_traj If vel_traj be had
            for (size_t i = 0; i < trajectory_knee.size(); i++)
            {
                mc.elmo_angle2pulse_mapping(trajectory_knee[i],temp);
                temp = temp + zeroPulse; // 偏置
                traj_knee_pulse.push_back(temp);
                       
            }

            // pos_traj->pulse_traj  
            // if only pos_traj data, this will solve vel_traj
            // for (size_t i = 0; i < trajectory_knee.size(); i++)
            // {
            //     mc.elmo_angle2pulse_mapping(trajectory_knee[i],temp);
            //     temp = temp + zeroPulse; // 偏置
            //     traj_knee_pulse.push_back(temp);
            //     // std::cout << " temp is: " << temp;
            //     if ((i+1)<trajectory_knee.size())
            //     {   
            //         vel_temp = (trajectory_knee[i+1] - trajectory_knee[i])/0.001;  // 这里的时间间隔要和后面命令发送周期一致
            //         if (std::abs(vel_temp)>=60)
            //         {
            //             // limit of max velocity
            //             if (vel_temp<0)
            //             {
            //                 vel_traj_knee.push_back(-60);
            //             }
            //             else
            //             {
            //                 vel_traj_knee.push_back(60);
            //             }

            //         }
            //         else
            //         {
            //             vel_traj_knee.push_back(vel_temp);
            //         }

            //         // std::cout << " vel_temp is: " << vel_temp;
            //         // std::cout << " vel_traj_knee[i] is: " << vel_traj_knee[i];
    
            //     }        
            // }
            // difference commercial -> acc
            for (size_t i = 0; i < vel_traj_knee.size()-1; i++)
            {
                
                acc_temp = (vel_traj_knee[i+1] - vel_traj_knee[i])/0.001;
                acc_traj_knee.push_back(acc_temp);
                
            }
            
            // print max vel & acc
            std::cout <<" acc_traj_knee's size is: "<<acc_traj_knee.size()<< std::endl;
            auto maxValueAcc = std::max_element(acc_traj_knee.begin(), acc_traj_knee.end());
            std::cout <<" acc_traj_knee's maxValue is: "<<*maxValueAcc<< std::endl;
            auto minValueAcc = std::min_element(acc_traj_knee.begin(), acc_traj_knee.end());
            std::cout <<" acc_traj_knee's minValue is: "<<*minValueAcc<< std::endl;
            
            std::cout <<" vel_traj_knee's size is: "<<vel_traj_knee.size()<< std::endl;
            auto maxValue = std::max_element(vel_traj_knee.begin(), vel_traj_knee.end());
            std::cout <<" vel_traj_knee's maxValue is: "<<*maxValue<< std::endl;
            auto minValue = std::min_element(vel_traj_knee.begin(), vel_traj_knee.end());
            std::cout <<" vel_traj_knee's minValue is: "<<*minValue<< std::endl;
            size_t max_index = std::distance(vel_traj_knee.begin(), maxValue);
            std::cout <<" vel_traj_knee's maxValue's index is: "<<max_index<< std::endl;

            // vel_traj->vel_pulse_traj
            for (size_t i = 0; i < vel_traj_knee.size(); i++)
            {
                mc.elmo_angle2pulse_mapping(abs(vel_traj_knee[i]), temp);
                vel_traj_knee_pulse.push_back(temp);
            }
            
            
            std::cout << std::endl;

            // // 等待输入继续执行
            // std::cout << "Press Enter to continue ..."<<std::endl;
            // std::cin.get();

            // 设置位置模式
            mc.elmo_set_pos_mode(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);
            

            // 设置速度限制
            mc.elmo_set_vel(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置加速度限制
            mc.elmo_set_acc(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置减速度限制
            mc.elmo_set_dec(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // int count2 = 1;

            // while (1)
            // {
            //     std::cout << count2 << "th set mode" << std::endl;
            //     mc.elmo_set_pos_mode(motorCANID);
            //     if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            //     usleep(100000);
            //     count2++;
            // }
            


            
            // 等待输入继续执行
            // std::cout << "Press Enter to continue ..."<<std::endl;
            // std::cin.get();

            // 打开管道，阻塞等待信号
            // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
            // if (pipe_fd == -1) {
            //     std::cerr << "管道打开失败！\n";
            //     return 1;
            // }

            // char buffer[10];
            // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
            // close(pipe_fd);

            // 收到信号后，继续执行后续逻辑
            // std::cout << "程序 " << "right leg ctrl" << " 开始执行！\n";

            int timeBetFrame = 200;
            auto total_comm = 0;  // 累计通信耗时（us）
            auto total_read_state = 0; // 累计休眠耗时（us）
            auto total_write = 0; // 累计写入耗时（us）
            
            end = std::chrono::high_resolution_clock::now();;
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            auto duration_read_state = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            auto cmd_sending_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::cout << "initial time is: " << duration;
            // // 等待输入继续执行
            // std::cout << "Press Enter to continue ..."<<std::endl;
            // std::cin.get();
            auto comStart = std::chrono::high_resolution_clock::now();
            auto comEnd = std::chrono::high_resolution_clock::now();
            auto cmd_send_t = std::chrono::high_resolution_clock::now();
            
            start = std::chrono::high_resolution_clock::now();
            for (size_t i = 0; i < traj_knee_pulse.size(); i= i+2)
            {
                // 设置期望位置
                // std::cout << i << "th cycle of command" << std::endl;
                comStart = std::chrono::high_resolution_clock::now();
                mc.elmo_set_pos_pulse(motorCANID, traj_knee_pulse[i]);
                comEnd = std::chrono::high_resolution_clock::now();
                // usleep(timeBetFrame);
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us

                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();
                // 设置速度
                comStart = std::chrono::high_resolution_clock::now();
                if (i<traj_knee_pulse.size()-1)
                {

                    mc.elmo_set_vel_traj(motorCANID, vel_traj_knee_pulse[i]);
                }
                else
                {
                    mc.elmo_set_vel_traj(motorCANID, 0);
                }
                comEnd = std::chrono::high_resolution_clock::now();
                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us

                // usleep(timeBetFrame);
                // 设置相对位移模式  进行了位置模式的修改
                comStart = std::chrono::high_resolution_clock::now();

                mc.elmo_config_rel_or_abs(motorCANID, 1, 0);
                comEnd = std::chrono::high_resolution_clock::now();
                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();


                // usleep(timeBetFrame);
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us


                // 6. 执行
                comStart = std::chrono::high_resolution_clock::now();

                mc.elmo_set_start(motorCANID);
                comEnd = std::chrono::high_resolution_clock::now();
                cmd_sending_time = std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - start).count();
                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();

                // usleep(timeBetFrame);
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us

                // 8. 读取电机当点状态
                
                // elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, re);
                // resReadState = elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, reSendNum);
                comStart = std::chrono::high_resolution_clock::now();

                elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, reSendNum);
                comEnd = std::chrono::high_resolution_clock::now();

                total_read_state += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();

                duration_read_state = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                // dataWriteTime(motorCANID, curState.pos, curState.vel, curState.tor, &filename_knee, duration);
                // 待验证， 记录了状态时间和指令发送时间（有通信时间的偏差
                dataWriteTimewithSendingTime(motorCANID, curState.pos, curState.vel, curState.tor, &filename_knee, duration_read_state, cmd_sending_time); 

                end = std::chrono::high_resolution_clock::now();
                total_write += std::chrono::duration_cast<std::chrono::milliseconds>(end - comEnd).count();

                
                // usleep(800);
                std::this_thread::sleep_for(std::chrono::microseconds(800)); //更精确的延时us

            }
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout  << "all time of ctrl is: " << duration << "\n"
            << "total_comm is: " << total_comm <<"\n"
            << "total_read_state is: " << total_read_state<<"\n"
            << "total_write is: " << total_write << std::endl;

            // 7. 检测是否达到对应位置
            do
            {
                mc.canopen_sdo_read(motorCANID, 0x6041, 0);
                // mc.receive_canopen(return_canObj, receive_flag);
                mc.receive_canopen_sdo(return_canObj, receive_flag);

                usleep(500);
                end = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            } while (!(  ((return_canObj.canData[5]&0x04)==0x04) || duration>=1000  ));

            if (duration>=1000)
            {
                std::cout << "do not reach target pos" << std::endl;
            }
            
            sleep(3);
            // 9. 达到则禁用使能
            mc.elmo_disable(motorCANID);
            file.close();

        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
                std::cout << "CANFrame's canID is: ";
                printHex(return_canObj.canId);
                std::cout << " CANFrame's canData[0] is: " ;
                printHex(return_canObj.canData[0]);
                std::cout << std::endl;


            }
            
            
        }
        
        
    }
    
    return 0;
}

/**
 * @brief 重载版本--位置轨迹控制+通信反馈--  两个膝关节联动控制
 * @param posDesMax 期望的位置
 * @param testNum 测试次数,用于命名文件
 */
int elmo_pos_traj_ctrl_SDO_3(double posDesMax1, double posDesMax2, int testNum1, int testNum2){
    // 1.初始话串口对象 以及一些变量
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;
    elmoState curState1, zeroState1;
    elmoState curState2, zeroState2;

    std::string filename_knee1 = std::string("test87_knee_double_L_") + std::to_string(testNum1) + ".txt";
    std::string filename_knee2 = std::string("test87_knee_double_R_") + std::to_string(testNum2) + ".txt";

    std::vector<double> trajectory_knee1, vel_traj_knee1, acc_traj_knee1;
    std::vector<double> trajectory_knee2, vel_traj_knee2, acc_traj_knee2;

    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // read csv data
    // std::vector<std::vector<double>> csvdata(9501, std::vector<double>(12));
    // CSV_string2double(&csvdata);
    
    //   二选一
    
    // std::cout << "i th row, 9th colum's value is: ";
    // for (size_t i = 0; i < csvdata.size(); i++)
    // {
    //     for (size_t j = 0; j < (csvdata)[i].size(); j++)
    //     {
    //         if (j==9)
    //         {
    //             // std::cout << rad2degree((csvdata)[i][j]) << "  ";
    //             trajectory_knee.push_back(rad2degree((csvdata)[i][j]));
    //         }
            
    //     }
        
    // }
    
    trajectoryGenerator(trajectory_knee1, vel_traj_knee1, posDesMax1);
    trajectoryGenerator(trajectory_knee2, vel_traj_knee2, posDesMax2);

    
    std::cout << std::endl;
    
    

    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    int motorID_knee_L = 3;
    int motorID_knee_R = 9;

    mc.addMotor(&motors[motorID_knee_L]);
    mc.addMotor(&motors[motorID_knee_R]);

    int motorCANID_knee_L = motors[motorID_knee_L].GetSlaveId();
    int motorCANID_knee_R = motors[motorID_knee_R].GetSlaveId();


    

    // 3.进入操作状态

    mc.canopen_operational(motorCANID_knee_L);
    mc.canopen_operational(motorCANID_knee_R);

    
    // 4.读取ELMO状态字  判断是否建立起了通信
    mc.canopen_sdo_read(motorCANID_knee_L, 0x6041, 0);
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
        return -1;
    }
    
    mc.canopen_sdo_read(motorCANID_knee_R, 0x6041, 0);
    mc.receive_canopen_sdo(return_canObj, receive_flag);

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
        return -1;
    }

            
    // 5. 配置位置模式
    // 使能
    mc.elmo_enable(motorCANID_knee_L);
    mc.elmo_enable(motorCANID_knee_R);

    // 读取一次电机当前的状态 作为零点
    elmo_read_state(curState1, zeroState1, mc, motorCANID_knee_L, return_canObj, receive_flag, true);
    zeroState1 = curState1;
    elmo_read_state(curState2, zeroState2, mc, motorCANID_knee_R, return_canObj, receive_flag, true);
    zeroState2 = curState2;

    // 设置当前脉冲数为0点，将位置轨迹转换为脉冲轨迹
    int zeroPulse1 = zeroState1.pulseNum;
    int zeroPulse2 = zeroState2.pulseNum;

    uint32_t temp = 0;

    double vel_temp = 0;
    double acc_temp = 0;
    std::vector<uint32_t> traj_knee_pulse1, vel_traj_knee_pulse1;
    std::vector<uint32_t> traj_knee_pulse2, vel_traj_knee_pulse2;

    
    for (size_t i = 0; i < trajectory_knee1.size(); i++)
    {
        mc.elmo_angle2pulse_mapping(trajectory_knee1[i],temp);
        temp = temp + zeroPulse1; // 偏置
        traj_knee_pulse1.push_back(temp);
        // std::cout << " temp is: " << temp;
        if ((i+1)<trajectory_knee1.size())
        {   
            vel_temp = (trajectory_knee1[i+1] - trajectory_knee1[i])/0.001;
            if (std::abs(vel_temp)>=60)
            {
                if (vel_temp<0)
                {
                    vel_traj_knee1.push_back(-60);
                }
                else
                {
                    vel_traj_knee1.push_back(60);
                }

            }
            else
            {
                vel_traj_knee1.push_back(vel_temp);
            }
        }        
    }
    for (size_t i = 0; i < trajectory_knee2.size(); i++)
    {
        mc.elmo_angle2pulse_mapping(trajectory_knee2[i],temp);
        temp = temp + zeroPulse2; // 偏置
        traj_knee_pulse2.push_back(temp);
        // std::cout << " temp is: " << temp;
        if ((i+1)<trajectory_knee2.size())
        {   
            vel_temp = (trajectory_knee2[i+1] - trajectory_knee2[i])/0.001;
            if (std::abs(vel_temp)>=60)
            {
                if (vel_temp<0)
                {
                    vel_traj_knee2.push_back(-60);
                }
                else
                {
                    vel_traj_knee2.push_back(60);
                }

            }
            else
            {
                vel_traj_knee2.push_back(vel_temp);
            }
        }        
    }
    // 计算加速度
    for (size_t i = 0; i < vel_traj_knee1.size()-1; i++)
    {
        
        acc_temp = (vel_traj_knee1[i+1] - vel_traj_knee1[i])/0.001;
        acc_traj_knee1.push_back(acc_temp);
        
    }
    for (size_t i = 0; i < vel_traj_knee2.size()-1; i++)
    {
        
        acc_temp = (vel_traj_knee2[i+1] - vel_traj_knee2[i])/0.001;
        acc_traj_knee2.push_back(acc_temp);
        
    }
    
    std::cout <<" acc_traj_knee's size is: "<<acc_traj_knee1.size()<< std::endl;
    auto maxValueAcc1 = std::max_element(acc_traj_knee1.begin(), acc_traj_knee1.end());
    std::cout <<" acc_traj_knee's maxValue is: "<<*maxValueAcc1<< std::endl;
    auto minValueAcc1 = std::min_element(acc_traj_knee1.begin(), acc_traj_knee1.end());
    std::cout <<" acc_traj_knee's minValue is: "<<*minValueAcc1<< std::endl;
    
    std::cout <<" vel_traj_knee's size is: "<<vel_traj_knee1.size()<< std::endl;
    auto maxValue1 = std::max_element(vel_traj_knee1.begin(), vel_traj_knee1.end());
    std::cout <<" vel_traj_knee's maxValue is: "<<*maxValue1<< std::endl;
    auto minValue1 = std::min_element(vel_traj_knee1.begin(), vel_traj_knee1.end());
    std::cout <<" vel_traj_knee's minValue is: "<<*minValue1<< std::endl;
    size_t max_index1 = std::distance(vel_traj_knee1.begin(), maxValue1);
    std::cout <<" vel_traj_knee's maxValue's index is: "<<max_index1<< std::endl;

    std::cout <<" acc_traj_knee's size is: "<<acc_traj_knee2.size()<< std::endl;
    auto maxValueAcc2 = std::max_element(acc_traj_knee2.begin(), acc_traj_knee2.end());
    std::cout <<" acc_traj_knee's maxValue is: "<<*maxValueAcc2<< std::endl;
    auto minValueAcc2 = std::min_element(acc_traj_knee2.begin(), acc_traj_knee2.end());
    std::cout <<" acc_traj_knee's minValue is: "<<*minValueAcc2<< std::endl;
    
    std::cout <<" vel_traj_knee's size is: "<<vel_traj_knee2.size()<< std::endl;
    auto maxValue2 = std::max_element(vel_traj_knee2.begin(), vel_traj_knee2.end());
    std::cout <<" vel_traj_knee's maxValue is: "<<*maxValue2<< std::endl;
    auto minValue2 = std::min_element(vel_traj_knee2.begin(), vel_traj_knee2.end());
    std::cout <<" vel_traj_knee's minValue is: "<<*minValue2<< std::endl;
    size_t max_index2 = std::distance(vel_traj_knee2.begin(), maxValue2);
    std::cout <<" vel_traj_knee's maxValue's index is: "<<max_index2<< std::endl;

    // 将速度从角度换算为脉冲/s单位  错点（已修正）profile velocity对象为无符号32位整型，所以负的速度需要取绝对值，否则隐式转换会变得很大
    for (size_t i = 0; i < vel_traj_knee1.size(); i++)
    {
        mc.elmo_angle2pulse_mapping(abs(vel_traj_knee1[i]), temp);
        vel_traj_knee_pulse1.push_back(temp);
    }
    for (size_t i = 0; i < vel_traj_knee2.size(); i++)
    {
        mc.elmo_angle2pulse_mapping(abs(vel_traj_knee2[i]), temp);
        vel_traj_knee_pulse2.push_back(temp);
    }
    


    // 设置位置模式
    mc.elmo_set_pos_mode(motorCANID_knee_L);
    mc.elmo_set_pos_mode(motorCANID_knee_R);
    usleep(100000);
    

    // 设置速度限制
    mc.elmo_set_vel(motorCANID_knee_L);
    mc.elmo_set_vel(motorCANID_knee_R);
    usleep(100000);

    // 设置加速度限制
    mc.elmo_set_acc(motorCANID_knee_L);
    mc.elmo_set_acc(motorCANID_knee_R);
    usleep(100000);

    // 设置减速度限制
    mc.elmo_set_dec(motorCANID_knee_L);
    mc.elmo_set_dec(motorCANID_knee_R);
    usleep(100000);
    
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();
    
    start = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < traj_knee_pulse1.size(); i= i+10)
    {
        // 设置期望位置
        
        mc.elmo_set_pos_pulse(motorCANID_knee_L, traj_knee_pulse1[i]);
        mc.elmo_set_pos_pulse(motorCANID_knee_R, traj_knee_pulse2[i]);

        
        // 设置速度
        if (i<traj_knee_pulse1.size()-1)
        {
            mc.elmo_set_vel_traj(motorCANID_knee_L, vel_traj_knee_pulse1[i]);
        }
        else
        {
            mc.elmo_set_vel_traj(motorCANID_knee_L, 0);
        }

        if (i<traj_knee_pulse2.size()-1)
        {
            mc.elmo_set_vel_traj(motorCANID_knee_R, vel_traj_knee_pulse2[i]);
        }
        else
        {
            mc.elmo_set_vel_traj(motorCANID_knee_R, 0);
        }

        // usleep(1000);
        // 设置相对位移模式  进行了位置模式的修改
        mc.elmo_config_rel_or_abs(motorCANID_knee_L, 1, 0);
        mc.elmo_config_rel_or_abs(motorCANID_knee_R, 1, 0);


        // // 设置end速度,也就是在pp模式中,到达目标点的速度 --待测试--有问题晚上继续
        // mc.elmo_set_end_vel(motorCANID, vel_traj_knee[i]);


        
        // 6. 执行
        mc.elmo_set_start(motorCANID_knee_L);
        mc.elmo_set_start(motorCANID_knee_R);

        usleep(1000); //实时改变时间--明天改

        // usleep(1000);
        // 8. 读取电机当点状态
        
        elmo_read_state(curState1, zeroState1, mc, motorCANID_knee_L, return_canObj, receive_flag, false);
        elmo_read_state(curState2, zeroState2, mc, motorCANID_knee_R, return_canObj, receive_flag, false);

        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTime(motorCANID_knee_L, curState1.pos, curState1.vel, curState1.tor, &filename_knee1, duration);
        dataWriteTime1(motorCANID_knee_R, curState2.pos, curState2.vel, curState2.tor, &filename_knee2, duration);

    }
    

    // 7. 检测是否达到对应位置
    do
    {
        mc.canopen_sdo_read(motorCANID_knee_L, 0x6041, 0);
        mc.receive_canopen(return_canObj, receive_flag);
        usleep(500);
        
    } while (!((return_canObj.canData[5]&0x04)==0x04));
    do
    {
        mc.canopen_sdo_read(motorCANID_knee_R, 0x6041, 0);
        mc.receive_canopen(return_canObj, receive_flag);
        usleep(500);
        
    } while (!((return_canObj.canData[5]&0x04)==0x04));


    sleep(3);
    // 9. 达到则禁用使能
    mc.elmo_disable(motorCANID_knee_L);
    mc.elmo_disable(motorCANID_knee_R);
    file.close();
    file1.close();
    
    return 0;
}

int elmo_pos_traj_ctrl_SDO_2_multiTraj(double posDesMax, int testNum){
    // 1.初始话串口对象 以及一些变量
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::CAN_Receive_Frame return_canObj; 
    int receive_flag;
    elmoState curState, zeroState;
    std::string filename_knee = std::string("test95_knee_L_final") + std::to_string(testNum) + ".txt";
    std::vector<double> trajectory_knee, vel_traj_knee, acc_traj_knee;
    
    
    // 多段五次多项式轨迹
    multiTrajGenerator_kneeVersion(trajectory_knee, posDesMax, 5);

    // 2.建立电机控制器对象，添加电机，
    motor_head::Motor_Control mc(serial);
    // int motorID = 9; //right knee
    int motorID = 3; //left knee

    mc.addMotor(&motors[motorID]);
    int motorCANID = motors[motorID].GetSlaveId();

    

    // 3.进入操作状态

    mc.canopen_operational(motorCANID);
    usleep(200);
    // 4.读取ELMO状态字  判断是否建立起了通信
    mc.canopen_sdo_read(motorCANID, 0x6041, 0);

    mc.receive_canopen_sdo(return_canObj, receive_flag);

    
    
    

    if (receive_flag == 0)
    {
        std::cerr << "ELMO's STATUSWORLD" <<"  receive failed" << std::endl;
    }
    else
    {
        if ((return_canObj.canId == static_cast<uint32_t>(0x580+motorCANID)) && ((return_canObj.canData[0] & static_cast<uint8_t>(0xF0)) == static_cast<uint8_t>(0x40))) //CANID==服务器响应ID，且反馈数据段操作码为0x43，且状态字第0位为1（即准备就绪）
        {
            std::cout << "状态字的低八位为: ";
            printHex(return_canObj.canData[4]);
            std::cout << " 状态字的高八位为: " ;
            printHex(return_canObj.canData[5]);
            std::cout << std::endl;
            
            // 5. 配置位置模式
            usleep(1000);
            // 使能
            mc.elmo_enable(motorCANID);
            
            // ensure first read state success
            bool initRes = false;
            do
            {
                initRes = elmo_read_init_state(zeroState, mc, motorCANID, return_canObj, receive_flag);
            } while (!initRes);
            
            
            elmo_read_state(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false);
            usleep(1000);
             
            
            reSendNumCanopen reSendNum;

            // 设置当前脉冲数为0点，将位置轨迹转换为脉冲轨迹
            int zeroPulse = zeroState.pulseNum;
            uint32_t temp = 0;
            double vel_temp = 0;
            double acc_temp = 0;
            std::vector<uint32_t> traj_knee_pulse, vel_traj_knee_pulse;
            

            int frequence = 500;
            double Time_cycle = 0.002;
            // pos_traj->pulse_traj If vel_traj be had
            // for (size_t i = 0; i < trajectory_knee.size(); i++)
            // {
            //     mc.elmo_angle2pulse_mapping(trajectory_knee[i],temp);
            //     temp = temp + zeroPulse; // 偏置
            //     traj_knee_pulse.push_back(temp);
                       
            // }

            // pos_traj->pulse_traj  
            // if only pos_traj data, this will solve vel_traj
            for (size_t i = 0; i < trajectory_knee.size(); i++)
            {
                mc.elmo_angle2pulse_mapping(trajectory_knee[i],temp);
                temp = temp + zeroPulse; // 偏置
                traj_knee_pulse.push_back(temp);
                // std::cout << " temp is: " << temp;
                if ((i+1)<trajectory_knee.size())
                {   
                    vel_temp = (trajectory_knee[i+1] - trajectory_knee[i])/0.001;  // 这里的时间间隔要和后面命令发送周期一致
                    if (std::abs(vel_temp)>=60)
                    {
                        // limit of max velocity
                        if (vel_temp<0)
                        {
                            vel_traj_knee.push_back(-60);
                        }
                        else
                        {
                            vel_traj_knee.push_back(60);
                        }
                    }
                    else
                    {
                        vel_traj_knee.push_back(vel_temp);
                    }

                    // std::cout << " vel_temp is: " << vel_temp;
                    // std::cout << " vel_traj_knee[i] is: " << vel_traj_knee[i];
    
                }        
            }
            // difference commercial -> acc
            for (size_t i = 0; i < vel_traj_knee.size()-1; i++)
            {
                
                acc_temp = (vel_traj_knee[i+1] - vel_traj_knee[i])/0.001;
                acc_traj_knee.push_back(acc_temp);
                
            }
            
            // print max vel & acc
            std::cout <<" acc_traj_knee's size is: "<<acc_traj_knee.size()<< std::endl;
            auto maxValueAcc = std::max_element(acc_traj_knee.begin(), acc_traj_knee.end());
            std::cout <<" acc_traj_knee's maxValue is: "<<*maxValueAcc<< std::endl;
            auto minValueAcc = std::min_element(acc_traj_knee.begin(), acc_traj_knee.end());
            std::cout <<" acc_traj_knee's minValue is: "<<*minValueAcc<< std::endl;
            
            std::cout <<" vel_traj_knee's size is: "<<vel_traj_knee.size()<< std::endl;
            auto maxValue = std::max_element(vel_traj_knee.begin(), vel_traj_knee.end());
            std::cout <<" vel_traj_knee's maxValue is: "<<*maxValue<< std::endl;
            auto minValue = std::min_element(vel_traj_knee.begin(), vel_traj_knee.end());
            std::cout <<" vel_traj_knee's minValue is: "<<*minValue<< std::endl;
            size_t max_index = std::distance(vel_traj_knee.begin(), maxValue);
            std::cout <<" vel_traj_knee's maxValue's index is: "<<max_index<< std::endl;

            // vel_traj->vel_pulse_traj
            for (size_t i = 0; i < vel_traj_knee.size(); i++)
            {
                mc.elmo_angle2pulse_mapping(abs(vel_traj_knee[i]), temp);
                vel_traj_knee_pulse.push_back(temp);
            }
            
            
            std::cout << std::endl;

            // // 等待输入继续执行
            // std::cout << "Press Enter to continue ..."<<std::endl;
            // std::cin.get();

            // 设置位置模式
            mc.elmo_set_pos_mode(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);
            

            // 设置速度限制
            mc.elmo_set_vel(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置加速度限制
            mc.elmo_set_acc(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            // 设置减速度限制
            mc.elmo_set_dec(motorCANID);
            if(!checkSDOCorrect(mc, return_canObj, receive_flag)) return-1;
            usleep(100000);

            
            // 等待输入继续执行
            // std::cout << "Press Enter to continue ..."<<std::endl;
            // std::cin.get();

            int timeBetFrame = 200;
            auto total_comm = 0;  // 累计通信耗时（us）
            auto total_read_state = 0; // 累计休眠耗时（us）
            auto total_write = 0; // 累计写入耗时（us）
            
            end = std::chrono::high_resolution_clock::now();;
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            auto duration_read_state = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            auto cmd_sending_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            std::cout << "initial time is: " << duration;
            // // 等待输入继续执行
            // std::cout << "Press Enter to continue ..."<<std::endl;
            // std::cin.get();
            auto comStart = std::chrono::high_resolution_clock::now();
            auto comEnd = std::chrono::high_resolution_clock::now();
            auto cmd_send_t = std::chrono::high_resolution_clock::now();
            
            start = std::chrono::high_resolution_clock::now();
            for (size_t i = 0; i < traj_knee_pulse.size(); i= i+2)
            {
                // 设置期望位置
                // std::cout << i << "th cycle of command" << std::endl;
                comStart = std::chrono::high_resolution_clock::now();
                mc.elmo_set_pos_pulse(motorCANID, traj_knee_pulse[i]);
                comEnd = std::chrono::high_resolution_clock::now();
                // usleep(timeBetFrame);
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us

                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();
                // 设置速度
                comStart = std::chrono::high_resolution_clock::now();
                if (i<traj_knee_pulse.size()-1)
                {

                    mc.elmo_set_vel_traj(motorCANID, vel_traj_knee_pulse[i]);
                }
                else
                {
                    mc.elmo_set_vel_traj(motorCANID, 0);
                }
                comEnd = std::chrono::high_resolution_clock::now();
                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us

                // usleep(timeBetFrame);
                // 设置相对位移模式  进行了位置模式的修改
                comStart = std::chrono::high_resolution_clock::now();

                mc.elmo_config_rel_or_abs(motorCANID, 1, 0);
                comEnd = std::chrono::high_resolution_clock::now();
                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();


                // usleep(timeBetFrame);
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us


                // 6. 执行
                comStart = std::chrono::high_resolution_clock::now();

                mc.elmo_set_start(motorCANID);
                comEnd = std::chrono::high_resolution_clock::now();
                cmd_sending_time = std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - start).count();
                total_comm += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();

                // usleep(timeBetFrame);
                std::this_thread::sleep_for(std::chrono::microseconds(timeBetFrame)); //更精确的延时us

                // 8. 读取电机当点状态
                
                // elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, re);
                // resReadState = elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, reSendNum);
                comStart = std::chrono::high_resolution_clock::now();

                elmo_read_stateV3(curState, zeroState, mc, motorCANID, return_canObj, receive_flag, false, reSendNum);
                comEnd = std::chrono::high_resolution_clock::now();

                total_read_state += std::chrono::duration_cast<std::chrono::milliseconds>(comEnd - comStart).count();

                duration_read_state = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                // dataWriteTime(motorCANID, curState.pos, curState.vel, curState.tor, &filename_knee, duration);
                // 待验证， 记录了状态时间和指令发送时间（有通信时间的偏差
                dataWriteTimewithSendingTime(motorCANID, curState.pos, curState.vel, curState.tor, &filename_knee, duration_read_state, cmd_sending_time); 

                end = std::chrono::high_resolution_clock::now();
                total_write += std::chrono::duration_cast<std::chrono::milliseconds>(end - comEnd).count();

                
                // usleep(800);
                std::this_thread::sleep_for(std::chrono::microseconds(800)); //更精确的延时us

            }
            end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::cout  << "all time of ctrl is: " << duration << "\n"
            << "total_comm is: " << total_comm <<"\n"
            << "total_read_state is: " << total_read_state<<"\n"
            << "total_write is: " << total_write << std::endl;

            // 7. 检测是否达到对应位置
            do
            {
                mc.canopen_sdo_read(motorCANID, 0x6041, 0);
                // mc.receive_canopen(return_canObj, receive_flag);
                mc.receive_canopen_sdo(return_canObj, receive_flag);

                usleep(500);
                end = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            } while (!(  ((return_canObj.canData[5]&0x04)==0x04) || duration>=1000  ));

            if (duration>=1000)
            {
                std::cout << "do not reach target pos" << std::endl;
            }
            
            sleep(3);
            // 9. 达到则禁用使能
            mc.elmo_disable(motorCANID);
            file.close();

        }
        else
        {
            if (return_canObj.canData[0] == 0x80)
            {
                std::cerr << "ELMO's STATUSWORLD" <<"  abort code" << std::endl;
                std::cerr << "exception code is " << return_canObj.canData[4] << std::endl;
            }
            else
            {
                std::cerr << "除了正常接收以及报错以外的异常" << std::endl;
                std::cout << "CANFrame's canID is: ";
                printHex(return_canObj.canId);
                std::cout << " CANFrame's canData[0] is: " ;
                printHex(return_canObj.canData[0]);
                std::cout << std::endl;


            }
            
            
        }
        
        
    }
    
    return 0;
}

// 新版本的打印函数,可以将整型打印为0x16进制的形式
/**
 * 输出任意整数的十六进制表示，修复uint8_t被当作字符输出的问题
 * @tparam T 整数类型（必须是整数，否则编译报错）
 * @param value 要输出的整数
 * @param prefix 是否添加 0x 前缀（默认 true）
 * @param uppercase 是否使用大写字母（默认 false）
 */
template <typename T>
typename std::enable_if<std::is_integral<T>::value>::type
printHex(T value, bool prefix, bool uppercase) {
    // 保存原始输出格式
    std::ios_base::fmtflags original_flags = std::cout.flags();
    
    // 关键修复：强制以数值形式输出，而非字符形式
    std::cout << std::noshowbase; // 临时关闭默认基数显示
    std::cout << std::hex;        // 切换到十六进制模式
    
    // 控制大小写
    if (uppercase) {
        std::cout << std::uppercase;
    } else {
        std::cout << std::nouppercase;
    }
    
    // 输出前缀
    if (prefix) {
        std::cout << "0x";
    }
    
    // 核心处理：对所有整数类型（尤其是uint8_t）强制转换为uintmax_t，
    // 确保以数值而非字符形式输出
    std::cout << static_cast<uintmax_t>(value);
    
    // 恢复原始格式
    std::cout.flags(original_flags);
}

bool checkSDOCorrect(motor_head::Motor_Control& mc_, motor_head::CAN_Receive_Frame& return_canObj, int& receive_flag)
{
    mc_.receive_canopen_sdo(return_canObj, receive_flag);
    if ((return_canObj.canId & 0xFF0)==0x580 && ((return_canObj.canData[0] == 0x60) || ((return_canObj.canData[0] & 0xF0) == 0x40)))
    {
        return true;
    }
    else
    {
        return false;
    }
    
}

bool elmo_read_init_state(elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_){
    bool res = false; // read failed
    // 1.读取当前脉冲
    mc_.elmo_read_pos(id, return_canObj_, receive_flag_);
    if (receive_flag_ == 1)  //receive suc
    {
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        // 从无符号整型, 变为这个对象本身的数据类型int32
        uint32_t dataObj = static_cast<int>(dataField); 
        zeroState_.pulseNum = dataObj; 
        // std::cout << "  state_.pulseNum's value is: " << dataObj << std::endl;
        std::cout << "  init pulseNum's value received success "<< std::endl;
        res = true;
    }else
    {
        std::cout << "  init pulseNum's value received failed "<< std::endl;
    }
    return res;
    
}


// 待测试 7.30--无问题8.1
void elmo_read_state(elmoState& state_, elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall){
    
    // 1.读取当前位置
    mc_.elmo_read_pos(id, return_canObj_, receive_flag_);
    if (receive_flag_ == 1)  //receive suc
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of pos obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        // 从无符号整型, 变为这个对象本身的数据类型int32
        uint32_t dataObj = static_cast<int>(dataField);
        if (FirstCall)
        {
            zeroState_.pulseNum = dataObj;
        }
        state_.pulseNum = dataObj;
        // std::cout << "  state_.pulseNum's value is: " << dataObj << std::endl;
        int deltPulse = state_.pulseNum - zeroState_.pulseNum;
        // std::cout << "  deltaPulse's value is: " << deltPulse;
        mc_.elmo_xData_mapping(deltPulse, state_.pos);
        std::cout << "  state_.pos's value is: " << state_.pos << std::endl;
    }else
    {
        std::cout << "  state_.pos's value received failed "<< std::endl;
    }
    
    
    usleep(1000);

    // 2.读取当前速度  暂时未用相对脉冲计算
    mc_.elmo_read_vel(id, return_canObj_, receive_flag_);
    if (receive_flag_== 1)
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of vel obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        mc_.elmo_xData_mapping(dataField, state_.vel);
        std::cout << "  state_.vel's value is: " << state_.vel << std::endl;
    }else
    {
        std::cout << "  state_.vel's value received failed "<< std::endl;
    }
    
    

    usleep(1000);
    // 3.读取当前力矩 需向电机厂商询问映射关系
    mc_.elmo_read_tor(id, return_canObj_, receive_flag_);
    if (receive_flag_ ==1)
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of cur obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 2);
        std::cout << "  current's value is: " << dataField << std::endl;
    }else
    {
        std::cout << "  state_.current's value received failed "<< std::endl;
    }
    
}
// 8.25 test success
void elmo_read_stateV2(elmoState& state_, elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall){
    
    // 1.读取当前位置
    mc_.elmo_read_posV2(id, return_canObj_, receive_flag_);
    if (receive_flag_ == 1)  //receive suc
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of pos obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        // 从无符号整型, 变为这个对象本身的数据类型int32
        uint32_t dataObj = static_cast<int>(dataField);
        if (FirstCall)
        {
            zeroState_.pulseNum = dataObj;
        }
        state_.pulseNum = dataObj;
        // std::cout << "  state_.pulseNum's value is: " << dataObj << std::endl;
        int deltPulse = state_.pulseNum - zeroState_.pulseNum;
        // std::cout << "  deltaPulse's value is: " << deltPulse;
        mc_.elmo_xData_mapping(deltPulse, state_.pos);
        std::cout << "  state_.pos's value is: " << state_.pos << std::endl;
    }else
    {
        std::cout << "  state_.pos's value received failed "<< std::endl;
    }
    
    
    usleep(200);

    // 2.读取当前速度  暂时未用相对脉冲计算
    mc_.elmo_read_velV2(id, return_canObj_, receive_flag_);
    if (receive_flag_== 1)
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of vel obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        mc_.elmo_xData_mapping(dataField, state_.vel);
        std::cout << "  state_.vel's value is: " << state_.vel << std::endl;
    }else
    {
        std::cout << "  state_.vel's value received failed "<< std::endl;
    }
    
    

    usleep(200);
    // 3.读取当前力矩 需向电机厂商询问映射关系
    mc_.elmo_read_torV2(id, return_canObj_, receive_flag_);
    if (receive_flag_ ==1)
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of cur obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 2);
        std::cout << "  current's value is: " << dataField << std::endl;
    }else
    {
        std::cout << "  state_.current's value received failed "<< std::endl;
    }
    
}

// 发送失败重新发送  待实现
int elmo_read_stateV3(elmoState& state_, elmoState& zeroState_, motor_head::Motor_Control& mc_, Motor_id id, motor_head::CAN_Receive_Frame& return_canObj_, int& receive_flag_, bool FirstCall, reSendNumCanopen& reSend){
    

    int SendNum = 0;
    int delayBetFrame = 200;
    // usleep(delayBetFrame);
    // 1.读取当前位置
    receive_flag_ = 0; // 保证不被上一次接受状态干扰
    do
    {
        SendNum++;
        // std::cout <<"   "<< SendNum << "th Send read pos" << std::endl;
        mc_.elmo_read_posV2(id, return_canObj_, receive_flag_);
        reSend.posSendNum = SendNum-1;
        if (SendNum==10)
        {
            // std::cout << "  state_.pos's value received failed "<< std::endl;
            
            // std::cout << "  communication may exception" << std::endl;
            return -1;
        }
        
    } while (!(receive_flag_==1));

    if (SendNum>=2 && SendNum<10)
    {
        // std::cout << "  REreceive success" << std::endl;
    }
    

    if (receive_flag_ == 1)  //receive suc
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of pos obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        // 从无符号整型, 变为这个对象本身的数据类型int32
        uint32_t dataObj = static_cast<int>(dataField);
        if (FirstCall)
        {
            zeroState_.pulseNum = dataObj;
        }
        state_.pulseNum = dataObj;
        // std::cout << "  state_.pulseNum's value is: " << dataObj << std::endl;
        int deltPulse = state_.pulseNum - zeroState_.pulseNum;
        // std::cout << "  deltaPulse's value is: " << deltPulse;
        mc_.elmo_xData_mapping(deltPulse, state_.pos);
        // std::cout << "  state_.pos's value is: " << state_.pos << std::endl;
    }
    
    usleep(delayBetFrame);

    // 2.读取当前速度  暂时未用相对脉冲计算
    receive_flag_ = 0; // 保证不被上一次接受状态干扰
    SendNum = 0;

    do
    {
        SendNum++;
        // std::cout <<"   "<< SendNum << "th Send read vel" << std::endl;
        mc_.elmo_read_velV2(id, return_canObj_, receive_flag_);
        reSend.velSendNum = SendNum-1;
        
        if (SendNum==10)
        {
            // std::cout << "  state_.vel's value received failed "<< std::endl;
            
            // std::cout << "  communication may exception" << std::endl;
            return -2;
        }
        
    } while (!(receive_flag_==1));

    if (SendNum>=2 && SendNum<10)
    {
        // std::cout << "  REreceive success" << std::endl;
    }
    

    if (receive_flag_== 1)
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of vel obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 4);
        mc_.elmo_xData_mapping(dataField, state_.vel);
        // std::cout << "  state_.vel's value is: " << state_.vel << std::endl;
    }else
    {
        // std::cout << "  state_.vel's value received failed "<< std::endl;
    }
    
    

    usleep(delayBetFrame);
    // 3.读取当前力矩 需向电机厂商询问映射关系
    receive_flag_ = 0; // 保证不被上一次接受状态干扰
    SendNum = 0;

    do
    {
        SendNum++;
        // std::cout <<"   "<< SendNum << "th Send read tor" << std::endl;
        mc_.elmo_read_torV2(id, return_canObj_, receive_flag_);
        reSend.torSendNum = SendNum-1;

        if (SendNum==10)
        {
            // std::cout << "  state_.current's value received failed "<< std::endl;
            // std::cout << "  communication may exception" << std::endl;
            return -3;
        }
        
    } while (!(receive_flag_==1));

    if (SendNum>=2 && SendNum<10)
    {
        // std::cout << "  REreceive success" << std::endl;
    }
    
    SendNum = 0;

    if (receive_flag_ ==1)
    {
        // std::cout << "receive CANFrame ID is: " ;
        // printHex(return_canObj_.canId);
        // std::cout << "  index of cur obj 's value is: ";
        // std::cout << " data[0] is: ";
        // printHex(return_canObj_.canData[0]);
        // std::cout << " data[1] is: ";
        // printHex(return_canObj_.canData[1]);
        // std::cout << " data[2] is: ";
        // printHex(return_canObj_.canData[2]);
        // std::cout << " data[3] is: ";
        // printHex(return_canObj_.canData[3]);
        // std::cout << " data[4] is: ";
        // printHex(return_canObj_.canData[4]);
        // std::cout << " data[5] is: ";
        // printHex(return_canObj_.canData[5]);
        // std::cout << " data[6] is: ";
        // printHex(return_canObj_.canData[6]);
        // std::cout << " data[7] is: ";
        // printHex(return_canObj_.canData[7]);
        uint32_t dataField = littleToBigEndian(&return_canObj_.canData[4], 2);
        // std::cout << "  current's value is: " << dataField << std::endl;
    }

    return 0;
    
}


/**
 * 将小于等于4字节的小端字节数组转换为大端字节序 AI初版,在我改正后正确
 * @param little_endian_bytes 输入的小端字节数组
 * @param byte_count 字节数量（1-4）
 * @return 转换后的大端字节数组（用uint32_t存储，高位补0）
 */
uint32_t littleToBigEndian(const uint8_t* little_endian_bytes, size_t byte_count) {
    // 检查输入合法性
    if (byte_count < 1 || byte_count > 4) {
        throw std::invalid_argument("字节数量必须在1到4之间");
    }
    if (little_endian_bytes == nullptr) {
        throw std::invalid_argument("输入字节数组不能为空指针");
    }

    uint32_t result = 0;
    
    // 小端转大端：反转字节顺序并组合
    // 例如：
    // 1字节: [b0] -> [b0]
    // 2字节: [b0, b1] -> [b1, b0]
    // 3字节: [b0, b1, b2] -> [b2, b1, b0]
    // 4字节: [b0, b1, b2, b3] -> [b3, b2, b1, b0]
    for (size_t i = 0; i < byte_count; ++i) {
        result |= static_cast<uint32_t>(little_endian_bytes[i]) << (8 * (i));
    }

    return result;
}


// 分割字符串（按逗号分割）
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    
    // 按分隔符分割字符串
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// 读取CSV文件并返回所有行（每行是一个字段向量）
std::vector<std::vector<std::string>> readCSV(const std::string& filename) {
    std::vector<std::vector<std::string>> data;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
    
    std::string line;
    // 逐行读取文件
    while (std::getline(file, line)) {
        // 分割当前行的字段
        std::vector<std::string> row = split(line, ',');
        data.push_back(row);
    }
    
    file.close();
    return data;
}

void CSV_string2double(std::vector<std::vector<double>>* result){
    try {
        // 读取CSV文件
        std::string package_name = "motor_ctrl";
        std::string cmd = "rospack find " + package_name;
        std::string package_path = execute_command(cmd.c_str()); // 实际需用popen解析输出
        // 删除末尾所有空白字符（包括 \n, \r, 空格等）
        size_t endpos = package_path.find_last_not_of(" \n\r\t");
        if (endpos != std::string::npos) {
            package_path = package_path.substr(0, endpos + 1);
        } else {
            package_path.clear(); // 若全为空白则清空
        }
        std::string file_path = package_path + "/doc/output_q.csv";
        std::cout << "file_path: " << file_path << std::endl;
        auto csvData = readCSV(file_path);
        
        // 输出CSV内容
        std::cout << "CSV文件内容（共 " << csvData.size() << " 行）：\n";
        for (size_t i = 0; i < csvData.size(); ++i) {
            // std::cout << "行 " << i + 1 << ": ";
            for (size_t j = 0; j < csvData[i].size(); j++)
            {
                // std::cout << "[" << csvData[i][j] << "] ";
                (*result)[i][j] = std::stod(csvData[i][j]);
            }
            // std::cout << "\n";
        }
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        result = nullptr;
    }
}