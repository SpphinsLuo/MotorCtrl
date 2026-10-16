#include <cstdio>
#include <cstdlib>
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
#include <thread>

// some globle var 

static std::ofstream file[6];  // 静态变量保持打开状态

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

// all motors
motor_head::Motor motors[12] = {
    // index == canID 参数列表：电机类型 从站ID 主站ID
    motor_head::Motor(motor_head::PA105, 0x03, 0x02), //hip pitch
    motor_head::Motor(motor_head::PA105, 0x04, 0x02), //hip roll
    motor_head::Motor(motor_head::PA105, 0x05, 0x02), //hip yaw
    motor_head::Motor(motor_head::PA105, 0x06, 0x02), //knee
    motor_head::Motor(motor_head::HA60, 0x07, 0x02), //ankle pitch
    motor_head::Motor(motor_head::HA60, 0x08, 0x02), // ankle roll

    motor_head::Motor(motor_head::PA105, 0x09, 0x02), //hip pitch
    motor_head::Motor(motor_head::PA105, 0x0A, 0x02), //hip roll
    motor_head::Motor(motor_head::PA105, 0x0B, 0x02), //hip yaw
    motor_head::Motor(motor_head::PA105, 0x0C, 0x02), //knee 12
    motor_head::Motor(motor_head::HA60, 0x0D, 0x02), //ankle pitch
    motor_head::Motor(motor_head::HA60, 0x0E, 0x02), // ankle roll

    // motor_head::Motor(motor_head::PA105, 0x601, 0x02), //knee 12


};

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

void testReadTraj(){
    // 关节索引映射
    const std::vector<int> joint_indices = {
        0, 1, 2, 3, 4, 5,  // 左腿关节
        6, 7, 8, 9, 10, 11  // 右腿关节
    };
    std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        std::cout << trajectory[i].joint_angles[0] << " ";
    }
    std::cout << std::endl;


}


void dataWriteTime(int canID,  int64_t time, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int whichFileObj){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor) + " " + std::to_string(time);
    std::ofstream& file_ = file[whichFileObj];
    
    if (!file_.is_open())
    {
        std::string save_path = "/home/ye/00_jfl_26.10/MotorCtrl/data/evo_motor_ctrl/";
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

void dataWriteTimewithSendingTime(int canID,  int64_t time, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int whichFileObj, int64_t sendingTime){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + 
                            std::to_string(motor_tor) + " " + std::to_string(time)+" "+ std::to_string(sendingTime);
    std::ofstream& file_ = file[whichFileObj];
    
    if (!file_.is_open())
    {
        std::string save_path = "/home/ye/00_jfl_26.10/MotorCtrl/data/evo_motor_ctrl_withSendingTime/";
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

std::string uint8_to_hex_str(uint8_t value){
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", value);
    return std::string(buf);
}

void dataWriteHex(int canID,  int64_t time, motor_head::CAN_Receive_Frame& receive_data_, const std::string* filename, int whichFileObj){
    std::string contest = std::to_string(canID)+" "+std::to_string(time);
    for (size_t i = 0; i < 8; i++)
    {
        contest+=" " + std::to_string(i) + "th byte" + " " + uint8_to_hex_str(receive_data_.canData[i]);
    }
    std::ofstream& file_ = file[whichFileObj];
    
    if (!file_.is_open())
    {
        std::string save_path = "/home/ye/00_jfl_26.10/MotorCtrl/data/motor_ctrl/evo_motor_ctrl/";
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

// 定义五次多项式函数
double quinticPolynomial(double t, double a0, double a1, double a2, double a3, double a4, double a5) {
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
}

// 定义五次多项式的一阶导数（速度）
double quinticPolynomialVelocity(double t, double a1, double a2, double a3, double a4, double a5) {
    return a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t + 5 * a5 * t * t * t * t;
}

double degree2rad(double& degree){
    return degree * pi / 180.0;
}
double rad2degree(double& rad){
    return rad * 180.0 / pi;
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
    
    // std::cout << "位置\n";
    // 打印第一部分轨迹
    // for (size_t i = 0; i < t.size(); ++i) {
    //     // std::cout << t[i] << "\t" << trajectory[i] << "\t" << velocity[i] <<"\n";
    //     std::cout << trajectory[i] << " ";
    // }

    // // 打印第二部分轨迹
    // for (size_t i = 0; i < t2.size(); ++i) {
    //     // std::cout << t2[i] << "\t" << trajectory[t.size() + i] << "\t" << velocity[t.size() + i] <<"\n";
    //     std::cout << trajectory[t.size() + i] << " ";

    // }
    // std::cout << "trajectory.size() is:" << trajectory.size() << "\n";
    // std::cout << "velocity.size() is:" << velocity.size() << "\n";
    // std::cout << "t.size() is:" << t.size() << "\n";
    // std::cout << "t2.size() is:" << t2.size() << "\n";


    
}

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
void multiTrajGenerator(std::vector<double>& trajectory, double gain_traj, int sectionNum){
    if (sectionNum<=0)
    {
        return;
    }
    
    for (size_t i = 0; i < sectionNum; i++)
    {
        std::vector<double> singleTraj, singleVelTraj;
        if (i%2==0)
        {
            trajectoryGenerator(singleTraj, singleVelTraj, gain_traj);
        }else{
            trajectoryGenerator(singleTraj, singleVelTraj, -gain_traj);
        }
        
        for (size_t j = 0; j < singleTraj.size(); j++)
        {
            trajectory.push_back(singleTraj[j]);
        }
    }
    
}

/**
 * @brief due to knee always + ，multi need to be wrote
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
    multiTrajGenerator(traj, 10, 5);
    writeVectorToTxt(traj, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj920.txt");
}

/**
 * @version 使用更精准的时间库
*/
int left_leg_hip_pitch_ctrl(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);//初始化串口
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 0;
    int motorCanID = motors[motorID].GetSlaveId();
    // int testNum = 1;

    std::string filename_hip_pitch = std::to_string(date)+std::string("left_hip_pitch_") + std::to_string(testNum) + ".txt";
    // std::string filename_hip_pitch_hex =  std::to_string(date)+std::string("left_hip_pitch_Hex") + std::to_string(testNum) + ".txt";


    


    
    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);


    // ZMP离线轨迹
    // // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch;

    // 五次多项式轨迹
    trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, targetpos);

    // ZMP轨迹
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    // }

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch;


    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 90;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 50;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0.01;


    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }
     
        
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

int test01(){
    left_leg_hip_pitch_ctrl(1, 20, 919);
}



/**
 * @version 控制新驱动板的测试程序
*/
int new_driver_ctrl(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 0;
    int motorCanID = 1;
    // int testNum = 1;

    std::string filename_hip_pitch = std::to_string(date)+std::string("new_driver_ctrl_") + std::to_string(testNum) + ".txt";
    // std::string filename_hip_pitch_hex =  std::to_string(date)+std::string("left_hip_pitch_Hex") + std::to_string(testNum) + ".txt";


    


    
    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);


    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch;

    // 五次多项式轨迹
    // trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, targetpos);
    multiTrajGenerator(trajectory_hip_pitch, targetpos, 5);

    // ZMP轨迹
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    // }

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch;


    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 90;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 2;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0;

    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }
     
        
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

void test_new_driver(){
    new_driver_ctrl(1, 40, 1010);
}


/**
 * @version 多段五次多项式轨迹
*/
int left_leg_hip_pitch_ctrl_multiTraj(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 0;
    int motorCanID = motors[motorID].GetSlaveId();
    // int testNum = 1;

    std::string filename_hip_pitch = std::to_string(date)+std::string("left_hip_pitch_") + std::to_string(testNum) + ".txt";
    // std::string filename_hip_pitch_hex =  std::to_string(date)+std::string("left_hip_pitch_Hex") + std::to_string(testNum) + ".txt";


    


    
    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);


    // ZMP离线轨迹
    // // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch;

    // 五次多项式轨迹
    // trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, targetpos);
    multiTrajGenerator(trajectory_hip_pitch, targetpos, 5);
    // ZMP轨迹
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    // }

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch;


    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 200;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 20;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0;


    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }

    for (size_t i = 0; i < 500; i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }
     
    mc.Motorevo_motor_servo_mode(motors[motorID], 0, mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
    
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

int test01_2(){
    left_leg_hip_pitch_ctrl_multiTraj(8, 20, 922);
    return 0;
}

// 未测试
int left_leg_hip_roll_ctrl(int testNum, int targetpos){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 1;

    // int testNum = 1;

    std::string filename_hip_roll = std::string("test94_hip_roll_left_") + std::to_string(testNum) + ".txt";

    mc.enable(motors[motorID]);


    
    mc.addMotor(&motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);
    
    // 关节索引映射
    const std::vector<int> joint_indices = {
        0, 1, 2, 3, 4, 5,  // 左腿关节
        6, 7, 8, 9, 10, 11  // 右腿关节
    };
    std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);
    // double ankleMotorH = ankleInvKinHighTheta1(10, 0);
    // double ankleMotorL = ankleInvKinLowTheta2(10, 0);
    // 轨迹产生
    std::vector<double> trajectory_hip_roll,velocity_hip_roll;
                        // trajectory_hip_roll,velocity_hip_roll,
                        // trajectory_hip_yaw,velocity_hip_yaw,
                        // trajectory_ankle_H,velocity_ankle_H,
                        // trajectory_ankle_L,velocity_ankle_L;

    // trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, targetpos);
    // trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, 10);
    // trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, 10);
    // trajectoryGenerator(trajectory_ankle_H, velocity_ankle_H, ankleMotorH);
    // trajectoryGenerator(trajectory_ankle_L, velocity_ankle_L, ankleMotorL);
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        trajectory_hip_roll.push_back(rad2degree(trajectory[i].joint_angles[1]));
    }


    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch;
    // , mc_hip_roll, mc_hip_yaw, mc_ankle_high, mc_ankle_low;

    mc_hip_pitch.pos_desired = trajectory_hip_roll;
    mc_hip_pitch.velocity_limit = 200;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 200;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0.01;

    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    for (size_t i = 0; i < trajectory_hip_roll.size(); i++)
    {
        // hip pitch
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTime(motorID, duration, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), motors[motorID].Get_tau(), &filename_hip_roll, 0); // 保存数据到文件中

        
        // // hip roll
        // dm.Motorevo_motor_servo_mode(motors[1], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        // dm.receive_evo();
        // dataWrite(motorID[1], motors[1].Get_Position(), motors[1].Get_Velocity(), motors[1].Get_tau(), &filename_hip_roll, 1); // 保存数据到文件中

        // // hip yaw
        // dm.Motorevo_motor_servo_mode(motors[2], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        // dm.receive_evo();
        // dataWrite(motorID[2], motors[2].Get_Position(), motors[2].Get_Velocity(), motors[2].Get_tau(), &filename_hip_yaw, 2); // 保存数据到文件中

        // usleep(1000);
        // // ankle high
        // dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        // dm.receive_evo();
        // dataWrite(4, motors[4].Get_Position(), motors[4].Get_Velocity(), motors[4].Get_tau(), &filename_ankle_high, 4); // 保存数据到文件中

        // // ankle low
        // dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        // dm.receive_evo();
        // dataWrite(5, motors[5].Get_Position(), motors[5].Get_Velocity(), motors[5].Get_tau(), &filename_ankle_low, 5); // 保存数据到文件中
        // usleep(1000);
    }
     
        
    
    // for (size_t i = 0; i < 5; i++)
    // {
    //     dm.disable(motors[motorID[i]]);
    // }
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

int test02(){
    left_leg_hip_roll_ctrl(2, 20);
}



int left_leg_hip_roll_ctrl_multiTraj(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 1;
    int motorCanID = motors[motorID].GetSlaveId();
    

    std::string filename_hip_pitch = std::to_string(date)+std::string("_left_hip_roll_") + std::to_string(testNum) + ".txt";
      

    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹产生
    std::vector<double> trajectory_hip_roll,velocity_hip_roll;

    // 五次多项式轨迹
    // trajectoryGenerator(trajectory_knee, velocity_knee, targetpos);  // 测试轨迹无问题927
    multiTrajGenerator(trajectory_hip_roll, targetpos, 5);
    // writeVectorToTxt(trajectory_knee, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj927_2.txt");  //测试轨迹无问题927

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_knee;


    mc_knee.pos_desired = trajectory_hip_roll;
    mc_knee.velocity_limit = 90;
    mc_knee.posKp = 50;
    mc_knee.posKd = 0;
    mc_knee.velKp = 20;
    mc_knee.velKd = 0;
    mc_knee.velKi = 0;

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    for (size_t i = 0; i < trajectory_hip_roll.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_knee.pos_desired[i], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }

    for (size_t i = 0; i < 500; i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_knee.pos_desired[trajectory_hip_roll.size()-1], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }
     
    mc.Motorevo_motor_servo_mode(motors[motorID], 0, mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
    
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

void test_hip_roll_multi(){
    left_leg_hip_roll_ctrl_multiTraj(4, 10, 1029);
}

int left_leg_hip_yaw_ctrl(int testNum, int targetpos){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 2;

    // int testNum = 1;

    std::string filename_hip_yaw = std::string("test94_hip_yaw_left_") + std::to_string(testNum) + ".txt";
    // std::string filename_hip_yaw_hex = std::string("test94_hip_yaw_left_") + std::to_string(testNum) + ".txt";


    // for (size_t i = 0; i < 2; i++)
    // {
    //     mc.disable();

    // }
    mc.disable(motors[motorID]);  // 清除故障状态
    
    

    mc.enable(motors[motorID]);


    
    mc.addMotor(&motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);
    // 关节索引映射
    const std::vector<int> joint_indices = {
        0, 1, 2, 3, 4, 5,  // 左腿关节
        6, 7, 8, 9, 10, 11  // 右腿关节
    };
    std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);
    // double ankleMotorH = ankleInvKinHighTheta1(10, 0);
    // double ankleMotorL = ankleInvKinLowTheta2(10, 0);

    
    // 轨迹产生
    std::vector<double>  trajectory_hip_yaw,velocity_hip_yaw;

                        // trajectory_hip_roll,velocity_hip_roll,
                        // trajectory_hip_yaw,velocity_hip_yaw,
                        // trajectory_ankle_H,velocity_ankle_H,
                        // trajectory_ankle_L,velocity_ankle_L;

    // trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, targetpos);
    // trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, 10);
    // trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, 10);
    // trajectoryGenerator(trajectory_ankle_H, velocity_ankle_H, ankleMotorH);
    // trajectoryGenerator(trajectory_ankle_L, velocity_ankle_L, ankleMotorL);

    for (size_t i = 0; i < trajectory.size(); i++)
    {
        trajectory_hip_yaw.push_back(rad2degree(trajectory[i].joint_angles[2]));
    }

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_yaw;
    // , mc_hip_roll, mc_hip_yaw, mc_ankle_high, mc_ankle_low;

    mc_hip_yaw.pos_desired = trajectory_hip_yaw;
    mc_hip_yaw.velocity_limit = 200;
    mc_hip_yaw.posKp = 50;
    mc_hip_yaw.posKd = 0;
    mc_hip_yaw.velKp = 200;
    mc_hip_yaw.velKd = 0;
    mc_hip_yaw.velKi = 0.01;

    // mc_hip_roll.pos_desired = trajectory_hip_roll;
    // mc_hip_roll.velocity_limit = 200;
    // mc_hip_roll.posKp = 50;
    // mc_hip_roll.posKd = 10;
    // mc_hip_roll.velKp = 50;
    // mc_hip_roll.velKd = 0;
    // mc_hip_roll.velKi = 0.001;

    // mc_hip_yaw.pos_desired = trajectory_hip_yaw;
    // mc_hip_yaw.velocity_limit = 200;
    // mc_hip_yaw.posKp = 50;
    // mc_hip_yaw.posKd = 10;
    // mc_hip_yaw.velKp = 50;
    // mc_hip_yaw.velKd = 0;
    // mc_hip_yaw.velKi = 0.001;

    // mc_ankle_high.pos_desired = trajectory_ankle_H;
    // mc_ankle_high.velocity_limit = 200;
    // mc_ankle_high.posKp = 50;
    // mc_ankle_high.posKd = 10;
    // mc_ankle_high.velKp = 50;
    // mc_ankle_high.velKd = 0;
    // mc_ankle_high.velKi = 0.001;

    // mc_ankle_low.pos_desired = trajectory_ankle_L;
    // mc_ankle_low.velocity_limit = 200;
    // mc_ankle_low.posKp = 50;
    // mc_ankle_low.posKd = 10;
    // mc_ankle_low.velKp = 50;
    // mc_ankle_low.velKd = 0;
    // mc_ankle_low.velKi = 0.001;

    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    for (size_t i = 0; i < trajectory_hip_yaw.size(); i++)
    {
        // hip yaw
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTime(motorID, duration, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), motors[motorID].Get_tau(), &filename_hip_yaw, 0); // 保存数据到文件中
        // dataWriteHex(motorID, duration, *(mc.Get_receive_data()), &filename_hip_yaw_hex, 1);
        
        // // hip roll
        // dm.Motorevo_motor_servo_mode(motors[1], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        // dm.receive_evo();
        // dataWrite(motorID[1], motors[1].Get_Position(), motors[1].Get_Velocity(), motors[1].Get_tau(), &filename_hip_roll, 1); // 保存数据到文件中

        // // hip yaw
        // dm.Motorevo_motor_servo_mode(motors[2], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        // dm.receive_evo();
        // dataWrite(motorID[2], motors[2].Get_Position(), motors[2].Get_Velocity(), motors[2].Get_tau(), &filename_hip_yaw, 2); // 保存数据到文件中

        // usleep(1000);
        // // ankle high
        // dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        // dm.receive_evo();
        // dataWrite(4, motors[4].Get_Position(), motors[4].Get_Velocity(), motors[4].Get_tau(), &filename_ankle_high, 4); // 保存数据到文件中

        // // ankle low
        // dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        // dm.receive_evo();
        // dataWrite(5, motors[5].Get_Position(), motors[5].Get_Velocity(), motors[5].Get_tau(), &filename_ankle_low, 5); // 保存数据到文件中
        // usleep(1000);
    }
     
        
    
    // for (size_t i = 0; i < 5; i++)
    // {
    //     dm.disable(motors[motorID[i]]);
    // }
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

int test03(){
    left_leg_hip_yaw_ctrl(2, 10);
}


int left_leg_knee_ctrl_multiTraj(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 3;
    int motorCanID = motors[motorID].GetSlaveId();
    

    std::string filename_hip_pitch = std::to_string(date)+std::string("left_knee_") + std::to_string(testNum) + ".txt";
      

    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position(motors[motorID]);

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹产生
    std::vector<double> trajectory_knee,velocity_knee;

    // 五次多项式轨迹
    // trajectoryGenerator(trajectory_knee, velocity_knee, targetpos);  // 测试轨迹无问题927
    multiTrajGenerator_kneeVersion(trajectory_knee, targetpos, 5);
    // writeVectorToTxt(trajectory_knee, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj927_2.txt");  //测试轨迹无问题927

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_knee;


    mc_knee.pos_desired = trajectory_knee;
    mc_knee.velocity_limit = 200;
    mc_knee.posKp = 50;
    mc_knee.posKd = 0;
    mc_knee.velKp = 20;
    mc_knee.velKd = 0;
    mc_knee.velKi = 0;

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    for (size_t i = 0; i < trajectory_knee.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_knee.pos_desired[i], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }

    for (size_t i = 0; i < 500; i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_knee.pos_desired[trajectory_knee.size()-1], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }
     
    mc.Motorevo_motor_servo_mode(motors[motorID], 0, mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
    
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

void test_knee(){
    left_leg_knee_ctrl_multiTraj(4, 40, 927);
}

/**
 * @param testNum
 * @param targetPitch
 * @param targetRoll
 * @param date 日期
 * @version 当前状态为：五次多项式+已编译 已经改进的内容包括：可记录发送指令时间、运用更高精度的时间库计时、将通信时间纳入考量中
*/
int left_leg_ankle_ctrl(int testNum, int targetposPitch, int targetposRoll, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID[2] = {10,11};
    
    // // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    

    std::string filename_ankle_high = std::to_string(date) + std::string("left_ankle_high_") + std::to_string(testNum) + ".txt";
    std::string filename_ankle_low = std::to_string(date) + std::string("left_ankle_low_") + std::to_string(testNum) + ".txt";


    mc.enable(motors[motorID[0]]);
    mc.enable(motors[motorID[1]]);



    for (size_t i = 0; i < 2; i++)
    {
        mc.addMotor(&motors[motorID[i]]);
        mc.Motorevo_set_zero_position(motors[motorID[i]]);
    }
    
    
    

    

    // 轨迹产生
    std::vector<double> trajectory_ankle_H,velocity_ankle_H,
                        trajectory_ankle_L,velocity_ankle_L;

    // 五次多项式轨迹
    double ankleMotorH = ankleInvKinHighTheta1(targetposPitch, targetposRoll);
    double ankleMotorL = ankleInvKinLowTheta2(targetposPitch, targetposRoll);
    trajectoryGenerator(trajectory_ankle_H, velocity_ankle_H, ankleMotorH);
    trajectoryGenerator(trajectory_ankle_L, velocity_ankle_L, ankleMotorL);

    // ZMP轨迹：关节轨迹转换为电机轨迹
    // std::vector<double> trajectory_ankle_motor_high, trajectory_ankle_motor_low;
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_ankle_H.push_back(ankleInvKinHighTheta1(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    //     trajectory_ankle_L.push_back(ankleInvKinLowTheta2(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    // }
    
    // for (size_t i = 0; i < trajectory_ankle_H.size(); i++)
    // {
    //     std::cout << trajectory_ankle_H[i] << " ";
    // }
    // std::cout << "\n" << "---------------" << std::endl;
    // for (size_t i = 0; i < trajectory_ankle_L.size(); i++)
    // {
    //     std::cout << trajectory_ankle_L[i] << " ";
    // }
    // std::cout << "\n" << "---------------" << std::endl;

    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_ankle_high, mc_ankle_low;

    mc_ankle_high.pos_desired = trajectory_ankle_H;
    mc_ankle_high.velocity_limit = 200;
    mc_ankle_high.posKp = 50;
    mc_ankle_high.posKd = 10;
    mc_ankle_high.velKp = 50;
    mc_ankle_high.velKd = 0;
    mc_ankle_high.velKi = 0.001;

    mc_ankle_low.pos_desired = trajectory_ankle_L;
    mc_ankle_low.velocity_limit = 200;
    mc_ankle_low.posKp = 50;
    mc_ankle_low.posKd = 10;
    mc_ankle_low.velKp = 50;
    mc_ankle_low.velKd = 0;
    mc_ankle_low.velKi = 0.001;

    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    auto end = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto sendingMoment = end;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - sendingMoment).count();  //相对于基准的指令时刻
    auto recordComTimeStart = end; //记录通信时间
    auto recordComTimeEnd = end; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(end - sendingMoment).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();  //相对于基准的通行结束时间
    for (size_t i = 0; i < trajectory_ankle_H.size(); i++)
    {

        // ankle high
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingMoment = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingMoment - start).count();
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTimewithSendingTime(motorID[0], duration, motors[motorID[0]].Get_Position(), 
                                    motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_ankle_high, 0, sendingTime); // 保存数据到文件中
        std::cout << "motors[motorID[0]]." << motors[motorID[0]].GetSlaveId()<<" " <<motors[motorID[0]].Get_Position() << " " << motors[motorID[0]].Get_Velocity() << "\n";

        // ankle low
        mc.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingMoment = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingMoment - start).count();
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTimewithSendingTime(motorID[1], duration, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                    motors[motorID[1]].Get_tau(), &filename_ankle_low, 1, sendingTime); // 保存数据到文件中
        std::cout << "motors[motorID[1]]." << motors[motorID[1]].GetSlaveId()<<" " <<motors[motorID[1]].Get_Position() << " " << motors[motorID[1]].Get_Velocity() << "\n";
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::milliseconds>(recordComTimeEnd - recordComTimeStart).count();
        // 休眠/延迟控制周期剩余的时间，即控制周期为1000us，通信时间为recordComTime，为了保证控制周期基本恒定，延迟时间为1000-recordComTime
        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(1000-recordComTime))); //更精确的延时us
        // usleep(1000);
    }
    mc.Motorevo_motor_servo_mode(motors[4], 0, mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
    mc.Motorevo_motor_servo_mode(motors[5], 0, mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);

     
        
    
    for (size_t i = 0; i < 2; i++)
    {
        mc.disable(motors[motorID[i]]);
    }

    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}


int test05(){
    left_leg_ankle_ctrl(1,10,0,919);
}

/**
 * @param testNum
 * @param targetPitch
 * @param targetRoll
 * @param date 日期
 * @version 当前状态为：五次多项式+已编译 已经改进的内容包括：可记录发送指令时间、运用更高精度的时间库计时、将通信时间纳入考量中
*/
int left_leg_ankle_ctrl_multiTraj(int testNum, int targetposPitch, int targetposRoll, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID[2] = {10,11};
    
    // // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    

    std::string filename_ankle_high = std::to_string(date) + std::string("left_ankle_high_") + std::to_string(testNum) + ".txt";
    std::string filename_ankle_low = std::to_string(date) + std::string("left_ankle_low_") + std::to_string(testNum) + ".txt";


    mc.disable(motors[motorID[0]]);
    mc.disable(motors[motorID[1]]);

    mc.enable(motors[motorID[0]]);
    mc.enable(motors[motorID[1]]);



    for (size_t i = 0; i < 2; i++)
    {
        mc.addMotor(&motors[motorID[i]]);
        mc.Motorevo_set_zero_position(motors[motorID[i]]);
    }
    
    
    

    

    // 轨迹产生
    std::vector<double> trajectory_ankle_H,velocity_ankle_H,
                        trajectory_ankle_L,velocity_ankle_L;

    // 五次多项式轨迹 多段
    double ankleMotorH = ankleInvKinHighTheta1(targetposPitch, targetposRoll);
    double ankleMotorL = ankleInvKinLowTheta2(targetposPitch, targetposRoll);
    multiTrajGenerator(trajectory_ankle_H, ankleMotorH, 5);
    multiTrajGenerator(trajectory_ankle_L, ankleMotorL, 5);


    // ZMP轨迹：关节轨迹转换为电机轨迹
    // std::vector<double> trajectory_ankle_motor_high, trajectory_ankle_motor_low;
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_ankle_H.push_back(ankleInvKinHighTheta1(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    //     trajectory_ankle_L.push_back(ankleInvKinLowTheta2(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    // }
    
    // for (size_t i = 0; i < trajectory_ankle_H.size(); i++)
    // {
    //     std::cout << trajectory_ankle_H[i] << " ";
    // }
    // std::cout << "\n" << "---------------" << std::endl;
    // for (size_t i = 0; i < trajectory_ankle_L.size(); i++)
    // {
    //     std::cout << trajectory_ankle_L[i] << " ";
    // }
    // std::cout << "\n" << "---------------" << std::endl;

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_ankle_high, mc_ankle_low;

    mc_ankle_high.pos_desired = trajectory_ankle_H;
    mc_ankle_high.velocity_limit = 200;
    mc_ankle_high.posKp = 60;
    mc_ankle_high.posKd = 0;
    mc_ankle_high.velKp = 40;
    mc_ankle_high.velKd = 0;
    mc_ankle_high.velKi = 0;

    mc_ankle_low.pos_desired = trajectory_ankle_L;
    mc_ankle_low.velocity_limit = 200;
    mc_ankle_low.posKp = 60;
    mc_ankle_low.posKd = 0;
    mc_ankle_low.velKp = 40;
    mc_ankle_low.velKd = 0;
    mc_ankle_low.velKi = 0;

    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    auto end = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto sendingMoment = end;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - sendingMoment).count();  //相对于基准的指令时刻
    auto recordComTimeStart = end; //记录通信时间
    auto recordComTimeEnd = end; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(end - sendingMoment).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();  //相对于基准的通行结束时间
    for (size_t i = 0; i < trajectory_ankle_H.size(); i++)
    {

        // ankle high
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID[0]], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingMoment = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingMoment - start).count();
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTimewithSendingTime(motorID[0], duration, motors[motorID[0]].Get_Position(), 
                                    motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_ankle_high, 0, sendingTime); // 保存数据到文件中
        // std::cout << "motors[motorID[0]]." << motors[motorID[0]].GetSlaveId()<<" " <<motors[motorID[0]].Get_Position() << " " << motors[motorID[0]].Get_Velocity() << "\n";

        // ankle low
        mc.Motorevo_motor_servo_mode(motors[motorID[1]], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingMoment = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingMoment - start).count();
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTimewithSendingTime(motorID[1], duration, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                    motors[motorID[1]].Get_tau(), &filename_ankle_low, 1, sendingTime); // 保存数据到文件中
        // std::cout << "motors[motorID[1]]." << motors[motorID[1]].GetSlaveId()<<" " <<motors[motorID[1]].Get_Position() << " " << motors[motorID[1]].Get_Velocity() << "\n";
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::milliseconds>(recordComTimeEnd - recordComTimeStart).count();
        // 休眠/延迟控制周期剩余的时间，即控制周期为1000us，通信时间为recordComTime，为了保证控制周期基本恒定，延迟时间为1000-recordComTime
        if (recordComTime>=1000)
        {
            std::cout << "recordComTime is: " << recordComTime << "\n"
                << "beyond control cycle" << std::endl;
        }else{
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(1000-recordComTime))); //更精确的延时us

        }
        
        // usleep(1000);
    }
    // 记录系统稳定后的状态 500ms
    for (size_t i = 0; i < 500; i++)
    {
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID[0]], mc_ankle_high.pos_desired[trajectory_ankle_H.size()-1], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingMoment = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingMoment - start).count();
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTimewithSendingTime(motorID[0], duration, motors[motorID[0]].Get_Position(), 
                                    motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_ankle_high, 0, sendingTime); // 保存数据到文件中
        // std::cout << "motors[motorID[0]]." << motors[motorID[0]].GetSlaveId()<<" " <<motors[motorID[0]].Get_Position() << " " << motors[motorID[0]].Get_Velocity() << "\n";

        // ankle low
        mc.Motorevo_motor_servo_mode(motors[motorID[1]], mc_ankle_low.pos_desired[trajectory_ankle_H.size()-1], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingMoment = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingMoment - start).count();
        mc.receive_evo();
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        dataWriteTimewithSendingTime(motorID[1], duration, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                    motors[motorID[1]].Get_tau(), &filename_ankle_low, 1, sendingTime); // 保存数据到文件中
        // std::cout << "motors[motorID[1]]." << motors[motorID[1]].GetSlaveId()<<" " <<motors[motorID[1]].Get_Position() << " " << motors[motorID[1]].Get_Velocity() << "\n";
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::milliseconds>(recordComTimeEnd - recordComTimeStart).count();
        // 休眠/延迟控制周期剩余的时间，即控制周期为1000us，通信时间为recordComTime，为了保证控制周期基本恒定，延迟时间为1000-recordComTime
        if (recordComTime>=1000)
        {
            std::cout << "recordComTime is: " << recordComTime << "\n"
                << "beyond control cycle" << std::endl;
        }else{
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(1000-recordComTime))); //更精确的延时us

        }
    }
    
    mc.Motorevo_motor_servo_mode(motors[motorID[0]], 0, mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
    mc.Motorevo_motor_servo_mode(motors[motorID[1]], 0, mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);

     
        
    
    for (size_t i = 0; i < 2; i++)
    {
        mc.disable(motors[motorID[i]]);
    }

    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

int test06(){
    left_leg_ankle_ctrl_multiTraj(1, 20, 0, 1010);
    return 0;
}

// 左腿除膝关节外的多关节轨迹控制 
int testFinal(int testNum, int date, double* amplitude_joints){
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    motor_head::Motor_Control dm(serial);
    std::this_thread::sleep_for(std::chrono::milliseconds(380));
    // 控制的i个电机的canID 
    int motorID[6] = {0, 1, 2, 4, 5};

    int64_t T = 4000; // 控制周期 4000us

    std::string filename_hip_pitch = std::to_string(date)+"_left_hip_pitch_"+ std::to_string(testNum) + ".txt";
    std::string filename_hip_roll = std::to_string(date)+"_left_hip_roll_" + std::to_string(testNum) + ".txt";
    std::string filename_hip_yaw = std::to_string(date)+"_left_hip_yaw_" + std::to_string(testNum) + ".txt";
    std::string filename_ankle_high = std::to_string(date)+"_left_ankle_high_" + std::to_string(testNum) + ".txt";  
    std::string filename_ankle_low = std::to_string(date)+"_left_ankle_low_" + std::to_string(testNum) + ".txt";

    // ZMP
    // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");

    dm.disable(motors[0]);
    dm.disable(motors[1]);
    dm.disable(motors[2]);
    dm.disable(motors[4]);
    dm.disable(motors[5]);


    dm.enable(motors[0]);
    dm.enable(motors[1]);
    dm.enable(motors[2]);
    dm.enable(motors[4]);
    dm.enable(motors[5]);


    for (size_t i = 0; i < 5; i++)
    {
        dm.addMotor(&motors[motorID[i]]);
        dm.Motorevo_set_zero_position(motors[motorID[i]]);
    }

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch,
                        trajectory_hip_roll,velocity_hip_roll,
                        trajectory_hip_yaw,velocity_hip_yaw,
                        trajectory_ankle_H,velocity_ankle_H,
                        trajectory_ankle_L,velocity_ankle_L;

    // ZMP
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    //     trajectory_hip_roll.push_back(rad2degree(trajectory[i].joint_angles[1]));
    //     trajectory_hip_yaw.push_back(rad2degree(trajectory[i].joint_angles[2]));
    //     trajectory_ankle_H.push_back(ankleInvKinHighTheta1(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    //     trajectory_ankle_L.push_back(ankleInvKinLowTheta2(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    // }

    double ankleMotorH = ankleInvKinHighTheta1(amplitude_joints[3], amplitude_joints[4]);
    double ankleMotorL = ankleInvKinLowTheta2(amplitude_joints[3], amplitude_joints[4]);
    trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, amplitude_joints[0]);
    trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, amplitude_joints[0]);
    trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, amplitude_joints[0]);
    trajectoryGenerator(trajectory_ankle_H, velocity_ankle_H, ankleMotorH);
    trajectoryGenerator(trajectory_ankle_L, velocity_ankle_L, ankleMotorL);




    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch, mc_hip_roll, mc_hip_yaw, mc_ankle_high, mc_ankle_low;

    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 90;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 50;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0.01;

    mc_hip_roll.pos_desired = trajectory_hip_roll;
    mc_hip_roll.velocity_limit = 90;
    mc_hip_roll.posKp = 50;
    mc_hip_roll.posKd = 0;
    mc_hip_roll.velKp = 50;
    mc_hip_roll.velKd = 0;
    mc_hip_roll.velKi = 0.01;

    mc_hip_yaw.pos_desired = trajectory_hip_yaw;
    mc_hip_yaw.velocity_limit = 90;
    mc_hip_yaw.posKp = 50;
    mc_hip_yaw.posKd = 0;
    mc_hip_yaw.velKp = 50;
    mc_hip_yaw.velKd = 0;
    mc_hip_yaw.velKi = 0.01;

    mc_ankle_high.pos_desired = trajectory_ankle_H;
    mc_ankle_high.velocity_limit = 90;
    mc_ankle_high.posKp = 50;
    mc_ankle_high.posKd = 0;
    mc_ankle_high.velKp = 50;
    mc_ankle_high.velKd = 0;
    mc_ankle_high.velKi = 0.01;

    mc_ankle_low.pos_desired = trajectory_ankle_L;
    mc_ankle_low.velocity_limit = 90;
    mc_ankle_low.posKp = 50;
    mc_ankle_low.posKd = 0;
    mc_ankle_low.velKp = 50;
    mc_ankle_low.velKd = 0;
    mc_ankle_low.velKi = 0.01;

    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "initial time is: " << duration;
    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    start = std::chrono::high_resolution_clock::now(); //基准时间
    
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        dm.Motorevo_motor_servo_mode(motors[0], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();

        dataWriteTimewithSendingTime(motorID[0], receiveTime, motors[motorID[0]].Get_Position(), 
                        motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(200)); //更精确的延时us
        
        // hip roll
        dm.Motorevo_motor_servo_mode(motors[1], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[1], receiveTime, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                motors[motorID[1]].Get_tau(), &filename_hip_roll, 1, sendingTime); // 保存数据到文件中
        
        std::this_thread::sleep_for(std::chrono::microseconds(200)); //更精确的延时us



        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[2], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[2], receiveTime, motors[motorID[2]].Get_Position(), motors[motorID[2]].Get_Velocity(), 
                                    motors[motorID[2]].Get_tau(), &filename_hip_yaw, 2, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(200)); //更精确的延时us



        // ankle high
        dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[3], receiveTime, motors[motorID[3]].Get_Position(), motors[motorID[3]].Get_Velocity(), 
                                    motors[motorID[3]].Get_tau(), &filename_ankle_high, 4, sendingTime); // 保存数据到文件中
        std::this_thread::sleep_for(std::chrono::microseconds(200)); //更精确的延时us
        
        
        // ankle low
        dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[4], receiveTime, motors[motorID[4]].Get_Position(), motors[motorID[4]].Get_Velocity(), 
                        motors[motorID[4]].Get_tau(), &filename_ankle_low, 5, sendingTime); // 保存数据到文件中
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = T - 200*4 - recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        
    }
     
        
    
    for (size_t i = 0; i < 5; i++)
    {
        dm.disable(motors[motorID[i]]);
    }
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}


/**
 * @brief 整条单腿的多电机多段轨迹控制, 膝关节已经换为髋关节相同电机
 * @param amplitude_joint 为数组，其维度为6,对应六个电机的轨迹的最大值
*/
int final_ctrl_multi(int testNum, int date, double* amplitude_joints){
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    motor_head::Motor_Control dm(serial);

    // 控制的i个电机的canID 
    int motorID[6] = {0, 1, 2, 3, 4, 5};

    // int64_t T = 2000; // 控制周期 2000us 500Hz
    int64_t T = 10000; // 控制周期 3000us 333Hz


    std::string filename_hip_pitch = std::to_string(date)+"_left_hip_pitch_"+ std::to_string(testNum) + ".txt";
    std::string filename_hip_roll = std::to_string(date)+"_left_hip_roll_" + std::to_string(testNum) + ".txt";
    std::string filename_hip_yaw = std::to_string(date)+"_left_hip_yaw_" + std::to_string(testNum) + ".txt";
    std::string filename_knee = std::to_string(date)+"_left_knee_" + std::to_string(testNum) + ".txt";
    std::string filename_ankle_high = std::to_string(date)+"_left_ankle_high_" + std::to_string(testNum) + ".txt";  
    std::string filename_ankle_low = std::to_string(date)+"_left_ankle_low_" + std::to_string(testNum) + ".txt";

    // ZMP
    // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");

    dm.disable(motors[0]);
    dm.disable(motors[1]);
    dm.disable(motors[2]);
    dm.disable(motors[3]);
    dm.disable(motors[4]);
    dm.disable(motors[5]);


    dm.enable(motors[0]);
    dm.enable(motors[1]);
    dm.enable(motors[2]);
    dm.enable(motors[3]);
    dm.enable(motors[4]);
    dm.enable(motors[5]);


    for (size_t i = 0; i < 6; i++)
    {
        dm.addMotor(&motors[motorID[i]]);
        dm.Motorevo_set_zero_position(motors[motorID[i]]);
    }

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch,
                        trajectory_hip_roll,velocity_hip_roll,
                        trajectory_hip_yaw,velocity_hip_yaw,
                        trajectory_knee,velocity_knee,
                        trajectory_ankle_H,velocity_ankle_H,
                        trajectory_ankle_L,velocity_ankle_L;

    // ZMP
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    //     trajectory_hip_roll.push_back(rad2degree(trajectory[i].joint_angles[1]));
    //     trajectory_hip_yaw.push_back(rad2degree(trajectory[i].joint_angles[2]));
    //     trajectory_ankle_H.push_back(ankleInvKinHighTheta1(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    //     trajectory_ankle_L.push_back(ankleInvKinLowTheta2(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    // }

    double ankleMotorH = ankleInvKinHighTheta1(amplitude_joints[4], amplitude_joints[5]);
    double ankleMotorL = ankleInvKinLowTheta2(amplitude_joints[4], amplitude_joints[5]);
    multiTrajGenerator(trajectory_hip_pitch, amplitude_joints[0], 5);
    multiTrajGenerator(trajectory_hip_roll, amplitude_joints[1], 5);
    multiTrajGenerator(trajectory_hip_yaw, amplitude_joints[2], 5);
    multiTrajGenerator_kneeVersion(trajectory_knee, amplitude_joints[3], 5);
    multiTrajGenerator(trajectory_ankle_H, ankleMotorH, 5);
    multiTrajGenerator(trajectory_ankle_L, ankleMotorL, 5);

    // writeVectorToTxt(trajectory_hip_pitch, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_1.txt");
    // writeVectorToTxt(trajectory_hip_roll, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_2.txt");
    // writeVectorToTxt(trajectory_hip_yaw, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_3.txt");
    // writeVectorToTxt(trajectory_knee, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_4.txt");
    // writeVectorToTxt(trajectory_ankle_H, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_5.txt");
    // writeVectorToTxt(trajectory_ankle_L, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_6.txt");


    // trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, amplitude_joints[0]);
    // trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, amplitude_joints[0]);
    // trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, amplitude_joints[0]);
    // trajectoryGenerator(trajectory_ankle_H, velocity_ankle_H, ankleMotorH);
    // trajectoryGenerator(trajectory_ankle_L, velocity_ankle_L, ankleMotorL);

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);


    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch, mc_hip_roll, mc_hip_yaw, mc_knee, mc_ankle_high, mc_ankle_low;

    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 90;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 20;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0;

    mc_hip_roll.pos_desired = trajectory_hip_roll;
    mc_hip_roll.velocity_limit = 90;
    mc_hip_roll.posKp = 50;
    mc_hip_roll.posKd = 0;
    mc_hip_roll.velKp = 20;
    mc_hip_roll.velKd = 0;
    mc_hip_roll.velKi = 0;

    mc_hip_yaw.pos_desired = trajectory_hip_yaw;
    mc_hip_yaw.velocity_limit = 90;
    mc_hip_yaw.posKp = 50;
    mc_hip_yaw.posKd = 0;
    mc_hip_yaw.velKp = 20;
    mc_hip_yaw.velKd = 0;
    mc_hip_yaw.velKi = 0;

    mc_knee.pos_desired = trajectory_knee;
    mc_knee.velocity_limit = 90;
    mc_knee.posKp = 50;
    mc_knee.posKd = 0;
    mc_knee.velKp = 20;
    mc_knee.velKd = 0;
    mc_knee.velKi = 0;

    mc_ankle_high.pos_desired = trajectory_ankle_H;
    mc_ankle_high.velocity_limit = 90;
    mc_ankle_high.posKp = 60;
    mc_ankle_high.posKd = 0;
    mc_ankle_high.velKp = 40;
    mc_ankle_high.velKd = 0;
    mc_ankle_high.velKi = 0;

    mc_ankle_low.pos_desired = trajectory_ankle_L;
    mc_ankle_low.velocity_limit = 90;
    mc_ankle_low.posKp = 60;
    mc_ankle_low.posKd = 0;
    mc_ankle_low.velKp = 40;
    mc_ankle_low.velKd = 0;
    mc_ankle_low.velKi = 0;

    
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // std::cout << "initial time is: " << duration;

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    start = std::chrono::high_resolution_clock::now(); //基准时间
    
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i = i+10)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        dm.Motorevo_motor_servo_mode(motors[0], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();

        dataWriteTimewithSendingTime(motorID[0], receiveTime, motors[motorID[0]].Get_Position(), 
                        motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us  帧间隔
        
        // hip roll
        // dm.Motorevo_motor_servo_mode(motors[1], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        dm.Motorevo_motor_servo_mode(motors[1], 0, mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[1], receiveTime, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                motors[motorID[1]].Get_tau(), &filename_hip_roll, 1, sendingTime); // 保存数据到文件中
        
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔



        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[2], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[2], receiveTime, motors[motorID[2]].Get_Position(), motors[motorID[2]].Get_Velocity(), 
                                    motors[motorID[2]].Get_tau(), &filename_hip_yaw, 2, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // knee
        dm.Motorevo_motor_servo_mode(motors[3], mc_knee.pos_desired[i], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[3], receiveTime, motors[motorID[3]].Get_Position(), motors[motorID[3]].Get_Velocity(), 
                                    motors[motorID[3]].Get_tau(), &filename_knee, 3, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // ankle high
        dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[4], receiveTime, motors[motorID[4]].Get_Position(), motors[motorID[4]].Get_Velocity(), 
                                    motors[motorID[4]].Get_tau(), &filename_ankle_high, 4, sendingTime); // 保存数据到文件中
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us
        
        
        // ankle low
        dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[5], receiveTime, motors[motorID[5]].Get_Position(), motors[motorID[5]].Get_Velocity(), 
                        motors[motorID[5]].Get_tau(), &filename_ankle_low, 5, sendingTime); // 保存数据到文件中
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        // remainingTime = T - 100*5 - recordComTime;  //recordComTime 以及包含了那几个100us的延时，重复减去了这个延时
        remainingTime = T - recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "recordComTime is:" << recordComTime << " ";
            std::cout << "通信时间超出控制周期" << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔
        }
        
    }

    // 系统稳定时间 0.5s
    for (size_t i = 0; i < 250; i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        dm.Motorevo_motor_servo_mode(motors[0], mc_hip_pitch.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();

        dataWriteTimewithSendingTime(motorID[0], receiveTime, motors[motorID[0]].Get_Position(), 
                        motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us  帧间隔
        
        // hip roll
        dm.Motorevo_motor_servo_mode(motors[1], mc_hip_roll.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[1], receiveTime, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                motors[motorID[1]].Get_tau(), &filename_hip_roll, 1, sendingTime); // 保存数据到文件中
        
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔



        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[2], mc_hip_yaw.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[2], receiveTime, motors[motorID[2]].Get_Position(), motors[motorID[2]].Get_Velocity(), 
                                    motors[motorID[2]].Get_tau(), &filename_hip_yaw, 2, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // knee
        dm.Motorevo_motor_servo_mode(motors[3], mc_knee.pos_desired[trajectory_hip_pitch.size()-1], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[3], receiveTime, motors[motorID[3]].Get_Position(), motors[motorID[3]].Get_Velocity(), 
                                    motors[motorID[3]].Get_tau(), &filename_knee, 3, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // ankle high
        dm.Motorevo_motor_servo_mode(motors[4], mc_ankle_high.pos_desired[trajectory_hip_pitch.size()-1], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[4], receiveTime, motors[motorID[4]].Get_Position(), motors[motorID[4]].Get_Velocity(), 
                                    motors[motorID[4]].Get_tau(), &filename_ankle_high, 4, sendingTime); // 保存数据到文件中
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us
        
        
        // ankle low
        dm.Motorevo_motor_servo_mode(motors[5], mc_ankle_low.pos_desired[trajectory_hip_pitch.size()-1], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[5], receiveTime, motors[motorID[5]].Get_Position(), motors[motorID[5]].Get_Velocity(), 
                        motors[motorID[5]].Get_tau(), &filename_ankle_low, 5, sendingTime); // 保存数据到文件中
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        // remainingTime = T - 100*5 - recordComTime;  //recordComTime 以及包含了那几个100us的延时，重复减去了这个延时
        remainingTime = T - recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "recordComTime is:" << recordComTime << " ";
            std::cout << "通信时间超出控制周期" << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔

        }
        
    }
     
        
    
    
    for (size_t i = 0; i < 6; i++)
    {
        dm.disable(motors[motorID[i]]);
        file[i].close();
    }
    
    
    return 0; 
}

void testFinal_multi(){
    double amplitude[6];
    amplitude[0] = 20;
    amplitude[1] = 10;
    amplitude[2] = 5;
    amplitude[3] = 40;
    amplitude[4] = 20;
    amplitude[5] = 5;

    final_ctrl_multi(8, 1029, amplitude);
}


/**
 * @brief 整条单腿的多电机多段轨迹控制, 膝关节已经换为髋关节相同电机---右腿
 * @param amplitude_joint 为数组，其维度为6,对应六个电机的轨迹的最大值
*/
int final_ctrl_multi_right(int testNum, int date, double* amplitude_joints){
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start;
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control dm(serial);

    // 控制的i个电机的canID 
    int motorID[6] = {6, 7, 8, 9, 10, 11};

    // int64_t T = 2000; // 控制周期 2000us 500Hz
    int64_t T = 10000; // 控制周期 3000us 333Hz


    std::string filename_hip_pitch = std::to_string(date)+"_right_hip_pitch_"+ std::to_string(testNum) + ".txt";
    std::string filename_hip_roll = std::to_string(date)+"_right_hip_roll_" + std::to_string(testNum) + ".txt";
    std::string filename_hip_yaw = std::to_string(date)+"_right_hip_yaw_" + std::to_string(testNum) + ".txt";
    std::string filename_knee = std::to_string(date)+"_right_knee_" + std::to_string(testNum) + ".txt";
    std::string filename_ankle_high = std::to_string(date)+"_right_ankle_high_" + std::to_string(testNum) + ".txt";  
    std::string filename_ankle_low = std::to_string(date)+"_right_ankle_low_" + std::to_string(testNum) + ".txt";

    // ZMP
    // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");

    for (size_t i = 0; i < 6; i++)
    {
        dm.disable(motors[motorID[i]]);
    }

    for (size_t i = 0; i < 6; i++)
    {
        dm.enable(motors[motorID[i]]);
    }
    

    for (size_t i = 0; i < 6; i++)
    {
        dm.addMotor(&motors[motorID[i]]);
        dm.Motorevo_set_zero_position(motors[motorID[i]]);
    }

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch,
                        trajectory_hip_roll,velocity_hip_roll,
                        trajectory_hip_yaw,velocity_hip_yaw,
                        trajectory_knee,velocity_knee,
                        trajectory_ankle_H,velocity_ankle_H,
                        trajectory_ankle_L,velocity_ankle_L;

    // ZMP
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    //     trajectory_hip_roll.push_back(rad2degree(trajectory[i].joint_angles[1]));
    //     trajectory_hip_yaw.push_back(rad2degree(trajectory[i].joint_angles[2]));
    //     trajectory_ankle_H.push_back(ankleInvKinHighTheta1(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    //     trajectory_ankle_L.push_back(ankleInvKinLowTheta2(rad2degree(trajectory[i].joint_angles[4]), rad2degree(trajectory[i].joint_angles[5])));
    // }


    // Right Leg InvKin
    double ankleMotorH = ankleInvKinHighTheta1_R(amplitude_joints[4], amplitude_joints[5]);
    double ankleMotorL = ankleInvKinLowTheta2_R(amplitude_joints[4], amplitude_joints[5]);
    multiTrajGenerator(trajectory_hip_pitch, amplitude_joints[0], 5);
    multiTrajGenerator(trajectory_hip_roll, amplitude_joints[1], 5);
    multiTrajGenerator(trajectory_hip_yaw, amplitude_joints[2], 5);
    multiTrajGenerator_kneeVersion(trajectory_knee, amplitude_joints[3], 5);
    multiTrajGenerator(trajectory_ankle_H, ankleMotorH, 5);
    multiTrajGenerator(trajectory_ankle_L, ankleMotorL, 5);

    // writeVectorToTxt(trajectory_hip_pitch, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_1.txt");
    // writeVectorToTxt(trajectory_hip_roll, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_2.txt");
    // writeVectorToTxt(trajectory_hip_yaw, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_3.txt");
    // writeVectorToTxt(trajectory_knee, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_4.txt");
    // writeVectorToTxt(trajectory_ankle_H, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_5.txt");
    // writeVectorToTxt(trajectory_ankle_L, "/home/ye/00_jfl_26.10/MotorCtrl/data/testTraj928_6.txt");


    // trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, amplitude_joints[0]);
    // trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, amplitude_joints[0]);
    // trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, amplitude_joints[0]);
    // trajectoryGenerator(trajectory_ankle_H, velocity_ankle_H, ankleMotorH);
    // trajectoryGenerator(trajectory_ankle_L, velocity_ankle_L, ankleMotorL);

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);


    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch, mc_hip_roll, mc_hip_yaw, mc_knee, mc_ankle_high, mc_ankle_low;

    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 90;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 20;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0;

    mc_hip_roll.pos_desired = trajectory_hip_roll;
    mc_hip_roll.velocity_limit = 90;
    mc_hip_roll.posKp = 50;
    mc_hip_roll.posKd = 0;
    mc_hip_roll.velKp = 20;
    mc_hip_roll.velKd = 0;
    mc_hip_roll.velKi = 0;

    mc_hip_yaw.pos_desired = trajectory_hip_yaw;
    mc_hip_yaw.velocity_limit = 90;
    mc_hip_yaw.posKp = 50;
    mc_hip_yaw.posKd = 0;
    mc_hip_yaw.velKp = 20;
    mc_hip_yaw.velKd = 0;
    mc_hip_yaw.velKi = 0;

    mc_knee.pos_desired = trajectory_knee;
    mc_knee.velocity_limit = 90;
    mc_knee.posKp = 50;
    mc_knee.posKd = 0;
    mc_knee.velKp = 20;
    mc_knee.velKd = 0;
    mc_knee.velKi = 0;

    mc_ankle_high.pos_desired = trajectory_ankle_H;
    mc_ankle_high.velocity_limit = 90;
    mc_ankle_high.posKp = 60;
    mc_ankle_high.posKd = 0;
    mc_ankle_high.velKp = 40;
    mc_ankle_high.velKd = 0;
    mc_ankle_high.velKi = 0;

    mc_ankle_low.pos_desired = trajectory_ankle_L;
    mc_ankle_low.velocity_limit = 90;
    mc_ankle_low.posKp = 60;
    mc_ankle_low.posKd = 0;
    mc_ankle_low.velKp = 40;
    mc_ankle_low.velKd = 0;
    mc_ankle_low.velKi = 0;

    
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // std::cout << "initial time is: " << duration;

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    start = std::chrono::high_resolution_clock::now(); //基准时间
    
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i = i+10)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        dm.Motorevo_motor_servo_mode(motors[motorID[0]], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();

        dataWriteTimewithSendingTime(motorID[0], receiveTime, motors[motorID[0]].Get_Position(), 
                        motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us  帧间隔
        
        // hip roll
        dm.Motorevo_motor_servo_mode(motors[motorID[1]], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[1], receiveTime, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                motors[motorID[1]].Get_tau(), &filename_hip_roll, 1, sendingTime); // 保存数据到文件中
        
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔



        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[motorID[2]], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[2], receiveTime, motors[motorID[2]].Get_Position(), motors[motorID[2]].Get_Velocity(), 
                                    motors[motorID[2]].Get_tau(), &filename_hip_yaw, 2, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // knee
        dm.Motorevo_motor_servo_mode(motors[motorID[3]], mc_knee.pos_desired[i], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[3], receiveTime, motors[motorID[3]].Get_Position(), motors[motorID[3]].Get_Velocity(), 
                                    motors[motorID[3]].Get_tau(), &filename_knee, 3, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // ankle high
        dm.Motorevo_motor_servo_mode(motors[motorID[4]], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[4], receiveTime, motors[motorID[4]].Get_Position(), motors[motorID[4]].Get_Velocity(), 
                                    motors[motorID[4]].Get_tau(), &filename_ankle_high, 4, sendingTime); // 保存数据到文件中
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us
        
        
        // ankle low
        dm.Motorevo_motor_servo_mode(motors[motorID[5]], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[5], receiveTime, motors[motorID[5]].Get_Position(), motors[motorID[5]].Get_Velocity(), 
                        motors[motorID[5]].Get_tau(), &filename_ankle_low, 5, sendingTime); // 保存数据到文件中
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        // remainingTime = T - 100*5 - recordComTime;  //recordComTime 以及包含了那几个100us的延时，重复减去了这个延时
        remainingTime = T - recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "recordComTime is:" << recordComTime << " ";
            std::cout << "通信时间超出控制周期" << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔
        }
        
    }

    // 系统稳定时间 0.5s
    for (size_t i = 0; i < 250; i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        dm.Motorevo_motor_servo_mode(motors[motorID[0]], mc_hip_pitch.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();

        dataWriteTimewithSendingTime(motorID[0], receiveTime, motors[motorID[0]].Get_Position(), 
                        motors[motorID[0]].Get_Velocity(), motors[motorID[0]].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us  帧间隔
        
        // hip roll
        dm.Motorevo_motor_servo_mode(motors[motorID[1]], mc_hip_roll.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[1], receiveTime, motors[motorID[1]].Get_Position(), motors[motorID[1]].Get_Velocity(), 
                                motors[motorID[1]].Get_tau(), &filename_hip_roll, 1, sendingTime); // 保存数据到文件中
        
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔



        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[motorID[2]], mc_hip_yaw.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[2], receiveTime, motors[motorID[2]].Get_Position(), motors[motorID[2]].Get_Velocity(), 
                                    motors[motorID[2]].Get_tau(), &filename_hip_yaw, 2, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // knee
        dm.Motorevo_motor_servo_mode(motors[motorID[3]], mc_knee.pos_desired[trajectory_hip_pitch.size()-1], mc_knee.posKp, mc_knee.posKd, mc_knee.velocity_limit, mc_knee.velKp, mc_knee.velKd, mc_knee.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[3], receiveTime, motors[motorID[3]].Get_Position(), motors[motorID[3]].Get_Velocity(), 
                                    motors[motorID[3]].Get_tau(), &filename_knee, 3, sendingTime); // 保存数据到文件中

        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us

        // ankle high
        dm.Motorevo_motor_servo_mode(motors[motorID[4]], mc_ankle_high.pos_desired[trajectory_hip_pitch.size()-1], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[4], receiveTime, motors[motorID[4]].Get_Position(), motors[motorID[4]].Get_Velocity(), 
                                    motors[motorID[4]].Get_tau(), &filename_ankle_high, 4, sendingTime); // 保存数据到文件中
        std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us
        
        
        // ankle low
        dm.Motorevo_motor_servo_mode(motors[motorID[5]], mc_ankle_low.pos_desired[trajectory_hip_pitch.size()-1], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        
        dm.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::microseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::microseconds>(receiveEnd - start).count();
        
        dataWriteTimewithSendingTime(motorID[5], receiveTime, motors[motorID[5]].Get_Position(), motors[motorID[5]].Get_Velocity(), 
                        motors[motorID[5]].Get_tau(), &filename_ankle_low, 5, sendingTime); // 保存数据到文件中
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        // remainingTime = T - 100*5 - recordComTime;  //recordComTime 以及包含了那几个100us的延时，重复减去了这个延时
        remainingTime = T - recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "recordComTime is:" << recordComTime << " ";
            std::cout << "通信时间超出控制周期" << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(100)); //更精确的延时us   帧间隔

        }
        
    }
     
        
    
    
    for (size_t i = 0; i < 6; i++)
    {
        dm.disable(motors[motorID[i]]);
        file[i].close();
    }
    
    
    return 0; 
}

void testFinal_multi_Right_Leg(){
    double amplitude[6];
    amplitude[0] = -20;
    amplitude[1] = -10;
    amplitude[2] = 5;
    amplitude[3] = -40;
    amplitude[4] = 20;
    amplitude[5] = 5;

    final_ctrl_multi_right(7, 1029, amplitude);
}

/**
 * @version 多段五次多项式轨迹
*/
int newDou_test_multiTraj(int testNum, int targetpos, int date){
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    motor_head::Motor_Control mc(serial);

    // 控制的i个电机的canID 
    int motorID = 12;
    int motorCanID = motors[motorID].GetSlaveId();
    // int testNum = 1;

    std::string filename_hip_pitch = std::to_string(date)+std::string("left_hip_pitch_") + std::to_string(testNum) + ".txt";
    // std::string filename_hip_pitch_hex =  std::to_string(date)+std::string("left_hip_pitch_Hex") + std::to_string(testNum) + ".txt";


    


    
    mc.addMotor(&motors[motorID]);
    mc.disable(motors[motorID]);
    mc.enable(motors[motorID]);
    mc.Motorevo_set_zero_position_V2(motors[motorID]);

    // debug
    mc.receive_evo();
    std::cout<<"pos: " << motors[motorID].Get_Position() << " vel: " << motors[motorID].Get_Velocity() << std::endl;
    // 等待输入继续执行
    std::cout << "Press Enter to continue ..."<<std::endl;
    std::cin.get();

    // ZMP离线轨迹
    // // 关节索引映射
    // const std::vector<int> joint_indices = {
    //     0, 1, 2, 3, 4, 5,  // 左腿关节
    //     6, 7, 8, 9, 10, 11  // 右腿关节
    // };
    // std::vector<TrajectoryPoint> trajectory = loadTrajectory("/home/ye/00_jfl_26.10/MotorCtrl/src/motor_ctrl/src/trajectory_data_V3.txt");
    

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();

    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch;

    // 五次多项式轨迹
    // trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, targetpos);
    multiTrajGenerator(trajectory_hip_pitch, targetpos, 5);
    // ZMP轨迹
    // for (size_t i = 0; i < trajectory.size(); i++)
    // {
    //     trajectory_hip_pitch.push_back(-rad2degree(trajectory[i].joint_angles[0]));
    // }

    // 轨迹转换为指令 
    // 定义结构体，增强指令可读性
    struct motorcmd
    {
        std::vector<double> pos_desired;
        double velocity_limit;
        double posKp, posKd;
        double velKp, velKd, velKi;
    }mc_hip_pitch;


    mc_hip_pitch.pos_desired = trajectory_hip_pitch;
    mc_hip_pitch.velocity_limit = 200;
    mc_hip_pitch.posKp = 50;
    mc_hip_pitch.posKd = 0;
    mc_hip_pitch.velKp = 20;
    mc_hip_pitch.velKd = 0;
    mc_hip_pitch.velKi = 0;


    // std::cout << "程序 " << "right leg ctrl" << " 启动，等待继续信号...\n";

    // // 打开管道，阻塞等待信号
    // int pipe_fd = open(PIPE_PATH.c_str(), O_RDONLY);
    // if (pipe_fd == -1) {
    //     std::cerr << "管道打开失败！\n";
    //     return 1;
    // }

    // char buffer[10];
    // read(pipe_fd, buffer, sizeof(buffer));  // 阻塞，直到收到信号
    // close(pipe_fd);

    // 收到信号后，继续执行后续逻辑
    // std::cout << "程序 " << "left leg ctrl" << " 开始执行！\n";
    int64_t remainingTime = 0;
    auto receiveEnd = std::chrono::high_resolution_clock::now(); //通信结束时刻
    auto receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - receiveEnd).count();  //相对于基准的通行结束时间
    auto sendingEnd = receiveEnd;  //指令发送时刻
    auto sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - sendingEnd).count();  //相对于基准的指令时刻
    auto recordComTimeStart = receiveEnd; //记录通信开始时间
    auto recordComTimeEnd = receiveEnd; //记录通信结束时间
    auto recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
    auto start = std::chrono::high_resolution_clock::now(); //基准时间
    for (size_t i = 0; i < trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }

    for (size_t i = 0; i < 500; i++)
    {
        // hip pitch
        recordComTimeStart = std::chrono::high_resolution_clock::now();
        mc.Motorevo_motor_servo_mode(motors[motorID], mc_hip_pitch.pos_desired[trajectory_hip_pitch.size()-1], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        sendingEnd = std::chrono::high_resolution_clock::now();
        mc.receive_evo();
        receiveEnd = std::chrono::high_resolution_clock::now();
        sendingTime = std::chrono::duration_cast<std::chrono::milliseconds>(sendingEnd - start).count();
        receiveTime = std::chrono::duration_cast<std::chrono::milliseconds>(receiveEnd - start).count();
        dataWriteTimewithSendingTime(motorID, receiveTime, motors[motorID].Get_Position(), motors[motorID].Get_Velocity(), 
                        motors[motorID].Get_tau(), &filename_hip_pitch, 0, sendingTime); // 保存数据到文件中
        
        recordComTimeEnd = std::chrono::high_resolution_clock::now();
        recordComTime = std::chrono::duration_cast<std::chrono::microseconds>(recordComTimeEnd - recordComTimeStart).count();
        remainingTime = 1000-recordComTime;
        if (remainingTime>0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(remainingTime))); //更精确的延时us

        }
        else
        {
            std::cout << "time of communicate is: " << recordComTime << "\n";
            std::cout << "通信时间超出控制周期" << std::endl;
        }
        

        // usleep(3600);
    }
     
    mc.Motorevo_motor_servo_mode(motors[motorID], 0, mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
    
    mc.disable(motors[motorID]);
    
    for (size_t i = 0; i < 6; i++)
    {
        file[i].close();
    }
    
    
    return 0; 
}

int main(int argc, char const *argv[])
{
    test01();
    // test01_2();
    // test_knee();
    // testFinal_multi();
    // test_new_driver();
    // newDou_test_multiTraj(1, 30, 1024);
    // testFinal_multi_Right_Leg();
    // test_hip_roll_multi();
    // test06();
    // double amplitude[5];
    // amplitude[0] = 10;
    // amplitude[1] = 0;
    // amplitude[2] = 0;
    // amplitude[3] = 10;
    // amplitude[4] = 0;

    // testFinal(1, 919, amplitude);
    return 0;
}
