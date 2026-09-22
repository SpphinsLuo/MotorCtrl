#include <cstdio>
#include <cstdlib>
#include "ros/ros.h"
#include <ros/package.h>
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
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

// 一些全局变量
// 管道文件路径（固定，供所有程序访问）
const std::string PIPE_PATH = "/home/rosnoeticf/aaaWs/motorctrltest/ros_sync_pipe";

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

double degree2rad(double& degree);
double rad2degree(double& rad);
std::string execute_command(const char* cmd);
double ankleInvKinHighTheta1_R(double alphy, double beta);
double ankleInvKinLowTheta2_R(double alphy, double beta);
double quinticPolynomial(double t, double a0, double a1, double a2, double a3, double a4, double a5);
double quinticPolynomialVelocity(double t, double a1, double a2, double a3, double a4, double a5);
void trajectoryGenerator(std::vector<double>& trajectory, std::vector<double>& velocity, double gain_traj = 1);
void dataWrite(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int whichFileObj);

// 右腿除膝关节外的多关节轨迹控制 
int test01(){
    serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    motor_head::Motor_Control dm(serial);

    // 控制的i个电机的canID 
    int motorID[5] = {6, 7, 8, 10, 11};

    int testNum = 1;

    std::string filename_hip_pitch = std::string("test87_hip_pitch_right_") + std::to_string(testNum) + ".txt";
    std::string filename_hip_roll = std::string("test87_hip_roll_right_") + std::to_string(testNum) + ".txt";
    std::string filename_hip_yaw = std::string("test87_hip_yaw_right_") + std::to_string(testNum) + ".txt";
    std::string filename_ankle_high = std::string("test87_ankle_high_right_") + std::to_string(testNum) + ".txt";  
    std::string filename_ankle_low = std::string("test87_ankle_low_right_") + std::to_string(testNum) + ".txt";

    // sleep(2);
    dm.enable(motors[6]);
    // dm.enable(motors[7]);
    // dm.enable(motors[8]);
    dm.enable(motors[10]);
    dm.enable(motors[11]);


    for (size_t i = 0; i < 5; i++)
    {
        dm.addMotor(&motors[motorID[i]]);
        dm.Motorevo_set_zero_position(motors[motorID[i]]);
    }

    // 等待输入继续执行
    // std::cout << "Press Enter to continue ..."<<std::endl;
    // std::cin.get();
    // sleep(2);
    double ankleMotorH = ankleInvKinHighTheta1_R(-30, 0);
    double ankleMotorL = ankleInvKinLowTheta2_R(-30, 0);
    // 轨迹产生
    std::vector<double> trajectory_hip_pitch,velocity_hip_pitch,
                        trajectory_hip_roll,velocity_hip_roll,
                        trajectory_hip_yaw,velocity_hip_yaw,
                        trajectory_ankle_H,velocity_ankle_H,
                        trajectory_ankle_L,velocity_ankle_L;

    trajectoryGenerator(trajectory_hip_pitch, velocity_hip_pitch, 30);
    trajectoryGenerator(trajectory_hip_roll, velocity_hip_roll, 10);
    trajectoryGenerator(trajectory_hip_yaw, velocity_hip_yaw, 10);
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
    mc_hip_yaw.velocity_limit = 200;
    mc_hip_yaw.posKp = 50;
    mc_hip_yaw.posKd = 10;
    mc_hip_yaw.velKp = 50;
    mc_hip_yaw.velKd = 0;
    mc_hip_yaw.velKi = 0.001;

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
    std::cout << "程序 " << "right leg ctrl" << " 开始执行！\n";

    for (size_t i = 0; i <= trajectory_hip_pitch.size(); i++)
    {
        // hip pitch
        dm.Motorevo_motor_servo_mode(motors[6], mc_hip_pitch.pos_desired[i], mc_hip_pitch.posKp, mc_hip_pitch.posKd, mc_hip_pitch.velocity_limit, mc_hip_pitch.velKp, mc_hip_pitch.velKd, mc_hip_pitch.velKi);
        dm.receive_evo();
        dataWrite(motorID[6], motors[6].Get_Position(), motors[6].Get_Velocity(), motors[6].Get_tau(), &filename_hip_pitch, 0); // 保存数据到文件中

        
        // hip roll
        dm.Motorevo_motor_servo_mode(motors[7], mc_hip_roll.pos_desired[i], mc_hip_roll.posKp, mc_hip_roll.posKd, mc_hip_roll.velocity_limit, mc_hip_roll.velKp, mc_hip_roll.velKd, mc_hip_roll.velKi);
        dm.receive_evo();
        dataWrite(motorID[7], motors[7].Get_Position(), motors[7].Get_Velocity(), motors[7].Get_tau(), &filename_hip_roll, 1); // 保存数据到文件中
        usleep(200);

        // hip yaw
        dm.Motorevo_motor_servo_mode(motors[8], mc_hip_yaw.pos_desired[i], mc_hip_yaw.posKp, mc_hip_yaw.posKd, mc_hip_yaw.velocity_limit, mc_hip_yaw.velKp, mc_hip_yaw.velKd, mc_hip_yaw.velKi);
        dm.receive_evo();
        dataWrite(motorID[8], motors[8].Get_Position(), motors[8].Get_Velocity(), motors[8].Get_tau(), &filename_hip_yaw, 2); // 保存数据到文件中

        
        // ankle high
        dm.Motorevo_motor_servo_mode(motors[10], mc_ankle_high.pos_desired[i], mc_ankle_high.posKp, mc_ankle_high.posKd, mc_ankle_high.velocity_limit, mc_ankle_high.velKp, mc_ankle_high.velKd, mc_ankle_high.velKi);
        dm.receive_evo();
        dataWrite(10, motors[10].Get_Position(), motors[10].Get_Velocity(), motors[10].Get_tau(), &filename_ankle_high, 4); // 保存数据到文件中

        usleep(200);

        // ankle low
        dm.Motorevo_motor_servo_mode(motors[11], mc_ankle_low.pos_desired[i], mc_ankle_low.posKp, mc_ankle_low.posKd, mc_ankle_low.velocity_limit, mc_ankle_low.velKp, mc_ankle_low.velKd, mc_ankle_low.velKi);
        dm.receive_evo();
        dataWrite(11, motors[11].Get_Position(), motors[11].Get_Velocity(), motors[11].Get_tau(), &filename_ankle_low, 5); // 保存数据到文件中
        usleep(1000);
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

int main(int argc, char const *argv[])
{
    test01();
    return 0;
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

double degree2rad(double& degree){
    return degree * pi / 180.0;
}
double rad2degree(double& rad){
    return rad * 180.0 / pi;
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

void dataWrite(int canID, float motor_pos, float motor_vel, float motor_tor, const std::string* filename, int whichFileObj){
    std::string contest = std::to_string(canID) + " " + std::to_string(motor_pos) + " " + std::to_string(motor_vel) + " " + std::to_string(motor_tor);
    std::ofstream& file_ = file[whichFileObj];
    if (!file_.is_open())
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
        file_.open(file_path, std::ios::app);
        if (!file)
        {
            std::cerr << "Failed to open file!";
            
        }
        
    }
    file_ << contest << "\n";
}