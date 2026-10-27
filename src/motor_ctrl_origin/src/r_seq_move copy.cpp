#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>

#define SLEEP_TIME 10000

motor_head::Motor motors[6] = {

    motor_head::Motor(motor_head::DM10010, 0x07, 0x27),  // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x08, 0x28),  // 髋并联 右  负
    motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋偏航
    motor_head::Motor(motor_head::PA105, 0x0b, 0x31),    // 膝
    motor_head::Motor(motor_head::HA60, 0x0c, 0x32),     // 踝滚转
};
std::shared_ptr<SerialPort> serial;
motor_head::Motor_Control mt(serial);

int main(int argc, char *argv[])
{
    // 编码器初始化//
    encoder_read reader;
    const char *serial_port = "/dev/ttyUSB0";
    reader.encoder_serial(serial_port, B2500000);
    // 电机串口初始化
    serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    mt = motor_head::Motor_Control(serial);

    // —— 1. 生成时间戳 ——
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string ts = oss.str();

    // —— 2. 拼接包含时间戳的文件路径 ——
    std::string dir = "/home/ye/dgz_motor/src/motor_ctrl/csv/"; // 根据需要改成你的目录
    std::string file_th_name = dir + "th_pva_" + ts + ".csv";
    std::string file_jt_name = dir + "joint_pva_" + ts + ".csv";

    // —— 3. 打开文件 ——
    std::ofstream file_th(file_th_name);
    std::ofstream file_jt(file_jt_name);
    // 写表头
    int J = 5; // 关节数，根据你的实际改

    // 期望 th_pva CSV 表头
    for (int j = 0; j < J; ++j)
        file_th << "q" << j + 1 << "_pos,";
    for (int j = 0; j < J; ++j)
        file_th << "q" << j + 1 << "_vel,";
    for (int j = 0; j < J; ++j)
        file_th << "q" << j + 1 << "_tau"
                << (j + 1 == J ? "\n" : ",");

    // 实际 joint_pva CSV 表头
    for (int j = 0; j < J; ++j)
        file_jt << "q" << j + 1 << "_pos,";
    for (int j = 0; j < J; ++j)
        file_jt << "q" << j + 1 << "_vel,";
    for (int j = 0; j < J; ++j)
        file_jt << "q" << j + 1 << "_tau"
                << (j + 1 == J ? "\n" : ",");
    // 添加电机
    for (int i = 0; i < 6; i++)
    {
        mt.addMotor(&motors[i]);
        if (i == 4)
        {
            mt.elmo_set_vel_mode(motors[i].GetSlaveId());

            mt.elmo_enable(motors[i].GetSlaveId());
        }
        else
        {
            mt.enable(motors[i]);
            usleep(10000);
        }
    }
    // mt.dm_control_pos_vel(motors[3], 0, 0.2);
    // sleep(1);
    // usleep(10000);
    // uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10}; //  0x0b 0x0d

    // std::cout << "addMotor successful" << std::endl;
    // for (int i = 0; i < 5; i++)
    // {
    //     mt.refresh_motor_status(motors[3]);
    //     std::cout << "motors[3].Get_Position() :" << motors[3].Get_Position() * 180 / M_PI << std::endl;
    //     //     reader.setadress(adr[i]);
    //     // std::cout << "reader.readPosition(reader.encoder())" << reader.readPosition(reader.encoder()) * 180 / M_PI << std::endl;

    //     sleep(1);
    // }
    // for (int i = 0; i < 5; i++)
    // {

    //     mt.disable(motors[i]);
    //     usleep(10000);
    // }
    // 定义各个关节PID 参数
    // 位置环PID
    float kp[6] = {25, 25.0, 25.0, 10, 0.45, 1};
    float ki[6] = {0.1, 0.1, 0.05, 0.01, 0.01, 0.05};
    float kd[6] = {1.2, 1.2, 0.3, 0.1, 0.015, 0.3};
    // 速度环PID
    float kp_v[6] = {7.6, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {1.25, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.4, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[6] = {5, 5, 5, 5, 5, 5};   /* 积分限幅 */
    float outputLimit_v[6] = {3, 3, 3, 5, 3, 3};   /* 速度限幅 */
    float outputLimit_tau[6] = {3, 3, 2, 5, 3, 3}; /* 力矩限幅 */
    float timeDelta = 10.0;
    float errorThreshold = 0.05;
    float v_outputs[6];
    float tau_output[6];

    // 初始化pid参数
    PIDController pidControllers[6];
    // 使用循环初始化每个 PID 控制器对象
    for (int i = 0; i < 6; ++i)
    {
        pidControllers[i] = PIDController(
            kp[i],            // kp
            ki[i],            // ki
            kd[i],            // kd
            integralLimit[i], // integralLimit
            outputLimit_v[i], // outputLimit
            timeDelta,        // timeDelta
            errorThreshold);
    }

    // 速度环PID
    PIDController speedPIDControllers[6];
    for (int i = 0; i < 6; ++i)
    {
        speedPIDControllers[i] = PIDController(
            kp_v[i],            // kp
            ki_v[i],            // ki
            kd_v[i],            // kd
            integralLimit[i],   // integralLimit
            outputLimit_tau[i], // outputLimit
            timeDelta,          // timeDelta
            errorThreshold);
    }

    // 编码器地址
    uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10}; //  0x0b 0x0d
    float JntCurrent[6] = {0, 0, 0, 0, 0, 0};
    // 目标位置
    float JntTarge[6] = {0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180}; // 从下往上 髋偏航 正 逆时针
    

    float v_output[6];

    std::cout << "init successful " << std::endl;
    sleep(1);
    std::cout << "-----------" << std::endl;
    // 设置仿真参数
    float tf = 2.0;
    float dt = 0.01;
    float tt = 0.0;
    // 关节运动
    int JNT_NUM = 6;
    float a0345[JNT_NUM][4];
    float qq_data[JNT_NUM][3];
    int g_iPlanFirstCircle_0 = 0; /* 规划次数 */
    int n = 3;                    // 设定读取的关节,n = 0、1、2、5
    /************************单关节精度实验（位置、速度、加速度）五次多项式*******************************/
    float qq;
    float qv;
    float qa;
    float Jnt_old[6]; // 上一时刻的位置
    float Jnt_v[6];
    float Jnt_v1[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 上一时刻的速度
    float Jnt_a[6];
    float cutoff_frequency = 1 / tf;            // 截止频率，单位rad/s
    std::vector<std::vector<double>> th_pva;    // 用于存储理论关节数据
    std::vector<std::vector<double>> joint_pva; // 用于存储实际关节数据 */
    float pos_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float motor_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float motor_inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double des_pos[6] = {0, 0, 0, 0, 0.0, 0};
    double des_vel[6] = {0, 0, 0, 0, 0.0, 0};
    double des_tau[6] = {0, 0, 0, 0, 0.0, 0};
    double vref[6] = {0, 0, 0, 0, 0.0, 0};

    // 髋偏航
    std::cout << "--------------" << std::endl;

    mt.dm_control_pos_vel(motors[3], 0, 0.2);
 
    sleep(0.5);
    tt = 0.0;
    g_iPlanFirstCircle_0 = 0;
    JntTarge[3] = 10 * M_PI / 180;
    // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    for (int i = 3; i < 4; i++)
    {
        mt.refresh_motor_status(motors[i]);
        JntCurrent[i] = motors[i].Get_Position();
        Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
        usleep(2000);
    }

    while (tt <= tf)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        g_iPlanFirstCircle_0++;
        tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
        for (int i = 3; i < 4; i++)
        {
            Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 弧度制
            qq = qq_data[i][0];
            qv = qq_data[i][1];
            qa = qq_data[i][2];
            mt.refresh_motor_status(motors[i]);

            JntCurrent[i] = motors[i].Get_Position();
            des_pos[i] = qq;
            pos_err[i] = des_pos[i] - JntCurrent[i];
            motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
            motor_inc[i] = des_pos[i] + motor_err[i];

            // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

            des_vel[i] = qv;
            vref[i] = des_vel[i];
            Jnt_v[i] = motors[i].Get_Velocity();

            Jnt_old[i] = JntCurrent[i];
            Jnt_v1[i] = Jnt_v[i];
            // 记录理论关节和实际关节参数
        }
        for (int i = 3; i < 4; i++)
        {
            if (vref[i] > outputLimit_v[i])
                vref[i] = outputLimit_v[i];
            if (vref[i] < -outputLimit_v[i])
                vref[i] = -outputLimit_v[i];
            mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
            std::cout << "t=" << tt
                      << " des_pos"
                      << "[" << i << "] : " << des_pos[i] * 180 / M_PI
                      << " vref="
                      << "[" << i << "]: " << vref[i]
                      << " act_pos="
                      << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
                      << std::endl;
        }
        // 写入一行期望数据（3*J 列）
        file_th << des_pos[3] * 180 / M_PI << ",";
        file_th << des_vel[3] << ",";
        file_th << des_tau[3] << "\n";

        file_jt << JntCurrent[3] * 180 / M_PI << ",";
        file_jt << Jnt_v[3] << ",";
        file_jt << motors[3].Get_tau() << "\n";

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    }
    sleep(0.5);
    tt = 0.0;
    g_iPlanFirstCircle_0 = 0;
    JntTarge[3] = -10 * M_PI / 180;
    // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    for (int i = 3; i < 4; i++)
    {
        mt.refresh_motor_status(motors[i]);
        JntCurrent[i] = motors[i].Get_Position();
        Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
        usleep(2000);
    }
    while (tt <= tf)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        g_iPlanFirstCircle_0++;
        tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
        for (int i = 3; i < 4; i++)
        {
            Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
            qq = qq_data[i][0];
            qv = qq_data[i][1];
            qa = qq_data[i][2];
            mt.refresh_motor_status(motors[i]);
            JntCurrent[i] = motors[i].Get_Position();
            des_pos[i] = qq;
            pos_err[i] = des_pos[i] - JntCurrent[i];
            motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
            motor_inc[i] = des_pos[i] + motor_err[i];
            // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);
            des_vel[i] = qv;
            vref[i] = des_vel[i];
            Jnt_v[i] = motors[i].Get_Velocity();
            Jnt_old[i] = JntCurrent[i];
            Jnt_v1[i] = Jnt_v[i];
            // 记录理论关节和实际关节参数
        }
        for (int i = 3; i < 4; i++)
        {
            if (vref[i] > outputLimit_v[i])
                vref[i] = outputLimit_v[i];
            if (vref[i] < -outputLimit_v[i])
                vref[i] = -outputLimit_v[i];
            mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
            std::cout << "t=" << tt
                      << " des_pos"
                      << "[" << i << "] : " << des_pos[i] * 180 / M_PI
                      << " vref="
                      << "[" << i << "]: " << vref[i]
                      << " act_pos="
                      << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
                      << std::endl;
        }
        file_th << des_pos[3] * 180 / M_PI << ",";
        file_th << des_vel[3] << ",";
        file_th << des_tau[3] << "\n";
        file_jt << JntCurrent[3] * 180 / M_PI << ",";
        file_jt << Jnt_v[3] << ",";
        file_jt << motors[3].Get_tau() << "\n";
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    }
    sleep(0.5);
    // 复位

    tt = 0.0;
    g_iPlanFirstCircle_0 = 0;
    JntTarge[3] = 0;
    // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    for (int i = 3; i < 4; i++)
    {
        mt.refresh_motor_status(motors[i]);
        JntCurrent[i] = motors[i].Get_Position();
        Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
        usleep(2000);
    }
    while (tt <= tf)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        g_iPlanFirstCircle_0++;
        tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
        for (int i = 3; i < 4; i++)
        {
            Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
            qq = qq_data[i][0];
            qv = qq_data[i][1];
            qa = qq_data[i][2];
            mt.refresh_motor_status(motors[i]);
            JntCurrent[i] = motors[i].Get_Position();
            des_pos[i] = qq;
            pos_err[i] = des_pos[i] - JntCurrent[i];
            motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
            motor_inc[i] = des_pos[i] + motor_err[i];
            // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);
            des_vel[i] = qv;
            vref[i] = des_vel[i];
            Jnt_v[i] = motors[i].Get_Velocity();
            Jnt_old[i] = JntCurrent[i];
            Jnt_v1[i] = Jnt_v[i];
            // 记录理论关节和实际关节参数
        }
        for (int i = 3; i < 4; i++)
        {
            if (vref[i] > outputLimit_v[i])
                vref[i] = outputLimit_v[i];
            if (vref[i] < -outputLimit_v[i])
                vref[i] = -outputLimit_v[i];
            mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
            std::cout << "t=" << tt
                      << " des_pos"
                      << "[" << i << "] : " << des_pos[i] * 180 / M_PI
                      << " vref="
                      << "[" << i << "]: " << vref[i]
                      << " act_pos="
                      << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
                      << std::endl;
        }
        file_th << des_pos[3] * 180 / M_PI << ",";
        file_th << des_vel[3] << ",";
        file_th << des_tau[3] << "\n";
        file_jt << JntCurrent[3] * 180 / M_PI << ",";
        file_jt << Jnt_v[3] << ",";
        file_jt << motors[3].Get_tau() << "\n";
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    }
    sleep(0.5);

    // // 髋并联
    // mt.dm_control_pos_vel(motors[0], 0, 0.2);
    // mt.dm_control_pos_vel(motors[1], 0, 0.2);
    // sleep(1);
    // // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[0] = -15 * M_PI / 180;
    // JntTarge[1] = 15 * M_PI / 180;
    // for (int i = 0; i < 2; i++)
    // {
    //     mt.refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();
    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化

    //     usleep(10000);
    // }
    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();

    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 0; i < 2; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.refresh_motor_status(motors[i]);

    //         JntCurrent[i] = motors[i].Get_Position();
    //         des_pos[i] = qq;
    //         pos_err[i] = des_pos[i] - JntCurrent[i];
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_pos[i] + motor_err[i];

    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         des_vel[i] = qv;
    //         vref[i] = des_vel[i];
    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }

    //     for (int i = 0; i < 2; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_pos[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_vel[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_tau[j]
    //                 << (j + 1 == J ? "\n" : ",");

    //     // 写入一行实际数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_jt << JntCurrent[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << Jnt_v[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << motors[j].Get_tau()
    //                 << (j + 1 == J ? "\n" : ",");
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);
    // // mt.dm_control_pos_vel(motors[0], 0, 0.2);
    // // mt.dm_control_pos_vel(motors[1], 0, 0.2);
    // sleep(1);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;

    // for (int i = 0; i < 2; i++)
    // {
    //     JntTarge[i] = 0;
    // }
    // // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    // for (int i = 0; i < 2; i++)
    // {
    //     mt.refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();
    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
    //     usleep(2000);
    // }

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();
    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 0; i < 2; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.refresh_motor_status(motors[i]);

    //         JntCurrent[i] = motors[i].Get_Position();
    //         des_pos[i] = qq;
    //         pos_err[i] = des_pos[i] - JntCurrent[i];
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_pos[i] + motor_err[i];

    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         des_vel[i] = qv;
    //         vref[i] = des_vel[i];
    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }
    //     for (int i = 0; i < 2; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_pos[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_vel[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_tau[j]
    //                 << (j + 1 == J ? "\n" : ",");

    //     // 写入一行实际数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_jt << JntCurrent[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << Jnt_v[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << motors[j].Get_tau()
    //                 << (j + 1 == J ? "\n" : ",");

    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);
    // 膝
    // n = 4;

    // JntTarge[4] = 20 * M_PI / 180;

    // reader.setadress(adr[6]);
    // JntCurrent[n] = reader.readPosition(reader.encoder()); // 弧度
    // Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[n], JntTarge[n], tf, a0345[n]);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();
    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt;                               // 规划次数*采样时间
    //     Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]); // 得到弧度制
    //     qq = qq_data[n][0];
    //     qv = qq_data[n][1];
    //     qa = qq_data[n][2];
    //     // std::cout << "qq:" << qq * 180 /M_PI<< std::endl;

    //     reader.setadress(adr[6]);
    //     JntCurrent[n] = reader.readPosition(reader.encoder()); // 弧度
    //     // mt.elmo_get_pos(motors[0].GetSlaveId());
    //     // mt.elmo_get_pos(motors[0].GetSlaveId());
    //     // mt.elmo_get_pos(motors[0].GetSlaveId());
    //     // JntCurrent[n] = mt.elmo_get_pos(motors[0].GetSlaveId());
    //     // if (JntCurrent[n] == 0)
    //     // {
    //     //   JntCurrent[n] = Jnt_old[n];
    //     // }
    //     // std::cout << "JntCurrent[n]:" << JntCurrent[n] * 180 /M_PI<< std::endl;
    //     motor_err[n] = qq - JntCurrent[n];
    //     float motor_ = pidControllers[n].PIDCompute_err(motor_err[n]);
    //     // float motor_inc = qv;
    //     float ff_gain = 1.0f; // 通过实验调整的最佳值
    //     float motor_inc = ff_gain * qv + motor_;
    //     // float motor_inc = qv + motor_ ;

    //     // // 限幅
    //     // if (motor_inc > outputLimit_v[n])
    //     //   motor_inc = outputLimit_v[n];
    //     // if (motor_inc < -outputLimit_v[n])
    //     //   motor_inc = -outputLimit_v[n];

    //     Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n])) / (timeDelta / 1000);
    //     Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
    //     // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
    //     // Jnt_v[n] = motors[n].Get_Velocity();
    //     // v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);
    //     // if (JntCurrent[n] * 180 / M_PI >= 30 || JntCurrent[n] * 180 / M_PI <= -30)
    //     // {
    //     //   mt.elmo_disable(motors[0].GetSlaveId());
    //     //   std::cout << "过限位" << std::endl;
    //     //   break;
    //     // }

    //     mt.elmo_set_speed(motors[n].GetSlaveId(), motor_inc); // 负 往后摆
    //     mt.elmo_set_start(motors[n].GetSlaveId());

    //     // 记录上一时刻的关节角，速度
    //     // 打印调试

    //     std::cout << "t=" << tt
    //               << "del_pos=" << qq * 180 / M_PI
    //               << " motor_inc=" << motor_inc
    //               << " act_pos=" << JntCurrent[n] * 180 / M_PI

    //               << std::endl;
    //     Jnt_old[n] = JntCurrent[n];
    //     Jnt_v1[n] = Jnt_v[n];

    //     // 记录理论关节和实际关节参数
    //     th_pva.push_back({qq * 180 / M_PI, qv, tau_output[n]});
    //     joint_pva.push_back({JntCurrent[n] * 180 / M_PI, Jnt_v[n], 0});
    //     auto &th = th_pva.back();
    //     auto &jt = joint_pva.back();
    //     file_th << th[0] << "," << th[1] << "," << th[2] << "\n";
    //     file_jt << jt[0] << "," << jt[1] << "," << jt[2] << "\n";
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }

    // sleep(0.5);

    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[4] = 0;
    // reader.setadress(adr[6]);
    // JntCurrent[n] = reader.readPosition(reader.encoder());
    // Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[n], JntTarge[n], tf, a0345[n]);

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();
    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt;                               // 规划次数*采样时间
    //     Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]); // 得到弧度制
    //     qq = qq_data[n][0];
    //     qv = qq_data[n][1];
    //     qa = qq_data[n][2];

    //     reader.setadress(adr[6]);
    //     JntCurrent[n] = reader.readPosition(reader.encoder()); // 弧度

    //     motor_err[n] = qq - JntCurrent[n];
    //     float motor_ = pidControllers[n].PIDCompute_err(motor_err[n]);
    //     // float motor_inc = qv;
    //     float ff_gain = 1.0f; // 通过实验调整的最佳值
    //     float motor_inc = ff_gain * qv + motor_;
    //     // float motor_inc = qv + motor_ ;

    //     // // 限幅
    //     // if (motor_inc > outputLimit_v[n])
    //     //   motor_inc = outputLimit_v[n];
    //     // if (motor_inc < -outputLimit_v[n])
    //     //   motor_inc = -outputLimit_v[n];

    //     Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n])) / (timeDelta / 1000);
    //     Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
    //     // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
    //     // Jnt_v[n] = motors[n].Get_Velocity();
    //     // v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);
    //     // if (JntCurrent[n] * 180 / M_PI >= 30 || JntCurrent[n] * 180 / M_PI <= -30)
    //     // {
    //     //   mt.elmo_disable(motors[0].GetSlaveId());
    //     //   std::cout << "过限位" << std::endl;
    //     //   break;
    //     // }
    //     mt.elmo_set_speed(motors[n].GetSlaveId(), motor_inc); // 负 往后摆
    //     mt.elmo_set_start(motors[n].GetSlaveId());

    //     // 记录上一时刻的关节角，速度
    //     // 打印调试

    //     std::cout << "t=" << tt
    //               << "del_pos=" << qq * 180 / M_PI
    //               << " motor_inc=" << motor_inc
    //               << " act_pos=" << JntCurrent[n] * 180 / M_PI

    //               << std::endl;
    //     Jnt_old[n] = JntCurrent[n];
    //     Jnt_v1[n] = Jnt_v[n];

    //     // 记录理论关节和实际关节参数
    //     th_pva.push_back({qq * 180 / M_PI, qv, tau_output[n]});
    //     joint_pva.push_back({JntCurrent[n] * 180 / M_PI, Jnt_v[n], 0});
    //     auto &th = th_pva.back();
    //     auto &jt = joint_pva.back();
    //     file_th << th[0] << "," << th[1] << "," << th[2] << "\n";
    //     file_jt << jt[0] << "," << jt[1] << "," << jt[2] << "\n";
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);
    // // 踝俯仰
    // mt.dm_control_pos_vel(motors[2], 0, 0.2);
    // sleep(0.5);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[2] = 15 * M_PI / 180;
    // // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    // for (int i = 2; i < 3; i++)
    // {
    //     mt.refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();
    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
    //     usleep(2000);
    // }

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();
    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 2; i < 3; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.refresh_motor_status(motors[i]);

    //         JntCurrent[i] = motors[i].Get_Position();
    //         des_pos[i] = qq;
    //         pos_err[i] = des_pos[i] - JntCurrent[i];
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_pos[i] + motor_err[i];

    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         des_vel[i] = qv;
    //         vref[i] = des_vel[i];
    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }
    //     for (int i = 2; i < 3; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_pos[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_vel[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_tau[j]
    //                 << (j + 1 == J ? "\n" : ",");

    //     // 写入一行实际数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_jt << JntCurrent[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << Jnt_v[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << motors[j].Get_tau()
    //                 << (j + 1 == J ? "\n" : ",");

    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[2] = -15 * M_PI / 180;
    // // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    // for (int i = 2; i < 3; i++)
    // {
    //     mt.refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();
    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
    //     usleep(2000);
    // }

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();
    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 2; i < 3; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.refresh_motor_status(motors[i]);

    //         JntCurrent[i] = motors[i].Get_Position();
    //         des_pos[i] = qq;
    //         pos_err[i] = des_pos[i] - JntCurrent[i];
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_pos[i] + motor_err[i];

    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         des_vel[i] = qv;
    //         vref[i] = des_vel[i];
    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }
    //     for (int i = 2; i < 3; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_pos[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_vel[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_tau[j]
    //                 << (j + 1 == J ? "\n" : ",");

    //     // 写入一行实际数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_jt << JntCurrent[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << Jnt_v[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << motors[j].Get_tau()
    //                 << (j + 1 == J ? "\n" : ",");

    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);
    // // 复位
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[2] = 0;
    // // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    // for (int i = 2; i < 3; i++)
    // {
    //     mt.refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();
    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
    //     usleep(2000);
    // }

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();
    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 2; i < 3; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.refresh_motor_status(motors[i]);

    //         JntCurrent[i] = motors[i].Get_Position();
    //         des_pos[i] = qq;
    //         pos_err[i] = des_pos[i] - JntCurrent[i];
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_pos[i] + motor_err[i];

    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         des_vel[i] = qv;
    //         vref[i] = des_vel[i];
    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }
    //     for (int i = 2; i < 3; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_pos[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_vel[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_th << des_tau[j]
    //                 << (j + 1 == J ? "\n" : ",");

    //     // 写入一行实际数据（3*J 列）
    //     for (int j = 0; j < J; ++j)
    //         file_jt << JntCurrent[j] * 180 / M_PI << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << Jnt_v[j] << ",";
    //     for (int j = 0; j < J; ++j)
    //         file_jt << motors[j].Get_tau()
    //                 << (j + 1 == J ? "\n" : ",");

    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);

    // 踝滚转
    // mt.Motorevo_motor_torque_position_mix_control_mode(motors[5], 0, 1, 0, 40, 4);
    // sleep(0.5);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[5] = 10 * M_PI / 180;
    // for (int i = 5; i < 6; i++)
    // {
    //     mt.motorevo_refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();

    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化

    //     usleep(2000);
    // }
    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();

    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 5; i < 6; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到弧度
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.motorevo_refresh_motor_status(motors[i]);
    //         JntCurrent[i] = motors[i].Get_Position();
    //         if (JntCurrent[i] >= 30 * M_PI / 180 || JntCurrent[i] <= -30 * M_PI / 180)
    //         {
    //             mt.disable(motors[i]);
    //         }
    //         des_pos[i] = qq;
    //         des_vel[i] = qv;
    //         pos_err[i] = qq - motors[i].Get_Position();
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_vel[i] + motor_err[i];
    //         vref[i] = motor_inc[i];
    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }

    //     for (int i = 5; i < 6; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], qq, vref[i], 0, 40, 4);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }

    //     file_th << des_pos[5] * 180 / M_PI << ",";
    //     file_th << des_vel[5] << ",";
    //     file_th << des_tau[5] << "\n";

    //     file_jt << JntCurrent[5] * 180 / M_PI << ",";
    //     file_jt << Jnt_v[5] << ",";
    //     file_jt << motors[5].Get_tau() << "\n";
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // mt.Motorevo_motor_torque_position_mix_control_mode(motors[5], 0, 1, 0, 40, 4);
    // sleep(0.5);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[5] = -10 * M_PI / 180;
    // for (int i = 5; i < 6; i++)
    // {
    //     mt.motorevo_refresh_motor_status(motors[i]);
    //     JntCurrent[i] = motors[i].Get_Position();

    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化

    //     usleep(2000);
    // }
    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();

    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 5; i < 6; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到弧度
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.motorevo_refresh_motor_status(motors[i]);
    //         JntCurrent[i] = motors[i].Get_Position();
    //         if (JntCurrent[i] >= 30 * M_PI / 180 || JntCurrent[i] <= -30 * M_PI / 180)
    //         {
    //             mt.disable(motors[i]);
    //         }
    //         des_pos[i] = qq;
    //         des_vel[i] = qv;
    //         pos_err[i] = qq - motors[i].Get_Position();
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_vel[i] + motor_err[i];
    //         vref[i] = motor_inc[i];
    //         // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }

    //     for (int i = 5; i < 6; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], qq, vref[i], 0, 40, 4);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     file_th << des_pos[5] * 180 / M_PI << ",";
    //     file_th << des_vel[5] << ",";
    //     file_th << des_tau[5] << "\n";

    //     file_jt << JntCurrent[5] * 180 / M_PI << ",";
    //     file_jt << Jnt_v[5] << ",";
    //     file_jt << motors[5].Get_tau() << "\n";
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(0.5);
    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;
    // JntTarge[5] = 0;

    // // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    // for (int i = 5; i < 6; i++)
    // {
    //     mt.motorevo_refresh_motor_status(motors[i]);

    //     JntCurrent[i] = motors[i].Get_Position();

    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化

    //     usleep(2000);
    // }

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();

    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 5; i < 6; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到弧度
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         mt.motorevo_refresh_motor_status(motors[i]);
    //         JntCurrent[i] = motors[i].Get_Position();
    //         if (JntCurrent[i] >= 30 * M_PI / 180 || JntCurrent[i] <= -30 * M_PI / 180)
    //         {
    //             mt.disable(motors[i]);
    //         }
    //         des_pos[i] = qq;
    //         des_vel[i] = qv;

    //         pos_err[i] = qq - motors[i].Get_Position();
    //         motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //         motor_inc[i] = des_vel[i] + motor_err[i];
    //         vref[i] = motor_inc[i];
    //         Jnt_v[i] = motors[i].Get_Velocity();

    //         Jnt_old[i] = JntCurrent[i];
    //         Jnt_v1[i] = Jnt_v[i];
    //         // 记录理论关节和实际关节参数
    //     }

    //     for (int i = 0; i < JNT_NUM; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], qq, vref[i], 0, 40, 4);
    //         std::cout << "t=" << tt
    //                   << " des_pos"
    //                   << "[" << i << "] : " << des_pos[i] * 180 / M_PI
    //                   << " vref="
    //                   << "[" << i << "]: " << vref[i]
    //                   << " act_pos="
    //                   << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
    //                   << std::endl;
    //     }
    //     // 写入一行期望数据（3*J 列）
    //     file_th << des_pos[5] * 180 / M_PI << ",";
    //     file_th << des_vel[5] << ",";
    //     file_th << des_tau[5] << "\n";

    //     file_jt << JntCurrent[5] * 180 / M_PI << ",";
    //     file_jt << Jnt_v[5] << ",";
    //     file_jt << motors[5].Get_tau() << "\n";
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    // sleep(1);

    file_th.close();
    file_jt.close();
    for (int i = 0; i < 6; i++)
    {
        if (i == 4)
        {
            mt.elmo_disable(motors[i].GetSlaveId());
        }
        else
        {
            mt.disable(motors[i]);
        }
        usleep(10000);
    }
    return 0;
}
