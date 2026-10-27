#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>
motor_head::Motor motors[5] = {

    motor_head::Motor(motor_head::DM10010, 0x01, 0x21),  // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x02, 0x22),  // 髋并联 右  负
    motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x04, 0x14), // 髋滚转
    motor_head::Motor(motor_head::PA105, 0x05, 0x25),    // 膝
    // motor_head::Motor(motor_head::HA60, 0x06, 0x26),     // 踝滚转
};
std::shared_ptr<SerialPort> serial;
motor_head::Motor_Control mt(serial);

int main(int argc, char *argv[])
{
    // 编码器初始化//
    encoder_read reader;
    // const char *serial_port = "/dev/ttyUSB0";
    // reader.encoder_serial(serial_port, B2500000);
    // 电机串口初始化
    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
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
    int J = 2; // 关节数，根据你的实际改

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
    for (int i = 0; i < 5; i++)
    {
        mt.addMotor(&motors[i]);
        mt.enable(motors[i]);
        usleep(10000);
    }
    // mt.addMotor(&motors[2]);
    // mt.enable(motors[2]);
    usleep(10000);
    std::cout << "addMotor successful" << std::endl;

    // 定义各个关节PID 参数
    // 位置环PID
    float kp[6] = {25, 25.0, 25.0, 25, 1.0, 40.0};
    float ki[6] = {0.1, 0.1, 0.05, 0.7, 0.05, 1.1};
    float kd[6] = {1.2, 1.2, 0.3, 0.2, 0.3, 1.2};
    // 速度环PID
    float kp_v[6] = {7.6, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {1.25, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.4, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[6] = {5, 5, 5, 5, 5, 5};    /* 积分限幅 */
    float outputLimit_v[6] = {3, 3, 3, 5, 3, 3};    /* 速度限幅 */
    float outputLimit_tau[6] = {3, 3, 2, 10, 3, 3}; /* 力矩限幅 */
    float timeDelta = 10.0;
    float errorThreshold = 0.05;
    float v_outputs[6];
    float tau_output[6];

    // 初始化pid参数
    //  位置环PID    motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋滚转

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
    uint8_t adr[6] = {0x00, 0x02, 0x04, 0x07, 0x08, 0x00};
    float JntCurrent[6] = {0, 0, 0, 0, 0, 0};
    // 目标位置
    float JntTarge[6] = {30, -30, 0, 0.0, 0, 0}; // 从下往上 髋偏航 正 逆时针
    float v_output[6];

    std::cout << "init successful " << std::endl;
    // if (mt.switchControlMode(motors[3], motor_head::MIT_MODE))
    //     std::cout << "Switch to MIT_MODE Success" << std::endl;
    // mt.save_motor_param(motors[3]);
    sleep(1);
    std::cout << "-----------" << std::endl;
    // 设置仿真参数
    float tf = 2.0;
    float dt = 0.005;
    float tt = 0.0;
    // 关节运动
    int JNT_NUM = 6;
    float a0345[JNT_NUM][4];
    float qq_data[JNT_NUM][3];
    int NodeNum = (tf / dt) + 1;
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
    mt.dm_control_pos_vel(motors[0],0,0.1);
    mt.dm_control_pos_vel(motors[1],0,0.1);
    sleep(2);
    // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    for (int i = 0; i < 2; i++)
    {
        mt.refresh_motor_status(motors[i]);
        JntCurrent[i] = motors[i].Get_Position() * 180 / M_PI;

        Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化

        usleep(2000);
    }
    while (tt <= tf)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        g_iPlanFirstCircle_0++;
        tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
        for (int i = 0; i < 2; i++)
        {
            Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
            qq = qq_data[i][0];
            qv = qq_data[i][1] / 180 * M_PI;
            qa = qq_data[i][2];
            mt.refresh_motor_status(motors[i]);

            // std::cout << "tt :" << tt << "qq :" << qq << std::eidl;
            JntCurrent[i] = motors[i].Get_Position();
            des_pos[i] = qq * M_PI / 180;
            pos_err[i] = des_pos[i] - motors[i].Get_Position();
            motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
            motor_inc[i] = des_pos[i] + motor_err[i];

            // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

            des_vel[i] = qv;
            Jnt_v[i] = motors[i].Get_Velocity();

            Jnt_old[i] = JntCurrent[i];
            Jnt_v1[i] = Jnt_v[i];
            // 记录理论关节和实际关节参数
        }

        for (int i = 0; i < 2; i++)
        {
            // if (vref[i] > outputLimit_v[i])
            //     vref[i] = outputLimit_v[i];
            // if (vref[i] < -outputLimit_v[i])
            //     vref[i] = -outputLimit_v[i];
            mt.dm_control_pos_vel(motors[i], motor_inc[i], des_vel[i]);
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
        for (int j = 0; j < J; ++j)
            file_th << des_pos[j] * 180 / M_PI << ",";
        for (int j = 0; j < J; ++j)
            file_th << des_vel[j] << ",";
        for (int j = 0; j < J; ++j)
            file_th << des_tau[j]
                    << (j + 1 == J ? "\n" : ",");

        // 写入一行实际数据（3*J 列）
        for (int j = 0; j < J; ++j)
            file_jt << JntCurrent[j] * 180 / M_PI << ",";
        for (int j = 0; j < J; ++j)
            file_jt << Jnt_v[j] << ",";
        for (int j = 0; j < J; ++j)
            file_jt << motors[j].Get_tau()
                    << (j + 1 == J ? "\n" : ",");
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(5000 - elapsed.count()); // 精确匹配500Hz
    }

    tt = 0.0;
    g_iPlanFirstCircle_0 = 0;

    for (int i = 0; i < 2; i++)
    {
        JntTarge[i] = 0;
    }
    // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    for (int i = 0; i < 2; i++)
    {
        mt.refresh_motor_status(motors[i]);
        JntCurrent[i] = motors[i].Get_Position() * 180 / M_PI;

        Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
        // std::cout << "initial--joint" << i + 1 << " " << JntCurrent[i] / M_PI * 180.0 << std::endl;
        usleep(2000);
    }

    while (tt <= tf)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        g_iPlanFirstCircle_0++;
        tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
        for (int i = 0; i < 2; i++)
        {
            Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
            qq = qq_data[i][0];
            qv = qq_data[i][1] / 180 * M_PI;
            qa = qq_data[i][2];

            // std::cout << "tt :" << tt << "qq :" << qq << std::eidl;
            JntCurrent[i] = motors[i].Get_Position();
            des_pos[i] = qq * M_PI / 180;
            pos_err[i] = des_pos[i] - motors[i].Get_Position();
            motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
            motor_inc[i] = des_pos[i] + motor_err[i];

            // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

            des_vel[i] = qv;
            Jnt_v[i] = motors[i].Get_Velocity();

            Jnt_old[i] = JntCurrent[i];
            Jnt_v1[i] = Jnt_v[i];
        }
        for (int i = 0; i < 2; i++)
        {
            // if (vref[i] > outputLimit_v[i])
            //     vref[i] = outputLimit_v[i];
            // if (vref[i] < -outputLimit_v[i])
            //     vref[i] = -outputLimit_v[i];
            mt.dm_control_pos_vel(motors[i], motor_inc[i], des_vel[i]);
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
        for (int j = 0; j < J; ++j)
            file_th << des_pos[j] * 180 / M_PI << ",";
        for (int j = 0; j < J; ++j)
            file_th << des_vel[j] << ",";
        for (int j = 0; j < J; ++j)
            file_th << des_tau[j]
                    << (j + 1 == J ? "\n" : ",");

        // 写入一行实际数据（3*J 列）
        for (int j = 0; j < J; ++j)
            file_jt << JntCurrent[j] * 180 / M_PI << ",";
        for (int j = 0; j < J; ++j)
            file_jt << Jnt_v[j] << ",";
        for (int j = 0; j < J; ++j)
            file_jt << motors[j].Get_tau()
                    << (j + 1 == J ? "\n" : ",");

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(5000 - elapsed.count()); // 精确匹配500Hz
    }
    // sleep(2);
    // mt.dm_control_pos_vel(motors[0], 0, 0.2);
    // mt.dm_control_pos_vel(motors[1], 0, 0.2);
    // sleep(4);
    file_th.close();
    file_jt.close();
    for (int i = 0; i < 5; i++)
    {

        mt.disable(motors[i]);
        usleep(10000);
    }
    return 0;
}
// 复位

// tt = 0.0;
// g_iPlanFirstCircle_0 = 0;
// // reader.setadress(adr[n]);
// // JntCurrent[n] = reader.readPosition(reader.encoder());
// // std::cout << "JNT: " << JntCurrent[n] << std::endl;

// for (int i = 0; i < 2; i++)
// {
//     JntTarge[i] = 0.0;

// }

// g_iPlanFirstCircle_0++;
// tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
// for (int i = 0; i < 1; i++)
// {
//     Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到角度制
//     qq = qq_data[i][0];
//     qv = qq_data[i][1] / 180 * M_PI;
//     qa = qq_data[i][2];
//     std::cout << ":JitCurreit[i]: " << JntCurrent[i] << std::endl;

//     // std::cout << "tt :" << tt << "qq :" << qq << std::eidl;
//     JntCurrent[i] = motors[i].Get_Position();
//     des_pos[i] = qq * M_PI / 180;
//     pos_err[i] = des_pos[i] - motors[i].Get_Position();
//     motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
//     motor_inc[i] = des_pos[i] + motor_err[i];
//     /* 关节闭环控制 */
//     // reader.setadress(adr[n]);
//     // JntCurrent[n] = reader.readPosition(reader.encoder());
//     // std::cout << "actual_pos ：" << actual_pos*180/M_PI << std::endl;
//     vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

//     // float tref = speedPIDControllers[n].PIDCompute(vref, Jnt_v[n]);

//     vref[i] = qv;
//     // vref = pos_delta;       // if (JntCurrent[n] * 180 / M_PI >= 20 || JntCurrent[n] * 180 / M_PI <= -20)
//     // {
//     //     mt.disable(motors[n]);
//     // }

//     // mt.dm_control_mit(motors[n], 200, 1.2, des_pos[n], vref, 0);
//     // Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n]) / 180.0 * M_PI) / (timeDelta / 1000);
//     // Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
//     // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
//     Jnt_v[i] = motors[i].Get_Velocity();
//     // v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);

//     // 记录上一时刻的关节角，速度
//     // 打印调试

//     // std::cout << "t=" << tt
//     //           << " des_pos=" << des_pos[i] * 180 / M_PI
//     //           << " vref=" << vref
//     //           << " act_pos=" << JntCurrent[i] * 180 / M_PI

//     //   << std::endl;
//     Jnt_old[i] = JntCurrent[i];
//     Jnt_v1[i] = Jnt_v[i];
// 记录理论关节和实际关节参数

////////// MIT

//  g_iPlanFirstCircle_0++;
//         tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
//         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]);

//         qq = qq_data[n][0];
//         qv = qq_data[n][1] / 180.0 * M_PI;
//         qa = qq_data[n][2];

//         /* 关节双环闭环控制 */
//         // reader.setadress(adr[n]);
//         // JntCurrent[n] = reader.readPosition(reader.encoder());
//         JntCurrent[n] = motors[n].Get_Position();

//         // Jnt_v[n] = (JntCurrent[n] - Jnt_old[n]) / (timeDelta / 1000);
//         Jnt_v[n] = motors[n].Get_Velocity();
//         // 滤波处理
//         // Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
//         // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);

//         float des_rad = qq * M_PI / 180.0f;
//         // v_outputs[n] = pidControllers[n].PIDCompute(des_rad, JntCurrent[n]);
//         float kp = 15;
//         float kd = 0.2;
//         float tau = kp * (des_rad - JntCurrent[n]) + kd * (qv - Jnt_v[n]) + 1;
//         // tau_output[n] = speedPIDControllers[n].PIDCompute(v_outputs, Jnt_v[n]);

//         // tau_output[n] = speedPIDControllers[n].PIDCompute(v_outputs[n], Jnt_v[n]);
//         // std::cout << "Jnt_v[n]： " << Jnt_v[n] << " tau_output[n]： " << tau_output[n] << " v_outputs[n]： " << v_outputs[n] << " qv： " << qv << std::endl;
//         // std::cout << "qq:" << qq << std::endl;
//         std::cout << JntCurrent[n] * 180 / M_PI<< " " << tau << std::endl;
//         // mt.dm_control_mit(motors[n], 0, 0, 0, 0, 2);
//         mt.dm_control_mit(motors[n], 0, 0, 0, 0, tau);
//   ;
//         // if (JntCurrent[n] * 180 / M_PI >= 20 || JntCurrent[n] * 180 / M_PI <= -20)
//         // {
//         //     mt.disable(motors[n]);
//         //     sleep(10);
//         // }
//         // 记录理论关节和实际关节参数

//         // std::cout << "t=" << tt
//         //           << " des_pos" << "[" << i << "] : " << des_pos[i] * 180 / M_PI
//         //           << " vref=" << "[" << i << "]: " << vref[i]
//         //           << " act_pos=" << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
//         //           << std::endl;