
#include <cstdio>
#include "ros/ros.h"

#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>
// 0x01 踝关节
// 0x07 膝关节
// 0x02 髋俯仰

motor_head::Motor motors[5] = {

    motor_head::Motor(motor_head::DM10010, 0x01, 0x21),  // 髋并联
    motor_head::Motor(motor_head::DM10010, 0x02, 0x22),  // 髋并联
    motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x07, 0x27), // 髋滚转
    motor_head::Motor(motor_head::PA105, 0x05, 0x25),    // 膝
    // motor_head::Motor(motor_head::HA60, 0x06, 0x26),     // 踝滚转
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
    std::string dir = "/home/ding/motor_ws/csv/"; // 根据需要改成你的目录
    std::string file_th_name = dir + "th_pva_" + ts + ".csv";
    std::string file_jt_name = dir + "joint_pva_" + ts + ".csv";

    // —— 3. 打开文件 ——
    std::ofstream file_th(file_th_name);
    std::ofstream file_jt(file_jt_name);
    // 写表头
    file_th << "des_pos,des_vel,des_tau\n";
    file_jt << "pos,vel,tau\n";

    // 添加电机
    for (int i = 0; i < 5; i++)
    {
        mt.addMotor(&motors[i]);
        mt.enable(motors[i]);
        usleep(10000);
    }
    mt.addMotor(&motors[2]);
    mt.enable(motors[2]);
    usleep(10000);
      // 关节角度限制
    float joint_max[7] = {M_PI, 85.0 / 180.0 * M_PI, 175.0 / 180.0 * M_PI, 115.0 / 180 * M_PI, 175.0 / 180 * M_PI, 85.0 / 180.0 * M_PI, 175.0 / 180.0 * M_PI};

    // 编码器地址
    uint8_t adr[8] = {0x02, 0x04, 0x07 ,0x08,0x0B, 0x0D,0x0E,0x10};
    std::cout << "addMotor successful" << std::endl;

    // 定义各个关节PID 参数
    // 位置环PID
    float kp[6] = {50, 3.0, 25.0, 25, 1.0, 40.0};
    float ki[6] = {0.05, 0.4, 0.05, 0.7, 0.05, 1.1};
    float kd[6] = {0.1, 1.0, 0.3, 0.2, 0.3, 1.2};
    // 速度环PID
    float kp_v[6] = {15, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {0.1, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.1, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[6] = {5, 5, 5, 5, 5, 5};    /* 积分限幅 */
    float outputLimit_v[6] = {3, 3, 3, 5, 3, 3};    /* 速度限幅 */
    float outputLimit_tau[6] = {3, 3, 2, 10, 3, 3}; /* 力矩限幅 */
    float timeDelta = 10.0;
    float errorThreshold = 0.05;
    float v_outputs[6];
    float tau_output[6];
    // PID肩2{3.6,0.6,0.32} 10s
    // PID{7.6,1.25,0.4} 5s 或 {9.0, 0.9 0.05}

    // 初始化pid参数
    //  位置环PID    motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋滚转

    PIDController pidControllers[6];
    // 使用循环初始化每个 PID 控制器对象
    for (int i = 1; i < 6; ++i)
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
    for (int i = 1; i < 6; ++i)
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
    uint8_t adr[6] = {0x00, 0x02, 0x08, 0x07, 0x08, 0x00}; // 0x08 踝俯仰
    float JntCurrent[6] = {0, 0, 0, 0, 0, 0};
    // 目标位置
    float JntTarge[6] = {0, 0, 0, -5.0, 0, 0};
    float v_output[6];

    // std::cout << "init successful " << std::endl;
    // if (mt.switchControlMode(motors[3], motor_head::MIT_MODE))
    //     std::cout << "Switch to MIT_MODE Success" << std::endl;
    // mt.save_motor_param(motors[3]);
    // // if (mt.switchControlMode(motors[1], motor_head::MIT_MODE))
    //     std::cout << "Switch to MIT_MODE Success" << std::endl;
    // mt.save_motor_param(motors[1]);
    // if (mt.switchControlMode(motors[2], motor_head::MIT_MODE))
    //     std::cout << "Switch to MIT_MODE Success" << std::endl;
    // mt.save_motor_param(motors[2]);
    // if (mt.switchControlMode(motors[3], motor_head::MIT_MODE))
    //     std::cout << "Switch to MIT_MODE Success" << std::endl;
    // mt.save_motor_param(motors[3]);
    sleep(1);
    // 设置仿真参数
    float tf = 5.0;
    float dt = 0.002;
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

    // mt.set_zero_position(motors[n]);

    double des_pos[6] = {0, 0, 0, 0, 0.0, 0};

    // 上一时刻关节位置初始化
    for (int i = 0; i < 6; i++)
    {
        Jnt_old[i] = JntCurrent[i];
    }
        

    JntCurrent[n] = motors[n].Get_Position() * 180 / M_PI;
    Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[n], JntTarge[n], tf, a0345[n]);

    while (tt <= tf)
    {
        g_iPlanFirstCircle_0++;

        // des_pos[n] = motors[n].Get_Position() + g_iPlanFirstCircle_0 * pos_delta;
        tt = g_iPlanFirstCircle_0 * dt;                               // 规划次数*采样时间
        Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]); // 得到角度制
        qq = qq_data[n][0];
        qv = qq_data[n][1] / 180 * M_PI;
        qa = qq_data[n][2];

        // std::cout << "tt :" << tt << "qq :" << qq << std::endl;
        JntCurrent[n] = motors[n].Get_Position();
        Jnt_v[n] = motors[n].Get_Velocity();
        des_pos[n] = qq * M_PI / 180;
        float pos_err = des_pos[n] - motors[n].Get_Position();
        float motor_err = pidControllers[n].PIDCompute_err(pos_err);
        float motor_inc = des_pos[n] + motor_err;
        /* 关节闭环控制 */
        // reader.setadress(adr[3]);
        // JntCurrent[n] = reader.readPosition(reader.encoder());

        // std::cout << "JntCurrent[n]：" << JntCurrent[n] << std::endl;

        float vref = pidControllers[n].PIDCompute(des_pos[n], JntCurrent[n]);

        // float tref = speedPIDControllers[n].PIDCompute(vref, Jnt_v[n]);

        vref = qv;
        // vref = pos_delta;

        // 限幅
        // if (vref > outputLimit_v[n])
        //     ;
        // vref = outputLimit_v[n];
        // if (vref < -outputLimit_v[n])
        //     vref = -outputLimit_v[n];

        // Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n]) / 180.0 * M_PI) / (timeDelta / 1000);
        // Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
        // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
        // Jnt_v[n] = motors[n].Get_Velocity();
        // v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);

        mt.dm_control_pos_vel(motors[n], motor_inc, qv);

        // 记录上一时刻的关节角，速度
        // 打印调试

        std::cout << "t=" << tt
                  << " des_pos=" << des_pos[n] * 180 / M_PI
                  << " vref=" << vref
                  << " act_pos=" << JntCurrent[n] * 180 / M_PI

                  << std::endl;
        Jnt_old[n] = JntCurrent[n];
        Jnt_v1[n] = Jnt_v[n];

        // 记录理论关节和实际关节参数
        th_pva.push_back({qq, qv, tau_output[n]});
        joint_pva.push_back({JntCurrent[n] * 180 / M_PI, Jnt_v[n], motors[n].Get_tau()});
        auto &th = th_pva.back();
        auto &jt = joint_pva.back();
        file_th << th[0] << "," << th[1] << "," << th[2] << "\n";
        file_jt << jt[0] << "," << jt[1] << "," << jt[2] << "\n";
        usleep(2000);
    }
   

    file_th.close();
    file_jt.close();
    for (int i = 0; i < 5; i++)
    {

        mt.disable(motors[i]);
        usleep(10000);
    }
    return 0;
}
