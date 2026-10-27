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
    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋滚转
    motor_head::Motor(motor_head::PA105, 0x0b, 0x31),    // 膝
    motor_head::Motor(motor_head::HA60, 0x0c, 0x32),     // 踝滚转
};
std::shared_ptr<SerialPort> serial1;
std::shared_ptr<SerialPort> serial2;
motor_head::Motor_Control mt(serial1);
motor_head::Motor_Control mt2(serial2);

int main(int argc, char *argv[])
{
    // 编码器初始化//
    encoder_read reader;
    const char *serial_port = "/dev/ttyUSB0";
    reader.encoder_serial(serial_port, B2500000);
    // 电机串口初始化
    serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    serial2 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    mt = motor_head::Motor_Control(serial1);
    mt2 = motor_head::Motor_Control(serial2);

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

        if (i == 4)
        {
            mt2.addMotor(&motors[i]);
            mt2.elmo_set_acc(motors[i].GetSlaveId());
            mt2.elmo_set_dec(motors[i].GetSlaveId());
            mt2.elmo_set_vel_mode(motors[i].GetSlaveId());

            mt2.elmo_enable(motors[i].GetSlaveId());
        }
        else
        {
            mt.addMotor(&motors[i]);
            mt.enable(motors[i]);
        }
        usleep(10000);
    }

    usleep(10000);
    sleep(2);
    std::cout << "addMotor successful" << std::endl;

    // 定义各个关节PID 参数
    // 位置环PID
    float kp[6] = {25, 25.0, 25.0, 10, 0.6, 1};
    float ki[6] = {0.1, 0.1, 0.05, 0.01, 0.01, 0.05};
    float kd[6] = {1.2, 1.2, 0.3, 0.1, 0.015, 0.3};
    // 速度环PID
    float kp_v[6] = {7.6, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {1.25, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.4, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[6] = {5, 5, 5, 5, 5, 5};    /* 积分限幅 */
    float outputLimit_v[6] = {3, 3, 3, 5, 3, 3};    /* 速度限幅 */
    float outputLimit_tau[6] = {5, 5, 2, 10, 3, 3}; /* 力矩限幅 */
    float timeDelta = 10.0;
    float errorThreshold = 0.02;
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
    uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10}; //  0x0b 0x0d

    double JntCurrent[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 目标位置
    // float JntTarge[6] = {-0  , 0 , -15 , 0 , 40, 0 };
    // 髋并联1（负） 髋并联2（正） 踝俯仰(下是负) 髋偏航 膝 踝滚转

    float v_output[6];

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
    int NodeNum = (tf / dt) + 1;
    // int n = 3;                    // 设定读取的关节,n = 0、1、2、5
    /************************单关节精度实验（位置、速度、加速度）五次多项式*******************************/

    for (int a = 0; a < 3; a++)
    {
        tt = 0.0;

        int g_iPlanFirstCircle_0 = 0; /* 规划次数 */

        float JntTarge[6] = {-20 * M_PI / 180, 20 * M_PI / 180, -20 * M_PI / 180, 0 * M_PI / 180, 50 * M_PI / 180, 0 * M_PI / 180};

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
        double pos_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double motor_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double motor_inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double des_pos[6] = {0, 0, 0, 0, 0.0, 0};
        double des_vel[6] = {0, 0, 0, 0, 0.0, 0};
        double des_tau[6] = {0, 0, 0, 0, 0.0, 0};
        double vref[6] = {0, 0, 0, 0, 0.0, 0};

        // JntTarge[0] = -10 * M_PI / 180;
        // JntTarge[1] = 10 * M_PI / 180;

        for (int i = 0; i < 6; i++)
        {
            if (i == 4)
            {
                reader.setadress(adr[6]);
                JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
                Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]);
            }
            else
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
            }

            usleep(10000);
        }
        while (tt <= tf)
        {
            auto loop_start = std::chrono::high_resolution_clock::now();

            g_iPlanFirstCircle_0++;
            tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
            for (int i = 0; i < 6; i++)
            {
                Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 弧度制
                qq = qq_data[i][0];
                qv = qq_data[i][1];
                qa = qq_data[i][2];
                if (i == 4)
                {
                    reader.setadress(adr[6]);
                    JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
                    des_pos[i] = qq;
                    pos_err[i] = des_pos[i] - JntCurrent[i];
                    motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                    motor_inc[i] = qv + motor_err[i];
                }
                else
                {
                    mt.refresh_motor_status(motors[i]);
                    JntCurrent[i] = motors[i].Get_Position();
                    Jnt_v[i] = motors[i].Get_Velocity();
                    des_pos[i] = qq;
                    pos_err[i] = des_pos[i] - JntCurrent[i];
                    motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                    motor_inc[i] = qq + motor_err[i];
                }

                // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

                des_vel[i] = qv;
                vref[i] = des_vel[i];

                Jnt_old[i] = JntCurrent[i];
                Jnt_v1[i] = Jnt_v[i];
                // 记录理论关节和实际关节参数
            }

            for (int i = 0; i < 6; i++)
            {
                if (vref[i] > outputLimit_v[i])
                    vref[i] = outputLimit_v[i];
                if (vref[i] < -outputLimit_v[i])
                    vref[i] = -outputLimit_v[i];
                if (i == 4)
                {
                    mt2.elmo_set_speed(motors[i].GetSlaveId(), motor_inc[i]); // 负 往后摆
                    mt2.elmo_set_start(motors[i].GetSlaveId());
                }
                else
                {
                    mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
                }
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
            usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
        }
        sleep(0.5);
        tt = 0.0;
        g_iPlanFirstCircle_0 = 0;

        for (int i = 0; i < 6; i++)
        {
            JntTarge[i] = 0;
        }

        for (int i = 0; i < 6; i++)
        {
            if (i == 4)
            {
                reader.setadress(adr[6]);
                JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
                Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]);
            }
            else
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
            }

            usleep(10000);
        }

        while (tt <= tf)
        {
            auto loop_start = std::chrono::high_resolution_clock::now();

            g_iPlanFirstCircle_0++;
            tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
            for (int i = 0; i < 6; i++)
            {
                Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 弧度制
                qq = qq_data[i][0];
                qv = qq_data[i][1];
                qa = qq_data[i][2];
                if (i == 4)
                {
                    reader.setadress(adr[6]);
                    JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
                    des_pos[i] = qq;
                    pos_err[i] = des_pos[i] - JntCurrent[i];
                    motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                    motor_inc[i] = qv + motor_err[i];
                }
                else
                {
                    mt.refresh_motor_status(motors[i]);
                    JntCurrent[i] = motors[i].Get_Position();
                    Jnt_v[i] = motors[i].Get_Velocity();
                    des_pos[i] = qq;
                    pos_err[i] = des_pos[i] - JntCurrent[i];
                    motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                    motor_inc[i] = qq + motor_err[i];
                }

                // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

                des_vel[i] = qv;
                vref[i] = des_vel[i];

                Jnt_old[i] = JntCurrent[i];
                Jnt_v1[i] = Jnt_v[i];
                // 记录理论关节和实际关节参数
            }

            for (int i = 0; i < 6; i++)
            {
                if (vref[i] > outputLimit_v[i])
                    vref[i] = outputLimit_v[i];
                if (vref[i] < -outputLimit_v[i])
                    vref[i] = -outputLimit_v[i];
                if (i == 4)
                {
                    mt2.elmo_set_speed(motors[i].GetSlaveId(), motor_inc[i]); // 负 往后摆
                    mt2.elmo_set_start(motors[i].GetSlaveId());
                }
                else
                {
                    mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
                }
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
            usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
        }
        usleep(10000);
    }
    sleep(0.5);

    file_th.close();
    file_jt.close();
    for (int i = 0; i < 6; i++)
    {
        if (i == 4)
        {
            mt2.elmo_disable(motors[i].GetSlaveId());
        }
        else
        {
            mt.disable(motors[i]);
        }
        usleep(10000);
    }
    return 0;
}
