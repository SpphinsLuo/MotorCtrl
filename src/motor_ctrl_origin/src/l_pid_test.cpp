#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>

#define SLEEP_TIME 10000
#define JOINT_NUM 5
#define JOINT_NUM_wei 4
#define ENCODER_ADR 6

motor_head::Motor motors[5] = {

    motor_head::Motor(motor_head::DM10010, 0x01, 0x21),  // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x02, 0x22),  // 髋并联 右  负
    motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋偏航
    // motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋偏航
    // motor_head::Motor(motor_head::PA105, 0x05, 0x25),    // 膝
    motor_head::Motor(motor_head::PA105, 0x0b, 0x31), // 膝
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
    int J = JOINT_NUM; // 关节数，根据你的实际改

    // 期望 th_pva CSV 表头
    for (int j = JOINT_NUM_wei; j < J; ++j)
        file_th << "q" << j + 1 << "_pos,";
    for (int j = JOINT_NUM_wei; j < J; ++j)
        file_th << "q" << j + 1 << "_vel,";
    for (int j = JOINT_NUM_wei; j < J; ++j)
        file_th << "q" << j + 1 << "_tau"
                << (j + 1 == J ? "\n" : ",");

    // 实际 joint_pva CSV 表头
    for (int j = JOINT_NUM_wei; j < J; ++j)
        file_jt << "q" << j + 1 << "_pos,";
    for (int j = JOINT_NUM_wei; j < J; ++j)
        file_jt << "q" << j + 1 << "_vel,";
    for (int j = JOINT_NUM_wei; j < J; ++j)
        file_jt << "q" << j + 1 << "_tau"
                << (j + 1 == J ? "\n" : ",");
    // 添加电机
    for (int i = 0; i < JOINT_NUM; i++)
    {
        mt.addMotor(&motors[i]);

        usleep(10000);
    }
    // mt.Motorevo_set_zero_position(motors[4]);

    for (int i = 0; i < JOINT_NUM; i++)
    {

        mt.enable(motors[i]);

        usleep(10000);
    }
    // for (int i = 0; i < 2; i++)
    // {
    //     mt.motorevo_refresh_motor_status(motors[4]);
    //     // sleep(0.5);
    //     std::cout << motors[4].Get_Position() * 180 / M_PI << std::endl;
    //     sleep(1);
    // }
    // 定义各个关节PID 参数
    // 位置环PID

    float kp[6] = {25, 25.0, 25.0, 10, 1, 1};
    float ki[6] = {0.1, 0.1, 0.05, 0.01, 0, 0.05};
    float kd[6] = {1.2, 1.2, 0.3, 0.1, 0, 0.3};
    // 速度环PID
    float kp_v[6] = {7.6, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {1.25, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.4, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[6] = {5, 5, 5, 5, 5, 5};   /* 积分限幅 */
    float outputLimit_v[6] = {3, 3, 3, 5, 1.5, 3};   /* 速度限幅 */
    float outputLimit_tau[6] = {3, 3, 2, 5, 2, 3}; /* 力矩限幅 */
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
    float JntTarge[6] = {0 * M_PI / 180, -0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, (-14.7644 + 20) * M_PI / 180, 0 * M_PI / 180}; //髋前提 正负
    // 右膝 正 后
    // 左膝 负 后
    float v_output[6];

    std::cout << "init successful " << std::endl;
    sleep(1);
    std::cout << "-----------" << std::endl;
    // 设置仿真参数
    float tf = 3.0;
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
    double Jnt_pos[6] = {0, 0, 0, 0, 0.0, 0};
    double Jnt_vel[6] = {0, 0, 0, 0, 0.0, 0};
    double Jnt_tau[6] = {0, 0, 0, 0, 0.0, 0};
    double vref[6] = {0, 0, 0, 0, 0.0, 0};
    float motor_v = 0;
    // 获取机械臂各个关节初始位置并进行五次多项式参数初始化
    sleep(0.5);
    // mt.dm_control_pos_vel(motors[0], 0, 0.5);
    // mt.dm_control_pos_vel(motors[1], 0, 0.5);
    // mt.dm_control_pos_vel(motors[2], 0, 0.5);

    for (int i = 0; i < JOINT_NUM; i++)
    {
        if (i == 4)
        {
            // reader.setadress(adr[ENCODER_ADR]);
            // JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
            mt.refresh_motor_status(motors[i]);
            JntCurrent[i] = motors[i].Get_Position(); //弧度
        }
        else
        {
            mt.refresh_motor_status(motors[i]);
            JntCurrent[i] = motors[i].Get_Position(); //弧度
        }

        Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
        usleep(2000);
    }

    while (tt <= tf)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();
        g_iPlanFirstCircle_0++;
        tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
        for (int i = 0; i < JOINT_NUM; i++)
        {
            Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到弧度制
            qq = qq_data[i][0];
            qv = qq_data[i][1];
            qa = qq_data[i][2];
            if (i == 4)
            {
                // reader.setadress(adr[ENCODER_ADR]);
                // JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();

                if (JntCurrent[i] > (-14.7644 + 23) * M_PI / 180)
                {
                    for (int i = 0; i < JOINT_NUM; i++)
                    {
                        file_th.close();
                        file_jt.close();

                        mt.disable(motors[i]);

                        usleep(10000);
                        return 0;
                    }
                }
                des_pos[i] = qq;
                // des_pos[i] = qq - (14.7644 * M_PI) / 180;
                // pos_err[i] = des_pos[i] - JntCurrent[i];
                des_vel[i] = qv;
                if (JntCurrent[i] == 0)
                {
                    JntCurrent[i] = Jnt_old[n];
                }
                std::cout << "JntCurrent[n]:" << JntCurrent[i] * 180 / M_PI << std::endl;
                motor_err[i] = qq - JntCurrent[n];
                float motor_ = pidControllers[n].PIDCompute_err(motor_err[i]);
                // float motor_inc = qv;
                float ff_gain = 1.0f; // 通过实验调整的最佳值
                motor_v = ff_gain * qv + motor_;
                motor_v = qv;
                // float motor_inc = qv + motor_ ;
                vref[i] =motors[i].Get_Velocity();
                Jnt_v[i] = ((JntCurrent[i] - Jnt_old[i])) / (timeDelta / 1000);
                Jnt_v[i] = reader.lowPassFilter(Jnt_v[i], Jnt_v1[i], cutoff_frequency, dt);
            }
            else
            {

                mt.refresh_motor_status(motors[i]);

                JntCurrent[i] = motors[i].Get_Position();
                if (JntCurrent[i] > 24 * M_PI / 180)
                {
                    for (int i = 0; i < JOINT_NUM; i++)
                    {
                        file_th.close();
                        file_jt.close();

                        mt.disable(motors[i]);

                        usleep(10000);
                        return 0;
                    }
                }
                des_pos[i] = qq * 180 / M_PI;
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                motor_inc[i] = des_pos[i] + motor_err[i];

                // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

                des_vel[i] = qv;
                vref[i] = des_vel[i];
                Jnt_v[i] = motors[i].Get_Velocity();

                Jnt_old[i] = JntCurrent[i];
                Jnt_v1[i] = Jnt_v[i];
            }
        }

        for (int i = 0; i < JOINT_NUM; i++)
        {
            if (vref[i] > outputLimit_v[i])
                vref[i] = outputLimit_v[i];
            if (vref[i] < -outputLimit_v[i])
                vref[i] = -outputLimit_v[i];
            if (i == 4)
            {
                // mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], des_pos[i], des_vel[i], 0, 30, 3); // 负 往后摆
                // mt.Mototevo_motor_velocity_control_mode(motors[i],motor_v,20,0,0);
                mt.Motorevo_motor_servo_mode(motors[i], des_pos[i], 120, 2, 5, 40, 5, 0.025);
            }
            else
            {
                // mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
            }
            std::cout << "t=" << tt
                      << " des_pos"
                      << "[" << i << "] : " << des_pos[i] * 180 / M_PI
                      //   << "[" << i << "] : " << des_pos[i]
                      << " vref="
                      << "[" << i << "]: " << vref[i]
                      << " act_pos="
                      << "[" << i << "]: " << JntCurrent[i] * 180 / M_PI
                      << std::endl;
        }
        // 写入一行期望数据（3*J 列）
        for (int j = JOINT_NUM_wei; j < J; ++j)
            file_th << des_pos[j] * 180 / M_PI << ",";
        // file_th << des_pos[j] << ",";
        for (int j = JOINT_NUM_wei; j < J; ++j)
            file_th << des_vel[j] << ",";
        for (int j = JOINT_NUM_wei; j < J; ++j)
            file_th << des_tau[j]
                    << (j + 1 == J ? "\n" : ",");

        // 写入一行实际数据（3*J 列）
        for (int j = JOINT_NUM_wei; j < J; ++j)
            file_jt << JntCurrent[j] * 180 / M_PI << ",";
        for (int j = JOINT_NUM_wei; j < J; ++j)
            file_jt << vref[j] << ",";
        for (int j = JOINT_NUM_wei; j < J; ++j)
            file_jt << motors[j].Get_tau()
                    << (j + 1 == J ? "\n" : ",");
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    }
    // // 复位

    // tt = 0.0;
    // g_iPlanFirstCircle_0 = 0;

    // for (int i = 0; i < JOINT_NUM; i++)
    // {
    //     JntTarge[i] = 0.0;
    // }
    // for (int i = 0; i < JOINT_NUM; i++)
    // {
    //     if (i == 4)
    //     {
    //         reader.setadress(adr[ENCODER_ADR]);
    //         JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
    //     }
    //     else
    //     {
    //         mt.refresh_motor_status(motors[i]);
    //         JntCurrent[i] = motors[i].Get_Position(); //弧度
    //     }

    //     Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[i], JntTarge[i], tf, a0345[i]); // 五次多项式参数初始化
    //     usleep(2000);
    // }

    // while (tt <= tf)
    // {
    //     auto loop_start = std::chrono::high_resolution_clock::now();

    //     g_iPlanFirstCircle_0++;
    //     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
    //     for (int i = 0; i < JOINT_NUM; i++)
    //     {
    //         Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[i], qq_data[i]); // 得到弧度制
    //         qq = qq_data[i][0];
    //         qv = qq_data[i][1];
    //         qa = qq_data[i][2];
    //         if (i == 4)
    //         {
    //             reader.setadress(adr[ENCODER_ADR]);
    //             JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
    //         //    if (JntCurrent[i] < 0 * M_PI / 180)
    //         //     {
    //         //         for (int i = 0; i < JOINT_NUM; i++)
    //         //         {

    //         //             mt.disable(motors[i]);

    //         //             usleep(10000);
    //         //         }
    //         //     }
    //             des_pos[i] = qq;
    //             if (JntCurrent[i] == 0)
    //             {
    //                 JntCurrent[i] = Jnt_old[n];
    //             }
    //             std::cout << "JntCurrent[n]:" << JntCurrent[i] * 180 / M_PI << std::endl;
    //             motor_err[i] = qq - JntCurrent[n];
    //             float motor_ = pidControllers[n].PIDCompute_err(motor_err[i]);
    //             // float motor_inc = qv;
    //             float ff_gain = 1.0f; // 通过实验调整的最佳值
    //             motor_v = ff_gain * qv + motor_;
    //             // float motor_inc = qv + motor_ ;

    //             Jnt_v[i] = ((JntCurrent[i] - Jnt_old[i])) / (timeDelta / 1000);
    //             Jnt_v[i] = reader.lowPassFilter(Jnt_v[i], Jnt_v1[i], cutoff_frequency, dt);
    //         }
    //         else
    //         {

    //             mt.refresh_motor_status(motors[i]);

    //             JntCurrent[i] = motors[i].Get_Position();
    //             des_pos[i] = qq;
    //             pos_err[i] = des_pos[i] - JntCurrent[i];
    //             motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
    //             motor_inc[i] = des_pos[i] + motor_err[i];

    //             // vref[i] = pidControllers[i].PIDCompute(des_pos[i], JntCurrent[i]);

    //             des_vel[i] = qv;
    //             vref[i] = des_vel[i];
    //             Jnt_v[i] = motors[i].Get_Velocity();

    //             Jnt_old[i] = JntCurrent[i];
    //             Jnt_v1[i] = Jnt_v[i];
    //         }
    //     }
    //     for (int i = 0; i < JOINT_NUM; i++)
    //     {
    //         if (vref[i] > outputLimit_v[i])
    //             vref[i] = outputLimit_v[i];
    //         if (vref[i] < -outputLimit_v[i])
    //             vref[i] = -outputLimit_v[i];
    //         if (i == 4)
    //         {
    //             mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], des_pos[i], motor_v, 0, 5, 0.5); // 负 往后摆
    //         }
    //         else
    //         {
    //             // mt.dm_control_pos_vel(motors[i], motor_inc[i], vref[i]);
    //         }
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
    //     for (int j = JOINT_NUM_wei; j < J; ++j)
    //         file_th << des_pos[j] * 180 / M_PI << ",";
    //     for (int j = JOINT_NUM_wei; j < J; ++j)
    //         file_th << des_vel[j] << ",";
    //     for (int j = JOINT_NUM_wei; j < J; ++j)
    //         file_th << des_tau[j]
    //                 << (j + 1 == J ? "\n" : ",");

    //     // 写入一行实际数据（3*J 列）
    //     for (int j = JOINT_NUM_wei; j < J; ++j)
    //         file_jt << JntCurrent[j] * 180 / M_PI << ",";
    //     for (int j = JOINT_NUM_wei; j < J; ++j)
    //         file_jt << Jnt_v[j] << ",";
    //     for (int j = JOINT_NUM_wei; j < J; ++j)
    //         file_jt << motors[j].Get_tau()
    //                 << (j + 1 == J ? "\n" : ",");
    //     auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    //         std::chrono::high_resolution_clock::now() - loop_start);
    //     usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    // }
    sleep(2);
    // mt.dm_control_pos_vel(motors[0], 0, 0.2);
    // mt.dm_control_pos_vel(motors[1], 0, 0.2);
    // mt.dm_control_pos_vel(motors[2], 0, 0.2);
    // sleep(2);
    file_th.close();
    file_jt.close();
    for (int i = 0; i < JOINT_NUM; i++)
    {

        mt.disable(motors[i]);

        usleep(10000);
    }
    return 0;
}
