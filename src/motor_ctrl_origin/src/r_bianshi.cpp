#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>

#define SLEEP_TIME 10000
#define ROWS 1901
#define COLS 6

motor_head::Motor motors[6] = {
    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋滚转
    motor_head::Motor(motor_head::DM10010, 0x07, 0x27),  // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x08, 0x28),  // 髋并联 右  负
    motor_head::Motor(motor_head::PA105, 0x0b, 0x31),    // 膝
    motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰
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

    std::ifstream file("/home/ye/dgz_motor/src/motor_ctrl/src/bianshiguiji.txt"); // 打开文件
    if (!file.is_open())
    {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }
    double LR_info[ROWS][COLS] = {0}; // 初始化二维数组为0
    double la[ROWS][6] = {0};
    double ra[ROWS][6] = {0};
    int row = 0;
    std::string line;
    // 读取文件中的每一行
    while (std::getline(file, line) && row < ROWS)
    {
        std::istringstream iss(line); // 使用字符串流处理每一行
        double value;
        int col = 0;
        // 从行中读取数值，直到达到COLS个或行末
        while (iss >> value && col < COLS)
        {
            LR_info[row][col] = value / 1.2; // 将读取的数值存储在二维数组中
            col++;
        }
        // 检查是否读取了足够的数值
        if (col < COLS)
        {
            std::cerr << "警告：在第" << row + 1 << "行读取的数值不足" << COLS << "个，只读取了" << col << "个" << std::endl;
        }
        row++; // 移动到下一行
    }
    // 检查是否读取了所有行
    if (row < ROWS && !file.eof())
    {
        std::cerr << "警告：文件中的行数少于预期的" << ROWS << "行，实际读取了" << row << "行" << std::endl;
    }
    // 关闭文件
    file.close();
    // for (int i = 0; i < ROWS; i++)
    // {
    //     for (int j = 0; j < COLS; j++)
    //     {
    //         std::cout << LR_info[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
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
    int J = 6; // 关节数，根据你的实际改

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
        mt.enable(motors[i]);
        usleep(10000);
    }

    usleep(10000);
    std::cout << "addMotor successful" << std::endl;

    // 定义各个关节PID 参数
    // 位置环PID
    float kp[6] = {10, 25.0, 25.0, 0.45, 25.0, 1};
    float ki[6] = {0.01, 0.1, 0.1, 0.01, 0.05, 0.05};
    float kd[6] = {0.1, 1.2, 1.2, 0.015, 0.3, 0.3};

    // 速度环PID
    float kp_v[6] = {7.6, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {1.25, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.4, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[6] = {5, 5, 5, 5, 5, 5}; /* 积分限幅 */
    float outputLimit_v[6] = {2, 2, 2, 2, 2, 2}; /* 速度限幅 */
    // float outputLimit_v[6] = {3, 3, 3, 5, 3, 3};    /* 速度限幅 */
    float outputLimit_tau[6] = {5, 5, 2, 10, 3, 3}; /* 力矩限幅 */
    float timeDelta = 10.0;
    float errorThreshold = 0.05;
    float v_outputs[6];
    float tau_output[6];

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
    // 编码器地址
    uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10}; //  0x0b 0x0d

    double JntCurrent[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 目标位置
    float JntTarge[6] = {-0 * M_PI / 180, 0 * M_PI / 180, -0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180};
    // 髋并联1 髋并联2 踝俯仰 髋偏航 膝 踝滚转

    float v_output[6];

    sleep(1);
    std::cout << "-----------" << std::endl;
    // 设置仿真参数
    float tf = 19.1;
    float dt = 0.01;
    float tt = 0.0;
    // 关节运动
    int JNT_NUM = 6;
    float a0345[JNT_NUM][4];
    float qq_data[JNT_NUM][3];
    int NodeNum = (tf / dt) + 1;
    int g_iPlanFirstCircle_0 = 0; /* 规划次数 */
    // int n = 3;                    // 设定读取的关节,n = 0、1、2、5
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
    double pos_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double motor_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double motor_inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double des_pos[6] = {0, 0, 0, 0, 0.0, 0};
    double des_vel[6] = {0, 0, 0, 0, 0.0, 0};
    double des_tau[6] = {0, 0, 0, 0, 0.0, 0};
    double vref[6] = {0, 0, 0, 0, 0.0, 0};

    // for (int i = 0; i < 6; i++)
    // {
    //     if (i == 4)
    //     {
    //         reader.setadress(adr[6]);
    //         JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度
    //     }
    //     else
    //     {
    //         mt.refresh_motor_status(motors[i]);
    //         JntCurrent[i] = motors[i].Get_Position();
    //     }

    //     usleep(10000);
    // }

    for (int j = 0; j < ROWS; j++)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 6; i++)
        {
            if (i == 3)
            {

                reader.setadress(adr[6]);
                JntCurrent[i] = reader.readPosition(reader.encoder()); // 弧度

                des_pos[i] = abs(LR_info[j][i] - 0.820200 / 5);
                if (j == 0)
                {
                    Jnt_old[i] = des_pos[i];
                }
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                // des_vel[i] = (des_pos[i] - JntCurrent[i]) / 0.01;
                des_vel[i] = (des_pos[i] - Jnt_old[i]) / 0.01;

                motor_inc[i] = des_vel[i] + motor_err[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 5)
            {
                mt.motorevo_refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();
                des_pos[i] = LR_info[j][i];
                // if (j == 0)
                // {
                //     Jnt_old[i] = des_pos[i];
                // }
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                motor_inc[i] = des_pos[i] + motor_err[i];
                // des_vel[i] = (des_pos[i] - Jnt_old[i]) / 0.01;
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / 0.01;
                motor_inc[i] = LR_info[j][i];
                Jnt_old[i] = des_pos[i];
            }
            else
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i];
                // if (j == 0)
                // {
                //     Jnt_old[i] = des_pos[i];
                // }
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                motor_inc[i] = des_pos[i] + motor_err[i];
                // des_vel[i] = (des_pos[i] - Jnt_old[i]) / 0.01;
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / 0.01;
                motor_inc[i] = LR_info[j][i];
                Jnt_old[i] = des_pos[i];
            }
        }
        for (int i = 0; i < COLS; i++)
        {
            if (des_vel[i] > outputLimit_v[i])
                des_vel[i] = outputLimit_v[i];
            if (des_vel[i] < -outputLimit_v[i])
                des_vel[i] = -outputLimit_v[i];
            if (i == 3)
            {
                 mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], motor_inc[i], des_vel[i], 0, 10, 1);
            }
            else if (i == 5)
            {
                mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], motor_inc[i], des_vel[i], 0, 10, 1);
            }
            else
            {
                mt.dm_control_pos_vel(motors[i], motor_inc[i], des_vel[i]);
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
        for (int z = 0; z < J; ++z)
            file_th << des_pos[z] * 180 / M_PI << ",";
        for (int z = 0; z < J; ++z)
            file_th << des_vel[z] << ",";
        for (int z = 0; z < J; ++z)
            file_th << des_tau[z]
                    << (z + 1 == J ? "\n" : ",");

        // 写入一行实际数据（3*J 列）
        for (int z = 0; z < J; ++z)
            file_jt << JntCurrent[z] * 180 / M_PI << ",";
        for (int z = 0; z < J; ++z)
            file_jt << Jnt_v[z] << ",";
        for (int z = 0; z < J; ++z)
            file_jt << motors[z].Get_tau()
                    << (z + 1 == J ? "\n" : ",");
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(SLEEP_TIME - elapsed.count()); // 精确匹配500Hz
    }

    sleep(0.5);
    file_th.close();
    file_jt.close();
    for (int i = 0; i < 6; i++)
    {

        mt.disable(motors[i]);

        usleep(10000);
    }
    return 0;
}
