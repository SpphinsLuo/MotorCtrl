#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>
#include <sstream>

#define SLEEP_TIME 10000
#define ROWS 2102
#define COLS 6 // 保持6列（第二到第七列共6列数据）
#define Knee_zero -14.7644 * M_PI / 180
#define time 0.01
motor_head::Motor motors[6] = {
    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋偏航

    motor_head::Motor(motor_head::DM10010, 0x07, 0x27), // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x08, 0x28), // 髋并联 右  负
    motor_head::Motor(motor_head::PA105, 0x0b, 0x31),   // 膝

    motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰
    motor_head::Motor(motor_head::HA60, 0x0c, 0x32),     // 踝滚转

};
std::shared_ptr<SerialPort> serial1;

motor_head::Motor_Control mt(serial1);

int main(int argc, char *argv[])
{
    // 编码器初始化//
    encoder_read reader;
    const char *serial_port = "/dev/ttyUSB0";
    reader.encoder_serial(serial_port, B2500000);
    // 电机串口初始化
    serial1 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    mt = motor_head::Motor_Control(serial1);

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
        int current_col = 0; // 记录当前读取到的列数（从1开始）

        // // 读取并跳过第一列
        // if (iss >> value)
        // {
        //     current_col++;
        // }
        // else
        // {
        //     // 行数据为空或只有不足一列，跳过当前行
        //     std::cerr << "警告：第" << row + 1 << "行数据不足，无法读取第一列" << std::endl;
        //     row++;
        //     continue;
        // }

        // 读取第二到第七列（共6列），对第二列（col=0）和第三列（col=1）取反
        while (iss >> value && current_col < 6 && col < COLS)
        {
            current_col++;
            // 核心修改：第二列（col=0）和第三列（col=1）取反
            if (col == 1 || col == 2)
            {
                LR_info[row][col] = -value ; // 取反后存储
            }
            else
            {
                LR_info[row][col] = value ; // 其他列保持原逻辑
            }
            col++;
        }

        // 检查是否读取到足够的列数
        if (col < COLS)
        {
            std::cerr << "警告：在第" << row + 1 << "行仅读取到" << col << "列（目标6列），当前行总列数：" << current_col << std::endl;
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

    //     // ---------------------- 只打印第500-600行（对应数组索引499-599） ----------------------
    // std::cout << "\n===== 开始打印第500-600行数据 =====" << std::endl;
    // int start_row = 4400;  // 第500行对应索引499
    // int end_row = 4500;    // 第600行对应索引599

    // // 边界检查：如果实际读取的行数不足，调整结束行
    // if (row <= start_row)
    // {
    //     std::cerr << "警告：实际读取行数（" << row << "）不足500行，无法打印目标范围数据" << std::endl;
    // }
    // else
    // {
    //     int actual_end = (end_row < row) ? end_row : (row - 1); // 避免越界
    //     for (int i = start_row; i <= actual_end; ++i)
    //     {
    //         std::cout << "第" << i + 1 << "行: ";  // i+1转换为实际行号（1-based）
    //         for (int j = 0; j < COLS; ++j)
    //         {
    //             std::cout << LR_info[i][j] << "\t"; // 制表符分隔列
    //         }
    //         std::cout << std::endl;
    //     }
    //     if (actual_end < end_row)
    //     {
    //         std::cerr << "警告：实际读取行数（" << row << "）不足600行，已打印到最后一行（第" << actual_end + 1 << "行）" << std::endl;
    //     }
    // }
    // std::cout << "===== 第500-600行数据打印结束 =====" << std::endl;
    // // -------------------------------------------------------------------

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
    float integralLimit[6] = {5, 5, 5, 5, 5, 5};    /* 积分限幅 */
    float outputLimit_v[6] = {2, 2, 2, 2, 2, 2};    /* 速度限幅 */
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
    uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10};

    double JntCurrent[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 目标位置
    float JntTarge[6] = {-0 * M_PI / 180, 0 * M_PI / 180, -0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180};

    float v_output[6];

    sleep(1);
    std::cout << "-----------" << std::endl;
    // 设置仿真参数
    float tf = 20;
    float dt = 0.01;
    float tt = 0.0;
    // 关节运动
    int JNT_NUM = 6;
    float a0345[JNT_NUM][4];
    float qq_data[JNT_NUM][3];
    int NodeNum = (tf / dt) + 1;
    int g_iPlanFirstCircle_0 = 0; /* 规划次数 */

    float qq;
    float qv;
    float qa;
    float Jnt_old[6]; // 上一时刻的位置
    float Jnt_v[6];
    float Jnt_v1[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 上一时刻的速度
    float Jnt_a[6];
    float cutoff_frequency = 1 / tf;
    std::vector<std::vector<double>> th_pva;
    std::vector<std::vector<double>> joint_pva;
    double pos_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double motor_err[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double motor_inc[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double des_pos[6] = {0, 0, 0, 0, 0.0, 0};
    double des_vel[6] = {0, 0, 0, 0, 0.0, 0};
    double des_tau[6] = {0, 0, 0, 0, 0.0, 0};
    double vref[6] = {0, 0, 0, 0, 0.0, 0};

    for (int j = 0; j < ROWS; j++)
    {
        auto loop_start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 6; i++)
        {
            if (i == 3)
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = (LR_info[j][i]) + Knee_zero;

                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - Jnt_old[i]) / time;
                motor_inc[i] = des_vel[i] + motor_err[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 5)
            {
                mt.motorevo_refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();

                Jnt_v[i] = motors[i].Get_Velocity();
                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;

                motor_inc[i] = LR_info[j][i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 4)
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = des_pos[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 1)
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = des_pos[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 2)
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = des_pos[i];
                Jnt_old[i] = des_pos[i];
            }
            else
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = des_pos[i];
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
                mt.Motorevo_motor_servo_mode(motors[i], des_pos[i], 120, 2, 5, 40, 5, 0.025);
            }
            else if (i == 5)
            {
                mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], des_pos[i], des_vel[i], 0, 40, 1);
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
        // 写入一行期望数据
        for (int z = 0; z < J; ++z)
            file_th << des_pos[z] * 180 / M_PI << ",";
        for (int z = 0; z < J; ++z)
            file_th << des_vel[z] << ",";
        for (int z = 0; z < J; ++z)
            file_th << des_tau[z]
                    << (z + 1 == J ? "\n" : ",");

        // 写入一行实际数据
        for (int z = 0; z < J; ++z)
            file_jt << JntCurrent[z] * 180 / M_PI << ",";
        for (int z = 0; z < J; ++z)
            file_jt << motors[z].Get_Velocity() << ",";
        for (int z = 0; z < J; ++z)
            file_jt << motors[z].Get_tau()
                    << (z + 1 == J ? "\n" : ",");
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - loop_start);
        usleep(SLEEP_TIME - elapsed.count());
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
