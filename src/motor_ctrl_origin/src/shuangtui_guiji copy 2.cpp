#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>
#include <sstream>

#define SLEEP_TIME 10000
#define ROWS 1301
#define COLS 12 
#define time 0.01
#define Knee_zero_l -9.36566 * M_PI / 180
#define Knee_zero_r -14.7644 * M_PI / 180

motor_head::Motor motors[12] = {

    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋偏航
    motor_head::Motor(motor_head::DM10010, 0x07, 0x27), // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x08, 0x28), // 髋并联 右  负
    motor_head::Motor(motor_head::PA105, 0x0b, 0x31),   // 膝   正值向后
    motor_head::Motor(motor_head::HA60, 0x0c, 0x32),    // 踝滚转
    motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰

    motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋偏航
    motor_head::Motor(motor_head::DM10010, 0x01, 0x21), // 髋并联 左  正
    motor_head::Motor(motor_head::DM10010, 0x02, 0x22), // 髋并联 右  负
    motor_head::Motor(motor_head::PA105, 0x05, 0x25),   // 膝  负值向后
    motor_head::Motor(motor_head::HA60, 0x06, 0x26),    // 踝滚转
    motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰

};
std::shared_ptr<SerialPort> serial1;

std::shared_ptr<SerialPort> serial2;

motor_head::Motor_Control mt(serial1); //右腿

motor_head::Motor_Control mt2(serial2); // 左腿

int main(int argc, char *argv[])
{
    // 编码器初始化//
    encoder_read reader;
    const char *serial_port = "/dev/ttyUSB0";
    reader.encoder_serial(serial_port, B2500000);
    // 电机串口初始化
    serial1 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);

    serial2 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);

    mt = motor_head::Motor_Control(serial1);

    mt2 = motor_head::Motor_Control(serial2);

    std::ifstream file("/home/ye/dgz_motor/src/motor_ctrl/src/tabu_t_res_motor.txt"); // 打开文件
    // std::ifstream file("/home/ye/dgz_motor/src/motor_ctrl/src/tabu_res_motor.txt"); // 打开文件
    if (!file.is_open())
    {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }
    // 第二到第十三列共12列
    double LR_info[ROWS][COLS] = {0};
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

        // // 第一步：跳过第一列
        if (iss >> value)
        {
            current_col++; // 成功读取第一列，计数变为1
        }
        else
        {
            std::cerr << "警告：第" << row + 1 << "行无数据，无法读取第二到第十三列" << std::endl;
            row++;
            continue;
        }

        // 第二步：读取第二到第十三列（共12列）
        while (iss >> value && current_col < 13 && col < COLS)
        // while (iss >> value && current_col < 13 && col < COLS)
        {
            current_col++; // 当前列数从2开始，直到13
            // 原处理逻辑
            if ( col == 2 || col == 3  || col == 8 || col == 5)
            {
                LR_info[row][col] = -value / 1; // 取反后存储
            }
            else
            {
                LR_info[row][col] = value / 1; // 其他列保持原逻辑
            }
            col++;
        }

        // 第三步：调换第九列（索引7）和第十列（索引8）的值
        if (col >= 9) // 确保至少读到第十列（索引8），避免越界
        {
            double temp = LR_info[row][7];     // 暂存第九列的值
            LR_info[row][7] = LR_info[row][8]; // 第十列的值赋给第九列
            LR_info[row][8] = temp;            // 原第九列的值赋给第十列
        }
        else
        {
            std::cerr << "警告：第" << row + 1 << "行列数不足，无法调换第九列和第十列" << std::endl;
        }

        // 检查是否读取到足够的12列
        if (col < COLS)
        {
            std::cerr << "警告：在第" << row + 1 << "行仅读取到" << col << "列（目标12列），当前行总列数：" << current_col << std::endl;
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

    // // ---------------------- 只打印第500-600行（对应数组索引499-599） ----------------------
    // std::cout << "\n===== 开始打印第500-600行数据 =====" << std::endl;
    // int start_row = 4400; // 第500行对应索引499
    // int end_row = 4500;   // 第600行对应索引599
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
    //         std::cout << "第" << i + 1 << "行: "; // i+1转换为实际行号（1-based）
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
    int J = 12; // 关节数，根据你的实际改

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
    for (int i = 0; i < 12; i++)
    {
        if (i < 6)
        {
            mt.addMotor(&motors[i]);
            mt.enable(motors[i]);
            usleep(10000);
        }
        else
        {
            mt2.addMotor(&motors[i]);
            mt2.enable(motors[i]);
            usleep(10000);
        }
    }
    // sleep(10);

    usleep(10000);
    std::cout << "addMotor successful" << std::endl;

    // 定义各个关节PID 参数
    // 位置环PID
    float kp[12] = {10,   25.0, 25.0, 0,  0,    25,   10,   25.0, 25.0, 0.45,  1,    25};
    float ki[12] = {0.01, 0.1,  0.1,  0,  0,    0.05, 0.01, 0.1,  0.1,  0.01,  0.05, 0.05};
    float kd[12] = {0.1,  1.2,  1.2,  0,  0,    0.3,  0.1,  1.2,  1.2,  0.015, 0.3,  0.3};
    
    // 速度环PID
    float kp_v[6] = {7.6, 7.6, 25.0, 15, 1.0, 10.0};
    float ki_v[6] = {1.25, 1.25, 0.05, 0.1, 0.05, 0.05};
    float kd_v[6] = {0.4, 0.4, 0.3, 0.02, 0.3, 0.3};
    // 限幅
    float integralLimit[12] = {5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5};     /* 积分限幅 */
    float outputLimit_v[12] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};     /* 速度限幅 */
    float outputLimit_tau[12] = {5, 5, 2, 10, 3, 3, 5, 5, 2, 10, 3, 3}; /* 力矩限幅 */
    float timeDelta = 5.0;
    float errorThreshold = 0.05;
    float v_outputs[12];
    float tau_output[12];

    PIDController pidControllers[12];
    // 使用循环初始化每个 PID 控制器对象
    for (int i = 0; i < 12; ++i)
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

    double JntCurrent[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // 目标位置
    float JntTarge[12] = {-0 * M_PI / 180, 0 * M_PI / 180, -0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, -0 * M_PI / 180, 0 * M_PI / 180, -0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180};

    float v_output[12];

    sleep(1);
    std::cout << "-----------" << std::endl;
    // 设置仿真参数
    float tf = 7.01;
    float tt = 0.01;
    // 关节运动
    int JNT_NUM = 12;

    float Jnt_old[12]; // 上一时刻的位置
    float Jnt_v[12];
    float Jnt_v1[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 上一时刻的速度
    float Jnt_a[12];
    float cutoff_frequency = 1 / tf;
    std::vector<std::vector<double>> th_pva;
    std::vector<std::vector<double>> joint_pva;
    double pos_err[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double motor_err[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double motor_inc[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double des_pos[12] = {0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double des_vel[12] = {0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double des_tau[12] = {0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double vref[12] = {0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int j = 0; j < ROWS; j++)
    {
        tt+=0.01;
        auto loop_start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < 12; i++)
        {
            if (i == 3)
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = (LR_info[j][i]) + Knee_zero_r;

                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - Jnt_old[i]) / time;
                motor_inc[i] = des_vel[i] + motor_err[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 9)
            {

                mt2.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = (LR_info[j][i]) + Knee_zero_l;

                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - Jnt_old[i]) / time;
                motor_inc[i] = des_vel[i] + motor_err[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 4)
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
            else if (i == 10)
            {
                mt2.motorevo_refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();
                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = LR_info[j][i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 5)
            {
                mt.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i] ;
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = des_pos[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i == 11)
            {
                mt2.refresh_motor_status(motors[i]);
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
            else if (i == 7)
            {
                mt2.refresh_motor_status(motors[i]);
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
            else if (i == 8)
            {
                mt2.refresh_motor_status(motors[i]);
                JntCurrent[i] = motors[i].Get_Position();
                Jnt_v[i] = motors[i].Get_Velocity();

                des_pos[i] = LR_info[j][i];
                pos_err[i] = des_pos[i] - JntCurrent[i];
                motor_err[i] = pidControllers[i].PIDCompute_err(pos_err[i]);
                des_vel[i] = (des_pos[i] - JntCurrent[i]) / time;
                motor_inc[i] = des_pos[i];
                Jnt_old[i] = des_pos[i];
            }
            else if (i < 6)
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
                mt2.refresh_motor_status(motors[i]);
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
            else if (i == 9)
            {

                mt2.Motorevo_motor_servo_mode(motors[i], des_pos[i], 120, 2, 3, 14, 0, 0);
            }
            else if (i == 4)
            {
                mt.Motorevo_motor_torque_position_mix_control_mode(motors[i], des_pos[i], des_vel[i], 0, 5, 0.5);
            }
            else if (i == 10)
            {
                mt2.Motorevo_motor_torque_position_mix_control_mode(motors[i], des_pos[i], des_vel[i], 0, 5, 0.5);
            }
            else if (i < 6)
            {
                mt.dm_control_pos_vel(motors[i], motor_inc[i], des_vel[i]);
            }
            else
            {
                mt2.dm_control_pos_vel(motors[i], motor_inc[i], des_vel[i]);
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
            file_jt << Jnt_v[z] << ",";
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
    for (int i = 0; i < 12; i++)
    {

        if (i < 6)
        {
            mt.disable(motors[i]);
        }
        else
        {
            mt2.disable(motors[i]);
        }
        usleep(10000);
    }
    return 0;
}
