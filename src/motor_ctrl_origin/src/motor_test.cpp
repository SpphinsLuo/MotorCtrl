// #include <cstdio>
// #include "ros/ros.h"
// #include <motor_ctrl/PID_ctrl.h>
// #include <motor_ctrl/encoder_read.h>
// #include "motor_ctrl/all_motor.h"
// #include "motor_ctrl/Interp5rdPoly.h"

// motor_head::Motor motors[12] = {

//     motor_head::Motor(motor_head::DM10010, 0x01, 0x21),  // 髋并联
//     motor_head::Motor(motor_head::DM10010, 0x02, 0x22),  // 髋并联
//     motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰
//     motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋滚转
//     motor_head::Motor(motor_head::PA105, 0x05, 0x25),    // 膝
//     motor_head::Motor(motor_head::HA60, 0x06, 0x26),     // 踝滚转

//     motor_head::Motor(motor_head::DM10010, 0x07, 0x27),  // 髋并联
//     motor_head::Motor(motor_head::DM10010, 0x08, 0x28),  // 髋并联
//     motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰
//     motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋滚转
//     motor_head::Motor(motor_head::PA105, 0x11, 0x31),    // 膝
//     motor_head::Motor(motor_head::HA60, 0x12, 0x32),     // 踝滚转

// };

// std::shared_ptr<SerialPort> serial;
// motor_head::Motor_Control mt(serial);



// int main(int argc, char **argv)
// {
//   encoder_read reader;

//   serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
//   mt = motor_head::Motor_Control(serial);

//   // 添加电机
//   for (int i = 0; i < 6; i++)
//   {
//     mt.addMotor(&motors[i]);
//     if (i == 4)
//     {
//       mt.elmo_set_pos_mode(motors[4].GetSlaveId());
//       mt.elmo_enable(motors[4].GetSlaveId());
//       mt.elmo_set_acc(motors[4].GetSlaveId());
//       mt.elmo_set_dec(motors[4].GetSlaveId());
//       mt.elmo_set_vel(motors[4].GetSlaveId());
//     }
//     else
//     {
//       mt.enable(motors[i]);
//     }
//     usleep(10000);
//   }

//   std::cout << "addMotor successful" << std::endl;

//   // 定义各个关节PID 参数
//   // 位置环PID
//   float kp[6] = {0.0, 6.6, 90.0, 4.0, 1.0, 3.0};
//   float ki[6] = {0.0, 1.25, 0.05, 0.1, 0.05, 0.0};
//   float kd[6] = {0.1, 0.4, 0.3, 0.02, 0.3, 0.0};
//   // 速度环PID
//   float kp_v[6] = {0.0, 6.6, 90.0, 4.0, 1.0, 160.0};
//   float ki_v[6] = {0.0, 1.25, 0.05, 0.1, 0.05, 3.0};
//   float kd_v[6] = {0.1, 0.4, 0.3, 0.02, 0.3, 0.0};
//   // 限幅
//   float integralLimit[6] = {5, 5, 5, 2, 5, 5};     /* 积分限幅 */
//   float outputLimit_v[6] = {3, 3, 3, 2, 3, 2};     /* 速度限幅 */
//   float outputLimit_tau[6] = {3, 3, 3, 2, 3, 0.5}; /* 力矩限幅 */
//   float timeDelta = 10.0;
//   float errorThreshold = 0.05;
//   float v_outputs_l[6];
//   float v_outputs_r[6];
//   float tau_output_l[6];
//   float tau_output_r[6];
//   // PID肩2{3.6,0.6,0.32} 10s
//   // PID{6.6,1.25,0.4} 5s 或 {9.0, 0.9 0.05}

//   // 初始化pid参数
//   //  位置环PID
//   PIDController pidControllers[6];
//   // 使用循环初始化每个 PID 控制器对象
//   for (int i = 1; i < 6; ++i)
//   {
//     if (i != 4)
//       {
//         pidControllers[i] = PIDController(
//             kp[i],            // kp
//             ki[i],            // ki
//             kd[i],            // kd
//             integralLimit[i], // integralLimit
//             outputLimit_v[i], // outputLimit
//             timeDelta,        // timeDelta
//             errorThreshold);
//       }
//   }
//   // 速度环PID
//   PIDController speedPIDControllers[6];
//   for (int i = 1; i < 6; ++i)
//   {
//     if (i != 4)
//     {
//       speedPIDControllers[i] = PIDController(
//           kp_v[i],            // kp
//           ki_v[i],            // ki
//           kd_v[i],            // kd
//           integralLimit[i],   // integralLimit
//           outputLimit_tau[i], // outputLimit
//           timeDelta,          // timeDelta
//           errorThreshold);
//     }
//   }

//   // 编码器地址
//   uint8_t adr_l[4] = {0x02, 0x04, 0x07, 0x08}; // 髋俯仰 髋滚转 膝俯仰 踝俯仰
//   uint8_t adr_r[4] = {0x11, 0x13, 0x14, 0x16}; // 髋俯仰 髋滚转 膝俯仰 踝俯仰
//   float JntCurrent_l[6] = {0, 0, 0, 0, 0, 0};
//   float JntCurrent_r[6] = {0, 0, 0, 0, 0, 0};
//   // 目标位置
//   float JntTarge_l[6] = {0, -30.0, 90.0, 60.0, 0, -50.0};
//   float JntTarge_r[6] = {0, -30.0, 90.0, 60.0, 0, -50.0};
//   float v_output_l[6];
//   float v_output_r[6];
//   /*获取机械腿各个关节位置*/
//   // 左腿
//   // 髋偏航 髋俯仰 髋滚转 膝俯仰 踝俯仰 踝滚转
//   for (int i = 0; i < 6; i++)
//   {
//     if (i != 0 && i != 5)
//     {
//       reader.setadress(adr_l[i - 1]);
//       JntCurrent_l[i] = reader.readPosition(reader.encoder());
//       std::cout << "joint" << i + 1 << " " << JntCurrent_l[i - 1] << std::endl;
//       usleep(100);
//     }
//     else if (i == 0)
//     {
//       JntCurrent_l[0] = motors[0].Get_Position() / M_PI * 180;
//     }
//     else if (i == 5)
//     {
//       JntCurrent_l[5] = motors[5].Get_Position() / M_PI * 180;
//     }
//   }

//   // 右腿
//   // 髋偏航 髋俯仰 髋滚转 膝俯仰 踝俯仰 踝滚转
//   for (int i = 0; i < 6; i++)
//   {
//     if (i != 0 && i != 5)
//     {
//       reader.setadress(adr_r[i - 1]);
//       JntCurrent_r[i] = reader.readPosition(reader.encoder());
//       std::cout << "joint" << i + 1 << " " << JntCurrent_r[i - 1] << std::endl;
//       usleep(100);
//     }
//     else if (i == 0)
//     {
//       JntCurrent_r[0] = motors[6].Get_Position() / M_PI * 180;
//     }
//     else if (i == 5)
//     {
//       JntCurrent_r[5] = motors[11].Get_Position() / M_PI * 180;
//     }
//   }

//   std::cout << "init successful " << std::endl;
//   sleep(1);
//   // 设置仿真参数
//   float tf = 10.0;
//   float dt = 0.002;
//   float tt = 0.0;
//   float targetAngle;
//   std::vector<std::vector<float>> jntData; // 用于存储关节数据
//   float Jnttarge = 0.0;
//   // 关节运动
//   int JNT_NUM = 6;
//   float a0345_l[JNT_NUM][4];
//   float a0345_r[JNT_NUM][4];
//   float qq_data_l[JNT_NUM][3];
//   float qq_data_r[JNT_NUM][3];
//   int NodeNum = (tf / dt) + 1;
//   int g_iPlanFirstCircle_0 = 0; /* 规划次数 */
//   int n = ;                    // 设定读取的关节,n = 0、1、2、5
//   /************************单关节精度实验（位置、速度、加速度）五次多项式*******************************/
//   float qq_l;
//   float qv_l;
//   float qa_l;
//   float qq_r;
//   float qv_r;
//   float qa_r;
//   float Jnt_old_l[6]; // 上一时刻的位置
//   float Jnt_old_r[6]; // 上一时刻的位置
//   float Jnt_v_l[6];
//   float Jnt_v_r[6];
//   float Jnt_v1_l[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 上一时刻的速度
//   float Jnt_v1_r[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 上一时刻的速度
//   float Jnt_a_l[6];
//   float Jnt_a_r[6];
//   float cutoff_frequency = 80.0 / tf;        // 截止频率，单位rad/s
//   std::vector<std::vector<float>> th_pva;    // 用于存储理论关节数据
//   std::vector<std::vector<float>> joint_pva; // 用于存储实际关节数据 */

//   // 上一时刻关节位置初始化
//   for (int i = 0; i < 6; i++)
//   {
//     Jnt_old_l[i] = JntCurrent_l[i];
//   }
//   for (int i = 0; i < 6; i++)
//   {
//     Jnt_old_r[i] = JntCurrent_r[i];
//   }
//   Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent_l[n], JntTarge_l[n], tf, a0345_l[n]);
//   Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent_r[n], JntTarge_r[n], tf, a0345_r[n]);
//   while (tt <= tf)
//   {
//     g_iPlanFirstCircle_0++;
//     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
//     Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345_l[n], qq_data_l[n]);
//     Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345_r[n], qq_data_r[n]);
//     qq_l = qq_data_l[n][0];
//     qv_l = qq_data_l[n][1] / 180.0 * M_PI;
//     qa_l = qq_data_l[n][2];
//     qq_r = qq_data_r[n][0];
//     qv_r = qq_data_r[n][1] / 180.0 * M_PI;
//     qa_r = qq_data_r[n][2];
//     /* 关节双环闭环控制 */
//     reader.setadress(adr_l[n]);
//     reader.setadress(adr_r[n]);
//     JntCurrent_l[n] = reader.readPosition(reader.encoder());
//     JntCurrent_r[n] = reader.readPosition(reader.encoder());
//     Jnt_v_l[n] = ((JntCurrent_l[n] - Jnt_old_l[n]) / 180.0 * M_PI) / (timeDelta / 1000);
//     Jnt_v_r[n] = ((JntCurrent_r[n] - Jnt_old_r[n]) / 180.0 * M_PI) / (timeDelta / 1000);
//     // 滤波处理
//     // Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
//     Jnt_a_l[n] = (Jnt_v_l[n] - Jnt_v1_l[n]) / (timeDelta / 1000);
//     Jnt_a_r[n] = (Jnt_v_r[n] - Jnt_v1_r[n]) / (timeDelta / 1000);
//     // outputLimit_v[5] = qv;
//     v_outputs_l[n] = pidControllers[n].PIDCompute(qq_l, JntCurrent_l[n]);
//     v_outputs_r[n] = pidControllers[n].PIDCompute(qq_r, JntCurrent_r[n]);
//     tau_output_l[n] = speedPIDControllers[n].PIDCompute(v_outputs_l[n], Jnt_v_l[n]);
//     tau_output_r[n] = speedPIDControllers[n].PIDCompute(v_outputs_r[n], Jnt_v_l[n]);
//     std::cout << " " << Jnt_v_l[n] << " " << tau_output_l[n] << " " << v_outputs_l[n] << " " << qv_l << std::endl;
//     std::cout << " " << Jnt_v_r[n] << " " << tau_output_r[n] << " " << v_outputs_r[n] << " " << qv_r << std::endl;
//     mt.dm_control_mit(motors[n], 0.0, 0.0, 0.0, 0.0, tau_output[n]);
//     // 记录上一时刻的关节角，速度
//     Jnt_old_l[n] = JntCurrent_l[n];
//     Jnt_old_r[n] = JntCurrent_r[n];
//     Jnt_v1_l[n] = Jnt_v_l[n];
//     Jnt_v1_r[n] = Jnt_v_r[n];
//     // 记录理论关节和实际关节参数
//     th_pva.push_back({qq_l, qv_l, qa_l});
//     th_pva.push_back({qq_r, qv_r, qa_r});
//     joint_pva.push_back({JntCurrent_l[n], Jnt_v_l[n], Jnt_a_l[n]});
//     joint_pva.push_back({JntCurrent_r[n], Jnt_v_r[n], Jnt_a_r[n]});
//     usleep(10000);
//   }
//   mt.dm_control_mit(motors[n], 0.0, 0.0, 0.0, 0.0, -0.01);
//   // 理论关节参数
//   /*   std::cout<<"compute data:"<<std::endl;
//    for (size_t i = 0; i < th_pva.size(); ++i) {
//           // 输出理论数据
//           std::cout << th_pva[i][0] << " ";
//           // 输出实际关节参数
//           std::cout << joint_pva[i][0] << std::endl;
//       }*/
//   float error[th_pva.size()][3];
//   // 检查两个数组的大小是否相同
//   if (th_pva.size() != joint_pva.size() || th_pva[0].size() != joint_pva[0].size())
//   {
//     std::cerr << "Error: Arrays are not the same size." << std::endl;
//   }
//   // 遍历 th_pva 数组
//   std::cout << "error data:" << std::endl;
//   for (size_t i = 0; i < th_pva.size(); ++i)
//   {
//     // 计算并打印每个关节的误差
//     for (size_t j = 0; j < 1; ++j)
//     {
//       error[i][j] = th_pva[i][j] - joint_pva[i][j];
//       std::cout << error[i][j] << " " << std::endl;
//       if ((j + 1) % 3 == 0)
//       { // 每3个数值换行
//         std::cout << std::endl;
//       }
//     }
//   }
//   reader.setadress(adr[n]);
//   JntCurrent[n] = reader.readPosition(reader.encoder());
//   std::cout << "JNT: " << JntCurrent[n] << std::endl;
//   usleep(100);

//   // 复位
//   tt = 0.0;
//   g_iPlanFirstCircle_0 = 0;
//   reader.setadress(adr[n]);
//   JntCurrent[n] = reader.readPosition(reader.encoder());
//   std::cout << "JNT: " << JntCurrent[n] << std::endl;
//   JntTarge[n] = 0.0;
//   // mt.control_vel(motors[n],0.0);
//   Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[n], JntTarge[n], tf, a0345[n]);
//   while (tt <= tf)
//   {
//     g_iPlanFirstCircle_0++;
//     tt = g_iPlanFirstCircle_0 * dt; // 规划次数*采样时间
//     Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]);
//     qq = qq_data[n][0];
//     qv = qq_data[n][1] / 180.0 * M_PI;
//     qa = qq_data[n][2];
//     /* 关节双环闭环控制 */
//     reader.setadress(adr[n]);
//     JntCurrent[n] = reader.readPosition(reader.encoder());
//     Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n]) / 180.0 * M_PI) / (timeDelta / 1000);
//     // 滤波处理
//     Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
//     Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
//     outputLimit_v[5] = qv;
//     v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);
//     tau_output[n] = speedPIDControllers[n].PIDCompute(v_outputs[n], Jnt_v[n]);
//     // std::cout<<" "<<Jnt_v[n]<<" "<<tau_output[n]<<" "<<v_outputs[n]<<" "<<qv<<std::endl;
//     mt.dm_control_mit(motors[n], 0.0, 0.0, 0.0, 0.0, tau_output[n]);
//     usleep(2000);
//   }
//   mt.dm_control_mit(motors[n], 0.0, 0.0, 0.0, 0.0, -0.01);
//   /******************************************************************************/

//   // mt.mt_control_pos_vel(motors[0], 0.0 / 180.0 * M_PI, 0.5);
//   // mt.mt_control_pos_vel(motors[1], 0.0 / 180.0 * M_PI, 0.5);
//   // mt.mt_control_pos_vel(motors[2], 0.0 / 180.0 * M_PI, 0.5);
//   // sleep(2);

//   // mt.mt_control_pos_vel(motors[2], 0.0 / 180.0 * M_PI, 0.5); // 负

//   // mt.set_zero_position(motors[0]);
//   // mt.set_zero_position(motors[1]);
//   // mt.set_zero_position(motors[2]);
//   // mt.Motorevo_set_zero_position(motors[4]);
//   // if(mt.switchControlMode(motors[0], motor_head::POS_VEL_MODE))
//   //    std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
//   // if(mt.switchControlMode(motors[1], motor_head::POS_VEL_MODE))
//   //    std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
//   //    mt.save_motor_param(motors[0]);
//   //    mt.save_motor_param(motors[1]);
//   // mt.mt_control_pos_vel(motors[0], 0.0 / 180.0 * M_PI, 10); // 正
//   // mt.mt_control_pos_vel(motors[1], 0.0 / 180.0 * M_PI, 10); // 负
//   // mt.mt_control_pos_vel(motors[2], 0.0 / 180.0 * M_PI, 10);
//   // mt.Motorevo_motor_torque_position_mix_control_mode(motors[4], 0, 1, 0, 10, 5);

//   sleep(3);
//   mt.disable(motors[0]);
//   mt.disable(motors[1]);
//   mt.disable(motors[2]);
//   mt.disable(motors[4]);
//   mt.elmo_disable(motors[5].GetSlaveId());

//   return 0;
// }
