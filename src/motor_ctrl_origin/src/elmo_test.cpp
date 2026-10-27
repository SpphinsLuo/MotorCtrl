#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/all_motor.h>
#include <motor_ctrl/encoder_read.h>
#include <motor_ctrl/Interp5rdPoly.h>
#include <fstream>

#define encoder_adr 2

motor_head::Motor motors[1] = {

    // motor_head::Motor(motor_head::PA105, 0x0b, 0x31), // 膝 后为负
    motor_head::Motor(motor_head::PA105, 0x05, 0x34), // 膝 后为负

};

std::shared_ptr<SerialPort> serial;
motor_head::Motor_Control mt(serial);

int main(int argc, char **argv)
{

  // serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  // mt = motor_head::Motor_Control(serial);

  // sleep(3);
  // std::cout << "--------" << std::endl;
  // mt.addMotor(&motors[0]);
  // mt.elmo_set_pos_mode(motors[0].GetSlaveId());
  // mt.elmo_enable(motors[0].GetSlaveId());

  // mt.elmo_set_vel(motors[0].GetSlaveId());

  // mt.elmo_set_pos(motors[0].GetSlaveId(), -10); // 负 往后摆
  // mt.elmo_set_rea(motors[0].GetSlaveId());
  // mt.elmo_set_start(motors[0].GetSlaveId());

  // sleep(2);

  // mt.elmo_set_pos(motors[0].GetSlaveId(), 30); // 负 往后摆
  // mt.elmo_set_rea(motors[0].GetSlaveId());
  // mt.elmo_set_start(motors[0].GetSlaveId());

  // sleep(2);

  // mt.elmo_disable(motors[0].GetSlaveId());

  serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
  mt = motor_head::Motor_Control(serial);
  encoder_read reader;
  const char *serial_port = "/dev/ttyUSB0";
  reader.encoder_serial(serial_port, B2500000);
  std::cout << "add motor successful" << std::endl;
  sleep(1);

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
  file_th << "des_pos,des_vel,des_tau\n";
  file_jt << "pos,vel,tau\n";

  // 添加电机

  mt.addMotor(&motors[0]);
  mt.elmo_set_acc(motors[0].GetSlaveId());
  mt.elmo_set_dec(motors[0].GetSlaveId());
  mt.elmo_set_vel_mode(motors[0].GetSlaveId());
  mt.elmo_enable(motors[0].GetSlaveId());
  // mt.elmo_set_acc(motors[0].GetSlaveId());
  // mt.elmo_set_dec(motors[0].GetSlaveId());
  // mt.elmo_set_vel(motors[0].GetSlaveId());
  usleep(10000);
  // uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10};

  std::cout << "addMotor successful" << std::endl;
  // for (int i = 0; i < 100; i++)
  // {
  //   // reader.setadress(adr[6]);
  //   // std::cout << reader.readPosition(reader.encoder()) * 180 / M_PI<< std::endl; // 弧度
  //   std::cout << "mt.elmo_get_pos(motors[0].GetSlaveId()): " << mt.elmo_get_pos(motors[0].GetSlaveId()) * 180 / M_PI<< std::endl;
  //   usleep(100);
  // }

  // 定义各个关节PID 参数
  // 位置环PID
  float kp[6] = {0.45, 3.0, 25.0, 25, 1.0, 40.0};
  float ki[6] = {0.01, 0.4, 0.05, 0.7, 0.05, 1.1};
  float kd[6] = {0.015, 1.0, 0.3, 0.2, 0.3, 1.2};
  // 速度环PID
  float kp_v[6] = {10, 7.6, 25.0, 15, 1.0, 10.0};
  float ki_v[6] = {0.1, 1.25, 0.05, 0.1, 0.05, 0.05};
  float kd_v[6] = {0.1, 0.4, 0.3, 0.02, 0.3, 0.3};
  // 限幅
  float integralLimit[6] = {5, 5, 5, 5, 5, 5};    /* 积分限幅 */
  float outputLimit_v[6] = {3, 3, 3, 5, 3, 3};    /* 速度限幅 */
  float outputLimit_tau[6] = {3, 3, 2, 10, 3, 3}; /* 力矩限幅 */
  float timeDelta = 10;
  float errorThreshold = 0.02;
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

  // 编码器地址  左腿 髋俯仰 髋滚转 膝俯仰 踝俯仰  右腿 髋俯仰 髋滚转 膝俯仰 踝俯仰
  uint8_t adr[8] = {0x02, 0x04, 0x07, 0x08, 0x0B, 0x0D, 0x0E, 0x10};

  float JntCurrent[6] = {0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180};
  // 目标位置
  float JntTarge[6] = {-10 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180};
  float v_output[6];

  sleep(1);
  // 设置仿真参数
  float tf = 2.0;
  float dt = 0.01;
  float tt = 0.0;
  // 关节运动
  int JNT_NUM = 6;
  float a0345[JNT_NUM][4];
  float qq_data[JNT_NUM][3];
  int NodeNum = (tf / dt) + 1;
  int g_iPlanFirstCircle_0 = 0; /* 规划次数 */
  int n = 0;                    // 设定读取的关节,n = 0、1、2、5
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
  double del_pos[6] = {0, 0, 0, 0, 0, 0};
  double motor_err[6] = {0, 0, 0, 0, 0.0, 0};
  // 上一时刻关节位置初始化
  // mt.elmo_get_pos(motors[0].GetSlaveId());
  // mt.elmo_get_pos(motors[0].GetSlaveId());
  // mt.elmo_get_pos(motors[0].GetSlaveId());
  // std::cout << "mt.elmo_get_pos(motors[0].GetSlaveId()): " << mt.elmo_get_pos(motors[0].GetSlaveId()) << std::endl;
  // sleep(2);
  // float rel_pos = mt.elmo_get_pos(motors[0].GetSlaveId());
  // if (rel_pos == 0)
  // {
  //   mt.elmo_disable(motors[0].GetSlaveId());
  // }
  // JntCurrent[n] = rel_pos;
  // JntTarge[n] = JntTarge[n] + rel_pos;
  double Jnt_tau[6] = {0, 0, 0, 0, 0.0, 0};
  reader.setadress(adr[encoder_adr]);
  JntCurrent[n] = reader.readPosition(reader.encoder()); // 弧度
  Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[n], JntTarge[n], tf, a0345[n]);
  for (int i = 0; i < 6; i++)
  {
    Jnt_old[i] = JntCurrent[i];
  }
  while (tt <= tf)
  {
    auto loop_start = std::chrono::high_resolution_clock::now();
    g_iPlanFirstCircle_0++;
    tt = g_iPlanFirstCircle_0 * dt;                               // 规划次数*采样时间
    Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]); // 得到弧度制
    qq = qq_data[n][0];
    qv = qq_data[n][1];
    qa = qq_data[n][2];

    reader.setadress(adr[encoder_adr]);
    JntCurrent[n] = reader.readPosition(reader.encoder()); // 弧度

    motor_err[n] = qq - JntCurrent[n];
    float motor_ = pidControllers[n].PIDCompute_err(motor_err[n]);
    // float motor_inc = qv;
    float ff_gain = 1.0f; // 通过实验调整的最佳值
    float motor_inc = ff_gain * qv + motor_;
    // float motor_inc = qv + motor_ ;

    // // 限幅
    // if (motor_inc > outputLimit_v[n])
    //   motor_inc = outputLimit_v[n];
    // if (motor_inc < -outputLimit_v[n])
    //   motor_inc = -outputLimit_v[n];

    Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n])) / (timeDelta / 1000);
    Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
    // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
    // Jnt_v[n] = motors[n].Get_Velocity();
    // v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);
    // if (JntCurrent[n] * 180 / M_PI >= 30 || JntCurrent[n] * 180 / M_PI <= -30)
    // {
    //   mt.elmo_disable(motors[0].GetSlaveId());
    //   std::cout << "过限位" << std::endl;
    //   break;
    // }
    // Jnt_tau[n] = mt.elmo_get_torque(motors[0].GetSlaveId());

    mt.elmo_set_speed(motors[0].GetSlaveId(), motor_inc); // 负 往后摆
    mt.elmo_set_start(motors[0].GetSlaveId());

    // 记录上一时刻的关节角，速度
    // 打印调试

    std::cout << "t=" << tt
              << "del_pos=" << qq * 180 / M_PI
              << " motor_inc=" << motor_inc
              << " act_pos=" << JntCurrent[n] * 180 / M_PI

              << std::endl;
    Jnt_old[n] = JntCurrent[n];
    Jnt_v1[n] = Jnt_v[n];

    // 记录理论关节和实际关节参数
    th_pva.push_back({qq * 180 / M_PI, qv, tau_output[n]});
    joint_pva.push_back({JntCurrent[n] * 180 / M_PI, Jnt_v[n], Jnt_tau[n]});
    auto &th = th_pva.back();
    auto &jt = joint_pva.back();
    file_th << th[0] << "," << th[1] << "," << th[2] << "\n";
    file_jt << jt[0] << "," << jt[1] << "," << jt[2] << "\n";
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - loop_start);
    usleep(10000 - elapsed.count()); // 精确匹配500Hz
  }

  sleep(0.5);

  tt = 0.0;
  g_iPlanFirstCircle_0 = 0;
  JntTarge[0] = 0;
  reader.setadress(adr[encoder_adr]);
  JntCurrent[n] = reader.readPosition(reader.encoder());
  Interp5rdPoly_Param_P2P_Zrros_fcn(JntCurrent[n], JntTarge[n], tf, a0345[n]);

  for (int i = 0; i < 6; i++)
  {
    Jnt_old[i] = JntCurrent[i];
  }
  while (tt <= tf)
  {
    auto loop_start = std::chrono::high_resolution_clock::now();
    g_iPlanFirstCircle_0++;
    tt = g_iPlanFirstCircle_0 * dt;                               // 规划次数*采样时间
    Interp5rdPoly_Data_t_P2P_Zrros_fcn(tt, a0345[n], qq_data[n]); // 得到弧度制
    qq = qq_data[n][0];
    qv = qq_data[n][1];
    qa = qq_data[n][2];

    reader.setadress(adr[encoder_adr]);
    JntCurrent[n] = reader.readPosition(reader.encoder()); // 弧度

    motor_err[n] = qq - JntCurrent[n];
    float motor_ = pidControllers[n].PIDCompute_err(motor_err[n]);
    // float motor_inc = qv;
    float ff_gain = 1.0f; // 通过实验调整的最佳值
    float motor_inc = ff_gain * qv + motor_;
    // float motor_inc = qv + motor_ ;

    // // 限幅
    // if (motor_inc > outputLimit_v[n])
    //   motor_inc = outputLimit_v[n];
    // if (motor_inc < -outputLimit_v[n])
    //   motor_inc = -outputLimit_v[n];

    Jnt_v[n] = ((JntCurrent[n] - Jnt_old[n])) / (timeDelta / 1000);
    Jnt_v[n] = reader.lowPassFilter(Jnt_v[n], Jnt_v1[n], cutoff_frequency, dt);
    // Jnt_a[n] = (Jnt_v[n] - Jnt_v1[n]) / (timeDelta / 1000);
    // Jnt_v[n] = motors[n].Get_Velocity();
    // v_outputs[n] = pidControllers[n].PIDCompute(qq, JntCurrent[n]);
    // if (JntCurrent[n] * 180 / M_PI >= 30 || JntCurrent[n] * 180 / M_PI <= -30)
    // {
    //   mt.elmo_disable(motors[0].GetSlaveId());
    //   std::cout << "过限位" << std::endl;
    //   break;
    // }
    Jnt_tau[n] = mt.elmo_get_torque(motors[0].GetSlaveId());

    mt.elmo_set_speed(motors[0].GetSlaveId(), motor_inc); // 负 往后摆
    mt.elmo_set_start(motors[0].GetSlaveId());

    // 记录上一时刻的关节角，速度
    // 打印调试

    std::cout << "t=" << tt
              << "del_pos=" << qq * 180 / M_PI
              << " motor_inc=" << motor_inc
              << " act_pos=" << JntCurrent[n] * 180 / M_PI

              << std::endl;
    Jnt_old[n] = JntCurrent[n];
    Jnt_v1[n] = Jnt_v[n];

    // 记录理论关节和实际关节参数
    th_pva.push_back({qq * 180 / M_PI, qv, tau_output[n]});
    joint_pva.push_back({JntCurrent[n] * 180 / M_PI, Jnt_v[n], Jnt_tau[n]});
    auto &th = th_pva.back();
    auto &jt = joint_pva.back();
    file_th << th[0] << "," << th[1] << "," << th[2] << "\n";
    file_jt << jt[0] << "," << jt[1] << "," << jt[2] << "\n";
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - loop_start);
    usleep(10000 - elapsed.count()); // 精确匹配500Hz
  }

  file_th.close();
  file_jt.close();
  mt.elmo_disable(motors[0].GetSlaveId());

  return 0;
}
