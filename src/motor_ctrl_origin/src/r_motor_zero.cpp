#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"

motor_head::Motor motors[4] = {

    // 右
    motor_head::Motor(motor_head::DM10010, 0x07, 0x27),  // 髋并联
    motor_head::Motor(motor_head::DM10010, 0x08, 0x28),  // 髋并联
    motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋
                                                         // motor_head::Motor(motor_head::HA60, 0x12, 0x26),     // 踝滚转

};

std::shared_ptr<SerialPort> serial1; // 左

motor_head::Motor_Control mt1(serial1);

int main(int argc, char **argv)
{
  encoder_read reader;

  serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);

  mt1 = motor_head::Motor_Control(serial1);

  //   添加电机
  for (int i = 0; i < 4; i++)
  {
    mt1.addMotor(&motors[i]);

    // mt1.set_zero_position(motors[i]);
    mt1.enable(motors[i]);
    usleep(10000);
  }
  mt1.dm_control_pos_vel(motors[0], 0, 0.5);
  mt1.dm_control_pos_vel(motors[1], 0, 0.5);
  sleep(2);
  mt1.dm_control_pos_vel(motors[2], 0, 0.5);
  sleep(2);
  mt1.dm_control_pos_vel(motors[3], 0, 0.5);
  sleep(2);

  while (1)
  {
    for (int i = 0; i < 4; i++)
    {
      mt1.refresh_motor_status(motors[i]);
      std::cout << motors[i].Get_Position() * 180 / M_PI << " ";

      usleep(10000);
    }
    std::cout << std::endl;
    sleep(1);
  }
  return 0;
}