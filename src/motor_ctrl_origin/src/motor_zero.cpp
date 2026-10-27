#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"

motor_head::Motor motors[8] = {
// 左
    motor_head::Motor(motor_head::DM10010, 0x01, 0x21),  // 髋并联
    motor_head::Motor(motor_head::DM10010, 0x02, 0x22),  // 髋并联
    motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋滚转
// 右
    motor_head::Motor(motor_head::DM10010, 0x07, 0x27),  // 髋并联
    motor_head::Motor(motor_head::DM10010, 0x08, 0x28),  // 髋并联
    motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋滚转


};

std::shared_ptr<SerialPort> serial1; // 左
std::shared_ptr<SerialPort> serial2; // 右
motor_head::Motor_Control mt1(serial1);
motor_head::Motor_Control mt2(serial2);



int main(int argc, char **argv)
{
  encoder_read reader;

  serial1 = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  serial2 = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
  mt1 = motor_head::Motor_Control(serial1);
  mt2 = motor_head::Motor_Control(serial2);

  // 添加电机
  for (int i = 0; i < 4; i++)
  {
    mt1.addMotor(&motors[i]);
    
    // mt1.set_zero_position(motors[i]);
    
    usleep(10000);
  }
  for (int i = 4; i < 8; i++)
  {
    mt2.addMotor(&motors[i]);
    
    // mt2.set_zero_position(motors[i]);
    
    usleep(10000);
  }

  for (int i = 0; i < 4; i++)
  {
      mt1.refresh_motor_status(motors[i]);
      std::cout << motors[i].Get_Position()*180/M_PI << std::endl;
    
    usleep(10000);
  }
  for (int i = 4; i < 8; i++)
  {
      mt2.refresh_motor_status(motors[i]);

      std::cout << motors[i].Get_Position()*180/M_PI << std::endl;
    
    usleep(10000);
  }

}