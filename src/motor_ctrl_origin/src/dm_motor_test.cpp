#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>

// #define num 1

motor_head::Motor motors[4] = {

    motor_head::Motor(motor_head::DM10010, 0x01, 0x21),  // 髋并联 左  正  向前
    motor_head::Motor(motor_head::DM10010, 0x02, 0x22),  // 髋并联 右    负  向前
    motor_head::Motor(motor_head::DM10010L, 0x03, 0x23), // 踝俯仰
    motor_head::Motor(motor_head::DM10010L, 0x04, 0x24), // 髋偏航

    // motor_head::Motor(motor_head::DM10010L, 0x10, 0x30), // 髋偏航
    // motor_head::Motor(motor_head::DM10010, 0x07, 0x27),  // 髋并联 左  正
    // motor_head::Motor(motor_head::DM10010, 0x08, 0x28),  // 髋并联 右  负
    // motor_head::Motor(motor_head::DM10010L, 0x09, 0x29), // 踝俯仰

};
std::shared_ptr<SerialPort> serial;
motor_head::Motor_Control dm(serial);

int main(int argc, char **argv)
{

    serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    dm = motor_head::Motor_Control(serial);
    // 添加电机
    for (int i = 0; i < 4; i++)
    {
        dm.addMotor(&motors[i]);

        // dm.enable(motors[i]);
        usleep(10000);
    }

    // if (dm.change_motor_param(motors[num], motor_head::KP_APR, 450))
    //     std::cout << "Change KP_APR Success" << std::endl;
    // if (dm.change_motor_param(motors[num], motor_head::KP_ASR, 0.05))
    //     std::cout << "Change KP_APR Success" << std::endl;
    for (int num = 3; num < 4; num++)
    {

        // if (dm.change_motor_param(motors[num], motor_head::KP_APR, 400))
        //     std::cout << "Change KP_APR Success" << std::endl;
        dm.refresh_motor_status(motors[num]);
        std::cout << "KP_APR:" << dm.read_motor_param(motors[num], motor_head::KP_APR) << std::endl;
        std::cout << "KP_ASR:" << dm.read_motor_param(motors[num], motor_head::KP_ASR) << std::endl;
        // dm.refresh_motor_status(motors[num]);

        std::cout << motors[num].Get_Position() * 180 / M_PI << std::endl;
        // dm.save_motor_param(motors[num]);
        sleep(1);
    }
    // dm.set_zero_position(motors[0]);
    // dm.set_zero_position(motors[1]);
    // dm.set_zero_position(motors[2]);
    // dm.Motorevo_set_zero_position(motors[4]);
    // dm.dm_control_pos_vel(motors[num], 0 / 180.0 * M_PI, 0.5); // 正

    //  for (int i = 0; i < 6; i++)
    //   {
    //     if (i % 2 == 0)
    //     {

    //       dm.dm_control_pos_vel(motors[0], 30.0 / 180.0 * M_PI, 0.5);  // 正
    //       dm.dm_control_pos_vel(motors[1], -30.0 / 180.0 * M_PI, 0.5); // 负
    //     }
    //     else
    //     {

    //       dm.dm_control_pos_vel(motors[0], 0.0 / 180.0 * M_PI, 0.5); // 正
    //       dm.dm_control_pos_vel(motors[1], 0.0 / 180.0 * M_PI, 0.5); // 负
    //     }

    //     sleep(1);
    //   }
    // dm.dm_control_pos_vel(motors[0],0.0/ 180.0 * M_PI,10);
    // dm.dm_control_pos_vel(motors[1],0.0/ 180.0 * M_PI,10);
    // dm.dm_control_pos_vel(motors[2],0.0/ 180.0 * M_PI,10);
    // dm.Motorevo_motor_torque_position_mix_control_mode(motors[4],0,1,0,10,5);

    // sleep(2);
    // dm.disable(motors[0]);
    // dm.disable(motors[1]);
    // dm.disable(motors[2]);
    // dm.disable(motors[4]);

    for (int i = 0; i < 4; i++)
    {

        dm.disable(motors[i]);

        usleep(10000);
    }
    return 0;
}
