#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/PID_ctrl.h>
#include <motor_ctrl/encoder_read.h>
#include "motor_ctrl/all_motor.h"
#include "motor_ctrl/Interp5rdPoly.h"
#include <fstream>

motor_head::Motor motors[1] = {
    motor_head::Motor(motor_head::HA60, 0x0c, 0x40), // 22.6984度的偏差
    // motor_head::Motor(motor_head::PA105, 0x05, 0x25), // 左膝 负值向后
    // motor_head::Motor(motor_head::PA105, 0x0b, 0x31),    // 右膝 正值向后
    //
    // motor_head::Motor(motor_head::HA60, 0x06, 0x26), // -1.08度偏差 
};
std::shared_ptr<SerialPort> serial;
motor_head::Motor_Control mt(serial);
// 使能前归零
int main()
{
    serial = std::make_shared<SerialPort>("/dev/ttyACM1", B921600);
    mt = motor_head::Motor_Control(serial);
    mt.addMotor(&motors[0]);
    // mt.Motorevo_set_zero_position(motors[0]);

    sleep(1);
    mt.enable(motors[0]);
    sleep(1);
    // mt.Motorevo_set_zero_position(motors[0]);

    for (int i = 0; i < 2; i++)
    {
        mt.motorevo_refresh_motor_status(motors[0]);
        // sleep(0.5);
        std::cout << motors[0].Get_Position() * 180 / M_PI << std::endl;
        sleep(1);
    }
    // mt.Motorevo_motor_torque_position_mix_control_mode(motors[0], 22 * M_PI / 180, 0, 0, 50, 1); //只能踝
    mt.motorevo_refresh_motor_status(motors[0]);
    // sleep(0.5);
    std::cout << motors[0].Get_tau() * 180 / M_PI << std::endl;
    // mt.motorevo_refresh_motor_status(motors[0]);
    // mt.Motorevo_motor_servo_mode(motors[0], -5, 120, 2, 5, 40, 5, 0.025);

    std::cout << motors[0].Get_Position() * 180 / M_PI << std::endl;
    sleep(2);
    // std::cout << motors[0].Get_Position() * 180 / M_PI << std::endl;

    // for (int i = 0; i < 5; i++)
    // {
    //     mt.motorevo_refresh_motor_status(motors[0]);
    //     sleep(0.5);
    //     std::cout << motors[0].Get_Position() * 180 / M_PI << std::endl;
    //     sleep(1);
    // }
    // sleep(2);
    // mt.disable(motors[0]);

    // // mt.Motorevo_set_zero_position(motors[0]);

    // sleep(2);

    for (int i = 0; i < 1; i++)
    {

        mt.disable(motors[i]);
        usleep(10000);
    }

    return 0;
}