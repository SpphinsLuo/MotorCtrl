#include <cstdio>
#include "ros/ros.h"
#include <motor_ctrl/encoder_read.h>
#include <fstream>

int main(int argc, char *argv[])
{
    // 编码器初始化//
    encoder_read reader;
    const char *serial_port = "/dev/ttyUSB0";
    reader.encoder_serial(serial_port, B2500000);
    // 电机串口初始化
    // serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
    // mt = motor_head::Motor_Control(serial);
    // 0x02, 0x04, 0x07 ,0x08,0x0B, 0x0D,0x0E,0x10
    // 编码器地址     左腿 髋俯仰 髋滚转 膝俯仰 踝俯仰 右腿 髋俯仰 髋滚转 膝俯仰 踝俯仰
    // uint8_t adr[8] = {0x02, 0x04, 0x07 ,0x08,     0x0B, 0x0D,0x0E,0x10}; //  0x0b 0x0d
    uint8_t adr[4] = {0x07 ,0x08, 0x0E,0x10};

    double JntCurrent[4] = {0.0, 0.0, 0.0, 0.0};
    usleep(200);

    while (1)
    {   
        for (int i = 0; i < 4; i++)
        {
            
            reader.setadress(adr[i]);
        
            JntCurrent[i] = reader.readPosition(reader.encoder());
            std::cout << JntCurrent[i] * 180 / M_PI << "   ";
            usleep(10000);
        }
            std::cout << std::endl;
        sleep(1);
    }
    return 0;
}
