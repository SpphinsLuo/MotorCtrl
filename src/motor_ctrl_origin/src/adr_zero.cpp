#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <iomanip>

#define SERIAL_PORT "/dev/ttyUSB0" // 确保这是正确的串口设备文件
#define BaudRate B2500000          // 使用标准的波特率定义

int openSerialPort()
{
    // 以读写方式打开串口，非阻塞模式已去除，改为阻塞模式
    int encoder = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (encoder == -1)
    {
        perror("Error opening serial port");
        return -1;
    }

    struct termios config;
    tcgetattr(encoder, &config);

    // 设置输入和输出波特率
    cfsetispeed(&config, BaudRate);
    //cfsetospeed(&config, BaudRate);

    config.c_cflag &= ~PARENB;   // 无奇偶校验位，增强数据传输的可靠性
    config.c_cflag &= ~CSTOPB;   // 使用1个停止位，这是最常见的设置
    config.c_cflag &= ~CSIZE;    // 清空字符大小位
    config.c_cflag |= CS8;       // 设置8位数据位，常用设置
    config.c_cflag |= CREAD | CLOCAL; // 使能接收器，并设置为本地连接模式，不改变所有者的行设置

    config.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控制，防止串口驱动在特定数据传输时暂停
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 使用非规范模式，适用于多种自定义的输入处理
    config.c_oflag &= ~OPOST;   // 原始输出，输出处理的关闭有助于减少数据传输时的干扰

    // 应用配置，使用TCSANOW使改动即刻生效
    if (tcsetattr(encoder, TCSANOW, &config) != 0)
    {
        perror("Error applying settings to serial port");
        return -1;
    }

    // 清空串口的输入和输出缓冲区，防止未处理的数据干扰新会话
    tcflush(encoder, TCIOFLUSH);

    return encoder;
}

   uint8_t calcCRC(uint8_t *buffer,uint8_t length)
    {
        uint8_t temp = *buffer++;
        while(--length)
        {
            temp = *buffer++ ^temp;
        }
        return temp;
    } 

int main()
{
    int encoder = openSerialPort();
    if (encoder == -1)
    {
        return 1;
    }

        /*设置编码器地址*/
/*    uint8_t byte_sequence[]={0xC2,0xC2,0xC4,0x46,0xC8,0x4A,0x4C,0xCE,0xD0};
   uint8_t byte_adr = 0x45;
   for (int i = 0; i < 9; i++) 
    {
        write(encoder,&byte_sequence[i],1);
        usleep(25);
        
    }
        ssize_t adr_set = write(encoder,&byte_adr,1);
    if (adr_set == -1) 
    {
    perror("write");
    } 
    else 
    {
        printf("adress change successfully");
    }  
 */
        /*设置编码器零位*/
/*         uint8_t byte_send[1] = {0x85};
        uint8_t byte_zero[1] = {0x25};  */
        // uint8_t byte_send[7] = {0x02,0x83,0x04,0x86,0x07};//ID
        // uint8_t byte_zero[7]= {0xA2,0x23,0xA4,0x26,0xA7};//置零
        // {0x02, 0x04, 0x07 ,0x08,0x0B, 0x0D,0x0E,0x10}
        // uint8_t byte_send[2] = {0x04,0x85};//ID
        // uint8_t byte_zero[2]= {0xA4,0x25};//置零
        // uint8_t byte_send[8] = {0x02, 0x04, 0x07 ,0x08,0x0B, 0x0D,0x0E,0x10};//ID
        uint8_t byte_send[1] = {0x0E};//ID
        // uint8_t byte_zero[8]= {0xA2, 0xA4, 0xA7 ,0xA8,0xAB, 0xAD,0xAE,0xB0};//置零
        uint8_t byte_zero[1]= {0xAE};//置零
        for(int j = 0;j < 1;j++)
        {
            for (int i = 0; i < 10; i++) 
                {
                    
                     write(encoder,&byte_send[j],1);
                     usleep(500);
                      write(encoder,&byte_zero[j],1);
                     usleep(500);
                 }
            printf("zeropisition set successfully\n");
        }
 
    /*CRC校验*/
/*     uint8_t calcCRC(uint8_t *buffer,uint8_t length)
    {
        uint8_t temp = *buffer++;
        while(--length)
        {
            temp = *buffer++ ^temp;
        }
        return temp;
    } */
    
    close(encoder); // 关闭串口，释放资源
    return 0;
}
