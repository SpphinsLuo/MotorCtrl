#ifndef ENCODER_READ_H
#define ENCODER_READ_H

#include <cstdint>
#include <termios.h>
#include <iostream>
#include <unistd.h> // 用于定义read()和write()函数
#include <fcntl.h>  // 用于定义open()函数
#include <sys/ioctl.h>
#include <termios.h>
#include <iomanip> // 对于 std::setw 和 std::setfill
#include <errno.h>
#include <vector>
#include <cmath>  
class encoder_read
{
private:
    //float cur_angle;
    uint8_t ec_ID;
    int encoder_;
    float last_angle;
    float last_ang[7]={0};
    float change_rate[7]={0};
    float predict_ang[7]={0};
    float filter_alpha[7]={0.08,0.08,0.08,0.08,0.8,0.8,0.8};
public:
    encoder_read(/* args */);
    ~encoder_read();
    int encoder() const { return encoder_; };
    void setadress(uint8_t adr);
    float readPosition(int fd); // 读取位置
    void setzero(int pos); // 设置零点
    int encoder_serial(const char *device, speed_t baudrate);
    void encoder_serial_close(int fd);
    uint8_t calcCRC(uint8_t *buffer,uint8_t length);
    float lowPassFilter(float input, float last_output, float cutoff_freq, float dt);
    float readPosition2(int fd,int encoder_num);
    float lowPassFilter1(float input, float last_output, float alpha);
};



encoder_read::encoder_read(/* args */)
{
}

encoder_read::~encoder_read()
{
}


uint8_t encoder_read::calcCRC(uint8_t *buffer,uint8_t length)
{
        uint8_t temp = *buffer++;
        while(--length)
        {
            temp = *buffer++ ^temp;
        }
        return temp;
}
 void encoder_read::setadress(uint8_t adr)
 {
    ec_ID = adr;
 }


/* float encoder_read::readPosition(int fd)
{
    // 发送设备 ID
    ssize_t bytesWritten = write(fd, &ec_ID, 1);
    if (bytesWritten <= 0) {
        perror("Write failed");
        return -1;
    }

    // 等待数据返回
    usleep(24.5); // 短暂等待设备响应

    std::vector<uint8_t> read_buffer;
    const uint8_t DATA_FRAME_SIZE = 6;
    bool valid = false;
    float cur_angle = -1.0;

    while (!valid) {
        uint8_t receive_data;
        ssize_t bytesRead = read(fd, &receive_data, 1);

        if (bytesRead <= 0) {
            if (bytesRead == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                std::cout << "No data available. Retrying..." << std::endl;
            } else {
                perror("Read failed");
                return -1;
            }
            continue;
        }

        read_buffer.push_back(receive_data);

        if (read_buffer.size() >= DATA_FRAME_SIZE) {
            uint8_t *data_frame = read_buffer.data();
            uint8_t crc_value = calcCRC(data_frame,DATA_FRAME_SIZE);  
           // uint8_t crc_value = calcCRC(data_frame, DATA_FRAME_SIZE - 1);

            // CRC 校验
            if (crc_value != 0) {
                std::cout << "-----------------------CRC Value--------: " << static_cast<int>(crc_value)<< std::endl;
                for (int i = 0; i < 6; i++) {
                     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data_frame[i] << " ";
                }
                std::cout << std::endl;
                crc_value = 0;
                //read_buffer.clear();
                //continue;
            }

            // 数据帧头校验
            if (data_frame[0] != ec_ID || data_frame[1] != 0x00) {
                std::cout << "Invalid data frame header." << std::endl;
                //read_buffer.clear();
                //continue;
            }

            // 角度解析
            uint32_t angle = data_frame[4] << 16 | data_frame[3] << 8 | data_frame[2];
            cur_angle = angle / static_cast<float>(1 << 17) * 360.0f;

            if (cur_angle >= 180.0f) {
                cur_angle = 360.0f - cur_angle;
            } else {
                cur_angle = -cur_angle;
            }

            cur_angle = cur_angle * M_PI / 180.0f;

            valid = true;
            read_buffer.erase(read_buffer.begin(), read_buffer.begin() + DATA_FRAME_SIZE);
        }
    }

    return cur_angle;
}
 */
     float encoder_read::readPosition(int fd)
 {
    // 实际的读取逻辑
    ssize_t writebytes;
    uint8_t reBuffer[6]= {0};
    ssize_t total_readBytes = 0;
    ssize_t readBytes;
    ssize_t bytesWritten = write(fd, &ec_ID, 1);
    usleep(24.5);
   // readBytes = read(fd, reBuffer , 6);

 while (total_readBytes < sizeof(reBuffer)) {
        readBytes = read(fd, reBuffer + total_readBytes, sizeof(reBuffer) - total_readBytes);
        if (readBytes < 0) {
            perror("Read failed");
            return -1; // 读取失败
        } else if (readBytes == 0) {
            std::cout << "No more data to read." << std::endl;
            break; // 没有更多数据可读
        }
        total_readBytes += readBytes;
    }
    if (total_readBytes < sizeof(reBuffer)) {
        std::cout << "Did not read complete data." << std::endl;
        return -1; // 没有读取到预期的数据量
    }
        std::cout << std::dec; //10进制 

        //*CRC校验
    uint8_t crc_value = calcCRC(reBuffer,sizeof(reBuffer));  
/*     std::cout << "-----------------------CRC Value--------: " << static_cast<int>(crc_value)<< std::endl;
        for (int i = 0; i < 6; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)reBuffer[i] << " ";
    }
        std::cout << std::endl; */
     while (crc_value != 0) 
    {
        std::cout << "-----------------------CRC Value----------------------------: " << static_cast<int>(crc_value)<< std::endl;
        for (int i = 0; i < 6; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)reBuffer[i] << " ";
    }
        std::cout << std::endl;
        crc_value = 0;
    } 
    //*转化角度
    uint32_t angle = reBuffer[4]<< 16 | reBuffer[3]<< 8 |reBuffer[2];
    float cur_angle = angle/(float)(1<<17)*360;  
      if(cur_angle>=180)
     {
        cur_angle = 360-cur_angle;
     }
     else
     {
        cur_angle = -cur_angle;
     }    
     //转换弧度制
     cur_angle = cur_angle/180.0*M_PI;
   
   /*  std::cout<<" cur_angle"<< cur_angle/M_PI*180.0<<std::endl;
    std::cout<<" last_angle"<< last_angle/M_PI*180.0<<std::endl; */
    last_angle=cur_angle;
    //清空接收数组的
     for (int i = 0; i < 6; ++i) {
        reBuffer[i] = 0;
    } 
    return cur_angle;
} 

//crc出错后 直接返回上次度数
/* float encoder_read::readPosition2(int fd,int encoder_num)
 {
    // 实际的读取逻辑
    ssize_t writebytes;
    uint8_t reBuffer[6]= {0};
    ssize_t total_readBytes = 0;
    ssize_t readBytes;
    ssize_t bytesWritten = write(fd, &ec_ID, 1);
    usleep(24.5);
while (total_readBytes < sizeof(reBuffer)) {
        readBytes = read(fd, reBuffer + total_readBytes, sizeof(reBuffer) - total_readBytes);
        if (readBytes < 0) {
            perror("Read failed");
            return -1; // 读取失败
        } else if (readBytes == 0) {
            std::cout << "No more data to read." << std::endl;
            break; // 没有更多数据可读
        }
        total_readBytes += readBytes;
    }
    if (total_readBytes < sizeof(reBuffer)) {
        std::cout << "Did not read complete data." << std::endl;
        return -1; // 没有读取到预期的数据量
    }
        std::cout << std::dec; //10进制

        //*CRC校验
    uint8_t crc_value = calcCRC(reBuffer,sizeof(reBuffer));  
    while (crc_value != 0) 
    {
        std::cout << "--------------------------------------------CRC Value--------: " << static_cast<int>(crc_value)<< std::endl;
        crc_value = 0;
        uint32_t angle = reBuffer[4]<< 16 | reBuffer[3]<< 8 |reBuffer[2];
        float cur_angle = angle/(float)(1<<17)*360;  
        if(cur_angle>=180)
       {
          cur_angle = 360-cur_angle;
       }
        else
      {
         cur_angle = -cur_angle;
      }    
     //转换弧度制
     cur_angle = cur_angle/180.0*M_PI;
     predict_ang[encoder_num]=last_ang[encoder_num]+change_rate[encoder_num];
     //cur_angle=0.98*predict_ang[encoder_num]+0.02*cur_angle;
     //cur_angle=predict_ang[encoder_num]-0.02*cur_angle;
     cur_angle=predict_ang[encoder_num];
     change_rate[encoder_num]=cur_angle-last_ang[encoder_num];
     last_ang[encoder_num]=cur_angle;
     std::cout<<" cur_angle****"<< encoder_num+1<<" "<< cur_angle/M_PI*180.0<<std::endl;
     std::cout<<" last_angle***"<< encoder_num+1<<" "<< last_ang[encoder_num]/M_PI*180.0<<std::endl;
     
     //std::cout<<" ----------cur_angle-------"<< encoder_num+1<<" "<< cur_angle/M_PI*180.0<<std::endl;
     //std::cout<<" ----------last_angle------"<< encoder_num+1<<" "<< last_ang[encoder_num]/M_PI*180.0<<std::endl;
    //清空接收数组的
       for (int i = 0; i < 6; ++i) {
        reBuffer[i] = 0;
    } 
    
    return cur_angle;

        //last_ang[encoder_num]=last_ang[encoder_num]+change_rate[encoder_num];
        //std::cout<<" last_angle-------"<< encoder_num+1<<" "<<last_ang[encoder_num]/M_PI*180.0<<std::endl;
        //return last_ang[encoder_num]; 
    }   
    //std::cout << std::dec; //10进制
    //*转化角度
    uint32_t angle = reBuffer[4]<< 16 | reBuffer[3]<< 8 |reBuffer[2];
    float cur_angle = angle/(float)(1<<17)*360;  
      if(cur_angle>=180)
     {
        cur_angle = 360-cur_angle;
     }
     else
     {
        cur_angle = -cur_angle;
     }    
     //转换弧度制
    cur_angle = cur_angle/180.0*M_PI;
    change_rate[encoder_num]=cur_angle-last_ang[encoder_num];
    //cur_angle=lowPassFilter1(cur_angle,last_ang[encoder_num],filter_alpha[encoder_num]);
    last_ang[encoder_num]=cur_angle;
    std::cout<<" cur_angle****"<< encoder_num+1<<" "<< cur_angle/M_PI*180.0<<std::endl;
    std::cout<<" last_angle***"<< encoder_num+1<<" "<< last_ang[encoder_num]/M_PI*180.0<<std::endl;
    //清空接收数组的
     for (int i = 0; i < 6; ++i) {
        reBuffer[i] = 0;
    } 
    return cur_angle;
}
 */

/*编码器串口初始化*/
int encoder_read::encoder_serial(const char *SERIAL_PORT_1, speed_t BaudRate)
{

    encoder_ = open(SERIAL_PORT_1, O_RDWR | O_NOCTTY);
    if (encoder_ == -1)
    {
        perror("Error opening serial port");
        return -1;
    }
    struct termios config;
    tcgetattr(encoder_, &config);
    // 设置输入和输出波特率
    cfsetispeed(&config, BaudRate);
    cfsetospeed(&config, BaudRate);
    config.c_cflag &= ~PARENB;   // 无奇偶校验位，增强数据传输的可靠性
    config.c_cflag &= ~CSTOPB;   // 使用1个停止位，这是最常见的设置
    config.c_cflag &= ~CSIZE;    // 清空字符大小位
    config.c_cflag |= CS8;       // 设置8位数据位，常用设置
    config.c_cflag |= CREAD | CLOCAL; // 使能接收器，并设置为本地连接模式，不改变所有者的行设置

    //options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL); // 禁用 ICRNL

    config.c_iflag &= ~(IXON | IXOFF | IXANY| ICRNL); // 关闭软件流控制，防止串口驱动在特定数据传输时暂停
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 使用非规范模式，适用于多种自定义的输入处理
    config.c_oflag &= ~OPOST;   // 原始输出，输出处理的关闭有助于减少数据传输时的干扰
    // 应用配置，使用TCSANOW使改动即刻生效
    if (tcsetattr(encoder_, TCSANOW, &config) != 0)
    {
        perror("Error applying settings to serial port");
        return -1;
    }
    // 清空串口的输入和输出缓冲区，防止未处理的数据干扰新会话
    tcflush(encoder_, TCIOFLUSH);

    return encoder_;
}

//滤波函数
float encoder_read::lowPassFilter(float input, float last_output, float cutoff_freq, float dt) {
    float alpha = cutoff_freq * dt / (cutoff_freq * dt + 1.0);
    return alpha * input + (1.0 - alpha) * last_output;
}

float encoder_read::lowPassFilter1(float input, float last_output, float alpha) {
    return alpha * input + (1.0 - alpha) * last_output;
}






// #ifndef ENCODER_READ_H
// #define ENCODER_READ_H

// #include <cstdint>
// #include <iostream>
// #include <unistd.h> // 用于定义read()和write()函数
// #include <fcntl.h>  // 用于定义open()函数
// #include <sys/ioctl.h>
// #include <termios.h>
// #include <errno.h>
// #include <vector>
// class encoder_read
// {
// private:
//     //float cur_angle;
//     uint8_t ec_ID;
//     int encoder_;
// public:
//     encoder_read(/* args */);
//     ~encoder_read();
//     int encoder() const { return encoder_; };
//     void setadress(uint8_t adr);
//     float readPosition(int fd); // 读取位置
//     void setzero(int pos); // 设置零点
//     int encoder_serial(const char *device, speed_t baudrate);
//     void encoder_serial_close(int fd);
//     uint8_t calcCRC(uint8_t *buffer,uint8_t length);
//     float lowPassFilter(float input, float last_output, float cutoff_freq, float dt) ;
    

// };



// encoder_read::encoder_read(/* args */)
// {

// }

// encoder_read::~encoder_read()
// {
// }


// uint8_t encoder_read::calcCRC(uint8_t *buffer,uint8_t length)
// {
//         uint8_t temp = *buffer++;
//         while(--length)
//         {
//             temp = *buffer++ ^temp;
//         }
//         return temp;
    
// }
//  void encoder_read::setadress(uint8_t adr)
//  {
//     ec_ID = adr;
//  }

// float encoder_read::readPosition(int fd)
//  {
//     // 实际的读取逻辑
//     ssize_t writebytes;
//     uint8_t reBuffer[6];
//     ssize_t bytesWritten = write(fd, &ec_ID, 1);
//     ssize_t total_readBytes = 0;
//      ssize_t readBytes;
//     usleep(24.5);
// while (total_readBytes < sizeof(reBuffer)) {
//         readBytes = read(fd, reBuffer + total_readBytes, sizeof(reBuffer) - total_readBytes);
//         if (readBytes < 0) {
//             perror("Read failed");
//             return -1; // 读取失败
//         } else if (readBytes == 0) {
//             std::cout << "No more data to read." << std::endl;
//             break; // 没有更多数据可读
//         }
//         total_readBytes += readBytes;
//     }
//     if (total_readBytes < sizeof(reBuffer)) {
//         std::cout << "Did not read complete data." << std::endl;
//         return -1; // 没有读取到预期的数据量
//     }
//         std::cout << std::dec; //10进制
//         /*CRC校验*/
//     uint8_t crc_value = calcCRC(reBuffer,sizeof(reBuffer));  
    
//          while (crc_value != 0) 
//         {
//             std::cout << "--------------------------------------------CRC Value--------: " << static_cast<int>(crc_value)<< std::endl;
//             crc_value = 0;
//         }   
//            /*转化角度*/
//     uint32_t angle = reBuffer[4]<< 16 | reBuffer[3]<< 8 |reBuffer[2];
//     float cur_angle = angle/(float)(1<<17)*360;  
//       if(cur_angle>=180)
//      {
//         cur_angle = 360-cur_angle;
//      }
//      else
//      {
//         cur_angle = -cur_angle;
//      }    
//      //转换弧度制
//      //cur_angle = cur_angle/180.0*M_PI;
//      //清空接收数组
//      for (int i = 0; i < 6; ++i) {
//         reBuffer[i] = 0;
//     } 
//     return cur_angle;
// }
 


// /*编码器串口初始化*/
// int encoder_read::encoder_serial(const char *SERIAL_PORT_1, speed_t BaudRate)
// {

//     encoder_ = open(SERIAL_PORT_1, O_RDWR | O_NOCTTY);
//     if (encoder_ == -1)
//     {
//         perror("Error opening serial port");
//         return -1;
//     }

//     struct termios config;
//     tcgetattr(encoder_, &config);
//     //config.c_cc[VTIME] = 5;  // 0.5秒超时（单位：0.1秒）
//     //config.c_cc[VMIN] = 6;   // 最小读取字符数为0（配合超时）

//     // 设置输入和输出波特率
//     cfsetispeed(&config, BaudRate);
//     cfsetospeed(&config, BaudRate);

//     config.c_cflag &= ~PARENB;   // 无奇偶校验位，增强数据传输的可靠性
//     config.c_cflag &= ~CSTOPB;   // 使用1个停止位，这是最常见的设置
//     config.c_cflag &= ~CSIZE;    // 清空字符大小位
//     config.c_cflag |= CS8;       // 设置8位数据位，常用设置
//     config.c_cflag |= CREAD | CLOCAL; // 使能接收器，并设置为本地连接模式，不改变所有者的行设置

//     config.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL); // 关闭软件流控制，防止串口驱动在特定数据传输时暂停
//     config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 使用非规范模式，适用于多种自定义的输入处理
//     config.c_oflag &= ~OPOST;   // 原始输出，输出处理的关闭有助于减少数据传输时的干扰

//     // 应用配置，使用TCSANOW使改动即刻生效
//     if (tcsetattr(encoder_, TCSANOW, &config) != 0)
//     {
//         perror("Error applying settings to serial port");
//         return -1;
//     }

//     // 清空串口的输入和输出缓冲区，防止未处理的数据干扰新会话
//     tcflush(encoder_, TCIOFLUSH);

//     return encoder_;
// }

// //滤波函数
// float encoder_read::lowPassFilter(float input, float last_output, float cutoff_freq, float dt) {
//     float alpha = cutoff_freq * dt / (cutoff_freq * dt + 1.0);
//     return alpha * input + (1.0 - alpha) * last_output;
// }


#endif