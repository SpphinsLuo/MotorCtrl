#ifndef __BSP_MOTOREVO_H
#define __BSP_MOTOREVO_H

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include "SerialPort.h"

#define Motor_id uint32_t

#define MOTOREVO_POSITION_MAX 12.5f
#define MOTOREVO_POSITION_MIN -12.5f
#define MOTOREVO_VELOCITY_MAX 10.0f
#define MOTOREVO_VELOCITY_MIN -10.0f
#define MOTOREVO_TORQUE_MAX 50.0f
#define MOTOREVO_TORQUE_MIN -50.0f
#define MOTOREVO_POSITION_KP_MAX 250.0f
#define MOTOREVO_POSITION_KP_MIN 0.0f
#define MOTOREVO_POSITION_KD_MAX 50.0f
#define MOTOREVO_POSITION_KD_MIN 0.0f
#define MOTOREVO_VELOCITY_KP_MAX 250.0f
#define MOTOREVO_VELOCITY_KP_MIN 0.0f
#define MOTOREVO_VELOCITY_KI_MAX 0.05f
#define MOTOREVO_VELOCITY_KI_MIN 0.0f
#define MOTOREVO_VELOCITY_KD_MAX 50.0f
#define MOTOREVO_VELOCITY_KD_MIN 0.0f

#define MOTOREVO_CONFIG_CONTROL_MODE_INDEX 0x11
#define MOTOREVO_CONFIG_ID_CONTROLLER_KP_INDEX 0x12
#define MOTOREVO_CONFIG_ID_CONTROLLER_KI_INDEX 0x13
#define MOTOREVO_CONFIG_IQ_CONTROLLER_KP_INDEX 0x14
#define MOTOREVO_CONFIG_IQ_CONTROLLER_KI_INDEX 0x15
#define MOTOREVO_CONFIG_CURRENT_DEAD_ZONE_INDEX 0x16
#define MOTOREVO_CONFIG_VELOCITY_DEAD_ZONE_INDEX 0x17
#define MOTOREVO_CONFIG_POSITION_DEAD_ZONE_INDEX 0x18
#define MOTOREVO_CONFIG_CURRENT_INTEGRAL_LIMIT_INDEX 0x19
#define MOTOREVO_CONFIG_VELOCITY_INTEGRAL_LIMIT_INDEX 0x20
#define MOTOREVO_CONFIG_TORQUE_LIMIT_INDEX 0x21
#define MOTOREVO_CONFIG_ELECTRIC_ANGLE_OFFSET_INDEX 0x22
#define MOTOREVO_CONFIG_MACHINE_ANGLE_OFFSET_INDEX 0x23
#define MOTOREVO_CONFIG_CAN_COM_THETA_MIN_INDEX 0x24
#define MOTOREVO_CONFIG_CAN_COM_THETA_MAX_INDEX 0x25
#define MOTOREVO_CONFIG_CAN_COM_VELOCITY_MIN_INDEX 0x26
#define MOTOREVO_CONFIG_CAN_COM_VELOCITY_MAX_INDEX 0x27
#define MOTOREVO_CONFIG_CAN_COM_KP_MIN_INDEX 0x28
#define MOTOREVO_CONFIG_CAN_COM_KP_MAX_INDEX 0x29
#define MOTOREVO_CONFIG_CAN_COM_KD_MIN_INDEX 0x30
#define MOTOREVO_CONFIG_CAN_COM_KD_MAX_INDEX 0x31
#define MOTOREVO_CONFIG_CAN_COM_kI_MIN_INDEX 0x32
#define MOTOREVO_CONFIG_CAN_COM_KI_MAX_INDEX 0x33
#define MOTOREVO_CONFIG_CAN_COM_TORQUE_KP_MIN_INDEX 0x34
#define MOTOREVO_CONFIG_CAN_COM_TORQUE_KP_MAX_INDEX 0x35
#define MOTOREVO_CONFIG_MOTOR_ID_INDEX 0x36
#define MOTOREVO_CONFIG_CAN_COM_TIME_OUT_INDEX 0x37
#define MOTOREVO_CONFIG_DEFAULT_POSITION_KP_INDEX 0x38
#define MOTOREVO_CONFIG_DEFAULT_POSITION_KD_INDEX 0x39
#define MOTOREVO_CONFIG_DEFAULT_VELOCITY_KP_INDEX 0x40
#define MOTOREVO_CONFIG_DEFAULT_VELOCITY_KI_INDEX 0x41
#define MOTOREVO_CONFIG_FIRMWARE_VERSION_INDEX 0x10
#define MOTOREVO_CONFIG_ACCELERATION_INDEX 0x42
#define MOTOREVO_CONFIG_TORQUE_SLOPE_INDEX 0x43
#define MOTOREVO_CONFIG_NPP_INDEX 0x44
#define MOTOREVO_CONFIG_GERA_RATIO_INDEX 0x45
#define MOTOREVO_CONFIG_TORQUE_CONSTANT_INDEX 0x46
#define MOTOREVO_CONFIG_ROTATE_DIR_INDEX 0x47
#define MOTOREVO_CONFIG_BUS_OV_LOCK_INDEX 0x49
#define MOTOREVO_CONFIG_BUS_UV_LOCK_INDEX 0x50
#define MOTOREVO_CONFIG_PHASE_OC_LOCK_INDEX 0x51
#define MOTOREVO_CONFIG_PCB_OT_LOCK_INDEX 0x52
#define MOTOREVO_CONFIG_STUCK_CURRENT_LOCK_INDEX 0x51
#define MOTOREVO_CONFIG_STUCK_CURRENT_INDEX 0x52
#define MOTOREVO_CONFIG_STUCK_VELOCITY_INDEX 0x54
#define MOTOREVO_CONFIG_STUCK_TIME_INDEX 0x55
#define MOTOREVO_CONFIG_PROTECT_SITCHS_INDEX 0x56

typedef enum
{
    Motorevo_Reset_Status,
    Motorevo_Motor_Status_Servo,
    Motorevo_Motor_Status_TPMix,
    Motorevo_Motor_Status_Velocity,
    Motorevo_Motor_Status_Torque,
    Motorevo_Motor_Status_torque4,
} Motorevo_Status;

typedef struct
{
    uint8_t id;

    struct
    {
        float position; // rad
        float velocity; // rad/s
        float torque;   // NM
    } baseData;

    struct
    {
        Motorevo_Status status; //
        float p_kp;             // kp
        float p_kd;             // kd
        float v_kp;             // kp
        float v_kd;             // kd
        float v_ki;             // ki
    } controlData;

    uint8_t temperature;
    uint8_t errorCode;
} Motorevo_DataHandleTypeDef;

/*
 * @brief 电机控制模式
 */
typedef enum Control_Mode
{
    SERVO_MODE = 1,
    POS_FORCE_MODE = 2,
    VEL_MODE = 3,
    TAU_MODE = 4,
} Control_Mode;

void Motorevo_motor_init(Motorevo_DataHandleTypeDef *device, uint8_t id, Motorevo_Status status);

void Motorevo_motor_cantx_communication(uint8_t id, uint8_t *data);

void Motorevo_motor_canrx_communication(Motorevo_DataHandleTypeDef *device, uint8_t *canrx_data);

void Motorevo_enter_motor_state(uint8_t id);

void Motorevo_enter_reset_state(uint8_t id);

void Motorevo_set_zero_position(uint8_t id);

void Motorevo_motor_servo_control_mode(
    uint8_t id,
    float set_position,
    float velocity_limit,
    float p_kp,
    float p_kd,
    float v_kp,
    float v_ki,
    float v_kd);

void Motorevo_motor_torque_position_mix_control_mode(
    uint8_t id,
    float set_position,
    float set_velocity,
    float set_torque,
    float p_kp,
    float p_kd);

void Motorevo_motor_torque_control_mode(uint8_t id, float set_torque);

void Mototevo_motor_velocity_control_mode(uint8_t id, float set_velocity, float v_kp, float v_ki, float v_kd);

void Motorevo_motor_torque_control_mode4(float set_torque1, float set_torque2, float set_torque3, float set_torque4);

void Motorevo_write_motor_config(uint8_t id, uint8_t index, uint8_t *data4);

void Motorevo_read_motor_config(uint8_t id, uint8_t index);

typedef struct
{
    uint8_t FrameHeader;
    uint8_t CMD; // 命令 0x00: 心跳
    //     0x01: receive fail 0x11: receive success
    //     0x02: send fail 0x12: send success
    //     0x03: set baudrate fail 0x13: set baudrate success
    //     0xEE: communication error 此时格式段为错误码
    //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
    uint8_t canDataLen : 6; // 数据长度
    uint8_t canIde : 1;     // 0: 标准帧 1: 扩展帧
    uint8_t canRtr : 1;     // 0: 数据帧 1: 远程帧
    uint32_t canId;         // 电机反馈的ID
    uint8_t canData[8];
    uint8_t frameEnd; // 帧尾
} CAN_Receive_Frame;

typedef struct CAN_Send_Frame
{
    uint8_t FrameHeader[2] = {0x55, 0xAA}; // 帧头
    uint8_t FrameLen = 0x1e;               // 帧长
    uint8_t CMD = 0x03;                    // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3: 非反馈CAN转发，不反馈发送状态
    uint32_t sendTimes = 1;                // 发送次数
    uint32_t timeInterval = 10;            // 时间间隔
    uint8_t IDType = 0;                    // ID类型 0：标准帧 1：扩展帧
    uint32_t canId = 0x01;                 // CAN ID 使用电机ID作为CAN ID
    uint8_t frameType = 0;                 // 帧类型 0： 数据帧 1：远程帧
    uint8_t len = 0x08;                    // len
    uint8_t idAcc = 0;
    uint8_t dataAcc = 0;
    uint8_t data[8] = {0};
    uint8_t crc = 0; // 未解析，任意值

    void modify(const Motor_id id, const uint8_t *send_data)
    {
        canId = id;
        std::copy(send_data, send_data + 8, data);
    }

} CAN_Send_Frame;

// 默认通讯串口，需要修改
SerialPort::SharedPtr serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);

/**
 * @brief 弧度转角度
 * @param rad  输入弧度值
 * @return     对应的角度值
 */
static inline double rad_to_deg(double rad)
{
    return rad * 180.0 / M_PI;
}
/**
 * @brief 角度转弧度
 * @param deg  输入角度值（°）
 * @return     对应的弧度值（rad）
 */
static inline double deg_to_rad(double deg)
{
    return deg * M_PI / 180.0;
}

/**
 * @brief 弧度每秒 (rad/s) 转 转速 RPM（revolutions per minute）
 *        1 转 = 2π 弧度
 * @param rad_per_sec  输入的弧度/秒
 * @return             对应的“转/分钟”（RPM）
 */
static inline double radps_to_rpm(double rad_per_sec)
{
    return rad_per_sec * 60.0 / (2.0 * M_PI);
}
/**
 * @brief RPM 转 弧度每秒 (rad/s)
 * @param rpm  转速（单位：转/分钟）
 * @return     弧度每秒（单位：rad/s）
 */
static inline double rpm_to_radps(double rpm)
{
    return rpm * 2.0 * M_PI / 60.0;
}
/**
 * @name           float_to_uint
 * @brief          浮点转无符号
 * @param[in]      x:浮点变量
 * @param[in]      x_min:最小值
 * @param[in]      x_max:最大值
 * @param[in]      bits:长度
 * @retval
 */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;

    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @name           uint16_to_float
 * @brief          无符号16位转浮点
 * @param[in]      x:无符号变量
 * @param[in]      x_min:最小值
 * @param[in]      x_max:最大值
 * @param[in]      bits:长度
 * @retval
 */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;

    return offset * x / span + x_min;
}

/**
 * @name           Motorevo_motor_init
 * @brief          电机句柄初始化函数
 * @param[in]      device:电机句柄
 * @param[in]      id:初始化id
 * @param[in]      status:初始化状态
 * @retval
 */
void Motorevo_motor_init(Motorevo_DataHandleTypeDef *device, uint8_t id, Motorevo_Status status)
{
    device->id = id;
    device->controlData.status = status;
    device->controlData.p_kp = 0;
    device->controlData.p_kd = 0;
    device->controlData.v_kp = 0;
    device->controlData.v_ki = 0;
    device->controlData.v_kd = 0;

    device->baseData.position = 0;
    device->baseData.torque = 0;
    device->baseData.velocity = 0;

    device->errorCode = 0;
    device->temperature = 0;
}

/**
 * @name           Motorevo_motor_canrx_communication
 * @brief          can接收函数
 * @note           CAN 标准帧，帧ID为Motor ID, 波特率为1Mhz
 * @param[in]      id:设备id
 * @param[in]      data:can发送报文
 * @retval
 */
void Motorevo_motor_canrx_communication(Motorevo_DataHandleTypeDef *device)
{
    CAN_Receive_Frame receive_data{};
    serial_->recv((uint8_t *)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));
    uint8_t *canrx_data = receive_data.canData;
    switch (canrx_data[0])
    {
    case Motorevo_Reset_Status:
    {
        device->controlData.status = Motorevo_Reset_Status;
        std::cout << "Motorevo_Reset_Status" << std::endl;
    }
    break;
    case Motorevo_Motor_Status_Servo:
    {
        device->controlData.status = Motorevo_Motor_Status_Servo;
        std::cout << "Motorevo_Motor_Status_Servo" << std::endl;
    }
    break;
    case Motorevo_Motor_Status_TPMix:
    {
        device->controlData.status = Motorevo_Motor_Status_TPMix;
        std::cout << "Motorevo_Motor_Status_TPMix" << std::endl;
    }
    break;
    case Motorevo_Motor_Status_Velocity:
    {
        device->controlData.status = Motorevo_Motor_Status_Velocity;
        std::cout << "Motorevo_Motor_Status_Velocity" << std::endl;
    }
    break;
    case Motorevo_Motor_Status_Torque:
    {
        device->controlData.status = Motorevo_Motor_Status_Torque;
        std::cout << "Motorevo_Motor_Status_Torque" << std::endl;
    }
    break;
    case Motorevo_Motor_Status_torque4:
    {
        device->controlData.status = Motorevo_Motor_Status_torque4;
        std::cout << "Motorevo_Motor_Status_torque4" << std::endl;
    }
    break;
    default:
    {
    }
    }

    float t_position = (canrx_data[1] << 8) | canrx_data[2];
    float t_velocity = (canrx_data[3] << 4) | ((canrx_data[4] >> 4) & 0xF);
    float t_torque = ((canrx_data[4] & 0xF) << 8) | canrx_data[5];

    device->baseData.position = uint16_to_float(t_position, MOTOREVO_POSITION_MIN, MOTOREVO_POSITION_MAX, 16);
    device->baseData.velocity = uint16_to_float(t_velocity, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 12);
    device->baseData.torque = uint16_to_float(t_torque, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 12);

    device->errorCode = canrx_data[6];
    device->temperature = canrx_data[7];
}

/**
 * @name           Motorevo_motor_cantx_communication
 * @brief          can发送函数
 * @note           CAN 标准帧，帧ID为Motor ID, 波特率为1Mhz
 * @param[in]      id:设备id
 * @param[in]      data:can发送报文
 * @retval
 */
void Motorevo_motor_cantx_communication(uint8_t id, uint8_t *data)
{
    CAN_Send_Frame motorevo_tx_message;
    // uint32_t send_mail_box;


    motorevo_tx_message.modify(id, data);

    serial_->send((uint8_t *)&motorevo_tx_message, sizeof(CAN_Send_Frame));
}

/**
 * @name           Motorevo_enter_motor_state
 * @brief          设置电机为运行模式
 * @note           在torrque4 mode下帧id为100
 * @param[in]      id:设备id
 * @retval         none
 */
void Motorevo_enter_motor_state(uint8_t id)
{
    uint8_t can_send_data[8];

    for (int i = 0; i < 7; i++)
    {
        can_send_data[i] = 0xFF;
    }
    can_send_data[7] = 0xFC;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Motorevo_enter_reset_state
 * @brief          设置电机为休眠模式
 * @note           在torrque4 mode下帧id为100
 * @param[in]      id:设备id
 * @retval         none
 */
void Motorevo_enter_reset_state(uint8_t id)
{
    uint8_t can_send_data[8];

    for (int i = 0; i < 7; i++)
    {
        can_send_data[i] = 0xFF;
    }
    can_send_data[7] = 0xFD;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Motorevo_set_zero_position
 * @brief          设置电机零点
 * @note           在torrque4 mode下帧id为100
 * @param[in]      id:设备id
 * @retval         none
 */
void Motorevo_set_zero_position(uint8_t id)
{
    uint8_t can_send_data[8];

    for (int i = 0; i < 7; i++)
    {
        can_send_data[i] = 0xFD;
    }
    can_send_data[7] = 0xFE;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Motorevo_motor_servo_control_mode
 * @brief          形参参数均需要通过CAN通信的方式设定，角度死区，力限保护参数需要在上位机设置
 * @param[in]      id:设备id
 * @param[in]      set_position:目标位置, rad, [-12.5 12.5]
 * @param[in]      velocity_limit:速度限幅, rad/s, [-10 10]
 * @param[in]      p_kp:位置环kp,8bits,[0.0 250.0]
 * @param[in]      p_kd:位置环kd,8bits,[0.0 50.0]
 * @param[in]      v_kp:速度环kp,8bits,[0.0 250.0]
 * @param[in]      v_ki:速度环ki,8bits,[0.0 250.0]
 * @param[in]      v_kd:速度环kd,8bits,[0.0 0.05]
 * @retval         none
 */
void Motorevo_motor_servo_control_mode(
    uint8_t id,
    float set_position,
    float velocity_limit,
    float p_kp,
    float p_kd,
    float v_kp,
    float v_ki,
    float v_kd)
{
    uint8_t can_send_data[8] = {0};
    float set_position_rad = deg_to_rad(set_position);
    float velocity_limit_radps = rpm_to_radps(velocity_limit);
    uint16_t t_p = float_to_uint(set_position_rad, MOTOREVO_POSITION_MIN, MOTOREVO_POSITION_MAX, 16);
    uint8_t t_v_limit = float_to_uint(velocity_limit_radps, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 8);
    uint8_t t_p_kp = float_to_uint(p_kp, MOTOREVO_POSITION_KP_MIN, MOTOREVO_POSITION_KP_MAX, 8);
    uint8_t t_p_kd = float_to_uint(p_kd, MOTOREVO_POSITION_KD_MIN, MOTOREVO_POSITION_KD_MAX, 8);
    uint8_t t_v_kp = float_to_uint(v_kp, MOTOREVO_VELOCITY_KP_MIN, MOTOREVO_VELOCITY_KP_MAX, 8);
    uint8_t t_v_ki = float_to_uint(v_ki, MOTOREVO_VELOCITY_KI_MIN, MOTOREVO_VELOCITY_KI_MAX, 8);
    uint8_t t_v_kd = float_to_uint(v_kd, MOTOREVO_VELOCITY_KD_MIN, MOTOREVO_VELOCITY_KD_MAX, 8);

    can_send_data[0] = t_p >> 8;
    can_send_data[1] = t_p & 0xFF;
    can_send_data[2] = t_v_limit;
    can_send_data[3] = t_p_kp;
    can_send_data[4] = t_p_kd;
    can_send_data[5] = t_v_kp;
    can_send_data[6] = t_v_kd;
    can_send_data[7] = t_v_ki;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Motorevo_motor_torque_position_mix_control_mode
 * @brief          使用 CAN 控制电机在力位混合模式（P-T-M Mode）运行
 * @note           形参参数均需要通过CAN通信的方式设定，力限保护参数需要在上位机设置
 * @param[in]      id:设备id
 * @param[in]      set_position:目标位置,12bits, rad, [-12.5 12.5]
 * @param[in]      set_velocity:目标速度,12bits, rad/s, [-10 10]
 * @param[in]      set_torque:目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      p_kp:位置环kp,12bits,[0.0 250.0]
 * @param[in]      p_kd:位置环kd,8bits,[0.0 50.0]
 * @retval         none
 */
void Motorevo_motor_torque_position_mix_control_mode(
    uint8_t id,
    float set_position,
    float set_velocity,
    float set_torque,
    float p_kp,
    float p_kd)
{
    uint8_t can_send_data[8] = {0};
    float set_position_rad = deg_to_rad(set_position);
    float set_velocity_radps = rpm_to_radps(set_velocity);

    uint16_t t_p = float_to_uint(set_position_rad, MOTOREVO_POSITION_MIN, MOTOREVO_POSITION_MAX, 16);
    uint16_t t_v = float_to_uint(set_velocity_radps, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 12);
    uint16_t t_t = float_to_uint(set_torque, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 12);
    uint16_t t_p_kp = float_to_uint(p_kp, MOTOREVO_POSITION_KP_MIN, MOTOREVO_POSITION_KP_MAX, 12);
    uint16_t t_p_kd = float_to_uint(p_kd, MOTOREVO_POSITION_KD_MIN, MOTOREVO_POSITION_KD_MAX, 12);

    can_send_data[0] = t_p >> 8;
    can_send_data[1] = t_p & 0xFF;
    can_send_data[2] = t_v >> 4;
    can_send_data[3] = ((t_v & 0xF) << 4) | ((t_p_kp >> 8) & 0xF);
    can_send_data[4] = t_p_kp & 0xFF;
    can_send_data[5] = t_p_kd >> 4;
    can_send_data[6] = ((t_p_kd & 0xF) << 4) | ((t_t >> 8) & 0xF);
    can_send_data[7] = t_t & 0xFF;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Motorevo_motor_torque_control_mode
 * @brief          使用CAN控制电机在力矩模式（Torque Mode）运行
 * @param[in]      id:设备id
 * @param[in]      set_torque:目标力矩,12bits, rad/s, [-50 50]
 * @retval         none
 */
void Motorevo_motor_torque_control_mode(uint8_t id, float set_torque)
{
    uint8_t can_send_data[8] = {0};

    uint16_t t_t = float_to_uint(set_torque, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);

    can_send_data[0] = t_t >> 8;
    can_send_data[1] = t_t & 0xFF;
    for (int i = 2; i < 7; i++)
    {
        can_send_data[i] = 0;
    }
    can_send_data[7] = 0xAB;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Mototevo_motor_velocity_control_mode
 * @brief          使用 CAN 控制电机在速度模式（Velocity Mode）运行
 * @param[in]      id:设备id
 * @param[in]      set_velocity:目标速度,12bits, rad/s, [-10 10]
 * @param[in]      v_kp:速度环kp,12bits,[0.0 250.0]
 * @param[in]      v_ki:速度环ki,12bits,[0.0 250.0]
 * @param[in]      v_kd:速度环kd,12bits,[0.0 0.05]
 * @retval         none
 */
void Mototevo_motor_velocity_control_mode(uint8_t id, float set_velocity, float v_kp, float v_ki, float v_kd)
{
    uint8_t can_send_data[8] = {0};
    float set_velocity_radps = rpm_to_radps(set_velocity);

    uint16_t t_v = float_to_uint(set_velocity_radps, MOTOREVO_VELOCITY_MIN, MOTOREVO_VELOCITY_MAX, 16);
    uint16_t t_v_kp = float_to_uint(v_kp, MOTOREVO_VELOCITY_KP_MIN, MOTOREVO_VELOCITY_KP_MAX, 12);
    uint16_t t_v_ki = float_to_uint(v_ki, MOTOREVO_VELOCITY_KI_MIN, MOTOREVO_VELOCITY_KI_MAX, 12);
    uint16_t t_v_kd = float_to_uint(v_kd, MOTOREVO_VELOCITY_KD_MIN, MOTOREVO_VELOCITY_KD_MAX, 12);

    can_send_data[0] = t_v >> 8;
    can_send_data[1] = t_v & 0xFF;
    can_send_data[2] = t_v_kp >> 4;
    can_send_data[3] = ((t_v_kp & 0xF) << 4) | ((t_v_kd >> 8) & 0xF);
    can_send_data[4] = t_v_kd & 0xFF;
    can_send_data[5] = t_v_ki >> 4;
    can_send_data[6] = (t_v_ki & 0xF) << 4;
    can_send_data[7] = 0xAC;

    Motorevo_motor_cantx_communication(id, can_send_data);
}

/**
 * @name           Motorevo_motor_torque_control_mode4
 * @brief          使用CAN控制电机在力矩4模式（Torque4 Mode）运行
 * @note           控制帧为CAN标准帧，帧ID为100，波特率为1Mhz
 *                 用一帧CAN指令对4个电机同时控制力矩
 * @param[in]      id:设备id
 * @param[in]      set_torque1:对于(Motor ID-1 % 4)=0的电机目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      set_torque2:对于(Motor ID-1 % 4)=1的电机目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      set_torque3:对于(Motor ID-1 % 4)=2的电机目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      set_torque4:对于(Motor ID-1 % 4)=3的电机目标力矩,12bits, rad/s, [-50 50]
 * @retval         none
 */
void Motorevo_motor_torque_control_mode4(float set_torque1, float set_torque2, float set_torque3, float set_torque4)
{
    uint8_t can_send_data[8] = {0};

    uint16_t t_t1 = float_to_uint(set_torque1, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);
    uint16_t t_t2 = float_to_uint(set_torque2, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);
    uint16_t t_t3 = float_to_uint(set_torque3, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);
    uint16_t t_t4 = float_to_uint(set_torque4, MOTOREVO_TORQUE_MIN, MOTOREVO_TORQUE_MAX, 16);

    can_send_data[0] = t_t1 >> 8;
    can_send_data[1] = t_t1 & 0xFF;
    can_send_data[2] = t_t2 >> 8;
    can_send_data[3] = t_t2 & 0xFF;
    can_send_data[4] = t_t3 >> 8;
    can_send_data[5] = t_t3 & 0xFF;
    can_send_data[6] = t_t4 >> 8;
    can_send_data[7] = t_t4 & 0xFF;

    Motorevo_motor_cantx_communication(100, can_send_data);
}

/**
 * @name           Motorevo_write_motor_config
 * @brief          使用CAN控制电机在力矩4模式（Torque4 Mode）运行
 * @note           控制帧为CAN标准帧，帧ID为100，波特率为1Mhz
 *                 用一帧CAN指令对4个电机同时控制力矩
 * @param[in]      id:设备id
 * @param[in]      set_torque1:对于(Motor ID-1 % 4)=0的电机目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      set_torque2:对于(Motor ID-1 % 4)=1的电机目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      set_torque3:对于(Motor ID-1 % 4)=2的电机目标力矩,12bits, rad/s, [-50 50]
 * @param[in]      set_torque4:对于(Motor ID-1 % 4)=3的电机目标力矩,12bits, rad/s, [-50 50]
 * @retval         none
 */
void Motorevo_write_motor_config(uint8_t id, uint8_t index, uint8_t *data4)
{
    uint8_t can_send_data[8] = {0};

    can_send_data[0] = 0x67;
    can_send_data[1] = index;
    can_send_data[2] = data4[0];
    can_send_data[3] = data4[1];
    can_send_data[4] = data4[2];
    can_send_data[5] = data4[3];
    can_send_data[6] = 0x15;
    can_send_data[7] = 0x76;
    Motorevo_motor_cantx_communication(id + 600, can_send_data);
}
void Motorevo_read_motor_config(Motorevo_DataHandleTypeDef *device, uint8_t id, uint8_t index)
{
    uint8_t can_send_data[8] = {0};

    can_send_data[0] = 0x67;
    can_send_data[1] = index;
    can_send_data[6] = 0x04;
    can_send_data[7] = 0x76;
    Motorevo_motor_cantx_communication(id + 600, can_send_data);
     Motorevo_motor_canrx_communication(device);

}
void Motorevo_save_flash(uint8_t id)
{
    uint8_t can_send_data[8] = {0};
    can_send_data[0] = 0x67;
    can_send_data[1] = 0x00;
    can_send_data[7] = 0x76;
    Motorevo_motor_cantx_communication(id + 600, can_send_data);
}

#endif