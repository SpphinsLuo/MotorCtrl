#ifndef elmo_H
#define elmo_H

#include "SerialPort.h"
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include "ros/ros.h"

// 整体框架

//

// ----- Application-specific -----
#define ENCODER_PPR 131072U
#define GEARBOX_RATIO 18.0f
#define COUNTS_PER_REV ((uint32_t)(ENCODER_PPR * GEARBOX_RATIO))
#define NODE_ID 0x05
#define RPDO1_COB_ID (0x200 + NODE_ID)
#define TPDO1_COB_ID (0x180 + NODE_ID)
#define OD_CONTROLWORD 0x6040
#define OD_STATUSWORD 0x6041
#define OD_MODE_OF_OPERATION 0x6060
#define OD_TARGET_POSITION 0x607A
#define OD_PROFILE_VELOCITY 0x6081
#define OD_PROFILE_ACCELERATION 0x6083
#define OD_PROFILE_DECELERATION 0x6084
#define OD_ACTUAL_POSITION 0x6064
#define CW_SHUTDOWN 0x0006
#define CW_SWITCH_ON 0x0007
#define CW_ENABLE_OPERATION 0x000F
#define MODE_PROFILE_POSITION 0x01
#define PDO_ENABLE 1
#define SDO_OFFSET 0x600

namespace elmo
{
#pragma pack(1)
#define Motor_id uint32_t

    /*!
     * @brief Motor Type 电机类型
     */
    enum elmo_Motor_Type
    {
        PA105,
        Num_Of_Motor
    };

    /*
     * @brief 电机控制模式
     */
    enum Control_Mode
    {
        POS_MODE = 1,
        VEL_MODE = 2,
        TAU_MODE = 3,
    };

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

    typedef struct can_send_frame
    {
        uint8_t FrameHeader[2] = {0x55, 0xAA}; // 帧头
        uint8_t FrameLen = 0x1e;               // 帧长
        uint8_t CMD = 0x03;                    // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3: 非反馈CAN转发，不反馈发送状态
        uint32_t sendTimes = 1;                // 发送次数
        uint32_t timeInterval = 10;            // 时间间隔
        uint8_t IDType = 0;                    // ID类型 0：标准帧 1：扩展帧
        uint32_t canId = 0x05;                 // CAN ID 使用电机ID作为CAN ID
        uint8_t frameType = 0;                 // 帧类型 0： 数据帧 1：远程帧
        uint8_t len = 0x08;                    // len
        uint8_t idAcc = 0;
        uint8_t dataAcc = 0;
        uint8_t data[8] = {0};
        uint8_t crc = 0; // 未解析，任意值

        void sdo_modify(const Motor_id id, const uint8_t *send_data)
        {
            canId = id + SDO_OFFSET;
            std::copy(send_data, send_data + 8, data);
        }

    } CAN_Send_Frame;

#pragma pack()

    typedef struct
    {
        float Q_MAX;
        float DQ_MAX;
        float TAU_MAX;
    } Limit_param;

    // 电机PMAX DQMAX TAUMAX参数
    Limit_param limit_param[Num_Of_Motor] =
        {
            {12.5, 20, 200} // pa105
    };

    class Motor
    {
    private:
        /* data */
        Motor_id Master_id;
        Motor_id Slave_id;
        float state_q = 0;
        float state_dq = 0;
        float state_tau = 0;
        Limit_param limit_param{};
        elmo_Motor_Type Motor_Type;

        union ValueUnion
        {
            float floatValue;
            uint32_t uint32Value;
        };

        struct ValueType
        {
            ValueUnion value;
            bool isFloat;
        };

        std::unordered_map<uint32_t, ValueType> param_map;

    public:
        /**
         * @brief Construct a new Motor object
         *
         * @param Motor_Type 电机类型
         * @param Slave_id canId 从机ID即电机ID
         * @param Master_id 主机ID建议主机ID不要都设为0x00
         *
         */
        Motor(elmo_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
            : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type)
        {
            this->limit_param = elmo::limit_param[Motor_Type];
        }

        Motor() : Master_id(0x15), Slave_id(0x05), Motor_Type(PA105)
        {
            this->limit_param = elmo::limit_param[PA105];
        }

        void receive_data(float q, float dq, float tau)
        {
            this->state_q = q;
            this->state_dq = dq;
            this->state_tau = tau;
        }

        elmo_Motor_Type GetMotorType() const { return this->Motor_Type; }

        /*
         * @brief get master id 获取主机ID
         * @return MasterID
         */
        Motor_id GetMasterId() const { return this->Master_id; }

        /*
         * @brief get motor slave id(can id)  获取电机CAN ID
         * @return SlaveID
         */
        Motor_id GetSlaveId() const { return this->Slave_id; }

        /*
         * @brief get motor position 获取电机位置
         * @return motor position 电机位置
         */
        float Get_Position() const { return this->state_q; }

        /*
         * @brief get motor velocity 获取电机速度
         * @return motor velocity 电机速度
         */
        float Get_Velocity() const { return this->state_dq; }

        /*
         * @brief get torque of the motor  获取电机实际输出扭矩
         * @return motor torque 电机实际输出扭矩
         */
        float Get_tau() const { return this->state_tau; }

        /*
         * @brief get limit param 获取电机限制参数
         * @return limit_param 电机限制参数
         */
        Limit_param get_limit_param() { return limit_param; }

        void set_param(int key, float value)
        {
            ValueType v{};
            v.value.floatValue = value;
            v.isFloat = true;
            param_map[key] = v;
        }

        void set_param(int key, uint32_t value)
        {
            ValueType v{};
            v.value.uint32Value = value;
            v.isFloat = false;
            param_map[key] = v;
        }

        float get_param_as_float(int key) const
        {
            auto it = param_map.find(key);
            if (it != param_map.end())
            {
                if (it->second.isFloat)
                {
                    return it->second.value.floatValue;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        uint32_t get_param_as_uint32(int key) const
        {
            auto it = param_map.find(key);
            if (it != param_map.end())
            {
                if (!it->second.isFloat)
                {
                    return it->second.value.uint32Value;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        bool is_have_param(int key) const
        {
            return param_map.find(key) != param_map.end();
        }
    };

    /**
     * @brief motor control class 电机控制类
     *
     * 使用USB转CAN进行通信，linux做虚拟串口
     */
    class Motor_Control
    {
    public:
        /*
         * @brief 定义电机控制对象
         * @param serial 串口对象
         * 默认串口为/dev/ttyACM0
         */
        Motor_Control(SerialPort::SharedPtr serial = nullptr) : serial_(std::move(serial))
        {
            if (serial_ == nullptr)
            {
                // default serial port
                serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
            }
        }

        ~Motor_Control() = default;

        /*
         * @brief enable the motor 使能电机
         * @param motor 电机对象
         */
        void enable(Motor_id node)
        {
            control_cmd(node, 0x06);
            usleep(1000); // 10ms
            control_cmd(node, 0x07);
            usleep(1000); // 10ms
            control_cmd(node, 0x0F);
            usleep(10000); // 10ms
        }
        /*
         * @brief  disable the motor 失能电机
         * @param  motor object 电机对象
         */
        void disable(Motor_id node)
        {
            control_cmd(node, 0x00);
            usleep(10000);
        }

        void set_vel_mode(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
        }
        void set_pos_mode(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
        }
        void set_tau_mode(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
        }

        /*
         * @description 函数库内部调用,用于解算电机can线返回的电机参数
         */
        void receive()
        {
            serial_->recv((uint8_t *)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

            if (receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
            {
                static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float
                {
                    float span = xmax - xmin;
                    float data_norm = float(x) / ((1 << bits) - 1);
                    float data = data_norm * span + xmin;
                    return data;
                };

                auto &data = receive_data.canData;

                uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
                uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
                uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

                if (motors.find(receive_data.canId) == motors.end())
                {
                    return;
                }

                auto m = motors[receive_data.canId];
                Limit_param limit_param_receive = m->get_limit_param();
                float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
                float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
                float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
                m->receive_data(receive_q, receive_dq, receive_tau);
                return;
            }
            else if (receive_data.CMD == 0x01) // receive fail
            {
                std::cout << "receive fail" << std::endl;
            }
            else if (receive_data.CMD == 0x02) // send fail
            {
                /* code */
            }
            else if (receive_data.CMD == 0x03) // send success
            {
                /* code */
            }
            else if (receive_data.CMD == 0xEE) // communication error
            {
                /* code */
            }
        }

        void receive_param()
        {
            serial_->recv((uint8_t *)&receive_data, 0xAA, sizeof(CAN_Receive_Frame));

            if (receive_data.CMD == 0x11 && receive_data.frameEnd == 0x55) // receive success
            {
                auto &data = receive_data.canData;
                if (data[2] == 0x33 or data[2] == 0x55)
                {
                    uint32_t slaveID = (uint32_t(data[1]) << 8) | data[0];
                    uint8_t RID = data[3];
                    if (motors.find(slaveID) == motors.end())
                    {
                        // can not found motor id
                        return;
                    }
                    if (is_in_ranges(RID))
                    {
                        uint32_t data_uint32 = (uint32_t(data[7]) << 24) | (uint32_t(data[6]) << 16) | (uint32_t(data[5]) << 8) | data[4];
                        motors[slaveID]->set_param(RID, data_uint32);
                    }
                    else
                    {
                        float data_float = uint8_to_float(data + 4);
                        motors[slaveID]->set_param(RID, data_float);
                    }
                }
                return;
            }
        }

        /**
         * @brief add motor to class 添加电机
         * @param elmo_Motor : motor object 电机对象
         */
        void addMotor(Motor *elmo_Motor)
        {
            motors.insert({elmo_Motor->GetSlaveId(), elmo_Motor});
            if (elmo_Motor->GetMasterId() != 0)
            {
                motors.insert({elmo_Motor->GetMasterId(), elmo_Motor});
            }
        }

        /*
         * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
         * @param elmo_Motor: motor object 电机对象
         * @param RID: register id 寄存器ID  example: elmo::UV_Value
         * @return: motor param 电机参数 如果没查询到返回的参数为0
         */
        float read_motor_param(Motor &elmo_Motor, uint8_t RID)
        {
            uint32_t id = elmo_Motor.GetSlaveId();
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
            send_data.sdo_modify(0x7FF, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(10000); // 10ms
            receive_param();
            if (motors.find(elmo_Motor.GetSlaveId()) == motors.end())
            {
                return 0;
            }
            else
            {
                if (motors[elmo_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return float(motors[elmo_Motor.GetSlaveId()]->get_param_as_uint32(RID));
                    }
                    else
                    {
                        return motors[elmo_Motor.GetSlaveId()]->get_param_as_float(RID);
                    }
                }
                else
                {
                    return 0;
                }
            }
        }

        /*
         * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
         * @param elmo_Motor: motor object 电机对象
         * @param RID: register id 寄存器ID
         * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
         * @return: bool true or false  是否修改成功
         */
        bool change_motor_param(Motor &elmo_Motor, uint8_t RID, float data)
        {
            if (is_in_ranges(RID))
            {
                // 居然传进来的是整型的范围 救一下
                uint32_t data_uint32 = float_to_uint32(data);
                uint8_t *data_uint8;
                data_uint8 = (uint8_t *)&data_uint32;
                write_motor_param(elmo_Motor, RID, data_uint8);
            }
            else
            {
                // is float
                uint8_t *data_uint8;
                data_uint8 = (uint8_t *)&data;
                write_motor_param(elmo_Motor, RID, data_uint8);
            }
            usleep(10000); // 10ms
            receive_param();
            if (motors.find(elmo_Motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            else
            {
                if (motors[elmo_Motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return motors[elmo_Motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
                    }
                    else
                    {
                        return fabsf(motors[elmo_Motor.GetSlaveId()]->get_param_as_float(RID) - data) < 0.1f;
                    }
                }
                else
                {
                    return false;
                }
            }
        }

        /**
         * @brief 切换到位置模式，并发送目标位置
         *
         * @param node CANopen 节点号
         * @param angle 目标输出轴角度（度），可正可负
         */
        void pos_mode(Motor_id node, float angle)
        {

            // 计算角度对应的脉冲数 (有符号 int32)
            int32_t target_pulses = static_cast<int32_t>(
                std::round(angle_pulse_converter(angle, ENCODER_PPR, GEARBOX_RATIO, false)));

            usleep(10000);
            // 写入目标位置 (Target Position)
            {
                // 小端格式发送 4 字节 int32
                uint8_t pdata[4];
                // pdata[0] = target_pulses & 0xFF;
                // pdata[1] = (target_pulses >> 8) & 0xFF;
                // pdata[2] = (target_pulses >> 16) & 0xFF;
                // pdata[3] = (target_pulses >> 24) & 0xFF;

                pdata[0] = (target_pulses >> 24) & 0xFF;
                pdata[1] = (target_pulses >> 16) & 0xFF;
                pdata[2] = (target_pulses >> 8) & 0xFF;
                pdata[3] = target_pulses & 0xFF;

                if (canopen_sdo_write(node, 0x607A, 0x00, pdata, 4) != 0)
                {
                    return;
                }
            }
            usleep(10000);

            // 发 “relative New set-point”（启动运动）
            {

                uint16_t ctrl = 0x5F; // 控制字 = New set-point + other位保持
                canopen_sdo_write(node, 0x6040, 0x00, &ctrl, 4);
                uint16_t start = 0x0f;
                canopen_sdo_write(node, 0x6040, 0x00, &start, 4);
            }
        }

        // ----- SDO expedited write -----
        int canopen_sdo_write(Motor_id node, uint16_t idx, uint8_t sub, void *data, uint8_t size)
        {
            uint8_t frame[8] = {0};
            uint8_t cs = (size == 1 ? 0x2F : size == 2 ? 0x2B
                                         : size == 3   ? 0x07
                                         : size == 4   ? 0x23
                                                       : 0);
            if (!cs)
                return -1;
            frame[0] = cs;
            frame[1] = idx & 0xFF;
            frame[2] = idx >> 8;
            frame[3] = sub;
            uint8_t *pdata = static_cast<uint8_t *>(data);
            for (int i = 0; i < size; ++i)
            {
                // pdata[size-1-i] 是大端数据的低字节往高字节方向取
                frame[4 + i] = pdata[size - 1 - i];
            }
            send_data.sdo_modify(node, frame);
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));

            // if (can_send(tx, frame))
            //     return -2;
            // uint8_t resp[8];
            // if (can_receive(rx, resp, 8, 1000) != 8)
            //     return -3;
            // if ((resp[0] & 0xE0) != 0x60)
            //     return -4;
            return 0;
        }

        // int can_send(uint32_t cob_id, uint8_t *data)
        // {
        //     can_send_frame frame;
        //     frame.FrameHeader[0] = 0x55;
        //     frame.FrameHeader[1] = 0xAA;
        //     frame.FrameLen = 0x1e;
        //     frame.CMD = 0x03;
        //     frame.sendTimes = 1;
        //     frame.timeInterval = 10;
        //     frame.IDType = 0;
        //     frame.canId = cob_id;
        //     frame.frameType = 0;
        //     frame.len = 0x08;
        //     frame.idAcc = 0;
        //     frame.dataAcc = 0;
        //     memset(frame.data, 0, 8);
        //     memcpy(frame.data, data, 8);
        //     frame.crc = 0; // placeholder
        //     serial_->send((uint8_t *)&frame, sizeof(frame));
        //     // ssize_t ret = serial_->send((uint8_t *)&frame, sizeof(frame));
        //     // return (ret == sizeof(frame)) ? 0 : -1;
        // }

        // int can_receive(uint32_t cob_id, uint8_t *data, uint8_t len, uint32_t timeout_ms)
        // {
        //     CAN_Receive_Frame rx;
        //     serial_->set_timeout(timeout_ms);
        //     while (1)
        //     {
        //         ssize_t r = serial_->recv((uint8_t *)&rx, sizeof(rx));
        //         if (r != sizeof(rx))
        //             return -1;
        //         if (rx.FrameHeader != 0x55)
        //             continue;
        //         if (rx.CMD != 0x03)
        //             continue;
        //         if (rx.canId != cob_id)
        //             continue;
        //         if (rx.canDataLen != len)
        //             return -1;
        //         memcpy(data, rx.canData, len);
        //         return len;
        //     }
        // }

        void config_rpdo1(void)
        {
            // 修改后的C++兼容版本
            {
                // 配置RPDO1参数（0x1400,子索引1）
                uint8_t data_1400_1[] = {0x05, 0x02, 0x00, 0x80}; // COB-ID=0x205
                canopen_sdo_write(NODE_ID, 0x1400, 1, data_1400_1, 4);

                // 配置RPDO1映射条目数（0x1600,子索引0）
                uint8_t data_1600_0[] = {0x00}; // 初始映射对象数为0
                canopen_sdo_write(NODE_ID, 0x1600, 0, data_1600_0, 1);

                // 启用RPDO1（0x1400,子索引2）
                uint8_t data_1400_2[] = {0x02}; // 启用TPDO1
                canopen_sdo_write(NODE_ID, 0x1400, 2, data_1400_2, 1);

                // 映射控制字（0x6040）到RPDO1（0x1600,子索引1）
                uint8_t data_1600_1[] = {0x10, 0x00, 0x40, 0x60}; // 0x60400010（控制字）
                canopen_sdo_write(NODE_ID, 0x1600, 1, data_1600_1, 4);

                // 映射目标位置（0x607A）到RPDO1（0x1600,子索引2）
                uint8_t data_1600_2[] = {0x20, 0x00, 0x7A, 0x60}; // 0x607A0020（目标位置）
                canopen_sdo_write(NODE_ID, 0x1600, 2, data_1600_2, 4);

                // 设置RPDO1映射条目数为2（0x1600,子索引0）
                uint8_t data_1600_0_final[] = {0x02}; // 映射对象数量=2
                canopen_sdo_write(NODE_ID, 0x1600, 0, data_1600_0_final, 1);

                // 重新启用RPDO1（0x1400,子索引1）
                uint8_t data_1400_1_final[] = {0x05, 0x02}; // COB-ID=0x205（启用）
                canopen_sdo_write(NODE_ID, 0x1400, 1, data_1400_1_final, 4);
            }
        }
        // 暂时单独写pp模式的映射
        void config_tpdo1(void)
        {
            {
                // 禁用TxPDO1映射（0x1800,子索引1）
                uint8_t disable_txpdo1[] = {0x85, 0x01, 0x00, 0x80}; // COB-ID=0x185（禁用）
                canopen_sdo_write(NODE_ID, 0x1800, 1, disable_txpdo1, 4);

                // 设置TxPDO1映射对象数量为0（0x1A00,子索引0）
                uint8_t zero_mapping_count[] = {0x00};
                canopen_sdo_write(NODE_ID, 0x1A00, 0, zero_mapping_count, 1);

                // 设置TxPDO传输类型为同步周期1（0x1800,子索引2）
                uint8_t transmission_type[] = {0x01}; // 同步类型1
                canopen_sdo_write(NODE_ID, 0x1800, 2, transmission_type, 1);

                // 设置禁止时间为10*100us（0x1800,子索引3）
                uint8_t inhibit_time[2] = {0x0A, 0x00}; // 小端格式0x000A=10*100μs=1ms
                canopen_sdo_write(NODE_ID, 0x1800, 3, inhibit_time, 2);

                // 映射对象1：状态字（0x6069）（0x1A00,子索引1）
                uint8_t txpdo1_mapping_1[] = {0x20, 0x00, 0x69, 0x60}; // 0x60690020（32位）
                canopen_sdo_write(NODE_ID, 0x1A00, 1, txpdo1_mapping_1, 4);

                // 映射对象2：速度实际值（0x6064）（0x1A00,子索引2）
                uint8_t txpdo1_mapping_2[] = {0x20, 0x00, 0x64, 0x60}; // 0x60640020（32位）
                canopen_sdo_write(NODE_ID, 0x1A00, 2, txpdo1_mapping_2, 4);

                // 设置TxPDO1映射对象数量为2（0x1A00,子索引0）
                uint8_t mapping_count[] = {0x02};
                canopen_sdo_write(NODE_ID, 0x1A00, 0, mapping_count, 1);

                // 重新启用TxPDO1映射（0x1800,子索引1）
                uint8_t enable_txpdo1[] = {0x85, 0x01, 0x00, 0x00}; // COB-ID=0x185（启用）
                canopen_sdo_write(NODE_ID, 0x1800, 1, enable_txpdo1, 4);
            }
        }
        void config_acc()
        {
            // 配置加速度参数 (0x6083)
            uint8_t accel_data[] = {0x40, 0x4B, 0x4C, 0x80}; // 小端格式，值=0x804C4B40
            canopen_sdo_write(NODE_ID, 0x6083, 0, accel_data, 4);
        }
        void config_dec()
        {
            // 配置减速度参数 (0x6084)
            uint8_t decel_data[] = {0x40, 0x4B, 0x4C, 0x80}; // 与加速度相同值
            canopen_sdo_write(NODE_ID, 0x6084, 0, decel_data, 4);
        }
        void config_vel()
        {
            // 配置速度参数 (0x6081)
            uint8_t speed_data[] = {0x40, 0x4B, 0x4C, 0x80}; // 相同值
            canopen_sdo_write(NODE_ID, 0x6081, 0, speed_data, 4);
        }
        SerialPort::SharedPtr serial_; // serial port
        void set_pos(Motor_id id, float angle)
        {
            int32_t target_pulses = static_cast<int32_t>(
                std::round(angle_pulse_converter(angle, ENCODER_PPR, GEARBOX_RATIO, false)));

            usleep(10000);
            // 写入目标位置 (Target Position)

            // 小端格式发送 4 字节 int32
            uint8_t pdata[4];
            pdata[0] = target_pulses & 0xFF;
            pdata[1] = (target_pulses >> 8) & 0xFF;
            pdata[2] = (target_pulses >> 16) & 0xFF;
            pdata[3] = (target_pulses >> 24) & 0xFF;

            usleep(10000);

            // std::array<uint8_t, 8> data_buf = {0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0x0b, 0x00};
            // std::array<uint8_t, 8> data_buf = {0x23, 0x7a, 0x60, 0x00, 0x00, 0x00, 0xf5, 0xff}; // 00 f5 ff ff顺时针
            std::array<uint8_t, 8> data_buf = {0x23, 0x7a, 0x60, 0x00, pdata[0], pdata[1], pdata[2], pdata[3]}; // 00 f5 ff ff顺时针
            std::cout << pdata[0] << " " << pdata[1] << " " << pdata[2] << " " << pdata[3] << " " <<std::endl;
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }
        /**
         * @brief 设置电机速度（CANopen速度模式）
         * @param id 电机ID
         * @param rad_per_sec 目标速度（单位：弧度/秒，正负号表示方向）
         */
        void set_speed(Motor_id id, float rad_per_sec)
        {
            // 1. 速度单位转换：弧度/秒 -> 脉冲/秒
            // 计算公式：脉冲频率 = (rad_per_sec * GEARBOX_RATIO * ENCODER_PPR) / (2π)
            float pulses_per_sec = (rad_per_sec * GEARBOX_RATIO * ENCODER_PPR) / (2 * M_PI);

            // 转换为有符号整型（驱动器接受的格式）
            int32_t target_speed = static_cast<int32_t>(std::round(pulses_per_sec));

            // 2. 构造目标速度数据（小端格式）
            uint8_t pdata[4];
            pdata[0] = target_speed & 0xFF; // 低字节
            pdata[1] = (target_speed >> 8) & 0xFF;
            pdata[2] = (target_speed >> 16) & 0xFF;
            pdata[3] = (target_speed >> 24) & 0xFF; // 高字节

            // 3. 构造CAN数据帧（假设目标速度对象字典索引为0x60FF）
            // 格式说明:
            //   0x22: 写入4字节数据 + 子索引
            //   0xFF, 0x60: 对象字典0x60FF（小端表示）
            //   0x00: 子索引（通常为0）
            std::array<uint8_t, 8> data_buf = {
                0x22,                                  // 写入命令 + 数据大小指示
                0xFF, 0x60,                            // 目标速度对象字典索引(0x60FF)
                0x00,                                  // 子索引
                pdata[0], pdata[1], pdata[2], pdata[3] // 速度值小端格式
            };

            // 4. 发送CAN命令
            usleep(10000); // 延时10ms（根据硬件调整）
            send_data.sdo_modify(id, data_buf.data());
            serial_->send(reinterpret_cast<uint8_t *>(&send_data), sizeof(can_send_frame));
            usleep(1000); // 短延时
        }
        void set_vel(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x81, 0x60, 0x00, 0x40, 0x4b, 0xff, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }
        void set_acc(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x83, 0x60, 0x00, 0x40, 0x4b, 0xff, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }
        void set_dec(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x84, 0x60, 0x00, 0x40, 0x4b, 0xff, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }
        void set_rea(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }
        void set_start(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }

    private:
        void control_cmd(Motor_id id, uint8_t cmd)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x40, 0x60, 0x00, cmd, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }
        /**
         * @brief 角度（浮点数）与脉冲数（有符号整数）的双向转换
         *
         * @param value 输入值（角度或脉冲数）
         * @param pulses_per_rev 编码器每圈脉冲数（默认131072）
         * @param gear_ratio 减速比（默认18）
         * @param to_angle true=脉冲转角度，false=角度转脉冲
         * @return 转换后的值（角度或脉冲数）
         */
        float angle_pulse_converter(float value,
                                    int pulses_per_rev = 131072,
                                    float gear_ratio = 18.0,
                                    bool to_angle = true)
        {
            if (to_angle)
            {
                // 脉冲数 → 角度（考虑减速比）
                float motor_revs = value / pulses_per_rev;   // 电机圈数
                float output_revs = motor_revs / gear_ratio; // 输出轴圈数
                float degrees = output_revs * 360.0;         // 输出轴角度
                return degrees;
            }
            else
            {
                // 角度 → 脉冲数（考虑减速比）
                float output_revs = value / 360.0;                                      // 输出轴圈数
                float motor_revs = output_revs * gear_ratio;                            // 电机圈数
                int pulses = static_cast<int>(std::round(motor_revs * pulses_per_rev)); // 总脉冲数
                return pulses;
            }
        }
        void write_motor_param(Motor &elmo_Motor, uint8_t RID, const uint8_t data[4])
        {
            uint32_t id = elmo_Motor.GetSlaveId();
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
            data_buf[4] = data[0];
            data_buf[5] = data[1];
            data_buf[6] = data[2];
            data_buf[7] = data[3];
            send_data.sdo_modify(0x7FF, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
        }

        static bool is_in_ranges(int number)
        {
            return (7 <= number && number <= 10) ||
                   (13 <= number && number <= 16) ||
                   (35 <= number && number <= 36);
        }

        static uint32_t float_to_uint32(float value)
        {
            return static_cast<uint32_t>(value);
        }

        static float uint32_to_float(uint32_t value)
        {
            return static_cast<float>(value);
        }

        static float uint8_to_float(const uint8_t data[4])
        {
            uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) |
                                (static_cast<uint32_t>(data[2]) << 16) |
                                (static_cast<uint32_t>(data[1]) << 8) |
                                static_cast<uint32_t>(data[0]);
            float result;
            memcpy(&result, &combined, sizeof(result));
            return result;
        }

        std::unordered_map<Motor_id, Motor *> motors;
        can_send_frame send_data;         // send data frame
        CAN_Receive_Frame receive_data{}; // receive data frame
    };

};

#endif
