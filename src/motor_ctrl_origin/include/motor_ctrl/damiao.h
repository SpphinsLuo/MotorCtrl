

#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#include <variant>
#include <cstdint>
#include "ros/ros.h"


// motorevo
#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300
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

// elmo
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

namespace damiao
{
#pragma pack(1)
#define Motor_id uint32_t

    /*!
     * @brief Motor Type 电机类型
     */
    enum motor_Type
    {
        DM10010L,
        DM10010,
        HA60,
        PA105,
        Num_Of_Motor
    };
    typedef enum
    {
        Motorevo_Reset_Status,
        Motorevo_Motor_Status_Servo,
        Motorevo_Motor_Status_TPMix,
        Motorevo_Motor_Status_Velocity,
        Motorevo_Motor_Status_Torque,
        Motorevo_Motor_Status_torque4,
    } Motorevo_Status;
    /*
     * @brief 电机控制模式
     */
    enum DM_Control_Mode
    {
        MIT_MODE = 1,
        POS_VEL_MODE = 2,
        VEL_MODE = 3,
        POS_FORCE_MODE = 4,
    };
    enum MT_Control_Mode
    {
        MT_SERVO_MODE = 1,
        MT_PT_MIX_MODE = 2,
        MT_VEL_MODE = 3,
        MT_TORQUE_MODE = 4,
    };
    enum Elmo_Control_Mode
    {
        ELMO_POS_MODE = 1,
        ELMO_VEL_MODE = 2,
        ELMO_TAU_MODE = 3,
    };
    /*
     * @brief 寄存器列表 具体参考达妙手册
     */
    enum DM_REG
    {
        UV_Value = 0,
        KT_Value = 1,
        OT_Value = 2,
        OC_Value = 3,
        ACC = 4,
        DEC = 5,
        MAX_SPD = 6,
        MST_ID = 7,
        ESC_ID = 8,
        TIMEOUT = 9,
        CTRL_MODE = 10,
        Damp = 11,
        Inertia = 12,
        hw_ver = 13,
        sw_ver = 14,
        SN = 15,
        NPP = 16,
        Rs = 17,
        LS = 18,
        Flux = 19,
        Gr = 20,
        PMAX = 21,
        VMAX = 22,
        TMAX = 23,
        I_BW = 24,
        KP_ASR = 25,
        KI_ASR = 26,
        KP_APR = 27,
        KI_APR = 28,
        OV_Value = 29,
        GREF = 30,
        Deta = 31,
        V_BW = 32,
        IQ_c1 = 33,
        VL_c1 = 34,
        can_br = 35,
        sub_ver = 36,
        u_off = 50,
        v_off = 51,
        k1 = 52,
        k2 = 53,
        m_off = 54,
        dir = 55,
        p_m = 80,
        xout = 81,
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
        void sdo_modify(const Motor_id id, const uint8_t *send_data)
        {
            canId = id + SDO_OFFSET;
            std::copy(send_data, send_data + 8, data);
        }

    } can_send_frame;

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
            {12.5, 25, 200}, // DM10010L
            {12.5, 20, 200}, // DM10010
            {12.5, 10, 50},  // HA60
            {12.5, 20, 200}  // pa105
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
        motor_Type Motor_Type;

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
        Motor(motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
            : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type)
        {
            this->limit_param = damiao::limit_param[Motor_Type];
        }

        Motor() : Master_id(0x01), Slave_id(0x11), Motor_Type(DM10010)
        {
            this->limit_param = damiao::limit_param[DM10010];
        }

        void receive_data(float q, float dq, float tau)
        {
            this->state_q = q;
            this->state_dq = dq;
            this->state_tau = tau;
        }

        motor_Type GetMotorType() const { return this->Motor_Type; }

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
        void enable(const Motor &motor)
        {
            control_cmd(motor.GetSlaveId(), 0xFC);
            usleep(10000); // 10ms
        }

        /*
         * @brief  disable the motor 失能电机
         * @param  motor object 电机对象
         */
        void disable(const Motor &motor)
        {
            control_cmd(motor.GetSlaveId(), 0xFD);
            usleep(10000);
        }

        /*
         * @brief set the now position as zero point 保存当前位置为电机零点
         * @param motor object 电机对象
         */
        void set_zero_position(const Motor &motor)
        {
            control_cmd(motor.GetSlaveId(), 0xFE);
            usleep(10000);
        }

        /* @description: MIT Control Mode MIT控制模式 具体描述请参考达妙手册
         *@param kp: 比例系数
         *@param kd: 微分系数
         *@param q: position 位置
         *@param dq: velocity 速度
         *@param tau: torque 扭矩
         */
        void dm_control_mit(Motor &motor, float kp, float kd, float q, float dq, float tau)
        {
            // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
            static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t
            {
                float span = xmax - xmin;
                float data_norm = (x - xmin) / span;
                uint16_t data_uint = data_norm * ((1 << bits) - 1);
                return data_uint;
            };
            Motor_id id = motor.GetSlaveId();
            if (motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            auto &m = motors[id];
            uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
            uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
            Limit_param limit_param_cmd = m->get_limit_param();
            uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
            uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX, limit_param_cmd.DQ_MAX, 12);
            uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = (q_uint >> 8) & 0xff;
            data_buf[1] = q_uint & 0xff;
            data_buf[2] = dq_uint >> 4;
            data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
            data_buf[4] = kp_uint & 0xff;
            data_buf[5] = kd_uint >> 4;
            data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
            data_buf[7] = tau_uint & 0xff;

            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description: Position Control Mode with velocity  位置速度控制模式
         * @param pos: position 位置
         * @param vel: velocity 速度
         * @param motor: Motor object 电机对象
         */
        void dm_control_pos_vel(Motor &motor, float pos, float vel)
        {
            Motor_id id = motor.GetSlaveId();
            if (motors.find(id) == motors.end())
            {
                throw std::runtime_error("POS_VEL ERROR : Motor_Control id not found");
            }
            std::array<uint8_t, 8> data_buf{};
            memcpy(data_buf.data(), &pos, sizeof(float));
            memcpy(data_buf.data() + 4, &vel, sizeof(float));
            id += POS_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send(reinterpret_cast<uint8_t *>(&send_data), sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description: velocity control mode 速度控制模式
         * @param motor: motor object 电机对象
         * @param vel: velocity 速度
         */
        void dm_control_vel(Motor &motor, float vel)
        {
            Motor_id id = motor.GetSlaveId();
            if (motors.find(id) == motors.end())
            {
                throw std::runtime_error("VEL ERROR : id not found");
            }
            std::array<uint8_t, 8> data_buf = {0};
            memcpy(data_buf.data(), &vel, sizeof(float));
            id = id + SPEED_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            this->receive();
        }

        /*
         * @description: Position control mode with torque  力位混合控制模式
         * @param motor: motor object 电机对象
         * @param pos: position 位置
         * @param vel: velocity 速度 范围0-10000 具体参考达妙手册
         * @param i: current 电流 范围0-10000具体参考达妙手册
         */
        void dm_control_pos_force(Motor &motor, float pos, uint16_t vel, uint16_t i)
        {
            Motor_id id = motor.GetSlaveId();
            if (motors.find(id) == motors.end())
            {
                throw std::runtime_error("pos_force ERROR : Motor_Control id not found");
            }
            std::array<uint8_t, 8> data_buf{};
            memcpy(data_buf.data(), &pos, sizeof(float));
            memcpy(data_buf.data() + 4, &vel, sizeof(uint16_t));
            memcpy(data_buf.data() + 6, &i, sizeof(uint16_t));
            id = id + POSI_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            this->receive();
        }
        void Motorevo_motor_torque_position_mix_control_mode(
            Motor &motor,
            float set_position,
            float set_velocity,
            float set_torque,
            float p_kp,
            float p_kd)
        {
            Motor_id id = motor.GetSlaveId();
            if (motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            uint8_t can_send_data[8] = {0};
            float set_position_rad = deg_to_rad(set_position);
            float set_velocity_radps = rpm_to_radps(set_velocity);
            static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t
            {
                float span = xmax - xmin;
                float data_norm = (x - xmin) / span;
                uint16_t data_uint = data_norm * ((1 << bits) - 1);
                return data_uint;
            };

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
            send_data.modify(id, can_send_data);
            serial_->send(reinterpret_cast<uint8_t *>(&send_data), sizeof(can_send_frame));
            this->receive();
        }
        void Motorevo_set_zero_position(Motor &motor)
        {
            uint8_t can_send_data[8];
            Motor_id id = motor.GetSlaveId();
            for (int i = 0; i < 7; i++)
            {
                can_send_data[i] = 0xFF;
            }
            can_send_data[7] = 0xFE;
            send_data.modify(id, can_send_data);
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            this->receive();
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
         * @param motor : motor object 电机对象
         */
        void addMotor(Motor *motor)
        {
            motors.insert({motor->GetSlaveId(), motor});
            if (motor->GetMasterId() != 0)
            {
                motors.insert({motor->GetMasterId(), motor});
            }
        }

        /*
         * @description: read motor register param 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
         * @param motor: motor object 电机对象
         * @param RID: register id 寄存器ID  example: damiao::UV_Value
         * @return: motor param 电机参数 如果没查询到返回的参数为0
         */
        float read_motor_param(Motor &motor, uint8_t RID)
        {
            uint32_t id = motor.GetSlaveId();
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(10000); // 10ms
            receive_param();
            if (motors.find(motor.GetSlaveId()) == motors.end())
            {
                return 0;
            }
            else
            {
                if (motors[motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return float(motors[motor.GetSlaveId()]->get_param_as_uint32(RID));
                    }
                    else
                    {
                        return motors[motor.GetSlaveId()]->get_param_as_float(RID);
                    }
                }
                else
                {
                    return 0;
                }
            }
        }

        /*
         * @description: switch control mode 切换电机控制模式
         * @param motor: motor object 电机对象
         * @param mode: control mode 控制模式 like:damiao::MIT_MODE, damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
         */
        bool switchControlMode(Motor &motor, DM_Control_Mode mode)
        {
            uint8_t write_data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
            uint8_t RID = 10;
            write_motor_param(motor, RID, write_data);
            receive_param();
            usleep(10000); // 10ms
            if (motors.find(motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            else
            {
                if (motors[motor.GetSlaveId()]->is_have_param(RID))
                {
                    return motors[motor.GetSlaveId()]->get_param_as_uint32(RID) == mode;
                }
                else
                {
                    return false;
                }
            }
        }

        /*
         * @description: change motor param 修改电机内部寄存器参数 具体寄存器列表请参考达妙手册
         * @param motor: motor object 电机对象
         * @param RID: register id 寄存器ID
         * @param data: param data 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
         * @return: bool true or false  是否修改成功
         */
        bool change_motor_param(Motor &motor, uint8_t RID, float data)
        {
            if (is_in_ranges(RID))
            {
                // 居然传进来的是整型的范围 救一下
                uint32_t data_uint32 = float_to_uint32(data);
                uint8_t *data_uint8;
                data_uint8 = (uint8_t *)&data_uint32;
                write_motor_param(motor, RID, data_uint8);
            }
            else
            {
                // is float
                uint8_t *data_uint8;
                data_uint8 = (uint8_t *)&data;
                write_motor_param(motor, RID, data_uint8);
            }
            usleep(10000); // 10ms
            receive_param();
            if (motors.find(motor.GetSlaveId()) == motors.end())
            {
                return false;
            }
            else
            {
                if (motors[motor.GetSlaveId()]->is_have_param(RID))
                {
                    if (is_in_ranges(RID))
                    {
                        return motors[motor.GetSlaveId()]->get_param_as_uint32(RID) == float_to_uint32(data);
                    }
                    else
                    {
                        return fabsf(motors[motor.GetSlaveId()]->get_param_as_float(RID) - data) < 0.1f;
                    }
                }
                else
                {
                    return false;
                }
            }
        }

        /*
         * @description: save all param to motor flash 保存电机的所有参数到flash里面
         * @param motor: motor object 电机对象
         * 电机默认参数不会写到flash里面，需要进行写操作
         */
        void save_motor_param(Motor &motor)
        {
            disable(motor);
            uint32_t id = motor.GetSlaveId();
            uint8_t id_low = id & 0xff;
            uint8_t id_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{id_low, id_high, 0xAA, 0x01, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(100000); // 100ms wait for save
        }

        /*
         * @description: change motor limit param 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
         * @param motor: motor object 电机对象
         * @param P_MAX: position max 位置最大值
         * @param Q_MAX: velocity max 速度最大值
         * @param T_MAX: torque max 扭矩最大值
         */
        static void changeMotorLimit(Motor &motor, float P_MAX, float Q_MAX, float T_MAX)
        {
            limit_param[motor.GetMotorType()] = {P_MAX, Q_MAX, T_MAX};
        }

        /*
         * @description: change motor PMAX 修改电机的最大PMAX
         * @param motor: motor object 电机对象
         * @param P_MAX: position max 位置最大值
         */
        static void changeMotorPMAX(Motor &motor, float P_MAX)
        {
            limit_param[motor.GetMotorType()].Q_MAX = P_MAX;
        }

        // elmo
        void elmo_enable(Motor_id node)
        {
            elmo_control_cmd(node, 0x06);
            usleep(1000); // 10ms
            elmo_control_cmd(node, 0x07);
            usleep(1000); // 10ms
            elmo_control_cmd(node, 0x0F);
            usleep(10000); // 10ms
        }
        void elmo_disable(Motor_id node)
        {
            elmo_control_cmd(node, 0x00);
            usleep(10000);
        }
        void elmo_set_vel_mode(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
        }
        void elmo_set_pos_mode(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
        }
        void elmo_set_tau_mode(Motor_id id)
        {
            std::array<uint8_t, 8> data_buf = {0x23, 0x60, 0x60, 0x00, 0x04, 0x00, 0x00, 0x00};
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
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
            send_data.sdo_modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
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
            std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t *)&send_data, sizeof(can_send_frame));
            usleep(1000);
            // receive();
        }

        void write_motor_param(Motor &motor, uint8_t RID, const uint8_t data[4])
        {
            uint32_t id = motor.GetSlaveId();
            uint8_t can_low = id & 0xff;
            uint8_t can_high = (id >> 8) & 0xff;
            std::array<uint8_t, 8> data_buf{can_low, can_high, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
            data_buf[4] = data[0];
            data_buf[5] = data[1];
            data_buf[6] = data[2];
            data_buf[7] = data[3];
            send_data.modify(0x7FF, data_buf.data());
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
        // elmo
        void elmo_control_cmd(Motor_id id, uint8_t cmd)
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
        // motorevo
        /**
         * @brief 弧度转角度
         * @param rad  输入弧度值
         * @return     对应的角度值
         */
        inline double rad_to_deg(double rad)
        {
            return rad * 180.0 / M_PI;
        }
        /**
         * @brief 角度转弧度
         * @param deg  输入角度值（°）
         * @return     对应的弧度值（rad）
         */
        inline double deg_to_rad(double deg)
        {
            return deg * M_PI / 180.0;
        }

        /**
         * @brief 弧度每秒 (rad/s) 转 转速 RPM（revolutions per minute）
         *        1 转 = 2π 弧度
         * @param rad_per_sec  输入的弧度/秒
         * @return             对应的“转/分钟”（RPM）
         */
        inline double radps_to_rpm(double rad_per_sec)
        {
            return rad_per_sec * 60.0 / (2.0 * M_PI);
        }
        /**
         * @brief RPM 转 弧度每秒 (rad/s)
         * @param rpm  转速（单位：转/分钟）
         * @return     弧度每秒（单位：rad/s）
         */
        inline double rpm_to_radps(double rpm)
        {
            return rpm * 2.0 * M_PI / 60.0;
        }
        std::unordered_map<Motor_id, Motor *> motors;
        SerialPort::SharedPtr serial_;    // serial port
        can_send_frame send_data;         // send data frame
        CAN_Receive_Frame receive_data{}; // receive data frame
    };

};

#endif
