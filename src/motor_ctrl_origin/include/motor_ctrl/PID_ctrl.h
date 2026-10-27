#ifndef PID_CTRL_H
#define PID_CTRL_H

#include "ros/ros.h"


#include "unistd.h"
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <iomanip>
#include <vector>
#include <iostream>

class PIDController
{
public:
    PIDController();
    PIDController(float kp, float ki, float kd, float integralLimit, float outputLimit, float timeDelta, float errorThreshold);
    // 位置式PID
    float PIDCompute(float targetPos, float currentPos);
    // 根据误差
    float PIDCompute_err(float error);
    // 增量式PID
    float PIDComputeWithFeedforward(float error, float feedforward);

private:
    float kp_;             // 比例增益
    float ki_;             // 积分增益
    float kd_;             // 微分增益
    float integralLimit_;  // 积分限幅
    float outputLimit_;    // 输出限幅
    float integral_;       // 积分项累计
    float error_1_;        // 上一次误差
    float error_2_;        // 上上次误差
    float timeDelta_;      // 控制周期
    float errorThreshold_; // 最小误差
    float output_prev_;
};


PIDController::PIDController() : kp_(0), ki_(0), kd_(0), integralLimit_(0), outputLimit_(0), timeDelta_(0), errorThreshold_(0), integral_(0), error_1_(0.0), error_2_(0.0), output_prev_(0.0) {}
PIDController::PIDController(float kp, float ki, float kd, float integralLimit, float outputLimit, float timeDelta, float errorThreshold)
    : kp_(kp), ki_(ki), kd_(kd),
      integralLimit_(integralLimit), outputLimit_(outputLimit), timeDelta_(timeDelta), errorThreshold_(errorThreshold),
      integral_(0.0), error_1_(0.0), error_2_(0.0), output_prev_(0.0)
{
}

float PIDController::PIDCompute(float targetPos, float currentPos)
{
    float error = targetPos - currentPos;
    // 积分限幅
    integral_ += error;
    if (integral_ > integralLimit_)
    {
        integral_ = integralLimit_;
    }
    else if (integral_ < -integralLimit_)
    {
        integral_ = -integralLimit_;
    }
    // 微分计算
    float derivative = error - error_1_;
    float filtered_derivative = 0.9 * derivative + 0.1 * (error_1_ - error_2_) / timeDelta_; // 一阶低通滤波
    // PID 控制输出
    float output_v = kp_ * error + ki_ * integral_ + kd_ * filtered_derivative;
    // 更新前一误差
    error_1_ = error;
    float v;
    if (fabs(error) < errorThreshold_)
    {
        v = 0.0;
    }
    else
    {
        // 输出限幅
        v = output_v;
        if (v > outputLimit_)
        {
            v = outputLimit_;
        }
        else if (v < -outputLimit_)
        {
            v = -outputLimit_;
        }
    }
    return v;
}

float PIDController::PIDCompute_err(float error)
{
    // std::cout << "Error: " << error << std::endl;
    //  积分限幅
    integral_ += error;
    if (integral_ > integralLimit_)
    {
        integral_ = integralLimit_;
    }
    else if (integral_ < -integralLimit_)
    {
        integral_ = -integralLimit_;
    }
    // 微分计算
    float derivative = error - error_1_;
    float filtered_derivative = 0.9 * derivative + 0.1 * (error_1_ - error_2_) / timeDelta_; // 一阶低通滤波
    // PID 控制输出
    float output_v = kp_ * error + ki_ * integral_ + kd_ * filtered_derivative;
    // 更新前一误差
    error_1_ = error;
    float v;
    /*     if (fabs(error) < errorThreshold_) {
           v = 0.0;
        }
        else */
    {
        // 输出限幅
        v = output_v;
        if (v > outputLimit_)
        {
            v = outputLimit_;
        }
        else if (v < -outputLimit_)
        {
            v = -outputLimit_;
        }
    }
    return v;
}

float PIDController::PIDComputeWithFeedforward(float error, float feedforward)
{

    // 增量式PID公式
    float delta_output = kp_ * (error - error_1_) + ki_ * error + kd_ * (error - 2 * error_1_ + error_2_);

    // 前馈 + PID 输出
    float output_v = output_prev_ + feedforward + delta_output;

    // 输出限幅
    if (output_v > outputLimit_)
    {
        output_v = outputLimit_;
    }
    else if (output_v < -outputLimit_)
    {
        output_v = -outputLimit_;
    }

    // 更新误差和输出
    error_2_ = error_1_;
    error_1_ = error;
    output_prev_ = output_v;
    std::cout << "error_2_" << error_2_ << std::endl;

    return output_v;
}

/* float PIDController::PID_increment(float error) {
    // 误差计算
    float error = targetPos - currentPos;

    // 增量式PID公式
    float delta_output = kp_ * (error - error_1_) + ki_ * error + kd_ * (error - 2 * error_1_ + error_2_);

    // 累加增量
    float output_v = output_prev_ + delta_output;

    // 输出限幅
    if (output_v > outputLimit_) {
        output_v = outputLimit_;
    } else if (output_v < -outputLimit_) {
        output_v = -outputLimit_;
    }

    // 更新误差和输出
    error_2_ = error_1_;
    error_1_ = error;
    //output_prev_ = output_v;

    return delta_output;
}
 */

#endif // PID_CONTROLLER_H