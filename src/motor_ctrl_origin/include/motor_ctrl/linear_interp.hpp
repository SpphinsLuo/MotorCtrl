// linear_interp.hpp
#pragma once

#include <vector>
#include <functional>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

/**
 * @brief 线性插值并发送关节指令到电机
 *
 * 将每个关节从初始位置 q0 平滑过渡到目标位置 qf，
 * 总用时 tf，控制周期 dt。在每个插值点，会调用用户
 * 提供的 sendFunc 回调，将当前的关节位置命令下发给电机。
 *
 * @param q0        初始关节位置向量 (rad)，长度为 n
 * @param qf        目标关节位置向量 (rad)，长度必须与 q0 相同
 * @param tf        总过渡时间 (s)，必须 > 0
 * @param dt        控制周期 (s)，必须 > 0 且 <= tf
 * @param sendFunc  发送接口回调，签名 void(const std::vector<double>& q_cmd)
 *                  例如串口/CAN/CANopen/EtherCAT 驱动代码
 * @return true     如果成功执行了整个插值过程
 * @return false    如果输入参数有误（如 q0 和 qf 大小不一致、tf 或 dt 不合法）
 */
bool linearInterpAndSend(
  const std::vector<double>& q0,
  const std::vector<double>& qf,
  double tf,
  double dt,
  const std::function<void(const std::vector<double>&)>& sendFunc)
{
    // 检查输入合法性
    if (q0.size() != qf.size()) {
        std::cerr << "[Error] q0 and qf must have the same length.\n";
        return false;
    }
    if (tf <= 0.0 || dt <= 0.0 || dt > tf) {
        std::cerr << "[Error] Invalid timing parameters: tf=" << tf << ", dt=" << dt << "\n";
        return false;
    }

    // 计算步数，向上取整以包含终点
    int steps = static_cast<int>(std::ceil(tf / dt));

    // 主循环：每个 dt 发送一次插值点
    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        if (t > tf) t = tf;                  // 最后一步可能 t > tf，截断
        double alpha = t / tf;               // 插值系数 ∈ [0,1]

        // 线性插值公式：q = q0 + α * (qf - q0)
        std::vector<double> q_cmd(q0.size());
        for (size_t j = 0; j < q0.size(); ++j) {
            q_cmd[j] = q0[j] + alpha * (qf[j] - q0[j]);
        }

        // 调用用户回调，将本周期的关节命令下发
        sendFunc(q_cmd);

        // 等待下一个控制周期
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    return true;
}
