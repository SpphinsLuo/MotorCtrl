# Wheeltec N100 IMU（本机接入说明）

独立 ROS 2 包，路径：`motorctrl/src/wheeltec_n100_imu`（不要放进 `biped_hw`）。
上游：https://github.com/NDHANA94/ros2_wheeltec_n100_imu

## 串口

本机 N100 是 **Silicon Labs CP210x**（`lsusb` `10c4:ea60`），枚举为 `/dev/ttyUSB*`，**不是** `ttyACM*`。

MBS 编码器 USB-RS485 是 **Prolific pl2303**，也会变成 `ttyUSB*`。两个口插拔顺序会换号；曾经把编码器工具开到 IMU 的 cp210x 上，解析出 `FC...FD` 垃圾帧。**一律用 by-id。**

- **N100（本机）**  
  `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0`
- **MBS USB-RS485（编码器，不要给 IMU）**  
  `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_BGA_b114J19-if00-port0`
- **`/dev/ttyACM0`、`/dev/ttyACM1` 预留给左右 USB2CAN**，不要把 IMU 接到这两路。

`src/imu_node.cpp` 的 `serial_port` 默认值和 launch 参数 `port` 都已指向上面的 N100 by-id。可用 CLI / launch 覆盖，不必改源码。

## 依赖 `serial`

`package.xml` 声明的是 ROS 包名 **`serial`**（`#include "serial/serial.h"`），不是 Humble 的 `ros-humble-serial-driver`。

Humble apt **没有** `ros-humble-serial`（`ros-humble-serial-driver` 是另一套库，对不上 `find_package(serial)`）。上游 README 要求克隆 [serial-ros2](https://github.com/RoverRobotics-forks/serial-ros2)。本工作空间已放在 `motorctrl/src/serial`。系统需 `libeigen3-dev`（已装）。

## 编译

```bash
cd /home/ye/00_jfl_26.10/MotorCtrl
source /opt/ros/humble/setup.bash
colcon build --packages-select serial wheeltec_n100_imu
# 或：colcon build --packages-up-to wheeltec_n100_imu
source install/setup.bash
```

仅 `--packages-select wheeltec_n100_imu` 时，工作空间里的 `serial` 若尚未编过会失败。

## 运行

```bash
# 默认即 N100 CP210x by-id（不是 ttyUSB0 / ttyACM3）
ros2 launch wheeltec_n100_imu imu.launch.py

# 覆盖串口
ros2 launch wheeltec_n100_imu imu.launch.py \
  port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0

# 或 ros2 run
ros2 run wheeltec_n100_imu imu_node --ros-args \
  -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0
```

编码器（另一条口）：

```bash
ros2 run biped_hw mbs_encoder_recv --iface d --addr 0x1F --baud 2500000
# 默认读 share/biped_hw/config/mbs_encoder.yaml 的 Prolific by-id
```

话题（上游）：`/imu`、`/imu_trueEast`、`/magnetic_field`、`/magnetic_pose_2d`。

## 与 biped_hw 的关系

N100 是 **P4 输入源**，目前 **没有** 接到 `biped_hw` 的总线周期 / 状态估计。本包独立运行，不要改 `motor_ctrl` 遗留代码。
