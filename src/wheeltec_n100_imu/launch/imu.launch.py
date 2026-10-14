# wheeltec_n100_imu — imu.launch.py
# 用 by-id，避免和 MBS USB-RS485（Prolific）抢 ttyUSB0。
# 覆盖：ros2 launch wheeltec_n100_imu imu.launch.py port:=/dev/serial/by-id/...
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_N100_BY_ID = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0"
)


def generate_launch_description():
    port = LaunchConfiguration("port")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value=_N100_BY_ID,
                description="N100 IMU serial device (by-id, not ttyUSB0)",
            ),
            Node(
                package="wheeltec_n100_imu",
                executable="imu_node",
                name="imu_node",
                output="screen",
                parameters=[{"serial_port": port}],
            ),
        ]
    )
