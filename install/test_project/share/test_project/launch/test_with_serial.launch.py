from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyACM0",
            description="Serial device connected to STM32",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Serial baudrate",
        ),
        Node(
            package="stm32_serial_comm",
            executable="serial_node",
            name="stm32_serial_comm_node",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": baudrate,
            }],
        ),
        Node(
            package="test_project",
            executable="test_node",
            name="test_node",
            output="screen",
        ),
    ])
