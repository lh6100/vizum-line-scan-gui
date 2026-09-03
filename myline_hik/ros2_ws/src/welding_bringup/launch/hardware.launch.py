from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("welding_hardware")) / "config" / "hardware.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("hardware_config", default_value=default_config),
        Node(package="welding_hardware", executable="hik_stereo_camera_node", output="screen",
             parameters=[LaunchConfiguration("hardware_config")]),
        Node(package="welding_hardware", executable="line_laser_safety_node", output="screen",
             parameters=[LaunchConfiguration("hardware_config")]),
    ])
