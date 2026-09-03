from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("file"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("scale_to_meters", default_value="1.0"),
        Node(package="welding_mapping", executable="pointcloud_file_publisher", output="screen",
             parameters=[{"file": LaunchConfiguration("file"), "frame_id": LaunchConfiguration("frame_id"),
                          "scale_to_meters": LaunchConfiguration("scale_to_meters")}]),
    ])
