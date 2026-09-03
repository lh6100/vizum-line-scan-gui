from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = ["calibration_package_id", "stereo_yaml", "left_intrinsics_yaml", "left_handeye_yaml",
             "right_intrinsics_yaml", "right_handeye_yaml"]
    arguments = [DeclareLaunchArgument(name) for name in names]
    arguments += [
        DeclareLaunchArgument("self_filter_valid", default_value="false"),
        DeclareLaunchArgument("map_frame", default_value="base_link"),
    ]
    arguments += [
        Node(package="welding_mapping", executable="stereo_depth_node", name="stereo_depth", output="screen",
             parameters=[{
                 "expected_calibration_package_id": LaunchConfiguration("calibration_package_id"),
                 "stereo_yaml": LaunchConfiguration("stereo_yaml"),
                 "left_intrinsics_yaml": LaunchConfiguration("left_intrinsics_yaml"),
                 "left_handeye_yaml": LaunchConfiguration("left_handeye_yaml"),
                 "right_intrinsics_yaml": LaunchConfiguration("right_intrinsics_yaml"),
                 "right_handeye_yaml": LaunchConfiguration("right_handeye_yaml"),
             }]),
        Node(package="welding_mapping", executable="octomap_builder_node", name="octomap_builder", output="screen",
             parameters=[{"self_filter_valid": LaunchConfiguration("self_filter_valid"),
                          "map_frame": LaunchConfiguration("map_frame")}]),
    ]
    return LaunchDescription(arguments)
