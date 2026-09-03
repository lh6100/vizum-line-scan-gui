from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    data_root = LaunchConfiguration("data_root")
    hardware_motion = LaunchConfiguration("allow_hardware_motion")
    return LaunchDescription([
        DeclareLaunchArgument("data_root", default_value=str(Path.cwd() / "data")),
        DeclareLaunchArgument("allow_hardware_motion", default_value="false"),
        DeclareLaunchArgument("srdf_sha256", default_value=""),
        DeclareLaunchArgument("workpiece_frame_version", default_value=""),
        DeclareLaunchArgument("planner_config_sha256", default_value=""),
        Node(package="welding_calibration", executable="calibration_registry_node", output="screen",
             parameters=[{"registry_root": PathJoinSubstitution([data_root, "registry", "calibration"])}]),
        Node(package="welding_calibration", executable="calibration_tf_publisher_node", output="screen"),
        Node(package="welding_data", executable="session_manager_node", output="screen",
             parameters=[{"session_root": PathJoinSubstitution([data_root, "sessions"])}]),
        Node(package="welding_planning", executable="planning_scene_registry_node", output="screen",
             parameters=[{"registry_root": PathJoinSubstitution([data_root, "registry", "planning_scenes"]),
                          "srdf_sha256": LaunchConfiguration("srdf_sha256"),
                          "workpiece_frame_version": LaunchConfiguration("workpiece_frame_version"),
                          "planner_config_sha256": LaunchConfiguration("planner_config_sha256")}]),
        Node(package="welding_execution", executable="robot_command_gateway_node", output="screen",
             parameters=[{"session_root": PathJoinSubstitution([data_root, "sessions"]),
                          "allow_hardware_motion": hardware_motion}]),
        Node(package="welding_workflows", executable="workflow_supervisor_node", output="screen",
             parameters=[{"session_root": PathJoinSubstitution([data_root, "sessions"])}]),
        Node(package="welding_hardware", executable="robot_feedback_bridge_node", output="screen"),
    ])
