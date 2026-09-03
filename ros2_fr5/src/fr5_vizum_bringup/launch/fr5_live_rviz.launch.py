from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_project_file(bringup_share: Path, relative_path: str) -> str:
    """Find a repository file from either a source or install launch path."""
    for directory in (bringup_share, *bringup_share.parents):
        candidate = directory / relative_path
        if candidate.is_file():
            return str(candidate)
    # Keep launch-argument overrides usable when the package is installed elsewhere.
    return relative_path


def find_project_config(bringup_share: Path, filename: str) -> str:
    return find_project_file(bringup_share, str(Path("config") / filename))


def generate_launch_description():
    description_share = Path(get_package_share_directory("fairino_description"))
    bringup_share = Path(get_package_share_directory("fr5_vizum_bringup"))

    urdf_path = description_share / "urdf" / "fairino5_v6.urdf"
    rviz_config = bringup_share / "rviz" / "fr5_live.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    robot_ip = LaunchConfiguration("robot_ip")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    log_pose_period_ms = LaunchConfiguration("log_pose_period_ms")
    base_frame = LaunchConfiguration("base_frame")
    flange_frame = LaunchConfiguration("flange_frame")
    publish_flange_tf = LaunchConfiguration("publish_flange_tf")
    publish_flange_marker = LaunchConfiguration("publish_flange_marker")
    publish_calibrated_frames = LaunchConfiguration("publish_calibrated_frames")
    publish_calibrated_markers = LaunchConfiguration("publish_calibrated_markers")
    publish_laser_planes = LaunchConfiguration("publish_laser_planes")
    tool_config_path = LaunchConfiguration("tool_config_path")
    tcp_frame = LaunchConfiguration("tcp_frame")
    calibration_marker_topic = LaunchConfiguration("calibration_marker_topic")
    laser_650_plane_path = LaunchConfiguration("laser_650_plane_path")
    laser_650_intrinsics_path = LaunchConfiguration("laser_650_intrinsics_path")
    laser_450_plane_path = LaunchConfiguration("laser_450_plane_path")
    laser_450_intrinsics_path = LaunchConfiguration("laser_450_intrinsics_path")
    use_rviz = LaunchConfiguration("use_rviz")

    state_publisher = Node(
        package="fr5_vizum_driver",
        executable="fairino_state_publisher",
        name="fairino_state_publisher",
        output="screen",
        parameters=[{
            "robot_ip": robot_ip,
            "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
            "log_pose_period_ms": ParameterValue(log_pose_period_ms, value_type=int),
            "base_frame": base_frame,
            "flange_frame": flange_frame,
            "publish_flange_tf": ParameterValue(publish_flange_tf, value_type=bool),
            "publish_flange_marker": ParameterValue(
                publish_flange_marker, value_type=bool
            ),
        }],
    )

    calibrated_frames_publisher = Node(
        package="fr5_vizum_bringup",
        executable="calibrated_frames_publisher",
        name="calibrated_frames_publisher",
        output="screen",
        parameters=[{
            "tool_config_path": tool_config_path,
            "flange_frame": flange_frame,
            "tcp_frame": tcp_frame,
            "marker_topic": calibration_marker_topic,
            "publish_markers": ParameterValue(
                publish_calibrated_markers, value_type=bool
            ),
            "publish_laser_planes": ParameterValue(
                publish_laser_planes, value_type=bool
            ),
            "laser_650_plane_path": laser_650_plane_path,
            "laser_650_intrinsics_path": laser_650_intrinsics_path,
            "laser_450_plane_path": laser_450_plane_path,
            "laser_450_intrinsics_path": laser_450_intrinsics_path,
        }],
        condition=IfCondition(publish_calibrated_frames),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.200",
            description="Fairino FR5 controller IP address.",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="20.0",
            description="Joint and flange state publish rate.",
        ),
        DeclareLaunchArgument(
            "log_pose_period_ms",
            default_value="1000",
            description="Terminal flange XYZ/RPY log period; zero disables.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Reference frame for the controller-reported flange pose.",
        ),
        DeclareLaunchArgument(
            "flange_frame",
            default_value="fairino_flange_reported",
            description="Child TF frame for the controller-reported flange pose.",
        ),
        DeclareLaunchArgument(
            "publish_flange_tf",
            default_value="true",
            description="Publish base_frame -> flange_frame TF.",
        ),
        DeclareLaunchArgument(
            "publish_flange_marker",
            default_value="true",
            description="Show live XYZ/RPY text next to the flange in RViz.",
        ),
        DeclareLaunchArgument(
            "publish_calibrated_frames",
            default_value="true",
            description="Publish the calibrated flange -> weld TCP TF.",
        ),
        DeclareLaunchArgument(
            "publish_calibrated_markers",
            default_value="true",
            description="Show the weld-TCP origin and calibrated geometry in RViz.",
        ),
        DeclareLaunchArgument(
            "publish_laser_planes",
            default_value="true",
            description="Show both calibrated HIK line-laser planes in RViz.",
        ),
        DeclareLaunchArgument(
            "tool_config_path",
            default_value=find_project_config(bringup_share, "tool_config.yaml"),
            description="Path to flange -> weld-gun TCP calibration YAML.",
        ),
        DeclareLaunchArgument(
            "tcp_frame",
            default_value="weld_gun_tcp",
            description="TF child frame at the calibrated weld-gun TCP.",
        ),
        DeclareLaunchArgument(
            "calibration_marker_topic",
            default_value="fairino/calibrated_points",
            description="MarkerArray topic for weld-TCP and HIK laser planes.",
        ),
        DeclareLaunchArgument(
            "laser_650_plane_path",
            default_value=find_project_file(
                bringup_share,
                "myline_hik/config/devices/scanner_650/hik_laser_plane.yaml",
            ),
            description="Path to the HIK 650 line-laser plane calibration YAML.",
        ),
        DeclareLaunchArgument(
            "laser_650_intrinsics_path",
            default_value=find_project_file(
                bringup_share,
                "myline_hik/config/devices/scanner_650/hik_intrinsics.yaml",
            ),
            description="Path to the HIK 650 camera intrinsics YAML.",
        ),
        DeclareLaunchArgument(
            "laser_450_plane_path",
            default_value=find_project_file(
                bringup_share,
                "myline_hik/config/devices/scanner_450/hik_laser_plane.yaml",
            ),
            description="Path to the HIK 450 line-laser plane calibration YAML.",
        ),
        DeclareLaunchArgument(
            "laser_450_intrinsics_path",
            default_value=find_project_file(
                bringup_share,
                "myline_hik/config/devices/scanner_450/hik_intrinsics.yaml",
            ),
            description="Path to the HIK 450 camera intrinsics YAML.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 when true.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="fr5_robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 30.0,
            }],
        ),
        state_publisher,
        calibrated_frames_publisher,
        RegisterEventHandler(
            OnProcessExit(
                target_action=state_publisher,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="Fairino state publisher exited; stopping FR5 bringup."
                ))],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=calibrated_frames_publisher,
                on_exit=[EmitEvent(event=Shutdown(
                    reason="Calibration frame publisher exited; stopping FR5 bringup."
                ))],
            ),
            condition=IfCondition(publish_calibrated_frames),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(use_rviz),
        ),
    ])
