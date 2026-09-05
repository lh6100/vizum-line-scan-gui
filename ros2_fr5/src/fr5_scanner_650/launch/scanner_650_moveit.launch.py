import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    servo_period_s = LaunchConfiguration("servo_period_s")
    start_camera = LaunchConfiguration("start_camera")
    camera_auto_connect = LaunchConfiguration("camera_auto_connect")
    camera_exposure_us = LaunchConfiguration("camera_exposure_us")
    camera_gain_db = LaunchConfiguration("camera_gain_db")
    intrinsics_path = LaunchConfiguration("intrinsics_path")
    laser_plane_path = LaunchConfiguration("laser_plane_path")
    output_ply = LaunchConfiguration("output_ply")
    checkpoint_path = LaunchConfiguration("checkpoint_path")
    laser_private_key_path = LaunchConfiguration("laser_private_key_path")
    laser_known_hosts_path = LaunchConfiguration("laser_known_hosts_path")
    start_laser_control = LaunchConfiguration("start_laser_control")
    laser_auto_connect = LaunchConfiguration("laser_auto_connect")
    start_reconstruction = LaunchConfiguration("start_reconstruction")
    use_rviz = LaunchConfiguration("use_rviz")
    use_control_gui = LaunchConfiguration("use_control_gui")
    allow_rviz_execution = LaunchConfiguration("allow_rviz_execution")
    rviz_execution_scans = LaunchConfiguration("rviz_execution_scans")
    allow_execution = LaunchConfiguration("allow_execution")
    require_laser_control = LaunchConfiguration("require_laser_control")
    scan_distance_m = LaunchConfiguration("scan_distance_m")
    scan_direction_frame = LaunchConfiguration("scan_direction_frame")
    scan_direction_axis = LaunchConfiguration("scan_direction_axis")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    start_workspace_planner = LaunchConfiguration("start_workspace_planner")
    collision_scene_validated = LaunchConfiguration("collision_scene_validated")
    require_environment_collision_objects = LaunchConfiguration(
        "require_environment_collision_objects"
    )
    start_table_collision_scene = LaunchConfiguration("start_table_collision_scene")
    table_surface_z_m = LaunchConfiguration("table_surface_z_m")
    table_center_x_m = LaunchConfiguration("table_center_x_m")
    table_center_y_m = LaunchConfiguration("table_center_y_m")
    table_size_x_m = LaunchConfiguration("table_size_x_m")
    table_size_y_m = LaunchConfiguration("table_size_y_m")
    table_thickness_m = LaunchConfiguration("table_thickness_m")

    scanner_share = Path(get_package_share_directory("fr5_scanner_650"))
    calibration_dir = scanner_share / "config" / "calibration" / "scanner_650"
    if "VIZUM_DATA_DIR" in os.environ:
        data_root = Path(os.environ["VIZUM_DATA_DIR"]).expanduser()
    else:
        xdg_data_home = Path(
            os.environ.get("XDG_DATA_HOME", str(Path.home() / ".local" / "share"))
        ).expanduser()
        data_root = xdg_data_home / "vizum-line-scan-gui"
    if "VIZUM_CONFIG_DIR" in os.environ:
        config_root = Path(os.environ["VIZUM_CONFIG_DIR"]).expanduser()
    else:
        xdg_config_home = Path(
            os.environ.get("XDG_CONFIG_HOME", str(Path.home() / ".config"))
        ).expanduser()
        config_root = xdg_config_home / "myline_hik"
    scan_data_dir = data_root / "scans" / "scanner_650"
    laser_config_dir = config_root / "laser-control"
    scanner_moveit_dir = scanner_share / "config" / "moveit"
    moveit_config = (
        MoveItConfigsBuilder(
            "fairino5_v6_robot", package_name="fairino5_v6_moveit2_config"
        )
        .robot_description(
            file_path=str(scanner_moveit_dir / "fairino5_v6_robot.urdf.xacro"),
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "servo_period_s": servo_period_s,
            }
        )
        .robot_description_semantic(
            file_path=str(scanner_moveit_dir / "fairino5_v6_robot.srdf")
        )
        .robot_description_kinematics(
            file_path=str(scanner_moveit_dir / "kinematics.yaml")
        )
        .joint_limits(file_path=str(scanner_moveit_dir / "joint_limits.yaml"))
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            # RViz sends an empty planner_id until the user explicitly picks
            # an algorithm.  OMPL accepts that and selects the group's default,
            # which is appropriate for arbitrary interactive-marker positioning.
            # scan_motion_commander independently requests Pilz/LIN below.
            default_planning_pipeline="ompl",
        )
        .pilz_cartesian_limits(
            file_path=str(scanner_moveit_dir / "pilz_cartesian_limits.yaml")
        )
        .trajectory_execution(
            file_path=str(scanner_share / "config" / "moveit_scan_controllers.yaml")
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    scanner_parameters = str(scanner_share / "config" / "scanner_650.yaml")
    controller_parameters = str(scanner_moveit_dir / "ros2_controllers.yaml")
    rviz_config = str(scanner_share / "rviz" / "scanner_650_moveit.rviz")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description, controller_parameters],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fairino5_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                # MoveIt executes through rviz_scan_trajectory_relay.  Its mode
                # decides whether an explicitly executed plan is positioning
                # (laser OFF) or an interlocked scan of that exact trajectory.
                "allow_trajectory_execution": ParameterValue(
                    allow_rviz_execution, value_type=bool
                ),
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 1.0,
                "trajectory_execution.allowed_start_tolerance": 0.01,
            },
        ],
    )
    table_collision_scene = Node(
        package="fr5_scanner_650",
        executable="table_collision_scene_node",
        name="table_collision_scene",
        output="screen",
        parameters=[
            scanner_parameters,
            {
                "surface_z_m": ParameterValue(table_surface_z_m, value_type=float),
                "center_x_m": ParameterValue(table_center_x_m, value_type=float),
                "center_y_m": ParameterValue(table_center_y_m, value_type=float),
                "size_x_m": ParameterValue(table_size_x_m, value_type=float),
                "size_y_m": ParameterValue(table_size_y_m, value_type=float),
                "thickness_m": ParameterValue(table_thickness_m, value_type=float),
            },
        ],
        condition=IfCondition(start_table_collision_scene),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="scanner_650_rviz",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )
    control_gui = Node(
        package="fr5_scanner_650",
        executable="scanner_control_gui",
        name="scanner_650_control_gui",
        output="screen",
        condition=IfCondition(use_control_gui),
    )
    camera = Node(
        package="fr5_scanner_650",
        executable="hik_single_camera_node",
        name="hik_single_camera",
        output="screen",
        parameters=[
            scanner_parameters,
            {
                "exposure_us": ParameterValue(camera_exposure_us, value_type=float),
                "gain_db": ParameterValue(camera_gain_db, value_type=float),
                "intrinsics_path": ParameterValue(intrinsics_path, value_type=str),
                "auto_connect": ParameterValue(camera_auto_connect, value_type=bool),
            },
        ],
        condition=IfCondition(start_camera),
    )
    reconstruction = Node(
        package="fr5_scanner_650",
        executable="line_laser_reconstruction_node",
        name="line_laser_reconstruction",
        output="screen",
        parameters=[
            scanner_parameters,
            {
                "intrinsics_path": ParameterValue(intrinsics_path, value_type=str),
                "laser_plane_path": ParameterValue(laser_plane_path, value_type=str),
                "output_ply": ParameterValue(output_ply, value_type=str),
            },
        ],
        condition=IfCondition(start_reconstruction),
    )
    laser_control = Node(
        package="fr5_scanner_650",
        executable="line_laser_control_node",
        name="line_laser_control",
        output="screen",
        parameters=[
            scanner_parameters,
            {
                "auto_connect": ParameterValue(
                    laser_auto_connect, value_type=bool
                ),
                "private_key_path": ParameterValue(
                    laser_private_key_path, value_type=str
                ),
                "known_hosts_path": ParameterValue(
                    laser_known_hosts_path, value_type=str
                ),
            },
        ],
        condition=IfCondition(start_laser_control),
    )
    rviz_scan_relay = Node(
        package="fr5_scanner_650",
        executable="rviz_scan_trajectory_relay",
        name="rviz_scan_trajectory_relay",
        output="screen",
        parameters=[
            {
                "allow_execution": ParameterValue(
                    allow_rviz_execution, value_type=bool
                ),
                "scan_on_execute": ParameterValue(
                    rviz_execution_scans, value_type=bool
                ),
                "require_laser_control": ParameterValue(
                    require_laser_control, value_type=bool
                ),
            },
            scanner_parameters,
        ],
    )
    commander = Node(
        package="fr5_scanner_650",
        executable="scan_motion_commander",
        name="scan_motion_commander",
        output="screen",
        parameters=[
            scanner_parameters,
            moveit_config.to_dict(),
            {
                "allow_execution": ParameterValue(allow_execution, value_type=bool),
                "require_laser_control": ParameterValue(
                    require_laser_control, value_type=bool
                ),
                "distance_m": ParameterValue(scan_distance_m, value_type=float),
                "direction_frame": ParameterValue(
                    scan_direction_frame, value_type=str
                ),
                "direction_axis": ParameterValue(
                    scan_direction_axis, value_type=str
                ),
                "velocity_scaling": ParameterValue(velocity_scaling, value_type=float),
                "acceleration_scaling": ParameterValue(
                    acceleration_scaling, value_type=float
                ),
                "maintain_base_height": True,
            },
        ],
    )
    workspace_planner = Node(
        package="fr5_scanner_650",
        executable="workspace_coarse_scan_planner",
        name="workspace_coarse_scan_planner",
        output="screen",
        parameters=[
            scanner_parameters,
            moveit_config.to_dict(),
            # Launch-time safety gates must be last. ROS 2 resolves duplicate
            # parameters in list order; putting scanner_650.yaml afterwards
            # silently restored collision_scene_validated to its fail-closed
            # default even when the operator explicitly passed :=true.
            {
                "allow_execution": ParameterValue(allow_execution, value_type=bool),
                "require_laser_control": ParameterValue(
                    require_laser_control, value_type=bool
                ),
                "collision_scene_validated": ParameterValue(
                    collision_scene_validated, value_type=bool
                ),
                "require_environment_collision_objects": ParameterValue(
                    require_environment_collision_objects, value_type=bool
                ),
                "checkpoint_path": ParameterValue(checkpoint_path, value_type=str),
            },
        ],
        condition=IfCondition(start_workspace_planner),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                description="Use ros2_control GenericSystem instead of connecting to FR5",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.1.200",
                description="Fairino controller IP used only with real hardware",
            ),
            DeclareLaunchArgument(
                "servo_period_s",
                default_value="0.008",
                description="Fairino ServoJ period (125 Hz = 0.008 s)",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description=(
                    "Start the Hikrobot connection-service node; this alone does not "
                    "open the camera"
                ),
            ),
            DeclareLaunchArgument(
                "camera_auto_connect",
                default_value="false",
                description="Connect and begin camera streaming during launch",
            ),
            DeclareLaunchArgument(
                "camera_exposure_us",
                default_value="1825.0",
                description="Hikrobot exposure in microseconds, applied when the camera starts",
            ),
            DeclareLaunchArgument(
                "camera_gain_db",
                default_value="0.0",
                description="Hikrobot analog gain in dB, applied when the camera starts",
            ),
            DeclareLaunchArgument(
                "intrinsics_path",
                default_value=str(calibration_dir / "hik_intrinsics.yaml"),
                description="Scanner 650 intrinsics YAML",
            ),
            DeclareLaunchArgument(
                "laser_plane_path",
                default_value=str(calibration_dir / "hik_laser_plane.yaml"),
                description="Scanner 650 laser-plane YAML",
            ),
            DeclareLaunchArgument(
                "output_ply",
                default_value=str(scan_data_dir / "scan_voxel.ply"),
                description=(
                    "Compatibility PLY path; its parent is the per-session scan "
                    "archive root"
                ),
            ),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=str(scan_data_dir / "workspace_coarse_checkpoint.yaml"),
                description="Workspace-scan resume checkpoint",
            ),
            DeclareLaunchArgument(
                "laser_private_key_path",
                default_value=str(laser_config_dir / "id_ed25519"),
                description="SSH private key for the fail-safe line-laser controller",
            ),
            DeclareLaunchArgument(
                "laser_known_hosts_path",
                default_value=str(laser_config_dir / "known_hosts"),
                description="SSH known_hosts for the line-laser controller",
            ),
            DeclareLaunchArgument(
                "start_reconstruction",
                default_value="true",
                description="Start line-laser profile and cloud reconstruction",
            ),
            DeclareLaunchArgument(
                "start_laser_control",
                default_value="true",
                description=(
                    "Start the laser connection-service node; this alone does not "
                    "connect or enable either TTL"
                ),
            ),
            DeclareLaunchArgument(
                "laser_auto_connect",
                default_value="false",
                description="Connect to the fail-safe LubanCat TTL daemon during launch",
            ),
            DeclareLaunchArgument(
                "require_laser_control",
                default_value="false",
                description="Refuse execution unless the 650 nm TTL is confirmed",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_control_gui",
                default_value="true",
                description="Start the standalone FR5 line-laser operator console",
            ),
            DeclareLaunchArgument(
                "allow_rviz_execution",
                default_value="false",
                description=(
                    "Allow RViz MotionPlanning Execute through the trajectory relay; "
                    "rviz_execution_scans selects positioning or scan mode"
                ),
            ),
            DeclareLaunchArgument(
                "rviz_execution_scans",
                default_value="false",
                description=(
                    "Make RViz Execute scan its complete planned trajectory with "
                    "laser and accumulation interlocks"
                ),
            ),
            DeclareLaunchArgument(
                "allow_execution",
                default_value="false",
                description="Unlock the explicit execute service; planning stays available",
            ),
            DeclareLaunchArgument(
                "start_workspace_planner",
                default_value="true",
                description="Start the initial-height single-plane sector scan planner",
            ),
            DeclareLaunchArgument(
                "collision_scene_validated",
                default_value="false",
                description=(
                    "Operator assertion that robot/scanner/cable/fixture/workpiece/"
                    "safety-zone collision geometry is loaded and validated"
                ),
            ),
            DeclareLaunchArgument(
                "require_environment_collision_objects",
                default_value="true",
                description="Refuse approval when the validated world has no collision objects",
            ),
            DeclareLaunchArgument(
                "start_table_collision_scene",
                default_value="false",
                description="Load the explicitly measured tabletop box into MoveIt",
            ),
            DeclareLaunchArgument("table_surface_z_m", default_value="-0.100"),
            DeclareLaunchArgument("table_center_x_m", default_value="0.200"),
            DeclareLaunchArgument("table_center_y_m", default_value="0.200"),
            DeclareLaunchArgument("table_size_x_m", default_value="0.300"),
            DeclareLaunchArgument("table_size_y_m", default_value="0.200"),
            DeclareLaunchArgument("table_thickness_m", default_value="0.010"),
            DeclareLaunchArgument(
                "scan_distance_m",
                default_value="0.05",
                description="Signed LIN scan distance for the scanner TCP",
            ),
            DeclareLaunchArgument(
                "scan_direction_frame",
                default_value="camera",
                description="Interpret scan axis in camera, tool or base frame",
            ),
            DeclareLaunchArgument(
                "scan_direction_axis",
                default_value="+y",
                description="Camera +Y/-Y is the calibrated cross-stripe scan direction",
            ),
            DeclareLaunchArgument("velocity_scaling", default_value="0.20"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.20"),
            robot_state_publisher,
            ros2_control,
            joint_state_broadcaster,
            trajectory_controller,
            move_group,
            table_collision_scene,
            reconstruction,
            laser_control,
            camera,
            rviz_scan_relay,
            commander,
            workspace_planner,
            rviz,
            control_gui,
        ]
    )
