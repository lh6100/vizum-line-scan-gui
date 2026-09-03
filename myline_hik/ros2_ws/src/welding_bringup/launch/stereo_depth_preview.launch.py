from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_tuning(project_root):
    path = project_root / "config" / "stereo_depth_tuner.yaml"
    with path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    camera = document.get("camera", {})
    depth = document.get("depth", {})
    multi_band = document.get("multi_band", {})
    ranges = multi_band.get(
        "ranges_mm",
        [[300.0, 600.0], [600.0, 1200.0], [1200.0, 2500.0]],
    )
    if not isinstance(ranges, list) or not ranges:
        raise RuntimeError(f"multi_band.ranges_mm is empty in {path}")
    normalized_ranges = []
    for index, pair in enumerate(ranges):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise RuntimeError(
                f"multi_band.ranges_mm[{index}] must be [minimum_mm, maximum_mm]"
            )
        minimum, maximum = float(pair[0]), float(pair[1])
        if minimum <= 0.0 or maximum <= minimum:
            raise RuntimeError(f"invalid multi-band depth range: {pair}")
        normalized_ranges.append((minimum, maximum))
    return path, camera, depth, multi_band, normalized_ranges


def generate_launch_description():
    bringup_share = Path(get_package_share_directory("welding_bringup"))
    project_root = bringup_share.parents[4]
    tuning_path, camera_tuning, depth_tuning, multi_band, ranges = _load_tuning(
        project_root
    )
    mapping_config = (
        Path(get_package_share_directory("welding_mapping"))
        / "config"
        / "stereo_depth.yaml"
    )
    rviz_config = (
        Path(get_package_share_directory("welding_ui"))
        / "config"
        / "stereo_depth.rviz"
    )

    exposure = LaunchConfiguration("exposure_us")
    left_exposure = LaunchConfiguration("left_exposure_us")
    right_exposure = LaunchConfiguration("right_exposure_us")
    fps = LaunchConfiguration("frames_per_second")
    maximum_skew = LaunchConfiguration("maximum_pair_skew_ms")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "exposure_us",
            default_value="-1.0",
            description="Positive value overrides both tuned camera exposures",
        ),
        DeclareLaunchArgument(
            "left_exposure_us",
            default_value=str(camera_tuning.get("left_exposure_us", 18000.0)),
            description="Left 1.6 MP camera exposure in microseconds",
        ),
        DeclareLaunchArgument(
            "right_exposure_us",
            default_value=str(camera_tuning.get("right_exposure_us", 18000.0)),
            description="Right 1.3 MP camera exposure in microseconds",
        ),
        DeclareLaunchArgument(
            "frames_per_second",
            default_value=str(camera_tuning.get("frames_per_second", 5.0)),
        ),
        DeclareLaunchArgument(
            "maximum_pair_skew_ms",
            default_value=str(camera_tuning.get("maximum_pair_skew_ms", 30.0)),
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        LogInfo(
            msg=(
                f"stereo tuning: {tuning_path}; depth bands: "
                + ", ".join(f"{low:g}-{high:g} mm" for low, high in ranges)
            )
        ),
        Node(
            package="welding_hardware",
            executable="hik_stereo_camera_node",
            name="hik_stereo_camera",
            output="screen",
            parameters=[{
                "left_ip": "192.168.7.45",
                "left_expected_serial": "DA8784601",
                "left_frame_id": "camera_650_optical_frame",
                "left_topic": "/welding_robot/camera_650/image_raw",
                "right_ip": "192.168.1.46",
                "right_expected_serial": "DB0403208",
                "right_frame_id": "camera_450_optical_frame",
                "right_topic": "/welding_robot/camera_450/image_raw",
                "exposure_us": ParameterValue(exposure, value_type=float),
                "left_exposure_us": ParameterValue(
                    left_exposure, value_type=float
                ),
                "right_exposure_us": ParameterValue(
                    right_exposure, value_type=float
                ),
                "gain_db": float(camera_tuning.get("gain_db", 0.0)),
                "frames_per_second": ParameterValue(fps, value_type=float),
                "maximum_pair_skew_ms": ParameterValue(
                    maximum_skew, value_type=float
                ),
            }],
        ),
        Node(
            package="welding_mapping",
            executable="stereo_depth_node",
            name="stereo_depth",
            output="screen",
            parameters=[str(mapping_config), {
                "require_active_calibration": False,
                "expected_calibration_package_id": "",
                "stereo_yaml": str(project_root / "config" / "hik_stereo.yaml"),
                "left_intrinsics_yaml": str(
                    project_root / "config" / "devices" / "scanner_650" / "hik_intrinsics.yaml"
                ),
                "left_handeye_yaml": str(
                    project_root / "config" / "devices" / "scanner_650" / "hik_handeye.yaml"
                ),
                "right_intrinsics_yaml": str(
                    project_root / "config" / "devices" / "scanner_450" / "hik_intrinsics.yaml"
                ),
                "right_handeye_yaml": str(
                    project_root / "config" / "devices" / "scanner_450" / "hik_handeye.yaml"
                ),
                "processing_width": int(depth_tuning.get("processing_width", 612)),
                "processing_height": int(depth_tuning.get("processing_height", 512)),
                "minimum_depth_mm": min(pair[0] for pair in ranges),
                "maximum_depth_mm": max(pair[1] for pair in ranges),
                "block_size": int(depth_tuning.get("block_size", 5)),
                "uniqueness_ratio": int(depth_tuning.get("uniqueness_ratio", 5)),
                "speckle_window_size": int(
                    depth_tuning.get("speckle_window_size", 50)
                ),
                "speckle_range": int(depth_tuning.get("speckle_range", 2)),
                "left_right_maximum_difference_px": int(
                    depth_tuning.get("left_right_maximum_difference_px", 2)
                ),
                "disparity_margin_px": int(
                    depth_tuning.get("disparity_margin_px", 16)
                ),
                "maximum_num_disparities": int(
                    depth_tuning.get("maximum_num_disparities", 512)
                ),
                "enable_left_right_check": bool(
                    depth_tuning.get("enable_left_right_check", False)
                ),
                "enable_clahe": bool(depth_tuning.get("enable_clahe", False)),
                "clahe_clip_limit": float(
                    depth_tuning.get("clahe_clip_limit", 2.0)
                ),
                "multi_band_enabled": bool(multi_band.get("enabled", True)),
                "multi_band_minimum_depths_mm": [pair[0] for pair in ranges],
                "multi_band_maximum_depths_mm": [pair[1] for pair in ranges],
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="stereo_depth_rviz",
            output="screen",
            arguments=["-d", str(rviz_config)],
            condition=IfCondition(use_rviz),
        ),
    ])
