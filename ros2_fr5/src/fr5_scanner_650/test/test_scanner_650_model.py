import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


THIS_FILE = Path(__file__).resolve()
WORKSPACE = THIS_FILE.parents[3]
PROJECT = WORKSPACE.parent
URDF_PATH = WORKSPACE / "src/fairino_description/urdf/fairino5_v6.urdf"
SRDF_PATH = (
    WORKSPACE
    / "src/fr5_scanner_650/config/moveit/fairino5_v6_robot.srdf"
)
CALIBRATION = PROJECT / "myline_hik/config/devices/scanner_650"
SCANNER_CONFIG = WORKSPACE / "src/fr5_scanner_650/config/scanner_650.yaml"
ROSBAG_QOS_CONFIG = (
    WORKSPACE / "src/fr5_scanner_650/config/scanner_650_rosbag_qos.yaml"
)
RVIZ_CONFIG = WORKSPACE / "src/fr5_scanner_650/rviz/scanner_650_moveit.rviz"
RECONSTRUCTION_SOURCE = (
    WORKSPACE / "src/fr5_scanner_650/src/line_laser_reconstruction_node.cpp"
)
LAUNCH_FILE = WORKSPACE / "src/fr5_scanner_650/launch/scanner_650_moveit.launch.py"


def _joint(root, name):
    return next(joint for joint in root.findall("joint") if joint.attrib["name"] == name)


def _rpy_matrix(roll, pitch, yaw):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def test_urdf_camera_transform_is_the_calibrated_handeye():
    handeye = yaml.safe_load((CALIBRATION / "hik_handeye.yaml").read_text())
    transform = handeye["T_flange_camera"]
    root = ET.parse(URDF_PATH).getroot()
    joint = _joint(root, "fairino_flange_model_to_hik_camera_optical")
    origin = joint.find("origin")
    xyz = [float(value) for value in origin.attrib["xyz"].split()]
    rpy = [float(value) for value in origin.attrib["rpy"].split()]

    assert joint.find("parent").attrib["link"] == "fairino_flange_model"
    assert joint.find("child").attrib["link"] == handeye["child_frame"]
    for actual, expected_mm in zip(xyz, [transform[3], transform[7], transform[11]]):
        assert math.isclose(actual, expected_mm * 0.001, abs_tol=1.0e-12)
    urdf_rotation = _rpy_matrix(*rpy)
    calibrated_rotation = [transform[0:3], transform[4:7], transform[8:11]]
    for actual_row, expected_row in zip(urdf_rotation, calibrated_rotation):
        for actual, expected in zip(actual_row, expected_row):
            assert math.isclose(actual, expected, abs_tol=1.0e-9)


def test_scan_tcp_lies_on_calibrated_laser_plane_and_inside_valid_range():
    laser = yaml.safe_load((CALIBRATION / "hik_laser_plane.yaml").read_text())
    nx, ny, nz, d_mm = laser["plane"]["coefficients"]
    root = ET.parse(URDF_PATH).getroot()
    joint = _joint(root, "hik_camera_optical_to_scanner_650_scan_tcp")
    x_m, y_m, z_m = [float(value) for value in joint.find("origin").attrib["xyz"].split()]
    residual_mm = nx * x_m * 1000.0 + ny * y_m * 1000.0 + nz * z_m * 1000.0 + d_mm

    assert joint.find("parent").attrib["link"] == "hik_camera_optical_frame"
    assert joint.find("child").attrib["link"] == "scanner_650_scan_tcp"
    assert abs(residual_mm) < 1.0e-6
    assert laser["validity"]["camera_z_min_mm"] <= z_m * 1000.0
    assert z_m * 1000.0 <= laser["validity"]["camera_z_max_mm"]


def test_moveit_group_ends_at_scan_tcp_and_scanner_has_collision_geometry():
    srdf = ET.parse(SRDF_PATH).getroot()
    group = next(item for item in srdf.findall("group") if item.attrib["name"] == "fairino5_v6_group")
    chain = group.find("chain")
    assert chain.attrib == {"base_link": "base_link", "tip_link": "scanner_650_scan_tcp"}

    urdf = ET.parse(URDF_PATH).getroot()
    scanner = next(link for link in urdf.findall("link") if link.attrib["name"] == "scanner_650_body")
    box = scanner.find("collision/geometry/box")
    assert box is not None
    assert all(float(size) > 0.0 for size in box.attrib["size"].split())


def test_measured_table_collision_box_has_requested_top_surface_and_dimensions():
    config = yaml.safe_load(SCANNER_CONFIG.read_text())
    table = config["table_collision_scene"]["ros__parameters"]

    assert table["frame_id"] == "base_link"
    assert math.isclose(table["surface_z_m"], -0.100, abs_tol=1.0e-12)
    assert math.isclose(table["center_x_m"], 0.200, abs_tol=1.0e-12)
    assert math.isclose(table["center_y_m"], 0.200, abs_tol=1.0e-12)
    assert math.isclose(table["size_x_m"], 0.300, abs_tol=1.0e-12)
    assert math.isclose(table["size_y_m"], 0.200, abs_tol=1.0e-12)
    assert math.isclose(table["thickness_m"], 0.010, abs_tol=1.0e-12)
    expected_center_z = table["surface_z_m"] - 0.5 * table["thickness_m"]
    assert math.isclose(expected_center_z, -0.105, abs_tol=1.0e-12)


def test_motion_compensation_fails_closed_on_unsynchronized_frames():
    config = yaml.safe_load(SCANNER_CONFIG.read_text())
    camera = config["hik_single_camera"]["ros__parameters"]
    reconstruction = config["line_laser_reconstruction"]["ros__parameters"]

    assert camera["camera_timestamp_reference"] == "exposure_start"
    assert camera["require_device_timestamp_mapping"] is True
    assert camera["camera_clock_mapping_window"] >= 30
    assert 2 <= camera["ros_publish_queue_capacity"] < camera["image_pool_capacity"]
    assert camera["camera_mapping_max_residual_us"] > 0.0
    assert camera["connection_service_name"] == "/scanner_650/set_camera_connected"
    assert camera["connection_timeout_s"] >= 1.0
    assert camera["auto_connect"] is False
    assert reconstruction["tf_lookup_timeout_s"] > 0.0
    assert (
        reconstruction["maximum_image_age_s"]
        > reconstruction["tf_lookup_timeout_s"]
    )


def test_gui_device_connection_services_are_explicit_and_separate_from_laser_output():
    config = yaml.safe_load(SCANNER_CONFIG.read_text())
    camera = config["hik_single_camera"]["ros__parameters"]
    laser = config["line_laser_control"]["ros__parameters"]

    assert camera["connection_service_name"] == "/scanner_650/set_camera_connected"
    assert laser["connection_service_name"] == "/scanner_650/set_laser_connected"
    assert laser["service_name"] == "/scanner_650/set_laser"
    assert laser["connection_service_name"] != laser["service_name"]
    assert laser["connection_timeout_s"] >= laser["command_timeout_s"]
    assert laser["auto_connect"] is False


def test_accumulated_cloud_is_expressed_in_robot_base_frame():
    config = yaml.safe_load(SCANNER_CONFIG.read_text())
    reconstruction = config["line_laser_reconstruction"]["ros__parameters"]

    assert reconstruction["camera_frame"] == "hik_camera_optical_frame"
    assert reconstruction["output_frame"] == "base_link"


def test_reconstruction_matches_proven_scanner_650_continuous_profile():
    config = yaml.safe_load(SCANNER_CONFIG.read_text())
    camera = config["hik_single_camera"]["ros__parameters"]
    reconstruction = config["line_laser_reconstruction"]["ros__parameters"]

    assert math.isclose(camera["frames_per_second"], 60.0, abs_tol=1.0e-12)
    assert reconstruction["centerline_mode"] == "legacy"
    assert reconstruction["reconstruction_queue_capacity"] >= 32
    assert reconstruction["reconstruction_worker_threads"] >= 2
    assert reconstruction["clear_cloud_on_accumulation_start"] is True
    assert reconstruction["accumulation_drain_timeout_s"] > 0.0
    assert reconstruction["scan_publish_period_s"] >= 0.25


def test_rviz_and_saved_ply_share_base_height_rgb_contract():
    rviz = RVIZ_CONFIG.read_text()
    accumulated_display = rviz.split("Name: Accumulated scan cloud", 1)[1].split(
        "- Class:", 1
    )[0]
    assert "Color Transformer: RGB8" in accumulated_display
    assert "Color Transformer: AxisColor" not in accumulated_display

    source = RECONSTRUCTION_SOURCE.read_text()
    assert '"rgb", 1, sensor_msgs::msg::PointField::FLOAT32' in source
    assert "format binary_little_endian 1.0" in source
    assert "comment units millimeter" in source
    xyz = source.index("property double x")
    rgb = source.index("property uchar red", xyz)
    scalars = source.index("property float confidence", rgb)
    assert xyz < rgb < scalars


def test_rosbag_preserves_reliable_60_fps_image_stream():
    qos = yaml.safe_load(ROSBAG_QOS_CONFIG.read_text())
    image = qos["/scanner_650/image_raw"]
    assert image["reliability"] == "reliable"
    assert image["depth"] >= 32


def test_runtime_configuration_is_checkout_portable():
    config_text = SCANNER_CONFIG.read_text()
    launch_text = LAUNCH_FILE.read_text()

    assert "/home/zhulong" not in config_text
    assert "/home/zhulong" not in launch_text
    assert 'scanner_share / "config" / "calibration" / "scanner_650"' in launch_text
    assert "VIZUM_DATA_DIR" in launch_text
    assert "VIZUM_CONFIG_DIR" in launch_text
