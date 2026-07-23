import argparse
import importlib.util
import math
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("build_twin.py")
SPEC = importlib.util.spec_from_file_location("build_twin", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_handeye_quaternion_is_normalized():
    handeye = MODULE.load_yaml(MODULE.DEFAULT_HANDEYE)
    matrix = MODULE.finite_float_list(handeye["T_flange_camera"], 16, "T_flange_camera")
    MODULE.validate_rigid_transform(matrix, "T_flange_camera")
    quaternion = MODULE.quaternion_xyzw_from_rotation(matrix)
    assert math.isclose(sum(value * value for value in quaternion), 1.0, abs_tol=1.0e-10)


def test_laser_visualization_points_lie_on_plane():
    laser = MODULE.load_yaml(MODULE.DEFAULT_LASER)
    coefficients = MODULE.finite_float_list(laser["plane"]["coefficients"], 4, "plane")
    coefficients[3] /= 1000.0
    corners = MODULE.laser_plane_corners(coefficients, 0.50, 0.80, 0.30)
    for point in corners:
        residual = sum(coefficients[index] * point[index] for index in range(3)) + coefficients[3]
        assert abs(residual) < 1.0e-12


def test_camera_usd_parameters_reproduce_focal_lengths():
    intrinsics = MODULE.load_yaml(MODULE.DEFAULT_INTRINSICS)
    matrix = MODULE.finite_float_list(intrinsics["camera_matrix"]["data"], 9, "K")
    width = intrinsics["image_width"]
    height = intrinsics["image_height"]
    physical_focal_mm = 6.116
    params = MODULE.camera_usd_parameters(matrix, width, height, physical_focal_mm)
    assert math.isclose(width * params["focal_length"] / params["horizontal_aperture"], matrix[0])
    assert math.isclose(height * params["focal_length"] / params["vertical_aperture"], matrix[4])
    # At metersPerUnit=1, USD camera attributes use tenths of a meter.
    assert math.isclose(params["focal_length"], physical_focal_mm / 100.0)


def test_calibration_hash_chain_matches_current_intrinsics():
    intrinsics_hash = MODULE.sha256_file(MODULE.DEFAULT_INTRINSICS)
    handeye = MODULE.load_yaml(MODULE.DEFAULT_HANDEYE)
    laser = MODULE.load_yaml(MODULE.DEFAULT_LASER)
    assert handeye["sources"]["intrinsics_sha256"] == intrinsics_hash
    assert laser["intrinsics"]["sha256"] == intrinsics_hash


def test_asset_tree_hash_covers_relative_names_and_contents(tmp_path):
    (tmp_path / "payloads").mkdir()
    root = tmp_path / "robot.usda"
    payload = tmp_path / "payloads" / "geometry.usda"
    root.write_text("root", encoding="utf-8")
    payload.write_text("geometry-v1", encoding="utf-8")
    first = MODULE.sha256_tree(tmp_path)
    payload.write_text("geometry-v2", encoding="utf-8")
    second = MODULE.sha256_tree(tmp_path)
    assert first != second


def test_proxy_frame_direction_is_explicit():
    config = MODULE.load_yaml(MODULE.DEFAULT_TWIN_CONFIG)
    proxy = config["wrist3_to_reported_flange"]
    assert proxy["transform_convention"] == MODULE.TRANSFORM_CONVENTION
    assert proxy["parent_frame"] == "wrist3_link"
    assert proxy["child_frame"] == "fairino_flange_reported"
    assert proxy["verified"] is False


def test_invalid_rigid_transform_is_rejected():
    reflection = [-1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  0.0, 0.0, 0.0, 1.0]
    with pytest.raises(MODULE.TwinConfigError, match="determinant"):
        MODULE.validate_rigid_transform(reflection, "reflection")


def test_non_finite_plane_bounds_are_rejected():
    with pytest.raises(MODULE.TwinConfigError, match="invalid laser plane"):
        MODULE.laser_plane_corners([0.0, 1.0, 0.0, 0.0], 0.5, 0.8, math.nan)


def test_opencv_pinhole_api_is_authored_as_non_custom_schema(tmp_path):
    from pxr import Usd, UsdGeom

    output = tmp_path / "camera.usda"
    stage = Usd.Stage.CreateNew(str(output))
    camera = UsdGeom.Camera.Define(stage, "/Camera")
    intrinsic = [1000.0, 0.0, 700.0, 0.0, 1001.0, 500.0, 0.0, 0.0, 1.0]
    distortion = [-0.1, 0.2, 0.001, -0.002, -0.3]
    MODULE._author_opencv_pinhole_api(camera.GetPrim(), intrinsic, distortion, 1440, 1080)
    stage.GetRootLayer().Save()

    reopened = Usd.Stage.Open(str(output))
    prim = reopened.GetPrimAtPath("/Camera")
    assert MODULE.OPENCV_PINHOLE_API in prim.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
    prefix = "omni:lensdistortion:opencvPinhole:"
    assert tuple(prim.GetAttribute(f"{prefix}imageSize").Get()) == (1440, 1080)
    assert prim.GetAttribute(f"{prefix}fx").Get() == pytest.approx(1000.0)
    assert prim.GetAttribute(f"{prefix}k3").Get() == pytest.approx(-0.3)
    assert not prim.GetAttribute(f"{prefix}fx").IsCustom()


def test_failed_build_preserves_previous_output_and_removes_temporary_file(tmp_path):
    from pxr import Usd

    robot_usd = tmp_path / "invalid_robot.usda"
    robot_stage = Usd.Stage.CreateNew(str(robot_usd))
    robot_stage.SetDefaultPrim(robot_stage.DefinePrim("/invalid_robot", "Xform"))
    robot_stage.GetRootLayer().Save()

    output = tmp_path / "twin.usda"
    output.write_text("previous-good-output", encoding="utf-8")
    args = argparse.Namespace(
        robot_usd=robot_usd,
        robot_source_hash="0" * 64,
        output=output,
        intrinsics=MODULE.DEFAULT_INTRINSICS,
        handeye=MODULE.DEFAULT_HANDEYE,
        laser=MODULE.DEFAULT_LASER,
        twin_config=MODULE.DEFAULT_TWIN_CONFIG,
    )
    with pytest.raises(MODULE.TwinConfigError, match="terminal link prim"):
        MODULE.build_stage(args)

    assert output.read_text(encoding="utf-8") == "previous-good-output"
    assert not list(tmp_path.glob(".twin.tmp-*.usda"))
