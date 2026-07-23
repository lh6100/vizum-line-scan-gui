#!/usr/bin/env python3
"""Build an FR5 + calibrated Hik camera + analytic line-laser USD stage."""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import uuid
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_USD = REPO_ROOT / "isaacSim/generated/fr5_hik_twin/fr5_hik_line_laser_twin.usda"
DEFAULT_INTRINSICS = REPO_ROOT / "myline_hik/config/hik_intrinsics.yaml"
DEFAULT_HANDEYE = REPO_ROOT / "myline_hik/config/hik_handeye.yaml"
DEFAULT_LASER = REPO_ROOT / "myline_hik/config/hik_laser_plane.yaml"
DEFAULT_TWIN_CONFIG = Path(__file__).with_name("config.yaml")

WRIST3_PATH = (
    "/World/FR5/Geometry/base_link/shoulder_link/upperarm_link/forearm_link/"
    "wrist1_link/wrist2_link/wrist3_link"
)
CAMERA_PATH = f"{WRIST3_PATH}/fairino_flange_reported_PROXY/hik_camera_optical_frame/HikCamera"
LASER_PLANE_PATH = (
    f"{WRIST3_PATH}/fairino_flange_reported_PROXY/"
    "hik_camera_optical_frame/LaserPlaneVisualization"
)
TRANSFORM_CONVENTION = "T_parent_child maps child coordinates into parent coordinates"
OPENCV_PINHOLE_API = "OmniLensDistortionOpenCvPinholeAPI"
BUILD_CONTRACT_VERSION = 2
RENDER_PRODUCT_PATH = "/World/Render/HikCameraRenderProduct"
RENDER_SETTINGS_PATH = "/World/Render/RenderSettings"
ROBOT_CONVERSION_CONTRACT = (
    "fr5-urdf-v3;fix_base=true;merge_fixed_joints=false;"
    "joint_stiffness=1000;joint_damping=50;joint_target_type=position;viz=none"
)


class TwinConfigError(ValueError):
    """Raised when an input calibration cannot define a safe twin."""


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise TwinConfigError(f"cannot read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise TwinConfigError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise TwinConfigError(f"{path} must contain a YAML mapping")
    return value


def required_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TwinConfigError(f"{label} must be a YAML mapping")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise TwinConfigError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def sha256_tree(root: Path) -> str:
    """Hash all files below a directory using canonical relative names."""
    resolved = root.resolve()
    if not resolved.is_dir():
        raise TwinConfigError(f"cannot hash missing directory: {resolved}")
    files = sorted(
        (path for path in resolved.rglob("*") if path.is_file()),
        key=lambda path: path.relative_to(resolved).as_posix(),
    )
    if not files:
        raise TwinConfigError(f"cannot hash empty directory: {resolved}")
    digest = hashlib.sha256()
    for path in files:
        relative_name = path.relative_to(resolved).as_posix()
        digest.update(relative_name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256_file(path).encode("ascii"))
        digest.update(b"\0")
    return digest.hexdigest()


def robot_source_fingerprint(
    repo_root: Path, isaaclab_root: Path, isaac_sim_version: str
) -> str:
    """Hash the FR5 sources and the exact conversion tool contract."""
    urdf = repo_root / "ros2_fr5/src/fairino_description/urdf/fairino5_v6.urdf"
    mesh_dir = repo_root / "ros2_fr5/src/fairino_description/meshes/fairino5_v6"
    converter = isaaclab_root / "scripts/tools/convert_urdf.py"
    version_file = isaaclab_root / "VERSION"
    try:
        isaaclab_version = version_file.read_text(encoding="utf-8").strip()
    except OSError as exc:
        raise TwinConfigError(f"cannot read {version_file}: {exc}") from exc
    meshes = sorted(mesh_dir.glob("*.STL"), key=lambda path: path.name)
    if not urdf.is_file() or not converter.is_file() or not meshes:
        raise TwinConfigError("FR5 sources or Isaac Lab URDF converter are incomplete")

    entries = [
        ("conversion-contract", ROBOT_CONVERSION_CONTRACT),
        ("isaac-lab-version", isaaclab_version),
        ("isaac-sim-version", isaac_sim_version),
        ("converter", sha256_file(converter)),
        ("urdf/fairino5_v6.urdf", sha256_file(urdf)),
    ]
    entries.extend((f"meshes/{path.name}", sha256_file(path)) for path in meshes)
    digest = hashlib.sha256()
    for label, value in entries:
        digest.update(label.encode("utf-8"))
        digest.update(b"\0")
        digest.update(value.encode("utf-8"))
        digest.update(b"\0")
    return digest.hexdigest()


def require_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise TwinConfigError(f"{label}: expected {expected!r}, got {actual!r}")


def positive_int(value: Any, label: str) -> int:
    if isinstance(value, bool):
        raise TwinConfigError(f"{label} must be a positive integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise TwinConfigError(f"{label} must be a positive integer") from exc
    if result <= 0 or result != value:
        raise TwinConfigError(f"{label} must be a positive integer")
    return result


def finite_float_list(value: Any, count: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != count:
        raise TwinConfigError(f"{label} must contain exactly {count} numbers")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool):
            raise TwinConfigError(f"{label} must contain only finite numbers")
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise TwinConfigError(f"{label} must contain only finite numbers") from exc
        if not math.isfinite(number):
            raise TwinConfigError(f"{label} must contain only finite numbers")
        result.append(number)
    return result


def validate_rigid_transform(matrix: list[float], label: str) -> None:
    if len(matrix) != 16:
        raise TwinConfigError(f"{label} must contain 16 row-major values")
    if any(abs(matrix[12 + col] - (1.0 if col == 3 else 0.0)) > 1.0e-8 for col in range(4)):
        raise TwinConfigError(f"{label} last row must be [0, 0, 0, 1]")
    rotation = [[matrix[row * 4 + col] for col in range(3)] for row in range(3)]
    for row in range(3):
        for col in range(3):
            dot = sum(rotation[k][row] * rotation[k][col] for k in range(3))
            expected = 1.0 if row == col else 0.0
            if abs(dot - expected) > 1.0e-5:
                raise TwinConfigError(f"{label} rotation is not orthonormal")
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1.0e-5:
        raise TwinConfigError(f"{label} rotation determinant is {determinant}, expected +1")


def quaternion_xyzw_from_rotation(matrix: list[float]) -> tuple[float, float, float, float]:
    """Convert the upper 3x3 of a row-major rigid matrix to a unit quaternion."""
    r00, r01, r02 = matrix[0], matrix[1], matrix[2]
    r10, r11, r12 = matrix[4], matrix[5], matrix[6]
    r20, r21, r22 = matrix[8], matrix[9], matrix[10]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = ((r21 - r12) / scale, (r02 - r20) / scale, (r10 - r01) / scale, 0.25 * scale)
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        quaternion = (0.25 * scale, (r01 + r10) / scale, (r02 + r20) / scale, (r21 - r12) / scale)
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        quaternion = ((r01 + r10) / scale, 0.25 * scale, (r12 + r21) / scale, (r02 - r20) / scale)
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        quaternion = ((r02 + r20) / scale, (r12 + r21) / scale, 0.25 * scale, (r10 - r01) / scale)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise TwinConfigError("rotation produced a zero quaternion")
    normalized = tuple(value / norm for value in quaternion)
    if normalized[3] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def laser_plane_corners(
    coefficients_m: list[float], z_min_m: float, z_max_m: float, half_width_m: float
) -> list[tuple[float, float, float]]:
    """Return a camera-optical-frame quadrilateral lying on the calibrated plane."""
    nx, ny, nz, d = coefficients_m
    if not all(math.isfinite(value) for value in (*coefficients_m, z_min_m, z_max_m, half_width_m)):
        raise TwinConfigError("invalid laser plane visualization bounds")
    if abs(ny) < 1.0e-8:
        raise TwinConfigError("laser plane visualization requires a non-zero ny coefficient")
    if not 0.0 < z_min_m < z_max_m or half_width_m <= 0.0:
        raise TwinConfigError("invalid laser plane visualization bounds")
    corners = []
    for x, z in ((-half_width_m, z_min_m), (half_width_m, z_min_m), (half_width_m, z_max_m), (-half_width_m, z_max_m)):
        y = -(nx * x + nz * z + d) / ny
        corners.append((x, y, z))
    return corners


def camera_usd_parameters(
    intrinsic: list[float],
    width: int,
    height: int,
    focal_length_mm: float,
    stage_meters_per_unit: float = 1.0,
) -> dict[str, float]:
    """Return raw UsdGeom.Camera values.

    USD focal/aperture values are measured in tenths of a stage unit. With a
    meter stage, a physical 6 mm focal length must therefore be authored as
    0.06, not 6.0.
    """
    fx, fy, cx, cy = intrinsic[0], intrinsic[4], intrinsic[2], intrinsic[5]
    if min(fx, fy, focal_length_mm, stage_meters_per_unit) <= 0.0 or min(width, height) <= 0:
        raise TwinConfigError("camera dimensions and focal lengths must be positive")
    mm_to_tenths_of_stage_unit = 0.001 / (0.1 * stage_meters_per_unit)
    return {
        "focal_length": focal_length_mm * mm_to_tenths_of_stage_unit,
        "horizontal_aperture": focal_length_mm * width / fx * mm_to_tenths_of_stage_unit,
        "vertical_aperture": focal_length_mm * height / fy * mm_to_tenths_of_stage_unit,
        "horizontal_aperture_offset": focal_length_mm * (cx - width / 2.0) / fx * mm_to_tenths_of_stage_unit,
        "vertical_aperture_offset": focal_length_mm * (cy - height / 2.0) / fy * mm_to_tenths_of_stage_unit,
    }


def _add_transform(xform, matrix: list[float], translation_scale: float) -> None:
    from pxr import Gf, UsdGeom

    quaternion = quaternion_xyzw_from_rotation(matrix)
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(matrix[3] * translation_scale, matrix[7] * translation_scale, matrix[11] * translation_scale)
    )
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(quaternion[3], Gf.Vec3d(quaternion[0], quaternion[1], quaternion[2]))
    )


def _author_opencv_pinhole_api(camera_prim, intrinsic: list[float], distortion: list[float], width: int, height: int) -> None:
    """Author Isaac Sim 6's OpenCV pinhole API without requiring Kit startup.

    The Omni schema plugin is registered when Isaac Sim opens the stage, but
    it is not registered in the lightweight pxr Python used by this builder.
    Authoring the API token and its non-custom properties directly is the
    supported USD equivalent of applying the schema inside Kit.
    """
    from pxr import Gf, Sdf

    api_schemas = Sdf.TokenListOp()
    api_schemas.prependedItems = [OPENCV_PINHOLE_API]
    camera_prim.SetMetadata("apiSchemas", api_schemas)

    prefix = "omni:lensdistortion:opencvPinhole:"

    def set_attr(name: str, value_type, value: Any) -> None:
        camera_prim.CreateAttribute(name, value_type, custom=False).Set(value)

    set_attr("omni:lensdistortion:model", Sdf.ValueTypeNames.Token, "opencvPinhole")
    set_attr(f"{prefix}imageSize", Sdf.ValueTypeNames.Int2, Gf.Vec2i(width, height))
    for name, value in (
        ("fx", intrinsic[0]),
        ("fy", intrinsic[4]),
        ("cx", intrinsic[2]),
        ("cy", intrinsic[5]),
        ("k1", distortion[0]),
        ("k2", distortion[1]),
        ("p1", distortion[2]),
        ("p2", distortion[3]),
        ("k3", distortion[4]),
        ("k4", 0.0),
        ("k5", 0.0),
        ("k6", 0.0),
        ("s1", 0.0),
        ("s2", 0.0),
        ("s3", 0.0),
        ("s4", 0.0),
    ):
        set_attr(f"{prefix}{name}", Sdf.ValueTypeNames.Float, float(value))


def _verify_composed_stage(
    stage,
    robot_usd_sha256: str,
    robot_asset_sha256: str,
    robot_source_sha256: str,
    input_hashes: dict[str, str],
) -> None:
    """Reject an incomplete stage before it replaces the last valid output."""
    from pxr import UsdGeom, UsdPhysics, UsdRender

    composition_errors = stage.GetCompositionErrors()
    if composition_errors:
        details = "; ".join(str(error) for error in composition_errors[:5])
        raise TwinConfigError(
            f"generated stage has {len(composition_errors)} composition error(s): {details}"
        )
    default_prim = stage.GetDefaultPrim()
    if not default_prim or default_prim.GetPath().pathString != "/World":
        raise TwinConfigError("generated stage must have /World as its default prim")
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0, abs_tol=1.0e-12):
        raise TwinConfigError("generated stage must use metersPerUnit=1")
    wrist3 = stage.GetPrimAtPath(WRIST3_PATH)
    if not wrist3.IsValid() or not wrist3.HasAPI(UsdPhysics.RigidBodyAPI):
        raise TwinConfigError(f"generated stage lacks a rigid terminal link at {WRIST3_PATH}")
    revolute_joint_count = sum(prim.GetTypeName() == "PhysicsRevoluteJoint" for prim in stage.Traverse())
    if revolute_joint_count != 6:
        raise TwinConfigError(f"generated FR5 must contain 6 revolute joints, got {revolute_joint_count}")
    if not stage.GetPrimAtPath("/World/PhysicsScene").IsA(UsdPhysics.Scene):
        raise TwinConfigError("generated standalone stage lacks /World/PhysicsScene")
    camera_prim = stage.GetPrimAtPath(CAMERA_PATH)
    if not camera_prim.IsA(UsdGeom.Camera):
        raise TwinConfigError(f"generated stage lacks camera at {CAMERA_PATH}")
    api_metadata = camera_prim.GetMetadata("apiSchemas")
    authored_apis = api_metadata.GetAddedOrExplicitItems() if api_metadata is not None else []
    if OPENCV_PINHOLE_API not in authored_apis:
        raise TwinConfigError(f"camera lacks {OPENCV_PINHOLE_API}")
    render_product = UsdRender.Product(stage.GetPrimAtPath(RENDER_PRODUCT_PATH))
    if not render_product:
        raise TwinConfigError(f"generated stage lacks render product at {RENDER_PRODUCT_PATH}")
    if render_product.GetCameraRel().GetTargets() != [camera_prim.GetPath()]:
        raise TwinConfigError("render product does not target the calibrated Hik camera")
    render_settings = UsdRender.Settings(stage.GetPrimAtPath(RENDER_SETTINGS_PATH))
    if not render_settings or render_settings.GetProductsRel().GetTargets() != [
        render_product.GetPath()
    ]:
        raise TwinConfigError("stage render settings do not target the Hik render product")
    laser_prim = stage.GetPrimAtPath(LASER_PLANE_PATH)
    if not laser_prim.IsA(UsdGeom.Mesh):
        raise TwinConfigError(f"generated stage lacks laser plane at {LASER_PLANE_PATH}")
    if UsdGeom.Imageable(laser_prim).ComputeVisibility() != UsdGeom.Tokens.invisible:
        raise TwinConfigError("laser debug plane must default to invisible")
    world_prim = stage.GetPrimAtPath("/World")
    if world_prim.GetAttribute("digital_twin:robot_usd_sha256").Get() != robot_usd_sha256:
        raise TwinConfigError("robot USD hash was not preserved in the generated stage")
    if world_prim.GetAttribute("digital_twin:robot_asset_sha256").Get() != robot_asset_sha256:
        raise TwinConfigError("robot asset-tree hash was not preserved in the generated stage")
    if world_prim.GetAttribute("digital_twin:robot_source_sha256").Get() != robot_source_sha256:
        raise TwinConfigError("robot source hash was not preserved in the generated stage")
    for name, expected in input_hashes.items():
        actual = world_prim.GetAttribute(f"digital_twin:{name}_sha256").Get()
        if actual != expected:
            raise TwinConfigError(f"generated stage {name} hash mismatch")


def _build_stage_to_path(
    args: argparse.Namespace, temporary_output: Path, output_usd: Path
) -> dict[str, Any]:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdRender, Vt

    robot_usd = args.robot_usd.resolve()
    if not robot_usd.is_file():
        raise TwinConfigError(f"robot USD does not exist: {robot_usd}; run build.sh first")

    intrinsics_cfg = load_yaml(args.intrinsics)
    handeye_cfg = load_yaml(args.handeye)
    laser_cfg = load_yaml(args.laser)
    twin_cfg = load_yaml(args.twin_config)

    for label, cfg in (("intrinsics", intrinsics_cfg), ("hand-eye", handeye_cfg), ("laser", laser_cfg), ("twin", twin_cfg)):
        require_equal(cfg.get("schema_version"), 1, f"{label} schema_version")
    require_equal(intrinsics_cfg.get("calibration_type"), "camera_intrinsics", "intrinsics calibration_type")
    require_equal(handeye_cfg.get("calibration_type"), "eye_in_hand", "hand-eye calibration_type")
    require_equal(laser_cfg.get("calibration_type"), "line_laser_plane", "laser calibration_type")

    input_hashes = {
        "intrinsics": sha256_file(args.intrinsics),
        "handeye": sha256_file(args.handeye),
        "laser": sha256_file(args.laser),
        "twin_config": sha256_file(args.twin_config),
    }
    intrinsics_sha256 = input_hashes["intrinsics"]
    robot_usd_sha256 = sha256_file(robot_usd)
    robot_asset_sha256 = sha256_tree(robot_usd.parent)
    robot_source_sha256 = str(args.robot_source_hash).lower()
    if len(robot_source_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in robot_source_sha256
    ):
        raise TwinConfigError("--robot-source-hash must be a 64-character SHA256 digest")
    handeye_sources = required_mapping(handeye_cfg.get("sources"), "hand-eye sources")
    laser_intrinsics = required_mapping(laser_cfg.get("intrinsics"), "laser intrinsics")
    require_equal(
        handeye_sources.get("intrinsics_sha256"),
        intrinsics_sha256,
        "hand-eye intrinsics_sha256",
    )
    require_equal(
        laser_intrinsics.get("sha256"),
        intrinsics_sha256,
        "laser-plane intrinsics sha256",
    )
    require_equal(handeye_cfg.get("translation_unit"), "mm", "hand-eye translation_unit")
    require_equal(handeye_cfg.get("rotation_unit"), "dimensionless", "hand-eye rotation_unit")
    require_equal(
        handeye_cfg.get("transform_convention"),
        TRANSFORM_CONVENTION,
        "hand-eye transform_convention",
    )
    require_equal(laser_cfg.get("length_unit"), "mm", "laser length_unit")
    require_equal(
        laser_cfg.get("coordinate_convention"),
        "x_right_y_down_z_forward",
        "laser coordinate_convention",
    )
    require_equal(
        handeye_cfg.get("child_frame"),
        laser_cfg.get("camera_frame"),
        "hand-eye child_frame versus laser camera_frame",
    )
    intrinsics_metadata = required_mapping(intrinsics_cfg.get("metadata"), "intrinsics metadata")
    require_equal(handeye_cfg.get("child_frame"), intrinsics_metadata.get("frame_id"), "hand-eye child_frame versus intrinsics frame_id")
    require_equal(
        handeye_cfg.get("parent_frame"),
        "fairino_flange_reported",
        "hand-eye parent_frame",
    )

    require_equal(intrinsics_cfg.get("distortion_model"), "plumb_bob", "intrinsics distortion_model")
    handeye_camera = required_mapping(handeye_cfg.get("camera"), "hand-eye camera")
    require_equal(handeye_camera.get("model"), intrinsics_metadata.get("camera_model"), "hand-eye camera model")
    require_equal(handeye_camera.get("serial"), intrinsics_metadata.get("camera_serial"), "hand-eye camera serial")

    width = positive_int(intrinsics_cfg.get("image_width"), "image_width")
    height = positive_int(intrinsics_cfg.get("image_height"), "image_height")
    camera_matrix_cfg = required_mapping(intrinsics_cfg.get("camera_matrix"), "camera_matrix")
    distortion_cfg = required_mapping(
        intrinsics_cfg.get("distortion_coefficients"), "distortion_coefficients"
    )
    intrinsic = finite_float_list(camera_matrix_cfg.get("data"), 9, "camera_matrix.data")
    distortion = finite_float_list(
        distortion_cfg.get("data"), 5, "distortion_coefficients.data"
    )
    require_equal(camera_matrix_cfg.get("rows"), 3, "camera_matrix.rows")
    require_equal(camera_matrix_cfg.get("cols"), 3, "camera_matrix.cols")
    require_equal(distortion_cfg.get("rows"), 1, "distortion_coefficients.rows")
    require_equal(distortion_cfg.get("cols"), 5, "distortion_coefficients.cols")
    if any(abs(intrinsic[index] - expected) > 1.0e-12 for index, expected in ((1, 0.0), (3, 0.0), (6, 0.0), (7, 0.0), (8, 1.0))):
        raise TwinConfigError("camera_matrix.data must have the standard pinhole form")

    if handeye_cfg.get("mode") != "camera_to_flange":
        raise TwinConfigError("hik_handeye.yaml must use mode: camera_to_flange")
    handeye = finite_float_list(handeye_cfg.get("T_flange_camera"), 16, "T_flange_camera")
    validate_rigid_transform(handeye, "T_flange_camera")

    laser_plane_cfg = required_mapping(laser_cfg.get("plane"), "plane")
    plane_mm = finite_float_list(laser_plane_cfg.get("coefficients"), 4, "plane.coefficients")
    plane_m = plane_mm[:3] + [plane_mm[3] / 1000.0]
    normal_norm = math.sqrt(sum(value * value for value in plane_m[:3]))
    if abs(normal_norm - 1.0) > 1.0e-5:
        raise TwinConfigError(f"laser plane normal must be normalized, got {normal_norm}")
    validity = required_mapping(laser_cfg.get("validity"), "validity")
    z_bounds_mm = finite_float_list(
        [validity.get("camera_z_min_mm"), validity.get("camera_z_max_mm")],
        2,
        "laser validity camera Z",
    )
    z_min_m, z_max_m = (value / 1000.0 for value in z_bounds_mm)

    proxy_cfg = required_mapping(
        twin_cfg.get("wrist3_to_reported_flange"), "wrist3_to_reported_flange"
    )
    require_equal(
        proxy_cfg.get("translation_unit"),
        "m",
        "wrist3_to_reported_flange.translation_unit",
    )
    require_equal(proxy_cfg.get("transform_convention"), TRANSFORM_CONVENTION, "wrist3_to_reported_flange.transform_convention")
    require_equal(proxy_cfg.get("parent_frame"), "wrist3_link", "wrist3_to_reported_flange.parent_frame")
    require_equal(proxy_cfg.get("child_frame"), handeye_cfg.get("parent_frame"), "wrist3_to_reported_flange.child_frame")
    proxy = finite_float_list(proxy_cfg.get("matrix_row_major"), 16, "wrist3_to_reported_flange.matrix_row_major")
    validate_rigid_transform(proxy, "wrist3_to_reported_flange.matrix_row_major")
    if not isinstance(proxy_cfg.get("verified"), bool):
        raise TwinConfigError("wrist3_to_reported_flange.verified must be true or false")
    proxy_verified = proxy_cfg["verified"]

    camera_cfg = required_mapping(twin_cfg.get("camera"), "camera")
    pixel_size_um = finite_float_list([camera_cfg.get("pixel_size_um")], 1, "camera.pixel_size_um")[0]
    if pixel_size_um <= 0.0:
        raise TwinConfigError("camera.pixel_size_um must be a positive finite number")
    focal_length_mm = 0.5 * (intrinsic[0] + intrinsic[4]) * pixel_size_um / 1000.0
    clipping = finite_float_list(camera_cfg.get("clipping_range_m", [0.05, 3.0]), 2, "camera.clipping_range_m")
    if not 0.0 < clipping[0] < clipping[1]:
        raise TwinConfigError("camera clipping_range_m must be positive and increasing")
    camera_params = camera_usd_parameters(intrinsic, width, height, focal_length_mm)

    plane_vis_cfg = required_mapping(
        twin_cfg.get("laser_plane_visualization"), "laser_plane_visualization"
    )
    half_width_m = finite_float_list([plane_vis_cfg.get("half_width_m", 0.30)], 1, "laser_plane_visualization.half_width_m")[0]
    corners = laser_plane_corners(plane_m, z_min_m, z_max_m, half_width_m)

    simulation_cfg = required_mapping(twin_cfg.get("simulation"), "simulation")
    time_codes_per_second = finite_float_list(
        [simulation_cfg.get("time_codes_per_second")], 1, "simulation.time_codes_per_second"
    )[0]
    gravity_m_s2 = finite_float_list([simulation_cfg.get("gravity_m_s2")], 1, "simulation.gravity_m_s2")[0]
    if time_codes_per_second <= 0.0 or gravity_m_s2 < 0.0:
        raise TwinConfigError("simulation time step must be positive and gravity non-negative")

    stage = Usd.Stage.CreateNew(str(temporary_output))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetTimeCodesPerSecond(time_codes_per_second)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    robot = UsdGeom.Xform.Define(stage, "/World/FR5")
    relative_robot_path = os.path.relpath(robot_usd, output_usd.parent)
    robot.GetPrim().GetReferences().AddReference(relative_robot_path)
    if not stage.GetPrimAtPath(WRIST3_PATH).IsValid():
        raise TwinConfigError(
            f"FR5 USD does not contain the expected terminal link prim: {WRIST3_PATH}"
        )

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(gravity_m_s2)

    world_prim = world.GetPrim()
    world_prim.CreateAttribute("digital_twin:metric_ready", Sdf.ValueTypeNames.Bool, custom=True).Set(
        proxy_verified
    )
    world_prim.CreateAttribute("digital_twin:status", Sdf.ValueTypeNames.String, custom=True).Set(
        "metric_ready" if proxy_verified else "visualization_only_unverified_wrist3_to_flange"
    )
    world_prim.CreateAttribute("digital_twin:build_contract_version", Sdf.ValueTypeNames.Int, custom=True).Set(
        BUILD_CONTRACT_VERSION
    )
    world_prim.CreateAttribute("digital_twin:robot_reference", Sdf.ValueTypeNames.String, custom=True).Set(
        relative_robot_path
    )
    world_prim.CreateAttribute("digital_twin:robot_usd_sha256", Sdf.ValueTypeNames.String, custom=True).Set(
        robot_usd_sha256
    )
    world_prim.CreateAttribute("digital_twin:robot_asset_sha256", Sdf.ValueTypeNames.String, custom=True).Set(
        robot_asset_sha256
    )
    world_prim.CreateAttribute("digital_twin:robot_source_sha256", Sdf.ValueTypeNames.String, custom=True).Set(
        robot_source_sha256
    )
    for name, digest in input_hashes.items():
        world_prim.CreateAttribute(f"digital_twin:{name}_sha256", Sdf.ValueTypeNames.String, custom=True).Set(digest)

    flange = UsdGeom.Xform.Define(stage, f"{WRIST3_PATH}/fairino_flange_reported_PROXY")
    _add_transform(flange, proxy, 1.0)
    flange_prim = flange.GetPrim()
    flange_prim.SetDisplayName("fairino_flange_reported (UNVERIFIED wrist3 proxy)" if not proxy_verified else "fairino_flange_reported")
    flange_prim.CreateAttribute("calibration:verified", Sdf.ValueTypeNames.Bool, custom=True).Set(proxy_verified)
    flange_prim.CreateAttribute("calibration:source", Sdf.ValueTypeNames.String, custom=True).Set(str(args.twin_config.resolve()))
    flange_prim.CreateAttribute("calibration:T_wrist3_flange_row_major_m", Sdf.ValueTypeNames.DoubleArray, custom=True).Set(proxy)

    optical_path = f"{WRIST3_PATH}/fairino_flange_reported_PROXY/hik_camera_optical_frame"
    optical = UsdGeom.Xform.Define(stage, optical_path)
    _add_transform(optical, handeye, 0.001)
    optical_prim = optical.GetPrim()
    optical_prim.CreateAttribute("calibration:frame_convention", Sdf.ValueTypeNames.String, custom=True).Set(
        "OpenCV optical: +X right, +Y down, +Z forward"
    )
    optical_prim.CreateAttribute("calibration:T_flange_camera_row_major_m", Sdf.ValueTypeNames.DoubleArray, custom=True).Set(
        [handeye[index] / 1000.0 if index in (3, 7, 11) else handeye[index] for index in range(16)]
    )
    optical_prim.CreateAttribute("calibration:handeye_source", Sdf.ValueTypeNames.String, custom=True).Set(str(args.handeye.resolve()))
    optical_prim.CreateAttribute("calibration:handeye_sha256", Sdf.ValueTypeNames.String, custom=True).Set(input_hashes["handeye"])

    camera = UsdGeom.Camera.Define(stage, f"{optical_path}/HikCamera")
    # USD cameras look along -Z with +Y up. A 180-degree X rotation maps that
    # convention onto the OpenCV optical frame (+Z forward, +Y down).
    camera.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(0.0, Gf.Vec3d(1.0, 0.0, 0.0)))
    camera.GetFocalLengthAttr().Set(camera_params["focal_length"])
    camera.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.GetHorizontalApertureAttr().Set(camera_params["horizontal_aperture"])
    camera.GetVerticalApertureAttr().Set(camera_params["vertical_aperture"])
    # The OpenCV API below is the single source of truth for cx/cy. Keep the
    # base UsdGeom.Camera centered so the principal point is not applied twice.
    camera.GetHorizontalApertureOffsetAttr().Set(0.0)
    camera.GetVerticalApertureOffsetAttr().Set(0.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(clipping[0], clipping[1]))
    camera.GetFStopAttr().Set(0.0)
    camera_prim = camera.GetPrim()
    _author_opencv_pinhole_api(camera_prim, intrinsic, distortion, width, height)
    camera_prim.CreateAttribute("calibration:image_width", Sdf.ValueTypeNames.Int, custom=True).Set(width)
    camera_prim.CreateAttribute("calibration:image_height", Sdf.ValueTypeNames.Int, custom=True).Set(height)
    camera_prim.CreateAttribute("calibration:pixel_size_um", Sdf.ValueTypeNames.Double, custom=True).Set(
        pixel_size_um
    )
    camera_prim.CreateAttribute("calibration:K_row_major", Sdf.ValueTypeNames.DoubleArray, custom=True).Set(intrinsic)
    camera_prim.CreateAttribute("calibration:distortion_model", Sdf.ValueTypeNames.String, custom=True).Set(
        str(intrinsics_cfg.get("distortion_model", "plumb_bob"))
    )
    camera_prim.CreateAttribute("calibration:D", Sdf.ValueTypeNames.DoubleArray, custom=True).Set(distortion)
    camera_prim.CreateAttribute("calibration:intrinsics_source", Sdf.ValueTypeNames.String, custom=True).Set(
        str(args.intrinsics.resolve())
    )
    camera_prim.CreateAttribute("calibration:intrinsics_sha256", Sdf.ValueTypeNames.String, custom=True).Set(
        intrinsics_sha256
    )
    camera_prim.CreateAttribute("calibration:rendering_note", Sdf.ValueTypeNames.String, custom=True).Set(
        "Isaac Sim 6 OpenCV pinhole API carries exact K/D; validate rendered pixels against cv2.projectPoints before metric use."
    )

    UsdGeom.Scope.Define(stage, "/World/Render")
    render_product = UsdRender.Product.Define(stage, RENDER_PRODUCT_PATH)
    render_product.CreateCameraRel().SetTargets([camera_prim.GetPath()])
    render_product.CreateResolutionAttr().Set(Gf.Vec2i(width, height))
    render_settings = UsdRender.Settings.Define(stage, RENDER_SETTINGS_PATH)
    render_settings.CreateProductsRel().SetTargets([render_product.GetPath()])
    stage.SetMetadata(UsdRender.Tokens.renderSettingsPrimPath, str(render_settings.GetPath()))

    plane_mesh = UsdGeom.Mesh.Define(stage, f"{optical_path}/LaserPlaneVisualization")
    plane_mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*point) for point in corners]))
    plane_mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    plane_mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    plane_mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    plane_mesh.CreateDoubleSidedAttr().Set(True)
    plane_mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.35, 1.0)]))
    plane_mesh.CreateDisplayOpacityAttr(Vt.FloatArray([0.20]))
    plane_mesh.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    plane_mesh.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
    plane_prim = plane_mesh.GetPrim()
    plane_prim.CreateAttribute("laser:representation", Sdf.ValueTypeNames.String, custom=True).Set("analytic_plane_only")
    plane_prim.CreateAttribute("laser:coefficients_camera_m", Sdf.ValueTypeNames.DoubleArray, custom=True).Set(plane_m)
    plane_prim.CreateAttribute("laser:valid_camera_z_m", Sdf.ValueTypeNames.Double2, custom=True).Set(
        Gf.Vec2d(z_min_m, z_max_m)
    )
    plane_prim.CreateAttribute("laser:source", Sdf.ValueTypeNames.String, custom=True).Set(str(args.laser.resolve()))
    plane_prim.CreateAttribute("laser:source_sha256", Sdf.ValueTypeNames.String, custom=True).Set(
        input_hashes["laser"]
    )
    plane_prim.CreateAttribute("laser:warning", Sdf.ValueTypeNames.String, custom=True).Set(
        "A calibrated plane does not uniquely determine the physical projector pose; no RectLight is authored."
    )

    stage.GetRootLayer().customLayerData = {
        "buildContractVersion": BUILD_CONTRACT_VERSION,
        "creator": "fr5_line_laser_twin/build_twin.py",
        "flangeProxyVerified": proxy_verified,
        "handeyeSha256": input_hashes["handeye"],
        "intrinsicsSha256": intrinsics_sha256,
        "laserSha256": input_hashes["laser"],
        "robotAssetSha256": robot_asset_sha256,
        "robotSourceSha256": robot_source_sha256,
        "robotUsdSha256": robot_usd_sha256,
        "twinConfigSha256": input_hashes["twin_config"],
        "warning": "Do not use for metric validation until wrist3_link to fairino_flange_reported is measured."
        if not proxy_verified
        else "",
    }
    _verify_composed_stage(
        stage,
        robot_usd_sha256,
        robot_asset_sha256,
        robot_source_sha256,
        input_hashes,
    )
    stage.GetRootLayer().Save()
    reopened_stage = Usd.Stage.Open(str(temporary_output))
    if reopened_stage is None:
        raise TwinConfigError(f"cannot reopen generated stage: {temporary_output}")
    _verify_composed_stage(
        reopened_stage,
        robot_usd_sha256,
        robot_asset_sha256,
        robot_source_sha256,
        input_hashes,
    )
    del reopened_stage
    del stage
    return {
        "robot_usd": robot_usd,
        "width": width,
        "height": height,
        "intrinsic": intrinsic,
        "pixel_size_um": pixel_size_um,
        "focal_length_mm": focal_length_mm,
        "camera_params": camera_params,
        "plane_m": plane_m,
        "proxy_verified": proxy_verified,
    }


def build_stage(args: argparse.Namespace) -> None:
    output_usd = args.output.resolve()
    output_usd.parent.mkdir(parents=True, exist_ok=True)
    temporary_output = output_usd.with_name(
        f".{output_usd.stem}.tmp-{os.getpid()}-{uuid.uuid4().hex}{output_usd.suffix}"
    )
    try:
        summary = _build_stage_to_path(args, temporary_output, output_usd)
        os.replace(temporary_output, output_usd)
    finally:
        temporary_output.unlink(missing_ok=True)

    print(f"Generated twin USD: {output_usd}")
    print(f"FR5 source USD: {summary['robot_usd']}")
    print(
        f"Camera: {summary['width']}x{summary['height']}, "
        f"fx={summary['intrinsic'][0]:.6f}, fy={summary['intrinsic'][4]:.6f}, "
        f"pixel={summary['pixel_size_um']:.4f} um, "
        f"physical focal={summary['focal_length_mm']:.6f} mm, "
        f"USD raw focal={summary['camera_params']['focal_length']:.9f}"
    )
    print(f"Laser plane [m]: {summary['plane_m']}")
    print(f"wrist3 -> reported flange verified: {summary['proxy_verified']}")
    if not summary["proxy_verified"]:
        print("WARNING: unverified wrist3-to-flange candidate is for visualization only.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-usd", type=Path, required=True)
    parser.add_argument("--robot-source-hash", required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_USD)
    parser.add_argument("--intrinsics", type=Path, default=DEFAULT_INTRINSICS)
    parser.add_argument("--handeye", type=Path, default=DEFAULT_HANDEYE)
    parser.add_argument("--laser", type=Path, default=DEFAULT_LASER)
    parser.add_argument("--twin-config", type=Path, default=DEFAULT_TWIN_CONFIG)
    return parser.parse_args()


if __name__ == "__main__":
    build_stage(parse_args())
