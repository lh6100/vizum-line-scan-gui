#!/usr/bin/env python3
"""Validate the generated FR5/Hik/line-laser USD and its source provenance."""

from __future__ import annotations

import argparse
import importlib.metadata
import importlib.util
import math
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).with_name("build_twin.py")
SPEC = importlib.util.spec_from_file_location("fr5_line_laser_build", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {MODULE_PATH}")
BUILD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILD)

ROBOT_WRIST3_PATH = (
    "/fairino5_v6_robot/Geometry/base_link/shoulder_link/upperarm_link/forearm_link/"
    "wrist1_link/wrist2_link/wrist3_link"
)


def close(actual: float, expected: float, label: str, tolerance: float = 1.0e-5) -> None:
    if not math.isclose(float(actual), float(expected), rel_tol=1.0e-7, abs_tol=tolerance):
        raise BUILD.TwinConfigError(f"{label}: expected {expected}, got {actual}")


def validate_robot_usd(path: Path) -> None:
    from pxr import Usd, UsdGeom, UsdPhysics

    resolved = path.resolve()
    if not resolved.is_file():
        raise BUILD.TwinConfigError(f"robot USD does not exist: {resolved}")
    stage = Usd.Stage.Open(str(resolved))
    if stage is None:
        raise BUILD.TwinConfigError(f"cannot open robot USD: {resolved}")
    composition_errors = stage.GetCompositionErrors()
    if composition_errors:
        details = "; ".join(str(error) for error in composition_errors[:5])
        raise BUILD.TwinConfigError(
            f"FR5 USD has {len(composition_errors)} composition error(s): {details}"
        )
    default_prim = stage.GetDefaultPrim()
    if not default_prim or default_prim.GetPath().pathString != "/fairino5_v6_robot":
        raise BUILD.TwinConfigError("FR5 USD default prim must be /fairino5_v6_robot")
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0, abs_tol=1.0e-12):
        raise BUILD.TwinConfigError("FR5 USD must use metersPerUnit=1")
    wrist3 = stage.GetPrimAtPath(ROBOT_WRIST3_PATH)
    if not wrist3.IsValid() or not wrist3.HasAPI(UsdPhysics.RigidBodyAPI):
        raise BUILD.TwinConfigError(f"FR5 USD lacks a rigid terminal link at {ROBOT_WRIST3_PATH}")
    revolute_joints = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsRevoluteJoint"]
    if len(revolute_joints) != 6:
        raise BUILD.TwinConfigError(f"FR5 USD must have 6 revolute joints, got {len(revolute_joints)}")
    articulation_roots = [
        prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        raise BUILD.TwinConfigError(
            f"FR5 USD must have one articulation root, got {len(articulation_roots)}"
        )
    for layer in stage.GetUsedLayers():
        if layer.anonymous:
            continue
        if layer.realPath and not Path(layer.realPath).is_file():
            raise BUILD.TwinConfigError(f"FR5 USD dependency is missing: {layer.realPath}")


def expected_transform_point(
    matrix: list[float], point: tuple[float, float, float], scale: float
) -> tuple[float, float, float]:
    return tuple(
        sum(matrix[row * 4 + col] * point[col] for col in range(3))
        + matrix[row * 4 + 3] * scale
        for row in range(3)
    )


def validate_xform(stage, path: str, matrix: list[float], translation_scale: float, label: str) -> None:
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xformable(stage.GetPrimAtPath(path))
    if not xform:
        raise BUILD.TwinConfigError(f"missing {label} transform at {path}")
    local_matrix = xform.GetLocalTransformation()
    for point in ((0.0, 0.0, 0.0), (0.13, -0.07, 0.22), (-0.11, 0.19, 0.03)):
        actual = local_matrix.Transform(Gf.Vec3d(*point))
        expected = expected_transform_point(matrix, point, translation_scale)
        for axis in range(3):
            close(actual[axis], expected[axis], f"{label} point transform axis {axis}", 1.0e-9)


def validate_twin(args: argparse.Namespace) -> None:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdRender

    output = args.output.resolve()
    if not output.is_file():
        raise BUILD.TwinConfigError(f"twin USD does not exist: {output}")
    stage = Usd.Stage.Open(str(output))
    if stage is None:
        raise BUILD.TwinConfigError(f"cannot open twin USD: {output}")

    input_paths = {
        "intrinsics": args.intrinsics,
        "handeye": args.handeye,
        "laser": args.laser,
        "twin_config": args.twin_config,
    }
    input_hashes = {name: BUILD.sha256_file(path) for name, path in input_paths.items()}
    world = stage.GetPrimAtPath("/World")
    robot_reference = world.GetAttribute("digital_twin:robot_reference").Get()
    if not isinstance(robot_reference, str) or not robot_reference:
        raise BUILD.TwinConfigError("twin USD does not record its robot reference")
    robot_usd = (output.parent / robot_reference).resolve()
    if args.robot_usd is not None and robot_usd != args.robot_usd.resolve():
        raise BUILD.TwinConfigError(
            f"twin references {robot_usd}, but build selected {args.robot_usd.resolve()}"
        )
    validate_robot_usd(robot_usd)
    robot_digest = BUILD.sha256_file(robot_usd)
    robot_asset_digest = BUILD.sha256_tree(robot_usd.parent)
    current_source_digest = BUILD.robot_source_fingerprint(
        BUILD.REPO_ROOT,
        args.isaaclab_root,
        importlib.metadata.version("isaacsim"),
    )
    BUILD._verify_composed_stage(
        stage,
        robot_digest,
        robot_asset_digest,
        current_source_digest,
        input_hashes,
    )

    if world.GetAttribute("digital_twin:build_contract_version").Get() != BUILD.BUILD_CONTRACT_VERSION:
        raise BUILD.TwinConfigError("twin build contract version is stale")

    intrinsics = BUILD.load_yaml(args.intrinsics)
    handeye_cfg = BUILD.load_yaml(args.handeye)
    laser_cfg = BUILD.load_yaml(args.laser)
    twin_cfg = BUILD.load_yaml(args.twin_config)
    width = BUILD.positive_int(intrinsics.get("image_width"), "image_width")
    height = BUILD.positive_int(intrinsics.get("image_height"), "image_height")
    intrinsic = BUILD.finite_float_list(intrinsics["camera_matrix"]["data"], 9, "K")
    distortion = BUILD.finite_float_list(
        intrinsics["distortion_coefficients"]["data"], 5, "D"
    )

    camera = UsdGeom.Camera(stage.GetPrimAtPath(BUILD.CAMERA_PATH))
    camera_prim = camera.GetPrim()
    prefix = "omni:lensdistortion:opencvPinhole:"
    if tuple(camera_prim.GetAttribute(f"{prefix}imageSize").Get()) != (width, height):
        raise BUILD.TwinConfigError("OpenCV camera imageSize does not match intrinsics")
    for name, expected in (
        ("fx", intrinsic[0]),
        ("fy", intrinsic[4]),
        ("cx", intrinsic[2]),
        ("cy", intrinsic[5]),
        ("k1", distortion[0]),
        ("k2", distortion[1]),
        ("p1", distortion[2]),
        ("p2", distortion[3]),
        ("k3", distortion[4]),
    ):
        close(camera_prim.GetAttribute(f"{prefix}{name}").Get(), expected, f"camera {name}", 2.0e-4)
    for name in ("k4", "k5", "k6", "s1", "s2", "s3", "s4"):
        close(camera_prim.GetAttribute(f"{prefix}{name}").Get(), 0.0, f"camera {name}")
    if camera_prim.GetAttribute("omni:lensdistortion:model").Get() != "opencvPinhole":
        raise BUILD.TwinConfigError("camera lens-distortion model is not opencvPinhole")
    render_product = UsdRender.Product(stage.GetPrimAtPath(BUILD.RENDER_PRODUCT_PATH))
    if tuple(render_product.GetResolutionAttr().Get()) != (width, height):
        raise BUILD.TwinConfigError("Hik render product resolution does not match intrinsics")

    pixel_size_um = BUILD.finite_float_list(
        [twin_cfg["camera"]["pixel_size_um"]], 1, "camera.pixel_size_um"
    )[0]
    focal_mm = 0.5 * (intrinsic[0] + intrinsic[4]) * pixel_size_um / 1000.0
    camera_params = BUILD.camera_usd_parameters(intrinsic, width, height, focal_mm)
    close(camera.GetFocalLengthAttr().Get(), camera_params["focal_length"], "USD raw focal length")
    close(
        camera.GetHorizontalApertureAttr().Get(),
        camera_params["horizontal_aperture"],
        "USD horizontal aperture",
    )
    close(
        camera.GetVerticalApertureAttr().Get(),
        camera_params["vertical_aperture"],
        "USD vertical aperture",
    )
    close(camera.GetHorizontalApertureOffsetAttr().Get(), 0.0, "USD horizontal aperture offset")
    close(camera.GetVerticalApertureOffsetAttr().Get(), 0.0, "USD vertical aperture offset")

    proxy_cfg = twin_cfg["wrist3_to_reported_flange"]
    proxy = BUILD.finite_float_list(proxy_cfg["matrix_row_major"], 16, "proxy")
    flange_path = f"{BUILD.WRIST3_PATH}/fairino_flange_reported_PROXY"
    validate_xform(stage, flange_path, proxy, 1.0, "T_wrist3_flange")
    handeye = BUILD.finite_float_list(handeye_cfg["T_flange_camera"], 16, "handeye")
    optical_path = f"{flange_path}/hik_camera_optical_frame"
    validate_xform(stage, optical_path, handeye, 0.001, "T_flange_camera")

    camera_local = UsdGeom.Xformable(camera_prim).GetLocalTransformation()
    optical_forward = camera_local.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    optical_down = camera_local.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
    for actual, expected, label in zip(optical_forward, (0.0, 0.0, 1.0), "xyz"):
        close(actual, expected, f"camera forward {label}", 1.0e-12)
    for actual, expected, label in zip(optical_down, (0.0, -1.0, 0.0), "xyz"):
        close(actual, expected, f"camera down {label}", 1.0e-12)

    plane_prim = stage.GetPrimAtPath(BUILD.LASER_PLANE_PATH)
    plane = UsdGeom.Mesh(plane_prim)
    if plane.GetPurposeAttr().Get() != UsdGeom.Tokens.guide:
        raise BUILD.TwinConfigError("laser debug plane purpose must be guide")
    if plane.ComputeVisibility() != UsdGeom.Tokens.invisible:
        raise BUILD.TwinConfigError("laser debug plane must be invisible by default")
    coefficients_mm = BUILD.finite_float_list(laser_cfg["plane"]["coefficients"], 4, "plane")
    coefficients_m = coefficients_mm[:3] + [coefficients_mm[3] / 1000.0]
    for point in plane.GetPointsAttr().Get():
        residual = sum(coefficients_m[index] * point[index] for index in range(3)) + coefficients_m[3]
        if abs(residual) > 5.0e-8:
            raise BUILD.TwinConfigError(
                f"laser debug point is off the calibrated plane by {residual} m"
            )

    scene = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
    simulation_cfg = twin_cfg["simulation"]
    close(
        stage.GetTimeCodesPerSecond(),
        simulation_cfg["time_codes_per_second"],
        "timeCodesPerSecond",
    )
    close(scene.GetGravityMagnitudeAttr().Get(), simulation_cfg["gravity_m_s2"], "gravity")
    if tuple(scene.GetGravityDirectionAttr().Get()) != (0.0, 0.0, -1.0):
        raise BUILD.TwinConfigError("gravity direction must be -Z")

    proxy_verified = proxy_cfg["verified"]
    if world.GetAttribute("digital_twin:metric_ready").Get() != proxy_verified:
        raise BUILD.TwinConfigError("metric_ready does not match flange verification state")

    print(f"Validated twin USD: {output}")
    print(f"FR5: 6 revolute joints; source={robot_usd}")
    print(f"Camera: {width}x{height}; OpenCV K/D schema + RenderProduct verified")
    print("Laser: analytic plane verified; debug mesh is guide + invisible")
    print(f"Metric ready: {proxy_verified}")
    if not proxy_verified:
        print("WARNING: wrist3_link -> fairino_flange_reported is still unverified")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-only", type=Path)
    parser.add_argument("--robot-usd", type=Path)
    parser.add_argument("--print-robot-source-hash", action="store_true")
    parser.add_argument("--print-robot-asset-hash", action="store_true")
    parser.add_argument(
        "--isaaclab-root",
        type=Path,
        default=Path("/home/zhulong/IsaacLab-3.0.0-beta2"),
    )
    parser.add_argument("--output", type=Path, default=BUILD.DEFAULT_OUTPUT_USD)
    parser.add_argument("--intrinsics", type=Path, default=BUILD.DEFAULT_INTRINSICS)
    parser.add_argument("--handeye", type=Path, default=BUILD.DEFAULT_HANDEYE)
    parser.add_argument("--laser", type=Path, default=BUILD.DEFAULT_LASER)
    parser.add_argument("--twin-config", type=Path, default=BUILD.DEFAULT_TWIN_CONFIG)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.print_robot_source_hash:
            print(
                BUILD.robot_source_fingerprint(
                    BUILD.REPO_ROOT,
                    args.isaaclab_root,
                    importlib.metadata.version("isaacsim"),
                )
            )
        elif args.print_robot_asset_hash:
            if args.robot_only is None:
                raise BUILD.TwinConfigError(
                    "--print-robot-asset-hash requires --robot-only ROBOT_USD"
                )
            validate_robot_usd(args.robot_only)
            print(BUILD.sha256_tree(args.robot_only.resolve().parent))
        elif args.robot_only is not None:
            validate_robot_usd(args.robot_only)
            print(f"Validated FR5 USD: {args.robot_only.resolve()}")
        else:
            validate_twin(args)
    except (BUILD.TwinConfigError, RuntimeError) as exc:
        print(f"VALIDATION FAILED: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
