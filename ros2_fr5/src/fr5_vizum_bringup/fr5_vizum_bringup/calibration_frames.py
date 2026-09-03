"""Load the repository's rigid transforms and line-laser plane calibrations."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

import yaml


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Matrix4 = Tuple[Tuple[float, float, float, float], ...]
Plane4 = Tuple[float, float, float, float]


class CalibrationConfigError(ValueError):
    """Raised when a calibration file is missing or is not a rigid transform."""


@dataclass(frozen=True)
class RigidTransform:
    translation_m: Vector3
    quaternion_xyzw: Quaternion


@dataclass(frozen=True)
class ToolCalibration:
    tool_id: int
    transform: RigidTransform


@dataclass(frozen=True)
class CameraCalibration:
    input_mode: str
    transform: RigidTransform


@dataclass(frozen=True)
class LaserPlaneCalibration:
    camera_frame: str
    coefficients_mm: Plane4
    vertices_m: Tuple[Vector3, Vector3, Vector3, Vector3]
    z_range_m: Tuple[float, float]


def _load_mapping(path: str) -> Mapping[str, Any]:
    config_path = Path(path).expanduser()
    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise CalibrationConfigError(
            f"cannot read calibration file {config_path}: {exc}"
        ) from exc
    except yaml.YAMLError as exc:
        raise CalibrationConfigError(
            f"invalid YAML in calibration file {config_path}: {exc}"
        ) from exc

    if not isinstance(data, Mapping):
        raise CalibrationConfigError(
            f"calibration file {config_path} must contain a YAML mapping"
        )
    return data


def _required_finite_float(config: Mapping[str, Any], key: str, path: str) -> float:
    if key not in config:
        raise CalibrationConfigError(f"{path}: missing required key '{key}'")
    value = config[key]
    if isinstance(value, bool):
        raise CalibrationConfigError(f"{path}: '{key}' must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationConfigError(
            f"{path}: '{key}' must be a finite number"
        ) from exc
    if not math.isfinite(number):
        raise CalibrationConfigError(f"{path}: '{key}' must be finite")
    return number


def _required_integer(config: Mapping[str, Any], key: str, path: str) -> int:
    number = _required_finite_float(config, key, path)
    integer = int(number)
    if float(integer) != number:
        raise CalibrationConfigError(f"{path}: '{key}' must be an integer")
    return integer


def _required_mapping(
    config: Mapping[str, Any], key: str, path: str
) -> Mapping[str, Any]:
    value = config.get(key)
    if not isinstance(value, Mapping):
        raise CalibrationConfigError(f"{path}: '{key}' must be a YAML mapping")
    return value


def _required_string(config: Mapping[str, Any], key: str, path: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value.strip():
        raise CalibrationConfigError(
            f"{path}: missing required non-empty string key '{key}'"
        )
    return value.strip()


def _finite_sequence(value: Any, size: int, label: str) -> Tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise CalibrationConfigError(f"{label} must contain {size} finite numbers")
    if len(value) != size:
        raise CalibrationConfigError(f"{label} must contain {size} finite numbers")

    numbers = []
    for item in value:
        if isinstance(item, bool):
            raise CalibrationConfigError(f"{label} must contain {size} finite numbers")
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise CalibrationConfigError(
                f"{label} must contain {size} finite numbers"
            ) from exc
        if not math.isfinite(number):
            raise CalibrationConfigError(f"{label} must contain only finite numbers")
        numbers.append(number)
    return tuple(numbers)


def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm < 1.0e-12:
        raise CalibrationConfigError("rotation produced an invalid quaternion")
    normalized = tuple(component / norm for component in quaternion)
    # q and -q describe the same rotation. Keep a stable representation for logs/tests.
    if normalized[3] < 0.0:
        normalized = tuple(-component for component in normalized)
    return normalized  # type: ignore[return-value]


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return _normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def _determinant3(rotation: Tuple[Tuple[float, float, float], ...]) -> float:
    return (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )


def _validate_rigid_matrix(matrix: Matrix4, path: str) -> None:
    expected_last_row = (0.0, 0.0, 0.0, 1.0)
    if any(abs(matrix[3][index] - expected_last_row[index]) > 1.0e-6 for index in range(4)):
        raise CalibrationConfigError(
            f"{path}: last matrix row must be [0, 0, 0, 1]"
        )

    rotation = tuple(tuple(matrix[row][col] for col in range(3)) for row in range(3))
    max_orthogonality_error = 0.0
    for row in range(3):
        for col in range(3):
            dot = sum(rotation[index][row] * rotation[index][col] for index in range(3))
            expected = 1.0 if row == col else 0.0
            max_orthogonality_error = max(max_orthogonality_error, abs(dot - expected))

    determinant = _determinant3(rotation)
    if max_orthogonality_error > 1.0e-5 or abs(determinant - 1.0) > 1.0e-5:
        raise CalibrationConfigError(
            f"{path}: hand-eye rotation is not rigid "
            f"(orthogonality error={max_orthogonality_error:.3g}, det={determinant:.9g})"
        )


def _inverse_rigid(matrix: Matrix4) -> Matrix4:
    rotation_transpose = tuple(
        tuple(matrix[col][row] for col in range(3)) for row in range(3)
    )
    translation = tuple(matrix[row][3] for row in range(3))
    inverse_translation = tuple(
        -sum(rotation_transpose[row][col] * translation[col] for col in range(3))
        for row in range(3)
    )
    return tuple(
        tuple(rotation_transpose[row][col] for col in range(3))
        + (inverse_translation[row],)
        for row in range(3)
    ) + ((0.0, 0.0, 0.0, 1.0),)


def _quaternion_from_matrix(matrix: Matrix4) -> Quaternion:
    r00, r01, r02 = matrix[0][:3]
    r10, r11, r12 = matrix[1][:3]
    r20, r21, r22 = matrix[2][:3]
    trace = r00 + r11 + r22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (r21 - r12) / scale,
            (r02 - r20) / scale,
            (r10 - r01) / scale,
            0.25 * scale,
        )
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        quaternion = (
            0.25 * scale,
            (r01 + r10) / scale,
            (r02 + r20) / scale,
            (r21 - r12) / scale,
        )
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        quaternion = (
            (r01 + r10) / scale,
            0.25 * scale,
            (r12 + r21) / scale,
            (r02 - r20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        quaternion = (
            (r02 + r20) / scale,
            (r12 + r21) / scale,
            0.25 * scale,
            (r10 - r01) / scale,
        )
    return _normalize_quaternion(quaternion)


def load_tool_calibration(path: str) -> ToolCalibration:
    """Load config/tool_config.yaml as T_flange_tcp, converting mm/deg to m/quaternion."""
    config = _load_mapping(path)
    tool_id = _required_integer(config, "tool_id", path)
    translation = tuple(
        _required_finite_float(config, key, path) / 1000.0
        for key in ("x", "y", "z")
    )
    roll, pitch, yaw = (
        math.radians(_required_finite_float(config, key, path))
        for key in ("rx", "ry", "rz")
    )
    return ToolCalibration(
        tool_id=tool_id,
        transform=RigidTransform(
            translation_m=translation,  # type: ignore[arg-type]
            quaternion_xyzw=_quaternion_from_rpy(roll, pitch, yaw),
        ),
    )


def load_camera_calibration(path: str) -> CameraCalibration:
    """Load hand-eye YAML and always return the normalized T_flange_camera."""
    config = _load_mapping(path)
    mode_value = config.get("mode")
    if not isinstance(mode_value, str):
        raise CalibrationConfigError(f"{path}: missing required string key 'mode'")
    mode = mode_value.strip()
    if mode not in ("camera_to_flange", "flange_to_camera"):
        raise CalibrationConfigError(
            f"{path}: mode must be 'camera_to_flange' or 'flange_to_camera'"
        )

    matrix = tuple(
        tuple(
            _required_finite_float(config, f"m{row}{col}", path)
            for col in range(4)
        )
        for row in range(4)
    )
    _validate_rigid_matrix(matrix, path)
    flange_to_camera = _inverse_rigid(matrix) if mode == "flange_to_camera" else matrix

    return CameraCalibration(
        input_mode=mode,
        transform=RigidTransform(
            translation_m=tuple(
                flange_to_camera[row][3] / 1000.0 for row in range(3)
            ),  # type: ignore[arg-type]
            quaternion_xyzw=_quaternion_from_matrix(flange_to_camera),
        ),
    )


def load_laser_plane_calibration(
    plane_path: str, intrinsics_path: str
) -> LaserPlaneCalibration:
    """Load a laser plane and make its calibrated-depth camera-frustum polygon.

    The returned vertices use metres and the camera optical convention
    x-right/y-down/z-forward, ready for an RViz Marker in ``camera_frame``.
    """
    plane_config = _load_mapping(plane_path)
    if plane_config.get("calibration_type") != "line_laser_plane":
        raise CalibrationConfigError(
            f"{plane_path}: calibration_type must be 'line_laser_plane'"
        )
    camera_frame = _required_string(plane_config, "camera_frame", plane_path)
    convention = _required_string(
        plane_config, "coordinate_convention", plane_path
    )
    if convention != "x_right_y_down_z_forward":
        raise CalibrationConfigError(
            f"{plane_path}: unsupported coordinate_convention '{convention}'"
        )
    unit = _required_string(plane_config, "length_unit", plane_path)
    if unit != "mm":
        raise CalibrationConfigError(
            f"{plane_path}: length_unit must be 'mm', got '{unit}'"
        )

    plane = _required_mapping(plane_config, "plane", plane_path)
    coefficients = _finite_sequence(
        plane.get("coefficients"), 4, f"{plane_path}: plane.coefficients"
    )
    nx, ny, nz, d = coefficients
    normal_norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if normal_norm < 1.0e-12:
        raise CalibrationConfigError(f"{plane_path}: plane normal must be nonzero")
    if abs(ny) < normal_norm * 1.0e-8:
        raise CalibrationConfigError(
            f"{plane_path}: plane ny is too small to make a horizontal-view polygon"
        )

    validity = _required_mapping(plane_config, "validity", plane_path)
    z_min_mm = _required_finite_float(
        validity, "camera_z_min_mm", f"{plane_path}: validity"
    )
    z_max_mm = _required_finite_float(
        validity, "camera_z_max_mm", f"{plane_path}: validity"
    )
    if z_min_mm <= 0.0 or z_max_mm <= z_min_mm:
        raise CalibrationConfigError(
            f"{plane_path}: validity requires 0 < camera_z_min_mm < camera_z_max_mm"
        )

    intrinsics_config = _load_mapping(intrinsics_path)
    if intrinsics_config.get("calibration_type") != "camera_intrinsics":
        raise CalibrationConfigError(
            f"{intrinsics_path}: calibration_type must be 'camera_intrinsics'"
        )
    width = _required_integer(intrinsics_config, "image_width", intrinsics_path)
    height = _required_integer(intrinsics_config, "image_height", intrinsics_path)
    if width < 2 or height < 2:
        raise CalibrationConfigError(
            f"{intrinsics_path}: image dimensions must both be at least 2"
        )
    camera_matrix = _required_mapping(
        intrinsics_config, "camera_matrix", intrinsics_path
    )
    matrix_data = _finite_sequence(
        camera_matrix.get("data"),
        9,
        f"{intrinsics_path}: camera_matrix.data",
    )
    fx, fy = matrix_data[0], matrix_data[4]
    cx, cy = matrix_data[2], matrix_data[5]
    if fx <= 0.0 or fy <= 0.0:
        raise CalibrationConfigError(
            f"{intrinsics_path}: camera focal lengths must be positive"
        )

    metadata = intrinsics_config.get("metadata")
    if isinstance(metadata, Mapping) and "frame_id" in metadata:
        intrinsics_frame = _required_string(metadata, "frame_id", intrinsics_path)
        if intrinsics_frame != camera_frame:
            raise CalibrationConfigError(
                f"camera-frame mismatch: plane uses '{camera_frame}', "
                f"intrinsics use '{intrinsics_frame}'"
            )

    intrinsics_reference = _required_mapping(
        plane_config, "intrinsics", plane_path
    )
    expected_sha256 = _required_string(
        intrinsics_reference, "sha256", f"{plane_path}: intrinsics"
    ).lower()
    try:
        actual_sha256 = hashlib.sha256(
            Path(intrinsics_path).expanduser().read_bytes()
        ).hexdigest()
    except OSError as exc:
        raise CalibrationConfigError(
            f"cannot hash intrinsics file {intrinsics_path}: {exc}"
        ) from exc
    if actual_sha256 != expected_sha256:
        raise CalibrationConfigError(
            f"{plane_path}: intrinsics SHA-256 mismatch for {intrinsics_path}"
        )

    vertices_mm = []
    # Order the four corners around the polygon so two triangles share a diagonal.
    for z_mm, u in (
        (z_min_mm, 0.0),
        (z_min_mm, float(width - 1)),
        (z_max_mm, float(width - 1)),
        (z_max_mm, 0.0),
    ):
        x_mm = (u - cx) * z_mm / fx
        y_mm = -(nx * x_mm + nz * z_mm + d) / ny
        v = fy * y_mm / z_mm + cy
        if v < -1.0e-6 or v > float(height - 1) + 1.0e-6:
            raise CalibrationConfigError(
                f"{plane_path}: calibrated plane leaves the image at "
                f"z={z_mm:g} mm, u={u:g} px (v={v:.3f} px)"
            )
        vertices_mm.append((x_mm, y_mm, z_mm))

    return LaserPlaneCalibration(
        camera_frame=camera_frame,
        coefficients_mm=coefficients,  # type: ignore[arg-type]
        vertices_m=tuple(
            tuple(component / 1000.0 for component in vertex)
            for vertex in vertices_mm
        ),  # type: ignore[arg-type]
        z_range_m=(z_min_mm / 1000.0, z_max_mm / 1000.0),
    )
