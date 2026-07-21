"""Load the repository's calibrated flange-to-TCP and flange-to-camera transforms."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Tuple

import yaml


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Matrix4 = Tuple[Tuple[float, float, float, float], ...]


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
