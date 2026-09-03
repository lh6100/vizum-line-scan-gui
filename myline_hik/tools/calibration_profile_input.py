#!/usr/bin/env python3
"""Read the small, stable subset of device calibration YAML used by Blender.

Blender's bundled Python does not guarantee PyYAML or OpenCV.  This module
therefore parses only the scalar and flat-array fields emitted by this
project's calibration writer, and fails loudly when the schema is incomplete.
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path


_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Cannot read calibration file {path}: {exc}") from exc


def _integer(text: str, key: str, path: Path) -> int:
    match = re.search(rf"(?m)^{re.escape(key)}:\s*(\d+)\s*$", text)
    if not match:
        raise ValueError(f"Missing integer field {key} in {path}")
    return int(match.group(1))


def _array(text: str, key: str, path: Path, expected: int) -> tuple[float, ...]:
    match = re.search(
        rf"(?m)^\s*{re.escape(key)}:\s*\[([^\]]+)\]\s*$", text
    )
    if not match:
        raise ValueError(f"Missing array field {key} in {path}")
    values = tuple(float(value) for value in re.findall(_NUMBER, match.group(1)))
    if len(values) != expected:
        raise ValueError(
            f"Field {key} in {path} has {len(values)} values; expected {expected}"
        )
    return values


def _camera_matrix(text: str, path: Path) -> tuple[float, ...]:
    match = re.search(
        r"(?ms)^camera_matrix:\s*$.*?^\s+data:\s*\[([^\]]+)\]\s*$", text
    )
    if not match:
        raise ValueError(f"Missing camera_matrix.data in {path}")
    values = tuple(float(value) for value in re.findall(_NUMBER, match.group(1)))
    if len(values) != 9:
        raise ValueError(f"camera_matrix.data in {path} must contain 9 values")
    return values


def _declared_intrinsics_sha256(text: str, path: Path) -> str:
    match = re.search(
        r'(?m)^\s*intrinsics_sha256:\s*["\']?([0-9a-fA-F]{64})["\']?\s*$',
        text,
    )
    if not match:
        raise ValueError(f"Missing sources.intrinsics_sha256 in {path}")
    return match.group(1).lower()


def load_intrinsics(project_root: Path, profile_id: str) -> dict[str, float | int]:
    path = project_root / "config" / "devices" / profile_id / "hik_intrinsics.yaml"
    text = _read(path)
    matrix = _camera_matrix(text, path)
    return {
        "width": _integer(text, "image_width", path),
        "height": _integer(text, "image_height", path),
        "fx": matrix[0],
        "fy": matrix[4],
        "cx": matrix[2],
        "cy": matrix[5],
    }


def load_handeye(project_root: Path, profile_id: str) -> tuple[tuple[float, ...], ...]:
    device_dir = project_root / "config" / "devices" / profile_id
    intrinsics_path = device_dir / "hik_intrinsics.yaml"
    handeye_path = device_dir / "hik_handeye.yaml"
    handeye_text = _read(handeye_path)
    declared_hash = _declared_intrinsics_sha256(handeye_text, handeye_path)
    actual_hash = hashlib.sha256(intrinsics_path.read_bytes()).hexdigest()
    if declared_hash != actual_hash:
        raise ValueError(
            f"{profile_id} hand-eye references intrinsics {declared_hash}, "
            f"but the current device intrinsics hash is {actual_hash}; recalibrate first"
        )
    values = _array(handeye_text, "T_flange_camera", handeye_path, 16)
    return tuple(tuple(values[row * 4 : row * 4 + 4]) for row in range(4))


def load_camera_profile(
    project_root: Path, profile_id: str, *, include_handeye: bool = False
) -> dict[str, object]:
    result: dict[str, object] = dict(load_intrinsics(project_root, profile_id))
    if include_handeye:
        result["transform"] = load_handeye(project_root, profile_id)
    return result
