from pathlib import Path

import pytest

from fr5_vizum_bringup.calibration_frames import (
    CalibrationConfigError,
    load_camera_calibration,
    load_laser_plane_calibration,
    load_tool_calibration,
)


PROJECT_ROOT = Path(__file__).resolve().parents[4]


def test_loads_repository_tool_calibration():
    calibration = load_tool_calibration(str(PROJECT_ROOT / "config/tool_config.yaml"))

    assert calibration.tool_id == 4
    assert calibration.transform.translation_m == pytest.approx(
        (0.010560, -0.205389, 0.368751), abs=1.0e-12
    )
    assert calibration.transform.quaternion_xyzw == pytest.approx(
        (-0.795949053, -0.598596690, -0.052814278, 0.073196723),
        abs=1.0e-8,
    )


def test_loads_repository_camera_to_flange_as_flange_to_camera():
    calibration = load_camera_calibration(
        str(PROJECT_ROOT / "config/handeye_config.yaml")
    )

    assert calibration.input_mode == "camera_to_flange"
    assert calibration.transform.translation_m == pytest.approx(
        (-0.114797756410, -0.051832484027, 0.126355615148), abs=1.0e-12
    )
    assert calibration.transform.quaternion_xyzw == pytest.approx(
        (0.082789193, -0.074566812, -0.993708382, 0.011375001),
        abs=1.0e-8,
    )


def test_inverts_flange_to_camera_input_mode(tmp_path):
    config = tmp_path / "inverse.yaml"
    config.write_text(
        """mode: flange_to_camera
m00: 1
m01: 0
m02: 0
m03: 100
m10: 0
m11: 1
m12: 0
m13: -200
m20: 0
m21: 0
m22: 1
m23: 300
m30: 0
m31: 0
m32: 0
m33: 1
""",
        encoding="utf-8",
    )

    calibration = load_camera_calibration(str(config))

    assert calibration.transform.translation_m == pytest.approx(
        (-0.1, 0.2, -0.3), abs=1.0e-12
    )
    assert calibration.transform.quaternion_xyzw == pytest.approx(
        (0.0, 0.0, 0.0, 1.0), abs=1.0e-12
    )


@pytest.mark.parametrize(
    "body, expected_message",
    [
        ("mode: wrong\n", "mode must be"),
        (
            """mode: camera_to_flange
m00: 2
m01: 0
m02: 0
m03: 0
m10: 0
m11: 1
m12: 0
m13: 0
m20: 0
m21: 0
m22: 1
m23: 0
m30: 0
m31: 0
m32: 0
m33: 1
""",
            "rotation is not rigid",
        ),
    ],
)
def test_rejects_invalid_handeye_config(tmp_path, body, expected_message):
    config = tmp_path / "invalid.yaml"
    config.write_text(body, encoding="utf-8")

    with pytest.raises(CalibrationConfigError, match=expected_message):
        load_camera_calibration(str(config))


def test_rejects_non_finite_tool_config(tmp_path):
    config = tmp_path / "invalid_tool.yaml"
    config.write_text(
        "tool_id: 4\nx: .nan\ny: 0\nz: 0\nrx: 0\nry: 0\nrz: 0\n",
        encoding="utf-8",
    )

    with pytest.raises(CalibrationConfigError, match="must be finite"):
        load_tool_calibration(str(config))


@pytest.mark.parametrize(
    "scanner, expected_frame",
    [
        ("scanner_650", "hik_camera_optical_frame"),
        ("scanner_450", "hik_450_camera_optical_frame"),
    ],
)
def test_loads_repository_laser_plane_in_camera_frustum(scanner, expected_frame):
    config_dir = PROJECT_ROOT / "myline_hik/config/devices" / scanner
    calibration = load_laser_plane_calibration(
        str(config_dir / "hik_laser_plane.yaml"),
        str(config_dir / "hik_intrinsics.yaml"),
    )

    assert calibration.camera_frame == expected_frame
    assert calibration.z_range_m == pytest.approx((0.4, 0.7), abs=1.0e-12)
    assert tuple(vertex[2] for vertex in calibration.vertices_m) == pytest.approx(
        (0.4, 0.4, 0.7, 0.7), abs=1.0e-12
    )
    nx, ny, nz, d = calibration.coefficients_mm
    for vertex_m in calibration.vertices_m:
        x_mm, y_mm, z_mm = (component * 1000.0 for component in vertex_m)
        assert nx * x_mm + ny * y_mm + nz * z_mm + d == pytest.approx(
            0.0, abs=1.0e-9
        )


def test_rejects_laser_plane_with_wrong_camera_intrinsics():
    devices = PROJECT_ROOT / "myline_hik/config/devices"

    with pytest.raises(CalibrationConfigError, match="camera-frame mismatch"):
        load_laser_plane_calibration(
            str(devices / "scanner_650/hik_laser_plane.yaml"),
            str(devices / "scanner_450/hik_intrinsics.yaml"),
        )
