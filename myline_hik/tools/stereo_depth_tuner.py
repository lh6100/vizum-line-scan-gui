#!/usr/bin/env python3
"""Direct Hikrobot stereo depth tuning GUI (no ROS 2 dependency)."""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import yaml
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
import cv2

# The non-headless opencv-python wheel injects its own Qt plugin directory.
# That xcb plugin is binary-incompatible with PyQt5 in some Conda environments.
# This application uses PyQt5 for all display, so discard only OpenCV's path.
for _qt_variable in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH"):
    _qt_value = os.environ.get(_qt_variable, "")
    if "/cv2/qt/plugins" in _qt_value:
        os.environ.pop(_qt_variable, None)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "stereo_depth_tuner.yaml"
DEFAULT_STEREO = PROJECT_ROOT / "config" / "hik_stereo.yaml"
DEFAULT_LEFT_INTRINSICS = (
    PROJECT_ROOT / "config" / "devices" / "scanner_650" / "hik_intrinsics.yaml"
)
DEFAULT_RIGHT_INTRINSICS = (
    PROJECT_ROOT / "config" / "devices" / "scanner_450" / "hik_intrinsics.yaml"
)
MVS_IMPORT_PATH = Path(
    os.environ.get("HIK_MVS_ROOT", "/opt/MVS")
) / "Samples/64/Python/MvImport"

MVS_IMPORT_ERROR: Optional[Exception] = None
if str(MVS_IMPORT_PATH) not in sys.path:
    sys.path.insert(0, str(MVS_IMPORT_PATH))
try:
    from MvCameraControl_class import *  # type: ignore # noqa: F403,F401
except Exception as exc:  # pragma: no cover - depends on installed vendor SDK
    MVS_IMPORT_ERROR = exc


def decode_c_string(value: Any) -> str:
    raw = memoryview(value).tobytes().split(b"\0", 1)[0]
    for encoding in ("utf-8", "gbk", "latin-1"):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            pass
    return raw.decode("latin-1", errors="replace")


def sdk_error(operation: str, code: int) -> RuntimeError:
    return RuntimeError(f"{operation} 失败，MVS错误码=0x{int(code) & 0xFFFFFFFF:08X}")


def require_sdk() -> None:
    if MVS_IMPORT_ERROR is not None:
        raise RuntimeError(
            f"无法加载海康 MVS Python SDK：{MVS_IMPORT_ERROR}；"
            f"期望路径为 {MVS_IMPORT_PATH}"
        )


@dataclass(frozen=True)
class CameraIdentity:
    serial: str
    model: str
    ip: str
    device_info: Any


def enumerate_gige_cameras() -> Dict[str, CameraIdentity]:
    require_sdk()
    devices = MV_CC_DEVICE_INFO_LIST()  # type: ignore # noqa: F405
    ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE, devices)  # type: ignore # noqa: F405
    if ret != 0:
        raise sdk_error("枚举 GigE 相机", ret)
    result: Dict[str, CameraIdentity] = {}
    for index in range(int(devices.nDeviceNum)):
        source = ctypes.cast(
            devices.pDeviceInfo[index], ctypes.POINTER(MV_CC_DEVICE_INFO)  # type: ignore # noqa: F405
        ).contents
        if int(source.nTLayerType) != int(MV_GIGE_DEVICE):  # type: ignore # noqa: F405
            continue
        info = source.SpecialInfo.stGigEInfo
        serial = decode_c_string(info.chSerialNumber)
        model = decode_c_string(info.chModelName)
        ip_value = int(info.nCurrentIp)
        ip = ".".join(
            str((ip_value >> shift) & 0xFF) for shift in (24, 16, 8, 0)
        )
        copied = MV_CC_DEVICE_INFO()  # type: ignore # noqa: F405
        ctypes.memmove(ctypes.byref(copied), ctypes.byref(source), ctypes.sizeof(copied))
        result[serial] = CameraIdentity(serial, model, ip, copied)
    return result


class CameraThread(QThread):
    frame_ready = pyqtSignal(str, object, int, int)
    opened = pyqtSignal(str, str)
    settings_applied = pyqtSignal(str, float, float, float)
    problem = pyqtSignal(str, str)
    log_message = pyqtSignal(str)

    def __init__(
        self,
        side: str,
        identity: CameraIdentity,
        expected_size: Tuple[int, int],
        exposure_us: float,
        gain_db: float,
        fps: float,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.side = side
        self.identity = identity
        self.expected_size = expected_size
        self._stop_event = threading.Event()
        self._settings_lock = threading.Lock()
        self._pending_settings: Optional[Tuple[float, float, float]] = (
            exposure_us,
            gain_db,
            fps,
        )
        self._camera: Any = None

    def request_settings(self, exposure_us: float, gain_db: float, fps: float) -> None:
        with self._settings_lock:
            self._pending_settings = (exposure_us, gain_db, fps)

    def request_stop(self) -> None:
        self._stop_event.set()

    def _take_settings(self) -> Optional[Tuple[float, float, float]]:
        with self._settings_lock:
            value = self._pending_settings
            self._pending_settings = None
            return value

    def _check(self, code: int, operation: str) -> None:
        if code != 0:
            raise sdk_error(f"{self.side}相机{operation}", code)

    def _reset_full_sensor(self) -> None:
        camera = self._camera
        for node in (
            "BinningHorizontal",
            "BinningVertical",
            "DecimationHorizontal",
            "DecimationVertical",
        ):
            ret = camera.MV_CC_SetEnumValue(node, 1)
            if ret != 0:
                self.log_message.emit(
                    f"{self.side}相机忽略不支持的 {node}=1，ret=0x{ret & 0xFFFFFFFF:08X}"
                )
        for node in ("OffsetX", "OffsetY"):
            value = MVCC_INTVALUE_EX()  # type: ignore # noqa: F405
            self._check(camera.MV_CC_GetIntValueEx(node, value), f"读取{node}")
            self._check(camera.MV_CC_SetIntValueEx(node, int(value.nMin)), f"设置{node}")
        configured = []
        for node in ("Width", "Height"):
            value = MVCC_INTVALUE_EX()  # type: ignore # noqa: F405
            self._check(camera.MV_CC_GetIntValueEx(node, value), f"读取{node}")
            self._check(camera.MV_CC_SetIntValueEx(node, int(value.nMax)), f"设置{node}")
            configured.append(int(value.nMax))
        if tuple(configured) != self.expected_size:
            raise RuntimeError(
                f"{self.side}相机全幅尺寸为 {configured[0]}x{configured[1]}，"
                f"但标定要求 {self.expected_size[0]}x{self.expected_size[1]}"
            )

    def _apply_settings(self, settings: Tuple[float, float, float]) -> None:
        exposure_us, gain_db, fps = settings
        camera = self._camera
        self._check(
            camera.MV_CC_SetEnumValueByString("ExposureAuto", "Off"),
            "关闭自动曝光",
        )
        self._check(
            camera.MV_CC_SetEnumValueByString("GainAuto", "Off"),
            "关闭自动增益",
        )
        self._check(
            camera.MV_CC_SetFloatValue("ExposureTime", float(exposure_us)),
            "设置曝光",
        )
        self._check(
            camera.MV_CC_SetFloatValue("Gain", float(gain_db)),
            "设置增益",
        )
        self._check(
            camera.MV_CC_SetBoolValue("AcquisitionFrameRateEnable", True),
            "启用帧率控制",
        )
        self._check(
            camera.MV_CC_SetFloatValue("AcquisitionFrameRate", float(fps)),
            "设置帧率",
        )
        actual_exposure = MVCC_FLOATVALUE()  # type: ignore # noqa: F405
        actual_gain = MVCC_FLOATVALUE()  # type: ignore # noqa: F405
        actual_fps = MVCC_FLOATVALUE()  # type: ignore # noqa: F405
        self._check(camera.MV_CC_GetFloatValue("ExposureTime", actual_exposure), "读取曝光")
        self._check(camera.MV_CC_GetFloatValue("Gain", actual_gain), "读取增益")
        self._check(
            camera.MV_CC_GetFloatValue("AcquisitionFrameRate", actual_fps),
            "读取帧率",
        )
        self.settings_applied.emit(
            self.side,
            float(actual_exposure.fCurValue),
            float(actual_gain.fCurValue),
            float(actual_fps.fCurValue),
        )

    def run(self) -> None:  # noqa: C901 - vendor lifecycle is intentionally linear
        camera = None
        grabbing = False
        opened = False
        try:
            camera = MvCamera()  # type: ignore # noqa: F405
            self._camera = camera
            self._check(camera.MV_CC_CreateHandle(self.identity.device_info), "创建句柄")
            self._check(camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0), "独占打开")  # type: ignore # noqa: F405
            opened = True
            self._reset_full_sensor()
            packet_size = int(camera.MV_CC_GetOptimalPacketSize())
            if 0 < packet_size < 65536:
                ret = camera.MV_CC_SetIntValueEx("GevSCPSPacketSize", packet_size)
                if ret == 0:
                    self.log_message.emit(f"{self.side}相机 GigE packet size={packet_size}")
            self._check(
                camera.MV_CC_SetEnumValueByString("PixelFormat", "Mono8"),
                "设置Mono8",
            )
            self._check(
                camera.MV_CC_SetEnumValueByString("AcquisitionMode", "Continuous"),
                "设置连续模式",
            )
            self._check(
                camera.MV_CC_SetEnumValueByString("TriggerMode", "Off"),
                "关闭触发模式",
            )
            settings = self._take_settings()
            if settings is None:
                raise RuntimeError(f"{self.side}相机缺少初始曝光设置")
            self._apply_settings(settings)
            ret = camera.MV_CC_SetImageNodeNum(8)
            if ret != 0:
                self.log_message.emit(
                    f"{self.side}相机无法设置SDK缓存节点数，ret=0x{ret & 0xFFFFFFFF:08X}"
                )
            self._check(camera.MV_CC_StartGrabbing(), "开始取流")
            grabbing = True
            self.opened.emit(
                self.side,
                f"{self.identity.model} / {self.identity.serial} / {self.identity.ip}",
            )
            out_frame = MV_FRAME_OUT()  # type: ignore # noqa: F405
            while not self._stop_event.is_set():
                settings = self._take_settings()
                if settings is not None:
                    self._apply_settings(settings)
                ctypes.memset(ctypes.byref(out_frame), 0, ctypes.sizeof(out_frame))
                ret = camera.MV_CC_GetImageBuffer(out_frame, 500)
                if ret != 0:
                    continue
                try:
                    info = out_frame.stFrameInfo
                    width = int(info.nExtendWidth or info.nWidth)
                    height = int(info.nExtendHeight or info.nHeight)
                    frame_length = int(info.nFrameLenEx or info.nFrameLen)
                    expected = width * height
                    if (
                        width != self.expected_size[0]
                        or height != self.expected_size[1]
                        or frame_length < expected
                        or int(info.enPixelType) != int(PixelType_Gvsp_Mono8)  # type: ignore # noqa: F405
                    ):
                        raise RuntimeError(
                            f"{self.side}相机帧不符合标定：{width}x{height}, "
                            f"pixel=0x{int(info.enPixelType):08X}, len={frame_length}"
                        )
                    data = ctypes.string_at(out_frame.pBufAddr, expected)
                    image = np.frombuffer(data, dtype=np.uint8).reshape(height, width).copy()
                    self.frame_ready.emit(
                        self.side,
                        image,
                        time.monotonic_ns(),
                        int(info.nFrameNum),
                    )
                finally:
                    camera.MV_CC_FreeImageBuffer(out_frame)
        except Exception as exc:
            self.problem.emit(self.side, str(exc))
        finally:
            if camera is not None and grabbing:
                camera.MV_CC_StopGrabbing()
            if camera is not None and opened:
                camera.MV_CC_CloseDevice()
            if camera is not None:
                camera.MV_CC_DestroyHandle()
            self._camera = None


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict):
        raise RuntimeError(f"YAML根节点不是映射：{path}")
    return value


def centered_crop(source: Tuple[int, int], target: Tuple[int, int]) -> Tuple[int, int, int, int]:
    source_width, source_height = source
    target_width, target_height = target
    target_aspect = target_width / target_height
    source_aspect = source_width / source_height
    if source_aspect > target_aspect:
        width = max(1, min(source_width, round(source_height * target_aspect)))
        return ((source_width - width) // 2, 0, width, source_height)
    height = max(1, min(source_height, round(source_width / target_aspect)))
    return (0, (source_height - height) // 2, source_width, height)


def adjusted_camera_matrix(
    source: np.ndarray,
    crop: Tuple[int, int, int, int],
    output_size: Tuple[int, int],
) -> np.ndarray:
    x, y, width, height = crop
    output_width, output_height = output_size
    result = np.array(source, dtype=np.float64, copy=True)
    scale_x = output_width / width
    scale_y = output_height / height
    result[0, 0] *= scale_x
    result[1, 1] *= scale_y
    result[0, 2] = (result[0, 2] - x) * scale_x
    result[1, 2] = (result[1, 2] - y) * scale_y
    return result


def crop_and_resize(
    source: np.ndarray,
    crop: Tuple[int, int, int, int],
    output_size: Tuple[int, int],
) -> np.ndarray:
    x, y, width, height = crop
    region = source[y : y + height, x : x + width]
    if (region.shape[1], region.shape[0]) == output_size:
        return region.copy()
    return cv2.resize(region, output_size, interpolation=cv2.INTER_AREA)


class StereoDepthEngine:
    def __init__(
        self,
        stereo_path: Path = DEFAULT_STEREO,
        left_intrinsics_path: Path = DEFAULT_LEFT_INTRINSICS,
        right_intrinsics_path: Path = DEFAULT_RIGHT_INTRINSICS,
    ) -> None:
        stereo = load_yaml(stereo_path)
        left = load_yaml(left_intrinsics_path)
        right = load_yaml(right_intrinsics_path)
        if stereo.get("left_camera_serial") != left.get("metadata", {}).get("camera_serial"):
            raise RuntimeError("双目标定左相机序列号与左内参不一致")
        if stereo.get("right_camera_serial") != right.get("metadata", {}).get("camera_serial"):
            raise RuntimeError("双目标定右相机序列号与右内参不一致")
        self.left_serial = str(stereo["left_camera_serial"])
        self.right_serial = str(stereo["right_camera_serial"])
        self.left_size = (int(left["image_width"]), int(left["image_height"]))
        self.right_size = (int(right["image_width"]), int(right["image_height"]))
        self.left_k = np.array(left["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        self.right_k = np.array(right["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        self.left_d = np.array(left["distortion_coefficients"]["data"], dtype=np.float64)
        self.right_d = np.array(right["distortion_coefficients"]["data"], dtype=np.float64)
        self.rotation = np.array(stereo["R_right_left"], dtype=np.float64).reshape(3, 3)
        self.translation_mm = np.array(stereo["T_right_left_mm"], dtype=np.float64).reshape(3, 1)
        self.baseline_mm = float(np.linalg.norm(self.translation_mm))
        self._signature: Optional[Tuple[Any, ...]] = None
        self._geometry: Dict[str, Any] = {}

    @staticmethod
    def _signature_for(config: Dict[str, Any]) -> Tuple[Any, ...]:
        keys = (
            "processing_width",
            "processing_height",
            "minimum_depth_mm",
            "maximum_depth_mm",
            "block_size",
            "uniqueness_ratio",
            "speckle_window_size",
            "speckle_range",
            "left_right_maximum_difference_px",
            "disparity_margin_px",
            "maximum_num_disparities",
            "enable_left_right_check",
            "enable_clahe",
            "clahe_clip_limit",
        )
        return tuple(config[key] for key in keys)

    def configure(self, config: Dict[str, Any]) -> None:
        signature = self._signature_for(config)
        if signature == self._signature:
            return
        width = int(config["processing_width"])
        height = int(config["processing_height"])
        minimum_depth = float(config["minimum_depth_mm"])
        maximum_depth = float(config["maximum_depth_mm"])
        block_size = int(config["block_size"])
        if width < 160 or height < 120:
            raise RuntimeError("处理分辨率不能小于160x120")
        if minimum_depth < 100 or maximum_depth <= minimum_depth:
            raise RuntimeError("深度范围无效")
        if block_size < 3 or block_size > 21 or block_size % 2 == 0:
            raise RuntimeError("SGBM块大小必须是3到21之间的奇数")
        output_size = (width, height)
        left_crop = centered_crop(self.left_size, output_size)
        right_crop = centered_crop(self.right_size, output_size)
        left_k = adjusted_camera_matrix(self.left_k, left_crop, output_size)
        right_k = adjusted_camera_matrix(self.right_k, right_crop, output_size)
        rectify = cv2.stereoRectify(
            left_k,
            self.left_d,
            right_k,
            self.right_d,
            output_size,
            self.rotation,
            self.translation_mm,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0.0,
            newImageSize=output_size,
        )
        left_r, right_r, left_p, right_p, q, left_roi, right_roi = rectify
        left_maps = cv2.initUndistortRectifyMap(
            left_k, self.left_d, left_r, left_p, output_size, cv2.CV_32FC1
        )
        right_maps = cv2.initUndistortRectifyMap(
            right_k, self.right_d, right_r, right_p, output_size, cv2.CV_32FC1
        )
        baseline_focal = -float(right_p[0, 3])
        if not math.isfinite(baseline_focal) or abs(baseline_focal) < 1.0:
            raise RuntimeError("校正后的基线或焦距退化")
        disparity_near = baseline_focal / minimum_depth
        disparity_far = baseline_focal / maximum_depth
        disparity_margin = int(config["disparity_margin_px"])
        minimum_disparity = math.floor(min(disparity_near, disparity_far)) - disparity_margin
        required = math.ceil(max(disparity_near, disparity_far)) - minimum_disparity + disparity_margin + 1
        number_disparities = max(16, ((required + 15) // 16) * 16)
        if number_disparities > int(config["maximum_num_disparities"]):
            raise RuntimeError(
                f"需要{number_disparities}个视差层，超过上限；请提高最小深度或提高最大视差数"
            )
        if number_disparities >= width:
            raise RuntimeError("视差范围大于处理图像宽度")
        valid_roi = cv2.getValidDisparityROI(
            tuple(left_roi),
            tuple(right_roi),
            int(minimum_disparity),
            int(number_disparities),
            block_size,
        )
        if valid_roi[2] <= 0 or valid_roi[3] <= 0:
            raise RuntimeError("立体校正后没有有效视差区域")
        self._geometry = {
            "output_size": output_size,
            "left_crop": left_crop,
            "right_crop": right_crop,
            "left_maps": left_maps,
            "right_maps": right_maps,
            "q": q,
            "valid_roi": valid_roi,
            "minimum_disparity": int(minimum_disparity),
            "number_disparities": int(number_disparities),
        }
        self._signature = signature

    @staticmethod
    def _matcher(config: Dict[str, Any], minimum: int, count: int) -> Any:
        block = int(config["block_size"])
        return cv2.StereoSGBM_create(
            minDisparity=minimum,
            numDisparities=count,
            blockSize=block,
            P1=8 * block * block,
            P2=32 * block * block,
            disp12MaxDiff=int(config["left_right_maximum_difference_px"]),
            preFilterCap=31,
            uniquenessRatio=int(config["uniqueness_ratio"]),
            speckleWindowSize=int(config["speckle_window_size"]),
            speckleRange=int(config["speckle_range"]),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def compute(
        self,
        left_raw: np.ndarray,
        right_raw: np.ndarray,
        config: Dict[str, Any],
        pair_skew_ms: float,
    ) -> Dict[str, Any]:
        started = time.perf_counter()
        self.configure(config)
        geometry = self._geometry
        output_size = geometry["output_size"]
        left = crop_and_resize(left_raw, geometry["left_crop"], output_size)
        right = crop_and_resize(right_raw, geometry["right_crop"], output_size)
        left_rect = cv2.remap(
            left, *geometry["left_maps"], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
        )
        right_rect = cv2.remap(
            right, *geometry["right_maps"], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
        )
        if bool(config["enable_clahe"]):
            clahe = cv2.createCLAHE(
                clipLimit=float(config["clahe_clip_limit"]), tileGridSize=(8, 8)
            )
            left_match = clahe.apply(left_rect)
            right_match = clahe.apply(right_rect)
        else:
            left_match = left_rect
            right_match = right_rect
        minimum = int(geometry["minimum_disparity"])
        count = int(geometry["number_disparities"])
        disparity = self._matcher(config, minimum, count).compute(
            left_match, right_match
        ).astype(np.float32) / 16.0
        valid = np.zeros(disparity.shape, dtype=bool)
        roi_x, roi_y, roi_w, roi_h = geometry["valid_roi"]
        valid[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w] = True
        valid &= disparity > float(minimum)
        valid &= disparity <= float(minimum + count - 1)
        confidence = np.zeros(disparity.shape, dtype=np.float32)
        if bool(config["enable_left_right_check"]):
            right_minimum = -(minimum + count - 1)
            right_disparity = self._matcher(config, right_minimum, count).compute(
                right_match, left_match
            ).astype(np.float32) / 16.0
            rows, columns = np.indices(disparity.shape)
            right_x = np.rint(columns - disparity).astype(np.int32)
            in_bounds = (right_x >= 0) & (right_x < disparity.shape[1])
            clipped_x = np.clip(right_x, 0, disparity.shape[1] - 1)
            residual = np.abs(disparity + right_disparity[rows, clipped_x])
            maximum_difference = float(config["left_right_maximum_difference_px"])
            valid &= in_bounds & np.isfinite(residual) & (residual <= maximum_difference)
            confidence[valid] = np.maximum(
                0.0,
                1.0 - residual[valid] / max(1.0, maximum_difference),
            )
        else:
            confidence[valid] = 1.0
        xyz_mm = cv2.reprojectImageTo3D(disparity, geometry["q"], handleMissingValues=False)
        depth_mm = xyz_mm[:, :, 2]
        minimum_depth = float(config["minimum_depth_mm"])
        maximum_depth = float(config["maximum_depth_mm"])
        valid &= np.isfinite(xyz_mm).all(axis=2)
        valid &= (depth_mm >= minimum_depth) & (depth_mm <= maximum_depth)
        valid_depths = depth_mm[valid]
        median_depth = float(np.median(valid_depths)) if valid_depths.size else 0.0
        normalized = np.zeros(disparity.shape, dtype=np.uint8)
        if maximum_depth > minimum_depth:
            ratio = 1.0 - np.clip(
                (depth_mm - minimum_depth) / (maximum_depth - minimum_depth),
                0.0,
                1.0,
            )
            normalized[valid] = np.rint(ratio[valid] * 255.0).astype(np.uint8)
        depth_preview = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        depth_preview[~valid] = 0
        epipolar = np.dstack((right_match, left_match, right_match))
        for row in range(0, epipolar.shape[0], 40):
            cv2.line(epipolar, (0, row), (epipolar.shape[1] - 1, row), (0, 255, 255), 1)
        return {
            "left_rect": left_rect,
            "right_rect": right_rect,
            "epipolar": epipolar,
            "depth_preview": depth_preview,
            "depth_mm": depth_mm,
            "valid_mask": valid,
            "confidence": confidence,
            "pair_skew_ms": pair_skew_ms,
            "valid_fraction": float(np.count_nonzero(valid) / valid.size),
            "median_depth_mm": median_depth,
            "minimum_disparity": minimum,
            "number_disparities": count,
            "processing_ms": (time.perf_counter() - started) * 1000.0,
        }


class DepthThread(QThread):
    result_ready = pyqtSignal(object)
    problem = pyqtSignal(str)

    def __init__(self, engine: StereoDepthEngine) -> None:
        super().__init__()
        self.engine = engine
        self._condition = threading.Condition()
        self._pending: Optional[Tuple[np.ndarray, np.ndarray, Dict[str, Any], float]] = None
        self._stop_requested = False

    def submit(
        self,
        left: np.ndarray,
        right: np.ndarray,
        config: Dict[str, Any],
        skew_ms: float,
    ) -> None:
        with self._condition:
            self._pending = (left, right, dict(config), skew_ms)
            self._condition.notify()

    def request_stop(self) -> None:
        with self._condition:
            self._stop_requested = True
            self._condition.notify()

    def run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._stop_requested:
                    self._condition.wait()
                if self._stop_requested:
                    return
                work = self._pending
                self._pending = None
            if work is None:
                continue
            try:
                self.result_ready.emit(self.engine.compute(*work))
            except Exception as exc:
                self.problem.emit(str(exc))


class ImageView(QLabel):
    def __init__(self, title: str) -> None:
        super().__init__(title)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(360, 260)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("QLabel { background: #17191c; color: #adb5bd; }")
        self._image: Optional[QImage] = None

    def set_array(self, array: np.ndarray) -> None:
        if array.ndim == 2:
            contiguous = np.ascontiguousarray(array)
            image = QImage(
                contiguous.data,
                contiguous.shape[1],
                contiguous.shape[0],
                contiguous.strides[0],
                QImage.Format_Grayscale8,
            )
        else:
            rgb = np.ascontiguousarray(cv2.cvtColor(array, cv2.COLOR_BGR2RGB))
            image = QImage(
                rgb.data,
                rgb.shape[1],
                rgb.shape[0],
                rgb.strides[0],
                QImage.Format_RGB888,
            )
        self._image = image.copy()
        self._refresh()

    def _refresh(self) -> None:
        if self._image is None:
            return
        pixmap = QPixmap.fromImage(self._image).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.setPixmap(pixmap)

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        self._refresh()


def default_settings() -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "camera": {
            "left_exposure_us": 18000.0,
            "right_exposure_us": 18000.0,
            "gain_db": 0.0,
            "frames_per_second": 5.0,
            "maximum_pair_skew_ms": 30.0,
        },
        "depth": {
            "processing_width": 612,
            "processing_height": 512,
            "minimum_depth_mm": 400.0,
            "maximum_depth_mm": 1200.0,
            "block_size": 5,
            "uniqueness_ratio": 5,
            "speckle_window_size": 50,
            "speckle_range": 2,
            "left_right_maximum_difference_px": 2,
            "disparity_margin_px": 16,
            "maximum_num_disparities": 512,
            "enable_left_right_check": False,
            "enable_clahe": True,
            "clahe_clip_limit": 2.0,
        },
        "multi_band": {
            "enabled": True,
            "ranges_mm": [
                [300.0, 600.0],
                [600.0, 1200.0],
                [1200.0, 2500.0],
            ],
        },
    }


def merge_settings(base: Dict[str, Any], loaded: Dict[str, Any]) -> Dict[str, Any]:
    result = {
        "schema_version": 1,
        "camera": dict(base["camera"]),
        "depth": dict(base["depth"]),
        "multi_band": {
            "enabled": bool(base["multi_band"]["enabled"]),
            "ranges_mm": [list(pair) for pair in base["multi_band"]["ranges_mm"]],
        },
    }
    if isinstance(loaded.get("camera"), dict):
        result["camera"].update(loaded["camera"])
    if isinstance(loaded.get("depth"), dict):
        result["depth"].update(loaded["depth"])
    if isinstance(loaded.get("multi_band"), dict):
        result["multi_band"].update(loaded["multi_band"])
    return result


class StereoDepthTunerWindow(QMainWindow):
    def __init__(self, config_path: Path) -> None:
        super().__init__()
        self.config_path = config_path
        self.engine = StereoDepthEngine()
        self.settings = default_settings()
        if config_path.exists():
            self.settings = merge_settings(self.settings, load_yaml(config_path))
        self.depth_config = dict(self.settings["depth"])
        self.camera_threads: Dict[str, CameraThread] = {}
        self.frame_queues: Dict[str, deque] = {
            "左": deque(maxlen=8),
            "右": deque(maxlen=8),
        }
        self.latest_raw: Dict[str, np.ndarray] = {}
        self.latest_result: Optional[Dict[str, Any]] = None
        self.last_pair_ids: Optional[Tuple[int, int]] = None
        self.sdk_initialized = False
        self.depth_thread = DepthThread(self.engine)
        self.depth_thread.result_ready.connect(self.on_depth_result)
        self.depth_thread.problem.connect(self.on_depth_problem)
        self.depth_thread.start()
        self._build_ui()
        self._load_widgets(self.settings)
        self._connect_depth_auto_apply()
        self.setWindowTitle("海康双目深度调优（直接相机，无 ROS2）")
        self.resize(1700, 980)
        self.append_log(
            f"正式标定已加载：左={self.engine.left_serial}，右={self.engine.right_serial}，"
            f"基线={self.engine.baseline_mm:.3f} mm。"
        )
        self.append_log("提示：双目纹理调优时请关闭两路线激光，避免激光线干扰匹配。")

    @staticmethod
    def _double_spin(minimum: float, maximum: float, value: float, decimals: int = 1) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    @staticmethod
    def _int_spin(minimum: int, maximum: int, value: int, step: int = 1) -> QSpinBox:
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        widget.setSingleStep(step)
        widget.setValue(value)
        widget.setKeyboardTracking(False)
        return widget

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("font-size: 15px; font-weight: 600; padding: 6px;")
        root.addWidget(self.status_label)
        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter, 1)

        images_widget = QWidget()
        image_grid = QGridLayout(images_widget)
        self.left_view = ImageView("左目原图（160万）")
        self.right_view = ImageView("右目原图（130万）")
        self.epipolar_view = ImageView("极线对齐：左绿 / 右紫 / 黄线应水平")
        self.depth_view = ImageView("深度伪彩色图")
        image_grid.addWidget(self._titled("左目原图（160万）", self.left_view), 0, 0)
        image_grid.addWidget(self._titled("右目原图（130万）", self.right_view), 0, 1)
        image_grid.addWidget(self._titled("极线对齐检查", self.epipolar_view), 1, 0)
        image_grid.addWidget(self._titled("深度图（近红、远蓝、无效黑）", self.depth_view), 1, 1)
        splitter.addWidget(images_widget)

        controls_scroll = QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setMinimumWidth(390)
        controls = QWidget()
        controls_scroll.setWidget(controls)
        controls_layout = QVBoxLayout(controls)
        splitter.addWidget(controls_scroll)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 0)

        camera_group = QGroupBox("相机参数（可在取流中应用）")
        camera_form = QFormLayout(camera_group)
        self.left_exposure = self._double_spin(50.0, 200000.0, 18000.0, 0)
        self.right_exposure = self._double_spin(50.0, 200000.0, 18000.0, 0)
        self.gain = self._double_spin(0.0, 24.0, 0.0, 2)
        self.fps = self._double_spin(1.0, 60.0, 5.0, 1)
        self.maximum_skew = self._double_spin(1.0, 200.0, 30.0, 1)
        camera_form.addRow("左目曝光 (μs)", self.left_exposure)
        camera_form.addRow("右目曝光 (μs)", self.right_exposure)
        camera_form.addRow("共同增益 (dB)", self.gain)
        camera_form.addRow("帧率 (fps)", self.fps)
        camera_form.addRow("最大配对偏差 (ms)", self.maximum_skew)
        controls_layout.addWidget(camera_group)

        depth_group = QGroupBox("深度与 SGBM 参数")
        depth_form = QFormLayout(depth_group)
        self.processing_width = self._int_spin(160, 1600, 612, 4)
        self.processing_height = self._int_spin(120, 1200, 512, 4)
        self.minimum_depth = self._double_spin(100.0, 10000.0, 350.0, 0)
        self.maximum_depth = self._double_spin(200.0, 20000.0, 3000.0, 0)
        self.block_size = self._int_spin(3, 21, 5, 2)
        self.uniqueness = self._int_spin(0, 50, 8)
        self.speckle_window = self._int_spin(0, 1000, 100, 10)
        self.speckle_range = self._int_spin(0, 32, 2)
        self.lr_difference = self._int_spin(1, 10, 1)
        self.disparity_margin = self._int_spin(0, 128, 16)
        self.maximum_disparities = self._int_spin(16, 1024, 512, 16)
        self.lr_check = QCheckBox("启用左右一致性检查")
        self.clahe = QCheckBox("启用 CLAHE 亮度/局部对比度增强")
        self.clahe_clip = self._double_spin(1.0, 10.0, 2.0, 1)
        depth_form.addRow("处理宽度", self.processing_width)
        depth_form.addRow("处理高度", self.processing_height)
        depth_form.addRow("最近深度 (mm)", self.minimum_depth)
        depth_form.addRow("最远深度 (mm)", self.maximum_depth)
        depth_form.addRow("块大小（奇数）", self.block_size)
        depth_form.addRow("唯一性比例", self.uniqueness)
        depth_form.addRow("散斑窗口", self.speckle_window)
        depth_form.addRow("散斑范围", self.speckle_range)
        depth_form.addRow("左右最大误差 (px)", self.lr_difference)
        depth_form.addRow("视差余量 (px)", self.disparity_margin)
        depth_form.addRow("最大视差数", self.maximum_disparities)
        depth_form.addRow(self.lr_check)
        depth_form.addRow(self.clahe)
        depth_form.addRow("CLAHE 强度", self.clahe_clip)
        controls_layout.addWidget(depth_group)

        buttons = QGridLayout()
        self.start_button = QPushButton("连接并启动")
        self.stop_button = QPushButton("停止并释放相机")
        self.apply_camera_button = QPushButton("应用相机参数")
        self.apply_depth_button = QPushButton("应用深度参数")
        self.save_config_button = QPushButton("保存调优配置")
        self.snapshot_button = QPushButton("保存当前帧")
        buttons.addWidget(self.start_button, 0, 0)
        buttons.addWidget(self.stop_button, 0, 1)
        buttons.addWidget(self.apply_camera_button, 1, 0)
        buttons.addWidget(self.apply_depth_button, 1, 1)
        buttons.addWidget(self.save_config_button, 2, 0)
        buttons.addWidget(self.snapshot_button, 2, 1)
        controls_layout.addLayout(buttons)

        self.statistics = QLabel("等待双目帧…")
        self.statistics.setWordWrap(True)
        self.statistics.setStyleSheet("padding: 8px; background: #22262a;")
        controls_layout.addWidget(self.statistics)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(180)
        controls_layout.addWidget(self.log)
        controls_layout.addStretch(1)

        self.start_button.clicked.connect(self.start_cameras)
        self.stop_button.clicked.connect(self.stop_cameras)
        self.apply_camera_button.clicked.connect(self.apply_camera_settings)
        self.apply_depth_button.clicked.connect(
            lambda: self.apply_depth_settings(automatic=False)
        )
        self.save_config_button.clicked.connect(self.save_config)
        self.snapshot_button.clicked.connect(self.save_snapshot)
        self.stop_button.setEnabled(False)

    @staticmethod
    def _titled(title: str, view: QWidget) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        label = QLabel(title)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-weight: 600; padding: 4px;")
        layout.addWidget(label)
        layout.addWidget(view, 1)
        return widget

    def append_log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.append(f"[{timestamp}] {message}")

    def _load_widgets(self, settings: Dict[str, Any]) -> None:
        camera = settings["camera"]
        depth = settings["depth"]
        self.left_exposure.setValue(float(camera["left_exposure_us"]))
        self.right_exposure.setValue(float(camera["right_exposure_us"]))
        self.gain.setValue(float(camera["gain_db"]))
        self.fps.setValue(float(camera["frames_per_second"]))
        self.maximum_skew.setValue(float(camera["maximum_pair_skew_ms"]))
        self.processing_width.setValue(int(depth["processing_width"]))
        self.processing_height.setValue(int(depth["processing_height"]))
        self.minimum_depth.setValue(float(depth["minimum_depth_mm"]))
        self.maximum_depth.setValue(float(depth["maximum_depth_mm"]))
        self.block_size.setValue(int(depth["block_size"]))
        self.uniqueness.setValue(int(depth["uniqueness_ratio"]))
        self.speckle_window.setValue(int(depth["speckle_window_size"]))
        self.speckle_range.setValue(int(depth["speckle_range"]))
        self.lr_difference.setValue(int(depth["left_right_maximum_difference_px"]))
        self.disparity_margin.setValue(int(depth["disparity_margin_px"]))
        self.maximum_disparities.setValue(int(depth["maximum_num_disparities"]))
        self.lr_check.setChecked(bool(depth["enable_left_right_check"]))
        self.clahe.setChecked(bool(depth["enable_clahe"]))
        self.clahe_clip.setValue(float(depth["clahe_clip_limit"]))

    def _connect_depth_auto_apply(self) -> None:
        self.depth_apply_timer = QTimer(self)
        self.depth_apply_timer.setSingleShot(True)
        self.depth_apply_timer.setInterval(450)
        self.depth_apply_timer.timeout.connect(
            lambda: self.apply_depth_settings(automatic=True)
        )
        for widget in (
            self.processing_width,
            self.processing_height,
            self.minimum_depth,
            self.maximum_depth,
            self.block_size,
            self.uniqueness,
            self.speckle_window,
            self.speckle_range,
            self.lr_difference,
            self.disparity_margin,
            self.maximum_disparities,
            self.clahe_clip,
        ):
            widget.valueChanged.connect(self.schedule_depth_apply)
        self.lr_check.toggled.connect(self.schedule_depth_apply)
        self.clahe.toggled.connect(self.schedule_depth_apply)

    def schedule_depth_apply(self, *_: Any) -> None:
        self.apply_depth_button.setText("应用深度参数（等待自动应用）")
        self.depth_apply_timer.start()

    def camera_config_from_widgets(self) -> Dict[str, Any]:
        return {
            "left_exposure_us": self.left_exposure.value(),
            "right_exposure_us": self.right_exposure.value(),
            "gain_db": self.gain.value(),
            "frames_per_second": self.fps.value(),
            "maximum_pair_skew_ms": self.maximum_skew.value(),
        }

    def depth_config_from_widgets(self) -> Dict[str, Any]:
        block_size = self.block_size.value()
        if block_size % 2 == 0:
            raise RuntimeError("块大小必须是奇数")
        return {
            "processing_width": self.processing_width.value(),
            "processing_height": self.processing_height.value(),
            "minimum_depth_mm": self.minimum_depth.value(),
            "maximum_depth_mm": self.maximum_depth.value(),
            "block_size": block_size,
            "uniqueness_ratio": self.uniqueness.value(),
            "speckle_window_size": self.speckle_window.value(),
            "speckle_range": self.speckle_range.value(),
            "left_right_maximum_difference_px": self.lr_difference.value(),
            "disparity_margin_px": self.disparity_margin.value(),
            "maximum_num_disparities": self.maximum_disparities.value(),
            "enable_left_right_check": self.lr_check.isChecked(),
            "enable_clahe": self.clahe.isChecked(),
            "clahe_clip_limit": self.clahe_clip.value(),
        }

    def start_cameras(self) -> None:
        if self.camera_threads:
            return
        try:
            require_sdk()
            ret = MvCamera.MV_CC_Initialize()  # type: ignore # noqa: F405
            if ret != 0:
                raise sdk_error("初始化MVS SDK", ret)
            self.sdk_initialized = True
            discovered = enumerate_gige_cameras()
            descriptions = "; ".join(
                f"{item.ip} ({item.model}, SN={item.serial})"
                for item in discovered.values()
            )
            self.append_log(f"枚举到相机：{descriptions or '无'}")
            missing = [
                serial
                for serial in (self.engine.left_serial, self.engine.right_serial)
                if serial not in discovered
            ]
            if missing:
                raise RuntimeError(f"未找到标定所需相机：{', '.join(missing)}")
            camera = self.camera_config_from_widgets()
            definitions = {
                "左": (self.engine.left_serial, self.engine.left_size, camera["left_exposure_us"]),
                "右": (self.engine.right_serial, self.engine.right_size, camera["right_exposure_us"]),
            }
            for side, (serial, size, exposure) in definitions.items():
                thread = CameraThread(
                    side,
                    discovered[serial],
                    size,
                    float(exposure),
                    float(camera["gain_db"]),
                    float(camera["frames_per_second"]),
                )
                thread.frame_ready.connect(self.on_camera_frame)
                thread.opened.connect(self.on_camera_opened)
                thread.settings_applied.connect(self.on_camera_settings_applied)
                thread.problem.connect(self.on_camera_problem)
                thread.log_message.connect(self.append_log)
                self.camera_threads[side] = thread
                thread.start()
            self.status_label.setText("正在连接两台相机…")
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        except Exception as exc:
            self.append_log(f"启动失败：{exc}")
            QMessageBox.critical(self, "无法启动双目", str(exc))
            self.stop_cameras()

    def stop_cameras(self) -> None:
        threads = list(self.camera_threads.values())
        for thread in threads:
            thread.request_stop()
        for thread in threads:
            thread.wait(3000)
        self.camera_threads.clear()
        self.frame_queues["左"].clear()
        self.frame_queues["右"].clear()
        if self.sdk_initialized:
            MvCamera.MV_CC_Finalize()  # type: ignore # noqa: F405
            self.sdk_initialized = False
        self.status_label.setText("已停止并释放两台相机")
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def apply_camera_settings(self) -> None:
        camera = self.camera_config_from_widgets()
        if not self.camera_threads:
            self.append_log("相机参数已保留，将在下次连接时应用。")
            return
        self.camera_threads["左"].request_settings(
            camera["left_exposure_us"], camera["gain_db"], camera["frames_per_second"]
        )
        self.camera_threads["右"].request_settings(
            camera["right_exposure_us"], camera["gain_db"], camera["frames_per_second"]
        )
        self.append_log("已请求应用左右独立曝光、共同增益和帧率。")

    def apply_depth_settings(self, automatic: bool = False) -> None:
        try:
            candidate = self.depth_config_from_widgets()
            # Validate with a private engine. The live engine belongs only to
            # DepthThread, avoiding a configure/compute race between threads.
            validator = StereoDepthEngine()
            validator.configure(candidate)
            self.depth_config = candidate
            self.apply_depth_button.setText("应用深度参数（已生效）")
            source = "自动" if automatic else "手动"
            self.append_log(
                f"深度参数已{source}应用：{candidate['minimum_depth_mm']:.0f}–"
                f"{candidate['maximum_depth_mm']:.0f} mm，block={candidate['block_size']}，"
                f"LR={'开' if candidate['enable_left_right_check'] else '关'}，"
                f"CLAHE={'开' if candidate['enable_clahe'] else '关'}。"
            )
        except Exception as exc:
            self.apply_depth_button.setText("应用深度参数（参数无效）")
            if not automatic:
                QMessageBox.warning(self, "深度参数无效", str(exc))
            self.append_log(f"深度参数拒绝：{exc}")

    def on_camera_opened(self, side: str, description: str) -> None:
        self.append_log(f"{side}相机已连接：{description}")
        if len([thread for thread in self.camera_threads.values() if thread.isRunning()]) == 2:
            self.status_label.setText("双目取流中，等待配对和深度计算…")

    def on_camera_settings_applied(
        self, side: str, exposure_us: float, gain_db: float, fps: float
    ) -> None:
        self.append_log(
            f"{side}相机实际参数：曝光={exposure_us:.0f} μs，"
            f"增益={gain_db:.2f} dB，帧率={fps:.2f} fps"
        )

    def on_camera_problem(self, side: str, message: str) -> None:
        self.append_log(f"{side}相机错误：{message}")
        self.status_label.setText(f"{side}相机错误")

    def on_camera_frame(self, side: str, image: np.ndarray, host_ns: int, frame_id: int) -> None:
        self.latest_raw[side] = image
        if side == "左":
            self.left_view.set_array(image)
        else:
            self.right_view.set_array(image)
        self.frame_queues[side].append((host_ns, frame_id, image))
        other_side = "右" if side == "左" else "左"
        if not self.frame_queues[other_side]:
            return
        current = self.frame_queues[side][-1]
        other = min(self.frame_queues[other_side], key=lambda item: abs(item[0] - current[0]))
        skew_ms = abs(current[0] - other[0]) / 1_000_000.0
        if skew_ms > self.maximum_skew.value():
            self.statistics.setText(
                f"等待同步帧：最近到达时间偏差 {skew_ms:.2f} ms，"
                f"门槛 {self.maximum_skew.value():.2f} ms"
            )
            return
        left = current if side == "左" else other
        right = current if side == "右" else other
        ids = (left[1], right[1])
        if ids == self.last_pair_ids:
            return
        self.last_pair_ids = ids
        self.frame_queues["左"].clear()
        self.frame_queues["右"].clear()
        self.depth_thread.submit(left[2], right[2], self.depth_config, skew_ms)

    def on_depth_result(self, result: Dict[str, Any]) -> None:
        self.latest_result = result
        self.epipolar_view.set_array(result["epipolar"])
        self.depth_view.set_array(result["depth_preview"])
        valid_percent = result["valid_fraction"] * 100.0
        self.statistics.setText(
            f"配对偏差：{result['pair_skew_ms']:.2f} ms\n"
            f"有效深度：{valid_percent:.2f}%\n"
            f"中值深度：{result['median_depth_mm']:.1f} mm\n"
            f"视差范围：{result['minimum_disparity']} + {result['number_disparities']}\n"
            f"处理耗时：{result['processing_ms']:.1f} ms\n"
            f"实际参数：{self.depth_config['minimum_depth_mm']:.0f}–"
            f"{self.depth_config['maximum_depth_mm']:.0f} mm，"
            f"block={self.depth_config['block_size']}，"
            f"LR={'开' if self.depth_config['enable_left_right_check'] else '关'}，"
            f"CLAHE={'开' if self.depth_config['enable_clahe'] else '关'}"
        )
        self.status_label.setText(
            f"双目深度运行中｜有效 {valid_percent:.2f}%｜"
            f"中值 {result['median_depth_mm']:.0f} mm"
        )

    def on_depth_problem(self, message: str) -> None:
        self.append_log(f"深度计算失败：{message}")
        self.status_label.setText("深度计算失败，请检查参数")

    def save_config(self) -> None:
        try:
            settings = {
                "schema_version": 1,
                "camera": self.camera_config_from_widgets(),
                "depth": self.depth_config_from_widgets(),
                "multi_band": self.settings.get(
                    "multi_band", default_settings()["multi_band"]
                ),
            }
            path_text, _ = QFileDialog.getSaveFileName(
                self,
                "保存双目调优配置",
                str(self.config_path),
                "YAML (*.yaml *.yml)",
            )
            if not path_text:
                return
            path = Path(path_text)
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(settings, handle, allow_unicode=True, sort_keys=False)
            self.config_path = path
            self.append_log(f"调优配置已保存：{path}")
        except Exception as exc:
            QMessageBox.critical(self, "保存失败", str(exc))

    def save_snapshot(self) -> None:
        if "左" not in self.latest_raw or "右" not in self.latest_raw or self.latest_result is None:
            QMessageBox.information(self, "没有图像", "请先获得一组有效双目深度结果。")
            return
        directory_text = QFileDialog.getExistingDirectory(
            self, "选择快照保存目录", str(PROJECT_ROOT / "data" / "stereo_tuner")
        )
        if not directory_text:
            return
        output = Path(directory_text) / datetime.now().strftime("snapshot_%Y%m%d_%H%M%S_%f")
        output.mkdir(parents=True, exist_ok=False)
        cv2.imwrite(str(output / "left_raw.png"), self.latest_raw["左"])
        cv2.imwrite(str(output / "right_raw.png"), self.latest_raw["右"])
        cv2.imwrite(str(output / "epipolar.png"), self.latest_result["epipolar"])
        cv2.imwrite(str(output / "depth_preview.png"), self.latest_result["depth_preview"])
        np.save(output / "depth_mm.npy", self.latest_result["depth_mm"])
        np.save(output / "valid_mask.npy", self.latest_result["valid_mask"])
        self.append_log(f"当前帧和毫米深度矩阵已保存：{output}")

    def closeEvent(self, event: Any) -> None:
        self.stop_cameras()
        self.depth_thread.request_stop()
        self.depth_thread.wait(3000)
        event.accept()


def run_self_test() -> int:
    engine = StereoDepthEngine()
    settings = default_settings()["depth"]
    engine.configure(settings)
    geometry = engine._geometry
    print(
        "self-test passed: "
        f"left={engine.left_serial} {engine.left_size}, "
        f"right={engine.right_serial} {engine.right_size}, "
        f"baseline={engine.baseline_mm:.3f} mm, "
        f"min_disparity={geometry['minimum_disparity']}, "
        f"num_disparities={geometry['number_disparities']}"
    )
    return 0


def list_cameras() -> int:
    require_sdk()
    ret = MvCamera.MV_CC_Initialize()  # type: ignore # noqa: F405
    if ret != 0:
        raise sdk_error("初始化MVS SDK", ret)
    try:
        for item in enumerate_gige_cameras().values():
            print(f"{item.ip}\t{item.model}\t{item.serial}")
    finally:
        MvCamera.MV_CC_Finalize()  # type: ignore # noqa: F405
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--list-cameras", action="store_true")
    args = parser.parse_args()
    try:
        if args.self_test:
            return run_self_test()
        if args.list_cameras:
            return list_cameras()
        application = QApplication(sys.argv[:1])
        application.setStyle("Fusion")
        window = StereoDepthTunerWindow(args.config.resolve())
        window.show()
        return application.exec_()
    except Exception as exc:
        print(f"stereo depth tuner startup failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
