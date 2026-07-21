#!/usr/bin/env python3
"""Publish calibrated weld-gun TCP and Vizum left-camera frames and RViz markers."""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from fr5_vizum_bringup.calibration_frames import (
    CalibrationConfigError,
    RigidTransform,
    load_camera_calibration,
    load_tool_calibration,
)


class CalibratedFramesPublisher(Node):
    def __init__(self) -> None:
        super().__init__("calibrated_frames_publisher")

        tool_config_path = self.declare_parameter("tool_config_path", "").value
        handeye_config_path = self.declare_parameter("handeye_config_path", "").value
        self._flange_frame = self.declare_parameter(
            "flange_frame", "fairino_flange_reported"
        ).value
        self._tcp_frame = self.declare_parameter("tcp_frame", "weld_gun_tcp").value
        self._camera_frame = self.declare_parameter(
            "camera_frame", "vizum_left_camera_optical_frame"
        ).value
        marker_topic = self.declare_parameter(
            "marker_topic", "fairino/calibrated_points"
        ).value
        publish_markers = self.declare_parameter("publish_markers", True).value

        self._validate_names(
            tool_config_path,
            handeye_config_path,
            marker_topic,
            publish_markers,
        )
        tool = load_tool_calibration(tool_config_path)
        camera = load_camera_calibration(handeye_config_path)

        stamp = self.get_clock().now().to_msg()
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._static_broadcaster.sendTransform([
            self._make_transform(
                stamp, self._flange_frame, self._tcp_frame, tool.transform
            ),
            self._make_transform(
                stamp, self._flange_frame, self._camera_frame, camera.transform
            ),
        ])

        self._marker_publisher = None
        if publish_markers:
            marker_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._marker_publisher = self.create_publisher(
                MarkerArray, marker_topic, marker_qos
            )
            self._marker_publisher.publish(
                self._make_markers(stamp, tool.tool_id)
            )

        self._log_transform(
            f"weld-gun TCP (config tool_id={tool.tool_id})",
            tool_config_path,
            self._flange_frame,
            self._tcp_frame,
            tool.transform,
        )
        self._log_transform(
            f"Vizum left camera ({camera.input_mode})",
            handeye_config_path,
            self._flange_frame,
            self._camera_frame,
            camera.transform,
        )
        self.get_logger().info(
            "Calibration display node is read-only and does not open a Fairino SDK connection."
        )

    @staticmethod
    def _make_transform(
        stamp, parent: str, child: str, transform: RigidTransform
    ) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        (
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
        ) = transform.translation_m
        (
            message.transform.rotation.x,
            message.transform.rotation.y,
            message.transform.rotation.z,
            message.transform.rotation.w,
        ) = transform.quaternion_xyzw
        return message

    def _make_markers(self, stamp, tool_id: int) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.extend([
            self._sphere_marker(
                stamp, self._tcp_frame, 0, 0.038, (1.0, 0.24, 0.05, 1.0)
            ),
            self._text_marker(
                stamp,
                self._tcp_frame,
                1,
                f"Weld-gun TCP (config tool {tool_id})",
                (1.0, 0.55, 0.12, 1.0),
            ),
            self._sphere_marker(
                stamp, self._camera_frame, 10, 0.038, (0.0, 0.72, 1.0, 1.0)
            ),
            self._text_marker(
                stamp,
                self._camera_frame,
                11,
                "Vizum left-camera origin",
                (0.2, 0.82, 1.0, 1.0),
            ),
        ])
        return markers

    @staticmethod
    def _base_marker(stamp, frame: str, marker_id: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.lifetime = Duration(seconds=0.0).to_msg()
        return marker

    @classmethod
    def _sphere_marker(
        cls, stamp, frame: str, marker_id: int, diameter: float, color
    ) -> Marker:
        marker = cls._base_marker(stamp, frame, marker_id, "calibrated_points")
        marker.type = Marker.SPHERE
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    @classmethod
    def _text_marker(
        cls, stamp, frame: str, marker_id: int, text: str, color
    ) -> Marker:
        marker = cls._base_marker(stamp, frame, marker_id, "calibrated_point_labels")
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 0.055
        marker.scale.z = 0.027
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.text = text
        return marker

    def _validate_names(
        self,
        tool_config_path,
        handeye_config_path,
        marker_topic,
        publish_markers,
    ) -> None:
        string_values = {
            "tool_config_path": tool_config_path,
            "handeye_config_path": handeye_config_path,
            "flange_frame": self._flange_frame,
            "tcp_frame": self._tcp_frame,
            "camera_frame": self._camera_frame,
        }
        for name, value in string_values.items():
            if not isinstance(value, str) or not value.strip():
                raise CalibrationConfigError(f"parameter '{name}' must not be empty")
        if len({self._flange_frame, self._tcp_frame, self._camera_frame}) != 3:
            raise CalibrationConfigError(
                "flange_frame, tcp_frame and camera_frame must be distinct"
            )
        if publish_markers and (not isinstance(marker_topic, str) or not marker_topic.strip()):
            raise CalibrationConfigError(
                "parameter 'marker_topic' must not be empty when markers are enabled"
            )

    def _log_transform(
        self,
        label: str,
        source_path: str,
        parent: str,
        child: str,
        transform: RigidTransform,
    ) -> None:
        x, y, z = transform.translation_m
        qx, qy, qz, qw = transform.quaternion_xyzw
        self.get_logger().info(
            f"Loaded {label} from {source_path}: {parent} -> {child}, "
            f"xyz_m=[{x:.9f}, {y:.9f}, {z:.9f}], "
            f"q_xyzw=[{qx:.9f}, {qy:.9f}, {qz:.9f}, {qw:.9f}]"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = CalibratedFramesPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # The launch must fail visibly on invalid calibration.
        rclpy.logging.get_logger("calibrated_frames_publisher").fatal(str(exc))
        exit_code = 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
