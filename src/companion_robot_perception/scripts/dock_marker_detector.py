#!/usr/bin/env python3

"""Detect the docking-station ArUco marker in the rear camera stream."""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def rotation_matrix_to_quaternion(matrix):
    """Convert a 3x3 rotation matrix into an x, y, z, w quaternion."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
        ) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
        ) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
        ) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return x, y, z, w


class DockMarkerDetector(Node):
    def __init__(self):
        super().__init__("dock_marker_detector")

        self.declare_parameter("image_topic", "/rear_camera/image_raw")
        self.declare_parameter(
            "camera_info_topic", "/rear_camera/camera_info"
        )
        self.declare_parameter(
            "optical_frame", "rear_camera_optical_frame"
        )
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.12)
        self.declare_parameter("detection_timeout", 0.60)
        self.declare_parameter("debug_image_enabled", True)

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.optical_frame = str(
            self.get_parameter("optical_frame").value
        )
        dictionary_name = str(self.get_parameter("dictionary").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = max(
            0.01, float(self.get_parameter("marker_size").value)
        )
        self.detection_timeout = max(
            0.1, float(self.get_parameter("detection_timeout").value)
        )
        self.debug_image_enabled = bool(
            self.get_parameter("debug_image_enabled").value
        )

        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}")
        dictionary_code = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_code)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_parameters = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.detector_parameters
            )
        else:
            self.detector = None

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_publisher = self.create_publisher(
            PoseStamped, "/dock_marker/pose", 10
        )
        self.visible_publisher = self.create_publisher(
            Bool, "/dock_marker/visible", 10
        )
        self.confidence_publisher = self.create_publisher(
            Float32, "/dock_marker/confidence", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/dock_marker/status", latched_qos
        )
        self.visualization_publisher = self.create_publisher(
            Marker, "/dock_marker/visualization", 10
        )
        self.debug_publisher = self.create_publisher(
            Image, "/dock_marker/debug_image", qos_profile_sensor_data
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.status = "UNKNOWN"
        self.ever_detected = False
        self.last_detection_at = None
        self.last_log_at = 0.0
        self._set_status("WAITING_FOR_CAMERA")
        self.get_logger().info(
            f"Waiting for ArUco marker {self.marker_id} on {image_topic}."
        )

    def _camera_info_callback(self, camera_info):
        matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self.camera_matrix = matrix
        self.distortion = np.asarray(camera_info.d, dtype=np.float64)
        if self.status == "WAITING_FOR_CAMERA":
            self._set_status("SEARCHING")

    def _detect_markers(self, gray_image):
        if self.detector is not None:
            return self.detector.detectMarkers(gray_image)
        return cv2.aruco.detectMarkers(
            gray_image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

    def _image_callback(self, image_message):
        if self.camera_matrix is None:
            self._publish_visibility(False, 0.0)
            return

        try:
            image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        except Exception as error:
            self.get_logger().error(
                f"Could not convert rear-camera image: {error}",
                throttle_duration_sec=2.0,
            )
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, identifiers, _ = self._detect_markers(gray)
        target_index = None
        if identifiers is not None:
            flat_identifiers = identifiers.flatten().tolist()
            if self.marker_id in flat_identifiers:
                target_index = flat_identifiers.index(self.marker_id)

        if target_index is None:
            self._handle_missing_marker()
            self._publish_visibility(False, 0.0)
            self._publish_debug(image_message, image, corners, identifiers)
            return

        target_corners = corners[target_index]
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [target_corners],
            self.marker_size,
            self.camera_matrix,
            self.distortion,
        )
        rotation_vector = rvecs[0][0]
        translation = tvecs[0][0]
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)

        pose = PoseStamped()
        pose.header.stamp = image_message.header.stamp
        pose.header.frame_id = self.optical_frame
        pose.pose.position.x = float(translation[0])
        pose.pose.position.y = float(translation[1])
        pose.pose.position.z = float(translation[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.pose_publisher.publish(pose)

        polygon = target_corners.reshape(-1, 2).astype(np.float32)
        area = abs(float(cv2.contourArea(polygon)))
        image_area = float(max(1, image.shape[0] * image.shape[1]))
        confidence = clamp(area / (image_area * 0.04), 0.0, 1.0)
        self._publish_visibility(True, confidence)
        self._publish_visualization(pose)
        self.last_detection_at = time.monotonic()
        self.ever_detected = True
        self._set_status("DETECTED")

        now = time.monotonic()
        if now - self.last_log_at >= 1.0:
            self.last_log_at = now
            self.get_logger().info(
                "Dock marker: "
                f"forward={translation[2]:.3f} m, "
                f"right={translation[0]:.3f} m, "
                f"confidence={confidence:.2f}"
            )

        cv2.aruco.drawDetectedMarkers(image, corners, identifiers)
        cv2.drawFrameAxes(
            image,
            self.camera_matrix,
            self.distortion,
            rotation_vector,
            translation,
            self.marker_size * 0.5,
        )
        self._publish_debug(image_message, image, [], None)

    def _handle_missing_marker(self):
        now = time.monotonic()
        if not self.ever_detected or self.last_detection_at is None:
            self._set_status("SEARCHING")
            self._delete_visualization()
            return

        if now - self.last_detection_at <= self.detection_timeout:
            self._set_status("OCCLUDED")
            return

        self._set_status("LOST")
        self._delete_visualization()

    def _publish_visibility(self, visible, confidence):
        visible_message = Bool()
        visible_message.data = visible
        self.visible_publisher.publish(visible_message)
        confidence_message = Float32()
        confidence_message.data = float(confidence)
        self.confidence_publisher.publish(confidence_message)

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"Dock-marker status: {status}")

    def _publish_visualization(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "dock_marker"
        marker.id = self.marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.01
        marker.color.r = 0.10
        marker.color.g = 1.00
        marker.color.b = 0.25
        marker.color.a = 0.65
        marker.lifetime.nanosec = 300_000_000
        self.visualization_publisher.publish(marker)

    def _delete_visualization(self):
        marker = Marker()
        marker.header.frame_id = self.optical_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dock_marker"
        marker.id = self.marker_id
        marker.action = Marker.DELETE
        self.visualization_publisher.publish(marker)

    def _publish_debug(self, source_message, image, corners, identifiers):
        if not self.debug_image_enabled:
            return
        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(image, corners, identifiers)
        cv2.putText(
            image,
            self.status,
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        debug_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_message.header = source_message.header
        self.debug_publisher.publish(debug_message)


def main(args=None):
    rclpy.init(args=args)
    node = DockMarkerDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
