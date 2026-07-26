#!/usr/bin/env python3

"""Detect the simulated person with RGB and associate LiDAR range returns."""

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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.declare_parameter("image_topic", "/front_camera/image_raw")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_frame", "base_footprint")
        self.declare_parameter("horizontal_fov", 1.3962634)
        self.declare_parameter("hsv_lower", [120, 70, 45])
        self.declare_parameter("hsv_upper", [170, 255, 255])
        self.declare_parameter("minimum_blob_area", 120.0)
        self.declare_parameter("minimum_blob_height", 12)
        self.declare_parameter("morphology_kernel", 5)
        self.declare_parameter("scan_timeout", 0.75)
        self.declare_parameter("detection_timeout", 1.0)
        self.declare_parameter("camera_timeout", 2.0)
        self.declare_parameter("scan_window_min", 0.10)
        self.declare_parameter("range_percentile", 25.0)
        self.declare_parameter("minimum_range_samples", 2)
        self.declare_parameter("range_cluster_tolerance", 0.25)
        self.declare_parameter("maximum_range_jump", 0.45)
        self.declare_parameter("maximum_range_rate", 1.0)
        self.declare_parameter("minimum_tracking_range", 0.30)
        self.declare_parameter("maximum_tracking_range", 2.0)
        self.declare_parameter("filter_alpha", 0.45)
        self.declare_parameter("debug_image_enabled", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.horizontal_fov = clamp(
            float(self.get_parameter("horizontal_fov").value), 0.2, math.pi
        )
        self.hsv_lower = np.asarray(
            self.get_parameter("hsv_lower").value, dtype=np.uint8
        )
        self.hsv_upper = np.asarray(
            self.get_parameter("hsv_upper").value, dtype=np.uint8
        )
        if self.hsv_lower.shape != (3,) or self.hsv_upper.shape != (3,):
            raise ValueError("HSV limits must each contain three values.")
        self.minimum_blob_area = max(
            10.0, float(self.get_parameter("minimum_blob_area").value)
        )
        self.minimum_blob_height = max(
            2, int(self.get_parameter("minimum_blob_height").value)
        )
        kernel_size = max(
            1, int(self.get_parameter("morphology_kernel").value)
        )
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        self.scan_timeout = max(
            0.1, float(self.get_parameter("scan_timeout").value)
        )
        self.detection_timeout = max(
            0.2, float(self.get_parameter("detection_timeout").value)
        )
        self.camera_timeout = max(
            self.detection_timeout,
            float(self.get_parameter("camera_timeout").value),
        )
        self.scan_window_min = max(
            0.01, float(self.get_parameter("scan_window_min").value)
        )
        self.range_percentile = clamp(
            float(self.get_parameter("range_percentile").value), 0.0, 100.0
        )
        self.minimum_range_samples = max(
            1, int(self.get_parameter("minimum_range_samples").value)
        )
        self.range_cluster_tolerance = max(
            0.05,
            float(self.get_parameter("range_cluster_tolerance").value),
        )
        self.maximum_range_jump = max(
            0.10, float(self.get_parameter("maximum_range_jump").value)
        )
        self.maximum_range_rate = max(
            0.20, float(self.get_parameter("maximum_range_rate").value)
        )
        self.minimum_tracking_range = max(
            0.0, float(self.get_parameter("minimum_tracking_range").value)
        )
        self.maximum_tracking_range = max(
            self.minimum_tracking_range + 0.10,
            float(self.get_parameter("maximum_tracking_range").value),
        )
        self.filter_alpha = clamp(
            float(self.get_parameter("filter_alpha").value), 0.05, 1.0
        )
        self.debug_image_enabled = bool(
            self.get_parameter("debug_image_enabled").value
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pose_publisher = self.create_publisher(
            PoseStamped, "/person_detection/pose", 10
        )
        self.visible_publisher = self.create_publisher(
            Bool, "/person_detection/visible", 10
        )
        self.bearing_publisher = self.create_publisher(
            Float32, "/person_detection/bearing", 10
        )
        self.distance_publisher = self.create_publisher(
            Float32, "/person_detection/distance", 10
        )
        self.confidence_publisher = self.create_publisher(
            Float32, "/person_detection/confidence", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/person_detection/status", latched_qos
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/person_detection/visualization", 10
        )
        self.debug_publisher = self.create_publisher(
            Image, "/person_detection/debug_image", 2
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.bridge = CvBridge()
        self.scan = None
        self.scan_received_at = None
        self.last_image_at = None
        self.last_visual_at = None
        self.last_detection_at = None
        self.filtered_bearing = None
        self.filtered_distance = None
        self.status = "UNKNOWN"
        self.timer = self.create_timer(0.2, self._check_freshness)

        self._set_status("WAITING_FOR_CAMERA")
        self.get_logger().info(
            f"Person detector waiting for {self.image_topic} and "
            f"LiDAR on {self.scan_topic}."
        )

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"Person-detection status: {status}")

    def _scan_callback(self, scan):
        self.scan = scan
        self.scan_received_at = time.monotonic()

    def _image_callback(self, image_message):
        now = time.monotonic()
        self.last_image_at = now
        try:
            image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        except Exception as error:
            self.get_logger().error(f"Failed to decode front-camera image: {error}")
            self._set_status("ERROR")
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self.morphology_kernel
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self.morphology_kernel, iterations=2
        )
        contours, _hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, width, height = cv2.boundingRect(contour)
            if area < self.minimum_blob_area or height < self.minimum_blob_height:
                continue
            candidates.append((area, contour, (x, y, width, height)))

        if not candidates:
            self._publish_debug(image, None, None, None, "SEARCHING")
            return

        area, contour, bounds = max(candidates, key=lambda item: item[0])
        x, y, width, height = bounds
        moments = cv2.moments(contour)
        if abs(moments["m00"]) > 1e-6:
            center_x = float(moments["m10"] / moments["m00"])
            center_y = float(moments["m01"] / moments["m00"])
        else:
            center_x = x + width / 2.0
            center_y = y + height / 2.0

        self.last_visual_at = now
        focal_x = image.shape[1] / (2.0 * math.tan(self.horizontal_fov / 2.0))
        bearing = math.atan2(image.shape[1] / 2.0 - center_x, focal_x)
        half_window = max(
            self.scan_window_min,
            math.atan2(max(1.0, width / 2.0), focal_x),
        )
        distance = self._range_at_bearing(bearing, half_window, now)
        confidence = clamp(
            area / max(1.0, image.shape[0] * image.shape[1] * 0.06),
            0.0,
            1.0,
        )

        if distance is None:
            self._publish_visible(False)
            self._publish_float(self.confidence_publisher, confidence)
            self._set_status("VISION_ONLY")
            self._publish_debug(
                image, bounds, center_x, center_y, "VISION ONLY"
            )
            return

        if self.filtered_bearing is None or self.last_detection_at is None:
            self.filtered_bearing = bearing
            self.filtered_distance = distance
        else:
            alpha = self.filter_alpha
            self.filtered_bearing += alpha * angle_difference(
                bearing, self.filtered_bearing
            )
            self.filtered_distance += alpha * (
                distance - self.filtered_distance
            )
        self.last_detection_at = now

        self._publish_detection(image_message, confidence)
        self._set_status("TRACKING")
        self._publish_debug(
            image,
            bounds,
            center_x,
            center_y,
            f"TRACK {self.filtered_distance:.2f} m",
        )

    def _range_at_bearing(self, bearing, half_window, now):
        if (
            self.scan is None
            or self.scan_received_at is None
            or now - self.scan_received_at > self.scan_timeout
            or abs(self.scan.angle_increment) < 1e-9
        ):
            return None

        valid_samples = []
        for index, raw_range in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + index * self.scan.angle_increment
            angular_offset = angle_difference(angle, bearing)
            if abs(angular_offset) > half_window:
                continue
            if not math.isfinite(raw_range):
                continue
            minimum_range = max(
                float(self.scan.range_min), self.minimum_tracking_range
            )
            if raw_range < minimum_range or raw_range > self.scan.range_max:
                continue
            if raw_range > self.maximum_tracking_range:
                continue
            valid_samples.append(
                (index, float(raw_range), abs(angular_offset))
            )

        if len(valid_samples) < self.minimum_range_samples:
            return None

        # A wide visual bounding box can cover both the person and a wall or
        # crate behind them. Split the returns into contiguous depth clusters
        # so a background surface cannot pull the estimated person range away.
        clusters = []
        current_cluster = []
        for sample in valid_samples:
            if current_cluster:
                previous = current_cluster[-1]
                contiguous = sample[0] == previous[0] + 1
                same_surface = (
                    abs(sample[1] - previous[1])
                    <= self.range_cluster_tolerance
                )
                if not contiguous or not same_surface:
                    if len(current_cluster) >= self.minimum_range_samples:
                        clusters.append(current_cluster)
                    current_cluster = []
            current_cluster.append(sample)
        if len(current_cluster) >= self.minimum_range_samples:
            clusters.append(current_cluster)
        if not clusters:
            return None

        cluster_ranges = [
            float(
                np.percentile(
                    [sample[1] for sample in cluster],
                    self.range_percentile,
                )
            )
            for cluster in clusters
        ]
        cluster_offsets = [
            min(sample[2] for sample in cluster) for cluster in clusters
        ]

        if (
            self.filtered_distance is not None
            and self.last_detection_at is not None
            and now - self.last_detection_at <= self.detection_timeout
        ):
            elapsed = max(0.0, now - self.last_detection_at)
            allowed_jump = min(
                self.maximum_range_jump,
                max(0.12, self.maximum_range_rate * elapsed),
            )
            closest_index = min(
                range(len(clusters)),
                key=lambda index: (
                    abs(cluster_ranges[index] - self.filtered_distance),
                    cluster_offsets[index],
                ),
            )
            if (
                abs(
                    cluster_ranges[closest_index]
                    - self.filtered_distance
                )
                <= allowed_jump
            ):
                return cluster_ranges[closest_index]
            # Keep the last good target instead of allowing a wall return to
            # drag the range outward a little on every camera frame.
            return None

        # On a fresh acquisition, select the depth cluster best aligned with
        # the centre of the purple visual region. The person masks the wall
        # directly behind them, while unrelated foreground objects usually
        # sit near one side of the camera box.
        nearest_index = min(
            range(len(clusters)),
            key=lambda index: (
                cluster_offsets[index],
                cluster_ranges[index],
            ),
        )
        return cluster_ranges[nearest_index]

    def _publish_detection(self, image_message, confidence):
        bearing = float(self.filtered_bearing)
        distance = float(self.filtered_distance)
        pose = PoseStamped()
        pose.header.stamp = image_message.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x = distance * math.cos(bearing)
        pose.pose.position.y = distance * math.sin(bearing)
        pose.pose.orientation.w = 1.0
        self.pose_publisher.publish(pose)
        self._publish_visible(True)
        self._publish_float(self.bearing_publisher, bearing)
        self._publish_float(self.distance_publisher, distance)
        self._publish_float(self.confidence_publisher, confidence)

        marker = Marker()
        marker.header = pose.header
        marker.ns = "detected_person"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = 0.45
        marker.scale.x = 0.36
        marker.scale.y = 0.36
        marker.scale.z = 0.90
        marker.color.r = 0.10
        marker.color.g = 1.0
        marker.color.b = 0.30
        marker.color.a = 0.65
        marker.lifetime.sec = 1
        self.marker_publisher.publish(marker)

    def _publish_visible(self, visible):
        message = Bool()
        message.data = bool(visible)
        self.visible_publisher.publish(message)

    @staticmethod
    def _publish_float(publisher, value):
        message = Float32()
        message.data = float(value)
        publisher.publish(message)

    def _publish_debug(self, image, bounds, center_x, center_y, label):
        if not self.debug_image_enabled:
            return
        output = image.copy()
        if bounds is not None:
            x, y, width, height = bounds
            cv2.rectangle(
                output, (x, y), (x + width, y + height), (0, 255, 0), 2
            )
        if center_x is not None and center_y is not None:
            cv2.circle(
                output, (int(center_x), int(center_y)), 4, (0, 255, 255), -1
            )
        cv2.putText(
            output,
            label,
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0) if label.startswith("TRACK") else (0, 190, 255),
            2,
            cv2.LINE_AA,
        )
        debug_message = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
        debug_message.header.stamp = self.get_clock().now().to_msg()
        debug_message.header.frame_id = "front_camera_optical_frame"
        self.debug_publisher.publish(debug_message)

    def _check_freshness(self):
        now = time.monotonic()
        if self.last_image_at is None:
            self._set_status("WAITING_FOR_CAMERA")
            self._publish_visible(False)
            return
        if now - self.last_image_at > self.camera_timeout:
            self._set_status("CAMERA_STALE")
            self._publish_visible(False)
            return
        if (
            self.last_detection_at is None
            or now - self.last_detection_at > self.detection_timeout
        ):
            self.filtered_bearing = None
            self.filtered_distance = None
            self._publish_visible(False)
            self._set_status(
                "SEARCHING"
                if self.last_visual_at is None
                or now - self.last_visual_at > self.detection_timeout
                else "VISION_ONLY"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
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
