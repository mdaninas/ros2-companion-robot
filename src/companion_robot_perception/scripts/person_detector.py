#!/usr/bin/env python3

"""Track selectable simulated people using RGB identity colours and LiDAR."""

import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


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
        self.declare_parameter(
            "identity_names", ["person_alpha", "person_beta"]
        )
        self.declare_parameter(
            "identity_hsv_lower",
            [120, 70, 45, 95, 100, 45],
        )
        self.declare_parameter(
            "identity_hsv_upper",
            [170, 255, 255, 119, 255, 255],
        )
        self.declare_parameter(
            "identity_marker_rgb",
            [0.72, 0.10, 0.96, 0.05, 0.35, 1.00],
        )
        self.declare_parameter("selected_identity", "person_alpha")
        self.declare_parameter("minimum_blob_area", 120.0)
        self.declare_parameter("minimum_blob_height", 12)
        self.declare_parameter("minimum_blob_aspect_ratio", 0.65)
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
        self.minimum_blob_area = max(
            10.0, float(self.get_parameter("minimum_blob_area").value)
        )
        self.minimum_blob_height = max(
            2, int(self.get_parameter("minimum_blob_height").value)
        )
        self.minimum_blob_aspect_ratio = max(
            0.1,
            float(
                self.get_parameter("minimum_blob_aspect_ratio").value
            ),
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

        self.identities = self._load_identities()
        requested_identity = str(
            self.get_parameter("selected_identity").value
        )
        if requested_identity not in self.identities:
            raise ValueError(
                f"selected_identity '{requested_identity}' is not in "
                f"identity_names {list(self.identities)}"
            )
        self.selected_identity = requested_identity

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
        self.selected_identity_publisher = self.create_publisher(
            String, "/person_detection/selected_identity", latched_qos
        )
        self.tracks_publisher = self.create_publisher(
            String, "/person_detection/tracks", latched_qos
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/person_detection/visualization", 10
        )
        self.tracks_marker_publisher = self.create_publisher(
            MarkerArray,
            "/person_detection/tracks_visualization",
            latched_qos,
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
        self.status_service = self.create_service(
            Trigger, "/get_person_targets", self._handle_targets
        )
        self.cycle_service = self.create_service(
            Trigger, "/cycle_person_target", self._handle_cycle_target
        )
        self.add_on_set_parameters_callback(self._parameter_callback)

        self.bridge = CvBridge()
        self.scan = None
        self.scan_received_at = None
        self.last_image_at = None
        self.status = "UNKNOWN"
        self.timer = self.create_timer(0.2, self._check_freshness)

        self._publish_selected_identity()
        self._set_status(f"WAITING_FOR_CAMERA:{self.selected_identity}")
        self.get_logger().info(
            "Multi-person detector tracking %s; selected target is %s."
            % (", ".join(self.identities), self.selected_identity)
        )

    def _load_identities(self):
        names = [
            str(name)
            for name in self.get_parameter("identity_names").value
        ]
        if not names or len(set(names)) != len(names):
            raise ValueError(
                "identity_names must contain unique non-empty names"
            )
        lower = [
            int(value)
            for value in self.get_parameter("identity_hsv_lower").value
        ]
        upper = [
            int(value)
            for value in self.get_parameter("identity_hsv_upper").value
        ]
        marker_rgb = [
            float(value)
            for value in self.get_parameter("identity_marker_rgb").value
        ]
        expected = 3 * len(names)
        if len(lower) != expected or len(upper) != expected:
            raise ValueError(
                "identity_hsv_lower/upper need three values per identity"
            )
        if len(marker_rgb) != expected:
            raise ValueError(
                "identity_marker_rgb needs three values per identity"
            )

        identities = {}
        for index, name in enumerate(names):
            offset = index * 3
            identities[name] = {
                "index": index,
                "lower": np.asarray(
                    lower[offset : offset + 3], dtype=np.uint8
                ),
                "upper": np.asarray(
                    upper[offset : offset + 3], dtype=np.uint8
                ),
                "color": tuple(
                    clamp(value, 0.0, 1.0)
                    for value in marker_rgb[offset : offset + 3]
                ),
                "filtered_bearing": None,
                "filtered_distance": None,
                "last_detection_at": None,
                "last_visual_at": None,
                "last_confidence": 0.0,
                "last_pose": None,
            }
        return identities

    def _parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name != "selected_identity":
                continue
            requested = str(parameter.value)
            if requested not in self.identities:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"Unknown identity '{requested}'. Choose one of "
                        f"{list(self.identities)}."
                    ),
                )
            self._select_identity(requested)
        return SetParametersResult(successful=True)

    def _select_identity(self, identity):
        if identity == self.selected_identity:
            return
        previous = self.selected_identity
        self.selected_identity = identity
        self._publish_visible(False)
        self._publish_selected_identity()
        self._set_status(f"TARGET_SELECTED:{identity}")
        self.get_logger().info(
            f"Person target changed from {previous} to {identity}."
        )

    def _handle_cycle_target(self, _request, response):
        names = list(self.identities)
        next_index = (names.index(self.selected_identity) + 1) % len(names)
        requested = names[next_index]
        results = self.set_parameters(
            [
                Parameter(
                    "selected_identity",
                    Parameter.Type.STRING,
                    requested,
                )
            ]
        )
        response.success = bool(results and results[0].successful)
        response.message = (
            f"Selected person target: {requested}"
            if response.success
            else results[0].reason
        )
        return response

    def _handle_targets(self, _request, response):
        response.success = True
        response.message = self._tracks_json(time.monotonic())
        return response

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"Person-detection status: {status}")

    def _publish_selected_identity(self):
        message = String()
        message.data = self.selected_identity
        self.selected_identity_publisher.publish(message)

    def _scan_callback(self, scan):
        self.scan = scan
        self.scan_received_at = time.monotonic()

    def _image_callback(self, image_message):
        now = time.monotonic()
        self.last_image_at = now
        try:
            image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        except Exception as error:
            self.get_logger().error(
                f"Failed to decode front-camera image: {error}"
            )
            self._set_status("ERROR")
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        focal_x = image.shape[1] / (
            2.0 * math.tan(self.horizontal_fov / 2.0)
        )
        annotations = []
        selected_detected = False
        selected_visual = False

        for identity, track in self.identities.items():
            candidate = self._find_candidate(hsv, track)
            if candidate is None:
                continue
            area, contour, bounds = candidate
            x, y, width, height = bounds
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                center_x = float(moments["m10"] / moments["m00"])
                center_y = float(moments["m01"] / moments["m00"])
            else:
                center_x = x + width / 2.0
                center_y = y + height / 2.0

            track["last_visual_at"] = now
            bearing = math.atan2(
                image.shape[1] / 2.0 - center_x, focal_x
            )
            half_window = max(
                self.scan_window_min,
                math.atan2(max(1.0, width / 2.0), focal_x),
            )
            distance = self._range_at_bearing(
                bearing, half_window, now, track
            )
            confidence = clamp(
                area
                / max(1.0, image.shape[0] * image.shape[1] * 0.06),
                0.0,
                1.0,
            )
            track["last_confidence"] = confidence
            annotations.append(
                {
                    "identity": identity,
                    "bounds": bounds,
                    "center": (center_x, center_y),
                    "distance": distance,
                    "selected": identity == self.selected_identity,
                    "color": track["color"],
                }
            )

            if identity == self.selected_identity:
                selected_visual = True
            if distance is None:
                continue

            if (
                track["filtered_bearing"] is None
                or track["last_detection_at"] is None
            ):
                track["filtered_bearing"] = bearing
                track["filtered_distance"] = distance
            else:
                alpha = self.filter_alpha
                track["filtered_bearing"] += alpha * angle_difference(
                    bearing, track["filtered_bearing"]
                )
                track["filtered_distance"] += alpha * (
                    distance - track["filtered_distance"]
                )
            track["last_detection_at"] = now
            track["last_pose"] = self._make_pose(
                image_message, track
            )
            if identity == self.selected_identity:
                selected_detected = True
                self._publish_selected_detection(
                    track, image_message.header.stamp
                )

        if selected_detected:
            self._set_status(f"TRACKING:{self.selected_identity}")
        elif selected_visual:
            self._publish_visible(False)
            self._set_status(f"VISION_ONLY:{self.selected_identity}")
        else:
            selected_track = self.identities[self.selected_identity]
            age = self._track_age(selected_track, now)
            if age is not None and age <= self.detection_timeout:
                self._set_status(
                    f"TEMPORARILY_OCCLUDED:{self.selected_identity}"
                )
            else:
                self._publish_visible(False)
                self._set_status(f"SEARCHING:{self.selected_identity}")

        self._publish_tracks(now)
        self._publish_debug(image, annotations)

    def _find_candidate(self, hsv, track):
        mask = cv2.inRange(hsv, track["lower"], track["upper"])
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
            aspect_ratio = height / max(1.0, float(width))
            if (
                area < self.minimum_blob_area
                or height < self.minimum_blob_height
                or aspect_ratio < self.minimum_blob_aspect_ratio
            ):
                continue
            candidates.append((area, contour, (x, y, width, height)))
        return (
            None
            if not candidates
            else max(candidates, key=lambda item: item[0])
        )

    def _range_at_bearing(self, bearing, half_window, now, track):
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
            track["filtered_distance"] is not None
            and track["last_detection_at"] is not None
            and now - track["last_detection_at"] <= self.detection_timeout
        ):
            elapsed = max(0.0, now - track["last_detection_at"])
            allowed_jump = min(
                self.maximum_range_jump,
                max(0.12, self.maximum_range_rate * elapsed),
            )
            closest_index = min(
                range(len(clusters)),
                key=lambda index: (
                    abs(
                        cluster_ranges[index]
                        - track["filtered_distance"]
                    ),
                    cluster_offsets[index],
                ),
            )
            if (
                abs(
                    cluster_ranges[closest_index]
                    - track["filtered_distance"]
                )
                <= allowed_jump
            ):
                return cluster_ranges[closest_index]
            return None

        nearest_index = min(
            range(len(clusters)),
            key=lambda index: (
                cluster_offsets[index],
                cluster_ranges[index],
            ),
        )
        return cluster_ranges[nearest_index]

    def _make_pose(self, image_message, track):
        bearing = float(track["filtered_bearing"])
        distance = float(track["filtered_distance"])
        pose = PoseStamped()
        pose.header.stamp = image_message.header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x = distance * math.cos(bearing)
        pose.pose.position.y = distance * math.sin(bearing)
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_selected_detection(self, track, stamp):
        pose = track["last_pose"]
        pose.header.stamp = stamp
        self.pose_publisher.publish(pose)
        self._publish_visible(True)
        self._publish_float(
            self.bearing_publisher, track["filtered_bearing"]
        )
        self._publish_float(
            self.distance_publisher, track["filtered_distance"]
        )
        self._publish_float(
            self.confidence_publisher, track["last_confidence"]
        )

        marker = self._make_person_marker(
            self.selected_identity,
            track,
            marker_id=0,
            selected=True,
        )
        self.marker_publisher.publish(marker)

    def _publish_tracks(self, now):
        tracks_message = String()
        tracks_message.data = self._tracks_json(now)
        self.tracks_publisher.publish(tracks_message)

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        for identity, track in self.identities.items():
            age = self._track_age(track, now)
            if age is None or age > self.detection_timeout:
                continue
            marker_id = track["index"] * 2
            markers.append(
                self._make_person_marker(
                    identity,
                    track,
                    marker_id,
                    identity == self.selected_identity,
                )
            )
            markers.append(
                self._make_label_marker(
                    identity,
                    track,
                    marker_id + 1,
                    identity == self.selected_identity,
                )
            )
        self.tracks_marker_publisher.publish(
            MarkerArray(markers=markers)
        )

    def _make_person_marker(
        self, identity, track, marker_id, selected
    ):
        del identity
        marker = Marker()
        marker.header = track["last_pose"].header
        marker.ns = "person_identity_tracks"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = track["last_pose"].pose
        marker.pose.position.z = 0.45
        diameter = 0.42 if selected else 0.32
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = 0.90
        marker.color.r = track["color"][0]
        marker.color.g = track["color"][1]
        marker.color.b = track["color"][2]
        marker.color.a = 0.85 if selected else 0.45
        marker.lifetime.sec = 1
        return marker

    def _make_label_marker(
        self, identity, track, marker_id, selected
    ):
        marker = Marker()
        marker.header = track["last_pose"].header
        marker.ns = "person_identity_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = track["last_pose"].pose
        marker.pose.position.z = 1.05
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.13
        marker.color.r = track["color"][0]
        marker.color.g = track["color"][1]
        marker.color.b = track["color"][2]
        marker.color.a = 1.0
        marker.text = f"{'* ' if selected else ''}{identity}"
        marker.lifetime.sec = 1
        return marker

    def _tracks_json(self, now):
        tracks = []
        for identity, track in self.identities.items():
            age = self._track_age(track, now)
            tracks.append(
                {
                    "identity": identity,
                    "selected": identity == self.selected_identity,
                    "visible": (
                        age is not None and age <= self.detection_timeout
                    ),
                    "age": age,
                    "distance": track["filtered_distance"],
                    "bearing": track["filtered_bearing"],
                    "confidence": track["last_confidence"],
                }
            )
        return json.dumps(
            {
                "selected_identity": self.selected_identity,
                "tracks": tracks,
            },
            separators=(",", ":"),
        )

    @staticmethod
    def _track_age(track, now):
        if track["last_detection_at"] is None:
            return None
        return max(0.0, now - track["last_detection_at"])

    def _publish_visible(self, visible):
        message = Bool()
        message.data = bool(visible)
        self.visible_publisher.publish(message)

    @staticmethod
    def _publish_float(publisher, value):
        message = Float32()
        message.data = float(value)
        publisher.publish(message)

    def _publish_debug(self, image, annotations):
        if not self.debug_image_enabled:
            return
        output = image.copy()
        for annotation in annotations:
            x, y, width, height = annotation["bounds"]
            red, green, blue = annotation["color"]
            bgr = (
                int(255 * blue),
                int(255 * green),
                int(255 * red),
            )
            thickness = 3 if annotation["selected"] else 1
            cv2.rectangle(
                output,
                (x, y),
                (x + width, y + height),
                bgr,
                thickness,
            )
            center_x, center_y = annotation["center"]
            cv2.circle(
                output, (int(center_x), int(center_y)), 4, bgr, -1
            )
            distance = annotation["distance"]
            suffix = "vision" if distance is None else f"{distance:.2f}m"
            label = (
                f"{'*' if annotation['selected'] else ''}"
                f"{annotation['identity']} {suffix}"
            )
            cv2.putText(
                output,
                label,
                (x, max(16, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                bgr,
                thickness,
                cv2.LINE_AA,
            )
        cv2.putText(
            output,
            f"TARGET: {self.selected_identity}",
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        debug_message = self.bridge.cv2_to_imgmsg(
            output, encoding="bgr8"
        )
        debug_message.header.stamp = self.get_clock().now().to_msg()
        debug_message.header.frame_id = "front_camera_optical_frame"
        self.debug_publisher.publish(debug_message)

    def _check_freshness(self):
        now = time.monotonic()
        if self.last_image_at is None:
            self._set_status(
                f"WAITING_FOR_CAMERA:{self.selected_identity}"
            )
            self._publish_visible(False)
            return
        if now - self.last_image_at > self.camera_timeout:
            self._set_status("CAMERA_STALE")
            self._publish_visible(False)
            return

        changed = False
        for track in self.identities.values():
            age = self._track_age(track, now)
            if age is None or age <= self.detection_timeout:
                continue
            if track["filtered_bearing"] is not None:
                changed = True
            track["filtered_bearing"] = None
            track["filtered_distance"] = None

        selected_track = self.identities[self.selected_identity]
        selected_age = self._track_age(selected_track, now)
        if selected_age is None or selected_age > self.detection_timeout:
            self._publish_visible(False)
            visual_age = (
                None
                if selected_track["last_visual_at"] is None
                else now - selected_track["last_visual_at"]
            )
            self._set_status(
                (
                    f"VISION_ONLY:{self.selected_identity}"
                    if visual_age is not None
                    and visual_age <= self.detection_timeout
                    else f"SEARCHING:{self.selected_identity}"
                )
            )
        if changed:
            self._publish_tracks(now)


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
