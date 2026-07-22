#!/usr/bin/env python3

"""Publish human-readable autonomy health for RViz and diagnostics tools."""

import json
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def finite_or_none(value):
    return value if math.isfinite(value) else None


class AutonomyDiagnostics(Node):
    def __init__(self):
        super().__init__("autonomy_diagnostics")

        self.declare_parameter("publish_frequency", 2.0)
        self.declare_parameter("startup_grace_seconds", 15.0)
        self.declare_parameter("sensor_warning_timeout", 1.5)
        self.declare_parameter("sensor_error_timeout", 4.0)
        self.declare_parameter("localization_warning_timeout", 6.0)
        self.declare_parameter("localization_error_timeout", 15.0)
        self.declare_parameter("localization_position_std_warn", 0.40)
        self.declare_parameter("localization_position_std_error", 0.80)
        self.declare_parameter("localization_yaw_std_warn", 0.60)
        self.declare_parameter("localization_yaw_std_error", 1.20)
        self.declare_parameter("low_battery_threshold", 0.25)
        self.declare_parameter("critical_battery_threshold", 0.12)
        self.declare_parameter("marker_frame", "base_footprint")

        self.publish_frequency = max(
            0.5, float(self.get_parameter("publish_frequency").value)
        )
        self.startup_grace = max(
            1.0, float(self.get_parameter("startup_grace_seconds").value)
        )
        self.sensor_warning_timeout = max(
            0.2, float(self.get_parameter("sensor_warning_timeout").value)
        )
        self.sensor_error_timeout = max(
            self.sensor_warning_timeout,
            float(self.get_parameter("sensor_error_timeout").value),
        )
        self.localization_warning_timeout = max(
            0.5,
            float(self.get_parameter("localization_warning_timeout").value),
        )
        self.localization_error_timeout = max(
            self.localization_warning_timeout,
            float(self.get_parameter("localization_error_timeout").value),
        )
        self.position_std_warn = max(
            0.05,
            float(
                self.get_parameter("localization_position_std_warn").value
            ),
        )
        self.position_std_error = max(
            self.position_std_warn,
            float(
                self.get_parameter("localization_position_std_error").value
            ),
        )
        self.yaw_std_warn = max(
            0.05,
            float(self.get_parameter("localization_yaw_std_warn").value),
        )
        self.yaw_std_error = max(
            self.yaw_std_warn,
            float(self.get_parameter("localization_yaw_std_error").value),
        )
        self.low_battery_threshold = clamp(
            float(self.get_parameter("low_battery_threshold").value),
            0.01,
            0.95,
        )
        self.critical_battery_threshold = clamp(
            float(self.get_parameter("critical_battery_threshold").value),
            0.0,
            self.low_battery_threshold,
        )
        self.marker_frame = str(self.get_parameter("marker_frame").value)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.health_publisher = self.create_publisher(
            String, "/autonomy/health", latched_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/autonomy/visualization", latched_qos
        )

        self.create_subscription(
            String, "/mission_status", self._mission_callback, latched_qos
        )
        self.create_subscription(
            String, "/mission_detail", self._detail_callback, latched_qos
        )
        self.create_subscription(
            String, "/patrol_status", self._patrol_callback, latched_qos
        )
        self.create_subscription(
            String, "/docking_status", self._docking_callback, latched_qos
        )
        self.create_subscription(
            String, "/dock_marker/status", self._marker_callback, latched_qos
        )
        self.create_subscription(
            BatteryState, "/battery_state", self._battery_callback, latched_qos
        )
        self.create_subscription(
            Odometry, "/odom", self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            10,
        )
        self.create_subscription(
            Twist, "/cmd_vel_nav", self._command_callback, 10
        )

        self.status_service = self.create_service(
            Trigger, "/get_autonomy_health", self._handle_status_request
        )

        self.started_at = time.monotonic()
        self.last_seen = {}
        self.mission_state = "INITIALIZING"
        self.mission_detail = "Waiting for autonomy data."
        self.patrol_state = "UNKNOWN"
        self.docking_state = "UNKNOWN"
        self.dock_marker_state = "UNKNOWN"
        self.battery_percentage = None
        self.localization_position_std = None
        self.localization_yaw_std = None
        self.navigation_commanded = False
        self.health_level = DiagnosticStatus.STALE
        self.health_label = "STARTING"
        self.health_reasons = ["Waiting for sensor and mission data."]

        self.timer = self.create_timer(
            1.0 / self.publish_frequency, self._publish
        )
        self.get_logger().info(
            "Autonomy diagnostics ready on /diagnostics, "
            "/autonomy/health, and /autonomy/visualization."
        )

    def _touch(self, source):
        self.last_seen[source] = time.monotonic()

    def _mission_callback(self, message):
        self.mission_state = message.data.strip().upper() or "UNKNOWN"
        self._touch("mission")

    def _detail_callback(self, message):
        self.mission_detail = message.data.strip()
        self._touch("mission_detail")

    def _patrol_callback(self, message):
        self.patrol_state = message.data.strip().upper() or "UNKNOWN"
        self._touch("patrol")

    def _docking_callback(self, message):
        self.docking_state = message.data.strip().upper() or "UNKNOWN"
        self._touch("docking")

    def _marker_callback(self, message):
        self.dock_marker_state = message.data.strip().upper() or "UNKNOWN"
        self._touch("dock_marker")

    def _battery_callback(self, message):
        if math.isfinite(message.percentage) and message.percentage >= 0.0:
            self.battery_percentage = clamp(message.percentage, 0.0, 1.0)
        self._touch("battery")

    def _odom_callback(self, message):
        del message
        self._touch("odom")

    def _scan_callback(self, message):
        del message
        self._touch("scan")

    def _amcl_callback(self, message):
        covariance = message.pose.covariance
        x_variance = max(0.0, finite_or_none(covariance[0]) or 0.0)
        y_variance = max(0.0, finite_or_none(covariance[7]) or 0.0)
        yaw_variance = max(0.0, finite_or_none(covariance[35]) or 0.0)
        self.localization_position_std = math.sqrt(
            max(x_variance, y_variance)
        )
        self.localization_yaw_std = math.sqrt(yaw_variance)
        self._touch("amcl")

    def _command_callback(self, message):
        self.navigation_commanded = (
            abs(message.linear.x) > 0.02
            or abs(message.linear.y) > 0.02
            or abs(message.angular.z) > 0.05
        )
        self._touch("command")

    def _age(self, source, now):
        stamp = self.last_seen.get(source)
        return None if stamp is None else max(0.0, now - stamp)

    def _status(self, name, level, message, values):
        status = DiagnosticStatus()
        status.name = f"companion_robot/{name}"
        status.hardware_id = "companion_robot_sim"
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        return status

    def _freshness_level(self, age, warn_timeout, error_timeout, starting):
        if age is None:
            return DiagnosticStatus.STALE if starting else DiagnosticStatus.ERROR
        if age >= error_timeout:
            return DiagnosticStatus.ERROR
        if age >= warn_timeout:
            return DiagnosticStatus.WARN
        return DiagnosticStatus.OK

    def _evaluate(self, now):
        starting = now - self.started_at < self.startup_grace
        statuses = []

        mission_level = DiagnosticStatus.OK
        if self.mission_state == "ERROR":
            mission_level = DiagnosticStatus.ERROR
        elif self.mission_state in {"RECOVERY", "INITIALIZING", "UNKNOWN"}:
            mission_level = DiagnosticStatus.WARN
        statuses.append(
            self._status(
                "mission",
                mission_level,
                self.mission_state,
                {
                    "detail": self.mission_detail,
                    "patrol": self.patrol_state,
                    "docking": self.docking_state,
                    "dock_marker": self.dock_marker_state,
                },
            )
        )

        battery_level = DiagnosticStatus.OK
        battery_text = "unknown"
        if self.battery_percentage is None:
            battery_level = (
                DiagnosticStatus.STALE if starting else DiagnosticStatus.ERROR
            )
        else:
            battery_text = f"{self.battery_percentage * 100.0:.1f}%"
            if self.battery_percentage <= self.critical_battery_threshold:
                battery_level = DiagnosticStatus.ERROR
            elif self.battery_percentage <= self.low_battery_threshold:
                battery_level = DiagnosticStatus.WARN
        statuses.append(
            self._status(
                "battery",
                battery_level,
                battery_text,
                {
                    "percentage": battery_text,
                    "age_seconds": self._age("battery", now),
                },
            )
        )

        odom_age = self._age("odom", now)
        scan_age = self._age("scan", now)
        odom_level = self._freshness_level(
            odom_age,
            self.sensor_warning_timeout,
            self.sensor_error_timeout,
            starting,
        )
        scan_level = self._freshness_level(
            scan_age,
            self.sensor_warning_timeout,
            self.sensor_error_timeout,
            starting,
        )
        statuses.append(
            self._status(
                "sensors",
                max(odom_level, scan_level),
                "odometry and LiDAR",
                {
                    "odometry_age_seconds": odom_age,
                    "lidar_age_seconds": scan_age,
                },
            )
        )

        amcl_age = self._age("amcl", now)
        localization_level = self._freshness_level(
            amcl_age,
            self.localization_warning_timeout,
            self.localization_error_timeout,
            starting,
        )
        if self.localization_position_std is not None:
            if self.localization_position_std >= self.position_std_error:
                localization_level = max(
                    localization_level, DiagnosticStatus.ERROR
                )
            elif self.localization_position_std >= self.position_std_warn:
                localization_level = max(
                    localization_level, DiagnosticStatus.WARN
                )
        if self.localization_yaw_std is not None:
            if self.localization_yaw_std >= self.yaw_std_error:
                localization_level = max(
                    localization_level, DiagnosticStatus.ERROR
                )
            elif self.localization_yaw_std >= self.yaw_std_warn:
                localization_level = max(
                    localization_level, DiagnosticStatus.WARN
                )
        statuses.append(
            self._status(
                "localization",
                localization_level,
                "AMCL localization",
                {
                    "amcl_age_seconds": amcl_age,
                    "position_std_m": self.localization_position_std,
                    "yaw_std_rad": self.localization_yaw_std,
                },
            )
        )

        navigation_level = DiagnosticStatus.OK
        if self.patrol_state == "ERROR":
            navigation_level = DiagnosticStatus.ERROR
        elif self.patrol_state in {"RECOVERY", "WAITING_FOR_NAV2"}:
            navigation_level = DiagnosticStatus.WARN
        statuses.append(
            self._status(
                "navigation",
                navigation_level,
                self.patrol_state,
                {
                    "commanded_motion": self.navigation_commanded,
                    "command_age_seconds": self._age("command", now),
                },
            )
        )
        return statuses

    def _publish(self):
        now = time.monotonic()
        statuses = self._evaluate(now)
        levels = [status.level for status in statuses]
        self.health_level = max(levels) if levels else DiagnosticStatus.STALE
        labels = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE",
        }
        self.health_label = labels.get(self.health_level, "UNKNOWN")
        self.health_reasons = [
            f"{status.name.split('/')[-1]}: {status.message}"
            for status in statuses
            if status.level == self.health_level
        ]

        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.status = statuses
        self.diagnostics_publisher.publish(diagnostic_array)

        health_message = String()
        health_message.data = self.health_label
        self.health_publisher.publish(health_message)
        self.marker_publisher.publish(self._make_markers())

    def _make_markers(self):
        now = self.get_clock().now().to_msg()
        colors = {
            DiagnosticStatus.OK: (0.10, 0.90, 0.35, 0.95),
            DiagnosticStatus.WARN: (1.00, 0.70, 0.05, 0.95),
            DiagnosticStatus.ERROR: (1.00, 0.12, 0.08, 0.95),
            DiagnosticStatus.STALE: (0.55, 0.60, 0.68, 0.95),
        }
        red, green, blue, alpha = colors.get(
            self.health_level, colors[DiagnosticStatus.STALE]
        )

        indicator = Marker()
        indicator.header.frame_id = self.marker_frame
        indicator.header.stamp = now
        indicator.ns = "autonomy_health"
        indicator.id = 0
        indicator.type = Marker.SPHERE
        indicator.action = Marker.ADD
        indicator.pose.position.z = 0.72
        indicator.pose.orientation.w = 1.0
        indicator.scale.x = 0.14
        indicator.scale.y = 0.14
        indicator.scale.z = 0.14
        indicator.color.r = red
        indicator.color.g = green
        indicator.color.b = blue
        indicator.color.a = alpha

        battery = (
            "unknown"
            if self.battery_percentage is None
            else f"{self.battery_percentage * 100.0:.0f}%"
        )
        text = Marker()
        text.header.frame_id = self.marker_frame
        text.header.stamp = now
        text.ns = "autonomy_health"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.z = 1.05
        text.pose.orientation.w = 1.0
        text.scale.z = 0.13
        text.color.r = red
        text.color.g = green
        text.color.b = blue
        text.color.a = 1.0
        text.text = (
            f"AUTONOMY {self.health_label}\n"
            f"Mission: {self.mission_state} | Battery: {battery}\n"
            f"Patrol: {self.patrol_state}\n"
            f"Dock: {self.docking_state} | Marker: {self.dock_marker_state}"
        )
        return MarkerArray(markers=[indicator, text])

    def _handle_status_request(self, request, response):
        del request
        summary = {
            "health": self.health_label,
            "reasons": self.health_reasons,
            "mission": self.mission_state,
            "patrol": self.patrol_state,
            "docking": self.docking_state,
            "dock_marker": self.dock_marker_state,
            "battery_percentage": self.battery_percentage,
            "localization_position_std": self.localization_position_std,
            "localization_yaw_std": self.localization_yaw_std,
        }
        response.success = self.health_level < DiagnosticStatus.ERROR
        response.message = json.dumps(summary, separators=(",", ":"))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyDiagnostics()
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

