#!/usr/bin/env python3

"""Filter navigation velocity using fresh, direction-aware LiDAR clearance."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class DirectionalSafetyFilter(Node):
    def __init__(self):
        super().__init__("directional_safety_filter")

        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("update_rate", 25.0)
        self.declare_parameter("command_timeout", 0.50)
        self.declare_parameter("scan_timeout", 0.50)
        self.declare_parameter("minimum_scan_range", 0.27)
        self.declare_parameter("slowdown_ratio", 0.35)
        self.declare_parameter("front_stop_distance", 0.40)
        self.declare_parameter("front_slowdown_distance", 0.85)
        self.declare_parameter("rear_stop_distance", 0.36)
        self.declare_parameter("rear_slowdown_distance", 0.65)
        self.declare_parameter("side_stop_distance", 0.34)
        self.declare_parameter("side_slowdown_distance", 0.50)
        self.declare_parameter("linear_epsilon", 0.01)
        self.declare_parameter("angular_epsilon", 0.03)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        update_rate = max(
            5.0, float(self.get_parameter("update_rate").value)
        )
        self.command_timeout = max(
            0.10, float(self.get_parameter("command_timeout").value)
        )
        self.scan_timeout = max(
            0.10, float(self.get_parameter("scan_timeout").value)
        )
        self.minimum_scan_range = max(
            0.0, float(self.get_parameter("minimum_scan_range").value)
        )
        self.slowdown_ratio = min(
            1.0,
            max(0.05, float(self.get_parameter("slowdown_ratio").value)),
        )
        self.front_stop_distance = float(
            self.get_parameter("front_stop_distance").value
        )
        self.front_slowdown_distance = max(
            self.front_stop_distance,
            float(self.get_parameter("front_slowdown_distance").value),
        )
        self.rear_stop_distance = float(
            self.get_parameter("rear_stop_distance").value
        )
        self.rear_slowdown_distance = max(
            self.rear_stop_distance,
            float(self.get_parameter("rear_slowdown_distance").value),
        )
        self.side_stop_distance = float(
            self.get_parameter("side_stop_distance").value
        )
        self.side_slowdown_distance = max(
            self.side_stop_distance,
            float(self.get_parameter("side_slowdown_distance").value),
        )
        self.linear_epsilon = max(
            0.0, float(self.get_parameter("linear_epsilon").value)
        )
        self.angular_epsilon = max(
            0.0, float(self.get_parameter("angular_epsilon").value)
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.command_publisher = self.create_publisher(
            Twist, self.output_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, "/collision_safety/status", latched_qos
        )
        self.command_subscription = self.create_subscription(
            Twist, self.input_topic, self._command_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.last_command = None
        self.last_command_at = None
        self.last_scan_at = None
        self.sector_ranges = {
            "front": math.inf,
            "rear": math.inf,
            "left": math.inf,
            "right": math.inf,
        }
        self.output_active = False
        self.status = "UNKNOWN"

        self.timer = self.create_timer(1.0 / update_rate, self._update)
        self._set_status("WAITING_FOR_SCAN")
        self.get_logger().info(
            "Directional safety filter ready: %s -> %s."
            % (self.input_topic, self.output_topic)
        )

    def _command_callback(self, command):
        self.last_command = command
        self.last_command_at = time.monotonic()

    def _scan_callback(self, scan):
        sectors = {
            "front": math.inf,
            "rear": math.inf,
            "left": math.inf,
            "right": math.inf,
        }
        front_half = math.radians(55.0)
        rear_start = math.pi - front_half
        side_start = math.radians(35.0)
        side_end = math.radians(145.0)
        valid_minimum = max(
            float(scan.range_min), self.minimum_scan_range
        )

        for index, raw_range in enumerate(scan.ranges):
            distance = float(raw_range)
            if not math.isfinite(distance):
                continue
            if distance < valid_minimum or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            absolute_angle = abs(angle)
            if absolute_angle <= front_half:
                sectors["front"] = min(sectors["front"], distance)
            if absolute_angle >= rear_start:
                sectors["rear"] = min(sectors["rear"], distance)
            if side_start <= angle <= side_end:
                sectors["left"] = min(sectors["left"], distance)
            if -side_end <= angle <= -side_start:
                sectors["right"] = min(sectors["right"], distance)

        self.sector_ranges = sectors
        self.last_scan_at = time.monotonic()

    def _update(self):
        now = time.monotonic()
        if (
            self.last_command is None
            or self.last_command_at is None
            or now - self.last_command_at > self.command_timeout
        ):
            if self.output_active:
                self.command_publisher.publish(Twist())
                self.output_active = False
            self._set_status("IDLE")
            return

        if (
            self.last_scan_at is None
            or now - self.last_scan_at > self.scan_timeout
        ):
            self.command_publisher.publish(Twist())
            self.output_active = True
            self._set_status("SCAN_STALE_STOP")
            return

        command = self.last_command
        sector = None
        stop_distance = 0.0
        slowdown_distance = 0.0

        if command.linear.x > self.linear_epsilon:
            sector = "front"
            stop_distance = self.front_stop_distance
            slowdown_distance = self.front_slowdown_distance
        elif command.linear.x < -self.linear_epsilon:
            # A person in front must not block a safe reverse command.
            sector = "rear"
            stop_distance = self.rear_stop_distance
            slowdown_distance = self.rear_slowdown_distance
        elif command.angular.z > self.angular_epsilon:
            sector = "left"
            stop_distance = self.side_stop_distance
            slowdown_distance = self.side_slowdown_distance
        elif command.angular.z < -self.angular_epsilon:
            sector = "right"
            stop_distance = self.side_stop_distance
            slowdown_distance = self.side_slowdown_distance

        distance = (
            math.inf if sector is None else self.sector_ranges[sector]
        )
        if distance <= stop_distance:
            output = Twist()
            # When translation is blocked, keep a safe in-place turn instead
            # of freezing every axis. This lets following behaviors continue
            # facing the person and can rotate the blocked rear sector away
            # from an obstacle before trying to reverse again.
            if abs(command.angular.z) > self.angular_epsilon:
                turn_sector = "left" if command.angular.z > 0.0 else "right"
                turn_distance = self.sector_ranges[turn_sector]
                if turn_distance > self.side_stop_distance:
                    turn_scale = (
                        self.slowdown_ratio
                        if turn_distance <= self.side_slowdown_distance
                        else 1.0
                    )
                    output.angular.z = command.angular.z * turn_scale
            state = "STOP_%s" % sector.upper()
        else:
            scale = (
                self.slowdown_ratio
                if distance <= slowdown_distance
                else 1.0
            )
            output = Twist()
            output.linear.x = command.linear.x * scale
            output.linear.y = command.linear.y * scale
            output.linear.z = command.linear.z * scale
            output.angular.x = command.angular.x * scale
            output.angular.y = command.angular.y * scale
            output.angular.z = command.angular.z * scale
            state = (
                "SLOWDOWN_%s" % sector.upper()
                if scale < 1.0 and sector is not None
                else "CLEAR"
            )

        self.command_publisher.publish(output)
        self.output_active = True
        self._set_status(state)

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info("Collision-safety status: %s" % status)

    def stop(self):
        self.command_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DirectionalSafetyFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
