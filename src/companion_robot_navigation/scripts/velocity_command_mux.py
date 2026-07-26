#!/usr/bin/env python3

"""Select exactly one velocity source before smoothing and safety filtering."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class VelocityCommandMux(Node):
    def __init__(self):
        super().__init__("velocity_command_mux")

        self.declare_parameter("navigation_topic", "/cmd_vel_nav_raw")
        self.declare_parameter("behavior_topic", "/cmd_vel_behavior_raw")
        self.declare_parameter(
            "behavior_active_topic", "/cmd_vel_behavior_active"
        )
        self.declare_parameter("output_topic", "/cmd_vel_selected")
        self.declare_parameter("update_rate", 25.0)
        self.declare_parameter("command_timeout", 0.50)
        self.declare_parameter("behavior_active_timeout", 0.50)
        self.declare_parameter("behavior_release_delay", 0.20)

        navigation_topic = str(
            self.get_parameter("navigation_topic").value
        )
        behavior_topic = str(self.get_parameter("behavior_topic").value)
        behavior_active_topic = str(
            self.get_parameter("behavior_active_topic").value
        )
        output_topic = str(self.get_parameter("output_topic").value)
        update_rate = max(
            5.0, float(self.get_parameter("update_rate").value)
        )
        self.command_timeout = max(
            0.10, float(self.get_parameter("command_timeout").value)
        )
        self.behavior_active_timeout = max(
            0.10,
            float(self.get_parameter("behavior_active_timeout").value),
        )
        self.behavior_release_delay = max(
            0.0,
            float(self.get_parameter("behavior_release_delay").value),
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.command_publisher = self.create_publisher(
            Twist, output_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, "/cmd_vel_mux/status", latched_qos
        )
        self.navigation_subscription = self.create_subscription(
            Twist, navigation_topic, self._navigation_callback, 10
        )
        self.behavior_subscription = self.create_subscription(
            Twist, behavior_topic, self._behavior_callback, 10
        )
        self.behavior_active_subscription = self.create_subscription(
            Bool,
            behavior_active_topic,
            self._behavior_active_callback,
            latched_qos,
        )

        self.last_navigation_command = None
        self.last_navigation_at = None
        self.last_behavior_command = None
        self.last_behavior_at = None
        self.behavior_active = False
        self.last_behavior_active_at = None
        self.behavior_release_until = 0.0
        self.status = "UNKNOWN"

        self.timer = self.create_timer(1.0 / update_rate, self._update)
        self._set_status("IDLE")
        self.get_logger().info(
            "Velocity command mux ready: navigation and behavior commands "
            "are mutually exclusive."
        )

    def _navigation_callback(self, command):
        self.last_navigation_command = command
        self.last_navigation_at = time.monotonic()

    def _behavior_callback(self, command):
        self.last_behavior_command = command
        self.last_behavior_at = time.monotonic()

    def _behavior_active_callback(self, active):
        now = time.monotonic()
        was_active = self.behavior_active
        self.behavior_active = bool(active.data)
        self.last_behavior_active_at = now
        if was_active and not self.behavior_active:
            self.behavior_release_until = now + self.behavior_release_delay

    def _update(self):
        now = time.monotonic()
        behavior_lease_fresh = (
            self.last_behavior_active_at is not None
            and now - self.last_behavior_active_at
            <= self.behavior_active_timeout
        )
        behavior_selected = self.behavior_active and behavior_lease_fresh

        if behavior_selected:
            if (
                self.last_behavior_command is not None
                and self.last_behavior_at is not None
                and now - self.last_behavior_at <= self.command_timeout
            ):
                self.command_publisher.publish(
                    self.last_behavior_command
                )
                self._set_status("BEHAVIOR")
            else:
                self.command_publisher.publish(Twist())
                self._set_status("BEHAVIOR_STALE_STOP")
            return

        if now < self.behavior_release_until:
            self.command_publisher.publish(Twist())
            self._set_status("BEHAVIOR_RELEASE_STOP")
            return

        if (
            self.last_navigation_command is not None
            and self.last_navigation_at is not None
            and now - self.last_navigation_at <= self.command_timeout
        ):
            self.command_publisher.publish(self.last_navigation_command)
            self._set_status("NAVIGATION")
        else:
            self.command_publisher.publish(Twist())
            self._set_status("IDLE")

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info("Velocity source: %s" % status)

    def stop(self):
        self.command_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommandMux()
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
