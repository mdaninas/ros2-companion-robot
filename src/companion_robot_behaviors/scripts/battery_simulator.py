#!/usr/bin/env python3

"""Simulate battery discharge, charging, and low-battery auto-docking."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class BatterySimulator(Node):
    DOCKED_STATES = {"DOCKED", "CHARGING", "FULLY_CHARGED"}
    AUTO_DOCK_READY_STATES = {"IDLE", "ERROR"}

    def __init__(self):
        super().__init__("battery_simulator")

        self.declare_parameter("initial_percentage", 0.75)
        self.declare_parameter("low_battery_threshold", 0.25)
        self.declare_parameter("auto_dock_rearm_threshold", 0.40)
        self.declare_parameter("auto_dock_enabled", True)
        self.declare_parameter("idle_discharge_rate", 0.00005)
        self.declare_parameter("motion_discharge_rate", 0.0015)
        self.declare_parameter("charging_rate", 0.02)
        self.declare_parameter("publish_frequency", 2.0)
        self.declare_parameter("command_timeout", 1.0)
        self.declare_parameter("nominal_voltage", 11.1)
        self.declare_parameter("full_voltage", 12.6)
        self.declare_parameter("empty_voltage", 9.9)
        self.declare_parameter("design_capacity_ah", 5.0)

        self.percentage = clamp(
            float(self.get_parameter("initial_percentage").value), 0.0, 1.0
        )
        self.low_threshold = clamp(
            float(self.get_parameter("low_battery_threshold").value),
            0.01,
            0.95,
        )
        self.rearm_threshold = clamp(
            float(self.get_parameter("auto_dock_rearm_threshold").value),
            self.low_threshold,
            1.0,
        )
        self.auto_dock_enabled = bool(
            self.get_parameter("auto_dock_enabled").value
        )
        self.idle_discharge_rate = max(
            0.0, float(self.get_parameter("idle_discharge_rate").value)
        )
        self.motion_discharge_rate = max(
            0.0, float(self.get_parameter("motion_discharge_rate").value)
        )
        self.charging_rate = max(
            0.0, float(self.get_parameter("charging_rate").value)
        )
        self.publish_frequency = max(
            0.2, float(self.get_parameter("publish_frequency").value)
        )
        self.command_timeout = max(
            0.1, float(self.get_parameter("command_timeout").value)
        )
        self.nominal_voltage = float(
            self.get_parameter("nominal_voltage").value
        )
        self.full_voltage = float(self.get_parameter("full_voltage").value)
        self.empty_voltage = float(
            self.get_parameter("empty_voltage").value
        )
        self.design_capacity_ah = max(
            0.1, float(self.get_parameter("design_capacity_ah").value)
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.battery_publisher = self.create_publisher(
            BatteryState, "/battery_state", latched_qos
        )
        self.docking_subscription = self.create_subscription(
            String,
            "/docking_status",
            self._docking_status_callback,
            latched_qos,
        )
        self.command_subscription = self.create_subscription(
            Twist, "/cmd_vel", self._command_callback, 10
        )
        self.low_battery_service = self.create_service(
            Trigger,
            "/simulate_low_battery",
            self._simulate_low_battery,
        )
        self.dock_client = self.create_client(Trigger, "/dock_robot")

        self.docking_state = "UNKNOWN"
        self.motion_level = 0.0
        self.last_command_at = None
        self.last_update_at = self.get_clock().now()
        self.auto_dock_latched = False
        self.auto_dock_future = None
        self.last_unavailable_log_at = 0.0
        self.last_percentage_log = None

        self.timer = self.create_timer(
            1.0 / self.publish_frequency, self._update
        )
        self._publish_battery()
        self.get_logger().info(
            "Battery simulation ready: "
            f"{self.percentage * 100.0:.1f}% available, "
            f"auto-dock threshold {self.low_threshold * 100.0:.1f}%."
        )

    def _docking_status_callback(self, message):
        self.docking_state = message.data

    def _command_callback(self, command):
        linear_load = abs(command.linear.x) / 0.22
        angular_load = abs(command.angular.z) / 0.70
        self.motion_level = clamp(
            linear_load + 0.5 * angular_load, 0.0, 1.0
        )
        self.last_command_at = self.get_clock().now()

    def _simulate_low_battery(self, request, response):
        del request
        self.percentage = max(0.01, self.low_threshold - 0.05)
        self.auto_dock_latched = False
        response.success = True
        response.message = (
            f"Battery set to {self.percentage * 100.0:.1f}%; "
            "automatic docking will be requested."
        )
        self.get_logger().warning(response.message)
        self._publish_battery()
        return response

    def _update(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_update_at).nanoseconds / 1_000_000_000.0
        self.last_update_at = now
        if elapsed < 0.0 or elapsed > 2.0:
            elapsed = 0.0

        charging = self.docking_state in self.DOCKED_STATES
        if charging:
            self.percentage += self.charging_rate * elapsed
        else:
            active_motion = self._active_motion(now)
            discharge_rate = (
                self.idle_discharge_rate
                + self.motion_discharge_rate * active_motion
            )
            self.percentage -= discharge_rate * elapsed

        self.percentage = clamp(self.percentage, 0.0, 1.0)
        if self.percentage >= self.rearm_threshold:
            self.auto_dock_latched = False

        self._publish_battery()
        self._log_percentage_changes()
        self._request_automatic_docking_if_needed()

    def _active_motion(self, now):
        if self.last_command_at is None:
            return 0.0
        age = (now - self.last_command_at).nanoseconds / 1_000_000_000.0
        if age < 0.0 or age > self.command_timeout:
            return 0.0
        return self.motion_level

    def _publish_battery(self):
        charging = self.docking_state in self.DOCKED_STATES
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = "base_link"
        battery.percentage = self.percentage
        battery.voltage = self.empty_voltage + (
            self.full_voltage - self.empty_voltage
        ) * self.percentage
        battery.capacity = self.design_capacity_ah * self.percentage
        battery.design_capacity = self.design_capacity_ah
        battery.present = True
        battery.location = "companion_robot_battery"
        battery.serial_number = "SIM-BATTERY-001"
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        )

        if charging and self.percentage >= 0.999:
            battery.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_FULL
            )
            battery.current = 0.0
        elif charging:
            battery.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING
            )
            battery.current = 1.5
        else:
            battery.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            )
            battery.current = -0.15 - 0.85 * self.motion_level

        self.battery_publisher.publish(battery)

    def _log_percentage_changes(self):
        rounded = int(self.percentage * 20.0) * 5
        if rounded == self.last_percentage_log:
            return
        self.last_percentage_log = rounded
        self.get_logger().info(
            f"Battery: {self.percentage * 100.0:.1f}% "
            f"(dock state: {self.docking_state})."
        )

    def _request_automatic_docking_if_needed(self):
        if (
            not self.auto_dock_enabled
            or self.percentage > self.low_threshold
            or self.auto_dock_latched
            or self.auto_dock_future is not None
            or self.docking_state not in self.AUTO_DOCK_READY_STATES
        ):
            return

        if not self.dock_client.service_is_ready():
            now = time.monotonic()
            if now - self.last_unavailable_log_at >= 2.0:
                self.last_unavailable_log_at = now
                self.get_logger().warning(
                    "Low battery; waiting for /dock_robot service."
                )
            return

        self.auto_dock_latched = True
        self.get_logger().warning(
            f"Low battery ({self.percentage * 100.0:.1f}%); "
            "requesting automatic docking."
        )
        self.auto_dock_future = self.dock_client.call_async(Trigger.Request())
        self.auto_dock_future.add_done_callback(self._auto_dock_response)

    def _auto_dock_response(self, future):
        self.auto_dock_future = None
        try:
            response = future.result()
        except Exception as error:
            self.auto_dock_latched = False
            self.get_logger().error(f"Automatic docking call failed: {error}")
            return

        if response.success:
            self.get_logger().info(
                f"Automatic docking accepted: {response.message}"
            )
        else:
            self.auto_dock_latched = False
            self.get_logger().error(
                f"Automatic docking rejected: {response.message}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()

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
