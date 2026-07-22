#!/usr/bin/env python3

"""Coordinate patrol, energy, docking, and autonomous recovery states."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


class MissionManager(Node):
    DOCKING_STATES = {
        "WAITING_FOR_NAV2",
        "NAVIGATING_TO_STAGING",
        "ALIGNING_WITH_DOCK",
        "ACQUIRING_DOCK_MARKER",
        "PRECISION_DOCKING",
        "RECOVERING_DOCK_MARKER",
    }
    CHARGING_STATES = {"DOCKED", "CHARGING"}

    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("low_battery_threshold", 0.25)
        self.declare_parameter("full_battery_threshold", 0.999)
        self.declare_parameter("full_charge_hold_seconds", 1.0)
        self.declare_parameter("recovery_retry_delay", 3.0)
        self.declare_parameter("max_patrol_recovery_attempts", 3)
        self.declare_parameter("max_docking_recovery_attempts", 3)
        self.declare_parameter("localization_startup_grace", 20.0)
        self.declare_parameter("localization_timeout", 15.0)
        self.declare_parameter("localization_position_std_limit", 0.80)
        self.declare_parameter("localization_yaw_std_limit", 1.20)
        self.declare_parameter("localization_recovery_cooldown", 10.0)
        self.declare_parameter("max_localization_recovery_attempts", 2)
        self.declare_parameter("localization_stable_reset_seconds", 8.0)
        self.declare_parameter("stuck_timeout", 15.0)
        self.declare_parameter("command_timeout", 1.0)
        self.declare_parameter("linear_command_threshold", 0.03)
        self.declare_parameter("angular_command_threshold", 0.08)
        self.declare_parameter("progress_distance_epsilon", 0.04)
        self.declare_parameter("progress_yaw_epsilon", 0.08)
        self.declare_parameter("update_frequency", 5.0)

        self.low_battery_threshold = clamp(
            float(self.get_parameter("low_battery_threshold").value),
            0.01,
            0.95,
        )
        self.full_battery_threshold = clamp(
            float(self.get_parameter("full_battery_threshold").value),
            self.low_battery_threshold,
            1.0,
        )
        self.full_charge_hold_seconds = max(
            0.0,
            float(self.get_parameter("full_charge_hold_seconds").value),
        )
        self.recovery_retry_delay = max(
            0.5, float(self.get_parameter("recovery_retry_delay").value)
        )
        self.max_patrol_recovery_attempts = max(
            1,
            int(
                self.get_parameter("max_patrol_recovery_attempts").value
            ),
        )
        self.max_docking_recovery_attempts = max(
            1,
            int(
                self.get_parameter("max_docking_recovery_attempts").value
            ),
        )
        self.localization_startup_grace = max(
            3.0,
            float(self.get_parameter("localization_startup_grace").value),
        )
        self.localization_timeout = max(
            3.0, float(self.get_parameter("localization_timeout").value)
        )
        self.localization_position_std_limit = max(
            0.10,
            float(
                self.get_parameter("localization_position_std_limit").value
            ),
        )
        self.localization_yaw_std_limit = max(
            0.10,
            float(self.get_parameter("localization_yaw_std_limit").value),
        )
        self.localization_recovery_cooldown = max(
            2.0,
            float(
                self.get_parameter("localization_recovery_cooldown").value
            ),
        )
        self.max_localization_recovery_attempts = max(
            1,
            int(
                self.get_parameter(
                    "max_localization_recovery_attempts"
                ).value
            ),
        )
        self.localization_stable_reset_seconds = max(
            2.0,
            float(
                self.get_parameter("localization_stable_reset_seconds").value
            ),
        )
        self.stuck_timeout = max(
            3.0, float(self.get_parameter("stuck_timeout").value)
        )
        self.command_timeout = max(
            0.2, float(self.get_parameter("command_timeout").value)
        )
        self.linear_command_threshold = max(
            0.0,
            float(self.get_parameter("linear_command_threshold").value),
        )
        self.angular_command_threshold = max(
            0.0,
            float(self.get_parameter("angular_command_threshold").value),
        )
        self.progress_distance_epsilon = max(
            0.005,
            float(self.get_parameter("progress_distance_epsilon").value),
        )
        self.progress_yaw_epsilon = max(
            0.01,
            float(self.get_parameter("progress_yaw_epsilon").value),
        )
        update_frequency = max(
            1.0, float(self.get_parameter("update_frequency").value)
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_publisher = self.create_publisher(
            String, "/mission_status", latched_qos
        )
        self.detail_publisher = self.create_publisher(
            String, "/mission_detail", latched_qos
        )
        self.battery_subscription = self.create_subscription(
            BatteryState,
            "/battery_state",
            self._battery_callback,
            latched_qos,
        )
        self.docking_subscription = self.create_subscription(
            String,
            "/docking_status",
            self._docking_status_callback,
            latched_qos,
        )
        self.patrol_subscription = self.create_subscription(
            String,
            "/patrol_status",
            self._patrol_status_callback,
            latched_qos,
        )
        self.marker_subscription = self.create_subscription(
            String,
            "/dock_marker/status",
            self._marker_status_callback,
            latched_qos,
        )
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 20
        )
        self.command_subscription = self.create_subscription(
            Twist, "/cmd_vel_nav", self._command_callback, 20
        )
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_callback,
            10,
        )

        self.dock_client = self.create_client(Trigger, "/dock_robot")
        self.undock_client = self.create_client(Trigger, "/undock_robot")
        self.patrol_recovery_client = self.create_client(
            Trigger, "/recover_patrol"
        )
        self.nomotion_update_client = self.create_client(
            Empty, "/request_nomotion_update"
        )
        self.global_localization_client = self.create_client(
            Empty, "/reinitialize_global_localization"
        )
        self.status_service = self.create_service(
            Trigger, "/get_mission_status", self._handle_status_request
        )
        self.recovery_service = self.create_service(
            Trigger, "/recover_mission", self._handle_recovery_request
        )

        self.state = "UNKNOWN"
        self.detail = ""
        self.battery_percentage = None
        self.docking_status = "UNKNOWN"
        self.patrol_status = "UNKNOWN"
        self.marker_status = "UNKNOWN"
        self.full_charge_started_at = None
        self.dock_cycle_active = False
        self.dock_future = None
        self.undock_future = None
        self.patrol_recovery_future = None
        self.docking_recovery_attempts = 0
        self.patrol_recovery_attempts = 0
        self.next_dock_attempt_at = 0.0
        self.next_undock_attempt_at = 0.0
        self.next_patrol_recovery_at = 0.0
        self.force_recovery_requested = False
        self.started_at = time.monotonic()
        self.last_command_at = None
        self.navigation_command_active = False
        self.last_progress_pose = None
        self.last_progress_at = time.monotonic()
        self.last_amcl_at = None
        self.localization_position_std = None
        self.localization_yaw_std = None
        self.localization_healthy_since = None
        self.localization_recovery_attempts = 0
        self.localization_recovery_future = None
        self.localization_recovery_was_global = False
        self.next_localization_recovery_at = 0.0

        self._set_state(
            "INITIALIZING", "Waiting for patrol, battery, and docking status."
        )
        self.timer = self.create_timer(1.0 / update_frequency, self._tick)
        self.get_logger().info(
            "Mission manager ready: patrol, energy, docking, and recovery "
            "are coordinated through /mission_status."
        )

    def _publish_string(self, publisher, value):
        message = String()
        message.data = value
        publisher.publish(message)

    def _set_state(self, state, detail):
        changed = state != self.state
        detail_changed = detail != self.detail
        self.state = state
        self.detail = detail
        if changed:
            self._publish_string(self.status_publisher, state)
            self.get_logger().info(f"Mission state: {state} - {detail}")
        if changed or detail_changed:
            self._publish_string(self.detail_publisher, detail)

    def _battery_callback(self, battery):
        if math.isfinite(battery.percentage) and battery.percentage >= 0.0:
            self.battery_percentage = clamp(battery.percentage, 0.0, 1.0)

    def _docking_status_callback(self, message):
        state = message.data.strip().upper()
        if not state:
            return
        previous = self.docking_status
        self.docking_status = state

        if state in self.DOCKING_STATES or state in self.CHARGING_STATES:
            self.dock_cycle_active = True
        if state == "FULLY_CHARGED":
            self.dock_cycle_active = True
            if previous != state:
                self.full_charge_started_at = time.monotonic()
        else:
            self.full_charge_started_at = None

        if state == "IDLE" and self.battery_percentage is not None:
            if self.battery_percentage > self.low_battery_threshold:
                self.dock_cycle_active = False
                self.docking_recovery_attempts = 0

    def _patrol_status_callback(self, message):
        state = message.data.strip().upper()
        if state:
            self.patrol_status = state

    def _marker_status_callback(self, message):
        state = message.data.strip().upper()
        if state:
            self.marker_status = state

    def _odom_callback(self, odometry):
        position = odometry.pose.pose.position
        yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        pose = (position.x, position.y, yaw)
        if self.last_progress_pose is None:
            self.last_progress_pose = pose
            self.last_progress_at = time.monotonic()
            return

        old_x, old_y, old_yaw = self.last_progress_pose
        distance = math.hypot(pose[0] - old_x, pose[1] - old_y)
        yaw_change = abs(angle_difference(pose[2], old_yaw))
        if (
            distance >= self.progress_distance_epsilon
            or yaw_change >= self.progress_yaw_epsilon
        ):
            self.last_progress_pose = pose
            self.last_progress_at = time.monotonic()
            if self.patrol_status in {"PATROLLING", "RETURNING_HOME"}:
                self.patrol_recovery_attempts = 0

    def _command_callback(self, command):
        self.last_command_at = time.monotonic()
        self.navigation_command_active = (
            abs(command.linear.x) >= self.linear_command_threshold
            or abs(command.linear.y) >= self.linear_command_threshold
            or abs(command.angular.z) >= self.angular_command_threshold
        )
        if not self.navigation_command_active:
            self.last_progress_at = self.last_command_at

    def _amcl_callback(self, localization):
        covariance = localization.pose.covariance
        self.localization_position_std = math.sqrt(
            max(0.0, covariance[0], covariance[7])
        )
        self.localization_yaw_std = math.sqrt(max(0.0, covariance[35]))
        self.last_amcl_at = time.monotonic()
        if (
            self.localization_position_std
            < self.localization_position_std_limit * 0.70
            and self.localization_yaw_std
            < self.localization_yaw_std_limit * 0.70
        ):
            if self.localization_healthy_since is None:
                self.localization_healthy_since = self.last_amcl_at
        else:
            self.localization_healthy_since = None

    def _tick(self):
        now = time.monotonic()
        if (
            self.battery_percentage is None
            or self.docking_status == "UNKNOWN"
            or self.patrol_status == "UNKNOWN"
            or self.marker_status == "UNKNOWN"
        ):
            self._set_state(
                "INITIALIZING",
                "Waiting for patrol, battery, and docking status.",
            )
            return

        if self.docking_status in self.DOCKING_STATES:
            self._set_state(
                "DOCKING",
                f"Docking subsystem: {self.docking_status}; "
                f"marker: {self.marker_status}.",
            )
            return

        if self.docking_status in self.CHARGING_STATES:
            self._set_state(
                "CHARGING",
                f"Battery at {self.battery_percentage * 100.0:.1f}%.",
            )
            return

        if self.docking_status == "FULLY_CHARGED":
            self._set_state(
                "FULLY_CHARGED", "Charge complete; preparing to undock."
            )
            if self.full_charge_started_at is None:
                self.full_charge_started_at = now
            if (
                self.battery_percentage >= self.full_battery_threshold
                and now - self.full_charge_started_at
                >= self.full_charge_hold_seconds
                and now >= self.next_undock_attempt_at
            ):
                self._request_undocking()
            return

        if self.docking_status == "UNDOCKING":
            self._set_state(
                "UNDOCKING", "Leaving the dock for the staging pose."
            )
            return

        if self.docking_status == "ERROR":
            if self.dock_cycle_active or self._battery_is_low():
                self._recover_docking(now)
            else:
                self._set_state(
                    "ERROR", "Docking failed outside an active energy cycle."
                )
            return

        if self._battery_is_low():
            if (
                self.docking_recovery_attempts
                >= self.max_docking_recovery_attempts
            ):
                self._set_state(
                    "ERROR",
                    "Low-battery docking exhausted its recovery attempts.",
                )
            elif now >= self.next_dock_attempt_at:
                self._set_state(
                    "RETURNING_HOME",
                    "Battery is low; requesting the docking cycle.",
                )
                self._request_docking()
            return

        localization_issue = self._localization_issue(now)
        if localization_issue is not None:
            self._recover_localization(now, localization_issue)
            return
        if (
            self.localization_healthy_since is not None
            and now - self.localization_healthy_since
            >= self.localization_stable_reset_seconds
        ):
            self.localization_recovery_attempts = 0

        if self.force_recovery_requested:
            self.force_recovery_requested = False
            self._request_patrol_recovery("Manual mission recovery requested.")
            return

        if self.patrol_status == "ERROR":
            if (
                self.patrol_recovery_attempts
                >= self.max_patrol_recovery_attempts
            ):
                self._set_state(
                    "ERROR", "Patrol exhausted its recovery attempts."
                )
            elif now >= self.next_patrol_recovery_at:
                self._request_patrol_recovery(
                    "Patrol reported a navigation failure."
                )
            return

        if self._navigation_is_stuck(now):
            self._request_patrol_recovery(
                f"No odometry progress for {self.stuck_timeout:.1f} seconds."
            )
            return

        if self.patrol_status == "RECOVERY":
            self._set_state("RECOVERY", "Patrol is replanning its goal.")
        elif self.patrol_status == "RETURNING_HOME":
            self._set_state("RETURNING_HOME", "Navigating to the home pose.")
        elif self.patrol_status == "PATROLLING":
            self._set_state("PATROLLING", "Following the waypoint route.")
        elif self.patrol_status == "PAUSED":
            self._set_state("IDLE", "Patrol is paused.")
        elif self.patrol_status == "COMPLETED":
            self._set_state("IDLE", "The configured patrol is complete.")
        else:
            self._set_state("IDLE", f"Patrol subsystem: {self.patrol_status}.")

    def _battery_is_low(self):
        return (
            self.battery_percentage is not None
            and self.battery_percentage <= self.low_battery_threshold
        )

    def _navigation_is_stuck(self, now):
        if (
            self.patrol_status not in {"PATROLLING", "RETURNING_HOME"}
            or self.docking_status != "IDLE"
            or self.last_command_at is None
            or now - self.last_command_at > self.command_timeout
            or not self.navigation_command_active
            or self.patrol_recovery_future is not None
            or now < self.next_patrol_recovery_at
        ):
            return False
        return now - self.last_progress_at >= self.stuck_timeout

    def _localization_issue(self, now):
        if (
            self.patrol_status not in {"PATROLLING", "RETURNING_HOME"}
            or self.docking_status != "IDLE"
            or now - self.started_at < self.localization_startup_grace
        ):
            return None
        if self.last_amcl_at is None:
            return "AMCL has not published a localization estimate."
        age = now - self.last_amcl_at
        if age >= self.localization_timeout:
            return f"AMCL estimate is stale by {age:.1f} seconds."
        if (
            self.localization_position_std is not None
            and self.localization_position_std
            >= self.localization_position_std_limit
        ):
            return (
                "AMCL position uncertainty is "
                f"{self.localization_position_std:.2f} m."
            )
        if (
            self.localization_yaw_std is not None
            and self.localization_yaw_std >= self.localization_yaw_std_limit
        ):
            return (
                "AMCL yaw uncertainty is "
                f"{self.localization_yaw_std:.2f} rad."
            )
        return None

    def _recover_localization(self, now, reason):
        self.localization_healthy_since = None
        if self.localization_recovery_future is not None:
            self._set_state("RECOVERY", "Waiting for AMCL recovery response.")
            return
        if now < self.next_localization_recovery_at:
            self._set_state("RECOVERY", reason + " Waiting before retry.")
            return
        if (
            self.localization_recovery_attempts
            >= self.max_localization_recovery_attempts
        ):
            self._set_state(
                "ERROR",
                reason + " Localization recovery attempts were exhausted.",
            )
            return

        use_global = self.localization_recovery_attempts > 0
        client = (
            self.global_localization_client
            if use_global
            else self.nomotion_update_client
        )
        action = (
            "global AMCL relocalization"
            if use_global
            else "AMCL no-motion sensor update"
        )
        if not client.service_is_ready():
            self._set_state("RECOVERY", f"Waiting for {action} service.")
            return

        self.localization_recovery_attempts += 1
        self.localization_recovery_was_global = use_global
        self.next_localization_recovery_at = (
            now + self.localization_recovery_cooldown
        )
        self._set_state(
            "RECOVERY",
            f"{reason} Requesting {action} "
            f"({self.localization_recovery_attempts}/"
            f"{self.max_localization_recovery_attempts}).",
        )
        self.localization_recovery_future = client.call_async(Empty.Request())
        self.localization_recovery_future.add_done_callback(
            self._localization_recovery_response
        )

    def _localization_recovery_response(self, future):
        self.localization_recovery_future = None
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(
                f"Localization recovery service call failed: {error}"
            )
            return
        if self.localization_recovery_was_global:
            self.get_logger().warning(
                "Global AMCL relocalization requested; replanning patrol."
            )
            self._request_patrol_recovery(
                "Localization was globally reinitialized."
            )
        else:
            self.get_logger().warning(
                "AMCL no-motion update requested to reduce uncertainty."
            )

    def _request_docking(self):
        if self.dock_future is not None:
            return
        if not self.dock_client.service_is_ready():
            self._set_state(
                "RETURNING_HOME", "Waiting for the docking service."
            )
            return

        self.docking_recovery_attempts += 1
        self.dock_cycle_active = True
        self.next_dock_attempt_at = (
            time.monotonic() + self.recovery_retry_delay
        )
        self.dock_future = self.dock_client.call_async(Trigger.Request())
        self.dock_future.add_done_callback(self._dock_response)

    def _dock_response(self, future):
        self.dock_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Docking service call failed: {error}")
            return
        if response.success:
            self.get_logger().info(
                f"Mission docking request accepted: {response.message}"
            )
        else:
            self.get_logger().error(
                f"Mission docking request rejected: {response.message}"
            )

    def _recover_docking(self, now):
        if (
            self.docking_recovery_attempts
            >= self.max_docking_recovery_attempts
        ):
            self._set_state(
                "ERROR", "Docking exhausted its recovery attempts."
            )
            return
        self._set_state("RECOVERY", "Retrying the failed docking cycle.")
        if now >= self.next_dock_attempt_at:
            self._request_docking()

    def _request_undocking(self):
        if self.undock_future is not None:
            return
        if not self.undock_client.service_is_ready():
            self._set_state(
                "FULLY_CHARGED", "Waiting for the undocking service."
            )
            return

        self.next_undock_attempt_at = (
            time.monotonic() + self.recovery_retry_delay
        )
        self.undock_future = self.undock_client.call_async(Trigger.Request())
        self.undock_future.add_done_callback(self._undock_response)

    def _undock_response(self, future):
        self.undock_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Undocking service call failed: {error}")
            return
        if response.success:
            self.get_logger().info(
                f"Mission undocking request accepted: {response.message}"
            )
        else:
            self.get_logger().error(
                f"Mission undocking request rejected: {response.message}"
            )

    def _request_patrol_recovery(self, reason):
        if self.patrol_recovery_future is not None:
            return
        if not self.patrol_recovery_client.service_is_ready():
            self._set_state("RECOVERY", "Waiting for patrol recovery service.")
            return

        self.patrol_recovery_attempts += 1
        self.next_patrol_recovery_at = (
            time.monotonic() + self.recovery_retry_delay
        )
        self.last_progress_at = time.monotonic()
        self._set_state(
            "RECOVERY",
            f"{reason} Attempt {self.patrol_recovery_attempts}/"
            f"{self.max_patrol_recovery_attempts}.",
        )
        self.patrol_recovery_future = self.patrol_recovery_client.call_async(
            Trigger.Request()
        )
        self.patrol_recovery_future.add_done_callback(
            self._patrol_recovery_response
        )

    def _patrol_recovery_response(self, future):
        self.patrol_recovery_future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"Patrol recovery service call failed: {error}"
            )
            return
        if response.success:
            self.get_logger().warning(
                f"Patrol recovery accepted: {response.message}"
            )
        else:
            self.get_logger().error(
                f"Patrol recovery rejected: {response.message}"
            )

    def _handle_status_request(self, request, response):
        del request
        summary = {
            "state": self.state,
            "detail": self.detail,
            "battery_percentage": self.battery_percentage,
            "patrol_status": self.patrol_status,
            "docking_status": self.docking_status,
            "dock_marker_status": self.marker_status,
            "patrol_recovery_attempts": self.patrol_recovery_attempts,
            "docking_recovery_attempts": self.docking_recovery_attempts,
            "localization_position_std": self.localization_position_std,
            "localization_yaw_std": self.localization_yaw_std,
            "localization_recovery_attempts": (
                self.localization_recovery_attempts
            ),
        }
        response.success = self.state != "ERROR"
        response.message = json.dumps(summary, separators=(",", ":"))
        return response

    def _handle_recovery_request(self, request, response):
        del request
        if (
            self.docking_status in self.DOCKING_STATES
            or self.docking_status in self.CHARGING_STATES
            or self.docking_status in {"FULLY_CHARGED", "UNDOCKING"}
        ):
            response.success = False
            response.message = (
                "Manual recovery is unavailable during an active docking cycle."
            )
            return response

        self.patrol_recovery_attempts = 0
        self.docking_recovery_attempts = 0
        self.localization_recovery_attempts = 0
        self.next_dock_attempt_at = 0.0
        self.next_patrol_recovery_at = 0.0
        self.next_localization_recovery_at = 0.0
        if self.docking_status == "ERROR":
            self.dock_cycle_active = True
        else:
            self.force_recovery_requested = True
        response.success = True
        response.message = "Mission recovery counters reset; retry requested."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
