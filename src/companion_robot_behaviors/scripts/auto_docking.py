#!/usr/bin/env python3

"""Navigate to a staging pose and reverse precisely into the simulated dock."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle):
    """Return an angle in the [-pi, pi] range."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class AutoDocking(Node):
    IDLE = "IDLE"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    NAVIGATING_TO_STAGING = "NAVIGATING_TO_STAGING"
    ALIGNING_WITH_DOCK = "ALIGNING_WITH_DOCK"
    PRECISION_DOCKING = "PRECISION_DOCKING"
    DOCKED = "DOCKED"
    CHARGING = "CHARGING"
    FULLY_CHARGED = "FULLY_CHARGED"
    UNDOCKING = "UNDOCKING"
    ERROR = "ERROR"

    def __init__(self):
        super().__init__("docking_behavior")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("staging_pose", [0.0, -1.70, 1.5708])
        self.declare_parameter("dock_pose", [0.0, -2.55, 1.5708])
        self.declare_parameter("approach_direction", "reverse")
        self.declare_parameter("controller_frequency", 20.0)
        self.declare_parameter("nav2_server_timeout", 30.0)
        self.declare_parameter("nav2_handoff_delay", 1.0)
        self.declare_parameter("max_reverse_speed", 0.08)
        self.declare_parameter("min_reverse_speed", 0.025)
        self.declare_parameter("max_forward_speed", 0.10)
        self.declare_parameter("min_forward_speed", 0.03)
        self.declare_parameter("max_angular_speed", 0.30)
        self.declare_parameter("linear_gain", 0.60)
        self.declare_parameter("angular_gain", 1.80)
        self.declare_parameter("lateral_gain", 3.00)
        self.declare_parameter("max_lateral_heading_correction", 0.30)
        self.declare_parameter("heading_stop_angle", 0.15)
        self.declare_parameter("alignment_yaw_tolerance", 0.04)
        self.declare_parameter("alignment_timeout", 10.0)
        self.declare_parameter("precision_docking_timeout", 45.0)
        self.declare_parameter("progress_timeout", 6.0)
        self.declare_parameter("progress_distance_epsilon", 0.01)
        self.declare_parameter("dock_position_tolerance", 0.05)
        self.declare_parameter("dock_yaw_tolerance", 0.18)
        self.declare_parameter("contact_position_tolerance", 0.09)
        self.declare_parameter("staging_position_tolerance", 0.08)
        self.declare_parameter("rear_sector_half_angle", 0.35)
        self.declare_parameter("rear_stop_distance", 0.15)
        self.declare_parameter("front_sector_half_angle", 0.35)
        self.declare_parameter("front_self_filter_distance", 0.16)
        self.declare_parameter("front_stop_distance", 0.28)
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("min_undock_battery", 0.50)

        self.frame_id = self.get_parameter("frame_id").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.staging_pose = self._pose_parameter("staging_pose")
        self.dock_pose = self._pose_parameter("dock_pose")
        self.approach_direction = self.get_parameter(
            "approach_direction"
        ).value
        if self.approach_direction != "reverse":
            raise ValueError("Only reverse docking is currently supported.")

        self.controller_frequency = max(
            1.0, float(self.get_parameter("controller_frequency").value)
        )
        self.nav2_server_timeout = max(
            1.0, float(self.get_parameter("nav2_server_timeout").value)
        )
        self.nav2_handoff_delay = max(
            0.0, float(self.get_parameter("nav2_handoff_delay").value)
        )
        self.max_reverse_speed = max(
            0.01, float(self.get_parameter("max_reverse_speed").value)
        )
        self.min_reverse_speed = clamp(
            float(self.get_parameter("min_reverse_speed").value),
            0.0,
            self.max_reverse_speed,
        )
        self.max_forward_speed = max(
            0.01, float(self.get_parameter("max_forward_speed").value)
        )
        self.min_forward_speed = clamp(
            float(self.get_parameter("min_forward_speed").value),
            0.0,
            self.max_forward_speed,
        )
        self.max_angular_speed = max(
            0.01, float(self.get_parameter("max_angular_speed").value)
        )
        self.linear_gain = max(
            0.0, float(self.get_parameter("linear_gain").value)
        )
        self.angular_gain = max(
            0.0, float(self.get_parameter("angular_gain").value)
        )
        self.lateral_gain = max(
            0.0, float(self.get_parameter("lateral_gain").value)
        )
        self.max_lateral_heading_correction = clamp(
            float(
                self.get_parameter(
                    "max_lateral_heading_correction"
                ).value
            ),
            0.05,
            math.pi / 4.0,
        )
        self.heading_stop_angle = max(
            0.05, float(self.get_parameter("heading_stop_angle").value)
        )
        self.alignment_yaw_tolerance = max(
            0.01,
            float(self.get_parameter("alignment_yaw_tolerance").value),
        )
        self.alignment_timeout = max(
            1.0, float(self.get_parameter("alignment_timeout").value)
        )
        self.precision_docking_timeout = max(
            5.0,
            float(self.get_parameter("precision_docking_timeout").value),
        )
        self.progress_timeout = max(
            1.0, float(self.get_parameter("progress_timeout").value)
        )
        self.progress_distance_epsilon = max(
            0.001,
            float(self.get_parameter("progress_distance_epsilon").value),
        )
        self.dock_position_tolerance = max(
            0.01,
            float(self.get_parameter("dock_position_tolerance").value),
        )
        self.dock_yaw_tolerance = max(
            0.01, float(self.get_parameter("dock_yaw_tolerance").value)
        )
        self.contact_position_tolerance = max(
            self.dock_position_tolerance,
            float(self.get_parameter("contact_position_tolerance").value),
        )
        self.staging_position_tolerance = max(
            0.01,
            float(self.get_parameter("staging_position_tolerance").value),
        )
        self.rear_sector_half_angle = clamp(
            float(self.get_parameter("rear_sector_half_angle").value),
            0.05,
            math.pi / 2.0,
        )
        self.rear_stop_distance = max(
            0.05, float(self.get_parameter("rear_stop_distance").value)
        )
        self.front_sector_half_angle = clamp(
            float(self.get_parameter("front_sector_half_angle").value),
            0.05,
            math.pi / 2.0,
        )
        self.front_self_filter_distance = max(
            0.0,
            float(self.get_parameter("front_self_filter_distance").value),
        )
        self.front_stop_distance = max(
            0.05, float(self.get_parameter("front_stop_distance").value)
        )
        self.sensor_timeout = max(
            0.1, float(self.get_parameter("sensor_timeout").value)
        )
        self.min_undock_battery = clamp(
            float(self.get_parameter("min_undock_battery").value),
            0.0,
            1.0,
        )

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, "/docking_status", status_qos
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, 10
        )
        self.battery_subscription = self.create_subscription(
            BatteryState, "/battery_state", self._battery_callback, 10
        )
        self.dock_service = self.create_service(
            Trigger, "/dock_robot", self._handle_dock_request
        )
        self.undock_service = self.create_service(
            Trigger, "/undock_robot", self._handle_undock_request
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = self.IDLE
        self.state_started_at = time.monotonic()
        self.last_scan_at = None
        self.rear_range = math.inf
        self.front_range = math.inf
        self.battery_percentage = None
        self.active_goal_handle = None
        self.last_feedback_log_at = 0.0
        self.last_control_log_at = 0.0
        self.best_dock_distance = math.inf
        self.last_dock_progress_at = time.monotonic()

        self.control_timer = self.create_timer(
            1.0 / self.controller_frequency, self._control_loop
        )
        self._publish_status()
        self.get_logger().info(
            "Docking manager ready. Use /dock_robot or /undock_robot."
        )

    def _pose_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 3:
            raise ValueError(f"Parameter '{name}' must contain x, y, yaw.")
        return tuple(values)

    def _set_state(self, state, message=None):
        if self.state == state and message is None:
            return
        self.state = state
        self.state_started_at = time.monotonic()
        if state == self.PRECISION_DOCKING:
            self.best_dock_distance = math.inf
            self.last_dock_progress_at = self.state_started_at
        self._publish_status()
        if message:
            self.get_logger().info(message)

    def _publish_status(self):
        message = String()
        message.data = self.state
        self.status_publisher.publish(message)

    def _handle_dock_request(self, request, response):
        del request
        if self.state in (self.DOCKED, self.CHARGING, self.FULLY_CHARGED):
            response.success = True
            response.message = "Robot is already docked."
            return response

        if self.state not in (self.IDLE, self.ERROR):
            response.success = False
            response.message = f"Docking is already active: {self.state}."
            return response

        self.active_goal_handle = None
        self._set_state(
            self.WAITING_FOR_NAV2,
            "Docking requested; waiting for Nav2.",
        )
        response.success = True
        response.message = "Docking request accepted."
        return response

    def _handle_undock_request(self, request, response):
        del request
        if self.state not in (
            self.DOCKED,
            self.CHARGING,
            self.FULLY_CHARGED,
        ):
            response.success = False
            response.message = f"Robot cannot undock from state {self.state}."
            return response

        if (
            self.battery_percentage is not None
            and self.battery_percentage < self.min_undock_battery
        ):
            response.success = False
            response.message = (
                "Battery is too low to undock: "
                f"{self.battery_percentage * 100.0:.1f}% available, "
                f"{self.min_undock_battery * 100.0:.1f}% required."
            )
            return response

        self._stop_robot()
        self._set_state(
            self.UNDOCKING,
            "Undocking requested; moving forward to the staging pose.",
        )
        response.success = True
        response.message = "Undocking request accepted."
        return response

    def _battery_callback(self, battery):
        if math.isfinite(battery.percentage) and battery.percentage >= 0.0:
            self.battery_percentage = battery.percentage

        if (
            self.state == self.DOCKED
            and battery.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        ):
            self._set_state(self.CHARGING, "Charging started.")
        elif (
            self.state in (self.DOCKED, self.CHARGING)
            and battery.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_FULL
        ):
            self._set_state(
                self.FULLY_CHARGED,
                "Battery fully charged; robot remains docked.",
            )

    def _control_loop(self):
        if self.state == self.WAITING_FOR_NAV2:
            self._wait_for_nav2()
        elif self.state == self.ALIGNING_WITH_DOCK:
            self._run_dock_alignment()
        elif self.state == self.PRECISION_DOCKING:
            self._run_precision_docking()
        elif self.state == self.UNDOCKING:
            self._run_precision_undocking()

    def _wait_for_nav2(self):
        elapsed = time.monotonic() - self.state_started_at
        if elapsed < self.nav2_handoff_delay:
            return

        if self.action_client.server_is_ready():
            self._send_staging_goal()
            return

        if elapsed > self.nav2_server_timeout:
            self._fail("Nav2 did not become ready before the timeout.")

    def _send_staging_goal(self):
        x, y, yaw = self.staging_pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._set_state(
            self.NAVIGATING_TO_STAGING,
            f"Sending staging goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}",
        )
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback
        )
        future.add_done_callback(self._staging_goal_response)

    def _staging_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy futures surface middleware errors.
            self._fail(f"Could not send staging goal: {error}")
            return

        if not goal_handle.accepted:
            self._fail("Nav2 rejected the staging goal.")
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._staging_result)

    def _navigation_feedback(self, feedback_message):
        now = time.monotonic()
        if now - self.last_feedback_log_at < 2.0:
            return
        self.last_feedback_log_at = now
        distance = feedback_message.feedback.distance_remaining
        self.get_logger().info(
            f"Distance to staging pose: {distance:.2f} m"
        )

    def _staging_result(self, future):
        self.active_goal_handle = None
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._fail(f"Staging navigation failed: {error}")
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            detail = wrapped_result.result.error_msg or "navigation failed"
            self._fail(f"Could not reach staging pose: {detail}")
            return

        self._stop_robot()
        self._set_state(
            self.ALIGNING_WITH_DOCK,
            "Staging pose reached; aligning with the dock centreline.",
        )

    def _scan_callback(self, scan):
        rear_closest = math.inf
        front_closest = math.inf
        angle = scan.angle_min
        for distance in scan.ranges:
            rear_error = abs(normalize_angle(angle - math.pi))
            front_error = abs(normalize_angle(angle))
            if (
                rear_error <= self.rear_sector_half_angle
                and math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
            ):
                rear_closest = min(rear_closest, distance)
            if (
                front_error <= self.front_sector_half_angle
                and math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
                and distance > self.front_self_filter_distance
            ):
                front_closest = min(front_closest, distance)
            angle += scan.angle_increment

        self.rear_range = rear_closest
        self.front_range = front_closest
        self.last_scan_at = time.monotonic()

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for {self.frame_id} -> {self.robot_frame} TF: {error}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sin_yaw = 2.0 * (
            rotation.w * rotation.z + rotation.x * rotation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z
        )
        yaw = math.atan2(sin_yaw, cos_yaw)
        return translation.x, translation.y, yaw

    def _run_precision_docking(self):
        if (
            self.last_scan_at is None
            or time.monotonic() - self.last_scan_at > self.sensor_timeout
        ):
            self._stop_robot()
            if time.monotonic() - self.state_started_at > self.sensor_timeout:
                self._fail("Rear LiDAR data is unavailable; docking stopped.")
            return

        pose = self._robot_pose()
        if pose is None:
            self._stop_robot()
            if time.monotonic() - self.state_started_at > self.sensor_timeout:
                self._fail("Robot pose is unavailable; docking stopped.")
            return

        x, y, yaw = pose
        dock_x, dock_y, dock_yaw = self.dock_pose
        delta_x = dock_x - x
        delta_y = dock_y - y
        distance = math.hypot(delta_x, delta_y)
        final_yaw_error = normalize_angle(dock_yaw - yaw)
        now = time.monotonic()

        if now - self.state_started_at > self.precision_docking_timeout:
            self._fail("Precision docking exceeded its time limit.")
            return

        if distance < (
            self.best_dock_distance - self.progress_distance_epsilon
        ):
            self.best_dock_distance = distance
            self.last_dock_progress_at = now
        elif now - self.last_dock_progress_at > self.progress_timeout:
            self._fail(
                "Precision docking is not making progress; "
                "the robot may be blocked by a guide."
            )
            return

        if (
            distance <= self.dock_position_tolerance
            and abs(final_yaw_error) <= self.dock_yaw_tolerance
        ):
            self._complete_docking(distance, final_yaw_error)
            return

        if self.rear_range <= self.rear_stop_distance:
            if (
                distance <= self.contact_position_tolerance
                and abs(final_yaw_error) <= self.dock_yaw_tolerance
            ):
                self._complete_docking(distance, final_yaw_error)
            else:
                self._fail(
                    "Rear obstacle detected before the expected dock position."
                )
            return

        dock_forward_x = math.cos(dock_yaw)
        dock_forward_y = math.sin(dock_yaw)
        dock_left_x = -dock_forward_y
        dock_left_y = dock_forward_x
        offset_x = x - dock_x
        offset_y = y - dock_y
        longitudinal_error = (
            offset_x * dock_forward_x + offset_y * dock_forward_y
        )
        lateral_error = offset_x * dock_left_x + offset_y * dock_left_y

        if longitudinal_error < -self.contact_position_tolerance:
            self._fail("Robot passed behind the configured dock pose.")
            return

        lateral_correction = clamp(
            self.lateral_gain * lateral_error,
            -self.max_lateral_heading_correction,
            self.max_lateral_heading_correction,
        )
        desired_robot_yaw = normalize_angle(dock_yaw + lateral_correction)
        heading_error = normalize_angle(desired_robot_yaw - yaw)

        speed = clamp(
            self.linear_gain * distance,
            self.min_reverse_speed,
            self.max_reverse_speed,
        )
        linear_x = -speed
        if abs(heading_error) > self.heading_stop_angle:
            linear_x = 0.0

        angular_z = clamp(
            self.angular_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_vel_publisher.publish(command)

        if now - self.last_control_log_at >= 1.0:
            self.last_control_log_at = now
            self.get_logger().info(
                "Dock approach: "
                f"distance={distance:.2f} m, "
                f"lateral_error={lateral_error:.2f} m, "
                f"heading_error={heading_error:.2f} rad, "
                f"rear_range={self.rear_range:.2f} m"
            )

    def _run_dock_alignment(self):
        pose = self._robot_pose()
        if pose is None:
            self._stop_robot()
            if time.monotonic() - self.state_started_at > self.sensor_timeout:
                self._fail("Robot pose is unavailable; alignment stopped.")
            return

        _, _, yaw = pose
        dock_yaw = self.dock_pose[2]
        yaw_error = normalize_angle(dock_yaw - yaw)
        if abs(yaw_error) <= self.alignment_yaw_tolerance:
            self._stop_robot()
            self._set_state(
                self.PRECISION_DOCKING,
                "Dock alignment complete; starting slow reverse approach.",
            )
            return

        if time.monotonic() - self.state_started_at > self.alignment_timeout:
            self._fail("Could not align with the dock before the timeout.")
            return

        command = Twist()
        command.angular.z = clamp(
            self.angular_gain * yaw_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.cmd_vel_publisher.publish(command)

        now = time.monotonic()
        if now - self.last_control_log_at >= 1.0:
            self.last_control_log_at = now
            self.get_logger().info(
                f"Dock alignment: yaw_error={yaw_error:.2f} rad"
            )

    def _run_precision_undocking(self):
        if (
            self.last_scan_at is None
            or time.monotonic() - self.last_scan_at > self.sensor_timeout
        ):
            self._stop_robot()
            if time.monotonic() - self.state_started_at > self.sensor_timeout:
                self._fail("Front LiDAR data is unavailable; undocking stopped.")
            return

        pose = self._robot_pose()
        if pose is None:
            self._stop_robot()
            if time.monotonic() - self.state_started_at > self.sensor_timeout:
                self._fail("Robot pose is unavailable; undocking stopped.")
            return

        x, y, yaw = pose
        staging_x, staging_y, staging_yaw = self.staging_pose
        delta_x = staging_x - x
        delta_y = staging_y - y
        distance = math.hypot(delta_x, delta_y)
        final_yaw_error = normalize_angle(staging_yaw - yaw)

        if (
            distance <= self.staging_position_tolerance
            and abs(final_yaw_error) <= self.dock_yaw_tolerance
        ):
            self._complete_undocking(distance, final_yaw_error)
            return

        if self.front_range <= self.front_stop_distance:
            self._fail("Front obstacle detected; undocking stopped.")
            return

        travel_heading = math.atan2(delta_y, delta_x)
        heading_error = normalize_angle(travel_heading - yaw)
        speed = clamp(
            self.linear_gain * distance,
            self.min_forward_speed,
            self.max_forward_speed,
        )
        linear_x = speed
        if abs(heading_error) > self.heading_stop_angle:
            linear_x = 0.0

        angular_z = clamp(
            self.angular_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_vel_publisher.publish(command)

        now = time.monotonic()
        if now - self.last_control_log_at >= 1.0:
            self.last_control_log_at = now
            self.get_logger().info(
                "Undock approach: "
                f"distance={distance:.2f} m, "
                f"heading_error={heading_error:.2f} rad, "
                f"front_range={self.front_range:.2f} m"
            )

    def _complete_docking(self, distance, yaw_error):
        self._stop_robot()
        self._set_state(
            self.DOCKED,
            "Docking complete: "
            f"position_error={distance:.3f} m, "
            f"yaw_error={yaw_error:.3f} rad. Status: DOCKED.",
        )

    def _complete_undocking(self, distance, yaw_error):
        self._stop_robot()
        self._set_state(
            self.IDLE,
            "Undocking complete: "
            f"position_error={distance:.3f} m, "
            f"yaw_error={yaw_error:.3f} rad. Status: IDLE.",
        )

    def _fail(self, message):
        self._stop_robot()
        self._set_state(self.ERROR)
        self.get_logger().error(message)

    def _stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def stop(self):
        self._stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = AutoDocking()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            # SIGINT may invalidate the ROS context before the final zero
            # velocity command can be published.
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
