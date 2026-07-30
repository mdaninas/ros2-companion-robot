#!/usr/bin/env python3

"""Predict and socially follow a camera-and-LiDAR person track with Nav2."""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_difference(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class HumanFollower(Node):
    def __init__(self):
        super().__init__("human_follower")

        self.declare_parameter("target_pose_topic", "/person_detection/pose")
        self.declare_parameter(
            "target_visible_topic", "/person_detection/visible"
        )
        self.declare_parameter(
            "target_identity_topic",
            "/person_detection/selected_identity",
        )
        self.declare_parameter(
            "direct_command_topic", "/cmd_vel_behavior_raw"
        )
        self.declare_parameter(
            "direct_control_active_topic", "/cmd_vel_behavior_active"
        )
        self.declare_parameter(
            "safety_status_topic", "/collision_safety/status"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("autostart", True)
        self.declare_parameter("follow_distance", 0.90)
        self.declare_parameter("minimum_distance", 0.75)
        self.declare_parameter("retreat_release_distance", 0.84)
        self.declare_parameter("distance_tolerance", 0.12)
        self.declare_parameter("face_yaw_tolerance", 0.12)
        self.declare_parameter("face_before_follow_yaw", 0.18)
        self.declare_parameter("face_angular_gain", 1.8)
        self.declare_parameter("max_face_angular_speed", 0.45)
        self.declare_parameter("retreat_linear_speed", 0.10)
        self.declare_parameter("retreat_angular_gain", 1.4)
        self.declare_parameter("max_retreat_angular_speed", 0.45)
        self.declare_parameter("retreat_blocked_timeout", 0.60)
        self.declare_parameter("retreat_escape_duration", 1.00)
        self.declare_parameter("retreat_escape_angular_speed", 0.30)
        self.declare_parameter("target_timeout", 1.5)
        self.declare_parameter("target_search_timeout", 15.0)
        self.declare_parameter("search_last_seen_duration", 3.0)
        self.declare_parameter("search_sweep_period", 2.5)
        self.declare_parameter("search_angular_speed", 0.28)
        self.declare_parameter("search_yaw_tolerance", 0.10)
        self.declare_parameter("reacquire_pause_duration", 3.0)
        self.declare_parameter("reacquire_sweep_duration", 24.0)
        self.declare_parameter("predictive_following_enabled", True)
        self.declare_parameter("velocity_filter_alpha", 0.28)
        self.declare_parameter("velocity_minimum_sample_dt", 0.08)
        self.declare_parameter("velocity_reset_timeout", 1.20)
        self.declare_parameter("maximum_person_speed", 0.65)
        self.declare_parameter("minimum_prediction_speed", 0.06)
        self.declare_parameter("prediction_horizon", 0.85)
        self.declare_parameter("lost_target_prediction_horizon", 1.40)
        self.declare_parameter("maximum_prediction_distance", 0.35)
        self.declare_parameter("motion_samples_for_confidence", 4)
        self.declare_parameter("motion_trailing_weight", 0.75)
        self.declare_parameter("adaptive_distance_speed_gain", 0.60)
        self.declare_parameter("maximum_follow_distance", 0.90)
        self.declare_parameter("social_yield_enabled", True)
        self.declare_parameter("social_yield_distance", 1.10)
        self.declare_parameter("social_yield_release_distance", 1.30)
        self.declare_parameter("social_yield_closing_speed", 0.08)
        self.declare_parameter("social_yield_release_speed", 0.03)
        self.declare_parameter("goal_update_interval", 1.0)
        self.declare_parameter("goal_update_distance", 0.20)
        self.declare_parameter("goal_update_yaw", 0.15)
        self.declare_parameter("minimum_goal_displacement", 0.10)
        self.declare_parameter("navigation_retry_delay", 1.5)
        self.declare_parameter("maximum_goal_failures", 5)
        self.declare_parameter("update_frequency", 5.0)

        self.target_pose_topic = str(
            self.get_parameter("target_pose_topic").value
        )
        self.target_visible_topic = str(
            self.get_parameter("target_visible_topic").value
        )
        self.target_identity_topic = str(
            self.get_parameter("target_identity_topic").value
        )
        self.direct_command_topic = str(
            self.get_parameter("direct_command_topic").value
        )
        self.direct_control_active_topic = str(
            self.get_parameter("direct_control_active_topic").value
        )
        self.safety_status_topic = str(
            self.get_parameter("safety_status_topic").value
        )
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.enabled = bool(self.get_parameter("autostart").value)
        self.follow_distance = max(
            0.35, float(self.get_parameter("follow_distance").value)
        )
        self.minimum_distance = clamp(
            float(self.get_parameter("minimum_distance").value),
            0.30,
            self.follow_distance - 0.05,
        )
        self.retreat_release_distance = clamp(
            float(self.get_parameter("retreat_release_distance").value),
            self.minimum_distance + 0.03,
            self.follow_distance,
        )
        self.distance_tolerance = max(
            0.03, float(self.get_parameter("distance_tolerance").value)
        )
        self.face_yaw_tolerance = max(
            0.04, float(self.get_parameter("face_yaw_tolerance").value)
        )
        self.face_before_follow_yaw = max(
            self.face_yaw_tolerance,
            float(self.get_parameter("face_before_follow_yaw").value),
        )
        self.face_angular_gain = max(
            0.2, float(self.get_parameter("face_angular_gain").value)
        )
        self.max_face_angular_speed = clamp(
            float(self.get_parameter("max_face_angular_speed").value),
            0.10,
            0.65,
        )
        self.retreat_linear_speed = clamp(
            float(self.get_parameter("retreat_linear_speed").value),
            0.04,
            0.15,
        )
        self.retreat_angular_gain = max(
            0.2, float(self.get_parameter("retreat_angular_gain").value)
        )
        self.max_retreat_angular_speed = clamp(
            float(self.get_parameter("max_retreat_angular_speed").value),
            0.10,
            0.65,
        )
        self.retreat_blocked_timeout = max(
            0.20,
            float(self.get_parameter("retreat_blocked_timeout").value),
        )
        self.retreat_escape_duration = max(
            0.40,
            float(self.get_parameter("retreat_escape_duration").value),
        )
        self.retreat_escape_angular_speed = clamp(
            float(
                self.get_parameter("retreat_escape_angular_speed").value
            ),
            0.15,
            0.45,
        )
        self.target_timeout = max(
            0.3, float(self.get_parameter("target_timeout").value)
        )
        self.target_search_timeout = max(
            2.0, float(self.get_parameter("target_search_timeout").value)
        )
        self.search_last_seen_duration = clamp(
            float(
                self.get_parameter("search_last_seen_duration").value
            ),
            0.5,
            self.target_search_timeout,
        )
        self.search_sweep_period = max(
            0.75, float(self.get_parameter("search_sweep_period").value)
        )
        self.search_angular_speed = clamp(
            float(self.get_parameter("search_angular_speed").value),
            0.12,
            0.45,
        )
        self.search_yaw_tolerance = max(
            0.04,
            float(self.get_parameter("search_yaw_tolerance").value),
        )
        self.reacquire_pause_duration = max(
            0.5,
            float(self.get_parameter("reacquire_pause_duration").value),
        )
        minimum_full_sweep_duration = (
            2.0 * math.pi / self.search_angular_speed + 1.0
        )
        self.reacquire_sweep_duration = max(
            minimum_full_sweep_duration,
            float(self.get_parameter("reacquire_sweep_duration").value),
        )
        self.predictive_following_enabled = bool(
            self.get_parameter("predictive_following_enabled").value
        )
        self.velocity_filter_alpha = clamp(
            float(self.get_parameter("velocity_filter_alpha").value),
            0.05,
            1.0,
        )
        self.velocity_minimum_sample_dt = max(
            0.02,
            float(
                self.get_parameter("velocity_minimum_sample_dt").value
            ),
        )
        self.velocity_reset_timeout = max(
            self.velocity_minimum_sample_dt * 2.0,
            float(self.get_parameter("velocity_reset_timeout").value),
        )
        self.maximum_person_speed = max(
            0.10, float(self.get_parameter("maximum_person_speed").value)
        )
        self.minimum_prediction_speed = clamp(
            float(self.get_parameter("minimum_prediction_speed").value),
            0.01,
            self.maximum_person_speed,
        )
        self.prediction_horizon = clamp(
            float(self.get_parameter("prediction_horizon").value),
            0.0,
            2.0,
        )
        self.lost_target_prediction_horizon = clamp(
            float(
                self.get_parameter("lost_target_prediction_horizon").value
            ),
            0.0,
            3.0,
        )
        self.maximum_prediction_distance = clamp(
            float(
                self.get_parameter("maximum_prediction_distance").value
            ),
            0.0,
            1.0,
        )
        self.motion_samples_for_confidence = max(
            1,
            int(
                self.get_parameter("motion_samples_for_confidence").value
            ),
        )
        self.motion_trailing_weight = clamp(
            float(self.get_parameter("motion_trailing_weight").value),
            0.0,
            1.0,
        )
        self.adaptive_distance_speed_gain = max(
            0.0,
            float(
                self.get_parameter("adaptive_distance_speed_gain").value
            ),
        )
        self.maximum_follow_distance = max(
            self.follow_distance,
            float(self.get_parameter("maximum_follow_distance").value),
        )
        self.social_yield_enabled = bool(
            self.get_parameter("social_yield_enabled").value
        )
        self.social_yield_distance = max(
            self.minimum_distance,
            float(self.get_parameter("social_yield_distance").value),
        )
        self.social_yield_release_distance = max(
            self.social_yield_distance + 0.05,
            float(
                self.get_parameter("social_yield_release_distance").value
            ),
        )
        self.social_yield_closing_speed = max(
            0.02,
            float(
                self.get_parameter("social_yield_closing_speed").value
            ),
        )
        self.social_yield_release_speed = clamp(
            float(
                self.get_parameter("social_yield_release_speed").value
            ),
            0.0,
            self.social_yield_closing_speed,
        )
        self.goal_update_interval = max(
            0.3, float(self.get_parameter("goal_update_interval").value)
        )
        self.goal_update_distance = max(
            0.05, float(self.get_parameter("goal_update_distance").value)
        )
        self.goal_update_yaw = max(
            0.05, float(self.get_parameter("goal_update_yaw").value)
        )
        self.minimum_goal_displacement = max(
            0.03,
            float(self.get_parameter("minimum_goal_displacement").value),
        )
        self.navigation_retry_delay = max(
            0.5,
            float(self.get_parameter("navigation_retry_delay").value),
        )
        self.maximum_goal_failures = max(
            1, int(self.get_parameter("maximum_goal_failures").value)
        )
        update_frequency = max(
            1.0, float(self.get_parameter("update_frequency").value)
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.status_publisher = self.create_publisher(
            String, "/human_following/status", latched_qos
        )
        self.visualization_publisher = self.create_publisher(
            MarkerArray, "/human_following/visualization", latched_qos
        )
        self.predicted_target_publisher = self.create_publisher(
            PoseStamped, "/human_following/predicted_target", 10
        )
        self.target_velocity_publisher = self.create_publisher(
            TwistStamped, "/human_following/target_velocity", 10
        )
        self.effective_distance_publisher = self.create_publisher(
            Float32, "/human_following/effective_distance", 10
        )
        self.direct_command_publisher = self.create_publisher(
            Twist, self.direct_command_topic, 10
        )
        self.direct_control_active_publisher = self.create_publisher(
            Bool, self.direct_control_active_topic, latched_qos
        )
        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            self.target_pose_topic,
            self._target_pose_callback,
            10,
        )
        self.target_visible_subscription = self.create_subscription(
            Bool,
            self.target_visible_topic,
            self._target_visible_callback,
            10,
        )
        self.target_identity_subscription = self.create_subscription(
            String,
            self.target_identity_topic,
            self._target_identity_callback,
            latched_qos,
        )
        self.safety_status_subscription = self.create_subscription(
            String,
            self.safety_status_topic,
            self._safety_status_callback,
            latched_qos,
        )

        self.start_service = self.create_service(
            Trigger, "/start_human_following", self._handle_start
        )
        self.stop_service = self.create_service(
            Trigger, "/stop_human_following", self._handle_stop
        )
        self.status_service = self.create_service(
            Trigger, "/get_human_following_status", self._handle_status
        )

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status = "UNKNOWN"
        self.selected_identity = "unknown"
        self.target_visible = False
        self.last_target_pose = None
        self.last_target_at = None
        self.last_target_distance = None
        self.last_target_bearing = None
        self.last_target_map = None
        self.predicted_target_map = None
        self.effective_follow_distance = self.follow_distance
        self.target_velocity_map = (0.0, 0.0)
        self.target_speed = 0.0
        self.motion_confidence = 0.0
        self.motion_sample_count = 0
        self.last_motion_sample_map = None
        self.last_motion_sample_time = None
        self.last_motion_sample_at = None
        self.target_pose_sequence = 0
        self.processed_target_pose_sequence = -1
        self.closing_speed = 0.0
        self.social_yield_active = False
        self.social_mode = "STATIONARY"
        self.last_goal = None
        self.last_goal_mode = None
        self.last_goal_sent_at = 0.0
        self.goal_request_pending = False
        self.active_goal_handle = None
        self.goal_sequence = 0
        self.goal_failures = 0
        self.next_navigation_attempt_at = 0.0
        self.last_tf_warning_at = 0.0
        self.direct_control_active = False
        self.retreat_active = False
        self.safety_status = "UNKNOWN"
        self.retreat_blocked_since = None
        self.retreat_escape_until = 0.0
        self.retreat_escape_direction = 1.0
        self.search_started_at = None
        self.search_phase = "NONE"

        self._publish_direct_control_active(False)
        self._set_status("SEARCHING" if self.enabled else "STOPPED")
        self.timer = self.create_timer(1.0 / update_frequency, self._update)
        self.get_logger().info(
            "Human follower ready. Services: /start_human_following, "
            "/stop_human_following, /get_human_following_status."
        )

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"Human-following status: {status}")

    def _target_pose_callback(self, pose):
        if pose.header.frame_id and pose.header.frame_id != self.robot_frame:
            self.get_logger().warning(
                f"Ignoring target pose in unsupported frame "
                f"'{pose.header.frame_id}'."
            )
            return
        self.last_target_pose = pose
        self.last_target_at = time.monotonic()
        self.target_pose_sequence += 1
        self.target_visible = True
        self.last_target_distance = math.hypot(
            pose.pose.position.x, pose.pose.position.y
        )
        self.search_started_at = None
        self.search_phase = "NONE"

    def _target_visible_callback(self, visible):
        self.target_visible = bool(visible.data)

    def _target_identity_callback(self, identity):
        requested = identity.data.strip()
        if not requested or requested == self.selected_identity:
            return
        previous = self.selected_identity
        self.selected_identity = requested
        self.target_visible = False
        self.last_target_pose = None
        self.last_target_at = None
        self.last_target_distance = None
        self.last_target_bearing = None
        self.last_target_map = None
        self._reset_motion_estimate()
        self.search_started_at = None
        self.search_phase = "NONE"
        self._cancel_active_goal("person identity changed")
        self._stop_direct_control()
        self.get_logger().info(
            f"Following target changed from {previous} to {requested}."
        )

    def _safety_status_callback(self, status):
        self.safety_status = str(status.data)

    def _handle_start(self, _request, response):
        self.enabled = True
        self.retreat_active = False
        self.social_yield_active = False
        self._reset_retreat_recovery()
        self._reset_motion_estimate()
        self.last_goal = None
        self.last_goal_mode = None
        self.goal_failures = 0
        self.next_navigation_attempt_at = 0.0
        self.search_started_at = None
        self.search_phase = "NONE"
        self._set_status("SEARCHING")
        response.success = True
        response.message = "Human following enabled; searching for a person."
        return response

    def _handle_stop(self, _request, response):
        self.enabled = False
        self.retreat_active = False
        self.social_yield_active = False
        self._reset_retreat_recovery()
        self._reset_motion_estimate()
        self._stop_direct_control()
        self._cancel_active_goal("following stopped")
        self.last_goal = None
        self.last_goal_mode = None
        self.search_started_at = None
        self.search_phase = "NONE"
        self._set_status("STOPPED")
        self._publish_visualization(None, None, None)
        response.success = True
        response.message = "Human following stopped safely."
        return response

    def _handle_status(self, _request, response):
        now = time.monotonic()
        target_age = (
            None
            if self.last_target_at is None
            else max(0.0, now - self.last_target_at)
        )
        payload = {
            "state": self.status,
            "enabled": self.enabled,
            "selected_identity": self.selected_identity,
            "target_visible": self.target_visible,
            "target_age": target_age,
            "target_distance": self.last_target_distance,
            "target_bearing": self.last_target_bearing,
            "follow_distance": self.follow_distance,
            "effective_follow_distance": self.effective_follow_distance,
            "minimum_distance": self.minimum_distance,
            "retreat_release_distance": self.retreat_release_distance,
            "retreat_active": self.retreat_active,
            "predictive_following_enabled": self.predictive_following_enabled,
            "target_velocity_map": self.target_velocity_map,
            "target_speed": self.target_speed,
            "motion_confidence": self.motion_confidence,
            "predicted_target_map": self.predicted_target_map,
            "closing_speed": self.closing_speed,
            "social_mode": self.social_mode,
            "social_yield_active": self.social_yield_active,
            "safety_status": self.safety_status,
            "retreat_escape_active": (
                time.monotonic() < self.retreat_escape_until
            ),
            "navigation_failures": self.goal_failures,
            "search_phase": self.search_phase,
            "search_elapsed": (
                None
                if self.search_started_at is None
                else max(0.0, now - self.search_started_at)
            ),
            "last_known_target_map": self.last_target_map,
        }
        response.success = self.status != "ERROR"
        response.message = json.dumps(payload, separators=(",", ":"))
        return response

    def _update(self):
        now = time.monotonic()
        if not self.enabled:
            self._stop_direct_control()
            self._set_status("STOPPED")
            return
        if not self.action_client.server_is_ready():
            self._stop_direct_control()
            self._set_status("WAITING_FOR_NAV2")
            return
        if self.goal_failures >= self.maximum_goal_failures:
            self._stop_direct_control()
            self._cancel_active_goal("navigation failures exhausted")
            self._set_status("ERROR")
            return

        target_age = (
            math.inf
            if self.last_target_at is None
            else now - self.last_target_at
        )
        if self.last_target_pose is None or target_age > self.target_timeout:
            self._handle_target_loss(now)
            return

        local_x = float(self.last_target_pose.pose.position.x)
        local_y = float(self.last_target_pose.pose.position.y)
        target_distance = math.hypot(local_x, local_y)
        target_bearing = math.atan2(local_y, local_x)
        self.last_target_distance = target_distance
        self.last_target_bearing = target_bearing
        self.search_started_at = None
        self.search_phase = "NONE"

        transform = self._lookup_robot_transform(now)
        if transform is None:
            self._stop_direct_control()
            self._set_status("WAITING_FOR_TF")
            return
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        robot_yaw = yaw_from_quaternion(transform.transform.rotation)
        cosine = math.cos(robot_yaw)
        sine = math.sin(robot_yaw)
        target_x = robot_x + cosine * local_x - sine * local_y
        target_y = robot_y + sine * local_x + cosine * local_y
        self.last_target_map = (target_x, target_y)
        robot = (robot_x, robot_y)
        target = self.last_target_map

        self._update_motion_estimate(target, now)
        self.predicted_target_map = self._predict_target(target)
        self.effective_follow_distance = self._adaptive_follow_distance()
        self.closing_speed = self._person_closing_speed(robot, target)
        self._publish_motion_diagnostics()

        if target_distance < self.minimum_distance:
            self.retreat_active = True
        elif (
            self.retreat_active
            and target_distance >= self.retreat_release_distance
        ):
            self.retreat_active = False
            self._reset_retreat_recovery()

        if self.retreat_active:
            self.social_yield_active = False
            self.social_mode = "PERSONAL_SPACE_RETREAT"
            self._cancel_active_goal("direct person-aware retreat")
            self.last_goal = None
            self.last_goal_mode = None
            self.goal_failures = 0
            if self._retreat_needs_escape_turn(now, target_bearing):
                self._publish_retreat_escape_command()
                retreat_status = "RETREAT_ESCAPE_TURN"
            else:
                self._publish_retreat_command(target_bearing)
                retreat_status = "RETREATING"
            self._publish_visualization(
                robot,
                target,
                None,
                self.predicted_target_map,
                self.target_velocity_map,
            )
            self._set_status(retreat_status)
            return

        self._reset_retreat_recovery()

        if self._should_socially_yield(target_distance):
            self.social_mode = "YIELDING_TO_APPROACHING_PERSON"
            self._cancel_active_goal("yielding to approaching person")
            self.last_goal = None
            self.last_goal_mode = None
            self.goal_failures = 0
            if abs(target_bearing) > self.face_yaw_tolerance:
                self._publish_facing_command(target_bearing)
            else:
                self._stop_direct_control()
            self._set_status("SOCIAL_YIELDING")
            self._publish_visualization(
                robot,
                target,
                None,
                self.predicted_target_map,
                self.target_velocity_map,
            )
            return

        yaw_limit = (
            self.face_yaw_tolerance
            if target_distance
            <= self.effective_follow_distance + self.distance_tolerance
            else self.face_before_follow_yaw
        )
        if abs(target_bearing) > yaw_limit:
            self.social_mode = "FACING_SELECTED_PERSON"
            self._cancel_active_goal("turning front camera toward person")
            self.last_goal = None
            self.last_goal_mode = None
            self.goal_failures = 0
            self._publish_facing_command(target_bearing)
            self._set_status("TURNING_TO_PERSON")
            self._publish_visualization(
                robot,
                target,
                None,
                self.predicted_target_map,
                self.target_velocity_map,
            )
            return

        if (
            target_distance
            <= self.effective_follow_distance + self.distance_tolerance
        ):
            self.social_mode = (
                "ADAPTIVE_DISTANCE_HOLD"
                if self._motion_is_reliable()
                else "STATIONARY_DISTANCE_HOLD"
            )
            self._stop_direct_control()
            self._cancel_active_goal("safe following distance reached")
            self.last_goal = None
            self.last_goal_mode = None
            self.goal_failures = 0
            self._set_status("HOLDING_DISTANCE")
            self._publish_visualization(
                robot,
                target,
                None,
                self.predicted_target_map,
                self.target_velocity_map,
            )
            return

        goal, goal_mode = self._social_follow_goal(robot, target)
        goal_displacement = math.hypot(
            goal[0] - robot_x, goal[1] - robot_y
        )
        if goal_displacement <= self.minimum_goal_displacement:
            self.social_mode = "GOAL_DEADBAND_HOLD"
            self._stop_direct_control()
            self._cancel_active_goal("goal displacement is negligible")
            self._set_status("HOLDING_DISTANCE")
            return
        self.social_mode = goal_mode
        self._stop_direct_control()
        self._publish_visualization(
            robot,
            target,
            goal,
            self.predicted_target_map,
            self.target_velocity_map,
        )

        self._update_navigation_goal(goal, goal_mode)
        self._set_status(
            "PREDICTIVE_FOLLOWING"
            if goal_mode == "PREDICTIVE_TRAILING"
            else "FOLLOWING"
        )

    def _reset_motion_estimate(self):
        self.predicted_target_map = None
        self.effective_follow_distance = self.follow_distance
        self.target_velocity_map = (0.0, 0.0)
        self.target_speed = 0.0
        self.motion_confidence = 0.0
        self.motion_sample_count = 0
        self.last_motion_sample_map = None
        self.last_motion_sample_time = None
        self.last_motion_sample_at = None
        self.processed_target_pose_sequence = self.target_pose_sequence
        self.closing_speed = 0.0
        self.social_yield_active = False
        self.social_mode = "STATIONARY"

    def _update_motion_estimate(self, target, now):
        if (
            self.target_pose_sequence
            == self.processed_target_pose_sequence
        ):
            return

        self.processed_target_pose_sequence = self.target_pose_sequence
        sample_time = (
            now if self.last_target_at is None else self.last_target_at
        )
        if (
            self.last_motion_sample_map is None
            or self.last_motion_sample_time is None
        ):
            self.last_motion_sample_map = target
            self.last_motion_sample_time = sample_time
            self.last_motion_sample_at = now
            return

        sample_dt = sample_time - self.last_motion_sample_time
        if sample_dt < self.velocity_minimum_sample_dt:
            return
        if sample_dt > self.velocity_reset_timeout:
            self.target_velocity_map = (0.0, 0.0)
            self.target_speed = 0.0
            self.motion_confidence = 0.0
            self.motion_sample_count = 0
            self.last_motion_sample_map = target
            self.last_motion_sample_time = sample_time
            self.last_motion_sample_at = now
            return

        raw_velocity_x = (
            target[0] - self.last_motion_sample_map[0]
        ) / sample_dt
        raw_velocity_y = (
            target[1] - self.last_motion_sample_map[1]
        ) / sample_dt
        raw_speed = math.hypot(raw_velocity_x, raw_velocity_y)

        # A large one-frame jump usually means a noisy detection or a
        # temporary track mismatch. Do not let it pull the prediction away.
        if raw_speed > self.maximum_person_speed * 2.0:
            self.motion_sample_count = max(
                0, self.motion_sample_count - 1
            )
            self.motion_confidence = min(
                1.0,
                self.motion_sample_count
                / self.motion_samples_for_confidence,
            )
            self.last_motion_sample_map = target
            self.last_motion_sample_time = sample_time
            return

        if raw_speed > self.maximum_person_speed:
            scale = self.maximum_person_speed / raw_speed
            raw_velocity_x *= scale
            raw_velocity_y *= scale

        previous_x, previous_y = self.target_velocity_map
        alpha = self.velocity_filter_alpha
        filtered_x = (
            alpha * raw_velocity_x + (1.0 - alpha) * previous_x
        )
        filtered_y = (
            alpha * raw_velocity_y + (1.0 - alpha) * previous_y
        )
        filtered_speed = math.hypot(filtered_x, filtered_y)
        if filtered_speed > self.maximum_person_speed:
            scale = self.maximum_person_speed / filtered_speed
            filtered_x *= scale
            filtered_y *= scale
            filtered_speed = self.maximum_person_speed

        self.target_velocity_map = (filtered_x, filtered_y)
        self.target_speed = filtered_speed
        self.motion_sample_count += 1
        self.motion_confidence = min(
            1.0,
            self.motion_sample_count / self.motion_samples_for_confidence,
        )
        self.last_motion_sample_map = target
        self.last_motion_sample_time = sample_time
        self.last_motion_sample_at = now

    def _motion_is_reliable(self):
        if not self.predictive_following_enabled:
            return False
        if self.last_motion_sample_at is None:
            return False
        return (
            self.motion_confidence >= 0.50
            and self.target_speed >= self.minimum_prediction_speed
            and time.monotonic() - self.last_motion_sample_at
            <= self.velocity_reset_timeout
        )

    def _limited_prediction(self, target, horizon):
        prediction_x = (
            self.target_velocity_map[0]
            * horizon
            * self.motion_confidence
        )
        prediction_y = (
            self.target_velocity_map[1]
            * horizon
            * self.motion_confidence
        )
        prediction_distance = math.hypot(prediction_x, prediction_y)
        if prediction_distance > self.maximum_prediction_distance:
            scale = (
                self.maximum_prediction_distance / prediction_distance
            )
            prediction_x *= scale
            prediction_y *= scale
        return (target[0] + prediction_x, target[1] + prediction_y)

    def _predict_target(self, target):
        if not self._motion_is_reliable():
            return target
        return self._limited_prediction(target, self.prediction_horizon)

    def _adaptive_follow_distance(self):
        if not self._motion_is_reliable():
            return self.follow_distance
        requested = self.follow_distance + (
            self.adaptive_distance_speed_gain
            * self.target_speed
            * self.motion_confidence
        )
        return clamp(
            requested,
            self.follow_distance,
            self.maximum_follow_distance,
        )

    def _person_closing_speed(self, robot, target):
        if not self._motion_is_reliable():
            return 0.0
        target_to_robot_x = robot[0] - target[0]
        target_to_robot_y = robot[1] - target[1]
        separation = math.hypot(target_to_robot_x, target_to_robot_y)
        if separation <= 1e-6:
            return 0.0
        return (
            self.target_velocity_map[0] * target_to_robot_x
            + self.target_velocity_map[1] * target_to_robot_y
        ) / separation

    def _should_socially_yield(self, target_distance):
        if not self.social_yield_enabled:
            self.social_yield_active = False
            return False

        if self.social_yield_active:
            if (
                target_distance >= self.social_yield_release_distance
                or self.closing_speed <= self.social_yield_release_speed
            ):
                self.social_yield_active = False
            return self.social_yield_active

        self.social_yield_active = (
            self._motion_is_reliable()
            and target_distance <= self.social_yield_distance
            and self.closing_speed >= self.social_yield_closing_speed
        )
        return self.social_yield_active

    def _social_follow_goal(self, robot, target):
        robot_to_target_x = target[0] - robot[0]
        robot_to_target_y = target[1] - robot[1]
        separation = max(
            math.hypot(robot_to_target_x, robot_to_target_y), 1e-6
        )
        direct_x = robot_to_target_x / separation
        direct_y = robot_to_target_y / separation
        anchor = self.predicted_target_map or target
        direction_x = direct_x
        direction_y = direct_y
        mode = "DIRECT_FOLLOW"

        if self._motion_is_reliable():
            motion_x = self.target_velocity_map[0] / self.target_speed
            motion_y = self.target_velocity_map[1] / self.target_speed
            speed_span = max(
                0.08,
                0.25 - self.minimum_prediction_speed,
            )
            speed_weight = clamp(
                (
                    self.target_speed
                    - self.minimum_prediction_speed
                )
                / speed_span,
                0.0,
                1.0,
            )
            trailing_weight = (
                self.motion_trailing_weight
                * self.motion_confidence
                * speed_weight
            )
            blended_x = (
                (1.0 - trailing_weight) * direct_x
                + trailing_weight * motion_x
            )
            blended_y = (
                (1.0 - trailing_weight) * direct_y
                + trailing_weight * motion_y
            )
            blended_length = math.hypot(blended_x, blended_y)
            if blended_length > 1e-6:
                direction_x = blended_x / blended_length
                direction_y = blended_y / blended_length
            if trailing_weight >= 0.15:
                mode = "PREDICTIVE_TRAILING"

        goal_x = (
            anchor[0] - direction_x * self.effective_follow_distance
        )
        goal_y = (
            anchor[1] - direction_y * self.effective_follow_distance
        )
        goal_yaw = math.atan2(
            target[1] - goal_y, target[0] - goal_x
        )
        return (goal_x, goal_y, goal_yaw), mode

    def _publish_motion_diagnostics(self):
        stamp = self.get_clock().now().to_msg()

        predicted = PoseStamped()
        predicted.header.frame_id = self.map_frame
        predicted.header.stamp = stamp
        if self.predicted_target_map is not None:
            predicted.pose.position.x = self.predicted_target_map[0]
            predicted.pose.position.y = self.predicted_target_map[1]
        predicted.pose.orientation.w = 1.0
        self.predicted_target_publisher.publish(predicted)

        velocity = TwistStamped()
        velocity.header.frame_id = self.map_frame
        velocity.header.stamp = stamp
        velocity.twist.linear.x = self.target_velocity_map[0]
        velocity.twist.linear.y = self.target_velocity_map[1]
        self.target_velocity_publisher.publish(velocity)

        effective_distance = Float32()
        effective_distance.data = float(self.effective_follow_distance)
        self.effective_distance_publisher.publish(effective_distance)

    def _lost_target_search_point(self, now):
        if self.last_target_map is None:
            return None
        if (
            not self.predictive_following_enabled
            or self.motion_confidence < 0.50
            or self.target_speed < self.minimum_prediction_speed
            or self.last_target_at is None
        ):
            return self.last_target_map
        unseen_duration = min(
            max(0.0, now - self.last_target_at),
            self.lost_target_prediction_horizon,
        )
        return self._limited_prediction(
            self.last_target_map, unseen_duration
        )

    def _handle_target_loss(self, now):
        self.target_visible = False
        self.retreat_active = False
        self.social_yield_active = False
        self.closing_speed = 0.0
        self.social_mode = "LOST_TARGET_PREDICTION"
        self._reset_retreat_recovery()
        self._cancel_active_goal("selected person target lost")
        self.last_goal = None
        self.last_goal_mode = None

        if self.search_started_at is None:
            self.search_started_at = now
        elapsed = now - self.search_started_at

        if self.last_target_at is None:
            self._run_periodic_reacquisition(
                elapsed,
                sweep_first=True,
            )
            self._publish_visualization(None, None, None)
            return

        search_target = self._lost_target_search_point(now)
        self.predicted_target_map = search_target

        if elapsed > self.target_search_timeout:
            self._run_periodic_reacquisition(
                elapsed - self.target_search_timeout,
                sweep_first=False,
            )
            self._publish_visualization(
                None,
                self.last_target_map,
                None,
                search_target,
                self.target_velocity_map,
            )
            return

        robot = None
        last_seen_error = None
        transform = self._lookup_robot_transform(now)
        if transform is not None:
            robot_x = float(transform.transform.translation.x)
            robot_y = float(transform.transform.translation.y)
            robot_yaw = yaw_from_quaternion(
                transform.transform.rotation
            )
            robot = (robot_x, robot_y)
            if search_target is not None:
                target_heading = math.atan2(
                    search_target[1] - robot_y,
                    search_target[0] - robot_x,
                )
                last_seen_error = angle_difference(
                    target_heading, robot_yaw
                )

        if (
            elapsed <= self.search_last_seen_duration
            and last_seen_error is not None
            and abs(last_seen_error) > self.search_yaw_tolerance
        ):
            self.search_phase = "LAST_KNOWN_POSITION"
            self._publish_search_command(
                clamp(
                    1.5 * last_seen_error,
                    -self.search_angular_speed,
                    self.search_angular_speed,
                )
            )
            self._set_status("SEARCHING_LAST_SEEN")
        else:
            self.search_phase = "ALTERNATING_SWEEP"
            sweep_elapsed = max(
                0.0, elapsed - self.search_last_seen_duration
            )
            sweep_index = int(
                sweep_elapsed / self.search_sweep_period
            )
            initial_direction = (
                -1.0
                if self.last_target_bearing is not None
                and self.last_target_bearing < 0.0
                else 1.0
            )
            direction = (
                initial_direction
                if sweep_index % 2 == 0
                else -initial_direction
            )
            self._publish_search_command(
                direction * self.search_angular_speed
            )
            self._set_status("SEARCHING_SWEEP")

        self._publish_visualization(
            robot,
            self.last_target_map,
            None,
            search_target,
            self.target_velocity_map,
        )

    def _run_periodic_reacquisition(self, elapsed, sweep_first):
        cycle_duration = (
            self.reacquire_sweep_duration + self.reacquire_pause_duration
        )
        cycle_index = int(max(0.0, elapsed) / cycle_duration)
        cycle_elapsed = max(0.0, elapsed) % cycle_duration

        if sweep_first:
            sweeping = cycle_elapsed < self.reacquire_sweep_duration
        else:
            sweeping = cycle_elapsed >= self.reacquire_pause_duration

        if not sweeping:
            self.search_phase = "PERIODIC_REACQUIRE_PAUSE"
            self._stop_direct_control()
            self._set_status("TARGET_LOST")
            return

        initial_direction = (
            -1.0
            if self.last_target_bearing is not None
            and self.last_target_bearing < 0.0
            else 1.0
        )
        direction = (
            initial_direction
            if cycle_index % 2 == 0
            else -initial_direction
        )
        self.search_phase = "PERIODIC_FULL_SWEEP"
        self._publish_search_command(
            direction * self.search_angular_speed
        )
        self._set_status("SEARCHING_REACQUIRE")

    def _publish_facing_command(self, target_bearing):
        command = Twist()
        command.angular.z = clamp(
            self.face_angular_gain * target_bearing,
            -self.max_face_angular_speed,
            self.max_face_angular_speed,
        )
        self._publish_direct_control_active(True)
        self.direct_command_publisher.publish(command)
        self.direct_control_active = True

    def _publish_search_command(self, angular_velocity):
        command = Twist()
        command.angular.z = clamp(
            angular_velocity,
            -self.search_angular_speed,
            self.search_angular_speed,
        )
        self._publish_direct_control_active(True)
        self.direct_command_publisher.publish(command)
        self.direct_control_active = True

    def _publish_retreat_command(self, target_bearing):
        command = Twist()
        command.linear.x = -self.retreat_linear_speed
        command.angular.z = clamp(
            self.retreat_angular_gain * target_bearing,
            -self.max_retreat_angular_speed,
            self.max_retreat_angular_speed,
        )
        self._publish_direct_control_active(True)
        self.direct_command_publisher.publish(command)
        self.direct_control_active = True

    def _retreat_needs_escape_turn(self, now, target_bearing):
        if now < self.retreat_escape_until:
            return True

        self.retreat_escape_until = 0.0
        if self.safety_status != "STOP_REAR":
            self.retreat_blocked_since = None
            return False

        if self.retreat_blocked_since is None:
            self.retreat_blocked_since = now
            return False
        if now - self.retreat_blocked_since < self.retreat_blocked_timeout:
            return False

        if abs(target_bearing) >= 0.05:
            # The requested person-facing turn is commonly the blocked side.
            # Try the opposite direction briefly to expose a clear rear arc.
            self.retreat_escape_direction = (
                -1.0 if target_bearing > 0.0 else 1.0
            )
        else:
            self.retreat_escape_direction *= -1.0
        self.retreat_escape_until = now + self.retreat_escape_duration
        self.retreat_blocked_since = None
        return True

    def _publish_retreat_escape_command(self):
        command = Twist()
        command.angular.z = (
            self.retreat_escape_direction
            * self.retreat_escape_angular_speed
        )
        self._publish_direct_control_active(True)
        self.direct_command_publisher.publish(command)
        self.direct_control_active = True

    def _reset_retreat_recovery(self):
        self.retreat_blocked_since = None
        self.retreat_escape_until = 0.0

    def _stop_direct_control(self, force=False):
        if not self.direct_control_active and not force:
            return
        self.direct_command_publisher.publish(Twist())
        self._publish_direct_control_active(False)
        self.direct_control_active = False

    def _publish_direct_control_active(self, active):
        message = Bool()
        message.data = bool(active)
        self.direct_control_active_publisher.publish(message)

    def _update_navigation_goal(self, goal, mode):
        now = time.monotonic()
        goal_x, goal_y, _goal_yaw = goal
        mode_changed = self.last_goal_mode != mode
        goal_moved = (
            self.last_goal is None
            or math.hypot(
                goal_x - self.last_goal[0], goal_y - self.last_goal[1]
            )
            >= self.goal_update_distance
        )
        goal_turned = (
            self.last_goal is None
            or abs(angle_difference(goal[2], self.last_goal[2]))
            >= self.goal_update_yaw
        )
        if (
            not self.goal_request_pending
            and now >= self.next_navigation_attempt_at
            and (
                mode_changed
                or now - self.last_goal_sent_at >= self.goal_update_interval
            )
            and (goal_moved or goal_turned or mode_changed)
        ):
            self._send_goal(goal, mode)

    def _lookup_robot_transform(self, now):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as error:
            if now - self.last_tf_warning_at >= 2.0:
                self.get_logger().warning(
                    f"Waiting for {self.map_frame} -> {self.robot_frame}: {error}"
                )
                self.last_tf_warning_at = now
            return None

    def _send_goal(self, goal_values, mode):
        goal_x, goal_y, goal_yaw = goal_values
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self.goal_sequence += 1
        sequence = self.goal_sequence
        self.goal_request_pending = True
        self.last_goal = goal_values
        self.last_goal_mode = mode
        self.last_goal_sent_at = time.monotonic()
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._goal_response(completed, sequence)
        )

    def _goal_response(self, future, sequence):
        try:
            goal_handle = future.result()
        except Exception as error:
            if sequence != self.goal_sequence:
                return
            self.goal_request_pending = False
            self.goal_failures += 1
            self.next_navigation_attempt_at = (
                time.monotonic() + self.navigation_retry_delay
            )
            self.get_logger().error(f"Failed to send following goal: {error}")
            self._set_status("RECOVERY")
            return
        if sequence != self.goal_sequence:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self.goal_request_pending = False
        if not goal_handle.accepted:
            self.goal_failures += 1
            self.next_navigation_attempt_at = (
                time.monotonic() + self.navigation_retry_delay
            )
            self.get_logger().warning("Nav2 rejected the following goal.")
            self._set_status("RECOVERY")
            return
        if not self.enabled:
            goal_handle.cancel_goal_async()
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._goal_result(completed, sequence)
        )

    def _goal_result(self, future, sequence):
        if sequence != self.goal_sequence:
            return
        self.active_goal_handle = None
        try:
            result = future.result()
        except Exception as error:
            self.goal_failures += 1
            self.get_logger().error(f"Following goal failed: {error}")
            self._set_status("RECOVERY")
            return
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_failures = 0
            self._set_status("HOLDING_DISTANCE")
        elif result.status not in {
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_UNKNOWN,
        }:
            self.goal_failures += 1
            self.next_navigation_attempt_at = (
                time.monotonic() + self.navigation_retry_delay
            )
            self.get_logger().warning(
                f"Following goal ended with status {result.status}."
            )
            self._set_status("RECOVERY")

    def _cancel_active_goal(self, reason):
        if self.active_goal_handle is None and not self.goal_request_pending:
            return
        handle = self.active_goal_handle
        self.active_goal_handle = None
        self.goal_request_pending = False
        self.goal_sequence += 1
        if handle is not None:
            handle.cancel_goal_async()
        self.get_logger().info(f"Canceling Nav2 goal: {reason}.")

    def _publish_visualization(
        self,
        robot,
        target,
        goal,
        predicted=None,
        velocity=None,
    ):
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        stamp = self.get_clock().now().to_msg()

        if target is not None:
            target_marker = Marker()
            target_marker.header.frame_id = self.map_frame
            target_marker.header.stamp = stamp
            target_marker.ns = "human_following_target"
            target_marker.id = 0
            target_marker.type = Marker.CYLINDER
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = target[0]
            target_marker.pose.position.y = target[1]
            target_marker.pose.position.z = 0.45
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.40
            target_marker.scale.y = 0.40
            target_marker.scale.z = 0.90
            target_marker.color.r = 0.60
            target_marker.color.g = 0.10
            target_marker.color.b = 1.0
            target_marker.color.a = 0.75
            markers.append(target_marker)

            personal_space = Marker()
            personal_space.header.frame_id = self.map_frame
            personal_space.header.stamp = stamp
            personal_space.ns = "human_following_social_space"
            personal_space.id = 0
            personal_space.type = Marker.CYLINDER
            personal_space.action = Marker.ADD
            personal_space.pose.position.x = target[0]
            personal_space.pose.position.y = target[1]
            personal_space.pose.position.z = 0.015
            personal_space.pose.orientation.w = 1.0
            personal_space.scale.x = (
                2.0 * self.effective_follow_distance
            )
            personal_space.scale.y = (
                2.0 * self.effective_follow_distance
            )
            personal_space.scale.z = 0.03
            personal_space.color.r = 1.0
            personal_space.color.g = 0.75
            personal_space.color.b = 0.05
            personal_space.color.a = 0.10
            markers.append(personal_space)

        if predicted is not None:
            predicted_marker = Marker()
            predicted_marker.header.frame_id = self.map_frame
            predicted_marker.header.stamp = stamp
            predicted_marker.ns = "human_following_prediction"
            predicted_marker.id = 0
            predicted_marker.type = Marker.SPHERE
            predicted_marker.action = Marker.ADD
            predicted_marker.pose.position.x = predicted[0]
            predicted_marker.pose.position.y = predicted[1]
            predicted_marker.pose.position.z = 0.12
            predicted_marker.pose.orientation.w = 1.0
            predicted_marker.scale.x = 0.14
            predicted_marker.scale.y = 0.14
            predicted_marker.scale.z = 0.14
            predicted_marker.color.r = 0.10
            predicted_marker.color.g = 1.0
            predicted_marker.color.b = 0.45
            predicted_marker.color.a = 0.95
            markers.append(predicted_marker)

        if (
            target is not None
            and velocity is not None
            and self._motion_is_reliable()
        ):
            velocity_arrow = Marker()
            velocity_arrow.header.frame_id = self.map_frame
            velocity_arrow.header.stamp = stamp
            velocity_arrow.ns = "human_following_velocity"
            velocity_arrow.id = 0
            velocity_arrow.type = Marker.ARROW
            velocity_arrow.action = Marker.ADD
            velocity_arrow.scale.x = 0.045
            velocity_arrow.scale.y = 0.085
            velocity_arrow.scale.z = 0.10
            velocity_arrow.color.r = 0.05
            velocity_arrow.color.g = 0.95
            velocity_arrow.color.b = 0.35
            velocity_arrow.color.a = 0.95
            arrow_start = Point()
            arrow_start.x = target[0]
            arrow_start.y = target[1]
            arrow_start.z = 0.18
            arrow_end = Point()
            arrow_end.x = target[0] + velocity[0] * 1.5
            arrow_end.y = target[1] + velocity[1] * 1.5
            arrow_end.z = 0.18
            velocity_arrow.points = [arrow_start, arrow_end]
            markers.append(velocity_arrow)

        if goal is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = self.map_frame
            goal_marker.header.stamp = stamp
            goal_marker.ns = "human_following_goal"
            goal_marker.id = 0
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = goal[0]
            goal_marker.pose.position.y = goal[1]
            goal_marker.pose.position.z = 0.08
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.18
            goal_marker.scale.y = 0.18
            goal_marker.scale.z = 0.18
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.55
            goal_marker.color.b = 0.05
            goal_marker.color.a = 0.95
            markers.append(goal_marker)

        if robot is not None and target is not None:
            line = Marker()
            line.header.frame_id = self.map_frame
            line.header.stamp = stamp
            line.ns = "human_following_line"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.035
            line.color.r = 0.10
            line.color.g = 0.85
            line.color.b = 1.0
            line.color.a = 0.90
            robot_point = Point()
            robot_point.x = robot[0]
            robot_point.y = robot[1]
            robot_point.z = 0.08
            target_point = Point()
            target_point.x = target[0]
            target_point.y = target[1]
            target_point.z = 0.08
            line.points = [robot_point, target_point]
            markers.append(line)

        if target is not None:
            status_text = Marker()
            status_text.header.frame_id = self.map_frame
            status_text.header.stamp = stamp
            status_text.ns = "human_following_social_status"
            status_text.id = 0
            status_text.type = Marker.TEXT_VIEW_FACING
            status_text.action = Marker.ADD
            status_text.pose.position.x = (
                predicted[0] if predicted is not None else target[0]
            )
            status_text.pose.position.y = (
                predicted[1] if predicted is not None else target[1]
            )
            status_text.pose.position.z = 1.20
            status_text.pose.orientation.w = 1.0
            status_text.scale.z = 0.16
            status_text.color.r = 0.95
            status_text.color.g = 0.95
            status_text.color.b = 1.0
            status_text.color.a = 0.95
            status_text.text = (
                f"{self.social_mode} | {self.target_speed:.2f} m/s | "
                f"gap {self.effective_follow_distance:.2f} m"
            )
            markers.append(status_text)

        self.visualization_publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = HumanFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            try:
                node._stop_direct_control(force=True)
                node._cancel_active_goal("node shutdown")
            except Exception:
                # SIGINT can invalidate the ROS context between the ok() check
                # and the final safety publication.
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
