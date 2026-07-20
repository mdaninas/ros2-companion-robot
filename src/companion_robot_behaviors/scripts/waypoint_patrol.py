#!/usr/bin/env python3

"""Visit configured map-frame waypoints sequentially through Nav2."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__("waypoint_patrol")

        self.declare_parameter("waypoints", [0.8, 0.0, 0.0])
        self.declare_parameter("loop_count", 1)
        self.declare_parameter("pause_seconds", 1.0)
        self.declare_parameter("retry_delay", 2.0)
        self.declare_parameter("max_goal_retries", 5)
        self.declare_parameter("home_pose", [0.0, 0.0, 0.0])

        flat_waypoints = [
            float(value) for value in self.get_parameter("waypoints").value
        ]
        if not flat_waypoints or len(flat_waypoints) % 3 != 0:
            raise ValueError("Parameter 'waypoints' must contain x, y, yaw groups.")

        self.waypoints = [
            tuple(flat_waypoints[index : index + 3])
            for index in range(0, len(flat_waypoints), 3)
        ]
        self.loop_count = int(self.get_parameter("loop_count").value)
        self.pause_seconds = max(
            0.0, float(self.get_parameter("pause_seconds").value)
        )
        self.retry_delay = max(
            0.1, float(self.get_parameter("retry_delay").value)
        )
        self.max_goal_retries = max(
            0, int(self.get_parameter("max_goal_retries").value)
        )
        home_pose = [
            float(value) for value in self.get_parameter("home_pose").value
        ]
        if len(home_pose) != 3:
            raise ValueError("Parameter 'home_pose' must contain x, y, yaw.")
        self.home_pose = tuple(home_pose)

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.return_home_service = self.create_service(
            Trigger, "/return_home", self._handle_return_home
        )
        self.current_waypoint = 0
        self.completed_loops = 0
        self.retry_count = 0
        self.finished = False
        self.successful = False
        self.return_state = None
        self.active_goal_handle = None
        self.active_goal_kind = None
        self.goal_request_pending = False
        self.cancel_in_progress = False
        self._scheduled_timer = None
        self._last_feedback_log_ns = 0

        self.get_logger().info(
            f"Patrol loaded {len(self.waypoints)} waypoints; waiting for Nav2."
        )
        self._server_timer = self.create_timer(1.0, self._wait_for_server)

    def _wait_for_server(self):
        if not self.action_client.server_is_ready():
            self.get_logger().info("Waiting for /navigate_to_pose ...")
            return

        self._server_timer.cancel()
        self.destroy_timer(self._server_timer)
        self._server_timer = None
        if self.return_state is not None:
            self._send_home_goal()
        else:
            self._send_current_waypoint()

    def _schedule(self, callback, delay_seconds):
        if self.finished:
            return

        def run_once():
            timer = self._scheduled_timer
            self._scheduled_timer = None
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
            callback()

        self._scheduled_timer = self.create_timer(
            max(0.01, delay_seconds), run_once
        )

    def _send_current_waypoint(self):
        if self.return_state is not None:
            self._send_home_goal()
            return

        x, y, yaw = self.waypoints[self.current_waypoint]
        number = self.current_waypoint + 1
        self._send_goal(
            x,
            y,
            yaw,
            "patrol",
            f"waypoint {number}/{len(self.waypoints)}",
        )

    def _send_home_goal(self):
        if self.return_state == "navigating":
            return

        self.return_state = "navigating"
        x, y, yaw = self.home_pose
        self._send_goal(x, y, yaw, "home", "home")

    def _send_goal(self, x, y, yaw, kind, label):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f"Sending {label}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )
        self.active_goal_kind = kind
        self.goal_request_pending = True
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(
            lambda response_future: self._goal_response_callback(
                response_future, kind
            )
        )

    def _goal_response_callback(self, future, kind):
        self.goal_request_pending = False
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_goal_kind = None
            if self.return_state is not None:
                self.return_state = "requested"
                self._retry_or_finish(
                    "Nav2 rejected the return-home goal",
                    self._send_home_goal,
                    "Return home",
                )
            else:
                self._retry_or_finish(
                    "Nav2 rejected the waypoint",
                    self._send_current_waypoint,
                    "Waypoint",
                )
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future: self._result_callback(
                completed_future, kind, goal_handle
            )
        )

        if kind == "patrol" and self.return_state is not None:
            self._cancel_active_patrol_goal()

    def _feedback_callback(self, feedback_message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 2_000_000_000:
            return
        self._last_feedback_log_ns = now_ns
        distance = feedback_message.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _result_callback(self, future, kind, goal_handle):
        wrapped_result = future.result()
        if self.active_goal_handle is goal_handle:
            self.active_goal_handle = None
            self.active_goal_kind = None

        if kind == "patrol" and self.return_state is not None:
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            message = wrapped_result.result.error_msg or "navigation failed"
            if kind == "home":
                self.return_state = "requested"
                self._retry_or_finish(
                    message, self._send_home_goal, "Return home"
                )
            else:
                self._retry_or_finish(
                    message, self._send_current_waypoint, "Waypoint"
                )
            return

        self.retry_count = 0
        if kind == "home":
            self.return_state = "complete"
            self.successful = True
            self.finished = True
            self.get_logger().info("Robot reached home successfully.")
            return

        self.get_logger().info(
            f"Waypoint {self.current_waypoint + 1} reached."
        )
        self.current_waypoint += 1

        if self.current_waypoint < len(self.waypoints):
            self._schedule(self._send_current_waypoint, self.pause_seconds)
            return

        self.completed_loops += 1
        self.current_waypoint = 0
        if self.loop_count == 0 or self.completed_loops < self.loop_count:
            self.get_logger().info(
                f"Patrol loop {self.completed_loops} complete; starting next loop."
            )
            self._schedule(self._send_current_waypoint, self.pause_seconds)
            return

        self.successful = True
        self.finished = True
        self.get_logger().info(
            f"Patrol completed {self.completed_loops} loop(s) successfully."
        )

    def _handle_return_home(self, request, response):
        del request
        if self.return_state is not None:
            response.success = True
            response.message = "Return-home is already in progress."
            return response

        self.get_logger().info("Return-home requested; stopping patrol.")
        self.return_state = "requested"
        self.retry_count = 0

        if self._scheduled_timer is not None:
            timer = self._scheduled_timer
            self._scheduled_timer = None
            timer.cancel()
            self.destroy_timer(timer)

        if self.active_goal_handle is not None:
            self._cancel_active_patrol_goal()
        elif not self.goal_request_pending and self._server_timer is None:
            self._schedule(self._send_home_goal, 0.01)

        response.success = True
        response.message = "Patrol stopped; robot is returning home."
        return response

    def _cancel_active_patrol_goal(self):
        if self.cancel_in_progress or self.active_goal_handle is None:
            return

        self.cancel_in_progress = True
        self.return_state = "canceling"
        future = self.active_goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_callback)

    def _cancel_callback(self, future):
        self.cancel_in_progress = False
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info("Active patrol goal canceled.")
        else:
            self.get_logger().warning(
                "Patrol goal had already finished before cancellation."
            )
        self.return_state = "requested"
        self._schedule(self._send_home_goal, 0.1)

    def _retry_or_finish(self, reason, retry_callback, label):
        self.retry_count += 1
        if self.retry_count > self.max_goal_retries:
            self.get_logger().error(
                f"{label} failed after {self.max_goal_retries} retries: {reason}"
            )
            self.finished = True
            return

        self.get_logger().warning(
            f"{reason}; retry {self.retry_count}/{self.max_goal_retries} "
            f"in {self.retry_delay:.1f} s."
        )
        self._schedule(retry_callback, self.retry_delay)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
