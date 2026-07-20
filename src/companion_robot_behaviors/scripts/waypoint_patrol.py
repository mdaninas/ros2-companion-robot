#!/usr/bin/env python3

"""Visit configured map-frame waypoints sequentially through Nav2."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__("waypoint_patrol")

        self.declare_parameter("waypoints", [0.8, 0.0, 0.0])
        self.declare_parameter("loop_count", 1)
        self.declare_parameter("pause_seconds", 1.0)
        self.declare_parameter("retry_delay", 2.0)
        self.declare_parameter("max_goal_retries", 5)

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

        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.current_waypoint = 0
        self.completed_loops = 0
        self.retry_count = 0
        self.finished = False
        self.successful = False
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
        x, y, yaw = self.waypoints[self.current_waypoint]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        number = self.current_waypoint + 1
        self.get_logger().info(
            f"Sending waypoint {number}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._retry_or_finish("Nav2 rejected the waypoint")
            return

        self.retry_count = 0
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_feedback_log_ns < 2_000_000_000:
            return
        self._last_feedback_log_ns = now_ns
        distance = feedback_message.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {distance:.2f} m")

    def _result_callback(self, future):
        wrapped_result = future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            message = wrapped_result.result.error_msg or "navigation failed"
            self._retry_or_finish(message)
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

    def _retry_or_finish(self, reason):
        self.retry_count += 1
        if self.retry_count > self.max_goal_retries:
            self.get_logger().error(
                f"Waypoint failed after {self.max_goal_retries} retries: {reason}"
            )
            self.finished = True
            return

        self.get_logger().warning(
            f"{reason}; retry {self.retry_count}/{self.max_goal_retries} "
            f"in {self.retry_delay:.1f} s."
        )
        self._schedule(self._send_current_waypoint, self.retry_delay)


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
