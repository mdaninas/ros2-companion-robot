#!/usr/bin/env python3

"""Visit configured map-frame waypoints sequentially through Nav2."""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class WaypointPatrol(Node):
    def __init__(self):
        super().__init__("waypoint_patrol")

        self.declare_parameter("waypoints", [0.8, 0.0, 0.0])
        self.declare_parameter("loop_count", 1)
        self.declare_parameter("pause_seconds", 1.0)
        self.declare_parameter("retry_delay", 2.0)
        self.declare_parameter("max_goal_retries", 5)
        self.declare_parameter("home_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("energy_resume_delay", 1.0)
        self.declare_parameter("nav2_activation_delay", 1.5)

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
        self.energy_resume_delay = max(
            0.1, float(self.get_parameter("energy_resume_delay").value)
        )
        self.nav2_activation_delay = max(
            0.0, float(self.get_parameter("nav2_activation_delay").value)
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
        self.recover_service = self.create_service(
            Trigger, "/recover_patrol", self._handle_recover
        )
        self.local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, "/patrol_status", status_qos
        )
        self.route_publisher = self.create_publisher(
            MarkerArray, "/patrol/visualization", status_qos
        )
        self.docking_status_subscription = self.create_subscription(
            String,
            "/docking_status",
            self._docking_status_callback,
            status_qos,
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
        self.paused_for_energy = False
        self.docking_state = "UNKNOWN"
        self.status = "UNKNOWN"
        self.recovery_callback = None
        self.recovery_goal_handle = None
        self._scheduled_timer = None
        self._last_feedback_log_ns = 0
        self.costmap_clear_futures = []

        self.get_logger().info(
            f"Patrol loaded {len(self.waypoints)} waypoints; waiting for Nav2."
        )
        self._set_status("WAITING_FOR_NAV2")
        self._publish_route_markers()
        self._server_timer = self.create_timer(1.0, self._wait_for_server)

    def _set_status(self, status):
        if status == self.status:
            return
        self.status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)
        self.get_logger().info(f"Patrol status: {status}")
        if hasattr(self, "route_publisher"):
            self._publish_route_markers()

    def _publish_route_markers(self):
        markers = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.append(clear)

        route = Marker()
        route.header.frame_id = "map"
        route.header.stamp = self.get_clock().now().to_msg()
        route.ns = "patrol_route"
        route.id = 0
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.pose.orientation.w = 1.0
        route.scale.x = 0.025
        route.color.r = 0.05
        route.color.g = 0.75
        route.color.b = 1.0
        route.color.a = 0.80
        for x, y, _yaw in self.waypoints:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.03
            route.points.append(point)
        if self.waypoints:
            closing_point = Point()
            closing_point.x = self.waypoints[0][0]
            closing_point.y = self.waypoints[0][1]
            closing_point.z = 0.03
            route.points.append(closing_point)
        markers.append(route)

        for index, (x, y, yaw) in enumerate(self.waypoints):
            waypoint = Marker()
            waypoint.header.frame_id = "map"
            waypoint.header.stamp = route.header.stamp
            waypoint.ns = "patrol_waypoints"
            waypoint.id = index
            waypoint.type = Marker.ARROW
            waypoint.action = Marker.ADD
            waypoint.pose.position.x = x
            waypoint.pose.position.y = y
            waypoint.pose.position.z = 0.05
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
            waypoint.scale.x = 0.28
            waypoint.scale.y = 0.07
            waypoint.scale.z = 0.07
            if index == self.current_waypoint and not self.finished:
                waypoint.color.r = 1.0
                waypoint.color.g = 0.45
                waypoint.color.b = 0.05
            else:
                waypoint.color.r = 0.05
                waypoint.color.g = 0.75
                waypoint.color.b = 1.0
            waypoint.color.a = 0.95
            markers.append(waypoint)

        home = Marker()
        home.header.frame_id = "map"
        home.header.stamp = route.header.stamp
        home.ns = "patrol_home"
        home.id = 0
        home.type = Marker.CUBE
        home.action = Marker.ADD
        home.pose.position.x = self.home_pose[0]
        home.pose.position.y = self.home_pose[1]
        home.pose.position.z = 0.05
        home.pose.orientation.w = 1.0
        home.scale.x = 0.14
        home.scale.y = 0.14
        home.scale.z = 0.10
        home.color.r = 0.10
        home.color.g = 0.95
        home.color.b = 0.35
        home.color.a = 0.95
        markers.append(home)
        self.route_publisher.publish(MarkerArray(markers=markers))

    def _wait_for_server(self):
        if not self.action_client.server_is_ready():
            self.get_logger().info("Waiting for /navigate_to_pose ...")
            return

        self._server_timer.cancel()
        self.destroy_timer(self._server_timer)
        self._server_timer = None
        self.get_logger().info(
            "Nav2 action server found; allowing lifecycle activation to settle."
        )
        callback = (
            self._send_home_goal
            if self.return_state is not None
            else self._send_current_waypoint
        )
        self._schedule(callback, self.nav2_activation_delay)

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
        if self.paused_for_energy:
            return

        if self.return_state is not None:
            self._send_home_goal()
            return

        self._set_status("PATROLLING")
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
        self._set_status("RETURNING_HOME")
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
            self._cancel_active_patrol_goal("return_home")
        elif kind == "patrol" and self.paused_for_energy:
            self._cancel_active_patrol_goal("energy_pause")

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

        if self.recovery_goal_handle is goal_handle:
            self.recovery_goal_handle = None
            return

        if kind == "patrol" and (
            self.return_state is not None or self.paused_for_energy
        ):
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
        self.recovery_callback = None
        if kind == "home":
            self.return_state = "complete"
            self.successful = True
            self.finished = True
            self._set_status("COMPLETED")
            self.get_logger().info("Robot reached home successfully.")
            return

        self._set_status("PATROLLING")
        self.get_logger().info(
            f"Waypoint {self.current_waypoint + 1} reached."
        )
        self.current_waypoint += 1
        self._publish_route_markers()

        if self.current_waypoint < len(self.waypoints):
            self._schedule(self._send_current_waypoint, self.pause_seconds)
            return

        self.completed_loops += 1
        self.current_waypoint = 0
        self._publish_route_markers()
        if self.loop_count == 0 or self.completed_loops < self.loop_count:
            self.get_logger().info(
                f"Patrol loop {self.completed_loops} complete; starting next loop."
            )
            self._schedule(self._send_current_waypoint, self.pause_seconds)
            return

        self.successful = True
        self.finished = True
        self._set_status("COMPLETED")
        self.get_logger().info(
            f"Patrol completed {self.completed_loops} loop(s) successfully."
        )

    def _handle_return_home(self, request, response):
        del request
        if self.paused_for_energy:
            response.success = False
            response.message = (
                "Return-home is unavailable while battery docking is active."
            )
            return response

        if self.return_state is not None:
            response.success = True
            response.message = "Return-home is already in progress."
            return response

        self.get_logger().info("Return-home requested; stopping patrol.")
        self.return_state = "requested"
        self.retry_count = 0
        self._set_status("RETURNING_HOME")

        self._cancel_scheduled_timer()

        if self.active_goal_handle is not None:
            self._cancel_active_patrol_goal("return_home")
        elif not self.goal_request_pending and self._server_timer is None:
            self._schedule(self._send_home_goal, 0.01)

        response.success = True
        response.message = "Patrol stopped; robot is returning home."
        return response

    def _handle_recover(self, request, response):
        del request
        if self.paused_for_energy:
            response.success = False
            response.message = "Patrol recovery is paused during docking."
            return response

        if self.cancel_in_progress or self.goal_request_pending:
            response.success = False
            response.message = "A patrol goal transition is already active."
            return response

        if self.finished and self.status == "COMPLETED":
            response.success = False
            response.message = "The configured patrol has already completed."
            return response

        self.finished = False
        self.successful = False
        self.retry_count = 0
        self._cancel_scheduled_timer()
        self._set_status("RECOVERY")
        self._clear_costmaps("manual patrol recovery")

        if self.active_goal_handle is not None:
            if self.return_state == "navigating":
                self.return_state = "requested"
            self._cancel_active_patrol_goal("recovery")
            response.success = True
            response.message = (
                "Active goal is being canceled and replanned."
            )
            return response

        callback = self.recovery_callback
        if callback is None:
            callback = (
                self._send_home_goal
                if self.return_state is not None
                else self._send_current_waypoint
            )
        self._schedule(callback, 0.1)
        response.success = True
        response.message = "Patrol recovery and replanning requested."
        return response

    def _docking_status_callback(self, message):
        state = message.data.strip().upper()
        self.docking_state = state

        if state == "IDLE":
            if not self.paused_for_energy:
                return

            self.paused_for_energy = False
            self.retry_count = 0
            self.get_logger().info(
                "Docking cycle complete; resuming the pending waypoint."
            )
            self._set_status("PATROLLING")
            if (
                self.return_state is None
                and not self.finished
                and not self.goal_request_pending
                and self.active_goal_handle is None
            ):
                self._schedule(
                    self._send_current_waypoint, self.energy_resume_delay
                )
            return

        if state == "UNKNOWN" or self.finished or self.return_state is not None:
            return

        if not self.paused_for_energy:
            self.paused_for_energy = True
            self._cancel_scheduled_timer()
            self._set_status("PAUSED")
            self.get_logger().info(
                "Battery docking started; patrol paused at "
                f"waypoint {self.current_waypoint + 1}."
            )

        if self.active_goal_kind == "patrol":
            self._cancel_active_patrol_goal("energy_pause")

    def _cancel_scheduled_timer(self):
        if self._scheduled_timer is None:
            return

        timer = self._scheduled_timer
        self._scheduled_timer = None
        timer.cancel()
        self.destroy_timer(timer)

    def _cancel_active_patrol_goal(self, reason):
        if self.cancel_in_progress or self.active_goal_handle is None:
            return

        self.cancel_in_progress = True
        if reason == "recovery":
            self.recovery_goal_handle = self.active_goal_handle
        self.active_goal_kind = None
        if reason == "return_home":
            self.return_state = "canceling"
        future = self.active_goal_handle.cancel_goal_async()
        future.add_done_callback(
            lambda cancel_future: self._cancel_callback(cancel_future, reason)
        )

    def _cancel_callback(self, future, reason):
        self.cancel_in_progress = False
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            if reason == "energy_pause":
                self.get_logger().info(
                    "Active waypoint canceled; patrol is waiting for charging."
                )
            else:
                self.get_logger().info("Active patrol goal canceled.")
        else:
            self.get_logger().warning(
                "Patrol goal had already finished before cancellation."
            )

        if reason == "energy_pause":
            return

        if reason == "recovery":
            callback = (
                self._send_home_goal
                if self.return_state is not None
                else self._send_current_waypoint
            )
            self._schedule(callback, 0.2)
            return

        self.return_state = "requested"
        self._schedule(self._send_home_goal, 0.1)

    def _retry_or_finish(self, reason, retry_callback, label):
        self.recovery_callback = retry_callback
        self.retry_count += 1
        if self.retry_count > self.max_goal_retries:
            self.get_logger().error(
                f"{label} failed after {self.max_goal_retries} retries: {reason}"
            )
            self.finished = True
            self._set_status("ERROR")
            return

        self._set_status("RECOVERY")
        self._clear_costmaps(f"{label.lower()} retry")
        self.get_logger().warning(
            f"{reason}; retry {self.retry_count}/{self.max_goal_retries} "
            f"in {self.retry_delay:.1f} s."
        )
        self._schedule(retry_callback, self.retry_delay)

    def _clear_costmaps(self, reason):
        requested = False
        clients = (
            ("local", self.local_costmap_client),
            ("global", self.global_costmap_client),
        )
        for name, client in clients:
            if not client.service_is_ready():
                self.get_logger().warning(
                    f"{name.capitalize()} costmap clearing service is not ready."
                )
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            self.costmap_clear_futures.append(future)
            future.add_done_callback(
                lambda done, costmap=name: self._costmap_clear_response(
                    done, costmap
                )
            )
            requested = True
        if requested:
            self.get_logger().warning(
                f"Clearing Nav2 costmaps before {reason}."
            )

    def _costmap_clear_response(self, future, costmap):
        if future in self.costmap_clear_futures:
            self.costmap_clear_futures.remove(future)
        try:
            future.result()
            self.get_logger().info(
                f"{costmap.capitalize()} costmap cleared successfully."
            )
        except Exception as error:
            self.get_logger().error(
                f"Failed to clear the {costmap} costmap: {error}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatrol()

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
