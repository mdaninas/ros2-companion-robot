#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class MovingObstacleController(Node):
    """Drive a physics-constrained pedestrian around a varied waypoint loop."""

    def __init__(self):
        super().__init__("moving_obstacle_controller")

        self.declare_parameter(
            "x_command_topic",
            "/model/moving_pedestrian/joint/pedestrian_x_joint/cmd_vel",
        )
        self.declare_parameter(
            "y_command_topic",
            "/model/moving_pedestrian/joint/pedestrian_y_joint/cmd_vel",
        )
        self.declare_parameter("robot_odom_topic", "/odom")
        self.declare_parameter("identity_name", "person_alpha")
        self.declare_parameter(
            "enable_service", "/set_person_alpha_enabled"
        )
        self.declare_parameter("start_x", 0.75)
        self.declare_parameter("start_y", -0.55)
        self.declare_parameter(
            "waypoints",
            [
                0.75,
                -0.55,
                0.75,
                -0.10,
                0.55,
                0.35,
                0.10,
                0.55,
                -0.35,
                0.35,
                -0.55,
                -0.10,
                -0.25,
                -0.55,
                0.25,
                -0.65,
            ],
        )
        self.declare_parameter(
            "segment_speeds",
            [0.18, 0.25, 0.16, 0.22, 0.19, 0.26, 0.17, 0.23],
        )
        self.declare_parameter(
            "pause_durations",
            [0.4, 0.1, 0.7, 0.2, 0.5, 0.15, 0.6, 0.3],
        )
        self.declare_parameter("waypoint_tolerance", 0.035)
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("robot_avoidance_distance", 0.70)
        self.declare_parameter("avoidance_hysteresis", 0.15)

        self.x_command_topic = str(
            self.get_parameter("x_command_topic").value
        )
        self.y_command_topic = str(
            self.get_parameter("y_command_topic").value
        )
        self.robot_odom_topic = str(
            self.get_parameter("robot_odom_topic").value
        )
        self.identity_name = str(
            self.get_parameter("identity_name").value
        )
        self.enable_service_name = str(
            self.get_parameter("enable_service").value
        )
        self.position = [
            float(self.get_parameter("start_x").value),
            float(self.get_parameter("start_y").value),
        ]
        self.waypoints = self._parse_waypoints(
            self.get_parameter("waypoints").value
        )
        self.segment_speeds = self._expand_per_waypoint_parameter(
            "segment_speeds", minimum=0.01
        )
        self.pause_durations = self._expand_per_waypoint_parameter(
            "pause_durations", minimum=0.0
        )
        self.waypoint_tolerance = max(
            0.01, float(self.get_parameter("waypoint_tolerance").value)
        )
        self.update_rate = max(
            1.0, float(self.get_parameter("update_rate").value)
        )
        self.robot_avoidance_distance = max(
            0.45,
            float(self.get_parameter("robot_avoidance_distance").value),
        )
        self.avoidance_hysteresis = max(
            0.05, float(self.get_parameter("avoidance_hysteresis").value)
        )

        if math.dist(self.position, self.waypoints[0]) > 0.02:
            raise ValueError(
                "The first waypoint must match start_x and start_y."
            )

        self.target_index = 1
        self.enabled = True
        self.pause_until = None
        self.last_time = self.get_clock().now()
        self.robot_position = None
        self.robot_clearance_active = False

        self.x_command_publisher = self.create_publisher(
            Float64, self.x_command_topic, 10
        )
        self.y_command_publisher = self.create_publisher(
            Float64, self.y_command_topic, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.robot_odom_topic,
            self._odom_callback,
            10,
        )
        self.enable_service = self.create_service(
            SetBool,
            self.enable_service_name,
            self._set_enabled,
        )
        self.timer = self.create_timer(1.0 / self.update_rate, self._update)

        self.get_logger().info(
            "%s ready with %d waypoints, speeds %.2f-%.2f m/s, robot "
            "clearance %.2f m, and pause service %s."
            % (
                self.identity_name,
                len(self.waypoints),
                min(self.segment_speeds),
                max(self.segment_speeds),
                self.robot_avoidance_distance,
                self.enable_service_name,
            )
        )

    def _parse_waypoints(self, flattened_values):
        values = [float(value) for value in flattened_values]
        if len(values) < 6 or len(values) % 2 != 0:
            raise ValueError(
                "waypoints must contain at least three flattened x/y pairs"
            )
        return [
            (values[index], values[index + 1])
            for index in range(0, len(values), 2)
        ]

    def _expand_per_waypoint_parameter(self, name, minimum):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) == 1:
            values *= len(self.waypoints)
        if len(values) != len(self.waypoints):
            raise ValueError(
                "%s must have one value or one value per waypoint" % name
            )
        return [max(minimum, value) for value in values]

    def _odom_callback(self, message):
        self.robot_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _set_enabled(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self._publish_velocity(0.0, 0.0)
        self.last_time = self.get_clock().now()
        response.success = True
        response.message = (
            f"{self.identity_name} resumed."
            if self.enabled
            else f"{self.identity_name} paused."
        )
        self.get_logger().info(response.message)
        return response

    def _update(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.enabled:
            self._publish_velocity(0.0, 0.0)
            return

        elapsed = min(max(elapsed, 0.0), 0.25)

        if self._hold_for_robot_clearance():
            return

        if self.pause_until is not None:
            if now < self.pause_until:
                self._publish_velocity(0.0, 0.0)
                return
            self.pause_until = None

        target = self.waypoints[self.target_index]
        delta_x = target[0] - self.position[0]
        delta_y = target[1] - self.position[1]
        distance = math.hypot(delta_x, delta_y)

        if distance <= self.waypoint_tolerance:
            self.position[0] = target[0]
            self.position[1] = target[1]
            reached_index = self.target_index
            self.target_index = (self.target_index + 1) % len(
                self.waypoints
            )
            pause = self.pause_durations[reached_index]
            self._publish_velocity(0.0, 0.0)
            if pause > 0.0:
                self.pause_until = now + Duration(seconds=pause)
            next_target = self.waypoints[self.target_index]
            self.get_logger().info(
                "%s reached waypoint %d; pausing %.2f s, then "
                "heading to (%.2f, %.2f)."
                % (
                    self.identity_name,
                    reached_index + 1,
                    pause,
                    next_target[0],
                    next_target[1],
                )
            )
            return

        speed = self.segment_speeds[self.target_index]
        travel = min(speed * elapsed, distance)
        velocity_x = speed * delta_x / distance
        velocity_y = speed * delta_y / distance
        self.position[0] += travel * delta_x / distance
        self.position[1] += travel * delta_y / distance
        self._publish_velocity(velocity_x, velocity_y)

    def _hold_for_robot_clearance(self):
        if self.robot_position is None:
            return False

        distance = math.dist(self.position, self.robot_position)
        resume_distance = (
            self.robot_avoidance_distance + self.avoidance_hysteresis
        )
        if self.robot_clearance_active:
            if distance >= resume_distance:
                self.robot_clearance_active = False
                self.get_logger().info(
                    "Robot cleared the pedestrian path; route resumed."
                )
                return False

        if distance <= self.robot_avoidance_distance:
            if not self.robot_clearance_active:
                self.robot_clearance_active = True
                self.get_logger().info(
                    "Robot entered the pedestrian clearance zone."
                )

        if not self.robot_clearance_active:
            return False

        if self._route_moves_away_from_robot(self.target_index):
            return False

        clearance_index = self._find_route_away_from_robot()
        if clearance_index is not None:
            if clearance_index != self.target_index:
                self.target_index = clearance_index
                self.pause_until = None
                target = self.waypoints[self.target_index]
                self.get_logger().info(
                    "%s changed route toward (%.2f, %.2f) to clear the robot."
                    % (self.identity_name, target[0], target[1])
                )
            return False

        # If none of the configured route segments initially increases
        # clearance, wait rather than introducing an artificial escape motion.
        self._publish_velocity(0.0, 0.0)
        return True

    def _route_moves_away_from_robot(self, waypoint_index):
        target = self.waypoints[waypoint_index]
        route_x = target[0] - self.position[0]
        route_y = target[1] - self.position[1]
        route_length = math.hypot(route_x, route_y)
        if route_length <= self.waypoint_tolerance:
            return False

        away_x = self.position[0] - self.robot_position[0]
        away_y = self.position[1] - self.robot_position[1]
        away_length = math.hypot(away_x, away_y)
        if away_length <= 1e-6:
            return False

        outward_alignment = (
            route_x * away_x + route_y * away_y
        ) / (route_length * away_length)
        return outward_alignment >= 0.10

    def _find_route_away_from_robot(self):
        for offset in range(1, len(self.waypoints)):
            candidate = (self.target_index + offset) % len(self.waypoints)
            if self._route_moves_away_from_robot(candidate):
                return candidate
        return None

    def _publish_velocity(self, velocity_x, velocity_y):
        x_message = Float64()
        x_message.data = float(velocity_x)
        self.x_command_publisher.publish(x_message)

        y_message = Float64()
        y_message.data = float(velocity_y)
        self.y_command_publisher.publish(y_message)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A bridge message can race with ROS shutdown when the complete launch
        # is stopped. Preserve real runtime failures while exiting cleanly if
        # the ROS context has already been invalidated.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node._publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
