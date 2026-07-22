#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class MovingObstacleController(Node):
    """Drive a physics-constrained Gazebo obstacle back and forth."""

    def __init__(self):
        super().__init__("moving_obstacle_controller")

        self.declare_parameter(
            "command_topic",
            "/model/moving_pedestrian/joint/pedestrian_slide_joint/cmd_vel",
        )
        self.declare_parameter("robot_odom_topic", "/odom")
        self.declare_parameter("x", 0.80)
        self.declare_parameter("y_min", -1.25)
        self.declare_parameter("y_max", 1.25)
        self.declare_parameter("speed", 0.22)
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("robot_avoidance_distance", 0.70)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.robot_odom_topic = str(
            self.get_parameter("robot_odom_topic").value
        )
        self.x = float(self.get_parameter("x").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.speed = max(0.01, float(self.get_parameter("speed").value))
        self.update_rate = max(
            1.0, float(self.get_parameter("update_rate").value)
        )
        self.robot_avoidance_distance = max(
            0.45,
            float(self.get_parameter("robot_avoidance_distance").value),
        )

        if self.y_min >= self.y_max:
            raise ValueError("y_min must be smaller than y_max")

        self.distance_along_path = 0.0
        self.path_length = self.y_max - self.y_min
        self.direction = 1.0
        self.enabled = True
        self.last_time = self.get_clock().now()
        self.robot_position = None

        self.command_publisher = self.create_publisher(
            Float64, self.command_topic, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.robot_odom_topic,
            self._odom_callback,
            10,
        )
        self.enable_service = self.create_service(
            SetBool,
            "/set_moving_obstacle_enabled",
            self._set_enabled,
        )
        self.timer = self.create_timer(1.0 / self.update_rate, self._update)

        self.get_logger().info(
            "Physics-based moving obstacle ready at y=[%.2f, %.2f], "
            "speed=%.2f m/s, robot clearance=%.2f m."
            % (
                self.y_min,
                self.y_max,
                self.speed,
                self.robot_avoidance_distance,
            )
        )

    def _odom_callback(self, message):
        self.robot_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _set_enabled(self, request, response):
        self.enabled = bool(request.data)
        self._publish_velocity(
            self.direction * self.speed if self.enabled else 0.0
        )
        response.success = True
        response.message = (
            "Moving obstacle resumed."
            if self.enabled
            else "Moving obstacle paused."
        )
        self.get_logger().info(response.message)
        return response

    def _update(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if not self.enabled:
            self._publish_velocity(0.0)
            return

        elapsed = min(max(elapsed, 0.0), 0.25)

        if self._robot_is_too_close_ahead():
            self.direction *= -1.0
            self.get_logger().info(
                "Robot entered the pedestrian clearance zone; reversing."
            )

        self.distance_along_path += self.direction * self.speed * elapsed

        if self.distance_along_path >= self.path_length:
            self.distance_along_path = self.path_length
            self.direction = -1.0
        elif self.distance_along_path <= 0.0:
            self.distance_along_path = 0.0
            self.direction = 1.0

        self._publish_velocity(self.direction * self.speed)

    def _robot_is_too_close_ahead(self):
        if self.robot_position is None:
            return False

        obstacle_y = self.y_min + self.distance_along_path
        delta_x = self.robot_position[0] - self.x
        delta_y = self.robot_position[1] - obstacle_y
        distance = math.hypot(delta_x, delta_y)
        moving_toward_robot = self.direction * delta_y >= 0.0
        return (
            moving_toward_robot
            and distance <= self.robot_avoidance_distance
        )

    def _publish_velocity(self, velocity):
        message = Float64()
        message.data = float(velocity)
        self.command_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_velocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
