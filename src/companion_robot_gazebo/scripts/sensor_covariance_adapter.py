#!/usr/bin/env python3

"""Annotate raw Gazebo wheel-odometry and IMU messages with covariance."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def covariance_diagonal(standard_deviations):
    covariance = [0.0] * 36
    for index, standard_deviation in enumerate(standard_deviations):
        covariance[index * 6 + index] = standard_deviation**2
    return covariance


def covariance_diagonal_3d(standard_deviations):
    covariance = [0.0] * 9
    for index, standard_deviation in enumerate(standard_deviations):
        covariance[index * 3 + index] = standard_deviation**2
    return covariance


class SensorCovarianceAdapter(Node):
    def __init__(self):
        super().__init__("sensor_covariance_adapter")

        self.declare_parameter("wheel_raw_topic", "/wheel_odom/raw")
        self.declare_parameter("wheel_output_topic", "/wheel_odom")
        self.declare_parameter("imu_raw_topic", "/imu/data_raw")
        self.declare_parameter("imu_output_topic", "/imu/data")
        self.declare_parameter(
            "wheel_pose_stddev", [0.04, 0.08, 1.0, 1.0, 1.0, 0.06]
        )
        self.declare_parameter(
            "wheel_twist_stddev", [0.025, 0.04, 0.20, 0.20, 0.20, 0.035]
        )
        self.declare_parameter(
            "imu_orientation_stddev", [0.035, 0.035, 0.06]
        )
        self.declare_parameter(
            "imu_angular_velocity_stddev", [0.012, 0.012, 0.018]
        )
        self.declare_parameter(
            "imu_linear_acceleration_stddev", [0.08, 0.08, 0.12]
        )

        self.wheel_pose_covariance = covariance_diagonal(
            self._standard_deviations("wheel_pose_stddev", 6)
        )
        self.wheel_twist_covariance = covariance_diagonal(
            self._standard_deviations("wheel_twist_stddev", 6)
        )
        self.imu_orientation_covariance = covariance_diagonal_3d(
            self._standard_deviations("imu_orientation_stddev", 3)
        )
        self.imu_angular_velocity_covariance = covariance_diagonal_3d(
            self._standard_deviations("imu_angular_velocity_stddev", 3)
        )
        self.imu_linear_acceleration_covariance = covariance_diagonal_3d(
            self._standard_deviations("imu_linear_acceleration_stddev", 3)
        )

        wheel_raw_topic = str(self.get_parameter("wheel_raw_topic").value)
        wheel_output_topic = str(
            self.get_parameter("wheel_output_topic").value
        )
        imu_raw_topic = str(self.get_parameter("imu_raw_topic").value)
        imu_output_topic = str(self.get_parameter("imu_output_topic").value)

        self.wheel_publisher = self.create_publisher(
            Odometry, wheel_output_topic, qos_profile_sensor_data
        )
        self.imu_publisher = self.create_publisher(
            Imu, imu_output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            wheel_raw_topic,
            self._wheel_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, imu_raw_topic, self._imu_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Covariance adapter: {wheel_raw_topic} -> {wheel_output_topic}, "
            f"{imu_raw_topic} -> {imu_output_topic}."
        )

    def _standard_deviations(self, parameter_name, expected_size):
        values = [
            float(value)
            for value in self.get_parameter(parameter_name).value
        ]
        if len(values) != expected_size:
            raise ValueError(
                f"{parameter_name} must contain {expected_size} values."
            )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError(
                f"{parameter_name} values must be finite and positive."
            )
        return values

    def _wheel_callback(self, message):
        message.pose.covariance = self.wheel_pose_covariance
        message.twist.covariance = self.wheel_twist_covariance
        self.wheel_publisher.publish(message)

    def _imu_callback(self, message):
        message.orientation_covariance = self.imu_orientation_covariance
        message.angular_velocity_covariance = (
            self.imu_angular_velocity_covariance
        )
        message.linear_acceleration_covariance = (
            self.imu_linear_acceleration_covariance
        )
        self.imu_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SensorCovarianceAdapter()
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
