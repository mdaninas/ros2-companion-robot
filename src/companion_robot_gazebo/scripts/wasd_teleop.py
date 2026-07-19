#!/usr/bin/env python3

"""Safe terminal teleoperation for the companion robot using WASD keys."""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
Kontrol Companion Robot
-----------------------
  W              : maju
  S              : mundur
  A              : putar kiri
  D              : putar kanan
  Spasi          : berhenti
  Q              : keluar

Tahan tombol untuk bergerak terus. Robot otomatis berhenti jika input terputus.
"""


def read_key(timeout: float = 0.05):
    """Read one keyboard character without blocking the ROS loop."""
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return None
    return sys.stdin.read(1)


class WasdTeleop(Node):
    def __init__(self):
        super().__init__("wasd_teleop")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.declare_parameter("linear_speed", 0.22)
        self.declare_parameter("angular_speed", 0.70)
        self.declare_parameter("command_timeout", 0.65)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)

        self.linear = 0.0
        self.angular = 0.0
        self.command_deadline = 0.0
        self.status = "BERHENTI"

    def command(self, linear: float, angular: float, status: str):
        self.linear = linear
        self.angular = angular
        self.command_deadline = time.monotonic() + self.command_timeout
        self.status = status

    def stop(self):
        self.linear = 0.0
        self.angular = 0.0
        self.command_deadline = 0.0
        self.status = "BERHENTI"

    def publish(self):
        if self.command_deadline and time.monotonic() > self.command_deadline:
            self.stop()

        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def publish_final_stop(self):
        self.stop()
        for _ in range(3):
            self.publish()
            rclpy.spin_once(self, timeout_sec=0.02)


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error("WASD teleop harus dijalankan di terminal interaktif.")
        node.destroy_node()
        rclpy.shutdown()
        return

    terminal_settings = termios.tcgetattr(sys.stdin)
    last_status = None

    print(HELP)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()

            if key in ("w", "W"):
                node.command(node.linear_speed, 0.0, "MAJU")
            elif key in ("s", "S"):
                node.command(-node.linear_speed, 0.0, "MUNDUR")
            elif key in ("a", "A"):
                node.command(0.0, node.angular_speed, "PUTAR KIRI")
            elif key in ("d", "D"):
                node.command(0.0, -node.angular_speed, "PUTAR KANAN")
            elif key == " ":
                node.stop()
            elif key in ("q", "Q", "\x03"):
                break
            elif key is not None:
                node.stop()

            node.publish()
            rclpy.spin_once(node, timeout_sec=0.0)

            if node.status != last_status:
                sys.stdout.write(f"\rPerintah: {node.status:<14}")
                sys.stdout.flush()
                last_status = node.status
    finally:
        node.publish_final_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        print("\nRobot dihentikan.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
