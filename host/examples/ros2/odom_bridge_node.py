"""ROS 2 bridge node for encoder-odometry-firmware (EXAMPLE, NOT RUN IN CI).

This file is only linted in CI; it has never been executed there, because CI
does not install ROS 2. It has also not been run against real hardware. Treat
it as a starting point and test it on your own robot.

It subscribes to ``cmd_vel`` (geometry_msgs/Twist), forwards each message to
the firmware as a VELOCITY_COMMAND, and publishes the firmware's ODOMETRY
frames as ``odom`` (nav_msgs/Odometry) plus the ``odom -> base_link``
transform. All protocol work is done by ``odobridge``; this file only moves
values in and out of ROS messages.

Run (on the Raspberry Pi, with ROS 2 sourced and odobridge installed)::

    python3 odom_bridge_node.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=921600

Parameters: ``port``, ``baud``, ``odom_frame`` (default ``odom``),
``base_frame`` (default ``base_link``), ``publish_tf`` (default true).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from odobridge.link import Client, SerialTransport
from odobridge.protocol import Odometry, VelocityCommand


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a rotation by ``yaw`` about z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class OdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("encoder_odometry_bridge")
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        baud = int(self.declare_parameter("baud", 921600).value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)

        self.client = Client(SerialTransport(str(port), baud))
        self.odom_pub = self.create_publisher(OdometryMsg, "odom", 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        # Poll the serial port faster than the firmware's 20 Hz telemetry.
        self.create_timer(0.01, self.poll)

    def on_cmd_vel(self, msg: Twist) -> None:
        # The firmware watchdog stops the motors if cmd_vel stops arriving,
        # so nothing here needs to time out.
        self.client.send(VelocityCommand(msg.linear.x, msg.linear.y, msg.angular.z))

    def poll(self) -> None:
        for packet in self.client.poll():
            if isinstance(packet.message, Odometry):
                self.publish(packet.message)

    def publish(self, o: Odometry) -> None:
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(o.theta)
        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = o.x
        msg.pose.pose.position.y = o.y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = o.vx
        msg.twist.twist.linear.y = o.vy
        msg.twist.twist.angular.z = o.wz
        self.odom_pub.publish(msg)
        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = o.x
            t.transform.translation.y = o.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.client.send(VelocityCommand(0.0, 0.0, 0.0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
