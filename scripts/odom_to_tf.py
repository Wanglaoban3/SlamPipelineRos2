#!/usr/bin/env python3
"""Relay FAST-LIO /Odometry into TF robot_odom -> base_link for motion prediction."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('frame_id', 'robot_odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.cb, 20)
        self.br = TransformBroadcaster(self)

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.get_parameter('frame_id').value
        t.child_frame_id = self.get_parameter('child_frame_id').value
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
