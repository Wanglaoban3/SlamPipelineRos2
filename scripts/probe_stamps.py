#!/usr/bin/env python3
"""Probe: subscribe pipeline topics during playback, report counts + first stamps."""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry


class Probe(Node):
    def __init__(self):
        super().__init__('probe')
        self.counts = {}
        self.first = {}
        self.last = {}
        for topic, typ in [('/points_raw', PointCloud2), ('/filtered_points', PointCloud2),
                           ('/Odometry', Odometry), ('/imu/data', Imu),
                           ('/cloud_registered_body', PointCloud2), ('/cloud_registered', PointCloud2)]:
            self.create_subscription(typ, topic, self.make_cb(topic), 50)

    def make_cb(self, topic):
        def cb(msg):
            self.counts[topic] = self.counts.get(topic, 0) + 1
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if topic not in self.first:
                self.first[topic] = t
                print('first %s stamp %.3f' % (topic, t), flush=True)
            self.last[topic] = t
        return cb


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    rclpy.init()
    node = Probe()
    import time
    t_end = time.time() + dur
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    print('===== counts =====')
    for topic in ['/points_raw', '/filtered_points', '/Odometry', '/imu/data',
                  '/cloud_registered_body', '/cloud_registered']:
        c = node.counts.get(topic, 0)
        f = node.first.get(topic)
        l = node.last.get(topic)
        span = (l - f) if (f and l) else 0
        print('%-18s %5d msgs  span %.2f s' % (topic, c, span))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
