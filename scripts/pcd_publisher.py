#!/usr/bin/env python3
"""Publish a binary PointXYZI PCD as sensor_msgs/PointCloud2 (transient_local QoS)."""
import argparse
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def read_binary_pcd(path):
    with open(path, 'rb') as f:
        n_points = None
        data_mode = None
        sizes = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('#'):
                continue
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'POINTS':
                n_points = int(parts[1])
            elif parts[0] == 'SIZE':
                sizes = [int(v) for v in parts[1:]]
            elif parts[0] == 'DATA':
                data_mode = parts[1]
                break
        if data_mode != 'binary' or n_points is None:
            raise RuntimeError('only binary PCD supported: ' + path)
        point_step = sum(sizes)
        raw = f.read(point_step * n_points)
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
    arr = np.frombuffer(raw[:len(raw) // 16 * 16], dtype=dt)
    return arr


class PcdPublisher(Node):
    def __init__(self, pcd_path, topic, frame_id):
        super().__init__('pcd_publisher_' + topic.strip('/').replace('/', '_'))
        self.declare_parameter('frame_id', frame_id)
        self.frame_id = frame_id
        arr = read_binary_pcd(pcd_path)
        n = len(arr)
        self.get_logger().info(f'loaded {n} points from {pcd_path}')

        # pack as x,y,z,intensity float32 (16 bytes/pt)
        buf = np.zeros(n, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')]))
        buf['x'] = arr['x']
        buf['y'] = arr['y']
        buf['z'] = arr['z']
        buf['i'] = arr['i']

        self.msg = PointCloud2()
        self.msg.header.frame_id = frame_id
        self.msg.height = 1
        self.msg.width = n
        self.msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.msg.is_bigendian = False
        self.msg.point_step = 16
        self.msg.row_step = 16 * n
        self.msg.is_dense = True
        self.msg.data = buf.tobytes()

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(PointCloud2, topic, qos)

        self.stamp = self.get_clock().now().to_msg()
        self.publish_once()
        self.timer = self.create_timer(2.0, self.publish_once)

    def publish_once(self):
        m = PointCloud2()
        m = self.msg
        m.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(m)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd')
    parser.add_argument('topic')
    parser.add_argument('--frame', default='map')
    args = parser.parse_args()

    rclpy.init()
    node = PcdPublisher(args.pcd, args.topic, args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
