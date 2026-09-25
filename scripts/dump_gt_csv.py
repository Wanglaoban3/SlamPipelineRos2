"""Dump a pose topic from a rosbag2 into the GT csv format used by eval_plot.py."""
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

bag, topic, out = sys.argv[1], sys.argv[2], sys.argv[3]
r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
rows = []
while r.has_next():
    t, data, _ = r.read_next()
    if t != topic:
        continue
    m = deserialize_message(data, Odometry)
    t_us = m.header.stamp.sec * 1_000_000 + m.header.stamp.nanosec // 1000
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    rows.append((t_us, p.x, p.y, p.z, q.w, q.x, q.y, q.z))

with open(out, 'w') as f:
    f.write('t_us,x,y,z,qw,qx,qy,qz\n')
    for row in rows:
        f.write('%d,%f,%f,%f,%f,%f,%f,%f\n' % row)
print('wrote %s: %d poses' % (out, len(rows)))
