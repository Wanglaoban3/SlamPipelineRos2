import sys
import math
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

bag = sys.argv[1]
topic = sys.argv[2]
r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
pts = []
while r.has_next():
    t, d, _ = r.read_next()
    if t == topic:
        m = deserialize_message(d, Odometry)
        p = m.pose.pose.position
        pts.append((p.x, p.y, p.z))
total = sum(math.dist(pts[i - 1], pts[i]) for i in range(1, len(pts))) if len(pts) > 1 else 0
print('topic %s: %d poses, path len %.1f m' % (topic, len(pts), total))
print('first:', ['%.2f' % v for v in pts[0]] if pts else None, ' last:', ['%.2f' % v for v in pts[-1]] if pts else None)
