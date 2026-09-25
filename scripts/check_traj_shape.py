import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

r = SequentialReader()
r.open(StorageOptions(uri='/root/data/rec_mapping', storage_id='sqlite3'), ConverterOptions('', ''))
pts, yaws = [], []
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == '/Odometry':
        m = deserialize_message(data, Odometry)
        p = m.pose.pose.position
        pts.append([p.x, p.y, p.z])
        q = m.pose.pose.orientation
        yaws.append(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

pts = np.array(pts)
yaws = np.unwrap(np.array(yaws))
path = np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1).sum()
net = np.linalg.norm(pts[-1, :2] - pts[0, :2])
print('poses: %d  path len: %.1f m  net displacement: %.1f m  ratio: %.2f' % (len(pts), path, net, net / max(path, 1e-9)))
print('first:', np.round(pts[0], 2), ' last:', np.round(pts[-1], 2))
print('yaw start %.1f deg -> end %.1f deg' % (np.degrees(yaws[0]), np.degrees(yaws[-1])))
print('z range: %.2f .. %.2f' % (pts[:, 2].min(), pts[:, 2].max()))
for i in range(0, len(pts), 5):
    print('  pose %2d: x=%7.2f y=%8.2f z=%6.2f yaw=%8.1f deg' % (i, pts[i, 0], pts[i, 1], pts[i, 2], np.degrees(yaws[i])))
