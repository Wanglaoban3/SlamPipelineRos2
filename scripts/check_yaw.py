import csv
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


r = SequentialReader()
r.open(StorageOptions(uri='/root/data/rec_mapping', storage_id='sqlite3'), ConverterOptions('', ''))
t_od, yaw_od, p_od = [], [], []
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == '/Odometry':
        m = deserialize_message(data, Odometry)
        t_od.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        yaw_od.append(yaw_from_quat(m.pose.pose.orientation))
        p_od.append([m.pose.pose.position.x, m.pose.pose.position.y])

gt = list(csv.DictReader(open('/root/data/nuscenes_demo_gt.csv')))
t_gt = np.array([int(g['t_us']) / 1e6 for g in gt])
yaw_gt = np.array([np.arctan2(2 * (float(g['qw']) * float(g['qz']) + float(g['qx']) * float(g['qy'])),
                              1 - 2 * (float(g['qy']) ** 2 + float(g['qz']) ** 2)) for g in gt])

t_od = np.array(t_od)
yaw_od = np.unwrap(np.array(yaw_od))
yaw_gt = np.unwrap(yaw_gt)

idx = np.abs(t_od[:, None] - t_gt[None, :]).argmin(axis=1)
rel_od = yaw_od - yaw_od[0]
rel_gt = yaw_gt[idx] - yaw_gt[idx[0]]
drift = rel_od - rel_gt
print('frames:', len(t_od))
print('GT total yaw change: %.1f deg' % np.degrees(rel_gt[-1]))
print('odom total yaw change: %.1f deg' % np.degrees(rel_od[-1]))
print('yaw drift over run: start %.1f deg -> end %.1f deg' % (np.degrees(drift[0]), np.degrees(drift[-1])))
print('yaw drift per frame (deg):', np.round(np.degrees(drift), 1))
