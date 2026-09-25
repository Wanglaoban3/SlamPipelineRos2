#!/usr/bin/env python3
"""Convert nuscenes2bag ROS1 bag (61.bag, 20Hz sweeps) to a ROS2 bag for the SLAM pipeline.

- lidar_top  -> /points_raw   (20Hz HDL-32E sweeps)
- /odom      -> /odom_gt      (154Hz ego_pose reference, global frame - for eval only)
- can_bus    -> /imu/data (50Hz, + 1.5s static init window), /gps/fix, /odom_wheel
- /tf_static from calibrated_sensor (base_link <- lidar_top)
Radar (custom msg) and cameras are skipped.
"""
import json
import os
import sys

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

BAG = '/mnt/h/datasets/nuscenes-rosbags/61.bag'
NUSCENES = '/mnt/h/datasets/nuscenes-mini'
CANBUS = '/mnt/h/datasets/can_bus_extract/can_bus'
SCENE = 'scene-0061'
OUT = sys.argv[1] if len(sys.argv) > 1 else '/root/data/61_ros2'

ts1 = get_typestore(Stores.ROS1_NOETIC)
ts2 = get_typestore(Stores.ROS2_HUMBLE)

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# ---------- calib from mini ----------
vdir = os.path.join(NUSCENES, 'v1.0-mini')
calib_rows = json.load(open(os.path.join(vdir, 'calibrated_sensor.json')))
sensor_rows = json.load(open(os.path.join(vdir, 'sensor.json')))
sensor_by_token = {s['token']: s for s in sensor_rows}
lidar_calib = next(c for c in calib_rows
                   if sensor_by_token[c['sensor_token']]['channel'] == 'LIDAR_TOP')
cal_T = lidar_calib['translation']
cal_R = lidar_calib['rotation']

# ---------- can_bus ----------
pose_data = json.load(open(os.path.join(CANBUS, f'{SCENE}_pose.json')))
cb_t = np.array([r['utime'] for r in pose_data]) / 1e6
print(f'can_bus: {len(cb_t)} poses @ ~{1 / np.diff(cb_t).mean():.0f} Hz')

bag_t0 = None
bag_t1 = None

if os.path.exists(OUT):
    print(f'ERROR: {OUT} exists')
    sys.exit(1)

writer = SequentialWriter()
writer.open(StorageOptions(uri=OUT, storage_id='sqlite3'), ConverterOptions('', ''))

topics = [
    ('/points_raw', 'sensor_msgs/msg/PointCloud2'),
    ('/odom_gt', 'nav_msgs/msg/Odometry'),
    ('/imu/data', 'sensor_msgs/msg/Imu'),
    ('/gps/fix', 'sensor_msgs/msg/NavSatFix'),
    ('/odom_wheel', 'nav_msgs/msg/Odometry'),
    ('/tf_static', 'tf2_msgs/msg/TFMessage'),
]
for name, typ in topics:
    qos = ''
    if name == '/tf_static':
        # transient_local is required: TF listeners subscribe transient_local and will
        # never receive a volatile /tf_static publisher (rosbag2 expects numeric enums)
        qos = ('[{"history": 1, "depth": 1, "reliability": 1, "durability": 1, '
               '"deadline": {"sec": 0, "nsec": 0}, "lifespan": {"sec": 0, "nsec": 0}, '
               '"liveliness": 1, "liveliness_lease_duration": {"sec": 0, "nsec": 0}, '
               '"avoid_ros_namespace_conventions": false}]')
    writer.create_topic(TopicMetadata(name=name, type=typ, serialization_format='cdr', offered_qos_profiles=qos))


def us2time(us):
    from builtin_interfaces.msg import Time
    m = Time()
    m.sec = int(us // 1_000_000)
    m.nanosec = int((us % 1_000_000) * 1000)
    return m


# static TF base_link <- lidar_top (stamp = bag start, keeps all header stamps monotonic;
# a backward jump at playback start breaks message_filters under sim time)
imu_t0 = cb_t[0]
tf_msg = TFMessage()
tr = TransformStamped()
tr.header.frame_id = 'base_link'
tr.header.stamp = us2time((imu_t0 - 1.5) * 1e6)
tr.child_frame_id = 'lidar_top'
tr.transform.translation.x = float(cal_T[0])
tr.transform.translation.y = float(cal_T[1])
tr.transform.translation.z = float(cal_T[2])
tr.transform.rotation.w = float(cal_R[0])
tr.transform.rotation.x = float(cal_R[1])
tr.transform.rotation.y = float(cal_R[2])
tr.transform.rotation.z = float(cal_R[3])
tf_msg.transforms.append(tr)
writer.write('/tf_static', serialize_message(tf_msg), int((imu_t0 - 1.5) * 1e9))

# static IMU init window (1.5s before first real IMU sample)
for k in range(75):
    t_us = (imu_t0 - 1.5 + k * 0.02) * 1e6
    m = Imu()
    m.header.frame_id = 'base_link'
    m.header.stamp = us2time(t_us)
    m.linear_acceleration.z = 9.5
    m.orientation.w = 1.0
    writer.write('/imu/data', serialize_message(m), int(t_us * 1000))

# can_bus streams
datum_lat, datum_lon = 42.345, -71.06
m_per_deg_lat = 111320.0
m_per_deg_lon = 111320.0 * np.cos(np.radians(datum_lat))
t_lo, t_hi = None, None
for i, t in enumerate(cb_t):
    t_us = t * 1e6
    imu = Imu()
    imu.header.frame_id = 'base_link'
    imu.header.stamp = us2time(t_us)
    imu.angular_velocity.x = float(pose_data[i]['rotation_rate'][0])
    imu.angular_velocity.y = float(pose_data[i]['rotation_rate'][1])
    imu.angular_velocity.z = float(pose_data[i]['rotation_rate'][2])
    imu.linear_acceleration.x = float(pose_data[i]['accel'][0])
    imu.linear_acceleration.y = float(pose_data[i]['accel'][1])
    imu.linear_acceleration.z = float(pose_data[i]['accel'][2])
    q = pose_data[i]['orientation']
    imu.orientation.w = float(q[0])
    imu.orientation.x = float(q[1])
    imu.orientation.y = float(q[2])
    imu.orientation.z = float(q[3])
    writer.write('/imu/data', serialize_message(imu), int(t_us * 1000))

    if i % 5 == 0:  # 10Hz
        pos = pose_data[i]['pos']
        fix = NavSatFix()
        fix.header.frame_id = 'gps'
        fix.header.stamp = us2time(t_us)
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.latitude = float(datum_lat + pos[1] / m_per_deg_lat)
        fix.longitude = float(datum_lon + pos[0] / m_per_deg_lon)
        fix.altitude = float(pos[2])
        writer.write('/gps/fix', serialize_message(fix), int(t_us * 1000))

        od = Odometry()
        od.header.frame_id = 'odom_wheel'
        od.header.stamp = us2time(t_us)
        od.child_frame_id = 'base_link'
        q = pose_data[i]['orientation']
        od.pose.pose.orientation.w = float(q[0])
        od.pose.pose.orientation.x = float(q[1])
        od.pose.pose.orientation.y = float(q[2])
        od.pose.pose.orientation.z = float(q[3])
        v = pose_data[i]['vel']
        od.twist.twist.linear.x = float(np.linalg.norm(v[:2]))
        od.twist.twist.angular.z = float(pose_data[i]['rotation_rate'][2])
        writer.write('/odom_wheel', serialize_message(od), int(t_us * 1000))
print('can_bus streams written')

# lidar + /odom_gt from the ROS1 bag
n_lidar = n_odom = 0
DT_XYZI = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
with Reader(BAG) as reader:
    conns = [c for c in reader.connections if c.topic in ('lidar_top', '/odom')]
    for conn, ts_, raw in reader.messages(connections=conns):
        msg = ts1.deserialize_ros1(raw, conn.msgtype)
        stamp_ns = (msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000) * 1_000_000
        if conn.topic == 'lidar_top':
            # nuScenes lidar stamps mark the END of the sweep; FAST-LIO treats the header
            # stamp as the FIRST point and spans [stamp, stamp + max(curvature)] = 50 ms.
            # Shift stamps back by one sweep so the IMU integration window covers the true
            # firing interval - otherwise undistortion uses the FUTURE window and turns
            # ghost by ~2.5 deg (25 deg/s yaw rate x 0.1 s window misalignment).
            stamp_ns -= 50_000_000
        if conn.topic == 'lidar_top':
            # elevation gate: HDL-32E beams span -31.5°..+11.5°; nuScenes sweeps contain
            # known ghost returns high in the air (|z| up to 100 m) that pollute maps
            arr = np.frombuffer(bytes(msg.data), dtype=DT_XYZI)
            rng = np.sqrt(arr['x'] ** 2 + arr['y'] ** 2 + arr['z'] ** 2)
            ok = (rng > 1.0) & (rng < 150.0) & (np.abs(arr['z']) / np.maximum(rng, 1e-6) < 0.21)
            keep = arr[ok]
            msg.width = len(keep)
            msg.row_step = 16 * len(keep)
            msg.data = np.frombuffer(keep.tobytes(), dtype=np.uint8)
            msg.is_dense = True
            data = bytes(ts2.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2'))
            writer.write('/points_raw', data, stamp_ns)
            n_lidar += 1
        else:
            data = bytes(ts2.serialize_cdr(msg, 'nav_msgs/msg/Odometry'))
            writer.write('/odom_gt', data, stamp_ns)
            n_odom += 1

print(f'bag written: {OUT}')
print(f'  /points_raw : {n_lidar} msgs (20Hz sweeps)')
print(f'  /odom_gt    : {n_odom} msgs (reference)')
