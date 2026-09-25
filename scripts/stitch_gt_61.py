#!/usr/bin/env python3
"""Stitch scene-0061 sweeps with GT reference poses (from /odom_gt in 61_ros2) + per-point
motion compensation, producing the upper-bound map. Diagnoses whether off-road artifacts
come from undistortion (fixable) or from moving objects / data (inherent)."""
import sys

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

BAG = sys.argv[1] if len(sys.argv) > 1 else '/root/data/61_ros2'
OUT = sys.argv[2] if len(sys.argv) > 2 else '/root/data/globalmap_61_gt.pcd'
SWEEP_DT = 0.05


def read_bag():
    r = SequentialReader()
    r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('', ''))
    scans, gt_t, gt_p, gt_q = [], [], [], []
    from sensor_msgs.msg import PointCloud2 as PC2
    from nav_msgs.msg import Odometry
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == '/points_raw':
            m = deserialize_message(data, PC2)
            arr = np.frombuffer(bytes(m.data), dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')]))
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          np.array([arr['x'], arr['y'], arr['z']]).T))
        elif topic == '/odom_gt':
            m = deserialize_message(data, Odometry)
            gt_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            gt_p.append([p.x, p.y, p.z])
            gt_q.append([q.w, q.x, q.y, q.z])
    return scans, np.array(gt_t), np.array(gt_p), np.array(gt_q)


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_slerp(q0, q1, a):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    if np.dot(q0, q1) < 0:
        q1 = -q1
    d = np.clip(np.dot(q0, q1), -1, 1)
    th = np.arccos(d)
    if th < 1e-6:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    s = np.sin(th)
    return np.sin((1 - a) * th) / s * q0 + np.sin(a * th) / s * q1


def pose_at(t):
    i = int(np.searchsorted(gt_t, t))
    i = max(1, min(i, len(gt_t) - 1))
    a = np.clip((t - gt_t[i - 1]) / max(1e-6, gt_t[i] - gt_t[i - 1]), 0, 1)
    q = quat_slerp(gt_q[i - 1], gt_q[i], a)
    p = gt_p[i - 1] * (1 - a) + gt_p[i] * a
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q)
    T[:3, 3] = p
    return T


scans, gt_t, gt_p, gt_q = read_bag()
print(f'scans: {len(scans)}, gt poses: {len(gt_t)} @ ~{1 / np.diff(gt_t).mean():.0f} Hz')

# static lidar->ego extrinsic (nuScenes calibrated_sensor, HDL-32E, same fleet across scenes)
CAL_Q = [0.70779551, -0.00649224, 0.01064621, -0.70630731]  # wxyz
CAL_T = np.array([0.943713, 0.0, 1.84023])
R_LI = quat_to_mat(CAL_Q)
T_LI = CAL_T

all_pts = []
for t_scan, pts in scans:
    n = len(pts)
    rng_all = np.linalg.norm(pts, axis=1)
    # near/far gate + HDL-32E elevation gate (drops nuScenes ghost returns high in the air)
    valid = np.isfinite(pts).all(axis=1) & (rng_all > 1.0) & (rng_all < 150.0) \
        & (np.abs(pts[:, 2]) / np.maximum(rng_all, 1e-6) < 0.21)
    pts = pts[valid]
    n = len(pts)
    if n == 0:
        continue
    T0 = pose_at(t_scan)  # GT base_link pose at scan end
    R_T0, t_T0 = T0[:3, :3], T0[:3, 3]
    fracs = np.linspace(0.0, 1.0, n, endpoint=False)
    pts_out = np.empty((n, 3))
    for chunk in np.array_split(np.arange(n), 8):
        i0, i1 = chunk[0], chunk[-1] + 1
        t_pt = t_scan - (1.0 - fracs[i0:i1].mean()) * SWEEP_DT
        T_pt = pose_at(t_pt)
        D = np.linalg.inv(T0) @ T_pt  # ego(t_pt) -> ego(t0)
        R_d, t_d = D[:3, :3], D[:3, 3]
        # lidar(t_pt) -> ego(t_pt) -> ego(t0) -> global
        p1 = pts[i0:i1] @ R_LI.T + T_LI
        p2 = p1 @ R_d.T + t_d
        pts_out[i0:i1] = p2 @ R_T0.T + t_T0
    all_pts.append(pts_out.astype(np.float32))

cloud = np.vstack(all_pts)
print(f'points before voxel: {len(cloud)}')
key = np.floor(cloud / 0.08).astype(np.int64)
_, idx = np.unique(key, axis=0, return_index=True)
cloud = cloud[np.sort(idx)]
print(f'after 0.08m voxel: {len(cloud)}')
print(f'extent x [{cloud[:,0].min():.1f}, {cloud[:,0].max():.1f}] y [{cloud[:,1].min():.1f}, {cloud[:,1].max():.1f}]')

n = len(cloud)
buf = np.zeros(n, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')]))
buf['x'], buf['y'], buf['z'] = cloud[:, 0], cloud[:, 1], cloud[:, 2]
header = (
    '# .PCD v0.7 - Point Cloud Data file format\n'
    'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
    f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
).encode('ascii')
with open(OUT, 'wb') as f:
    f.write(header)
    f.write(buf.tobytes())
print('written:', OUT)
