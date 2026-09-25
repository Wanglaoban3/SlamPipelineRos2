#!/usr/bin/env python3
"""Stitch an upper-bound map from the same nuScenes scans using GT ego poses.

Per-point motion compensation uses the can_bus 50Hz pose channel (relative ego motion
within each 50ms sweep), anchored at the nuScenes ego_pose GT pose of each sweep.
If this map is clean while odometry-built maps are ghosted, the bottleneck is the
2Hz keyframe data, not the SLAM pipeline.
"""
import argparse
import json
import os
import pickle

import numpy as np


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_slerp(q0, q1, t):
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    if np.dot(q0, q1) < 0:
        q1 = -q1
    d = np.clip(np.dot(q0, q1), -1, 1)
    theta = np.arccos(d)
    if theta < 1e-6:
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    s = np.sin(theta)
    return (np.sin((1 - t) * theta) / s) * q0 + (np.sin(t * theta) / s) * q1


def pose_from_quat_pos(q, p):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q)
    T[:3, 3] = p
    return T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nuscenes', required=True)
    parser.add_argument('--canbus', required=True)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--voxel', type=float, default=0.08)
    args = parser.parse_args()

    vdir = os.path.join(args.nuscenes, 'v1.0-mini')
    sd_rows = json.load(open(os.path.join(vdir, 'sample_data.json')))
    ep_rows = json.load(open(os.path.join(vdir, 'ego_pose.json')))
    cs_rows = json.load(open(os.path.join(vdir, 'calibrated_sensor.json')))
    sensor_rows = json.load(open(os.path.join(vdir, 'sensor.json')))
    scene_rows = json.load(open(os.path.join(vdir, 'scene.json')))
    sample_rows = json.load(open(os.path.join(vdir, 'sample.json')))

    scene = next(s for s in scene_rows if s['name'] == args.scene)
    sample_by_token = {s['token']: s for s in sample_rows}
    tokens = set()
    tok = scene['first_sample_token']
    while tok:
        tokens.add(tok)
        tok = sample_by_token[tok]['next']

    sensor_by_token = {s['token']: s for s in sensor_rows}
    cs_by_token = {c['token']: c for c in cs_rows}
    ep_by_token = {e['token']: e for e in ep_rows}

    lidar_cs = {c['token'] for c in cs_rows
                if sensor_by_token[c['sensor_token']]['channel'] == 'LIDAR_TOP'}
    lidar_sds = sorted((r for r in sd_rows
                        if r['calibrated_sensor_token'] in lidar_cs
                        and r['sample_token'] in tokens
                        and os.path.isfile(os.path.join(args.nuscenes, r['filename']))),
                       key=lambda r: r['timestamp'])
    print(f'lidar sweeps: {len(lidar_sds)}')

    calib = cs_by_token[lidar_sds[0]['calibrated_sensor_token']]
    T_ego_lidar = pose_from_quat_pos(calib['rotation'], calib['translation'])

    # can_bus 50Hz fused INS (for intra-sweep relative motion)
    pose_file = os.path.join(args.canbus, f'{args.scene}_pose.json')
    if not os.path.isfile(pose_file):
        pose_file = os.path.join(args.canbus, 'can_bus', f'{args.scene}_pose.json')
    can = json.load(open(pose_file))
    cb_t = np.array([r['utime'] for r in can], dtype=np.float64) / 1e6
    cb_q = np.array([r['orientation'] for r in can])
    cb_p = np.array([r['pos'] for r in can])
    print(f'can_bus poses: {len(cb_t)} @ ~{1 / np.diff(cb_t).mean():.0f} Hz')

    def cb_pose(t):
        i = int(np.searchsorted(cb_t, t))
        i = max(1, min(i, len(cb_t) - 1))
        t0, t1 = cb_t[i - 1], cb_t[i]
        a = np.clip((t - t0) / max(1e-6, t1 - t0), 0, 1)
        q = quat_slerp(cb_q[i - 1], cb_q[i], a)
        p = cb_p[i - 1] * (1 - a) + cb_p[i] * a
        return pose_from_quat_pos(q, p)

    all_pts = []
    sweep_dt = 0.05  # HDL-32E at 20Hz
    for sd in lidar_sds:
        t0 = sd['timestamp'] / 1e6
        pc = np.fromfile(os.path.join(args.nuscenes, sd['filename']), dtype=np.float32).reshape(-1, 5)
        xyz = pc[:, :3]
        valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1.0)
        xyz = xyz[valid]
        n = len(xyz)
        ring_frac = np.linspace(0.0, 1.0, n, endpoint=False)  # points ordered along the sweep

        ep = ep_by_token[sd['ego_pose_token']]
        T_ge = pose_from_quat_pos(ep['rotation'], ep['translation'])

        T0 = T_ge @ T_ego_lidar  # lidar(t0) -> global

        # process in chunks: relative ego motion per point from can_bus
        pts_out = np.empty((n, 3), dtype=np.float64)
        pts_out[:] = xyz @ T0[:3, :3].T + T0[:3, 3]
        # motion compensation: t_pt spans [t0 - sweep_dt, t0]
        for frac_chunk in np.array_split(np.arange(n), 8):
            i0, i1 = frac_chunk[0], frac_chunk[-1] + 1
            t_pt = t0 - (1.0 - ring_frac[i0:i1].mean()) * sweep_dt
            dT = cb_pose(t_pt) @ np.linalg.inv(cb_pose(t0))  # ego-frame motion since t0
            chunk = xyz[i0:i1] @ (T_ego_lidar[:3, :3] @ dT[:3, :3]).T + \
                    (T_ego_lidar[:3, :3] @ dT[:3, 3] + T_ego_lidar[:3, 3])
            pts_out[i0:i1] = chunk @ T_ge[:3, :3].T + T_ge[:3, 3]
        all_pts.append(pts_out.astype(np.float32))

    cloud = np.vstack(all_pts)
    print(f'total points before voxel: {len(cloud)}')

    # voxel grid downsample
    key = np.floor(cloud / args.voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    cloud = cloud[np.sort(idx)]
    print(f'after {args.voxel}m voxel: {len(cloud)}')
    print(f'extent x [{cloud[:,0].min():.1f}, {cloud[:,0].max():.1f}] '
          f'y [{cloud[:,1].min():.1f}, {cloud[:,1].max():.1f}]')

    # write binary PCD (PointXYZI, intensity=0)
    n = len(cloud)
    buf = io_ = np.zeros(n, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')]))
    buf['x'] = cloud[:, 0]
    buf['y'] = cloud[:, 1]
    buf['z'] = cloud[:, 2]
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n'
    ).encode('ascii')
    with open(args.out, 'wb') as f:
        f.write(header)
        f.write(buf.tobytes())
    print('written:', args.out)


if __name__ == '__main__':
    main()
