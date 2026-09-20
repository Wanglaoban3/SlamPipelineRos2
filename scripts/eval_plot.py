#!/usr/bin/env python3
"""
Evaluate recorded odometry/localization bags against nuScenes ego_pose ground truth.

Reads one or more rosbag2 dirs, extracts pose trajectories from the given topics,
time-aligns them to the GT CSV (nearest timestamp), Umeyama-aligns each trajectory
to GT (SE(3), similarity without scale), and reports ATE (RMSE of position error).
Plots BEV trajectories to a PNG.

Usage:
  python3 eval_plot.py --gt /root/data/xxx_gt.csv \
      --bag /root/data/rec_fastlio=/Odometry \
      --bag /root/data/rec_fused=/odometry/filtered \
      --bag /root/data/rec_gps=/odometry/gps \
      --bag /root/data/rec_reloc=/hdl_localization/odom \
      --out /mnt/h/projects/slam-pipeline-ros2/results/trajectories.png
"""
import argparse
import csv
import math
import os

import numpy as np


def read_gt(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            t = int(r['t_us']) / 1e6
            p = np.array([float(r['x']), float(r['y']), float(r['z'])])
            q = np.array([float(r['qw']), float(r['qx']), float(r['qy']), float(r['qz'])])
            rows.append((t, p, q))
    return rows


def umeyama(src, dst):
    """Least-squares rigid+uniform-scale transform mapping src->dst. Returns (R, t, s)."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    s_centered = src - mu_s
    d_centered = dst - mu_d
    cov = d_centered.T @ s_centered / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (s_centered ** 2).sum() / len(src)
    scale = np.trace(np.diag(D) @ S) / var_s
    t = mu_d - scale * R @ mu_s
    return R, t, scale


def apply_se3(R, t, scale, pts):
    return (scale * (R @ pts.T)).T + t


def ate(gt_pts, est_pts):
    return float(np.sqrt(((gt_pts - est_pts) ** 2).sum(axis=1).mean()))


def read_bag_poses(bag_dir, topic):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'), ConverterOptions('', ''))
    type_map = {}
    for ti in reader.get_all_topics_and_types():
        type_map[ti.name] = ti.type
    if topic not in type_map:
        return []
    msg_type = get_message(type_map[topic])

    poses = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        poses.append((t, np.array([p.x, p.y, p.z])))
    return poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', required=True)
    parser.add_argument('--bag', action='append', required=True, help='bagdir=topic (repeatable)')
    parser.add_argument('--out', required=True)
    parser.add_argument('--title', default='trajectory evaluation')
    args = parser.parse_args()

    gt = read_gt(args.gt)
    gt_t = np.array([g[0] for g in gt])
    gt_p = np.array([g[1] for g in gt])

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 9))
    gt_xy = np.array([p[:2] for _, p, _ in gt])
    ax.plot(gt_xy[:, 0], gt_xy[:, 1], 'k-', lw=2, label='ground truth (ego_pose)')

    report = []
    colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange', 'tab:purple']
    for i, spec in enumerate(args.bag):
        bag_dir, topic = spec.split('=', 1)
        poses = read_bag_poses(bag_dir, topic)
        if not poses:
            report.append(f'{topic}: NO MESSAGES in {bag_dir}')
            continue
        t_est = np.array([p[0] for p in poses])
        p_est = np.array([p[1] for p in poses])

        # nearest-time match against GT
        idx = np.abs(t_est[:, None] - gt_t[None, :]).argmin(axis=1)
        dt = np.abs(t_est - gt_t[idx])
        mask = dt < 0.2
        if mask.sum() < 4:
            report.append(f'{topic}: only {mask.sum()} time-matched samples')
            continue

        matched_est = p_est[mask]
        matched_gt = gt_p[idx[mask]]

        R, t, s = umeyama(matched_est, matched_gt)
        aligned = apply_se3(R, t, s, matched_est)
        err = ate(matched_gt, aligned)
        report.append(f'{topic}: {len(matched_est)} matched poses, ATE = {err:.3f} m (after SE3+scale align)')

        a2 = apply_se3(R, t, s, p_est[:, :2].T).T if False else None
        # plot aligned full trajectory (2D): apply R,s,t to xy only approximation using full 3D then take xy
        full_aligned = apply_se3(R, t, s, p_est)
        ax.plot(full_aligned[:, 0], full_aligned[:, 1], color=colors[i % len(colors)], lw=1.2,
                label=f'{topic} (ATE {err:.2f} m)')

    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9)
    ax.set_title(args.title)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out, dpi=130, bbox_inches='tight')
    print(f'plot saved: {args.out}')
    print('\n===== EVALUATION REPORT =====')
    for r in report:
        print(r)


if __name__ == '__main__':
    main()
