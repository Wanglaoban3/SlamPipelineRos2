#!/usr/bin/env python3
"""
nuScenes(+can_bus) -> ROS2 bag converter for the SLAM pipeline demo.

can_bus (json repack) channels used:
  <scene>_pose.json : 50Hz fused RT3000 INS: accel, orientation(wxyz, yaw-only),
                      rotation_rate, vel, pos (metric local frame), utime (us)

Generates a bag containing (for one selected scene):
  /points_raw   sensor_msgs/PointCloud2   LIDAR_TOP sweeps (frame: lidar_top)
  /imu/data     sensor_msgs/Imu           can_bus pose channel (fused INS) (frame: base_link)
  /gps/fix      sensor_msgs/NavSatFix     synthesized from pos around a datum (real UTM factor path)
  /odom_wheel   nav_msgs/Odometry         vehicle speed + yaw rate (wheel-like odometry)
  /tf_static    tf2_msgs/TFMessage        base_link -> lidar_top (from calibrated_sensor)

Also writes ground truth CSV next to the bag: <out>_gt.csv with ego_pose at lidar stamps.

Usage (inside WSL with ROS2 sourced):
  python3 convert_nuscenes_to_bag.py \
      --nuscenes /mnt/h/datasets/nuscenes-mini \
      --canbus /mnt/h/datasets/can_bus_extract/can_bus \
      --scene scene-0916 \
      --out /root/data/nuscenes_demo
"""
import argparse
import json
import os
import sys

import numpy as np


def load_json(path):
    with open(path, 'r') as f:
        return json.load(f)


def index_by_key(rows, key='token'):
    return {row[key]: row for row in rows}


def find_canbus_file(canbus_root, scene, channel):
    for base in (canbus_root, os.path.join(canbus_root, 'can_bus'), os.path.dirname(canbus_root)):
        cand = os.path.join(base, f'{scene}_{channel}.json')
        if os.path.isfile(cand):
            return cand
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nuscenes', required=True)
    parser.add_argument('--canbus', required=True)
    parser.add_argument('--scene', default=None, help='scene name, e.g. scene-0916; default: pick the longest')
    parser.add_argument('--out', required=True, help='output bag directory (without .db3 suffix)')
    parser.add_argument('--lidar-decim', type=int, default=1, help='take every Nth lidar sweep')
    parser.add_argument('--datum-lat', type=float, default=42.345, help='datum latitude for pos->latlon synthesis (Boston Seaport)')
    parser.add_argument('--datum-lon', type=float, default=-71.06, help='datum longitude')
    args = parser.parse_args()

    root = args.nuscenes
    version_dir = os.path.join(root, 'v1.0-mini')

    scene_rows = load_json(os.path.join(version_dir, 'scene.json'))
    log_rows = load_json(os.path.join(version_dir, 'log.json'))
    sample_rows = load_json(os.path.join(version_dir, 'sample.json'))
    sample_data_rows = load_json(os.path.join(version_dir, 'sample_data.json'))
    ego_pose_rows = load_json(os.path.join(version_dir, 'ego_pose.json'))
    calib_rows = load_json(os.path.join(version_dir, 'calibrated_sensor.json'))
    sensor_rows = load_json(os.path.join(version_dir, 'sensor.json'))

    sample_by_token = index_by_key(sample_rows)
    sd_by_token = index_by_key(sample_data_rows)
    ep_by_token = index_by_key(ego_pose_rows)
    calib_by_token = index_by_key(calib_rows)
    sensor_by_token = index_by_key(sensor_rows)
    log_by_token = index_by_key(log_rows)

    # collect lidar sweeps from sample_data (some slim releases omit sample['data']),
    # keep only entries whose file exists locally (this mini has keyframes only)
    lidar_calib_tokens = {c['token'] for c in calib_rows
                          if sensor_by_token[c['sensor_token']]['channel'] == 'LIDAR_TOP'}
    scene_lidar = {}  # scene_token -> [lidar sample_data]
    n_exist = 0
    for sd in sample_data_rows:
        if sd['calibrated_sensor_token'] not in lidar_calib_tokens:
            continue
        if not os.path.isfile(os.path.join(root, sd['filename'])):
            continue
        n_exist += 1
        sample = sample_by_token.get(sd['sample_token'])
        if sample is None:
            continue
        scene_lidar.setdefault(sample['scene_token'], []).append(sd)
    print(f'debug: lidar calib tokens={len(lidar_calib_tokens)} existing lidar files={n_exist}')

    scenes = []
    for scene in scene_rows:
        sweeps = sorted(scene_lidar.get(scene['token'], []), key=lambda sd: sd['timestamp'])
        scenes.append((scene, sweeps))
    scenes = [s for s in scenes if s[1]]

    if args.scene:
        scene, lidar_sweeps = next(s for s in scenes if s[0]['name'] == args.scene)
    else:
        scene, lidar_sweeps = max(scenes, key=lambda s: s[1][-1]['timestamp'] - s[1][0]['timestamp'])
        print(f'auto-selected {scene["name"]}')
    print(f'scene {scene["name"]}: {len(lidar_sweeps)} lidar sweeps, '
          f'duration {(lidar_sweeps[-1]["timestamp"] - lidar_sweeps[0]["timestamp"]) / 1e6:.1f} s')

    lidar_sweeps = lidar_sweeps[::args.lidar_decim]

    log = log_by_token[scene['log_token']]
    print(f'log: {log["logfile"]}, location: {log["location"]}')

    # lidar calibration (lidar -> ego/base_link)
    lidar_sd0 = lidar_sweeps[0]
    calib = calib_by_token[lidar_sd0['calibrated_sensor_token']]
    cal_T = np.array(calib['translation'])
    cal_R = np.array(calib['rotation'])
    print(f'lidar translation: {cal_T}, quat(wxyz): {cal_R}')

    # ------------------------------------------------------------------
    # can_bus json data for this scene
    # ------------------------------------------------------------------
    pose_file = find_canbus_file(args.canbus, scene['name'], 'pose')
    if pose_file is None:
        print(f'ERROR: can_bus pose json for {scene["name"]} not found under {args.canbus}')
        sys.exit(1)
    print(f'can_bus pose file: {pose_file}')
    pose_data = load_json(pose_file)
    print(f'can_bus pose records: {len(pose_data)}')
    print('keys:', sorted(pose_data[0].keys()))

    pose_uts = np.array([r['utime'] for r in pose_data], dtype=np.float64) / 1e6  # seconds
    pose_accel = np.array([r['accel'] for r in pose_data])
    pose_rot_rate = np.array([r['rotation_rate'] for r in pose_data])
    pose_orientation = np.array([r['orientation'] for r in pose_data])  # wxyz
    pose_vel = np.array([r['vel'] for r in pose_data])
    pose_pos = np.array([r['pos'] for r in pose_data])
    mean_dt = np.diff(pose_uts).mean()
    print(f'pose channel: mean dt = {mean_dt * 1000:.1f} ms (~{1 / mean_dt:.0f} Hz)')

    # validation: pose-channel orientation vs ego_pose quaternion
    print('\n--- validation: can_bus orientation vs ego_pose ---')
    for sd in lidar_sweeps[::max(1, len(lidar_sweeps) // 5)][:5]:
        ep = ep_by_token[sd['ego_pose_token']]
        t = sd['timestamp'] / 1e6
        i = int(np.argmin(np.abs(pose_uts - t)))
        dt = pose_uts[i] - t
        q_ep = np.array(ep['rotation'])
        q_cb = pose_orientation[i]
        dot = abs(np.dot(q_ep, q_cb))
        ang = 2 * np.arccos(np.clip(dot, -1, 1))
        print(f'  t={t:.3f} dt={dt * 1e3:.1f}ms |q| angle diff={np.degrees(ang):.2f}deg '
              f'(ego q={np.round(q_ep, 3)}, can_bus q={np.round(q_cb, 3)})')

    # ------------------------------------------------------------------
    # ROS2 bag writing
    # ------------------------------------------------------------------
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointCloud2, PointField, Imu, NavSatFix, NavSatStatus
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from builtin_interfaces.msg import Time
    import struct

    def us2time(us):
        msg = Time()
        msg.sec = int(us // 1_000_000)
        msg.nanosec = int((us % 1_000_000) * 1000)
        return msg

    if os.path.exists(args.out):
        print(f'ERROR: output {args.out} exists')
        sys.exit(1)
    # note: rosbag2 creates the bag directory itself

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.out, storage_id='sqlite3'),
                ConverterOptions('', ''))

    topics = [
        ('/points_raw', 'sensor_msgs/msg/PointCloud2'),
        ('/imu/data', 'sensor_msgs/msg/Imu'),
        ('/gps/fix', 'sensor_msgs/msg/NavSatFix'),
        ('/odom_wheel', 'nav_msgs/msg/Odometry'),
        ('/tf_static', 'tf2_msgs/msg/TFMessage'),
    ]
    for name, typ in topics:
        qos = ''
        if name == '/tf_static':
            # transient_local so late-joining TF listeners receive it
            # (rosbag2 humble expects numeric rmw qos enums: KEEP_LAST=1, RELIABLE=1, TRANSIENT_LOCAL=1, AUTOMATIC=1)
            qos = ('[{"history": 1, "depth": 1, "reliability": 1, "durability": 1, '
                   '"deadline": {"sec": 0, "nsec": 0}, "lifespan": {"sec": 0, "nsec": 0}, '
                   '"liveliness": 1, "liveliness_lease_duration": {"sec": 0, "nsec": 0}, '
                   '"avoid_ros_namespace_conventions": false}]')
        writer.create_topic(TopicMetadata(name=name, type=typ, serialization_format='cdr', offered_qos_profiles=qos))

    # static TF: base_link <- lidar_top
    tf_msg = TFMessage()
    tr = TransformStamped()
    tr.header.frame_id = 'base_link'
    tr.header.stamp = us2time(lidar_sweeps[0]['timestamp'])
    tr.child_frame_id = 'lidar_top'
    tr.transform.translation.x = float(cal_T[0])
    tr.transform.translation.y = float(cal_T[1])
    tr.transform.translation.z = float(cal_T[2])
    tr.transform.rotation.w = float(cal_R[0])
    tr.transform.rotation.x = float(cal_R[1])
    tr.transform.rotation.y = float(cal_R[2])
    tr.transform.rotation.z = float(cal_R[3])
    tf_msg.transforms.append(tr)
    writer.write('/tf_static', serialize_message(tf_msg), int(lidar_sweeps[0]['timestamp'] * 1000))

    point_fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    ]

    def find_closest(t_us, max_dt=0.1):
        t = t_us / 1e6
        i = int(np.argmin(np.abs(pose_uts - t)))
        if abs(pose_uts[i] - t) > max_dt:
            return -1
        return i

    # synthesize NavSatFix from local metric pos around a datum so that the
    # standard lat/lon -> UTM -> GPS-factor path is exercised end-to-end
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * np.cos(np.radians(args.datum_lat))

    gt_rows = []
    n_imu = n_gps = n_odom = n_points = 0

    t_start_us = lidar_sweeps[0]['timestamp']
    t_end_us = lidar_sweeps[-1]['timestamp']

    # imu @ pose channel rate
    for i, t in enumerate(pose_uts):
        t_us = t * 1e6
        if t_us < t_start_us - 1e6 or t_us > t_end_us + 1e6:
            continue
        msg = Imu()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = us2time(t_us)
        msg.angular_velocity.x = float(pose_rot_rate[i, 0])
        msg.angular_velocity.y = float(pose_rot_rate[i, 1])
        msg.angular_velocity.z = float(pose_rot_rate[i, 2])
        msg.linear_acceleration.x = float(pose_accel[i, 0])
        msg.linear_acceleration.y = float(pose_accel[i, 1])
        msg.linear_acceleration.z = float(pose_accel[i, 2])
        q = pose_orientation[i]
        msg.orientation.w = float(q[0])
        msg.orientation.x = float(q[1])
        msg.orientation.y = float(q[2])
        msg.orientation.z = float(q[3])
        writer.write('/imu/data', serialize_message(msg), int(t_us * 1000))
        n_imu += 1

    # gps + wheel odom @ 10Hz
    step = max(1, int(round((1.0 / (pose_uts[1] - pose_uts[0])) / 10.0)))
    for i in range(0, len(pose_uts), step):
        t_us = pose_uts[i] * 1e6
        if t_us < t_start_us - 1e6 or t_us > t_end_us + 1e6:
            continue

        fix = NavSatFix()
        fix.header.frame_id = 'gps'
        fix.header.stamp = us2time(t_us)
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(args.datum_lat + pose_pos[i, 1] / meters_per_deg_lat)
        fix.longitude = float(args.datum_lon + pose_pos[i, 0] / meters_per_deg_lon)
        fix.altitude = float(pose_pos[i, 2])
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        writer.write('/gps/fix', serialize_message(fix), int(t_us * 1000))
        n_gps += 1

        od = Odometry()
        od.header.frame_id = 'odom_wheel'
        od.header.stamp = us2time(t_us)
        od.child_frame_id = 'base_link'
        q = pose_orientation[i]
        od.pose.pose.orientation.w = float(q[0])
        od.pose.pose.orientation.x = float(q[1])
        od.pose.pose.orientation.y = float(q[2])
        od.pose.pose.orientation.z = float(q[3])
        od.twist.twist.linear.x = float(np.linalg.norm(pose_vel[i, :2]))
        od.twist.twist.angular.z = float(pose_rot_rate[i, 2])
        writer.write('/odom_wheel', serialize_message(od), int(t_us * 1000))
        n_odom += 1

    # lidar sweeps + ground truth
    for sd in lidar_sweeps:
        t_us = sd['timestamp']
        bin_path = os.path.join(root, sd['filename'])
        pc = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 5)  # x y z intensity ring
        buf = bytearray()
        for x, y, z, inten, ring in pc:
            fx = float(x) if np.isfinite(x) else 0.0
            fy = float(y) if np.isfinite(y) else 0.0
            fz = float(z) if np.isfinite(z) else 0.0
            fi = float(inten) if np.isfinite(inten) else 0.0
            ri = int(ring) if np.isfinite(ring) else 0
            ri = 0 if ri < 0 else (31 if ri > 31 else ri)
            buf += struct.pack('<ffffH2x', fx, fy, fz, fi, ri)
        msg = PointCloud2()
        msg.header.frame_id = 'lidar_top'
        msg.header.stamp = us2time(t_us)
        msg.height = 1
        msg.width = len(pc)
        msg.fields = point_fields
        msg.is_bigendian = False
        msg.point_step = 20
        msg.row_step = 20 * len(pc)
        msg.is_dense = True
        msg.data = bytes(buf)
        writer.write('/points_raw', serialize_message(msg), int(t_us * 1000))
        n_points += 1

        ep = ep_by_token[sd['ego_pose_token']]
        gt_rows.append((t_us, *ep['translation'], *ep['rotation']))

    gt_path = args.out.rstrip('/') + '_gt.csv'
    with open(gt_path, 'w') as f:
        f.write('t_us,x,y,z,qw,qx,qy,qz\n')
        for row in gt_rows:
            f.write('%d,%f,%f,%f,%f,%f,%f,%f\n' % row)

    print(f'\nbag written: {args.out}')
    print(f'  /points_raw  : {n_points} msgs')
    print(f'  /imu/data    : {n_imu} msgs')
    print(f'  /gps/fix     : {n_gps} msgs')
    print(f'  /odom_wheel  : {n_odom} msgs')
    print(f'  gt csv       : {gt_path}')


if __name__ == '__main__':
    main()
