#!/usr/bin/env python3
"""Generate the fast_lio config for nuScenes HDL-32E using the real LiDAR->ego extrinsic."""
import argparse
import json
import os
import numpy as np


def quat_to_mat(w, x, y, z):
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    return R


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nuscenes', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    vdir = os.path.join(args.nuscenes, 'v1.0-mini')
    sd = json.load(open(os.path.join(vdir, 'sample_data.json')))
    calib = json.load(open(os.path.join(vdir, 'calibrated_sensor.json')))
    sensor = json.load(open(os.path.join(vdir, 'sensor.json')))
    calib_by_token = {c['token']: c for c in calib}
    sensor_by_token = {s['token']: s for s in sensor}

    lidar_sd = next(s for s in sd if sensor_by_token[calib_by_token[s['calibrated_sensor_token']]['sensor_token']]['channel'] == 'LIDAR_TOP')
    c = calib_by_token[lidar_sd['calibrated_sensor_token']]
    T = c['translation']
    w, x, y, z = c['rotation']
    R = quat_to_mat(w, x, y, z)
    flat = ', '.join(f'{v:.6f}' for v in R.flatten())

    yaml = f"""/**:
    ros__parameters:
        use_sim_time: true

        feature_extract_enable: false
        point_filter_num: 2
        max_iteration: 3
        filter_size_surf: 0.3
        filter_size_map: 0.3
        cube_side_length: 500.0
        runtime_pos_log_enable: false
        map_file_path: "/root/data/fastlio_map.pcd"

        common:
            lid_topic: "/points_raw"
            imu_topic: "/imu/data"
            time_sync_en: false
            time_offset_lidar_to_imu: 0.0

        preprocess:
            lidar_type: 2              # Velodyne HDL-32E
            scan_line: 32
            scan_rate: 20              # nuScenes HDL-32E rotates at 20Hz
            timestamp_unit: 2
            blind: 1.0

        mapping:
            acc_cov: 0.1
            gyr_cov: 0.1
            b_acc_cov: 0.0001
            b_gyr_cov: 0.0001
            fov_degree: 360.0
            det_range: 100.0
            extrinsic_est_en: false
            extrinsic_T: [ {T[0]:.6f}, {T[1]:.6f}, {T[2]:.6f} ]
            extrinsic_R: [ {flat} ]

        publish:
            path_en: true
            scan_publish_en: true
            dense_publish_en: true
            scan_bodyframe_pub_en: false

        pcd_save:
            pcd_save_en: false
            interval: -1
"""
    with open(args.out, 'w') as f:
        f.write(yaml)
    print(f'written: {args.out}')
    print(f'extrinsic_T = {T}')
    print(f'extrinsic_R = {R.tolist()}')


if __name__ == '__main__':
    main()
