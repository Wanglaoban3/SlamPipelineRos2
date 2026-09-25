import json, os
import numpy as np

v = '/mnt/h/datasets/nuscenes-mini/v1.0-mini/'
root = '/mnt/h/datasets/nuscenes-mini'
sd = json.load(open(v + 'sample_data.json'))
ep = {e['token']: e for e in json.load(open(v + 'ego_pose.json'))}
cs = {c['token']: c for c in json.load(open(v + 'calibrated_sensor.json'))}
sensor = {s['token']: s for s in json.load(open(v + 'sensor.json'))}
scene_rows = json.load(open(v + 'scene.json'))
sample = {s['token']: s for s in json.load(open(v + 'sample.json'))}

# scene-1077 sample token set (follow the linked list)
scene = next(s for s in scene_rows if s['name'] == 'scene-1077')
tokens = set()
tok = scene['first_sample_token']
while tok:
    tokens.add(tok)
    tok = sample[tok]['next']
print('scene-1077 samples:', len(tokens))

lidar_cs = {c['token'] for c in cs.values() if sensor[c['sensor_token']]['channel'] == 'LIDAR_TOP'}
rows = sorted((r for r in sd if r['calibrated_sensor_token'] in lidar_cs
               and r['sample_token'] in tokens
               and os.path.isfile(os.path.join(root, r['filename']))), key=lambda r: r['timestamp'])
print('lidar sweeps:', len(rows))
gt = np.array([ep[r['ego_pose_token']]['translation'][:2] for r in rows])
gt_t = np.array([r['timestamp'] / 1e6 for r in rows])

can = json.load(open('/mnt/h/datasets/can_bus_extract/can_bus/scene-1077_pose.json'))
cb_t = np.array([r['utime'] for r in can]) / 1e6
cb_p = np.array([r['pos'] for r in can])[:, :2]

# nearest-time pairing can_bus -> GT sweeps
idx = np.abs(gt_t[:, None] - cb_t[None, :]).argmin(axis=1)
dt = np.abs(gt_t - cb_t[idx])
cb_pair = cb_p[idx]
print('pairing dt: max %.1f ms' % (dt.max() * 1e3))


def umeyama2d(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    sc, dc = src - ms, dst - md
    cov = dc.T @ sc / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[1, 1] = -1
    R = U @ S @ Vt
    var = (sc ** 2).sum() / len(src)
    scale = np.trace(np.diag(D) @ S) / var
    t = md - scale * R @ ms
    return R, t, scale


R, t, s = umeyama2d(cb_pair, gt)
theta = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
aligned = (s * R @ cb_pair.T).T + t
err = np.linalg.norm(aligned - gt, axis=1)

gt_len = np.linalg.norm(np.diff(gt, axis=0), axis=1).sum()
cb_len = np.linalg.norm(np.diff(cb_pair, axis=0), axis=1).sum()
print(f'GT path: {gt_len:.1f} m   can_bus path: {cb_len:.1f} m   scale fit: {s:.4f}')
print(f'yaw offset can_bus->GT: {theta:.2f} deg')
print(f'position error after alignment: mean {err.mean():.2f} m, max {err.max():.2f} m')
