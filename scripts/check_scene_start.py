import json
import numpy as np

can = json.load(open('/mnt/h/datasets/can_bus_extract/can_bus/scene-1077_pose.json'))
t = np.array([r['utime'] for r in can]) / 1e6
gyro = np.array([r['rotation_rate'] for r in can])
q = np.array([r['orientation'] for r in can])

yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                           1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
print('can_bus gyro z over the first 2 s (50Hz):')
print(np.round(np.degrees(gyro[:100, 2]), 1))
print('fused-attitude yaw rate first 2 s (deg/s):')
print(np.round(np.degrees(np.gradient(yaw, t)[:100]), 1))
print('gyro z mean over first 0.5s: %.3f rad/s = %.1f deg/s' % (gyro[:25, 2].mean(), np.degrees(gyro[:25, 2].mean())))
print('gyro z mean over 2-20s:      %.3f rad/s = %.1f deg/s' % (gyro[100:, 2].mean(), np.degrees(gyro[100:, 2].mean())))
