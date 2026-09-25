import json
import numpy as np

can = json.load(open('/mnt/h/datasets/can_bus_extract/can_bus/scene-1077_pose.json'))
t = np.array([r['utime'] for r in can]) / 1e6
gyro = np.array([r['rotation_rate'] for r in can])
q = np.array([r['orientation'] for r in can])

yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                           1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
yaw_rate = np.gradient(yaw, t)

print('gyro z: mean %.4f rad/s (%.1f deg/s), std %.4f' % (gyro[:, 2].mean(), np.degrees(gyro[:, 2].mean()), gyro[:, 2].std()))
print('yaw rate from fused orientation: mean %.4f rad/s, std %.4f' % (yaw_rate.mean(), yaw_rate.std()))
bias = gyro[:, 2] - yaw_rate
print('implied gyro z bias: median %.4f rad/s (%.1f deg/s)' % (np.median(bias), np.degrees(np.median(bias))))
print('gyro x: mean %.4f   gyro y: mean %.4f' % (gyro[:, 0].mean(), gyro[:, 1].mean()))
print('after z-debias, residual std: %.4f rad/s' % np.std(gyro[:, 2] - np.median(bias)))
