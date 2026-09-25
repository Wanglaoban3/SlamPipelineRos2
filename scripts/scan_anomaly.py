import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2 as PC2

bag = sys.argv[1]
topic = sys.argv[2] if len(sys.argv) > 2 else '/cloud_undistorted'
r = SequentialReader()
r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
DT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
idx = 0
while r.has_next():
    t, data, _ = r.read_next()
    if t != topic:
        continue
    m = deserialize_message(data, PC2)
    arr = np.frombuffer(bytes(m.data), dtype=DT)
    bad_nan = (~np.isfinite(arr['x']) | ~np.isfinite(arr['y']) | ~np.isfinite(arr['z'])).sum()
    big = (np.abs(arr['x']) > 500) | (np.abs(arr['y']) > 500) | (np.abs(arr['z']) > 200)
    nan_i = ~np.isfinite(arr['i'])
    if bad_nan or big.any() or nan_i.any() or len(arr) == 0:
        print('msg %5d: n=%6d nan=%d big=%d nan_i=%d  zmax=%.3g' % (
            idx, len(arr), bad_nan, big.sum(), nan_i.sum(),
            np.nanmax(np.abs(arr['z'])) if len(arr) else 0))
        if big.any() or bad_nan:
            j = np.where(big | (~np.isfinite(arr['x'])))[0][0]
            print('   example:', arr[j])
    idx += 1
print('scanned', idx, 'messages')
