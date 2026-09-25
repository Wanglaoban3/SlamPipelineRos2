import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2 as PC2

r = SequentialReader()
r.open(StorageOptions(uri='/root/data/61_ros2', storage_id='sqlite3'), ConverterOptions('', ''))
while r.has_next():
    topic, data, _ = r.read_next()
    if topic == '/points_raw':
        m = deserialize_message(data, PC2)
        arr = np.frombuffer(bytes(m.data), dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')]))
        az = np.degrees(np.arctan2(arr['y'][:20000], arr['x'][:20000]))
        el = np.degrees(np.arctan2(arr['z'][:20000], np.hypot(arr['x'][:20000], arr['y'][:20000])))
        print('first 20 azimuths:', np.round(az[:20], 1))
        print('first 20 elevations:', np.round(el[:20], 1))
        # if ring-major, elevation stays constant for ~1000 consecutive points then jumps
        print('elevation every 200 pts:', np.round(el[::200], 2))
        # if firing order, azimuth advances monotonically (with wraps)
        daz = np.diff(az[:2000])
        print('azimuth step sign changes in first 2000:', int(np.sum(np.abs(daz) > 180)))
        break
