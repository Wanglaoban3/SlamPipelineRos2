from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix

r = SequentialReader()
r.open(StorageOptions(uri='/root/data/nuscenes_demo', storage_id='sqlite3'), ConverterOptions('', ''))
lats, lons = [], []
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/gps/fix':
        m = deserialize_message(d, NavSatFix)
        lats.append(m.latitude)
        lons.append(m.longitude)
print('gps/fix count:', len(lats))
if lats:
    print('lat range: %.8f .. %.8f (span %.2f m)' % (min(lats), max(lats), (max(lats) - min(lats)) * 111320.0))
    print('lon range: %.8f .. %.8f (span %.2f m)' % (min(lons), max(lons), (max(lons) - min(lons)) * 111320.0 * 0.74))
    print('first:', lats[0], lons[0], ' last:', lats[-1], lons[-1])
