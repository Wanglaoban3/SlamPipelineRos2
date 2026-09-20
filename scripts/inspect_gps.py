from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

r = SequentialReader()
r.open(StorageOptions(uri='/root/data/rec_fusion', storage_id='sqlite3'), ConverterOptions('', ''))
n = 0
while r.has_next():
    t, d, _ = r.read_next()
    if t == '/odometry/gps':
        m = deserialize_message(d, Odometry)
        p = m.pose.pose.position
        if n < 3 or n % 50 == 0 or n == 197:
            print('gps pose %d: %.2f %.2f %.2f' % (n, p.x, p.y, p.z))
        n += 1
print('total', n)
