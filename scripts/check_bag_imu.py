from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

r = SequentialReader()
r.open(StorageOptions(uri='/root/data/nuscenes_demo', storage_id='sqlite3'), ConverterOptions('', ''))
n = 0
shown = 0
first_lidar = None
while r.has_next():
    topic, data, ts = r.read_next()
    if topic == '/imu/data':
        m = deserialize_message(data, Imu)
        if shown < 3 or (76 <= shown <= 80):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            print('imu #%3d t=%.3f gyro_z=%+.4f acc_z=%.2f' % (
                shown, t, m.angular_velocity.z, m.linear_acceleration.z))
        shown += 1
    elif topic == '/points_raw' and first_lidar is None:
        from sensor_msgs.msg import PointCloud2
        m = deserialize_message(data, PointCloud2)
        first_lidar = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        print('first lidar t=%.3f' % first_lidar)
print('imu total:', shown)
