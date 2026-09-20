#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
ros2 bag play /root/data/nuscenes_demo --clock > /dev/null 2>&1 &
PLAYER=$!
sleep 2
echo "=== /points_raw stamps ==="
timeout 8 ros2 topic echo /points_raw sensor_msgs/msg/PointCloud2 --field header.stamp --once 2>/dev/null
echo "=== /filtered_points stamps ==="
timeout 8 ros2 topic echo /filtered_points --field header.stamp --once 2>/dev/null
echo "=== /Odometry stamps ==="
timeout 8 ros2 topic echo /Odometry --field header.stamp --once 2>/dev/null
kill $PLAYER 2>/dev/null
wait 2>/dev/null
echo STAMP_CHECK_DONE
