#!/usr/bin/env bash
source /root/ws/install/setup.bash
ros2 launch /mnt/h/projects/slam-pipeline-ros2/launch/mapping.launch.py > /root/data/diag_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 6
ros2 bag play /root/data/nuscenes_demo --clock > /dev/null 2>&1 &
PLAYER=$!
sleep 2
echo "=== /points_raw ==="
timeout 8 ros2 topic echo /points_raw --field header.stamp --once 2>/dev/null
echo "=== /filtered_points ==="
timeout 8 ros2 topic echo /filtered_points --field header.stamp --once 2>/dev/null
echo "=== /Odometry ==="
timeout 8 ros2 topic echo /Odometry --field header.stamp --once 2>/dev/null
echo "=== /Odometry frame ==="
timeout 8 ros2 topic echo /Odometry --field child_frame_id --once 2>/dev/null
echo "=== sync callback lines ==="
grep -c 'sync callback' /root/data/diag_launch.log 2>/dev/null
grep -m2 'sync callback' /root/data/diag_launch.log 2>/dev/null
kill $PLAYER $LAUNCH_PID 2>/dev/null
sleep 3
kill -9 $PLAYER $LAUNCH_PID 2>/dev/null
echo STAMP_CHECK_DONE
