#!/usr/bin/env bash
source /root/ws/install/setup.bash
ros2 launch /mnt/h/projects/slam-pipeline-ros2/launch/odometry_fusion.launch.py > /root/data/tf_diag.log 2>&1 &
LP=$!
sleep 6
echo "=== tf2_echo base_link -> gps (10s) ==="
timeout 10 ros2 run tf2_ros tf2_echo base_link gps 2>&1 | head -12
echo "=== playing bag 5s for clock ==="
timeout 5 ros2 bag play /root/data/nuscenes_demo --clock > /dev/null 2>&1
echo "=== tf2_echo again with clock ==="
timeout 6 ros2 run tf2_ros tf2_echo base_link gps 2>&1 | head -8
kill -9 $LP 2>/dev/null
pkill -9 -x fastlio_mapping 2>/dev/null
pkill -9 -x ekf_node 2>/dev/null
pkill -9 -x navsat_transform_node 2>/dev/null
pkill -9 -x static_transform_publisher 2>/dev/null
echo TF_PROBE_DONE
