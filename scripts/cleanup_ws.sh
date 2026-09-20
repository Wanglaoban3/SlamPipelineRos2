#!/usr/bin/env bash
# cleanup stray pipeline processes only (never touch /root/data artifacts!)
for name in hdl_graph_slam_node prefiltering_node floor_detection_node scan_matching_odometry_node hdl_localization_node fastlio_mapping; do
  pkill -x "$name" 2>/dev/null
done
pkill -f "ros2 launch" 2>/dev/null
pkill -f "ros2 bag" 2>/dev/null
sleep 1
echo CLEANED
