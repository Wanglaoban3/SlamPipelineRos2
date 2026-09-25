#!/usr/bin/env bash
# cleanup stray pipeline processes only (never touch /root/data artifacts!)
for name in hdl_graph_slam_node prefiltering_node floor_detection_node scan_matching_odometry_node hdl_localization_node fastlio_mapping ekf_node navsat_transform_node; do
  pkill -x "$name" 2>/dev/null
done
# python helper scripts run as 'python3' -> pkill -x can't match them; use -f with the
# [b]racket trick so this line can never match its own process
pkill -f "undistort_[r]elay" 2>/dev/null
pkill -f "odom_to_[t]f" 2>/dev/null
pkill -f "ros2 [l]aunch" 2>/dev/null
pkill -f "ros2 [b]ag" 2>/dev/null
sleep 1
echo CLEANED
