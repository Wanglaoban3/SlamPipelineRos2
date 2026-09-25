#!/usr/bin/env bash
# Stage 1: mapping with hdl_graph_slam
# usage: run_mapping.sh <bag_dir>
set -e
BAG=$1
[ -z "$BAG" ] && { echo "usage: $0 <bag_dir>"; exit 1; }

REC=/root/data/rec_mapping
rm -rf $REC

source /root/ws/install/setup.bash

echo ">>> starting mapping nodes"
ros2 launch ${LAUNCH_FILE:-/mnt/h/projects/slam-pipeline-ros2/launch/mapping.launch.py} &
LAUNCH_PID=$!
sleep 5

echo ">>> recording ${ODOM_TOPIC:-/odom} /odom_gt"
ros2 bag record -o $REC ${ODOM_TOPIC:-/odom} /odom_gt &
REC_PID=$!
sleep 1

echo ">>> playing bag: $BAG"
ros2 bag play $BAG --clock
echo ">>> bag done, waiting for graph optimization"
sleep 15

echo ">>> saving map via service"
source /root/ws/install/setup.bash
ros2 service call /hdl_graph_slam/save_map hdl_graph_slam_ros2/srv/SaveMap "{utm: false, resolution: 0.1, destination: '/root/data/globalmap.pcd'}" || true
sleep 2

echo ">>> stopping"
# stop the recorder FIRST with SIGINT so it finalizes metadata.yaml, then hard-kill the rest
kill -INT $REC_PID 2>/dev/null || true
sleep 3
kill -9 $LAUNCH_PID $REC_PID 2>/dev/null || true
pkill -9 -x hdl_graph_slam_node 2>/dev/null || true
pkill -9 -x prefiltering_node 2>/dev/null || true
pkill -9 -x floor_detection_node 2>/dev/null || true
pkill -9 -x scan_matching_odometry_node 2>/dev/null || true
wait 2>/dev/null || true
echo ">>> mapping stage done"
ls -la /root/data/globalmap.pcd || echo "WARNING: globalmap.pcd missing"
