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
ros2 launch /mnt/h/projects/slam-pipeline-ros2/launch/mapping.launch.py &
LAUNCH_PID=$!
sleep 5

echo ">>> recording /Odometry"
ros2 bag record -o $REC /Odometry &
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
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 5
# map was already saved via the service call above -> hard stop is safe
kill -9 $LAUNCH_PID $REC_PID 2>/dev/null || true
pkill -9 -x hdl_graph_slam_node 2>/dev/null || true
pkill -9 -x prefiltering_node 2>/dev/null || true
pkill -9 -x floor_detection_node 2>/dev/null || true
pkill -9 -x fastlio_mapping 2>/dev/null || true
wait 2>/dev/null || true
echo ">>> mapping stage done"
ls -la /root/data/globalmap.pcd || echo "WARNING: globalmap.pcd missing"
