#!/usr/bin/env bash
# Stage 2: FAST-LIO2 odometry + robot_localization EKF fusion + navsat GNSS fusion
# usage: run_odometry_fusion.sh <bag_dir> <fastlio_config>
set -e
BAG=$1
CFG=$2
[ -z "$BAG" ] && { echo "usage: $0 <bag_dir> <fastlio_config>"; exit 1; }

REC=/root/data/rec_fusion
rm -rf $REC

source /root/ws/install/setup.bash

echo ">>> starting fastlio + ekf + navsat"
FASTLIO_CONFIG=$CFG ros2 launch /mnt/h/projects/slam-pipeline-ros2/launch/odometry_fusion.launch.py &
LAUNCH_PID=$!
sleep 5

echo ">>> recording /Odometry /odometry/filtered /odometry/gps /odom"
ros2 bag record -o $REC /Odometry /odometry/filtered /odometry/gps &
REC_PID=$!
sleep 1

echo ">>> playing bag: $BAG"
ros2 bag play $BAG --clock
echo ">>> bag done"
sleep 5

echo ">>> bag done"
sleep 5

# stop the recorder FIRST with SIGINT so it finalizes metadata.yaml, then hard-kill the rest
kill -INT $REC_PID 2>/dev/null || true
sleep 3
kill -9 $LAUNCH_PID $REC_PID 2>/dev/null || true
pkill -9 -x fastlio_mapping 2>/dev/null || true
pkill -9 -x ekf_node 2>/dev/null || true
pkill -9 -x navsat_transform_node 2>/dev/null || true
wait 2>/dev/null || true
echo ">>> odometry/fusion stage done"
