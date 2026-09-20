#!/usr/bin/env bash
# Stage 3: relocalization on the prior map with hdl_localization (NDT)
# usage: run_localization.sh <bag_dir>
set -e
BAG=$1
[ -z "$BAG" ] && { echo "usage: $0 <bag_dir>"; exit 1; }

REC=/root/data/rec_reloc
rm -rf $REC

source /root/ws/install/setup.bash

echo ">>> starting hdl_localization (loads /root/data/globalmap.pcd)"
ros2 launch /mnt/h/projects/slam-pipeline-ros2/launch/localization.launch.py &
LAUNCH_PID=$!
sleep 5

echo ">>> recording /hdl_localization/odom"
ros2 bag record -o $REC /hdl_localization/odom &
REC_PID=$!
sleep 1

echo ">>> replaying bag: $BAG"
ros2 bag play $BAG --clock
echo ">>> bag done"
sleep 5

echo ">>> bag done"
sleep 5

# stop the recorder FIRST with SIGINT so it finalizes metadata.yaml, then hard-kill the rest
kill -INT $REC_PID 2>/dev/null || true
sleep 3
kill -9 $LAUNCH_PID $REC_PID 2>/dev/null || true
pkill -9 -x hdl_localization_node 2>/dev/null || true
wait 2>/dev/null || true
echo ">>> localization stage done"
