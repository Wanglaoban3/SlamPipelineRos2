#!/usr/bin/env bash
# diagnostic: launch mapping nodes, inspect connections, play bag, inspect again
source /root/ws/install/setup.bash
ros2 launch /mnt/h/projects/slam-pipeline-ros2/launch/mapping.launch.py > /root/data/diag_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 6
echo "===== node info (before) ====="
ros2 node info /hdl_graph_slam_node 2>/dev/null | head -40
echo "===== playing bag ====="
ros2 bag play /root/data/nuscenes_demo --clock > /root/data/diag_play.log 2>&1
sleep 3
echo "===== node info (after) ====="
ros2 node info /hdl_graph_slam_node 2>/dev/null | head -40
echo "===== graph slam log tail ====="
tail -15 /root/data/diag_launch.log
kill -INT $LAUNCH_PID 2>/dev/null
sleep 5
kill $LAUNCH_PID 2>/dev/null
echo DIAG_DONE
