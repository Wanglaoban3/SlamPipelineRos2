#!/usr/bin/env bash
# rerun stages 2+3 only (map already exists)
LOGDIR=/root/data
bash /root/cleanup.sh
pkill -9 -x hdl_graph_slam_node 2>/dev/null
pkill -9 -x prefiltering_node 2>/dev/null
pkill -9 -x floor_detection_node 2>/dev/null
pkill -9 -x fastlio_mapping 2>/dev/null
pkill -f "mapping.launch" 2>/dev/null
sleep 2
ls -la $LOGDIR/globalmap.pcd

echo "########## STAGE 2: odometry + fusion ##########"
bash /root/run_fusion.sh $LOGDIR/nuscenes_demo $LOGDIR/fastlio_nuscenes.yaml > $LOGDIR/fusion_run.log 2>&1
source /opt/ros/humble/setup.bash
ros2 bag info $LOGDIR/rec_fusion 2>/dev/null | head -12

echo "########## STAGE 3: relocalization ##########"
bash /root/run_localization.sh $LOGDIR/nuscenes_demo > $LOGDIR/reloc_run.log 2>&1
ros2 bag info $LOGDIR/rec_reloc 2>/dev/null | head -12
echo "########## DONE ##########"
