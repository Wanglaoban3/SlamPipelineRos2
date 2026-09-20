#!/usr/bin/env bash
# hard reset every pipeline-related process, then run stage2+stage3 once, sequentially
pkill -9 -f run_all.sh 2>/dev/null
pkill -9 -f run_mapping.sh 2>/dev/null
pkill -9 -f run_fusion.sh 2>/dev/null
pkill -9 -f run_localization.sh 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "ros2 bag" 2>/dev/null
sleep 2
for name in hdl_graph_slam_node prefiltering_node floor_detection_node scan_matching_odometry_node hdl_localization_node fastlio_mapping ekf_node navsat_transform_node; do
  pkill -9 -x "$name" 2>/dev/null
done
sleep 2
echo RESET_DONE
pgrep -a -x ekf_node
pgrep -a -x fastlio_mapping
echo "########## STAGE 2 ##########"
bash /root/run_fusion.sh /root/data/nuscenes_demo /root/data/fastlio_nuscenes.yaml > /root/data/fusion_run.log 2>&1
source /opt/ros/humble/setup.bash
ros2 bag info /root/data/rec_fusion 2>/dev/null | head -12
echo "########## STAGE 3 ##########"
bash /root/run_localization.sh /root/data/nuscenes_demo > /root/data/reloc_run.log 2>&1
ros2 bag info /root/data/rec_reloc 2>/dev/null | head -12
echo "########## PIPELINE_DONE ##########"
