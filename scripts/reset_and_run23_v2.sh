#!/usr/bin/env bash
# hard reset + rerun stage2/stage3 (with up-to-date kill logic)
pkill -9 -f "run_fusion.sh" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "ros2 bag" 2>/dev/null
sleep 2
for name in fastlio_mapping ekf_node navsat_transform_node hdl_localization_node; do
  pkill -9 -x "$name" 2>/dev/null
done
sleep 1
echo RESET_DONE

# refresh script copies from H:
for f in run_mapping run_odometry_fusion run_localization; do
  tr -d '\r' < /mnt/h/projects/slam-pipeline-ros2/scripts/$f.sh > /root/$f.sh
done
cp /root/run_odometry_fusion.sh /root/run_fusion.sh
cp /root/run_localization.sh /root/run_localization.sh

source /opt/ros/humble/setup.bash
echo "########## STAGE 2 ##########"
bash /root/run_fusion.sh /root/data/nuscenes_demo /root/data/fastlio_nuscenes.yaml > /root/data/fusion_run.log 2>&1
ros2 bag info /root/data/rec_fusion 2>/dev/null | head -12
echo "########## STAGE 3 ##########"
bash /root/run_localization.sh /root/data/nuscenes_demo > /root/data/reloc_run.log 2>&1
ros2 bag info /root/data/rec_reloc 2>/dev/null | head -12
echo "########## PIPELINE_DONE ##########"
