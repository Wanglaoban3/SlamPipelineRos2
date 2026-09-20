#!/usr/bin/env bash
# full pipeline: mapping -> odometry+fusion -> relocalization
set -u
LOGDIR=/root/data

echo "########## STAGE 1: mapping ##########"
bash /root/cleanup.sh
bash /root/run_mapping.sh $LOGDIR/nuscenes_demo > $LOGDIR/mapping_run.log 2>&1
ls -la $LOGDIR/globalmap.pcd 2>/dev/null || echo "STAGE1_MAP_MISSING"

echo "########## STAGE 2: odometry + fusion ##########"
bash /root/cleanup.sh
bash /root/run_fusion.sh $LOGDIR/nuscenes_demo $LOGDIR/fastlio_nuscenes.yaml > $LOGDIR/fusion_run.log 2>&1
source /opt/ros/humble/setup.bash
ros2 bag info $LOGDIR/rec_fusion 2>/dev/null | head -12

echo "########## STAGE 3: relocalization ##########"
bash /root/cleanup.sh
bash /root/run_localization.sh $LOGDIR/nuscenes_demo > $LOGDIR/reloc_run.log 2>&1
ros2 bag info $LOGDIR/rec_reloc 2>/dev/null | head -12

echo "########## ALL STAGES DONE ##########"
echo "died processes: mapping=$(grep -c 'process has died' $LOGDIR/mapping_run.log) fusion=$(grep -c 'process has died' $LOGDIR/fusion_run.log) reloc=$(grep -c 'process has died' $LOGDIR/reloc_run.log)"
