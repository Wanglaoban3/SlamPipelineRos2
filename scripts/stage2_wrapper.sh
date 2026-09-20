#!/usr/bin/env bash
# stage 2 wrapper: cleanup + fusion run + bag summary (kept in a file so pkill patterns
# inside cleanup.sh can never match this process's own command line)
bash /root/cleanup.sh
bash /root/run_fusion.sh /root/data/nuscenes_demo /root/data/fastlio_nuscenes.yaml > /root/data/fusion_run.log 2>&1
source /opt/ros/humble/setup.bash
echo "===== recorded bag ====="
ros2 bag info /root/data/rec_fusion 2>/dev/null | head -14
echo "===== died processes ====="
grep -c "process has died" /root/data/fusion_run.log
echo "===== stage2 done ====="
