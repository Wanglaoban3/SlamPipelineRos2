#!/usr/bin/env bash
# stage 3 wrapper: cleanup + relocalization run + summary
bash /root/cleanup.sh
bash /root/run_localization.sh /root/data/nuscenes_demo > /root/data/reloc_run.log 2>&1
source /opt/ros/humble/setup.bash
echo "===== recorded bag ====="
ros2 bag info /root/data/rec_reloc 2>/dev/null | head -12
echo "===== died processes ====="
grep -c "process has died" /root/data/reloc_run.log
echo "===== reloc log tail ====="
grep -vE "Failed to find match" /root/data/reloc_run.log | tail -10
echo "===== stage3 done ====="
