#!/usr/bin/env bash
# Visualize mapping results in RViz2 (WSLg window pops up on the Windows desktop)
# usage: view_maps.sh [icp_pcd] [fastlio_pcd]
# NOTE: no `set -u` here - ros setup.bash references unbound variables
ICP_PCD=${1:-/mnt/h/projects/slam-pipeline-ros2/results/globalmap_icp.pcd}
FASTLIO_PCD=${2:-/mnt/h/projects/slam-pipeline-ros2/results/globalmap.pcd}
RVIZ_CFG=/mnt/h/projects/slam-pipeline-ros2/results/rviz_maps.rviz

source /opt/ros/humble/setup.bash
export DISPLAY=${DISPLAY:-:0}
# WSLg exposes the host GPU via D3D12 (verified: glxinfo -> "D3D12 (AMD Radeon...)").
# Do NOT set LIBGL_ALWAYS_SOFTWARE here - software rendering freezes rviz with large maps.

python3 /mnt/h/projects/slam-pipeline-ros2/scripts/pcd_publisher.py "$ICP_PCD" /globalmap_icp --frame map &
P1=$!
if [ -f "$FASTLIO_PCD" ]; then
  python3 /mnt/h/projects/slam-pipeline-ros2/scripts/pcd_publisher.py "$FASTLIO_PCD" /globalmap_fastlio --frame map &
  P2=$!
else
  P2=""
fi
sleep 3

rviz2 -d "$RVIZ_CFG"
RC=$?

kill $P1 ${P2:+$P2} 2>/dev/null
echo "RVIZ_CLOSED rc=$RC"
