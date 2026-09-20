#!/usr/bin/env bash
# 构建ROS2工作区: ndt_omp_ros2, fast_gicp, livox_ros_driver2, fast_lio, hdl_graph_slam_ros2, hdl_localization_ros2
set -e
log() { echo -e "\n===== [$(date +%H:%M:%S)] $1 =====\n"; }

SRC=/mnt/h/projects/slam-pipeline-ros2/ros2_ws/src
WS=/root/ws

log "sync sources into ext4"
mkdir -p $WS
rsync -a --delete --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' $SRC/ $WS/src/

source /opt/ros/humble/setup.bash

log "colcon build"
cd $WS
colcon build --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  > /root/ws/colcon_build.log 2>&1 || { echo COLCON_FAIL; tail -60 /root/ws/colcon_build.log; exit 1; }
tail -25 /root/ws/colcon_build.log

log "verify packages"
source $WS/install/setup.bash
for pkg in ndt_omp_ros2 fast_gicp livox_ros_driver2 fast_lio hdl_graph_slam_ros2 hdl_localization_ros2; do
  ros2 pkg prefix $pkg >/dev/null 2>&1 && echo "PKG_OK $pkg" || echo "PKG_FAIL $pkg"
done
