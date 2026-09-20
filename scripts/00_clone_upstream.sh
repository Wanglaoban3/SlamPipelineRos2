#!/usr/bin/env bash
# Re-fetch all third-party upstream repos into upstream/ (not stored in this repo).
# Network-restricted environments: export HTTP_PROXY/HTTPS_PROXY first, e.g.
#   export HTTP_PROXY=http://127.0.0.1:10808 HTTPS_PROXY=http://127.0.0.1:10808
set -e
UP=$(dirname "$0")/../upstream
mkdir -p "$UP"
clone() {
  local url=$1 dir=$2 branch=${3:-}
  [ -d "$UP/$dir/.git" ] && { echo "skip $dir (exists)"; return; }
  if [ -n "$branch" ]; then
    git clone --depth 1 --branch "$branch" "$url" "$UP/$dir"
  else
    git clone --depth 1 "$url" "$UP/$dir"
  fi
}

clone https://github.com/koide3/hdl_graph_slam.git hdl_graph_slam
clone https://github.com/koide3/hdl_localization.git hdl_localization
clone https://github.com/koide3/ndt_omp.git ndt_omp
clone https://github.com/koide3/fast_gicp.git fast_gicp
clone https://github.com/Ericsii/FAST_LIO.git FAST_LIO
clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ros_driver2
clone https://github.com/Livox-SDK/Livox-SDK2.git Livox-SDK2
clone https://github.com/borglab/gtsam.git gtsam 4.2.0
clone https://github.com/RainerKuemmerle/g2o.git g2o

# FAST_LIO needs its ikd-Tree submodule
git -C "$UP/FAST_LIO" submodule update --init --depth 1

echo "upstream ready"
