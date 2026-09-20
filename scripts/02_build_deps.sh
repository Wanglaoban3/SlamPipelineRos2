#!/usr/bin/env bash
# 构建第三方依赖: g2o + gtsam (安装到 /usr/local)
# 源码先复制到 WSL ext4 内以获得编译速度
set -e
log() { echo -e "\n===== [$(date +%H:%M:%S)] $1 =====\n"; }

UPSTREAM=/mnt/h/projects/slam-pipeline-ros2/upstream
DEPS=/root/deps
NPROC=$(nproc)

mkdir -p $DEPS

log "copy sources into ext4"
rm -rf $DEPS/gtsam $DEPS/g2o
cp -r $UPSTREAM/gtsam $DEPS/gtsam
cp -r $UPSTREAM/g2o $DEPS/g2o

log "build gtsam 4.2.0"
cmake -S $DEPS/gtsam -B $DEPS/gtsam/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_STATIC_LIB=OFF
cmake --build $DEPS/gtsam/build -j$NPROC
cmake --install $DEPS/gtsam/build
ldconfig

log "build g2o"
cmake -S $DEPS/g2o -B $DEPS/g2o/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DG2O_BUILD_APPS=OFF \
  -DG2O_BUILD_EXAMPLES=OFF \
  -DG2O_USE_OPENGL=OFF
cmake --build $DEPS/g2o/build -j$NPROC
cmake --install $DEPS/g2o/build
ldconfig

log "build Livox-SDK2 (required by livox_ros_driver2)"
cp -r $UPSTREAM/Livox-SDK2 $DEPS/Livox-SDK2
cmake -S $DEPS/Livox-SDK2 -B $DEPS/Livox-SDK2/build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build $DEPS/Livox-SDK2/build -j$NPROC
cmake --install $DEPS/Livox-SDK2/build
ldconfig

log "verify"
ls /usr/local/lib | grep -E "libg2o_core|libgtsam" | head -5
echo DEPS_BUILD_OK
