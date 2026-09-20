#!/usr/bin/env bash
# WSL(Ubuntu 22.04) 初始化: 清华镜像源 + ROS2 Humble + 编译依赖
# 全部直连(不走代理), 依赖国内镜像加速
set -e

log() { echo -e "\n===== [$(date +%H:%M:%S)] $1 =====\n"; }

export DEBIAN_FRONTEND=noninteractive

log "backup and replace apt sources with TUNA mirror"
cp /etc/apt/sources.list /etc/apt/sources.list.bak 2>/dev/null || true
cat > /etc/apt/sources.list <<'EOF'
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-security main restricted universe multiverse
EOF

log "apt update"
apt-get update -y

log "install base toolchain"
apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release \
    build-essential cmake git pkg-config \
    libeigen3-dev libpcl-dev libboost-all-dev \
    libsuitesparse-dev libgoogle-glog-dev libgflags-dev \
    libgeographic-dev libomp-dev libyaml-cpp-dev \
    python3-pip python3-numpy python3-matplotlib \
    tzdata

log "check libg2o availability"
apt-cache policy libg2o-dev || true

log "add ROS2 apt repo (TUNA mirror)"
# key: ros/rosdistro 仓库中的 ros.key (属于仓库文件)
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  curl -fsSL -m 30 -o /tmp/ros.key https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    || curl -fsSL -m 30 -x http://$(ip route show default | awk '{print $3}'):10808 -o /tmp/ros.key https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
  gpg --dearmor < /tmp/ros.key > /usr/share/keyrings/ros-archive-keyring.gpg 2>/dev/null \
    || cp /tmp/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list
apt-get update -y

log "install ROS2 Humble core + pipeline deps (this is big)"
apt-get install -y --no-install-recommends \
    ros-humble-ros-base ros-dev-tools \
    ros-humble-pcl-ros \
    ros-humble-robot-localization \
    ros-humble-geographic-msgs \
    ros-humble-message-filters \
    ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
    ros-humble-rosbag2 ros-humble-rosbag2-py \
    ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs \
    ros-humble-visualization-msgs ros-humble-std-msgs \
    python3-colcon-common-extensions

log "done, source and verify"
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null 2>&1 && echo "ROS2_HUMBLE_OK: $(ros2 pkg prefix rclcpp)" || { echo "ROS2_VERIFY_FAIL"; exit 1; }
dpkg -l | grep -E "ros-humble-robot-localization|ros-humble-pcl-ros" | awk '{print $2, $3}'
