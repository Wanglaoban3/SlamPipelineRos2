# slam-pipeline-ros2 — 多源融合定位与建图全链路（ROS2 Humble）

在 WSL2 (Ubuntu 22.04) + ROS2 Humble 中跑通的一套工业参考架构（数据：nuScenes mini + can_bus），三个阶段全部可复现，评估报告见 `results/report.md`。

```
                     ┌────────────────────────────────────────────────────┐
                     │                    rosbag2 (输入)                   │
                     │  /points_raw  /imu/data  /gps/fix  /odom_wheel     │
                     └───────┬──────────────┬──────────────┬──────────────┘
                             │              │              │
        ┌────────────────────▼───┐          │              │
        │  FAST-LIO2 (fast_lio)  │          │              │
        │  IMU紧耦合雷达里程计     │          │              │
        │  → /Odometry           │          │              │
        └───┬────────────┬───────┘          │              │
            │            │                  │              │
  ┌─────────▼─────────┐  │       ┌──────────▼──────────────▼──────┐
  │ robot_localization│  │       │  hdl_graph_slam_ros2 (移植)     │
  │  ekf_node (ESKF)  │  │       │  预滤波→地板检测→位姿图(外部odom)  │
  │  navsat_transform │  └──────▶│  +GPS先验+地板边+回环检测(g2o)    │
  │  → /odometry/     │          │  → globalmap.pcd (建图)         │
  │     filtered/gps  │          └──────────────┬─────────────────┘
  └───────────────────┘                         │ prior map
                                       ┌────────▼─────────────────┐
                                       │ hdl_localization_ros2    │
                                       │ NDT先验地图重定位(UKF预测)  │
                                       │ → /hdl_localization/odom │
                                       └──────────────────────────┘
```

> 注：hdl_graph_slam 自带的 scan_matching_odometry 节点（已一并移植，在本工作区可独立运行）被 FAST-LIO2 作为里程计来源替代——这也是更贴近量产的架构：滤波前端供高频里程计，图优化后端只负责建图与回环。

## 目录结构

- `upstream/` — 上游开源仓库原样克隆（hdl_graph_slam、hdl_localization、ndt_omp、fast_gicp、FAST_LIO(Ericsii ROS2版)、livox_ros_driver2、gtsam、g2o、Livox-SDK2）。**不入库**，用 `scripts/00_clone_upstream.sh` 重新获取；`ros2_ws/src` 内的 fast_gicp / fast_lio / livox_ros_driver2 是可直接编译的拷贝（fast_lio 的 doc/ 演示文件已移除）
- `ros2_ws/src/` — ROS2 工作区（6 个包全部编译通过）：
  - `ndt_omp_ros2` — koide3/ndt_omp 的 ROS2 包化（库核心零 ROS 依赖，原样保留）
  - `fast_gicp` / `fast_lio` / `livox_ros_driver2` — 上游原样（fast_gicp 原生支持 ROS2；FAST_LIO 用社区 ROS2 版，ikd-Tree 子模块已内置）
  - `hdl_graph_slam_ros2` — **ROS1 → ROS2 移植**：prefiltering / floor_detection / scan_matching_odometry / hdl_graph_slam 四节点 + FloorCoeffs/ScanMatchingStatus 消息 + SaveMap/LoadGraph/DumpGraph 服务；g2o 自定义边原样保留；UTM 用 GeographicLib；收尾可用服务保存地图
  - `hdl_localization_ros2` — **ROS1 → ROS2 移植**：UKF 预测 + NDT_OMP 逐帧配准；地图由参数直接加载
- `scripts/` — 环境安装(01)、依赖构建(02)、工作区构建(03)、语法预检(syntax_check)、数据转换、三阶段运行与评估
- `config/` — graph_slam.yaml、ekf.yaml、localization.yaml
- `launch/` — mapping.launch.py / odometry_fusion.launch.py / localization.launch.py
- `results/` — 评估报告、轨迹图、地图渲染、globalmap.pcd、运行日志

## 环境（全部在 H 盘）

- WSL 发行版 `slam-ros2`（Ubuntu 22.04，WSL2）：`H:\WSL\slam-ros2`
- ROS2 Humble（ros-base + pcl_ros + robot_localization 等，清华镜像直连安装）
- 源码依赖装到 /usr/local：gtsam 4.2.0、g2o(含 cholmod 求解器，配套 `scripts/CHOLMODConfig.cmake` 适配 jammy)、Livox-SDK2
- 数据：`H:\datasets\nuscenes-mini`（LiDAR 2Hz 关键帧）+ `H:\datasets\can_bus`（json 版，`H:\datasets\can_bus_extract\can_bus`）

## 快速复现

```bash
wsl -d slam-ros2

# 0) 生成 bag（nuScenes + can_bus → /points_raw /imu/data /gps/fix /odom_wheel /tf_static）
python3 /mnt/h/projects/slam-pipeline-ros2/scripts/convert_nuscenes_to_bag.py \
    --nuscenes /mnt/h/datasets/nuscenes-mini --canbus /mnt/h/datasets/can_bus_extract/can_bus \
    --out /root/data/nuscenes_demo
python3 /mnt/h/projects/slam-pipeline-ros2/scripts/gen_fastlio_config.py \
    --nuscenes /mnt/h/datasets/nuscenes-mini --out /root/data/fastlio_nuscenes.yaml

# 1) 建图（FAST-LIO2 前端 + hdl_graph_slam 后端，服务保存 globalmap.pcd）
bash /mnt/h/projects/slam-pipeline-ros2/scripts/run_mapping.sh /root/data/nuscenes_demo

# 2) 里程计 + EKF/GNSS 融合
bash /mnt/h/projects/slam-pipeline-ros2/scripts/run_odometry_fusion.sh /root/data/nuscenes_demo /root/data/fastlio_nuscenes.yaml

# 3) 重定位（NDT + UKF）
bash /mnt/h/projects/slam-pipeline-ros2/scripts/run_localization.sh /root/data/nuscenes_demo

# 4) 评估出图
python3 /mnt/h/projects/slam-pipeline-ros2/scripts/eval_plot.py --gt /root/data/nuscenes_demo_gt.csv \
    --bag /root/data/rec_mapping=/Odometry --bag /root/data/rec_fusion=/odometry/filtered \
    --bag /root/data/rec_fusion=/odometry/gps --bag /root/data/rec_reloc=/hdl_localization/odom \
    --out /mnt/h/projects/slam-pipeline-ros2/results/trajectories.png
python3 /mnt/h/projects/slam-pipeline-ros2/scripts/render_map.py \
    --pcd /root/data/globalmap.pcd --out /mnt/h/projects/slam-pipeline-ros2/results/globalmap.png
```

## 移植说明（ROS1 → ROS2 的具体改动）

| ROS1 | ROS2 | 说明 |
|---|---|---|
| nodelet::Nodelet | 独立 rclcpp::Node 可执行文件 | 4+1 个节点各自独立进程，launch 编排 |
| ros::NodeHandle::param<T> | param_or / ParamNodeHandle（裸指针垫片） | 缺省参数自动 declare，YAML 覆盖直接生效 |
| ros::Time / WallTimer | rclcpp::Time / wall timer | KeyFrame 哈希键改用 stamp.nanoseconds()；注意 pcl 头时间戳是 µs |
| tf (tf1) | tf2_ros | base_link 变换改用 tf2+Eigen |
| message_filters (ROS1) | message_filters (ROS2 humble) | ApproximateTime 同步 odom+cloud 接口一致 |
| geodesy::UTMPoint | GeographicLib::UTMUPS | GPS→UTM（jammy 无 CMake config，CMake 里直接 find_path/find_library） |
| nmea_msgs/Sentence 输入 | 移除 | 统一用 sensor_msgs/NavSatFix |
| std_msgs/Header read_until 节流话题 | 移除 | rosbag2 无该节流机制 |
| rviz 交互式 save-map 按钮 | SaveMap/LoadGraph/DumpGraph 服务 | headless 友好；LoadGraph/DumpGraph 保留图重载能力 |
| boost::filesystem | std::filesystem | |
| hdl_global_localization 服务 | 未移植 | 重定位用 specify_init_pose 参数 + /initialpose 话题 |
| g2o 求解器名 "lm_var" | "lm_var_cholmod" | 新版 g2o 的注册名变化 |
| 消息内裸 Header | std_msgs/Header + rosidl DEPENDENCIES | ROS2 rosidl 要求显式依赖声明 |

踩过的其他坑（供后人）：rosbag2 的 offered_qos_profiles 只认数字枚举；/tf_static 必须 transient_local 才能被 TF 监听器收到；`can't compare times with different time sources` 需统一 RCL_ROS_TIME；rclcpp YAML 序列不允许 int/double 混排；后台脚本里 pkill -f 会匹配到自身命令行。

## 数据说明与局限

- nuScenes mini 只有 **2Hz LiDAR 关键帧**（本场景 41 帧/20s，帧间位移约 6.3m），远低于真实车端 10-20Hz，所有模块都在超负荷工况下工作；本项目验证的是**链路架构与算法流程**，不代表真实精度水平。
- IMU/GNSS 来自 can_bus 的 RT3000 组合导航（本身就是融合结果）：50Hz 加速度/角速度/偏航四元数（与 ego_pose 真值差 1-2.5°，已验证一致）+ 10Hz 位置。原始 can_bus 打包无经纬度，NavSatFix 由米制 pos 围绕波士顿 datum 反投影合成，走真实的 lat/lon→UTM→GPS 因子管线。
- 轮速里程计由 RT3000 速度构成，非真实轮轴编码器。

## 已知问题

- `navsat_transform` 的 `/odometry/gps` 输出恒为零位姿（datum/静态 TF 均已配置且可查），未再深挖；GNSS 的实际融合效果由建图链 GPS 因子体现（地图被真实经纬度约束）。
- 重定位在 6m/帧跳变下后半程有横向抖动（ATE 18.2m，见 `results/report.md` 的解读）。
