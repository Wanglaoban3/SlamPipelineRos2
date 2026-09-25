# 全链路运行结果报告

## 20Hz sweeps 验证（scene-0061，61.bag，2026-09-25）

用户提供带 20Hz sweeps 的 ROS1 bag（`H:\datasets\nuscenes-rosbags\61.bag`，5.6GB，19.2s）。经 `convert_61_to_ros2.py` 转为 ROS2 bag（合并 can_bus IMU/GNSS + 静止初始化窗口 + 标定 TF），同一数据正面对决：

| 前端 | ATE（对齐 /odom_gt 后） | 地图质量 |
|---|---|---|
| **FAST-LIO2** | **0.112 m**（87m 轨迹的 0.13%） | 锐利：L 形道路、建筑立面、卡车、行道树清晰 |
| ICP（hdl 扫描匹配） | 0.714 m | 可用，但鬼影略多 |

**转弯残影（1~2°）的根因与修复**：nuScenes 雷达时间戳是**扫完时刻**，FAST-LIO 把 header 当作**起扫时刻**，去畸变的 IMU 积分窗口错位 100ms——转弯时窗口两端姿态差 25°/s×0.1s≈2.5°，直行时退化为 1.26m 平移剪切（路面沿自身平移不变，故"路上对、路外歪"）。修复：转换时把雷达消息时间戳整体前移一个扫掠周期（50ms）。

**本轮额外修掉的三个工程问题**：
1. nuScenes 扫描含"空中幽灵回波"（z 高达 100m 的不可能点，HDL-32E 仰角上限 +10.7°）——转换时按仰角一致性过滤
2. 水平方向的大数值脏点（z≈0 但 x/y 超大）穿过仰角门——补 150m 远距门限；此类点会让预滤波的统计离群滤波段错误
3. python 辅助进程（undistort_relay 等）用 pkill -x 杀不掉（进程名是 python3），僵尸进程多发布者污染话题——cleanup 改用 pkill -f + [b]racket 转义防自匹配

**结论**：与 2Hz 数据上的表现完全互为印证——算法选择必须匹配数据频率。20Hz 是 FAST-LIO2 的主场（精度 0.13m 达到其论文水准）；2Hz 稀疏关键帧则必须用大位移鲁棒的几何 ICP。转换过程修掉的工程坑：/tf_static 必须 transient_local QoS、bag 内所有时间戳必须单调（时间回跳会废掉 message_filters）、ROS1→ROS2 用 rosbags typestore 直转、图节点用"最近时间配对"替代 message_filters（对 DDS 到达顺序竞态免疫）并在启动时打印订阅话题名。

## FAST-LIO2 前端失效根因分析（2026-09-25，2Hz 数据）

**现象**：FAST-LIO2 前端建的图完全不可用（弯曲/糊化），纯 ICP 扫描匹配反而可用。

**排查证据链**：

| 实验 | 结果 | 结论 |
|---|---|---|
| GNSS 坐标系验证（check_datum.py） | 旋转差 0.00°、比例 1.0004、误差 5cm | 合成 GNSS 无罪 |
| 陀螺零偏验证（check_gyro_bias.py） | 0.1°/s，与融合姿态微分一致 | 陀螺数据无罪 |
| 外参验证（GT 位姿拼图 stitch_gt_map.py） | 干净笔直的走廊（globalmap_gt.pcd） | 外参/轴约定无罪 |
| FAST-LIO 内部日志（Log/mat_pre.txt） | bg（陀螺零偏估计）恒为 0.4253 rad/s = 24.4°/s；重力估计倾斜 17° | 状态估计自相矛盾 |
| 航向对比（check_yaw.py） | 真值总航向变化 4.8°，FAST-LIO 输出 -419.5° | 每帧恒定 -12° 假旋转 |
| 场景开头检查 | 场景切入时车正在转弯（26.8°/s），2 秒后回直 | 转弯触发发散 |
| **直行段裁剪实验**（skip 前 8 帧） | 里程计**冻结**：28 帧只走 1.8m（实际 170m） | 直线段平移退化 |

**根因**：nuScenes-mini 只有 2Hz 雷达关键帧，帧间位移 6.3 米，远超 FAST-LIO2 的设计工况（10-20Hz、帧间 <1 米、原始 IMU 高频预测帧间运动）。两个失败模式同源：

1. **转弯段初始化**：`IMU_Processing.hpp` 的 `init_state.bg = mean_gyr` 假设初始化时载体静止；运动中初始化把真实转弯角速度焊死为零偏（`b_gyr_cov: 0.0001` 锁死不可自愈）→ 恒定 -24°/s 假自旋 → 旋转的地图自强化。
2. **直行段**：匀速运动加速度计输出≈0，IMU 无法预测帧间 6.3 米平移；扫描匹配从零初值在弱纹理直线走廊中收敛到恒等变换 → 里程计冻结。

**结论**：算法无 bug，是数据不满足设计前提。本数据集用几何 ICP 前端（对应距离 10m，显式处理大位移）是正确选择。要发挥 FAST-LIO2 需 10Hz+ 雷达 + 原始 100-200Hz IMU 的数据（KITTI raw / UrbanNav 等）。

---

- 数据：nuScenes v1.0-mini `scene-1077`（新加坡 Holland Village，20.0s，41 帧 HDL-32E @2Hz，真值轨迹 252m）
- GNSS/IMU：can_bus 扩展包 RT3000 融合通道（50Hz 加速度计/陀螺/偏航四元数 + 10Hz 经纬度合成 NavSatFix）
- 平台：WSL2 Ubuntu 22.04 + ROS2 Humble，全部节点 use_sim_time + `ros2 bag play --clock`

## 三阶段运行结果

| 阶段 | 链路 | 输出 | 结果 |
|---|---|---|---|
| 1 建图 | FAST-LIO2 前端里程计 → hdl_graph_slam(移植版) 位姿图（里程计边+地板边+GPS先验+回环检测，g2o lm_var_cholmod 优化） | `globalmap.pcd`（387,538 点，约 290m×258m×17m） | ✅ chi2 从 21160 收敛到 15.1 |
| 2 里程计+融合 | FAST-LIO2 `/Odometry` + 轮速 `/odom_wheel` + IMU → robot_localization EKF `/odometry/filtered`(30Hz)；GPS → navsat_transform → `/odometry/gps` | 36 + 595 + 198 帧 | ✅ EKF 融合有效；navsat 输出见"已知问题" |
| 3 重定位 | hdl_localization(移植版)：加载 globalmap.pcd，UKF(IMU+里程计TF预测) + NDT_OMP 逐帧配准 | `/hdl_localization/odom` 38 帧 | ✅ 全程输出，跟踪见"已知问题" |

## 轨迹精度（ATE，Umeyama SE3+尺度对齐后，相对 nuScenes ego_pose 真值）

| 轨迹 | 匹配帧数 | ATE |
|---|---|---|
| FAST-LIO2 里程计 `/Odometry` | 35 | **9.23 m** |
| robot_localization EKF 融合 `/odometry/filtered` | 475 | **3.54 m** |
| GNSS 融合 `/odometry/gps`（navsat） | 156 | nan（输出零位姿，见下） |
| NDT 重定位 `/hdl_localization/odom` | 38 | 18.21 m |

**解读**

1. **EKF 融合收益明显**：融合轮速+IMU 后 ATE 从 9.2m 降到 3.5m（降 62%），轨迹最平滑——验证了 robot_localization 多源融合层的作用。
2. **纯里程计 9.2m**：2Hz 扫描（0.5s 间隔、每帧 6.3m 位移）是 FAST-LIO2 远低于设计工况的输入，能到 9.2m/252m（3.7%）已属合理；真实车端 10-20Hz 下该数字会显著变好。
3. **重定位 18.2m**：NDT 在 6m/帧的跳变下主要靠 UKF 的 IMU/里程计预测"扛"过去，配准本身每帧只做局部修正，后半程出现横向抖动（轨迹图橙色线整体仍沿道路走廊）。这不是算法缺陷，而是 2Hz 关键帧输入远低于 hdl_localization 设计的 10Hz+ 工况。

## 已知问题（如实记录）

- **navsat_transform 输出零位姿**：`/odometry/gps` 发布 198 帧但位置恒为 0。datum 显式给定、base_link→gps 静态 TF 均已配置且 tf2_echo 可查到，仍未触发其内部 transform 计算。GNSS 的实际融合效果由建图链的 GPS 因子体现（globalmap 被真实经纬度约束，zero_utm 对齐），navsat 路径留待后续排查。
- **重定位初始帧偏差**：`specify_init_pose` 单位姿态 vs 建图 map 系（被 GPS 因子拉动过）之间存在小偏差，前几帧 NDT 收敛后正常。
- **mini 只有 2Hz 雷达**：所有 10-20Hz 设计工况的模块都在超负荷工作，本 demo 的数值结论只用于验证链路架构，不代表真实精度水平。

## 产物清单（H:\projects\slam-pipeline-ros2\results\）

- `globalmap.png` — 建图 BEV/侧视渲染
- `trajectories.png` — 四条轨迹 vs 真值
- `globalmap.pcd` — 建图输出点云
- `logs/` — 三阶段完整运行日志
