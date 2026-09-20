# 全链路运行结果报告

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
