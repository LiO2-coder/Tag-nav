# 联调故障排查

[English](troubleshooting_en.md)

本文记录 Gazebo 仿真、EKF、AprilTag 定位和 TF 联调中最容易混淆的行为。排查 TF 时，先确认“哪一个节点发布哪一条边”和“消息使用的时间戳/数据源”，再判断是不是 AprilTag 算法问题。

## TF 发布职责

在 `factory_apriltag_localization.launch`、`planner.launch` 和 `factory_navigation.launch` 中，发布职责应保持为：

```text
robot_localization:  /agv/odom + /agv/imu/data -> odom -> base_footprint
AprilTag:            T_map_base * inverse(T_odom_base) -> map -> odom
最终链路:             map -> odom -> base_footprint
```

`correction` 模式由 AprilTag 节点发布 `map -> odom`；`localization` 模式才直接发布 `map -> base_footprint`。不要让 Gazebo、EKF 和定位节点同时发布同一条动态 TF。`factory_gazebo.launch` 的 `publish_odom_tf` 默认值为 `true`，而带 EKF 的入口会将它设为 `false`。

早期出现过 `TF_REPEATED_DATA`，原因是同一 parent/child 在仿真时间下被重复广播相同时间戳。当前代码发布前会检查 `parent + child + stamp` 的严格递增关系；仿真时间回退时会清空检查状态。

## 丢失 Tag 时的定位状态

没有 Tag 时，轮式里程计和 EKF 仍应持续运行。`correction` 模式会以当前时间戳重发缓存的 `map -> odom`，保持 TF 链可查询，但视觉定位应标记为无效。

通过定位消息查看状态：

```bash
rostopic echo /apriltag_localization/localization
```

重点查看消息中的：

- `valid`：当前融合位姿是否有效；
- `tf_status`：TF 发布状态，例如 `published`、`correction_held` 或 `no_valid_pose`；
- `localization_age_sec`：最近一次有效视觉定位的年龄。

`valid: false` 表示视觉定位当前无效，不表示 `/agv/odom` 或 `/agv/odometry/filtered` 停止发布。

## 手动拖动 Gazebo 模型

差速插件使用 `<odometrySource>encoder</odometrySource>`，而不是把 Gazebo 世界真值直接写入 `/agv/odom`。手动瞬移模型仍可能让视觉观测产生较大修正，但 EKF 的 `odom -> base_footprint` 不应因为世界坐标瞬移而被直接重置。

EKF 配置还关闭了 IMU 的绝对世界航向；视觉节点通过新的标签观测更新 `map -> odom`。如果看到跳变，分别检查 `/agv/odom`、`/agv/odometry/filtered` 和 `/tf` 的时间戳与发布者。

## 图像时间戳与高速旋转

同步器的 `wait_sec` 是等待其他相机帧到达的延迟，不是从图像测量时间戳中减去的时间。处理时间戳为 `t` 的图像时，应在同一 `t` 查询 `T_odom_base(t)`。

高速原地旋转时仍可能出现小幅角度误差，常见原因是图像采集、传输和检测延迟；这不应被解释为同步器主动改变了测量时间。

## 下视鱼眼相机与地面 Tag

下视相机使用 `equidistant` 分支整流后，再进行 AprilTag 检测和 IPPE 方形 PnP。OpenCV `SOLVEPNP_IPPE_SQUARE` 对角点顺序有严格要求，地面地图将 Tag 法向定义为 `+Z`。

当前代码先按 OpenCV 要求解算并验证四个角点为正深度，再通过 `diag(1, -1, -1)` 将 PnP Tag 坐标转换到地图的地面 Tag 坐标约定。

![相机配置示意图](../assets/camera_setup.png)

## Gazebo GUI 警告

以下提示来自 Gazebo Classic GUI 的 `JointControlWidget`：

```text
Node::Advertise(): Error advertising topic [/warehouse_agv/joint_cmd]
```

这是 Gazebo Transport 内部话题，不是 ROS 话题。只要 `/agv/cmd_vel`、相机图像、`/agv/odom` 和 `/tf` 正常，该警告通常可以忽略。

不需要 Gazebo GUI 时可以关闭它：

```bash
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false
```

多网卡、VPN 或 Docker 环境下，只有在双方需要显式指定 Gazebo 网络地址时才设置相应环境变量：

```bash
export GAZEBO_IP=10.10.151.14
export IGN_IP=10.10.151.14
```

## 推荐诊断顺序

### 1. 确认输入是否持续

```bash
rostopic hz /agv/odom
rostopic hz /agv/odometry/filtered
rostopic hz /agv/camera/bottom/image_raw
```

### 2. 确认 TF 链和方向

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map odom
rosrun tf tf_echo map base_footprint
rosrun tf view_frames
```

### 3. 查看 AprilTag 状态

```bash
rostopic echo /apriltag_localization/localization
```

### 4. 确认没有多个动态 TF 发布者

```bash
rostopic info /tf
rosnode info /agv_ekf_odom
rosnode info /apriltag_localization
```

## 相关文档

- [主 README](../README.md)
- [定位配置](../src/tag_nav_localization/config/CONFIG.md)
- [标签地图配置](../src/tag_nav_bringup/worlds/maps/CONFIG.md)
- [规划器配置](../src/tag_nav_planner/config/CONFIG.md)
