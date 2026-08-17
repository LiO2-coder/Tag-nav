# laser-tag_nav

ROS 1 仓库 AGV 仿真与 AprilTag 定位项目。

This repository contains a warehouse-style AGV description, a Gazebo Classic
factory world, simulated sensors, a quality-weighted multi-camera AprilTag
localization node, and a static occupancy map that can be loaded by ROS
`map_server`.

> 项目当前面向 ROS Noetic，仍处于持续开发阶段。

## 功能概览

- 参数化 `warehouse_agv` 机器人模型：差速驱动、轮子/脚轮、IMU、Velodyne VLP-16、四个广角相机和一个下视鱼眼相机。
- `factory.world` Gazebo Classic 世界：AprilTag 地面、围墙、货架、圆柱和立柱等障碍物。
- `tag_nav_localization`：从多个相机同步图像中检测 `tag36h11`，按质量加权融合位姿，并发布有效性、质量和 TF 状态。
- 工厂演示启动文件：Gazebo 里由 EKF 负责 `odom -> base_footprint`，AprilTag 定位节点负责 `map -> odom` 修正。
- `factory_map.pgm` / `factory_map.yaml`：依据 Gazebo 碰撞几何投影生成的 `map_server` 静态地图。
- `tag_nav_planner`：基于标签网格图（AprilTag grid graph）的 A* 路径规划器与定点跟随控制器，提供 RViz/独立 GUI 两种操控方式。
- `src/3rd_party/pytagmapper`：用于 AprilTag 地图/工具链的第三方子模块。

## 软件环境

推荐环境：

| 组件 | 版本 |
| --- | --- |
| Ubuntu | 20.04 |
| ROS | Noetic |
| Gazebo | Classic（由 `gazebo_ros` 提供） |
| 构建工具 | `catkin_tools` |
| 编译标准 | C++14 |

主要 ROS 依赖包括 `gazebo_ros`、`gazebo_plugins`、`xacro`、`robot_state_publisher`、
`velodyne_description`、`velodyne_gazebo_plugins`、`robot_localization`、
`cv_bridge`、`image_transport`、`tf` 和 `rviz`。使用静态地图时还需要安装
`map_server`。定位节点还需要系统的 OpenCV、JsonCpp 和 AprilTag 库（`pkg-config`
名称为 `apriltag`）。

## 获取代码

仓库包含 `pytagmapper` Git 子模块，请递归克隆：

```bash
git clone --recurse-submodules <repository-url>
cd laser-tag_nav
```

已有工作副本可执行：

```bash
git submodule update --init --recursive
```

## 安装依赖与构建

先加载 ROS 环境，再使用 `rosdep` 安装仓库声明的依赖：

```bash
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

如果系统尚未安装 catkin 工具：

```bash
sudo apt install python3-catkin-tools
```

如果需要加载仓库提供的静态地图：

```bash
sudo apt install ros-noetic-map-server
```

在工作区根目录构建并加载开发空间：

```bash
catkin build
source devel/setup.bash
```

也可以只构建相关包：

```bash
catkin build tag_nav_description tag_nav_bringup tag_nav_localization tag_nav_planner
```

## 快速开始

### 1. 启动工厂 Gazebo 世界

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tag_nav_bringup factory_gazebo.launch
```

常用参数：

```bash
roslaunch tag_nav_bringup factory_gazebo.launch gui:=false paused:=false
```

启动文件默认使用 `/agv` 命名空间、机器人名 `warehouse_agv` 和
`src/tag_nav_bringup/worlds/factory.world`。可通过 `robot_name`、
`namespace`、`x`、`y`、`z` 和 `yaw` 调整机器人生成参数。

### 2. 仅查看机器人模型

```bash
roslaunch tag_nav_description display.launch use_rviz:=true
```

### 3. 启动 AprilTag 定位演示

该启动文件会同时启动工厂 Gazebo、`robot_localization` EKF 和
AprilTag 定位节点：

```bash
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false
```

在这个模式中：

- EKF 融合 `/agv/odom` 与 `/agv/imu/data`，是 `odom -> base_footprint` 的唯一发布者；
- AprilTag 节点在 `correction` 模式下根据标签位姿发布 `map -> odom`；
- 默认只有下视相机启用，因为工厂 AprilTag 地图铺设在地面；
- 调试图像可通过 `debug_images:=true` 打开（配置文件默认也已开启）。

单独启动定位节点时：

```bash
roslaunch tag_nav_localization apriltag_localization.launch
```

该节点要求私有参数 `~cameras_json`。默认启动文件会将
`config/gazebo_cameras.json` 的内容通过 `textfile` 上传到参数服务器。

## 话题与 TF

仿真中常用的输入/输出如下：

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/agv/cmd_vel` | `geometry_msgs/Twist` | 差速驱动速度指令 |
| `/agv/odom` | `nav_msgs/Odometry` | Gazebo 轮式里程计 |
| `/agv/imu/data` | `sensor_msgs/Imu` | 模拟 IMU |
| `/agv/camera/<name>/image_raw` | `sensor_msgs/Image` | 相机图像，`name` 为 `front`、`rear`、`left`、`right` 或 `bottom` |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | VLP-16 点云 |
| `/apriltag_localization/camera_best_tags` | `tag_nav_localization/CameraBestTagArray` | 每个相机的最佳标签结果 |
| `/apriltag_localization/localization` | `tag_nav_localization/FusedAprilTagLocalization` | 融合位姿、质量和 TF 状态 |
| `/apriltag_localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 融合后的位姿 |

机器人状态发布器负责 `base_footprint` 到传感器链路的静态/固定关节。
工厂定位演示的 TF 链为：

```text
map -> odom -> base_footprint -> base_link -> sensors
```

不要让 Gazebo、EKF 和定位节点同时发布同一条 TF。若不使用工厂定位启动文件，
`factory_gazebo.launch` 的 `publish_odom_tf` 默认值为 `true`；使用 EKF 时应设为
`false`，正如 `factory_apriltag_localization.launch` 的配置。

## 联调故障复盘与已验证修复

这一节记录本项目在 Gazebo 中实际复现、定位并修复过的问题。排查 TF 时，
先确认“哪一个节点发布哪一条边”和“消息的时间戳/数据源”，再判断是不是
AprilTag 算法问题。

### 1. 修正模式的 TF 链和重复时间戳

修正模式不是让 AprilTag 节点发布 `odom -> base_footprint`，而是保持以下职责：

```text
robot_localization:  /agv/odom + /agv/imu/data -> odom -> base_footprint
AprilTag:            T_map_base * inverse(T_odom_base) -> map -> odom
最终链路:             map -> odom -> base_footprint
```

定位模式才直接发布 `map -> base_footprint`。修正模式下，标签融合结果和
`~pose` 仍然表示 `map` 下的 `base_footprint`，只有广播的 TF 子坐标系变为
`odom`。`output.tf_mode` 与 `fusion.mode` 是两个不同概念：前者选择 TF 发布
方式，后者选择 2D/2.5D/3D 位姿模型。

早期出现过：

```text
TF_REPEATED_DATA ignoring data with redundant timestamp
```

原因是同一 parent/child 在仿真时间下被重复广播相同时间戳。现在发布前会对
`parent + child + stamp` 做严格递增检查；仿真时间回退时清空检查状态，允许
Gazebo 重新开始发布。该检查只影响 TF，不影响 `~localization` 和 `~pose`。

### 2. 丢失 Tag 时为什么看起来像“里程计停了”

没有 Tag 时，里程计仍应持续运行。之前看到 `map -> base_footprint` 停住，
并不是 `odom -> base_footprint` 停止，而是旧的 `map -> odom` 时间戳限制了
整条 TF 链的时间查询。

现在 correction 模式在有效 Tag 到来时缓存最后一次 `T_map_odom`；丢 Tag 后：

- 不重新计算、不伪造新的视觉观测；
- 以当前时间戳重发缓存的 `map -> odom`；
- `~localization.valid` 在定位超时后仍为 `false`；
- `~pose` 不发布无效的视觉位姿；
- `~localization.tf_status` 标记为 `correction_held`。

因此 `map -> odom -> base_footprint` 可以继续随独立里程计变化，同时明确区分
“TF 链仍可用”和“当前视觉定位仍然有效”。长时间没有 Tag 时，`map` 下的位置
会随着里程计漂移，这是修正模式的正常语义，直到下一次有效 Tag 更新修正量。

### 3. 手动拖动 Gazebo 模型导致 `odom` 跳变

最重要的仿真问题是差速插件曾使用：

```xml
<odometrySource>world</odometrySource>
```

这会把 Gazebo 世界真值直接写入 `/agv/odom`。在 GUI 中拖动或通过
`/gazebo/set_model_state` 瞬移模型时，世界真值也会瞬移，EKF 再忠实地把跳变
传播到 `odom -> base_footprint`，看起来就像 AprilTag 反向修改了里程计。

当前已改为：

```xml
<odometrySource>encoder</odometrySource>
```

并在 [ekf_odom.yaml](src/tag_nav_localization/config/ekf_odom.yaml) 中关闭
IMU 的绝对世界航向，只融合角速度等局部信息。这样手动改变 Gazebo 世界位姿时，
没有对应的轮编码器/角速度运动，`odom -> base_footprint` 保持不变；视觉节点
通过新的标签观测更新 `map -> odom`，从而修正组合后的 `map -> base_footprint`。

实测瞬移约 1 m 后的预期结果是：

```text
odom -> base_footprint: 基本不变
map  -> odom:           变化约 1 m
map  -> base_footprint: 跟随全局标签修正
```

这也是判断 TF 方向是否正确的最小回归测试。需要注意，真实机器人必须提供
编码器/里程计和 IMU；Gazebo 的 `encoder` 只解决仿真中不应使用世界真值的问题。

### 4. 图像时间戳、同步等待与高速旋转误差

同步器的 `wait_sec` 是“等待其他相机帧到达”的延迟，不是从图像测量时间戳
中减去的时间。处理一帧时间为 `t` 的图像时，应在同一 `t` 查询
`T_odom_base(t)`；若把图像时间错误地改为 `t - wait_sec`，运动中的机器人会
产生额外的 `map -> odom` 假修正。当前同步逻辑还会：

- 只选择不超过 `slop_sec` 的最近帧；
- 跳过超过 `stale_frame_timeout_sec` 的帧；
- 同时遵守 `process_rate_hz` 和 `min_batch_interval_sec`；
- 允许部分相机参与，不会被缺失相机阻塞。

高速原地旋转时仍可能看到小幅角度误差，主要来自图像采集、传输和检测延迟，
不是 `map -> odom` 与 `odom -> base_footprint` 方向错误。应结合图像时间戳、
`processing_time_sec` 和里程计时间戳一起判断。

### 5. 下视鱼眼相机和地面 Tag 的坐标约定

下视相机曾使用错误的焦距参数，导致距离和姿态不稳定。当前工厂配置中的
`bottom` 鱼眼内参与 `robot.gazebo.xacro` 的相机模型保持一致，使用
`equidistant` 分支整流后再进行 AprilTag 检测和 IPPE 方形 PnP。

OpenCV `SOLVEPNP_IPPE_SQUARE` 对角点顺序有严格要求，而地面地图将 Tag 法向
定义为 `+Z`。当前代码先按 OpenCV 要求解算并验证四个角点为正深度，再通过
`diag(1, -1, -1)` 将 PnP Tag 坐标转换到地图的地面 Tag 坐标约定。修改相机
内参、外参或地图 Tag 朝向时，必须重新检查正深度和 `T_map_tag` 的方向。

### 6. Gazebo `joint_cmd` / discovery 警告

下面的提示：

```text
Node::Advertise(): Error advertising topic [/warehouse_agv/joint_cmd]
```

来自 Gazebo Classic GUI 的 `JointControlWidget`，是 GUI 选中模型时为关节控制
面板创建的 Gazebo Transport 内部话题，不是 ROS 话题，也不是 AprilTag、EKF 或
TF 的发布失败。项目 URDF 中没有依赖这个话题；只要 `/agv/cmd_vel`、相机图像、
`/agv/odom` 和 `/tf` 正常，该警告通常可以忽略。

排查/规避方法：

```bash
# 不需要 Gazebo GUI 时，直接关闭 GUI，避免 JointControlWidget
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false

# 多网卡、VPN 或 Docker 环境下，指定双方可达的真实网卡地址
export GAZEBO_IP=10.10.151.14
export IGN_IP=10.10.151.14
```

不要把 `GAZEBO_IP`/`IGN_IP` 固定为 `127.0.0.1` 后再进行相机 Transport 联调：
这可能使底部 wide-angle 相机的 Ignition 图像通道无法发现。远程联调时应使用
Gazebo 服务端和客户端都能路由到的实际地址。`GAZEBO_MODEL_PATH` 中若包含地图
图片等非模型文件，只会产生额外的路径提示，与 TF/定位无关。

### 7. 推荐的故障定位顺序

```bash
# 1) 确认输入是否持续
rostopic hz /agv/odom
rostopic hz /agv/odometry/filtered
rostopic hz /agv/camera/bottom/image_raw

# 2) 确认 TF 的直接发布者和方向
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map odom
rosrun tf tf_echo map base_footprint
rosrun tf view_frames

# 3) 查看 AprilTag 状态，区分视觉无效与 TF 不可用
rostopic echo /apriltag_localization/localization

# 4) 确认同一条 TF 没有多个动态发布者
rostopic info /tf
rosnode info /agv_ekf_odom
rosnode info /apriltag_localization
```

判断原则是：`odom -> base_footprint` 只应由 EKF 发布；修正模式下 AprilTag 只应
发布 `map -> odom`；`map -> base_footprint` 是两者组合结果，不应直接拿它的
冻结/跳动来推断里程计是否工作。

## 静态地图（map_server）

地图文件位于：

- `src/tag_nav_bringup/maps/factory_map.yaml`
- `src/tag_nav_bringup/maps/factory_map.pgm`

加载地图：

```bash
rosrun map_server map_server \
 "$(rospack find tag_nav_bringup)/maps/factory_map.yaml"
```

地图分辨率为 `0.05 m/pixel`，原点为 `[-10.15, -12.65, 0.0]`。这是一张由
`factory.world` 中静态碰撞几何生成的理想化地图，不是由真实激光雷达在线建图得到的
SLAM 地图。

当世界中的货架或墙体发生变化时，可重新生成地图：

```bash
rosrun tag_nav_bringup generate_factory_map.py
```

脚本默认读取 `worlds/factory.world` 并写入 `maps/`，也支持自定义路径：

```bash
rosrun tag_nav_bringup generate_factory_map.py \
  --world /path/to/world.sdf \
  --output-dir /path/to/maps
```

## 工厂导航验证（A* + DWA）

安装 ROS Noetic 导航栈后，可启动基于 AprilTag 定位的最小导航闭环：

```bash
sudo apt install ros-noetic-navigation
roslaunch tag_nav_bringup factory_navigation.launch
```

该启动文件会启动工厂 Gazebo、AprilTag 定位、`map_server`、`move_base` 和 RViz。
定位继续使用 `map -> odom -> base_footprint` TF 链；全局规划器
`global_planner/GlobalPlanner` 以 A* 生成路径，局部规划器使用
`dwa_local_planner/DWAPlannerROS`，速度上限为 `0.6 m/s` 和 `0.8 rad/s`。
在 RViz 中选择 **2D Nav Goal** 即可发送导航目标。传入 `gui:=false` 可同时关闭
Gazebo GUI 与 RViz；若只想关闭 Gazebo GUI 而保留 RViz，可另传 `rviz:=true`。

此配置只使用 `factory_map` 静态障碍层，未接入 `/velodyne_points` 或动态避障，目的是
隔离并验证标签定位对导航闭环的影响。

## 标签图规划器（A* + waypoint 跟随）

规划器以 `apriltagMap.json` 中的标签网格为图，在 `start_tag` 与 `goal_tag`
之间运行 A*（8 方向连通），并驱动 AGV 沿规划的标签路径定点跟随
（stop-and-go）：

```bash
roslaunch tag_nav_bringup planner.launch
```

该启动文件会启动工厂 Gazebo、AprilTag 定位、`map_server` 和规划器节点，
默认启用 RViz 显示（`rviz:=false` 关闭）。规划参数位于
[planner.yaml](src/tag_nav_planner/config/planner.yaml)：设
`auto_start: true` 会在启动时自动规划 `start_tag -> goal_tag`；保持
`false` 时通过 `tag_nav_gui.py`（或 `~plan_path` 服务）触发规划：

```bash
rosrun tag_nav_planner tag_nav_gui.py
```

单独调试规划器节点（不含定位/Gazebo）时，需要外部提供
`map -> base_footprint` TF 链：

```bash
roslaunch tag_nav_planner planner_node.launch
```

标签连通性可用 `tag_connectivity_editor.py` 可视化编辑：

```bash
rosrun tag_nav_planner tag_connectivity_editor.py
```

## AprilTag 定位配置

定位配置文件为 [gazebo_cameras.json](src/tag_nav_localization/config/gazebo_cameras.json)，
标签地图为 [apriltagMap.json](src/tag_nav_bringup/worlds/maps/apriltagMap.json)。

配置文件包含以下主要部分：

- `cameras`：图像话题、相机内参、相机到机器人基座的外参和启用状态；
- `detector`：标签族、线程数、降采样和边缘细化；
- `synchronization` / `runtime`：多相机同步、处理频率和超时；
- `quality`：面积、畸变、决策边缘和角点清晰度质量项；
- `fusion`：`auto`、`2d`、`2.5d`、`3d` 模式、相机权重和离群值门限；
- `validation`：范围、标签面积、未映射标签及过期帧检查；
- `output`：坐标系、TF 模式、调试图像和无效位姿方差。

每个相机的 `enabled` 字段控制是否订阅图像。需要使用真实相机时，应替换
`image_topic`、内参和 `base_to_camera` 外参，并确认相机消息的 `frame_id` 与 TF 一致。
也可以通过启动参数覆盖标签地图：

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/apriltagMap.json
```

## 常用验证命令

```bash
# 查看节点和话题
rosnode list
rostopic list

# 检查输入频率
rostopic hz /agv/imu/data
rostopic hz /agv/camera/bottom/image_raw

# 检查 TF
rosrun tf view_frames
rosrun tf tf_echo map base_footprint

# 展开并检查 Xacro
rosrun xacro xacro src/tag_nav_description/urdf/robot.urdf.xacro \
  > /tmp/warehouse_agv.urdf
```

## 目录结构

```text
.
├── LICENSE
├── README.md
├── .gitmodules
└── src/
    ├── tag_nav_bringup/          # 完整演示 launch、地图、rviz、worlds
    │   ├── launch/factory_gazebo.launch
    │   ├── launch/factory_navigation.launch
    │   ├── launch/planner.launch
    │   ├── maps/factory_map.{yaml,pgm}
    │   ├── scripts/generate_factory_map.py
    │   └── worlds/factory.world
    ├── tag_nav_description/
    │   ├── launch/{display,gazebo}.launch
    │   ├── rviz/warehouse_agv.rviz
    │   └── urdf/
    ├── tag_nav_localization/
    │   ├── config/{gazebo_cameras.json,ekf_odom.yaml}
    │   ├── launch/apriltag_localization.launch
    │   ├── msg/
    │   └── src/apriltag_localization_node.cpp
    ├── tag_nav_planner/
    │   ├── config/{planner.yaml,connectivity.json}
    │   ├── launch/planner_node.launch
    │   ├── scripts/{tag_nav_gui,tag_connectivity_editor}.py
    │   └── srv/PlanPath.srv
    └── 3rd_party/pytagmapper/   # Git submodule
```

## 开发与贡献

欢迎提交 issue、文档改进、仿真场景、定位算法和测试改进。建议的工作流：

1. Fork 仓库并创建功能分支；
2. 保持 ROS 包依赖写在对应的 `package.xml` 中；
3. 提交前运行 `catkin build` 确认可正常构建；
4. 在 issue 或 PR 中说明 ROS 发行版、Gazebo 版本、启动命令和复现步骤。

请不要提交 `build/`、`devel/`、`logs/`、`install/` 等生成目录；这些目录已在
`.gitignore` 中忽略。

## 许可证与第三方代码

本仓库（含 `tag_nav_bringup`、`tag_nav_description`、`tag_nav_localization`、
`tag_nav_planner` 四个 ROS 包）以 MIT 许可证发布，详见根目录 [LICENSE](LICENSE)。
Copyright (c) 2026 LiO2-coder。

`src/3rd_party/pytagmapper` 是独立的 Git 子模块，遵循其上游仓库的许可证和版权声明；
使用、修改或重新分发时请同时遵守上游条款。

## English summary

`laser-tag_nav` is a ROS Noetic / Gazebo Classic warehouse AGV simulation with
multi-camera AprilTag localization and tag-grid path planning. It includes a
parameterized AGV URDF, VLP-16 and IMU simulation, a tagged factory world, EKF
odometry, quality-weighted 2D/2.5D/3D tag-pose fusion, an A* planner over the
tag graph with stop-and-go waypoint following, and a `map_server`-compatible
factory occupancy map. See the sections above for installation, launch
commands, topics, TF ownership, configuration, and contribution guidelines.
