# tag_nav

[English Version](README_en.md)

tag_nav 是一个面向仓库 AGV 的 ROS 1 Noetic / Gazebo Classic 仿真与 AprilTag 定位项目。项目包含参数化 AGV 模型、带 AprilTag 地面的工厂世界、多相机质量加权融合定位节点、A* 路径规划器和静态地图服务。

![项目架构图](assets/architecture.png)

项目当前面向 ROS Noetic，仍处于持续开发阶段。

## 功能概览

- **参数化机器人模型**：`warehouse_agv` 差速驱动 AGV，配备 IMU、四个广角相机和一个下视鱼眼相机
- **工厂仿真世界**：`factory.world` 包含 AprilTag 地面、围墙、货架、圆柱和立柱等障碍物
- **多相机 AprilTag 定位**：从多个相机同步检测 `tag36h11`，按质量加权融合位姿，发布有效性和 TF 状态
- **EKF 里程计**：使用 `robot_localization` 融合轮式里程计与 IMU
- **静态地图服务**：依据 Gazebo 碰撞几何投影生成的 `map_server` 兼容地图
- **标签图路径规划**：基于 AprilTag 网格图的 A* 规划器与定点跟随控制器
- **导航栈集成**：支持 `move_base` + DWA 局部规划器的完整导航闭环

![Gazebo仿真场景](assets/gazebo_scene.png)

## 项目结构

```text
.
├── LICENSE
├── README.md
├── README_en.md
├── .gitmodules
├── scripts/
│   └── install.bash              # 一键依赖安装脚本
├── assets/                       # 文档插图目录
│   ├── architecture.png          # 项目架构图
│   ├── gazebo_scene.png          # Gazebo仿真场景
│   ├── factory_world.png         # 工厂世界场景
│   ├── apriltag_localization.png # AprilTag定位效果
│   ├── path_planning.png         # 路径规划演示
│   ├── tf_tree.png               # TF坐标树
│   ├── camera_setup.png          # 相机配置示意图
│   ├── factory_map.png           # 静态地图
│   ├── navigation_demo.png       # 导航闭环演示
│   └── planner_gui.png           # 标签图规划器GUI
└── src/
    ├── tag_nav_bringup/          # 完整演示 launch、地图、rviz、worlds
    │   ├── launch/factory_gazebo.launch
    │   ├── launch/factory_navigation.launch
    │   ├── launch/planner.launch
    │   ├── launch/factory_apriltag_localization.launch
    │   ├── maps/factory_map.{yaml,pgm}
    │   ├── scripts/generate_factory_map.py
    │   └── worlds/factory.world
    ├── tag_nav_description/
    │   ├── launch/{display,gazebo}.launch
    │   ├── rviz/warehouse_agv.rviz
    │   └── urdf/robot.urdf.xacro
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

## 软件环境

推荐环境：


| 组件     | 版本                           |
| ---------- | -------------------------------- |
| Ubuntu   | 20.04                          |
| ROS      | Noetic                         |
| Gazebo   | Classic（由`gazebo_ros` 提供） |
| 构建工具 | `catkin_tools`                 |
| 编译标准 | C++14                          |

系统侧建议准备：

- ROS Noetic 完整桌面版
- 足够的磁盘空间用于 Gazebo 模型和编译产物

## 如何安装

### 1. 克隆仓库

仓库包含 `pytagmapper` Git 子模块，请递归克隆：

```bash
git clone --recurse-submodules https://github.com/LiO2-coder/tag_nav.git
cd tag_nav
```

已有工作副本可执行：

```bash
git submodule update --init --recursive
```

### 2. 一键安装依赖

项目提供了便捷的安装脚本，可自动安装系统依赖、ROS 包和 Python 依赖：

```bash
source /opt/ros/noetic/setup.bash
./scripts/install.bash
```

安装脚本会：

- 更新系统包列表
- 安装 `build-essential`、`cmake`、`git` 等构建工具
- 安装 `catkin_tools`、OpenCV、JsonCpp、AprilTag 等系统库
- 安装 ROS Navigation、Map Server、Robot Localization 等功能包
- 安装 Python 依赖（numpy、matplotlib、opencv-python 等）
- 初始化 `rosdep` 并安装工作区依赖
- 询问是否立即编译项目

### 3. 手动安装（可选）

如果需要手动控制安装过程，可参考以下步骤：

```bash
# 加载 ROS 环境
source /opt/ros/noetic/setup.bash

# 更新系统包
sudo apt update

# 安装 catkin 工具
sudo apt install python3-catkin-tools

# 使用 rosdep 安装依赖
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 安装导航栈（如果需要导航功能）
sudo apt install ros-noetic-navigation
```

### 4. 编译项目

一键安装脚本会询问是否编译，也可手动编译：

```bash
cd /path/to/tag_nav
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

或只编译特定包：

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
`src/tag_nav_bringup/worlds/factory.world`。

![工厂世界场景](assets/factory_world.png)

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

- EKF 融合 `/agv/odom` 与 `/agv/imu/data`，发布 `odom -> base_footprint`
- AprilTag 节点在 `correction` 模式下根据标签位姿发布 `map -> odom`
- 默认只有下视相机启用，因为工厂 AprilTag 地图铺设在地面

![AprilTag定位效果](assets/apriltag_localization.png)

### 4. 标签图规划器演示

```bash
roslaunch tag_nav_bringup planner.launch
```

该启动文件会启动工厂 Gazebo、AprilTag 定位、`map_server` 和规划器节点，
默认启用 RViz 显示（`rviz:=false` 关闭）。

规划器配置位于 [planner.yaml](src/tag_nav_planner/config/planner.yaml) 和
[connectivity.json](src/tag_nav_planner/config/connectivity.json)。

![路径规划演示](assets/path_planning.png)

详细配置说明请参阅 [规划器配置说明](src/tag_nav_planner/config/CONFIG.md)。

### 5. 工厂导航验证（A* + DWA）

```bash
roslaunch tag_nav_bringup factory_navigation.launch
```

该启动文件启动完整的导航闭环：Gazebo、AprilTag 定位、`map_server`、
`move_base` 和 RViz。在 RViz 中选择 **2D Nav Goal** 即可发送导航目标。

![导航闭环演示](assets/navigation_demo.png)

## 话题与 TF

仿真中常用的输入/输出如下：


| 名称                                      | 类型                                             | 说明                                                             |
| ------------------------------------------- | -------------------------------------------------- | ------------------------------------------------------------------ |
| `/agv/cmd_vel`                            | `geometry_msgs/Twist`                            | 差速驱动速度指令                                                 |
| `/agv/odom`                               | `nav_msgs/Odometry`                              | Gazebo 轮式里程计                                                |
| `/agv/imu/data`                           | `sensor_msgs/Imu`                                | 模拟 IMU                                                         |
| `/agv/camera/<name>/image_raw`            | `sensor_msgs/Image`                              | 相机图像，`name` 为 `front`、`rear`、`left`、`right` 或 `bottom` |
| `/apriltag_localization/camera_best_tags` | `tag_nav_localization/CameraBestTagArray`        | 每个相机的最佳标签结果                                           |
| `/apriltag_localization/localization`     | `tag_nav_localization/FusedAprilTagLocalization` | 融合位姿、质量和 TF 状态                                         |
| `/apriltag_localization/pose`             | `geometry_msgs/PoseWithCovarianceStamped`        | 融合后的位姿                                                     |

工厂定位演示的 TF 链为：

```text
map -> odom -> base_footprint -> base_link -> sensors
```

![TF坐标树](assets/tf_tree.png)

重要提示：不要让 Gazebo、EKF 和定位节点同时发布同一条 TF。`factory_gazebo.launch` 的
`publish_odom_tf` 默认值为 `true`；使用 EKF 时应设为 `false`，正如
`factory_apriltag_localization.launch` 的配置。

## 联调故障复盘与已验证修复

这一节记录本项目在 Gazebo 中实际复现、定位并修复过的问题。排查 TF 时，
先确认"哪一个节点发布哪一条边"和"消息的时间戳/数据源"，再判断是不是
AprilTag 算法问题。

### 1. 修正模式的 TF 链和重复时间戳

修正模式不是让 AprilTag 节点发布 `odom -> base_footprint`，而是保持以下职责：

```text
robot_localization:  /agv/odom + /agv/imu/data -> odom -> base_footprint
AprilTag:            T_map_base * inverse(T_odom_base) -> map -> odom
最终链路:             map -> odom -> base_footprint
```

定位模式才直接发布 `map -> base_footprint`。

早期出现过 `TF_REPEATED_DATA` 警告，原因是同一 parent/child 在仿真时间下被重复广播
相同时间戳。现在发布前会对 `parent + child + stamp` 做严格递增检查；仿真时间回退时清空检查状态。

### 2. 丢失 Tag 时为什么看起来像"里程计停了"

没有 Tag 时，里程计仍应持续运行。之前看到 `map -> base_footprint` 停住，
并不是 `odom -> base_footprint` 停止，而是旧的 `map -> odom` 时间戳限制了整条 TF 链。

现在 correction 模式在丢 Tag 后会以当前时间戳重发缓存的 `map -> odom`，同时标记
`~localization.valid` 为 `false`。这样可以保持 TF 链可用，同时明确区分"视觉定位无效"。

### 3. 手动拖动 Gazebo 模型导致 `odom` 跳变

这是最重要的仿真问题。差速插件曾使用 `<odometrySource>world</odometrySource>`，这会把
Gazebo 世界真值直接写入 `/agv/odom`。手动瞬移模型时，EKF 会把跳变传播到 TF 链。

当前已改为 `<odometrySource>encoder</odometrySource>`，并在 EKF 配置中关闭 IMU 的绝对世界航向。
这样手动改变 Gazebo 世界位姿时，`odom -> base_footprint` 保持不变；视觉节点通过新的标签观测
更新 `map -> odom`。

### 4. 图像时间戳、同步等待与高速旋转误差

同步器的 `wait_sec` 是"等待其他相机帧到达"的延迟，不是从图像测量时间戳中减去的时间。
处理一帧时间为 `t` 的图像时，应在同一 `t` 查询 `T_odom_base(t)`。

高速原地旋转时仍可能看到小幅角度误差，主要来自图像采集、传输和检测延迟。

### 5. 下视鱼眼相机和地面 Tag 的坐标约定

下视相机使用 `equidistant` 分支整流后再进行 AprilTag 检测和 IPPE 方形 PnP。
OpenCV `SOLVEPNP_IPPE_SQUARE` 对角点顺序有严格要求，地面地图将 Tag 法向定义为 `+Z`。

当前代码先按 OpenCV 要求解算并验证四个角点为正深度，再通过 `diag(1, -1, -1)` 将 PnP Tag
坐标转换到地图的地面 Tag 坐标约定。

![相机配置示意图](assets/camera_setup.png)

### 6. Gazebo `joint_cmd` / discovery 警告

以下提示来自 Gazebo Classic GUI 的 `JointControlWidget`：

```text
Node::Advertise(): Error advertising topic [/warehouse_agv/joint_cmd]
```

这是 Gazebo Transport 内部话题，不是 ROS 话题。只要 `/agv/cmd_vel`、相机图像、
`/agv/odom` 和 `/tf` 正常，该警告可以忽略。

排查/规避方法：

```bash
# 不需要 Gazebo GUI 时，直接关闭 GUI
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false

# 多网卡、VPN 或 Docker 环境下，指定双方可达的真实网卡地址
export GAZEBO_IP=10.10.151.14
export IGN_IP=10.10.151.14
```

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

# 3) 查看 AprilTag 状态
rostopic echo /apriltag_localization/localization

# 4) 确认 TF 没有多个动态发布者
rostopic info /tf
rosnode info /agv_ekf_odom
rosnode info /apriltag_localization
```

## 配置说明

项目包含多个配置文件，详细说明请点击查看：

| 配置类型 | 文件路径 | 说明 |
|----------|----------|------|
| **定位配置** | [gazebo_cameras.json 配置说明](src/tag_nav_localization/config/CONFIG.md) | 多相机 AprilTag 检测、融合、输出配置 |
| **标签地图** | [apriltagMap.json 配置说明](src/tag_nav_bringup/worlds/maps/CONFIG.md) | 标签位置、边长、坐标系定义 |
| **规划器配置** | [planner.yaml / connectivity.json 配置说明](src/tag_nav_planner/config/CONFIG.md) | A* 路径规划、运动控制、连通性配置 |

### 快速配置参考

#### 定位节点核心配置

- **标签地图路径**：`gazebo_cameras.json` → `map.uri`
- **相机启用状态**：`gazebo_cameras.json` → `cameras[].enabled`
- **TF 模式**：`gazebo_cameras.json` → `output.tf_mode` (correction/localization)
- **融合模式**：`gazebo_cameras.json` → `fusion.mode` (auto/2d/2.5d/3d)

#### 规划器核心配置

- **起终点标签**：`planner.yaml` → `start_tag` / `goal_tag`
- **运动速度**：`planner.yaml` → `linear_velocity` / `angular_velocity`
- **到达容差**：`planner.yaml` → `position_tolerance` / `heading_tolerance`
- **标签连通性**：`connectivity.json` → `connectivity[标签ID]`

## AprilTag 定位配置

定位配置文件为 [gazebo_cameras.json](src/tag_nav_localization/config/gazebo_cameras.json)，
标签地图为 [apriltagMap.json](src/tag_nav_bringup/worlds/maps/apriltagMap.json)。

配置文件包含以下主要部分：

- `cameras`：图像话题、相机内参、相机到机器人基座的外参和启用状态
- `detector`：标签族、线程数、降采样和边缘细化
- `synchronization` / `runtime`：多相机同步、处理频率和超时
- `quality`：面积、畸变、决策边缘和角点清晰度质量项
- `fusion`：`auto`、`2d`、`2.5d`、`3d` 模式、相机权重和离群值门限
- `validation`：范围、标签面积、未映射标签及过期帧检查
- `output`：坐标系、TF 模式、调试图像和无效位姿方差

可通过启动参数覆盖标签地图：

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/apriltagMap.json
```

详细信息请参阅 [定位配置说明](src/tag_nav_localization/config/CONFIG.md)。

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

# 手动控制 AGV
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/agv/cmd_vel
```

## TODO: 开发计划

- [ ]  支持 ROS 2 迁移
- [ ]  接入激光雷达数据进行动态避障
- [ ]  增加更多标签配置和地图编辑工具
- [ ]  优化多相机同步算法
- [ ]  Sim2Real 对接真实 AGV 平台

## 贡献

欢迎提交 Issue、PR 或实验记录。比较建议的贡献方式：

- 报告安装问题时附上系统版本、ROS 发行版、Gazebo 版本和完整报错
- 修改定位或规划算法时，尽量附带一组可复现实验结果
- 不要提交 `build/`、`devel/`、`logs/`、`install/` 等生成目录
- 如果新增第三方依赖，请说明来源、许可证和安装方式

## 鸣谢

本项目感谢以下开源项目和社区：

- AprilTag：视觉标签检测与定位
- Gazebo：机器人仿真环境
- ROS / ROS Navigation：机器人操作系统与导航栈
- robot_localization：传感器融合与状态估计
- OpenCV：计算机视觉与图像处理
- pytagmapper：AprilTag 地图工具链

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

`src/3rd_party/pytagmapper` 是独立的 Git 子模块，遵循其上游仓库的许可证和版权声明。

第三方模型、SDK、预训练权重和机器人资产分别遵循其原始项目或服务条款。

## 作者

- **GitHub**: [LiO2-coder](https://github.com/LiO2-coder)

方向：ROS 机器人开发、AGV 定位与导航、仿真感知

## 版本历史

### v1.0

- 搭建 Gazebo 工厂 AGV 仿真场景
- 实现多相机 AprilTag 质量加权融合定位
- 集成 EKF 里程计与 IMU 融合
- 实现 A* 标签图路径规划器
- 集成 ROS Navigation 栈
- 添加一键依赖安装脚本
- 完善联调故障复盘文档

## 支持

如果运行失败，优先检查这些信息：

```bash
# 检查环境
echo $ROS_DISTRO
catkin --version

# 检查依赖
dpkg -l | grep -E "ros-noetic|gazebo|apriltag"
python3 -m pip list | grep -E "numpy|opencv|matplotlib"

# 检查子模块
git submodule status

# 检查编译
catkin list
```

常见问题包括：

- **依赖安装失败**：确保已正确 source ROS 环境，`ROS_DISTRO` 为 `noetic`
- **catkin build 失败**：检查是否安装了 `python3-catkin-tools`
- **Gazebo 启动失败**：检查 OpenGL 环境，尝试 `export LIBGL_ALWAYS_SOFTWARE=1`
- **子模块缺失**：运行 `git submodule update --init --recursive`
- **相机话题无数据**：确认 Gazebo 插件已正确加载，检查 `rostopic list`

需要帮助时，请带上完整报错、运行命令、系统环境和相关信息。

您可以通过以下方式联系：

- GitHub Issues: [项目 Issues 页面](https://github.com/LiO2-coder/tag_nav/issues)

---

⭐ 如果这个项目对您有帮助，请给个 Star！
