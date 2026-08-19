# Tag_nav

[English](README_en.md)

`Tag_nav` 是一个面向仓库 AGV 的 ROS 1 Noetic / Gazebo Classic 仿真项目，提供参数化 AGV、AprilTag 地面、多相机质量加权定位、EKF 里程计、标签图规划，以及基于 `move_base` 的导航验证环境。

![项目架构图](assets/architecture.png)

项目当前面向 ROS Noetic，仍处于持续开发阶段。

## 功能概览

- **参数化机器人模型**：`warehouse_agv` 差速驱动 AGV，带 IMU、四个广角相机和一个下视鱼眼相机。
- **工厂仿真世界**：`factory.world` 包含 AprilTag 地面、围墙、货架、圆柱和立柱等障碍物。
- **多相机 AprilTag 定位**：同步检测 `tag36h11` 标签，按质量加权融合位姿并发布定位、TF 状态。
- **EKF 里程计**：使用 `robot_localization` 融合轮式里程计与 IMU。
- **静态地图服务**：从 Gazebo 碰撞几何投影生成 `map_server` 兼容地图。
- **标签图路径规划**：基于 AprilTag 网格图的 A* 规划器与定点跟随控制器。
- **导航栈集成**：使用 `move_base`、`global_planner` 的 A* 全局规划和 DWA 局部规划器完成导航闭环。

![Gazebo仿真场景](assets/gazebo_scene.png)

## 项目结构

```text
.
├── LICENSE
├── README.md
├── README_en.md
├── .gitmodules
├── assets/                         # 文档插图目录（图片待补充）
│   ├── architecture.png
│   ├── gazebo_scene.png
│   ├── factory_world.png
│   ├── apriltag_localization.png
│   ├── path_planning.png
│   ├── tf_tree.png
│   ├── camera_setup.png
│   ├── factory_map.png
│   ├── navigation_demo.png
│   └── planner_gui.png
├── docs/
│   ├── troubleshooting.md
│   └── troubleshooting_en.md
├── scripts/
│   └── install.bash
└── src/
    ├── tag_nav_bringup/
    │   ├── config/factory_navigation.yaml
    │   ├── launch/{factory_gazebo,factory_apriltag_localization,planner,factory_navigation}.launch
    │   ├── maps/factory_map.{yaml,pgm}
    │   ├── scripts/{generate_apriltag_floor,generate_factory_map}.py
    │   └── worlds/
    │       ├── factory.world
    │       └── maps/{apriltagMap.json,CONFIG.md,CONFIG_en.md}
    ├── tag_nav_description/
    │   ├── launch/{display,gazebo}.launch
    │   └── urdf/robot.urdf.xacro
    ├── tag_nav_localization/
    │   ├── config/{gazebo_cameras.json,ekf_odom.yaml,CONFIG.md,CONFIG_en.md}
    │   └── launch/apriltag_localization.launch
    ├── tag_nav_planner/
    │   ├── config/{planner.yaml,connectivity.json,CONFIG.md,CONFIG_en.md}
    │   ├── scripts/{tag_nav_gui,tag_connectivity_editor}.py
    │   └── srv/PlanPath.srv
    └── 3rd_party/pytagmapper/      # Git 子模块
```

## 软件环境


| 组件     | 版本                           |
| ---------- | -------------------------------- |
| Ubuntu   | 20.04                          |
| ROS      | Noetic                         |
| Gazebo   | Classic（由`gazebo_ros` 提供） |
| 构建工具 | `catkin_tools`                 |
| 编译标准 | C++14                          |

建议安装 ROS Noetic Desktop Full，并预留 Gazebo 模型和构建产物所需的磁盘空间。

## 安装与构建

### 1. 克隆仓库

仓库包含 `pytagmapper` Git 子模块，请递归克隆：

```bash
git clone --recurse-submodules https://github.com/LiO2-coder/Tag_nav.git
cd Tag_nav
```

已有工作副本可执行：

```bash
git submodule update --init --recursive
```

### 2. 一键安装依赖

安装脚本会安装系统、ROS、Python 依赖，初始化 `rosdep`，并询问是否编译项目：

```bash
source /opt/ros/noetic/setup.bash
./scripts/install.bash
```

### 3. 手动安装依赖

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install python3-catkin-tools ros-noetic-navigation
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. 编译工作区

```bash
cd ~/Tag_nav
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

也可以只编译项目包：

```bash
catkin build tag_nav_description tag_nav_bringup tag_nav_localization tag_nav_planner
```

## 快速开始

每个启动终端都应先加载 ROS 与工作区环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 1. 启动工厂 Gazebo 世界

```bash
roslaunch tag_nav_bringup factory_gazebo.launch
```

常用参数：

```bash
roslaunch tag_nav_bringup factory_gazebo.launch gui:=false paused:=false
```

默认命名空间为 `/agv`，机器人模型名为 `warehouse_agv`，世界文件为 `src/tag_nav_bringup/worlds/factory.world`。

![工厂世界场景](assets/factory_world.png)

### 2. 仅查看机器人模型

```bash
roslaunch tag_nav_description display.launch use_rviz:=true
```

### 3. 启动 AprilTag 定位演示

该入口同时启动 Gazebo、`robot_localization` EKF 和 AprilTag 定位节点：

```bash
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false
```

在默认配置中，EKF 发布 `odom -> base_footprint`，AprilTag 节点以 `correction` 模式发布 `map -> odom`；工厂地面标签场景只启用下视相机。

![AprilTag定位效果](assets/apriltag_localization.png)

![相机配置示意图](assets/camera_setup.png)

### 4. 启动标签图规划器

```bash
roslaunch tag_nav_bringup planner.launch
```

该入口启动 Gazebo、定位、`map_server`、标签图规划器和 RViz。规划器默认 `auto_start: false`；可另开终端启动 GUI 并选择起终点：

```bash
rosrun tag_nav_planner tag_nav_gui.py
```

使用 `rviz:=false` 可关闭 RViz。规划器参数见 [规划器配置说明](src/tag_nav_planner/config/CONFIG.md)。

![路径规划演示](assets/path_planning.png)

### 5. 工厂导航验证（A* + DWA）

```bash
roslaunch tag_nav_bringup factory_navigation.launch
```

该入口启动 Gazebo、AprilTag 定位、`map_server`、`move_base` 和 RViz。`move_base` 配置使用 `global_planner/GlobalPlanner` 的 A* 搜索和 `dwa_local_planner/DWAPlannerROS`；在 RViz 中选择 **2D Nav Goal** 即可发送导航目标。

![导航闭环演示](assets/navigation_demo.png)

## 运行接口

### 主要话题


| 名称                                      | 类型                                             | 方向 | 说明                                                             |
| ------------------------------------------- | -------------------------------------------------- | ------ | ------------------------------------------------------------------ |
| `/agv/cmd_vel`                            | `geometry_msgs/Twist`                            | 输入 | AGV 差速驱动速度指令                                             |
| `/agv/odom`                               | `nav_msgs/Odometry`                              | 输出 | Gazebo 轮式里程计                                                |
| `/agv/odometry/filtered`                  | `nav_msgs/Odometry`                              | 输出 | EKF 滤波后的里程计，供 DWA 使用                                  |
| `/agv/imu/data`                           | `sensor_msgs/Imu`                                | 输出 | 模拟 IMU 数据                                                    |
| `/agv/camera/<name>/image_raw`            | `sensor_msgs/Image`                              | 输出 | 相机图像；`name` 为 `front`、`rear`、`left`、`right` 或 `bottom` |
| `/map`                                    | `nav_msgs/OccupancyGrid`                         | 输出 | `map_server` 发布的静态工厂地图                                  |
| `/apriltag_localization/camera_best_tags` | `tag_nav_localization/CameraBestTagArray`        | 输出 | 每个相机的最佳标签结果                                           |
| `/apriltag_localization/localization`     | `tag_nav_localization/FusedAprilTagLocalization` | 输出 | 融合位姿、`valid`、`tf_status` 和定位时效                        |
| `/apriltag_localization/pose`             | `geometry_msgs/PoseWithCovarianceStamped`        | 输出 | 有效的融合位姿                                                   |
| `/planner/path`                           | `nav_msgs/Path`                                  | 输出 | 标签图规划器生成的路径                                           |
| `/planner/state`                          | `std_msgs/String`                                | 输出 | 标签图规划器当前状态                                             |
| `/planner/cancel`                         | `std_msgs/Empty`                                 | 输入 | 取消标签图跟随                                                   |

### 服务


| 名称                 | 类型                       | 说明                                                                    |
| ---------------------- | ---------------------------- | ------------------------------------------------------------------------- |
| `/planner/plan_path` | `tag_nav_planner/PlanPath` | 传入`start_tag` 与 `goal_tag`，返回规划是否成功、消息和 `nav_msgs/Path` |

工厂定位演示的 TF 链为：

```text
map -> odom -> base_footprint -> base_link -> sensors
```

![TF坐标树](assets/tf_tree.png)

`factory_gazebo.launch` 默认由 Gazebo 发布 `odom -> base_footprint`。使用 EKF 的启动入口会将 `publish_odom_tf` 设为 `false`，避免多个节点发布同一条动态 TF。详细原因和排障步骤见 [联调故障排查](docs/troubleshooting.md)。

## 文档索引


| 文档                                                          | 内容                                         |
| --------------------------------------------------------------- | ---------------------------------------------- |
| [定位配置说明](src/tag_nav_localization/config/CONFIG.md)     | 相机、检测、同步、融合、TF 输出和运行参数    |
| [标签地图配置说明](src/tag_nav_bringup/worlds/maps/CONFIG.md) | 地图格式、坐标约定、版本字段、生成与验证流程 |
| [规划器配置说明](src/tag_nav_planner/config/CONFIG.md)        | A* 参数、路径跟随、连通性掩码和 GUI 工具     |
| [联调故障排查](docs/troubleshooting.md)                       | TF 职责、定位丢失、Gazebo 与时间戳诊断       |

## 常用验证命令

```bash
rosnode list
rostopic list
rostopic hz /agv/imu/data
rostopic hz /agv/camera/bottom/image_raw
rostopic echo /apriltag_localization/localization
rosrun tf tf_echo map base_footprint
rosrun tf view_frames
rosrun xacro xacro src/tag_nav_description/urdf/robot.urdf.xacro > /tmp/warehouse_agv.urdf
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/agv/cmd_vel
```

## 开发计划

- [ ]  支持 ROS 2 迁移
- [ ]  接入激光雷达数据进行动态避障
- [ ]  增加更多标签配置和地图编辑工具
- [ ]  优化多相机同步算法
- [ ]  Sim2Real 对接真实 AGV 平台

## 贡献

欢迎提交 Issue、PR 或实验记录。报告安装问题时请附上系统版本、ROS 发行版、Gazebo 版本、运行命令和完整报错；修改定位或规划算法时，请尽量附带可复现实验结果。请勿提交 `build/`、`devel/`、`logs/`、`install/` 等生成目录。

## 许可证与致谢

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE)。`src/3rd_party/pytagmapper` 是独立 Git 子模块，遵循其上游许可证和版权声明。

项目使用并感谢 AprilTag、Gazebo、ROS Navigation、`robot_localization`、OpenCV 和 pytagmapper 社区。

## 作者与支持

- GitHub: [LiO2-coder](https://github.com/LiO2-coder)
- Issues: [项目 Issues 页面](https://github.com/LiO2-coder/tag_nav/issues)
