# laser-tag_nav

ROS 1 仓库 AGV 仿真与 AprilTag 定位项目。

This repository contains a warehouse-style AGV description, a Gazebo Classic
factory world, simulated sensors, a quality-weighted multi-camera AprilTag
localization node, and a static occupancy map that can be loaded by ROS
`map_server`.

> 项目当前面向 ROS Noetic，仍处于持续开发阶段。公开发布前请根据实际
> 仓库地址、维护者信息和硬件接口补充本文档中的项目元数据。

## 功能概览

- 参数化 `warehouse_agv` 机器人模型：差速驱动、轮子/脚轮、IMU、Velodyne VLP-16、四个广角相机和一个下视鱼眼相机。
- `factory.world` Gazebo Classic 世界：AprilTag 地面、围墙、货架、圆柱和立柱等障碍物。
- `laser_tag_nav_localization`：从多个相机同步图像中检测 `tag36h11`，按质量加权融合位姿，并发布有效性、质量和 TF 状态。
- 工厂演示启动文件：Gazebo 里由 EKF 负责 `odom -> base_footprint`，AprilTag 定位节点负责 `map -> odom` 修正。
- `factory_map.pgm` / `factory_map.yaml`：依据 Gazebo 碰撞几何投影生成的 `map_server` 静态地图。
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
catkin build laser-tag_nav_description laser-tag_nav_bringup laser_tag_nav_localization
```

## 快速开始

### 1. 启动工厂 Gazebo 世界

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch laser-tag_nav_bringup factory_gazebo.launch
```

常用参数：

```bash
roslaunch laser-tag_nav_bringup factory_gazebo.launch gui:=false paused:=false
```

启动文件默认使用 `/agv` 命名空间、机器人名 `warehouse_agv` 和
`src/laser-tag_nav_bringup/worlds/factory.world`。可通过 `robot_name`、
`namespace`、`x`、`y`、`z` 和 `yaw` 调整机器人生成参数。

### 2. 仅查看机器人模型

```bash
roslaunch laser-tag_nav_description display.launch use_rviz:=true
```

### 3. 启动 AprilTag 定位演示

该启动文件会同时启动工厂 Gazebo、`robot_localization` EKF 和
AprilTag 定位节点：

```bash
roslaunch laser_tag_nav_localization factory_apriltag_localization.launch gui:=false
```

在这个模式中：

- EKF 融合 `/agv/odom` 与 `/agv/imu/data`，是 `odom -> base_footprint` 的唯一发布者；
- AprilTag 节点在 `correction` 模式下根据标签位姿发布 `map -> odom`；
- 默认只有下视相机启用，因为工厂 AprilTag 地图铺设在地面；
- 调试图像可通过 `debug_images:=true` 打开（配置文件默认也已开启）。

单独启动定位节点时：

```bash
roslaunch laser_tag_nav_localization apriltag_localization.launch
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
| `/apriltag_localization/camera_best_tags` | `laser_tag_nav_localization/CameraBestTagArray` | 每个相机的最佳标签结果 |
| `/apriltag_localization/localization` | `laser_tag_nav_localization/FusedAprilTagLocalization` | 融合位姿、质量和 TF 状态 |
| `/apriltag_localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 融合后的位姿 |

机器人状态发布器负责 `base_footprint` 到传感器链路的静态/固定关节。
工厂定位演示的 TF 链为：

```text
map -> odom -> base_footprint -> base_link -> sensors
```

不要让 Gazebo、EKF 和定位节点同时发布同一条 TF。若不使用工厂定位启动文件，
`factory_gazebo.launch` 的 `publish_odom_tf` 默认值为 `true`；使用 EKF 时应设为
`false`，正如 `factory_apriltag_localization.launch` 的配置。

## 静态地图（map_server）

地图文件位于：

- `src/laser-tag_nav_bringup/maps/factory_map.yaml`
- `src/laser-tag_nav_bringup/maps/factory_map.pgm`

加载地图：

```bash
rosrun map_server map_server \
 "$(rospack find laser-tag_nav_bringup)/maps/factory_map.yaml"
```

地图分辨率为 `0.05 m/pixel`，原点为 `[-10.15, -12.65, 0.0]`。这是一张由
`factory.world` 中静态碰撞几何生成的理想化地图，不是由真实激光雷达在线建图得到的
SLAM 地图。

当世界中的货架或墙体发生变化时，可重新生成地图：

```bash
rosrun laser-tag_nav_bringup generate_factory_map.py
```

脚本默认读取 `worlds/factory.world` 并写入 `maps/`，也支持自定义路径：

```bash
rosrun laser-tag_nav_bringup generate_factory_map.py \
  --world /path/to/world.sdf \
  --output-dir /path/to/maps
```

## AprilTag 定位配置

定位配置文件为 [gazebo_cameras.json](src/laser_tag_nav_localization/config/gazebo_cameras.json)，
标签地图为 [apriltagMap.json](src/laser-tag_nav_bringup/worlds/maps/apriltagMap.json)。

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
roslaunch laser_tag_nav_localization apriltag_localization.launch \
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
rosrun xacro xacro src/laser-tag_nav_description/urdf/robot.urdf.xacro \
  > /tmp/warehouse_agv.urdf

# 运行定位包测试
catkin test laser_tag_nav_localization --no-status
```

## 目录结构

```text
.
├── README.md
├── .gitmodules
└── src/
    ├── laser-tag_nav_bringup/
    │   ├── launch/factory_gazebo.launch
    │   ├── maps/factory_map.{yaml,pgm}
    │   ├── scripts/generate_factory_map.py
    │   └── worlds/factory.world
    ├── laser-tag_nav_description/
    │   ├── launch/{display,gazebo}.launch
    │   ├── rviz/warehouse_agv.rviz
    │   └── urdf/
    ├── laser_tag_nav_localization/
    │   ├── config/{gazebo_cameras.json,ekf_odom.yaml}
    │   ├── launch/{apriltag_localization,factory_apriltag_localization}.launch
    │   ├── msg/
    │   └── src/apriltag_localization_node.cpp
    └── 3rd_party/pytagmapper/   # Git submodule
```

## 开发与贡献

欢迎提交 issue、文档改进、仿真场景、定位算法和测试改进。建议的工作流：

1. Fork 仓库并创建功能分支；
2. 保持 ROS 包依赖写在对应的 `package.xml` 中；
3. 提交前运行 `catkin build` 和相关测试；
4. 在 issue 或 PR 中说明 ROS 发行版、Gazebo 版本、启动命令和复现步骤。

请不要提交 `build/`、`devel/`、`logs/`、`install/` 等生成目录；这些目录已在
`.gitignore` 中忽略。

## 许可证与第三方代码

三个 ROS 包的 `package.xml` 当前声明为 BSD-3-Clause。仓库根目录尚未包含正式的
`LICENSE` 文件；在公开发布前，请补充与所有者确认过的许可证文本、维护者姓名和联系邮箱。

`src/3rd_party/pytagmapper` 是独立的 Git 子模块，遵循其上游仓库的许可证和版权声明；
使用、修改或重新分发时请同时遵守上游条款。

## English summary

`laser-tag_nav` is a ROS Noetic / Gazebo Classic warehouse AGV simulation with
multi-camera AprilTag localization. It includes a parameterized AGV URDF,
VLP-16 and IMU simulation, a tagged factory world, EKF odometry, quality-weighted
2D/2.5D/3D tag-pose fusion, and a `map_server`-compatible factory occupancy map.
See the sections above for installation, launch commands, topics, TF ownership,
configuration, and contribution guidelines.
