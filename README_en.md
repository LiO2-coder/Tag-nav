# Tag_nav

[中文](README.md)

`Tag_nav` is a ROS 1 Noetic / Gazebo Classic simulation project for warehouse AGVs. It provides a parametric AGV, an AprilTag floor, quality-weighted multi-camera localization, EKF odometry, tag-graph planning, and a `move_base` navigation validation environment.

![Project architecture](assets/architecture.png)

The project currently targets ROS Noetic and is under active development.

## Features

- **Parametric robot model**: `warehouse_agv`, a differential-drive AGV with an IMU, four wide-angle cameras, and one downward fisheye camera.
- **Factory simulation world**: `factory.world` contains the AprilTag floor, walls, racks, cylinders, and pillars.
- **Multi-camera AprilTag localization**: synchronized `tag36h11` detection with quality-weighted pose fusion and TF status reporting.
- **EKF odometry**: wheel odometry and IMU fusion through `robot_localization`.
- **Static map service**: a `map_server`-compatible map projected from Gazebo collision geometry.
- **Tag-graph planning**: an AprilTag grid A* planner with stop-and-go waypoint following.
- **Navigation integration**: `move_base`, `global_planner` A*, and DWA local planning for a complete navigation loop.

![Gazebo simulation](assets/gazebo_scene.png)

## Repository layout

```text
.
├── LICENSE
├── README.md
├── README_en.md
├── .gitmodules
├── assets/                         # Documentation images (to be added)
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
    └── 3rd_party/pytagmapper/      # Git submodule
```

## Software environment


| Component    | Version                          |
| -------------- | ---------------------------------- |
| Ubuntu       | 20.04                            |
| ROS          | Noetic                           |
| Gazebo       | Classic, provided by`gazebo_ros` |
| Build tool   | `catkin_tools`                   |
| C++ standard | C++14                            |

ROS Noetic Desktop Full is recommended. Leave enough disk space for Gazebo models and build artifacts.

## Installation and build

### 1. Clone the repository

The repository contains the `pytagmapper` Git submodule:

```bash
git clone --recurse-submodules https://github.com/LiO2-coder/Tag_nav.git
cd Tag_nav
```

For an existing checkout:

```bash
git submodule update --init --recursive
```

### 2. Install dependencies

The helper script installs system, ROS, and Python dependencies, initializes `rosdep`, and asks whether to build the workspace:

```bash
source /opt/ros/noetic/setup.bash
./scripts/install.bash
```

For manual installation:

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install python3-catkin-tools ros-noetic-navigation
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the workspace

```bash
cd ~/Tag_nav
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

To build only the project packages:

```bash
catkin build tag_nav_description tag_nav_bringup tag_nav_localization tag_nav_planner
```

## Quick start

Source both ROS and the workspace in every terminal that launches or inspects the system:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 1. Start the factory Gazebo world

```bash
roslaunch tag_nav_bringup factory_gazebo.launch
```

Common arguments:

```bash
roslaunch tag_nav_bringup factory_gazebo.launch gui:=false paused:=false
```

The default namespace is `/agv`, the robot model is `warehouse_agv`, and the world file is `src/tag_nav_bringup/worlds/factory.world`.

![Factory world](assets/factory_world.png)

### 2. View the robot model only

```bash
roslaunch tag_nav_description display.launch use_rviz:=true
```

### 3. Start the AprilTag localization demo

This entry point starts Gazebo, the `robot_localization` EKF, and the AprilTag localization node:

```bash
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false
```

With the default configuration, the EKF publishes `odom -> base_footprint`, the AprilTag node publishes `map -> odom` in `correction` mode, and only the downward camera is enabled for the floor tags.

![AprilTag localization](assets/apriltag_localization.png)

![Camera configuration](assets/camera_setup.png)

### 4. Start the tag-graph planner

```bash
roslaunch tag_nav_bringup planner.launch
```

This starts Gazebo, localization, `map_server`, the tag-graph planner, and RViz. The planner defaults to `auto_start: false`; start the GUI in another terminal to choose the start and goal tags:

```bash
rosrun tag_nav_planner tag_nav_gui.py
```

Use `rviz:=false` to disable RViz. See the [planner configuration](src/tag_nav_planner/config/CONFIG_en.md).

![Path planning](assets/path_planning.png)

### 5. Validate factory navigation (A* + DWA)

```bash
roslaunch tag_nav_bringup factory_navigation.launch
```

This starts Gazebo, AprilTag localization, `map_server`, `move_base`, and RViz. `move_base` uses `global_planner/GlobalPlanner` for A* search and `dwa_local_planner/DWAPlannerROS` for local control. Select **2D Nav Goal** in RViz to send a goal.

![Navigation demo](assets/navigation_demo.png)

## Runtime interfaces

### Topics


| Name                                      | Type                                             | Direction | Description                                                   |
| ------------------------------------------- | -------------------------------------------------- | ----------- | --------------------------------------------------------------- |
| `/agv/cmd_vel`                            | `geometry_msgs/Twist`                            | input     | Differential-drive velocity command                           |
| `/agv/odom`                               | `nav_msgs/Odometry`                              | output    | Gazebo wheel odometry                                         |
| `/agv/odometry/filtered`                  | `nav_msgs/Odometry`                              | output    | EKF-filtered odometry used by DWA                             |
| `/agv/imu/data`                           | `sensor_msgs/Imu`                                | output    | Simulated IMU data                                            |
| `/agv/camera/<name>/image_raw`            | `sensor_msgs/Image`                              | output    | Camera image for`front`, `rear`, `left`, `right`, or `bottom` |
| `/map`                                    | `nav_msgs/OccupancyGrid`                         | output    | Static factory map from`map_server`                           |
| `/apriltag_localization/camera_best_tags` | `tag_nav_localization/CameraBestTagArray`        | output    | Best Tag result from each camera                              |
| `/apriltag_localization/localization`     | `tag_nav_localization/FusedAprilTagLocalization` | output    | Fused pose,`valid`, `tf_status`, and localization age         |
| `/apriltag_localization/pose`             | `geometry_msgs/PoseWithCovarianceStamped`        | output    | Valid fused pose                                              |
| `/planner/path`                           | `nav_msgs/Path`                                  | output    | Planned tag-graph path                                        |
| `/planner/state`                          | `std_msgs/String`                                | output    | Current planner state                                         |
| `/planner/cancel`                         | `std_msgs/Empty`                                 | input     | Cancel tag-graph following                                    |

### Services


| Name                 | Type                       | Description                                                                               |
| ---------------------- | ---------------------------- | ------------------------------------------------------------------------------------------- |
| `/planner/plan_path` | `tag_nav_planner/PlanPath` | Plan between`start_tag` and `goal_tag`, returning success, a message, and `nav_msgs/Path` |

The TF chain in the factory localization demo is:

```text
map -> odom -> base_footprint -> base_link -> sensors
```

![TF tree](assets/tf_tree.png)

`factory_gazebo.launch` defaults to Gazebo publishing `odom -> base_footprint`. The EKF launch entry sets `publish_odom_tf` to `false` to avoid duplicate dynamic TF publishers. See [integration troubleshooting](docs/troubleshooting_en.md) for details.

## Documentation index


| Document                                                                   | Contents                                                                      |
| ---------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| [Localization configuration](src/tag_nav_localization/config/CONFIG_en.md) | Camera, detection, synchronization, fusion, TF output, and runtime parameters |
| [Tag map configuration](src/tag_nav_bringup/worlds/maps/CONFIG_en.md)      | Map formats, coordinate conventions, versioning, generation, and validation   |
| [Planner configuration](src/tag_nav_planner/config/CONFIG_en.md)           | A* parameters, waypoint following, connectivity masks, and GUI tools          |
| [Integration troubleshooting](docs/troubleshooting_en.md)                  | TF ownership, lost localization, Gazebo, and timestamp diagnostics            |

## Common validation commands

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

## Roadmap

- [ ]  ROS 2 migration
- [ ]  LiDAR-based dynamic obstacle avoidance
- [ ]  More Tag configurations and map editing tools
- [ ]  Improved multi-camera synchronization
- [ ]  Sim2Real integration with a physical AGV

## Contributing

Issues and pull requests are welcome. Include the system version, ROS distribution, Gazebo version, command, and complete error output when reporting installation problems. Include reproducible experiment results when changing localization or planning algorithms. Do not commit generated `build/`, `devel/`, `logs/`, or `install/` directories.

## License and acknowledgements

This project is released under the MIT License; see [LICENSE](LICENSE). `src/3rd_party/pytagmapper` is an independent Git submodule and follows its upstream license and copyright notices.

The project uses and thanks the AprilTag, Gazebo, ROS Navigation, `robot_localization`, OpenCV, and pytagmapper communities.

## Author and support

- GitHub: [LiO2-coder](https://github.com/LiO2-coder)
- Issues: [Project issue tracker](https://github.com/LiO2-coder/tag_nav/issues)
