laser-tag_nav/                          # 工作空间根目录
├── src/                                # ROS 源代码空间
│   ├── laser-tag_nav_bringup/          # 启动文件和系统集成
│   │   ├── config/                     # 所有配置文件
│   │   │   ├── navigation/             # 导航参数
│   │   │   │   ├── costmap_common_params.yaml
│   │   │   │   ├── local_costmap_params.yaml
│   │   │   │   ├── global_costmap_params.yaml
│   │   │   │   └── move_base_params.yaml
│   │   │   ├── camera/                 # 相机参数
│   │   │   │   └── camera_params.yaml
│   │   │   └── gazebo/                 # Gazebo 相关配置
│   │   │       ├── physics_params.yaml
│   │   │       └── sensor_params.yaml
│   │   ├── launch/                     # 主启动文件
│   │   │   ├── robot.launch            # 机器人启动
│   │   │   ├── navigation.launch       # 导航启动
│   │   │   ├── gazebo.launch           # Gazebo 世界启动
│   │   │   └── all.launch              # 全部启动
│   │   ├── maps/                       # 建图结果
│   │   │   ├── warehouse_gridmap.pgm
│   │   │   ├── warehouse_gridmap.yaml
│   │   │   └── apriltag_map.json
│   │   └── worlds/                     # Gazebo 世界文件 ⭐新增
│   │       └── warehouse.world
│   ├── laser-tag_nav_localization/     # 自定义定位节点
│   │   ├── src/
│   │   │   └── localization_node.cpp
│   │   ├── include/
│   │   │   └── localization_node.h
│   │   ├── launch/
│   │   │   └── localization.launch
│   │   └── CMakeLists.txt
│   ├── laser-tag_nav_description/      # 机器人URDF模型
│   │   ├── urdf/
│   │   │   ├── robot.urdf.xacro
│   │   │   └── robot.gazebo
│   │   ├── meshes/                     # 3D 模型文件
│   │   │   ├── base.stl
│   │   │   └── wheels.stl
│   │   └── config/
│   │       └── joint_limits.yaml
├── scripts/                            # 辅助脚本
│   ├── data_processing/
│   │   ├── convert_logs.py
│   │   └── analyze_trajectory.py
│   ├── coordinate_transform/
│   │   └── tf_utils.py
│   └── setup_external.sh               # 外部项目安装脚本 ⭐新增
├── resources/                          # 资源文件 ⭐新增
│   ├── textures/                       # 地面贴图
│   │   ├── floor_1.jpg
│   │   ├── floor_2.png
│   │   ├── floor_3.bmp
│   │   └── README_textures.md          # 纹理说明
│   └── models/                         # Gazebo 自定义模型 ⭐新增
│       ├── ground_image/               # 地面图片模型
│       │   ├── model.config
│       │   ├── model.sdf
│       │   └── materials/
│       │       └── textures/
│       │           └── photo.jpg
│       └── custom_obstacle/            # 其他自定义模型
├── docs/                               # 文档 ⭐新增
│   ├── images/                         # README 图片资源
│   │   ├── architecture_diagram.png
│   │   ├── demo_screenshot.jpg
│   │   ├── robot_model.png
│   │   ├── world_overview.png
│   │   └── flowchart.svg
│   ├── api/                            # API 文档
│   │   └── localization_api.md
│   └── tutorials/                      # 教程文档
│       ├── setup_guide.md
│       └── usage_examples.md
├── config/                             # 全局配置文件 ⭐新增
│   ├── gazebo_world_config.yaml        # 世界配置（尺寸、光照等）
│   └── image_placement_config.yaml     # 图片放置配置
├── build/                              # 构建目录（自动生成）
├── devel/                              # 开发空间（自动生成）
├── .gitignore                          # Git忽略文件
├── .gitmodules                         # Git子模块配置 ⭐新增
├── CMakeLists.txt                      # 顶层CMake
├── setup.sh                            # 环境设置脚本
├── install_dependencies.sh             # 依赖安装脚本 ⭐新增
├── Dockerfile                          # Docker配置（可选）
├── docker-compose.yml                  # Docker Compose（可选）
└── README.md                           # 项目说明文档
