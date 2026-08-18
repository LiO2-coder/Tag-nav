# AprilTag 定位配置说明

本目录包含 AprilTag 定位节点的配置文件。

## 配置文件

### gazebo_cameras.json

多相机 AprilTag 定位的主配置文件，控制检测、同步、融合和输出行为。

#### 配置结构

```json
{
  "schema_version": 1,
  "map": { ... },
  "quality": { ... },
  "detector": { ... },
  "synchronization": { ... },
  "runtime": { ... },
  "fusion": { ... },
  "temporal_filter": { ... },
  "validation": { ... },
  "output": { ... },
  "cameras": [ ... ]
}
```

#### 配置项详解

##### map

标签地图配置：

| 字段 | 类型 | 说明 |
|------|------|------|
| `uri` | string | AprilTag 地图文件路径，支持 `package://` 协议 |

##### quality

标签质量评估参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mask` | string | "1111" | 质量掩码，四位分别对应面积/畸变/边距/清晰度 |
| `metric_exponents` | object | - | 各质量项的指数权重 |
| `area_reference_px` | float | 1600.0 | 参考面积（像素） |
| `distortion_scale` | float | 0.20 | 畸变尺度参数 |
| `margin_reference` | float | 100.0 | 参考边距（像素） |
| `sharpness_reference` | float | 128.0 | 参考清晰度 |
| `min_quality` | float | 0.01 | 最小质量阈值 |

##### detector

AprilTag 检测器参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `tag_family` | string | "tag36h11" | 标签族，支持 tag16h5, tag25h9, tag36h11 等 |
| `threads` | int | 2 | 检测线程数 |
| `decimate` | float | 1.0 | 降采样因子，1.0 表示不降采样 |
| `blur` | float | 0.0 | 高斯模糊半径 |
| `refine_edges` | bool | true | 是否细化边缘 |
| `max_hamming_dist` | int | 0 | 最大汉明距离，用于错误检测 |

##### synchronization

多相机同步参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `queue_size` | int | 10 | 图像队列大小 |
| `slop_sec` | float | 0.05 | 同步时间窗口（秒） |
| `wait_sec` | float | 0.02 | 等待其他相机帧的时间（秒） |
| `min_batch_interval_sec` | float | 0.0667 | 最小批次间隔（约 15Hz） |

##### runtime

运行时参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `process_rate_hz` | float | 15.0 | 处理频率（Hz） |
| `localization_timeout_sec` | float | 0.50 | 定位超时时间（秒） |

##### fusion

位姿融合参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mode` | string | "auto" | 融合模式：auto, 2d, 2.5d, 3d |
| `min_contributing_cameras` | int | 1 | 最小参与相机数 |
| `camera_confidence_multipliers` | object | {} | 各相机置信度乘数 |
| `outlier_gate.enabled` | bool | false | 是否启用离群值过滤 |
| `outlier_gate.max_position_residual_m` | float | 0.30 | 最大位置残差（米） |
| `min_position_stddev_m` | float | 0.02 | 最小位置标准差（米） |
| `min_yaw_stddev_rad` | float | 0.035 | 最小航向标准差（弧度） |

##### temporal_filter

时域滤波参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | true | 是否启用滤波 |
| `position_time_constant_sec` | float | 0.25 | 位置滤波时间常数（秒） |
| `orientation_time_constant_sec` | float | 0.25 | 姿态滤波时间常数（秒） |
| `max_dt_sec` | float | 1.0 | 最大时间差（秒） |

##### validation

验证参数：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_tag_area_px` | float | 0.0 | 最小标签面积（像素） |
| `max_tag_range_m` | float | 0.0 | 最大标签距离（米），0 表示不限制 |
| `reject_unmapped_tags` | bool | true | 是否拒绝未映射标签 |
| `stale_frame_timeout_sec` | float | 0.20 | 过期帧超时（秒） |

##### output

输出配置：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map_frame` | string | "map" | 地图坐标系 |
| `odom_frame` | string | "odom" | 里程计坐标系 |
| `base_frame` | string | "base_footprint" | 机器人基座坐标系 |
| `tf_mode` | string | "correction" | TF 模式：correction 或 localization |
| `publish_tf` | bool | true | 是否发布 TF |
| `tf_lookup_timeout_sec` | float | 0.05 | TF 查找超时（秒） |
| `correction_tf_tolerance_sec` | float | 0.25 | 修正 TF 容差（秒） |
| `correction_tf_publish_rate_hz` | float | 10.0 | 修正 TF 发布频率 |
| `debug_images` | bool | false | 是否发布调试图像 |
| `invalid_variance` | float | 1000000.0 | 无效位姿方差 |

##### cameras

相机配置数组，每个相机包含：

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | string | 相机名称（front/rear/left/right/bottom） |
| `enabled` | bool | 是否启用该相机 |
| `image_topic` | string | 图像话题 |
| `transport` | string | 图像传输类型（raw/compressed） |
| `data_format` | string | 图像数据格式（rgb8/bgr8等） |
| `frame_id` | string | 相机坐标系 |
| `intrinsics` | object | 相机内参 |
| `intrinsics.K` | array[9] | 相机内参矩阵（fx, fy, cx, cy） |
| `intrinsics.D` | array[5] | 畸变系数 |
| `intrinsics.distortion_model` | string | 畸变模型（plumb_bob/equidistant） |
| `base_to_camera.translation` | array[3] | 基座到相机的平移 |
| `base_to_camera.rotation_rpy` | array[3] | 基座到相机的旋转（roll/pitch/yaw） |

### apriltagMap.json

AprilTag 地图文件，定义标签在世界坐标系中的位置。

#### 配置结构

```json
{
  "schema_version": 1,
  "tag_side_lengths": {
    "default": 0.10
  },
  "tag_locations": {
    "0": [-9.0, 12.0, 0.0],
    "1": [-8.0, 12.0, 0.0],
    ...
  },
  "map_type": "2d"
}
```

#### 配置项详解

| 字段 | 类型 | 说明 |
|------|------|------|
| `schema_version` | int | 地图格式版本号 |
| `tag_side_lengths` | object | 标签边长配置 |
| `tag_side_lengths.default` | float | 默认标签边长（米） |
| `tag_locations` | object | 标签位置映射（ID → [x, y, z]） |
| `map_type` | string | 地图类型（2d/3d） |

#### 标签位置格式

每个标签的位置使用世界坐标系表示：

```json
"0": [x, y, z]
```

- `x`：世界坐标 X 方向位置（米）
- `y`：世界坐标 Y 方向位置（米）
- `z`：世界坐标 Z 方向位置（米），通常为 0（地面）

#### 工厂地图布局

默认工厂地图使用 19x25 的网格布局：

- X 范围：[-9.0, 9.0] 米
- Y 范围：[-12.0, 12.0] 米
- 标签间距：1.0 米
- 标签 ID 按行递增

## 使用示例

### 自定义相机配置

要添加真实相机，修改 `cameras` 数组：

```json
{
  "name": "custom_camera",
  "enabled": true,
  "image_topic": "/camera/image_raw",
  "transport": "raw",
  "data_format": "bgr8",
  "frame_id": "camera_custom_optical_frame",
  "intrinsics": {
    "K": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
    "D": [k1, k2, p1, p2, k3],
    "distortion_model": "plumb_bob"
  },
  "base_to_camera": {
    "translation": [x, y, z],
    "rotation_rpy": [roll, pitch, yaw]
  }
}
```

### 调试模式

启用调试图像以可视化检测过程：

```json
{
  "output": {
    "debug_images": true
  }
}
```

### 更换标签地图

可通过启动参数覆盖：

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/custom_map.json
```

## 故障排查

### 检测不到标签

1. 确认相机已启用（`enabled: true`）
2. 检查图像话题是否正确（`rostopic echo /agv/camera/bottom/image_raw`）
3. 验证相机内参是否正确
4. 检查标签族是否匹配（`tag_family`）

### 定位不稳定

1. 调整质量阈值（`quality.min_quality`）
2. 启用时域滤波（`temporal_filter.enabled: true`）
3. 增加最小参与相机数（`fusion.min_contributing_cameras`）
4. 启用离群值过滤（`fusion.outlier_gate.enabled: true`）

### TF 发布失败

1. 检查坐标系名称是否匹配（`map_frame`, `odom_frame`, `base_frame`）
2. 确认 TF 查找超时合理（`tf_lookup_timeout_sec`）
3. 验证相机外参是否正确

## 相关文档

- [主 README](../../../README.md)
- [EKF 配置](ekf_odom.yaml)
- [规划器配置](../../tag_nav_planner/config/CONFIG.md)