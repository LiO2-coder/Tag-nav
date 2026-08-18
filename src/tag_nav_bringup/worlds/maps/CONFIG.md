# AprilTag 地图配置说明

本目录包含 AprilTag 标签地图文件和相关资源。

## 地图文件

### apriltagMap.json

AprilTag 标签在世界坐标系中的位置定义，被定位节点和规划器共同使用。

#### 文件位置

```
src/tag_nav_bringup/worlds/maps/apriltagMap.json
```

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
    "2": [-7.0, 12.0, 0.0],
    ...
  },
  "map_type": "2d"
}
```

#### 配置项详解

| 字段 | 类型 | 说明 |
|------|------|------|
| `schema_version` | int | 地图格式版本号，当前为 1 |
| `tag_side_lengths` | object | 标签边长配置 |
| `tag_side_lengths.default` | float | 默认标签边长（米） |
| `tag_locations` | object | 标签位置映射，键为标签 ID，值为 [x, y, z] 坐标 |
| `map_type` | string | 地图类型，"2d" 表示 2D 平面地图 |

#### 标签位置格式

每个标签使用世界坐标系表示位置：

```json
"标签ID": [x, y, z]
```

坐标说明：
- **x**：世界坐标 X 方向位置（米），正方向向右
- **y**：世界坐标 Y 方向位置（米），正方向向前
- **z**：世界坐标 Z 方向位置（米），通常为 0（地面）

#### 坐标系约定

本地图采用以下坐标约定：

1. **世界坐标系原点**：工厂中心地面
2. **标签法向**：+Z 方向向上（地面垂直）
3. **标签朝向**：标签正面朝向 +Z

注意：地面标签与相机检测的坐标系可能存在变换，定位节点会自动处理。

## 工厂地图布局

默认工厂地图使用 19×25 的网格布局：

| 参数 | 值 |
|------|-----|
| X 范围 | [-9.0, 9.0] 米 |
| Y 范围 | [-12.0, 12.0] 米 |
| 标签间距 | 1.0 米 |
| 标签总数 | 475 个（ID 0-474） |
| 标签边长 | 0.10 米 |

### 标签编号规则

标签按行递增编号：

```
行 0 (Y=12.0):  ID 0-18
行 1 (Y=11.0):  ID 19-37
行 2 (Y=10.0):  ID 38-56
...
行 24 (Y=-12.0): ID 456-474
```

每行 19 个标签，从 X = -9.0 开始递增到 X = 9.0。

### 障碍物区域

工厂世界中包含以下障碍物（标签不可通行）：

- **货架区域**：主要通道两侧
- **围墙边界**：地图四周
- **圆柱立柱**：特定位置

这些障碍物在 `connectivity.json` 中体现为受限的连通性。

## 创建自定义地图

### 方法一：使用 pytagmapper

```bash
# 从相机标定结果生成地图
rosrun pytagmapper generate_map.py \
  --input calibration_results.json \
  --output apriltagMap.json
```

### 方法二：手动创建

创建符合格式的 JSON 文件：

```json
{
  "schema_version": 1,
  "tag_side_lengths": {
    "default": 0.10,
    "0": 0.15  // 可以为特定标签设置不同边长
  },
  "tag_locations": {
    "0": [0.0, 0.0, 0.0],
    "1": [1.0, 0.0, 0.0],
    "2": [2.0, 0.0, 0.0]
  },
  "map_type": "2d"
}
```

### 方法三：从 Gazebo 世界生成

如果标签已放置在 Gazebo 世界中，可以提取位置信息：

```bash
# 获取所有模型的位姿
gz model -m

# 转换为地图格式
python scripts/extract_tags_from_world.py
```

## 使用地图

### 在定位节点中使用

配置文件 `gazebo_cameras.json` 中引用：

```json
{
  "map": {
    "uri": "package://tag_nav_bringup/worlds/maps/apriltagMap.json"
  }
}
```

或通过启动参数覆盖：

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/custom_map.json
```

### 在规划器中使用

配置文件 `planner.yaml` 中引用：

```yaml
tag_map_file: "package://tag_nav_bringup/worlds/maps/apriltagMap.json"
```

## 地图验证

### 检查地图格式

```bash
# 使用 Python 验证 JSON 格式
python3 -m json.tool apriltagMap.json

# 检查必需字段
python3 -c "
import json
with open('apriltagMap.json') as f:
    data = json.load(f)
    assert 'schema_version' in data
    assert 'tag_locations' in data
    assert 'tag_side_lengths' in data
    print('地图格式验证通过')
"
```

### 可视化地图

使用规划器 GUI 可视化标签位置：

```bash
roslaunch tag_nav_bringup planner.launch
rosrun tag_nav_planner tag_nav_gui.py
```

### 检查标签检测

启动定位后查看检测到的标签：

```bash
rostopic echo /apriltag_localization/camera_best_tags
```

## 故障排查

### 标签 ID 不匹配

**症状**：定位节点报告未映射标签

**解决**：
1. 确认标签 ID 在地图中存在
2. 检查 `map_type` 是否正确（2d/3d）
3. 验证标签边长设置

### 定位偏差较大

**症状**：检测到标签但位置偏差

**解决**：
1. 检查标签位置的坐标系是否正确
2. 验证标签边长是否与实际一致
3. 确认相机外参标定准确

### 标签位置重复

**症状**：多个标签使用相同位置

**解决**：
确保每个标签 ID 对应唯一的位置，地图中不应有重复。

## 与其他配置的关联

| 文件 | 关联方式 |
|------|----------|
| `gazebo_cameras.json` | 通过 `map.uri` 引用 |
| `planner.yaml` | 通过 `tag_map_file` 引用 |
| `connectivity.json` | 标签 ID 必须一致 |
| `factory.world` | 标签物理位置应匹配 |

## 相关文档

- [主 README](../../../README.md)
- [定位配置](../../tag_nav_localization/config/CONFIG.md)
- [规划器配置](../../tag_nav_planner/config/CONFIG.md)