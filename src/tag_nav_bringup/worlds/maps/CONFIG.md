# AprilTag 地图配置说明

[English](CONFIG_en.md)

本目录包含工厂 AprilTag 标签地图。该地图由定位节点和标签图规划器共同使用。

## 地图文件

默认文件：[apriltagMap.json](apriltagMap.json)

```text
src/tag_nav_bringup/worlds/maps/apriltagMap.json
```

### JSON 格式

```json
{
  "schema_version": 1,
  "tag_side_lengths": {
    "default": 0.10
  },
  "tag_locations": {
    "0": [-9.0, 12.0, 0.0],
    "1": [-8.0, 12.0, 0.0]
  },
  "map_type": "2d"
}
```

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `schema_version` | int | 本项目维护地图的格式版本，当前为 `1`。定位与规划器也兼容没有该字段的 legacy/pytagmapper 输出。 |
| `tag_side_lengths` | object | 标签边长（米）。`default` 用于未单独指定尺寸的标签。 |
| `tag_locations` | object | 标签 ID 到标签位姿的映射。键必须可转换为整数。 |
| `map_type` | string | 地图表示方式：`2d`、`2.5d` 或 `3d`。 |

### 标签位姿格式

`tag_locations` 的数组格式由 `map_type` 决定：

| `map_type` | 每个标签的格式 | 支持者 |
| --- | --- | --- |
| `2d` | `[x, y, yaw]` | 定位节点和标签图规划器 |
| `2.5d` | `[x, y, yaw, z]` | 仅定位节点 |
| `3d` | 4x4 齐次变换矩阵，按行嵌套数组表示 | 仅定位节点 |

其中 `x`、`y`、`z` 的单位为米，`yaw` 的单位为弧度。`2d` 的第三个数值是标签绕 `+Z` 的朝向，不是高度；这也是默认工厂地图中全部为 `0.0` 的原因。

对于工厂地面标签，地图坐标系原点位于工厂中心地面，`+X` 向右、`+Y` 向前，标签法向和正面均指向 `+Z`。定位节点会处理地面标签坐标系与相机 PnP 坐标系之间的变换。

## 默认工厂地图

默认地图是 19 x 25 网格：

| 参数 | 值 |
| --- | --- |
| X 范围 | `[-9.0, 9.0]` m |
| Y 范围 | `[-12.0, 12.0]` m |
| 标签间距 | `1.0` m |
| 标签数量 | 475（ID `0` 至 `474`） |
| 默认标签边长 | `0.10` m |

标签按行递增编号：

```text
行 0  (Y= 12.0): ID 0-18
行 1  (Y= 11.0): ID 19-37
...
行 24 (Y=-12.0): ID 456-474
```

每行有 19 个标签，X 从 `-9.0` 递增至 `9.0`。

## 创建自定义地图

### 手动创建

创建 `2d` 地图时，为每个标签填写 `[x, y, yaw]`：

```json
{
  "schema_version": 1,
  "tag_side_lengths": {
    "default": 0.10,
    "0": 0.15
  },
  "tag_locations": {
    "0": [0.0, 0.0, 0.0],
    "1": [1.0, 0.0, 0.0],
    "2": [2.0, 0.0, 1.5707963268]
  },
  "map_type": "2d"
}
```

### 使用 pytagmapper 构建

`pytagmapper` 是 Git 子模块，不是 ROS package，因此不能使用 `rosrun pytagmapper generate_map.py`。在子模块目录运行实际工具：

```bash
cd src/3rd_party/pytagmapper
python3 pytagmapper_tools/build_map.py <input-dir> --mode 2d --output-dir <output-dir>
```

该命令在 `<output-dir>` 生成 `map.json`。对 `2d` 模式，它输出的标签位置已经是 `[x, y, yaw]`。复制或转换为项目使用的 `apriltagMap.json` 后，根对象应包含版本字段：

```json
{
  "schema_version": 1,
  "tag_side_lengths": {
    "default": 0.10
  },
  "tag_locations": {
    "0": [0.0, 0.0, 0.0]
  },
  "map_type": "2d"
}
```

并确认 `tag_side_lengths`、`tag_locations` 与 `map_type` 保持不变。pytagmapper 的输入目录格式和运行限制请参阅该子模块自己的 README。

### 从 Gazebo 世界生成的限制

仓库目前**没有**从 `factory.world` 自动提取标签位置并生成 `apriltagMap.json` 的脚本。`generate_apriltag_floor.py` 用于生成标签地板的 Gazebo 资源，`generate_factory_map.py` 用于生成占据栅格地图；两者都不生成标签位姿地图。

## 使用地图

### 定位节点

`gazebo_cameras.json` 的 `map.uri` 指向地图：

```json
{
  "map": {
    "uri": "package://tag_nav_bringup/worlds/maps/apriltagMap.json"
  }
}
```

也可以在启动时覆盖：

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/custom_map.json
```

定位节点支持 `2d`、`2.5d` 和 `3d`；当 `fusion.mode` 不是 `auto` 时，它必须与地图的 `map_type` 一致。

### 标签图规划器

`planner.yaml` 的 `tag_map_file` 指向地图：

```yaml
tag_map_file: "package://tag_nav_bringup/worlds/maps/apriltagMap.json"
```

标签图规划器只支持 `2d` 地图，因为它需要规则网格上的 `[x, y, yaw]` 标签位姿。

## 验证

在仓库根目录检查默认地图：

```bash
python3 -m json.tool src/tag_nav_bringup/worlds/maps/apriltagMap.json >/dev/null
python3 - <<'PY'
import json

path = "src/tag_nav_bringup/worlds/maps/apriltagMap.json"
with open(path) as stream:
    data = json.load(stream)

assert data["schema_version"] == 1
assert data["map_type"] == "2d"
assert len(data["tag_locations"]) == 475
assert all(len(pose) == 3 for pose in data["tag_locations"].values())
print("tag map validation passed")
PY
```

可通过规划器 GUI 查看标签位置与连通性：

```bash
roslaunch tag_nav_bringup planner.launch
rosrun tag_nav_planner tag_nav_gui.py
```

启动定位后可查看检测结果：

```bash
rostopic echo /apriltag_localization/camera_best_tags
```

## 与其他配置的关联

| 文件 | 关联方式 |
| --- | --- |
| `gazebo_cameras.json` | 通过 `map.uri` 引用 |
| `planner.yaml` | 通过 `tag_map_file` 引用 |
| `connectivity.json` | 标签 ID 必须与地图一致 |
| `factory.world` | 标签的物理位置与地图应保持一致 |

## 相关文档

- [主 README](../../../../README.md)
- [定位配置](../../../tag_nav_localization/config/CONFIG.md)
- [规划器配置](../../../tag_nav_planner/config/CONFIG.md)
