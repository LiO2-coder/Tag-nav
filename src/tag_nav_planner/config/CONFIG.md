# 标签图规划器配置说明

[English](CONFIG_en.md)

本目录包含标签图路径规划器的配置文件。规划器通过 `<rosparam>` 将 `planner.yaml` 加载到节点私有命名空间，并读取标签地图与连通性 JSON。

## planner.yaml

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `start_tag` | int | 0 | 自动规划的起始标签 ID |
| `goal_tag` | int | 474 | 自动规划的目标标签 ID |
| `auto_start` | bool | false | 启动时是否自动规划 |
| `linear_velocity` | float | 0.30 | 线速度上限（m/s） |
| `angular_velocity` | float | 0.50 | 角速度上限（rad/s） |
| `position_tolerance` | float | 0.05 | 位置容差（m） |
| `heading_tolerance` | float | 0.05 | 航向容差（rad） |
| `slow_radius` | float | 0.50 | 开始减速的目标半径（m） |
| `allow_diagonal` | bool | true | 是否允许 8 方向 A* 对角线边 |
| `grid_spacing` | float | 1.0 | 标签网格间距（m） |
| `controller_rate` | float | 20.0 | 路径跟随控制频率（Hz） |
| `pose_timeout` | float | 0.5 | `map -> base_footprint` 位姿超时（s） |
| `map_frame` | string | `map` | 地图坐标系 |
| `base_frame` | string | `base_footprint` | 机器人基座坐标系 |
| `connectivity_json` | string | `package://...` | 连通性 JSON 路径 |
| `tag_map_file` | string | `package://...` | 标签地图路径；规划器要求 `map_type: 2d` |

速度参数控制定点跟随的上限；`position_tolerance` 和 `heading_tolerance` 控制每个航点的到达判定；`slow_radius` 控制接近目标时的减速范围。

## connectivity.json

每个标签使用一个 8 位掩码（0-255）表示指向相邻网格标签的有向边：

| 位 | 方向 | 值 |
| --- | --- | --- |
| 0 | 正北（+Y） | 1 |
| 1 | 东北 | 2 |
| 2 | 正东（+X） | 4 |
| 3 | 东南 | 8 |
| 4 | 正南（-Y） | 16 |
| 5 | 西南 | 32 |
| 6 | 正西（-X） | 64 |
| 7 | 西北 | 128 |

```json
{
  "schema_version": 1,
  "connectivity": {
    "0": 28,
    "1": 124
  }
}
```

例如 `28 = 4 + 8 + 16`，表示允许向东、东南、正南移动。掩码是有向的；如果希望边是双向的，应同时设置两个标签的相反方向。

默认文件主要限制网格外边界方向：内部标签大多为 `255`，边界标签根据网格边缘清除越界方向。`connectivity.json` 本身**没有**完整编码工厂中的货架、圆柱和立柱碰撞区域；需要避开这些障碍时，请使用编辑器为相关标签设置受限掩码，并同时检查 `map_server` 的静态占据地图。

## 使用示例

### 调整速度与容差

```yaml
linear_velocity: 0.20
angular_velocity: 0.30
position_tolerance: 0.02
heading_tolerance: 0.02
slow_radius: 0.80
```

### 自动启动规划

```yaml
auto_start: true
start_tag: 0
goal_tag: 474
```

### GUI 与服务

保持 `auto_start: false` 时，可以使用 GUI：

```bash
rosrun tag_nav_planner tag_nav_gui.py
```

或者调用服务：

```python
import rospy
from tag_nav_planner.srv import PlanPath

rospy.wait_for_service('/planner/plan_path')
plan_path = rospy.ServiceProxy('/planner/plan_path', PlanPath)
response = plan_path(start_tag=0, goal_tag=474)
```

编辑连通性：

```bash
rosrun tag_nav_planner tag_connectivity_editor.py
```

编辑器会同步设置相邻标签的反向边，并将结果保存到 `connectivity.json`。

## 工作流程

1. 加载 `tag_map_file` 和 `connectivity_json`。
2. 将 `2d` 标签位置按 `grid_spacing` 建立 8 邻域图。
3. 在起点和终点之间运行 A*，代价为直线边 `grid_spacing` 或对角边 `sqrt(2) * grid_spacing`。
4. 使用 `map -> base_footprint` TF 进行 stop-and-go 航点跟随。
5. 接近目标时按 `slow_radius` 降低速度，并用位置/航向容差判定到达。

## 故障排查

### 路径规划失败

确认起终点存在于标签地图中、连通性掩码范围为 0-255、`map_type` 为 `2d`，并检查日志中是否报告了孤立节点或越界边。

### 无法到达目标

检查 `map -> base_footprint` TF 是否连续、`pose_timeout` 是否过小、速度是否适合机器人尺寸，并确认静态地图没有把目标区域标记为障碍物。

### 连通性与障碍物不一致

检查标签地图网格、`connectivity.json` 掩码和 `factory_map.pgm` 是否使用同一坐标系与分辨率。货架和立柱不会自动从 Gazebo 世界同步到连通性 JSON。

## 相关工具

- `tag_nav_gui.py`：显示标签网格和规划路径，选择起点/终点并触发服务。
- `tag_connectivity_editor.py`：交互式编辑 8 方向掩码并导入/导出 JSON。

## 相关文档

- [主 README](../../../README.md)
- [定位配置](../../tag_nav_localization/config/CONFIG.md)
- [标签地图配置](../../tag_nav_bringup/worlds/maps/CONFIG.md)
