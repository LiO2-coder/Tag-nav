# 规划器配置说明

本目录包含标签图路径规划器的配置文件。

## 配置文件

### planner.yaml

规划器运行时参数，通过 `<rosparam>` 加载到节点的私有命名空间。

#### 配置项详解

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `start_tag` | int | 0 | 起始标签 ID |
| `goal_tag` | int | 474 | 目标标签 ID |
| `auto_start` | bool | false | 是否在启动时自动规划 |
| `linear_velocity` | float | 0.30 | 线速度上限（m/s） |
| `angular_velocity` | float | 0.50 | 角速度上限（rad/s） |
| `position_tolerance` | float | 0.05 | 位置容差（m） |
| `heading_tolerance` | float | 0.05 | 航向容差（rad） |
| `slow_radius` | float | 0.50 | 减速半径（m） |
| `allow_diagonal` | bool | true | 是否允许对角线移动 |
| `grid_spacing` | float | 1.0 | 网格间距（m） |
| `controller_rate` | float | 20.0 | 控制器频率（Hz） |
| `pose_timeout` | float | 0.5 | 位姿超时时间（s） |
| `map_frame` | string | "map" | 地图坐标系 |
| `base_frame` | string | "base_footprint" | 机器人基座坐标系 |
| `connectivity_json` | string | package://... | 连通性配置文件路径 |
| `tag_map_file` | string | package://... | 标签地图文件路径 |

#### 控制参数说明

**速度参数：**
- `linear_velocity`：直线运动的最大速度
- `angular_velocity`：旋转运动的最大角速度

**容差参数：**
- `position_tolerance`：到达目标点的距离容差，小于该值认为到达
- `heading_tolerance`：到达目标点的角度容差

**减速半径：**
- `slow_radius`：距离目标小于此值时开始减速

**网格参数：**
- `allow_diagonal`：允许 A* 搜索使用对角线连接（8 方向）
- `grid_spacing`：标签之间的物理间距，用于估算路径长度

### connectivity.json

标签连通性配置，定义标签之间的可通行关系。

#### 配置结构

```json
{
  "schema_version": 1,
  "connectivity": {
    "0": 28,
    "1": 124,
    ...
  }
}
```

#### 连通性编码

每个标签使用一个 8 位掩码（0-255）表示与其相邻标签的连通性：

| 位 | 方向 | 说明 |
|----|------|------|
| 0 | 正北（+Y） | 1 << 0 = 1 |
| 1 | 东北 | 1 << 1 = 2 |
| 2 | 正东（+X） | 1 << 2 = 4 |
| 3 | 东南 | 1 << 3 = 8 |
| 4 | 正南（-Y） | 1 << 4 = 16 |
| 5 | 西南 | 1 << 5 = 32 |
| 6 | 正西（-X） | 1 << 6 = 64 |
| 7 | 西北 | 1 << 7 = 128 |

#### 编码示例

| 值 | 二进制 | 说明 |
|----|--------|------|
| 1 | 00000001 | 只允许向北移动 |
| 4 | 00000100 | 只允许向东移动 |
| 28 | 00011100 | 允许向北、东北、东移动 |
| 124 | 01111100 | 允许向北、东北、东、东南、南移动 |
| 255 | 11111111 | 全方向通行 |

#### 默认工厂布局

默认连通性反映了工厂地图的障碍物布局：

- **内部区域（ID 20-455）**：全方向通行（255）
- **北侧边界**：向南通行受限
- **南侧边界**：向北通行受限
- **东侧边界**：向西通行受限
- **西侧边界**：向东通行受限

## 使用示例

### 自定义规划参数

修改 `planner.yaml` 调整运动行为：

```yaml
# 更保守的速度设置
linear_velocity: 0.20
angular_velocity: 0.30

# 更精确的到达判定
position_tolerance: 0.02
heading_tolerance: 0.02

# 更大的减速区域
slow_radius: 0.80
```

### 自动启动规划

设置 `auto_start: true` 使规划器在启动时自动规划路径：

```yaml
auto_start: true
start_tag: 0
goal_tag: 474
```

### 通过 GUI 触发规划

保持 `auto_start: false`，使用 GUI 或服务调用：

```bash
rosrun tag_nav_planner tag_nav_gui.py
```

或在代码中调用服务：

```python
rospy.wait_for_service('/planner/plan_path')
plan_path = rospy.ServiceProxy('/planner/plan_path', PlanPath)
response = plan_path(start_tag=0, goal_tag=474)
```

### 自定义连通性

使用连通性编辑器可视化编辑：

```bash
rosrun tag_nav_planner tag_connectivity_editor.py
```

或手动修改 JSON 文件：

```json
{
  "connectivity": {
    "0": 31,   // 北、东北、东、东南、南（屏蔽西侧）
    "1": 63    // 北、东北、东、东南、南、西南（屏蔽西侧）
  }
}
```

## 规划器工作流程

1. **初始化**：加载标签地图和连通性配置
2. **路径规划**：
   - 在 `start_tag` 和 `goal_tag` 之间运行 A* 算法
   - 使用 8 方向连通性（`allow_diagonal: true`）
3. **路径跟随**：
   - 沿规划路径逐个标签移动
   - 使用定点跟随（stop-and-go）策略
   - 在每个标签处检查位置容差和航向容差
4. **速度控制**：
   - 接近目标时进入减速半径
   - 动态调整线速度和角速度

## 故障排查

### 路径规划失败

1. 确认 `start_tag` 和 `goal_tag` 存在于标签地图中
2. 检查连通性配置是否正确
3. 验证路径是否存在（没有隔离区域）
4. 查看日志中的路径规划错误信息

### 无法到达目标点

1. 调大 `position_tolerance` 和 `heading_tolerance`
2. 检查 `slow_radius` 是否过小
3. 确认机器人速度设置合理
4. 验证 `map -> base_footprint` TF 链正常

### 连通性错误

1. 检查掩码值是否在 0-255 范围内
2. 确认网格布局与标签地图一致
3. 使用编辑器可视化验证连通性

## 相关工具

### tag_nav_gui.py

规划器图形界面，提供：
- 可视化标签地图和连通性
- 设置起点和终点
- 触发路径规划
- 显示规划结果

### tag_connectivity_editor.py

连通性可视化编辑器，支持：
- 显示标签网格和连通性
- 交互式修改连通性掩码
- 导出/导入连通性配置

## 相关文档

- [主 README](../../../README.md)
- [定位配置](../../tag_nav_localization/config/CONFIG.md)