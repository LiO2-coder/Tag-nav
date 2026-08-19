# Tag-Graph Planner Configuration

[中文](CONFIG.md)

This directory contains the tag-graph planner configuration. The node loads `planner.yaml` into its private namespace through `<rosparam>` and reads the Tag map and connectivity JSON files.

## planner.yaml

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `start_tag` | int | 0 | Start Tag ID for automatic planning |
| `goal_tag` | int | 474 | Goal Tag ID for automatic planning |
| `auto_start` | bool | false | Plan immediately at startup |
| `linear_velocity` | float | 0.30 | Maximum linear velocity (m/s) |
| `angular_velocity` | float | 0.50 | Maximum angular velocity (rad/s) |
| `position_tolerance` | float | 0.05 | Position tolerance (m) |
| `heading_tolerance` | float | 0.05 | Heading tolerance (rad) |
| `slow_radius` | float | 0.50 | Radius for beginning goal deceleration (m) |
| `allow_diagonal` | bool | true | Allow diagonal edges in 8-connected A* |
| `grid_spacing` | float | 1.0 | Tag-grid spacing (m) |
| `controller_rate` | float | 20.0 | Waypoint-following control rate (Hz) |
| `pose_timeout` | float | 0.5 | `map -> base_footprint` pose timeout (s) |
| `map_frame` | string | `map` | Map frame |
| `base_frame` | string | `base_footprint` | Robot base frame |
| `connectivity_json` | string | `package://...` | Connectivity JSON path |
| `tag_map_file` | string | `package://...` | Tag map path; the planner requires `map_type: 2d` |

Velocity parameters cap stop-and-go following. `position_tolerance` and `heading_tolerance` define waypoint arrival. `slow_radius` controls deceleration near a goal.

## connectivity.json

Every Tag uses an 8-bit mask, from 0 to 255, to represent directed edges to adjacent grid Tags:

| Bit | Direction | Value |
| --- | --- | --- |
| 0 | North (+Y) | 1 |
| 1 | North-east | 2 |
| 2 | East (+X) | 4 |
| 3 | South-east | 8 |
| 4 | South (-Y) | 16 |
| 5 | South-west | 32 |
| 6 | West (-X) | 64 |
| 7 | North-west | 128 |

```json
{
  "schema_version": 1,
  "connectivity": {
    "0": 28,
    "1": 124
  }
}
```

For example, `28 = 4 + 8 + 16` permits east, south-east, and south. Masks are directed. For a bidirectional edge, set the corresponding opposite bit on the neighbor too.

The default file primarily blocks directions that leave the grid: most interior Tags use `255`, and boundary Tags clear out-of-grid directions. `connectivity.json` does **not** fully encode the factory racks, cylinders, and pillars. Use the editor to restrict the relevant Tags when the graph must avoid those obstacles, and check the static occupancy map used by `map_server` as well.

## Examples

### Tune speed and tolerances

```yaml
linear_velocity: 0.20
angular_velocity: 0.30
position_tolerance: 0.02
heading_tolerance: 0.02
slow_radius: 0.80
```

### Plan automatically at startup

```yaml
auto_start: true
start_tag: 0
goal_tag: 474
```

### GUI and service

With `auto_start: false`, start the GUI:

```bash
rosrun tag_nav_planner tag_nav_gui.py
```

Or call the service:

```python
import rospy
from tag_nav_planner.srv import PlanPath

rospy.wait_for_service('/planner/plan_path')
plan_path = rospy.ServiceProxy('/planner/plan_path', PlanPath)
response = plan_path(start_tag=0, goal_tag=474)
```

Edit connectivity with:

```bash
rosrun tag_nav_planner tag_connectivity_editor.py
```

The editor updates the opposite edge on the neighboring Tag and saves the result to `connectivity.json`.

## Workflow

1. Load `tag_map_file` and `connectivity_json`.
2. Build an 8-connected graph from `2d` Tag poses using `grid_spacing`.
3. Run A* between the selected Tags. Straight edges cost `grid_spacing`; diagonal edges cost `sqrt(2) * grid_spacing`.
4. Follow the waypoints with the `map -> base_footprint` transform.
5. Slow near the goal using `slow_radius`, then apply position and heading tolerances.

## Troubleshooting

### Planning fails

Confirm the start and goal exist in the Tag map, mask values are in 0-255, and the map has `map_type: 2d`. Check logs for isolated nodes or edges pointing outside the grid.

### The robot cannot reach the goal

Check that `map -> base_footprint` is continuous, `pose_timeout` is not too small, velocities suit the robot dimensions, and the static map does not mark the goal area as occupied.

### Connectivity and obstacles disagree

Confirm that the Tag grid, `connectivity.json`, and `factory_map.pgm` use the same coordinate convention and scale. Gazebo obstacles are not automatically synchronized into the connectivity JSON.

## Tools

- `tag_nav_gui.py`: displays the Tag grid and planned path, selects start/goal Tags, and calls the planning service.
- `tag_connectivity_editor.py`: interactively edits 8-direction masks and imports/exports JSON.

## Related documents

- [Main README](../../../README_en.md)
- [Localization configuration](../../tag_nav_localization/config/CONFIG_en.md)
- [Tag map configuration](../../tag_nav_bringup/worlds/maps/CONFIG_en.md)
