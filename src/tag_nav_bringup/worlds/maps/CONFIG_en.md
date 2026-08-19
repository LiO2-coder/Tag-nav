# AprilTag Map Configuration

[中文](CONFIG.md)

This directory contains the factory AprilTag map. The localization node and the tag-graph planner both use it.

## Map file

Default file: [apriltagMap.json](apriltagMap.json)

```text
src/tag_nav_bringup/worlds/maps/apriltagMap.json
```

### JSON format

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

| Field | Type | Description |
| --- | --- | --- |
| `schema_version` | int | Format version for maps maintained by this project. The current value is `1`. The localization node and planner remain compatible with legacy/pytagmapper output that omits it. |
| `tag_side_lengths` | object | Tag side lengths in meters. `default` applies to Tags without an individual length. |
| `tag_locations` | object | Mapping from Tag ID to Tag pose. Keys must be convertible to integers. |
| `map_type` | string | Representation: `2d`, `2.5d`, or `3d`. |

### Tag pose formats

The shape of every `tag_locations` value depends on `map_type`:

| `map_type` | Per-Tag format | Supported by |
| --- | --- | --- |
| `2d` | `[x, y, yaw]` | Localization node and tag-graph planner |
| `2.5d` | `[x, y, yaw, z]` | Localization node only |
| `3d` | 4x4 homogeneous transform as nested row-major arrays | Localization node only |

`x`, `y`, and `z` are in meters; `yaw` is in radians. The third value in a `2d` pose is the Tag rotation around `+Z`, not its height. This is why every pose in the default factory map has a third value of `0.0`.

For the factory floor Tags, the map origin is on the factory-center floor, `+X` points right, `+Y` points forward, and both the normal and front side of a Tag point toward `+Z`. The localization node handles the conversion between this floor convention and the camera PnP convention.

## Default factory map

The default map is a 19 x 25 grid:

| Property | Value |
| --- | --- |
| X range | `[-9.0, 9.0]` m |
| Y range | `[-12.0, 12.0]` m |
| Tag spacing | `1.0` m |
| Tag count | 475, IDs `0` through `474` |
| Default Tag side length | `0.10` m |

Tag IDs increase by row:

```text
Row 0  (Y= 12.0): IDs 0-18
Row 1  (Y= 11.0): IDs 19-37
...
Row 24 (Y=-12.0): IDs 456-474
```

Every row has 19 Tags, with X increasing from `-9.0` to `9.0`.

## Creating a custom map

### Create it manually

For a `2d` map, write `[x, y, yaw]` for each Tag:

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

### Build with pytagmapper

`pytagmapper` is a Git submodule, not a ROS package, so `rosrun pytagmapper generate_map.py` is not available. Run its actual tool from the submodule:

```bash
cd src/3rd_party/pytagmapper
python3 pytagmapper_tools/build_map.py <input-dir> --mode 2d --output-dir <output-dir>
```

The command writes `map.json` to `<output-dir>`. In `2d` mode its Tag locations are already `[x, y, yaw]`. Copy or convert it to the project's `apriltagMap.json`, then ensure the root object includes the version field:

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

Keep `tag_side_lengths`, `tag_locations`, and `map_type` unchanged. See the submodule README for the required input directory format and tool limitations.

### Limitation of Gazebo-world generation

The repository currently has **no** script that extracts Tag positions from `factory.world` and writes `apriltagMap.json`. `generate_apriltag_floor.py` creates Gazebo floor resources, while `generate_factory_map.py` creates an occupancy grid; neither produces a Tag pose map.

## Using a map

### Localization node

The `map.uri` field in `gazebo_cameras.json` points to the map:

```json
{
  "map": {
    "uri": "package://tag_nav_bringup/worlds/maps/apriltagMap.json"
  }
}
```

Override it at launch time:

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/custom_map.json
```

The localization node supports `2d`, `2.5d`, and `3d`. If `fusion.mode` is not `auto`, it must match the map's `map_type`.

### Tag-graph planner

The `tag_map_file` field in `planner.yaml` points to the map:

```yaml
tag_map_file: "package://tag_nav_bringup/worlds/maps/apriltagMap.json"
```

The tag-graph planner supports only `2d` maps because it requires `[x, y, yaw]` poses on a regular grid.

## Validation

From the repository root, validate the default map:

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

Visualize Tags and connectivity through the planner GUI:

```bash
roslaunch tag_nav_bringup planner.launch
rosrun tag_nav_planner tag_nav_gui.py
```

After starting localization, inspect detections with:

```bash
rostopic echo /apriltag_localization/camera_best_tags
```

## Related configuration

| File | Relationship |
| --- | --- |
| `gazebo_cameras.json` | References the map through `map.uri` |
| `planner.yaml` | References the map through `tag_map_file` |
| `connectivity.json` | Tag IDs must match the map |
| `factory.world` | Physical Tag positions should match the map |

## Related documents

- [Main README](../../../../README_en.md)
- [Localization configuration](../../../tag_nav_localization/config/CONFIG_en.md)
- [Planner configuration](../../../tag_nav_planner/config/CONFIG_en.md)
