# tag_nav_localization

`apriltag_localization_node` is a topic-driven ROS 1 node. It synchronizes the
latest frames from the configured cameras, detects mapped AprilTags, and
publishes a quality-weighted pose for `base_footprint`.

## Architecture

The node keeps the established ROS 1 interface, but its implementation is
layered so localization math can be tested without a ROS master:

* `tag_nav_localization_core` contains configuration and tag-map parsing,
  rigid-transform math, AprilTag/PnP recognition, quality scoring, and fusion.
  Its headers have no ROS, TF, image-transport, cv_bridge, or generated-message
  dependencies.
* `tag_nav_localization_ros` adapts private ROS parameters, TF lookup and
  broadcast, and the existing output messages to that core library.
* `apriltag_localization_node` owns subscriptions, image queues, scheduling,
  cv_bridge conversion, debug publishing, and object assembly only.

This split is intentionally internal: executable name, launch files, JSON v1
and legacy configuration support, parameter precedence, private topics, message
types, and TF modes remain unchanged.

## Supported AprilTag families

The detector links against apriltag 0.10.0 and supports its full built-in family
set: `tag16h5`, `tag25h9`, `tag36h10`, `tag36h11`, `tagCircle21h7`,
`tagCircle49h12`, `tagCustom48h12`, `tagStandard41h12`, and `tagStandard52h13`.
`detector.tag_family` must name one of these; any other value fails at startup.
Custom (self-generated) families are not configurable and require recompiling the
node.

The node requires these private parameters:

* `cameras_json`: JSON string loaded by launch `textfile`. The version-1 object
  contains `map.uri`, `quality`, `detector`, `synchronization`, `runtime`,
  `fusion`, `validation`, `output`, and the `cameras` array. The legacy JSON
  array is still accepted when `tag_map_file` is supplied separately.
* `tag_map_file`: optional explicit override for `map.uri`. The JSON path may
  be absolute or use `package://package_name/relative/path`.

The launch file uploads the JSON file to the parameter server using `textfile`:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch tag_nav_localization apriltag_localization.launch
```

The factory demonstration also starts the existing Gazebo world:

```bash
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false
```

Outputs are `~camera_best_tags`, `~localization`, and the valid-only
`~pose`. `~pose` and `~localization.pose` always represent the tag-fused
`map -> base_footprint` pose.

The `output` section controls TF ownership independently from `fusion.mode`:

* `tf_mode: localization` publishes `map_frame -> base_frame` directly from
  the fused pose.
* `tf_mode: correction` queries the existing `odom_frame -> base_frame` at
  the image timestamp and publishes `map_frame -> odom_frame`, computed as
  `T_map_odom = T_map_base * inverse(T_odom_base)`.

For the factory AprilTag launch, the Gazebo diff-drive TF broadcaster is
disabled while its raw `/agv/odom` topic remains enabled; the
`robot_localization` EKF becomes the sole owner of
`odom -> base_footprint`; it fuses encoder-based `/agv/odom` with
`/agv/imu/data` and publishes `/agv/odometry/filtered`. The simulated
diff-drive plugin must use `odometrySource=encoder`, not `world`: the latter
publishes Gazebo ground truth and would make the odometry frame jump when a
model is manually moved. The local EKF also uses IMU angular velocity rather
than an absolute world-referenced yaw. If the odom lookup is unavailable, the
tag pose remains valid but this node skips the TF broadcast for that batch.
`~localization` reports `tf_published`, `tf_status`, `tf_parent_frame`, and
`tf_child_frame`.
Set `output.publish_tf` to `false` to disable either TF mode.

In correction mode, `output.correction_tf_tolerance_sec` (default `0.25`) and
`output.correction_tf_publish_rate_hz` (default `30`) keep the most recent
`map -> odom` correction valid slightly ahead of the current ROS time. This is
needed by control nodes that transform a freshly generated `map`-frame plan to
the `odom` frame while AprilTag image processing is in progress. The transform
value is still computed from the image timestamp and matching odometry sample.

The quality mask is a four-character `ADMS` string from left to right:
projected area, quadrilateral distortion, decision margin, and corner
sharpness. Disabled terms are reported but excluded from the normalized
geometric mean. The default mask `1111` preserves the four-term quality rule.

Timing controls are independent:

* `runtime.process_rate_hz` limits the detector scheduling rate.
* `synchronization.slop_sec` is the maximum timestamp difference allowed in a
  synchronized batch; `wait_sec` gives slower cameras time to contribute.
* `synchronization.min_batch_interval_sec` prevents duplicate batches and is
  combined with the rate as `max(1/process_rate_hz, min_batch_interval_sec)`.
* `validation.stale_frame_timeout_sec` rejects old input frames.
* `runtime.localization_timeout_sec` publishes `valid=false` after the last
  valid fusion has expired. It is a localization validity timeout, not a hard
  interruption of the AprilTag library call.

`fusion.mode` can be `auto`, `2d`, `2.5d`, or `3d`. `auto` follows the map's
`map_type`; an explicit mode must match it. The existing factory map is 2D and
uses `[x, y, yaw]`. A 2.5D map uses `[x, y, yaw, z]`; a 3D map uses nested
row-major 4x4 tag transforms and fuses a sign-aligned quaternion.
