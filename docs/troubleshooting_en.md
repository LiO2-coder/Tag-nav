# Integration Troubleshooting

[中文](troubleshooting.md)

This document records the behaviors that are easiest to confuse while integrating Gazebo, the EKF, AprilTag localization, and TF. When debugging TF, first identify which node publishes each edge and which timestamp/data source it uses before investigating the AprilTag algorithm.

## TF ownership

The launch files `factory_apriltag_localization.launch`, `planner.launch`, and `factory_navigation.launch` should keep this ownership:

```text
robot_localization:  /agv/odom + /agv/imu/data -> odom -> base_footprint
AprilTag:            T_map_base * inverse(T_odom_base) -> map -> odom
Final chain:         map -> odom -> base_footprint
```

In `correction` mode the AprilTag node publishes `map -> odom`; in `localization` mode it publishes `map -> base_footprint` directly. Gazebo, the EKF, and the localization node must not publish the same dynamic TF edge. `factory_gazebo.launch` defaults `publish_odom_tf` to `true`, while the EKF launch entry sets it to `false`.

The earlier `TF_REPEATED_DATA` warning was caused by repeatedly broadcasting the same parent/child pair with an identical simulated timestamp. The current code requires strictly increasing `parent + child + stamp` values before publishing and clears the check when simulated time moves backward.

## Localization state when Tags are lost

Wheel odometry and the EKF should continue while no Tag is visible. In `correction` mode the node republishes the cached `map -> odom` correction with the current timestamp so the TF chain remains queryable, while the visual localization result is marked invalid.

Inspect the localization message:

```bash
rostopic echo /apriltag_localization/localization
```

The important fields are:

- `valid`: whether the current fused pose is valid;
- `tf_status`: TF publication state, such as `published`, `correction_held`, or `no_valid_pose`;
- `localization_age_sec`: age of the most recent valid visual localization.

`valid: false` means visual localization is currently invalid. It does not mean `/agv/odom` or `/agv/odometry/filtered` has stopped publishing.

## Manually moving the Gazebo model

The differential-drive plugin uses `<odometrySource>encoder</odometrySource>` instead of writing Gazebo world truth directly into `/agv/odom`. Manually teleporting the model can still produce a large visual correction, but it should not directly reset the EKF's `odom -> base_footprint` transform.

The EKF also disables the IMU's absolute world heading. The visual node updates `map -> odom` from new Tag observations. If a jump is observed, inspect timestamps and publishers for `/agv/odom`, `/agv/odometry/filtered`, and `/tf` separately.

## Image timestamps and fast rotation

The synchronizer's `wait_sec` is the delay used to wait for other camera frames; it is not subtracted from an image measurement timestamp. For an image stamped at `t`, query `T_odom_base(t)` at the same `t`.

Small heading errors during fast in-place rotation are usually caused by image acquisition, transport, and detection latency. They should not be interpreted as the synchronizer changing the measurement time.

## Downward fisheye camera and floor Tags

The downward camera uses the `equidistant` rectification branch before AprilTag detection and square IPPE PnP. OpenCV `SOLVEPNP_IPPE_SQUARE` requires a strict corner order, while the floor map defines the Tag normal as `+Z`.

The implementation solves and verifies positive depth using the OpenCV convention, then applies `diag(1, -1, -1)` to convert the PnP Tag coordinates to the floor-map convention.

![Camera configuration](../assets/camera_setup.png)

## Gazebo GUI warnings

The following message comes from Gazebo Classic's `JointControlWidget`:

```text
Node::Advertise(): Error advertising topic [/warehouse_agv/joint_cmd]
```

This is an internal Gazebo Transport topic, not a ROS topic. The warning can usually be ignored when `/agv/cmd_vel`, the camera images, `/agv/odom`, and `/tf` are healthy.

When the Gazebo GUI is not needed, disable it:

```bash
roslaunch tag_nav_bringup factory_apriltag_localization.launch gui:=false
```

On multi-NIC, VPN, or Docker setups, set the Gazebo network variables only when both sides need an explicitly reachable address:

```bash
export GAZEBO_IP=10.10.151.14
export IGN_IP=10.10.151.14
```

## Recommended diagnostic order

### 1. Confirm that inputs are continuous

```bash
rostopic hz /agv/odom
rostopic hz /agv/odometry/filtered
rostopic hz /agv/camera/bottom/image_raw
```

### 2. Confirm the TF chain and direction

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map odom
rosrun tf tf_echo map base_footprint
rosrun tf view_frames
```

### 3. Inspect AprilTag state

```bash
rostopic echo /apriltag_localization/localization
```

### 4. Confirm that there are no duplicate dynamic TF publishers

```bash
rostopic info /tf
rosnode info /agv_ekf_odom
rosnode info /apriltag_localization
```

## Related documents

- [Main README](../README_en.md)
- [Localization configuration](../src/tag_nav_localization/config/CONFIG_en.md)
- [Tag map configuration](../src/tag_nav_bringup/worlds/maps/CONFIG_en.md)
- [Planner configuration](../src/tag_nav_planner/config/CONFIG_en.md)
