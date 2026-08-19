# AprilTag Localization Configuration

[中文](CONFIG.md)

This directory contains configuration for the AprilTag localization node.

## gazebo_cameras.json

`gazebo_cameras.json` is the main multi-camera localization configuration. It controls Tag detection, synchronization, fusion, validation, TF output, and camera calibration.

### Structure overview

```text
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

### Map

| Field | Type | Description |
| --- | --- | --- |
| `uri` | string | AprilTag map path. `package://` URIs are supported. |

The map schema, `2d`/`2.5d`/`3d` pose formats, and generation procedure are maintained in the [AprilTag map configuration](../../tag_nav_bringup/worlds/maps/CONFIG_en.md). The localization node supports all three map types. With `fusion.mode: auto`, it adopts the map `map_type`; otherwise they must match.

### Quality scoring

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `mask` | string | `"1111"` | Four quality flags for area, distortion, margin, and sharpness. |
| `metric_exponents` | object | - | Exponent weights for quality metrics. |
| `area_reference_px` | float | 1600.0 | Reference Tag area in pixels. |
| `distortion_scale` | float | 0.20 | Distortion scale. |
| `margin_reference` | float | 100.0 | Reference detector decision margin. |
| `sharpness_reference` | float | 128.0 | Reference corner sharpness. |
| `min_quality` | float | 0.01 | Minimum accepted quality. |

### Detector

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `tag_family` | string | `"tag36h11"` | Tag family, such as `tag16h5`, `tag25h9`, or `tag36h11`. |
| `threads` | int | 2 | Detector worker threads. |
| `decimate` | float | 1.0 | Image decimation; `1.0` means none. |
| `blur` | float | 0.0 | Gaussian blur radius. |
| `refine_edges` | bool | true | Refine Tag edges. |
| `max_hamming_dist` | int | 0 | Maximum Hamming distance for detection error filtering. |

### Synchronization and runtime

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `synchronization.queue_size` | int | 10 | Image queue size. |
| `synchronization.slop_sec` | float | 0.05 | Camera synchronization window in seconds. |
| `synchronization.wait_sec` | float | 0.02 | Time to wait for other camera frames; it does not alter image timestamps. |
| `synchronization.min_batch_interval_sec` | float | 0.0667 | Minimum interval between processing batches, about 15 Hz. |
| `runtime.process_rate_hz` | float | 15.0 | Processing rate in Hz. |
| `runtime.localization_timeout_sec` | float | 0.50 | Time after which the latest visual localization is stale. |

### Fusion

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `mode` | string | `"auto"` | `auto`, `2d`, `2.5d`, or `3d`. |
| `min_contributing_cameras` | int | 1 | Minimum number of contributing cameras. |
| `camera_confidence_multipliers` | object | `{}` | Per-camera confidence multipliers. |
| `outlier_gate.enabled` | bool | false | Enable outlier rejection. |
| `outlier_gate.max_position_residual_m` | float | 0.30 | Maximum position residual in meters. |
| `outlier_gate.max_yaw_residual_rad` | float | 0.35 | Maximum yaw residual in radians. |
| `outlier_gate.max_orientation_residual_rad` | float | 0.35 | Maximum orientation residual in radians. |
| `min_position_stddev_m` | float | 0.02 | Minimum position standard deviation. |
| `min_yaw_stddev_rad` | float | 0.035 | Minimum yaw standard deviation. |

### Temporal filter and validation

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `temporal_filter.enabled` | bool | true | Enable temporal filtering. |
| `temporal_filter.position_time_constant_sec` | float | 0.25 | Position filter time constant. |
| `temporal_filter.orientation_time_constant_sec` | float | 0.25 | Orientation filter time constant. |
| `temporal_filter.max_dt_sec` | float | 1.0 | Largest accepted time step. |
| `validation.min_tag_area_px` | float | 0.0 | Minimum detected Tag area. |
| `validation.max_tag_range_m` | float | 0.0 | Maximum Tag range; `0` disables the limit. |
| `validation.reject_unmapped_tags` | bool | true | Reject Tags absent from the map. |
| `validation.stale_frame_timeout_sec` | float | 0.20 | Frame staleness timeout. |

### Output and TF

| Field | Type | Default | Description |
| --- | --- | --- | --- |
| `map_frame` | string | `"map"` | Map frame. |
| `odom_frame` | string | `"odom"` | Odometry frame. |
| `base_frame` | string | `"base_footprint"` | Robot base frame. |
| `tf_mode` | string | `"correction"` | `correction` publishes `map -> odom`; `localization` publishes `map -> base_frame`. |
| `publish_tf` | bool | true | Publish the selected TF transform. |
| `tf_lookup_timeout_sec` | float | 0.05 | Timeout for TF lookups. |
| `correction_tf_tolerance_sec` | float | 0.25 | Future tolerance applied when publishing a correction TF. |
| `correction_tf_publish_rate_hz` | float | 10.0 | Correction TF republish rate. |
| `debug_images` | bool | false | Publish debug images. |
| `invalid_variance` | float | 1000000.0 | Covariance value for an invalid pose. |

### Cameras

| Field | Type | Description |
| --- | --- | --- |
| `name` | string | Camera name, such as `front`, `rear`, `left`, `right`, or `bottom`. |
| `enabled` | bool | Subscribe to this camera. |
| `image_topic` | string | Image topic. |
| `transport` | string | Image transport, such as `raw` or `compressed`. |
| `data_format` | string | Image encoding, such as `rgb8` or `bgr8`. |
| `frame_id` | string | Camera frame. |
| `intrinsics.K` | array[9] | Camera matrix. |
| `intrinsics.D` | array | Distortion coefficients. `equidistant` and `fisheye` use four values; `plumb_bob` uses the camera calibration result. |
| `intrinsics.distortion_model` | string | `plumb_bob`, `rational_polynomial`, `equidistant`, `fisheye`, or `none`. |
| `base_to_camera.translation` | array[3] | Translation from the base frame to the camera. |
| `base_to_camera.rotation_rpy` | array[3] | Base-to-camera roll, pitch, and yaw. |

## Examples

### Add a calibrated camera

```json
{
  "name": "custom_camera",
  "enabled": true,
  "image_topic": "/camera/image_raw",
  "transport": "raw",
  "data_format": "bgr8",
  "frame_id": "camera_custom_optical_frame",
  "intrinsics": {
    "K": [184.75, 0.0, 320.0, 0.0, 184.75, 240.0, 0.0, 0.0, 1.0],
    "D": [0.0, 0.0, 0.0, 0.0, 0.0],
    "distortion_model": "plumb_bob"
  },
  "base_to_camera": {
    "translation": [0.0, 0.0, 0.36],
    "rotation_rpy": [0.0, 0.0, 0.0]
  }
}
```

### Enable debug images

```json
{
  "output": {
    "debug_images": true
  }
}
```

### Override the Tag map

```bash
roslaunch tag_nav_localization apriltag_localization.launch \
  tag_map_file:=/absolute/path/to/custom_map.json
```

## Troubleshooting

### No Tags are detected

1. Confirm the camera is enabled.
2. Check the image topic, for example `rostopic echo /agv/camera/bottom/image_raw`.
3. Verify the camera calibration and distortion model.
4. Confirm that `tag_family` matches the physical Tags.

### Localization is unstable

1. Tune `quality.min_quality`.
2. Enable the temporal filter.
3. Increase `fusion.min_contributing_cameras` when sufficient cameras are available.
4. Enable `fusion.outlier_gate.enabled`.

### TF is not published

1. Check that `map_frame`, `odom_frame`, and `base_frame` match the rest of the system.
2. Check `tf_lookup_timeout_sec`.
3. Verify camera extrinsics and inspect `/apriltag_localization/localization` fields `valid` and `tf_status`.

## Related documents

- [Main README](../../../README_en.md)
- [EKF configuration](ekf_odom.yaml)
- [Tag map configuration](../../tag_nav_bringup/worlds/maps/CONFIG_en.md)
- [Planner configuration](../../tag_nav_planner/config/CONFIG_en.md)
