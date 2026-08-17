#include <stdexcept>
#include <string>

#include <ros/package.h>

#include <tag_nav_localization/core/configuration.h>
#include <tag_nav_localization/ros/ros_config_loader.h>

namespace tag_nav_localization
{
namespace ros_adapter
{
namespace
{

std::string resolvePackageUri(const std::string& uri)
{
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) != 0)
    return uri;
  const std::string remainder = uri.substr(prefix.size());
  const std::size_t slash = remainder.find('/');
  if (slash == std::string::npos || slash == 0 || slash + 1 >= remainder.size())
    throw std::runtime_error("package URI must be package://<package>/<relative-path>: " + uri);
  const std::string package_name = remainder.substr(0, slash);
  const std::string package_path = ros::package::getPath(package_name);
  if (package_path.empty())
    throw std::runtime_error("unable to resolve package URI: " + uri);
  return package_path + "/" + remainder.substr(slash + 1);
}

}  // namespace

RosConfigLoader::RosConfigLoader(const ros::NodeHandle& private_node_handle)
  : private_node_handle_(private_node_handle)
{
}

core::LocalizationConfig RosConfigLoader::load() const
{
  std::string cameras_json;
  if (!private_node_handle_.getParam("cameras_json", cameras_json))
    throw std::runtime_error("required private parameter ~cameras_json is missing");
  std::string tag_map_override;
  private_node_handle_.getParam("tag_map_file", tag_map_override);
  core::LocalizationConfig config = core::parseConfiguration(
      cameras_json, tag_map_override, resolvePackageUri);

  private_node_handle_.param<std::string>("tag_family", config.detector.tag_family,
                                           config.detector.tag_family);
  private_node_handle_.param("tag_threads", config.detector.threads, config.detector.threads);
  private_node_handle_.param("tag_decimate", config.detector.decimate, config.detector.decimate);
  private_node_handle_.param("tag_blur", config.detector.blur, config.detector.blur);
  private_node_handle_.param("tag_refine_edges", config.detector.refine_edges,
                             config.detector.refine_edges);
  private_node_handle_.param("max_hamming_dist", config.detector.max_hamming_dist,
                             config.detector.max_hamming_dist);
  private_node_handle_.param("sync_slop", config.synchronization.slop_sec,
                             config.synchronization.slop_sec);
  private_node_handle_.param("sync_wait", config.synchronization.wait_sec,
                             config.synchronization.wait_sec);
  private_node_handle_.param("min_batch_interval", config.synchronization.min_batch_interval_sec,
                             config.synchronization.min_batch_interval_sec);
  private_node_handle_.param("process_rate", config.runtime.process_rate_hz,
                             config.runtime.process_rate_hz);
  private_node_handle_.param("localization_timeout", config.runtime.localization_timeout_sec,
                             config.runtime.localization_timeout_sec);
  private_node_handle_.param("stale_frame_timeout", config.validation.stale_frame_timeout_sec,
                             config.validation.stale_frame_timeout_sec);
  private_node_handle_.param("queue_size", config.synchronization.queue_size,
                             config.synchronization.queue_size);
  private_node_handle_.param("map_frame", config.output.map_frame, config.output.map_frame);
  private_node_handle_.param("odom_frame", config.output.odom_frame, config.output.odom_frame);
  private_node_handle_.param("base_frame", config.output.base_frame, config.output.base_frame);
  private_node_handle_.param("tf_mode", config.output.tf_mode, config.output.tf_mode);
  private_node_handle_.param("publish_tf", config.output.publish_tf, config.output.publish_tf);
  private_node_handle_.param("tf_lookup_timeout", config.output.tf_lookup_timeout_sec,
                             config.output.tf_lookup_timeout_sec);
  private_node_handle_.param("correction_tf_tolerance", config.output.correction_tf_tolerance_sec,
                             config.output.correction_tf_tolerance_sec);
  private_node_handle_.param("correction_tf_publish_rate", config.output.correction_tf_publish_rate_hz,
                             config.output.correction_tf_publish_rate_hz);
  private_node_handle_.param("debug_images", config.output.debug_images, config.output.debug_images);
  private_node_handle_.param("area_reference_px", config.quality.area_reference_px,
                             config.quality.area_reference_px);
  private_node_handle_.param("margin_reference", config.quality.margin_reference,
                             config.quality.margin_reference);
  private_node_handle_.param("sharpness_reference", config.quality.sharpness_reference,
                             config.quality.sharpness_reference);
  private_node_handle_.param("distortion_scale", config.quality.distortion_scale,
                             config.quality.distortion_scale);
  private_node_handle_.param("min_quality", config.quality.min_quality, config.quality.min_quality);
  private_node_handle_.param("quality_mask", config.quality.mask, config.quality.mask);
  private_node_handle_.param("fusion_mode", config.fusion.mode, config.fusion.mode);
  private_node_handle_.param("min_contributing_cameras", config.fusion.min_contributing_cameras,
                             config.fusion.min_contributing_cameras);
  private_node_handle_.param("min_tag_area_px", config.validation.min_tag_area_px,
                             config.validation.min_tag_area_px);
  private_node_handle_.param("max_tag_range_m", config.validation.max_tag_range_m,
                             config.validation.max_tag_range_m);
  private_node_handle_.param("min_position_stddev", config.fusion.min_position_stddev,
                             config.fusion.min_position_stddev);
  private_node_handle_.param("min_yaw_stddev", config.fusion.min_yaw_stddev,
                             config.fusion.min_yaw_stddev);
  private_node_handle_.param("temporal_filter_enabled", config.temporal_filter.enabled,
                             config.temporal_filter.enabled);
  private_node_handle_.param("temporal_filter_position_time_constant",
                             config.temporal_filter.position_time_constant_sec,
                             config.temporal_filter.position_time_constant_sec);
  private_node_handle_.param("temporal_filter_orientation_time_constant",
                             config.temporal_filter.orientation_time_constant_sec,
                             config.temporal_filter.orientation_time_constant_sec);
  private_node_handle_.param("invalid_variance", config.output.invalid_variance,
                             config.output.invalid_variance);

  core::validateConfiguration(config);
  core::loadTagMap(config);
  core::validateConfiguration(config);
  return config;
}

}  // namespace ros_adapter
}  // namespace tag_nav_localization
