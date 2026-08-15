#ifndef LASER_TAG_NAV_LOCALIZATION_CORE_TYPES_H
#define LASER_TAG_NAV_LOCALIZATION_CORE_TYPES_H

#include <array>
#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <laser_tag_nav_localization/quality_math.h>

namespace laser_tag_nav_localization
{
namespace core
{

using Transform = cv::Matx44d;

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct CameraModel
{
  std::string name;
  std::string topic;
  std::string transport = "raw";
  std::string data_format;
  std::string frame_id;
  bool enabled = true;
  cv::Matx33d K = cv::Matx33d::eye();
  cv::Mat D;
  std::string distortion_model = "plumb_bob";
  Transform base_to_camera = Transform::eye();
  double confidence_multiplier = 1.0;
};

struct TagMapEntry
{
  Transform map_to_tag = Transform::eye();
  double size = 0.10;
};

struct DetectorConfig
{
  std::string tag_family = "tag36h11";
  int threads = 2;
  double decimate = 1.0;
  double blur = 0.0;
  bool refine_edges = true;
  int max_hamming_dist = 0;
};

struct SynchronizationConfig
{
  int queue_size = 10;
  double slop_sec = 0.05;
  double wait_sec = 0.02;
  double min_batch_interval_sec = 0.0;
};

struct RuntimeConfig
{
  double process_rate_hz = 15.0;
  double localization_timeout_sec = 0.50;
};

struct QualityConfig
{
  std::string mask = "1111";
  std::array<double, 4> requested_metric_exponents{{1.0, 1.0, 1.0, 1.0}};
  std::array<double, 4> metric_exponents{{0.25, 0.25, 0.25, 0.25}};
  double area_reference_px = 1600.0;
  double margin_reference = 100.0;
  double sharpness_reference = 128.0;
  double distortion_scale = 0.20;
  double min_quality = 0.01;
};

struct FusionConfig
{
  std::string mode = "auto";
  int min_contributing_cameras = 1;
  std::map<std::string, double> camera_confidence_multipliers;
  bool outlier_gate_enabled = false;
  double outlier_position_threshold = 0.30;
  double outlier_yaw_threshold = 0.35;
  double outlier_orientation_threshold = 0.35;
  double min_position_stddev = 0.02;
  double min_yaw_stddev = 0.035;
};

struct TemporalFilterConfig
{
  bool enabled = false;
  double position_time_constant_sec = 0.25;
  double orientation_time_constant_sec = 0.25;
  double max_dt_sec = 1.0;
};

struct ValidationConfig
{
  double min_tag_area_px = 0.0;
  double max_tag_range_m = 0.0;
  bool reject_unmapped_tags = true;
  double stale_frame_timeout_sec = 0.20;
};

struct OutputConfig
{
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_footprint";
  std::string tf_mode = "localization";
  bool publish_tf = false;
  double tf_lookup_timeout_sec = 0.05;
  double correction_tf_tolerance_sec = 0.25;
  double correction_tf_publish_rate_hz = 30.0;
  bool debug_images = false;
  double invalid_variance = 1e6;
};

struct LocalizationConfig
{
  std::string tag_map_file;
  std::string map_type;
  std::vector<CameraModel> cameras;
  std::map<int, TagMapEntry> tag_map;
  DetectorConfig detector;
  SynchronizationConfig synchronization;
  RuntimeConfig runtime;
  QualityConfig quality;
  FusionConfig fusion;
  TemporalFilterConfig temporal_filter;
  ValidationConfig validation;
  OutputConfig output;

  double effectiveBatchInterval() const;
};

struct TagCandidate
{
  int tag_id = -1;
  std::array<cv::Point2d, 4> corners{};
  Transform camera_to_tag = Transform::eye();
  Transform map_to_base = Transform::eye();
  QualityScores scores;
  int hamming = 0;
  double range_m = 0.0;
  double weighted_quality = 0.0;
};

struct CameraObservation
{
  std::size_t camera_index = 0;
  bool has_candidate = false;
  TagCandidate candidate;
  bool rejected = false;
  std::string status;
};

struct FusedPose
{
  Transform map_to_base = Transform::eye();
  std::array<double, 36> covariance{};
};

struct FusionResult
{
  std::vector<CameraObservation> observations;
  double quality_sum = 0.0;
  bool valid = false;
  FusedPose pose;
  std::vector<std::string> camera_names;
  std::vector<int> tag_ids;
  std::vector<double> qualities;
  std::vector<double> weights;
  std::size_t contributing_camera_count = 0;
};

using UriResolver = std::function<std::string(const std::string&)>;

}  // namespace core
}  // namespace laser_tag_nav_localization

#endif
