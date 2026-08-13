#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <jsoncpp/json/json.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <apriltag/apriltag.h>
#include <apriltag/common/zarray.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

#include <laser_tag_nav_localization/CameraBestTag.h>
#include <laser_tag_nav_localization/CameraBestTagArray.h>
#include <laser_tag_nav_localization/FusedAprilTagLocalization.h>
#include <laser_tag_nav_localization/quality_math.h>

namespace laser_tag_nav_localization
{
namespace
{

using Matrix4 = cv::Matx44d;

struct CameraConfig
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
  Matrix4 base_to_camera = Matrix4::eye();
  double confidence_multiplier = 1.0;

  std::deque<sensor_msgs::ImageConstPtr> queue;
  cv::Mat map1;
  cv::Mat map2;
  cv::Size map_size;
  image_transport::Publisher debug_publisher;
};

struct TagMapEntry
{
  Matrix4 map_to_tag = Matrix4::eye();
  double size = 0.10;
};

struct Candidate
{
  int tag_id = -1;
  std::array<cv::Point2d, 4> corners{};
  Matrix4 camera_to_tag = Matrix4::eye();
  Matrix4 map_to_base = Matrix4::eye();
  QualityScores scores;
  int hamming = 0;
  double range_m = 0.0;
  double weighted_quality = 0.0;
};

struct CameraResult
{
  size_t camera_index = 0;
  sensor_msgs::ImageConstPtr frame;
  std::shared_ptr<Candidate> candidate;
  bool rejected = false;
  std::string status;
};

struct FamilyHandle
{
  apriltag_family_t* family = nullptr;
  void (*destroy)(apriltag_family_t*) = nullptr;
};

double getNumber(const Json::Value& value, const std::string& name)
{
  if (!value.isNumeric())
    throw std::runtime_error("JSON field '" + name + "' must be numeric");
  return value.asDouble();
}

std::vector<double> getNumberArray(const Json::Value& value, const std::string& name)
{
  if (!value.isArray())
    throw std::runtime_error("JSON field '" + name + "' must be an array");
  std::vector<double> output;
  output.reserve(value.size());
  for (Json::ArrayIndex i = 0; i < value.size(); ++i)
    output.push_back(getNumber(value[i], name));
  return output;
}

const Json::Value& jsonSection(const Json::Value& root, const std::string& name)
{
  static const Json::Value empty(Json::objectValue);
  if (!root.isObject() || !root.isMember(name) || !root[name].isObject())
    return empty;
  return root[name];
}

std::string jsonString(const Json::Value& section, const std::string& name,
                      const std::string& fallback)
{
  if (!section.isMember(name))
    return fallback;
  if (!section[name].isString())
    throw std::runtime_error("JSON field '" + name + "' must be a string");
  return section[name].asString();
}

double jsonDouble(const Json::Value& section, const std::string& name, double fallback)
{
  if (!section.isMember(name))
    return fallback;
  return getNumber(section[name], name);
}

int jsonInt(const Json::Value& section, const std::string& name, int fallback)
{
  if (!section.isMember(name))
    return fallback;
  if (!section[name].isInt() && !section[name].isUInt())
    throw std::runtime_error("JSON field '" + name + "' must be an integer");
  return section[name].asInt();
}

bool jsonBool(const Json::Value& section, const std::string& name, bool fallback)
{
  if (!section.isMember(name))
    return fallback;
  if (!section[name].isBool())
    throw std::runtime_error("JSON field '" + name + "' must be boolean");
  return section[name].asBool();
}

std::string resolveUri(const std::string& uri)
{
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) != 0)
    return uri;
  const std::string remainder = uri.substr(prefix.size());
  const size_t slash = remainder.find('/');
  if (slash == std::string::npos || slash == 0 || slash + 1 >= remainder.size())
    throw std::runtime_error("package URI must be package://<package>/<relative-path>: " + uri);
  const std::string package_name = remainder.substr(0, slash);
  const std::string relative_path = remainder.substr(slash + 1);
  const std::string package_path = ros::package::getPath(package_name);
  if (package_path.empty())
    throw std::runtime_error("unable to resolve package URI: " + uri);
  return package_path + "/" + relative_path;
}

cv::Matx33d parseCameraMatrix(const Json::Value& value)
{
  if (!value.isArray())
    throw std::runtime_error("intrinsics.K must be an array");
  std::vector<double> values;
  if (value.size() == 9)
  {
    values = getNumberArray(value, "intrinsics.K");
  }
  else if (value.size() == 3)
  {
    for (Json::ArrayIndex row = 0; row < 3; ++row)
    {
      const std::vector<double> row_values = getNumberArray(value[row], "intrinsics.K");
      if (row_values.size() != 3)
        throw std::runtime_error("intrinsics.K must be 3x3 or flat length 9");
      values.insert(values.end(), row_values.begin(), row_values.end());
    }
  }
  else
  {
    throw std::runtime_error("intrinsics.K must be 3x3 or flat length 9");
  }
  return cv::Matx33d(values[0], values[1], values[2],
                    values[3], values[4], values[5],
                    values[6], values[7], values[8]);
}

Matrix4 matrixFromTranslationRpy(const std::vector<double>& translation,
                                  const std::vector<double>& rpy)
{
  if (translation.size() != 3 || rpy.size() != 3)
    throw std::runtime_error("base_to_camera translation and rotation_rpy must have length 3");

  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  const cv::Matx33d rotation(
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr);

  Matrix4 result = Matrix4::eye();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = rotation(row, col);
  result(0, 3) = translation[0];
  result(1, 3) = translation[1];
  result(2, 3) = translation[2];
  return result;
}

Matrix4 inverseRigid(const Matrix4& transform)
{
  Matrix4 inverse = Matrix4::eye();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      inverse(row, col) = transform(col, row);
  for (int row = 0; row < 3; ++row)
  {
    inverse(row, 3) = 0.0;
    for (int col = 0; col < 3; ++col)
      inverse(row, 3) -= inverse(row, col) * transform(col, 3);
  }
  return inverse;
}

Matrix4 mapTagTransform(double x, double y, double yaw)
{
  Matrix4 result = Matrix4::eye();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  result(0, 0) = c;
  result(0, 1) = -s;
  result(1, 0) = s;
  result(1, 1) = c;
  result(0, 3) = x;
  result(1, 3) = y;
  return result;
}

Matrix4 matrixFromJson(const Json::Value& value, const std::string& name)
{
  if (!value.isArray() || value.size() != 4)
    throw std::runtime_error(name + " must be a 4x4 array");
  Matrix4 result = Matrix4::eye();
  for (Json::ArrayIndex row = 0; row < 4; ++row)
  {
    const std::vector<double> values = getNumberArray(value[row], name);
    if (values.size() != 4)
      throw std::runtime_error(name + " must be a 4x4 array");
    for (size_t col = 0; col < 4; ++col)
      result(static_cast<int>(row), static_cast<int>(col)) = values[col];
  }
  if (std::abs(result(3, 0)) > 1e-6 || std::abs(result(3, 1)) > 1e-6 ||
      std::abs(result(3, 2)) > 1e-6 || std::abs(result(3, 3) - 1.0) > 1e-6)
    throw std::runtime_error(name + " must be a homogeneous transform");
  return result;
}

double yawFromTransform(const Matrix4& transform)
{
  return std::atan2(transform(1, 0), transform(0, 0));
}

void poseFromTransform(const Matrix4& transform, geometry_msgs::Pose& pose)
{
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  tf::Matrix3x3 rotation(
      transform(0, 0), transform(0, 1), transform(0, 2),
      transform(1, 0), transform(1, 1), transform(1, 2),
      transform(2, 0), transform(2, 1), transform(2, 2));
  tf::Quaternion quaternion;
  rotation.getRotation(quaternion);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
}

Matrix4 matrixFromTf(const tf::Transform& transform)
{
  Matrix4 result = Matrix4::eye();
  const tf::Matrix3x3 basis = transform.getBasis();
  const tf::Vector3 origin = transform.getOrigin();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = basis[row][col];
  result(0, 3) = origin.x();
  result(1, 3) = origin.y();
  result(2, 3) = origin.z();
  return result;
}

tf::Transform tfFromMatrix(const Matrix4& transform)
{
  tf::Matrix3x3 basis(
      transform(0, 0), transform(0, 1), transform(0, 2),
      transform(1, 0), transform(1, 1), transform(1, 2),
      transform(2, 0), transform(2, 1), transform(2, 2));
  tf::Quaternion rotation;
  basis.getRotation(rotation);
  return tf::Transform(rotation,
                       tf::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));
}

tf::Quaternion quaternionFromTransform(const Matrix4& transform)
{
  tf::Matrix3x3 rotation(
      transform(0, 0), transform(0, 1), transform(0, 2),
      transform(1, 0), transform(1, 1), transform(1, 2),
      transform(2, 0), transform(2, 1), transform(2, 2));
  tf::Quaternion quaternion;
  rotation.getRotation(quaternion);
  quaternion.normalize();
  return quaternion;
}

tf::Quaternion weightedQuaternion(const std::vector<tf::Quaternion>& quaternions,
                                  const std::vector<double>& weights)
{
  if (quaternions.empty())
    return tf::Quaternion(0.0, 0.0, 0.0, 1.0);
  tf::Quaternion reference = quaternions.front();
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;
  for (size_t i = 0; i < quaternions.size() && i < weights.size(); ++i)
  {
    tf::Quaternion q = quaternions[i];
    if (reference.dot(q) < 0.0)
      q = tf::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
    x += weights[i] * q.x();
    y += weights[i] * q.y();
    z += weights[i] * q.z();
    w += weights[i] * q.w();
  }
  tf::Quaternion result(x, y, z, w);
  result.normalize();
  return result;
}

double quaternionAngularDistance(const tf::Quaternion& a, const tf::Quaternion& b)
{
  const double dot = std::max(-1.0, std::min(1.0, std::abs(a.dot(b))));
  return 2.0 * std::acos(dot);
}

FamilyHandle createFamily(const std::string& family_name)
{
  FamilyHandle handle;
#define APRILTAG_FAMILY(name) \
  if (family_name == #name) { handle.family = name##_create(); handle.destroy = name##_destroy; }
  APRILTAG_FAMILY(tag16h5)
  else APRILTAG_FAMILY(tag25h9)
  else APRILTAG_FAMILY(tag36h10)
  else APRILTAG_FAMILY(tag36h11)
  else APRILTAG_FAMILY(tagCircle21h7)
  else APRILTAG_FAMILY(tagCircle49h12)
  else APRILTAG_FAMILY(tagCustom48h12)
  else APRILTAG_FAMILY(tagStandard41h12)
  else APRILTAG_FAMILY(tagStandard52h13)
#undef APRILTAG_FAMILY
  if (!handle.family)
    throw std::runtime_error("unsupported or unavailable AprilTag family: " + family_name);
  return handle;
}

std::string frameIdFor(const CameraConfig& camera, const sensor_msgs::ImageConstPtr& image)
{
  if (!image->header.frame_id.empty())
    return image->header.frame_id;
  if (!camera.frame_id.empty())
    return camera.frame_id;
  return camera.name + "_optical_frame";
}

}  // namespace

class AprilTagLocalizationNode
{
public:
  AprilTagLocalizationNode()
    : nh_(), pnh_("~"), image_transport_(nh_)
  {
    loadParameters();
    loadCameras();
    loadMap();
    initializeDetector();

    camera_results_publisher_ = pnh_.advertise<CameraBestTagArray>("camera_best_tags", 1);
    localization_publisher_ = pnh_.advertise<FusedAprilTagLocalization>("localization", 1);
    pose_publisher_ = pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);

    for (size_t i = 0; i < cameras_.size(); ++i)
    {
      if (!cameras_[i].enabled)
        continue;
      if (debug_images_)
        cameras_[i].debug_publisher = image_transport_.advertise("debug/" + cameras_[i].name, 1);
      image_subscribers_.push_back(image_transport_.subscribe(
          cameras_[i].topic, queue_size_,
          boost::bind(&AprilTagLocalizationNode::imageCallback, this, _1, i),
          ros::VoidPtr(), image_transport::TransportHints(cameras_[i].transport)));
    }

    sync_timer_ = nh_.createTimer(ros::Duration(1.0 / process_rate_),
                                  &AprilTagLocalizationNode::syncTimer, this);
    ROS_INFO("AprilTag localization ready with %zu configured cameras and %zu map tags",
             cameras_.size(), tag_map_.size());
  }

  ~AprilTagLocalizationNode()
  {
    if (detector_)
      apriltag_detector_destroy(detector_);
    if (family_.family && family_.destroy)
      family_.destroy(family_.family);
  }

private:
  void loadParameters()
  {
    if (!pnh_.getParam("cameras_json", cameras_json_))
      throw std::runtime_error("required private parameter ~cameras_json is missing");

    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    std::string errors;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(cameras_json_.data(), cameras_json_.data() + cameras_json_.size(),
                       &config_root_, &errors))
      throw std::runtime_error("invalid ~cameras_json: " + errors);
    const bool legacy = config_root_.isArray();
    if (!legacy && config_root_.isMember("schema_version") &&
        (!config_root_["schema_version"].isInt() || config_root_["schema_version"].asInt() != 1))
      throw std::runtime_error("unsupported cameras JSON schema_version; expected 1");
    const Json::Value& detector = jsonSection(config_root_, "detector");
    const Json::Value& synchronization = jsonSection(config_root_, "synchronization");
    const Json::Value& runtime = jsonSection(config_root_, "runtime");
    const Json::Value& quality = jsonSection(config_root_, "quality");
    const Json::Value& fusion = jsonSection(config_root_, "fusion");
    const Json::Value& validation = jsonSection(config_root_, "validation");
    const Json::Value& output = jsonSection(config_root_, "output");

    tag_family_ = jsonString(detector, "tag_family", "tag36h11");
    tag_threads_ = jsonInt(detector, "threads", 2);
    tag_decimate_ = jsonDouble(detector, "decimate", 1.0);
    tag_blur_ = jsonDouble(detector, "blur", 0.0);
    tag_refine_edges_ = jsonBool(detector, "refine_edges", true) ? 1 : 0;
    max_hamming_dist_ = jsonInt(detector, "max_hamming_dist", 0);
    sync_slop_ = jsonDouble(synchronization, "slop_sec", 0.05);
    sync_wait_ = jsonDouble(synchronization, "wait_sec", 0.02);
    queue_size_ = jsonInt(synchronization, "queue_size", 10);
    process_rate_ = jsonDouble(runtime, "process_rate_hz", 15.0);
    localization_timeout_ = jsonDouble(runtime, "localization_timeout_sec", 0.50);
    min_batch_interval_ = jsonDouble(synchronization, "min_batch_interval_sec", 0.0);
    stale_frame_timeout_ = jsonDouble(validation, "stale_frame_timeout_sec", 0.20);
    area_reference_px_ = jsonDouble(quality, "area_reference_px", 1600.0);
    margin_reference_ = jsonDouble(quality, "margin_reference", 100.0);
    sharpness_reference_ = jsonDouble(quality, "sharpness_reference", 128.0);
    distortion_scale_ = jsonDouble(quality, "distortion_scale", 0.20);
    min_quality_ = jsonDouble(quality, "min_quality", 0.01);
    quality_mask_ = jsonString(quality, "mask", "1111");
    min_tag_area_px_ = jsonDouble(validation, "min_tag_area_px", 0.0);
    max_tag_range_m_ = jsonDouble(validation, "max_tag_range_m", 0.0);
    reject_unmapped_tags_ = jsonBool(validation, "reject_unmapped_tags", true);
    localization_mode_ = jsonString(fusion, "mode", "auto");
    min_contributing_cameras_ = jsonInt(fusion, "min_contributing_cameras", 1);
    min_position_stddev_ = jsonDouble(fusion, "min_position_stddev_m", 0.02);
    min_yaw_stddev_ = jsonDouble(fusion, "min_yaw_stddev_rad", 0.035);
    parseFusionConfiguration(fusion);
    map_frame_ = jsonString(output, "map_frame", "map");
    odom_frame_ = jsonString(output, "odom_frame", "odom");
    base_frame_ = jsonString(output, "base_frame", "base_footprint");
    tf_mode_ = jsonString(output, "tf_mode", "localization");
    publish_tf_ = jsonBool(output, "publish_tf", false);
    tf_lookup_timeout_ = jsonDouble(output, "tf_lookup_timeout_sec", 0.05);
    debug_images_ = jsonBool(output, "debug_images", false);
    invalid_variance_ = jsonDouble(output, "invalid_variance", 1e6);

    Json::Value map_config = jsonSection(config_root_, "map");
    tag_map_file_ = jsonString(map_config, "uri", "");
    std::string override_map;
    if (pnh_.getParam("tag_map_file", override_map) && !override_map.empty())
      tag_map_file_ = override_map;
    if (tag_map_file_.empty())
      throw std::runtime_error("map.uri or explicit ~tag_map_file is required");
    if (!tag_map_file_.empty())
      tag_map_file_ = resolveUri(tag_map_file_);

    // Explicit ROS parameters remain supported as higher-priority overrides.
    pnh_.param<std::string>("tag_family", tag_family_, tag_family_);
    pnh_.param("tag_threads", tag_threads_, tag_threads_);
    pnh_.param("tag_decimate", tag_decimate_, tag_decimate_);
    pnh_.param("tag_blur", tag_blur_, tag_blur_);
    pnh_.param("tag_refine_edges", tag_refine_edges_, tag_refine_edges_);
    pnh_.param("max_hamming_dist", max_hamming_dist_, max_hamming_dist_);
    pnh_.param("sync_slop", sync_slop_, sync_slop_);
    pnh_.param("sync_wait", sync_wait_, sync_wait_);
    pnh_.param("min_batch_interval", min_batch_interval_, min_batch_interval_);
    pnh_.param("process_rate", process_rate_, process_rate_);
    pnh_.param("localization_timeout", localization_timeout_, localization_timeout_);
    pnh_.param("stale_frame_timeout", stale_frame_timeout_, stale_frame_timeout_);
    pnh_.param("queue_size", queue_size_, queue_size_);
    pnh_.param("map_frame", map_frame_, map_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    pnh_.param("base_frame", base_frame_, base_frame_);
    pnh_.param("tf_mode", tf_mode_, tf_mode_);
    pnh_.param("publish_tf", publish_tf_, publish_tf_);
    pnh_.param("tf_lookup_timeout", tf_lookup_timeout_, tf_lookup_timeout_);
    pnh_.param("debug_images", debug_images_, debug_images_);
    pnh_.param("area_reference_px", area_reference_px_, area_reference_px_);
    pnh_.param("margin_reference", margin_reference_, margin_reference_);
    pnh_.param("sharpness_reference", sharpness_reference_, sharpness_reference_);
    pnh_.param("distortion_scale", distortion_scale_, distortion_scale_);
    pnh_.param("min_quality", min_quality_, min_quality_);
    pnh_.param("quality_mask", quality_mask_, quality_mask_);
    pnh_.param("fusion_mode", localization_mode_, localization_mode_);
    pnh_.param("min_contributing_cameras", min_contributing_cameras_, min_contributing_cameras_);
    pnh_.param("min_tag_area_px", min_tag_area_px_, min_tag_area_px_);
    pnh_.param("max_tag_range_m", max_tag_range_m_, max_tag_range_m_);
    pnh_.param("min_position_stddev", min_position_stddev_, min_position_stddev_);
    pnh_.param("min_yaw_stddev", min_yaw_stddev_, min_yaw_stddev_);
    pnh_.param("invalid_variance", invalid_variance_, invalid_variance_);

    parseQualityConfiguration(quality);
    if (localization_mode_ != "auto" && localization_mode_ != "2d" &&
        localization_mode_ != "2.5d" && localization_mode_ != "3d")
      throw std::runtime_error("fusion_mode must be auto, 2d, 2.5d or 3d");

    if (tf_mode_ != "localization" && tf_mode_ != "correction")
      throw std::runtime_error("tf_mode must be localization or correction");
    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty())
      throw std::runtime_error("map_frame, odom_frame and base_frame cannot be empty");
    if (tf_mode_ == "correction" &&
        (map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_))
      throw std::runtime_error("map, odom and base frames must be distinct in correction mode");
    if (tf_lookup_timeout_ < 0.0)
      throw std::runtime_error("tf_lookup_timeout must be non-negative");

    if (process_rate_ <= 0.0 || sync_slop_ < 0.0 || sync_wait_ < 0.0 || queue_size_ <= 0 ||
        localization_timeout_ < 0.0 || stale_frame_timeout_ < 0.0 || min_batch_interval_ < 0.0)
      throw std::runtime_error("invalid timing or queue configuration");
    if (min_tag_area_px_ < 0.0 || max_tag_range_m_ < 0.0)
      throw std::runtime_error("validation thresholds cannot be negative");
    if (min_contributing_cameras_ < 1)
      throw std::runtime_error("min_contributing_cameras must be at least one");
    const double rate_interval = 1.0 / process_rate_;
    effective_batch_interval_ = std::max(rate_interval, min_batch_interval_);
  }

  void parseQualityConfiguration(const Json::Value& quality)
  {
    if (!validQualityMask(quality_mask_))
      throw std::runtime_error("quality.mask must be a non-zero four-bit ADMS string");
    metric_exponents_ = {{1.0, 1.0, 1.0, 1.0}};
    if (quality.isMember("metric_exponents"))
    {
      const Json::Value& exponents = quality["metric_exponents"];
      if (!exponents.isObject())
        throw std::runtime_error("quality.metric_exponents must be an object");
      metric_exponents_[0] = jsonDouble(exponents, "area", 1.0);
      metric_exponents_[1] = jsonDouble(exponents, "distortion", 1.0);
      metric_exponents_[2] = jsonDouble(exponents, "margin", 1.0);
      metric_exponents_[3] = jsonDouble(exponents, "sharpness", 1.0);
    }
    try
    {
      metric_exponents_ = normalizedQualityExponents(quality_mask_, metric_exponents_);
    }
    catch (const std::invalid_argument& error)
    {
      throw std::runtime_error(error.what());
    }
  }

  void parseFusionConfiguration(const Json::Value& fusion)
  {
    if (localization_mode_ != "auto" && localization_mode_ != "2d" &&
        localization_mode_ != "2.5d" && localization_mode_ != "3d")
      throw std::runtime_error("fusion.mode must be auto, 2d, 2.5d or 3d");
    const Json::Value& multipliers = fusion["camera_confidence_multipliers"];
    if (!multipliers.isNull())
    {
      if (!multipliers.isObject())
        throw std::runtime_error("fusion.camera_confidence_multipliers must be an object");
      for (const std::string& name : multipliers.getMemberNames())
      {
        const double value = getNumber(multipliers[name], name);
        if (value < 0.0)
          throw std::runtime_error("camera confidence multipliers cannot be negative");
        camera_confidence_multipliers_[name] = value;
      }
    }
    const Json::Value& gate = fusion["outlier_gate"];
    outlier_gate_enabled_ = jsonBool(gate, "enabled", false);
    outlier_position_threshold_ = jsonDouble(gate, "max_position_residual_m", 0.30);
    outlier_yaw_threshold_ = jsonDouble(gate, "max_yaw_residual_rad", 0.35);
    outlier_orientation_threshold_ = jsonDouble(gate, "max_orientation_residual_rad", 0.35);
    if (min_contributing_cameras_ < 1 || outlier_position_threshold_ < 0.0 ||
        outlier_yaw_threshold_ < 0.0 || outlier_orientation_threshold_ < 0.0)
      throw std::runtime_error("invalid fusion configuration");
  }

  void loadCameras()
  {
    const Json::Value entries = config_root_.isArray() ? config_root_ : config_root_["cameras"];
    if (!entries.isArray() || entries.empty())
      throw std::runtime_error("~cameras_json must contain a non-empty cameras array");

    std::map<std::string, bool> names;
    for (Json::ArrayIndex i = 0; i < entries.size(); ++i)
    {
      const Json::Value& value = entries[i];
      if (!value.isObject() || !value.isMember("name") || !value.isMember("image_topic"))
        throw std::runtime_error("each camera needs name and image_topic");
      CameraConfig camera;
      camera.name = value["name"].asString();
      camera.topic = value["image_topic"].asString();
      camera.enabled = !value.isMember("enabled") || value["enabled"].asBool();
      camera.transport = value.get("transport", "raw").asString();
      camera.data_format = value.get("data_format", "").asString();
      camera.frame_id = value.get("frame_id", "").asString();
      camera.confidence_multiplier = value.get("confidence_multiplier", 1.0).asDouble();
      const std::map<std::string, double>::const_iterator multiplier =
          camera_confidence_multipliers_.find(camera.name);
      if (multiplier != camera_confidence_multipliers_.end())
        camera.confidence_multiplier = multiplier->second;
      if (camera.confidence_multiplier < 0.0)
        throw std::runtime_error("camera " + camera.name + " has negative confidence multiplier");
      if (camera.name.empty() || camera.topic.empty() || names[camera.name])
        throw std::runtime_error("camera names must be unique and non-empty");
      names[camera.name] = true;

      if (!value.isMember("intrinsics") || !value["intrinsics"].isObject())
        throw std::runtime_error("camera " + camera.name + " needs intrinsics");
      const Json::Value& intrinsics = value["intrinsics"];
      if (!intrinsics.isMember("K"))
        throw std::runtime_error("camera " + camera.name + " needs intrinsics.K");
      camera.K = parseCameraMatrix(intrinsics["K"]);
      if (camera.K(0, 0) <= 0.0 || camera.K(1, 1) <= 0.0)
        throw std::runtime_error("camera " + camera.name + " has invalid focal length");
      const std::vector<double> distortion = intrinsics.isMember("D")
          ? getNumberArray(intrinsics["D"], "intrinsics.D") : std::vector<double>();
      if (!distortion.empty())
        camera.D = cv::Mat(distortion).reshape(1, 1).clone();
      camera.distortion_model = intrinsics.get("distortion_model", "plumb_bob").asString();
      std::string model = camera.distortion_model;
      std::transform(model.begin(), model.end(), model.begin(), ::tolower);
      camera.distortion_model = model;
      if ((model == "equidistant" || model == "fisheye") &&
          !camera.D.empty() && camera.D.total() != 4)
        throw std::runtime_error("camera " + camera.name + " fisheye D must have length 4");
      if (model != "plumb_bob" && model != "rational_polynomial" &&
          model != "equidistant" && model != "fisheye" && model != "none")
        throw std::runtime_error("unsupported distortion_model for camera " + camera.name);

      if (!value.isMember("base_to_camera"))
        throw std::runtime_error("camera " + camera.name + " needs base_to_camera");
      const Json::Value& extrinsic = value["base_to_camera"];
      camera.base_to_camera = matrixFromTranslationRpy(
          getNumberArray(extrinsic["translation"], "base_to_camera.translation"),
          getNumberArray(extrinsic["rotation_rpy"], "base_to_camera.rotation_rpy"));
      cameras_.push_back(camera);
    }
  }

  void loadMap()
  {
    std::ifstream input(tag_map_file_.c_str());
    if (!input)
      throw std::runtime_error("unable to open tag map: " + tag_map_file_);
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string errors;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::ostringstream contents;
    contents << input.rdbuf();
    const std::string text = contents.str();
    if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors))
      throw std::runtime_error("invalid tag map JSON: " + errors);
    map_type_ = root.get("map_type", "").asString();
    if (map_type_ != "2d" && map_type_ != "2.5d" && map_type_ != "3d")
      throw std::runtime_error("tag map map_type must be 2d, 2.5d or 3d");
    if (localization_mode_ == "auto")
      localization_mode_ = map_type_;
    else if (localization_mode_ != map_type_)
      throw std::runtime_error("fusion.mode does not match tag map map_type");
    if (!root.isMember("tag_locations") || !root["tag_locations"].isObject())
      throw std::runtime_error("tag map needs tag_locations");
    const Json::Value lengths = root.get("tag_side_lengths", Json::Value(Json::objectValue));
    const double default_size = lengths.get("default", 0.10).asDouble();
    for (const std::string& key : root["tag_locations"].getMemberNames())
    {
      const int id = std::stoi(key);
      TagMapEntry entry;
      if (map_type_ == "2d")
      {
        const std::vector<double> location = getNumberArray(root["tag_locations"][key], key);
        if (location.size() != 3)
          throw std::runtime_error("tag location " + key + " must be [x,y,yaw]");
        entry.map_to_tag = mapTagTransform(location[0], location[1], location[2]);
      }
      else if (map_type_ == "2.5d")
      {
        const std::vector<double> location = getNumberArray(root["tag_locations"][key], key);
        if (location.size() != 4)
          throw std::runtime_error("tag location " + key + " must be [x,y,yaw,z]");
        entry.map_to_tag = mapTagTransform(location[0], location[1], location[2]);
        entry.map_to_tag(2, 3) = location[3];
      }
      else
      {
        entry.map_to_tag = matrixFromJson(root["tag_locations"][key], "tag location " + key);
      }
      entry.size = lengths.isMember(key) ? lengths[key].asDouble() : default_size;
      if (entry.size <= 0.0)
        throw std::runtime_error("tag " + key + " has non-positive size");
      tag_map_[id] = entry;
    }
    if (tag_map_.empty())
      throw std::runtime_error("tag map contains no locations");
  }

  void initializeDetector()
  {
    family_ = createFamily(tag_family_);
    detector_ = apriltag_detector_create();
    if (!detector_)
      throw std::runtime_error("apriltag_detector_create failed");
    apriltag_detector_add_family_bits(detector_, family_.family, max_hamming_dist_);
    detector_->nthreads = std::max(1, tag_threads_);
    detector_->quad_decimate = static_cast<float>(std::max(0.1, tag_decimate_));
    detector_->quad_sigma = static_cast<float>(tag_blur_);
    detector_->refine_edges = tag_refine_edges_;
  }

  void ensureRectification(CameraConfig& camera, const cv::Size& size)
  {
    if (camera.map_size == size)
      return;
    camera.map1.release();
    camera.map2.release();
    const bool has_distortion = !camera.D.empty() && camera.D.total() > 0;
    if (has_distortion || camera.distortion_model == "equidistant" ||
        camera.distortion_model == "fisheye")
    {
      const cv::Mat K = cv::Mat(camera.K).clone();
      const cv::Mat new_K = cv::Mat(camera.K).clone();
      if (camera.distortion_model == "equidistant" || camera.distortion_model == "fisheye")
      {
        cv::Mat D = camera.D;
        if (D.empty())
          D = cv::Mat::zeros(1, 4, CV_64F);
        cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F),
                                             new_K, size, CV_32FC1, camera.map1, camera.map2);
      }
      else
      {
        cv::initUndistortRectifyMap(K, camera.D, cv::Mat::eye(3, 3, CV_64F),
                                    new_K, size, CV_32FC1, camera.map1, camera.map2);
      }
    }
    camera.map_size = size;
  }

  cv::Mat rectify(const sensor_msgs::ImageConstPtr& image, CameraConfig& camera)
  {
    cv_bridge::CvImagePtr converted = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    cv::Mat gray = converted->image;
    ensureRectification(camera, gray.size());
    if (!camera.map1.empty())
    {
      cv::Mat rectified;
      cv::remap(gray, rectified, camera.map1, camera.map2, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar(0));
      return rectified;
    }
    return gray;
  }

  double cornerSharpness(const cv::Mat& gray, const std::array<cv::Point2d, 4>& corners,
                         cv::Mat& gradient) const
  {
    if (gradient.empty())
    {
      cv::Mat gx, gy;
      cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
      cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
      cv::magnitude(gx, gy, gradient);
    }
    double total = 0.0;
    int samples = 0;
    constexpr int radius = 4;
    for (const cv::Point2d& corner : corners)
    {
      const int x = static_cast<int>(std::round(corner.x));
      const int y = static_cast<int>(std::round(corner.y));
      const cv::Rect image_rect(0, 0, gradient.cols, gradient.rows);
      const cv::Rect roi = cv::Rect(x - radius, y - radius, 2 * radius + 1, 2 * radius + 1) & image_rect;
      if (roi.empty())
        continue;
      total += cv::mean(gradient(roi))[0] * roi.area();
      samples += roi.area();
    }
    return samples > 0 ? total / samples : 0.0;
  }

  bool makeCandidate(const cv::Mat& gray, const cv::Mat& gradient,
                     apriltag_detection_t* detection, const CameraConfig& camera,
                     Candidate& candidate) const
  {
    const auto map_it = tag_map_.find(detection->id);
    if (map_it == tag_map_.end() || detection->hamming > max_hamming_dist_)
      return false;

    for (size_t i = 0; i < 4; ++i)
      candidate.corners[i] = cv::Point2d(detection->p[i][0], detection->p[i][1]);
    candidate.tag_id = detection->id;
    candidate.hamming = detection->hamming;

    // Keep the exact IPPE_SQUARE object-point order required by OpenCV. AprilTag
    // corners are counter-clockwise in image coordinates (+Y down), so this
    // PnP tag frame has its normal opposite to the map's +Z-up ground-tag frame.
    std::vector<cv::Point3d> object_points{
        cv::Point3d(-map_it->second.size / 2.0,  map_it->second.size / 2.0, 0.0),
        cv::Point3d( map_it->second.size / 2.0,  map_it->second.size / 2.0, 0.0),
        cv::Point3d( map_it->second.size / 2.0, -map_it->second.size / 2.0, 0.0),
        cv::Point3d(-map_it->second.size / 2.0, -map_it->second.size / 2.0, 0.0)};
    std::vector<cv::Point2d> image_points(candidate.corners.begin(), candidate.corners.end());
    cv::Mat rvec, tvec;
    bool solved = cv::solvePnP(object_points, image_points, cv::Mat(camera.K),
                               cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
    if (!solved)
      solved = cv::solvePnP(object_points, image_points, cv::Mat(camera.K),
                            cv::Mat(), rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!solved)
      return false;

    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    candidate.camera_to_tag = Matrix4::eye();
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        candidate.camera_to_tag(row, col) = rotation.at<double>(row, col);
    candidate.camera_to_tag(0, 3) = tvec.at<double>(0);
    candidate.camera_to_tag(1, 3) = tvec.at<double>(1);
    candidate.camera_to_tag(2, 3) = tvec.at<double>(2);
    candidate.range_m = cv::norm(cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1),
                                           tvec.at<double>(2)));
    if (max_tag_range_m_ > 0.0 && candidate.range_m > max_tag_range_m_)
      return false;

    for (const cv::Point3d& object_point : object_points)
    {
      const double z = candidate.camera_to_tag(2, 0) * object_point.x +
          candidate.camera_to_tag(2, 1) * object_point.y + candidate.camera_to_tag(2, 3);
      if (z <= 1e-6)
        return false;
    }

    // map_to_tag uses +Z as the upward floor normal. Convert from the IPPE
    // tag frame to that map tag frame only after positive-depth validation.
    Matrix4 pnp_tag_to_map_tag = Matrix4::eye();
    pnp_tag_to_map_tag(1, 1) = -1.0;
    pnp_tag_to_map_tag(2, 2) = -1.0;
    candidate.camera_to_tag = candidate.camera_to_tag * pnp_tag_to_map_tag;

    candidate.scores.area_px = polygonArea(candidate.corners);
    if (min_tag_area_px_ > 0.0 && candidate.scores.area_px < min_tag_area_px_)
      return false;
    candidate.scores.area = clamp01(candidate.scores.area_px / area_reference_px_);
    candidate.scores.distortion_error = quadrilateralDistortionError(candidate.corners);
    candidate.scores.distortion = std::exp(-candidate.scores.distortion_error /
                                           std::max(1e-6, distortion_scale_));
    candidate.scores.decision_margin = detection->decision_margin;
    candidate.scores.margin = clamp01(detection->decision_margin / margin_reference_);
    cv::Mat gradient_copy = gradient;
    candidate.scores.corner_gradient = cornerSharpness(gray, candidate.corners, gradient_copy);
    candidate.scores.sharpness = clamp01(candidate.scores.corner_gradient / sharpness_reference_);
    candidate.scores.quality = geometricQuality(
        {{candidate.scores.area, candidate.scores.distortion,
          candidate.scores.margin, candidate.scores.sharpness}}, metric_exponents_);
    candidate.weighted_quality = candidate.scores.quality * camera.confidence_multiplier;
    if (candidate.weighted_quality < min_quality_)
      return false;

    candidate.map_to_base = map_it->second.map_to_tag * inverseRigid(candidate.camera_to_tag) *
                           inverseRigid(camera.base_to_camera);
    return true;
  }

  std::shared_ptr<Candidate> detectBest(const cv::Mat& gray, CameraConfig& camera)
  {
    cv::Mat gradient;
    image_u8_t image = {gray.cols, gray.rows, static_cast<int32_t>(gray.step[0]), gray.data};
    zarray_t* detections = apriltag_detector_detect(detector_, &image);
    std::shared_ptr<Candidate> best;
    for (int i = 0; i < zarray_size(detections); ++i)
    {
      apriltag_detection_t* detection = nullptr;
      zarray_get(detections, i, &detection);
      Candidate candidate;
      if (makeCandidate(gray, gradient, detection, camera, candidate) &&
          (!best || candidate.scores.quality > best->scores.quality))
        best.reset(new Candidate(candidate));
    }
    apriltag_detections_destroy(detections);
    return best;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image, size_t camera_index)
  {
    CameraConfig& camera = cameras_[camera_index];
    camera.queue.push_back(image);
    while (camera.queue.size() > static_cast<size_t>(std::max(1, queue_size_)))
      camera.queue.pop_front();
  }

  sensor_msgs::ImageConstPtr nearestFrame(const CameraConfig& camera, const ros::Time& target) const
  {
    sensor_msgs::ImageConstPtr best;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const sensor_msgs::ImageConstPtr& frame : camera.queue)
    {
      const ros::Time stamp = frame->header.stamp.isZero() ? target : frame->header.stamp;
      const double delta = std::abs((stamp - target).toSec());
      if (delta < best_delta)
      {
        best_delta = delta;
        best = frame;
      }
    }
    return best_delta <= sync_slop_ ? best : sensor_msgs::ImageConstPtr();
  }

  bool latestEligibleAnchor(const ros::Time& cutoff, ros::Time& anchor) const
  {
    bool have_anchor = false;
    for (const CameraConfig& camera : cameras_)
    {
      if (!camera.enabled)
        continue;
      for (auto frame_it = camera.queue.rbegin(); frame_it != camera.queue.rend(); ++frame_it)
      {
        const ros::Time stamp = (*frame_it)->header.stamp;
        if (stamp.isZero() || stamp > cutoff)
          continue;
        if (!have_anchor || stamp > anchor)
        {
          anchor = stamp;
          have_anchor = true;
        }
        break;
      }
    }
    return have_anchor;
  }

  void syncTimer(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    // The wait is an arrival delay, not an offset applied to the measurement
    // timestamp. Processing an image at t with odometry from t-wait produces
    // a false map->odom correction whenever the base is moving.
    const ros::Time cutoff = now - ros::Duration(sync_wait_);
    ros::Time target;
    if (!latestEligibleAnchor(cutoff, target))
    {
      publishTimeoutState(now);
      return;
    }
    if (!last_processed_stamp_.isZero() && target <= last_processed_stamp_)
    {
      publishTimeoutState(now);
      return;
    }
    if (!last_batch_time_.isZero() && (target - last_batch_time_).toSec() < effective_batch_interval_)
    {
      publishTimeoutState(now);
      return;
    }

    const ros::WallTime processing_start = ros::WallTime::now();
    std::vector<CameraResult> results;
    for (size_t i = 0; i < cameras_.size(); ++i)
    {
      if (!cameras_[i].enabled)
        continue;
      sensor_msgs::ImageConstPtr frame = nearestFrame(cameras_[i], target);
      if (!frame)
        continue;
      if (stale_frame_timeout_ > 0.0 && !frame->header.stamp.isZero() &&
          (now - frame->header.stamp).toSec() > stale_frame_timeout_)
      {
        CameraResult stale_result;
        stale_result.camera_index = i;
        stale_result.frame = frame;
        stale_result.status = "stale_frame";
        results.push_back(stale_result);
        continue;
      }
      CameraResult result;
      result.camera_index = i;
      result.frame = frame;
      try
      {
        const cv::Mat gray = rectify(frame, cameras_[i]);
        result.candidate = detectBest(gray, cameras_[i]);
        result.status = result.candidate ? "valid" : "no_mapped_tag";
        publishDebug(cameras_[i], gray, result.candidate, frame->header);
      }
      catch (const cv_bridge::Exception& error)
      {
        result.status = "image_conversion_error";
        ROS_WARN_THROTTLE(2.0, "unable to convert camera %s image: %s",
                          cameras_[i].name.c_str(), error.what());
      }
      catch (const cv::Exception& error)
      {
        result.status = "image_processing_error";
        ROS_WARN_THROTTLE(2.0, "image processing failed for camera %s: %s",
                          cameras_[i].name.c_str(), error.what());
      }
      results.push_back(result);
    }
    last_processed_stamp_ = target;
    last_batch_time_ = target;
    last_processing_time_sec_ = (ros::WallTime::now() - processing_start).toSec();
    publishResults(target, results);
  }

  void publishTimeoutState(const ros::Time& now)
  {
    if (last_timeout_publish_time_.isZero() ||
        (now - last_timeout_publish_time_).toSec() >= effective_batch_interval_)
    {
      last_timeout_publish_time_ = now;
      if (last_valid_stamp_.isZero() || localization_timeout_ <= 0.0 ||
          (now - last_valid_stamp_).toSec() >= localization_timeout_)
      {
        last_processing_time_sec_ = 0.0;
        publishResults(now, std::vector<CameraResult>());
      }
      else
      {
        // Keep the last map->odom correction temporally valid while the
        // visual localization status is still within its timeout window.
        // This does not change the correction value or report a new visual
        // observation; it only lets odom->base continue to compose in TF.
        publishHeldCorrection(nullptr);
      }
    }
  }

  void publishDebug(CameraConfig& camera, const cv::Mat& gray,
                    const std::shared_ptr<Candidate>& candidate,
                    const std_msgs::Header& header)
  {
    if (!debug_images_ || camera.debug_publisher.getNumSubscribers() == 0)
      return;
    cv::Mat debug;
    cv::cvtColor(gray, debug, cv::COLOR_GRAY2BGR);
    if (candidate)
    {
      for (size_t i = 0; i < 4; ++i)
        cv::line(debug, candidate->corners[i], candidate->corners[(i + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      std::ostringstream label;
      label << "id=" << candidate->tag_id << " q=" << candidate->scores.quality;
      cv::putText(debug, label.str(), candidate->corners[0], cv::FONT_HERSHEY_SIMPLEX,
                  0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
    else
    {
      cv::putText(debug, "no mapped tag", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                  0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    cv_bridge::CvImage output(header, sensor_msgs::image_encodings::BGR8, debug);
    camera.debug_publisher.publish(output.toImageMsg());
  }

  void setCameraMessage(const CameraResult& result, double weight,
                        CameraBestTag& message) const
  {
    const CameraConfig& camera = cameras_[result.camera_index];
    message.camera_name = camera.name;
    message.header.stamp = result.frame ? result.frame->header.stamp : ros::Time::now();
    message.header.frame_id = result.frame ? frameIdFor(camera, result.frame) : camera.frame_id;
    message.valid = static_cast<bool>(result.candidate) && !result.rejected;
    message.status = result.rejected ? "outlier" :
        (result.candidate ? "valid" : (result.status.empty() ? "no_tag" : result.status));
    message.tag_id = result.candidate ? result.candidate->tag_id : -1;
    message.weight = weight;
    message.quality_mask = quality_mask_;
    if (!result.candidate)
      return;
    const Candidate& candidate = *result.candidate;
    message.camera_tag_pose.header = message.header;
    poseFromTransform(candidate.camera_to_tag, message.camera_tag_pose.pose.pose);
    message.area_px = candidate.scores.area_px;
    message.area_score = candidate.scores.area;
    message.distortion_error = candidate.scores.distortion_error;
    message.distortion_score = candidate.scores.distortion;
    message.decision_margin = candidate.scores.decision_margin;
    message.margin_score = candidate.scores.margin;
    message.corner_gradient = candidate.scores.corner_gradient;
    message.sharpness_score = candidate.scores.sharpness;
    message.quality = candidate.scores.quality;
    message.weighted_quality = candidate.weighted_quality;
    message.range_m = candidate.range_m;
    for (size_t i = 0; i < 4; ++i)
    {
      message.corners[2 * i] = candidate.corners[i].x;
      message.corners[2 * i + 1] = candidate.corners[i].y;
    }
  }

  void applyOutlierGate(std::vector<CameraResult>& results) const
  {
    if (!outlier_gate_enabled_)
      return;
    std::vector<size_t> indices;
    std::vector<double> weights;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    std::vector<double> yaws;
    std::vector<tf::Quaternion> quaternions;
    for (size_t i = 0; i < results.size(); ++i)
    {
      if (!results[i].candidate || results[i].rejected)
        continue;
      indices.push_back(i);
      weights.push_back(results[i].candidate->weighted_quality);
      xs.push_back(results[i].candidate->map_to_base(0, 3));
      ys.push_back(results[i].candidate->map_to_base(1, 3));
      zs.push_back(results[i].candidate->map_to_base(2, 3));
      yaws.push_back(yawFromTransform(results[i].candidate->map_to_base));
      quaternions.push_back(quaternionFromTransform(results[i].candidate->map_to_base));
    }
    if (indices.size() < 2)
      return;
    const double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (weight_sum <= 1e-12)
      return;
    for (double& weight : weights)
      weight /= weight_sum;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    for (size_t i = 0; i < weights.size(); ++i)
    {
      x += weights[i] * xs[i];
      y += weights[i] * ys[i];
      z += weights[i] * zs[i];
    }
    const double yaw = weightedYaw(yaws, weights);
    const tf::Quaternion quaternion = weightedQuaternion(quaternions, weights);
    for (size_t i = 0; i < indices.size(); ++i)
    {
      const double dz = localization_mode_ == "2d" ? 0.0 : zs[i] - z;
      const double position_error = std::sqrt((xs[i] - x) * (xs[i] - x) +
                                              (ys[i] - y) * (ys[i] - y) + dz * dz);
      const double orientation_error = localization_mode_ == "3d"
          ? quaternionAngularDistance(quaternions[i], quaternion)
          : std::abs(wrapAngle(yaws[i] - yaw));
      const bool position_outlier = outlier_position_threshold_ > 0.0 &&
                                    position_error > outlier_position_threshold_;
      const bool orientation_outlier = localization_mode_ == "3d"
          ? (outlier_orientation_threshold_ > 0.0 &&
             orientation_error > outlier_orientation_threshold_)
          : (outlier_yaw_threshold_ > 0.0 && orientation_error > outlier_yaw_threshold_);
      if (position_outlier || orientation_outlier)
      {
        results[indices[i]].rejected = true;
        results[indices[i]].status = "outlier";
      }
    }
  }

  void publishResults(const ros::Time& stamp, const std::vector<CameraResult>& input_results)
  {
    std::vector<CameraResult> results = input_results;
    applyOutlierGate(results);
    std::vector<const CameraResult*> result_for_camera(cameras_.size(), nullptr);
    for (const CameraResult& result : results)
      result_for_camera[result.camera_index] = &result;

    double quality_sum = 0.0;
    for (const CameraResult* result : result_for_camera)
      if (result && result->candidate && !result->rejected)
        quality_sum += result->candidate->weighted_quality;

    CameraBestTagArray camera_array;
    camera_array.header.stamp = stamp;
    camera_array.header.frame_id = map_frame_;
    camera_array.quality_sum = quality_sum;
    std::vector<double> weights;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> yaws;
    std::vector<double> rolls;
    std::vector<double> pitches;
    std::vector<tf::Quaternion> quaternions;
    std::vector<std::shared_ptr<Candidate>> candidates;
    std::vector<size_t> candidate_result_indices;
    for (size_t camera_index = 0; camera_index < cameras_.size(); ++camera_index)
    {
      if (!cameras_[camera_index].enabled)
        continue;
      CameraResult missing_result;
      missing_result.camera_index = camera_index;
      missing_result.status = "not_in_sync_window";
      const CameraResult& result = result_for_camera[camera_index]
          ? *result_for_camera[camera_index] : missing_result;
      const double weight = result.candidate && !result.rejected && quality_sum > 1e-12
          ? result.candidate->weighted_quality / quality_sum : 0.0;
      CameraBestTag camera_message;
      setCameraMessage(result, weight, camera_message);
      camera_array.cameras.push_back(camera_message);
      if (result.candidate && !result.rejected && weight > 0.0)
      {
        weights.push_back(weight);
        xs.push_back(result.candidate->map_to_base(0, 3));
        ys.push_back(result.candidate->map_to_base(1, 3));
        yaws.push_back(yawFromTransform(result.candidate->map_to_base));
        const tf::Quaternion quaternion = quaternionFromTransform(result.candidate->map_to_base);
        quaternions.push_back(quaternion);
        double roll = 0.0;
        double pitch = 0.0;
        double candidate_yaw = 0.0;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, candidate_yaw);
        rolls.push_back(roll);
        pitches.push_back(pitch);
        candidates.push_back(result.candidate);
        candidate_result_indices.push_back(camera_index);
      }
    }
    camera_results_publisher_.publish(camera_array);

    FusedAprilTagLocalization localization;
    localization.header = camera_array.header;
    localization.quality_sum = quality_sum;
    localization.localization_mode = localization_mode_;
    localization.contributing_camera_count = 0;
    localization.processing_time_sec = last_processing_time_sec_;
    localization.localization_age_sec = last_valid_stamp_.isZero()
        ? -1.0 : std::max(0.0, (stamp - last_valid_stamp_).toSec());
    localization.valid = weights.size() >= static_cast<size_t>(min_contributing_cameras_);
    localization.pose.header = camera_array.header;
    localization.pose.pose.pose.orientation.w = 1.0;
    std::fill(localization.pose.pose.covariance.begin(),
              localization.pose.pose.covariance.end(), 0.0);
    if (localization.valid)
    {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      for (size_t i = 0; i < weights.size(); ++i)
      {
        x += weights[i] * xs[i];
        y += weights[i] * ys[i];
        z += weights[i] * candidates[i]->map_to_base(2, 3);
      }
      const double yaw = weightedYaw(yaws, weights);
      const tf::Quaternion fused_quaternion =
          localization_mode_ == "3d" ? weightedQuaternion(quaternions, weights)
                                      : tf::Quaternion(0.0, 0.0, std::sin(yaw / 2.0),
                                                       std::cos(yaw / 2.0));
      double fused_roll = 0.0;
      double fused_pitch = 0.0;
      double fused_yaw = 0.0;
      tf::Matrix3x3(fused_quaternion).getRPY(fused_roll, fused_pitch, fused_yaw);
      localization.pose.pose.pose.position.x = x;
      localization.pose.pose.pose.position.y = y;
      localization.pose.pose.pose.position.z = localization_mode_ == "2d" ? 0.0 : z;
      localization.pose.pose.pose.orientation.x = fused_quaternion.x();
      localization.pose.pose.pose.orientation.y = fused_quaternion.y();
      localization.pose.pose.pose.orientation.z = fused_quaternion.z();
      localization.pose.pose.pose.orientation.w = fused_quaternion.w();
      const double base_x_variance = min_position_stddev_ * min_position_stddev_;
      const double base_yaw_variance = min_yaw_stddev_ * min_yaw_stddev_;
      double x_variance = base_x_variance;
      double y_variance = base_x_variance;
      double z_variance = localization_mode_ == "2d" ? invalid_variance_ : base_position_variance();
      double roll_variance = invalid_variance_;
      double pitch_variance = invalid_variance_;
      double yaw_variance = base_yaw_variance;
      if (localization_mode_ == "3d")
      {
        roll_variance = min_yaw_stddev_ * min_yaw_stddev_;
        pitch_variance = min_yaw_stddev_ * min_yaw_stddev_;
      }
      for (size_t i = 0; i < weights.size(); ++i)
      {
        x_variance += weights[i] * (xs[i] - x) * (xs[i] - x);
        y_variance += weights[i] * (ys[i] - y) * (ys[i] - y);
        if (localization_mode_ != "2d")
          z_variance += weights[i] * (candidates[i]->map_to_base(2, 3) - z) *
                        (candidates[i]->map_to_base(2, 3) - z);
        const double yaw_error = wrapAngle(yaws[i] - yaw);
        yaw_variance += weights[i] * yaw_error * yaw_error;
        if (localization_mode_ == "3d")
        {
          const double roll_error = wrapAngle(rolls[i] - fused_roll);
          const double pitch_error = wrapAngle(pitches[i] - fused_pitch);
          roll_variance += weights[i] * roll_error * roll_error;
          pitch_variance += weights[i] * pitch_error * pitch_error;
        }
        localization.camera_names.push_back(cameras_[candidate_result_indices[i]].name);
        localization.tag_ids.push_back(candidates[i]->tag_id);
        localization.qualities.push_back(candidates[i]->scores.quality);
        localization.weights.push_back(weights[i]);
      }
      localization.contributing_camera_count = static_cast<uint32_t>(weights.size());
      localization.pose.pose.covariance[0] = x_variance;
      localization.pose.pose.covariance[7] = y_variance;
      localization.pose.pose.covariance[14] = z_variance;
      localization.pose.pose.covariance[21] = localization_mode_ == "3d" ? roll_variance : invalid_variance_;
      localization.pose.pose.covariance[28] = localization_mode_ == "3d" ? pitch_variance : invalid_variance_;
      localization.pose.pose.covariance[35] = yaw_variance;
      last_valid_stamp_ = stamp;
      localization.localization_age_sec = 0.0;
      const Matrix4 map_to_base = matrixFromTf(tf::Transform(
          fused_quaternion,
          tf::Vector3(x, y, localization_mode_ == "2d" ? 0.0 : z)));
      publishFusedTf(map_to_base, stamp, localization);
      localization_publisher_.publish(localization);
      pose_publisher_.publish(localization.pose);
    }
    else
    {
      localization.pose.pose.covariance[0] = invalid_variance_;
      localization.pose.pose.covariance[7] = invalid_variance_;
      localization.pose.pose.covariance[14] = invalid_variance_;
      localization.pose.pose.covariance[21] = invalid_variance_;
      localization.pose.pose.covariance[28] = invalid_variance_;
      localization.pose.pose.covariance[35] = invalid_variance_;
      localization.tf_published = false;
      localization.tf_parent_frame = map_frame_;
      localization.tf_child_frame = tf_mode_ == "correction" ? odom_frame_ : base_frame_;
      if (!publish_tf_)
      {
        localization.tf_status = "disabled";
      }
      else if (tf_mode_ == "correction" && have_map_to_odom_)
      {
        // Invalid means no current visual observation. It must not stop the
        // independent odometry chain from being usable in the map frame.
        publishHeldCorrection(&localization);
      }
      else
      {
        localization.tf_status = "no_valid_pose";
      }
      localization_publisher_.publish(localization);
    }
  }

  double base_position_variance() const
  {
    return min_position_stddev_ * min_position_stddev_;
  }

  void publishFusedTf(const Matrix4& map_to_base, const ros::Time& stamp,
                      FusedAprilTagLocalization& localization)
  {
    localization.tf_published = false;
    localization.tf_parent_frame = map_frame_;
    localization.tf_child_frame = tf_mode_ == "correction" ? odom_frame_ : base_frame_;
    if (!publish_tf_)
    {
      localization.tf_status = "disabled";
      return;
    }

    Matrix4 transform = map_to_base;
    if (tf_mode_ == "correction")
    {
      tf::StampedTransform odom_to_base;
      try
      {
        tf_listener_.waitForTransform(
            odom_frame_, base_frame_, stamp, ros::Duration(tf_lookup_timeout_));
        tf_listener_.lookupTransform(odom_frame_, base_frame_, stamp, odom_to_base);
      }
      catch (const tf::TransformException& error)
      {
        localization.tf_status = "odom_base_unavailable";
        ROS_WARN_THROTTLE(2.0,
                          "unable to compute map->odom from odom->base at %.3f: %s",
                          stamp.toSec(), error.what());
        return;
      }
      transform = map_to_base * inverseRigid(matrixFromTf(odom_to_base));
      last_map_to_odom_ = transform;
      have_map_to_odom_ = true;

      // The correction is evaluated from transforms at the image stamp, but
      // map->odom is a global correction. Publish it at the current TF time
      // so a preceding held transform cannot make the timestamp go backwards.
      publishTransform(transform, ros::Time::now(), localization);
      return;
    }

    publishTransform(transform, stamp, localization);
  }

  void publishHeldCorrection(FusedAprilTagLocalization* localization)
  {
    if (!publish_tf_ || tf_mode_ != "correction" || !have_map_to_odom_)
      return;

    FusedAprilTagLocalization held_status;
    FusedAprilTagLocalization& status = localization ? *localization : held_status;
    status.tf_published = false;
    status.tf_parent_frame = map_frame_;
    status.tf_child_frame = odom_frame_;
    publishTransform(last_map_to_odom_, ros::Time::now(), status, "correction_held");
  }

  void publishTransform(const Matrix4& transform, const ros::Time& stamp,
                        FusedAprilTagLocalization& localization,
                        const std::string& success_status = "published")
  {
    if (stamp < last_tf_stamp_)
    {
      ROS_WARN_THROTTLE(2.0, "TF timestamp moved backwards; resetting TF timestamp guard");
      last_tf_stamp_ = ros::Time(0);
      last_tf_parent_.clear();
      last_tf_child_.clear();
    }
    if (!last_tf_stamp_.isZero() && stamp <= last_tf_stamp_ &&
        localization.tf_parent_frame == last_tf_parent_ &&
        localization.tf_child_frame == last_tf_child_)
    {
      localization.tf_status = "duplicate_timestamp_skipped";
      return;
    }

    tf_broadcaster_.sendTransform(tf::StampedTransform(
        tfFromMatrix(transform), stamp, localization.tf_parent_frame,
        localization.tf_child_frame));
    last_tf_stamp_ = stamp;
    last_tf_parent_ = localization.tf_parent_frame;
    last_tf_child_ = localization.tf_child_frame;
    localization.tf_published = true;
    localization.tf_status = success_status;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  std::vector<CameraConfig> cameras_;
  std::map<int, TagMapEntry> tag_map_;
  std::vector<image_transport::Subscriber> image_subscribers_;
  ros::Timer sync_timer_;
  ros::Publisher camera_results_publisher_;
  ros::Publisher localization_publisher_;
  ros::Publisher pose_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  FamilyHandle family_;
  apriltag_detector_t* detector_ = nullptr;

  std::string cameras_json_;
  std::string tag_map_file_;
  std::string map_type_;
  std::string localization_mode_ = "2d";
  std::string quality_mask_ = "1111";
  Json::Value config_root_;
  std::array<double, 4> metric_exponents_{{0.25, 0.25, 0.25, 0.25}};
  std::map<std::string, double> camera_confidence_multipliers_;
  std::string tag_family_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string tf_mode_ = "localization";
  int tag_threads_ = 2;
  int max_hamming_dist_ = 0;
  int tag_refine_edges_ = 1;
  int queue_size_ = 10;
  double tag_decimate_ = 1.0;
  double tag_blur_ = 0.0;
  double sync_slop_ = 0.05;
  double sync_wait_ = 0.02;
  double min_batch_interval_ = 0.0;
  double effective_batch_interval_ = 1.0 / 15.0;
  double process_rate_ = 15.0;
  double localization_timeout_ = 0.50;
  double stale_frame_timeout_ = 0.20;
  double area_reference_px_ = 1600.0;
  double margin_reference_ = 100.0;
  double sharpness_reference_ = 128.0;
  double distortion_scale_ = 0.20;
  double min_quality_ = 0.01;
  double min_position_stddev_ = 0.02;
  double min_yaw_stddev_ = 0.035;
  double invalid_variance_ = 1e6;
  double tf_lookup_timeout_ = 0.05;
  double min_tag_area_px_ = 0.0;
  double max_tag_range_m_ = 0.0;
  double outlier_position_threshold_ = 0.30;
  double outlier_yaw_threshold_ = 0.35;
  double outlier_orientation_threshold_ = 0.35;
  double last_processing_time_sec_ = 0.0;
  int min_contributing_cameras_ = 1;
  bool reject_unmapped_tags_ = true;
  bool outlier_gate_enabled_ = false;
  bool publish_tf_ = false;
  bool debug_images_ = false;
  ros::Time last_processed_stamp_;
  ros::Time last_batch_time_;
  ros::Time last_valid_stamp_;
  ros::Time last_timeout_publish_time_;
  ros::Time last_tf_stamp_;
  std::string last_tf_parent_;
  std::string last_tf_child_;
  Matrix4 last_map_to_odom_ = Matrix4::eye();
  bool have_map_to_odom_ = false;
};

}  // namespace laser_tag_nav_localization

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_localization_node");
  try
  {
    laser_tag_nav_localization::AprilTagLocalizationNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("AprilTag localization failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
