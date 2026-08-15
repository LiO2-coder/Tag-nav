#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <jsoncpp/json/json.h>

#include <laser_tag_nav_localization/core/configuration.h>
#include <laser_tag_nav_localization/core/transform_math.h>

namespace laser_tag_nav_localization
{
namespace core
{
namespace
{

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
  for (Json::ArrayIndex index = 0; index < value.size(); ++index)
    output.push_back(getNumber(value[index], name));
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
  return section.isMember(name) ? getNumber(section[name], name) : fallback;
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
  return cv::Matx33d(values[0], values[1], values[2], values[3], values[4], values[5],
                     values[6], values[7], values[8]);
}

Transform matrixFromJson(const Json::Value& value, const std::string& name)
{
  if (!value.isArray() || value.size() != 4)
    throw std::runtime_error(name + " must be a 4x4 array");
  Transform result = Transform::eye();
  for (Json::ArrayIndex row = 0; row < 4; ++row)
  {
    const std::vector<double> values = getNumberArray(value[row], name);
    if (values.size() != 4)
      throw std::runtime_error(name + " must be a 4x4 array");
    for (std::size_t col = 0; col < 4; ++col)
      result(static_cast<int>(row), static_cast<int>(col)) = values[col];
  }
  if (std::abs(result(3, 0)) > 1e-6 || std::abs(result(3, 1)) > 1e-6 ||
      std::abs(result(3, 2)) > 1e-6 || std::abs(result(3, 3) - 1.0) > 1e-6)
    throw std::runtime_error(name + " must be a homogeneous transform");
  return result;
}

void parseQualityConfiguration(const Json::Value& section, QualityConfig& quality)
{
  quality.mask = jsonString(section, "mask", quality.mask);
  quality.area_reference_px = jsonDouble(section, "area_reference_px", quality.area_reference_px);
  quality.margin_reference = jsonDouble(section, "margin_reference", quality.margin_reference);
  quality.sharpness_reference = jsonDouble(section, "sharpness_reference", quality.sharpness_reference);
  quality.distortion_scale = jsonDouble(section, "distortion_scale", quality.distortion_scale);
  quality.min_quality = jsonDouble(section, "min_quality", quality.min_quality);
  quality.requested_metric_exponents = {{1.0, 1.0, 1.0, 1.0}};
  if (section.isMember("metric_exponents"))
  {
    const Json::Value& exponents = section["metric_exponents"];
    if (!exponents.isObject())
      throw std::runtime_error("quality.metric_exponents must be an object");
    quality.requested_metric_exponents[0] = jsonDouble(exponents, "area", 1.0);
    quality.requested_metric_exponents[1] = jsonDouble(exponents, "distortion", 1.0);
    quality.requested_metric_exponents[2] = jsonDouble(exponents, "margin", 1.0);
    quality.requested_metric_exponents[3] = jsonDouble(exponents, "sharpness", 1.0);
  }
}

void parseFusionConfiguration(const Json::Value& section, FusionConfig& fusion)
{
  fusion.mode = jsonString(section, "mode", fusion.mode);
  fusion.min_contributing_cameras = jsonInt(section, "min_contributing_cameras",
                                            fusion.min_contributing_cameras);
  fusion.min_position_stddev = jsonDouble(section, "min_position_stddev_m",
                                          fusion.min_position_stddev);
  fusion.min_yaw_stddev = jsonDouble(section, "min_yaw_stddev_rad", fusion.min_yaw_stddev);
  const Json::Value& multipliers = section["camera_confidence_multipliers"];
  if (!multipliers.isNull())
  {
    if (!multipliers.isObject())
      throw std::runtime_error("fusion.camera_confidence_multipliers must be an object");
    for (const std::string& name : multipliers.getMemberNames())
      fusion.camera_confidence_multipliers[name] = getNumber(multipliers[name], name);
  }
  const Json::Value& gate = section["outlier_gate"];
  fusion.outlier_gate_enabled = jsonBool(gate, "enabled", fusion.outlier_gate_enabled);
  fusion.outlier_position_threshold = jsonDouble(
      gate, "max_position_residual_m", fusion.outlier_position_threshold);
  fusion.outlier_yaw_threshold = jsonDouble(gate, "max_yaw_residual_rad", fusion.outlier_yaw_threshold);
  fusion.outlier_orientation_threshold = jsonDouble(
      gate, "max_orientation_residual_rad", fusion.outlier_orientation_threshold);
}

void parseTemporalFilterConfiguration(const Json::Value& section, TemporalFilterConfig& filter)
{
  filter.enabled = jsonBool(section, "enabled", filter.enabled);
  filter.position_time_constant_sec = jsonDouble(
      section, "position_time_constant_sec", filter.position_time_constant_sec);
  filter.orientation_time_constant_sec = jsonDouble(
      section, "orientation_time_constant_sec", filter.orientation_time_constant_sec);
  filter.max_dt_sec = jsonDouble(section, "max_dt_sec", filter.max_dt_sec);
}

Json::Value parseJson(const std::string& text, const std::string& source)
{
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  std::string errors;
  Json::Value root;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors))
    throw std::runtime_error("invalid " + source + ": " + errors);
  return root;
}

}  // namespace

double LocalizationConfig::effectiveBatchInterval() const
{
  return std::max(1.0 / runtime.process_rate_hz, synchronization.min_batch_interval_sec);
}

LocalizationConfig parseConfiguration(const std::string& cameras_json,
                                      const std::string& tag_map_override,
                                      const UriResolver& resolve_uri)
{
  LocalizationConfig config;
  const Json::Value root = parseJson(cameras_json, "~cameras_json");
  const bool legacy = root.isArray();
  if (!legacy && root.isMember("schema_version") &&
      (!root["schema_version"].isInt() || root["schema_version"].asInt() != 1))
    throw std::runtime_error("unsupported cameras JSON schema_version; expected 1");

  const Json::Value& detector = jsonSection(root, "detector");
  config.detector.tag_family = jsonString(detector, "tag_family", config.detector.tag_family);
  config.detector.threads = jsonInt(detector, "threads", config.detector.threads);
  config.detector.decimate = jsonDouble(detector, "decimate", config.detector.decimate);
  config.detector.blur = jsonDouble(detector, "blur", config.detector.blur);
  config.detector.refine_edges = jsonBool(detector, "refine_edges", config.detector.refine_edges);
  config.detector.max_hamming_dist = jsonInt(detector, "max_hamming_dist",
                                              config.detector.max_hamming_dist);

  const Json::Value& synchronization = jsonSection(root, "synchronization");
  config.synchronization.queue_size = jsonInt(synchronization, "queue_size",
                                              config.synchronization.queue_size);
  config.synchronization.slop_sec = jsonDouble(synchronization, "slop_sec",
                                                config.synchronization.slop_sec);
  config.synchronization.wait_sec = jsonDouble(synchronization, "wait_sec",
                                                config.synchronization.wait_sec);
  config.synchronization.min_batch_interval_sec = jsonDouble(
      synchronization, "min_batch_interval_sec", config.synchronization.min_batch_interval_sec);

  const Json::Value& runtime = jsonSection(root, "runtime");
  config.runtime.process_rate_hz = jsonDouble(runtime, "process_rate_hz", config.runtime.process_rate_hz);
  config.runtime.localization_timeout_sec = jsonDouble(
      runtime, "localization_timeout_sec", config.runtime.localization_timeout_sec);
  parseQualityConfiguration(jsonSection(root, "quality"), config.quality);
  parseFusionConfiguration(jsonSection(root, "fusion"), config.fusion);
  parseTemporalFilterConfiguration(jsonSection(root, "temporal_filter"), config.temporal_filter);

  const Json::Value& validation = jsonSection(root, "validation");
  config.validation.min_tag_area_px = jsonDouble(validation, "min_tag_area_px",
                                                 config.validation.min_tag_area_px);
  config.validation.max_tag_range_m = jsonDouble(validation, "max_tag_range_m",
                                                 config.validation.max_tag_range_m);
  config.validation.reject_unmapped_tags = jsonBool(validation, "reject_unmapped_tags",
                                                    config.validation.reject_unmapped_tags);
  config.validation.stale_frame_timeout_sec = jsonDouble(
      validation, "stale_frame_timeout_sec", config.validation.stale_frame_timeout_sec);

  const Json::Value& output = jsonSection(root, "output");
  config.output.map_frame = jsonString(output, "map_frame", config.output.map_frame);
  config.output.odom_frame = jsonString(output, "odom_frame", config.output.odom_frame);
  config.output.base_frame = jsonString(output, "base_frame", config.output.base_frame);
  config.output.tf_mode = jsonString(output, "tf_mode", config.output.tf_mode);
  config.output.publish_tf = jsonBool(output, "publish_tf", config.output.publish_tf);
  config.output.tf_lookup_timeout_sec = jsonDouble(output, "tf_lookup_timeout_sec",
                                                    config.output.tf_lookup_timeout_sec);
  config.output.correction_tf_tolerance_sec = jsonDouble(
      output, "correction_tf_tolerance_sec", config.output.correction_tf_tolerance_sec);
  config.output.correction_tf_publish_rate_hz = jsonDouble(
      output, "correction_tf_publish_rate_hz", config.output.correction_tf_publish_rate_hz);
  config.output.debug_images = jsonBool(output, "debug_images", config.output.debug_images);
  config.output.invalid_variance = jsonDouble(output, "invalid_variance", config.output.invalid_variance);

  const Json::Value& map = jsonSection(root, "map");
  config.tag_map_file = tag_map_override.empty() ? jsonString(map, "uri", "") : tag_map_override;
  if (config.tag_map_file.empty())
    throw std::runtime_error("map.uri or explicit ~tag_map_file is required");
  config.tag_map_file = resolve_uri ? resolve_uri(config.tag_map_file) : config.tag_map_file;

  const Json::Value entries = legacy ? root : root["cameras"];
  if (!entries.isArray() || entries.empty())
    throw std::runtime_error("~cameras_json must contain a non-empty cameras array");
  std::map<std::string, bool> names;
  for (Json::ArrayIndex index = 0; index < entries.size(); ++index)
  {
    const Json::Value& value = entries[index];
    if (!value.isObject() || !value.isMember("name") || !value.isMember("image_topic"))
      throw std::runtime_error("each camera needs name and image_topic");
    CameraModel camera;
    camera.name = value["name"].asString();
    camera.topic = value["image_topic"].asString();
    camera.enabled = !value.isMember("enabled") || value["enabled"].asBool();
    camera.transport = value.get("transport", "raw").asString();
    camera.data_format = value.get("data_format", "").asString();
    camera.frame_id = value.get("frame_id", "").asString();
    camera.confidence_multiplier = value.get("confidence_multiplier", 1.0).asDouble();
    const auto multiplier = config.fusion.camera_confidence_multipliers.find(camera.name);
    if (multiplier != config.fusion.camera_confidence_multipliers.end())
      camera.confidence_multiplier = multiplier->second;
    if (camera.name.empty() || camera.topic.empty() || names[camera.name])
      throw std::runtime_error("camera names must be unique and non-empty");
    names[camera.name] = true;
    if (!value.isMember("intrinsics") || !value["intrinsics"].isObject() ||
        !value["intrinsics"].isMember("K"))
      throw std::runtime_error("camera " + camera.name + " needs intrinsics.K");
    const Json::Value& intrinsics = value["intrinsics"];
    camera.K = parseCameraMatrix(intrinsics["K"]);
    const std::vector<double> distortion = intrinsics.isMember("D")
        ? getNumberArray(intrinsics["D"], "intrinsics.D") : std::vector<double>();
    if (!distortion.empty())
      camera.D = cv::Mat(distortion).reshape(1, 1).clone();
    camera.distortion_model = intrinsics.get("distortion_model", "plumb_bob").asString();
    std::transform(camera.distortion_model.begin(), camera.distortion_model.end(),
                   camera.distortion_model.begin(), ::tolower);
    if (!value.isMember("base_to_camera"))
      throw std::runtime_error("camera " + camera.name + " needs base_to_camera");
    const Json::Value& extrinsic = value["base_to_camera"];
    camera.base_to_camera = transformFromTranslationRpy(
        getNumberArray(extrinsic["translation"], "base_to_camera.translation"),
        getNumberArray(extrinsic["rotation_rpy"], "base_to_camera.rotation_rpy"));
    config.cameras.push_back(camera);
  }
  return config;
}

void loadTagMap(LocalizationConfig& config)
{
  std::ifstream input(config.tag_map_file.c_str());
  if (!input)
    throw std::runtime_error("unable to open tag map: " + config.tag_map_file);
  std::ostringstream contents;
  contents << input.rdbuf();
  const Json::Value root = parseJson(contents.str(), "tag map JSON");
  config.map_type = root.get("map_type", "").asString();
  if (config.map_type != "2d" && config.map_type != "2.5d" && config.map_type != "3d")
    throw std::runtime_error("tag map map_type must be 2d, 2.5d or 3d");
  if (config.fusion.mode == "auto")
    config.fusion.mode = config.map_type;
  else if (config.fusion.mode != config.map_type)
    throw std::runtime_error("fusion.mode does not match tag map map_type");
  if (!root.isMember("tag_locations") || !root["tag_locations"].isObject())
    throw std::runtime_error("tag map needs tag_locations");
  const Json::Value lengths = root.get("tag_side_lengths", Json::Value(Json::objectValue));
  const double default_size = lengths.get("default", 0.10).asDouble();
  config.tag_map.clear();
  for (const std::string& key : root["tag_locations"].getMemberNames())
  {
    TagMapEntry entry;
    if (config.map_type == "2d" || config.map_type == "2.5d")
    {
      const std::vector<double> location = getNumberArray(root["tag_locations"][key], key);
      const std::size_t expected_size = config.map_type == "2d" ? 3 : 4;
      if (location.size() != expected_size)
        throw std::runtime_error("tag location " + key + " has invalid dimension");
      entry.map_to_tag = mapTagTransform(location[0], location[1], location[2]);
      if (config.map_type == "2.5d")
        entry.map_to_tag(2, 3) = location[3];
    }
    else
    {
      entry.map_to_tag = matrixFromJson(root["tag_locations"][key], "tag location " + key);
    }
    entry.size = lengths.isMember(key) ? lengths[key].asDouble() : default_size;
    if (entry.size <= 0.0)
      throw std::runtime_error("tag " + key + " has non-positive size");
    config.tag_map[std::stoi(key)] = entry;
  }
  if (config.tag_map.empty())
    throw std::runtime_error("tag map contains no locations");
}

void validateConfiguration(LocalizationConfig& config)
{
  if (!validQualityMask(config.quality.mask))
    throw std::runtime_error("quality.mask must be a non-zero four-bit ADMS string");
  try
  {
    config.quality.metric_exponents = normalizedQualityExponents(
        config.quality.mask, config.quality.requested_metric_exponents);
  }
  catch (const std::invalid_argument& error)
  {
    throw std::runtime_error(error.what());
  }
  if (config.fusion.mode != "auto" && config.fusion.mode != "2d" &&
      config.fusion.mode != "2.5d" && config.fusion.mode != "3d")
    throw std::runtime_error("fusion_mode must be auto, 2d, 2.5d or 3d");
  if (config.output.tf_mode != "localization" && config.output.tf_mode != "correction")
    throw std::runtime_error("tf_mode must be localization or correction");
  if (config.output.map_frame.empty() || config.output.odom_frame.empty() || config.output.base_frame.empty())
    throw std::runtime_error("map_frame, odom_frame and base_frame cannot be empty");
  if (config.output.tf_mode == "correction" &&
      (config.output.map_frame == config.output.odom_frame ||
       config.output.map_frame == config.output.base_frame ||
       config.output.odom_frame == config.output.base_frame))
    throw std::runtime_error("map, odom and base frames must be distinct in correction mode");
  if (config.output.tf_lookup_timeout_sec < 0.0 || config.output.correction_tf_tolerance_sec < 0.0 ||
      config.output.correction_tf_publish_rate_hz <= 0.0)
    throw std::runtime_error("TF timing parameters must be non-negative and publish rate positive");
  if (config.runtime.process_rate_hz <= 0.0 || config.synchronization.slop_sec < 0.0 ||
      config.synchronization.wait_sec < 0.0 || config.synchronization.queue_size <= 0 ||
      config.runtime.localization_timeout_sec < 0.0 || config.validation.stale_frame_timeout_sec < 0.0 ||
      config.synchronization.min_batch_interval_sec < 0.0)
    throw std::runtime_error("invalid timing or queue configuration");
  if (config.validation.min_tag_area_px < 0.0 || config.validation.max_tag_range_m < 0.0)
    throw std::runtime_error("validation thresholds cannot be negative");
  if (config.fusion.min_contributing_cameras < 1 || config.fusion.outlier_position_threshold < 0.0 ||
      config.fusion.outlier_yaw_threshold < 0.0 || config.fusion.outlier_orientation_threshold < 0.0)
    throw std::runtime_error("invalid fusion configuration");
  if (config.temporal_filter.enabled &&
      (config.temporal_filter.position_time_constant_sec <= 0.0 ||
       config.temporal_filter.orientation_time_constant_sec <= 0.0 ||
       config.temporal_filter.max_dt_sec <= 0.0))
    throw std::runtime_error("temporal_filter time constants and max_dt_sec must be positive");
  for (const auto& camera : config.cameras)
  {
    if (camera.K(0, 0) <= 0.0 || camera.K(1, 1) <= 0.0)
      throw std::runtime_error("camera " + camera.name + " has invalid focal length");
    if (camera.confidence_multiplier < 0.0)
      throw std::runtime_error("camera " + camera.name + " has negative confidence multiplier");
    if ((camera.distortion_model == "equidistant" || camera.distortion_model == "fisheye") &&
        !camera.D.empty() && camera.D.total() != 4)
      throw std::runtime_error("camera " + camera.name + " fisheye D must have length 4");
    if (camera.distortion_model != "plumb_bob" && camera.distortion_model != "rational_polynomial" &&
        camera.distortion_model != "equidistant" && camera.distortion_model != "fisheye" &&
        camera.distortion_model != "none")
      throw std::runtime_error("unsupported distortion_model for camera " + camera.name);
  }
}

}  // namespace core
}  // namespace laser_tag_nav_localization
