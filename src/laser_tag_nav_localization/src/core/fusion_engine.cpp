#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <laser_tag_nav_localization/core/fusion_engine.h>
#include <laser_tag_nav_localization/core/transform_math.h>

namespace laser_tag_nav_localization
{
namespace core
{
namespace
{

void applyOutlierGate(const LocalizationConfig& config,
                      std::vector<CameraObservation>& observations)
{
  if (!config.fusion.outlier_gate_enabled)
    return;
  std::vector<std::size_t> indices;
  std::vector<double> weights;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<double> yaws;
  std::vector<Quaternion> quaternions;
  for (std::size_t index = 0; index < observations.size(); ++index)
  {
    const CameraObservation& observation = observations[index];
    if (!observation.has_candidate || observation.rejected)
      continue;
    indices.push_back(index);
    weights.push_back(observation.candidate.weighted_quality);
    xs.push_back(observation.candidate.map_to_base(0, 3));
    ys.push_back(observation.candidate.map_to_base(1, 3));
    zs.push_back(observation.candidate.map_to_base(2, 3));
    yaws.push_back(yawFromTransform(observation.candidate.map_to_base));
    quaternions.push_back(quaternionFromTransform(observation.candidate.map_to_base));
  }
  if (indices.size() < 2)
    return;
  const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
  if (sum <= 1e-12)
    return;
  for (double& weight : weights)
    weight /= sum;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  for (std::size_t index = 0; index < weights.size(); ++index)
  {
    x += weights[index] * xs[index];
    y += weights[index] * ys[index];
    z += weights[index] * zs[index];
  }
  const double yaw = weightedYaw(yaws, weights);
  const Quaternion quaternion = weightedQuaternion(quaternions, weights);
  for (std::size_t index = 0; index < indices.size(); ++index)
  {
    const double dz = config.fusion.mode == "2d" ? 0.0 : zs[index] - z;
    const double position_error = std::sqrt((xs[index] - x) * (xs[index] - x) +
                                            (ys[index] - y) * (ys[index] - y) + dz * dz);
    const double orientation_error = config.fusion.mode == "3d"
        ? quaternionAngularDistance(quaternions[index], quaternion)
        : std::abs(wrapAngle(yaws[index] - yaw));
    const bool position_outlier = config.fusion.outlier_position_threshold > 0.0 &&
        position_error > config.fusion.outlier_position_threshold;
    const bool orientation_outlier = config.fusion.mode == "3d"
        ? (config.fusion.outlier_orientation_threshold > 0.0 &&
           orientation_error > config.fusion.outlier_orientation_threshold)
        : (config.fusion.outlier_yaw_threshold > 0.0 &&
           orientation_error > config.fusion.outlier_yaw_threshold);
    if (position_outlier || orientation_outlier)
    {
      observations[indices[index]].rejected = true;
      observations[indices[index]].status = "outlier";
    }
  }
}

}  // namespace

FusionEngine::FusionEngine(const LocalizationConfig& config) : config_(config)
{
}

FusionResult FusionEngine::fuse(const std::vector<CameraObservation>& observations) const
{
  FusionResult result;
  result.observations = observations;
  applyOutlierGate(config_, result.observations);

  std::vector<double> weights;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<double> yaws;
  std::vector<double> rolls;
  std::vector<double> pitches;
  std::vector<Quaternion> quaternions;
  std::vector<const CameraObservation*> contributors;
  for (const CameraObservation& observation : result.observations)
  {
    if (!observation.has_candidate || observation.rejected)
      continue;
    result.quality_sum += observation.candidate.weighted_quality;
  }
  if (result.quality_sum > 1e-12)
  {
    for (const CameraObservation& observation : result.observations)
    {
      if (!observation.has_candidate || observation.rejected)
        continue;
      const double weight = observation.candidate.weighted_quality / result.quality_sum;
      weights.push_back(weight);
      xs.push_back(observation.candidate.map_to_base(0, 3));
      ys.push_back(observation.candidate.map_to_base(1, 3));
      zs.push_back(observation.candidate.map_to_base(2, 3));
      yaws.push_back(yawFromTransform(observation.candidate.map_to_base));
      const Quaternion quaternion = quaternionFromTransform(observation.candidate.map_to_base);
      quaternions.push_back(quaternion);
      double roll = 0.0;
      double pitch = 0.0;
      double ignored_yaw = 0.0;
      rpyFromQuaternion(quaternion, roll, pitch, ignored_yaw);
      rolls.push_back(roll);
      pitches.push_back(pitch);
      contributors.push_back(&observation);
    }
  }
  result.valid = weights.size() >= static_cast<std::size_t>(config_.fusion.min_contributing_cameras);
  result.contributing_camera_count = result.valid ? weights.size() : 0;
  result.pose.covariance.fill(0.0);
  if (!result.valid)
  {
    result.pose.covariance[0] = config_.output.invalid_variance;
    result.pose.covariance[7] = config_.output.invalid_variance;
    result.pose.covariance[14] = config_.output.invalid_variance;
    result.pose.covariance[21] = config_.output.invalid_variance;
    result.pose.covariance[28] = config_.output.invalid_variance;
    result.pose.covariance[35] = config_.output.invalid_variance;
    return result;
  }

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  for (std::size_t index = 0; index < weights.size(); ++index)
  {
    x += weights[index] * xs[index];
    y += weights[index] * ys[index];
    z += weights[index] * zs[index];
  }
  const double yaw = weightedYaw(yaws, weights);
  const Quaternion fused_quaternion = config_.fusion.mode == "3d"
      ? weightedQuaternion(quaternions, weights) : quaternionFromYaw(yaw);
  double fused_roll = 0.0;
  double fused_pitch = 0.0;
  double fused_yaw = 0.0;
  rpyFromQuaternion(fused_quaternion, fused_roll, fused_pitch, fused_yaw);
  const double base_position_variance = config_.fusion.min_position_stddev *
                                        config_.fusion.min_position_stddev;
  const double base_yaw_variance = config_.fusion.min_yaw_stddev * config_.fusion.min_yaw_stddev;
  double x_variance = base_position_variance;
  double y_variance = base_position_variance;
  double z_variance = config_.fusion.mode == "2d" ? config_.output.invalid_variance : base_position_variance;
  double roll_variance = config_.output.invalid_variance;
  double pitch_variance = config_.output.invalid_variance;
  double yaw_variance = base_yaw_variance;
  if (config_.fusion.mode == "3d")
  {
    roll_variance = base_yaw_variance;
    pitch_variance = base_yaw_variance;
  }
  for (std::size_t index = 0; index < weights.size(); ++index)
  {
    x_variance += weights[index] * (xs[index] - x) * (xs[index] - x);
    y_variance += weights[index] * (ys[index] - y) * (ys[index] - y);
    if (config_.fusion.mode != "2d")
      z_variance += weights[index] * (zs[index] - z) * (zs[index] - z);
    const double yaw_error = wrapAngle(yaws[index] - yaw);
    yaw_variance += weights[index] * yaw_error * yaw_error;
    if (config_.fusion.mode == "3d")
    {
      const double roll_error = wrapAngle(rolls[index] - fused_roll);
      const double pitch_error = wrapAngle(pitches[index] - fused_pitch);
      roll_variance += weights[index] * roll_error * roll_error;
      pitch_variance += weights[index] * pitch_error * pitch_error;
    }
    const CameraObservation& observation = *contributors[index];
    result.camera_names.push_back(config_.cameras[observation.camera_index].name);
    result.tag_ids.push_back(observation.candidate.tag_id);
    result.qualities.push_back(observation.candidate.scores.quality);
    result.weights.push_back(weights[index]);
  }
  result.pose.map_to_base = transformFromQuaternion(
      fused_quaternion, x, y, config_.fusion.mode == "2d" ? 0.0 : z);
  result.pose.covariance[0] = x_variance;
  result.pose.covariance[7] = y_variance;
  result.pose.covariance[14] = z_variance;
  result.pose.covariance[21] = config_.fusion.mode == "3d" ? roll_variance : config_.output.invalid_variance;
  result.pose.covariance[28] = config_.fusion.mode == "3d" ? pitch_variance : config_.output.invalid_variance;
  result.pose.covariance[35] = yaw_variance;
  return result;
}

}  // namespace core
}  // namespace laser_tag_nav_localization
