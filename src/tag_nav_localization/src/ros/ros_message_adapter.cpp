#include <algorithm>

#include <geometry_msgs/Pose.h>

#include <tag_nav_localization/core/transform_math.h>
#include <tag_nav_localization/ros/ros_message_adapter.h>

namespace tag_nav_localization
{
namespace ros_adapter
{
namespace
{

std::string frameIdFor(const core::CameraModel& camera, const CameraFrame& frame)
{
  if (frame.available && !frame.header.frame_id.empty())
    return frame.header.frame_id;
  if (!camera.frame_id.empty())
    return camera.frame_id;
  return camera.name + "_optical_frame";
}

const core::CameraObservation* findObservation(const core::FusionResult& result,
                                               std::size_t camera_index)
{
  for (const core::CameraObservation& observation : result.observations)
    if (observation.camera_index == camera_index)
      return &observation;
  return nullptr;
}

void setCameraMessage(const core::LocalizationConfig& config,
                      const core::CameraObservation* observation,
                      const CameraFrame& frame, double weight,
                      CameraBestTag& message)
{
  const core::CameraModel& camera = config.cameras[observation ? observation->camera_index : 0];
  message.camera_name = camera.name;
  message.header.stamp = frame.available ? frame.header.stamp : ros::Time::now();
  message.header.frame_id = frameIdFor(camera, frame);
  message.valid = observation && observation->has_candidate && !observation->rejected;
  message.status = observation && observation->rejected ? "outlier" :
      (observation && observation->has_candidate ? "valid" :
       (observation && !observation->status.empty() ? observation->status : "not_in_sync_window"));
  const bool has_report = observation && observation->candidate.tag_id >= 0;
  message.tag_id = has_report ? observation->candidate.tag_id : -1;
  message.weight = weight;
  message.quality_mask = config.quality.mask;
  if (!has_report)
    return;
  const core::TagCandidate& candidate = observation->candidate;
  if (observation->has_candidate)
  {
    message.camera_tag_pose.header = message.header;
    poseFromTransform(candidate.camera_to_tag, message.camera_tag_pose.pose.pose);
    message.range_m = candidate.range_m;
  }
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
  for (std::size_t index = 0; index < candidate.corners.size(); ++index)
  {
    message.corners[2 * index] = candidate.corners[index].x;
    message.corners[2 * index + 1] = candidate.corners[index].y;
  }
}

}  // namespace

void poseFromTransform(const core::Transform& transform, geometry_msgs::Pose& pose)
{
  const core::Quaternion quaternion = core::quaternionFromTransform(transform);
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  pose.orientation.x = quaternion.x;
  pose.orientation.y = quaternion.y;
  pose.orientation.z = quaternion.z;
  pose.orientation.w = quaternion.w;
}

CameraBestTagArray makeCameraArray(const ros::Time& stamp,
                                   const core::LocalizationConfig& config,
                                   const core::FusionResult& result,
                                   const std::vector<CameraFrame>& frames)
{
  CameraBestTagArray message;
  message.header.stamp = stamp;
  message.header.frame_id = config.output.map_frame;
  message.quality_sum = result.quality_sum;
  for (std::size_t camera_index = 0; camera_index < config.cameras.size(); ++camera_index)
  {
    const core::CameraModel& camera = config.cameras[camera_index];
    if (!camera.enabled)
      continue;
    const core::CameraObservation* observation = findObservation(result, camera_index);
    const CameraFrame frame = camera_index < frames.size() ? frames[camera_index] : CameraFrame();
    const double weight = observation && observation->has_candidate && !observation->rejected &&
        result.quality_sum > 1e-12 ? observation->candidate.weighted_quality / result.quality_sum : 0.0;
    CameraBestTag camera_message;
    if (observation)
    {
      setCameraMessage(config, observation, frame, weight, camera_message);
    }
    else
    {
      core::CameraObservation missing;
      missing.camera_index = camera_index;
      missing.status = "not_in_sync_window";
      setCameraMessage(config, &missing, frame, weight, camera_message);
    }
    message.cameras.push_back(camera_message);
  }
  return message;
}

FusedAprilTagLocalization makeLocalization(const ros::Time& stamp,
                                           const core::LocalizationConfig& config,
                                           const core::FusionResult& result,
                                           double processing_time_sec,
                                           double localization_age_sec,
                                           const TfPublicationStatus& tf_status)
{
  FusedAprilTagLocalization message;
  message.header.stamp = stamp;
  message.header.frame_id = config.output.map_frame;
  message.valid = result.valid;
  message.localization_mode = config.fusion.mode;
  message.pose.header = message.header;
  poseFromTransform(result.pose.map_to_base, message.pose.pose.pose);
  std::copy(result.pose.covariance.begin(), result.pose.covariance.end(),
            message.pose.pose.covariance.begin());
  message.camera_names = result.camera_names;
  message.tag_ids = result.tag_ids;
  message.qualities = result.qualities;
  message.weights = result.weights;
  message.quality_sum = result.quality_sum;
  message.contributing_camera_count = static_cast<uint32_t>(result.contributing_camera_count);
  message.processing_time_sec = processing_time_sec;
  message.localization_age_sec = localization_age_sec;
  message.tf_published = tf_status.published;
  message.tf_status = tf_status.status;
  message.tf_parent_frame = tf_status.parent_frame;
  message.tf_child_frame = tf_status.child_frame;
  return message;
}

}  // namespace ros_adapter
}  // namespace tag_nav_localization
