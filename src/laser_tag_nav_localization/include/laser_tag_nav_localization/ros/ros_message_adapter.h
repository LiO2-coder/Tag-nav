#ifndef LASER_TAG_NAV_LOCALIZATION_ROS_MESSAGE_ADAPTER_H
#define LASER_TAG_NAV_LOCALIZATION_ROS_MESSAGE_ADAPTER_H

#include <vector>

#include <std_msgs/Header.h>

#include <laser_tag_nav_localization/CameraBestTagArray.h>
#include <laser_tag_nav_localization/FusedAprilTagLocalization.h>
#include <laser_tag_nav_localization/core/types.h>
#include <laser_tag_nav_localization/ros/ros_tf_bridge.h>

namespace laser_tag_nav_localization
{
namespace ros_adapter
{

struct CameraFrame
{
  bool available = false;
  std_msgs::Header header;
};

CameraBestTagArray makeCameraArray(const ros::Time& stamp,
                                   const core::LocalizationConfig& config,
                                   const core::FusionResult& result,
                                   const std::vector<CameraFrame>& frames);

FusedAprilTagLocalization makeLocalization(const ros::Time& stamp,
                                           const core::LocalizationConfig& config,
                                           const core::FusionResult& result,
                                           double processing_time_sec,
                                           double localization_age_sec,
                                           const TfPublicationStatus& tf_status);

void poseFromTransform(const core::Transform& transform, geometry_msgs::Pose& pose);

}  // namespace ros_adapter
}  // namespace laser_tag_nav_localization

#endif
