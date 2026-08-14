#ifndef LASER_TAG_NAV_LOCALIZATION_ROS_TF_BRIDGE_H
#define LASER_TAG_NAV_LOCALIZATION_ROS_TF_BRIDGE_H

#include <string>

#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <laser_tag_nav_localization/core/types.h>

namespace laser_tag_nav_localization
{
namespace ros_adapter
{

struct TfPublicationStatus
{
  bool published = false;
  std::string status;
  std::string parent_frame;
  std::string child_frame;
};

class RosTfBridge
{
public:
  explicit RosTfBridge(const core::OutputConfig& config);

  TfPublicationStatus publishFused(const core::Transform& map_to_base,
                                   const ros::Time& stamp);
  TfPublicationStatus publishHeldCorrection();
  void republishHeldCorrection();

private:
  TfPublicationStatus publish(const core::Transform& transform, const ros::Time& stamp,
                              const std::string& parent, const std::string& child,
                              const std::string& success_status);
  TfPublicationStatus publishCorrection(const core::Transform& transform,
                                        const std::string& success_status);

  core::OutputConfig config_;
  tf::TransformBroadcaster broadcaster_;
  tf::TransformListener listener_;
  ros::Time last_tf_stamp_;
  std::string last_tf_parent_;
  std::string last_tf_child_;
  core::Transform last_map_to_odom_ = core::Transform::eye();
  bool have_map_to_odom_ = false;
};

}  // namespace ros_adapter
}  // namespace laser_tag_nav_localization

#endif
