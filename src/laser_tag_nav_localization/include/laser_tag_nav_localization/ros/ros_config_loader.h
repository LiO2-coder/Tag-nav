#ifndef LASER_TAG_NAV_LOCALIZATION_ROS_CONFIG_LOADER_H
#define LASER_TAG_NAV_LOCALIZATION_ROS_CONFIG_LOADER_H

#include <ros/node_handle.h>

#include <laser_tag_nav_localization/core/types.h>

namespace laser_tag_nav_localization
{
namespace ros_adapter
{

class RosConfigLoader
{
public:
  explicit RosConfigLoader(const ros::NodeHandle& private_node_handle);
  core::LocalizationConfig load() const;

private:
  ros::NodeHandle private_node_handle_;
};

}  // namespace ros_adapter
}  // namespace laser_tag_nav_localization

#endif
