#ifndef LASER_TAG_NAV_PLANNER_ROS_ROS_PLANNER_CONFIG_H
#define LASER_TAG_NAV_PLANNER_ROS_ROS_PLANNER_CONFIG_H

#include <ros/ros.h>

#include <string>

namespace laser_tag_nav_planner
{
namespace ros_adapter
{

struct PlannerConfig
{
  int start_tag = 0;
  int goal_tag = 0;
  std::string connectivity_json;
  std::string tag_map_file;
  std::string map_frame = "map";
  std::string base_frame = "base_footprint";
  double linear_velocity = 0.30;
  double angular_velocity = 0.50;
  double position_tolerance = 0.05;
  double heading_tolerance = 0.05;
  double slow_radius = 0.50;
  bool allow_diagonal = true;
  double grid_spacing = 1.0;
  double controller_rate = 20.0;
  double pose_timeout = 0.5;
  bool auto_start = false;
};

class RosPlannerConfigLoader
{
public:
  explicit RosPlannerConfigLoader(const ros::NodeHandle& private_node_handle);
  PlannerConfig load() const;

private:
  ros::NodeHandle private_node_handle_;
};

}  // namespace ros_adapter
}  // namespace laser_tag_nav_planner

#endif
