#ifndef LASER_TAG_NAV_PLANNER_ROS_WAYPOINT_FOLLOWER_H
#define LASER_TAG_NAV_PLANNER_ROS_WAYPOINT_FOLLOWER_H

#include <ros/ros.h>

#include <cstddef>
#include <string>

#include <geometry_msgs/Twist.h>

#include <laser_tag_nav_planner/core/types.h>
#include <laser_tag_nav_planner/ros/ros_planner_config.h>

namespace laser_tag_nav_planner
{
namespace ros_adapter
{

enum class FollowerState
{
  IDLE,
  TURN,
  DRIVE,
  ARRIVED,
  NO_PATH,
  LOCALIZATION_LOST
};

const char* toString(FollowerState state);

// Stop-and-go waypoint follower. Call setPose() (with the latest fused
// map -> base pose) and update() at the controller rate from a single thread,
// e.g. a ros::Timer callback. It advances the target through the waypoint list,
// turning in place at each node to face the next before driving straight.
class WaypointFollower
{
public:
  explicit WaypointFollower(const PlannerConfig& config);

  void setPath(const core::PathResult& path);
  void clearPath();

  void setPose(const ros::Time& stamp, double x, double y, double yaw);

  geometry_msgs::Twist update(const ros::Time& now);

  FollowerState state() const { return state_; }
  std::size_t current_index() const { return target_index_; }

private:
  PlannerConfig config_;
  core::PathResult path_;
  std::size_t target_index_ = 0;
  FollowerState state_ = FollowerState::IDLE;

  bool have_pose_ = false;
  ros::Time pose_stamp_;
  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double pose_yaw_ = 0.0;
};

}  // namespace ros_adapter
}  // namespace laser_tag_nav_planner

#endif
