#include <algorithm>
#include <cmath>

#include <laser_tag_nav_planner/core/direction.h>
#include <laser_tag_nav_planner/ros/waypoint_follower.h>

namespace laser_tag_nav_planner
{
namespace ros_adapter
{
namespace
{

constexpr double kHeadingProportionalGain = 4.0;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

}  // namespace

const char* toString(FollowerState state)
{
  switch (state)
  {
    case FollowerState::IDLE: return "IDLE";
    case FollowerState::TURN: return "TURN";
    case FollowerState::DRIVE: return "DRIVE";
    case FollowerState::ARRIVED: return "ARRIVED";
    case FollowerState::NO_PATH: return "NO_PATH";
    case FollowerState::LOCALIZATION_LOST: return "LOCALIZATION_LOST";
  }
  return "UNKNOWN";
}

WaypointFollower::WaypointFollower(const PlannerConfig& config) : config_(config)
{
}

void WaypointFollower::setPath(const core::PathResult& path)
{
  path_ = path;
  target_index_ = 0;
  state_ = FollowerState::IDLE;
}

void WaypointFollower::clearPath()
{
  path_ = core::PathResult();
  target_index_ = 0;
  state_ = FollowerState::IDLE;
}

void WaypointFollower::setPose(const ros::Time& stamp, double x, double y, double yaw)
{
  have_pose_ = true;
  pose_stamp_ = stamp;
  pose_x_ = x;
  pose_y_ = y;
  pose_yaw_ = yaw;
}

geometry_msgs::Twist WaypointFollower::update(const ros::Time& now)
{
  geometry_msgs::Twist command;

  if (!have_pose_ || (now - pose_stamp_).toSec() > config_.pose_timeout)
  {
    state_ = FollowerState::LOCALIZATION_LOST;
    return command;
  }
  if (!path_.found || path_.waypoints.empty())
  {
    state_ = FollowerState::NO_PATH;
    return command;
  }
  if (state_ == FollowerState::ARRIVED)
    return command;

  // Advance the target past any intermediate waypoint we have already reached.
  // The final waypoint is only considered reached by the ARRIVED check below.
  while (target_index_ + 1 < path_.waypoints.size() &&
         std::hypot(path_.waypoints[target_index_].x - pose_x_,
                    path_.waypoints[target_index_].y - pose_y_) < config_.position_tolerance)
    ++target_index_;

  const core::Waypoint& target = path_.waypoints[target_index_];
  const double dx = target.x - pose_x_;
  const double dy = target.y - pose_y_;
  const double dist = std::hypot(dx, dy);

  if (target_index_ + 1 >= path_.waypoints.size() && dist < config_.position_tolerance)
  {
    state_ = FollowerState::ARRIVED;
    return command;
  }

  const double target_yaw = std::atan2(dy, dx);
  const double heading_error = core::wrapToPi(target_yaw - pose_yaw_);

  if (std::abs(heading_error) > config_.heading_tolerance)
  {
    state_ = FollowerState::TURN;
    command.angular.z = clamp(-heading_error * kHeadingProportionalGain,
                              -config_.angular_velocity, config_.angular_velocity);
    return command;
  }

  state_ = FollowerState::DRIVE;
  double speed = config_.linear_velocity;
  if (dist < config_.slow_radius)
    speed = config_.linear_velocity * (dist / config_.slow_radius);
  command.linear.x = speed;
  command.angular.z = clamp(-heading_error * kHeadingProportionalGain,
                            -config_.angular_velocity, config_.angular_velocity);
  return command;
}

}  // namespace ros_adapter
}  // namespace laser_tag_nav_planner
