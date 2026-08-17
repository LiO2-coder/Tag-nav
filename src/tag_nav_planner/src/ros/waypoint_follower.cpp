#include <algorithm>
#include <cmath>

#include <tag_nav_planner/core/direction.h>
#include <tag_nav_planner/ros/waypoint_follower.h>

namespace tag_nav_planner
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

void WaypointFollower::clearPose()
{
  have_pose_ = false;
}

double WaypointFollower::alongTrack(const core::Waypoint& waypoint) const
{
  const double dx = waypoint.x - pose_x_;
  const double dy = waypoint.y - pose_y_;
  return dx * std::cos(pose_yaw_) + dy * std::sin(pose_yaw_);
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

  // On the first tick after setPath, skip any waypoint the robot is already on
  // (e.g. the start tag under it) using Euclidean distance, since the robot has
  // not turned toward the first target yet.
  if (state_ == FollowerState::IDLE)
  {
    while (target_index_ + 1 < path_.waypoints.size() &&
           std::hypot(path_.waypoints[target_index_].x - pose_x_,
                      path_.waypoints[target_index_].y - pose_y_) <= config_.position_tolerance)
      ++target_index_;
    if (target_index_ + 1 >= path_.waypoints.size() &&
        std::hypot(path_.waypoints[target_index_].x - pose_x_,
                   path_.waypoints[target_index_].y - pose_y_) <= config_.position_tolerance)
    {
      state_ = FollowerState::ARRIVED;
      return command;
    }
    state_ = FollowerState::TURN;
  }

  const core::Waypoint& target = path_.waypoints[target_index_];
  const double dx = target.x - pose_x_;
  const double dy = target.y - pose_y_;
  const double along = alongTrack(target);
  const double heading_error = core::wrapToPi(std::atan2(dy, dx) - pose_yaw_);

  // Rotate in place until facing the target. Reached only right after stopping at
  // the previous tag (or at start), so the robot stops, turns, then drives.
  if (state_ == FollowerState::TURN)
  {
    if (std::abs(heading_error) > config_.heading_tolerance)
    {
      command.angular.z = clamp(heading_error * kHeadingProportionalGain,
                                -config_.angular_velocity, config_.angular_velocity);
      return command;
    }
    state_ = FollowerState::DRIVE;
  }

  // A segment ends when the target tag is no longer ahead of the robot (small
  // along-track distance), not when the robot is laterally centered on it.
  if (along <= config_.position_tolerance)
  {
    if (target_index_ + 1 >= path_.waypoints.size())
    {
      state_ = FollowerState::ARRIVED;
      return command;
    }
    ++target_index_;
    state_ = FollowerState::TURN;
    return command;  // zero velocity this tick; the next tick rotates in place
  }

  // Drive forward with a gentle heading correction to stay on course.
  command.linear.x = config_.linear_velocity;
  command.angular.z = clamp(heading_error * kHeadingProportionalGain,
                            -config_.angular_velocity, config_.angular_velocity);
  return command;
}

}  // namespace ros_adapter
}  // namespace tag_nav_planner
