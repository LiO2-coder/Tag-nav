#include <cstdint>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <laser_tag_nav_planner/PlanPath.h>
#include <laser_tag_nav_planner/core/astar.h>
#include <laser_tag_nav_planner/core/connectivity_loader.h>
#include <laser_tag_nav_planner/core/graph.h>
#include <laser_tag_nav_planner/core/tag_map_loader.h>
#include <laser_tag_nav_planner/ros/ros_planner_config.h>
#include <laser_tag_nav_planner/ros/waypoint_follower.h>

namespace laser_tag_nav_planner
{
namespace
{

class PlannerNode
{
public:
  PlannerNode()
    : nh_(), private_nh_("~"),
      config_(ros_adapter::RosPlannerConfigLoader(private_nh_).load()),
      follower_(config_)
  {
    const std::map<int, core::NodePose> nodes = core::loadTagMap(config_.tag_map_file);
    const std::map<int, uint8_t> connectivity = core::loadConnectivity(config_.connectivity_json);

    options_.allow_diagonal = config_.allow_diagonal;
    const core::GraphBuildResult build = core::buildGraph(
        nodes, connectivity, options_, config_.grid_spacing);
    graph_ = build.graph;

    for (const std::string& warning : build.warnings)
      ROS_WARN("%s", warning.c_str());
    ROS_INFO("Planner graph: %zu tag locations, %zu connectivity entries, %zu warnings",
             nodes.size(), connectivity.size(), build.warnings.size());

    path_publisher_ = nh_.advertise<nav_msgs::Path>("planner/path", 1, true);
    state_publisher_ = nh_.advertise<std_msgs::String>("planner/state", 1);
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    plan_service_ = nh_.advertiseService("planner/plan_path", &PlannerNode::planCallback, this);
    cancel_subscriber_ = nh_.subscribe("planner/cancel", 1, &PlannerNode::cancelCallback, this);

    if (config_.auto_start)
    {
      const core::PathResult path =
          core::planPath(graph_, config_.start_tag, config_.goal_tag, options_);
      follower_.setPath(path);
      publishPath(path);
      if (path.found)
      {
        ROS_INFO("Planned %zu waypoints from tag %d to tag %d",
                 path.waypoints.size(), config_.start_tag, config_.goal_tag);
      }
      else
      {
        ROS_WARN("No path from tag %d to tag %d", config_.start_tag, config_.goal_tag);
      }
    }
    else
    {
      follower_.clearPath();
      ROS_INFO("Planner idle (auto_start=false); waiting for /planner/plan_path");
    }

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.controller_rate),
                                     &PlannerNode::controlTick, this);
  }

private:
  void controlTick(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    applyPendingPlan();
    updatePose(now);
    cmd_vel_publisher_.publish(follower_.update(now));
    publishState();
  }

  // Reads the combined EKF+tag pose straight from TF (map -> base_footprint),
  // exactly like move_base's costmaps do.
  void updatePose(const ros::Time& now)
  {
    tf::StampedTransform map_to_base;
    try
    {
      tf_listener_.lookupTransform(config_.map_frame, config_.base_frame, ros::Time(0),
                                   map_to_base);
      follower_.setPose(now, map_to_base.getOrigin().x(), map_to_base.getOrigin().y(),
                        tf::getYaw(map_to_base.getRotation()));
    }
    catch (const tf::TransformException& error)
    {
      ROS_WARN_THROTTLE(2.0, "TF lookup %s -> %s failed: %s", config_.map_frame.c_str(),
                        config_.base_frame.c_str(), error.what());
      follower_.clearPose();
    }
  }

  // Applies a plan/cancel queued by the service or cancel callback. The follower
  // is only mutated here on the timer thread, so no lock guards setPath/update.
  void applyPendingPlan()
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if (has_pending_cancel_)
    {
      follower_.clearPath();
      has_pending_cancel_ = false;
      has_pending_path_ = false;
    }
    else if (has_pending_path_)
    {
      follower_.setPath(pending_path_);
      publishPath(pending_path_);
      has_pending_path_ = false;
    }
  }

  nav_msgs::Path makePathMessage(const core::PathResult& path) const
  {
    nav_msgs::Path message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = config_.map_frame;
    for (const core::Waypoint& waypoint : path.waypoints)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint.heading_yaw);
      message.poses.push_back(pose);
    }
    return message;
  }

  void publishPath(const core::PathResult& path)
  {
    path_publisher_.publish(makePathMessage(path));
  }

  bool planCallback(laser_tag_nav_planner::PlanPath::Request& request,
                    laser_tag_nav_planner::PlanPath::Response& response)
  {
    if (!graph_.nodes.count(request.start_tag) || !graph_.nodes.count(request.goal_tag))
    {
      response.success = false;
      response.message = "start_tag or goal_tag not present in the tag map";
      return true;
    }
    const core::PathResult path =
        core::planPath(graph_, request.start_tag, request.goal_tag, options_);
    response.path = makePathMessage(path);
    response.success = path.found;
    response.message = path.found ? "" : "no path found between the given tags";
    if (path.found)
    {
      std::lock_guard<std::mutex> lock(plan_mutex_);
      pending_path_ = path;
      has_pending_path_ = true;
    }
    return true;
  }

  void cancelCallback(const std_msgs::Empty::ConstPtr&)
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    has_pending_cancel_ = true;
  }

  void publishState()
  {
    std_msgs::String message;
    message.data = ros_adapter::toString(follower_.state());
    state_publisher_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros_adapter::PlannerConfig config_;
  ros_adapter::WaypointFollower follower_;
  core::Graph graph_;
  core::PlannerOptions options_;

  ros::Publisher path_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher cmd_vel_publisher_;
  ros::ServiceServer plan_service_;
  ros::Subscriber cancel_subscriber_;
  ros::Timer control_timer_;

  tf::TransformListener tf_listener_;

  std::mutex plan_mutex_;  // guards the pending plan/cancel handoff to the control tick
  bool has_pending_path_ = false;
  core::PathResult pending_path_;
  bool has_pending_cancel_ = false;
};

}  // namespace
}  // namespace laser_tag_nav_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_tag_nav_planner");
  try
  {
    laser_tag_nav_planner::PlannerNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Planner failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
