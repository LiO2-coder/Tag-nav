#include <stdexcept>
#include <string>

#include <ros/package.h>

#include <laser_tag_nav_planner/ros/ros_planner_config.h>

namespace laser_tag_nav_planner
{
namespace ros_adapter
{
namespace
{

std::string resolvePackageUri(const std::string& uri)
{
  const std::string prefix = "package://";
  if (uri.compare(0, prefix.size(), prefix) != 0)
    return uri;
  const std::string remainder = uri.substr(prefix.size());
  const std::size_t slash = remainder.find('/');
  if (slash == std::string::npos || slash == 0 || slash + 1 >= remainder.size())
    throw std::runtime_error("package URI must be package://<package>/<relative-path>: " + uri);
  const std::string package_name = remainder.substr(0, slash);
  const std::string package_path = ros::package::getPath(package_name);
  if (package_path.empty())
    throw std::runtime_error("unable to resolve package URI: " + uri);
  return package_path + "/" + remainder.substr(slash + 1);
}

}  // namespace

RosPlannerConfigLoader::RosPlannerConfigLoader(const ros::NodeHandle& private_node_handle)
  : private_node_handle_(private_node_handle)
{
}

PlannerConfig RosPlannerConfigLoader::load() const
{
  PlannerConfig config;
  if (!private_node_handle_.getParam("start_tag", config.start_tag))
    throw std::runtime_error("required private parameter ~start_tag is missing");
  if (!private_node_handle_.getParam("goal_tag", config.goal_tag))
    throw std::runtime_error("required private parameter ~goal_tag is missing");
  if (config.start_tag < 0 || config.goal_tag < 0)
    throw std::runtime_error("start_tag and goal_tag must be non-negative");

  private_node_handle_.param<std::string>("connectivity_json", config.connectivity_json, "");
  if (config.connectivity_json.empty())
    throw std::runtime_error("required private parameter ~connectivity_json is missing");

  private_node_handle_.param<std::string>("tag_map_file", config.tag_map_file,
      "package://laser-tag_nav_bringup/worlds/maps/apriltagMap.json");
  private_node_handle_.param<std::string>("map_frame", config.map_frame, config.map_frame);
  private_node_handle_.param<std::string>("base_frame", config.base_frame, config.base_frame);
  private_node_handle_.param("linear_velocity", config.linear_velocity, config.linear_velocity);
  private_node_handle_.param("angular_velocity", config.angular_velocity, config.angular_velocity);
  private_node_handle_.param("position_tolerance", config.position_tolerance,
                             config.position_tolerance);
  private_node_handle_.param("heading_tolerance", config.heading_tolerance,
                             config.heading_tolerance);
  private_node_handle_.param("slow_radius", config.slow_radius, config.slow_radius);
  private_node_handle_.param("allow_diagonal", config.allow_diagonal, config.allow_diagonal);
  private_node_handle_.param("grid_spacing", config.grid_spacing, config.grid_spacing);
  private_node_handle_.param("controller_rate", config.controller_rate, config.controller_rate);
  private_node_handle_.param("pose_timeout", config.pose_timeout, config.pose_timeout);
  private_node_handle_.param("auto_start", config.auto_start, config.auto_start);

  if (config.linear_velocity < 0.0 || config.angular_velocity < 0.0)
    throw std::runtime_error("linear_velocity and angular_velocity must be non-negative");
  if (config.position_tolerance < 0.0 || config.heading_tolerance < 0.0)
    throw std::runtime_error("position_tolerance and heading_tolerance must be non-negative");
  if (config.grid_spacing <= 0.0 || config.controller_rate <= 0.0 || config.pose_timeout <= 0.0)
    throw std::runtime_error("grid_spacing, controller_rate and pose_timeout must be positive");

  config.connectivity_json = resolvePackageUri(config.connectivity_json);
  config.tag_map_file = resolvePackageUri(config.tag_map_file);
  return config;
}

}  // namespace ros_adapter
}  // namespace laser_tag_nav_planner
