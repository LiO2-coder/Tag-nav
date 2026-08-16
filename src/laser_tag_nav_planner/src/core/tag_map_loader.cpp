#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <jsoncpp/json/json.h>

#include <laser_tag_nav_planner/core/json_util.h>
#include <laser_tag_nav_planner/core/tag_map_loader.h>

namespace laser_tag_nav_planner
{
namespace core
{

std::map<int, NodePose> loadTagMap(const std::string& path)
{
  const Json::Value root = parseJsonFile(path, "tag map JSON");
  if (root.get("map_type", "").asString() != "2d")
    throw std::runtime_error("tag map map_type must be \"2d\" for the planner");
  if (!root.isMember("tag_locations") || !root["tag_locations"].isObject())
    throw std::runtime_error("tag map needs a tag_locations object");

  std::map<int, NodePose> nodes;
  const Json::Value& locations = root["tag_locations"];
  for (const std::string& key : locations.getMemberNames())
  {
    const std::vector<double> location = numberArray(locations[key], "tag location " + key);
    if (location.size() != 3)
      throw std::runtime_error("tag location " + key + " must be [x, y, yaw]");
    NodePose pose;
    pose.x = location[0];
    pose.y = location[1];
    pose.yaw = location[2];
    nodes[std::stoi(key)] = pose;
  }
  if (nodes.empty())
    throw std::runtime_error("tag map contains no locations");
  return nodes;
}

}  // namespace core
}  // namespace laser_tag_nav_planner
