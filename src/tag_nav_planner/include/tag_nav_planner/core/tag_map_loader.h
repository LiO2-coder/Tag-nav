#ifndef TAG_NAV_PLANNER_CORE_TAG_MAP_LOADER_H
#define TAG_NAV_PLANNER_CORE_TAG_MAP_LOADER_H

#include <map>
#include <string>

#include <tag_nav_planner/core/types.h>

namespace tag_nav_planner
{
namespace core
{

// Reads an apriltagMap.json (map_type "2d", tag_locations id -> [x, y, yaw])
// and returns id -> position. Only x and y are consumed; yaw is retained for
// future use. Throws std::runtime_error on malformed input.
std::map<int, NodePose> loadTagMap(const std::string& path);

}  // namespace core
}  // namespace tag_nav_planner

#endif
