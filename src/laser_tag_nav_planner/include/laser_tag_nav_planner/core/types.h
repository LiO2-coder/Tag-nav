#ifndef LASER_TAG_NAV_PLANNER_CORE_TYPES_H
#define LASER_TAG_NAV_PLANNER_CORE_TYPES_H

#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

namespace laser_tag_nav_planner
{
namespace core
{

// Compass direction on the 8-connected tag grid. Values are the bit index used
// in the connectivity mask (bit N = 1 << N).
enum class Direction : uint8_t
{
  N = 0, NE = 1, E = 2, SE = 3, S = 4, SW = 5, W = 6, NW = 7
};

struct Vec2d
{
  double x = 0.0;
  double y = 0.0;
};

// Position of a tag node. yaw records the tag's own in-map orientation and is
// not used as a driving heading.
struct NodePose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Waypoint
{
  int tag_id = -1;
  double x = 0.0;
  double y = 0.0;
  Direction direction = Direction::N;  // departure direction; arrival direction on the last node
  double heading_yaw = 0.0;            // heading toward the next node (atan2(dy, dx))
};

struct PathResult
{
  bool found = false;
  std::vector<Waypoint> waypoints;
};

struct PlannerOptions
{
  bool allow_diagonal = true;
};

struct Edge
{
  int to = -1;
  Direction dir = Direction::N;
  double cost = 0.0;
};

struct Graph
{
  std::map<int, NodePose> nodes;
  std::unordered_map<int, std::vector<Edge>> adjacency;
};

}  // namespace core
}  // namespace laser_tag_nav_planner

#endif
