#ifndef LASER_TAG_NAV_PLANNER_CORE_GRAPH_H
#define LASER_TAG_NAV_PLANNER_CORE_GRAPH_H

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <laser_tag_nav_planner/core/types.h>

namespace laser_tag_nav_planner
{
namespace core
{

struct GraphBuildResult
{
  Graph graph;
  std::vector<std::string> warnings;
};

// Builds the directed tag graph from node positions and per-node 8-bit masks.
// A set mask bit adds an edge to the neighbor one grid step away in that
// direction. Diagonal edges are skipped when !options.allow_diagonal. Mask bits
// pointing at no mapped tag (or a tag id absent from the map) produce warnings
// instead of edges.
GraphBuildResult buildGraph(const std::map<int, NodePose>& nodes,
                            const std::map<int, uint8_t>& connectivity,
                            const PlannerOptions& options,
                            double grid_spacing = 1.0);

}  // namespace core
}  // namespace laser_tag_nav_planner

#endif
