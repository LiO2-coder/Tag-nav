#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <tag_nav_planner/core/direction.h>
#include <tag_nav_planner/core/graph.h>

namespace tag_nav_planner
{
namespace core
{
namespace
{

std::pair<int, int> gridCell(double x, double y, double spacing)
{
  return {static_cast<int>(std::lround(x / spacing)),
          static_cast<int>(std::lround(y / spacing))};
}

bool isDiagonal(Direction direction)
{
  return direction == Direction::NE || direction == Direction::SE ||
         direction == Direction::SW || direction == Direction::NW;
}

}  // namespace

GraphBuildResult buildGraph(const std::map<int, NodePose>& nodes,
                            const std::map<int, uint8_t>& connectivity,
                            const PlannerOptions& options,
                            double grid_spacing)
{
  if (grid_spacing <= 0.0)
    throw std::runtime_error("grid_spacing must be positive");

  GraphBuildResult result;
  Graph& graph = result.graph;
  graph.nodes = nodes;

  std::map<std::pair<int, int>, int> index;
  for (const auto& entry : nodes)
    index[gridCell(entry.second.x, entry.second.y, grid_spacing)] = entry.first;

  const double diagonal_cost = std::sqrt(2.0) * grid_spacing;

  for (const auto& node : nodes)
  {
    const int id = node.first;
    const uint8_t mask = connectivity.count(id) ? connectivity.at(id) : 0u;
    for (int d = 0; d < kDirectionCount; ++d)
    {
      const Direction direction = static_cast<Direction>(d);
      if ((mask & directionBit(direction)) == 0)
        continue;
      if (isDiagonal(direction) && !options.allow_diagonal)
        continue;
      const Vec2d step = directionStep(direction);
      const std::pair<int, int> cell = gridCell(
          node.second.x + step.x * grid_spacing, node.second.y + step.y * grid_spacing,
          grid_spacing);
      const auto neighbor = index.find(cell);
      if (neighbor == index.end())
      {
        result.warnings.push_back("tag " + std::to_string(id) + " mask bit " +
                                  std::to_string(d) + " points at no mapped tag");
        continue;
      }
      Edge edge;
      edge.to = neighbor->second;
      edge.dir = direction;
      edge.cost = isDiagonal(direction) ? diagonal_cost : grid_spacing;
      graph.adjacency[id].push_back(edge);
    }
  }

  for (const auto& entry : connectivity)
    if (!nodes.count(entry.first))
      result.warnings.push_back("connectivity tag " + std::to_string(entry.first) +
                                " is absent from the tag map");

  return result;
}

}  // namespace core
}  // namespace tag_nav_planner
