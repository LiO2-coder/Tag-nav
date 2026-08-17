#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

#include <tag_nav_planner/core/astar.h>
#include <tag_nav_planner/core/direction.h>

namespace tag_nav_planner
{
namespace core
{
namespace
{

struct QueueEntry
{
  double f;
  double g;
  int id;
  bool operator>(const QueueEntry& other) const { return f > other.f; }
};

double heuristic(const NodePose& from, const NodePose& to, bool allow_diagonal)
{
  const double dx = std::abs(from.x - to.x);
  const double dy = std::abs(from.y - to.y);
  const double min_d = std::min(dx, dy);
  const double max_d = std::max(dx, dy);
  if (allow_diagonal)
    return std::sqrt(2.0) * min_d + (max_d - min_d);
  return dx + dy;
}

}  // namespace

PathResult planPath(const Graph& graph, int start_id, int goal_id,
                    const PlannerOptions& options)
{
  PathResult result;
  if (!graph.nodes.count(start_id) || !graph.nodes.count(goal_id))
    return result;

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, int> came_from;
  std::unordered_map<int, Direction> came_dir;
  std::unordered_map<int, bool> closed;

  const NodePose& goal_pose = graph.nodes.at(goal_id);
  g_score[start_id] = 0.0;
  open.push({heuristic(graph.nodes.at(start_id), goal_pose, options.allow_diagonal), 0.0,
             start_id});

  while (!open.empty())
  {
    const QueueEntry current = open.top();
    open.pop();
    if (closed[current.id])
      continue;
    closed[current.id] = true;
    if (current.id == goal_id)
      break;

    const auto edges = graph.adjacency.find(current.id);
    if (edges == graph.adjacency.end())
      continue;
    for (const Edge& edge : edges->second)
    {
      if (closed[edge.to])
        continue;
      const double tentative = current.g + edge.cost;
      const auto known = g_score.find(edge.to);
      if (known == g_score.end() || tentative < known->second)
      {
        g_score[edge.to] = tentative;
        came_from[edge.to] = current.id;
        came_dir[edge.to] = edge.dir;
        const double f = tentative +
            heuristic(graph.nodes.at(edge.to), goal_pose, options.allow_diagonal);
        open.push({f, tentative, edge.to});
      }
    }
  }

  if (start_id != goal_id && !came_from.count(goal_id))
    return result;

  std::vector<int> ids;
  int cursor = goal_id;
  while (true)
  {
    ids.push_back(cursor);
    if (cursor == start_id)
      break;
    cursor = came_from.at(cursor);
  }
  std::reverse(ids.begin(), ids.end());

  result.found = true;
  result.waypoints.reserve(ids.size());
  for (std::size_t i = 0; i < ids.size(); ++i)
  {
    Waypoint waypoint;
    waypoint.tag_id = ids[i];
    const NodePose& pose = graph.nodes.at(ids[i]);
    waypoint.x = pose.x;
    waypoint.y = pose.y;
    if (i + 1 < ids.size())
    {
      const NodePose& next = graph.nodes.at(ids[i + 1]);
      waypoint.heading_yaw = std::atan2(next.y - waypoint.y, next.x - waypoint.x);
      waypoint.direction = came_dir.at(ids[i + 1]);
    }
    else if (came_dir.count(ids[i]))
    {
      waypoint.direction = came_dir.at(ids[i]);
      waypoint.heading_yaw = directionYaw(waypoint.direction);
    }
    result.waypoints.push_back(waypoint);
  }
  return result;
}

}  // namespace core
}  // namespace tag_nav_planner
