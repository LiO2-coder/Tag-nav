#ifndef TAG_NAV_PLANNER_CORE_ASTAR_H
#define TAG_NAV_PLANNER_CORE_ASTAR_H

#include <tag_nav_planner/core/types.h>

namespace tag_nav_planner
{
namespace core
{

// A* shortest path over the 8-connected tag graph. Returns a found=false result
// when the start or goal node is absent, or when no path exists. The
// allow_diagonal option must match the one used to build the graph so the
// heuristic stays admissible.
PathResult planPath(const Graph& graph, int start_id, int goal_id,
                    const PlannerOptions& options);

}  // namespace core
}  // namespace tag_nav_planner

#endif
