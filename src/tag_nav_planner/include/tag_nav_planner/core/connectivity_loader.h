#ifndef TAG_NAV_PLANNER_CORE_CONNECTIVITY_LOADER_H
#define TAG_NAV_PLANNER_CORE_CONNECTIVITY_LOADER_H

#include <cstdint>
#include <map>
#include <string>

namespace tag_nav_planner
{
namespace core
{

// Reads the planner connectivity JSON (id -> 8-bit direction mask) and returns
// id -> mask. Throws std::runtime_error on malformed input.
std::map<int, uint8_t> loadConnectivity(const std::string& path);

}  // namespace core
}  // namespace tag_nav_planner

#endif
