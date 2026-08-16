#ifndef LASER_TAG_NAV_PLANNER_CORE_JSON_UTIL_H
#define LASER_TAG_NAV_PLANNER_CORE_JSON_UTIL_H

#include <string>
#include <vector>

#include <jsoncpp/json/json.h>

namespace laser_tag_nav_planner
{
namespace core
{

// Reads and parses a JSON file, throwing std::runtime_error on failure.
Json::Value parseJsonFile(const std::string& path, const std::string& source);

// Reads a JSON array of numbers, throwing std::runtime_error on type errors.
std::vector<double> numberArray(const Json::Value& value, const std::string& name);

}  // namespace core
}  // namespace laser_tag_nav_planner

#endif
