#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

#include <jsoncpp/json/json.h>

#include <laser_tag_nav_planner/core/connectivity_loader.h>
#include <laser_tag_nav_planner/core/json_util.h>

namespace laser_tag_nav_planner
{
namespace core
{

std::map<int, uint8_t> loadConnectivity(const std::string& path)
{
  const Json::Value root = parseJsonFile(path, "connectivity JSON");
  if (root.isMember("schema_version") &&
      (!root["schema_version"].isInt() || root["schema_version"].asInt() != 1))
    throw std::runtime_error("unsupported connectivity JSON schema_version; expected 1");
  if (!root.isMember("connectivity") || !root["connectivity"].isObject())
    throw std::runtime_error("connectivity JSON needs a connectivity object");

  std::map<int, uint8_t> connectivity;
  const Json::Value& entries = root["connectivity"];
  for (const std::string& key : entries.getMemberNames())
  {
    const Json::Value& value = entries[key];
    if (!value.isInt() && !value.isUInt())
      throw std::runtime_error("connectivity mask for tag " + key + " must be an integer");
    const int mask = value.asInt();
    if (mask < 0 || mask > 255)
      throw std::runtime_error("connectivity mask for tag " + key + " must be in [0, 255]");
    connectivity[std::stoi(key)] = static_cast<uint8_t>(mask);
  }
  if (connectivity.empty())
    throw std::runtime_error("connectivity JSON contains no entries");
  return connectivity;
}

}  // namespace core
}  // namespace laser_tag_nav_planner
