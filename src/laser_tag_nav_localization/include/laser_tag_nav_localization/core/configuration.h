#ifndef LASER_TAG_NAV_LOCALIZATION_CORE_CONFIGURATION_H
#define LASER_TAG_NAV_LOCALIZATION_CORE_CONFIGURATION_H

#include <string>

#include <laser_tag_nav_localization/core/types.h>

namespace laser_tag_nav_localization
{
namespace core
{

LocalizationConfig parseConfiguration(const std::string& cameras_json,
                                      const std::string& tag_map_override,
                                      const UriResolver& resolve_uri);
void loadTagMap(LocalizationConfig& config);
void validateConfiguration(LocalizationConfig& config);

}  // namespace core
}  // namespace laser_tag_nav_localization

#endif
