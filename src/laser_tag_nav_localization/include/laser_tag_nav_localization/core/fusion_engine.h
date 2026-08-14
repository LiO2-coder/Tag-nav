#ifndef LASER_TAG_NAV_LOCALIZATION_CORE_FUSION_ENGINE_H
#define LASER_TAG_NAV_LOCALIZATION_CORE_FUSION_ENGINE_H

#include <laser_tag_nav_localization/core/types.h>

namespace laser_tag_nav_localization
{
namespace core
{

class FusionEngine
{
public:
  FusionEngine(const LocalizationConfig& config);
  FusionResult fuse(const std::vector<CameraObservation>& observations) const;

private:
  LocalizationConfig config_;
};

}  // namespace core
}  // namespace laser_tag_nav_localization

#endif
