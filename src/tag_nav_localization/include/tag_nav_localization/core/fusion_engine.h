#ifndef TAG_NAV_LOCALIZATION_CORE_FUSION_ENGINE_H
#define TAG_NAV_LOCALIZATION_CORE_FUSION_ENGINE_H

#include <tag_nav_localization/core/types.h>

namespace tag_nav_localization
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
}  // namespace tag_nav_localization

#endif
