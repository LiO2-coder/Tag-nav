#ifndef LASER_TAG_NAV_LOCALIZATION_CORE_POSE_TEMPORAL_FILTER_H
#define LASER_TAG_NAV_LOCALIZATION_CORE_POSE_TEMPORAL_FILTER_H

#include <laser_tag_nav_localization/core/types.h>

namespace laser_tag_nav_localization
{
namespace core
{

// Exponential (low-pass) smoother over successive fused poses. Position is
// smoothed per-axis; orientation is blended with the sign-aligned
// weightedQuaternion so planar yaw and full 3D orientation are both handled.
class PoseTemporalFilter
{
public:
  explicit PoseTemporalFilter(const TemporalFilterConfig& config);
  Transform filter(const Transform& map_to_base, double timestamp_sec);
  void reset();

private:
  void seed(const Transform& map_to_base, double timestamp_sec);

  TemporalFilterConfig config_;
  bool initialized_ = false;
  double last_timestamp_sec_ = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  Quaternion q_;
};

}  // namespace core
}  // namespace laser_tag_nav_localization

#endif
