#include <cmath>

#include <tag_nav_localization/core/pose_temporal_filter.h>
#include <tag_nav_localization/core/transform_math.h>

namespace tag_nav_localization
{
namespace core
{

PoseTemporalFilter::PoseTemporalFilter(const TemporalFilterConfig& config) : config_(config)
{
}

void PoseTemporalFilter::seed(const Transform& map_to_base, double timestamp_sec)
{
  x_ = map_to_base(0, 3);
  y_ = map_to_base(1, 3);
  z_ = map_to_base(2, 3);
  q_ = quaternionFromTransform(map_to_base);
  last_timestamp_sec_ = timestamp_sec;
  initialized_ = true;
}

Transform PoseTemporalFilter::filter(const Transform& map_to_base, double timestamp_sec)
{
  if (!initialized_)
  {
    seed(map_to_base, timestamp_sec);
    return map_to_base;
  }
  const double dt = timestamp_sec - last_timestamp_sec_;
  if (dt <= 0.0 || dt > config_.max_dt_sec)
  {
    seed(map_to_base, timestamp_sec);
    return map_to_base;
  }
  const double alpha_position = config_.position_time_constant_sec > 0.0
      ? 1.0 - std::exp(-dt / config_.position_time_constant_sec) : 1.0;
  const double alpha_orientation = config_.orientation_time_constant_sec > 0.0
      ? 1.0 - std::exp(-dt / config_.orientation_time_constant_sec) : 1.0;

  x_ += alpha_position * (map_to_base(0, 3) - x_);
  y_ += alpha_position * (map_to_base(1, 3) - y_);
  z_ += alpha_position * (map_to_base(2, 3) - z_);
  const Quaternion next = quaternionFromTransform(map_to_base);
  q_ = weightedQuaternion({q_, next}, {1.0 - alpha_orientation, alpha_orientation});
  last_timestamp_sec_ = timestamp_sec;
  return transformFromQuaternion(q_, x_, y_, z_);
}

void PoseTemporalFilter::reset()
{
  initialized_ = false;
  last_timestamp_sec_ = 0.0;
  x_ = 0.0;
  y_ = 0.0;
  z_ = 0.0;
  q_ = Quaternion();
}

}  // namespace core
}  // namespace tag_nav_localization
