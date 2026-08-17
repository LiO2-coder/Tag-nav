#ifndef TAG_NAV_LOCALIZATION_CORE_APRILTAG_RECOGNIZER_H
#define TAG_NAV_LOCALIZATION_CORE_APRILTAG_RECOGNIZER_H

#include <map>
#include <memory>

#include <tag_nav_localization/core/types.h>

namespace tag_nav_localization
{
namespace core
{

class AprilTagRecognizer
{
public:
  AprilTagRecognizer(const LocalizationConfig& config);
  ~AprilTagRecognizer();
  AprilTagRecognizer(const AprilTagRecognizer&) = delete;
  AprilTagRecognizer& operator=(const AprilTagRecognizer&) = delete;

  cv::Mat rectify(const cv::Mat& gray, const CameraModel& camera);
  CameraObservation recognize(const cv::Mat& rectified_gray, std::size_t camera_index);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace core
}  // namespace tag_nav_localization

#endif
