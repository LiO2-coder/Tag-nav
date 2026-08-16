#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/common/zarray.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

#include <laser_tag_nav_localization/core/apriltag_recognizer.h>
#include <laser_tag_nav_localization/core/transform_math.h>

namespace laser_tag_nav_localization
{
namespace core
{
namespace
{

struct FamilyHandle
{
  apriltag_family_t* family = nullptr;
  void (*destroy)(apriltag_family_t*) = nullptr;
};

FamilyHandle createFamily(const std::string& family_name)
{
  FamilyHandle handle;
#define APRILTAG_FAMILY(name) \
  if (family_name == #name) { handle.family = name##_create(); handle.destroy = name##_destroy; }
  APRILTAG_FAMILY(tag16h5)
  else APRILTAG_FAMILY(tag25h9)
  else APRILTAG_FAMILY(tag36h10)
  else APRILTAG_FAMILY(tag36h11)
  else APRILTAG_FAMILY(tagCircle21h7)
  else APRILTAG_FAMILY(tagCircle49h12)
  else APRILTAG_FAMILY(tagCustom48h12)
  else APRILTAG_FAMILY(tagStandard41h12)
  else APRILTAG_FAMILY(tagStandard52h13)
#undef APRILTAG_FAMILY
  if (!handle.family)
    throw std::runtime_error(
        "unsupported or unavailable AprilTag family: '" + family_name + "'. Supported "
        "families (apriltag 0.10.0): tag16h5, tag25h9, tag36h10, tag36h11, tagCircle21h7, "
        "tagCircle49h12, tagCustom48h12, tagStandard41h12, tagStandard52h13. Custom "
        "families require recompiling this node.");
  return handle;
}

}  // namespace

struct AprilTagRecognizer::Impl
{
  struct CameraCache
  {
    cv::Mat map1;
    cv::Mat map2;
    cv::Size map_size;
  };

  LocalizationConfig config;
  FamilyHandle family;
  apriltag_detector_t* detector = nullptr;
  std::vector<CameraCache> camera_cache;

  explicit Impl(const LocalizationConfig& value) : config(value), camera_cache(value.cameras.size())
  {
    family = createFamily(config.detector.tag_family);
    detector = apriltag_detector_create();
    if (!detector)
      throw std::runtime_error("apriltag_detector_create failed");
    apriltag_detector_add_family_bits(detector, family.family, config.detector.max_hamming_dist);
    detector->nthreads = std::max(1, config.detector.threads);
    detector->quad_decimate = static_cast<float>(std::max(0.1, config.detector.decimate));
    detector->quad_sigma = static_cast<float>(config.detector.blur);
    detector->refine_edges = config.detector.refine_edges ? 1 : 0;
  }

  ~Impl()
  {
    if (detector)
      apriltag_detector_destroy(detector);
    if (family.family && family.destroy)
      family.destroy(family.family);
  }

  void ensureRectification(std::size_t index, const cv::Size& size)
  {
    CameraCache& cache = camera_cache[index];
    const CameraModel& camera = config.cameras[index];
    if (cache.map_size == size)
      return;
    cache.map1.release();
    cache.map2.release();
    const bool has_distortion = !camera.D.empty() && camera.D.total() > 0;
    if (has_distortion || camera.distortion_model == "equidistant" ||
        camera.distortion_model == "fisheye")
    {
      const cv::Mat K = cv::Mat(camera.K).clone();
      const cv::Mat new_K = cv::Mat(camera.K).clone();
      if (camera.distortion_model == "equidistant" || camera.distortion_model == "fisheye")
      {
        cv::Mat D = camera.D;
        if (D.empty())
          D = cv::Mat::zeros(1, 4, CV_64F);
        cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat::eye(3, 3, CV_64F),
                                             new_K, size, CV_32FC1, cache.map1, cache.map2);
      }
      else
      {
        cv::initUndistortRectifyMap(K, camera.D, cv::Mat::eye(3, 3, CV_64F),
                                    new_K, size, CV_32FC1, cache.map1, cache.map2);
      }
    }
    cache.map_size = size;
  }

  double cornerSharpness(const cv::Mat& gray,
                         const std::array<cv::Point2d, 4>& corners,
                         cv::Mat& gradient) const
  {
    if (gradient.empty())
    {
      cv::Mat gx, gy;
      cv::Sobel(gray, gx, CV_32F, 1, 0, 3);
      cv::Sobel(gray, gy, CV_32F, 0, 1, 3);
      cv::magnitude(gx, gy, gradient);
    }
    double total = 0.0;
    int samples = 0;
    constexpr int radius = 4;
    for (const cv::Point2d& corner : corners)
    {
      const int x = static_cast<int>(std::round(corner.x));
      const int y = static_cast<int>(std::round(corner.y));
      const cv::Rect image_rect(0, 0, gradient.cols, gradient.rows);
      const cv::Rect roi = cv::Rect(x - radius, y - radius, 2 * radius + 1, 2 * radius + 1) & image_rect;
      if (roi.empty())
        continue;
      total += cv::mean(gradient(roi))[0] * roi.area();
      samples += roi.area();
    }
    return samples > 0 ? total / samples : 0.0;
  }

  bool mapped(int tag_id) const
  {
    return config.tag_map.find(tag_id) != config.tag_map.end();
  }

  // Fills corners, tag id, hamming and every quality score. Does not require a
  // map entry or a tag size, so it can report tags that are detected but not
  // mapped. Pose fields (camera_to_tag, range_m, map_to_base) stay untouched.
  void computeScores(const cv::Mat& gray, cv::Mat& gradient,
                     apriltag_detection_t* detection, std::size_t camera_index,
                     TagCandidate& candidate) const
  {
    const CameraModel& camera = config.cameras[camera_index];
    for (std::size_t index = 0; index < 4; ++index)
      candidate.corners[index] = cv::Point2d(detection->p[index][0], detection->p[index][1]);
    candidate.tag_id = detection->id;
    candidate.hamming = detection->hamming;
    candidate.scores.area_px = polygonArea(candidate.corners);
    candidate.scores.area = clamp01(candidate.scores.area_px / config.quality.area_reference_px);
    candidate.scores.distortion_error = quadrilateralDistortionError(candidate.corners);
    candidate.scores.distortion = std::exp(-candidate.scores.distortion_error /
                                           std::max(1e-6, config.quality.distortion_scale));
    candidate.scores.decision_margin = detection->decision_margin;
    candidate.scores.margin = clamp01(detection->decision_margin / config.quality.margin_reference);
    candidate.scores.corner_gradient = cornerSharpness(gray, candidate.corners, gradient);
    candidate.scores.sharpness = clamp01(candidate.scores.corner_gradient /
                                          config.quality.sharpness_reference);
    candidate.scores.quality = geometricQuality(
        {{candidate.scores.area, candidate.scores.distortion, candidate.scores.margin,
          candidate.scores.sharpness}}, config.quality.metric_exponents);
    candidate.weighted_quality = candidate.scores.quality * camera.confidence_multiplier;
  }

  bool makeCandidate(const cv::Mat& gray, cv::Mat& gradient,
                     apriltag_detection_t* detection, std::size_t camera_index,
                     TagCandidate& candidate) const
  {
    const CameraModel& camera = config.cameras[camera_index];
    const auto map_it = config.tag_map.find(detection->id);
    if (map_it == config.tag_map.end() || detection->hamming > config.detector.max_hamming_dist)
      return false;
    computeScores(gray, gradient, detection, camera_index, candidate);

    const double half_size = map_it->second.size / 2.0;
    std::vector<cv::Point3d> object_points{
        cv::Point3d(-half_size, half_size, 0.0), cv::Point3d(half_size, half_size, 0.0),
        cv::Point3d(half_size, -half_size, 0.0), cv::Point3d(-half_size, -half_size, 0.0)};
    const std::vector<cv::Point2d> image_points(candidate.corners.begin(), candidate.corners.end());
    cv::Mat rvec, tvec;
    bool solved = cv::solvePnP(object_points, image_points, cv::Mat(camera.K), cv::Mat(),
                               rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
    if (!solved)
      solved = cv::solvePnP(object_points, image_points, cv::Mat(camera.K), cv::Mat(),
                            rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!solved)
      return false;

    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    candidate.camera_to_tag = Transform::eye();
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        candidate.camera_to_tag(row, col) = rotation.at<double>(row, col);
    candidate.camera_to_tag(0, 3) = tvec.at<double>(0);
    candidate.camera_to_tag(1, 3) = tvec.at<double>(1);
    candidate.camera_to_tag(2, 3) = tvec.at<double>(2);
    candidate.range_m = cv::norm(cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
    if (config.validation.max_tag_range_m > 0.0 &&
        candidate.range_m > config.validation.max_tag_range_m)
      return false;
    for (const cv::Point3d& point : object_points)
    {
      const double depth = candidate.camera_to_tag(2, 0) * point.x +
          candidate.camera_to_tag(2, 1) * point.y + candidate.camera_to_tag(2, 3);
      if (depth <= 1e-6)
        return false;
    }

    Transform pnp_tag_to_map_tag = Transform::eye();
    pnp_tag_to_map_tag(1, 1) = -1.0;
    pnp_tag_to_map_tag(2, 2) = -1.0;
    candidate.camera_to_tag = candidate.camera_to_tag * pnp_tag_to_map_tag;
    if (config.validation.min_tag_area_px > 0.0 &&
        candidate.scores.area_px < config.validation.min_tag_area_px)
      return false;
    if (candidate.weighted_quality < config.quality.min_quality)
      return false;
    candidate.map_to_base = map_it->second.map_to_tag * inverseRigid(candidate.camera_to_tag) *
                            inverseRigid(camera.base_to_camera);
    return true;
  }
};

AprilTagRecognizer::AprilTagRecognizer(const LocalizationConfig& config)
  : impl_(new Impl(config))
{
}

AprilTagRecognizer::~AprilTagRecognizer() = default;

cv::Mat AprilTagRecognizer::rectify(const cv::Mat& gray, const CameraModel& camera)
{
  std::size_t index = 0;
  for (; index < impl_->config.cameras.size(); ++index)
    if (&impl_->config.cameras[index] == &camera || impl_->config.cameras[index].name == camera.name)
      break;
  if (index >= impl_->config.cameras.size())
    throw std::runtime_error("camera is not part of recognizer configuration");
  impl_->ensureRectification(index, gray.size());
  const auto& cache = impl_->camera_cache[index];
  if (cache.map1.empty())
    return gray;
  cv::Mat rectified;
  cv::remap(gray, rectified, cache.map1, cache.map2, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar(0));
  return rectified;
}

CameraObservation AprilTagRecognizer::recognize(const cv::Mat& gray, std::size_t camera_index)
{
  if (camera_index >= impl_->config.cameras.size())
    throw std::out_of_range("camera index out of range");
  CameraObservation result;
  result.camera_index = camera_index;
  if (gray.empty() || gray.type() != CV_8UC1)
  {
    result.status = "image_processing_error";
    return result;
  }
  image_u8_t image = {gray.cols, gray.rows, static_cast<int32_t>(gray.step[0]), gray.data};
  zarray_t* detections = apriltag_detector_detect(impl_->detector, &image);
  cv::Mat gradient;
  bool have_candidate = false;
  bool have_unmapped = false;
  TagCandidate best;
  TagCandidate best_unmapped;
  for (int index = 0; index < zarray_size(detections); ++index)
  {
    apriltag_detection_t* detection = nullptr;
    zarray_get(detections, index, &detection);
    if (impl_->mapped(detection->id))
    {
      TagCandidate candidate;
      if (impl_->makeCandidate(gray, gradient, detection, camera_index, candidate) &&
          (!have_candidate || candidate.scores.quality > best.scores.quality))
      {
        best = candidate;
        have_candidate = true;
      }
    }
    else if (!impl_->config.validation.reject_unmapped_tags &&
             detection->hamming <= impl_->config.detector.max_hamming_dist)
    {
      TagCandidate candidate;
      impl_->computeScores(gray, gradient, detection, camera_index, candidate);
      if (!have_unmapped || candidate.scores.quality > best_unmapped.scores.quality)
      {
        best_unmapped = candidate;
        have_unmapped = true;
      }
    }
  }
  apriltag_detections_destroy(detections);
  if (have_candidate)
  {
    result.has_candidate = true;
    result.status = "valid";
    result.candidate = best;
  }
  else if (have_unmapped)
  {
    result.has_candidate = false;
    result.status = "unmapped_tag";
    result.candidate = best_unmapped;
  }
  else
  {
    result.status = "no_mapped_tag";
  }
  return result;
}

}  // namespace core
}  // namespace laser_tag_nav_localization
