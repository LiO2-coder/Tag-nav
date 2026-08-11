#ifndef LASER_TAG_NAV_LOCALIZATION_QUALITY_MATH_H
#define LASER_TAG_NAV_LOCALIZATION_QUALITY_MATH_H

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace laser_tag_nav_localization
{

struct QualityScores
{
  double area_px = 0.0;
  double area = 0.0;
  double distortion_error = 0.0;
  double distortion = 0.0;
  double decision_margin = 0.0;
  double margin = 0.0;
  double corner_gradient = 0.0;
  double sharpness = 0.0;
  double quality = 0.0;
};

inline double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

inline double polygonArea(const std::array<cv::Point2d, 4>& corners)
{
  double twice_area = 0.0;
  for (size_t i = 0; i < corners.size(); ++i)
  {
    const cv::Point2d& a = corners[i];
    const cv::Point2d& b = corners[(i + 1) % corners.size()];
    twice_area += a.x * b.y - b.x * a.y;
  }
  return std::abs(twice_area) * 0.5;
}

inline double quadrilateralDistortionError(const std::array<cv::Point2d, 4>& corners)
{
  std::array<double, 4> sides{};
  for (size_t i = 0; i < sides.size(); ++i)
    sides[i] = cv::norm(corners[(i + 1) % 4] - corners[i]);

  const double side_mean = std::accumulate(sides.begin(), sides.end(), 0.0) / 4.0;
  if (side_mean <= 1e-9)
    return 1e9;

  double side_sq = 0.0;
  for (double side : sides)
    side_sq += (side - side_mean) * (side - side_mean);
  const double side_cv = std::sqrt(side_sq / 4.0) / side_mean;

  const double diagonal_a = cv::norm(corners[2] - corners[0]);
  const double diagonal_b = cv::norm(corners[3] - corners[1]);
  const double diagonal_mean = 0.5 * (diagonal_a + diagonal_b);
  const double diagonal_error = diagonal_mean > 1e-9
      ? std::abs(diagonal_a - diagonal_b) / diagonal_mean : 1e9;

  double angle_error = 0.0;
  for (size_t i = 0; i < 4; ++i)
  {
    const cv::Point2d incoming = corners[(i + 3) % 4] - corners[i];
    const cv::Point2d outgoing = corners[(i + 1) % 4] - corners[i];
    const double incoming_norm = cv::norm(incoming);
    const double outgoing_norm = cv::norm(outgoing);
    if (incoming_norm <= 1e-9 || outgoing_norm <= 1e-9)
    {
      angle_error += 1.0;
      continue;
    }
    const double cosine = std::max(-1.0, std::min(1.0,
        (incoming.dot(outgoing)) / (incoming_norm * outgoing_norm)));
    angle_error += std::abs(std::acos(cosine) - M_PI / 2.0) / (M_PI / 2.0);
  }
  angle_error /= 4.0;

  return (side_cv + diagonal_error + angle_error) / 3.0;
}

inline double geometricQuality(double area_score, double distortion_score,
                               double margin_score, double sharpness_score,
                               const std::array<double, 4>& exponents =
                                   {{0.25, 0.25, 0.25, 0.25}})
{
  const std::array<double, 4> scores = {{
      clamp01(area_score), clamp01(distortion_score),
      clamp01(margin_score), clamp01(sharpness_score)}};
  double quality = 1.0;
  for (size_t i = 0; i < scores.size(); ++i)
  {
    if (scores[i] <= 0.0 && exponents[i] > 0.0)
      return 0.0;
    quality *= std::pow(std::max(scores[i], 1e-12), exponents[i]);
  }
  return clamp01(quality);
}

inline double geometricQuality(const std::array<double, 4>& scores,
                               const std::array<double, 4>& exponents)
{
  return geometricQuality(scores[0], scores[1], scores[2], scores[3], exponents);
}

inline double wrapAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

inline double weightedYaw(const std::vector<double>& yaws,
                          const std::vector<double>& weights)
{
  double sine = 0.0;
  double cosine = 0.0;
  const size_t count = std::min(yaws.size(), weights.size());
  for (size_t i = 0; i < count; ++i)
  {
    sine += weights[i] * std::sin(yaws[i]);
    cosine += weights[i] * std::cos(yaws[i]);
  }
  return std::atan2(sine, cosine);
}

inline bool validQualityMask(const std::string& mask)
{
  return mask.size() == 4 && mask.find_first_not_of("01") == std::string::npos && mask != "0000";
}

inline std::array<double, 4> normalizedQualityExponents(
    const std::string& mask, const std::array<double, 4>& requested)
{
  if (!validQualityMask(mask))
    throw std::invalid_argument("quality mask must be a non-zero four-bit string");
  std::array<double, 4> output = requested;
  double sum = 0.0;
  for (size_t i = 0; i < output.size(); ++i)
  {
    if (output[i] < 0.0)
      throw std::invalid_argument("quality exponents cannot be negative");
    if (mask[i] == '0')
      output[i] = 0.0;
    sum += output[i];
  }
  if (sum <= 0.0)
    throw std::invalid_argument("quality mask enables no positive exponent");
  for (double& exponent : output)
    exponent /= sum;
  return output;
}

}  // namespace laser_tag_nav_localization

#endif
