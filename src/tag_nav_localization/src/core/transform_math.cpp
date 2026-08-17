#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <tag_nav_localization/core/transform_math.h>

namespace tag_nav_localization
{
namespace core
{

namespace
{
constexpr double kEpsilon = 1e-12;
}

Transform inverseRigid(const Transform& transform)
{
  Transform inverse = Transform::eye();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      inverse(row, col) = transform(col, row);
  for (int row = 0; row < 3; ++row)
  {
    inverse(row, 3) = 0.0;
    for (int col = 0; col < 3; ++col)
      inverse(row, 3) -= inverse(row, col) * transform(col, 3);
  }
  return inverse;
}

Transform transformFromTranslationRpy(const std::vector<double>& translation,
                                      const std::vector<double>& rpy)
{
  if (translation.size() != 3 || rpy.size() != 3)
    throw std::runtime_error("base_to_camera translation and rotation_rpy must have length 3");
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  Transform result = Transform::eye();
  const cv::Matx33d rotation(
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = rotation(row, col);
  result(0, 3) = translation[0];
  result(1, 3) = translation[1];
  result(2, 3) = translation[2];
  return result;
}

Transform mapTagTransform(double x, double y, double yaw)
{
  Transform result = Transform::eye();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  result(0, 0) = c;
  result(0, 1) = -s;
  result(1, 0) = s;
  result(1, 1) = c;
  result(0, 3) = x;
  result(1, 3) = y;
  return result;
}

double yawFromTransform(const Transform& transform)
{
  return std::atan2(transform(1, 0), transform(0, 0));
}

Quaternion normalizedQuaternion(Quaternion quaternion)
{
  const double norm = std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                                quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm <= kEpsilon)
    return Quaternion();
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

Quaternion quaternionFromTransform(const Transform& transform)
{
  const double trace = transform(0, 0) + transform(1, 1) + transform(2, 2);
  Quaternion result;
  if (trace > 0.0)
  {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    result.w = 0.25 * scale;
    result.x = (transform(2, 1) - transform(1, 2)) / scale;
    result.y = (transform(0, 2) - transform(2, 0)) / scale;
    result.z = (transform(1, 0) - transform(0, 1)) / scale;
  }
  else if (transform(0, 0) > transform(1, 1) && transform(0, 0) > transform(2, 2))
  {
    const double scale = std::sqrt(1.0 + transform(0, 0) - transform(1, 1) - transform(2, 2)) * 2.0;
    result.w = (transform(2, 1) - transform(1, 2)) / scale;
    result.x = 0.25 * scale;
    result.y = (transform(0, 1) + transform(1, 0)) / scale;
    result.z = (transform(0, 2) + transform(2, 0)) / scale;
  }
  else if (transform(1, 1) > transform(2, 2))
  {
    const double scale = std::sqrt(1.0 + transform(1, 1) - transform(0, 0) - transform(2, 2)) * 2.0;
    result.w = (transform(0, 2) - transform(2, 0)) / scale;
    result.x = (transform(0, 1) + transform(1, 0)) / scale;
    result.y = 0.25 * scale;
    result.z = (transform(1, 2) + transform(2, 1)) / scale;
  }
  else
  {
    const double scale = std::sqrt(1.0 + transform(2, 2) - transform(0, 0) - transform(1, 1)) * 2.0;
    result.w = (transform(1, 0) - transform(0, 1)) / scale;
    result.x = (transform(0, 2) + transform(2, 0)) / scale;
    result.y = (transform(1, 2) + transform(2, 1)) / scale;
    result.z = 0.25 * scale;
  }
  return normalizedQuaternion(result);
}

Transform transformFromQuaternion(const Quaternion& quaternion, double x, double y, double z)
{
  const Quaternion q = normalizedQuaternion(quaternion);
  Transform result = Transform::eye();
  result(0, 0) = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  result(0, 1) = 2.0 * (q.x * q.y - q.z * q.w);
  result(0, 2) = 2.0 * (q.x * q.z + q.y * q.w);
  result(1, 0) = 2.0 * (q.x * q.y + q.z * q.w);
  result(1, 1) = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
  result(1, 2) = 2.0 * (q.y * q.z - q.x * q.w);
  result(2, 0) = 2.0 * (q.x * q.z - q.y * q.w);
  result(2, 1) = 2.0 * (q.y * q.z + q.x * q.w);
  result(2, 2) = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  result(0, 3) = x;
  result(1, 3) = y;
  result(2, 3) = z;
  return result;
}

Quaternion quaternionFromYaw(double yaw)
{
  Quaternion result;
  result.z = std::sin(yaw / 2.0);
  result.w = std::cos(yaw / 2.0);
  return result;
}

double quaternionDot(const Quaternion& left, const Quaternion& right)
{
  return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
}

Quaternion weightedQuaternion(const std::vector<Quaternion>& quaternions,
                              const std::vector<double>& weights)
{
  if (quaternions.empty())
    return Quaternion();
  const Quaternion reference = normalizedQuaternion(quaternions.front());
  Quaternion result{0.0, 0.0, 0.0, 0.0};
  const std::size_t count = std::min(quaternions.size(), weights.size());
  for (std::size_t index = 0; index < count; ++index)
  {
    Quaternion value = normalizedQuaternion(quaternions[index]);
    if (quaternionDot(reference, value) < 0.0)
    {
      value.x = -value.x;
      value.y = -value.y;
      value.z = -value.z;
      value.w = -value.w;
    }
    result.x += weights[index] * value.x;
    result.y += weights[index] * value.y;
    result.z += weights[index] * value.z;
    result.w += weights[index] * value.w;
  }
  return normalizedQuaternion(result);
}

double quaternionAngularDistance(const Quaternion& left, const Quaternion& right)
{
  const double dot = std::max(-1.0, std::min(1.0, std::abs(quaternionDot(
      normalizedQuaternion(left), normalizedQuaternion(right)))));
  return 2.0 * std::acos(dot);
}

void rpyFromQuaternion(const Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
  const Quaternion q = normalizedQuaternion(quaternion);
  const double sin_roll = 2.0 * (q.w * q.x + q.y * q.z);
  const double cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sin_roll, cos_roll);
  const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
  pitch = std::abs(sin_pitch) >= 1.0 ? std::copysign(M_PI / 2.0, sin_pitch) : std::asin(sin_pitch);
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(sin_yaw, cos_yaw);
}

}  // namespace core
}  // namespace tag_nav_localization
