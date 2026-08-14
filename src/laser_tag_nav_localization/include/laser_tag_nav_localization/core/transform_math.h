#ifndef LASER_TAG_NAV_LOCALIZATION_CORE_TRANSFORM_MATH_H
#define LASER_TAG_NAV_LOCALIZATION_CORE_TRANSFORM_MATH_H

#include <vector>

#include <laser_tag_nav_localization/core/types.h>

namespace laser_tag_nav_localization
{
namespace core
{

Transform inverseRigid(const Transform& transform);
Transform transformFromTranslationRpy(const std::vector<double>& translation,
                                      const std::vector<double>& rpy);
Transform mapTagTransform(double x, double y, double yaw);
double yawFromTransform(const Transform& transform);
Quaternion quaternionFromTransform(const Transform& transform);
Transform transformFromQuaternion(const Quaternion& quaternion, double x, double y, double z);
Quaternion quaternionFromYaw(double yaw);
Quaternion normalizedQuaternion(Quaternion quaternion);
double quaternionDot(const Quaternion& left, const Quaternion& right);
Quaternion weightedQuaternion(const std::vector<Quaternion>& quaternions,
                              const std::vector<double>& weights);
double quaternionAngularDistance(const Quaternion& left, const Quaternion& right);
void rpyFromQuaternion(const Quaternion& quaternion, double& roll, double& pitch, double& yaw);

}  // namespace core
}  // namespace laser_tag_nav_localization

#endif
