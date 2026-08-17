#include <algorithm>

#include <ros/console.h>

#include <tag_nav_localization/core/transform_math.h>
#include <tag_nav_localization/ros/ros_tf_bridge.h>

namespace tag_nav_localization
{
namespace ros_adapter
{
namespace
{

core::Transform transformFromTf(const tf::Transform& transform)
{
  core::Transform result = core::Transform::eye();
  const tf::Matrix3x3 basis = transform.getBasis();
  const tf::Vector3 origin = transform.getOrigin();
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = basis[row][col];
  result(0, 3) = origin.x();
  result(1, 3) = origin.y();
  result(2, 3) = origin.z();
  return result;
}

tf::Transform transformToTf(const core::Transform& transform)
{
  const core::Quaternion q = core::quaternionFromTransform(transform);
  return tf::Transform(tf::Quaternion(q.x, q.y, q.z, q.w),
                       tf::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));
}

}  // namespace

RosTfBridge::RosTfBridge(const core::OutputConfig& config) : config_(config)
{
}

TfPublicationStatus RosTfBridge::publishFused(const core::Transform& map_to_base,
                                               const ros::Time& stamp)
{
  TfPublicationStatus result;
  result.parent_frame = config_.map_frame;
  result.child_frame = config_.tf_mode == "correction"
      ? config_.odom_frame : config_.base_frame;
  if (!config_.publish_tf)
  {
    result.status = "disabled";
    return result;
  }
  if (config_.tf_mode != "correction")
    return publish(map_to_base, stamp, result.parent_frame, result.child_frame, "published");

  tf::StampedTransform odom_to_base;
  try
  {
    listener_.waitForTransform(config_.odom_frame, config_.base_frame, stamp,
                               ros::Duration(config_.tf_lookup_timeout_sec));
    listener_.lookupTransform(config_.odom_frame, config_.base_frame, stamp,
                              odom_to_base);
  }
  catch (const tf::TransformException& error)
  {
    ROS_WARN_THROTTLE(2.0, "unable to compute map->odom from odom->base at %.3f: %s",
                      stamp.toSec(), error.what());
    result.status = "odom_base_unavailable";
    return result;
  }
  const core::Transform map_to_odom = map_to_base * core::inverseRigid(transformFromTf(odom_to_base));
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    last_map_to_odom_ = map_to_odom;
    have_map_to_odom_ = true;
  }
  return publishCorrection(map_to_odom, "published");
}

TfPublicationStatus RosTfBridge::publishHeldCorrection()
{
  TfPublicationStatus result;
  result.parent_frame = config_.map_frame;
  result.child_frame = config_.odom_frame;
  core::Transform held;
  bool have = false;
  {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    have = have_map_to_odom_;
    held = last_map_to_odom_;
  }
  if (!config_.publish_tf || config_.tf_mode != "correction" || !have)
    return result;
  return publishCorrection(held, "correction_held");
}

void RosTfBridge::republishHeldCorrection()
{
  publishHeldCorrection();
}

TfPublicationStatus RosTfBridge::publishCorrection(const core::Transform& transform,
                                                    const std::string& success_status)
{
  return publish(transform, ros::Time::now() + ros::Duration(config_.correction_tf_tolerance_sec),
                 config_.map_frame, config_.odom_frame, success_status);
}

TfPublicationStatus RosTfBridge::publish(const core::Transform& transform, const ros::Time& stamp,
                                          const std::string& parent, const std::string& child,
                                          const std::string& success_status)
{
  TfPublicationStatus result;
  result.parent_frame = parent;
  result.child_frame = child;
  std::lock_guard<std::mutex> lock(tf_mutex_);
  if (stamp < last_tf_stamp_)
  {
    ROS_WARN_THROTTLE(2.0, "TF timestamp moved backwards; resetting TF timestamp guard");
    last_tf_stamp_ = ros::Time(0);
    last_tf_parent_.clear();
    last_tf_child_.clear();
  }
  if (!last_tf_stamp_.isZero() && stamp <= last_tf_stamp_ && parent == last_tf_parent_ &&
      child == last_tf_child_)
  {
    result.status = "duplicate_timestamp_skipped";
    return result;
  }
  broadcaster_.sendTransform(tf::StampedTransform(transformToTf(transform), stamp, parent, child));
  last_tf_stamp_ = stamp;
  last_tf_parent_ = parent;
  last_tf_child_ = child;
  result.published = true;
  result.status = success_status;
  return result;
}

}  // namespace ros_adapter
}  // namespace tag_nav_localization
