#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/bind.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <tag_nav_localization/CameraBestTagArray.h>
#include <tag_nav_localization/FusedAprilTagLocalization.h>
#include <tag_nav_localization/core/apriltag_recognizer.h>
#include <tag_nav_localization/core/fusion_engine.h>
#include <tag_nav_localization/core/pose_temporal_filter.h>
#include <tag_nav_localization/ros/ros_config_loader.h>
#include <tag_nav_localization/ros/ros_message_adapter.h>
#include <tag_nav_localization/ros/ros_tf_bridge.h>

namespace tag_nav_localization
{
namespace
{

struct CameraRuntime
{
  std::deque<sensor_msgs::ImageConstPtr> queue;
  image_transport::Publisher debug_publisher;
  std::string target_encoding;
};

// Resolves the per-camera data_format into a cv_bridge target encoding. An
// empty data_format keeps the previous MONO8 default. Unknown encodings are
// rejected here so a typo fails at startup rather than per frame.
std::string resolveTargetEncoding(const core::CameraModel& camera)
{
  if (camera.data_format.empty())
    return sensor_msgs::image_encodings::MONO8;
  const std::string& format = camera.data_format;
  if (format == sensor_msgs::image_encodings::MONO8 ||
      format == sensor_msgs::image_encodings::MONO16 ||
      format == sensor_msgs::image_encodings::RGB8 ||
      format == sensor_msgs::image_encodings::BGR8 ||
      format == sensor_msgs::image_encodings::RGBA8 ||
      format == sensor_msgs::image_encodings::BGRA8)
    return format;
  throw std::runtime_error("camera '" + camera.name + "' has unsupported data_format '" +
                           format + "' (expected mono8, mono16, rgb8, bgr8, rgba8 or bgra8)");
}

// AprilTag detection runs on a single-channel 8-bit luminance image, whatever
// data_format the camera was converted to.
cv::Mat toGray(const cv::Mat& image, const std::string& encoding)
{
  if (image.channels() == 1)
  {
    if (encoding == sensor_msgs::image_encodings::MONO16)
    {
      cv::Mat gray;
      image.convertTo(gray, CV_8UC1, 1.0 / 256.0);
      return gray;
    }
    return image;
  }
  cv::Mat gray;
  if (encoding == sensor_msgs::image_encodings::RGB8)
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
  else if (encoding == sensor_msgs::image_encodings::RGBA8)
    cv::cvtColor(image, gray, cv::COLOR_RGBA2GRAY);
  else if (encoding == sensor_msgs::image_encodings::BGRA8)
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  else
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  return gray;
}

class AprilTagLocalizationNode
{
public:
  AprilTagLocalizationNode()
    : nh_(), private_nh_("~"), image_transport_(nh_),
      config_(ros_adapter::RosConfigLoader(private_nh_).load()),
      recognizer_(config_), fusion_engine_(config_), pose_filter_(config_.temporal_filter),
      tf_bridge_(config_.output), camera_runtime_(config_.cameras.size())
  {
    camera_results_publisher_ = private_nh_.advertise<CameraBestTagArray>("camera_best_tags", 1);
    localization_publisher_ = private_nh_.advertise<FusedAprilTagLocalization>("localization", 1);
    pose_publisher_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);

    for (std::size_t index = 0; index < config_.cameras.size(); ++index)
    {
      const core::CameraModel& camera = config_.cameras[index];
      if (!camera.enabled)
        continue;
      camera_runtime_[index].target_encoding = resolveTargetEncoding(camera);
      if (config_.output.debug_images)
        camera_runtime_[index].debug_publisher = image_transport_.advertise("debug/" + camera.name, 1);
      image_subscribers_.push_back(image_transport_.subscribe(
          camera.topic, config_.synchronization.queue_size,
          boost::bind(&AprilTagLocalizationNode::imageCallback, this, _1, index), ros::VoidPtr(),
          image_transport::TransportHints(camera.transport)));
    }
    sync_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.runtime.process_rate_hz),
                                  &AprilTagLocalizationNode::syncTimer, this);
    if (config_.output.publish_tf && config_.output.tf_mode == "correction")
    {
      correction_tf_timer_ = nh_.createTimer(
          ros::Duration(1.0 / config_.output.correction_tf_publish_rate_hz),
          &AprilTagLocalizationNode::correctionTfTimer, this);
    }
    ROS_INFO("AprilTag localization ready with %zu configured cameras and %zu map tags",
             config_.cameras.size(), config_.tag_map.size());
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& image, std::size_t camera_index)
  {
    if (image->header.stamp.isZero())
      ROS_WARN_THROTTLE(2.0, "camera %s publishes frames without a timestamp; these frames "
                        "cannot be time-synchronized", config_.cameras[camera_index].name.c_str());
    std::lock_guard<std::mutex> lock(sync_mutex_);
    CameraRuntime& runtime = camera_runtime_[camera_index];
    runtime.queue.push_back(image);
    const std::size_t maximum = static_cast<std::size_t>(std::max(1, config_.synchronization.queue_size));
    while (runtime.queue.size() > maximum)
      runtime.queue.pop_front();
  }

  sensor_msgs::ImageConstPtr nearestFrame(std::size_t camera_index, const ros::Time& target) const
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    sensor_msgs::ImageConstPtr best;
    double best_delta = std::numeric_limits<double>::infinity();
    for (const sensor_msgs::ImageConstPtr& frame : camera_runtime_[camera_index].queue)
    {
      if (frame->header.stamp.isZero())
        continue;
      const double delta = std::abs((frame->header.stamp - target).toSec());
      if (delta < best_delta)
      {
        best_delta = delta;
        best = frame;
      }
    }
    return best_delta <= config_.synchronization.slop_sec ? best : sensor_msgs::ImageConstPtr();
  }

  bool latestEligibleAnchor(const ros::Time& cutoff, ros::Time& anchor) const
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    bool have_anchor = false;
    for (std::size_t index = 0; index < config_.cameras.size(); ++index)
    {
      if (!config_.cameras[index].enabled)
        continue;
      const std::deque<sensor_msgs::ImageConstPtr>& queue = camera_runtime_[index].queue;
      for (auto frame = queue.rbegin(); frame != queue.rend(); ++frame)
      {
        const ros::Time stamp = (*frame)->header.stamp;
        if (stamp.isZero() || stamp > cutoff)
          continue;
        if (!have_anchor || stamp > anchor)
        {
          anchor = stamp;
          have_anchor = true;
        }
        break;
      }
    }
    return have_anchor;
  }

  void syncTimer(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();
    const ros::Time cutoff = now - ros::Duration(config_.synchronization.wait_sec);
    ros::Time target;
    if (!latestEligibleAnchor(cutoff, target) ||
        (!last_processed_stamp_.isZero() && target <= last_processed_stamp_) ||
        (!last_batch_time_.isZero() &&
         (target - last_batch_time_).toSec() < config_.effectiveBatchInterval()))
    {
      publishTimeoutState(now);
      return;
    }

    const ros::WallTime processing_start = ros::WallTime::now();
    std::vector<core::CameraObservation> observations;
    std::vector<ros_adapter::CameraFrame> frames(config_.cameras.size());
    for (std::size_t index = 0; index < config_.cameras.size(); ++index)
    {
      const core::CameraModel& camera = config_.cameras[index];
      if (!camera.enabled)
        continue;
      const sensor_msgs::ImageConstPtr frame = nearestFrame(index, target);
      if (!frame)
        continue;
      frames[index].available = true;
      frames[index].header = frame->header;
      core::CameraObservation observation;
      observation.camera_index = index;
      if (config_.validation.stale_frame_timeout_sec > 0.0 && !frame->header.stamp.isZero() &&
          (now - frame->header.stamp).toSec() > config_.validation.stale_frame_timeout_sec)
      {
        observation.status = "stale_frame";
        observations.push_back(observation);
        continue;
      }
      try
      {
        const cv_bridge::CvImagePtr converted = cv_bridge::toCvCopy(
            frame, camera_runtime_[index].target_encoding);
        const cv::Mat gray = recognizer_.rectify(
            toGray(converted->image, camera_runtime_[index].target_encoding), camera);
        observation = recognizer_.recognize(gray, index);
        publishDebug(index, gray, observation, frame->header);
      }
      catch (const cv_bridge::Exception& error)
      {
        observation.status = "image_conversion_error";
        ROS_WARN_THROTTLE(2.0, "unable to convert camera %s image: %s",
                          camera.name.c_str(), error.what());
      }
      catch (const cv::Exception& error)
      {
        observation.status = "image_processing_error";
        ROS_WARN_THROTTLE(2.0, "image processing failed for camera %s: %s",
                          camera.name.c_str(), error.what());
      }
      observations.push_back(observation);
    }
    last_processed_stamp_ = target;
    last_batch_time_ = target;
    last_processing_time_sec_ = (ros::WallTime::now() - processing_start).toSec();
    publishResults(target, observations, frames);
  }

  void publishTimeoutState(const ros::Time& now)
  {
    if (!last_timeout_publish_time_.isZero() &&
        (now - last_timeout_publish_time_).toSec() < config_.effectiveBatchInterval())
      return;
    last_timeout_publish_time_ = now;
    if (last_valid_stamp_.isZero() || config_.runtime.localization_timeout_sec <= 0.0 ||
        (now - last_valid_stamp_).toSec() >= config_.runtime.localization_timeout_sec)
    {
      last_processing_time_sec_ = 0.0;
      publishResults(now, std::vector<core::CameraObservation>(),
                     std::vector<ros_adapter::CameraFrame>(config_.cameras.size()));
    }
    else
    {
      tf_bridge_.republishHeldCorrection();
    }
  }

  void publishDebug(std::size_t camera_index, const cv::Mat& gray,
                    const core::CameraObservation& observation, const std_msgs::Header& header)
  {
    image_transport::Publisher& publisher = camera_runtime_[camera_index].debug_publisher;
    if (!config_.output.debug_images || publisher.getNumSubscribers() == 0)
      return;
    cv::Mat debug;
    cv::cvtColor(gray, debug, cv::COLOR_GRAY2BGR);
    if (observation.has_candidate)
    {
      const core::TagCandidate& candidate = observation.candidate;
      for (std::size_t index = 0; index < candidate.corners.size(); ++index)
        cv::line(debug, candidate.corners[index], candidate.corners[(index + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      std::ostringstream label;
      label << "id=" << candidate.tag_id << " q=" << candidate.scores.quality;
      cv::putText(debug, label.str(), candidate.corners[0], cv::FONT_HERSHEY_SIMPLEX,
                  0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
    else
    {
      cv::putText(debug, "no mapped tag", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                  0.6, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    cv_bridge::CvImage output(header, sensor_msgs::image_encodings::BGR8, debug);
    publisher.publish(output.toImageMsg());
  }

  void publishResults(const ros::Time& stamp,
                      const std::vector<core::CameraObservation>& observations,
                      const std::vector<ros_adapter::CameraFrame>& frames)
  {
    const core::FusionResult fusion = fusion_engine_.fuse(observations);
    core::FusionResult smoothed = fusion;
    if (fusion.valid && config_.temporal_filter.enabled)
      smoothed.pose.map_to_base = pose_filter_.filter(fusion.pose.map_to_base, stamp.toSec());
    camera_results_publisher_.publish(ros_adapter::makeCameraArray(stamp, config_, fusion, frames));

    ros_adapter::TfPublicationStatus tf_status;
    double age = last_valid_stamp_.isZero() ? -1.0 :
        std::max(0.0, (stamp - last_valid_stamp_).toSec());
    if (fusion.valid)
    {
      last_valid_stamp_ = stamp;
      age = 0.0;
      tf_status = tf_bridge_.publishFused(smoothed.pose.map_to_base, stamp);
    }
    else
    {
      tf_status.parent_frame = config_.output.map_frame;
      tf_status.child_frame = config_.output.tf_mode == "correction"
          ? config_.output.odom_frame : config_.output.base_frame;
      if (!config_.output.publish_tf)
      {
        tf_status.status = "disabled";
      }
      else if (config_.output.tf_mode == "correction")
      {
        tf_status = tf_bridge_.publishHeldCorrection();
        if (tf_status.status.empty())
        {
          tf_status.parent_frame = config_.output.map_frame;
          tf_status.child_frame = config_.output.odom_frame;
          tf_status.status = "no_valid_pose";
        }
      }
      else
      {
        tf_status.status = "no_valid_pose";
      }
    }
    FusedAprilTagLocalization localization = ros_adapter::makeLocalization(
        stamp, config_, smoothed, last_processing_time_sec_, age, tf_status);
    localization_publisher_.publish(localization);
    if (fusion.valid)
      pose_publisher_.publish(localization.pose);
  }

  void correctionTfTimer(const ros::TimerEvent&)
  {
    tf_bridge_.republishHeldCorrection();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  core::LocalizationConfig config_;
  core::AprilTagRecognizer recognizer_;
  core::FusionEngine fusion_engine_;
  core::PoseTemporalFilter pose_filter_;
  ros_adapter::RosTfBridge tf_bridge_;
  mutable std::mutex sync_mutex_;  // guards the per-camera image queues shared with image callbacks
  std::vector<CameraRuntime> camera_runtime_;
  std::vector<image_transport::Subscriber> image_subscribers_;
  ros::Timer sync_timer_;
  ros::Timer correction_tf_timer_;
  ros::Publisher camera_results_publisher_;
  ros::Publisher localization_publisher_;
  ros::Publisher pose_publisher_;
  ros::Time last_processed_stamp_;
  ros::Time last_batch_time_;
  ros::Time last_valid_stamp_;
  ros::Time last_timeout_publish_time_;
  double last_processing_time_sec_ = 0.0;
};

}  // namespace
}  // namespace tag_nav_localization

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apriltag_localization_node");
  try
  {
    tag_nav_localization::AprilTagLocalizationNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("AprilTag localization failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
