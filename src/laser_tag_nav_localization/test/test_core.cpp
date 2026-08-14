#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <laser_tag_nav_localization/core/configuration.h>
#include <laser_tag_nav_localization/core/apriltag_recognizer.h>
#include <laser_tag_nav_localization/core/fusion_engine.h>
#include <laser_tag_nav_localization/core/transform_math.h>

namespace core = laser_tag_nav_localization::core;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace
{

core::LocalizationConfig fusionConfig(const std::string& mode = "2d")
{
  core::LocalizationConfig config;
  config.fusion.mode = mode;
  config.fusion.min_contributing_cameras = 1;
  config.output.invalid_variance = 1000000.0;
  config.cameras.resize(2);
  config.cameras[0].name = "front";
  config.cameras[1].name = "rear";
  return config;
}

core::CameraObservation observation(std::size_t camera_index, double x, double y,
                                    double yaw, double quality)
{
  core::CameraObservation value;
  value.camera_index = camera_index;
  value.has_candidate = true;
  value.status = "valid";
  value.candidate.tag_id = static_cast<int>(camera_index + 10);
  value.candidate.map_to_base = core::mapTagTransform(x, y, yaw);
  value.candidate.scores.quality = quality;
  value.candidate.weighted_quality = quality;
  return value;
}

std::string temporaryMapPath()
{
  return "/tmp/laser_tag_nav_localization_test_map.json";
}

}  // namespace

TEST(TransformMath, InverseAndCorrectionChain)
{
  const core::Transform map_to_base = core::mapTagTransform(4.0, -2.0, M_PI / 2.0);
  const core::Transform odom_to_base = core::mapTagTransform(1.0, 3.0, M_PI / 4.0);
  const core::Transform map_to_odom = map_to_base * core::inverseRigid(odom_to_base);
  const core::Transform reconstructed = map_to_odom * odom_to_base;
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      EXPECT_NEAR(reconstructed(row, column), map_to_base(row, column), 1e-12);
}

TEST(TransformMath, QuaternionAverageSignAligns)
{
  const core::Quaternion positive = core::quaternionFromYaw(1.0);
  core::Quaternion negative = positive;
  negative.x = -negative.x;
  negative.y = -negative.y;
  negative.z = -negative.z;
  negative.w = -negative.w;
  const core::Quaternion fused = core::weightedQuaternion({positive, negative}, {0.5, 0.5});
  EXPECT_NEAR(core::quaternionAngularDistance(positive, fused), 0.0, 1e-12);
}

TEST(FusionEngine, Weighted2dFusionAndCovariance)
{
  core::LocalizationConfig config = fusionConfig();
  config.fusion.min_position_stddev = 0.1;
  config.fusion.min_yaw_stddev = 0.2;
  core::FusionEngine engine(config);
  const core::FusionResult result = engine.fuse({
      observation(0, 0.0, 0.0, 0.0, 1.0), observation(1, 2.0, 4.0, 0.0, 3.0)});
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.contributing_camera_count, 2u);
  EXPECT_NEAR(result.pose.map_to_base(0, 3), 1.5, 1e-12);
  EXPECT_NEAR(result.pose.map_to_base(1, 3), 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.pose.map_to_base(2, 3), 0.0);
  EXPECT_DOUBLE_EQ(result.pose.covariance[14], config.output.invalid_variance);
  EXPECT_DOUBLE_EQ(result.pose.covariance[21], config.output.invalid_variance);
}

TEST(FusionEngine, OutlierGateRejectsFarCandidate)
{
  core::LocalizationConfig config = fusionConfig();
  config.fusion.outlier_gate_enabled = true;
  config.fusion.outlier_position_threshold = 1.0;
  config.fusion.outlier_yaw_threshold = 1.0;
  core::FusionEngine engine(config);
  const core::FusionResult result = engine.fuse({
      observation(0, 0.0, 0.0, 0.0, 0.9), observation(1, 10.0, 0.0, 0.0, 0.1)});
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.observations[1].rejected);
  EXPECT_EQ(result.observations[1].status, "outlier");
  EXPECT_NEAR(result.pose.map_to_base(0, 3), 0.0, 1e-12);
}

TEST(FusionEngine, InvalidWhenMinimumContributorsMissing)
{
  core::LocalizationConfig config = fusionConfig();
  config.fusion.min_contributing_cameras = 2;
  core::FusionEngine engine(config);
  const core::FusionResult result = engine.fuse({observation(0, 0.0, 0.0, 0.0, 1.0)});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.contributing_camera_count, 0u);
  EXPECT_DOUBLE_EQ(result.pose.covariance[0], config.output.invalid_variance);
}

TEST(FusionEngine, Fuses3dOrientationAndHeight)
{
  core::LocalizationConfig config = fusionConfig("3d");
  core::FusionEngine engine(config);
  core::CameraObservation first = observation(0, 1.0, 2.0, 0.2, 1.0);
  core::CameraObservation second = observation(1, 1.0, 2.0, 0.2, 1.0);
  first.candidate.map_to_base(2, 3) = 0.4;
  second.candidate.map_to_base(2, 3) = 0.6;
  const core::FusionResult result = engine.fuse({first, second});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.pose.map_to_base(2, 3), 0.5, 1e-12);
  EXPECT_LT(core::quaternionAngularDistance(
      core::quaternionFromTransform(first.candidate.map_to_base),
      core::quaternionFromTransform(result.pose.map_to_base)), 1e-12);
  EXPECT_LT(result.pose.covariance[21], config.output.invalid_variance);
  EXPECT_LT(result.pose.covariance[28], config.output.invalid_variance);
}

TEST(AprilTagRecognizer, BlankImageHasNoMappedTag)
{
  core::LocalizationConfig config = fusionConfig();
  config.detector.tag_family = "tag36h11";
  config.cameras.resize(1);
  config.cameras[0].name = "front";
  config.cameras[0].K = cv::Matx33d(100.0, 0.0, 32.0, 0.0, 100.0, 24.0, 0.0, 0.0, 1.0);
  core::AprilTagRecognizer recognizer(config);
  const core::CameraObservation result = recognizer.recognize(cv::Mat::zeros(48, 64, CV_8UC1), 0);
  EXPECT_FALSE(result.has_candidate);
  EXPECT_EQ(result.status, "no_mapped_tag");
}

TEST(Configuration, ParsesMapAndPackageUriThroughInjectedResolver)
{
  const std::string path = temporaryMapPath();
  {
    std::ofstream map(path);
    map << R"({"map_type":"2d","tag_side_lengths":{"default":0.2},"tag_locations":{"7":[1.0,2.0,0.5]}})";
  }
  const std::string cameras = R"({
    "schema_version": 1,
    "map": {"uri": "package://test/map.json"},
    "quality": {"mask": "1010", "metric_exponents": {"area": 1, "margin": 3}},
    "cameras": [{"name": "front", "image_topic": "/image",
      "intrinsics": {"K": [100,0,10,0,100,10,0,0,1]},
      "base_to_camera": {"translation": [0,0,0], "rotation_rpy": [0,0,0]}}]
  })";
  core::LocalizationConfig config = core::parseConfiguration(
      cameras, "", [path](const std::string& uri) {
        EXPECT_EQ(uri, "package://test/map.json");
        return path;
      });
  core::validateConfiguration(config);
  core::loadTagMap(config);
  EXPECT_EQ(config.fusion.mode, "2d");
  EXPECT_EQ(config.tag_map.size(), 1u);
  EXPECT_NEAR(config.quality.metric_exponents[0], 0.25, 1e-12);
  EXPECT_NEAR(config.quality.metric_exponents[2], 0.75, 1e-12);
  std::remove(path.c_str());
}

TEST(Configuration, RejectsInvalidSchemaAndFrames)
{
  const std::string invalid_schema = R"({"schema_version":2,"cameras":[]})";
  EXPECT_THROW(core::parseConfiguration(invalid_schema, "", core::UriResolver()), std::runtime_error);

  core::LocalizationConfig invalid = fusionConfig();
  invalid.output.tf_mode = "correction";
  invalid.output.map_frame = "map";
  invalid.output.odom_frame = "map";
  invalid.output.base_frame = "base";
  EXPECT_THROW(core::validateConfiguration(invalid), std::runtime_error);
}
