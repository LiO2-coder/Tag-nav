#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <laser_tag_nav_localization/core/configuration.h>
#include <laser_tag_nav_localization/core/pose_temporal_filter.h>
#include <laser_tag_nav_localization/core/transform_math.h>

namespace core = laser_tag_nav_localization::core;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace
{

core::TemporalFilterConfig filterConfig()
{
  core::TemporalFilterConfig config;
  config.enabled = true;
  config.position_time_constant_sec = 0.25;
  config.orientation_time_constant_sec = 0.25;
  config.max_dt_sec = 1.0;
  return config;
}

}  // namespace

TEST(PoseTemporalFilter, FirstCallSeedsAndReturnsInput)
{
  core::PoseTemporalFilter filter(filterConfig());
  const core::Transform pose = core::mapTagTransform(1.0, 2.0, 0.5);
  const core::Transform result = filter.filter(pose, 10.0);
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      EXPECT_DOUBLE_EQ(result(row, column), pose(row, column));
}

TEST(PoseTemporalFilter, PositionEmaUsesTimeConstant)
{
  core::TemporalFilterConfig config = filterConfig();
  config.position_time_constant_sec = 1.0;
  core::PoseTemporalFilter filter(config);
  filter.filter(core::mapTagTransform(0.0, 0.0, 0.0), 0.0);
  const core::Transform result = filter.filter(core::mapTagTransform(1.0, 0.0, 0.0), 1.0);
  const double alpha = 1.0 - std::exp(-1.0);
  EXPECT_NEAR(result(0, 3), alpha, 1e-12);
}

TEST(PoseTemporalFilter, ConvergesTowardConstantInput)
{
  core::PoseTemporalFilter filter(filterConfig());
  const core::Transform target = core::mapTagTransform(2.0, -1.0, 0.3);
  filter.filter(core::mapTagTransform(0.0, 0.0, 0.0), 0.0);
  core::Transform current;
  for (int step = 0; step < 200; ++step)
    current = filter.filter(target, (step + 1) * 0.0667);
  EXPECT_NEAR(current(0, 3), 2.0, 1e-9);
  EXPECT_NEAR(current(1, 3), -1.0, 1e-9);
  EXPECT_NEAR(core::yawFromTransform(current), 0.3, 1e-9);
}

TEST(PoseTemporalFilter, LargeDtReinitializes)
{
  core::PoseTemporalFilter filter(filterConfig());
  filter.filter(core::mapTagTransform(0.0, 0.0, 0.0), 0.0);
  const core::Transform jumped = core::mapTagTransform(5.0, 5.0, 0.0);
  const core::Transform result = filter.filter(jumped, 10.0);
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      EXPECT_DOUBLE_EQ(result(row, column), jumped(row, column));
}

TEST(PoseTemporalFilter, NonPositiveDtReinitializes)
{
  core::PoseTemporalFilter filter(filterConfig());
  filter.filter(core::mapTagTransform(0.0, 0.0, 0.0), 1.0);
  const core::Transform input = core::mapTagTransform(1.0, 0.0, 0.0);
  const core::Transform result = filter.filter(input, 1.0);
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      EXPECT_DOUBLE_EQ(result(row, column), input(row, column));
}

TEST(Configuration, ParsesTemporalFilterSection)
{
  const std::string cameras = R"({
    "schema_version": 1,
    "map": {"uri": "package://test/map.json"},
    "temporal_filter": {"enabled": true, "position_time_constant_sec": 0.5,
      "orientation_time_constant_sec": 0.4, "max_dt_sec": 2.0},
    "cameras": [{"name": "front", "image_topic": "/image",
      "intrinsics": {"K": [100,0,10,0,100,10,0,0,1]},
      "base_to_camera": {"translation": [0,0,0], "rotation_rpy": [0,0,0]}}]
  })";
  core::LocalizationConfig config = core::parseConfiguration(
      cameras, "", [](const std::string& uri) { return uri; });
  core::validateConfiguration(config);
  EXPECT_TRUE(config.temporal_filter.enabled);
  EXPECT_DOUBLE_EQ(config.temporal_filter.position_time_constant_sec, 0.5);
  EXPECT_DOUBLE_EQ(config.temporal_filter.orientation_time_constant_sec, 0.4);
  EXPECT_DOUBLE_EQ(config.temporal_filter.max_dt_sec, 2.0);
}
