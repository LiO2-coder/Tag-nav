#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include <laser_tag_nav_localization/quality_math.h>

using laser_tag_nav_localization::geometricQuality;
using laser_tag_nav_localization::correctionTransform;
using laser_tag_nav_localization::normalizedQualityExponents;
using laser_tag_nav_localization::quadrilateralDistortionError;
using laser_tag_nav_localization::validQualityMask;
using laser_tag_nav_localization::weightedYaw;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(QualityMath, GeometricMeanAndZeroHandling)
{
  EXPECT_NEAR(geometricQuality(1.0, 1.0, 1.0, 1.0), 1.0, 1e-12);
  EXPECT_NEAR(geometricQuality(0.25, 0.25, 0.25, 0.25), 0.25, 1e-12);
  EXPECT_DOUBLE_EQ(geometricQuality(0.0, 1.0, 1.0, 1.0), 0.0);
}

TEST(QualityMath, SquareHasSmallDistortion)
{
  const std::array<cv::Point2d, 4> square{{
      cv::Point2d(0.0, 0.0), cv::Point2d(10.0, 0.0),
      cv::Point2d(10.0, 10.0), cv::Point2d(0.0, 10.0)}};
  const std::array<cv::Point2d, 4> skew{{
      cv::Point2d(0.0, 0.0), cv::Point2d(14.0, 1.0),
      cv::Point2d(10.0, 10.0), cv::Point2d(0.0, 8.0)}};
  EXPECT_LT(quadrilateralDistortionError(square), 1e-9);
  EXPECT_GT(quadrilateralDistortionError(skew), 0.05);
}

TEST(QualityMath, CircularYawAverageCrossesPi)
{
  const std::vector<double> yaws{3.13, -3.13};
  const std::vector<double> weights{0.5, 0.5};
  const double average = weightedYaw(yaws, weights);
  EXPECT_NEAR(std::abs(average), M_PI, 0.02);
}

TEST(QualityMath, QualityMaskNormalizesEnabledMetrics)
{
  EXPECT_TRUE(validQualityMask("1111"));
  EXPECT_TRUE(validQualityMask("1010"));
  EXPECT_FALSE(validQualityMask("0000"));
  EXPECT_FALSE(validQualityMask("100"));
  const std::array<double, 4> exponents = normalizedQualityExponents(
      "1010", {{1.0, 1.0, 3.0, 1.0}});
  EXPECT_NEAR(exponents[0], 0.25, 1e-12);
  EXPECT_DOUBLE_EQ(exponents[1], 0.0);
  EXPECT_NEAR(exponents[2], 0.75, 1e-12);
  EXPECT_DOUBLE_EQ(exponents[3], 0.0);
  EXPECT_NEAR(exponents[0] + exponents[2], 1.0, 1e-12);
}

TEST(QualityMath, CorrectionTransformPreservesMapBaseChain)
{
  cv::Matx44d map_to_base = cv::Matx44d::eye();
  map_to_base(0, 3) = 4.0;
  map_to_base(1, 3) = -2.0;
  cv::Matx44d odom_to_base = cv::Matx44d::eye();
  odom_to_base(0, 3) = 1.5;
  odom_to_base(1, 3) = 0.5;
  const cv::Matx44d map_to_odom = correctionTransform(map_to_base, odom_to_base);
  EXPECT_NEAR(map_to_odom(0, 3), 2.5, 1e-12);
  EXPECT_NEAR(map_to_odom(1, 3), -2.5, 1e-12);
}

TEST(QualityMath, CorrectionTransformPreservesRotatedMapBaseChain)
{
  cv::Matx44d map_to_base = cv::Matx44d::eye();
  map_to_base(0, 0) = 0.0;
  map_to_base(0, 1) = -1.0;
  map_to_base(1, 0) = 1.0;
  map_to_base(1, 1) = 0.0;
  map_to_base(0, 3) = 3.0;
  map_to_base(1, 3) = 4.0;

  cv::Matx44d odom_to_base = map_to_base;
  odom_to_base(0, 3) = 1.0;
  odom_to_base(1, 3) = 2.0;

  const cv::Matx44d map_to_odom = correctionTransform(map_to_base, odom_to_base);
  const cv::Matx44d reconstructed_map_to_base = map_to_odom * odom_to_base;
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      EXPECT_NEAR(reconstructed_map_to_base(row, col), map_to_base(row, col), 1e-12);
}
