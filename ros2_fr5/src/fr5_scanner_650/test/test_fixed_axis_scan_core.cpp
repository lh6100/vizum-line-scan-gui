#include "fixed_axis_scan_core.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

namespace
{

TEST(FixedAxisScanCore, ProjectsCameraAxisOntoBaseHorizontalPlane)
{
  std::array<double, 3> output{};
  std::string error;
  ASSERT_TRUE(
    fr5_scanner_650::fixed_axis_scan::normalizeDirection(
      {1.0, 2.0, -0.3}, true, &output, &error)) << error;
  EXPECT_NEAR(output[0], 1.0 / std::sqrt(5.0), 1.0e-12);
  EXPECT_NEAR(output[1], 2.0 / std::sqrt(5.0), 1.0e-12);
  EXPECT_DOUBLE_EQ(output[2], 0.0);
}

TEST(FixedAxisScanCore, RejectsOpticalAxisWithoutHorizontalProjection)
{
  std::array<double, 3> output{};
  std::string error;
  EXPECT_FALSE(
    fr5_scanner_650::fixed_axis_scan::normalizeDirection(
      {0.0, 0.0, -1.0}, true, &output, &error));
  EXPECT_FALSE(error.empty());
}

TEST(FixedAxisScanCore, PreservesThreeDimensionalDirectionWhenHeightIsUnconstrained)
{
  std::array<double, 3> output{};
  std::string error;
  ASSERT_TRUE(
    fr5_scanner_650::fixed_axis_scan::normalizeDirection(
      {0.0, 3.0, 4.0}, false, &output, &error)) << error;
  EXPECT_DOUBLE_EQ(output[0], 0.0);
  EXPECT_NEAR(output[1], 0.6, 1.0e-12);
  EXPECT_NEAR(output[2], 0.8, 1.0e-12);
}

}  // namespace
