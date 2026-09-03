#include "world_height_color.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

namespace color = fr5_scanner_650::world_height_color;

TEST(WorldHeightColor, ComputesRobustRangeInWorldZ)
{
  std::vector<double> heights;
  for (int index = 0; index <= 100; ++index) {
    heights.push_back(static_cast<double>(index) * 0.001);
  }
  heights.push_back(-100.0);
  heights.push_back(100.0);
  heights.push_back(std::numeric_limits<double>::quiet_NaN());
  color::Range range;
  std::string error;
  ASSERT_TRUE(color::computeRange(heights, 1.0, 99.0, &range, &error)) << error;
  EXPECT_TRUE(range.valid);
  EXPECT_GT(range.lower_z_m, -1.0);
  EXPECT_LT(range.upper_z_m, 1.0);
  EXPECT_LT(range.lower_z_m, range.upper_z_m);
}

TEST(WorldHeightColor, HigherWorldZMovesFromBlueTowardRed)
{
  const color::Range range{0.0, 1.0, true};
  const color::Rgb low = color::colorForWorldZ(0.0, range);
  const color::Rgb high = color::colorForWorldZ(1.0, range);
  EXPECT_GT(low.blue, low.red);
  EXPECT_GT(high.red, high.blue);
}

TEST(WorldHeightColor, ConstantHeightUsesMiddleOfColorMap)
{
  const color::Range range{0.25, 0.25, true};
  const color::Rgb first = color::colorForWorldZ(0.25, range);
  const color::Rgb second = color::colorForWorldZ(0.50, range);
  EXPECT_EQ(first.red, second.red);
  EXPECT_EQ(first.green, second.green);
  EXPECT_EQ(first.blue, second.blue);
}
