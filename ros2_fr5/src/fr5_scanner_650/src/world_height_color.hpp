#ifndef FR5_SCANNER_650_WORLD_HEIGHT_COLOR_HPP_
#define FR5_SCANNER_650_WORLD_HEIGHT_COLOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace fr5_scanner_650::world_height_color
{

struct Range
{
  double lower_z_m{0.0};
  double upper_z_m{0.0};
  bool valid{false};
};

struct Rgb
{
  std::uint8_t red{128U};
  std::uint8_t green{128U};
  std::uint8_t blue{128U};
};

// Derive a robust presentation range from world-frame Z coordinates. Point
// coordinates are not modified; percentiles only suppress isolated outliers.
bool computeRange(
  std::vector<double> world_z_m, double lower_percentile, double upper_percentile,
  Range * range, std::string * error = nullptr);

// Lower world Z is purple/blue and higher world Z is orange/red.
Rgb colorForWorldZ(double world_z_m, const Range & range);

}  // namespace fr5_scanner_650::world_height_color

#endif  // FR5_SCANNER_650_WORLD_HEIGHT_COLOR_HPP_
