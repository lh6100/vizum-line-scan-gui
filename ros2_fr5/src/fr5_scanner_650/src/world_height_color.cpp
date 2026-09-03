#include "world_height_color.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace fr5_scanner_650::world_height_color
{
namespace
{

double percentile(const std::vector<double> & sorted, double value)
{
  const double position = static_cast<double>(sorted.size() - 1U) * value / 100.0;
  const std::size_t lower = static_cast<std::size_t>(std::floor(position));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {return sorted[lower];}
  const double fraction = position - static_cast<double>(lower);
  return sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction;
}

}  // namespace

bool computeRange(
  std::vector<double> world_z_m, double lower_percentile, double upper_percentile,
  Range * range, std::string * error)
{
  if (!range || !std::isfinite(lower_percentile) || !std::isfinite(upper_percentile) ||
    lower_percentile < 0.0 || lower_percentile >= upper_percentile ||
    upper_percentile > 100.0)
  {
    if (error) {*error = "invalid world-Z color percentile range";}
    return false;
  }
  world_z_m.erase(
    std::remove_if(
      world_z_m.begin(), world_z_m.end(),
      [](double value) {return !std::isfinite(value);}),
    world_z_m.end());
  *range = Range{};
  if (world_z_m.empty()) {
    if (error) {*error = "no finite world-Z values are available for coloring";}
    return false;
  }
  std::sort(world_z_m.begin(), world_z_m.end());
  range->lower_z_m = percentile(world_z_m, lower_percentile);
  range->upper_z_m = percentile(world_z_m, upper_percentile);
  range->valid = std::isfinite(range->lower_z_m) && std::isfinite(range->upper_z_m);
  if (!range->valid) {
    if (error) {*error = "computed world-Z color range is non-finite";}
    return false;
  }
  if (error) {error->clear();}
  return true;
}

Rgb colorForWorldZ(double world_z_m, const Range & range)
{
  if (!std::isfinite(world_z_m) || !range.valid) {return Rgb{};}
  struct Stop
  {
    double position;
    Rgb color;
  };
  static const std::array<Stop, 9> stops{{
    {0.00, {48U, 18U, 59U}},
    {0.13, {67U, 97U, 209U}},
    {0.25, {32U, 183U, 233U}},
    {0.38, {47U, 238U, 174U}},
    {0.50, {164U, 252U, 60U}},
    {0.63, {238U, 208U, 35U}},
    {0.75, {251U, 126U, 32U}},
    {0.88, {204U, 45U, 12U}},
    {1.00, {122U, 4U, 3U}}
  }};
  double normalized = 0.5;
  if (range.upper_z_m > range.lower_z_m) {
    normalized = (world_z_m - range.lower_z_m) / (range.upper_z_m - range.lower_z_m);
  }
  normalized = std::clamp(normalized, 0.0, 1.0);
  for (std::size_t index = 1U; index < stops.size(); ++index) {
    if (normalized > stops[index].position) {continue;}
    const Stop & first = stops[index - 1U];
    const Stop & second = stops[index];
    const double fraction =
      (normalized - first.position) / (second.position - first.position);
    const auto interpolate = [fraction](std::uint8_t first_value, std::uint8_t second_value) {
        return static_cast<std::uint8_t>(std::lround(
                 static_cast<double>(first_value) +
                 (static_cast<double>(second_value) - static_cast<double>(first_value)) *
                 fraction));
      };
    return Rgb{
      interpolate(first.color.red, second.color.red),
      interpolate(first.color.green, second.color.green),
      interpolate(first.color.blue, second.color.blue)};
  }
  return stops.back().color;
}

}  // namespace fr5_scanner_650::world_height_color
