#pragma once

#include <array>
#include <string>

namespace fr5_scanner_650::fixed_axis_scan
{

bool normalizeDirection(
  const std::array<double, 3> & input, bool maintain_base_height,
  std::array<double, 3> * output, std::string * error);

}  // namespace fr5_scanner_650::fixed_axis_scan
