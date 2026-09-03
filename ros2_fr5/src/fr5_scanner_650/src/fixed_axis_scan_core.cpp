#include "fixed_axis_scan_core.hpp"

#include <cmath>

namespace fr5_scanner_650::fixed_axis_scan
{

bool normalizeDirection(
  const std::array<double, 3> & input, bool maintain_base_height,
  std::array<double, 3> * output, std::string * error)
{
  if (!output) {
    if (error) {*error = "output direction is null";}
    return false;
  }
  std::array<double, 3> candidate = input;
  if (maintain_base_height) {
    candidate[2] = 0.0;
  }
  const double norm = std::sqrt(
    candidate[0] * candidate[0] + candidate[1] * candidate[1] +
    candidate[2] * candidate[2]);
  if (!std::isfinite(norm) || norm < 1.0e-6) {
    if (error) {
      *error = maintain_base_height ?
        "selected camera axis has no usable projection on the base XY plane" :
        "selected scan direction is invalid";
    }
    return false;
  }
  (*output)[0] = candidate[0] / norm;
  (*output)[1] = candidate[1] / norm;
  (*output)[2] = candidate[2] / norm;
  return true;
}

}  // namespace fr5_scanner_650::fixed_axis_scan
