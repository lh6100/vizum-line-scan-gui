#ifndef FR5_VIZUM_DRIVER_JOINT_STATE_UTILS_HPP_
#define FR5_VIZUM_DRIVER_JOINT_STATE_UTILS_HPP_

#include <array>
#include <string>
#include <vector>

namespace fr5_vizum_driver {

constexpr std::size_t kJointCount = 6;

double degreesToRadians(double degrees);

std::array<double, kJointCount> degreesToRadians(
    const std::array<double, kJointCount>& degrees);

std::vector<std::string> defaultJointNames();

} // namespace fr5_vizum_driver

#endif // FR5_VIZUM_DRIVER_JOINT_STATE_UTILS_HPP_
