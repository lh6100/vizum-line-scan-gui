#include "fr5_vizum_driver/joint_state_utils.hpp"

#include <cmath>

namespace fr5_vizum_driver {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

std::array<double, kJointCount> degreesToRadians(
    const std::array<double, kJointCount>& degrees) {
    std::array<double, kJointCount> radians{};
    for (std::size_t i = 0; i < kJointCount; ++i) {
        radians[i] = degreesToRadians(degrees[i]);
    }
    return radians;
}

std::vector<std::string> defaultJointNames() {
    return {"j1", "j2", "j3", "j4", "j5", "j6"};
}

} // namespace fr5_vizum_driver
