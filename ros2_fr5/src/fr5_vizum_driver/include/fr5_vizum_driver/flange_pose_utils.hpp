#ifndef FR5_VIZUM_DRIVER_FLANGE_POSE_UTILS_HPP_
#define FR5_VIZUM_DRIVER_FLANGE_POSE_UTILS_HPP_

#include <array>

namespace fr5_vizum_driver {

constexpr std::size_t kCartesianDimension = 3;
constexpr std::size_t kPoseDimension = 6;

struct FlangePoseValues {
    std::array<double, kCartesianDimension> translationMeters{};
    std::array<double, kCartesianDimension> rpyRadians{};
};

FlangePoseValues fairinoFlangePoseToRos(
    const std::array<double, kPoseDimension>& poseMillimetersDegrees);

bool allFinite(const std::array<double, kPoseDimension>& values);

} // namespace fr5_vizum_driver

#endif // FR5_VIZUM_DRIVER_FLANGE_POSE_UTILS_HPP_
