#include "fr5_vizum_driver/flange_pose_utils.hpp"

#include "fr5_vizum_driver/joint_state_utils.hpp"

#include <cmath>

namespace fr5_vizum_driver {

namespace {

constexpr double kMillimetersToMeters = 0.001;

} // namespace

FlangePoseValues fairinoFlangePoseToRos(
    const std::array<double, kPoseDimension>& poseMillimetersDegrees) {
    FlangePoseValues result;
    for (std::size_t i = 0; i < kCartesianDimension; ++i) {
        result.translationMeters[i] = poseMillimetersDegrees[i] * kMillimetersToMeters;
        result.rpyRadians[i] = degreesToRadians(
            poseMillimetersDegrees[i + kCartesianDimension]);
    }
    return result;
}

bool allFinite(const std::array<double, kPoseDimension>& values) {
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

} // namespace fr5_vizum_driver
