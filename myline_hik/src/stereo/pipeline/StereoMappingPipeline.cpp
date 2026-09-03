#include "stereo/pipeline/StereoMappingPipeline.h"

#include <cmath>

namespace hik_stereo {
namespace {

void setError(const std::string& value, std::string* error) {
    if (error) *error = value;
}

}  // namespace

bool StereoMappingPipeline::configure(
        const StereoRigCalibration& rig,
        const StereoDepthOptions& depthOptions,
        const OccupancyMapOptions& mapOptions,
        std::string* error) {
    configured_ = false;
    std::string detail;
    if (!validateStereoRig(rig, &detail) ||
        !depthEngine_.configure(rig, depthOptions, &detail) ||
        !map_.configure(mapOptions, &detail)) {
        setError(detail, error);
        return false;
    }
    rig_ = rig;
    configured_ = true;
    return true;
}

bool StereoMappingPipeline::process(
        const cv::Mat& leftGray,
        const cv::Mat& rightGray,
        const cv::Matx44d& baseFromFlange,
        StereoMappingResult* result) {
    if (!result) return false;
    *result = StereoMappingResult();
    if (!configured_) {
        result->error = "stereo mapping pipeline is not configured";
        return false;
    }
    for (double value : baseFromFlange.val) {
        if (!std::isfinite(value)) {
            result->error = "base-from-flange transform contains non-finite values";
            return false;
        }
    }
    if (!depthEngine_.compute(leftGray, rightGray, &result->depth)) {
        result->error = result->depth.error;
        return false;
    }
    const cv::Matx44d baseFromLeft =
        baseFromFlange * rig_.left.handEye.flangeFromCamera;
    std::string detail;
    if (!map_.integrate(result->depth.xyzLeftMm,
                        result->depth.validMask,
                        baseFromLeft, &detail)) {
        result->error = detail;
        return false;
    }
    result->map = map_.statistics();
    result->ok = true;
    return true;
}

void StereoMappingPipeline::clearMap() { map_.clear(); }

bool StereoMappingPipeline::saveMap(
        const std::string& occupiedPlyPath,
        const std::string& gridPgmPath,
        const std::string& gridYamlPath,
        double minimumGridHeightMm,
        double maximumGridHeightMm,
        std::string* error) const {
    std::string detail;
    if (!configured_) {
        setError("stereo mapping pipeline is not configured", error);
        return false;
    }
    if (!map_.saveOccupiedPly(occupiedPlyPath, &detail) ||
        !map_.save2DGrid(gridPgmPath, gridYamlPath,
                        minimumGridHeightMm, maximumGridHeightMm,
                        &detail)) {
        setError(detail, error);
        return false;
    }
    return true;
}

}  // namespace hik_stereo
