#ifndef MYLINE_HIK_STEREO_MAPPING_PIPELINE_H
#define MYLINE_HIK_STEREO_MAPPING_PIPELINE_H

#include "stereo/core/StereoDepthEngine.h"
#include "stereo/map/VoxelOccupancyMap.h"

#include <opencv2/core.hpp>

#include <string>

namespace hik_stereo {

struct StereoMappingResult {
    bool ok{false};
    std::string error;
    StereoDepthResult depth;
    OccupancyMapStatistics map;
};

// Cohesive algorithm facade: one calibrated stereo observation enters, one
// depth result and one transactional occupancy update leave. Device lifetime,
// Qt rendering and robot commands deliberately remain outside this class.
class StereoMappingPipeline {
public:
    bool configure(const StereoRigCalibration& rig,
                   const StereoDepthOptions& depthOptions,
                   const OccupancyMapOptions& mapOptions,
                   std::string* error = nullptr);

    bool process(const cv::Mat& leftGray,
                 const cv::Mat& rightGray,
                 const cv::Matx44d& baseFromFlange,
                 StereoMappingResult* result);

    void clearMap();
    bool saveMap(const std::string& occupiedPlyPath,
                 const std::string& gridPgmPath,
                 const std::string& gridYamlPath,
                 double minimumGridHeightMm,
                 double maximumGridHeightMm,
                 std::string* error = nullptr) const;

    const StereoRigCalibration& rig() const { return rig_; }
    const StereoDepthEngine& depthEngine() const { return depthEngine_; }
    OccupancyMapStatistics mapStatistics() const { return map_.statistics(); }

private:
    bool configured_{false};
    StereoRigCalibration rig_;
    StereoDepthEngine depthEngine_;
    VoxelOccupancyMap map_;
};

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_MAPPING_PIPELINE_H
