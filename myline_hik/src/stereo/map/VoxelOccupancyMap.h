#ifndef MYLINE_HIK_VOXEL_OCCUPANCY_MAP_H
#define MYLINE_HIK_VOXEL_OCCUPANCY_MAP_H

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace hik_stereo {

struct OccupancyMapOptions {
    double voxelSizeMm{25.0};
    int pixelStride{4};
    int maximumRaySteps{250};
    double hitLogOdds{0.85};
    double missLogOdds{-0.4};
    double minimumLogOdds{-2.0};
    double maximumLogOdds{3.5};
    double occupiedThresholdLogOdds{0.6};
};

struct OccupancyMapStatistics {
    std::uint64_t integratedFrames{0};
    std::uint64_t inputDepthPoints{0};
    std::uint64_t integratedEndpoints{0};
    std::uint64_t freeUpdates{0};
    std::uint64_t occupiedUpdates{0};
    std::size_t allocatedVoxels{0};
    std::size_t occupiedVoxels{0};
};

class VoxelOccupancyMap {
public:
    VoxelOccupancyMap();
    ~VoxelOccupancyMap();
    VoxelOccupancyMap(VoxelOccupancyMap&&) noexcept;
    VoxelOccupancyMap& operator=(VoxelOccupancyMap&&) noexcept;
    VoxelOccupancyMap(const VoxelOccupancyMap&) = delete;
    VoxelOccupancyMap& operator=(const VoxelOccupancyMap&) = delete;

    bool configure(const OccupancyMapOptions& options,
                   std::string* error = nullptr);
    void clear();

    bool integrate(const cv::Mat& xyzCameraMm,
                   const cv::Mat& validMask,
                   const cv::Matx44d& baseFromCamera,
                   std::string* error = nullptr);

    bool saveOccupiedPly(const std::string& path,
                         std::string* error = nullptr) const;
    bool save2DGrid(const std::string& pgmPath,
                    const std::string& yamlPath,
                    double minimumHeightMm,
                    double maximumHeightMm,
                    std::string* error = nullptr) const;

    OccupancyMapStatistics statistics() const;
    const OccupancyMapOptions& options() const { return options_; }

private:
    struct Impl;
    Impl* impl_{nullptr};
    OccupancyMapOptions options_;
};

}  // namespace hik_stereo

#endif  // MYLINE_HIK_VOXEL_OCCUPANCY_MAP_H
