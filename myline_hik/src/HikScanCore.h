#ifndef MYLINE_HIK_HIK_SCAN_CORE_H
#define MYLINE_HIK_HIK_SCAN_CORE_H

#include "HikCalibrationCore.h"

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hik_scan {

struct Pose6D {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;

    Pose6D();
};

bool buildLinearFlangePath(const Pose6D& start,
                           const Pose6D& end,
                           double stepMm,
                           int maximumPointCount,
                           std::vector<Pose6D>* targets,
                           std::string* error = 0);

struct HandEyeFile {
    bool ok;
    std::string error;
    std::string mode;
    std::string parentFrame;
    std::string childFrame;
    std::string cameraSerial;
    std::string intrinsicsSha256;
    cv::Matx44d flangeFromCamera;

    HandEyeFile();
};

bool loadHandEyeYaml(const std::string& path,
                     HandEyeFile* handEye,
                     std::string* error = 0);

struct CloudPoint {
    cv::Point3d basePointMm;
    cv::Point3d cameraPointMm;
    double confidence;
    double response;
    int profileIndex;
    double pixelU;
    double pixelV;
    std::uint32_t qualityFlags;
    std::uint32_t observationCount;

    CloudPoint();
};

enum CloudPointQualityFlag : std::uint32_t {
    CLOUD_QUALITY_NONE = 0U,
    CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED = 1U << 0U,
    CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT = 1U << 1U,
    CLOUD_QUALITY_REJECTED_INVALID_BASE_POINT = 1U << 2U,
    CLOUD_QUALITY_VOXEL_AGGREGATED = 1U << 3U,
    // The 2-D observation passed the local SNR/width/saturation/multi-peak,
    // symmetry, fit and dynamic-programming hard gates.
    CLOUD_QUALITY_OPTICAL_ACCEPTED = 1U << 4U
};

bool cloudPointHasQualityFlag(const CloudPoint& point,
                              CloudPointQualityFlag flag);

bool appendProfileInBase(const hik_calibration::StaticProfileResult& profile,
                         const cv::Matx44d& baseFromFlange,
                         const cv::Matx44d& flangeFromCamera,
                         int profileIndex,
                         std::vector<CloudPoint>* cloud,
                         std::string* error = 0);

// Continuous acquisition already computes T_base_camera at the exposure
// midpoint. Use it directly so reconstruction cannot accidentally fall back to
// a before/after or callback-time robot pose.
bool appendProfileUsingBaseFromCamera(
    const hik_calibration::StaticProfileResult& profile,
    const cv::Matx44d& baseFromCamera,
    int profileIndex,
    std::vector<CloudPoint>* cloud,
    std::string* error = 0);

// Appends an explicitly selected profile-point set. This keeps cloud assembly
// independent from whether the caller selected legacy, shadow-quality or
// quality-gated points.
bool appendProfilePointsUsingBaseFromCamera(
    const std::vector<hik_calibration::StaticProfilePoint>& points,
    const cv::Matx44d& baseFromCamera,
    int profileIndex,
    std::vector<CloudPoint>* cloud,
    std::string* error = 0);

struct VoxelDownsampleOptions {
    double voxelSizeMm;
    bool confidenceWeighted;
    // A zero-confidence observation still receives this small positive
    // geometry weight. This prevents a voxel containing only zero-confidence
    // observations from becoming undefined.
    double minimumConfidenceWeight;

    VoxelDownsampleOptions();
};

struct VoxelDownsampleStatistics {
    std::size_t inputPointCount;
    std::size_t finitePointCount;
    std::size_t rejectedNonFinitePointCount;
    std::size_t outputPointCount;
    bool confidenceWeighted;

    VoxelDownsampleStatistics();
};

// Extended voxel reducer. When confidenceWeighted is true, base/camera
// coordinates, response and source pixels use confidence as their weight.
// Confidence itself remains an arithmetic mean, observationCount is summed
// (with uint32 saturation), and qualityFlags are OR-combined.
std::vector<CloudPoint> voxelDownsample(
    const std::vector<CloudPoint>& cloud,
    const VoxelDownsampleOptions& options,
    VoxelDownsampleStatistics* statistics = 0);

// Compatibility wrapper: preserves the original equal-per-input-point voxel
// averaging semantics.
std::vector<CloudPoint> voxelDownsample(const std::vector<CloudPoint>& cloud,
                                        double voxelSizeMm);

struct AdjacentProfileSupportOptions {
    // Disabled by default so adding this quality stage cannot silently change
    // an existing production cloud.
    bool enabled;
    double radiusMm;
    int minimumSupportingProfiles;
    int maximumProfileGap;

    AdjacentProfileSupportOptions();
};

struct AdjacentProfileSupportStatistics {
    std::size_t inputPointCount;
    std::size_t validPointCount;
    std::size_t keptPointCount;
    std::size_t rejectedPointCount;
    std::size_t invalidPointCount;
    std::size_t insufficientSupportPointCount;

    AdjacentProfileSupportStatistics();
};

struct AdjacentProfileSupportResult {
    bool applied;
    std::vector<CloudPoint> kept;
    std::vector<CloudPoint> rejected;
    AdjacentProfileSupportStatistics statistics;

    AdjacentProfileSupportResult();
};

// Keeps a point only when at least minimumSupportingProfiles distinct, other
// profile indices contain a base-frame point within radiusMm and within
// maximumProfileGap. Run this on raw per-profile points before voxelization;
// a voxel stores only one representative profile index. This filter removes
// sparse/isolated observations, but a coherent false surface repeated across
// adjacent profiles will intentionally pass and needs optical/temporal quality
// evidence at a different layer.
bool filterByAdjacentProfileSupport(
    const std::vector<CloudPoint>& cloud,
    const AdjacentProfileSupportOptions& options,
    AdjacentProfileSupportResult* result,
    std::string* error = 0);

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 std::string* error = 0);

}  // namespace hik_scan

#endif  // MYLINE_HIK_HIK_SCAN_CORE_H
