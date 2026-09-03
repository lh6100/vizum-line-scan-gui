#ifndef MYLINE_HIK_HIK_CONTINUOUS_RECONSTRUCTION_H
#define MYLINE_HIK_HIK_CONTINUOUS_RECONSTRUCTION_H

#include "HikCalibrationCore.h"
#include "HikScanCore.h"
#include "HikSynchronizationCore.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hik_scan {

struct ContinuousReconstructionOptions {
    std::size_t queueCapacity{256U};
    std::size_t workerThreads{4U};
    double voxelSizeMm{0.5};
    bool binaryPly{false};
    std::string outputDirectory;
    std::string intrinsicsSha256;
    std::string laserPlaneSha256;
    std::string handEyeSha256;
    bool saveQualityCloud{true};
    bool retainQualityArtifacts{false};
    // V-groove temporal validation is an explicit adaptive-scanning feature.
    // It is disabled by default so scanner_450's Quality centerline policy
    // cannot silently turn it on or alter the formal raw/voxel cloud.
    bool enableVGrooveTemporalValidation{false};
    bool enableAdjacentProfileSupport{false};
    double adjacentSupportRadiusMm{1.0};
    int adjacentMinimumSupportingProfiles{1};
    int adjacentMaximumProfileGap{2};
};

using ContinuousReconstructionProgressCallback =
    std::function<void(int percent, const std::string& stage)>;

struct ContinuousReconstructionStatistics {
    uint64_t synchronizedFramesSeen{0U};
    uint64_t invalidSyncFramesSkipped{0U};
    uint64_t missingImageFramesSkipped{0U};
    uint64_t invalidTransformFramesSkipped{0U};
    uint64_t queueFullDrops{0U};
    uint64_t queueContentionDrops{0U};
    uint64_t reconstructionTasksStarted{0U};
    uint64_t reconstructedFrames{0U};
    uint64_t reconstructionFailures{0U};
    uint64_t lineQualityWarnings{0U};
    uint64_t multipathAuditOnlyFrames{0U};
    uint64_t qualityFramesPassed{0U};
    uint64_t qualityFramesRejected{0U};
    uint64_t rawPointCount{0U};
    uint64_t voxelPointCount{0U};
    uint64_t qualityMultipathIntervalCount{0U};
    uint64_t qualityMultipathCandidatePointCount{0U};
    uint64_t qualityShadowMaskedLegacyRejectedPointCount{0U};
    uint64_t qualityProfileGateRejectedPointCount{0U};
    uint64_t qualityOpticalRejectedCandidatePointCount{0U};
    uint64_t qualityVGroovePromotedCandidatePointCount{0U};
    uint64_t qualityVGrooveRejectedCandidatePointCount{0U};
    uint64_t qualityVGrooveAmbiguousGroupCount{0U};
    uint64_t qualityVGrooveInsufficientGroupCount{0U};
    uint64_t qualityVGrooveInvalidGeometryGroupCount{0U};
    uint64_t qualityOpticalPointCount{0U};
    uint64_t qualityFilteredPointCount{0U};
    uint64_t qualityAdjacentRejectedPointCount{0U};
    uint64_t qualityRejectedPointCount{0U};
    uint64_t qualityVoxelPointCount{0U};
    double meanReconstructionMs{0.0};
    double maximumReconstructionMs{0.0};
    bool rawPlySaved{false};
    bool voxelPlySaved{false};
    bool qualityOpticalPlySaved{false};
    bool qualityPlySaved{false};
    bool qualityRejectedPlySaved{false};
    bool qualityVoxelPlySaved{false};
    std::string rawPlyPath;
    std::string voxelPlyPath;
    std::string qualityOpticalPlyPath;
    std::string qualityPlyPath;
    std::string qualityRejectedPlyPath;
    std::string qualityVoxelPlyPath;
    std::string detailCsvPath;
    std::string summaryJsonPath;
};

struct ContinuousFrameViewpoint {
    uint64_t frameId{0U};
    int profileIndex{0};
    int segmentId{-1};
    cv::Point3d cameraOriginBaseMm{0.0, 0.0, 0.0};
};

// Optional in-memory handoff for adaptive planning. Formal geometry,
// quality-accepted evidence and rejected evidence remain separate so
// scanner_650 Shadow data cannot be counted twice as surface coverage.
struct ContinuousReconstructionArtifacts {
    std::vector<CloudPoint> formal;
    std::vector<CloudPoint> qualityAccepted;
    std::vector<CloudPoint> rejected;
    std::vector<ContinuousFrameViewpoint> viewpoints;
};

// Bounded, best-effort continuous reconstruction. The synchronization callback
// calls tryEnqueue(), which never waits for a queue mutex or free slot. All
// image processing, cloud accumulation, voxelization and file I/O happen on
// background workers or after acquisition has stopped.
class ContinuousReconstructionPipeline {
public:
    ContinuousReconstructionPipeline();
    ~ContinuousReconstructionPipeline();

    ContinuousReconstructionPipeline(
        const ContinuousReconstructionPipeline&) = delete;
    ContinuousReconstructionPipeline& operator=(
        const ContinuousReconstructionPipeline&) = delete;

    bool start(
        const ContinuousReconstructionOptions& options,
        const hik_calibration::IntrinsicCalibrationResult& intrinsics,
        const hik_calibration::LaserPlaneFitResult& laserPlane,
        const hik_calibration::SingleFrameProfileOptions& profileOptions,
        std::string* error = nullptr);

    // Returns false when the frame is invalid, the fixed-capacity queue is
    // full, or its short critical section is currently busy. It never waits.
    bool tryEnqueue(const hik_sync::SynchronizedFrame& frame) noexcept;

    // Called only after SynchronizationSession::stop() has joined the producer.
    // It drains workers, writes the unchanged formal raw/voxel outputs and,
    // when enabled, the separately named optical-gated, adjacent-supported,
    // rejected and confidence-weighted quality outputs.
    bool stopAndSave(ContinuousReconstructionStatistics* statistics = nullptr,
                     std::string* error = nullptr,
                     ContinuousReconstructionArtifacts* artifacts = nullptr,
                     const ContinuousReconstructionProgressCallback&
                         progress = ContinuousReconstructionProgressCallback());

    bool running() const;
    ContinuousReconstructionStatistics statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ContinuousReconstructionStatistics lastStatistics_;
};

}  // namespace hik_scan

#endif  // MYLINE_HIK_HIK_CONTINUOUS_RECONSTRUCTION_H
