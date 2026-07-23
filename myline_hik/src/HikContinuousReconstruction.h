#ifndef MYLINE_HIK_HIK_CONTINUOUS_RECONSTRUCTION_H
#define MYLINE_HIK_HIK_CONTINUOUS_RECONSTRUCTION_H

#include "HikCalibrationCore.h"
#include "HikSynchronizationCore.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace hik_scan {

struct ContinuousReconstructionOptions {
    std::size_t queueCapacity{64U};
    std::size_t workerThreads{2U};
    double voxelSizeMm{0.5};
    std::string outputDirectory;
    std::string intrinsicsSha256;
    std::string laserPlaneSha256;
    std::string handEyeSha256;
};

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
    uint64_t rawPointCount{0U};
    uint64_t voxelPointCount{0U};
    double meanReconstructionMs{0.0};
    double maximumReconstructionMs{0.0};
    bool rawPlySaved{false};
    bool voxelPlySaved{false};
    std::string rawPlyPath;
    std::string voxelPlyPath;
    std::string detailCsvPath;
    std::string summaryJsonPath;
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
    // It drains workers, writes continuous_raw.ply/continuous_voxel.ply and
    // releases retained image buffers.
    bool stopAndSave(ContinuousReconstructionStatistics* statistics = nullptr,
                     std::string* error = nullptr);

    bool running() const;
    ContinuousReconstructionStatistics statistics() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    ContinuousReconstructionStatistics lastStatistics_;
};

}  // namespace hik_scan

#endif  // MYLINE_HIK_HIK_CONTINUOUS_RECONSTRUCTION_H
