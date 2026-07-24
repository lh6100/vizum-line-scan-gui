#include "HikContinuousReconstruction.h"

#include "HikScanCore.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace hik_scan {
namespace {

void setError(const std::string& message, std::string* error) {
    if (error) *error = message;
}

std::string jsonEscape(const std::string& input) {
    std::ostringstream output;
    for (const unsigned char value : input) {
        switch (value) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (value < 0x20U) {
                output << "\\u" << std::hex << std::setw(4)
                       << std::setfill('0') << static_cast<int>(value)
                       << std::dec << std::setfill(' ');
            } else {
                output << static_cast<char>(value);
            }
        }
    }
    return output.str();
}

std::string csvQuote(const std::string& input) {
    std::string escaped;
    escaped.reserve(input.size() + 2U);
    escaped.push_back('"');
    for (const char value : input) {
        if (value == '"') escaped.push_back('"');
        escaped.push_back(value);
    }
    escaped.push_back('"');
    return escaped;
}

cv::Matx44d eigenToCv(const Eigen::Matrix4d& input) {
    cv::Matx44d output;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            output(row, column) = input(row, column);
        }
    }
    return output;
}

bool validImage(const hik_sync::ImageBuffer& image) {
    if (image.width <= 0 || image.height <= 0 ||
        image.stride < image.width) {
        return false;
    }
    const std::size_t stride = static_cast<std::size_t>(image.stride);
    const std::size_t height = static_cast<std::size_t>(image.height);
    const std::size_t width = static_cast<std::size_t>(image.width);
    if (height > std::numeric_limits<std::size_t>::max() / stride) return false;
    const std::size_t required = (height - 1U) * stride + width;
    return image.bytes.size() >= required;
}

}  // namespace

struct ContinuousReconstructionPipeline::Impl {
    struct Task {
        uint64_t frameId{0U};
        int64_t alignedTimestampNs{0};
        std::shared_ptr<hik_sync::ImageBuffer> image;
        Eigen::Matrix4d baseFromCamera{Eigen::Matrix4d::Identity()};
    };

    struct FrameResult {
        uint64_t frameId{0U};
        int64_t alignedTimestampNs{0};
        bool reconstructed{false};
        bool lineQualityPassed{false};
        bool qualityAttempted{false};
        bool qualityExtractionPassed{false};
        std::size_t pointCount{0U};
        std::size_t qualityPointCount{0U};
        double lineRmsMm{0.0};
        double reconstructionMs{0.0};
        std::string error;
        std::string qualityError;
        std::string centerlineAlgorithmVersion;
        hik_stripe::Diagnostics qualityDiagnostics;
        hik_calibration::StripeShadowComparison shadowComparison;
        std::vector<CloudPoint> points;
        std::vector<CloudPoint> qualityPoints;
    };

    ContinuousReconstructionOptions options;
    hik_calibration::IntrinsicCalibrationResult intrinsics;
    hik_calibration::LaserPlaneFitResult laserPlane;
    hik_calibration::SingleFrameProfileOptions profileOptions;

    std::vector<Task> queue;
    std::size_t queueHead{0U};
    std::size_t queueTail{0U};
    std::size_t queueSize{0U};
    mutable std::mutex queueMutex;
    std::condition_variable queueCondition;
    std::atomic<bool> accepting{false};
    bool stopping{false};
    std::vector<std::thread> workers;

    mutable std::mutex resultMutex;
    std::vector<FrameResult> results;

    std::atomic<uint64_t> synchronizedFramesSeen{0U};
    std::atomic<uint64_t> invalidSyncFramesSkipped{0U};
    std::atomic<uint64_t> missingImageFramesSkipped{0U};
    std::atomic<uint64_t> invalidTransformFramesSkipped{0U};
    std::atomic<uint64_t> queueFullDrops{0U};
    std::atomic<uint64_t> queueContentionDrops{0U};
    std::atomic<uint64_t> reconstructionTasksStarted{0U};
    std::atomic<uint64_t> reconstructedFrames{0U};
    std::atomic<uint64_t> reconstructionFailures{0U};
    std::atomic<uint64_t> lineQualityWarnings{0U};
    std::atomic<uint64_t> qualityFramesPassed{0U};
    std::atomic<uint64_t> qualityFramesRejected{0U};

    bool tryPush(const hik_sync::SynchronizedFrame& frame) noexcept {
        synchronizedFramesSeen.fetch_add(1U, std::memory_order_relaxed);
        if (frame.quality != hik_sync::SyncQuality::VALID) {
            invalidSyncFramesSkipped.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        if (!frame.image || !validImage(*frame.image)) {
            missingImageFramesSkipped.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        if (!frame.hasBaseFromCamera ||
            !frame.baseFromCamera.allFinite()) {
            invalidTransformFramesSkipped.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        if (!accepting.load(std::memory_order_acquire)) return false;

        std::unique_lock<std::mutex> lock(queueMutex, std::try_to_lock);
        if (!lock.owns_lock()) {
            queueContentionDrops.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        if (!accepting.load(std::memory_order_relaxed) ||
            queueSize >= queue.size()) {
            queueFullDrops.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        Task& task = queue[queueTail];
        task.frameId = frame.frameId;
        task.alignedTimestampNs = frame.alignedTimestampNs;
        task.image = frame.image;
        task.baseFromCamera = frame.baseFromCamera;
        queueTail = (queueTail + 1U) % queue.size();
        ++queueSize;
        lock.unlock();
        queueCondition.notify_one();
        return true;
    }

    bool pop(Task* task) {
        if (!task) return false;
        std::unique_lock<std::mutex> lock(queueMutex);
        queueCondition.wait(lock, [this] {
            return stopping || queueSize > 0U;
        });
        if (queueSize == 0U) return false;
        *task = std::move(queue[queueHead]);
        queue[queueHead] = Task{};
        queueHead = (queueHead + 1U) % queue.size();
        --queueSize;
        return true;
    }

    void process(Task task) {
        reconstructionTasksStarted.fetch_add(1U, std::memory_order_relaxed);
        const auto started = std::chrono::steady_clock::now();
        FrameResult result;
        result.frameId = task.frameId;
        result.alignedTimestampNs = task.alignedTimestampNs;

        try {
            if (!task.image || !validImage(*task.image)) {
                result.error =
                    "image buffer became invalid before reconstruction";
            } else {
                const hik_sync::ImageBuffer& buffer = *task.image;
                const cv::Mat gray(
                    buffer.height, buffer.width, CV_8UC1,
                    const_cast<uint8_t*>(buffer.bytes.data()),
                    static_cast<std::size_t>(buffer.stride));
                hik_calibration::StaticProfileResult profile;
                const std::string sampleId =
                    "continuous_frame_" + std::to_string(task.frameId);
                const bool profileOk =
                    hik_calibration::reconstructSingleFrameProfile(
                        gray, sampleId, intrinsics, laserPlane,
                        profileOptions, &profile);
                result.qualityAttempted =
                    profileOptions.reconstruction.stripe.mode !=
                    hik_calibration::StripeExtractionMode::Legacy;
                result.qualityExtractionPassed =
                    profile.qualityExtractionPassed;
                result.qualityError =
                    profile.qualityExtractionError;
                result.centerlineAlgorithmVersion =
                    profile.centerlineAlgorithmVersion;
                result.qualityDiagnostics =
                    profile.qualityDiagnostics;
                result.shadowComparison =
                    profile.shadowComparison;
                if (!profileOk || !profile.ok) {
                    result.error = profile.error.empty()
                        ? "single-frame profile reconstruction failed"
                        : profile.error;
                } else {
                    std::string appendError;
                    const cv::Matx44d baseFromCamera =
                        eigenToCv(task.baseFromCamera);
                    const int profileIndex =
                        task.frameId <= static_cast<uint64_t>(
                            std::numeric_limits<int>::max())
                        ? static_cast<int>(task.frameId)
                        : std::numeric_limits<int>::max();
                    if (!appendProfileUsingBaseFromCamera(
                            profile, baseFromCamera, profileIndex,
                            &result.points, &appendError) ||
                        result.points.empty()) {
                        result.error = appendError.empty()
                            ? "profile produced no finite base-frame points"
                            : appendError;
                    } else {
                        result.reconstructed = true;
                        result.lineQualityPassed =
                            profile.lineQualityPassed;
                        result.lineRmsMm = profile.lineDistanceMm.rms;
                        result.pointCount = result.points.size();
                        if (options.saveQualityCloud &&
                            profile.qualityExtractionPassed &&
                            !profile.qualityPoints.empty()) {
                            std::string qualityAppendError;
                            if (!appendProfilePointsUsingBaseFromCamera(
                                    profile.qualityPoints,
                                    baseFromCamera, profileIndex,
                                    &result.qualityPoints,
                                    &qualityAppendError) ||
                                result.qualityPoints.empty()) {
                                result.qualityExtractionPassed = false;
                                result.qualityError =
                                    qualityAppendError.empty()
                                        ? "quality profile produced no finite "
                                          "base-frame points"
                                        : qualityAppendError;
                            } else {
                                result.qualityPointCount =
                                    result.qualityPoints.size();
                            }
                        }
                    }
                }
            }
        } catch (const cv::Exception& exception) {
            result.reconstructed = false;
            result.points.clear();
            result.qualityPoints.clear();
            result.pointCount = 0U;
            result.qualityPointCount = 0U;
            result.error =
                std::string("OpenCV exception: ") + exception.what();
        } catch (const std::exception& exception) {
            result.reconstructed = false;
            result.points.clear();
            result.qualityPoints.clear();
            result.pointCount = 0U;
            result.qualityPointCount = 0U;
            result.error =
                std::string("reconstruction exception: ") + exception.what();
        } catch (...) {
            result.reconstructed = false;
            result.points.clear();
            result.qualityPoints.clear();
            result.pointCount = 0U;
            result.qualityPointCount = 0U;
            result.error = "unknown reconstruction exception";
        }

        result.reconstructionMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
        if (result.reconstructed) {
            reconstructedFrames.fetch_add(1U, std::memory_order_relaxed);
            if (!result.lineQualityPassed) {
                lineQualityWarnings.fetch_add(1U, std::memory_order_relaxed);
            }
        } else {
            reconstructionFailures.fetch_add(1U, std::memory_order_relaxed);
        }
        if (result.qualityAttempted) {
            if (result.qualityExtractionPassed) {
                qualityFramesPassed.fetch_add(
                    1U, std::memory_order_relaxed);
            } else {
                qualityFramesRejected.fetch_add(
                    1U, std::memory_order_relaxed);
            }
        }
        std::lock_guard<std::mutex> lock(resultMutex);
        results.push_back(std::move(result));
    }

    void workerLoop() {
        Task task;
        while (pop(&task)) {
            process(std::move(task));
            task = Task{};
        }
    }

    ContinuousReconstructionStatistics snapshot() const {
        ContinuousReconstructionStatistics statistics;
        statistics.synchronizedFramesSeen =
            synchronizedFramesSeen.load(std::memory_order_relaxed);
        statistics.invalidSyncFramesSkipped =
            invalidSyncFramesSkipped.load(std::memory_order_relaxed);
        statistics.missingImageFramesSkipped =
            missingImageFramesSkipped.load(std::memory_order_relaxed);
        statistics.invalidTransformFramesSkipped =
            invalidTransformFramesSkipped.load(std::memory_order_relaxed);
        statistics.queueFullDrops =
            queueFullDrops.load(std::memory_order_relaxed);
        statistics.queueContentionDrops =
            queueContentionDrops.load(std::memory_order_relaxed);
        statistics.reconstructionTasksStarted =
            reconstructionTasksStarted.load(std::memory_order_relaxed);
        statistics.reconstructedFrames =
            reconstructedFrames.load(std::memory_order_relaxed);
        statistics.reconstructionFailures =
            reconstructionFailures.load(std::memory_order_relaxed);
        statistics.lineQualityWarnings =
            lineQualityWarnings.load(std::memory_order_relaxed);
        statistics.qualityFramesPassed =
            qualityFramesPassed.load(std::memory_order_relaxed);
        statistics.qualityFramesRejected =
            qualityFramesRejected.load(std::memory_order_relaxed);
        statistics.rawPlyPath =
            options.outputDirectory + "/continuous_raw.ply";
        statistics.voxelPlyPath =
            options.outputDirectory + "/continuous_voxel.ply";
        statistics.qualityOpticalPlyPath =
            options.outputDirectory + "/continuous_quality_optical.ply";
        statistics.qualityPlyPath =
            options.outputDirectory + "/continuous_quality_filtered.ply";
        statistics.qualityRejectedPlyPath =
            options.outputDirectory + "/continuous_quality_rejected.ply";
        statistics.qualityVoxelPlyPath =
            options.outputDirectory + "/continuous_quality_voxel.ply";
        statistics.detailCsvPath =
            options.outputDirectory + "/continuous_reconstruction.csv";
        statistics.summaryJsonPath =
            options.outputDirectory + "/continuous_reconstruction_summary.json";
        return statistics;
    }

    bool writeDetailCsv(const std::vector<FrameResult>& sorted,
                        const std::string& path,
                        std::string* error) const {
        std::ofstream output(path, std::ios::out | std::ios::trunc);
        if (!output) {
            setError("cannot create continuous reconstruction CSV: " + path,
                     error);
            return false;
        }
        output << "frame_id,aligned_timestamp_ns,status,point_count,"
                  "line_quality_passed,line_rms_mm,quality_attempted,"
                  "quality_passed,quality_point_count,total_candidates,"
                  "accepted_candidates,selected_points,selected_gaps,"
                  "saturated_candidates,multi_peak_scanlines,"
                  "rejected_low_prominence,rejected_width,"
                  "rejected_saturation,rejected_multi_peak,"
                  "rejected_asymmetry,rejected_fit,rejected_quality,"
                  "rejected_mask,ambiguous_path_points,"
                  "mean_selected_quality,mean_selected_fwhm_px,"
                  "mean_selected_snr,mean_selected_gradient_asymmetry,"
                  "mean_selected_fit_residual,"
                  "mean_selected_second_peak_ratio,"
                  "selected_saturated_ratio,"
                  "best_path_cost,second_path_cost,"
                  "path_cost_margin_per_point,"
                  "offset_matched_points,offset_robust_matched_points,"
                  "offset_gross_mismatch_points,offset_signed_mean_px,"
                  "offset_signed_median_px,offset_robust_signed_mean_px,"
                  "offset_robust_gate_px,"
                  "offset_abs_median_px,offset_abs_p95_px,"
                  "offset_abs_max_px,centerline_algorithm,"
                  "reconstruction_ms,quality_error,error\n";
        output << std::setprecision(12);
        for (const FrameResult& result : sorted) {
            output << result.frameId << ',' << result.alignedTimestampNs << ','
                   << (result.reconstructed ? "RECONSTRUCTED" : "FAILED") << ','
                   << result.pointCount << ','
                   << (result.lineQualityPassed ? 1 : 0) << ','
                   << result.lineRmsMm << ','
                   << (result.qualityAttempted ? 1 : 0) << ','
                   << (result.qualityExtractionPassed ? 1 : 0) << ','
                   << result.qualityPointCount << ','
                   << result.qualityDiagnostics.totalCandidateCount << ','
                   << result.qualityDiagnostics.acceptedCandidateCount << ','
                   << result.qualityDiagnostics.selectedPointCount << ','
                   << result.qualityDiagnostics.selectedGapCount << ','
                   << result.qualityDiagnostics.saturatedCandidateCount << ','
                   << result.qualityDiagnostics.multiPeakScanlineCount << ','
                   << result.qualityDiagnostics.rejectedLowProminenceCount
                   << ','
                   << result.qualityDiagnostics.rejectedWidthCount << ','
                   << result.qualityDiagnostics.rejectedSaturationCount << ','
                   << result.qualityDiagnostics.rejectedMultiPeakCount << ','
                   << result.qualityDiagnostics.rejectedAsymmetryCount << ','
                   << result.qualityDiagnostics.rejectedFitCount << ','
                   << result.qualityDiagnostics.rejectedQualityCount << ','
                   << result.qualityDiagnostics.rejectedMaskCount << ','
                   << result.qualityDiagnostics.ambiguousPathPointCount << ','
                   << result.qualityDiagnostics.meanSelectedQuality << ','
                   << result.qualityDiagnostics.meanSelectedFwhmPx << ','
                   << result.qualityDiagnostics.meanSelectedSnr << ','
                   << result.qualityDiagnostics
                          .meanSelectedGradientAsymmetry << ','
                   << result.qualityDiagnostics.meanSelectedFitResidual
                   << ','
                   << result.qualityDiagnostics
                          .meanSelectedSecondPeakRatio << ','
                   << result.qualityDiagnostics.selectedSaturatedRatio << ','
                   << result.qualityDiagnostics.bestPathCost << ','
                   << result.qualityDiagnostics.secondPathCost << ','
                   << result.qualityDiagnostics.pathCostMarginPerPoint << ','
                   << result.shadowComparison.matchedPointCount << ','
                   << result.shadowComparison.robustMatchedPointCount << ','
                   << result.shadowComparison.grossMismatchPointCount << ','
                   << result.shadowComparison.signedMeanOffsetPx << ','
                   << result.shadowComparison.signedMedianOffsetPx << ','
                   << result.shadowComparison.robustSignedMeanOffsetPx << ','
                   << result.shadowComparison.robustGatePx << ','
                   << result.shadowComparison.absoluteMedianOffsetPx << ','
                   << result.shadowComparison.absoluteP95OffsetPx << ','
                   << result.shadowComparison.absoluteMaximumOffsetPx << ','
                   << csvQuote(result.centerlineAlgorithmVersion) << ','
                   << result.reconstructionMs << ','
                   << csvQuote(result.qualityError) << ','
                   << csvQuote(result.error) << '\n';
        }
        if (!output.good()) {
            setError("failed while writing continuous reconstruction CSV: " +
                     path, error);
            return false;
        }
        return true;
    }

    void writeSummary(const ContinuousReconstructionStatistics& statistics,
                      const std::string& saveError) const {
        std::ofstream output(statistics.summaryJsonPath,
                             std::ios::out | std::ios::trunc);
        if (!output) return;
        output << std::setprecision(12)
               << "{\n"
               << "  \"queue_capacity\": " << options.queueCapacity << ",\n"
               << "  \"worker_threads\": " << options.workerThreads << ",\n"
               << "  \"voxel_size_mm\": " << options.voxelSizeMm << ",\n"
               << "  \"save_quality_cloud\": "
               << (options.saveQualityCloud ? "true" : "false") << ",\n"
               << "  \"adjacent_profile_support_enabled\": "
               << (options.enableAdjacentProfileSupport
                       ? "true" : "false") << ",\n"
               << "  \"adjacent_support_radius_mm\": "
               << options.adjacentSupportRadiusMm << ",\n"
               << "  \"adjacent_minimum_supporting_profiles\": "
               << options.adjacentMinimumSupportingProfiles << ",\n"
               << "  \"adjacent_maximum_profile_gap\": "
               << options.adjacentMaximumProfileGap << ",\n"
               << "  \"synchronized_frames_seen\": "
               << statistics.synchronizedFramesSeen << ",\n"
               << "  \"invalid_sync_frames_skipped\": "
               << statistics.invalidSyncFramesSkipped << ",\n"
               << "  \"missing_image_frames_skipped\": "
               << statistics.missingImageFramesSkipped << ",\n"
               << "  \"invalid_transform_frames_skipped\": "
               << statistics.invalidTransformFramesSkipped << ",\n"
               << "  \"queue_full_drops\": "
               << statistics.queueFullDrops << ",\n"
               << "  \"queue_contention_drops\": "
               << statistics.queueContentionDrops << ",\n"
               << "  \"reconstruction_tasks_started\": "
               << statistics.reconstructionTasksStarted << ",\n"
               << "  \"reconstructed_frames\": "
               << statistics.reconstructedFrames << ",\n"
               << "  \"reconstruction_failures\": "
               << statistics.reconstructionFailures << ",\n"
               << "  \"line_quality_warnings\": "
               << statistics.lineQualityWarnings << ",\n"
               << "  \"quality_frames_passed\": "
               << statistics.qualityFramesPassed << ",\n"
               << "  \"quality_frames_rejected\": "
               << statistics.qualityFramesRejected << ",\n"
               << "  \"raw_point_count\": "
               << statistics.rawPointCount << ",\n"
               << "  \"voxel_point_count\": "
               << statistics.voxelPointCount << ",\n"
               << "  \"quality_optical_point_count\": "
               << statistics.qualityOpticalPointCount << ",\n"
               << "  \"quality_filtered_point_count\": "
               << statistics.qualityFilteredPointCount << ",\n"
               << "  \"quality_rejected_point_count\": "
               << statistics.qualityRejectedPointCount << ",\n"
               << "  \"quality_voxel_point_count\": "
               << statistics.qualityVoxelPointCount << ",\n"
               << "  \"mean_reconstruction_ms\": "
               << statistics.meanReconstructionMs << ",\n"
               << "  \"maximum_reconstruction_ms\": "
               << statistics.maximumReconstructionMs << ",\n"
               << "  \"raw_ply_saved\": "
               << (statistics.rawPlySaved ? "true" : "false") << ",\n"
               << "  \"voxel_ply_saved\": "
               << (statistics.voxelPlySaved ? "true" : "false") << ",\n"
               << "  \"quality_optical_ply_saved\": "
               << (statistics.qualityOpticalPlySaved
                       ? "true" : "false") << ",\n"
               << "  \"quality_filtered_ply_saved\": "
               << (statistics.qualityPlySaved ? "true" : "false") << ",\n"
               << "  \"quality_rejected_ply_saved\": "
               << (statistics.qualityRejectedPlySaved
                       ? "true" : "false") << ",\n"
               << "  \"quality_voxel_ply_saved\": "
               << (statistics.qualityVoxelPlySaved
                       ? "true" : "false") << ",\n"
               << "  \"raw_ply_path\": \""
               << jsonEscape(statistics.rawPlyPath) << "\",\n"
               << "  \"voxel_ply_path\": \""
               << jsonEscape(statistics.voxelPlyPath) << "\",\n"
               << "  \"quality_optical_ply_path\": \""
               << jsonEscape(statistics.qualityOpticalPlyPath) << "\",\n"
               << "  \"quality_filtered_ply_path\": \""
               << jsonEscape(statistics.qualityPlyPath) << "\",\n"
               << "  \"quality_rejected_ply_path\": \""
               << jsonEscape(statistics.qualityRejectedPlyPath) << "\",\n"
               << "  \"quality_voxel_ply_path\": \""
               << jsonEscape(statistics.qualityVoxelPlyPath) << "\",\n"
               << "  \"intrinsics_sha256\": \""
               << jsonEscape(options.intrinsicsSha256) << "\",\n"
               << "  \"laser_plane_sha256\": \""
               << jsonEscape(options.laserPlaneSha256) << "\",\n"
               << "  \"handeye_sha256\": \""
               << jsonEscape(options.handEyeSha256) << "\",\n"
               << "  \"error\": \"" << jsonEscape(saveError) << "\"\n"
               << "}\n";
    }
};

ContinuousReconstructionPipeline::ContinuousReconstructionPipeline() = default;

ContinuousReconstructionPipeline::~ContinuousReconstructionPipeline() {
    if (!impl_) return;
    impl_->accepting.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(impl_->queueMutex);
        impl_->stopping = true;
    }
    impl_->queueCondition.notify_all();
    for (std::thread& worker : impl_->workers) {
        if (worker.joinable()) worker.join();
    }
}

bool ContinuousReconstructionPipeline::start(
        const ContinuousReconstructionOptions& options,
        const hik_calibration::IntrinsicCalibrationResult& intrinsics,
        const hik_calibration::LaserPlaneFitResult& laserPlane,
        const hik_calibration::SingleFrameProfileOptions& profileOptions,
        std::string* error) {
    if (impl_) {
        setError("continuous reconstruction is already running", error);
        return false;
    }
    if (options.queueCapacity == 0U || options.workerThreads == 0U ||
        options.workerThreads > 16U || options.outputDirectory.empty() ||
        !std::isfinite(options.voxelSizeMm) || options.voxelSizeMm < 0.0 ||
        (options.enableAdjacentProfileSupport &&
         (!std::isfinite(options.adjacentSupportRadiusMm) ||
          options.adjacentSupportRadiusMm <= 0.0 ||
          options.adjacentMinimumSupportingProfiles < 1 ||
          options.adjacentMaximumProfileGap < 1)) ||
        !intrinsics.ok || !laserPlane.ok) {
        setError("invalid continuous reconstruction configuration or calibration",
                 error);
        return false;
    }
    std::error_code directoryError;
    std::filesystem::create_directories(options.outputDirectory, directoryError);
    if (directoryError) {
        setError("cannot create continuous reconstruction output directory: " +
                 directoryError.message(), error);
        return false;
    }

    std::unique_ptr<Impl> created;
    try {
        created = std::make_unique<Impl>();
        created->options = options;
        created->intrinsics = intrinsics;
        created->laserPlane = laserPlane;
        created->profileOptions = profileOptions;
        created->queue.resize(options.queueCapacity);
        created->results.reserve(options.queueCapacity);
        created->accepting.store(true, std::memory_order_release);
        created->workers.reserve(options.workerThreads);
        for (std::size_t index = 0U; index < options.workerThreads; ++index) {
            created->workers.emplace_back(
                [pointer = created.get()] { pointer->workerLoop(); });
        }
    } catch (const std::exception& exception) {
        if (created) {
            created->accepting.store(false, std::memory_order_release);
            {
                std::lock_guard<std::mutex> lock(created->queueMutex);
                created->stopping = true;
            }
            created->queueCondition.notify_all();
            for (std::thread& worker : created->workers) {
                if (worker.joinable()) worker.join();
            }
        }
        setError(std::string("cannot start continuous reconstruction workers: ") +
                 exception.what(), error);
        return false;
    }
    impl_ = std::move(created);
    lastStatistics_ = ContinuousReconstructionStatistics{};
    if (error) error->clear();
    return true;
}

bool ContinuousReconstructionPipeline::tryEnqueue(
        const hik_sync::SynchronizedFrame& frame) noexcept {
    Impl* implementation = impl_.get();
    return implementation && implementation->tryPush(frame);
}

bool ContinuousReconstructionPipeline::stopAndSave(
        ContinuousReconstructionStatistics* statistics,
        std::string* error) {
    if (!impl_) {
        if (statistics) *statistics = lastStatistics_;
        setError("continuous reconstruction is not running", error);
        return false;
    }
    Impl* implementation = impl_.get();
    implementation->accepting.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(implementation->queueMutex);
        implementation->stopping = true;
    }
    implementation->queueCondition.notify_all();
    for (std::thread& worker : implementation->workers) {
        if (worker.joinable()) worker.join();
    }

    std::vector<Impl::FrameResult> sorted;
    {
        std::lock_guard<std::mutex> lock(implementation->resultMutex);
        sorted = std::move(implementation->results);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const Impl::FrameResult& first,
                 const Impl::FrameResult& second) {
                  return first.frameId < second.frameId;
              });

    ContinuousReconstructionStatistics finalStatistics =
        implementation->snapshot();
    std::vector<CloudPoint> rawCloud;
    std::vector<CloudPoint> qualityOpticalCloud;
    std::size_t rawPointCount = 0U;
    std::size_t qualityOpticalPointCount = 0U;
    long double durationSumMs = 0.0L;
    for (const Impl::FrameResult& result : sorted) {
        rawPointCount += result.points.size();
        qualityOpticalPointCount += result.qualityPoints.size();
        durationSumMs += result.reconstructionMs;
        finalStatistics.maximumReconstructionMs = std::max(
            finalStatistics.maximumReconstructionMs,
            result.reconstructionMs);
    }
    if (!sorted.empty()) {
        finalStatistics.meanReconstructionMs =
            static_cast<double>(durationSumMs /
                                static_cast<long double>(sorted.size()));
    }
    rawCloud.reserve(rawPointCount);
    qualityOpticalCloud.reserve(qualityOpticalPointCount);
    for (Impl::FrameResult& result : sorted) {
        rawCloud.insert(rawCloud.end(),
                        std::make_move_iterator(result.points.begin()),
                        std::make_move_iterator(result.points.end()));
        qualityOpticalCloud.insert(
            qualityOpticalCloud.end(),
            std::make_move_iterator(result.qualityPoints.begin()),
            std::make_move_iterator(result.qualityPoints.end()));
    }
    finalStatistics.rawPointCount = rawCloud.size();
    finalStatistics.qualityOpticalPointCount =
        qualityOpticalCloud.size();

    std::string combinedError;
    std::string detailError;
    if (!implementation->writeDetailCsv(
            sorted, finalStatistics.detailCsvPath, &detailError)) {
        combinedError = detailError;
    }

    if (rawCloud.empty()) {
        if (!combinedError.empty()) combinedError += "; ";
        combinedError += "continuous reconstruction produced an empty cloud";
    } else {
        std::string saveError;
        finalStatistics.rawPlySaved = saveScanPly(
            finalStatistics.rawPlyPath, rawCloud, "base_link", &saveError);
        if (!finalStatistics.rawPlySaved) {
            if (!combinedError.empty()) combinedError += "; ";
            combinedError += saveError;
        }
        const std::vector<CloudPoint> voxelCloud =
            voxelDownsample(rawCloud, implementation->options.voxelSizeMm);
        finalStatistics.voxelPointCount = voxelCloud.size();
        saveError.clear();
        finalStatistics.voxelPlySaved = saveScanPly(
            finalStatistics.voxelPlyPath, voxelCloud, "base_link", &saveError);
        if (!finalStatistics.voxelPlySaved) {
            if (!combinedError.empty()) combinedError += "; ";
            combinedError += saveError;
        }
    }

    if (implementation->options.saveQualityCloud &&
        !qualityOpticalCloud.empty()) {
        std::string saveError;
        finalStatistics.qualityOpticalPlySaved = saveScanPly(
            finalStatistics.qualityOpticalPlyPath,
            qualityOpticalCloud, "base_link", &saveError);
        if (!finalStatistics.qualityOpticalPlySaved) {
            if (!combinedError.empty()) combinedError += "; ";
            combinedError += saveError;
        }

        AdjacentProfileSupportOptions supportOptions;
        supportOptions.enabled =
            implementation->options.enableAdjacentProfileSupport;
        supportOptions.radiusMm =
            implementation->options.adjacentSupportRadiusMm;
        supportOptions.minimumSupportingProfiles =
            implementation->options.adjacentMinimumSupportingProfiles;
        supportOptions.maximumProfileGap =
            implementation->options.adjacentMaximumProfileGap;
        AdjacentProfileSupportResult support;
        std::string supportError;
        if (!filterByAdjacentProfileSupport(
                qualityOpticalCloud, supportOptions,
                &support, &supportError)) {
            if (!combinedError.empty()) combinedError += "; ";
            combinedError +=
                "quality adjacent-profile filter failed: " +
                supportError;
        } else {
            finalStatistics.qualityFilteredPointCount =
                support.kept.size();
            finalStatistics.qualityRejectedPointCount =
                support.rejected.size();
            if (!support.kept.empty()) {
                saveError.clear();
                finalStatistics.qualityPlySaved = saveScanPly(
                    finalStatistics.qualityPlyPath,
                    support.kept, "base_link", &saveError);
                if (!finalStatistics.qualityPlySaved) {
                    if (!combinedError.empty()) combinedError += "; ";
                    combinedError += saveError;
                }

                VoxelDownsampleOptions voxelOptions;
                voxelOptions.voxelSizeMm =
                    implementation->options.voxelSizeMm;
                voxelOptions.confidenceWeighted = true;
                const std::vector<CloudPoint> qualityVoxelCloud =
                    voxelDownsample(support.kept, voxelOptions);
                finalStatistics.qualityVoxelPointCount =
                    qualityVoxelCloud.size();
                if (!qualityVoxelCloud.empty()) {
                    saveError.clear();
                    finalStatistics.qualityVoxelPlySaved = saveScanPly(
                        finalStatistics.qualityVoxelPlyPath,
                        qualityVoxelCloud, "base_link", &saveError);
                    if (!finalStatistics.qualityVoxelPlySaved) {
                        if (!combinedError.empty()) combinedError += "; ";
                        combinedError += saveError;
                    }
                }
            }
            if (!support.rejected.empty()) {
                saveError.clear();
                finalStatistics.qualityRejectedPlySaved = saveScanPly(
                    finalStatistics.qualityRejectedPlyPath,
                    support.rejected, "base_link", &saveError);
                if (!finalStatistics.qualityRejectedPlySaved) {
                    if (!combinedError.empty()) combinedError += "; ";
                    combinedError += saveError;
                }
            }
        }
    }
    implementation->writeSummary(finalStatistics, combinedError);

    lastStatistics_ = finalStatistics;
    impl_.reset();
    if (statistics) *statistics = finalStatistics;
    if (error) *error = combinedError;
    return combinedError.empty() &&
           finalStatistics.rawPlySaved &&
           finalStatistics.voxelPlySaved;
}

bool ContinuousReconstructionPipeline::running() const {
    return impl_ && impl_->accepting.load(std::memory_order_acquire);
}

ContinuousReconstructionStatistics
ContinuousReconstructionPipeline::statistics() const {
    return impl_ ? impl_->snapshot() : lastStatistics_;
}

}  // namespace hik_scan
