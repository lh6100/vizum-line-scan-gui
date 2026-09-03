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
#include <set>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace hik_scan {
namespace {

void setError(const std::string& message, std::string* error) {
    if (error) *error = message;
}

bool finitePoint(const cv::Point3d& point) {
    return std::isfinite(point.x) &&
           std::isfinite(point.y) &&
           std::isfinite(point.z);
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

std::uint64_t ambiguityGroupId(int profileIndex, int intervalId) {
    return
        (static_cast<std::uint64_t>(
             static_cast<std::uint32_t>(profileIndex)) << 32U) |
        static_cast<std::uint32_t>(intervalId);
}

}  // namespace

struct ContinuousReconstructionPipeline::Impl {
    struct Task {
        uint64_t frameId{0U};
        int segmentId{-1};
        int64_t alignedTimestampNs{0};
        std::shared_ptr<hik_sync::ImageBuffer> image;
        Eigen::Matrix4d baseFromCamera{Eigen::Matrix4d::Identity()};
    };

    struct FrameResult {
        uint64_t frameId{0U};
        int segmentId{-1};
        int64_t alignedTimestampNs{0};
        bool reconstructed{false};
        bool lineQualityPassed{false};
        bool qualityAttempted{false};
        bool qualityAnalysisCompleted{false};
        bool qualityExtractionPassed{false};
        bool multipathAuditOnly{false};
        std::size_t pointCount{0U};
        std::size_t qualityPointCount{0U};
        std::size_t qualityAmbiguousIntervalCount{0U};
        std::size_t qualityAmbiguousCandidatePointCount{0U};
        std::size_t ambiguityMaskedLegacyPointCount{0U};
        double lineRmsMm{0.0};
        double reconstructionMs{0.0};
        std::string error;
        std::string qualityError;
        std::string centerlineAlgorithmVersion;
        cv::Point3d cameraOriginBaseMm{0.0, 0.0, 0.0};
        hik_stripe::Diagnostics qualityDiagnostics;
        hik_calibration::StripeShadowComparison shadowComparison;
        std::vector<CloudPoint> points;
        std::vector<CloudPoint> qualityPoints;
        std::vector<CloudPoint> shadowMaskedLegacyRejectedPoints;
        std::vector<CloudPoint> publicationGateRejectedPoints;
        std::vector<CloudPoint> qualityRejectedCandidatePoints;
        std::vector<VGrooveCandidateBranch> qualityAmbiguousBranches;
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
        task.segmentId = frame.measurementSegmentId;
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
        result.segmentId = task.segmentId;
        result.alignedTimestampNs = task.alignedTimestampNs;
        result.cameraOriginBaseMm = cv::Point3d(
            task.baseFromCamera(0, 3),
            task.baseFromCamera(1, 3),
            task.baseFromCamera(2, 3));

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
                result.qualityAnalysisCompleted =
                    profile.qualityAnalysisCompleted;
                result.qualityExtractionPassed =
                    profile.qualityExtractionPassed;
                result.multipathAuditOnly =
                    profile.multipathAuditOnly;
                result.qualityError =
                    profile.qualityExtractionError;
                result.centerlineAlgorithmVersion =
                    profile.centerlineAlgorithmVersion;
                result.qualityDiagnostics =
                    profile.qualityDiagnostics;
                result.shadowComparison =
                    profile.shadowComparison;
                result.ambiguityMaskedLegacyPointCount =
                    profile.ambiguityMaskedLegacyPoints.size();
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
                    const bool qualityArtifactsEnabled =
                        options.saveQualityCloud ||
                        options.retainQualityArtifacts;
                    const bool collectVGrooveEvidence =
                        options.enableVGrooveTemporalValidation;
                    const bool formalAppendFailed =
                        !profile.points.empty() &&
                        (!appendProfileUsingBaseFromCamera(
                             profile, baseFromCamera, profileIndex,
                             &result.points, &appendError) ||
                         result.points.empty());
                    if (formalAppendFailed ||
                        (profile.points.empty() &&
                         !profile.multipathAuditOnly)) {
                        result.error = appendError.empty()
                            ? "profile produced neither formal base-frame "
                              "points nor a multipath audit-only result"
                            : appendError;
                    } else {
                        result.reconstructed = true;
                        result.lineQualityPassed =
                            profile.lineQualityPassed;
                        result.lineRmsMm = profile.lineDistanceMm.rms;
                        result.pointCount = result.points.size();
                        if (qualityArtifactsEnabled &&
                            profile.qualityExtractionPassed &&
                            !profile.multipathAuditOnly &&
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
                        bool multipathAuditFailed = false;
                        std::string multipathAuditError;
                        if (qualityArtifactsEnabled &&
                            !profile.ambiguityMaskedLegacyPoints.empty()) {
                            std::string maskedLegacyAppendError;
                            if (!appendProfilePointsUsingBaseFromCamera(
                                    profile.ambiguityMaskedLegacyPoints,
                                    baseFromCamera, profileIndex,
                                    &result
                                         .shadowMaskedLegacyRejectedPoints,
                                    &maskedLegacyAppendError) ||
                                result.shadowMaskedLegacyRejectedPoints
                                    .empty()) {
                                multipathAuditFailed = true;
                                multipathAuditError =
                                    maskedLegacyAppendError.empty()
                                    ? "shadow-masked legacy points produced "
                                      "no finite base-frame audit points"
                                    : maskedLegacyAppendError;
                            } else {
                                for (CloudPoint& point :
                                     result
                                         .shadowMaskedLegacyRejectedPoints) {
                                    point.qualityFlags |=
                                        CLOUD_QUALITY_REJECTED_SHADOW_LEGACY_MULTIPATH;
                                }
                            }
                        }
                        if (qualityArtifactsEnabled &&
                            !multipathAuditFailed &&
                            !profile.publicationGateRejectedPoints.empty()) {
                            std::string publicationGateAppendError;
                            if (!appendProfilePointsUsingBaseFromCamera(
                                    profile.publicationGateRejectedPoints,
                                    baseFromCamera, profileIndex,
                                    &result.publicationGateRejectedPoints,
                                    &publicationGateAppendError) ||
                                result.publicationGateRejectedPoints.empty()) {
                                multipathAuditFailed = true;
                                multipathAuditError =
                                    publicationGateAppendError.empty()
                                    ? "publication-gate rejected points "
                                      "produced no finite base-frame audit "
                                      "points"
                                    : publicationGateAppendError;
                            } else {
                                for (CloudPoint& point :
                                     result
                                         .publicationGateRejectedPoints) {
                                    point.qualityFlags |=
                                        CLOUD_QUALITY_REJECTED_PROFILE_PUBLICATION_GATE;
                                }
                            }
                        }
                        if (qualityArtifactsEnabled &&
                            !multipathAuditFailed &&
                            !profile.qualityRejectedCandidatePoints.empty()) {
                            std::string rejectedCandidateAppendError;
                            if (!appendProfilePointsUsingBaseFromCamera(
                                    profile.qualityRejectedCandidatePoints,
                                    baseFromCamera, profileIndex,
                                    &result.qualityRejectedCandidatePoints,
                                    &rejectedCandidateAppendError)) {
                                multipathAuditFailed = true;
                                multipathAuditError =
                                    rejectedCandidateAppendError.empty()
                                    ? "quality rejected candidates produced "
                                      "no finite base-frame audit points"
                                    : rejectedCandidateAppendError;
                            }
                        }
                        std::set<int> ambiguousIntervals;
                        for (const hik_calibration::
                                 StaticProfileAmbiguousBranch& input :
                             profile.qualityAmbiguousBranches) {
                            if (!collectVGrooveEvidence) {
                                break;
                            }
                            if (multipathAuditFailed) {
                                break;
                            }
                            if (input.points.empty()) {
                                continue;
                            }
                            VGrooveCandidateBranch branch;
                            branch.ambiguityGroupId =
                                ambiguityGroupId(
                                    profileIndex,
                                    input.intervalId);
                            branch.branchId = input.branchId;
                            branch.formalPublicationEligible =
                                result.qualityExtractionPassed &&
                                !profile.multipathAuditOnly;
                            std::string ambiguousAppendError;
                            if (!appendProfilePointsUsingBaseFromCamera(
                                    input.points, baseFromCamera,
                                    profileIndex, &branch.points,
                                    &ambiguousAppendError) ||
                                branch.points.empty()) {
                                if (!result.qualityError.empty()) {
                                    result.qualityError += "; ";
                                }
                                result.qualityError +=
                                    ambiguousAppendError.empty()
                                    ? "multipath branch produced no finite "
                                      "base-frame points"
                                    : ambiguousAppendError;
                                multipathAuditFailed = true;
                                multipathAuditError =
                                    ambiguousAppendError.empty()
                                    ? "multipath branch produced no finite "
                                      "base-frame audit points"
                                    : ambiguousAppendError;
                                break;
                            }
                            for (CloudPoint& point : branch.points) {
                                point.qualityFlags |=
                                    CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE;
                            }
                            ambiguousIntervals.insert(
                                input.intervalId);
                            result.qualityAmbiguousCandidatePointCount +=
                                branch.points.size();
                            result.qualityAmbiguousBranches.push_back(
                                std::move(branch));
                        }
                        if (multipathAuditFailed) {
                            result.reconstructed = false;
                            result.error =
                                "multipath rejected-audit reconstruction "
                                "failed: " +
                                multipathAuditError;
                            result.points.clear();
                            result.qualityPoints.clear();
                            result.shadowMaskedLegacyRejectedPoints.clear();
                            result.publicationGateRejectedPoints.clear();
                            result.qualityRejectedCandidatePoints.clear();
                            result.qualityAmbiguousBranches.clear();
                            result.pointCount = 0U;
                            result.qualityPointCount = 0U;
                            result.qualityAmbiguousCandidatePointCount = 0U;
                            result.qualityAmbiguousIntervalCount = 0U;
                        } else {
                            result.qualityAmbiguousIntervalCount =
                                ambiguousIntervals.size();
                        }
                    }
                }
            }
        } catch (const cv::Exception& exception) {
            result.reconstructed = false;
            result.points.clear();
            result.qualityPoints.clear();
            result.shadowMaskedLegacyRejectedPoints.clear();
            result.publicationGateRejectedPoints.clear();
            result.qualityRejectedCandidatePoints.clear();
            result.qualityAmbiguousBranches.clear();
            result.pointCount = 0U;
            result.qualityPointCount = 0U;
            result.error =
                std::string("OpenCV exception: ") + exception.what();
        } catch (const std::exception& exception) {
            result.reconstructed = false;
            result.points.clear();
            result.qualityPoints.clear();
            result.shadowMaskedLegacyRejectedPoints.clear();
            result.publicationGateRejectedPoints.clear();
            result.qualityRejectedCandidatePoints.clear();
            result.qualityAmbiguousBranches.clear();
            result.pointCount = 0U;
            result.qualityPointCount = 0U;
            result.error =
                std::string("reconstruction exception: ") + exception.what();
        } catch (...) {
            result.reconstructed = false;
            result.points.clear();
            result.qualityPoints.clear();
            result.shadowMaskedLegacyRejectedPoints.clear();
            result.publicationGateRejectedPoints.clear();
            result.qualityRejectedCandidatePoints.clear();
            result.qualityAmbiguousBranches.clear();
            result.pointCount = 0U;
            result.qualityPointCount = 0U;
            result.error = "unknown reconstruction exception";
        }

        result.reconstructionMs =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
        if (result.reconstructed) {
            reconstructedFrames.fetch_add(1U, std::memory_order_relaxed);
            if (!result.multipathAuditOnly &&
                !result.lineQualityPassed) {
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
        output << "frame_id,segment_id,aligned_timestamp_ns,status,point_count,"
                  "line_quality_passed,line_rms_mm,quality_attempted,"
                  "quality_analysis_completed,quality_passed,"
                  "multipath_audit_only,"
                  "quality_point_count,total_candidates,"
                  "accepted_candidates,path_usable_candidates,"
                  "provisional_selected_points,publishable_selected_points,"
                  "selected_points,selected_gaps,multipath_intervals,"
                  "multipath_ambiguous_scanlines,"
                  "multipath_candidate_points,"
                  "ambiguity_masked_legacy_points,"
                  "publication_gate_rejected_points,"
                  "optical_rejected_candidate_points,"
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
            output << result.frameId << ',' << result.segmentId << ','
                   << result.alignedTimestampNs << ','
                   << (result.reconstructed
                           ? (result.multipathAuditOnly
                                  ? "AUDIT_ONLY"
                                  : "RECONSTRUCTED")
                           : "FAILED")
                   << ','
                   << result.pointCount << ','
                   << (result.lineQualityPassed ? 1 : 0) << ','
                   << result.lineRmsMm << ','
                   << (result.qualityAttempted ? 1 : 0) << ','
                   << (result.qualityAnalysisCompleted ? 1 : 0) << ','
                   << (result.qualityExtractionPassed ? 1 : 0) << ','
                   << (result.multipathAuditOnly ? 1 : 0) << ','
                   << result.qualityPointCount << ','
                   << result.qualityDiagnostics.totalCandidateCount << ','
                   << result.qualityDiagnostics.acceptedCandidateCount << ','
                   << result.qualityDiagnostics.pathUsableCandidateCount
                   << ','
                   << result.qualityDiagnostics
                          .provisionalSelectedPointCount << ','
                   << result.qualityDiagnostics
                          .publishableSelectedPointCount << ','
                   << result.qualityDiagnostics.selectedPointCount << ','
                   << result.qualityDiagnostics.selectedGapCount << ','
                   << result.qualityAmbiguousIntervalCount << ','
                   << result.qualityDiagnostics
                          .multipathAmbiguousScanlineCount << ','
                   << result.qualityAmbiguousCandidatePointCount << ','
                   << result.ambiguityMaskedLegacyPointCount << ','
                   << result.publicationGateRejectedPoints.size() << ','
                   << result.qualityRejectedCandidatePoints.size() << ','
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

    bool writeSummary(
            const ContinuousReconstructionStatistics& statistics,
            const std::string& saveError,
            std::string* error) const {
        const std::string temporaryPath =
            statistics.summaryJsonPath + ".tmp";
        std::ofstream output(temporaryPath,
                             std::ios::out | std::ios::trunc);
        if (!output) {
            setError(
                "cannot create continuous reconstruction summary: " +
                    temporaryPath,
                error);
            return false;
        }
        output << std::setprecision(12)
               << "{\n"
               << "  \"schema_version\": 2,\n"
               << "  \"queue_capacity\": " << options.queueCapacity << ",\n"
               << "  \"worker_threads\": " << options.workerThreads << ",\n"
               << "  \"voxel_size_mm\": " << options.voxelSizeMm << ",\n"
               << "  \"camera_z_min_mm\": "
               << profileOptions.reconstruction.minimumDepthMm << ",\n"
               << "  \"camera_z_max_mm\": "
               << profileOptions.reconstruction.maximumDepthMm << ",\n"
               << "  \"voxel_color_map\": \"turbo\",\n"
               << "  \"voxel_color_scalar\": \"base_z_mm\",\n"
               << "  \"voxel_color_percentile\": [1, 99],\n"
               << "  \"ply_encoding\": \""
               << (options.binaryPly
                       ? "binary_little_endian"
                       : "ascii")
               << "\",\n"
               << "  \"save_quality_cloud\": "
               << (options.saveQualityCloud ? "true" : "false") << ",\n"
               << "  \"retain_quality_artifacts\": "
               << (options.retainQualityArtifacts ? "true" : "false")
               << ",\n"
               << "  \"v_groove_temporal_validation_enabled\": "
               << (options.enableVGrooveTemporalValidation
                       ? "true" : "false")
               << ",\n"
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
               << "  \"multipath_audit_only_frames\": "
               << statistics.multipathAuditOnlyFrames << ",\n"
               << "  \"quality_frames_passed\": "
               << statistics.qualityFramesPassed << ",\n"
               << "  \"quality_frames_rejected\": "
               << statistics.qualityFramesRejected << ",\n"
               << "  \"raw_point_count\": "
               << statistics.rawPointCount << ",\n"
               << "  \"voxel_point_count\": "
               << statistics.voxelPointCount << ",\n"
               << "  \"quality_multipath_interval_count\": "
               << statistics.qualityMultipathIntervalCount << ",\n"
               << "  \"quality_multipath_candidate_point_count\": "
               << statistics.qualityMultipathCandidatePointCount
               << ",\n"
               << "  \"quality_shadow_masked_legacy_rejected_point_count\": "
               << statistics
                      .qualityShadowMaskedLegacyRejectedPointCount
               << ",\n"
               << "  \"quality_profile_gate_rejected_point_count\": "
               << statistics.qualityProfileGateRejectedPointCount
               << ",\n"
               << "  \"quality_optical_rejected_candidate_point_count\": "
               << statistics.qualityOpticalRejectedCandidatePointCount
               << ",\n"
               << "  \"quality_v_groove_promoted_candidate_point_count\": "
               << statistics
                      .qualityVGroovePromotedCandidatePointCount
               << ",\n"
               << "  \"quality_v_groove_rejected_candidate_point_count\": "
               << statistics
                      .qualityVGrooveRejectedCandidatePointCount
               << ",\n"
               << "  \"quality_v_groove_ambiguous_group_count\": "
               << statistics.qualityVGrooveAmbiguousGroupCount
               << ",\n"
               << "  \"quality_v_groove_insufficient_group_count\": "
               << statistics.qualityVGrooveInsufficientGroupCount
               << ",\n"
               << "  \"quality_v_groove_invalid_geometry_group_count\": "
               << statistics
                      .qualityVGrooveInvalidGeometryGroupCount
               << ",\n"
               << "  \"quality_optical_point_count\": "
               << statistics.qualityOpticalPointCount << ",\n"
               << "  \"quality_filtered_point_count\": "
               << statistics.qualityFilteredPointCount << ",\n"
               << "  \"quality_adjacent_rejected_point_count\": "
               << statistics.qualityAdjacentRejectedPointCount
               << ",\n"
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
        output.flush();
        output.close();
        if (output.fail()) {
            setError(
                "failed while writing continuous reconstruction summary: " +
                    temporaryPath,
                error);
            return false;
        }
        std::error_code renameError;
        std::filesystem::rename(
            temporaryPath, statistics.summaryJsonPath, renameError);
        if (renameError) {
            setError(
                "cannot atomically publish continuous reconstruction "
                "summary: " +
                    renameError.message(),
                error);
            return false;
        }
        if (error) error->clear();
        return true;
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
        (options.enableVGrooveTemporalValidation &&
         !options.saveQualityCloud &&
         !options.retainQualityArtifacts) ||
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
        std::string* error,
        ContinuousReconstructionArtifacts* artifacts,
        const ContinuousReconstructionProgressCallback& progress) {
    const auto reportProgress =
        [&progress](int percent, const char* stage) {
            if (!progress) return;
            try {
                progress(percent, stage ? stage : "");
            } catch (...) {
                // Progress reporting is diagnostic only and must never make
                // point-cloud finalization fail.
            }
        };
    reportProgress(1, "正在停止重建输入");
    if (artifacts) *artifacts = ContinuousReconstructionArtifacts{};
    if (!impl_) {
        if (statistics) *statistics = lastStatistics_;
        setError("continuous reconstruction is not running", error);
        return false;
    }
    Impl* implementation = impl_.get();
    const auto savePly =
        [implementation](const std::string& path,
                         const std::vector<CloudPoint>& cloud,
                         const std::string& frameId,
                         std::string* saveError) {
            return implementation->options.binaryPly
                ? saveScanPlyBinary(path, cloud, frameId, saveError)
                : saveScanPly(path, cloud, frameId, saveError);
        };
    implementation->accepting.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(implementation->queueMutex);
        implementation->stopping = true;
    }
    implementation->queueCondition.notify_all();
    reportProgress(5, "正在清空重建队列");
    for (std::thread& worker : implementation->workers) {
        if (worker.joinable()) worker.join();
    }
    reportProgress(20, "重建队列已清空，正在合并帧结果");

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
    std::vector<CloudPoint> shadowMaskedLegacyRejectedCloud;
    std::vector<CloudPoint> publicationGateRejectedCloud;
    std::vector<CloudPoint> opticalRejectedCandidateCloud;
    std::vector<VGrooveCandidateBranch> ambiguousBranches;
    std::size_t rawPointCount = 0U;
    std::size_t qualityOpticalPointCount = 0U;
    std::size_t shadowMaskedLegacyRejectedPointCount = 0U;
    std::size_t publicationGateRejectedPointCount = 0U;
    std::size_t opticalRejectedCandidatePointCount = 0U;
    std::size_t ambiguousBranchCount = 0U;
    long double durationSumMs = 0.0L;
    for (const Impl::FrameResult& result : sorted) {
        if (result.reconstructed &&
            result.multipathAuditOnly) {
            ++finalStatistics.multipathAuditOnlyFrames;
        }
        rawPointCount += result.points.size();
        qualityOpticalPointCount += result.qualityPoints.size();
        shadowMaskedLegacyRejectedPointCount +=
            result.shadowMaskedLegacyRejectedPoints.size();
        publicationGateRejectedPointCount +=
            result.publicationGateRejectedPoints.size();
        opticalRejectedCandidatePointCount +=
            result.qualityRejectedCandidatePoints.size();
        ambiguousBranchCount +=
            result.qualityAmbiguousBranches.size();
        finalStatistics.qualityMultipathIntervalCount +=
            result.qualityAmbiguousIntervalCount;
        finalStatistics.qualityMultipathCandidatePointCount +=
            result.qualityAmbiguousCandidatePointCount;
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
    shadowMaskedLegacyRejectedCloud.reserve(
        shadowMaskedLegacyRejectedPointCount);
    publicationGateRejectedCloud.reserve(
        publicationGateRejectedPointCount);
    opticalRejectedCandidateCloud.reserve(
        opticalRejectedCandidatePointCount);
    ambiguousBranches.reserve(ambiguousBranchCount);
    for (Impl::FrameResult& result : sorted) {
        rawCloud.insert(rawCloud.end(),
                        std::make_move_iterator(result.points.begin()),
                        std::make_move_iterator(result.points.end()));
        qualityOpticalCloud.insert(
            qualityOpticalCloud.end(),
            std::make_move_iterator(result.qualityPoints.begin()),
            std::make_move_iterator(result.qualityPoints.end()));
        shadowMaskedLegacyRejectedCloud.insert(
            shadowMaskedLegacyRejectedCloud.end(),
            std::make_move_iterator(
                result.shadowMaskedLegacyRejectedPoints.begin()),
            std::make_move_iterator(
                result.shadowMaskedLegacyRejectedPoints.end()));
        publicationGateRejectedCloud.insert(
            publicationGateRejectedCloud.end(),
            std::make_move_iterator(
                result.publicationGateRejectedPoints.begin()),
            std::make_move_iterator(
                result.publicationGateRejectedPoints.end()));
        opticalRejectedCandidateCloud.insert(
            opticalRejectedCandidateCloud.end(),
            std::make_move_iterator(
                result.qualityRejectedCandidatePoints.begin()),
            std::make_move_iterator(
                result.qualityRejectedCandidatePoints.end()));
        ambiguousBranches.insert(
            ambiguousBranches.end(),
            std::make_move_iterator(
                result.qualityAmbiguousBranches.begin()),
            std::make_move_iterator(
                result.qualityAmbiguousBranches.end()));
    }

    std::string combinedError;
    std::vector<CloudPoint> vGrooveRejectedCloud;
    if (implementation->options.enableVGrooveTemporalValidation &&
        !ambiguousBranches.empty()) {
        reportProgress(30, "正在执行V槽时序质量验证");
        VGrooveTemporalValidationOptions vGrooveOptions;
        VGrooveTemporalValidationResult vGroove;
        std::string validationError;
        if (!validateVGrooveTemporalGeometry(
                qualityOpticalCloud, ambiguousBranches,
                vGrooveOptions, &vGroove, &validationError)) {
            combinedError =
                "V-groove temporal validation failed: " +
                validationError;
            for (const VGrooveCandidateBranch& branch :
                 ambiguousBranches) {
                for (CloudPoint point : branch.points) {
                    point.qualityFlags |=
                        CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT;
                    vGrooveRejectedCloud.push_back(point);
                }
            }
        } else {
            qualityOpticalCloud =
                std::move(vGroove.passThroughPublishable);
            qualityOpticalCloud.insert(
                qualityOpticalCloud.end(),
                std::make_move_iterator(
                    vGroove.promotedCandidates.begin()),
                std::make_move_iterator(
                    vGroove.promotedCandidates.end()));
            vGrooveRejectedCloud =
                std::move(vGroove.rejectedCandidates);
            finalStatistics
                .qualityVGroovePromotedCandidatePointCount =
                vGroove.statistics.promotedCandidatePointCount;
            finalStatistics
                .qualityVGrooveRejectedCandidatePointCount =
                vGroove.statistics.rejectedCandidatePointCount;
            for (const VGrooveAmbiguityGroupValidation& group :
                 vGroove.ambiguityGroups) {
                switch (group.status) {
                case VGrooveProfileStatus::PromotedUnique:
                case VGrooveProfileStatus::NotEvaluated:
                    break;
                case VGrooveProfileStatus::RejectedAmbiguous:
                    ++finalStatistics
                          .qualityVGrooveAmbiguousGroupCount;
                    break;
                case VGrooveProfileStatus::
                        RejectedInsufficientEvidence:
                    ++finalStatistics
                          .qualityVGrooveInsufficientGroupCount;
                    break;
                case VGrooveProfileStatus::RejectedInvalidGeometry:
                    ++finalStatistics
                          .qualityVGrooveInvalidGeometryGroupCount;
                    break;
                }
            }
        }
    }
    reportProgress(62, "正在写入逐帧重建诊断");
    finalStatistics.qualityVGrooveRejectedCandidatePointCount =
        vGrooveRejectedCloud.size();
    finalStatistics.rawPointCount = rawCloud.size();
    finalStatistics.qualityOpticalPointCount =
        qualityOpticalCloud.size();

    std::string detailError;
    if (!implementation->writeDetailCsv(
            sorted, finalStatistics.detailCsvPath, &detailError)) {
        if (!combinedError.empty()) combinedError += "; ";
        combinedError += detailError;
    }

    std::string saveError;
    reportProgress(68, "正在保存 continuous_raw.ply");
    finalStatistics.rawPlySaved = savePly(
        finalStatistics.rawPlyPath, rawCloud, "base_link", &saveError);
    if (!finalStatistics.rawPlySaved) {
        if (!combinedError.empty()) combinedError += "; ";
        combinedError += saveError;
    }
    reportProgress(77, "正在计算 continuous_voxel.ply");
    const std::vector<CloudPoint> voxelCloud =
        voxelDownsample(rawCloud, implementation->options.voxelSizeMm);
    finalStatistics.voxelPointCount = voxelCloud.size();
    saveError.clear();
    reportProgress(85, "正在保存世界 Z 高度着色 continuous_voxel.ply");
    PlyColorOptions voxelColorOptions;
    voxelColorOptions.scalar = PlyColorScalar::BaseZ;
    finalStatistics.voxelPlySaved =
        implementation->options.binaryPly
            ? saveScanPlyBinary(
                  finalStatistics.voxelPlyPath, voxelCloud, "base_link",
                  voxelColorOptions, &saveError)
            : saveScanPly(
                  finalStatistics.voxelPlyPath, voxelCloud, "base_link",
                  voxelColorOptions, &saveError);
    if (!finalStatistics.voxelPlySaved) {
        if (!combinedError.empty()) combinedError += "; ";
        combinedError += saveError;
    }

    std::vector<CloudPoint> qualityRejectedCloud;
    std::vector<CloudPoint> finalQualityAcceptedCloud;
    const bool qualityArtifactsEnabled =
        implementation->options.saveQualityCloud ||
        implementation->options.retainQualityArtifacts;
    if (qualityArtifactsEnabled) {
        qualityRejectedCloud = std::move(vGrooveRejectedCloud);
        finalStatistics
            .qualityShadowMaskedLegacyRejectedPointCount =
            shadowMaskedLegacyRejectedCloud.size();
        finalStatistics.qualityProfileGateRejectedPointCount =
            publicationGateRejectedCloud.size();
        finalStatistics.qualityOpticalRejectedCandidatePointCount =
            opticalRejectedCandidateCloud.size();
        qualityRejectedCloud.insert(
            qualityRejectedCloud.end(),
            std::make_move_iterator(
                shadowMaskedLegacyRejectedCloud.begin()),
            std::make_move_iterator(
                shadowMaskedLegacyRejectedCloud.end()));
        qualityRejectedCloud.insert(
            qualityRejectedCloud.end(),
            std::make_move_iterator(
                publicationGateRejectedCloud.begin()),
            std::make_move_iterator(
                publicationGateRejectedCloud.end()));
        qualityRejectedCloud.insert(
            qualityRejectedCloud.end(),
            std::make_move_iterator(
                opticalRejectedCandidateCloud.begin()),
            std::make_move_iterator(
                opticalRejectedCandidateCloud.end()));
    }
    if (qualityArtifactsEnabled &&
        !qualityOpticalCloud.empty()) {
        if (implementation->options.saveQualityCloud) {
            std::string qualitySaveError;
            finalStatistics.qualityOpticalPlySaved = savePly(
                finalStatistics.qualityOpticalPlyPath,
                qualityOpticalCloud, "base_link", &qualitySaveError);
            if (!finalStatistics.qualityOpticalPlySaved) {
                if (!combinedError.empty()) combinedError += "; ";
                combinedError += qualitySaveError;
            }
        }

        AdjacentProfileSupportOptions supportOptions;
        reportProgress(91, "正在执行相邻轮廓质量过滤");
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
            finalQualityAcceptedCloud = support.kept;
            finalStatistics.qualityFilteredPointCount =
                support.kept.size();
            finalStatistics.qualityAdjacentRejectedPointCount =
                support.rejected.size();
            qualityRejectedCloud.insert(
                qualityRejectedCloud.end(),
                std::make_move_iterator(
                    support.rejected.begin()),
                std::make_move_iterator(
                    support.rejected.end()));
            if (implementation->options.saveQualityCloud &&
                !support.kept.empty()) {
                std::string qualitySaveError;
                finalStatistics.qualityPlySaved = savePly(
                    finalStatistics.qualityPlyPath,
                    support.kept, "base_link", &qualitySaveError);
                if (!finalStatistics.qualityPlySaved) {
                    if (!combinedError.empty()) combinedError += "; ";
                    combinedError += qualitySaveError;
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
                    qualitySaveError.clear();
                    finalStatistics.qualityVoxelPlySaved = savePly(
                        finalStatistics.qualityVoxelPlyPath,
                        qualityVoxelCloud, "base_link",
                        &qualitySaveError);
                    if (!finalStatistics.qualityVoxelPlySaved) {
                        if (!combinedError.empty()) combinedError += "; ";
                        combinedError += qualitySaveError;
                    }
                }
            }
        }
    }
    finalStatistics.qualityRejectedPointCount =
        qualityRejectedCloud.size();
    if (implementation->options.saveQualityCloud &&
        !qualityRejectedCloud.empty()) {
        std::string saveError;
        finalStatistics.qualityRejectedPlySaved = savePly(
            finalStatistics.qualityRejectedPlyPath,
            qualityRejectedCloud, "base_link", &saveError);
        if (!finalStatistics.qualityRejectedPlySaved) {
            if (!combinedError.empty()) combinedError += "; ";
            combinedError += saveError;
        }
    }
    std::string summaryError;
    reportProgress(97, "正在写入重建摘要");
    if (!implementation->writeSummary(
            finalStatistics, combinedError, &summaryError)) {
        if (!combinedError.empty()) combinedError += "; ";
        combinedError += summaryError;
    }

    if (artifacts) {
        artifacts->formal = std::move(rawCloud);
        artifacts->qualityAccepted =
            std::move(finalQualityAcceptedCloud);
        artifacts->rejected =
            std::move(qualityRejectedCloud);
        artifacts->viewpoints.reserve(sorted.size());
        for (const Impl::FrameResult& frame : sorted) {
            if (!frame.reconstructed ||
                !finitePoint(frame.cameraOriginBaseMm)) {
                continue;
            }
            ContinuousFrameViewpoint viewpoint;
            viewpoint.frameId = frame.frameId;
            viewpoint.profileIndex =
                frame.frameId <= static_cast<std::uint64_t>(
                    std::numeric_limits<int>::max())
                ? static_cast<int>(frame.frameId)
                : std::numeric_limits<int>::max();
            viewpoint.segmentId = frame.segmentId;
            viewpoint.cameraOriginBaseMm =
                frame.cameraOriginBaseMm;
            artifacts->viewpoints.push_back(viewpoint);
        }
    }

    lastStatistics_ = finalStatistics;
    impl_.reset();
    if (statistics) *statistics = finalStatistics;
    if (error) *error = combinedError;
    reportProgress(100, "连续点云计算与保存完成");
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
