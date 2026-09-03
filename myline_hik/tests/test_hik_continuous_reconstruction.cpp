#include "HikContinuousReconstruction.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {

int failures = 0;

#define CHECK_TRUE(condition, message)                                           \
    do {                                                                          \
        if (!(condition)) {                                                        \
            std::cerr << "FAIL: " << (message) << " (line " << __LINE__ << ")\n"; \
            ++failures;                                                            \
        }                                                                          \
    } while (false)

std::string readText(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::ostringstream output;
    output << input.rdbuf();
    return output.str();
}

std::size_t csvColumnCount(const std::string& line) {
    if (line.empty()) return 0U;
    std::size_t columns = 1U;
    bool quoted = false;
    for (std::size_t index = 0U; index < line.size(); ++index) {
        if (line[index] == '"') {
            if (quoted && index + 1U < line.size() &&
                line[index + 1U] == '"') {
                ++index;
            } else {
                quoted = !quoted;
            }
        } else if (line[index] == ',' && !quoted) {
            ++columns;
        }
    }
    return columns;
}

bool near(double first, double second, double tolerance) {
    return std::abs(first - second) <= tolerance;
}

std::shared_ptr<hik_sync::ImageBuffer> syntheticLaserImage() {
    const cv::Size imageSize(120, 100);
    cv::Mat image(imageSize, CV_8UC1);
    for (int row = 0; row < image.rows; ++row) {
        for (int column = 0; column < image.cols; ++column) {
            image.at<unsigned char>(row, column) =
                static_cast<unsigned char>(30 + column / 20 + row / 25);
        }
        const int values[5] = {75, 155, 245, 155, 75};
        for (int offset = -2; offset <= 2; ++offset) {
            image.at<unsigned char>(row, 70 + offset) = values[offset + 2];
        }
    }

    auto buffer = std::make_shared<hik_sync::ImageBuffer>();
    buffer->width = image.cols;
    buffer->height = image.rows;
    buffer->stride = static_cast<int>(image.step);
    buffer->bytes.assign(image.datastart, image.dataend);
    return buffer;
}

std::shared_ptr<hik_sync::ImageBuffer> syntheticMultipathLaserImage() {
    const cv::Size imageSize(120, 100);
    cv::Mat image(imageSize, CV_8UC1, cv::Scalar(30));
    const auto addHorizontalGaussian =
        [&image](double centerY,
                 double amplitude,
                 int firstColumn,
                 int lastColumn) {
            for (int column = std::max(0, firstColumn);
                 column < std::min(image.cols, lastColumn);
                 ++column) {
                for (int row = 0; row < image.rows; ++row) {
                    const double distance =
                        (static_cast<double>(row) - centerY) / 1.10;
                    const int signal = static_cast<int>(std::lround(
                        amplitude *
                        std::exp(-0.5 * distance * distance)));
                    image.at<unsigned char>(row, column) =
                        static_cast<unsigned char>(std::min(
                            255,
                            static_cast<int>(
                                image.at<unsigned char>(
                                    row, column)) +
                                signal));
                }
            }
        };
    addHorizontalGaussian(45.0, 100.0, 0, image.cols);
    const int forkFirst = 50;
    const int forkLast = 69;
    for (int column = forkFirst; column <= forkLast; ++column) {
        const int offset = column - forkFirst;
        const int distanceToEnd =
            std::min(offset, forkLast - column);
        addHorizontalGaussian(
            45.0 + 4.0 * static_cast<double>(distanceToEnd),
            100.0, column, column + 1);
    }

    auto buffer = std::make_shared<hik_sync::ImageBuffer>();
    buffer->width = image.cols;
    buffer->height = image.rows;
    buffer->stride = static_cast<int>(image.step);
    buffer->bytes.assign(image.datastart, image.dataend);
    return buffer;
}

bool enqueueEventually(hik_scan::ContinuousReconstructionPipeline* pipeline,
                       const hik_sync::SynchronizedFrame& frame) {
    if (!pipeline) return false;
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (pipeline->tryEnqueue(frame)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

void testContinuousReconstructionPipeline() {
    hik_calibration::IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.imageSize = cv::Size(120, 100);
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        100.0, 0.0, 60.0,
        0.0, 100.0, 50.0,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    hik_calibration::LaserPlaneFitResult laserPlane;
    laserPlane.ok = true;
    laserPlane.plane.normal = cv::Vec3d(0.0, 0.0, 1.0);
    laserPlane.plane.dMm = -500.0;

    hik_calibration::SingleFrameProfileOptions profileOptions;
    profileOptions.reconstruction.minimumDepthMm = 450.0;
    profileOptions.reconstruction.maximumDepthMm = 550.0;
    profileOptions.reconstruction.stripe.minPointCount = 80;
    profileOptions.reconstruction.minReconstructedPoints = 80;
    profileOptions.reconstruction.stripe.mode =
        hik_calibration::StripeExtractionMode::Shadow;
    profileOptions.reconstruction.stripe.quality.orientation =
        hik_stripe::Orientation::Vertical;
    profileOptions.minimumRawIntensity = 60;

    const std::filesystem::path output =
        std::filesystem::temp_directory_path() /
        ("hik_continuous_reconstruction_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));
    hik_scan::ContinuousReconstructionOptions options;
    options.queueCapacity = 8U;
    options.workerThreads = 2U;
    options.voxelSizeMm = 0.5;
    options.outputDirectory = output.string();
    options.intrinsicsSha256 = "intrinsics-test";
    options.laserPlaneSha256 = "laser-test";
    options.handEyeSha256 = "handeye-test";
    options.saveQualityCloud = true;
    options.enableAdjacentProfileSupport = true;
    options.adjacentSupportRadiusMm = 1.0;
    options.adjacentMinimumSupportingProfiles = 1;
    options.adjacentMaximumProfileGap = 1;

    hik_scan::ContinuousReconstructionPipeline pipeline;
    std::string error;
    CHECK_TRUE(pipeline.start(
                   options, intrinsics, laserPlane, profileOptions, &error),
               std::string("pipeline start failed: ") + error);

    const std::shared_ptr<hik_sync::ImageBuffer> image =
        syntheticLaserImage();
    for (uint64_t frameId = 1U; frameId <= 2U; ++frameId) {
        hik_sync::SynchronizedFrame frame;
        frame.frameId = frameId;
        frame.alignedTimestampNs =
            1000000000LL + static_cast<int64_t>(frameId) * 16666667LL;
        frame.quality = hik_sync::SyncQuality::VALID;
        frame.measurementSegmentId = 11;
        frame.measurementActive = true;
        frame.image = image;
        frame.hasBaseFromCamera = true;
        frame.baseFromCamera = Eigen::Matrix4d::Identity();
        frame.baseFromCamera(0, 3) =
            100.0 + 0.2 * static_cast<double>(frameId - 1U);
        CHECK_TRUE(enqueueEventually(&pipeline, frame),
                   "valid synchronized frame must enter reconstruction queue");
    }

    hik_sync::SynchronizedFrame invalid;
    invalid.frameId = 3U;
    invalid.quality = hik_sync::SyncQuality::SPEED_NOT_STABLE;
    invalid.image = image;
    invalid.hasBaseFromCamera = true;
    CHECK_TRUE(!pipeline.tryEnqueue(invalid),
               "invalid synchronization quality must be skipped");

    hik_scan::ContinuousReconstructionStatistics statistics;
    hik_scan::ContinuousReconstructionArtifacts artifacts;
    CHECK_TRUE(pipeline.stopAndSave(
                   &statistics, &error, &artifacts),
               std::string("pipeline stop/save failed: ") + error);
    CHECK_TRUE(statistics.reconstructedFrames == 2U &&
               statistics.reconstructionFailures == 0U,
               "both valid frames must reconstruct");
    CHECK_TRUE(statistics.invalidSyncFramesSkipped == 1U,
               "invalid synchronization frame must be counted");
    CHECK_TRUE(statistics.rawPointCount == 200U,
               "two synthetic laser profiles must produce 200 raw points");
    CHECK_TRUE(statistics.rawPlySaved && statistics.voxelPlySaved,
               "raw and voxel PLY files must be saved");
    CHECK_TRUE(statistics.qualityFramesPassed == 2U &&
                   statistics.qualityFramesRejected == 0U,
               "shadow quality extraction must pass both synthetic frames");
    CHECK_TRUE(statistics.multipathAuditOnlyFrames == 0U,
               "clean profiles must never be downgraded to audit-only");
    CHECK_TRUE(statistics.qualityOpticalPointCount == 200U &&
                   statistics.qualityFilteredPointCount == 200U &&
                   statistics.qualityRejectedPointCount == 0U,
               "quality cloud must retain all adjacent supported points");
    CHECK_TRUE(
        artifacts.formal.size() == 200U &&
        artifacts.qualityAccepted.size() == 200U &&
        artifacts.rejected.empty(),
        "in-memory handoff keeps formal and quality evidence separate "
        "without double-counting either cloud");
    CHECK_TRUE(
        artifacts.viewpoints.size() == 2U &&
        artifacts.viewpoints[0].segmentId == 11 &&
        artifacts.viewpoints[1].segmentId == 11 &&
        near(artifacts.viewpoints[0].cameraOriginBaseMm.x,
             100.0, 1e-12) &&
        near(artifacts.viewpoints[1].cameraOriginBaseMm.x,
             100.2, 1e-12),
        "adaptive handoff retains per-frame base_link camera origins and "
        "measurement segment ids");
    CHECK_TRUE(
        !artifacts.qualityAccepted.empty() &&
        artifacts.qualityAccepted.front().opticalMetricsValid &&
        std::isfinite(artifacts.qualityAccepted.front().snr) &&
        std::isfinite(artifacts.qualityAccepted.front().fwhmPx),
        "quality handoff preserves point-local SNR/FWHM evidence");
    CHECK_TRUE(statistics.qualityOpticalPlySaved &&
                   statistics.qualityPlySaved &&
                   statistics.qualityVoxelPlySaved,
               "quality before/after and weighted voxel PLYs must be saved");
    CHECK_TRUE(std::filesystem::exists(output / "continuous_raw.ply") &&
               std::filesystem::exists(output / "continuous_voxel.ply") &&
               std::filesystem::exists(
                   output / "continuous_quality_optical.ply") &&
               std::filesystem::exists(
                   output / "continuous_quality_filtered.ply") &&
               std::filesystem::exists(
                   output / "continuous_quality_voxel.ply") &&
               std::filesystem::exists(
                   output / "continuous_reconstruction.csv") &&
               std::filesystem::exists(
                   output / "continuous_reconstruction_summary.json") &&
               !std::filesystem::exists(
                   output /
                   "continuous_reconstruction_summary.json.tmp"),
               "continuous reconstruction must produce all output artifacts");
    const std::string rawPly = readText(output / "continuous_raw.ply");
    const std::string voxelPly =
        readText(output / "continuous_voxel.ply");
    CHECK_TRUE(rawPly.find("element vertex 200") != std::string::npos &&
               rawPly.find("comment frame_id base_link") != std::string::npos,
               "continuous raw PLY must contain base_link points");
    CHECK_TRUE(
        voxelPly.find("comment color_map turbo") != std::string::npos &&
            voxelPly.find("comment color_scalar base_z_mm") !=
                std::string::npos,
        "continuous voxel PLY must carry automatic base-Z height colors");
    const std::string qualityOpticalPly =
        readText(output / "continuous_quality_optical.ply");
    CHECK_TRUE(
        qualityOpticalPly.find("element vertex 200") !=
            std::string::npos &&
        qualityOpticalPly.find(
            "property uchar optical_metrics_valid") !=
            std::string::npos &&
        qualityOpticalPly.find(" 16 1 1 ") != std::string::npos,
        "quality optical PLY must mark hard-gated observations without "
        "mutating the formal raw cloud");
    const std::string detail =
        readText(output / "continuous_reconstruction.csv");
    CHECK_TRUE(
        detail.find("mean_selected_snr") != std::string::npos &&
        detail.find("mean_selected_gradient_asymmetry") !=
            std::string::npos &&
        detail.find("offset_abs_p95_px") != std::string::npos,
        "per-frame shadow CSV must retain optical quality and center-offset "
        "diagnostics");
    std::istringstream detailLines(detail);
    std::string detailHeader;
    std::string detailFirstRow;
    std::getline(detailLines, detailHeader);
    std::getline(detailLines, detailFirstRow);
    CHECK_TRUE(
        csvColumnCount(detailHeader) ==
            csvColumnCount(detailFirstRow),
        "continuous reconstruction CSV header and data rows must remain "
        "column-aligned as diagnostics evolve");
    const std::string summary =
        readText(output / "continuous_reconstruction_summary.json");
    CHECK_TRUE(summary.find("\"schema_version\": 2") !=
                   std::string::npos &&
               summary.find("\"reconstructed_frames\": 2") !=
                   std::string::npos &&
               summary.find("\"intrinsics_sha256\": \"intrinsics-test\"") !=
                   std::string::npos &&
               summary.find("\"quality_frames_passed\": 2") !=
                   std::string::npos &&
               summary.find(
                   "\"voxel_color_scalar\": \"base_z_mm\"") !=
                   std::string::npos &&
               summary.find("\"camera_z_min_mm\": 450") !=
                   std::string::npos &&
               summary.find("\"camera_z_max_mm\": 550") !=
                   std::string::npos &&
               summary.find("\"quality_filtered_point_count\": 200") !=
                   std::string::npos,
               "continuous summary must retain counts and calibration identity");

    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

void testRawVoxelOnlyMode() {
    hik_calibration::IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.imageSize = cv::Size(120, 100);
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        100.0, 0.0, 60.0,
        0.0, 100.0, 50.0,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    hik_calibration::LaserPlaneFitResult laserPlane;
    laserPlane.ok = true;
    laserPlane.plane.normal = cv::Vec3d(0.0, 0.0, 1.0);
    laserPlane.plane.dMm = -500.0;

    hik_calibration::SingleFrameProfileOptions profileOptions;
    profileOptions.reconstruction.minimumDepthMm = 450.0;
    profileOptions.reconstruction.maximumDepthMm = 550.0;
    profileOptions.reconstruction.stripe.minPointCount = 80;
    profileOptions.reconstruction.minReconstructedPoints = 80;
    profileOptions.reconstruction.stripe.mode =
        hik_calibration::StripeExtractionMode::Quality;
    profileOptions.reconstruction.stripe.quality.orientation =
        hik_stripe::Orientation::Vertical;
    profileOptions.minimumRawIntensity = 60;

    const std::filesystem::path output =
        std::filesystem::temp_directory_path() /
        ("hik_continuous_raw_voxel_only_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));
    hik_scan::ContinuousReconstructionOptions options;
    options.queueCapacity = 8U;
    options.workerThreads = 2U;
    options.voxelSizeMm = 0.5;
    options.binaryPly = true;
    options.outputDirectory = output.string();
    options.saveQualityCloud = false;
    options.enableAdjacentProfileSupport = false;

    hik_scan::ContinuousReconstructionPipeline pipeline;
    std::string error;
    CHECK_TRUE(pipeline.start(
                   options, intrinsics, laserPlane, profileOptions, &error),
               std::string("raw/voxel-only pipeline start failed: ") +
                   error);
    const std::shared_ptr<hik_sync::ImageBuffer> image =
        syntheticLaserImage();
    for (uint64_t frameId = 1U; frameId <= 2U; ++frameId) {
        hik_sync::SynchronizedFrame frame;
        frame.frameId = frameId;
        frame.alignedTimestampNs =
            3000000000LL +
            static_cast<int64_t>(frameId) * 16666667LL;
        frame.quality = hik_sync::SyncQuality::VALID;
        frame.image = image;
        frame.hasBaseFromCamera = true;
        frame.baseFromCamera = Eigen::Matrix4d::Identity();
        frame.baseFromCamera(0, 3) =
            0.2 * static_cast<double>(frameId - 1U);
        CHECK_TRUE(enqueueEventually(&pipeline, frame),
                   "raw/voxel-only frame must enter reconstruction queue");
    }

    hik_scan::ContinuousReconstructionStatistics statistics;
    std::vector<int> progressPercentages;
    CHECK_TRUE(pipeline.stopAndSave(
                   &statistics, &error, nullptr,
                   [&progressPercentages](
                           int percent, const std::string&) {
                       progressPercentages.push_back(percent);
                   }),
               std::string("raw/voxel-only stop/save failed: ") + error);
    CHECK_TRUE(statistics.rawPlySaved && statistics.voxelPlySaved &&
                   !statistics.qualityOpticalPlySaved &&
                   !statistics.qualityPlySaved &&
                   !statistics.qualityRejectedPlySaved &&
                   !statistics.qualityVoxelPlySaved,
               "raw/voxel-only mode must not save auxiliary quality PLYs");
    CHECK_TRUE(
        std::filesystem::exists(output / "continuous_raw.ply") &&
            std::filesystem::exists(output / "continuous_voxel.ply") &&
            !std::filesystem::exists(
                output / "continuous_quality_optical.ply") &&
            !std::filesystem::exists(
                output / "continuous_quality_filtered.ply") &&
            !std::filesystem::exists(
                output / "continuous_quality_rejected.ply") &&
            !std::filesystem::exists(
                output / "continuous_quality_voxel.ply"),
        "raw/voxel-only output directory must contain only two PLY files");
    std::size_t plyFileCount = 0U;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(output)) {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".ply") {
            ++plyFileCount;
        }
    }
    CHECK_TRUE(plyFileCount == 2U,
               "raw/voxel-only mode produced an unexpected PLY file");
    const std::string summary =
        readText(output / "continuous_reconstruction_summary.json");
    const std::string rawPly =
        readText(output / "continuous_raw.ply");
    const std::string voxelPly =
        readText(output / "continuous_voxel.ply");
    CHECK_TRUE(
        summary.find("\"save_quality_cloud\": false") !=
                std::string::npos &&
            summary.find(
                "\"v_groove_temporal_validation_enabled\": false") !=
                std::string::npos &&
            summary.find(
                "\"ply_encoding\": \"binary_little_endian\"") !=
                std::string::npos &&
            rawPly.find("format binary_little_endian 1.0") !=
                std::string::npos &&
            voxelPly.find("comment color_scalar base_z_mm") !=
                std::string::npos,
        "scanner_450-style raw/voxel-only summary must record disabled "
        "V-groove validation and binary output");
    CHECK_TRUE(
        !progressPercentages.empty() &&
            progressPercentages.front() == 1 &&
            progressPercentages.back() == 100 &&
            std::is_sorted(progressPercentages.begin(),
                           progressPercentages.end()),
        "continuous finalization progress must start, advance monotonically "
        "and report completion");

    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

void testRetainedQualityArtifactsWithoutQualityPly() {
    hik_calibration::IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.imageSize = cv::Size(120, 100);
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        100.0, 0.0, 60.0,
        0.0, 100.0, 50.0,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    hik_calibration::LaserPlaneFitResult laserPlane;
    laserPlane.ok = true;
    laserPlane.plane.normal = cv::Vec3d(0.0, 0.0, 1.0);
    laserPlane.plane.dMm = -500.0;

    hik_calibration::SingleFrameProfileOptions profileOptions;
    profileOptions.reconstruction.minimumDepthMm = 450.0;
    profileOptions.reconstruction.maximumDepthMm = 550.0;
    profileOptions.reconstruction.stripe.minPointCount = 80;
    profileOptions.reconstruction.minReconstructedPoints = 80;
    profileOptions.reconstruction.stripe.mode =
        hik_calibration::StripeExtractionMode::Shadow;
    profileOptions.reconstruction.stripe.quality.orientation =
        hik_stripe::Orientation::Vertical;
    profileOptions.minimumRawIntensity = 60;

    const std::filesystem::path output =
        std::filesystem::temp_directory_path() /
        ("hik_continuous_retained_quality_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));
    hik_scan::ContinuousReconstructionOptions options;
    options.queueCapacity = 8U;
    options.workerThreads = 2U;
    options.voxelSizeMm = 0.5;
    options.outputDirectory = output.string();
    options.saveQualityCloud = false;
    options.retainQualityArtifacts = true;
    options.enableAdjacentProfileSupport = true;
    options.adjacentSupportRadiusMm = 1.0;
    options.adjacentMinimumSupportingProfiles = 1;
    options.adjacentMaximumProfileGap = 1;

    hik_scan::ContinuousReconstructionPipeline pipeline;
    std::string error;
    CHECK_TRUE(pipeline.start(
                   options, intrinsics, laserPlane, profileOptions, &error),
               std::string("retained-quality pipeline start failed: ") +
                   error);
    const std::shared_ptr<hik_sync::ImageBuffer> image =
        syntheticLaserImage();
    for (uint64_t frameId = 1U; frameId <= 2U; ++frameId) {
        hik_sync::SynchronizedFrame frame;
        frame.frameId = frameId;
        frame.alignedTimestampNs =
            4000000000LL +
            static_cast<int64_t>(frameId) * 16666667LL;
        frame.quality = hik_sync::SyncQuality::VALID;
        frame.measurementSegmentId = 41;
        frame.measurementActive = true;
        frame.image = image;
        frame.hasBaseFromCamera = true;
        frame.baseFromCamera = Eigen::Matrix4d::Identity();
        frame.baseFromCamera(0, 3) =
            0.2 * static_cast<double>(frameId - 1U);
        CHECK_TRUE(enqueueEventually(&pipeline, frame),
                   "retained-quality frame must enter reconstruction queue");
    }

    hik_scan::ContinuousReconstructionStatistics statistics;
    hik_scan::ContinuousReconstructionArtifacts artifacts;
    CHECK_TRUE(pipeline.stopAndSave(
                   &statistics, &error, &artifacts),
               std::string("retained-quality stop/save failed: ") + error);
    CHECK_TRUE(statistics.rawPlySaved && statistics.voxelPlySaved &&
                   statistics.qualityFilteredPointCount == 200U &&
                   !statistics.qualityOpticalPlySaved &&
                   !statistics.qualityPlySaved &&
                   !statistics.qualityRejectedPlySaved &&
                   !statistics.qualityVoxelPlySaved,
               "retained quality evidence must not write auxiliary PLYs");
    CHECK_TRUE(artifacts.formal.size() == 200U &&
                   artifacts.qualityAccepted.size() == 200U &&
                   artifacts.rejected.empty() &&
                   artifacts.viewpoints.size() == 2U,
               "adaptive in-memory quality evidence must survive disabled "
               "quality PLY output");
    std::size_t plyFileCount = 0U;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(output)) {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".ply") {
            ++plyFileCount;
        }
    }
    CHECK_TRUE(plyFileCount == 2U,
               "retained-quality mode produced an unexpected PLY file");
    const std::string summary =
        readText(output / "continuous_reconstruction_summary.json");
    CHECK_TRUE(
        summary.find("\"save_quality_cloud\": false") !=
                std::string::npos &&
            summary.find("\"retain_quality_artifacts\": true") !=
                std::string::npos,
        "retained-quality summary must distinguish memory from PLY output");

    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

void testContinuousMultipathIsRejectedFailClosed() {
    hik_calibration::IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.imageSize = cv::Size(120, 100);
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        100.0, 0.0, 60.0,
        0.0, 100.0, 50.0,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    hik_calibration::LaserPlaneFitResult laserPlane;
    laserPlane.ok = true;
    laserPlane.plane.normal = cv::Vec3d(0.0, 0.0, 1.0);
    laserPlane.plane.dMm = -500.0;

    hik_calibration::SingleFrameProfileOptions profileOptions;
    profileOptions.reconstruction.minimumDepthMm = 450.0;
    profileOptions.reconstruction.maximumDepthMm = 550.0;
    profileOptions.reconstruction.stripe.minPointCount = 115;
    profileOptions.reconstruction.minReconstructedPoints = 115;
    profileOptions.reconstruction.stripe.mode =
        hik_calibration::StripeExtractionMode::Shadow;
    profileOptions.reconstruction.stripe.quality.orientation =
        hik_stripe::Orientation::Horizontal;
    profileOptions.reconstruction.stripe.quality.pathMaximumStepPx =
        6.0;
    profileOptions.reconstruction.stripe.quality
        .pathAmbiguityMarginPerPoint = 1.50;
    profileOptions.reconstruction.stripe.quality
        .pathAmbiguityMinimumSeparationPx = 5.0;
    profileOptions.reconstruction.stripe.quality
        .pathAmbiguityPaddingScanlines = 2;
    profileOptions.minimumRawIntensity = 60;

    const std::filesystem::path output =
        std::filesystem::temp_directory_path() /
        ("hik_continuous_multipath_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));
    hik_scan::ContinuousReconstructionOptions options;
    options.queueCapacity = 8U;
    options.workerThreads = 2U;
    options.voxelSizeMm = 0.5;
    options.outputDirectory = output.string();
    options.intrinsicsSha256 = "intrinsics-multipath-test";
    options.laserPlaneSha256 = "laser-multipath-test";
    options.handEyeSha256 = "handeye-multipath-test";
    options.saveQualityCloud = true;
    options.enableVGrooveTemporalValidation = true;
    options.enableAdjacentProfileSupport = false;

    hik_scan::ContinuousReconstructionPipeline pipeline;
    std::string error;
    CHECK_TRUE(pipeline.start(
                   options, intrinsics, laserPlane,
                   profileOptions, &error),
               std::string("multipath pipeline start failed: ") +
                   error);
    const std::shared_ptr<hik_sync::ImageBuffer> image =
        syntheticMultipathLaserImage();
    for (uint64_t frameId = 1U; frameId <= 3U; ++frameId) {
        hik_sync::SynchronizedFrame frame;
        frame.frameId = frameId;
        frame.alignedTimestampNs =
            2000000000LL +
            static_cast<int64_t>(frameId) * 16666667LL;
        frame.quality = hik_sync::SyncQuality::VALID;
        frame.measurementSegmentId = 23;
        frame.measurementActive = true;
        frame.image = image;
        frame.hasBaseFromCamera = true;
        frame.baseFromCamera = Eigen::Matrix4d::Identity();
        frame.baseFromCamera(0, 3) =
            0.2 * static_cast<double>(frameId - 1U);
        CHECK_TRUE(enqueueEventually(&pipeline, frame),
                   "multipath frame must enter reconstruction queue");
    }

    hik_scan::ContinuousReconstructionStatistics statistics;
    hik_scan::ContinuousReconstructionArtifacts artifacts;
    CHECK_TRUE(pipeline.stopAndSave(
                   &statistics, &error, &artifacts),
               std::string("multipath pipeline stop/save failed: ") +
                   error);
    CHECK_TRUE(statistics.reconstructedFrames == 3U &&
                   statistics.reconstructionFailures == 0U,
               "all synthetic multipath frames must reconstruct before "
               "candidate validation");
    CHECK_TRUE(statistics.multipathAuditOnlyFrames == 3U &&
                   statistics.rawPointCount == 0U,
               "profiles below the post-mask publication gate must retain "
               "audit evidence while publishing no sparse formal points");
    CHECK_TRUE(statistics.qualityMultipathIntervalCount >= 3U &&
                   statistics.qualityMultipathCandidatePointCount > 0U,
               "2-D fork evidence must reach the continuous base_link "
               "validator with explicit branches");
    CHECK_TRUE(
        statistics.qualityVGroovePromotedCandidatePointCount == 0U &&
            statistics.qualityVGrooveRejectedCandidatePointCount > 0U,
        "a coplanar false fork must not be promoted as a physical V groove");
    const std::string rawPly =
        readText(output / "continuous_raw.ply");
    CHECK_TRUE(statistics.rawPlySaved &&
                   rawPly.find("element vertex 0") != std::string::npos &&
                   rawPly.find("comment frame_id base_link") !=
                       std::string::npos,
               "an audit-only scan must save an explicit empty formal PLY "
               "instead of leaking a partial multipath profile");
    CHECK_TRUE(
        statistics.qualityShadowMaskedLegacyRejectedPointCount > 0U,
        "legacy observations removed by the Shadow hard mask must be "
        "retained only in the rejected audit cloud");
    CHECK_TRUE(
        artifacts.formal.empty() &&
        artifacts.qualityAccepted.empty() &&
        !artifacts.rejected.empty() &&
        artifacts.viewpoints.size() == 3U &&
        artifacts.viewpoints.front().segmentId == 23,
        "audit-only multipath evidence reaches the adaptive rejected handoff "
        "but never formal or quality-accepted geometry");
    CHECK_TRUE(statistics.qualityProfileGateRejectedPointCount > 0U &&
                   statistics.qualityOpticalPointCount == 0U,
               "the otherwise selected fragment of an audit-only profile "
               "must move to rejected, never optical or formal output");
    CHECK_TRUE(statistics.qualityRejectedPointCount >=
                   statistics
                           .qualityVGrooveRejectedCandidatePointCount +
                       statistics
                           .qualityShadowMaskedLegacyRejectedPointCount +
                       statistics
                           .qualityProfileGateRejectedPointCount &&
                   statistics.qualityRejectedPlySaved,
               "all V-rejected candidates must be preserved in the rejected "
               "cloud even when adjacent support filtering is disabled");
    const std::string rejectedPly =
        readText(output / "continuous_quality_rejected.ply");
    CHECK_TRUE(
        rejectedPly.find("element vertex 0") == std::string::npos &&
            rejectedPly.find("comment frame_id base_link") !=
                std::string::npos,
        "rejected PLY must contain auditable base_link candidate points");
    const std::string summary =
        readText(output / "continuous_reconstruction_summary.json");
    CHECK_TRUE(
        summary.find("\"multipath_audit_only_frames\": 3") !=
                std::string::npos &&
            summary.find(
                "\"quality_profile_gate_rejected_point_count\": ") !=
                std::string::npos,
        "continuous summary must make audit-only frames and publication-gate "
        "rejections explicitly machine-readable");

    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

}  // namespace

int main() {
    testContinuousReconstructionPipeline();
    testRawVoxelOnlyMode();
    testRetainedQualityArtifactsWithoutQualityPly();
    testContinuousMultipathIsRejectedFailClosed();
    if (failures != 0) {
        std::cerr << failures << " continuous reconstruction test(s) failed\n";
        return 1;
    }
    std::cout << "All continuous reconstruction tests passed\n";
    return 0;
}
