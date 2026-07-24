#include "HikContinuousReconstruction.h"

#include <opencv2/core.hpp>

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
    CHECK_TRUE(pipeline.stopAndSave(&statistics, &error),
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
    CHECK_TRUE(statistics.qualityOpticalPointCount == 200U &&
                   statistics.qualityFilteredPointCount == 200U &&
                   statistics.qualityRejectedPointCount == 0U,
               "quality cloud must retain all adjacent supported points");
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
                   output / "continuous_reconstruction_summary.json"),
               "continuous reconstruction must produce all output artifacts");
    const std::string rawPly = readText(output / "continuous_raw.ply");
    CHECK_TRUE(rawPly.find("element vertex 200") != std::string::npos &&
               rawPly.find("comment frame_id base_link") != std::string::npos,
               "continuous raw PLY must contain base_link points");
    const std::string qualityOpticalPly =
        readText(output / "continuous_quality_optical.ply");
    CHECK_TRUE(
        qualityOpticalPly.find("element vertex 200") !=
            std::string::npos &&
        qualityOpticalPly.find(" 16 1\n") != std::string::npos,
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
    CHECK_TRUE(summary.find("\"reconstructed_frames\": 2") !=
                   std::string::npos &&
               summary.find("\"intrinsics_sha256\": \"intrinsics-test\"") !=
                   std::string::npos &&
               summary.find("\"quality_frames_passed\": 2") !=
                   std::string::npos &&
               summary.find("\"quality_filtered_point_count\": 200") !=
                   std::string::npos,
               "continuous summary must retain counts and calibration identity");

    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

}  // namespace

int main() {
    testContinuousReconstructionPipeline();
    if (failures != 0) {
        std::cerr << failures << " continuous reconstruction test(s) failed\n";
        return 1;
    }
    std::cout << "All continuous reconstruction tests passed\n";
    return 0;
}
