#include "HikCalibrationCore.h"
#include "HandEyeCalibrationCore.h"
#include "HikScanCore.h"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace {

int gFailures = 0;

void fail(const std::string& message, int line) {
    ++gFailures;
    std::cerr << "FAIL line " << line << ": " << message << std::endl;
}

#define CHECK_TRUE(condition, message) \
    do {                                \
        if (!(condition)) {             \
            fail((message), __LINE__);  \
        }                               \
    } while (false)

bool nearlyEqual(double a, double b, double tolerance) {
    return std::isfinite(a) && std::isfinite(b) && std::fabs(a - b) <= tolerance;
}

std::string numberText(double value) {
    std::ostringstream out;
    out.precision(17);
    out << value;
    return out.str();
}

std::string readTextFile(const std::string& path) {
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    if (!input) {
        return std::string();
    }
    return std::string(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
}

bool writeTextFile(const std::string& path, const std::string& text) {
    std::ofstream output(path.c_str(), std::ios::out | std::ios::binary);
    output.write(text.data(), static_cast<std::streamsize>(text.size()));
    return static_cast<bool>(output);
}

std::string replacedOnce(std::string text,
                         const std::string& from,
                         const std::string& to) {
    const std::string::size_type position = text.find(from);
    if (position != std::string::npos) {
        text.replace(position, from.size(), to);
    }
    return text;
}

bool matricesNear(const cv::Mat& lhs, const cv::Mat& rhs, double tolerance) {
    if (lhs.empty() || rhs.empty() || lhs.total() != rhs.total()) {
        return false;
    }
    cv::Mat lhs64;
    cv::Mat rhs64;
    lhs.convertTo(lhs64, CV_64F);
    rhs.convertTo(rhs64, CV_64F);
    lhs64 = lhs64.reshape(1, 1);
    rhs64 = rhs64.reshape(1, 1);
    return cv::norm(lhs64, rhs64, cv::NORM_INF) <= tolerance;
}

struct TemporaryFiles {
    std::vector<std::string> paths;

    ~TemporaryFiles() {
        for (std::size_t i = 0; i < paths.size(); ++i) {
            std::remove(paths[i].c_str());
        }
    }

    std::string add(const std::string& suffix) {
        std::ostringstream path;
        path << "/tmp/myline_hik_calibration_core_test_"
             << static_cast<long long>(::getpid()) << '_' << suffix;
        const std::string value = path.str();
        std::remove(value.c_str());
        paths.push_back(value);
        return value;
    }
};

void testDefaultBoardSpec() {
    using namespace hik_calibration;

    const BoardSpec board;
    std::string error;
    CHECK_TRUE(validateBoardSpec(board, &error),
               std::string("default board should be valid: ") + error);
    CHECK_TRUE(board.squaresX == 5, "default squaresX must be exactly 5");
    CHECK_TRUE(board.squaresY == 7, "default squaresY must be exactly 7");
    CHECK_TRUE(nearlyEqual(board.squareLengthMm, 22.0, 0.0),
               "default square length must be exactly 22 mm");
    CHECK_TRUE(nearlyEqual(board.markerLengthMm, 16.0, 0.0),
               "default marker length must be exactly 16 mm");
    CHECK_TRUE(board.dictionaryId == cv::aruco::DICT_4X4_50,
               "default dictionary must be DICT_4X4_50");
    CHECK_TRUE(dictionaryName(board.dictionaryId) == "DICT_4X4_50",
               "dictionaryName must report DICT_4X4_50");

    CHECK_TRUE(board.charucoCornerCount() == 24,
               "5x7 ChArUco board must contain (5-1)*(7-1)=24 corners");
    CHECK_TRUE(nearlyEqual(board.widthMm(), 110.0, 1e-12),
               "5 squares * 22 mm must produce a 110 mm board width");
    CHECK_TRUE(nearlyEqual(board.heightMm(), 154.0, 1e-12),
               "7 squares * 22 mm must produce a 154 mm board height");
}

void testSyntheticCharucoDetection() {
    using namespace hik_calibration;

    const BoardSpec boardSpec;
    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(boardSpec.dictionaryId);
    const cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
        boardSpec.squaresX, boardSpec.squaresY,
        static_cast<float>(boardSpec.squareLengthMm),
        static_cast<float>(boardSpec.markerLengthMm), dictionary);
    cv::Mat image;
    board->draw(cv::Size(900, 1260), image, 40, 1);

    DetectionOptions options;
    options.minLaplacianVariance = 30.0;
    // OpenCV's synthetic board has perfectly saturated white squares; real
    // captures retain the production 1% saturation gate.
    options.maxSaturationRatio = 1.0;
    CharucoDetectionResult detection;
    const bool ok = detectCharuco(
        image, "synthetic_charuco", boardSpec, options, &detection);
    CHECK_TRUE(ok, std::string("synthetic ChArUco detection failed: ") +
                       detection.error);
    CHECK_TRUE(detection.observation.ids.size() ==
                   static_cast<std::size_t>(boardSpec.charucoCornerCount()),
               "synthetic board should expose all 24 ChArUco corners");
    CHECK_TRUE(detection.observation.quality.laplacianVariance >= 30.0,
               "synthetic board should pass the configured sharpness gate");
}

std::vector<hik_calibration::LaserPlaneSample> makeSyntheticPlaneSamples(
    const cv::Vec3d& unitNormal,
    double dMm,
    int poseCount,
    int inliersPerPose,
    int outliersPerPose) {
    std::vector<hik_calibration::LaserPlaneSample> samples;
    samples.reserve(static_cast<std::size_t>(poseCount));

    cv::Vec3d tangentU = unitNormal.cross(cv::Vec3d(0.0, 0.0, 1.0));
    tangentU *= 1.0 / cv::norm(tangentU);
    cv::Vec3d tangentV = unitNormal.cross(tangentU);
    tangentV *= 1.0 / cv::norm(tangentV);
    const cv::Vec3d pointOnPlane = -dMm * unitNormal;

    for (int pose = 0; pose < poseCount; ++pose) {
        hik_calibration::LaserPlaneSample sample;
        std::ostringstream id;
        id << "synthetic_pose_" << pose;
        sample.sampleId = id.str();
        sample.cameraPointsMm.reserve(
            static_cast<std::size_t>(inliersPerPose + outliersPerPose));

        for (int i = 0; i < inliersPerPose; ++i) {
            const int column = i % 10;
            const int row = i / 10;
            const double u = -105.0 + 21.0 * column + 2.7 * pose;
            const double v = -66.0 + 24.0 * row - 1.9 * pose;
            const double normalNoise = 0.025 * std::sin(0.71 * i + 0.43 * pose);
            const cv::Vec3d p = pointOnPlane + u * tangentU + v * tangentV +
                                normalNoise * unitNormal;
            sample.cameraPointsMm.push_back(cv::Point3d(p[0], p[1], p[2]));
        }

        for (int i = 0; i < outliersPerPose; ++i) {
            const double u = -90.0 + 19.0 * i + 1.5 * pose;
            const double v = -55.0 + 13.0 * ((i * 3 + pose) % 9);
            const double offset = 5.0 + 0.8 * i + 0.2 * pose;
            const cv::Vec3d p = pointOnPlane + u * tangentU + v * tangentV +
                                offset * unitNormal;
            sample.cameraPointsMm.push_back(cv::Point3d(p[0], p[1], p[2]));
        }
        samples.push_back(sample);
    }
    return samples;
}

void testSyntheticLaserPlane(hik_calibration::LaserPlaneFitResult* fittedResult) {
    using namespace hik_calibration;

    cv::Vec3d expectedNormal(0.18, -0.31, 0.933);
    expectedNormal *= 1.0 / cv::norm(expectedNormal);
    const double expectedDmm = -250.0;
    const int poseCount = 8;
    const int inliersPerPose = 60;
    const int outliersPerPose = 10;
    const std::vector<LaserPlaneSample> samples = makeSyntheticPlaneSamples(
        expectedNormal, expectedDmm, poseCount, inliersPerPose, outliersPerPose);

    PlaneFitOptions options;
    options.minPoseCount = poseCount;
    options.minPointsPerPose = 40;
    options.maxPointsPerPose = 1000;
    options.ransacIterations = 2500;
    options.ransacThresholdMm = 0.20;
    options.minInlierRatio = 0.75;
    options.randomSeed = 0x5a17u;

    LaserPlaneFitResult result;
    const bool ok = fitLaserPlane(samples, options, &result);
    CHECK_TRUE(ok, std::string("fitLaserPlane returned false: ") + result.error);
    CHECK_TRUE(result.ok, std::string("fit result is not marked OK: ") + result.error);
    if (!ok || !result.ok) {
        if (fittedResult) {
            *fittedResult = result;
        }
        return;
    }

    const double normalLength = cv::norm(result.plane.normal);
    const double dot = std::max(-1.0, std::min(1.0,
        result.plane.normal.dot(expectedNormal) / normalLength));
    const double angleDeg = std::acos(dot) * 180.0 / 3.14159265358979323846;

    CHECK_TRUE(nearlyEqual(normalLength, 1.0, 1e-9),
               "fitted laser-plane normal must be normalized");
    CHECK_TRUE(result.plane.dMm <= 0.0,
               "laser-plane canonical sign must satisfy d <= 0");
    CHECK_TRUE(angleDeg < 0.10,
               std::string("plane normal angular error too large: ") +
                   numberText(angleDeg) + " deg");
    CHECK_TRUE(std::fabs(result.plane.dMm - expectedDmm) < 0.10,
               std::string("plane d recovery error too large: fitted=") +
                   numberText(result.plane.dMm));

    const int expectedPointCount = poseCount * (inliersPerPose + outliersPerPose);
    CHECK_TRUE(result.poseCount == poseCount, "fit must retain all eight poses");
    CHECK_TRUE(result.pointCount == expectedPointCount,
               "fit point count must include the complete synthetic dataset");
    CHECK_TRUE(result.inlierCount >= poseCount * (inliersPerPose - 1),
               "almost every low-noise plane point should be an inlier");
    CHECK_TRUE(result.inlierCount < result.pointCount,
               "injected off-plane points must not all become inliers");
    CHECK_TRUE(result.inlierRatio > 0.82 && result.inlierRatio < 0.92,
               std::string("unexpected inlier ratio: ") + numberText(result.inlierRatio));
    CHECK_TRUE(result.inlierDistanceMm.mean < 0.04,
               "mean inlier distance should remain below 0.04 mm");
    CHECK_TRUE(result.inlierDistanceMm.rms < 0.05,
               "RMS inlier distance should remain below 0.05 mm");
    CHECK_TRUE(result.inlierDistanceMm.p95 < 0.06,
               "P95 inlier distance should remain below 0.06 mm");
    CHECK_TRUE(result.inlierDistanceMm.maximum <= options.ransacThresholdMm + 1e-9,
               "maximum accepted inlier distance must obey the RANSAC threshold");
    CHECK_TRUE(result.poses.size() == static_cast<std::size_t>(poseCount),
               "per-pose metrics must be reported for all poses");
    for (std::size_t i = 0; i < result.poses.size(); ++i) {
        const PlanePoseStatistics& stats = result.poses[i];
        CHECK_TRUE(stats.pointCount == inliersPerPose + outliersPerPose,
                   "per-pose point count is inconsistent");
        CHECK_TRUE(stats.inlierCount >= inliersPerPose - 1,
                   "per-pose inlier count is unexpectedly low");
        CHECK_TRUE(stats.distanceMm.rms < 0.05,
                   "per-pose RMS plane distance is unexpectedly high");
    }

    if (fittedResult) {
        *fittedResult = result;
    }
}

void checkPlainYaml(const std::string& path, const std::string& label) {
    const std::string text = readTextFile(path);
    CHECK_TRUE(!text.empty(), label + " YAML should be readable and non-empty");
    CHECK_TRUE(text.find("!!opencv-matrix") == std::string::npos,
               label + " YAML must not contain !!opencv-matrix");
}

void testYamlRoundTrips(const hik_calibration::LaserPlaneFitResult& planeFit) {
    using namespace hik_calibration;

    TemporaryFiles temporaryFiles;
    const std::string intrinsicsPath = temporaryFiles.add("intrinsics.yaml");
    const std::string planePath = temporaryFiles.add("laser_plane.yaml");
    const std::string wrongModelPath = temporaryFiles.add("wrong_model.yaml");
    const std::string wrongPlaneTypePath = temporaryFiles.add("wrong_plane_type.yaml");

    IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.board = BoardSpec();
    intrinsics.imageSize = cv::Size(1440, 1080);
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        1210.125, 0.0, 719.875,
        0.0, 1208.625, 539.375,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.08125, 0.0145, 0.00031, -0.00027, -0.0021);
    intrinsics.intrinsicStdDeviations = cv::Mat::zeros(1, 9, CV_64F);
    intrinsics.calibrationRmsPx = 0.2175;
    intrinsics.inputViewCount = 12;
    intrinsics.acceptedViewCount = 10;
    intrinsics.rejectedViewCount = 2;

    IntrinsicsYamlMetadata intrinsicsMetadata;
    intrinsicsMetadata.cameraName = "hik_line_laser_camera";
    intrinsicsMetadata.cameraModel = "MV-CS016-10GM";
    intrinsicsMetadata.cameraSerial = "DA8784601";
    intrinsicsMetadata.frameId = "hik_camera_optical_frame";
    intrinsicsMetadata.pixelFormat = "Mono8";
    intrinsicsMetadata.printedPatternSha256 = "board-sha256-test";
    intrinsicsMetadata.generatedAt = "2026-07-16T15:00:00+08:00";

    std::string error;
    const bool intrinsicsSaved = saveIntrinsicsYaml(
        intrinsicsPath, intrinsics, intrinsicsMetadata, &error);
    CHECK_TRUE(intrinsicsSaved,
               std::string("saveIntrinsicsYaml failed: ") + error);
    if (intrinsicsSaved) {
        checkPlainYaml(intrinsicsPath, "intrinsics");

        IntrinsicCalibrationResult loaded;
        IntrinsicsYamlMetadata loadedMetadata;
        error.clear();
        const bool loadedOk = loadIntrinsicsYaml(
            intrinsicsPath, &loaded, &loadedMetadata, &error);
        CHECK_TRUE(loadedOk,
                   std::string("loadIntrinsicsYaml failed: ") + error);
        if (loadedOk) {
            CHECK_TRUE(loaded.imageSize == intrinsics.imageSize,
                       "intrinsics image size changed during YAML round trip");
            CHECK_TRUE(matricesNear(loaded.cameraMatrix, intrinsics.cameraMatrix, 1e-10),
                       "camera matrix changed during YAML round trip");
            CHECK_TRUE(matricesNear(loaded.distCoeffs, intrinsics.distCoeffs, 1e-10),
                       "distortion coefficients changed during YAML round trip");
            CHECK_TRUE(nearlyEqual(loaded.calibrationRmsPx,
                                   intrinsics.calibrationRmsPx, 1e-10),
                       "intrinsic RMS changed during YAML round trip");
            CHECK_TRUE(loaded.board.squaresX == intrinsics.board.squaresX &&
                           loaded.board.squaresY == intrinsics.board.squaresY &&
                           nearlyEqual(loaded.board.squareLengthMm,
                                       intrinsics.board.squareLengthMm, 1e-12) &&
                           nearlyEqual(loaded.board.markerLengthMm,
                                       intrinsics.board.markerLengthMm, 1e-12) &&
                           loaded.board.dictionaryId == intrinsics.board.dictionaryId,
                       "board specification changed in intrinsics YAML round trip");
            CHECK_TRUE(loadedMetadata.cameraName == intrinsicsMetadata.cameraName &&
                           loadedMetadata.cameraModel == intrinsicsMetadata.cameraModel &&
                           loadedMetadata.cameraSerial == intrinsicsMetadata.cameraSerial &&
                           loadedMetadata.frameId == intrinsicsMetadata.frameId &&
                           loadedMetadata.pixelFormat == intrinsicsMetadata.pixelFormat &&
                           loadedMetadata.printedPatternSha256 ==
                               intrinsicsMetadata.printedPatternSha256 &&
                           loadedMetadata.generatedAt == intrinsicsMetadata.generatedAt,
                       "intrinsics metadata changed during YAML round trip");
        }

        const std::string wrongModel = replacedOnce(
            readTextFile(intrinsicsPath),
            "distortion_model: \"plumb_bob\"",
            "distortion_model: \"equidistant\"");
        CHECK_TRUE(writeTextFile(wrongModelPath, wrongModel),
                   "could not create wrong-model intrinsic YAML");
        IntrinsicCalibrationResult rejectedIntrinsics;
        error.clear();
        CHECK_TRUE(!loadIntrinsicsYaml(wrongModelPath, &rejectedIntrinsics, nullptr, &error),
                   "intrinsic loader accepted an equidistant model as plumb_bob");
        CHECK_TRUE(!error.empty(), "wrong intrinsic model produced no error reason");
    }

    CHECK_TRUE(planeFit.ok,
               "laser-plane YAML test requires a successful synthetic fit");
    if (!planeFit.ok) {
        return;
    }

    LaserPlaneYamlMetadata planeMetadata;
    planeMetadata.cameraFrame = "hik_camera_optical_frame";
    planeMetadata.intrinsicsFile = "hik_intrinsics.yaml";
    planeMetadata.intrinsicsSha256 = "intrinsics-sha256-test";
    planeMetadata.printedPatternSha256 = "board-sha256-test";
    planeMetadata.datasetManifest = "laser_plane_manifest.csv";
    planeMetadata.datasetManifestSha256 = "manifest-sha256-test";
    planeMetadata.generatedAt = "2026-07-16T15:01:00+08:00";
    planeMetadata.validCameraZMinMm = 120.0;
    planeMetadata.validCameraZMaxMm = 650.0;

    const BoardSpec board;
    error.clear();
    const bool planeSaved = saveLaserPlaneYaml(
        planePath, planeFit, board, planeMetadata, &error);
    CHECK_TRUE(planeSaved,
               std::string("saveLaserPlaneYaml failed: ") + error);
    if (planeSaved) {
        checkPlainYaml(planePath, "laser-plane");

        LaserPlaneFitResult loadedFit;
        BoardSpec loadedBoard;
        LaserPlaneYamlMetadata loadedMetadata;
        error.clear();
        const bool loadedOk = loadLaserPlaneYaml(
            planePath, &loadedFit, &loadedBoard, &loadedMetadata, &error);
        CHECK_TRUE(loadedOk,
                   std::string("loadLaserPlaneYaml failed: ") + error);
        if (loadedOk) {
            CHECK_TRUE(cv::norm(loadedFit.plane.normal - planeFit.plane.normal) <= 1e-10,
                       "laser-plane normal changed during YAML round trip");
            CHECK_TRUE(nearlyEqual(loadedFit.plane.dMm, planeFit.plane.dMm, 1e-10),
                       "laser-plane d changed during YAML round trip");
            CHECK_TRUE(nearlyEqual(loadedFit.inlierRatio,
                                   planeFit.inlierRatio, 1e-10),
                       "laser-plane inlier ratio changed during YAML round trip");
            CHECK_TRUE(nearlyEqual(loadedFit.inlierDistanceMm.rms,
                                   planeFit.inlierDistanceMm.rms, 1e-10),
                       "laser-plane RMS changed during YAML round trip");
            CHECK_TRUE(nearlyEqual(loadedFit.inlierDistanceMm.p95,
                                   planeFit.inlierDistanceMm.p95, 1e-10),
                       "laser-plane P95 changed during YAML round trip");
            CHECK_TRUE(loadedBoard.squaresX == board.squaresX &&
                           loadedBoard.squaresY == board.squaresY &&
                           nearlyEqual(loadedBoard.squareLengthMm,
                                       board.squareLengthMm, 1e-12) &&
                           nearlyEqual(loadedBoard.markerLengthMm,
                                       board.markerLengthMm, 1e-12) &&
                           loadedBoard.dictionaryId == board.dictionaryId,
                       "board specification changed in laser-plane YAML round trip");
            CHECK_TRUE(loadedMetadata.cameraFrame == planeMetadata.cameraFrame &&
                           loadedMetadata.intrinsicsFile == planeMetadata.intrinsicsFile &&
                           loadedMetadata.intrinsicsSha256 ==
                               planeMetadata.intrinsicsSha256 &&
                           loadedMetadata.printedPatternSha256 ==
                               planeMetadata.printedPatternSha256 &&
                           loadedMetadata.datasetManifest ==
                               planeMetadata.datasetManifest &&
                           loadedMetadata.datasetManifestSha256 ==
                               planeMetadata.datasetManifestSha256 &&
                           loadedMetadata.generatedAt == planeMetadata.generatedAt &&
                           nearlyEqual(loadedMetadata.validCameraZMinMm,
                                       planeMetadata.validCameraZMinMm, 1e-10) &&
                           nearlyEqual(loadedMetadata.validCameraZMaxMm,
                                       planeMetadata.validCameraZMaxMm, 1e-10),
                       "laser-plane metadata changed during YAML round trip");
        }

        const std::string wrongPlaneType = replacedOnce(
            readTextFile(planePath),
            "calibration_type: \"line_laser_plane\"",
            "calibration_type: \"camera_intrinsics\"");
        CHECK_TRUE(writeTextFile(wrongPlaneTypePath, wrongPlaneType),
                   "could not create wrong-type laser YAML");
        LaserPlaneFitResult rejectedPlane;
        error.clear();
        CHECK_TRUE(!loadLaserPlaneYaml(
                       wrongPlaneTypePath, &rejectedPlane, nullptr, nullptr, &error),
                   "laser-plane loader accepted the wrong calibration type");
        CHECK_TRUE(!error.empty(), "wrong laser-plane type produced no error reason");
    }
}

void expectInvalidBoard(const hik_calibration::BoardSpec& board,
                        const std::string& caseName) {
    std::string error;
    const bool accepted = hik_calibration::validateBoardSpec(board, &error);
    CHECK_TRUE(!accepted, "invalid board was accepted: " + caseName);
    CHECK_TRUE(!error.empty(), "invalid board produced no reason: " + caseName);
}

void testInvalidBoardSpecs() {
    using namespace hik_calibration;

    BoardSpec board;
    board.squaresX = 1;
    expectInvalidBoard(board, "squaresX < 2");

    board = BoardSpec();
    board.squaresY = 1;
    expectInvalidBoard(board, "squaresY < 2");

    board = BoardSpec();
    board.squareLengthMm = 0.0;
    expectInvalidBoard(board, "zero square length");

    board = BoardSpec();
    board.squareLengthMm = std::numeric_limits<double>::quiet_NaN();
    expectInvalidBoard(board, "non-finite square length");

    board = BoardSpec();
    board.markerLengthMm = board.squareLengthMm;
    expectInvalidBoard(board, "marker length is not smaller than square length");

    board = BoardSpec();
    board.dictionaryId = -12345;
    expectInvalidBoard(board, "unsupported dictionary id");
}

void testSyntheticStaticProfileReconstruction() {
    using namespace hik_calibration;

    const cv::Size imageSize(120, 100);
    cv::Mat laserOff(imageSize, CV_8UC1, cv::Scalar(30));
    cv::Mat laserOn = laserOff.clone();
    for (int row = 0; row < laserOn.rows; ++row) {
        laserOn.at<unsigned char>(row, 68) = 70;
        laserOn.at<unsigned char>(row, 69) = 150;
        laserOn.at<unsigned char>(row, 70) = 250;
        laserOn.at<unsigned char>(row, 71) = 150;
        laserOn.at<unsigned char>(row, 72) = 70;
    }

    IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.imageSize = imageSize;
    intrinsics.board = BoardSpec();
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        100.0, 0.0, 60.0,
        0.0, 100.0, 50.0,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    LaserPlaneFitResult laserPlane;
    laserPlane.ok = true;
    laserPlane.plane.normal = cv::Vec3d(0.0, 0.0, 1.0);
    laserPlane.plane.dMm = -500.0;

    StaticProfileOptions options;
    options.minimumDepthMm = 450.0;
    options.maximumDepthMm = 550.0;
    options.stripe.minPointCount = 80;
    options.minReconstructedPoints = 80;

    StaticProfileResult result;
    const bool ok = reconstructStaticProfile(
        laserOff, laserOn, "synthetic_profile", intrinsics, laserPlane,
        intrinsics.board, options, &result);
    CHECK_TRUE(ok, std::string("synthetic profile reconstruction failed: ") + result.error);
    CHECK_TRUE(result.ok, "synthetic profile result must be marked OK");
    CHECK_TRUE(result.points.size() == 100,
               "one laser center must be reconstructed for every image row");
    CHECK_TRUE(nearlyEqual(result.minimumDepthMm, 500.0, 1e-6) &&
                   nearlyEqual(result.maximumDepthMm, 500.0, 1e-6),
               "synthetic profile depth must equal the z=500 mm laser plane");
    CHECK_TRUE(result.lineDistanceMm.rms < 1e-6,
               "perfect synthetic stripe must reconstruct as a straight 3-D line");
    CHECK_TRUE(result.lineQualityPassed,
               "perfect synthetic stripe must pass line-quality threshold");
    CHECK_TRUE(!result.boardValidationAvailable,
               "plain synthetic images must not claim ChArUco plane validation");

    cv::Mat horizontalLaserOn(imageSize, CV_8UC1, cv::Scalar(30));
    for (int column = 0; column < horizontalLaserOn.cols; ++column) {
        horizontalLaserOn.at<unsigned char>(43, column) = 70;
        horizontalLaserOn.at<unsigned char>(44, column) = 150;
        horizontalLaserOn.at<unsigned char>(45, column) = 250;
        horizontalLaserOn.at<unsigned char>(46, column) = 150;
        horizontalLaserOn.at<unsigned char>(47, column) = 70;
    }
    StaticProfileResult horizontalResult;
    CHECK_TRUE(reconstructStaticProfile(
                   laserOff, horizontalLaserOn, "horizontal_profile", intrinsics,
                   laserPlane, intrinsics.board, options, &horizontalResult),
               std::string("horizontal profile reconstruction failed: ") +
                   horizontalResult.error);
    CHECK_TRUE(horizontalResult.ok && horizontalResult.points.size() == 120,
               "horizontal stripe must reconstruct one center for every image column");
    CHECK_TRUE(std::fabs(horizontalResult.stripe.front().pixel.y - 45.0) < 0.2,
               "horizontal stripe must retain its sub-pixel vertical center");
    CHECK_TRUE(horizontalResult.stripe.front().row == 45 &&
                   horizontalResult.stripe.front().peakX == 0,
               "horizontal stripe indices must remain true image row and column");

    TemporaryFiles files;
    const std::string plyPath = files.add("profile.ply");
    const std::string csvPath = files.add("profile.csv");
    std::string error;
    CHECK_TRUE(saveStaticProfilePly(
                   plyPath, result, "hik_camera_optical_frame", &error),
               std::string("profile PLY save failed: ") + error);
    CHECK_TRUE(saveStaticProfileCsv(csvPath, result, &error),
               std::string("profile CSV save failed: ") + error);
    CHECK_TRUE(readTextFile(plyPath).find("element vertex 100") != std::string::npos,
               "PLY header must contain the reconstructed point count");
    CHECK_TRUE(readTextFile(csvPath).find("u_px,v_px,x_mm,y_mm,z_mm") == 0,
               "CSV must expose pixel and camera-frame coordinates");

    options.maximumDepthMm = 490.0;
    StaticProfileResult rejected;
    CHECK_TRUE(!reconstructStaticProfile(
                   laserOff, laserOn, "outside_depth", intrinsics, laserPlane,
                   intrinsics.board, options, &rejected),
               "profile outside calibrated depth range must be rejected");
    CHECK_TRUE(rejected.rejectedDepthCount == 100,
               "every out-of-range point must be counted as a depth rejection");

    const BoardSpec boardSpec;
    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(boardSpec.dictionaryId);
    const cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
        boardSpec.squaresX, boardSpec.squaresY,
        static_cast<float>(boardSpec.squareLengthMm),
        static_cast<float>(boardSpec.markerLengthMm), dictionary);
    cv::Mat rendered;
    board->draw(cv::Size(900, 1260), rendered, 40, 1);
    cv::Mat boardOff;
    rendered.convertTo(boardOff, CV_8U, 0.5, 30.0);
    cv::Mat boardOn = boardOff.clone();
    for (int row = 0; row < boardOn.rows; ++row) {
        const int additions[5] = {40, 80, 120, 80, 40};
        for (int offset = -2; offset <= 2; ++offset) {
            unsigned char& pixel = boardOn.at<unsigned char>(row, 450 + offset);
            pixel = static_cast<unsigned char>(std::min(
                255, static_cast<int>(pixel) + additions[offset + 2]));
        }
    }

    IntrinsicCalibrationResult boardIntrinsics;
    boardIntrinsics.ok = true;
    boardIntrinsics.imageSize = boardOff.size();
    boardIntrinsics.board = boardSpec;
    boardIntrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        1200.0, 0.0, 450.0,
        0.0, 1200.0, 630.0,
        0.0, 0.0, 1.0);
    boardIntrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    DetectionOptions boardDetectionOptions;
    boardDetectionOptions.minLaplacianVariance = 5.0;
    boardDetectionOptions.maxSaturationRatio = 1.0;
    CharucoDetectionResult boardDetection;
    CHECK_TRUE(detectCharuco(
                   boardOff, "profile_validation_board", boardSpec,
                   boardDetectionOptions, &boardDetection,
                   boardIntrinsics.cameraMatrix, boardIntrinsics.distCoeffs),
               std::string("validation board detection failed: ") + boardDetection.error);
    BoardPoseOptions boardPoseOptions;
    BoardPoseResult boardPose;
    CHECK_TRUE(estimateBoardPose(
                   boardDetection.observation, boardSpec,
                   boardIntrinsics.cameraMatrix, boardIntrinsics.distCoeffs,
                   boardPoseOptions, &boardPose),
               std::string("validation board pose failed: ") + boardPose.error);
    if (boardPose.ok) {
        LaserPlaneFitResult boardPlane;
        boardPlane.ok = true;
        boardPlane.plane.normal = cv::Vec3d(
            boardPose.rotation(0, 2), boardPose.rotation(1, 2),
            boardPose.rotation(2, 2));
        boardPlane.plane.dMm = -boardPlane.plane.normal.dot(boardPose.tvec);
        if (boardPlane.plane.dMm > 0.0) {
            boardPlane.plane.normal *= -1.0;
            boardPlane.plane.dMm *= -1.0;
        }
        StaticProfileOptions boardOptions;
        boardOptions.minimumDepthMm = 10.0;
        boardOptions.maximumDepthMm = 5000.0;
        boardOptions.boardDetection = boardDetectionOptions;
        boardOptions.boardPose = boardPoseOptions;
        StaticProfileResult boardResult;
        CHECK_TRUE(reconstructStaticProfile(
                       boardOff, boardOn, "profile_with_board", boardIntrinsics,
                       boardPlane, boardSpec, boardOptions, &boardResult),
                   std::string("board profile reconstruction failed: ") + boardResult.error);
        CHECK_TRUE(boardResult.boardValidationAvailable,
                   std::string("ChArUco plane residual should be available: ") +
                       boardResult.boardValidationMessage);
        CHECK_TRUE(boardResult.boardValidationPointCount >=
                       boardOptions.minBoardValidationPoints,
                   "enough reconstructed points must lie inside the physical board");
        CHECK_TRUE(boardResult.boardPlaneDistanceMm.rms < 1e-6,
                   "profile reconstructed on the validation plane must have near-zero RMS");
        CHECK_TRUE(boardResult.boardValidationPassed,
                   "near-zero synthetic board residual must pass validation");
    }
}

void testConstantLaserScanCore() {
    using namespace hik_calibration;

    const cv::Size imageSize(120, 100);
    cv::Mat laserOn(imageSize, CV_8UC1);
    for (int row = 0; row < laserOn.rows; ++row) {
        for (int column = 0; column < laserOn.cols; ++column) {
            laserOn.at<unsigned char>(row, column) =
                static_cast<unsigned char>(30 + column / 20 + row / 25);
        }
        const int values[5] = {75, 155, 245, 155, 75};
        for (int offset = -2; offset <= 2; ++offset) {
            laserOn.at<unsigned char>(row, 70 + offset) = values[offset + 2];
        }
    }

    IntrinsicCalibrationResult intrinsics;
    intrinsics.ok = true;
    intrinsics.imageSize = imageSize;
    intrinsics.board = BoardSpec();
    intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        100.0, 0.0, 60.0,
        0.0, 100.0, 50.0,
        0.0, 0.0, 1.0);
    intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    LaserPlaneFitResult laserPlane;
    laserPlane.ok = true;
    laserPlane.plane.normal = cv::Vec3d(0.0, 0.0, 1.0);
    laserPlane.plane.dMm = -500.0;

    SingleFrameProfileOptions options;
    options.reconstruction.minimumDepthMm = 450.0;
    options.reconstruction.maximumDepthMm = 550.0;
    options.reconstruction.stripe.minPointCount = 80;
    options.reconstruction.minReconstructedPoints = 80;
    options.minimumRawIntensity = 60;
    StaticProfileResult profile;
    CHECK_TRUE(reconstructSingleFrameProfile(
                   laserOn, "constant_laser", intrinsics, laserPlane,
                   options, &profile),
               std::string("constant-laser profile failed: ") + profile.error);
    CHECK_TRUE(profile.ok && profile.points.size() == 100,
               "constant-laser profile must reconstruct one point per row");
    CHECK_TRUE(nearlyEqual(profile.minimumDepthMm, 500.0, 1e-6) &&
                   nearlyEqual(profile.maximumDepthMm, 500.0, 1e-6),
               "constant-laser points must intersect the z=500 mm plane");
    CHECK_TRUE(profile.lineQualityPassed,
               "perfect constant-laser ridge must pass line quality");
    CHECK_TRUE(std::fabs(profile.stripe.front().pixel.x - 70.0) < 0.2,
               "constant-laser ridge must retain its sub-pixel center");

    cv::Mat horizontalLaserOn(imageSize, CV_8UC1);
    for (int row = 0; row < horizontalLaserOn.rows; ++row) {
        for (int column = 0; column < horizontalLaserOn.cols; ++column) {
            horizontalLaserOn.at<unsigned char>(row, column) =
                static_cast<unsigned char>(30 + column / 20 + row / 25);
        }
    }
    const int horizontalValues[5] = {75, 155, 245, 155, 75};
    for (int column = 0; column < horizontalLaserOn.cols; ++column) {
        for (int offset = -2; offset <= 2; ++offset) {
            horizontalLaserOn.at<unsigned char>(45 + offset, column) =
                horizontalValues[offset + 2];
        }
    }
    StaticProfileResult horizontalProfile;
    CHECK_TRUE(reconstructSingleFrameProfile(
                   horizontalLaserOn, "constant_horizontal_laser", intrinsics,
                   laserPlane, options, &horizontalProfile),
               std::string("constant horizontal-laser profile failed: ") +
                   horizontalProfile.error);
    CHECK_TRUE(horizontalProfile.ok && horizontalProfile.points.size() == 120,
               "constant horizontal laser must reconstruct one point per column");
    CHECK_TRUE(std::fabs(horizontalProfile.stripe.front().pixel.y - 45.0) < 0.2,
               "constant horizontal-laser ridge must retain its sub-pixel center");

    cv::Mat noLaser(imageSize, CV_8UC1, cv::Scalar(35));
    StaticProfileResult rejected;
    CHECK_TRUE(!reconstructSingleFrameProfile(
                   noLaser, "no_laser", intrinsics, laserPlane,
                   options, &rejected),
               "an image without a laser ridge must be rejected");

    hik_scan::Pose6D start;
    start.x = 0.0; start.y = 1.0; start.z = 2.0;
    start.rx = 3.0; start.ry = 4.0; start.rz = 5.0;
    hik_scan::Pose6D end;
    end.x = 10.0; end.y = 1.0; end.z = 2.0;
    end.rx = 30.0; end.ry = 40.0; end.rz = 50.0;
    std::vector<hik_scan::Pose6D> targets;
    std::string error;
    CHECK_TRUE(hik_scan::buildLinearFlangePath(
                   start, end, 3.0, 10, &targets, &error),
               std::string("linear scan path failed: ") + error);
    CHECK_TRUE(targets.size() == 5,
               "10 mm path with a 3 mm maximum step must have five targets");
    CHECK_TRUE(nearlyEqual(targets[1].x, 2.5, 1e-12) &&
                   nearlyEqual(targets.back().x, 10.0, 1e-12),
               "linear targets must be equally spaced and include the endpoint");
    CHECK_TRUE(nearlyEqual(targets.back().rx, start.rx, 0.0) &&
                   nearlyEqual(targets.back().ry, start.ry, 0.0) &&
                   nearlyEqual(targets.back().rz, start.rz, 0.0),
               "scan path must lock orientation to the start pose");
    CHECK_TRUE(!hik_scan::buildLinearFlangePath(
                   start, end, 1.0, 5, &targets, &error),
               "path exceeding the point limit must be rejected");

    TemporaryFiles files;
    const std::string handEyePath = files.add("scan_handeye.yaml");
    const std::string handEyeYaml =
        "schema_version: 1\n"
        "calibration_type: eye_in_hand\n"
        "mode: camera_to_flange\n"
        "parent_frame: fairino_flange_reported\n"
        "child_frame: hik_camera_optical_frame\n"
        "T_flange_camera: [1, 0, 0, 10, 0, 1, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1]\n"
        "camera:\n  serial: camera-1\n"
        "sources:\n  intrinsics_sha256: abc123\n";
    CHECK_TRUE(writeTextFile(handEyePath, handEyeYaml),
               "scan hand-eye fixture must be writable");
    hik_scan::HandEyeFile handEye;
    CHECK_TRUE(hik_scan::loadHandEyeYaml(handEyePath, &handEye, &error),
               std::string("scan hand-eye load failed: ") + error);
    CHECK_TRUE(handEye.ok && nearlyEqual(handEye.flangeFromCamera(1, 3), 20.0, 0.0),
               "scan hand-eye loader must preserve T_flange_camera");
    const std::string invalidHandEyePath = files.add("scan_handeye_invalid.yaml");
    CHECK_TRUE(writeTextFile(invalidHandEyePath,
                            replacedOnce(handEyeYaml, "camera_to_flange", "flange_to_camera")),
               "invalid hand-eye fixture must be writable");
    CHECK_TRUE(!hik_scan::loadHandEyeYaml(invalidHandEyePath, &handEye, &error),
               "opposite hand-eye transform direction must be rejected");

    StaticProfileResult onePointProfile;
    onePointProfile.ok = true;
    StaticProfilePoint point;
    point.cameraPointMm = cv::Point3d(1.0, 2.0, 3.0);
    point.stripe.pixel = cv::Point2d(70.0, 50.0);
    point.stripe.confidence = 0.9;
    point.stripe.peakDifference = 120.0;
    onePointProfile.points.push_back(point);
    cv::Matx44d baseFromFlange = cv::Matx44d::eye();
    baseFromFlange(0, 3) = 100.0;
    cv::Matx44d flangeFromCamera = cv::Matx44d::eye();
    flangeFromCamera(1, 3) = 20.0;
    std::vector<hik_scan::CloudPoint> cloud;
    CHECK_TRUE(hik_scan::appendProfileInBase(
                   onePointProfile, baseFromFlange, flangeFromCamera,
                   7, &cloud, &error),
               std::string("base-frame profile append failed: ") + error);
    CHECK_TRUE(cloud.size() == 1 &&
                   nearlyEqual(cloud[0].basePointMm.x, 101.0, 1e-12) &&
                   nearlyEqual(cloud[0].basePointMm.y, 22.0, 1e-12) &&
                   nearlyEqual(cloud[0].basePointMm.z, 3.0, 1e-12),
               "cloud transform must apply T_base_flange*T_flange_camera");
    cv::Matx44d exposureMidpointBaseFromCamera = cv::Matx44d::eye();
    exposureMidpointBaseFromCamera(2, 3) = 40.0;
    std::vector<hik_scan::CloudPoint> directlyTransformed;
    CHECK_TRUE(hik_scan::appendProfileUsingBaseFromCamera(
                   onePointProfile, exposureMidpointBaseFromCamera, 8,
                   &directlyTransformed, &error),
               std::string("direct T_base_camera append failed: ") + error);
    CHECK_TRUE(directlyTransformed.size() == 1 &&
                   nearlyEqual(directlyTransformed[0].basePointMm.x, 1.0, 1e-12) &&
                   nearlyEqual(directlyTransformed[0].basePointMm.y, 2.0, 1e-12) &&
                   nearlyEqual(directlyTransformed[0].basePointMm.z, 43.0, 1e-12),
               "continuous cloud must use exposure-midpoint T_base_camera directly");
    hik_scan::CloudPoint nearby = cloud.front();
    nearby.basePointMm.x += 0.1;
    cloud.push_back(nearby);
    const std::vector<hik_scan::CloudPoint> voxel =
        hik_scan::voxelDownsample(cloud, 1.0);
    CHECK_TRUE(voxel.size() == 1,
               "points in one 1 mm voxel must be reduced to one point");
    const std::string plyPath = files.add("scan_cloud.ply");
    CHECK_TRUE(hik_scan::saveScanPly(plyPath, voxel, "base_link", &error),
               std::string("scan PLY save failed: ") + error);
    const std::string ply = readTextFile(plyPath);
    CHECK_TRUE(ply.find("comment frame_id base_link") != std::string::npos &&
                   ply.find("element vertex 1") != std::string::npos,
               "scan PLY must declare base_link and its point count");
}

void testSyntheticHandEyeCalibration() {
    using namespace hik_calibration;

    const cv::Matx44d flangeFromCamera = fairinoBaseFromFlange(
        42.5, -18.0, 126.0, 7.0, -12.0, 3.5);
    const cv::Matx44d baseFromBoard = fairinoBaseFromFlange(
        620.0, 35.0, 180.0, 175.0, 2.0, -88.0);
    std::vector<HandEyeSample> samples;
    for (int index = 0; index < 18; ++index) {
        const double phase = static_cast<double>(index);
        HandEyeSample sample;
        sample.sampleId = std::string("handeye_") + numberText(index);
        sample.baseFromFlange = fairinoBaseFromFlange(
            380.0 + 14.0 * phase,
            -180.0 + 25.0 * static_cast<double>(index % 6),
            360.0 + 18.0 * static_cast<double>(index % 5),
            -25.0 + 7.0 * static_cast<double>(index % 8),
            -20.0 + 9.0 * static_cast<double>((index * 3) % 6),
            -35.0 + 11.0 * static_cast<double>((index * 5) % 7));
        sample.cameraFromBoard = flangeFromCamera.inv() *
                                 sample.baseFromFlange.inv() *
                                 baseFromBoard;
        sample.boardPoseRmsPx = 0.1;
        samples.push_back(sample);
    }

    HandEyeOptions options;
    HandEyeCalibrationResult result;
    CHECK_TRUE(calibrateHandEyeRobust(samples, options, &result),
               std::string("synthetic hand-eye solve failed: ") + result.error);
    CHECK_TRUE(result.ok, "synthetic hand-eye result must be marked OK");
    CHECK_TRUE(result.acceptedSampleCount == 18,
               "perfect synthetic hand-eye set must keep every sample");
    CHECK_TRUE(rigidTranslationDistanceMm(
                   result.flangeFromCamera, flangeFromCamera) < 1e-5,
               "recovered hand-eye translation must match ground truth");
    CHECK_TRUE(rigidRotationDistanceDeg(
                   result.flangeFromCamera, flangeFromCamera) < 1e-5,
               "recovered hand-eye rotation must match ground truth");
    CHECK_TRUE(result.translationConsistencyMm.rms < 1e-6,
               "perfect fixed-board translation consistency must be near zero");
    CHECK_TRUE(result.rotationConsistencyDeg.rms < 1e-6,
               "perfect fixed-board rotation consistency must be near zero");

    TemporaryFiles files;
    const std::string yamlPath = files.add("handeye.yaml");
    HandEyeYamlMetadata metadata;
    metadata.cameraSerial = "synthetic-camera";
    metadata.intrinsicsSha256 = "abc123";
    metadata.datasetManifest = "handeye_manifest.csv";
    std::string error;
    CHECK_TRUE(saveHandEyeYaml(yamlPath, result, metadata, &error),
               std::string("hand-eye YAML save failed: ") + error);
    const std::string yaml = readTextFile(yamlPath);
    CHECK_TRUE(yaml.find("mode: camera_to_flange") != std::string::npos,
               "hand-eye YAML must state camera_to_flange direction");
    CHECK_TRUE(yaml.find("T_flange_camera: [") != std::string::npos,
               "hand-eye YAML must name the matrix direction explicitly");
    CHECK_TRUE(yaml.find("translation_unit: mm") != std::string::npos,
               "hand-eye YAML must state millimetre translation units");

    std::vector<HandEyeSample> degenerate(15, samples.front());
    for (std::size_t index = 0; index < degenerate.size(); ++index) {
        degenerate[index].sampleId = std::string("degenerate_") + numberText(index);
    }
    HandEyeCalibrationResult rejected;
    CHECK_TRUE(!calibrateHandEyeRobust(degenerate, options, &rejected),
               "identical robot poses must fail diversity validation");
    CHECK_TRUE(rejected.error.find("insufficient robot pose diversity") != std::string::npos,
               "degenerate hand-eye failure must explain missing pose diversity");

    std::vector<HandEyeSample> singleAxis;
    for (int index = 0; index < 15; ++index) {
        HandEyeSample sample;
        sample.sampleId = std::string("single_axis_") + numberText(index);
        sample.baseFromFlange = fairinoBaseFromFlange(
            300.0 + 10.0 * index, -100.0 + 8.0 * index, 420.0,
            -35.0 + 5.0 * index, 0.0, 0.0);
        sample.cameraFromBoard = flangeFromCamera.inv() *
                                 sample.baseFromFlange.inv() * baseFromBoard;
        singleAxis.push_back(sample);
    }
    HandEyeCalibrationResult singleAxisRejected;
    CHECK_TRUE(!calibrateHandEyeRobust(singleAxis, options, &singleAxisRejected),
               "poses rotating around only one axis must fail excitation validation");
    CHECK_TRUE(singleAxisRejected.error.find("secondary-axis spread") != std::string::npos,
               "single-axis rejection must report the secondary rotation spread");
}

} // namespace

int main() {
    testDefaultBoardSpec();
    testSyntheticCharucoDetection();

    hik_calibration::LaserPlaneFitResult planeFit;
    testSyntheticLaserPlane(&planeFit);
    testYamlRoundTrips(planeFit);
    testInvalidBoardSpecs();
    testSyntheticStaticProfileReconstruction();
    testConstantLaserScanCore();
    testSyntheticHandEyeCalibration();

    if (gFailures != 0) {
        std::cerr << "HikCalibrationCore tests failed: " << gFailures
                  << " assertion(s)" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "HikCalibrationCore tests passed" << std::endl;
    return EXIT_SUCCESS;
}
