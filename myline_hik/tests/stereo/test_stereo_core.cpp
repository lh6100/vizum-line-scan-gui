#include "stereo/core/StereoDepthEngine.h"
#include "stereo/core/StereoFrameSynchronizer.h"
#include "stereo/calibration/StereoCalibrationEngine.h"
#include "stereo/map/VoxelOccupancyMap.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "FAIL " << __FILE__ << ':' << __LINE__ \
                      << " " << message << '\n'; \
            ++failures; \
        } \
    } while (false)

hik_stereo::StereoRigCalibration syntheticRig() {
    hik_stereo::StereoRigCalibration rig;
    rig.left.intrinsics.ok = true;
    rig.right.intrinsics.ok = true;
    rig.left.intrinsics.imageSize = cv::Size(320, 240);
    rig.right.intrinsics.imageSize = cv::Size(320, 240);
    rig.left.intrinsics.cameraMatrix = (cv::Mat_<double>(3, 3) <<
        200.0, 0.0, 160.0, 0.0, 200.0, 120.0, 0.0, 0.0, 1.0);
    rig.right.intrinsics.cameraMatrix =
        rig.left.intrinsics.cameraMatrix.clone();
    rig.left.intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    rig.right.intrinsics.distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    rig.left.handEye.ok = true;
    rig.right.handEye.ok = true;
    rig.left.handEye.flangeFromCamera = cv::Matx44d::eye();
    rig.right.handEye.flangeFromCamera = cv::Matx44d::eye();
    rig.right.handEye.flangeFromCamera(0, 3) = 100.0;
    rig.rightFromLeft = cv::Matx44d::eye();
    rig.rightFromLeft(0, 3) = -100.0;
    rig.rotationRightFromLeft = cv::Mat::eye(3, 3, CV_64F);
    rig.translationRightFromLeft = (cv::Mat_<double>(3, 1) <<
        -100.0, 0.0, 0.0);
    rig.baselineMm = 100.0;
    rig.relativeRotationDeg = 0.0;
    rig.ok = true;
    return rig;
}

void testFramePairing() {
    hik_stereo::StereoFramePairer pairer(3.0, 4U);
    hik_sync::CameraFrame left;
    left.frameId = 10;
    left.hostCallbackNs = 1000000000LL;
    hik_sync::CameraFrame right;
    right.frameId = 20;
    right.hostCallbackNs = 1002000000LL;
    pairer.pushRight(right);
    pairer.pushLeft(left);
    hik_stereo::StereoFramePair pair;
    CHECK(pairer.takePair(&pair), "two frames inside skew must pair");
    CHECK(pair.left.frameId == 10 && pair.right.frameId == 20,
          "left/right identity must be retained");
    CHECK(std::abs(pair.skewMs - 2.0) < 1.0e-9,
          "pair skew must use monotonic callbacks");

    pairer.configure(30.0, 4U);
    left.frameId = 11;
    left.hostCallbackNs = 2000000000LL;
    right.frameId = 21;
    right.hostCallbackNs = 2018000000LL;
    pairer.pushLeft(left);
    pairer.pushRight(right);
    CHECK(pairer.takePair(&pair),
          "stationary free-running calibration frames inside 30 ms must pair");
    CHECK(std::abs(pair.skewMs - 18.0) < 1.0e-9,
          "free-running calibration pair must retain its measured skew");
}

void testPoseSynchronization() {
    hik_stereo::StereoPoseSynchronizer sync;
    sync.configure(0.0, 20.0);
    hik_sync::RobotSample before;
    before.valid = true;
    before.hostReceiveNs = 1000000000LL;
    before.flangePositionMm = Eigen::Vector3d(0.0, 0.0, 0.0);
    before.flangeOrientation = Eigen::Quaterniond::Identity();
    hik_sync::RobotSample after = before;
    after.hostReceiveNs = 1010000000LL;
    after.flangePositionMm = Eigen::Vector3d(10.0, 0.0, 0.0);
    hik_stereo::StereoFramePair pair;
    pair.left.exposureUs = 0.0;
    pair.right.exposureUs = 0.0;
    pair.midpointHostNs = 1005000000LL;
    sync.pushRobot(before);
    sync.pushPair(pair);
    hik_stereo::SynchronizedStereoFrame output;
    CHECK(!sync.takeSynchronized(&output),
          "pair must wait rather than extrapolate past latest robot sample");
    sync.pushRobot(after);
    CHECK(sync.takeSynchronized(&output),
          "bracketed stereo pair must synchronize");
    CHECK(std::abs(output.baseFromFlange(0, 3) - 5.0) < 1.0e-8,
          "robot pose must interpolate at exposure midpoint");
}

void testSyntheticDepth() {
    hik_stereo::StereoDepthOptions options;
    options.processingSize = cv::Size(320, 240);
    options.minimumDepthMm = 500.0;
    options.maximumDepthMm = 2000.0;
    options.blockSize = 5;
    options.maximumNumDisparities = 128;
    options.enableLeftRightCheck = false;
    hik_stereo::StereoDepthEngine engine;
    std::string error;
    CHECK(engine.configure(syntheticRig(), options, &error), error);

    cv::Mat left(240, 320, CV_8UC1);
    cv::RNG random(12345);
    random.fill(left, cv::RNG::UNIFORM, 0, 255);
    cv::GaussianBlur(left, left, cv::Size(3, 3), 0.7);
    const int disparity = 20;
    cv::Mat right = cv::Mat::zeros(left.size(), left.type());
    left(cv::Rect(disparity, 0, left.cols - disparity, left.rows))
        .copyTo(right(cv::Rect(0, 0, right.cols - disparity, right.rows)));
    hik_stereo::StereoDepthResult result;
    CHECK(engine.compute(left, right, &result), result.error);
    CHECK(result.statistics.validFraction > 0.35,
          "synthetic textured plane must produce dense depth");
    CHECK(std::abs(result.statistics.medianDepthMm - 1000.0) < 35.0,
          "f=200 px, B=100 mm, d=20 px must reconstruct near 1000 mm");
}

void testSyntheticMultiBandDepth() {
    hik_stereo::StereoDepthOptions options;
    options.processingSize = cv::Size(320, 240);
    options.minimumDepthMm = 300.0;
    options.maximumDepthMm = 2500.0;
    options.blockSize = 5;
    options.uniquenessRatio = 5;
    options.speckleWindowSize = 50;
    options.maximumNumDisparities = 128;
    options.enableLeftRightCheck = false;
    options.enableClahe = true;
    options.claheClipLimit = 2.0;
    options.depthBands = {
        {300.0, 600.0},
        {600.0, 1200.0},
        {1200.0, 2500.0},
    };
    hik_stereo::StereoDepthEngine engine;
    std::string error;
    CHECK(engine.configure(syntheticRig(), options, &error), error);

    cv::Mat left(240, 320, CV_8UC1);
    cv::RNG random(54321);
    random.fill(left, cv::RNG::UNIFORM, 0, 255);
    cv::GaussianBlur(left, left, cv::Size(3, 3), 0.7);
    const int disparity = 20;
    cv::Mat right = cv::Mat::zeros(left.size(), left.type());
    left(cv::Rect(disparity, 0, left.cols - disparity, left.rows))
        .copyTo(right(cv::Rect(0, 0, right.cols - disparity, right.rows)));
    hik_stereo::StereoDepthResult result;
    CHECK(engine.compute(left, right, &result), result.error);
    CHECK(result.statistics.bandCount == 3,
          "multi-band result must report all configured depth bands");
    CHECK(result.statistics.totalBandDisparities > 0,
          "multi-band result must report its bounded matching work");
    CHECK(result.statistics.validFraction > 0.35,
          "multi-band fusion must retain a dense textured plane");
    CHECK(std::abs(result.statistics.medianDepthMm - 1000.0) < 35.0,
          "multi-band fusion must reconstruct the band containing the plane");

    options.depthBands = {
        {300.0, 600.0},
        {700.0, 1200.0},
        {1200.0, 2500.0},
    };
    hik_stereo::StereoDepthEngine rejected;
    CHECK(!rejected.configure(syntheticRig(), options, &error),
          "multi-band configuration with a depth gap must be rejected");
}

void testInstalledRigConfiguration() {
    const std::string source = HIK_CALIBRATION_SOURCE_DIR;
    const std::string stereoPath = "/tmp/hik_stereo_loader_test_" +
                                   std::to_string(std::rand()) + ".yaml";
    {
        std::ofstream output(stereoPath.c_str());
        output << "schema_version: 1\n"
               << "calibration_type: stereo_rig\n"
               << "left_camera_serial: \"DA8784601\"\n"
               << "right_camera_serial: \"DB0403208\"\n"
               << "R_right_left: [0.999917414, -0.000142449, 0.012850868, "
                  "0.000312158, 0.999912762, -0.013204931, "
                  "-0.012847866, 0.013207852, 0.999830228]\n"
               << "T_right_left_mm: [126.885212, 0.0718516, 17.4791127]\n";
    }
    hik_stereo::StereoRigCalibration rig;
    std::string error;
    CHECK(hik_stereo::loadStereoRigFromStereoYaml(
              stereoPath,
              source + "/config/devices/scanner_650/hik_intrinsics.yaml",
              source + "/config/devices/scanner_650/hik_handeye.yaml",
              source + "/config/devices/scanner_450/hik_intrinsics.yaml",
              source + "/config/devices/scanner_450/hik_handeye.yaml",
              &rig, &error), error);
    CHECK(std::abs(rig.baselineMm - 128.08) < 0.2,
          "installed physical-left to physical-right baseline must remain near 128 mm");
    for (const cv::Size size : {cv::Size(612, 512), cv::Size(1224, 1024)}) {
        hik_stereo::StereoDepthOptions options;
        options.processingSize = size;
        options.minimumDepthMm = 450.0;
        options.maximumDepthMm = 3000.0;
        options.maximumNumDisparities = 512;
        hik_stereo::StereoDepthEngine engine;
        CHECK(engine.configure(rig, options, &error), error);
        CHECK(engine.numberOfDisparities() <= 512,
              "installed rig must fit the bounded SGBM disparity budget");
    }
    std::remove(stereoPath.c_str());
}

void testFixedIntrinsicStereoCalibration() {
    hik_stereo::StereoRigCalibration rig = syntheticRig();
    rig.left.intrinsics.distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.10, 0.04, -0.0003, 0.0005, -0.01);
    rig.right.intrinsics.distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.38, 0.28, -0.0009, 0.0014, -0.03);
    rig.left.intrinsics.board = hik_calibration::BoardSpec();
    rig.right.intrinsics.board = rig.left.intrinsics.board;
    rig.left.metadata.cameraSerial = "left";
    rig.right.metadata.cameraSerial = "right";
    std::vector<cv::Point3f> boardPoints;
    std::vector<int> ids;
    for (int y = 0; y < rig.left.intrinsics.board.squaresY - 1; ++y) {
        for (int x = 0; x < rig.left.intrinsics.board.squaresX - 1; ++x) {
            boardPoints.push_back(cv::Point3f(
                static_cast<float>((x + 1) * rig.left.intrinsics.board.squareLengthMm),
                static_cast<float>((y + 1) * rig.left.intrinsics.board.squareLengthMm),
                0.0F));
            ids.push_back(static_cast<int>(ids.size()));
        }
    }
    std::vector<hik_stereo::StereoCalibrationSample> samples;
    cv::RNG noise(7654);
    for (int view = 0; view < 18; ++view) {
        const cv::Vec3d rvec(
            (view % 3 - 1) * 0.08,
            ((view / 3) % 3 - 1) * 0.07,
            (view % 5 - 2) * 0.025);
        const cv::Vec3d leftT(
            (view % 4 - 1.5) * 35.0,
            ((view / 4) % 3 - 1.0) * 25.0,
            750.0 + (view % 3) * 120.0);
        cv::Mat rotation;
        cv::Rodrigues(rvec, rotation);
        cv::Mat rightRotation = rig.rotationRightFromLeft * rotation;
        cv::Vec3d rightRvec;
        cv::Rodrigues(rightRotation, rightRvec);
        const cv::Mat rightTMat = rig.rotationRightFromLeft * cv::Mat(leftT) +
                                  rig.translationRightFromLeft;
        const cv::Vec3d rightT(
            rightTMat.at<double>(0), rightTMat.at<double>(1),
            rightTMat.at<double>(2));
        hik_stereo::StereoCalibrationSample sample;
        sample.id = "view_" + std::to_string(view);
        sample.left.ids = ids;
        sample.right.ids = ids;
        cv::projectPoints(boardPoints, rvec, leftT,
                          rig.left.intrinsics.cameraMatrix,
                          rig.left.intrinsics.distCoeffs,
                          sample.left.corners);
        cv::projectPoints(boardPoints, rightRvec, rightT,
                          rig.right.intrinsics.cameraMatrix,
                          rig.right.intrinsics.distCoeffs,
                          sample.right.corners);
        for (cv::Point2f& point : sample.left.corners) {
            point.x += static_cast<float>(noise.gaussian(0.03));
            point.y += static_cast<float>(noise.gaussian(0.03));
        }
        for (cv::Point2f& point : sample.right.corners) {
            point.x += static_cast<float>(noise.gaussian(0.03));
            point.y += static_cast<float>(noise.gaussian(0.03));
        }
        samples.push_back(std::move(sample));
    }
    hik_stereo::StereoCalibrationResult result;
    hik_stereo::StereoCalibrationOptions options;
    CHECK(hik_stereo::calibrateStereoFixedIntrinsics(
              samples, rig, options, &result), result.error);
    CHECK(result.passed, "low-noise synthetic stereo fit must pass quality gates");
    CHECK(std::abs(result.baselineMm - 100.0) < 0.5,
          "stereo calibration must recover the known baseline");
}

void testOccupancyExport() {
    hik_stereo::VoxelOccupancyMap map;
    hik_stereo::OccupancyMapOptions options;
    options.voxelSizeMm = 20.0;
    options.pixelStride = 1;
    std::string error;
    CHECK(map.configure(options, &error), error);
    cv::Mat xyz(10, 10, CV_32FC3);
    cv::Mat valid(10, 10, CV_8UC1, cv::Scalar(255));
    for (int y = 0; y < xyz.rows; ++y) {
        cv::Vec3f* row = xyz.ptr<cv::Vec3f>(y);
        for (int x = 0; x < xyz.cols; ++x) {
            row[x] = cv::Vec3f(
                static_cast<float>((x - 5) * 10),
                static_cast<float>((y - 5) * 10), 500.0F);
        }
    }
    CHECK(map.integrate(xyz, valid, cv::Matx44d::eye(), &error), error);
    CHECK(map.statistics().occupiedVoxels > 0U,
          "plane endpoints must allocate occupied voxels");
    const std::string prefix = "/tmp/hik_stereo_core_test_" +
                               std::to_string(std::rand());
    const std::string ply = prefix + ".ply";
    const std::string pgm = prefix + ".pgm";
    const std::string yaml = prefix + ".yaml";
    CHECK(map.saveOccupiedPly(ply, &error), error);
    CHECK(map.save2DGrid(pgm, yaml, 450.0, 550.0, &error), error);
    std::ifstream plyInput(ply.c_str());
    std::ifstream gridInput(pgm.c_str(), std::ios::binary);
    CHECK(plyInput.good() && gridInput.good(),
          "occupancy exporters must create readable artifacts");
    std::remove(ply.c_str());
    std::remove(pgm.c_str());
    std::remove(yaml.c_str());
}

}  // namespace

int main() {
    testFramePairing();
    testPoseSynchronization();
    testSyntheticDepth();
    testSyntheticMultiBandDepth();
    testInstalledRigConfiguration();
    testFixedIntrinsicStereoCalibration();
    testOccupancyExport();
    if (failures != 0) {
        std::cerr << failures << " stereo-core checks failed\n";
        return 1;
    }
    std::cout << "stereo core checks passed\n";
    return 0;
}
