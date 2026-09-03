#include "AutomaticCalibrationCore.h"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>

namespace {

int failures = 0;

#define CHECK(condition, message) do { \
    if (!(condition)) { \
        ++failures; \
        std::cerr << "FAIL line " << __LINE__ << ": " << message << '\n'; \
    } \
} while (false)

void testPlan() {
    using namespace hik_calibration;
    const BoardSpec board;
    const cv::Matx44d baseFromFlange = fairinoBaseFromFlange(
        400.0, -100.0, 500.0, 170.0, 5.0, 30.0);
    const cv::Matx44d flangeFromCamera = fairinoBaseFromFlange(
        65.0, 55.0, 70.0, 10.0, 1.0, 179.0);
    cv::Matx44d cameraFromBoard = cv::Matx44d::eye();
    cameraFromBoard(0, 3) = -board.widthMm() * 0.5;
    cameraFromBoard(1, 3) = -board.heightMm() * 0.5;
    cameraFromBoard(2, 3) = 550.0;
    AutomaticCalibrationPlanOptions options;
    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        900.0, 0.0, 640.0,
        0.0, 900.0, 512.0,
        0.0, 0.0, 1.0);
    const cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    AutomaticCalibrationPlan plan;
    CHECK(buildAutomaticCalibrationPlan(
              baseFromFlange, flangeFromCamera, cameraFromBoard,
              board, cameraMatrix, distCoeffs, cv::Size(1280, 1024),
              options, &plan), plan.error);
    CHECK(plan.ok, "plan should be valid");
    CHECK(plan.targets.size() == 36U, "plan must contain 36 targets");
    CHECK(plan.trainingTargetCount == 30, "plan must contain 30 training targets");
    CHECK(plan.holdoutTargetCount == 6, "plan must contain 6 holdout targets");
    int nearTraining = 0;
    int middleTraining = 0;
    int farTraining = 0;
    for (const AutomaticCalibrationTarget& target : plan.targets) {
        if (!target.holdout) {
            if (target.requestedBoardDepthMm < 500.0) ++nearTraining;
            else if (target.requestedBoardDepthMm < 600.0) ++middleTraining;
            else ++farTraining;
        }
        const cv::Matx44d recovered = fairinoBaseFromFlange(
            target.baseFromFlangePose.x, target.baseFromFlangePose.y,
            target.baseFromFlangePose.z, target.baseFromFlangePose.rx,
            target.baseFromFlangePose.ry, target.baseFromFlangePose.rz);
        CHECK(rigidTranslationDistanceMm(
                  recovered,
                  plan.baseFromBoardSeed * target.cameraFromBoard.inv() *
                      flangeFromCamera.inv()) < 1.0e-6,
              "pose conversion must preserve translation");
    }
    CHECK(nearTraining == 10 && middleTraining == 10 && farTraining == 10,
          "each depth bin must contain 10 training targets");
}

void testHoldoutValidation() {
    using namespace hik_calibration;
    const cv::Matx44d flangeFromCamera = fairinoBaseFromFlange(
        60.0, 20.0, 70.0, 0.0, 0.0, 180.0);
    const cv::Matx44d baseFromBoard = fairinoBaseFromFlange(
        700.0, -40.0, 320.0, 0.0, 0.0, 20.0);
    std::vector<AutomaticHoldoutSample> samples;
    for (int index = 0; index < 6; ++index) {
        AutomaticHoldoutSample sample;
        sample.sampleId = "holdout_" + std::to_string(index);
        sample.baseFromFlange = fairinoBaseFromFlange(
            350.0 + 25.0 * index, -100.0 + 8.0 * index,
            520.0, 170.0, -10.0 + 4.0 * index, 30.0);
        sample.cameraFromBoard =
            (sample.baseFromFlange * flangeFromCamera).inv() * baseFromBoard;
        sample.reprojectionRmsPx = 0.20 + 0.01 * index;
        samples.push_back(sample);
    }
    AutomaticCalibrationValidation validation;
    CHECK(validateAutomaticCalibrationHoldout(
              samples, flangeFromCamera, baseFromBoard,
              0.40, 0.50, 0.20, &validation), validation.error);
    CHECK(validation.ok && validation.passed,
          "perfect synthetic holdout must pass");
    CHECK(validation.baseBoardTranslationMm.rms < 1.0e-8,
          "synthetic holdout translation residual must be zero");
}

void testNarrowCameraBoardVisibility() {
    using namespace hik_calibration;
    const BoardSpec board;
    cv::Matx44d cameraFromBoard = cv::Matx44d::eye();
    cameraFromBoard(0, 3) = -board.widthMm() * 0.5;
    cameraFromBoard(1, 3) = -board.heightMm() * 0.5;
    cameraFromBoard(2, 3) = 550.0;
    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        1882.0410290210543, 0.0, 626.66386213737394,
        0.0, 1881.5523132060634, 524.96634567464855,
        0.0, 0.0, 1.0);
    const cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.38125905388470721, 0.28759554806034426,
        -0.00066657910097410174, 0.00068341947992974666,
        0.1068470753985815);
    AutomaticCalibrationPlan plan;
    AutomaticCalibrationPlanOptions options;
    CHECK(buildAutomaticCalibrationPlan(
              cv::Matx44d::eye(), cv::Matx44d::eye(), cameraFromBoard,
              board, cameraMatrix, distCoeffs, cv::Size(1224, 1024),
              options, &plan), plan.error);
    int diverseNearTargets = 0;
    const std::vector<cv::Point3f> corners = {
        cv::Point3f(0.0F, 0.0F, 0.0F),
        cv::Point3f(264.0F, 0.0F, 0.0F),
        cv::Point3f(264.0F, 192.0F, 0.0F),
        cv::Point3f(0.0F, 192.0F, 0.0F)
    };
    for (const AutomaticCalibrationTarget& target : plan.targets) {
        cv::Mat rotation(3, 3, CV_64F);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                rotation.at<double>(row, column) =
                    target.cameraFromBoard(row, column);
            }
        }
        cv::Vec3d rvec;
        cv::Rodrigues(rotation, rvec);
        const cv::Vec3d tvec(
            target.cameraFromBoard(0, 3), target.cameraFromBoard(1, 3),
            target.cameraFromBoard(2, 3));
        std::vector<cv::Point2f> pixels;
        cv::projectPoints(corners, rvec, tvec,
                          cameraMatrix, distCoeffs, pixels);
        for (const cv::Point2f& pixel : pixels) {
            CHECK(pixel.x >= 12.0F && pixel.x <= 1211.0F &&
                      pixel.y >= 12.0F && pixel.y <= 1011.0F,
                  "every planned physical board corner must stay visible");
        }
        if (target.requestedBoardDepthMm < 500.0 &&
            (std::abs(target.requestedBoardCenterXmm) > 1.0 ||
             std::abs(target.requestedBoardCenterYmm) > 1.0 ||
             std::abs(target.requestedTiltXDeg) > 1.0 ||
             std::abs(target.requestedTiltYDeg) > 1.0)) {
            ++diverseNearTargets;
        }
    }
    CHECK(diverseNearTargets >= 6,
          "narrow-camera near targets must retain useful pose diversity");
}

void testDualExtrinsicsYaml() {
    using namespace hik_calibration;
    const cv::Matx44d flangeFromFirst = fairinoBaseFromFlange(
        60.0, 40.0, 70.0, 0.0, 0.0, 180.0);
    const cv::Matx44d flangeFromSecond = fairinoBaseFromFlange(
        -60.0, 40.0, 70.0, 0.0, 0.0, 0.0);
    DualCameraExtrinsics result;
    CHECK(computeDualCameraExtrinsics(
              flangeFromFirst, flangeFromSecond, &result), result.error);
    CHECK(result.ok && std::abs(result.baselineMm - 120.0) < 1.0e-8,
          "dual-camera baseline must be 120 mm");
    CHECK(std::abs(result.relativeRotationDeg - 180.0) < 1.0e-8,
          "opposing cameras must have 180 degree relative rotation");
    DualCameraYamlMetadata metadata;
    metadata.firstCameraFrame = "camera_a";
    metadata.secondCameraFrame = "camera_b";
    metadata.firstHandEyeFile = "a.yaml";
    metadata.firstHandEyeSha256 = "aaa";
    metadata.secondHandEyeFile = "b.yaml";
    metadata.secondHandEyeSha256 = "bbb";
    metadata.generatedAt = "2026-08-02T00:00:00.000Z";
    const std::string path = "/tmp/automatic_calibration_core_" +
                             std::to_string(::getpid()) + ".yaml";
    std::string error;
    CHECK(saveDualCameraExtrinsicsYaml(path, result, metadata, &error), error);
    std::remove(path.c_str());
}

}  // namespace

int main() {
    testPlan();
    testHoldoutValidation();
    testNarrowCameraBoardVisibility();
    testDualExtrinsicsYaml();
    if (failures != 0) {
        std::cerr << failures << " automatic calibration test(s) failed\n";
        return 1;
    }
    std::cout << "automatic calibration core tests passed\n";
    return 0;
}
