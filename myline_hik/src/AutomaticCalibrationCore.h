#ifndef MYLINE_HIK_AUTOMATIC_CALIBRATION_CORE_H
#define MYLINE_HIK_AUTOMATIC_CALIBRATION_CORE_H

#include "HandEyeCalibrationCore.h"
#include "HikScanCore.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace hik_calibration {

// Deterministic stop-and-shoot trajectory around a stationary ChArUco board.
// A previously approved calibration is used only as a motion-planning seed;
// none of its numeric values are copied into the solved result.
struct AutomaticCalibrationPlanOptions {
    double nearDepthMm;
    double middleDepthMm;
    double farDepthMm;
    double maximumTranslationFromStartMm;
    double maximumRotationFromStartDeg;
    double speedMmS;
    double accelerationMmS2;

    AutomaticCalibrationPlanOptions();
};

struct AutomaticCalibrationTarget {
    int index;
    bool holdout;
    double requestedBoardDepthMm;
    double requestedBoardCenterXmm;
    double requestedBoardCenterYmm;
    double requestedTiltXDeg;
    double requestedTiltYDeg;
    double requestedRollDeg;
    cv::Matx44d cameraFromBoard;
    hik_scan::Pose6D baseFromFlangePose;

    AutomaticCalibrationTarget();
};

struct AutomaticCalibrationPlan {
    bool ok;
    std::string error;
    cv::Matx44d baseFromBoardSeed;
    hik_scan::Pose6D homePose;
    std::vector<AutomaticCalibrationTarget> targets;
    int trainingTargetCount;
    int holdoutTargetCount;
    double maximumTranslationFromStartMm;
    double maximumRotationFromStartDeg;

    AutomaticCalibrationPlan();
};

bool poseFromFairinoTransform(const cv::Matx44d& baseFromFlange,
                              hik_scan::Pose6D* pose,
                              std::string* error = nullptr);

bool buildAutomaticCalibrationPlan(
    const cv::Matx44d& baseFromFlangeStart,
    const cv::Matx44d& flangeFromCameraSeed,
    const cv::Matx44d& cameraFromBoardSeed,
    const BoardSpec& board,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    const cv::Size& imageSize,
    const AutomaticCalibrationPlanOptions& options,
    AutomaticCalibrationPlan* plan);

struct AutomaticHoldoutSample {
    std::string sampleId;
    cv::Matx44d baseFromFlange;
    cv::Matx44d cameraFromBoard;
    double reprojectionRmsPx;

    AutomaticHoldoutSample();
};

struct AutomaticCalibrationValidation {
    bool ok;
    bool passed;
    std::string error;
    int sampleCount;
    ErrorMetrics reprojectionPx;
    ErrorMetrics baseBoardTranslationMm;
    ErrorMetrics baseBoardRotationDeg;

    AutomaticCalibrationValidation();
};

// Validates newly solved intrinsics/hand-eye on samples that were not supplied
// to either solver. baseFromBoardReference normally comes from the training
// hand-eye result.
bool validateAutomaticCalibrationHoldout(
    const std::vector<AutomaticHoldoutSample>& samples,
    const cv::Matx44d& flangeFromCamera,
    const cv::Matx44d& baseFromBoardReference,
    double maximumReprojectionRmsPx,
    double maximumTranslationRmsMm,
    double maximumRotationRmsDeg,
    AutomaticCalibrationValidation* validation);

struct DualCameraExtrinsics {
    bool ok;
    std::string error;
    cv::Matx44d firstCameraFromSecondCamera;
    double baselineMm;
    double relativeRotationDeg;

    DualCameraExtrinsics();
};

// Both input transforms map their camera optical coordinates into the same
// reported FR5 flange frame. Output maps second-camera coordinates into the
// first camera's optical frame.
bool computeDualCameraExtrinsics(
    const cv::Matx44d& flangeFromFirstCamera,
    const cv::Matx44d& flangeFromSecondCamera,
    DualCameraExtrinsics* extrinsics);

struct DualCameraYamlMetadata {
    std::string firstCameraFrame;
    std::string secondCameraFrame;
    std::string firstHandEyeFile;
    std::string firstHandEyeSha256;
    std::string secondHandEyeFile;
    std::string secondHandEyeSha256;
    std::string generatedAt;
};

bool saveDualCameraExtrinsicsYaml(
    const std::string& path,
    const DualCameraExtrinsics& extrinsics,
    const DualCameraYamlMetadata& metadata,
    std::string* error = nullptr);

}  // namespace hik_calibration

#endif  // MYLINE_HIK_AUTOMATIC_CALIBRATION_CORE_H
