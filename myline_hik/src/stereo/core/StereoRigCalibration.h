#ifndef MYLINE_HIK_STEREO_RIG_CALIBRATION_H
#define MYLINE_HIK_STEREO_RIG_CALIBRATION_H

#include "HikCalibrationCore.h"
#include "HikScanCore.h"

#include <opencv2/core.hpp>

#include <string>

namespace hik_stereo {

struct StereoCameraCalibration {
    hik_calibration::IntrinsicCalibrationResult intrinsics;
    hik_calibration::IntrinsicsYamlMetadata metadata;
    hik_scan::HandEyeFile handEye;
    std::string intrinsicsPath;
    std::string handEyePath;
};

// OpenCV stereo geometry uses X_right = R * X_left + T. All translations are
// millimetres, matching the hand-eye and robot transforms in this project.
struct StereoRigCalibration {
    bool ok{false};
    std::string error;
    StereoCameraCalibration left;
    StereoCameraCalibration right;
    cv::Matx44d rightFromLeft{cv::Matx44d::eye()};
    cv::Mat rotationRightFromLeft;
    cv::Mat translationRightFromLeft;
    double baselineMm{0.0};
    double relativeRotationDeg{0.0};
    bool derivedFromIndependentHandEye{true};
};

struct StereoProcessingGeometry {
    bool ok{false};
    std::string error;
    cv::Size outputSize;
    cv::Rect leftCrop;
    cv::Rect rightCrop;
    cv::Mat leftCameraMatrix;
    cv::Mat rightCameraMatrix;
};

bool loadStereoRigFromHandEye(
    const std::string& leftIntrinsicsPath,
    const std::string& leftHandEyePath,
    const std::string& rightIntrinsicsPath,
    const std::string& rightHandEyePath,
    StereoRigCalibration* rig,
    std::string* error = nullptr);

// Loads the same formal intrinsics/hand-eye files for camera identity and
// base-frame mapping, but replaces the relative R/T with a dedicated
// synchronized stereoCalibrate result. Mapping must prefer this path.
bool loadStereoRigFromStereoYaml(
    const std::string& stereoYamlPath,
    const std::string& leftIntrinsicsPath,
    const std::string& leftHandEyePath,
    const std::string& rightIntrinsicsPath,
    const std::string& rightHandEyePath,
    StereoRigCalibration* rig,
    std::string* error = nullptr);

bool validateStereoRig(const StereoRigCalibration& rig,
                       std::string* error = nullptr);

bool prepareStereoProcessingGeometry(
    const StereoRigCalibration& rig,
    const cv::Size& outputSize,
    StereoProcessingGeometry* geometry,
    std::string* error = nullptr);

bool cropAndResizeStereoImage(const cv::Mat& source,
                              const cv::Rect& crop,
                              const cv::Size& outputSize,
                              cv::Mat* output,
                              std::string* error = nullptr);

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_RIG_CALIBRATION_H
