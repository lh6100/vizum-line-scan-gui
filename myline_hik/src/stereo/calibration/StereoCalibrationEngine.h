#ifndef MYLINE_HIK_STEREO_CALIBRATION_ENGINE_H
#define MYLINE_HIK_STEREO_CALIBRATION_ENGINE_H

#include "stereo/core/StereoRigCalibration.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace hik_stereo {

struct StereoCalibrationSample {
    std::string id;
    hik_calibration::CharucoObservation left;
    hik_calibration::CharucoObservation right;
};

struct StereoCalibrationSampleResult {
    std::string id;
    bool accepted{false};
    int commonCornerCount{0};
    double epipolarRmsPx{0.0};
    std::string reason;
};

struct StereoCalibrationOptions {
    int minimumSamples{15};
    int minimumCommonCorners{12};
    double maximumStereoRmsPx{0.6};
    double maximumEpipolarRmsPx{0.5};
    double hardMaximumViewEpipolarRmsPx{1.0};
};

struct StereoCalibrationResult {
    bool ok{false};
    bool passed{false};
    std::string error;
    int inputSamples{0};
    int acceptedSamples{0};
    int rejectedSamples{0};
    double stereoRmsPx{0.0};
    double epipolarRmsPx{0.0};
    cv::Mat rotationRightFromLeft;
    cv::Mat translationRightFromLeft;
    cv::Mat essentialMatrix;
    cv::Mat fundamentalMatrix;
    double baselineMm{0.0};
    double relativeRotationDeg{0.0};
    std::vector<StereoCalibrationSampleResult> samples;
};

bool detectStereoCharucoSample(
    const cv::Mat& leftGray,
    const cv::Mat& rightGray,
    const std::string& sampleId,
    const StereoRigCalibration& rig,
    StereoCalibrationSample* sample,
    std::string* error = nullptr);

bool calibrateStereoFixedIntrinsics(
    const std::vector<StereoCalibrationSample>& samples,
    const StereoRigCalibration& rig,
    const StereoCalibrationOptions& options,
    StereoCalibrationResult* result);

bool saveStereoCalibrationYaml(
    const std::string& path,
    const StereoCalibrationResult& result,
    const StereoRigCalibration& rig,
    const std::string& generatedAt,
    std::string* error = nullptr);

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_CALIBRATION_ENGINE_H
