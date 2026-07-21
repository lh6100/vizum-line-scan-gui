#ifndef MYLINE_HIK_HAND_EYE_CALIBRATION_CORE_H
#define MYLINE_HIK_HAND_EYE_CALIBRATION_CORE_H

#include "HikCalibrationCore.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace hik_calibration {

// Fairino reports T_base_flange as XYZ millimetres and fixed-axis RPY degrees.
// The rotation convention is Rz(rz) * Ry(ry) * Rx(rx).
cv::Matx44d fairinoBaseFromFlange(double xMm,
                                  double yMm,
                                  double zMm,
                                  double rxDeg,
                                  double ryDeg,
                                  double rzDeg);

double rigidTranslationDistanceMm(const cv::Matx44d& first,
                                  const cv::Matx44d& second);
double rigidRotationDistanceDeg(const cv::Matx44d& first,
                                const cv::Matx44d& second);
cv::Matx44d interpolateRigidHalf(const cv::Matx44d& first,
                                 const cv::Matx44d& second);

struct HandEyeSample {
    std::string sampleId;
    cv::Matx44d baseFromFlange;
    cv::Matx44d cameraFromBoard;
    double boardPoseRmsPx;

    HandEyeSample();
};

struct HandEyeOptions {
    int minSamples;
    int maxOutlierRounds;
    double minTranslationSpanMm;
    double minRotationSpanDeg;
    double minSecondaryRotationSpreadDeg;
    double outlierTranslationFloorMm;
    double outlierRotationFloorDeg;
    double madScale;

    HandEyeOptions();
};

struct HandEyeSampleResult {
    std::string sampleId;
    bool accepted;
    std::string rejectReason;
    double baseBoardTranslationResidualMm;
    double baseBoardRotationResidualDeg;

    HandEyeSampleResult();
};

struct HandEyeCalibrationResult {
    bool ok;
    std::string error;
    std::string method;
    cv::Matx44d flangeFromCamera;
    cv::Matx44d baseFromBoardMean;
    int inputSampleCount;
    int acceptedSampleCount;
    int rejectedSampleCount;
    double translationSpanMm;
    double rotationSpanDeg;
    double secondaryRotationSpreadDeg;
    ErrorMetrics translationConsistencyMm;
    ErrorMetrics rotationConsistencyDeg;
    std::vector<HandEyeSampleResult> samples;

    HandEyeCalibrationResult();
};

// Eye-in-hand solve. Inputs are T_base_flange and T_camera_board. Output is
// T_flange_camera, so no caller-side inversion is required.
bool calibrateHandEyeRobust(const std::vector<HandEyeSample>& samples,
                            const HandEyeOptions& options,
                            HandEyeCalibrationResult* result);

struct HandEyeYamlMetadata {
    std::string cameraModel;
    std::string cameraSerial;
    std::string cameraFrame;
    std::string flangeFrame;
    std::string baseFrame;
    std::string intrinsicsFile;
    std::string intrinsicsSha256;
    std::string datasetManifest;
    std::string datasetManifestSha256;
    std::string generatedAt;

    HandEyeYamlMetadata();
};

bool saveHandEyeYaml(const std::string& path,
                     const HandEyeCalibrationResult& calibration,
                     const HandEyeYamlMetadata& metadata,
                     std::string* error = 0);

}  // namespace hik_calibration

#endif  // MYLINE_HIK_HAND_EYE_CALIBRATION_CORE_H
