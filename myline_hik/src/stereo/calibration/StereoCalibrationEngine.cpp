#include "stereo/calibration/StereoCalibrationEngine.h"

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>

namespace hik_stereo {
namespace {

void setError(const std::string& value, std::string* error) {
    if (error) *error = value;
}

bool commonPoints(const StereoCalibrationSample& sample,
                  const hik_calibration::BoardSpec& spec,
                  std::vector<cv::Point3f>* object,
                  std::vector<cv::Point2f>* left,
                  std::vector<cv::Point2f>* right) {
    if (!object || !left || !right ||
        sample.left.ids.size() != sample.left.corners.size() ||
        sample.right.ids.size() != sample.right.corners.size()) return false;
    std::map<int, cv::Point2f> leftById;
    for (std::size_t index = 0; index < sample.left.ids.size(); ++index)
        leftById[sample.left.ids[index]] = sample.left.corners[index];
    object->clear();
    left->clear();
    right->clear();
    const int columns = spec.squaresX - 1;
    const int rows = spec.squaresY - 1;
    for (std::size_t index = 0; index < sample.right.ids.size(); ++index) {
        const int id = sample.right.ids[index];
        const auto found = leftById.find(id);
        if (found == leftById.end() || id < 0 || id >= columns * rows) continue;
        const int y = id / columns;
        const int x = id % columns;
        object->push_back(cv::Point3f(
            static_cast<float>((x + 1) * spec.squareLengthMm),
            static_cast<float>((y + 1) * spec.squareLengthMm), 0.0F));
        left->push_back(found->second);
        right->push_back(sample.right.corners[index]);
    }
    return object->size() >= 4U;
}

double epipolarRms(const std::vector<cv::Point2f>& left,
                   const std::vector<cv::Point2f>& right,
                   const cv::Mat& fundamental,
                   const hik_calibration::IntrinsicCalibrationResult&
                       leftCalibration,
                   const hik_calibration::IntrinsicCalibrationResult&
                       rightCalibration) {
    if (left.size() != right.size() || left.empty() ||
        fundamental.rows != 3 || fundamental.cols != 3) return 1.0e9;
    std::vector<cv::Point2f> leftUndistorted;
    std::vector<cv::Point2f> rightUndistorted;
    try {
        // F describes the ideal pinhole image planes. Applying it directly to
        // raw distorted pixels creates a large false residual, especially for
        // the short-lens right camera. Keep pixel units by projecting the
        // undistorted normalized points back through the same K matrices.
        cv::undistortPoints(
            left, leftUndistorted,
            leftCalibration.cameraMatrix,
            leftCalibration.distCoeffs,
            cv::noArray(), leftCalibration.cameraMatrix);
        cv::undistortPoints(
            right, rightUndistorted,
            rightCalibration.cameraMatrix,
            rightCalibration.distCoeffs,
            cv::noArray(), rightCalibration.cameraMatrix);
    } catch (const cv::Exception&) {
        return 1.0e9;
    }
    if (leftUndistorted.size() != left.size() ||
        rightUndistorted.size() != right.size()) return 1.0e9;
    cv::Mat f;
    fundamental.convertTo(f, CV_64F);
    double squared = 0.0;
    std::size_t count = 0U;
    for (std::size_t index = 0; index < left.size(); ++index) {
        const cv::Vec3d x1(
            leftUndistorted[index].x, leftUndistorted[index].y, 1.0);
        const cv::Vec3d x2(
            rightUndistorted[index].x, rightUndistorted[index].y, 1.0);
        cv::Vec3d line2;
        cv::Vec3d line1;
        for (int row = 0; row < 3; ++row) {
            line2[row] = f.at<double>(row, 0) * x1[0] +
                         f.at<double>(row, 1) * x1[1] +
                         f.at<double>(row, 2);
            line1[row] = f.at<double>(0, row) * x2[0] +
                         f.at<double>(1, row) * x2[1] +
                         f.at<double>(2, row);
        }
        const double numerator = x2.dot(line2);
        const double denominator1 = line1[0] * line1[0] +
                                    line1[1] * line1[1];
        const double denominator2 = line2[0] * line2[0] +
                                    line2[1] * line2[1];
        if (denominator1 <= 1.0e-12 || denominator2 <= 1.0e-12) continue;
        squared += 0.5 * numerator * numerator *
                   (1.0 / denominator1 + 1.0 / denominator2);
        ++count;
    }
    return count > 0U ? std::sqrt(squared / count) : 1.0e9;
}

bool fit(const std::vector<std::vector<cv::Point3f>>& object,
         const std::vector<std::vector<cv::Point2f>>& left,
         const std::vector<std::vector<cv::Point2f>>& right,
         const StereoRigCalibration& rig,
         double* rms,
         cv::Mat* rotation,
         cv::Mat* translation,
         cv::Mat* essential,
         cv::Mat* fundamental,
         std::string* error) {
    if (object.empty() || object.size() != left.size() ||
        object.size() != right.size()) return false;
    cv::Mat leftK = rig.left.intrinsics.cameraMatrix.clone();
    cv::Mat leftD = rig.left.intrinsics.distCoeffs.clone();
    cv::Mat rightK = rig.right.intrinsics.cameraMatrix.clone();
    cv::Mat rightD = rig.right.intrinsics.distCoeffs.clone();
    try {
        *rms = cv::stereoCalibrate(
            object, left, right,
            leftK, leftD, rightK, rightD,
            rig.left.intrinsics.imageSize,
            *rotation, *translation, *essential, *fundamental,
            cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT |
                             cv::TermCriteria::EPS, 200, 1.0e-10));
    } catch (const cv::Exception& exception) {
        setError(std::string("stereoCalibrate failed: ") + exception.what(), error);
        return false;
    }
    return std::isfinite(*rms) && !rotation->empty() &&
           !translation->empty() && cv::checkRange(*rotation) &&
           cv::checkRange(*translation);
}

void writeArray(std::ostream& output, const cv::Mat& matrix) {
    cv::Mat values;
    matrix.reshape(1, 1).convertTo(values, CV_64F);
    output << '[';
    for (int column = 0; column < values.cols; ++column) {
        if (column != 0) output << ", ";
        output << values.at<double>(0, column);
    }
    output << ']';
}

}  // namespace

bool detectStereoCharucoSample(
        const cv::Mat& leftGray,
        const cv::Mat& rightGray,
        const std::string& sampleId,
        const StereoRigCalibration& rig,
        StereoCalibrationSample* sample,
        std::string* error) {
    if (!sample || leftGray.empty() || rightGray.empty()) {
        setError("stereo ChArUco input is empty", error);
        return false;
    }
    hik_calibration::DetectionOptions options;
    options.minMarkers = 6;
    options.minCorners = 12;
    options.minHullAreaRatio = 0.015;
    options.minBorderDistancePx = 5.0;
    options.minLaplacianVariance = 15.0;
    hik_calibration::CharucoDetectionResult leftDetection;
    hik_calibration::CharucoDetectionResult rightDetection;
    if (!hik_calibration::detectCharuco(
            leftGray, sampleId + "_left", rig.left.intrinsics.board,
            options, &leftDetection, rig.left.intrinsics.cameraMatrix,
            rig.left.intrinsics.distCoeffs) ||
        !hik_calibration::detectCharuco(
            rightGray, sampleId + "_right", rig.right.intrinsics.board,
            options, &rightDetection, rig.right.intrinsics.cameraMatrix,
            rig.right.intrinsics.distCoeffs)) {
        setError("ChArUco board was not accepted in both cameras: left=" +
            leftDetection.observation.quality.rejectReason + "; right=" +
            rightDetection.observation.quality.rejectReason, error);
        return false;
    }
    sample->id = sampleId;
    sample->left = std::move(leftDetection.observation);
    sample->right = std::move(rightDetection.observation);
    std::vector<cv::Point3f> object;
    std::vector<cv::Point2f> left;
    std::vector<cv::Point2f> right;
    if (!commonPoints(*sample, rig.left.intrinsics.board,
                      &object, &left, &right) || object.size() < 12U) {
        setError("fewer than 12 common ChArUco corners in the stereo pair", error);
        return false;
    }
    return true;
}

bool calibrateStereoFixedIntrinsics(
        const std::vector<StereoCalibrationSample>& samples,
        const StereoRigCalibration& rig,
        const StereoCalibrationOptions& options,
        StereoCalibrationResult* result) {
    if (!result) return false;
    *result = StereoCalibrationResult();
    result->inputSamples = static_cast<int>(samples.size());
    std::string rigError;
    if (!validateStereoRig(rig, &rigError) ||
        options.minimumSamples < 8 ||
        options.minimumCommonCorners < 6 ||
        !std::isfinite(options.maximumStereoRmsPx) ||
        !std::isfinite(options.maximumEpipolarRmsPx) ||
        !std::isfinite(options.hardMaximumViewEpipolarRmsPx)) {
        result->error = rigError.empty() ?
            "stereo calibration options are invalid" : rigError;
        return false;
    }
    std::vector<std::vector<cv::Point3f>> object;
    std::vector<std::vector<cv::Point2f>> left;
    std::vector<std::vector<cv::Point2f>> right;
    std::vector<std::size_t> sourceIndices;
    result->samples.resize(samples.size());
    for (std::size_t index = 0; index < samples.size(); ++index) {
        result->samples[index].id = samples[index].id;
        std::vector<cv::Point3f> objectView;
        std::vector<cv::Point2f> leftView;
        std::vector<cv::Point2f> rightView;
        if (!commonPoints(samples[index], rig.left.intrinsics.board,
                          &objectView, &leftView, &rightView) ||
            static_cast<int>(objectView.size()) < options.minimumCommonCorners) {
            result->samples[index].reason = "too few common ChArUco corners";
            continue;
        }
        result->samples[index].commonCornerCount =
            static_cast<int>(objectView.size());
        object.push_back(std::move(objectView));
        left.push_back(std::move(leftView));
        right.push_back(std::move(rightView));
        sourceIndices.push_back(index);
    }
    if (static_cast<int>(object.size()) < options.minimumSamples) {
        result->error = "too few valid synchronized stereo calibration pairs";
        result->rejectedSamples = result->inputSamples;
        return false;
    }
    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat essential;
    cv::Mat fundamental;
    double rms = 0.0;
    if (!fit(object, left, right, rig, &rms, &rotation, &translation,
             &essential, &fundamental, &result->error)) return false;

    std::vector<std::vector<cv::Point3f>> acceptedObject;
    std::vector<std::vector<cv::Point2f>> acceptedLeft;
    std::vector<std::vector<cv::Point2f>> acceptedRight;
    std::vector<std::size_t> acceptedIndices;
    for (std::size_t view = 0; view < object.size(); ++view) {
        const double viewRms = epipolarRms(
            left[view], right[view], fundamental,
            rig.left.intrinsics, rig.right.intrinsics);
        StereoCalibrationSampleResult& sampleResult =
            result->samples[sourceIndices[view]];
        sampleResult.epipolarRmsPx = viewRms;
        if (viewRms <= options.hardMaximumViewEpipolarRmsPx) {
            sampleResult.accepted = true;
            acceptedObject.push_back(object[view]);
            acceptedLeft.push_back(left[view]);
            acceptedRight.push_back(right[view]);
            acceptedIndices.push_back(sourceIndices[view]);
        } else {
            sampleResult.reason = "epipolar RMS exceeds per-view hard limit";
        }
    }
    if (static_cast<int>(acceptedObject.size()) < options.minimumSamples) {
        result->error = "epipolar outlier rejection left too few stereo pairs";
        result->rejectedSamples = result->inputSamples;
        return false;
    }
    if (acceptedObject.size() != object.size() &&
        !fit(acceptedObject, acceptedLeft, acceptedRight, rig,
             &rms, &rotation, &translation, &essential,
             &fundamental, &result->error)) return false;
    double epipolarSquared = 0.0;
    for (std::size_t view = 0; view < acceptedObject.size(); ++view) {
        const double value = epipolarRms(
            acceptedLeft[view], acceptedRight[view], fundamental,
            rig.left.intrinsics, rig.right.intrinsics);
        result->samples[acceptedIndices[view]].epipolarRmsPx = value;
        epipolarSquared += value * value;
    }
    result->stereoRmsPx = rms;
    result->epipolarRmsPx = std::sqrt(
        epipolarSquared / acceptedObject.size());
    result->rotationRightFromLeft = rotation;
    result->translationRightFromLeft = translation;
    result->essentialMatrix = essential;
    result->fundamentalMatrix = fundamental;
    result->baselineMm = cv::norm(translation);
    const double trace = cv::trace(rotation)[0];
    result->relativeRotationDeg = std::acos(std::max(
        -1.0, std::min(1.0, (trace - 1.0) * 0.5))) * 180.0 / CV_PI;
    result->acceptedSamples = static_cast<int>(acceptedObject.size());
    result->rejectedSamples = result->inputSamples - result->acceptedSamples;
    result->passed = result->stereoRmsPx <= options.maximumStereoRmsPx &&
        result->epipolarRmsPx <= options.maximumEpipolarRmsPx &&
        result->baselineMm >= 20.0 && result->baselineMm <= 1000.0 &&
        result->relativeRotationDeg <= 60.0;
    result->ok = true;
    return true;
}

bool saveStereoCalibrationYaml(
        const std::string& path,
        const StereoCalibrationResult& result,
        const StereoRigCalibration& rig,
        const std::string& generatedAt,
        std::string* error) {
    if (!result.ok || !result.passed ||
        result.rotationRightFromLeft.empty() ||
        result.translationRightFromLeft.empty()) {
        setError("only a passed stereo calibration may be saved", error);
        return false;
    }
    std::ofstream output(path.c_str());
    if (!output) {
        setError("cannot open stereo calibration YAML: " + path, error);
        return false;
    }
    output << std::setprecision(15)
           << "schema_version: 1\n"
           << "calibration_type: stereo_rig\n"
           << "transform_convention: \"X_right = R_right_left * X_left + T_right_left_mm\"\n"
           << "translation_unit: mm\n"
           << "left_camera_serial: \"" << rig.left.metadata.cameraSerial << "\"\n"
           << "right_camera_serial: \"" << rig.right.metadata.cameraSerial << "\"\n"
           << "left_frame: \"" << rig.left.metadata.frameId << "\"\n"
           << "right_frame: \"" << rig.right.metadata.frameId << "\"\n"
           << "R_right_left: ";
    writeArray(output, result.rotationRightFromLeft);
    output << "\nT_right_left_mm: ";
    writeArray(output, result.translationRightFromLeft);
    output << "\nbaseline_mm: " << result.baselineMm
           << "\nrelative_rotation_deg: " << result.relativeRotationDeg
           << "\nquality:\n"
           << "  input_pairs: " << result.inputSamples << '\n'
           << "  accepted_pairs: " << result.acceptedSamples << '\n'
           << "  rejected_pairs: " << result.rejectedSamples << '\n'
           << "  stereo_rms_px: " << result.stereoRmsPx << '\n'
           << "  epipolar_rms_px: " << result.epipolarRmsPx << '\n'
           << "sources:\n"
           << "  left_intrinsics: \"" << rig.left.intrinsicsPath << "\"\n"
           << "  right_intrinsics: \"" << rig.right.intrinsicsPath << "\"\n"
           << "generated_at: \"" << generatedAt << "\"\n";
    if (!output) {
        setError("failed while writing stereo calibration YAML: " + path, error);
        return false;
    }
    return true;
}

}  // namespace hik_stereo
