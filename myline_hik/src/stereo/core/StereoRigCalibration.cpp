#include "stereo/core/StereoRigCalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace hik_stereo {
namespace {

void setError(const std::string& value, std::string* error) {
    if (error) *error = value;
}

cv::Matx44d inverseRigid(const cv::Matx44d& transform) {
    cv::Matx33d rotation;
    cv::Vec3d translation;
    for (int row = 0; row < 3; ++row) {
        translation[row] = transform(row, 3);
        for (int column = 0; column < 3; ++column) {
            rotation(row, column) = transform(row, column);
        }
    }
    const cv::Matx33d inverseRotation = rotation.t();
    const cv::Vec3d inverseTranslation = -(inverseRotation * translation);
    cv::Matx44d result = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        result(row, 3) = inverseTranslation[row];
        for (int column = 0; column < 3; ++column) {
            result(row, column) = inverseRotation(row, column);
        }
    }
    return result;
}

bool finiteRigid(const cv::Matx44d& transform) {
    for (double value : transform.val) {
        if (!std::isfinite(value)) return false;
    }
    const cv::Matx33d rotation(
        transform(0, 0), transform(0, 1), transform(0, 2),
        transform(1, 0), transform(1, 1), transform(1, 2),
        transform(2, 0), transform(2, 1), transform(2, 2));
    return cv::norm(rotation.t() * rotation - cv::Matx33d::eye()) < 1.0e-4 &&
           std::abs(cv::determinant(cv::Mat(rotation)) - 1.0) < 1.0e-4 &&
           std::abs(transform(3, 0)) < 1.0e-9 &&
           std::abs(transform(3, 1)) < 1.0e-9 &&
           std::abs(transform(3, 2)) < 1.0e-9 &&
           std::abs(transform(3, 3) - 1.0) < 1.0e-9;
}

cv::Mat rotationMat(const cv::Matx44d& transform) {
    cv::Mat result(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.at<double>(row, column) = transform(row, column);
        }
    }
    return result;
}

cv::Mat translationMat(const cv::Matx44d& transform) {
    return (cv::Mat_<double>(3, 1) <<
        transform(0, 3), transform(1, 3), transform(2, 3));
}

std::string trim(const std::string& value) {
    const std::string whitespace(" \t\r\n");
    const std::string::size_type first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) return std::string();
    return value.substr(first, value.find_last_not_of(whitespace) - first + 1U);
}

std::string unquote(const std::string& value) {
    const std::string clean = trim(value);
    if (clean.size() >= 2U &&
        ((clean.front() == '"' && clean.back() == '"') ||
         (clean.front() == '\'' && clean.back() == '\''))) {
        return clean.substr(1U, clean.size() - 2U);
    }
    return clean;
}

bool parseList(const std::string& value,
               std::size_t expected,
               std::vector<double>* output) {
    if (!output) return false;
    const std::string clean = trim(value);
    if (clean.size() < 2U || clean.front() != '[' || clean.back() != ']')
        return false;
    output->clear();
    std::istringstream stream(clean.substr(1U, clean.size() - 2U));
    std::string item;
    while (std::getline(stream, item, ',')) {
        try {
            std::size_t consumed = 0U;
            const double parsed = std::stod(trim(item), &consumed);
            if (consumed != trim(item).size() || !std::isfinite(parsed))
                return false;
            output->push_back(parsed);
        } catch (...) {
            return false;
        }
    }
    return output->size() == expected;
}

cv::Rect centeredCrop(const cv::Size& source, const cv::Size& target) {
    const double targetAspect = static_cast<double>(target.width) /
                                static_cast<double>(target.height);
    const double sourceAspect = static_cast<double>(source.width) /
                                static_cast<double>(source.height);
    if (sourceAspect > targetAspect) {
        const int width = std::max(1, std::min(
            source.width,
            static_cast<int>(std::llround(source.height * targetAspect))));
        return cv::Rect((source.width - width) / 2, 0, width, source.height);
    }
    const int height = std::max(1, std::min(
        source.height,
        static_cast<int>(std::llround(source.width / targetAspect))));
    return cv::Rect(0, (source.height - height) / 2, source.width, height);
}

cv::Mat adjustedCameraMatrix(const cv::Mat& source,
                             const cv::Rect& crop,
                             const cv::Size& output) {
    cv::Mat result;
    source.convertTo(result, CV_64F);
    const double scaleX = static_cast<double>(output.width) / crop.width;
    const double scaleY = static_cast<double>(output.height) / crop.height;
    result.at<double>(0, 0) *= scaleX;
    result.at<double>(1, 1) *= scaleY;
    result.at<double>(0, 2) =
        (result.at<double>(0, 2) - crop.x) * scaleX;
    result.at<double>(1, 2) =
        (result.at<double>(1, 2) - crop.y) * scaleY;
    return result;
}

}  // namespace

bool loadStereoRigFromHandEye(
        const std::string& leftIntrinsicsPath,
        const std::string& leftHandEyePath,
        const std::string& rightIntrinsicsPath,
        const std::string& rightHandEyePath,
        StereoRigCalibration* rig,
        std::string* error) {
    if (!rig) {
        setError("stereo rig output is null", error);
        return false;
    }
    *rig = StereoRigCalibration();
    rig->left.intrinsicsPath = leftIntrinsicsPath;
    rig->left.handEyePath = leftHandEyePath;
    rig->right.intrinsicsPath = rightIntrinsicsPath;
    rig->right.handEyePath = rightHandEyePath;
    std::string detail;
    if (!hik_calibration::loadIntrinsicsYaml(
            leftIntrinsicsPath, &rig->left.intrinsics,
            &rig->left.metadata, &detail) ||
        !hik_scan::loadHandEyeYaml(
            leftHandEyePath, &rig->left.handEye, &detail) ||
        !hik_calibration::loadIntrinsicsYaml(
            rightIntrinsicsPath, &rig->right.intrinsics,
            &rig->right.metadata, &detail) ||
        !hik_scan::loadHandEyeYaml(
            rightHandEyePath, &rig->right.handEye, &detail)) {
        rig->error = detail;
        setError(detail, error);
        return false;
    }
    rig->rightFromLeft = inverseRigid(
        rig->right.handEye.flangeFromCamera) *
        rig->left.handEye.flangeFromCamera;
    rig->rotationRightFromLeft = rotationMat(rig->rightFromLeft);
    rig->translationRightFromLeft = translationMat(rig->rightFromLeft);
    rig->baselineMm = cv::norm(rig->translationRightFromLeft);
    const double trace = cv::trace(rig->rotationRightFromLeft)[0];
    rig->relativeRotationDeg = std::acos(std::max(
        -1.0, std::min(1.0, (trace - 1.0) * 0.5))) * 180.0 / CV_PI;
    if (!validateStereoRig(*rig, &detail)) {
        rig->error = detail;
        setError(detail, error);
        return false;
    }
    rig->ok = true;
    return true;
}

bool loadStereoRigFromStereoYaml(
        const std::string& stereoYamlPath,
        const std::string& leftIntrinsicsPath,
        const std::string& leftHandEyePath,
        const std::string& rightIntrinsicsPath,
        const std::string& rightHandEyePath,
        StereoRigCalibration* rig,
        std::string* error) {
    if (!loadStereoRigFromHandEye(
            leftIntrinsicsPath, leftHandEyePath,
            rightIntrinsicsPath, rightHandEyePath, rig, error)) {
        return false;
    }
    std::ifstream input(stereoYamlPath.c_str());
    if (!input) {
        rig->ok = false;
        rig->error = "cannot open dedicated stereo YAML: " + stereoYamlPath;
        setError(rig->error, error);
        return false;
    }
    int schemaVersion = 0;
    std::string calibrationType;
    std::string leftSerial;
    std::string rightSerial;
    std::vector<double> rotation;
    std::vector<double> translation;
    std::string line;
    while (std::getline(input, line)) {
        const std::string::size_type comment = line.find('#');
        if (comment != std::string::npos) line = line.substr(0, comment);
        const std::string::size_type colon = line.find(':');
        if (colon == std::string::npos) continue;
        const std::string key = trim(line.substr(0, colon));
        const std::string value = trim(line.substr(colon + 1U));
        if (key == "schema_version") {
            try { schemaVersion = std::stoi(value); } catch (...) { schemaVersion = 0; }
        } else if (key == "calibration_type") {
            calibrationType = unquote(value);
        } else if (key == "left_camera_serial") {
            leftSerial = unquote(value);
        } else if (key == "right_camera_serial") {
            rightSerial = unquote(value);
        } else if (key == "R_right_left") {
            (void)parseList(value, 9U, &rotation);
        } else if (key == "T_right_left_mm") {
            (void)parseList(value, 3U, &translation);
        }
    }
    const std::string expectedLeft = rig->left.metadata.cameraSerial;
    const std::string expectedRight = rig->right.metadata.cameraSerial;
    if (schemaVersion != 1 || calibrationType != "stereo_rig" ||
        rotation.size() != 9U || translation.size() != 3U ||
        leftSerial.empty() || rightSerial.empty() ||
        (!expectedLeft.empty() && leftSerial != expectedLeft) ||
        (!expectedRight.empty() && rightSerial != expectedRight)) {
        rig->ok = false;
        rig->error = "dedicated stereo YAML schema, matrix, or camera identity is invalid";
        setError(rig->error, error);
        return false;
    }
    rig->rightFromLeft = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        rig->rightFromLeft(row, 3) = translation[static_cast<std::size_t>(row)];
        for (int column = 0; column < 3; ++column) {
            rig->rightFromLeft(row, column) =
                rotation[static_cast<std::size_t>(row * 3 + column)];
        }
    }
    rig->rotationRightFromLeft = rotationMat(rig->rightFromLeft);
    rig->translationRightFromLeft = translationMat(rig->rightFromLeft);
    rig->baselineMm = cv::norm(rig->translationRightFromLeft);
    const double trace = cv::trace(rig->rotationRightFromLeft)[0];
    rig->relativeRotationDeg = std::acos(std::max(
        -1.0, std::min(1.0, (trace - 1.0) * 0.5))) * 180.0 / CV_PI;
    rig->derivedFromIndependentHandEye = false;
    std::string detail;
    if (!validateStereoRig(*rig, &detail)) {
        rig->ok = false;
        rig->error = detail;
        setError(detail, error);
        return false;
    }
    rig->ok = true;
    rig->error.clear();
    return true;
}

bool validateStereoRig(const StereoRigCalibration& rig,
                       std::string* error) {
    const auto validIntrinsics = [](const StereoCameraCalibration& camera) {
        return camera.intrinsics.ok &&
               camera.intrinsics.imageSize.width > 0 &&
               camera.intrinsics.imageSize.height > 0 &&
               camera.intrinsics.cameraMatrix.rows == 3 &&
               camera.intrinsics.cameraMatrix.cols == 3 &&
               !camera.intrinsics.distCoeffs.empty() &&
               cv::checkRange(camera.intrinsics.cameraMatrix) &&
               cv::checkRange(camera.intrinsics.distCoeffs) &&
               camera.handEye.ok &&
               finiteRigid(camera.handEye.flangeFromCamera);
    };
    if (!validIntrinsics(rig.left) || !validIntrinsics(rig.right) ||
        !finiteRigid(rig.rightFromLeft) ||
        rig.rotationRightFromLeft.rows != 3 ||
        rig.rotationRightFromLeft.cols != 3 ||
        rig.translationRightFromLeft.total() != 3U ||
        !std::isfinite(rig.baselineMm) ||
        rig.baselineMm < 20.0 || rig.baselineMm > 1000.0 ||
        !std::isfinite(rig.relativeRotationDeg) ||
        rig.relativeRotationDeg > 60.0) {
        setError("stereo rig is invalid, has an implausible baseline, or has more than 60 degrees relative optical rotation", error);
        return false;
    }
    return true;
}

bool prepareStereoProcessingGeometry(
        const StereoRigCalibration& rig,
        const cv::Size& outputSize,
        StereoProcessingGeometry* geometry,
        std::string* error) {
    if (!geometry) {
        setError("stereo processing geometry output is null", error);
        return false;
    }
    *geometry = StereoProcessingGeometry();
    std::string detail;
    if (!validateStereoRig(rig, &detail) ||
        outputSize.width < 160 || outputSize.height < 120 ||
        outputSize.width > 4096 || outputSize.height > 4096) {
        geometry->error = detail.empty() ?
            "stereo processing size is invalid" : detail;
        setError(geometry->error, error);
        return false;
    }
    geometry->outputSize = outputSize;
    geometry->leftCrop = centeredCrop(
        rig.left.intrinsics.imageSize, outputSize);
    geometry->rightCrop = centeredCrop(
        rig.right.intrinsics.imageSize, outputSize);
    geometry->leftCameraMatrix = adjustedCameraMatrix(
        rig.left.intrinsics.cameraMatrix,
        geometry->leftCrop, outputSize);
    geometry->rightCameraMatrix = adjustedCameraMatrix(
        rig.right.intrinsics.cameraMatrix,
        geometry->rightCrop, outputSize);
    geometry->ok = true;
    return true;
}

bool cropAndResizeStereoImage(const cv::Mat& source,
                              const cv::Rect& crop,
                              const cv::Size& outputSize,
                              cv::Mat* output,
                              std::string* error) {
    if (!output || source.empty() || source.type() != CV_8UC1 ||
        crop.x < 0 || crop.y < 0 || crop.width <= 0 || crop.height <= 0 ||
        crop.x + crop.width > source.cols ||
        crop.y + crop.height > source.rows ||
        outputSize.width <= 0 || outputSize.height <= 0) {
        setError("stereo source image/crop/output is invalid", error);
        return false;
    }
    const cv::Mat cropped = source(crop);
    if (cropped.size() == outputSize) {
        cropped.copyTo(*output);
    } else {
        cv::resize(cropped, *output, outputSize, 0.0, 0.0, cv::INTER_AREA);
    }
    return true;
}

}  // namespace hik_stereo
