#include "HikCalibrationCore.h"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <utility>

namespace hik_calibration {
namespace {

const double kPi = 3.14159265358979323846;

bool finiteNumber(double value) {
    return std::isfinite(value) != 0;
}

void setError(const std::string& message, std::string* error) {
    if (error) {
        *error = message;
    }
}

std::string trim(const std::string& value) {
    const std::string whitespace(" \t\r\n");
    const std::string::size_type first = value.find_first_not_of(whitespace);
    if (first == std::string::npos) {
        return std::string();
    }
    const std::string::size_type last = value.find_last_not_of(whitespace);
    return value.substr(first, last - first + 1);
}

double percentile(std::vector<double> values, double probability) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    probability = std::max(0.0, std::min(1.0, probability));
    const double position = probability * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    if (lower == upper) {
        return values[lower];
    }
    const double alpha = position - static_cast<double>(lower);
    return values[lower] * (1.0 - alpha) + values[upper] * alpha;
}

double median(const std::vector<double>& values) {
    return percentile(values, 0.5);
}

double medianAbsoluteDeviation(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    const double center = median(values);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
        deviations.push_back(std::fabs(values[i] - center));
    }
    return median(deviations);
}

ErrorMetrics metricsFromValues(const std::vector<double>& values) {
    ErrorMetrics result;
    if (values.empty()) {
        return result;
    }
    double sum = 0.0;
    double sumSquares = 0.0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        sum += values[i];
        sumSquares += values[i] * values[i];
        result.maximum = std::max(result.maximum, values[i]);
    }
    result.mean = sum / static_cast<double>(values.size());
    result.rms = std::sqrt(sumSquares / static_cast<double>(values.size()));
    result.p95 = percentile(values, 0.95);
    return result;
}

bool allFinite(const cv::Mat& matrix) {
    if (matrix.empty()) {
        return false;
    }
    cv::Mat values;
    matrix.convertTo(values, CV_64F);
    const double* begin = values.ptr<double>(0);
    const double* end = begin + values.total() * values.channels();
    for (const double* it = begin; it != end; ++it) {
        if (!finiteNumber(*it)) {
            return false;
        }
    }
    return true;
}

bool validateIntrinsics(const cv::Mat& cameraMatrix,
                        const cv::Mat& distCoeffs,
                        std::string* error) {
    if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3 || !allFinite(cameraMatrix)) {
        setError("camera matrix must be a finite 3x3 matrix", error);
        return false;
    }
    cv::Mat camera64;
    cameraMatrix.convertTo(camera64, CV_64F);
    if (camera64.at<double>(0, 0) <= 0.0 || camera64.at<double>(1, 1) <= 0.0 ||
        std::fabs(camera64.at<double>(2, 2) - 1.0) > 1.0e-6) {
        setError("camera matrix has an invalid focal length or homogeneous scale", error);
        return false;
    }
    if (distCoeffs.empty() || !allFinite(distCoeffs)) {
        setError("distortion coefficients must be finite and non-empty", error);
        return false;
    }
    const std::size_t coefficientCount = distCoeffs.total() * distCoeffs.channels();
    if (coefficientCount != 4 && coefficientCount != 5 && coefficientCount != 8 &&
        coefficientCount != 12 && coefficientCount != 14) {
        setError("distortion coefficient count must be 4, 5, 8, 12, or 14", error);
        return false;
    }
    return true;
}

cv::Mat toGray8(const cv::Mat& image, std::string* error) {
    if (image.empty()) {
        setError("image is empty", error);
        return cv::Mat();
    }
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        setError("image must have 1, 3, or 4 channels", error);
        return cv::Mat();
    }
    if (gray.depth() == CV_8U) {
        return gray.clone();
    }
    double minimum = 0.0;
    double maximum = 0.0;
    cv::minMaxLoc(gray, &minimum, &maximum);
    if (!finiteNumber(minimum) || !finiteNumber(maximum) || maximum <= minimum) {
        setError("image cannot be converted to 8-bit grayscale", error);
        return cv::Mat();
    }
    gray.convertTo(gray, CV_8U, 255.0 / (maximum - minimum),
                   -minimum * 255.0 / (maximum - minimum));
    return gray;
}

cv::Ptr<cv::aruco::CharucoBoard> makeBoard(const BoardSpec& spec,
                                            std::string* error) {
    if (!validateBoardSpec(spec, error)) {
        return cv::Ptr<cv::aruco::CharucoBoard>();
    }
    try {
        const cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(spec.dictionaryId);
        return cv::aruco::CharucoBoard::create(
            spec.squaresX, spec.squaresY,
            static_cast<float>(spec.squareLengthMm),
            static_cast<float>(spec.markerLengthMm), dictionary);
    } catch (const cv::Exception& exception) {
        setError(std::string("cannot create ChArUco board: ") + exception.what(), error);
        return cv::Ptr<cv::aruco::CharucoBoard>();
    }
}

bool objectAndImagePoints(const CharucoObservation& observation,
                          const cv::Ptr<cv::aruco::CharucoBoard>& board,
                          std::vector<cv::Point3f>* objectPoints,
                          std::vector<cv::Point2f>* imagePoints,
                          std::string* error) {
    if (!objectPoints || !imagePoints) {
        setError("output point vector is null", error);
        return false;
    }
    objectPoints->clear();
    imagePoints->clear();
    if (!board || observation.corners.size() != observation.ids.size()) {
        setError("invalid board or mismatched ChArUco corner/id arrays", error);
        return false;
    }
    std::set<int> seen;
    for (std::size_t i = 0; i < observation.ids.size(); ++i) {
        const int id = observation.ids[i];
        if (id < 0 || id >= static_cast<int>(board->chessboardCorners.size()) ||
            !seen.insert(id).second) {
            continue;
        }
        const cv::Point2f imagePoint = observation.corners[i];
        if (!finiteNumber(imagePoint.x) || !finiteNumber(imagePoint.y)) {
            continue;
        }
        objectPoints->push_back(board->chessboardCorners[static_cast<std::size_t>(id)]);
        imagePoints->push_back(imagePoint);
    }
    if (objectPoints->size() < 4) {
        setError("fewer than four valid ChArUco correspondences", error);
        return false;
    }
    return true;
}

ErrorMetrics reprojectionMetrics(const CharucoObservation& observation,
                                 const cv::Ptr<cv::aruco::CharucoBoard>& board,
                                 const cv::Vec3d& rvec,
                                 const cv::Vec3d& tvec,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs) {
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    std::string ignored;
    if (!objectAndImagePoints(observation, board, &objectPoints, &imagePoints, &ignored)) {
        return ErrorMetrics();
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projected);
    std::vector<double> errors;
    errors.reserve(projected.size());
    for (std::size_t i = 0; i < projected.size(); ++i) {
        errors.push_back(cv::norm(projected[i] - imagePoints[i]));
    }
    return metricsFromValues(errors);
}

cv::Vec3d vec3FromMat(const cv::Mat& matrix) {
    cv::Mat converted;
    matrix.reshape(1, 3).convertTo(converted, CV_64F);
    return cv::Vec3d(converted.at<double>(0, 0),
                     converted.at<double>(1, 0),
                     converted.at<double>(2, 0));
}

struct IntrinsicFitState {
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat intrinsicStdDeviations;
    double rms;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    std::vector<ErrorMetrics> viewErrors;
};

bool runIntrinsicFit(const std::vector<CharucoObservation>& observations,
                     const std::vector<int>& active,
                     const cv::Ptr<cv::aruco::CharucoBoard>& board,
                     const cv::Size& imageSize,
                     const IntrinsicCalibrationOptions& options,
                     IntrinsicFitState* state,
                     std::string* error) {
    if (!state) {
        setError("intrinsic fit output is null", error);
        return false;
    }
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<std::vector<int> > ids;
    corners.reserve(active.size());
    ids.reserve(active.size());
    for (std::size_t i = 0; i < active.size(); ++i) {
        corners.push_back(observations[static_cast<std::size_t>(active[i])].corners);
        ids.push_back(observations[static_cast<std::size_t>(active[i])].ids);
    }

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rvecMats;
    std::vector<cv::Mat> tvecMats;
    cv::Mat stdIntrinsics;
    cv::Mat stdExtrinsics;
    cv::Mat perViewErrors;
    try {
        state->rms = cv::aruco::calibrateCameraCharuco(
            corners, ids, board, imageSize,
            cameraMatrix, distCoeffs, rvecMats, tvecMats,
            stdIntrinsics, stdExtrinsics, perViewErrors,
            options.calibrationFlags,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                             options.maxIterations, options.epsilon));
    } catch (const cv::Exception& exception) {
        setError(std::string("calibrateCameraCharuco failed: ") + exception.what(), error);
        return false;
    }
    if (!finiteNumber(state->rms) || !validateIntrinsics(cameraMatrix, distCoeffs, error) ||
        rvecMats.size() != active.size() || tvecMats.size() != active.size()) {
        if (error && error->empty()) {
            *error = "intrinsic calibration returned inconsistent output";
        }
        return false;
    }

    state->cameraMatrix = cameraMatrix.clone();
    state->distCoeffs = distCoeffs.clone();
    state->intrinsicStdDeviations = stdIntrinsics.clone();
    state->rvecs.clear();
    state->tvecs.clear();
    state->viewErrors.clear();
    for (std::size_t i = 0; i < active.size(); ++i) {
        const cv::Vec3d rvec = vec3FromMat(rvecMats[i]);
        const cv::Vec3d tvec = vec3FromMat(tvecMats[i]);
        state->rvecs.push_back(rvec);
        state->tvecs.push_back(tvec);
        state->viewErrors.push_back(reprojectionMetrics(
            observations[static_cast<std::size_t>(active[i])], board,
            rvec, tvec, cameraMatrix, distCoeffs));
    }
    return true;
}

double rotationDifferenceDeg(const cv::Matx33d& first,
                             const cv::Matx33d& second) {
    const cv::Matx33d delta = second * first.t();
    double cosine = (cv::trace(cv::Mat(delta))[0] - 1.0) * 0.5;
    cosine = std::max(-1.0, std::min(1.0, cosine));
    return std::acos(cosine) * 180.0 / kPi;
}

}  // namespace

BoardSpec::BoardSpec()
    : squaresX(5),
      squaresY(7),
      squareLengthMm(22.0),
      markerLengthMm(16.0),
      dictionaryId(cv::aruco::DICT_4X4_50) {}

int BoardSpec::charucoCornerCount() const {
    return std::max(0, squaresX - 1) * std::max(0, squaresY - 1);
}

double BoardSpec::widthMm() const {
    return static_cast<double>(squaresX) * squareLengthMm;
}

double BoardSpec::heightMm() const {
    return static_cast<double>(squaresY) * squareLengthMm;
}

bool validateBoardSpec(const BoardSpec& spec, std::string* error) {
    if (spec.squaresX < 2 || spec.squaresY < 2) {
        setError("ChArUco board must have at least 2x2 squares", error);
        return false;
    }
    if (!finiteNumber(spec.squareLengthMm) || !finiteNumber(spec.markerLengthMm) ||
        spec.squareLengthMm <= 0.0 || spec.markerLengthMm <= 0.0 ||
        spec.markerLengthMm >= spec.squareLengthMm) {
        setError("board lengths must be finite and 0 < markerLength < squareLength", error);
        return false;
    }
    if (dictionaryName(spec.dictionaryId).empty()) {
        setError("unsupported ArUco dictionary id", error);
        return false;
    }
    return true;
}

std::string dictionaryName(int dictionaryId) {
    switch (dictionaryId) {
    case cv::aruco::DICT_4X4_50: return "DICT_4X4_50";
    case cv::aruco::DICT_4X4_100: return "DICT_4X4_100";
    case cv::aruco::DICT_4X4_250: return "DICT_4X4_250";
    case cv::aruco::DICT_4X4_1000: return "DICT_4X4_1000";
    case cv::aruco::DICT_5X5_50: return "DICT_5X5_50";
    case cv::aruco::DICT_5X5_100: return "DICT_5X5_100";
    case cv::aruco::DICT_5X5_250: return "DICT_5X5_250";
    case cv::aruco::DICT_5X5_1000: return "DICT_5X5_1000";
    case cv::aruco::DICT_6X6_50: return "DICT_6X6_50";
    case cv::aruco::DICT_6X6_100: return "DICT_6X6_100";
    case cv::aruco::DICT_6X6_250: return "DICT_6X6_250";
    case cv::aruco::DICT_6X6_1000: return "DICT_6X6_1000";
    case cv::aruco::DICT_7X7_50: return "DICT_7X7_50";
    case cv::aruco::DICT_7X7_100: return "DICT_7X7_100";
    case cv::aruco::DICT_7X7_250: return "DICT_7X7_250";
    case cv::aruco::DICT_7X7_1000: return "DICT_7X7_1000";
    case cv::aruco::DICT_ARUCO_ORIGINAL: return "DICT_ARUCO_ORIGINAL";
    default: return std::string();
    }
}

ErrorMetrics::ErrorMetrics() : mean(0.0), rms(0.0), p95(0.0), maximum(0.0) {}

CharucoQuality::CharucoQuality()
    : accepted(false), markerCount(0), cornerCount(0), hullAreaRatio(0.0),
      minimumBorderDistancePx(0.0), meanIntensity(0.0), saturationRatio(0.0),
      laplacianVariance(0.0) {}

DetectionOptions::DetectionOptions()
    : minMarkers(6), minCorners(12), minHullAreaRatio(0.02),
      minBorderDistancePx(8.0), saturationThreshold(250),
      maxSaturationRatio(0.01), minLaplacianVariance(0.0),
      refineMarkerCorners(true) {}

CharucoDetectionResult::CharucoDetectionResult() : ok(false) {}

bool detectCharuco(const cv::Mat& image,
                   const std::string& sampleId,
                   const BoardSpec& boardSpec,
                   const DetectionOptions& options,
                   CharucoDetectionResult* result,
                   const cv::Mat& cameraMatrix,
                   const cv::Mat& distCoeffs) {
    if (!result) {
        return false;
    }
    *result = CharucoDetectionResult();
    result->observation.sampleId = sampleId;

    std::string error;
    cv::Mat gray = toGray8(image, &error);
    if (gray.empty()) {
        result->error = error;
        return false;
    }
    result->observation.imageSize = gray.size();
    const cv::Ptr<cv::aruco::CharucoBoard> board = makeBoard(boardSpec, &error);
    if (!board) {
        result->error = error;
        return false;
    }
    if ((!cameraMatrix.empty() || !distCoeffs.empty()) &&
        !validateIntrinsics(cameraMatrix, distCoeffs, &error)) {
        result->error = error;
        return false;
    }

    const cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(boardSpec.dictionaryId);
    cv::Ptr<cv::aruco::DetectorParameters> parameters =
        cv::aruco::DetectorParameters::create();
    if (options.refineMarkerCorners) {
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    }
    std::vector<std::vector<cv::Point2f> > rejected;
    try {
        cv::aruco::detectMarkers(gray, dictionary,
                                 result->markerCorners, result->markerIds,
                                 parameters, rejected);
    } catch (const cv::Exception& exception) {
        result->error = std::string("detectMarkers failed: ") + exception.what();
        return false;
    }

    std::set<int> boardMarkerIds(board->ids.begin(), board->ids.end());
    for (std::size_t i = 0; i < result->markerIds.size(); ++i) {
        if (boardMarkerIds.count(result->markerIds[i]) == 0) {
            result->error = "detected marker id is not part of the configured board";
            return false;
        }
    }

    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    if (!result->markerIds.empty()) {
        try {
            if (cameraMatrix.empty()) {
                cv::aruco::interpolateCornersCharuco(
                    result->markerCorners, result->markerIds, gray, board,
                    charucoCorners, charucoIds);
            } else {
                cv::aruco::interpolateCornersCharuco(
                    result->markerCorners, result->markerIds, gray, board,
                    charucoCorners, charucoIds, cameraMatrix, distCoeffs);
            }
        } catch (const cv::Exception& exception) {
            result->error = std::string("interpolateCornersCharuco failed: ") + exception.what();
            return false;
        }
    }
    result->observation.corners = charucoCorners;
    result->observation.ids = charucoIds;

    CharucoQuality quality;
    quality.markerCount = static_cast<int>(result->markerIds.size());
    quality.cornerCount = static_cast<int>(charucoCorners.size());
    if (!charucoCorners.empty()) {
        std::vector<cv::Point2f> hull;
        cv::convexHull(charucoCorners, hull);
        if (hull.size() >= 3) {
            quality.hullAreaRatio = std::fabs(cv::contourArea(hull)) /
                static_cast<double>(gray.rows * gray.cols);
        }
        quality.minimumBorderDistancePx = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < charucoCorners.size(); ++i) {
            const cv::Point2f point = charucoCorners[i];
            const double border = std::min(
                std::min(static_cast<double>(point.x), static_cast<double>(point.y)),
                std::min(static_cast<double>(gray.cols - 1) - point.x,
                         static_cast<double>(gray.rows - 1) - point.y));
            quality.minimumBorderDistancePx =
                std::min(quality.minimumBorderDistancePx, border);
        }

        cv::Mat mask(gray.size(), CV_8U, cv::Scalar(0));
        if (hull.size() >= 3) {
            std::vector<cv::Point> integerHull;
            integerHull.reserve(hull.size());
            for (std::size_t i = 0; i < hull.size(); ++i) {
                integerHull.push_back(cv::Point(
                    static_cast<int>(std::lround(hull[i].x)),
                    static_cast<int>(std::lround(hull[i].y))));
            }
            cv::fillConvexPoly(mask, integerHull, cv::Scalar(255));
        } else {
            mask.setTo(255);
        }
        quality.meanIntensity = cv::mean(gray, mask)[0];
        cv::Mat saturated;
        cv::compare(gray, cv::Scalar(options.saturationThreshold), saturated, cv::CMP_GE);
        cv::bitwise_and(saturated, mask, saturated);
        const int maskPixels = cv::countNonZero(mask);
        quality.saturationRatio = maskPixels > 0
            ? static_cast<double>(cv::countNonZero(saturated)) / maskPixels : 0.0;
        cv::Mat laplacian;
        cv::Laplacian(gray, laplacian, CV_64F, 3);
        cv::Scalar lapMean;
        cv::Scalar lapStddev;
        cv::meanStdDev(laplacian, lapMean, lapStddev, mask);
        quality.laplacianVariance = lapStddev[0] * lapStddev[0];
    }

    std::vector<std::string> reasons;
    if (quality.markerCount < options.minMarkers) {
        reasons.push_back("too few markers");
    }
    if (quality.cornerCount < options.minCorners) {
        reasons.push_back("too few ChArUco corners");
    }
    if (quality.hullAreaRatio < options.minHullAreaRatio) {
        reasons.push_back("board image area is too small");
    }
    if (!charucoCorners.empty() &&
        quality.minimumBorderDistancePx < options.minBorderDistancePx) {
        reasons.push_back("board corners are too close to the image border");
    }
    if (quality.saturationRatio > options.maxSaturationRatio) {
        reasons.push_back("board region is saturated");
    }
    if (quality.laplacianVariance < options.minLaplacianVariance) {
        reasons.push_back("image is too blurred");
    }
    std::ostringstream reason;
    for (std::size_t i = 0; i < reasons.size(); ++i) {
        if (i != 0) {
            reason << "; ";
        }
        reason << reasons[i];
    }
    quality.rejectReason = reason.str();
    quality.accepted = reasons.empty();
    result->observation.quality = quality;
    result->ok = quality.accepted;
    result->error = quality.rejectReason;
    return result->ok;
}

IntrinsicCalibrationOptions::IntrinsicCalibrationOptions()
    : minViews(15), maxOutlierRounds(3), madScale(3.0),
      minOutlierThresholdPx(0.35), hardMaxViewRmsPx(0.8),
      requireAcceptedDetection(true), calibrationFlags(0),
      maxIterations(100), epsilon(1.0e-12) {}

IntrinsicViewResult::IntrinsicViewResult()
    : accepted(false), cornerCount(0), rvec(0.0, 0.0, 0.0),
      tvec(0.0, 0.0, 0.0) {}

IntrinsicCalibrationResult::IntrinsicCalibrationResult()
    : ok(false), imageSize(0, 0), calibrationRmsPx(0.0),
      inputViewCount(0), acceptedViewCount(0), rejectedViewCount(0) {}

bool calibrateIntrinsics(const std::vector<CharucoObservation>& observations,
                         const BoardSpec& boardSpec,
                         const IntrinsicCalibrationOptions& options,
                         IntrinsicCalibrationResult* result) {
    if (!result) {
        return false;
    }
    *result = IntrinsicCalibrationResult();
    result->board = boardSpec;
    result->inputViewCount = static_cast<int>(observations.size());
    result->views.resize(observations.size());

    std::string error;
    const cv::Ptr<cv::aruco::CharucoBoard> board = makeBoard(boardSpec, &error);
    if (!board) {
        result->error = error;
        return false;
    }
    if (options.minViews < 3 || options.maxOutlierRounds < 0 ||
        options.madScale < 0.0 || options.minOutlierThresholdPx <= 0.0 ||
        options.hardMaxViewRmsPx <= 0.0) {
        result->error = "invalid intrinsic calibration options";
        return false;
    }

    std::vector<int> active;
    cv::Size imageSize;
    for (std::size_t i = 0; i < observations.size(); ++i) {
        IntrinsicViewResult& view = result->views[i];
        view.sampleId = observations[i].sampleId;
        view.cornerCount = static_cast<int>(observations[i].corners.size());
        if (observations[i].imageSize.width <= 0 || observations[i].imageSize.height <= 0 ||
            observations[i].corners.size() != observations[i].ids.size() ||
            observations[i].corners.size() < 4) {
            view.rejectReason = "invalid ChArUco observation";
            continue;
        }
        if (options.requireAcceptedDetection && !observations[i].quality.accepted) {
            view.rejectReason = observations[i].quality.rejectReason.empty()
                ? "observation did not pass detection quality gates"
                : observations[i].quality.rejectReason;
            continue;
        }
        if (imageSize.width == 0) {
            imageSize = observations[i].imageSize;
        } else if (imageSize != observations[i].imageSize) {
            view.rejectReason = "image size differs from the calibration dataset";
            continue;
        }
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;
        if (!objectAndImagePoints(observations[i], board,
                                  &objectPoints, &imagePoints, &view.rejectReason)) {
            continue;
        }
        active.push_back(static_cast<int>(i));
    }
    result->imageSize = imageSize;
    if (static_cast<int>(active.size()) < options.minViews) {
        std::ostringstream message;
        message << "only " << active.size() << " valid views; at least "
                << options.minViews << " are required";
        result->error = message.str();
        result->rejectedViewCount = result->inputViewCount;
        return false;
    }

    IntrinsicFitState finalState;
    for (int round = 0; round <= options.maxOutlierRounds; ++round) {
        IntrinsicFitState state;
        if (!runIntrinsicFit(observations, active, board, imageSize,
                             options, &state, &error)) {
            result->error = error;
            return false;
        }
        finalState = state;
        if (round == options.maxOutlierRounds) {
            break;
        }

        std::vector<double> rmsValues;
        rmsValues.reserve(state.viewErrors.size());
        for (std::size_t i = 0; i < state.viewErrors.size(); ++i) {
            rmsValues.push_back(state.viewErrors[i].rms);
        }
        const double center = median(rmsValues);
        const double scaledMad = 1.4826 * medianAbsoluteDeviation(rmsValues);
        double threshold = std::max(options.minOutlierThresholdPx,
                                    center + options.madScale * scaledMad);
        threshold = std::min(threshold, options.hardMaxViewRmsPx);

        std::vector<std::pair<double, int> > candidates;
        for (std::size_t i = 0; i < active.size(); ++i) {
            if (state.viewErrors[i].rms > threshold) {
                candidates.push_back(std::make_pair(state.viewErrors[i].rms,
                                                     static_cast<int>(i)));
            }
        }
        if (candidates.empty()) {
            break;
        }
        std::sort(candidates.begin(), candidates.end(),
                  std::greater<std::pair<double, int> >());
        const int removable = static_cast<int>(active.size()) - options.minViews;
        if (removable <= 0) {
            break;
        }
        const int removalCount = std::min(removable,
            static_cast<int>(candidates.size()));
        std::set<int> activePositionsToRemove;
        for (int i = 0; i < removalCount; ++i) {
            const int activePosition = candidates[static_cast<std::size_t>(i)].second;
            activePositionsToRemove.insert(activePosition);
            IntrinsicViewResult& rejected =
                result->views[static_cast<std::size_t>(active[activePosition])];
            std::ostringstream reason;
            reason << "MAD reprojection outlier: rms="
                   << candidates[static_cast<std::size_t>(i)].first
                   << " px, threshold=" << threshold << " px";
            rejected.rejectReason = reason.str();
        }
        std::vector<int> kept;
        kept.reserve(active.size() - activePositionsToRemove.size());
        for (std::size_t i = 0; i < active.size(); ++i) {
            if (activePositionsToRemove.count(static_cast<int>(i)) == 0) {
                kept.push_back(active[i]);
            }
        }
        active.swap(kept);
    }

    // Refit once when the last loop iteration removed observations.
    if (finalState.viewErrors.size() != active.size()) {
        if (!runIntrinsicFit(observations, active, board, imageSize,
                             options, &finalState, &error)) {
            result->error = error;
            return false;
        }
    }

    result->cameraMatrix = finalState.cameraMatrix.clone();
    result->distCoeffs = finalState.distCoeffs.clone();
    result->intrinsicStdDeviations = finalState.intrinsicStdDeviations.clone();
    result->calibrationRmsPx = finalState.rms;
    result->acceptedObservationIndices = active;
    result->acceptedViewCount = static_cast<int>(active.size());
    result->rejectedViewCount = result->inputViewCount - result->acceptedViewCount;
    for (std::size_t i = 0; i < active.size(); ++i) {
        IntrinsicViewResult& view =
            result->views[static_cast<std::size_t>(active[i])];
        view.accepted = true;
        view.rejectReason.clear();
        view.reprojection = finalState.viewErrors[i];
        view.rvec = finalState.rvecs[i];
        view.tvec = finalState.tvecs[i];
    }
    result->ok = true;
    return true;
}

BoardPoseOptions::BoardPoseOptions()
    : minCorners(12), maxRmsErrorPx(0.5), maxPointErrorPx(1.0), refineLm(true) {}

BoardPoseResult::BoardPoseResult()
    : ok(false), rvec(0.0, 0.0, 0.0), tvec(0.0, 0.0, 0.0),
      rotation(cv::Matx33d::eye()), cameraFromBoard(cv::Matx44d::eye()) {}

bool estimateBoardPose(const CharucoObservation& observation,
                       const BoardSpec& boardSpec,
                       const cv::Mat& cameraMatrix,
                       const cv::Mat& distCoeffs,
                       const BoardPoseOptions& options,
                       BoardPoseResult* result) {
    if (!result) {
        return false;
    }
    *result = BoardPoseResult();
    std::string error;
    if (!validateIntrinsics(cameraMatrix, distCoeffs, &error)) {
        result->error = error;
        return false;
    }
    const cv::Ptr<cv::aruco::CharucoBoard> board = makeBoard(boardSpec, &error);
    if (!board) {
        result->error = error;
        return false;
    }
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    if (!objectAndImagePoints(observation, board, &objectPoints, &imagePoints, &error)) {
        result->error = error;
        return false;
    }
    if (static_cast<int>(objectPoints.size()) < options.minCorners) {
        result->error = "too few ChArUco corners for board pose";
        return false;
    }

    cv::Vec3d rvec(0.0, 0.0, 0.0);
    cv::Vec3d tvec(0.0, 0.0, 0.0);
    bool solved = false;
    try {
        solved = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                              rvec, tvec, false, cv::SOLVEPNP_IPPE);
        if (!solved) {
            solved = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
                                  rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        }
        if (solved && options.refineLm) {
            cv::solvePnPRefineLM(
                objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                 50, 1.0e-10));
        }
    } catch (const cv::Exception& exception) {
        result->error = std::string("board solvePnP failed: ") + exception.what();
        return false;
    }
    if (!solved || !finiteNumber(tvec[0]) || !finiteNumber(tvec[1]) ||
        !finiteNumber(tvec[2])) {
        result->error = "board pose did not converge";
        return false;
    }

    cv::Mat rotationMat;
    cv::Rodrigues(rvec, rotationMat);
    cv::Matx33d rotation;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = rotationMat.at<double>(row, col);
        }
    }
    for (std::size_t i = 0; i < objectPoints.size(); ++i) {
        const cv::Vec3d point(objectPoints[i].x, objectPoints[i].y, objectPoints[i].z);
        if ((rotation * point + tvec)[2] <= 0.0) {
            result->error = "board pose places a calibration point behind the camera";
            return false;
        }
    }

    result->rvec = rvec;
    result->tvec = tvec;
    result->rotation = rotation;
    result->cameraFromBoard = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            result->cameraFromBoard(row, col) = rotation(row, col);
        }
        result->cameraFromBoard(row, 3) = tvec[row];
    }
    result->reprojection = reprojectionMetrics(
        observation, board, rvec, tvec, cameraMatrix, distCoeffs);
    if (result->reprojection.rms > options.maxRmsErrorPx ||
        result->reprojection.maximum > options.maxPointErrorPx) {
        std::ostringstream message;
        message << "board pose reprojection error is too high: rms="
                << result->reprojection.rms << " px, max="
                << result->reprojection.maximum << " px";
        result->error = message.str();
        return false;
    }
    result->ok = true;
    return true;
}

StripeExtractionOptions::StripeExtractionOptions()
    : minimumDifference(15), thresholdStddevScale(3.0),
      minPointCount(80), maxGapRows(12), maxWidthPx(20.0),
      minConfidence(0.05), continuityPenalty(1.0),
      mode(StripeExtractionMode::Legacy) {}

StripePoint::StripePoint()
    : pixel(0.0, 0.0), row(0), peakX(0), peakDifference(0.0),
      widthPx(0.0), confidence(0.0), localBaseline(0.0),
      localNoiseMad(0.0), prominence(0.0), snr(0.0),
      saturatedFraction(0.0), saturatedPlateauWidthPx(0),
      secondPeakRatio(0.0), gradientAsymmetry(0.0),
      fitResidual(0.0), centerSigmaPx(0.0),
      rejectFlags(hik_stripe::REJECT_NONE), qualityExtractor(false) {}

StripeShadowComparison::StripeShadowComparison()
    : matchedPointCount(0U), robustMatchedPointCount(0U),
      grossMismatchPointCount(0U), signedMeanOffsetPx(0.0),
      signedMedianOffsetPx(0.0), robustSignedMeanOffsetPx(0.0),
      robustGatePx(0.0),
      absoluteMedianOffsetPx(0.0), absoluteP95OffsetPx(0.0),
      absoluteMaximumOffsetPx(0.0) {}

PairMotionMetrics::PairMotionMetrics()
    : commonCornerCount(0), poseTranslationDeltaMm(0.0),
      poseRotationDeltaDeg(0.0) {}

LaserPairOptions::LaserPairOptions()
    : maxLaserOnSaturationRatio(0.05),
      minCommonCorners(8), maxMeanCornerShiftPx(0.35),
      maxP95CornerShiftPx(0.75), maxPoseTranslationDeltaMm(0.5),
      maxPoseRotationDeltaDeg(0.2), boardBoundsMarginMm(1.0),
      minIntersectionPoints(60) {}

LaserCalibrationPairResult::LaserCalibrationPairResult() : ok(false) {}

namespace {

struct StripeCandidate {
    int peakX;
    int peakValue;
    double centerX;
    double width;
    double confidence;
};

bool extractVerticalStripe(const cv::Mat& difference,
                           const StripeExtractionOptions& options,
                           std::vector<StripePoint>* points,
                           std::string* error) {
    if (!points) {
        setError("stripe output is null", error);
        return false;
    }
    points->clear();
    if (difference.empty() || difference.type() != CV_8UC1 ||
        options.minimumDifference < 1 ||
        options.minPointCount < 1 || options.maxGapRows < 0 ||
        options.maxWidthPx <= 0.0 || options.minConfidence < 0.0 ||
        options.minConfidence > 1.0 || options.continuityPenalty < 0.0) {
        setError("invalid difference image or stripe options", error);
        return false;
    }

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(difference, mean, stddev);
    const double thresholdDouble = std::max(
        static_cast<double>(options.minimumDifference),
        mean[0] + options.thresholdStddevScale * stddev[0]);
    const int threshold = std::max(1, std::min(254,
        static_cast<int>(std::lround(thresholdDouble))));
    const double halfThreshold = std::max(1.0, threshold * 0.5);

    bool havePrevious = false;
    double previousX = 0.0;
    int gap = 0;
    for (int row = 0; row < difference.rows; ++row) {
        const unsigned char* values = difference.ptr<unsigned char>(row);
        std::vector<int> peaks;
        for (int col = 1; col + 1 < difference.cols; ++col) {
            if (values[col] >= threshold && values[col] >= values[col - 1] &&
                values[col] > values[col + 1]) {
                peaks.push_back(col);
            }
        }
        if (peaks.empty()) {
            ++gap;
            if (gap > options.maxGapRows) {
                havePrevious = false;
            }
            continue;
        }

        std::vector<StripeCandidate> candidates;
        for (std::size_t peakIndex = 0; peakIndex < peaks.size(); ++peakIndex) {
            const int peak = peaks[peakIndex];
            int left = peak;
            int right = peak;
            while (left > 0 && values[left - 1] >= halfThreshold) {
                --left;
            }
            while (right + 1 < difference.cols && values[right + 1] >= halfThreshold) {
                ++right;
            }
            const double width = static_cast<double>(right - left + 1);
            if (width > options.maxWidthPx) {
                continue;
            }
            // The local-maximum index sits on the falling edge of a saturated
            // flat-top ridge because the detector requires values[col] >
            // values[col + 1].  Centering a fixed window on that index biases
            // wide close-range stripes toward one edge.  The half-height
            // interval has already passed maxWidthPx, so use the complete
            // interval for an unbiased centre of mass.
            const int windowLeft = left;
            const int windowRight = right;
            double weightedX = 0.0;
            double weightSum = 0.0;
            for (int col = windowLeft; col <= windowRight; ++col) {
                const double weight = std::max(0.0,
                    static_cast<double>(values[col]) - halfThreshold);
                weightedX += static_cast<double>(col) * weight;
                weightSum += weight;
            }
            if (weightSum <= 0.0) {
                continue;
            }
            StripeCandidate candidate;
            candidate.peakX = peak;
            candidate.peakValue = values[peak];
            candidate.centerX = weightedX / weightSum;
            candidate.width = width;
            const double signal = static_cast<double>(candidate.peakValue - threshold) /
                std::max(1.0, 255.0 - threshold);
            const double widthScore = std::min(1.0, width / 2.0) *
                std::min(1.0, options.maxWidthPx / std::max(width, 1.0));
            candidate.confidence = std::max(0.0, std::min(1.0, signal * widthScore));
            if (candidate.confidence < options.minConfidence) {
                continue;
            }
            candidates.push_back(candidate);
        }
        if (candidates.empty()) {
            ++gap;
            continue;
        }

        std::size_t best = 0;
        double bestScore = -std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            double score = candidates[i].peakValue;
            if (havePrevious && gap <= options.maxGapRows) {
                score -= options.continuityPenalty *
                    std::fabs(candidates[i].centerX - previousX);
            }
            if (score > bestScore) {
                bestScore = score;
                best = i;
            }
        }
        const StripeCandidate& selected = candidates[best];
        StripePoint point;
        point.pixel = cv::Point2d(selected.centerX, static_cast<double>(row));
        point.row = row;
        point.peakX = selected.peakX;
        point.peakDifference = selected.peakValue;
        point.widthPx = selected.width;
        point.confidence = selected.confidence;
        points->push_back(point);
        previousX = selected.centerX;
        havePrevious = true;
        gap = 0;
    }
    if (static_cast<int>(points->size()) < options.minPointCount) {
        std::ostringstream message;
        message << "only " << points->size() << " stripe points; at least "
                << options.minPointCount << " are required";
        setError(message.str(), error);
        return false;
    }
    return true;
}

bool extractHorizontalStripe(const cv::Mat& difference,
                             const StripeExtractionOptions& options,
                             std::vector<StripePoint>* points,
                             std::string* error) {
    if (!points) {
        setError("stripe output is null", error);
        return false;
    }
    points->clear();

    cv::Mat transposed;
    if (!difference.empty()) {
        cv::transpose(difference, transposed);
    }
    std::vector<StripePoint> transposedPoints;
    std::string transposedError;
    const bool ok = extractVerticalStripe(
        transposed, options, &transposedPoints, &transposedError);

    points->reserve(transposedPoints.size());
    for (std::size_t index = 0; index < transposedPoints.size(); ++index) {
        const StripePoint& input = transposedPoints[index];
        StripePoint output = input;
        output.pixel = cv::Point2d(input.pixel.y, input.pixel.x);
        output.row = input.peakX;
        output.peakX = input.row;
        points->push_back(output);
    }
    if (!ok) {
        setError(transposedError, error);
    }
    return ok;
}

bool commonCornerMotion(const CharucoObservation& off,
                        const CharucoObservation& on,
                        PairMotionMetrics* motion,
                        std::string* error) {
    if (!motion) {
        setError("pair motion output is null", error);
        return false;
    }
    std::map<int, cv::Point2f> offById;
    for (std::size_t i = 0; i < off.ids.size(); ++i) {
        offById[off.ids[i]] = off.corners[i];
    }
    std::vector<double> shifts;
    for (std::size_t i = 0; i < on.ids.size(); ++i) {
        const std::map<int, cv::Point2f>::const_iterator found = offById.find(on.ids[i]);
        if (found != offById.end()) {
            shifts.push_back(cv::norm(on.corners[i] - found->second));
        }
    }
    motion->commonCornerCount = static_cast<int>(shifts.size());
    motion->cornerShiftPx = metricsFromValues(shifts);
    if (shifts.empty()) {
        setError("laser-off/on images have no common ChArUco corners", error);
        return false;
    }
    return true;
}

}  // namespace

bool extractVerticalLaserStripe(const cv::Mat& differenceImage,
                                const StripeExtractionOptions& options,
                                std::vector<StripePoint>* points,
                                std::string* error) {
    return extractVerticalStripe(differenceImage, options, points, error);
}

bool extractHorizontalLaserStripe(const cv::Mat& differenceImage,
                                  const StripeExtractionOptions& options,
                                  std::vector<StripePoint>* points,
                                  std::string* error) {
    return extractHorizontalStripe(differenceImage, options, points, error);
}

bool extractLaserStripe(const cv::Mat& differenceImage,
                        const StripeExtractionOptions& options,
                        std::vector<StripePoint>* points,
                        std::string* error) {
    if (!points) {
        setError("stripe output is null", error);
        return false;
    }

    std::vector<StripePoint> verticalPoints;
    std::vector<StripePoint> horizontalPoints;
    std::string verticalError;
    std::string horizontalError;
    const bool verticalOk = extractVerticalStripe(
        differenceImage, options, &verticalPoints, &verticalError);
    const bool horizontalOk = extractHorizontalStripe(
        differenceImage, options, &horizontalPoints, &horizontalError);

    const bool selectHorizontal = horizontalPoints.size() > verticalPoints.size();
    *points = selectHorizontal ? horizontalPoints : verticalPoints;
    if (verticalOk || horizontalOk) {
        return true;
    }

    if (differenceImage.empty() || differenceImage.type() != CV_8UC1) {
        setError(verticalError.empty() ? horizontalError : verticalError, error);
        return false;
    }
    std::ostringstream message;
    message << "only " << points->size() << " stripe points in best "
            << (selectHorizontal ? "horizontal" : "vertical")
            << " direction; at least " << options.minPointCount
            << " are required (other direction: "
            << (selectHorizontal ? verticalPoints.size() : horizontalPoints.size())
            << ')';
    setError(message.str(), error);
    return false;
}

namespace {

std::vector<StripePoint> qualityStripePoints(
    const hik_stripe::Result& extraction);

}  // namespace

bool processLaserCalibrationPair(const cv::Mat& laserOffImage,
                                 const cv::Mat& laserOnImage,
                                 const std::string& sampleId,
                                 const BoardSpec& board,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 const LaserPairOptions& options,
                                 LaserCalibrationPairResult* result) {
    if (!result) {
        return false;
    }
    *result = LaserCalibrationPairResult();
    result->sampleId = sampleId;
    std::string error;
    if (!validateIntrinsics(cameraMatrix, distCoeffs, &error)) {
        result->error = error;
        return false;
    }
    cv::Mat offGray = toGray8(laserOffImage, &error);
    cv::Mat onGray = toGray8(laserOnImage, &error);
    if (offGray.empty() || onGray.empty()) {
        result->error = error;
        return false;
    }
    if (offGray.size() != onGray.size()) {
        result->error = "laser-off/on image sizes differ";
        return false;
    }
    if (!finiteNumber(options.maxLaserOnSaturationRatio) ||
        options.maxLaserOnSaturationRatio < options.detection.maxSaturationRatio ||
        options.maxLaserOnSaturationRatio > 1.0) {
        result->error = "invalid laser-on saturation ratio limit";
        return false;
    }

    if (!detectCharuco(offGray, sampleId + "_off", board, options.detection,
                       &result->laserOffDetection, cameraMatrix, distCoeffs)) {
        result->error = "laser-off ChArUco detection failed: " +
            result->laserOffDetection.error;
        return false;
    }
    DetectionOptions laserOnDetectionOptions = options.detection;
    laserOnDetectionOptions.maxSaturationRatio =
        options.maxLaserOnSaturationRatio;
    if (!detectCharuco(onGray, sampleId + "_on", board, laserOnDetectionOptions,
                       &result->laserOnDetection, cameraMatrix, distCoeffs)) {
        result->error = "laser-on ChArUco detection failed: " +
            result->laserOnDetection.error;
        if (result->laserOnDetection.observation.quality.saturationRatio >
            options.maxLaserOnSaturationRatio) {
            std::ostringstream saturation;
            saturation << " (board saturation ratio="
                       << result->laserOnDetection.observation.quality.saturationRatio
                       << ", laser-on limit="
                       << options.maxLaserOnSaturationRatio << ')';
            result->error += saturation.str();
        }
        return false;
    }
    if (!commonCornerMotion(result->laserOffDetection.observation,
                            result->laserOnDetection.observation,
                            &result->motion, &error)) {
        result->error = error;
        return false;
    }
    if (result->motion.commonCornerCount < options.minCommonCorners ||
        result->motion.cornerShiftPx.mean > options.maxMeanCornerShiftPx ||
        result->motion.cornerShiftPx.p95 > options.maxP95CornerShiftPx) {
        std::ostringstream message;
        message << "laser-off/on pair moved in image: common="
                << result->motion.commonCornerCount << ", mean="
                << result->motion.cornerShiftPx.mean << " px, p95="
                << result->motion.cornerShiftPx.p95 << " px";
        result->error = message.str();
        return false;
    }

    if (!estimateBoardPose(result->laserOffDetection.observation, board,
                           cameraMatrix, distCoeffs, options.pose,
                           &result->laserOffPose)) {
        result->error = "laser-off board pose failed: " + result->laserOffPose.error;
        return false;
    }
    if (!estimateBoardPose(result->laserOnDetection.observation, board,
                           cameraMatrix, distCoeffs, options.pose,
                           &result->laserOnPose)) {
        result->error = "laser-on board pose failed: " + result->laserOnPose.error;
        return false;
    }
    result->motion.poseTranslationDeltaMm =
        cv::norm(result->laserOnPose.tvec - result->laserOffPose.tvec);
    result->motion.poseRotationDeltaDeg = rotationDifferenceDeg(
        result->laserOffPose.rotation, result->laserOnPose.rotation);
    if (result->motion.poseTranslationDeltaMm > options.maxPoseTranslationDeltaMm ||
        result->motion.poseRotationDeltaDeg > options.maxPoseRotationDeltaDeg) {
        std::ostringstream message;
        message << "laser-off/on board pose changed: translation="
                << result->motion.poseTranslationDeltaMm << " mm, rotation="
                << result->motion.poseRotationDeltaDeg << " deg";
        result->error = message.str();
        return false;
    }

    cv::subtract(onGray, offGray, result->differenceImage);
    if (options.stripe.mode == StripeExtractionMode::Quality) {
        hik_stripe::Result quality;
        if (!hik_stripe::extractCenterline(
                result->differenceImage, onGray,
                options.stripe.quality, &quality) ||
            static_cast<int>(quality.selected.size()) <
                options.stripe.minPointCount) {
            std::ostringstream message;
            message << "quality stripe extraction failed: "
                    << quality.error << "; retained="
                    << quality.selected.size() << ", required="
                    << options.stripe.minPointCount;
            result->error = message.str();
            return false;
        }
        result->stripe = qualityStripePoints(quality);
    } else if (!extractLaserStripe(
                   result->differenceImage, options.stripe,
                   &result->stripe, &error)) {
        result->error = "stripe extraction failed: " + error;
        return false;
    }

    std::vector<cv::Point2f> distorted;
    distorted.reserve(result->stripe.size());
    for (std::size_t i = 0; i < result->stripe.size(); ++i) {
        distorted.push_back(cv::Point2f(
            static_cast<float>(result->stripe[i].pixel.x),
            static_cast<float>(result->stripe[i].pixel.y)));
    }
    std::vector<cv::Point2f> normalized;
    try {
        // Omitting P is intentional: the output is normalized camera rays.
        cv::undistortPoints(distorted, normalized, cameraMatrix, distCoeffs);
    } catch (const cv::Exception& exception) {
        result->error = std::string("undistortPoints failed: ") + exception.what();
        return false;
    }

    const cv::Matx33d& rotation = result->laserOffPose.rotation;
    const cv::Vec3d& translation = result->laserOffPose.tvec;
    const cv::Vec3d boardNormalCamera(rotation(0, 2), rotation(1, 2), rotation(2, 2));
    const double boardD = -boardNormalCamera.dot(translation);
    const cv::Matx33d cameraToBoardRotation = rotation.t();
    for (std::size_t i = 0; i < normalized.size(); ++i) {
        const cv::Vec3d ray(normalized[i].x, normalized[i].y, 1.0);
        const double denominator = boardNormalCamera.dot(ray);
        if (std::fabs(denominator) < 1.0e-8) {
            continue;
        }
        const double scale = -boardD / denominator;
        if (!finiteNumber(scale) || scale <= 0.0) {
            continue;
        }
        const cv::Vec3d cameraPoint = ray * scale;
        const cv::Vec3d boardPoint = cameraToBoardRotation * (cameraPoint - translation);
        if (boardPoint[0] < -options.boardBoundsMarginMm ||
            boardPoint[0] > board.widthMm() + options.boardBoundsMarginMm ||
            boardPoint[1] < -options.boardBoundsMarginMm ||
            boardPoint[1] > board.heightMm() + options.boardBoundsMarginMm) {
            continue;
        }
        result->cameraPointsMm.push_back(cv::Point3d(
            cameraPoint[0], cameraPoint[1], cameraPoint[2]));
        result->boardPointsMm.push_back(cv::Point3d(
            boardPoint[0], boardPoint[1], boardPoint[2]));
    }
    if (static_cast<int>(result->cameraPointsMm.size()) < options.minIntersectionPoints) {
        std::ostringstream message;
        message << "only " << result->cameraPointsMm.size()
                << " stripe rays intersect the physical board; at least "
                << options.minIntersectionPoints << " are required";
        result->error = message.str();
        return false;
    }
    result->ok = true;
    return true;
}

LaserPlaneSample planeSampleFromPair(const LaserCalibrationPairResult& pair) {
    LaserPlaneSample sample;
    sample.sampleId = pair.sampleId;
    if (pair.ok) {
        sample.cameraPointsMm = pair.cameraPointsMm;
    }
    return sample;
}

PlaneFitOptions::PlaneFitOptions()
    : minPoseCount(3), minPointsPerPose(40), maxPointsPerPose(250),
      ransacIterations(2000), ransacThresholdMm(0.3),
      minInlierRatio(0.8), randomSeed(0x48494b31U) {}

LaserPlane::LaserPlane() : normal(0.0, 0.0, 1.0), dMm(0.0) {}

PlanePoseStatistics::PlanePoseStatistics() : pointCount(0), inlierCount(0) {}

LaserPlaneFitResult::LaserPlaneFitResult()
    : ok(false), poseCount(0), pointCount(0), inlierCount(0), inlierRatio(0.0) {}

StaticProfileOptions::StaticProfileOptions()
    : minReconstructedPoints(80), rayPlaneDenominatorEpsilon(1.0e-8),
      minimumDepthMm(100.0), maximumDepthMm(2000.0),
      boardBoundsMarginMm(1.0), minBoardValidationPoints(60),
      maxLineRmsMm(0.30), maxBoardPlaneRmsMm(0.30) {}

SingleFrameProfileOptions::SingleFrameProfileOptions()
    : backgroundKernelWidth(31), backgroundKernelHeight(3),
      minimumRawIntensity(60) {}

StaticProfilePoint::StaticProfilePoint()
    : cameraPointMm(0.0, 0.0, 0.0),
      boardPlaneDistanceMm(std::numeric_limits<double>::quiet_NaN()),
      insideBoard(false) {}

StaticProfileResult::StaticProfileResult()
    : ok(false), legacyExtractionPassed(false),
      qualityExtractionPassed(false), rejectedParallelRayCount(0),
      rejectedBehindCameraCount(0),
      rejectedDepthCount(0), minimumDepthMm(0.0), maximumDepthMm(0.0),
      meanConfidence(0.0), lineQualityPassed(false),
      boardValidationAvailable(false), boardValidationPassed(false),
      boardValidationPointCount(0) {}

namespace {

struct StripeReconstructionCounters {
    int parallel{0};
    int behindCamera{0};
    int outsideDepth{0};
    double minimumDepth{std::numeric_limits<double>::max()};
    double maximumDepth{-std::numeric_limits<double>::max()};
    double confidenceSum{0.0};
};

std::vector<StripePoint> qualityStripePoints(
        const hik_stripe::Result& extraction) {
    std::vector<StripePoint> points;
    points.reserve(extraction.selected.size());
    for (const hik_stripe::Candidate& input : extraction.selected) {
        if (!input.accepted()) {
            continue;
        }
        StripePoint output;
        output.pixel = input.pixel;
        output.row = static_cast<int>(std::lround(input.pixel.y));
        output.peakX = static_cast<int>(std::lround(input.pixel.x));
        output.peakDifference = input.responsePeak;
        output.widthPx = input.fwhmPx;
        output.confidence = input.quality;
        output.localBaseline = input.localBaseline;
        output.localNoiseMad = input.localNoiseMad;
        output.prominence = input.prominence;
        output.snr = input.snr;
        output.saturatedFraction = input.saturatedFraction;
        output.saturatedPlateauWidthPx =
            input.saturatedPlateauWidthPx;
        output.secondPeakRatio = input.secondPeakRatio;
        output.gradientAsymmetry = input.gradientAsymmetry;
        output.fitResidual = input.fitResidual;
        output.centerSigmaPx = input.centerSigmaPx;
        output.rejectFlags = input.rejectFlags;
        output.qualityExtractor = true;
        points.push_back(output);
    }
    return points;
}

bool reconstructStripePoints(
        const std::vector<StripePoint>& stripe,
        const IntrinsicCalibrationResult& intrinsics,
        const LaserPlaneFitResult& laserPlane,
        const StaticProfileOptions& options,
        std::vector<StaticProfilePoint>* points,
        StripeReconstructionCounters* counters,
        std::string* error) {
    if (!points || !counters) {
        setError("stripe reconstruction output is null", error);
        return false;
    }
    points->clear();
    *counters = StripeReconstructionCounters();
    if (stripe.empty()) {
        return true;
    }
    std::vector<cv::Point2f> distorted;
    distorted.reserve(stripe.size());
    for (const StripePoint& point : stripe) {
        distorted.push_back(cv::Point2f(
            static_cast<float>(point.pixel.x),
            static_cast<float>(point.pixel.y)));
    }
    std::vector<cv::Point2f> normalized;
    try {
        cv::undistortPoints(
            distorted, normalized, intrinsics.cameraMatrix,
            intrinsics.distCoeffs);
    } catch (const cv::Exception& exception) {
        setError(
            std::string("undistortPoints failed: ") +
                exception.what(),
            error);
        return false;
    }
    points->reserve(stripe.size());
    for (std::size_t index = 0U;
         index < normalized.size(); ++index) {
        const cv::Vec3d ray(
            normalized[index].x, normalized[index].y, 1.0);
        const double denominator =
            laserPlane.plane.normal.dot(ray);
        if (!finiteNumber(denominator) ||
            std::fabs(denominator) <
                options.rayPlaneDenominatorEpsilon) {
            ++counters->parallel;
            continue;
        }
        const double scale =
            -laserPlane.plane.dMm / denominator;
        if (!finiteNumber(scale) || scale <= 0.0) {
            ++counters->behindCamera;
            continue;
        }
        const cv::Vec3d point = ray * scale;
        if (!finiteNumber(point[0]) ||
            !finiteNumber(point[1]) ||
            !finiteNumber(point[2]) ||
            point[2] < options.minimumDepthMm ||
            point[2] > options.maximumDepthMm) {
            ++counters->outsideDepth;
            continue;
        }
        StaticProfilePoint output;
        output.stripe = stripe[index];
        output.cameraPointMm =
            cv::Point3d(point[0], point[1], point[2]);
        points->push_back(output);
        counters->confidenceSum += output.stripe.confidence;
        counters->minimumDepth =
            std::min(counters->minimumDepth, point[2]);
        counters->maximumDepth =
            std::max(counters->maximumDepth, point[2]);
    }
    if (error) {
        error->clear();
    }
    return true;
}

StripeShadowComparison compareStripeCenters(
        const std::vector<StripePoint>& legacy,
        const std::vector<StripePoint>& quality,
        hik_stripe::Orientation orientation) {
    StripeShadowComparison comparison;
    std::map<int, double> legacyByScanline;
    for (const StripePoint& point : legacy) {
        const int scanline =
            orientation == hik_stripe::Orientation::Horizontal
            ? static_cast<int>(std::lround(point.pixel.x))
            : static_cast<int>(std::lround(point.pixel.y));
        const double coordinate =
            orientation == hik_stripe::Orientation::Horizontal
            ? point.pixel.y
            : point.pixel.x;
        legacyByScanline[scanline] = coordinate;
    }
    std::vector<double> absoluteOffsets;
    std::vector<double> signedOffsets;
    double signedSum = 0.0;
    for (const StripePoint& point : quality) {
        const int scanline =
            orientation == hik_stripe::Orientation::Horizontal
            ? static_cast<int>(std::lround(point.pixel.x))
            : static_cast<int>(std::lround(point.pixel.y));
        const std::map<int, double>::const_iterator found =
            legacyByScanline.find(scanline);
        if (found == legacyByScanline.end()) {
            continue;
        }
        const double coordinate =
            orientation == hik_stripe::Orientation::Horizontal
            ? point.pixel.y
            : point.pixel.x;
        const double offset = coordinate - found->second;
        signedSum += offset;
        signedOffsets.push_back(offset);
        absoluteOffsets.push_back(std::fabs(offset));
    }
    comparison.matchedPointCount = absoluteOffsets.size();
    if (!absoluteOffsets.empty()) {
        comparison.signedMeanOffsetPx =
            signedSum /
            static_cast<double>(absoluteOffsets.size());
        comparison.signedMedianOffsetPx =
            median(signedOffsets);
        const double scaledMad =
            1.4826 * medianAbsoluteDeviation(signedOffsets);
        comparison.robustGatePx =
            std::max(0.5, 4.0 * scaledMad);
        const double grossGate =
            std::max(2.0, comparison.robustGatePx);
        double robustSum = 0.0;
        for (const double offset : signedOffsets) {
            const double centered = std::fabs(
                offset - comparison.signedMedianOffsetPx);
            if (centered <= comparison.robustGatePx) {
                robustSum += offset;
                ++comparison.robustMatchedPointCount;
            }
            if (centered > grossGate) {
                ++comparison.grossMismatchPointCount;
            }
        }
        if (comparison.robustMatchedPointCount > 0U) {
            comparison.robustSignedMeanOffsetPx =
                robustSum /
                static_cast<double>(
                    comparison.robustMatchedPointCount);
        }
        const ErrorMetrics metrics =
            metricsFromValues(absoluteOffsets);
        std::sort(absoluteOffsets.begin(), absoluteOffsets.end());
        const std::size_t medianIndex =
            absoluteOffsets.size() / 2U;
        comparison.absoluteMedianOffsetPx =
            (absoluteOffsets.size() & 1U) != 0U
            ? absoluteOffsets[medianIndex]
            : 0.5 * (
                absoluteOffsets[medianIndex - 1U] +
                absoluteOffsets[medianIndex]);
        comparison.absoluteP95OffsetPx = metrics.p95;
        comparison.absoluteMaximumOffsetPx = metrics.maximum;
    }
    return comparison;
}

}  // namespace

bool buildLaserPlaneValidityMask(
        const cv::Size& imageSize,
        const IntrinsicCalibrationResult& intrinsics,
        const LaserPlaneFitResult& laserPlane,
        double minimumDepthMm,
        double maximumDepthMm,
        const cv::Rect& requestedRoi,
        cv::Mat* mask,
        std::string* error) {
    if (!mask) {
        setError("laser-plane validity mask output is null", error);
        return false;
    }
    mask->release();
    std::string validationError;
    const double normalNorm = cv::norm(laserPlane.plane.normal);
    if (imageSize.width < 1 || imageSize.height < 1 ||
        !intrinsics.ok ||
        !validateIntrinsics(
            intrinsics.cameraMatrix, intrinsics.distCoeffs,
            &validationError) ||
        !laserPlane.ok || !finiteNumber(normalNorm) ||
        std::fabs(normalNorm - 1.0) > 1.0e-6 ||
        !finiteNumber(laserPlane.plane.dMm) ||
        !finiteNumber(minimumDepthMm) ||
        !finiteNumber(maximumDepthMm) ||
        minimumDepthMm <= 0.0 ||
        maximumDepthMm <= minimumDepthMm) {
        setError(
            validationError.empty()
                ? "invalid calibration or depth range for stripe mask"
                : validationError,
            error);
        return false;
    }
    const cv::Rect roi = requestedRoi.empty()
        ? cv::Rect(0, 0, imageSize.width, imageSize.height)
        : requestedRoi;
    if (roi.x < 0 || roi.y < 0 ||
        roi.width < 1 || roi.height < 1 ||
        roi.x + roi.width > imageSize.width ||
        roi.y + roi.height > imageSize.height) {
        setError(
            "stripe mask ROI is outside the calibrated image",
            error);
        return false;
    }

    std::vector<cv::Point2f> distorted;
    distorted.reserve(
        static_cast<std::size_t>(roi.width) *
        static_cast<std::size_t>(roi.height));
    for (int row = roi.y; row < roi.y + roi.height; ++row) {
        for (int column = roi.x;
             column < roi.x + roi.width; ++column) {
            distorted.push_back(cv::Point2f(
                static_cast<float>(column),
                static_cast<float>(row)));
        }
    }
    std::vector<cv::Point2f> normalized;
    try {
        cv::undistortPoints(
            distorted, normalized, intrinsics.cameraMatrix,
            intrinsics.distCoeffs);
    } catch (const cv::Exception& exception) {
        setError(
            std::string("cannot build stripe validity mask: ") +
                exception.what(),
            error);
        return false;
    }
    *mask = cv::Mat::zeros(imageSize, CV_8UC1);
    std::size_t index = 0U;
    for (int row = roi.y; row < roi.y + roi.height; ++row) {
        unsigned char* output = mask->ptr<unsigned char>(row);
        for (int column = roi.x;
             column < roi.x + roi.width;
             ++column, ++index) {
            const cv::Vec3d ray(
                normalized[index].x, normalized[index].y, 1.0);
            const double denominator =
                laserPlane.plane.normal.dot(ray);
            if (!finiteNumber(denominator) ||
                std::fabs(denominator) <= 1.0e-12) {
                continue;
            }
            const double depth =
                -laserPlane.plane.dMm / denominator;
            if (finiteNumber(depth) &&
                depth >= minimumDepthMm &&
                depth <= maximumDepthMm) {
                output[column] = 255U;
            }
        }
    }
    if (cv::countNonZero(*mask) == 0) {
        mask->release();
        setError(
            "formal depth range produced an empty stripe validity mask",
            error);
        return false;
    }
    if (error) {
        error->clear();
    }
    return true;
}

bool reconstructStaticProfile(const cv::Mat& laserOffImage,
                              const cv::Mat& laserOnImage,
                              const std::string& sampleId,
                              const IntrinsicCalibrationResult& intrinsics,
                              const LaserPlaneFitResult& laserPlane,
                              const BoardSpec& validationBoard,
                              const StaticProfileOptions& options,
                              StaticProfileResult* result) {
    if (!result) {
        return false;
    }
    *result = StaticProfileResult();
    result->sampleId = sampleId;

    std::string error;
    if (!intrinsics.ok || !validateIntrinsics(
            intrinsics.cameraMatrix, intrinsics.distCoeffs, &error)) {
        result->error = error.empty() ? "intrinsics are not valid" : error;
        return false;
    }
    const double normalNorm = cv::norm(laserPlane.plane.normal);
    if (!laserPlane.ok || !finiteNumber(normalNorm) ||
        std::fabs(normalNorm - 1.0) > 1.0e-6 ||
        !finiteNumber(laserPlane.plane.dMm)) {
        result->error = "laser plane is not valid or its normal is not normalized";
        return false;
    }
    if (options.minReconstructedPoints < 1 ||
        options.rayPlaneDenominatorEpsilon <= 0.0 ||
        !finiteNumber(options.minimumDepthMm) ||
        !finiteNumber(options.maximumDepthMm) ||
        options.minimumDepthMm <= 0.0 ||
        options.maximumDepthMm <= options.minimumDepthMm) {
        result->error = "static profile reconstruction options are invalid";
        return false;
    }

    const cv::Mat offGray = toGray8(laserOffImage, &error);
    const cv::Mat onGray = toGray8(laserOnImage, &error);
    if (offGray.empty() || onGray.empty()) {
        result->error = error;
        return false;
    }
    if (offGray.size() != onGray.size()) {
        result->error = "laser-off/on image sizes differ";
        return false;
    }
    if (intrinsics.imageSize.width > 0 && intrinsics.imageSize.height > 0 &&
        offGray.size() != intrinsics.imageSize) {
        std::ostringstream message;
        message << "image size " << offGray.cols << "x" << offGray.rows
                << " differs from calibrated size " << intrinsics.imageSize.width
                << "x" << intrinsics.imageSize.height;
        result->error = message.str();
        return false;
    }

    if (!options.stripeValidityMask.empty() &&
        (options.stripeValidityMask.type() != CV_8UC1 ||
         options.stripeValidityMask.size() != onGray.size())) {
        result->error =
            "stripe validity mask differs from the calibrated image";
        return false;
    }

    cv::subtract(onGray, offGray, result->differenceImage);
    result->legacyExtractionPassed = extractLaserStripe(
        result->differenceImage, options.stripe,
        &result->legacyStripe, &result->legacyExtractionError);
    if (!result->legacyExtractionPassed &&
        options.stripe.mode != StripeExtractionMode::Quality) {
        result->error =
            "legacy stripe extraction failed: " +
            result->legacyExtractionError;
        return false;
    }

    hik_stripe::Result qualityExtraction;
    if (options.stripe.mode != StripeExtractionMode::Legacy) {
        result->centerlineAlgorithmVersion =
            hik_stripe::algorithmVersion();
        result->qualityExtractionPassed =
            hik_stripe::extractCenterline(
                result->differenceImage, onGray,
                options.stripe.quality, &qualityExtraction,
                options.stripeValidityMask);
        result->qualityDiagnostics =
            qualityExtraction.diagnostics;
        result->qualityExtractionError =
            qualityExtraction.error;
        result->qualityStripe =
            qualityStripePoints(qualityExtraction);
        if (static_cast<int>(result->qualityStripe.size()) <
            options.stripe.minPointCount) {
            std::ostringstream message;
            message << "quality extractor retained "
                    << result->qualityStripe.size()
                    << " points; at least "
                    << options.stripe.minPointCount
                    << " are required";
            if (!result->qualityExtractionError.empty()) {
                message << " (" << result->qualityExtractionError
                        << ')';
            }
            result->qualityExtractionError = message.str();
            result->qualityExtractionPassed = false;
        }
        if (!result->legacyStripe.empty() &&
            !result->qualityStripe.empty()) {
            result->shadowComparison = compareStripeCenters(
                result->legacyStripe, result->qualityStripe,
                qualityExtraction.orientation);
        }
        if (!result->qualityExtractionPassed &&
            options.stripe.mode == StripeExtractionMode::Quality) {
            result->error =
                "quality stripe extraction failed: " +
                result->qualityExtractionError;
            return false;
        }
    }

    StripeReconstructionCounters legacyCounters;
    StripeReconstructionCounters qualityCounters;
    if (result->legacyExtractionPassed &&
        !reconstructStripePoints(
            result->legacyStripe, intrinsics, laserPlane,
            options, &result->legacyPoints, &legacyCounters,
            &result->legacyExtractionError)) {
        result->legacyExtractionPassed = false;
        if (options.stripe.mode != StripeExtractionMode::Quality) {
            result->error =
                "legacy stripe reconstruction failed: " +
                result->legacyExtractionError;
            return false;
        }
    }
    if (result->qualityExtractionPassed &&
        !reconstructStripePoints(
            result->qualityStripe, intrinsics, laserPlane,
            options, &result->qualityPoints, &qualityCounters,
            &result->qualityExtractionError)) {
        result->qualityExtractionPassed = false;
        if (options.stripe.mode == StripeExtractionMode::Quality) {
            result->error =
                "quality stripe reconstruction failed: " +
                result->qualityExtractionError;
            return false;
        }
    }
    if (result->qualityExtractionPassed &&
        static_cast<int>(result->qualityPoints.size()) <
            options.minReconstructedPoints) {
        std::ostringstream message;
        message << "quality path produced "
                << result->qualityPoints.size()
                << " valid 3-D points; at least "
                << options.minReconstructedPoints
                << " are required";
        result->qualityExtractionError = message.str();
        result->qualityExtractionPassed = false;
        if (options.stripe.mode == StripeExtractionMode::Quality) {
            result->error = result->qualityExtractionError;
            return false;
        }
    }

    StripeReconstructionCounters selectedCounters;
    if (options.stripe.mode == StripeExtractionMode::Quality) {
        result->stripe = result->qualityStripe;
        result->points = result->qualityPoints;
        selectedCounters = qualityCounters;
    } else {
        result->stripe = result->legacyStripe;
        result->points = result->legacyPoints;
        selectedCounters = legacyCounters;
    }
    result->rejectedParallelRayCount =
        selectedCounters.parallel;
    result->rejectedBehindCameraCount =
        selectedCounters.behindCamera;
    result->rejectedDepthCount =
        selectedCounters.outsideDepth;
    if (static_cast<int>(result->points.size()) < options.minReconstructedPoints) {
        std::ostringstream message;
        message << "only " << result->points.size()
                << " valid reconstructed points; at least "
                << options.minReconstructedPoints << " are required (depth range "
                << options.minimumDepthMm << ".." << options.maximumDepthMm << " mm)";
        result->error = message.str();
        return false;
    }
    result->minimumDepthMm = selectedCounters.minimumDepth;
    result->maximumDepthMm = selectedCounters.maximumDepth;
    result->meanConfidence = selectedCounters.confidenceSum /
        static_cast<double>(result->points.size());

    cv::Vec3d centroid(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < result->points.size(); ++i) {
        const cv::Point3d& point = result->points[i].cameraPointMm;
        centroid += cv::Vec3d(point.x, point.y, point.z);
    }
    centroid *= 1.0 / static_cast<double>(result->points.size());
    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (std::size_t i = 0; i < result->points.size(); ++i) {
        const cv::Point3d& point = result->points[i].cameraPointMm;
        const cv::Vec3d centered(point.x - centroid[0], point.y - centroid[1],
                                 point.z - centroid[2]);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                covariance(row, col) += centered[row] * centered[col];
            }
        }
    }
    cv::Mat eigenvalues;
    cv::Mat eigenvectors;
    cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors);
    cv::Vec3d direction(eigenvectors.at<double>(0, 0),
                        eigenvectors.at<double>(0, 1),
                        eigenvectors.at<double>(0, 2));
    const double directionNorm = cv::norm(direction);
    if (!finiteNumber(directionNorm) || directionNorm <= 1.0e-12) {
        result->error = "3-D line fit is degenerate";
        return false;
    }
    direction *= 1.0 / directionNorm;
    std::vector<double> lineDistances;
    lineDistances.reserve(result->points.size());
    for (std::size_t i = 0; i < result->points.size(); ++i) {
        const cv::Point3d& point = result->points[i].cameraPointMm;
        const cv::Vec3d centered(point.x - centroid[0], point.y - centroid[1],
                                 point.z - centroid[2]);
        lineDistances.push_back(cv::norm(centered.cross(direction)));
    }
    result->lineDistanceMm = metricsFromValues(lineDistances);
    result->lineQualityPassed = result->lineDistanceMm.rms <= options.maxLineRmsMm;

    if (validateBoardSpec(validationBoard, &error) &&
        detectCharuco(offGray, sampleId + "_validation_board", validationBoard,
                      options.boardDetection, &result->boardDetection,
                      intrinsics.cameraMatrix, intrinsics.distCoeffs) &&
        estimateBoardPose(result->boardDetection.observation, validationBoard,
                          intrinsics.cameraMatrix, intrinsics.distCoeffs,
                          options.boardPose, &result->boardPose)) {
        const cv::Matx33d& rotation = result->boardPose.rotation;
        const cv::Vec3d& translation = result->boardPose.tvec;
        const cv::Vec3d boardNormal(rotation(0, 2), rotation(1, 2), rotation(2, 2));
        const double boardD = -boardNormal.dot(translation);
        const cv::Matx33d cameraToBoard = rotation.t();
        std::vector<double> boardDistances;
        for (std::size_t i = 0; i < result->points.size(); ++i) {
            StaticProfilePoint& point = result->points[i];
            const cv::Vec3d cameraPoint(point.cameraPointMm.x,
                                        point.cameraPointMm.y,
                                        point.cameraPointMm.z);
            const cv::Vec3d boardPoint = cameraToBoard * (cameraPoint - translation);
            if (boardPoint[0] < -options.boardBoundsMarginMm ||
                boardPoint[0] > validationBoard.widthMm() + options.boardBoundsMarginMm ||
                boardPoint[1] < -options.boardBoundsMarginMm ||
                boardPoint[1] > validationBoard.heightMm() + options.boardBoundsMarginMm) {
                continue;
            }
            point.insideBoard = true;
            point.boardPlaneDistanceMm = std::fabs(boardNormal.dot(cameraPoint) + boardD);
            boardDistances.push_back(point.boardPlaneDistanceMm);
        }
        result->boardValidationPointCount = static_cast<int>(boardDistances.size());
        if (result->boardValidationPointCount >= options.minBoardValidationPoints) {
            result->boardValidationAvailable = true;
            result->boardPlaneDistanceMm = metricsFromValues(boardDistances);
            result->boardValidationPassed =
                result->boardPlaneDistanceMm.rms <= options.maxBoardPlaneRmsMm;
            result->boardValidationMessage = result->boardValidationPassed
                ? "ChArUco board-plane validation passed"
                : "ChArUco board-plane RMS exceeds the configured limit";
        } else {
            std::ostringstream message;
            message << "ChArUco pose found but only " << boardDistances.size()
                    << " profile points lie inside the physical board; at least "
                    << options.minBoardValidationPoints << " are required";
            result->boardValidationMessage = message.str();
        }
    } else {
        if (!result->boardDetection.ok) {
            result->boardValidationMessage =
                "ChArUco board validation unavailable: " + result->boardDetection.error;
        } else if (!result->boardPose.ok) {
            result->boardValidationMessage =
                "ChArUco board pose unavailable: " + result->boardPose.error;
        } else {
            result->boardValidationMessage = "ChArUco board validation unavailable";
        }
    }

    result->ok = true;
    return true;
}

bool reconstructSingleFrameProfile(const cv::Mat& laserOnImage,
                                   const std::string& sampleId,
                                   const IntrinsicCalibrationResult& intrinsics,
                                   const LaserPlaneFitResult& laserPlane,
                                   const SingleFrameProfileOptions& options,
                                   StaticProfileResult* result) {
    if (!result) {
        return false;
    }
    *result = StaticProfileResult();
    std::string error;
    const cv::Mat onGray = toGray8(laserOnImage, &error);
    if (onGray.empty()) {
        result->sampleId = sampleId;
        result->error = error.empty() ? "single-frame image is empty" : error;
        return false;
    }
    if (options.backgroundKernelWidth < 5 ||
        options.backgroundKernelWidth % 2 == 0 ||
        options.backgroundKernelHeight < 1 ||
        options.backgroundKernelHeight % 2 == 0 ||
        options.minimumRawIntensity < 0 || options.minimumRawIntensity > 255) {
        result->sampleId = sampleId;
        result->error = "single-frame background options are invalid";
        return false;
    }
    const cv::Mat horizontalKernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(options.backgroundKernelWidth, options.backgroundKernelHeight));
    const cv::Mat verticalKernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(options.backgroundKernelHeight, options.backgroundKernelWidth));
    cv::Mat horizontalBackground;
    cv::Mat verticalBackground;
    cv::morphologyEx(onGray, horizontalBackground, cv::MORPH_OPEN, horizontalKernel,
                     cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    cv::morphologyEx(onGray, verticalBackground, cv::MORPH_OPEN, verticalKernel,
                     cv::Point(-1, -1), 1, cv::BORDER_REPLICATE);
    cv::Mat background;
    if (options.reconstruction.stripe.mode ==
            StripeExtractionMode::Quality &&
        options.reconstruction.stripe.quality.orientation ==
            hik_stripe::Orientation::Horizontal) {
        // A tall opening removes a thin horizontal ridge while retaining the
        // slowly varying scene background.
        background = verticalBackground;
    } else if (options.reconstruction.stripe.mode ==
                   StripeExtractionMode::Quality &&
               options.reconstruction.stripe.quality.orientation ==
                   hik_stripe::Orientation::Vertical) {
        background = horizontalBackground;
    } else {
        // Preserve the established response exactly for Legacy/Shadow. The
        // calibrated scanner_650 therefore gets diagnostics without silently
        // changing its production pixel definition.
        cv::min(
            horizontalBackground, verticalBackground, background);
    }
    cv::Mat response;
    cv::subtract(onGray, background, response);
    if (options.minimumRawIntensity > 0) {
        cv::Mat darkMask;
        cv::compare(onGray, options.minimumRawIntensity, darkMask, cv::CMP_LT);
        response.setTo(0, darkMask);
    }
    cv::Mat filteredOn;
    cv::add(background, response, filteredOn);
    // reconstructStaticProfile computes on-off internally. Supplying the
    // estimated background and masked response preserves one common
    // reconstruction implementation.
    BoardSpec noValidationBoard;
    noValidationBoard.squaresX = 1;
    const bool ok = reconstructStaticProfile(
        background, filteredOn, sampleId, intrinsics, laserPlane,
        noValidationBoard, options.reconstruction, result);
    if (!ok && result->differenceImage.empty()) {
        result->differenceImage = response;
    }
    return ok;
}

bool saveStaticProfilePly(const std::string& path,
                          const StaticProfileResult& profile,
                          const std::string& frameId,
                          std::string* error) {
    if (!profile.ok || profile.points.empty()) {
        setError("cannot save an empty or invalid static profile", error);
        return false;
    }
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output) {
        setError("cannot open static profile PLY for writing: " + path, error);
        return false;
    }
    output << "ply\nformat ascii 1.0\n";
    output << "comment frame_id " << frameId << "\n";
    output << "comment units millimeter\n";
    output << "comment sample_id " << profile.sampleId << "\n";
    output << "element vertex " << profile.points.size() << "\n";
    output << "property double x\nproperty double y\nproperty double z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "property float confidence\nend_header\n";
    output << std::setprecision(12);
    for (std::size_t i = 0; i < profile.points.size(); ++i) {
        const StaticProfilePoint& point = profile.points[i];
        output << point.cameraPointMm.x << ' ' << point.cameraPointMm.y << ' '
               << point.cameraPointMm.z << " 255 0 0 "
               << point.stripe.confidence << '\n';
    }
    if (!output.good()) {
        setError("failed while writing static profile PLY: " + path, error);
        return false;
    }
    return true;
}

bool saveStaticProfileCsv(const std::string& path,
                          const StaticProfileResult& profile,
                          std::string* error) {
    if (!profile.ok || profile.points.empty()) {
        setError("cannot save an empty or invalid static profile", error);
        return false;
    }
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output) {
        setError("cannot open static profile CSV for writing: " + path, error);
        return false;
    }
    output << "u_px,v_px,x_mm,y_mm,z_mm,peak_difference,width_px,confidence,"
              "inside_charuco_board,board_plane_distance_mm\n";
    output << std::setprecision(12);
    for (std::size_t i = 0; i < profile.points.size(); ++i) {
        const StaticProfilePoint& point = profile.points[i];
        output << point.stripe.pixel.x << ',' << point.stripe.pixel.y << ','
               << point.cameraPointMm.x << ',' << point.cameraPointMm.y << ','
               << point.cameraPointMm.z << ',' << point.stripe.peakDifference << ','
               << point.stripe.widthPx << ',' << point.stripe.confidence << ','
               << (point.insideBoard ? 1 : 0) << ',';
        if (point.insideBoard && finiteNumber(point.boardPlaneDistanceMm)) {
            output << point.boardPlaneDistanceMm;
        }
        output << '\n';
    }
    if (!output.good()) {
        setError("failed while writing static profile CSV: " + path, error);
        return false;
    }
    return true;
}

namespace {

struct FitPose {
    std::string sampleId;
    std::vector<cv::Point3d> points;
};

struct TaggedPoint {
    cv::Point3d point;
    int poseIndex;
};

bool finitePoint(const cv::Point3d& point) {
    return finiteNumber(point.x) && finiteNumber(point.y) && finiteNumber(point.z);
}

double pointPlaneDistance(const LaserPlane& plane, const cv::Point3d& point) {
    return std::fabs(plane.normal.dot(cv::Vec3d(point.x, point.y, point.z)) +
                     plane.dMm);
}

bool planeFromThreePoints(const cv::Point3d& first,
                          const cv::Point3d& second,
                          const cv::Point3d& third,
                          LaserPlane* plane) {
    const cv::Vec3d a(second.x - first.x, second.y - first.y, second.z - first.z);
    const cv::Vec3d b(third.x - first.x, third.y - first.y, third.z - first.z);
    cv::Vec3d normal = a.cross(b);
    const double norm = cv::norm(normal);
    if (!finiteNumber(norm) || norm < 1.0e-8) {
        return false;
    }
    normal *= 1.0 / norm;
    double d = -normal.dot(cv::Vec3d(first.x, first.y, first.z));
    if (d > 0.0) {
        normal *= -1.0;
        d *= -1.0;
    }
    plane->normal = normal;
    plane->dMm = d;
    return true;
}

std::vector<TaggedPoint> balancedPoints(const std::vector<FitPose>& poses,
                                        int maximumPerPose) {
    std::vector<TaggedPoint> output;
    if (poses.empty()) {
        return output;
    }
    std::size_t count = static_cast<std::size_t>(std::max(1, maximumPerPose));
    for (std::size_t pose = 0; pose < poses.size(); ++pose) {
        count = std::min(count, poses[pose].points.size());
    }
    for (std::size_t pose = 0; pose < poses.size(); ++pose) {
        const std::vector<cv::Point3d>& points = poses[pose].points;
        for (std::size_t i = 0; i < count; ++i) {
            const std::size_t index = count == 1 ? 0 :
                static_cast<std::size_t>(std::lround(
                    static_cast<double>(i) * (points.size() - 1) / (count - 1)));
            TaggedPoint tagged;
            tagged.point = points[index];
            tagged.poseIndex = static_cast<int>(pose);
            output.push_back(tagged);
        }
    }
    return output;
}

bool weightedSvdPlane(const std::vector<FitPose>& poses,
                      const LaserPlane& seed,
                      double inlierThreshold,
                      LaserPlane* plane) {
    std::vector<std::vector<cv::Point3d> > inliers(poses.size());
    std::size_t total = 0;
    for (std::size_t pose = 0; pose < poses.size(); ++pose) {
        for (std::size_t i = 0; i < poses[pose].points.size(); ++i) {
            if (pointPlaneDistance(seed, poses[pose].points[i]) <= inlierThreshold) {
                inliers[pose].push_back(poses[pose].points[i]);
                ++total;
            }
        }
    }
    int contributingPoses = 0;
    for (std::size_t pose = 0; pose < inliers.size(); ++pose) {
        if (!inliers[pose].empty()) {
            ++contributingPoses;
        }
    }
    if (contributingPoses < 3 || total < 3) {
        return false;
    }

    cv::Vec3d centroid(0.0, 0.0, 0.0);
    double totalWeight = 0.0;
    for (std::size_t pose = 0; pose < inliers.size(); ++pose) {
        if (inliers[pose].empty()) {
            continue;
        }
        const double pointWeight = 1.0 / static_cast<double>(inliers[pose].size());
        for (std::size_t i = 0; i < inliers[pose].size(); ++i) {
            centroid += cv::Vec3d(inliers[pose][i].x,
                                  inliers[pose][i].y,
                                  inliers[pose][i].z) * pointWeight;
            totalWeight += pointWeight;
        }
    }
    centroid *= 1.0 / totalWeight;

    // Only the right singular vector associated with the smallest singular
    // value is needed. Building FULL_UV for an N-by-3 design matrix allocates
    // an N-by-N U matrix, which becomes prohibitively large for dense horizontal
    // stripes. The 3-by-3 scatter matrix below is design.t() * design and has
    // exactly the same eigenvector while using constant memory.
    cv::Matx33d scatter = cv::Matx33d::zeros();
    for (std::size_t pose = 0; pose < inliers.size(); ++pose) {
        if (inliers[pose].empty()) {
            continue;
        }
        const double scale = std::sqrt(1.0 / static_cast<double>(inliers[pose].size()));
        for (std::size_t i = 0; i < inliers[pose].size(); ++i) {
            const cv::Vec3d centered(
                (inliers[pose][i].x - centroid[0]) * scale,
                (inliers[pose][i].y - centroid[1]) * scale,
                (inliers[pose][i].z - centroid[2]) * scale);
            for (int row = 0; row < 3; ++row) {
                for (int column = row; column < 3; ++column) {
                    scatter(row, column) += centered[row] * centered[column];
                }
            }
        }
    }
    scatter(1, 0) = scatter(0, 1);
    scatter(2, 0) = scatter(0, 2);
    scatter(2, 1) = scatter(1, 2);

    cv::Mat eigenvalues;
    cv::Mat eigenvectors;
    if (!cv::eigen(cv::Mat(scatter), eigenvalues, eigenvectors) ||
        eigenvectors.rows != 3 || eigenvectors.cols != 3) {
        return false;
    }
    cv::Vec3d normal(eigenvectors.at<double>(2, 0),
                     eigenvectors.at<double>(2, 1),
                     eigenvectors.at<double>(2, 2));
    const double norm = cv::norm(normal);
    if (!finiteNumber(norm) || norm < 1.0e-12) {
        return false;
    }
    normal *= 1.0 / norm;
    double d = -normal.dot(centroid);
    if (d > 0.0) {
        normal *= -1.0;
        d *= -1.0;
    }
    plane->normal = normal;
    plane->dMm = d;
    return true;
}

}  // namespace

bool fitLaserPlane(const std::vector<LaserPlaneSample>& samples,
                   const PlaneFitOptions& options,
                   LaserPlaneFitResult* result) {
    if (!result) {
        return false;
    }
    *result = LaserPlaneFitResult();
    if (options.minPoseCount < 3 || options.minPointsPerPose < 3 ||
        options.maxPointsPerPose < options.minPointsPerPose ||
        options.ransacIterations < 1 || options.ransacThresholdMm <= 0.0 ||
        options.minInlierRatio <= 0.0 || options.minInlierRatio > 1.0) {
        result->error = "invalid laser-plane fit options";
        return false;
    }

    std::vector<FitPose> poses;
    for (std::size_t sample = 0; sample < samples.size(); ++sample) {
        FitPose pose;
        pose.sampleId = samples[sample].sampleId;
        for (std::size_t i = 0; i < samples[sample].cameraPointsMm.size(); ++i) {
            if (finitePoint(samples[sample].cameraPointsMm[i])) {
                pose.points.push_back(samples[sample].cameraPointsMm[i]);
            }
        }
        if (static_cast<int>(pose.points.size()) >= options.minPointsPerPose) {
            poses.push_back(pose);
        }
    }
    result->poseCount = static_cast<int>(poses.size());
    if (result->poseCount < options.minPoseCount) {
        std::ostringstream message;
        message << "only " << result->poseCount
                << " valid poses; at least " << options.minPoseCount << " are required";
        result->error = message.str();
        return false;
    }

    const std::vector<TaggedPoint> balanced =
        balancedPoints(poses, options.maxPointsPerPose);
    if (balanced.size() < 3) {
        result->error = "too few balanced points for laser-plane fit";
        return false;
    }
    std::vector<std::vector<int> > balancedIndicesByPose(poses.size());
    for (std::size_t i = 0; i < balanced.size(); ++i) {
        balancedIndicesByPose[static_cast<std::size_t>(balanced[i].poseIndex)].push_back(
            static_cast<int>(i));
    }

    std::mt19937 generator(options.randomSeed);
    std::vector<int> poseOrder(poses.size());
    std::iota(poseOrder.begin(), poseOrder.end(), 0);
    LaserPlane bestPlane;
    int bestInliers = -1;
    double bestSquaredError = std::numeric_limits<double>::max();
    for (int iteration = 0; iteration < options.ransacIterations; ++iteration) {
        std::shuffle(poseOrder.begin(), poseOrder.end(), generator);
        cv::Point3d chosen[3];
        bool valid = true;
        for (int i = 0; i < 3; ++i) {
            const std::vector<int>& indices =
                balancedIndicesByPose[static_cast<std::size_t>(poseOrder[i])];
            if (indices.empty()) {
                valid = false;
                break;
            }
            std::uniform_int_distribution<int> distribution(
                0, static_cast<int>(indices.size()) - 1);
            chosen[i] = balanced[static_cast<std::size_t>(
                indices[static_cast<std::size_t>(distribution(generator))])].point;
        }
        LaserPlane candidate;
        if (!valid || !planeFromThreePoints(chosen[0], chosen[1], chosen[2], &candidate)) {
            continue;
        }
        int inliers = 0;
        double squaredError = 0.0;
        for (std::size_t i = 0; i < balanced.size(); ++i) {
            const double distance = pointPlaneDistance(candidate, balanced[i].point);
            if (distance <= options.ransacThresholdMm) {
                ++inliers;
                squaredError += distance * distance;
            }
        }
        if (inliers > bestInliers ||
            (inliers == bestInliers && squaredError < bestSquaredError)) {
            bestPlane = candidate;
            bestInliers = inliers;
            bestSquaredError = squaredError;
        }
    }
    if (bestInliers < 3) {
        result->error = "balanced cross-pose RANSAC could not find a laser plane";
        return false;
    }

    LaserPlane refined = bestPlane;
    for (int refinement = 0; refinement < 2; ++refinement) {
        LaserPlane next;
        if (!weightedSvdPlane(poses, refined, options.ransacThresholdMm, &next)) {
            result->error = "weighted SVD laser-plane refinement failed";
            return false;
        }
        refined = next;
    }
    result->plane = refined;

    std::vector<double> allInlierDistances;
    for (std::size_t pose = 0; pose < poses.size(); ++pose) {
        PlanePoseStatistics statistics;
        statistics.sampleId = poses[pose].sampleId;
        statistics.pointCount = static_cast<int>(poses[pose].points.size());
        result->pointCount += statistics.pointCount;
        std::vector<double> poseInliers;
        for (std::size_t i = 0; i < poses[pose].points.size(); ++i) {
            const double distance = pointPlaneDistance(refined, poses[pose].points[i]);
            if (distance <= options.ransacThresholdMm) {
                poseInliers.push_back(distance);
                allInlierDistances.push_back(distance);
            }
        }
        statistics.inlierCount = static_cast<int>(poseInliers.size());
        statistics.distanceMm = metricsFromValues(poseInliers);
        result->inlierCount += statistics.inlierCount;
        result->poses.push_back(statistics);
    }
    result->inlierRatio = result->pointCount > 0
        ? static_cast<double>(result->inlierCount) / result->pointCount : 0.0;
    result->inlierDistanceMm = metricsFromValues(allInlierDistances);
    if (result->inlierRatio < options.minInlierRatio) {
        std::ostringstream message;
        message << "laser-plane inlier ratio " << result->inlierRatio
                << " is below " << options.minInlierRatio;
        result->error = message.str();
        return false;
    }
    int contributingPoses = 0;
    for (std::size_t i = 0; i < result->poses.size(); ++i) {
        if (result->poses[i].inlierCount >= 3) {
            ++contributingPoses;
        }
    }
    if (contributingPoses < options.minPoseCount) {
        result->error = "laser-plane inliers do not span enough distinct poses";
        return false;
    }
    result->ok = true;
    return true;
}

IntrinsicsYamlMetadata::IntrinsicsYamlMetadata()
    : cameraName("hik_camera"), pixelFormat("Mono8") {}

LaserPlaneYamlMetadata::LaserPlaneYamlMetadata()
    : validCameraZMinMm(0.0), validCameraZMaxMm(0.0) {}

namespace {

std::string yamlQuote(const std::string& value) {
    std::string output("\"");
    for (std::size_t i = 0; i < value.size(); ++i) {
        switch (value[i]) {
        case '\\': output += "\\\\"; break;
        case '"': output += "\\\""; break;
        case '\n': output += "\\n"; break;
        case '\r': output += "\\r"; break;
        case '\t': output += "\\t"; break;
        default: output += value[i]; break;
        }
    }
    output += '"';
    return output;
}

std::string yamlUnquote(const std::string& value) {
    const std::string input = trim(value);
    if (input.size() < 2 || input.front() != '"' || input.back() != '"') {
        return input;
    }
    std::string output;
    bool escaped = false;
    for (std::size_t i = 1; i + 1 < input.size(); ++i) {
        const char character = input[i];
        if (!escaped && character == '\\') {
            escaped = true;
            continue;
        }
        if (escaped) {
            switch (character) {
            case 'n': output += '\n'; break;
            case 'r': output += '\r'; break;
            case 't': output += '\t'; break;
            default: output += character; break;
            }
            escaped = false;
        } else {
            output += character;
        }
    }
    return output;
}

std::string withoutYamlComment(const std::string& line) {
    bool quoted = false;
    bool escaped = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char character = line[i];
        if (escaped) {
            escaped = false;
            continue;
        }
        if (character == '\\' && quoted) {
            escaped = true;
        } else if (character == '"') {
            quoted = !quoted;
        } else if (character == '#' && !quoted) {
            return line.substr(0, i);
        }
    }
    return line;
}

bool parsePlainYaml(const std::string& path,
                    std::map<std::string, std::string>* values,
                    std::string* error) {
    if (!values) {
        setError("YAML output map is null", error);
        return false;
    }
    values->clear();
    std::ifstream input(path.c_str());
    if (!input) {
        setError("cannot open YAML file: " + path, error);
        return false;
    }
    std::vector<std::pair<int, std::string> > sections;
    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        line = withoutYamlComment(line);
        if (trim(line).empty()) {
            continue;
        }
        int indentation = 0;
        while (indentation < static_cast<int>(line.size()) &&
               line[static_cast<std::size_t>(indentation)] == ' ') {
            ++indentation;
        }
        if (indentation < static_cast<int>(line.size()) &&
            line[static_cast<std::size_t>(indentation)] == '\t') {
            std::ostringstream message;
            message << path << ':' << lineNumber << ": YAML tabs are not supported";
            setError(message.str(), error);
            return false;
        }
        const std::string content = trim(line.substr(static_cast<std::size_t>(indentation)));
        if (content.empty() || content[0] == '-') {
            std::ostringstream message;
            message << path << ':' << lineNumber
                    << ": only mappings and one-line arrays are supported";
            setError(message.str(), error);
            return false;
        }
        const std::string::size_type colon = content.find(':');
        if (colon == std::string::npos) {
            std::ostringstream message;
            message << path << ':' << lineNumber << ": missing ':'";
            setError(message.str(), error);
            return false;
        }
        const std::string key = trim(content.substr(0, colon));
        const std::string scalar = trim(content.substr(colon + 1));
        if (key.empty()) {
            std::ostringstream message;
            message << path << ':' << lineNumber << ": empty key";
            setError(message.str(), error);
            return false;
        }
        while (!sections.empty() && sections.back().first >= indentation) {
            sections.pop_back();
        }
        std::string fullKey;
        for (std::size_t i = 0; i < sections.size(); ++i) {
            if (!fullKey.empty()) {
                fullKey += '.';
            }
            fullKey += sections[i].second;
        }
        if (!fullKey.empty()) {
            fullKey += '.';
        }
        fullKey += key;
        if (scalar.empty()) {
            sections.push_back(std::make_pair(indentation, key));
        } else {
            (*values)[fullKey] = yamlUnquote(scalar);
        }
    }
    if (!input.eof()) {
        setError("failed while reading YAML file: " + path, error);
        return false;
    }
    return true;
}

bool lookup(const std::map<std::string, std::string>& values,
            const std::string& key,
            std::string* value,
            std::string* error) {
    const std::map<std::string, std::string>::const_iterator found = values.find(key);
    if (found == values.end()) {
        setError("missing YAML key: " + key, error);
        return false;
    }
    if (value) {
        *value = found->second;
    }
    return true;
}

bool optionalLookup(const std::map<std::string, std::string>& values,
                    const std::string& key,
                    std::string* value) {
    const std::map<std::string, std::string>::const_iterator found = values.find(key);
    if (found == values.end()) {
        return false;
    }
    if (value) {
        *value = found->second;
    }
    return true;
}

bool parseDouble(const std::string& value, double* output) {
    if (!output) {
        return false;
    }
    std::istringstream stream(trim(value));
    double parsed = 0.0;
    stream >> parsed;
    stream >> std::ws;
    if (!stream.eof() || !finiteNumber(parsed)) {
        return false;
    }
    *output = parsed;
    return true;
}

bool parseInt(const std::string& value, int* output) {
    if (!output) {
        return false;
    }
    std::istringstream stream(trim(value));
    int parsed = 0;
    stream >> parsed;
    stream >> std::ws;
    if (!stream.eof()) {
        return false;
    }
    *output = parsed;
    return true;
}

bool requiredDouble(const std::map<std::string, std::string>& values,
                    const std::string& key,
                    double* output,
                    std::string* error) {
    std::string value;
    if (!lookup(values, key, &value, error)) {
        return false;
    }
    if (!parseDouble(value, output)) {
        setError("YAML key is not a finite number: " + key, error);
        return false;
    }
    return true;
}

bool requiredInt(const std::map<std::string, std::string>& values,
                 const std::string& key,
                 int* output,
                 std::string* error) {
    std::string value;
    if (!lookup(values, key, &value, error)) {
        return false;
    }
    if (!parseInt(value, output)) {
        setError("YAML key is not an integer: " + key, error);
        return false;
    }
    return true;
}

bool parseDoubleList(const std::string& value,
                     std::vector<double>* output) {
    if (!output) {
        return false;
    }
    output->clear();
    const std::string input = trim(value);
    if (input.size() < 2 || input.front() != '[' || input.back() != ']') {
        return false;
    }
    const std::string body = input.substr(1, input.size() - 2);
    if (trim(body).empty()) {
        return true;
    }
    std::istringstream stream(body);
    std::string token;
    while (std::getline(stream, token, ',')) {
        double number = 0.0;
        if (!parseDouble(token, &number)) {
            output->clear();
            return false;
        }
        output->push_back(number);
    }
    return true;
}

bool requiredDoubleList(const std::map<std::string, std::string>& values,
                        const std::string& key,
                        std::size_t expectedSize,
                        std::vector<double>* output,
                        std::string* error) {
    std::string value;
    if (!lookup(values, key, &value, error)) {
        return false;
    }
    if (!parseDoubleList(value, output) || output->size() != expectedSize) {
        std::ostringstream message;
        message << "YAML key " << key << " must contain "
                << expectedSize << " finite values";
        setError(message.str(), error);
        return false;
    }
    return true;
}

void writeArray(std::ostream& output, const std::vector<double>& values) {
    output << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            output << ", ";
        }
        output << values[i];
    }
    output << ']';
}

std::vector<double> matrixValues(const cv::Mat& matrix) {
    cv::Mat converted;
    matrix.convertTo(converted, CV_64F);
    if (!converted.isContinuous()) {
        converted = converted.clone();
    }
    const double* begin = converted.ptr<double>(0);
    return std::vector<double>(
        begin, begin + converted.total() * converted.channels());
}

int dictionaryIdFromName(const std::string& name) {
    const int ids[] = {
        cv::aruco::DICT_4X4_50, cv::aruco::DICT_4X4_100,
        cv::aruco::DICT_4X4_250, cv::aruco::DICT_4X4_1000,
        cv::aruco::DICT_5X5_50, cv::aruco::DICT_5X5_100,
        cv::aruco::DICT_5X5_250, cv::aruco::DICT_5X5_1000,
        cv::aruco::DICT_6X6_50, cv::aruco::DICT_6X6_100,
        cv::aruco::DICT_6X6_250, cv::aruco::DICT_6X6_1000,
        cv::aruco::DICT_7X7_50, cv::aruco::DICT_7X7_100,
        cv::aruco::DICT_7X7_250, cv::aruco::DICT_7X7_1000,
        cv::aruco::DICT_ARUCO_ORIGINAL
    };
    for (std::size_t i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        if (dictionaryName(ids[i]) == name) {
            return ids[i];
        }
    }
    return -1;
}

bool loadBoard(const std::map<std::string, std::string>& values,
               const std::string& prefix,
               BoardSpec* board,
               std::string* error) {
    if (!board) {
        setError("board output is null", error);
        return false;
    }
    BoardSpec parsed;
    if (!requiredInt(values, prefix + ".squares_x", &parsed.squaresX, error) ||
        !requiredInt(values, prefix + ".squares_y", &parsed.squaresY, error) ||
        !requiredDouble(values, prefix + ".square_length_mm", &parsed.squareLengthMm, error) ||
        !requiredDouble(values, prefix + ".marker_length_mm", &parsed.markerLengthMm, error)) {
        return false;
    }
    std::string dictionary;
    if (!lookup(values, prefix + ".dictionary", &dictionary, error)) {
        return false;
    }
    parsed.dictionaryId = dictionaryIdFromName(dictionary);
    if (!validateBoardSpec(parsed, error)) {
        return false;
    }
    *board = parsed;
    return true;
}

void writeBoard(std::ostream& output,
                const std::string& indentation,
                const BoardSpec& board,
                const std::string& patternSha256) {
    output << indentation << "type: \"charuco\"\n";
    output << indentation << "squares_x: " << board.squaresX << '\n';
    output << indentation << "squares_y: " << board.squaresY << '\n';
    output << indentation << "square_length_mm: " << board.squareLengthMm << '\n';
    output << indentation << "marker_length_mm: " << board.markerLengthMm << '\n';
    output << indentation << "dictionary: " << yamlQuote(dictionaryName(board.dictionaryId)) << '\n';
    output << indentation << "board_width_mm: " << board.widthMm() << '\n';
    output << indentation << "board_height_mm: " << board.heightMm() << '\n';
    output << indentation << "opencv_pattern: \"legacy_pre_4_6\"\n";
    output << indentation << "printed_pattern_sha256: " << yamlQuote(patternSha256) << '\n';
}

}  // namespace

bool saveIntrinsicsYaml(const std::string& path,
                        const IntrinsicCalibrationResult& calibration,
                        const IntrinsicsYamlMetadata& metadata,
                        std::string* error) {
    std::string validationError;
    if (!calibration.ok || calibration.imageSize.width <= 0 ||
        calibration.imageSize.height <= 0 ||
        !validateBoardSpec(calibration.board, &validationError) ||
        !validateIntrinsics(calibration.cameraMatrix, calibration.distCoeffs,
                            &validationError)) {
        setError("cannot save invalid intrinsic calibration: " + validationError, error);
        return false;
    }
    std::ofstream output(path.c_str());
    if (!output) {
        setError("cannot create intrinsic YAML file: " + path, error);
        return false;
    }
    output << std::setprecision(17);
    const cv::Mat camera64 = calibration.cameraMatrix;
    const std::vector<double> camera = matrixValues(camera64);
    const std::vector<double> distortion = matrixValues(calibration.distCoeffs);
    const std::vector<double> identity = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    const std::vector<double> projection = {
        camera[0], camera[1], camera[2], 0.0,
        camera[3], camera[4], camera[5], 0.0,
        camera[6], camera[7], camera[8], 0.0
    };
    output << "schema_version: 1\n";
    output << "calibration_type: \"camera_intrinsics\"\n";
    output << "image_width: " << calibration.imageSize.width << '\n';
    output << "image_height: " << calibration.imageSize.height << '\n';
    output << "camera_name: " << yamlQuote(metadata.cameraName) << "\n\n";
    output << "camera_matrix:\n  rows: 3\n  cols: 3\n  data: ";
    writeArray(output, camera);
    output << "\n\ndistortion_model: \"plumb_bob\"\n";
    output << "distortion_coefficients:\n  rows: 1\n  cols: "
           << distortion.size() << "\n  data: ";
    writeArray(output, distortion);
    output << "\n\nrectification_matrix:\n  rows: 3\n  cols: 3\n  data: ";
    writeArray(output, identity);
    output << "\n\nprojection_matrix:\n  rows: 3\n  cols: 4\n  data: ";
    writeArray(output, projection);
    output << "\n\nmetadata:\n";
    output << "  camera_model: " << yamlQuote(metadata.cameraModel) << '\n';
    output << "  camera_serial: " << yamlQuote(metadata.cameraSerial) << '\n';
    output << "  frame_id: " << yamlQuote(metadata.frameId) << '\n';
    output << "  pixel_format: " << yamlQuote(metadata.pixelFormat) << '\n';
    output << "  opencv_version: \"" << CV_VERSION << "\"\n";
    output << "  board:\n";
    writeBoard(output, "    ", calibration.board, metadata.printedPatternSha256);
    output << "  result:\n";
    output << "    input_views: " << calibration.inputViewCount << '\n';
    output << "    accepted_views: " << calibration.acceptedViewCount << '\n';
    output << "    rejected_views: " << calibration.rejectedViewCount << '\n';
    output << "    reprojection_rms_px: " << calibration.calibrationRmsPx << '\n';
    output << "  generated_at: " << yamlQuote(metadata.generatedAt) << '\n';
    if (!output) {
        setError("failed while writing intrinsic YAML file: " + path, error);
        return false;
    }
    return true;
}

bool loadIntrinsicsYaml(const std::string& path,
                        IntrinsicCalibrationResult* calibration,
                        IntrinsicsYamlMetadata* metadata,
                        std::string* error) {
    if (!calibration) {
        setError("intrinsic calibration output is null", error);
        return false;
    }
    std::map<std::string, std::string> values;
    if (!parsePlainYaml(path, &values, error)) {
        return false;
    }
    int schemaVersion = 0;
    int cameraRows = 0;
    int cameraColumns = 0;
    int distortionRows = 0;
    std::string calibrationType;
    std::string distortionModel;
    std::string boardType;
    std::string patternConvention;
    if (!requiredInt(values, "schema_version", &schemaVersion, error) ||
        !lookup(values, "calibration_type", &calibrationType, error) ||
        !lookup(values, "distortion_model", &distortionModel, error) ||
        !requiredInt(values, "camera_matrix.rows", &cameraRows, error) ||
        !requiredInt(values, "camera_matrix.cols", &cameraColumns, error) ||
        !requiredInt(values, "distortion_coefficients.rows", &distortionRows, error) ||
        !lookup(values, "metadata.board.type", &boardType, error) ||
        !lookup(values, "metadata.board.opencv_pattern", &patternConvention, error)) {
        return false;
    }
    if (schemaVersion != 1 || calibrationType != "camera_intrinsics" ||
        distortionModel != "plumb_bob" || cameraRows != 3 ||
        cameraColumns != 3 || distortionRows != 1 || boardType != "charuco" ||
        patternConvention != "legacy_pre_4_6") {
        setError("unsupported or inconsistent intrinsic YAML schema/model", error);
        return false;
    }
    IntrinsicCalibrationResult parsed;
    if (!requiredInt(values, "image_width", &parsed.imageSize.width, error) ||
        !requiredInt(values, "image_height", &parsed.imageSize.height, error)) {
        return false;
    }
    if (parsed.imageSize.width <= 0 || parsed.imageSize.height <= 0) {
        setError("intrinsic YAML image size must be positive", error);
        return false;
    }
    std::vector<double> camera;
    if (!requiredDoubleList(values, "camera_matrix.data", 9, &camera, error)) {
        return false;
    }
    int distortionColumns = 0;
    if (!requiredInt(values, "distortion_coefficients.cols", &distortionColumns, error) ||
        (distortionColumns != 4 && distortionColumns != 5 &&
         distortionColumns != 8 && distortionColumns != 12 &&
         distortionColumns != 14)) {
        setError("invalid distortion_coefficients.cols", error);
        return false;
    }
    std::vector<double> distortion;
    if (!requiredDoubleList(values, "distortion_coefficients.data",
                            static_cast<std::size_t>(distortionColumns),
                            &distortion, error)) {
        return false;
    }
    parsed.cameraMatrix = cv::Mat(3, 3, CV_64F);
    std::copy(camera.begin(), camera.end(), parsed.cameraMatrix.ptr<double>(0));
    parsed.distCoeffs = cv::Mat(1, distortionColumns, CV_64F);
    std::copy(distortion.begin(), distortion.end(), parsed.distCoeffs.ptr<double>(0));
    if (!loadBoard(values, "metadata.board", &parsed.board, error) ||
        !validateIntrinsics(parsed.cameraMatrix, parsed.distCoeffs, error)) {
        return false;
    }
    if (!requiredDouble(values, "metadata.result.reprojection_rms_px",
                        &parsed.calibrationRmsPx, error) ||
        !requiredInt(values, "metadata.result.input_views", &parsed.inputViewCount, error) ||
        !requiredInt(values, "metadata.result.accepted_views", &parsed.acceptedViewCount, error) ||
        !requiredInt(values, "metadata.result.rejected_views", &parsed.rejectedViewCount, error)) {
        return false;
    }
    if (parsed.calibrationRmsPx < 0.0 || parsed.inputViewCount <= 0 ||
        parsed.acceptedViewCount <= 0 || parsed.rejectedViewCount < 0 ||
        parsed.acceptedViewCount + parsed.rejectedViewCount != parsed.inputViewCount) {
        setError("intrinsic YAML quality statistics are inconsistent", error);
        return false;
    }
    parsed.ok = true;

    if (metadata) {
        *metadata = IntrinsicsYamlMetadata();
        optionalLookup(values, "camera_name", &metadata->cameraName);
        optionalLookup(values, "metadata.camera_model", &metadata->cameraModel);
        optionalLookup(values, "metadata.camera_serial", &metadata->cameraSerial);
        optionalLookup(values, "metadata.frame_id", &metadata->frameId);
        optionalLookup(values, "metadata.pixel_format", &metadata->pixelFormat);
        optionalLookup(values, "metadata.board.printed_pattern_sha256",
                       &metadata->printedPatternSha256);
        optionalLookup(values, "metadata.generated_at", &metadata->generatedAt);
    }
    *calibration = parsed;
    return true;
}

bool saveLaserPlaneYaml(const std::string& path,
                        const LaserPlaneFitResult& fit,
                        const BoardSpec& board,
                        const LaserPlaneYamlMetadata& metadata,
                        std::string* error) {
    std::string validationError;
    const double normalNorm = cv::norm(fit.plane.normal);
    if (!fit.ok || !validateBoardSpec(board, &validationError) ||
        !finiteNumber(normalNorm) || std::fabs(normalNorm - 1.0) > 1.0e-6 ||
        !finiteNumber(fit.plane.dMm)) {
        setError("cannot save invalid laser-plane calibration: " + validationError, error);
        return false;
    }
    std::ofstream output(path.c_str());
    if (!output) {
        setError("cannot create laser-plane YAML file: " + path, error);
        return false;
    }
    output << std::setprecision(17);
    output << "schema_version: 1\n";
    output << "calibration_type: \"line_laser_plane\"\n";
    output << "camera_frame: " << yamlQuote(metadata.cameraFrame) << '\n';
    output << "coordinate_convention: \"x_right_y_down_z_forward\"\n";
    output << "length_unit: \"mm\"\n\n";
    output << "plane:\n";
    output << "  equation: \"nx*x + ny*y + nz*z + d = 0\"\n";
    output << "  coefficients: ";
    writeArray(output, std::vector<double>{
        fit.plane.normal[0], fit.plane.normal[1],
        fit.plane.normal[2], fit.plane.dMm});
    output << '\n';
    output << "  normal_norm: " << normalNorm << '\n';
    output << "  sign_convention: \"d_nonpositive\"\n\n";
    output << "intrinsics:\n";
    output << "  file: " << yamlQuote(metadata.intrinsicsFile) << '\n';
    output << "  sha256: " << yamlQuote(metadata.intrinsicsSha256) << "\n\n";
    output << "board:\n";
    writeBoard(output, "  ", board, metadata.printedPatternSha256);
    output << "\nfit:\n";
    output << "  method: \"balanced_cross_pose_ransac_weighted_svd\"\n";
    output << "  pose_count: " << fit.poseCount << '\n';
    output << "  point_count: " << fit.pointCount << '\n';
    output << "  inlier_count: " << fit.inlierCount << '\n';
    output << "  inlier_ratio: " << fit.inlierRatio << '\n';
    output << "  inlier_mean_mm: " << fit.inlierDistanceMm.mean << '\n';
    output << "  inlier_rms_mm: " << fit.inlierDistanceMm.rms << '\n';
    output << "  inlier_p95_mm: " << fit.inlierDistanceMm.p95 << '\n';
    output << "  inlier_max_mm: " << fit.inlierDistanceMm.maximum << "\n\n";
    output << "validity:\n";
    output << "  camera_z_min_mm: " << metadata.validCameraZMinMm << '\n';
    output << "  camera_z_max_mm: " << metadata.validCameraZMaxMm << "\n\n";
    output << "dataset:\n";
    output << "  manifest_file: " << yamlQuote(metadata.datasetManifest) << '\n';
    output << "  manifest_sha256: " << yamlQuote(metadata.datasetManifestSha256) << "\n\n";
    output << "opencv_version: \"" << CV_VERSION << "\"\n";
    output << "generated_at: " << yamlQuote(metadata.generatedAt) << '\n';
    if (!output) {
        setError("failed while writing laser-plane YAML file: " + path, error);
        return false;
    }
    return true;
}

bool loadLaserPlaneYaml(const std::string& path,
                        LaserPlaneFitResult* fit,
                        BoardSpec* board,
                        LaserPlaneYamlMetadata* metadata,
                        std::string* error) {
    if (!fit) {
        setError("laser-plane fit output is null", error);
        return false;
    }
    std::map<std::string, std::string> values;
    if (!parsePlainYaml(path, &values, error)) {
        return false;
    }
    int schemaVersion = 0;
    std::string calibrationType;
    std::string coordinateConvention;
    std::string lengthUnit;
    std::string boardType;
    std::string patternConvention;
    if (!requiredInt(values, "schema_version", &schemaVersion, error) ||
        !lookup(values, "calibration_type", &calibrationType, error) ||
        !lookup(values, "coordinate_convention", &coordinateConvention, error) ||
        !lookup(values, "length_unit", &lengthUnit, error) ||
        !lookup(values, "board.type", &boardType, error) ||
        !lookup(values, "board.opencv_pattern", &patternConvention, error)) {
        return false;
    }
    if (schemaVersion != 1 || calibrationType != "line_laser_plane" ||
        coordinateConvention != "x_right_y_down_z_forward" ||
        lengthUnit != "mm" || boardType != "charuco" ||
        patternConvention != "legacy_pre_4_6") {
        setError("unsupported or inconsistent laser-plane YAML schema/model", error);
        return false;
    }
    std::vector<double> coefficients;
    if (!requiredDoubleList(values, "plane.coefficients", 4, &coefficients, error)) {
        return false;
    }
    cv::Vec3d normal(coefficients[0], coefficients[1], coefficients[2]);
    const double norm = cv::norm(normal);
    if (!finiteNumber(norm) || std::fabs(norm - 1.0) > 1.0e-6) {
        setError("laser-plane normal must be finite and unit length", error);
        return false;
    }
    double d = coefficients[3];
    normal *= 1.0 / norm;
    d /= norm;
    if (d > 0.0) {
        normal *= -1.0;
        d *= -1.0;
    }
    LaserPlaneFitResult parsed;
    parsed.plane.normal = normal;
    parsed.plane.dMm = d;
    if (!requiredInt(values, "fit.pose_count", &parsed.poseCount, error) ||
        !requiredInt(values, "fit.point_count", &parsed.pointCount, error) ||
        !requiredInt(values, "fit.inlier_count", &parsed.inlierCount, error) ||
        !requiredDouble(values, "fit.inlier_ratio", &parsed.inlierRatio, error) ||
        !requiredDouble(values, "fit.inlier_mean_mm", &parsed.inlierDistanceMm.mean, error) ||
        !requiredDouble(values, "fit.inlier_rms_mm", &parsed.inlierDistanceMm.rms, error) ||
        !requiredDouble(values, "fit.inlier_p95_mm", &parsed.inlierDistanceMm.p95, error) ||
        !requiredDouble(values, "fit.inlier_max_mm", &parsed.inlierDistanceMm.maximum, error)) {
        return false;
    }
    if (parsed.poseCount <= 0 || parsed.pointCount <= 0 || parsed.inlierCount <= 0 ||
        parsed.inlierCount > parsed.pointCount || parsed.inlierRatio < 0.0 ||
        parsed.inlierRatio > 1.0 || parsed.inlierDistanceMm.mean < 0.0 ||
        parsed.inlierDistanceMm.rms < 0.0 || parsed.inlierDistanceMm.p95 < 0.0 ||
        parsed.inlierDistanceMm.maximum < 0.0) {
        setError("laser-plane YAML quality statistics are inconsistent", error);
        return false;
    }

    BoardSpec parsedBoard;
    if (!loadBoard(values, "board", &parsedBoard, error)) {
        return false;
    }
    if (board) {
        *board = parsedBoard;
    }
    if (metadata) {
        *metadata = LaserPlaneYamlMetadata();
        optionalLookup(values, "camera_frame", &metadata->cameraFrame);
        optionalLookup(values, "intrinsics.file", &metadata->intrinsicsFile);
        optionalLookup(values, "intrinsics.sha256", &metadata->intrinsicsSha256);
        optionalLookup(values, "board.printed_pattern_sha256",
                       &metadata->printedPatternSha256);
        optionalLookup(values, "dataset.manifest_file", &metadata->datasetManifest);
        optionalLookup(values, "dataset.manifest_sha256",
                       &metadata->datasetManifestSha256);
        optionalLookup(values, "generated_at", &metadata->generatedAt);
        std::string value;
        if (optionalLookup(values, "validity.camera_z_min_mm", &value)) {
            parseDouble(value, &metadata->validCameraZMinMm);
        }
        if (optionalLookup(values, "validity.camera_z_max_mm", &value)) {
            parseDouble(value, &metadata->validCameraZMaxMm);
        }
    }
    parsed.ok = true;
    *fit = parsed;
    return true;
}

}  // namespace hik_calibration
