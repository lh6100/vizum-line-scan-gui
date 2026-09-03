#include "stereo/core/StereoDepthEngine.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace hik_stereo {
namespace {

void setError(const std::string& value, std::string* error) {
    if (error) *error = value;
}

int roundUp16(int value) {
    return std::max(16, ((value + 15) / 16) * 16);
}

cv::Ptr<cv::StereoSGBM> createMatcher(int minimumDisparity,
                                      int numberOfDisparities,
                                      const StereoDepthOptions& options) {
    cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(
        minimumDisparity, numberOfDisparities, options.blockSize);
    matcher->setP1(8 * options.blockSize * options.blockSize);
    matcher->setP2(32 * options.blockSize * options.blockSize);
    matcher->setPreFilterCap(31);
    matcher->setUniquenessRatio(options.uniquenessRatio);
    matcher->setSpeckleWindowSize(options.speckleWindowSize);
    matcher->setSpeckleRange(options.speckleRange);
    matcher->setDisp12MaxDiff(options.leftRightMaximumDifferencePx);
    matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    return matcher;
}

cv::Mat colorizeFloat(const cv::Mat& values,
                      const cv::Mat& validMask,
                      double low,
                      double high,
                      bool invert) {
    cv::Mat normalized(values.size(), CV_8UC1, cv::Scalar(0));
    if (high > low) {
        for (int y = 0; y < values.rows; ++y) {
            const float* source = values.ptr<float>(y);
            const uchar* valid = validMask.ptr<uchar>(y);
            uchar* target = normalized.ptr<uchar>(y);
            for (int x = 0; x < values.cols; ++x) {
                if (!valid[x] || !std::isfinite(source[x])) continue;
                double ratio = (source[x] - low) / (high - low);
                ratio = std::max(0.0, std::min(1.0, ratio));
                if (invert) ratio = 1.0 - ratio;
                target[x] = static_cast<uchar>(std::llround(ratio * 255.0));
            }
        }
    }
    cv::Mat color;
    cv::applyColorMap(normalized, color, cv::COLORMAP_TURBO);
    color.setTo(cv::Scalar(0, 0, 0), validMask == 0);
    return color;
}

}  // namespace

bool StereoDepthEngine::configure(const StereoRigCalibration& rig,
                                  const StereoDepthOptions& options,
                                  std::string* error) {
    configured_ = false;
    if (options.minimumDepthMm < 100.0 ||
        options.maximumDepthMm <= options.minimumDepthMm ||
        options.maximumDepthMm > 20000.0 ||
        options.blockSize < 3 || options.blockSize > 21 ||
        options.blockSize % 2 == 0 ||
        options.uniquenessRatio < 0 || options.uniquenessRatio > 50 ||
        options.leftRightMaximumDifferencePx < 0 ||
        options.leftRightMaximumDifferencePx > 10 ||
        options.disparityMarginPx < 0 ||
        options.maximumNumDisparities < 16 ||
        !std::isfinite(options.claheClipLimit) ||
        options.claheClipLimit < 1.0 || options.claheClipLimit > 40.0) {
        setError("stereo depth options are invalid", error);
        return false;
    }
    StereoProcessingGeometry geometry;
    std::string detail;
    if (!prepareStereoProcessingGeometry(
            rig, options.processingSize, &geometry, &detail)) {
        setError(detail, error);
        return false;
    }
    cv::Mat leftR;
    cv::Mat rightR;
    cv::Mat leftP;
    cv::Mat rightP;
    cv::Mat q;
    cv::Rect leftValid;
    cv::Rect rightValid;
    try {
        cv::stereoRectify(
            geometry.leftCameraMatrix,
            rig.left.intrinsics.distCoeffs,
            geometry.rightCameraMatrix,
            rig.right.intrinsics.distCoeffs,
            geometry.outputSize,
            rig.rotationRightFromLeft,
            rig.translationRightFromLeft,
            leftR, rightR, leftP, rightP, q,
            cv::CALIB_ZERO_DISPARITY, 0.0,
            geometry.outputSize, &leftValid, &rightValid);
        cv::initUndistortRectifyMap(
            geometry.leftCameraMatrix,
            rig.left.intrinsics.distCoeffs,
            leftR, leftP, geometry.outputSize, CV_32FC1,
            leftMapX_, leftMapY_);
        cv::initUndistortRectifyMap(
            geometry.rightCameraMatrix,
            rig.right.intrinsics.distCoeffs,
            rightR, rightP, geometry.outputSize, CV_32FC1,
            rightMapX_, rightMapY_);
    } catch (const cv::Exception& exception) {
        setError(std::string("stereo rectification failed: ") +
                 exception.what(), error);
        return false;
    }
    const double projectedBaselineTimesFocal = -rightP.at<double>(0, 3);
    if (!std::isfinite(projectedBaselineTimesFocal) ||
        std::abs(projectedBaselineTimesFocal) < 1.0) {
        setError("rectified stereo baseline is degenerate", error);
        return false;
    }
    std::vector<StereoDepthBand> requestedBands = options.depthBands;
    if (requestedBands.empty()) {
        requestedBands.push_back(
            StereoDepthBand{options.minimumDepthMm, options.maximumDepthMm});
    }
    bands_.clear();
    totalBandDisparities_ = 0;
    int envelopeMinimum = std::numeric_limits<int>::max();
    int envelopeMaximum = std::numeric_limits<int>::min();
    double previousMaximum = options.minimumDepthMm;
    for (std::size_t index = 0; index < requestedBands.size(); ++index) {
        const StereoDepthBand& requested = requestedBands[index];
        if (!std::isfinite(requested.minimumDepthMm) ||
            !std::isfinite(requested.maximumDepthMm) ||
            requested.minimumDepthMm < options.minimumDepthMm ||
            requested.maximumDepthMm > options.maximumDepthMm ||
            requested.maximumDepthMm <= requested.minimumDepthMm ||
            (index > 0U && requested.minimumDepthMm > previousMaximum + 1.0e-6)) {
            setError("stereo depth bands are invalid, outside the global range, or leave a gap", error);
            return false;
        }
        const double disparityNear = projectedBaselineTimesFocal /
                                     requested.minimumDepthMm;
        const double disparityFar = projectedBaselineTimesFocal /
                                    requested.maximumDepthMm;
        const double lower = std::min(disparityNear, disparityFar);
        const double upper = std::max(disparityNear, disparityFar);
        ConfiguredBand configured;
        configured.minimumDepthMm = requested.minimumDepthMm;
        configured.maximumDepthMm = requested.maximumDepthMm;
        configured.minimumDisparity = static_cast<int>(std::floor(lower)) -
                                      options.disparityMarginPx;
        configured.numberOfDisparities = roundUp16(
            static_cast<int>(std::ceil(upper)) -
            configured.minimumDisparity + options.disparityMarginPx + 1);
        if (configured.numberOfDisparities > options.maximumNumDisparities ||
            configured.numberOfDisparities >= geometry.outputSize.width) {
            setError("a stereo depth band exceeds the configured disparity budget or image width", error);
            return false;
        }
        configured.validDisparityRoi = cv::getValidDisparityROI(
            leftValid, rightValid, configured.minimumDisparity,
            configured.numberOfDisparities, options.blockSize);
        if (configured.validDisparityRoi.area() <= 0) {
            setError("stereo rectification leaves no valid disparity ROI for a depth band", error);
            return false;
        }
        envelopeMinimum = std::min(
            envelopeMinimum, configured.minimumDisparity);
        envelopeMaximum = std::max(
            envelopeMaximum,
            configured.minimumDisparity + configured.numberOfDisparities - 1);
        totalBandDisparities_ += configured.numberOfDisparities;
        previousMaximum = std::max(previousMaximum, requested.maximumDepthMm);
        bands_.push_back(configured);
    }
    if (requestedBands.front().minimumDepthMm > options.minimumDepthMm + 1.0e-6 ||
        previousMaximum < options.maximumDepthMm - 1.0e-6) {
        setError("stereo depth bands do not cover the full global depth range", error);
        return false;
    }
    minimumDisparity_ = envelopeMinimum;
    numberOfDisparities_ = envelopeMaximum - envelopeMinimum + 1;
    options_ = options;
    geometry_ = geometry;
    reprojectionMatrix_ = q;
    configured_ = true;
    return true;
}

bool StereoDepthEngine::compute(const cv::Mat& leftGray,
                                const cv::Mat& rightGray,
                                StereoDepthResult* result) const {
    if (!result) return false;
    *result = StereoDepthResult();
    if (!configured_) {
        result->error = "stereo depth engine is not configured";
        return false;
    }
    if (leftGray.empty() || rightGray.empty() ||
        leftGray.type() != CV_8UC1 || rightGray.type() != CV_8UC1) {
        result->error = "stereo input images must be non-empty Mono8 matrices";
        return false;
    }
    const auto started = std::chrono::steady_clock::now();
    cv::Mat leftPrepared;
    cv::Mat rightPrepared;
    std::string detail;
    if (!cropAndResizeStereoImage(
            leftGray, geometry_.leftCrop, geometry_.outputSize,
            &leftPrepared, &detail) ||
        !cropAndResizeStereoImage(
            rightGray, geometry_.rightCrop, geometry_.outputSize,
            &rightPrepared, &detail)) {
        result->error = detail;
        return false;
    }
    try {
        cv::remap(leftPrepared, result->rectifiedLeft,
                  leftMapX_, leftMapY_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
        cv::remap(rightPrepared, result->rectifiedRight,
                  rightMapX_, rightMapY_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
        cv::Mat leftMatch = result->rectifiedLeft;
        cv::Mat rightMatch = result->rectifiedRight;
        cv::Mat leftEnhanced;
        cv::Mat rightEnhanced;
        if (options_.enableClahe) {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
                options_.claheClipLimit, cv::Size(8, 8));
            clahe->apply(result->rectifiedLeft, leftEnhanced);
            clahe->apply(result->rectifiedRight, rightEnhanced);
            leftMatch = leftEnhanced;
            rightMatch = rightEnhanced;
        }

        const float nan = std::numeric_limits<float>::quiet_NaN();
        result->disparityPx = cv::Mat(
            geometry_.outputSize, CV_32FC1, cv::Scalar(nan));
        result->xyzLeftMm = cv::Mat(
            geometry_.outputSize, CV_32FC3, cv::Scalar(nan, nan, nan));
        result->validMask = cv::Mat::zeros(
            geometry_.outputSize, CV_8UC1);
        result->confidence = cv::Mat::zeros(
            geometry_.outputSize, CV_32FC1);

        for (const ConfiguredBand& band : bands_) {
            cv::Mat disparityFixed;
            createMatcher(
                band.minimumDisparity,
                band.numberOfDisparities, options_)
                ->compute(leftMatch, rightMatch, disparityFixed);
            cv::Mat disparity;
            disparityFixed.convertTo(disparity, CV_32F, 1.0 / 16.0);
            cv::Mat bandValid = cv::Mat::zeros(
                geometry_.outputSize, CV_8UC1);
            bandValid(band.validDisparityRoi).setTo(255);
            const float minimumValid = static_cast<float>(
                band.minimumDisparity);
            const float maximumValid = static_cast<float>(
                band.minimumDisparity + band.numberOfDisparities - 1);
            bandValid.setTo(
                0, (disparity <= minimumValid) |
                   (disparity > maximumValid));

            cv::Mat rightDisparity;
            if (options_.enableLeftRightCheck) {
                const int rightMinimum = -(
                    band.minimumDisparity +
                    band.numberOfDisparities - 1);
                cv::Mat rightFixed;
                createMatcher(
                    rightMinimum, band.numberOfDisparities, options_)
                    ->compute(rightMatch, leftMatch, rightFixed);
                rightFixed.convertTo(
                    rightDisparity, CV_32F, 1.0 / 16.0);
            }

            cv::Mat candidateXyz;
            cv::reprojectImageTo3D(
                disparity, candidateXyz,
                reprojectionMatrix_, false, CV_32F);
            for (int y = 0; y < disparity.rows; ++y) {
                const float* candidateDisparity = disparity.ptr<float>(y);
                const cv::Vec3f* candidatePoints =
                    candidateXyz.ptr<cv::Vec3f>(y);
                const uchar* candidateMask = bandValid.ptr<uchar>(y);
                const uchar* leftPixels = leftMatch.ptr<uchar>(y);
                const uchar* rightPixels = rightMatch.ptr<uchar>(y);
                const float* rightValues = options_.enableLeftRightCheck
                    ? rightDisparity.ptr<float>(y) : nullptr;
                float* mergedDisparity = result->disparityPx.ptr<float>(y);
                cv::Vec3f* mergedPoints = result->xyzLeftMm.ptr<cv::Vec3f>(y);
                uchar* mergedMask = result->validMask.ptr<uchar>(y);
                float* mergedConfidence = result->confidence.ptr<float>(y);
                for (int x = 0; x < disparity.cols; ++x) {
                    if (!candidateMask[x] ||
                        !std::isfinite(candidateDisparity[x])) continue;
                    const int rightX = static_cast<int>(
                        std::llround(x - candidateDisparity[x]));
                    if (rightX < 0 || rightX >= disparity.cols) continue;
                    float leftRightConfidence = 1.0F;
                    if (rightValues) {
                        const double residual = std::abs(
                            candidateDisparity[x] + rightValues[rightX]);
                        if (!std::isfinite(residual) ||
                            residual > options_.leftRightMaximumDifferencePx) {
                            continue;
                        }
                        leftRightConfidence = static_cast<float>(std::max(
                            0.0, 1.0 - residual /
                                std::max(1, options_.leftRightMaximumDifferencePx)));
                    }
                    const cv::Vec3f& point = candidatePoints[x];
                    const float depth = point[2];
                    if (!std::isfinite(point[0]) ||
                        !std::isfinite(point[1]) ||
                        !std::isfinite(depth) ||
                        depth < band.minimumDepthMm ||
                        depth > band.maximumDepthMm) continue;
                    const double photometricResidual = std::abs(
                        static_cast<int>(leftPixels[x]) -
                        static_cast<int>(rightPixels[rightX]));
                    const float photometricConfidence = static_cast<float>(
                        std::max(0.0, 1.0 - photometricResidual / 255.0));
                    const float candidateConfidence =
                        (0.25F + 0.75F * photometricConfidence) *
                        leftRightConfidence;
                    const bool replace = !mergedMask[x] ||
                        candidateConfidence > mergedConfidence[x] + 0.02F ||
                        (std::abs(candidateConfidence - mergedConfidence[x]) <= 0.02F &&
                         depth < mergedPoints[x][2]);
                    if (!replace) continue;
                    mergedDisparity[x] = candidateDisparity[x];
                    mergedPoints[x] = point;
                    mergedMask[x] = 255;
                    mergedConfidence[x] = candidateConfidence;
                }
            }
        }
    } catch (const cv::Exception& exception) {
        result->error = std::string("stereo matching failed: ") +
                        exception.what();
        return false;
    }

    std::vector<float> depths;
    depths.reserve(result->xyzLeftMm.total() / 2U);
    for (int y = 0; y < result->xyzLeftMm.rows; ++y) {
        const cv::Vec3f* xyz = result->xyzLeftMm.ptr<cv::Vec3f>(y);
        uchar* valid = result->validMask.ptr<uchar>(y);
        for (int x = 0; x < result->xyzLeftMm.cols; ++x) {
            if (!valid[x]) continue;
            const float depth = xyz[x][2];
            if (!std::isfinite(xyz[x][0]) ||
                !std::isfinite(xyz[x][1]) ||
                !std::isfinite(depth) ||
                depth < options_.minimumDepthMm ||
                depth > options_.maximumDepthMm) {
                valid[x] = 0;
                continue;
            }
            depths.push_back(depth);
        }
    }
    if (!depths.empty()) {
        const std::size_t middle = depths.size() / 2U;
        std::nth_element(depths.begin(), depths.begin() + middle,
                         depths.end());
        result->statistics.medianDepthMm = depths[middle];
    }
    cv::Mat depth(result->xyzLeftMm.size(), CV_32FC1);
    for (int y = 0; y < depth.rows; ++y) {
        const cv::Vec3f* xyz = result->xyzLeftMm.ptr<cv::Vec3f>(y);
        float* target = depth.ptr<float>(y);
        for (int x = 0; x < depth.cols; ++x) target[x] = xyz[x][2];
    }
    result->disparityPreviewBgr = colorizeFloat(
        result->disparityPx, result->validMask,
        minimumDisparity_, minimumDisparity_ + numberOfDisparities_, false);
    result->depthPreviewBgr = colorizeFloat(
        depth, result->validMask,
        options_.minimumDepthMm, options_.maximumDepthMm, true);
    result->statistics.inputPixels = result->validMask.total();
    result->statistics.validPixels = static_cast<std::uint64_t>(
        cv::countNonZero(result->validMask));
    result->statistics.validFraction =
        result->statistics.inputPixels > 0U
            ? static_cast<double>(result->statistics.validPixels) /
                  result->statistics.inputPixels
            : 0.0;
    result->statistics.minimumDisparity = minimumDisparity_;
    result->statistics.numberOfDisparities = numberOfDisparities_;
    result->statistics.bandCount = static_cast<int>(bands_.size());
    result->statistics.totalBandDisparities = totalBandDisparities_;
    result->statistics.processingMs =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count();
    result->ok = true;
    return true;
}

}  // namespace hik_stereo
