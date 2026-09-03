#include "StripeCenterlineExtractor.h"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

#define CHECK_TRUE(condition, message)                                           \
    do {                                                                         \
        if (!(condition)) {                                                       \
            std::cerr << "FAIL: " << (message) << '\n';                          \
            ++gFailures;                                                         \
        }                                                                         \
    } while (false)

struct SyntheticFrame {
    cv::Mat response;
    cv::Mat raw;

    SyntheticFrame(int rows, int columns, int responseBase = 4, int rawBase = 24)
        : response(rows, columns, CV_8UC1, cv::Scalar(responseBase)),
          raw(rows, columns, CV_8UC1, cv::Scalar(rawBase)) {}
};

unsigned char saturatedAdd(unsigned char value, double addition) {
    return static_cast<unsigned char>(std::max(
        0.0, std::min(255.0, static_cast<double>(value) + addition)));
}

void addHorizontalGaussian(SyntheticFrame* frame,
                           double centerY,
                           double amplitude,
                           double sigma,
                           int firstColumn = 0,
                           int lastColumn = -1) {
    if (!frame || sigma <= 0.0) {
        return;
    }
    const int begin = std::max(0, firstColumn);
    const int end = lastColumn < 0
        ? frame->response.cols : std::min(frame->response.cols, lastColumn);
    for (int column = begin; column < end; ++column) {
        for (int row = 0; row < frame->response.rows; ++row) {
            const double distance = (static_cast<double>(row) - centerY) / sigma;
            const double signal = amplitude * std::exp(-0.5 * distance * distance);
            frame->response.at<unsigned char>(row, column) = saturatedAdd(
                frame->response.at<unsigned char>(row, column), signal);
            frame->raw.at<unsigned char>(row, column) = saturatedAdd(
                frame->raw.at<unsigned char>(row, column), signal);
        }
    }
}

void addVerticalGaussian(SyntheticFrame* frame,
                         double centerX,
                         double amplitude,
                         double sigma,
                         int firstRow = 0,
                         int lastRow = -1) {
    if (!frame || sigma <= 0.0) {
        return;
    }
    const int begin = std::max(0, firstRow);
    const int end = lastRow < 0
        ? frame->response.rows : std::min(frame->response.rows, lastRow);
    for (int row = begin; row < end; ++row) {
        for (int column = 0; column < frame->response.cols; ++column) {
            const double distance = (static_cast<double>(column) - centerX) / sigma;
            const double signal = amplitude * std::exp(-0.5 * distance * distance);
            frame->response.at<unsigned char>(row, column) = saturatedAdd(
                frame->response.at<unsigned char>(row, column), signal);
            frame->raw.at<unsigned char>(row, column) = saturatedAdd(
                frame->raw.at<unsigned char>(row, column), signal);
        }
    }
}

void addHorizontalPlateau(SyntheticFrame* frame,
                          int firstRow,
                          int lastRowInclusive,
                          int value,
                          int firstColumn = 0,
                          int lastColumn = -1) {
    if (!frame || firstRow > lastRowInclusive) {
        return;
    }
    const int beginColumn = std::max(0, firstColumn);
    const int endColumn = lastColumn < 0
        ? frame->response.cols : std::min(frame->response.cols, lastColumn);
    const int beginRow = std::max(0, firstRow);
    const int endRow = std::min(frame->response.rows - 1, lastRowInclusive);
    for (int column = beginColumn; column < endColumn; ++column) {
        for (int row = beginRow; row <= endRow; ++row) {
            frame->response.at<unsigned char>(row, column) =
                static_cast<unsigned char>(value);
            frame->raw.at<unsigned char>(row, column) =
                static_cast<unsigned char>(value);
        }
    }
}

hik_stripe::Options horizontalOptions(const cv::Size& imageSize) {
    hik_stripe::Options options;
    options.orientation = hik_stripe::Orientation::Horizontal;
    options.roi = cv::Rect(0, 0, imageSize.width, imageSize.height);
    return options;
}

hik_stripe::Options verticalOptions(const cv::Size& imageSize) {
    hik_stripe::Options options;
    options.orientation = hik_stripe::Orientation::Vertical;
    options.roi = cv::Rect(0, 0, imageSize.width, imageSize.height);
    return options;
}

hik_stripe::Result extract(const SyntheticFrame& frame,
                           const hik_stripe::Options& options,
                           const cv::Mat& mask = cv::Mat(),
                           bool expectSuccess = true) {
    hik_stripe::Result result;
    const bool returned = hik_stripe::extractCenterline(
        frame.response, frame.raw, options, &result, mask);
    CHECK_TRUE(returned == result.ok,
               "extractCenterline return value and Result::ok must agree");
    if (!returned && expectSuccess) {
        std::cerr << "extractCenterline error: " << result.error << '\n';
    }
    return result;
}

double coordinateNormalToStripe(const hik_stripe::Candidate& candidate,
                                hik_stripe::Orientation orientation) {
    return orientation == hik_stripe::Orientation::Horizontal
        ? candidate.pixel.y : candidate.pixel.x;
}

double maximumCenterError(const hik_stripe::Result& result, double expected) {
    double maximum = 0.0;
    for (const hik_stripe::Candidate& point : result.selected) {
        maximum = std::max(maximum, std::fabs(
            coordinateNormalToStripe(point, result.orientation) - expected));
    }
    return maximum;
}

std::size_t selectedInScanRange(const hik_stripe::Result& result,
                                int first,
                                int lastInclusive) {
    return static_cast<std::size_t>(std::count_if(
        result.selected.begin(), result.selected.end(),
        [first, lastInclusive](const hik_stripe::Candidate& point) {
            return point.scanIndex >= first && point.scanIndex <= lastInclusive;
        }));
}

bool anyCandidateHas(const hik_stripe::Result& result,
                     hik_stripe::RejectReason reason) {
    return std::any_of(
        result.candidates.begin(), result.candidates.end(),
        [reason](const hik_stripe::Candidate& candidate) {
            return hik_stripe::hasRejectReason(candidate, reason);
        });
}

bool anyProvisionalHas(const hik_stripe::Result& result,
                       hik_stripe::RejectReason reason) {
    return std::any_of(
        result.provisionalSelected.begin(),
        result.provisionalSelected.end(),
        [reason](const hik_stripe::Candidate& candidate) {
            return hik_stripe::hasRejectReason(candidate, reason);
        });
}

bool branchIsOrderedAndTagged(
        const hik_stripe::MultipathInterval& interval,
        const hik_stripe::MultipathBranch& branch) {
    int previousScan = std::numeric_limits<int>::min();
    for (const hik_stripe::Candidate& candidate : branch.candidates) {
        if (candidate.scanIndex <= previousScan ||
            candidate.scanIndex < interval.firstScanIndex ||
            candidate.scanIndex > interval.lastScanIndex ||
            candidate.ambiguityIntervalId != interval.intervalId ||
            candidate.ambiguityBranchId != branch.branchId ||
            !hik_stripe::hasRejectReason(
                candidate,
                hik_stripe::REJECT_AMBIGUOUS_MULTIPATH)) {
            return false;
        }
        previousScan = candidate.scanIndex;
    }
    return !branch.candidates.empty();
}

bool allSelectedFinite(const hik_stripe::Result& result) {
    return std::all_of(
        result.selected.begin(), result.selected.end(),
        [](const hik_stripe::Candidate& candidate) {
            return std::isfinite(candidate.pixel.x) &&
                   std::isfinite(candidate.pixel.y) &&
                   std::isfinite(candidate.centerSigmaPx) &&
                   std::isfinite(candidate.taylorOffsetPx) &&
                   std::isfinite(candidate.smoothedFirstDerivative) &&
                   std::isfinite(candidate.smoothedSecondDerivative);
        });
}

bool allSelectedPublishable(const hik_stripe::Result& result) {
    return std::all_of(
        result.selected.begin(), result.selected.end(),
        [](const hik_stripe::Candidate& candidate) {
            return candidate.accepted() &&
                   candidate.ambiguityIntervalId < 0 &&
                   candidate.ambiguityBranchId < 0;
        });
}

bool allSelectedUse(const hik_stripe::Result& result,
                    hik_stripe::CenterMethod method) {
    return !result.selected.empty() &&
           std::all_of(
               result.selected.begin(), result.selected.end(),
               [method](const hik_stripe::Candidate& candidate) {
                   return candidate.centerMethod == method;
               });
}

bool selectedDiagnosticsFiniteAndBounded(
        const hik_stripe::Diagnostics& diagnostics) {
    return std::isfinite(diagnostics.meanSelectedSnr) &&
           diagnostics.meanSelectedSnr >= 0.0 &&
           std::isfinite(
               diagnostics.meanSelectedGradientAsymmetry) &&
           diagnostics.meanSelectedGradientAsymmetry >= 0.0 &&
           diagnostics.meanSelectedGradientAsymmetry <= 1.0 &&
           std::isfinite(diagnostics.meanSelectedFitResidual) &&
           diagnostics.meanSelectedFitResidual >= 0.0 &&
           std::isfinite(
               diagnostics.meanSelectedSecondPeakRatio) &&
           diagnostics.meanSelectedSecondPeakRatio >= 0.0 &&
           diagnostics.meanSelectedSecondPeakRatio <= 1.0;
}

void testNonIntegerGaussianUsesDerivativeTaylorCenter() {
    SyntheticFrame frame(78, 104);
    const double expectedCenter = 37.37;
    addHorizontalGaussian(
        &frame, expectedCenter, 168.0, 1.15);

    hik_stripe::Options options =
        horizontalOptions(frame.response.size());
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok,
               "a non-integer unsaturated Gaussian ridge must be extracted");
    CHECK_TRUE(result.selected.size() >= 94,
               "the Gaussian ridge must cover at least 90% of scanlines");
    CHECK_TRUE(allSelectedFinite(result),
               "Taylor center diagnostics must never contain NaN or infinity");
    CHECK_TRUE(selectedDiagnosticsFiniteAndBounded(
                   result.diagnostics),
               "selected SNR/asymmetry/fit/multi-peak diagnostics must be finite and bounded");
    CHECK_TRUE(result.diagnostics.meanSelectedSnr >
                   options.thresholdMadScale,
               "a clean Gaussian ridge must report an SNR above the configured MAD scale");
    CHECK_TRUE(result.diagnostics.meanSelectedSecondPeakRatio <= 1e-12,
               "a single clean Gaussian ridge must report no competing second peak");
    CHECK_TRUE(allSelectedUse(
                   result,
                   hik_stripe::CenterMethod::GaussianDerivativeTaylor),
               "an unsaturated Gaussian ridge must use the Gaussian-derivative Taylor center");
    CHECK_TRUE(maximumCenterError(result, expectedCenter) <= 0.10,
               "the derivative Taylor center must recover a non-integer Gaussian center within 0.10 px");
    for (const hik_stripe::Candidate& candidate : result.selected) {
        CHECK_TRUE(candidate.smoothedSecondDerivative < 0.0,
                   "a Taylor-located bright ridge must have negative smoothed curvature");
        CHECK_TRUE(std::fabs(candidate.taylorOffsetPx) <= 0.5 + 1e-12,
                   "a Taylor center displacement must remain inside its source pixel");
    }
}

void testLocalThresholdTracksBrightnessRamp() {
    SyntheticFrame frame(72, 120, 0, 0);
    for (int column = 0; column < frame.response.cols; ++column) {
        const int baseline = 12 + (125 * column) / (frame.response.cols - 1);
        frame.response.col(column).setTo(baseline);
        frame.raw.col(column).setTo(std::min(190, baseline + 25));
    }
    addHorizontalGaussian(&frame, 36.25, 42.0, 1.15);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.minimumProminence = 12.0;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "local-threshold ramp extraction must succeed");
    CHECK_TRUE(result.selected.size() >= 108,
               "local threshold must retain at least 90% of a dim ridge over a ramp");
    CHECK_TRUE(maximumCenterError(result, 36.25) <= 0.35,
               "local background ramp must not bias the sub-pixel center");
}

void testContinuousDimPathBeatsShortBrightPeak() {
    SyntheticFrame frame(80, 120);
    addHorizontalGaussian(&frame, 45.2, 112.0, 1.20);
    addHorizontalGaussian(&frame, 19.0, 210.0, 1.15, 32, 84);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.maximumSecondPeakRatio = 1.0;
    options.pathPositionPenalty = std::max(2.0, options.pathPositionPenalty);
    options.pathMaximumStepPx = 4.0;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "continuous dim path extraction must succeed");
    CHECK_TRUE(result.selected.size() >= 108,
               "the continuous true path must be retained through a shorter ghost");
    CHECK_TRUE(maximumCenterError(result, 45.2) <= 0.45,
               "a shorter brighter peak must not pull the selected path off the true ridge");
}

void testBrighterParallelSaturatedGhostIsRejected() {
    SyntheticFrame frame(84, 110);
    addHorizontalGaussian(&frame, 54.0, 135.0, 1.15);
    addHorizontalGaussian(&frame, 20.0, 180.0, 3.2);
    addHorizontalPlateau(&frame, 17, 23, 255);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.maximumSaturatedPlateauWidthPx = 4;
    options.maximumSaturatedFraction = 0.45;
    options.maximumSecondPeakRatio = 1.0;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "parallel ghost extraction must complete");
    CHECK_TRUE(result.selected.size() >= 99,
               "the valid narrow ridge must survive a full-length bright ghost");
    CHECK_TRUE(maximumCenterError(result, 54.0) <= 0.35,
               "a saturated wide ghost must not become the selected centerline");
    CHECK_TRUE(anyCandidateHas(
                   result, hik_stripe::REJECT_SATURATED_WIDE_PLATEAU) ||
                   anyCandidateHas(
                       result, hik_stripe::REJECT_WIDTH_OUT_OF_RANGE),
               "the wide saturated ghost must carry an explicit rejection reason");
}

void testEqualParallelPathsAreMarkedAmbiguous() {
    SyntheticFrame frame(82, 100);
    addHorizontalGaussian(&frame, 28.0, 145.0, 1.25);
    addHorizontalGaussian(&frame, 52.0, 145.0, 1.25);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.maximumSecondPeakRatio = 1.0;
    options.pathAmbiguityMarginPerPoint =
        std::max(0.5, options.pathAmbiguityMarginPerPoint);
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "ambiguous-path extraction must complete");
    CHECK_TRUE(result.diagnostics.pathCostMarginPerPoint <=
                   options.pathAmbiguityMarginPerPoint + 1e-9,
               "equal parallel paths must have an ambiguity-sized cost margin");
    CHECK_TRUE(result.diagnostics.ambiguousPathPointCount > 0 ||
                   anyCandidateHas(result, hik_stripe::REJECT_PATH_AMBIGUOUS) ||
                   result.selected.empty(),
               "equal parallel ridges must be reported or rejected as path ambiguity");
    CHECK_TRUE(!result.multipathIntervals.empty(),
               "equal full-length ridges must expose an explicit multipath interval");
    if (!result.multipathIntervals.empty()) {
        const hik_stripe::MultipathInterval& interval =
            result.multipathIntervals.front();
        CHECK_TRUE(interval.leftBoundaryOpen &&
                       interval.rightBoundaryOpen &&
                       interval.firstScanIndex == 0 &&
                       interval.lastScanIndex ==
                           frame.response.cols - 1,
                   "a branch without common endpoints must be marked open and fail closed to both segment boundaries");
        CHECK_TRUE(result.selected.empty(),
                   "an open-ended equal multipath frame must publish no fabricated centerline");
    }
}

void testSymmetricNarrowPlateauUsesItsMiddle() {
    SyntheticFrame frame(70, 90);
    addHorizontalGaussian(&frame, 31.0, 120.0, 2.0);
    addHorizontalPlateau(&frame, 30, 32, 255);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.maximumSaturatedFraction = 1.0;
    options.maximumSaturatedPlateauWidthPx = 4;
    options.maximumGradientAsymmetry = 1.0;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "symmetric narrow plateau extraction must succeed");
    CHECK_TRUE(result.selected.size() >= 81,
               "a symmetric three-pixel plateau must remain measurable");
    CHECK_TRUE(maximumCenterError(result, 31.0) <= 0.25,
               "a symmetric plateau center must use both flanks");
    CHECK_TRUE(allSelectedFinite(result),
               "the saturated midpoint path must remain finite");
    CHECK_TRUE(allSelectedUse(
                   result,
                   hik_stripe::CenterMethod::SaturatedHalfHeightMidpoint),
               "a narrow saturated plateau must explicitly use the two-wing half-height midpoint");
}

void testNarrowAsymmetricSaturatedRidgeIsRejected() {
    SyntheticFrame frame(72, 90);
    const int rows[] = {26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};
    const int values[] = {25, 50, 90, 150, 255, 255, 230, 190, 145, 105, 60};
    for (int column = 0; column < frame.response.cols; ++column) {
        for (std::size_t index = 0U;
             index < sizeof(rows) / sizeof(rows[0]); ++index) {
            frame.response.at<unsigned char>(rows[index], column) =
                static_cast<unsigned char>(values[index]);
            frame.raw.at<unsigned char>(rows[index], column) =
                static_cast<unsigned char>(values[index]);
        }
    }

    hik_stripe::Options options =
        horizontalOptions(frame.response.size());
    // Keep the plateau-width gate unchanged and isolate the asymmetric-wing
    // decision with a stricter (not looser) asymmetry limit.
    options.maximumSaturatedFraction = 1.0;
    options.maximumGradientAsymmetry = 0.05;
    const hik_stripe::Result result =
        extract(frame, options, cv::Mat(), false);

    CHECK_TRUE(!result.ok,
               "a frame containing only an asymmetric saturated ridge must fail closed");
    CHECK_TRUE(result.selected.empty(),
               "an asymmetric saturated ridge must not enter the optimized path");
    CHECK_TRUE(anyCandidateHas(
                   result, hik_stripe::REJECT_SATURATED_ASYMMETRIC),
               "the asymmetric saturated ridge must retain its stable rejection reason");
    for (const hik_stripe::Candidate& candidate : result.candidates) {
        if (candidate.saturatedFraction > 0.0) {
            CHECK_TRUE(
                candidate.centerMethod ==
                    hik_stripe::CenterMethod::SaturatedHalfHeightMidpoint,
                "even a rejected saturated candidate must use the two-wing midpoint, never Taylor curvature");
            CHECK_TRUE(std::isfinite(candidate.pixel.x) &&
                           std::isfinite(candidate.pixel.y),
                       "a rejected saturated candidate must still expose finite diagnostics");
        }
    }
}

void testWideAsymmetricPlateauIsRejected() {
    SyntheticFrame frame(76, 90);
    addHorizontalPlateau(&frame, 24, 35, 255);
    for (int column = 0; column < frame.response.cols; ++column) {
        frame.response.at<unsigned char>(36, column) = 120;
        frame.raw.at<unsigned char>(36, column) = 144;
    }

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.maximumSaturatedPlateauWidthPx = 4;
    options.maximumSaturatedFraction = 0.40;
    options.maximumFwhmPx = std::min(9.0, options.maximumFwhmPx);
    const hik_stripe::Result result =
        extract(frame, options, cv::Mat(), false);

    CHECK_TRUE(!result.ok,
               "a frame containing only an invalid wide plateau must fail closed");
    CHECK_TRUE(result.selected.empty(),
               "a full-frame wide asymmetric saturated plateau must not be selected");
    CHECK_TRUE(anyCandidateHas(
                   result, hik_stripe::REJECT_SATURATED_WIDE_PLATEAU) ||
                   anyCandidateHas(
                       result, hik_stripe::REJECT_SATURATED_ASYMMETRIC) ||
                   anyCandidateHas(
                       result, hik_stripe::REJECT_WIDTH_OUT_OF_RANGE),
               "wide/asymmetric saturation must have a diagnostic rejection reason");
}

void testGapIsPreservedWithoutInterpolation() {
    SyntheticFrame frame(72, 120);
    addHorizontalGaussian(&frame, 38.0, 150.0, 1.15, 0, 48);
    addHorizontalGaussian(&frame, 38.0, 150.0, 1.15, 60, 120);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.pathMaximumGap = 16;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "gapped centerline extraction must complete");
    CHECK_TRUE(selectedInScanRange(result, 0, 47) >= 43,
               "the path before a real gap must be retained");
    CHECK_TRUE(selectedInScanRange(result, 60, 119) >= 54,
               "the path after a real gap must be retained");
    CHECK_TRUE(selectedInScanRange(result, 48, 59) == 0,
               "the extractor must preserve a real gap instead of fabricating centers");
    CHECK_TRUE(result.diagnostics.selectedGapCount >= 12,
               "diagnostics must count the deliberately missing scanlines");
}

void testLargeJumpDoesNotCreateAConnectingRamp() {
    SyntheticFrame frame(92, 100);
    addHorizontalGaussian(&frame, 24.0, 155.0, 1.15, 0, 48);
    addHorizontalGaussian(&frame, 67.0, 155.0, 1.15, 52, 100);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.pathMaximumStepPx = 4.0;
    options.pathMaximumGap = 2;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "large-jump extraction must complete");
    double largestSelectedStep = 0.0;
    for (std::size_t index = 1; index < result.selected.size(); ++index) {
        const hik_stripe::Candidate& previous = result.selected[index - 1];
        const hik_stripe::Candidate& current = result.selected[index];
        if (current.scanIndex == previous.scanIndex + 1) {
            largestSelectedStep = std::max(
                largestSelectedStep,
                std::fabs(current.pixel.y - previous.pixel.y));
        }
    }
    CHECK_TRUE(largestSelectedStep <= options.pathMaximumStepPx + 1e-9,
               "selected adjacent points must never bridge an impossible jump");
    CHECK_TRUE(result.selected.size() < 98 ||
                   anyCandidateHas(result, hik_stripe::REJECT_PATH_JUMP),
               "a large discontinuity must create a break or a path-jump rejection");
}

void testFixedHorizontalAndVerticalOrientations() {
    SyntheticFrame frame(88, 112);
    addHorizontalGaussian(&frame, 61.0, 145.0, 1.10);
    addVerticalGaussian(&frame, 24.0, 145.0, 1.10);

    hik_stripe::Options horizontal =
        horizontalOptions(frame.response.size());
    horizontal.maximumSecondPeakRatio = 1.0;
    const hik_stripe::Result horizontalResult = extract(frame, horizontal);
    CHECK_TRUE(horizontalResult.orientation ==
                   hik_stripe::Orientation::Horizontal,
               "explicit horizontal orientation must never auto-switch");
    CHECK_TRUE(horizontalResult.selected.size() >= 100,
               "horizontal extraction must cover almost every image column");
    CHECK_TRUE(maximumCenterError(horizontalResult, 61.0) <= 0.45,
               "horizontal extraction must follow the horizontal ridge");

    hik_stripe::Options vertical = verticalOptions(frame.response.size());
    vertical.maximumSecondPeakRatio = 1.0;
    const hik_stripe::Result verticalResult = extract(frame, vertical);
    CHECK_TRUE(verticalResult.orientation == hik_stripe::Orientation::Vertical,
               "explicit vertical orientation must never auto-switch");
    CHECK_TRUE(verticalResult.selected.size() >= 79,
               "vertical extraction must cover almost every image row");
    CHECK_TRUE(maximumCenterError(verticalResult, 24.0) <= 0.45,
               "vertical extraction must follow the vertical ridge");
}

void testSoftwareRoiConstrainsEveryOutput() {
    SyntheticFrame frame(86, 120);
    addHorizontalGaussian(&frame, 48.0, 145.0, 1.15);
    addHorizontalGaussian(&frame, 14.0, 225.0, 1.15);

    hik_stripe::Options options = horizontalOptions(frame.response.size());
    options.roi = cv::Rect(18, 39, 74, 20);
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok, "ROI-constrained extraction must complete");
    CHECK_TRUE(result.diagnostics.appliedRoi == options.roi,
               "diagnostics must record the applied software ROI");
    CHECK_TRUE(result.selected.size() >= 66,
               "the in-ROI true ridge must remain available");
    for (const hik_stripe::Candidate& candidate : result.candidates) {
        CHECK_TRUE(options.roi.contains(cv::Point(
                       static_cast<int>(std::lround(candidate.pixel.x)),
                       static_cast<int>(std::lround(candidate.pixel.y)))),
                   "no candidate may be generated outside the software ROI");
    }
    CHECK_TRUE(maximumCenterError(result, 48.0) <= 0.40,
               "a brighter out-of-ROI ridge must not affect the selected centerline");
}

void testShortLocalForkIsRejectedAsAnExplicitInterval() {
    SyntheticFrame frame(92, 200);
    addHorizontalGaussian(&frame, 44.0, 100.0, 1.10);
    const int forkBegin = 92;
    const int forkEnd = 103;
    for (int column = forkBegin; column <= forkEnd; ++column) {
        const int distanceToEnd = std::min(
            column - forkBegin, forkEnd - column);
        const double branchCenter =
            44.0 + 4.0 * static_cast<double>(distanceToEnd);
        addHorizontalGaussian(
            &frame, branchCenter, 100.0, 1.10,
            column, column + 1);
    }

    hik_stripe::Options options =
        horizontalOptions(frame.response.size());
    options.pathMaximumStepPx = 6.0;
    options.pathAmbiguityMarginPerPoint = 1.50;
    options.pathAmbiguityMinimumSeparationPx = 5.0;
    options.pathAmbiguityPaddingScanlines = 2;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok,
               "a short local fork must complete extraction");
    CHECK_TRUE(result.provisionalSelected.size() >= 180,
               "the provisional path must preserve the long surrounding ridge");
    CHECK_TRUE(result.multipathIntervals.size() == 1U,
               "a fork occupying only 2-7% of a long path must produce one local ambiguity interval");
    CHECK_TRUE(anyProvisionalHas(
                   result,
                   hik_stripe::REJECT_AMBIGUOUS_MULTIPATH),
               "the provisional branch inside the interval must carry the hard multipath flag");
    if (!result.multipathIntervals.empty()) {
        const hik_stripe::MultipathInterval& interval =
            result.multipathIntervals.front();
        CHECK_TRUE(interval.coreFirstScanIndex >= forkBegin &&
                       interval.coreLastScanIndex <= forkEnd,
                   "the ambiguity core must cover the complete differing path component inside the synthetic fork");
        CHECK_TRUE(!interval.leftBoundaryOpen &&
                       !interval.rightBoundaryOpen &&
                       interval.leftAnchorScanIndex ==
                           interval.coreFirstScanIndex - 1 &&
                       interval.rightAnchorScanIndex ==
                           interval.coreLastScanIndex + 1,
                   "a closed fork must expose the common best/alternate anchor at both ends");
        CHECK_TRUE(interval.firstScanIndex <=
                       interval.leftAnchorScanIndex &&
                       interval.lastScanIndex >=
                           interval.rightAnchorScanIndex &&
                       interval.firstScanIndex <=
                           interval.coreFirstScanIndex -
                               options.pathAmbiguityPaddingScanlines &&
                       interval.lastScanIndex >=
                           interval.coreLastScanIndex +
                               options.pathAmbiguityPaddingScanlines,
                   "the hard-rejected interval must include both anchors and the configured protection padding");
        CHECK_TRUE(interval.branches.size() >= 2,
                   "an ambiguity interval must expose two complete path hypotheses");
        for (const hik_stripe::MultipathBranch& branch :
             interval.branches) {
            CHECK_TRUE(branchIsOrderedAndTagged(interval, branch),
                       "each 3D-validation branch must be ordered and carry stable interval/branch IDs");
            CHECK_TRUE(
                branch.candidates.front().scanIndex <=
                    interval.leftAnchorScanIndex &&
                branch.candidates.back().scanIndex >=
                    interval.rightAnchorScanIndex,
                "each exported branch must span both convergence anchors, not only the low-margin evidence rows");
        }
        CHECK_TRUE(selectedInScanRange(
                       result,
                       interval.firstScanIndex,
                       interval.lastScanIndex) == 0,
                   "publishable selected must hard-reject the entire padded ambiguity interval without interpolation");
        CHECK_TRUE(
            result.diagnostics.multipathAmbiguousScanlineCount ==
                static_cast<std::size_t>(
                    interval.lastScanIndex -
                    interval.firstScanIndex + 1),
            "multipath scanline diagnostics must count the complete hard "
            "protection interval, not only low-margin evidence columns");
    }
    CHECK_TRUE(
        result.diagnostics.provisionalSelectedPointCount ==
            result.provisionalSelected.size() &&
        result.diagnostics.publishableSelectedPointCount ==
            result.selected.size() &&
        result.diagnostics.selectedPointCount ==
            result.selected.size(),
        "provisional and publishable point counts must have unambiguous semantics");
}

void testNearbyForkFragmentsMergeAndRebuildBranches() {
    SyntheticFrame frame(92, 140);
    addHorizontalGaussian(&frame, 43.0, 100.0, 1.10);
    const int forkBegins[] = {42, 56};
    const int forkLength = 10;
    for (const int forkBegin : forkBegins) {
        for (int offset = 0; offset < forkLength; ++offset) {
            const int distanceToEnd = std::min(
                offset, forkLength - 1 - offset);
            const double branchCenter =
                43.0 + 4.0 * static_cast<double>(distanceToEnd);
            addHorizontalGaussian(
                &frame, branchCenter, 100.0, 1.10,
                forkBegin + offset,
                forkBegin + offset + 1);
        }
    }

    hik_stripe::Options options =
        horizontalOptions(frame.response.size());
    options.pathMaximumStepPx = 6.0;
    options.pathAmbiguityMarginPerPoint = 1.50;
    options.pathAmbiguityMinimumSeparationPx = 5.0;
    options.pathAmbiguityPaddingScanlines = 2;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok,
               "two nearby local forks must complete extraction");
    CHECK_TRUE(result.multipathIntervals.size() == 1U,
               "overlapping/nearby protected fork ranges must merge into one interval");
    if (result.multipathIntervals.size() == 1U) {
        const hik_stripe::MultipathInterval& interval =
            result.multipathIntervals.front();
        CHECK_TRUE(interval.coreFirstScanIndex <
                       forkBegins[0] + forkLength &&
                       interval.coreLastScanIndex >=
                           forkBegins[1],
                   "the merged core must cover both fragmented divergence components and their connecting uncertainty");
        CHECK_TRUE(interval.branches.size() >= 3U,
                   "the merged interval must rebuild one primary plus the distinct local alternative hypotheses");
        for (const hik_stripe::MultipathBranch& branch :
             interval.branches) {
            CHECK_TRUE(branchIsOrderedAndTagged(interval, branch),
                       "every merged K-best branch must be a complete ordered provenance sequence");
            CHECK_TRUE(
                branch.candidates.front().scanIndex <=
                    interval.leftAnchorScanIndex &&
                branch.candidates.back().scanIndex >=
                    interval.rightAnchorScanIndex,
                "merged branch hypotheses must be rebuilt over the complete protected range");
        }
        CHECK_TRUE(selectedInScanRange(
                       result,
                       interval.firstScanIndex,
                       interval.lastScanIndex) == 0U,
                   "the union of nearby ambiguity fragments must be hard-rejected without holes");
    }
}

void testSecondPeakThresholdIsSoftAndContinuous() {
    const auto extractRatio =
        [](double competingAmplitude) {
            SyntheticFrame frame(86, 120);
            addHorizontalGaussian(
                &frame, 31.0, 100.0, 1.10);
            addHorizontalGaussian(
                &frame, 55.0, competingAmplitude, 1.10);
            hik_stripe::Options options =
                horizontalOptions(frame.response.size());
            options.maximumSecondPeakRatio = 0.80;
            options.pathAmbiguityMarginPerPoint = 0.0;
            return extract(frame, options);
        };

    const hik_stripe::Result below = extractRatio(79.0);
    const hik_stripe::Result above = extractRatio(81.0);
    CHECK_TRUE(below.ok && above.ok,
               "both sides of the 0.80 soft multipath threshold must remain solvable");
    CHECK_TRUE(below.provisionalSelected.size() >= 108 &&
                   above.provisionalSelected.size() >= 108,
               "crossing 0.79/0.81 must not turn complete candidate rows into GAP");
    CHECK_TRUE(below.selected.size() >= 108 &&
                   above.selected.size() >= 108,
               "a continuity-resolved stronger branch must remain publishable on both sides of the soft threshold");
    CHECK_TRUE(allSelectedPublishable(below) &&
                   allSelectedPublishable(above),
               "formal selected must contain only publishable copies after lattice ambiguity is resolved");
    CHECK_TRUE(!anyCandidateHas(
                   below,
                   hik_stripe::REJECT_MULTI_PEAK_AMBIGUOUS) &&
                   anyCandidateHas(
                       above,
                       hik_stripe::REJECT_MULTI_PEAK_AMBIGUOUS),
               "the 0.80 flag may change, but it must remain soft evidence for the lattice");
    CHECK_TRUE(maximumCenterError(below, 31.0) <= 0.45 &&
                   maximumCenterError(above, 31.0) <= 0.45,
               "a small ratio change around 0.80 must not switch the selected physical branch");
}

void testDuplicateRidgePeaksAreMergedButSeparatedBranchesRemain() {
    SyntheticFrame duplicate(82, 90);
    for (int column = 0; column < duplicate.response.cols; ++column) {
        const int rows[] = {37, 38, 39, 40, 41, 42, 43, 44};
        const int values[] = {30, 80, 150, 205, 190, 202, 145, 65};
        for (std::size_t index = 0U;
             index < sizeof(rows) / sizeof(rows[0]); ++index) {
            duplicate.response.at<unsigned char>(
                rows[index], column) =
                static_cast<unsigned char>(values[index]);
            duplicate.raw.at<unsigned char>(
                rows[index], column) =
                static_cast<unsigned char>(
                    std::min(249, values[index] + 20));
        }
    }
    hik_stripe::Options duplicateOptions =
        horizontalOptions(duplicate.response.size());
    duplicateOptions.maximumSecondPeakRatio = 0.50;
    duplicateOptions.maximumGradientAsymmetry = 1.0;
    duplicateOptions.maximumFitResidual = 1.0;
    duplicateOptions.maximumFwhmPx = 14.0;
    const hik_stripe::Result merged =
        extract(duplicate, duplicateOptions);

    CHECK_TRUE(merged.ok && merged.selected.size() >= 81,
               "two adjacent maxima from one physical ridge must merge into one publishable path");
    CHECK_TRUE(merged.diagnostics.totalCandidateCount <= 95,
               "same-ridge NMS must leave approximately one candidate per scanline");
    CHECK_TRUE(!anyCandidateHas(
                   merged,
                   hik_stripe::REJECT_MULTI_PEAK_AMBIGUOUS),
               "same-ridge duplicate maxima must not masquerade as a second physical branch");

    SyntheticFrame separated(82, 90);
    addHorizontalGaussian(
        &separated, 30.0, 130.0, 1.10);
    addHorizontalGaussian(
        &separated, 50.0, 130.0, 1.10);
    hik_stripe::Options separatedOptions =
        horizontalOptions(separated.response.size());
    separatedOptions.maximumSecondPeakRatio = 0.50;
    separatedOptions.pathAmbiguityMarginPerPoint = 0.10;
    const hik_stripe::Result branches =
        extract(separated, separatedOptions);
    CHECK_TRUE(branches.ok,
               "two separated physical branches must still enter the lattice");
    CHECK_TRUE(anyCandidateHas(
                   branches,
                   hik_stripe::REJECT_MULTI_PEAK_AMBIGUOUS),
               "a genuine 20 px branch separation must retain multipath evidence");
    CHECK_TRUE(!branches.multipathIntervals.empty() &&
                   branches.selected.empty(),
               "equal 20 px branches must be exposed as alternatives and hard-rejected from publishable output");
    if (!branches.multipathIntervals.empty()) {
        CHECK_TRUE(
            branches.multipathIntervals.front()
                    .maximumSeparationPx >= 18.0,
            "the interval diagnostic must preserve the physical branch separation");
    }
}

void testGapCannotAuthorizeAChangedBranch() {
    SyntheticFrame frame(92, 100);
    addHorizontalGaussian(
        &frame, 25.0, 150.0, 1.10, 0, 46);
    addHorizontalGaussian(
        &frame, 40.0, 150.0, 1.10, 50, 100);

    hik_stripe::Options options =
        horizontalOptions(frame.response.size());
    options.pathMaximumStepPx = 4.0;
    options.pathMaximumGap = 5;
    options.pathMaximumPredictionResidualPx = 3.0;
    const hik_stripe::Result result = extract(frame, options);

    CHECK_TRUE(result.ok,
               "a branch-changing gap must fail closed without crashing extraction");
    bool bridgedChangedBranch = false;
    for (std::size_t index = 1U;
         index < result.provisionalSelected.size(); ++index) {
        const hik_stripe::Candidate& previous =
            result.provisionalSelected[index - 1U];
        const hik_stripe::Candidate& current =
            result.provisionalSelected[index];
        if (current.scanIndex - previous.scanIndex > 1 &&
            std::fabs(current.pixel.y - previous.pixel.y) > 10.0) {
            bridgedChangedBranch = true;
        }
    }
    CHECK_TRUE(!bridgedChangedBranch,
               "a missing interval must not dilute a 15 px branch switch into an acceptable average slope");
    CHECK_TRUE(result.provisionalSelected.size() < 90,
               "an unreachable changed branch must create a real break rather than one connected provisional path");
}

}  // namespace

int main() {
    testNonIntegerGaussianUsesDerivativeTaylorCenter();
    testLocalThresholdTracksBrightnessRamp();
    testContinuousDimPathBeatsShortBrightPeak();
    testBrighterParallelSaturatedGhostIsRejected();
    testEqualParallelPathsAreMarkedAmbiguous();
    testSymmetricNarrowPlateauUsesItsMiddle();
    testNarrowAsymmetricSaturatedRidgeIsRejected();
    testWideAsymmetricPlateauIsRejected();
    testGapIsPreservedWithoutInterpolation();
    testLargeJumpDoesNotCreateAConnectingRamp();
    testFixedHorizontalAndVerticalOrientations();
    testSoftwareRoiConstrainsEveryOutput();
    testShortLocalForkIsRejectedAsAnExplicitInterval();
    testNearbyForkFragmentsMergeAndRebuildBranches();
    testSecondPeakThresholdIsSoftAndContinuous();
    testDuplicateRidgePeaksAreMergedButSeparatedBranchesRemain();
    testGapCannotAuthorizeAChangedBranch();

    if (gFailures != 0) {
        std::cerr << gFailures
                  << " stripe-centerline extractor checks failed\n";
        return 1;
    }
    std::cout << "StripeCenterlineExtractor synthetic tests passed ("
              << hik_stripe::algorithmVersion() << ")\n";
    return 0;
}
