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

    if (gFailures != 0) {
        std::cerr << gFailures
                  << " stripe-centerline extractor checks failed\n";
        return 1;
    }
    std::cout << "StripeCenterlineExtractor synthetic tests passed ("
              << hik_stripe::algorithmVersion() << ")\n";
    return 0;
}
