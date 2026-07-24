#include "StripeCenterlineExtractor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <utility>

namespace hik_stripe {
namespace {

constexpr double kEpsilon = 1.0e-9;
constexpr int kGaussianDerivativeRadius = 3;
constexpr double kMaximumTaylorOffsetPx = 0.5;
// Discrete, unit-sigma Gaussian samples at offsets -3..3, normalized to one.
// The first/second derivatives below are central differences of this smoothed
// profile. Keeping the kernel fixed makes the reported algorithm version
// reproducible and is adequate for the current roughly 2--5 px FWHM ridges.
constexpr std::array<double, 2 * kGaussianDerivativeRadius + 1>
    kGaussianDerivativeKernel{{
        0.00443304817524375,
        0.0540055826224145,
        0.242036229376114,
        0.399050279652455,
        0.242036229376114,
        0.0540055826224145,
        0.00443304817524375
    }};

void setError(const std::string& message, std::string* error) {
    if (error) {
        *error = message;
    }
}

double clamp01(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::max(0.0, std::min(1.0, value));
}

double huber(double absoluteValue, double transition) {
    const double value = std::fabs(absoluteValue);
    if (value <= transition) {
        return 0.5 * value * value;
    }
    return transition * (value - 0.5 * transition);
}

cv::Rect appliedRoi(const Options& options, const cv::Size& size) {
    return options.roi.empty()
        ? cv::Rect(0, 0, size.width, size.height)
        : options.roi;
}

int scanlineBegin(Orientation orientation, const cv::Rect& roi) {
    return orientation == Orientation::Vertical ? roi.y : roi.x;
}

int scanlineEnd(Orientation orientation, const cv::Rect& roi) {
    return orientation == Orientation::Vertical
        ? roi.y + roi.height
        : roi.x + roi.width;
}

int minorBegin(Orientation orientation, const cv::Rect& roi) {
    return orientation == Orientation::Vertical ? roi.x : roi.y;
}

int minorEnd(Orientation orientation, const cv::Rect& roi) {
    return orientation == Orientation::Vertical
        ? roi.x + roi.width
        : roi.y + roi.height;
}

unsigned char orientedValue(const cv::Mat& image,
                            Orientation orientation,
                            int scanIndex,
                            int minorIndex) {
    return orientation == Orientation::Vertical
        ? image.at<unsigned char>(scanIndex, minorIndex)
        : image.at<unsigned char>(minorIndex, scanIndex);
}

cv::Point2d orientedPoint(Orientation orientation,
                          int scanIndex,
                          double minorCoordinate) {
    return orientation == Orientation::Vertical
        ? cv::Point2d(minorCoordinate, static_cast<double>(scanIndex))
        : cv::Point2d(static_cast<double>(scanIndex), minorCoordinate);
}

bool maskAllows(const cv::Mat& mask,
                Orientation orientation,
                int scanIndex,
                int minorIndex) {
    if (mask.empty()) {
        return true;
    }
    return orientedValue(mask, orientation, scanIndex, minorIndex) != 0U;
}

double histogramMedian(const std::array<int, 256>& histogram, int count) {
    if (count <= 0) {
        return 0.0;
    }
    const int lowerRank = (count - 1) / 2;
    const int upperRank = count / 2;
    int cumulative = 0;
    int lower = 0;
    int upper = 0;
    bool foundLower = false;
    for (int value = 0; value < 256; ++value) {
        cumulative += histogram[static_cast<std::size_t>(value)];
        if (!foundLower && cumulative > lowerRank) {
            lower = value;
            foundLower = true;
        }
        if (cumulative > upperRank) {
            upper = value;
            break;
        }
    }
    return 0.5 * static_cast<double>(lower + upper);
}

double histogramMad(const std::array<int, 256>& histogram,
                    int count,
                    double median) {
    std::array<int, 256> deviations{};
    for (int value = 0; value < 256; ++value) {
        const int deviation = std::max(
            0, std::min(255, static_cast<int>(
                std::lround(std::fabs(static_cast<double>(value) - median)))));
        deviations[static_cast<std::size_t>(deviation)] +=
            histogram[static_cast<std::size_t>(value)];
    }
    return histogramMedian(deviations, count);
}

double vectorMedian(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if ((values.size() & 1U) != 0U) {
        return upper;
    }
    const double lower = *std::max_element(
        values.begin(), values.begin() + middle);
    return 0.5 * (lower + upper);
}

double localMedian(const cv::Mat& response,
                   Orientation orientation,
                   int scanIndex,
                   int peakIndex,
                   int begin,
                   int end,
                   int radius,
                   int exclusion,
                   double fallback) {
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(2 * std::max(1, radius)));
    const int leftBegin = std::max(begin, peakIndex - radius);
    const int leftEnd = std::max(leftBegin, peakIndex - exclusion);
    for (int index = leftBegin; index < leftEnd; ++index) {
        values.push_back(orientedValue(
            response, orientation, scanIndex, index));
    }
    const int rightBegin = std::min(end, peakIndex + exclusion + 1);
    const int rightEnd = std::min(end, peakIndex + radius + 1);
    for (int index = rightBegin; index < rightEnd; ++index) {
        values.push_back(orientedValue(
            response, orientation, scanIndex, index));
    }
    return values.empty() ? fallback : vectorMedian(std::move(values));
}

double localMad(const cv::Mat& response,
                Orientation orientation,
                int scanIndex,
                int peakIndex,
                int begin,
                int end,
                int radius,
                int exclusion,
                double median) {
    std::vector<double> deviations;
    deviations.reserve(static_cast<std::size_t>(2 * std::max(1, radius)));
    const int leftBegin = std::max(begin, peakIndex - radius);
    const int leftEnd = std::max(leftBegin, peakIndex - exclusion);
    for (int index = leftBegin; index < leftEnd; ++index) {
        deviations.push_back(std::fabs(
            static_cast<double>(orientedValue(
                response, orientation, scanIndex, index)) - median));
    }
    const int rightBegin = std::min(end, peakIndex + exclusion + 1);
    const int rightEnd = std::min(end, peakIndex + radius + 1);
    for (int index = rightBegin; index < rightEnd; ++index) {
        deviations.push_back(std::fabs(
            static_cast<double>(orientedValue(
                response, orientation, scanIndex, index)) - median));
    }
    return deviations.empty() ? 0.0 : vectorMedian(std::move(deviations));
}

double interpolateCrossing(double firstPosition,
                           double firstValue,
                           double secondPosition,
                           double secondValue,
                           double target) {
    const double denominator = secondValue - firstValue;
    if (std::fabs(denominator) <= kEpsilon) {
        return 0.5 * (firstPosition + secondPosition);
    }
    const double ratio = clamp01((target - firstValue) / denominator);
    return firstPosition + ratio * (secondPosition - firstPosition);
}

double sampleLinear(const cv::Mat& response,
                    Orientation orientation,
                    int scanIndex,
                    double minorCoordinate,
                    int begin,
                    int end) {
    const double bounded = std::max(
        static_cast<double>(begin),
        std::min(static_cast<double>(end - 1), minorCoordinate));
    const int left = static_cast<int>(std::floor(bounded));
    const int right = std::min(end - 1, left + 1);
    const double ratio = bounded - static_cast<double>(left);
    const double leftValue = orientedValue(
        response, orientation, scanIndex, left);
    const double rightValue = orientedValue(
        response, orientation, scanIndex, right);
    return leftValue + ratio * (rightValue - leftValue);
}

double minorCoordinate(const Candidate& candidate, Orientation orientation) {
    return orientation == Orientation::Vertical
        ? candidate.pixel.x
        : candidate.pixel.y;
}

bool gaussianSmoothedSample(const cv::Mat& response,
                            Orientation orientation,
                            int scanIndex,
                            int minorIndex,
                            int begin,
                            int end,
                            double* value) {
    if (!value ||
        minorIndex - kGaussianDerivativeRadius < begin ||
        minorIndex + kGaussianDerivativeRadius >= end) {
        return false;
    }
    double sum = 0.0;
    for (int offset = -kGaussianDerivativeRadius;
         offset <= kGaussianDerivativeRadius; ++offset) {
        sum += kGaussianDerivativeKernel[static_cast<std::size_t>(
                   offset + kGaussianDerivativeRadius)] *
               static_cast<double>(orientedValue(
                   response, orientation, scanIndex,
                   minorIndex + offset));
    }
    if (!std::isfinite(sum)) {
        return false;
    }
    *value = sum;
    return true;
}

bool gaussianDerivativeTaylorCenter(const cv::Mat& response,
                                    Orientation orientation,
                                    int scanIndex,
                                    int peakIndex,
                                    int begin,
                                    int end,
                                    double prominence,
                                    double leftCross,
                                    double rightCross,
                                    double* center,
                                    double* offset,
                                    double* firstDerivative,
                                    double* secondDerivative) {
    if (!center || !offset || !firstDerivative || !secondDerivative ||
        !std::isfinite(prominence) || prominence <= 0.0) {
        return false;
    }

    double previous = 0.0;
    double current = 0.0;
    double next = 0.0;
    if (!gaussianSmoothedSample(
            response, orientation, scanIndex, peakIndex - 1,
            begin, end, &previous) ||
        !gaussianSmoothedSample(
            response, orientation, scanIndex, peakIndex,
            begin, end, &current) ||
        !gaussianSmoothedSample(
            response, orientation, scanIndex, peakIndex + 1,
            begin, end, &next)) {
        return false;
    }

    const double first = 0.5 * (next - previous);
    const double second = next - 2.0 * current + previous;
    // A bright ridge must have negative curvature. The relative floor avoids
    // dividing by almost-flat noise without introducing a new acceptance
    // threshold: failure here simply uses the established centroid fallback.
    const double curvatureFloor = std::max(
        1.0e-6, 0.01 * prominence);
    if (!std::isfinite(first) || !std::isfinite(second) ||
        second >= -curvatureFloor) {
        return false;
    }

    const double taylorOffset = -first / second;
    if (!std::isfinite(taylorOffset) ||
        std::fabs(taylorOffset) >
            kMaximumTaylorOffsetPx + 1.0e-12) {
        return false;
    }
    const double estimate =
        static_cast<double>(peakIndex) + taylorOffset;
    if (!std::isfinite(estimate) ||
        estimate < static_cast<double>(begin) ||
        estimate > static_cast<double>(end - 1) ||
        estimate < leftCross - 1.0e-9 ||
        estimate > rightCross + 1.0e-9) {
        return false;
    }

    *center = estimate;
    *offset = taylorOffset;
    *firstDerivative = first;
    *secondDerivative = second;
    return true;
}

void countRejectFlags(const Candidate& candidate, Diagnostics* diagnostics) {
    if (!diagnostics) {
        return;
    }
    if ((candidate.rejectFlags & REJECT_LOW_PROMINENCE) != 0U) {
        ++diagnostics->rejectedLowProminenceCount;
    }
    if ((candidate.rejectFlags & REJECT_WIDTH_OUT_OF_RANGE) != 0U) {
        ++diagnostics->rejectedWidthCount;
    }
    if ((candidate.rejectFlags &
         (REJECT_SATURATED_WIDE_PLATEAU |
          REJECT_SATURATED_ASYMMETRIC)) != 0U) {
        ++diagnostics->rejectedSaturationCount;
    }
    if ((candidate.rejectFlags & REJECT_MULTI_PEAK_AMBIGUOUS) != 0U) {
        ++diagnostics->rejectedMultiPeakCount;
    }
    if ((candidate.rejectFlags & REJECT_PROFILE_ASYMMETRIC) != 0U) {
        ++diagnostics->rejectedAsymmetryCount;
    }
    if ((candidate.rejectFlags & REJECT_FIT_RESIDUAL_HIGH) != 0U) {
        ++diagnostics->rejectedFitCount;
    }
    if ((candidate.rejectFlags & REJECT_QUALITY_LOW) != 0U) {
        ++diagnostics->rejectedQualityCount;
    }
    if ((candidate.rejectFlags & REJECT_OUTSIDE_VALIDITY_MASK) != 0U) {
        ++diagnostics->rejectedMaskCount;
    }
}

struct LineCandidates {
    int scanIndex{0};
    std::vector<Candidate> values;
};

Candidate buildCandidate(const cv::Mat& response,
                         const cv::Mat& raw,
                         const cv::Mat& validityMask,
                         Orientation orientation,
                         int scanIndex,
                         int peakIndex,
                         int begin,
                         int end,
                         double scanlineMedian,
                         const Options& options) {
    Candidate candidate;
    candidate.scanIndex = scanIndex;
    candidate.peakIndex = peakIndex;
    candidate.responsePeak = orientedValue(
        response, orientation, scanIndex, peakIndex);
    candidate.rawPeak = orientedValue(
        raw, orientation, scanIndex, peakIndex);
    candidate.localBaseline = localMedian(
        response, orientation, scanIndex, peakIndex, begin, end,
        options.localBaselineRadius, options.baselineExclusionRadius,
        scanlineMedian);
    const double mad = localMad(
        response, orientation, scanIndex, peakIndex, begin, end,
        options.localBaselineRadius, options.baselineExclusionRadius,
        candidate.localBaseline);
    candidate.localNoiseMad = 1.4826 * mad;
    candidate.prominence = std::max(
        0.0, candidate.responsePeak - candidate.localBaseline);
    candidate.snr = candidate.prominence /
        std::max(1.0, candidate.localNoiseMad);
    const double requiredProminence = std::max(
        options.minimumProminence,
        options.thresholdMadScale * candidate.localNoiseMad);
    if (candidate.prominence < requiredProminence) {
        candidate.rejectFlags |= REJECT_LOW_PROMINENCE;
    }

    const double halfHeight =
        candidate.localBaseline + 0.5 * candidate.prominence;
    int leftInside = peakIndex;
    while (leftInside > begin &&
           orientedValue(response, orientation, scanIndex, leftInside - 1) >=
               halfHeight) {
        --leftInside;
    }
    int rightInside = peakIndex;
    while (rightInside + 1 < end &&
           orientedValue(response, orientation, scanIndex, rightInside + 1) >=
               halfHeight) {
        ++rightInside;
    }

    double leftCross = static_cast<double>(leftInside) - 0.5;
    if (leftInside > begin) {
        leftCross = interpolateCrossing(
            leftInside - 1,
            orientedValue(response, orientation, scanIndex, leftInside - 1),
            leftInside,
            orientedValue(response, orientation, scanIndex, leftInside),
            halfHeight);
    } else if (orientedValue(
                   response, orientation, scanIndex, leftInside) >=
               halfHeight) {
        candidate.rejectFlags |= REJECT_WIDTH_OUT_OF_RANGE;
    }
    double rightCross = static_cast<double>(rightInside) + 0.5;
    if (rightInside + 1 < end) {
        rightCross = interpolateCrossing(
            rightInside,
            orientedValue(response, orientation, scanIndex, rightInside),
            rightInside + 1,
            orientedValue(response, orientation, scanIndex, rightInside + 1),
            halfHeight);
    } else if (orientedValue(
                   response, orientation, scanIndex, rightInside) >=
               halfHeight) {
        candidate.rejectFlags |= REJECT_WIDTH_OUT_OF_RANGE;
    }
    candidate.fwhmPx = std::max(0.0, rightCross - leftCross);
    if (candidate.fwhmPx < options.minimumFwhmPx ||
        candidate.fwhmPx > options.maximumFwhmPx) {
        candidate.rejectFlags |= REJECT_WIDTH_OUT_OF_RANGE;
    }

    const int supportBegin = std::max(
        begin, static_cast<int>(std::ceil(leftCross)));
    const int supportEnd = std::min(
        end - 1, static_cast<int>(std::floor(rightCross)));
    double weightedCoordinate = 0.0;
    double weightSum = 0.0;
    int saturatedCount = 0;
    int plateauWidth = 0;
    int currentPlateauWidth = 0;
    for (int index = supportBegin; index <= supportEnd; ++index) {
        const double weight = std::max(
            0.0,
            static_cast<double>(orientedValue(
                response, orientation, scanIndex, index)) -
                candidate.localBaseline);
        weightedCoordinate += static_cast<double>(index) * weight;
        weightSum += weight;
        if (orientedValue(raw, orientation, scanIndex, index) >=
            options.rawSaturationThreshold) {
            ++saturatedCount;
            ++currentPlateauWidth;
            plateauWidth = std::max(plateauWidth, currentPlateauWidth);
        } else {
            currentPlateauWidth = 0;
        }
    }
    const int supportCount = std::max(0, supportEnd - supportBegin + 1);
    candidate.saturatedFraction = supportCount > 0
        ? static_cast<double>(saturatedCount) /
              static_cast<double>(supportCount)
        : 0.0;
    candidate.saturatedPlateauWidthPx = plateauWidth;
    const double edgeCenter = 0.5 * (leftCross + rightCross);
    const double centroid = weightSum > kEpsilon
        ? weightedCoordinate / weightSum
        : edgeCenter;
    double center = centroid;
    candidate.centerMethod =
        CenterMethod::BackgroundSubtractedCentroid;
    if (saturatedCount > 0) {
        // Derivatives across a clipped flat top do not locate the optical
        // ridge. Keep the existing two-wing rule: interpolate both true
        // half-height crossings and take their midpoint.
        center = edgeCenter;
        candidate.centerMethod =
            CenterMethod::SaturatedHalfHeightMidpoint;
    } else {
        double taylorCenter = 0.0;
        double taylorOffset = 0.0;
        double firstDerivative = 0.0;
        double secondDerivative = 0.0;
        if (gaussianDerivativeTaylorCenter(
                response, orientation, scanIndex, peakIndex,
                begin, end, candidate.prominence,
                leftCross, rightCross, &taylorCenter,
                &taylorOffset, &firstDerivative,
                &secondDerivative)) {
            center = taylorCenter;
            candidate.centerMethod =
                CenterMethod::GaussianDerivativeTaylor;
            candidate.taylorOffsetPx = taylorOffset;
            candidate.smoothedFirstDerivative = firstDerivative;
            candidate.smoothedSecondDerivative = secondDerivative;
        }
    }
    if (!std::isfinite(center)) {
        // This is a defensive last line; centroid and edgeCenter are already
        // finite for a valid support. Never let an exceptional profile inject
        // NaN into path optimization.
        center = static_cast<double>(peakIndex);
        candidate.centerMethod =
            CenterMethod::BackgroundSubtractedCentroid;
        candidate.taylorOffsetPx = 0.0;
        candidate.smoothedFirstDerivative = 0.0;
        candidate.smoothedSecondDerivative = 0.0;
    }
    candidate.pixel = orientedPoint(orientation, scanIndex, center);

    const int maskIndex = std::max(
        begin, std::min(end - 1, static_cast<int>(std::lround(center))));
    if (!maskAllows(
            validityMask, orientation, scanIndex, maskIndex)) {
        candidate.rejectFlags |= REJECT_OUTSIDE_VALIDITY_MASK;
    }

    const int symmetrySamples = std::max(
        1, static_cast<int>(std::ceil(candidate.fwhmPx * 0.5)));
    double leftEnergy = 0.0;
    double rightEnergy = 0.0;
    double squaredDifference = 0.0;
    int residualSamples = 0;
    for (int offset = 1; offset <= symmetrySamples; ++offset) {
        const double left = std::max(
            0.0, sampleLinear(
                response, orientation, scanIndex, center - offset,
                begin, end) - candidate.localBaseline);
        const double right = std::max(
            0.0, sampleLinear(
                response, orientation, scanIndex, center + offset,
                begin, end) - candidate.localBaseline);
        leftEnergy += left;
        rightEnergy += right;
        const double normalizedDifference =
            (left - right) / std::max(1.0, candidate.prominence);
        squaredDifference +=
            normalizedDifference * normalizedDifference;
        ++residualSamples;
    }
    candidate.gradientAsymmetry = std::fabs(leftEnergy - rightEnergy) /
        std::max(1.0, leftEnergy + rightEnergy);
    candidate.fitResidual = residualSamples > 0
        ? std::sqrt(
              squaredDifference / static_cast<double>(residualSamples))
        : 1.0;

    if (candidate.saturatedFraction >
            options.maximumSaturatedFraction ||
        candidate.saturatedPlateauWidthPx >
            options.maximumSaturatedPlateauWidthPx) {
        candidate.rejectFlags |= REJECT_SATURATED_WIDE_PLATEAU;
    }
    if (saturatedCount > 0 &&
        candidate.gradientAsymmetry >
            options.maximumGradientAsymmetry) {
        candidate.rejectFlags |= REJECT_SATURATED_ASYMMETRIC;
    } else if (candidate.gradientAsymmetry >
               options.maximumGradientAsymmetry) {
        candidate.rejectFlags |= REJECT_PROFILE_ASYMMETRIC;
    }
    if (candidate.fitResidual > options.maximumFitResidual) {
        candidate.rejectFlags |= REJECT_FIT_RESIDUAL_HIGH;
    }

    const double prominenceScore = clamp01(
        candidate.prominence /
        std::max(1.0, 4.0 * options.minimumProminence));
    const double snrScore = clamp01(candidate.snr / 10.0);
    const double widthMiddle =
        0.5 * (options.minimumFwhmPx + options.maximumFwhmPx);
    const double widthRadius =
        std::max(0.5, 0.5 *
            (options.maximumFwhmPx - options.minimumFwhmPx));
    const double widthScore = clamp01(
        1.0 - 0.5 * std::fabs(candidate.fwhmPx - widthMiddle) /
                  widthRadius);
    const double saturationScore = clamp01(
        1.0 - candidate.saturatedFraction /
                  std::max(0.01, options.maximumSaturatedFraction));
    const double symmetryScore = clamp01(
        1.0 - candidate.gradientAsymmetry /
                  std::max(0.01, options.maximumGradientAsymmetry));
    const double fitScore = clamp01(
        1.0 - candidate.fitResidual /
                  std::max(0.01, options.maximumFitResidual));
    candidate.quality =
        0.25 * prominenceScore +
        0.20 * snrScore +
        0.15 * widthScore +
        0.15 * saturationScore +
        0.15 * symmetryScore +
        0.10 * fitScore;
    candidate.centerSigmaPx = std::max(
        0.02, std::min(
            2.0,
            0.03 +
            0.35 * candidate.fitResidual +
            0.25 * candidate.gradientAsymmetry +
            0.50 * candidate.saturatedFraction +
            0.20 / std::max(1.0, candidate.snr)));
    return candidate;
}

void finalizeLineCandidates(LineCandidates* line,
                            const Options& options,
                            Diagnostics* diagnostics) {
    if (!line || line->values.empty()) {
        return;
    }
    std::sort(
        line->values.begin(), line->values.end(),
        [](const Candidate& first, const Candidate& second) {
            if (first.prominence != second.prominence) {
                return first.prominence > second.prominence;
            }
            return first.peakIndex < second.peakIndex;
        });
    if (line->values.size() > static_cast<std::size_t>(
            options.maximumCandidatesPerScanline)) {
        line->values.resize(static_cast<std::size_t>(
            options.maximumCandidatesPerScanline));
    }
    const double secondPeakRatio = line->values.size() >= 2U
        ? line->values[1].prominence /
              std::max(1.0, line->values[0].prominence)
        : 0.0;
    if (secondPeakRatio > options.maximumSecondPeakRatio &&
        diagnostics) {
        ++diagnostics->multiPeakScanlineCount;
    }
    for (Candidate& candidate : line->values) {
        candidate.secondPeakRatio = secondPeakRatio;
        if (secondPeakRatio > options.maximumSecondPeakRatio) {
            candidate.rejectFlags |= REJECT_MULTI_PEAK_AMBIGUOUS;
        }
        const double multiPeakScore = clamp01(
            1.0 - candidate.secondPeakRatio /
                      std::max(0.01, options.maximumSecondPeakRatio));
        candidate.quality =
            0.95 * candidate.quality + 0.05 * multiPeakScore;
        if (candidate.quality < options.minimumQuality) {
            candidate.rejectFlags |= REJECT_QUALITY_LOW;
        }
        if (diagnostics) {
            ++diagnostics->totalCandidateCount;
            if (candidate.saturatedFraction > 0.0) {
                ++diagnostics->saturatedCandidateCount;
            }
            if (candidate.accepted()) {
                ++diagnostics->acceptedCandidateCount;
            } else {
                countRejectFlags(candidate, diagnostics);
            }
        }
    }
}

struct PathState {
    bool valid{false};
    double cost{std::numeric_limits<double>::infinity()};
    int previousLine{-1};
    int previousCandidate{-1};
    double lastSlope{0.0};
    bool hasSlope{false};
    int pointCount{0};
};

bool optimizePath(std::vector<LineCandidates>* lines,
                  Orientation orientation,
                  const Options& options,
                  Result* result) {
    if (!lines || !result || lines->empty()) {
        return false;
    }
    std::vector<std::vector<PathState>> states(lines->size());
    for (std::size_t lineOffset = 0U;
         lineOffset < lines->size(); ++lineOffset) {
        LineCandidates& line = (*lines)[lineOffset];
        states[lineOffset].resize(line.values.size());
        for (std::size_t candidateIndex = 0U;
             candidateIndex < line.values.size(); ++candidateIndex) {
            Candidate& candidate = line.values[candidateIndex];
            if (!candidate.accepted()) {
                continue;
            }
            PathState best;
            const double nodeCost =
                -options.pathCandidateReward * candidate.quality;
            best.valid = true;
            best.cost =
                static_cast<double>(lineOffset) *
                    options.pathGapExtendPenalty +
                nodeCost;
            best.pointCount = 1;

            const std::size_t earliest =
                lineOffset > static_cast<std::size_t>(
                    options.pathMaximumGap + 1)
                ? lineOffset - static_cast<std::size_t>(
                      options.pathMaximumGap + 1)
                : 0U;
            for (std::size_t previousLine = earliest;
                 previousLine < lineOffset; ++previousLine) {
                const int scanDelta =
                    line.scanIndex -
                    (*lines)[previousLine].scanIndex;
                if (scanDelta <= 0) {
                    continue;
                }
                const int gap = scanDelta - 1;
                if (gap > options.pathMaximumGap) {
                    continue;
                }
                for (std::size_t previousCandidate = 0U;
                     previousCandidate <
                         (*lines)[previousLine].values.size();
                     ++previousCandidate) {
                    const PathState& previousState =
                        states[previousLine][previousCandidate];
                    if (!previousState.valid) {
                        continue;
                    }
                    const Candidate& previous =
                        (*lines)[previousLine].values[previousCandidate];
                    const double signedSlope =
                        (minorCoordinate(candidate, orientation) -
                         minorCoordinate(previous, orientation)) /
                        static_cast<double>(scanDelta);
                    const double step = std::fabs(signedSlope);
                    if (step > options.pathMaximumStepPx) {
                        continue;
                    }
                    double transitionCost =
                        options.pathPositionPenalty *
                            huber(step, 1.0);
                    if (previousState.hasSlope) {
                        transitionCost +=
                            options.pathCurvaturePenalty *
                            huber(
                                std::fabs(
                                    signedSlope -
                                    previousState.lastSlope),
                                0.5);
                    }
                    if (gap > 0) {
                        transitionCost +=
                            options.pathGapOpenPenalty +
                            options.pathGapExtendPenalty *
                                static_cast<double>(gap);
                    }
                    const double cost =
                        previousState.cost + transitionCost + nodeCost;
                    if (cost < best.cost) {
                        best.valid = true;
                        best.cost = cost;
                        best.previousLine =
                            static_cast<int>(previousLine);
                        best.previousCandidate =
                            static_cast<int>(previousCandidate);
                        best.lastSlope = signedSlope;
                        best.hasSlope = true;
                        best.pointCount =
                            previousState.pointCount + 1;
                    }
                }
            }
            states[lineOffset][candidateIndex] = best;
        }
    }

    struct Ending {
        double cost;
        int line;
        int candidate;
        int pointCount;
    };
    std::vector<Ending> endings;
    const double allGapCost =
        static_cast<double>(lines->size()) *
        options.pathGapExtendPenalty;
    endings.push_back(Ending{
        allGapCost, -1, -1, 0});
    for (std::size_t lineOffset = 0U;
         lineOffset < lines->size(); ++lineOffset) {
        const double trailing =
            static_cast<double>(lines->size() - lineOffset - 1U) *
            options.pathGapExtendPenalty;
        for (std::size_t candidateIndex = 0U;
             candidateIndex < states[lineOffset].size();
             ++candidateIndex) {
            const PathState& state =
                states[lineOffset][candidateIndex];
            if (!state.valid) {
                continue;
            }
            endings.push_back(Ending{
                state.cost + trailing,
                static_cast<int>(lineOffset),
                static_cast<int>(candidateIndex),
                state.pointCount});
        }
    }
    std::sort(
        endings.begin(), endings.end(),
        [](const Ending& first, const Ending& second) {
            if (first.cost != second.cost) {
                return first.cost < second.cost;
            }
            return first.pointCount > second.pointCount;
        });
    if (endings.empty() || endings.front().line < 0) {
        result->error =
            "quality path optimizer selected only GAP states";
        result->diagnostics.bestPathCost = allGapCost;
        return false;
    }
    const Ending bestEnding = endings.front();
    const auto traceback =
        [&states](const Ending& ending) {
            std::vector<std::pair<int, int>> reversed;
            int line = ending.line;
            int candidateIndex = ending.candidate;
            while (line >= 0 && candidateIndex >= 0) {
                reversed.push_back(
                    std::make_pair(line, candidateIndex));
                const PathState& state =
                    states[static_cast<std::size_t>(line)]
                          [static_cast<std::size_t>(candidateIndex)];
                const int previousLine = state.previousLine;
                const int previousCandidate =
                    state.previousCandidate;
                line = previousLine;
                candidateIndex = previousCandidate;
            }
            std::reverse(reversed.begin(), reversed.end());
            return reversed;
        };
    const std::vector<std::pair<int, int>> reversed =
        traceback(bestEnding);
    double secondCost =
        std::numeric_limits<double>::infinity();
    // End states that merely truncate the same path are not a competing
    // physical stripe. Select the first alternative whose candidate sequence
    // differs on at least 10% of the shorter path.
    for (std::size_t endingIndex = 1U;
         endingIndex < endings.size(); ++endingIndex) {
        if (endings[endingIndex].line < 0) {
            continue;
        }
        const std::vector<std::pair<int, int>> alternative =
            traceback(endings[endingIndex]);
        if (alternative.empty()) {
            continue;
        }
        std::size_t same = 0U;
        std::size_t firstIndex = 0U;
        std::size_t secondIndex = 0U;
        while (firstIndex < reversed.size() &&
               secondIndex < alternative.size()) {
            if (reversed[firstIndex] == alternative[secondIndex]) {
                ++same;
                ++firstIndex;
                ++secondIndex;
            } else if (reversed[firstIndex] <
                       alternative[secondIndex]) {
                ++firstIndex;
            } else {
                ++secondIndex;
            }
        }
        const std::size_t shorter = std::min(
            reversed.size(), alternative.size());
        const double sameFraction = shorter > 0U
            ? static_cast<double>(same) /
                  static_cast<double>(shorter)
            : 1.0;
        if (sameFraction <= 0.90) {
            secondCost = endings[endingIndex].cost;
            break;
        }
    }
    result->diagnostics.bestPathCost = bestEnding.cost;
    result->diagnostics.secondPathCost = secondCost;
    result->diagnostics.pathCostMargin =
        std::isfinite(secondCost)
        ? secondCost - bestEnding.cost
        : std::numeric_limits<double>::infinity();
    result->diagnostics.pathCostMarginPerPoint =
        result->diagnostics.pathCostMargin /
        static_cast<double>(std::max(1, bestEnding.pointCount));

    result->selected.reserve(reversed.size());
    for (const std::pair<int, int>& index : reversed) {
        result->selected.push_back(
            (*lines)[static_cast<std::size_t>(index.first)]
                .values[static_cast<std::size_t>(index.second)]);
    }

    const bool ambiguous =
        std::isfinite(
            result->diagnostics.pathCostMarginPerPoint) &&
        result->diagnostics.pathCostMarginPerPoint <
            options.pathAmbiguityMarginPerPoint;
    if (ambiguous) {
        for (Candidate& candidate : result->selected) {
            candidate.rejectFlags |= REJECT_PATH_AMBIGUOUS;
        }
        result->diagnostics.ambiguousPathPointCount =
            result->selected.size();
    }
    return !result->selected.empty();
}

bool lineHasAcceptedCandidate(const LineCandidates& line) {
    return std::any_of(
        line.values.begin(), line.values.end(),
        [](const Candidate& candidate) {
            return candidate.accepted();
        });
}

bool optimizePathSegments(const std::vector<LineCandidates>& lines,
                          Orientation orientation,
                          const Options& options,
                          Result* result) {
    if (!result || lines.empty()) {
        return false;
    }
    std::vector<std::pair<std::size_t, std::size_t>> spans;
    std::size_t spanBegin = 0U;
    std::size_t lastAccepted = 0U;
    bool insideSpan = false;
    for (std::size_t index = 0U; index < lines.size(); ++index) {
        if (!lineHasAcceptedCandidate(lines[index])) {
            continue;
        }
        if (!insideSpan) {
            spanBegin = index;
            lastAccepted = index;
            insideSpan = true;
            continue;
        }
        const int scanGap =
            lines[index].scanIndex -
            lines[lastAccepted].scanIndex - 1;
        if (scanGap > options.pathMaximumGap) {
            spans.push_back(
                std::make_pair(spanBegin, lastAccepted));
            spanBegin = index;
        }
        lastAccepted = index;
    }
    if (insideSpan) {
        spans.push_back(std::make_pair(spanBegin, lastAccepted));
    }
    if (spans.empty()) {
        result->error =
            "no scanline contains an accepted stripe candidate";
        return false;
    }

    double bestCostSum = 0.0;
    double secondCostSum = 0.0;
    bool allSecondCostsFinite = true;
    double minimumMargin =
        std::numeric_limits<double>::infinity();
    double minimumMarginPerPoint =
        std::numeric_limits<double>::infinity();
    std::size_t ambiguousPointCount = 0U;
    std::size_t successfulSegments = 0U;
    for (const std::pair<std::size_t, std::size_t>& span : spans) {
        std::vector<LineCandidates> segment(
            lines.begin() + static_cast<std::ptrdiff_t>(span.first),
            lines.begin() + static_cast<std::ptrdiff_t>(span.second + 1U));
        Result segmentResult;
        segmentResult.orientation = orientation;
        if (!optimizePath(
                &segment, orientation, options, &segmentResult)) {
            continue;
        }
        ++successfulSegments;
        result->selected.insert(
            result->selected.end(),
            segmentResult.selected.begin(),
            segmentResult.selected.end());
        bestCostSum +=
            segmentResult.diagnostics.bestPathCost;
        if (std::isfinite(
                segmentResult.diagnostics.secondPathCost)) {
            secondCostSum +=
                segmentResult.diagnostics.secondPathCost;
        } else {
            allSecondCostsFinite = false;
        }
        minimumMargin = std::min(
            minimumMargin,
            segmentResult.diagnostics.pathCostMargin);
        minimumMarginPerPoint = std::min(
            minimumMarginPerPoint,
            segmentResult.diagnostics.pathCostMarginPerPoint);
        ambiguousPointCount +=
            segmentResult.diagnostics.ambiguousPathPointCount;
    }
    if (successfulSegments == 0U || result->selected.empty()) {
        result->error =
            "no quality path segment survived optimization";
        return false;
    }
    std::sort(
        result->selected.begin(), result->selected.end(),
        [](const Candidate& first, const Candidate& second) {
            if (first.scanIndex != second.scanIndex) {
                return first.scanIndex < second.scanIndex;
            }
            return first.peakIndex < second.peakIndex;
        });
    result->diagnostics.bestPathCost = bestCostSum;
    result->diagnostics.secondPathCost =
        allSecondCostsFinite
        ? secondCostSum
        : std::numeric_limits<double>::infinity();
    result->diagnostics.pathCostMargin =
        minimumMargin;
    result->diagnostics.pathCostMarginPerPoint =
        minimumMarginPerPoint;
    result->diagnostics.ambiguousPathPointCount =
        ambiguousPointCount;
    return true;
}

double resultPreference(const Result& result) {
    if (result.diagnostics.scanlineCount == 0U) {
        return -std::numeric_limits<double>::infinity();
    }
    const double coverage =
        static_cast<double>(result.selected.size()) /
        static_cast<double>(result.diagnostics.scanlineCount);
    const double ambiguityPenalty =
        result.diagnostics.ambiguousPathPointCount > 0U
        ? 0.5
        : 0.0;
    return 2.0 * coverage +
           result.diagnostics.meanSelectedQuality -
           ambiguityPenalty;
}

bool extractOne(const cv::Mat& response,
                const cv::Mat& raw,
                const cv::Mat& validityMask,
                const Options& options,
                Orientation orientation,
                Result* result) {
    if (!result) {
        return false;
    }
    *result = Result();
    result->orientation = orientation;
    result->diagnostics.requestedOrientation =
        options.orientation;
    result->diagnostics.selectedOrientation = orientation;
    const cv::Rect roi = appliedRoi(options, response.size());
    result->diagnostics.appliedRoi = roi;
    const int scanBegin = scanlineBegin(orientation, roi);
    const int scanEnd = scanlineEnd(orientation, roi);
    const int minorStart = minorBegin(orientation, roi);
    const int minorStop = minorEnd(orientation, roi);
    result->diagnostics.scanlineCount =
        static_cast<std::size_t>(std::max(0, scanEnd - scanBegin));

    std::vector<LineCandidates> lines;
    lines.reserve(result->diagnostics.scanlineCount);
    for (int scanIndex = scanBegin;
         scanIndex < scanEnd; ++scanIndex) {
        LineCandidates line;
        line.scanIndex = scanIndex;
        std::array<int, 256> histogram{};
        int histogramCount = 0;
        for (int minor = minorStart; minor < minorStop; ++minor) {
            if (!maskAllows(
                    validityMask, orientation, scanIndex, minor)) {
                continue;
            }
            ++histogram[static_cast<std::size_t>(
                orientedValue(
                    response, orientation, scanIndex, minor))];
            ++histogramCount;
        }
        const double scanMedian =
            histogramMedian(histogram, histogramCount);
        const double scanMad =
            1.4826 * histogramMad(
                histogram, histogramCount, scanMedian);
        const double scanThreshold =
            scanMedian + std::max(
                options.minimumProminence,
                options.thresholdMadScale * scanMad);

        for (int minor = minorStart + 1;
             minor + 1 < minorStop; ++minor) {
            if (!maskAllows(
                    validityMask, orientation, scanIndex, minor)) {
                continue;
            }
            const int value = orientedValue(
                response, orientation, scanIndex, minor);
            if (value < scanThreshold) {
                continue;
            }
            const int previous = orientedValue(
                response, orientation, scanIndex, minor - 1);
            const int next = orientedValue(
                response, orientation, scanIndex, minor + 1);
            if (value < previous || value <= next) {
                continue;
            }
            line.values.push_back(buildCandidate(
                response, raw, validityMask, orientation,
                scanIndex, minor, minorStart, minorStop,
                scanMedian, options));
        }
        if (!line.values.empty()) {
            ++result->diagnostics.scanlinesWithCandidates;
            finalizeLineCandidates(
                &line, options, &result->diagnostics);
        }
        lines.push_back(std::move(line));
    }

    result->candidates.reserve(
        result->diagnostics.totalCandidateCount);
    for (const LineCandidates& line : lines) {
        result->candidates.insert(
            result->candidates.end(),
            line.values.begin(), line.values.end());
    }
    if (!optimizePathSegments(
            lines, orientation, options, result)) {
        if (result->error.empty()) {
            result->error =
                "no quality-gated centerline path was found";
        }
        return false;
    }

    result->diagnostics.selectedPointCount =
        result->selected.size();
    result->diagnostics.selectedGapCount =
        result->diagnostics.scanlineCount >
            result->selected.size()
        ? result->diagnostics.scanlineCount -
              result->selected.size()
        : 0U;
    double qualitySum = 0.0;
    double widthSum = 0.0;
    double snrSum = 0.0;
    double gradientAsymmetrySum = 0.0;
    double fitResidualSum = 0.0;
    double secondPeakRatioSum = 0.0;
    std::size_t selectedSaturated = 0U;
    for (const Candidate& candidate : result->selected) {
        qualitySum += candidate.quality;
        widthSum += candidate.fwhmPx;
        snrSum += candidate.snr;
        gradientAsymmetrySum += candidate.gradientAsymmetry;
        fitResidualSum += candidate.fitResidual;
        secondPeakRatioSum += candidate.secondPeakRatio;
        if (candidate.saturatedFraction > 0.0) {
            ++selectedSaturated;
        }
    }
    const double inverse =
        1.0 / static_cast<double>(
            std::max<std::size_t>(1U, result->selected.size()));
    result->diagnostics.meanSelectedQuality =
        qualitySum * inverse;
    result->diagnostics.meanSelectedFwhmPx =
        widthSum * inverse;
    result->diagnostics.meanSelectedSnr =
        snrSum * inverse;
    result->diagnostics.meanSelectedGradientAsymmetry =
        gradientAsymmetrySum * inverse;
    result->diagnostics.meanSelectedFitResidual =
        fitResidualSum * inverse;
    result->diagnostics.meanSelectedSecondPeakRatio =
        secondPeakRatioSum * inverse;
    result->diagnostics.selectedSaturatedRatio =
        static_cast<double>(selectedSaturated) * inverse;
    result->ok = true;
    return true;
}

}  // namespace

Options::Options()
    : orientation(Orientation::Auto),
      localBaselineRadius(18),
      baselineExclusionRadius(5),
      maximumCandidatesPerScanline(8),
      minimumProminence(12.0),
      thresholdMadScale(4.0),
      minimumFwhmPx(1.0),
      maximumFwhmPx(12.0),
      rawSaturationThreshold(250),
      maximumSaturatedFraction(0.35),
      maximumSaturatedPlateauWidthPx(3),
      maximumSecondPeakRatio(0.80),
      maximumGradientAsymmetry(0.55),
      maximumFitResidual(0.45),
      minimumQuality(0.25),
      pathCandidateReward(3.0),
      pathPositionPenalty(0.30),
      pathCurvaturePenalty(0.45),
      pathGapOpenPenalty(1.5),
      pathGapExtendPenalty(1.0),
      pathMaximumStepPx(10.0),
      pathMaximumGap(12),
      pathAmbiguityMarginPerPoint(0.01) {}

bool Options::validate(const cv::Size& imageSize,
                       std::string* error) const {
    if (imageSize.width < 3 || imageSize.height < 3 ||
        (orientation != Orientation::Auto &&
         orientation != Orientation::Vertical &&
         orientation != Orientation::Horizontal) ||
        localBaselineRadius < 3 ||
        baselineExclusionRadius < 1 ||
        baselineExclusionRadius >= localBaselineRadius ||
        maximumCandidatesPerScanline < 1 ||
        !std::isfinite(minimumProminence) ||
        minimumProminence <= 0.0 ||
        !std::isfinite(thresholdMadScale) ||
        thresholdMadScale < 0.0 ||
        !std::isfinite(minimumFwhmPx) ||
        !std::isfinite(maximumFwhmPx) ||
        minimumFwhmPx <= 0.0 ||
        maximumFwhmPx <= minimumFwhmPx ||
        rawSaturationThreshold < 1 ||
        rawSaturationThreshold > 255 ||
        !std::isfinite(maximumSaturatedFraction) ||
        maximumSaturatedFraction < 0.0 ||
        maximumSaturatedFraction > 1.0 ||
        maximumSaturatedPlateauWidthPx < 0 ||
        !std::isfinite(maximumSecondPeakRatio) ||
        maximumSecondPeakRatio < 0.0 ||
        maximumSecondPeakRatio > 1.0 ||
        !std::isfinite(maximumGradientAsymmetry) ||
        maximumGradientAsymmetry <= 0.0 ||
        maximumGradientAsymmetry > 1.0 ||
        !std::isfinite(maximumFitResidual) ||
        maximumFitResidual <= 0.0 ||
        !std::isfinite(minimumQuality) ||
        minimumQuality < 0.0 ||
        minimumQuality > 1.0 ||
        !std::isfinite(pathCandidateReward) ||
        pathCandidateReward <= 0.0 ||
        !std::isfinite(pathPositionPenalty) ||
        pathPositionPenalty < 0.0 ||
        !std::isfinite(pathCurvaturePenalty) ||
        pathCurvaturePenalty < 0.0 ||
        !std::isfinite(pathGapOpenPenalty) ||
        pathGapOpenPenalty < 0.0 ||
        !std::isfinite(pathGapExtendPenalty) ||
        pathGapExtendPenalty <= 0.0 ||
        !std::isfinite(pathMaximumStepPx) ||
        pathMaximumStepPx <= 0.0 ||
        pathMaximumGap < 0 ||
        !std::isfinite(pathAmbiguityMarginPerPoint) ||
        pathAmbiguityMarginPerPoint < 0.0) {
        setError("stripe centerline options are invalid", error);
        return false;
    }
    if (!roi.empty() &&
        (roi.x < 0 || roi.y < 0 ||
         roi.width < 3 || roi.height < 3 ||
         roi.x + roi.width > imageSize.width ||
         roi.y + roi.height > imageSize.height)) {
        setError(
            "stripe centerline ROI is outside the calibrated image",
            error);
        return false;
    }
    if (error) {
        error->clear();
    }
    return true;
}

Candidate::Candidate()
    : pixel(0.0, 0.0), scanIndex(0), peakIndex(0),
      rawPeak(0.0), responsePeak(0.0), localBaseline(0.0),
      localNoiseMad(0.0), prominence(0.0), snr(0.0),
      fwhmPx(0.0), saturatedFraction(0.0),
      saturatedPlateauWidthPx(0), secondPeakRatio(0.0),
      gradientAsymmetry(0.0), fitResidual(0.0), quality(0.0),
      centerSigmaPx(0.0),
      centerMethod(CenterMethod::BackgroundSubtractedCentroid),
      taylorOffsetPx(0.0), smoothedFirstDerivative(0.0),
      smoothedSecondDerivative(0.0), rejectFlags(REJECT_NONE) {}

bool Candidate::accepted() const {
    return rejectFlags == REJECT_NONE;
}

Diagnostics::Diagnostics()
    : requestedOrientation(Orientation::Auto),
      selectedOrientation(Orientation::Auto),
      scanlineCount(0U), scanlinesWithCandidates(0U),
      totalCandidateCount(0U), acceptedCandidateCount(0U),
      selectedPointCount(0U), selectedGapCount(0U),
      saturatedCandidateCount(0U), multiPeakScanlineCount(0U),
      ambiguousPathPointCount(0U),
      rejectedLowProminenceCount(0U), rejectedWidthCount(0U),
      rejectedSaturationCount(0U), rejectedMultiPeakCount(0U),
      rejectedAsymmetryCount(0U), rejectedFitCount(0U),
      rejectedQualityCount(0U), rejectedMaskCount(0U),
      meanSelectedQuality(0.0), meanSelectedFwhmPx(0.0),
      meanSelectedSnr(0.0),
      meanSelectedGradientAsymmetry(0.0),
      meanSelectedFitResidual(0.0),
      meanSelectedSecondPeakRatio(0.0),
      selectedSaturatedRatio(0.0),
      bestPathCost(std::numeric_limits<double>::infinity()),
      secondPathCost(std::numeric_limits<double>::infinity()),
      pathCostMargin(0.0), pathCostMarginPerPoint(0.0) {}

Result::Result()
    : ok(false), orientation(Orientation::Auto) {}

bool extractCenterline(const cv::Mat& response8,
                       const cv::Mat& raw8,
                       const Options& options,
                       Result* result,
                       const cv::Mat& validityMask) {
    if (!result) {
        return false;
    }
    *result = Result();
    if (response8.empty() || raw8.empty() ||
        response8.type() != CV_8UC1 ||
        raw8.type() != CV_8UC1 ||
        response8.size() != raw8.size()) {
        result->error =
            "response/raw stripe images must be same-size CV_8UC1";
        return false;
    }
    if (!validityMask.empty() &&
        (validityMask.type() != CV_8UC1 ||
         validityMask.size() != response8.size())) {
        result->error =
            "stripe validity mask must be empty or same-size CV_8UC1";
        return false;
    }
    std::string validationError;
    if (!options.validate(response8.size(), &validationError)) {
        result->error = validationError;
        return false;
    }

    if (options.orientation != Orientation::Auto) {
        return extractOne(
            response8, raw8, validityMask, options,
            options.orientation, result);
    }

    Result vertical;
    Result horizontal;
    const bool verticalOk = extractOne(
        response8, raw8, validityMask, options,
        Orientation::Vertical, &vertical);
    const bool horizontalOk = extractOne(
        response8, raw8, validityMask, options,
        Orientation::Horizontal, &horizontal);
    if (verticalOk && horizontalOk) {
        *result = resultPreference(horizontal) >
                resultPreference(vertical)
            ? std::move(horizontal)
            : std::move(vertical);
        result->diagnostics.requestedOrientation =
            Orientation::Auto;
        return true;
    }
    if (horizontalOk) {
        *result = std::move(horizontal);
        result->diagnostics.requestedOrientation =
            Orientation::Auto;
        return true;
    }
    if (verticalOk) {
        *result = std::move(vertical);
        result->diagnostics.requestedOrientation =
            Orientation::Auto;
        return true;
    }
    *result = resultPreference(horizontal) >
            resultPreference(vertical)
        ? std::move(horizontal)
        : std::move(vertical);
    result->diagnostics.requestedOrientation =
        Orientation::Auto;
    if (result->error.empty()) {
        result->error =
            "neither stripe orientation produced a valid quality path";
    }
    return false;
}

const char* orientationName(Orientation orientation) {
    switch (orientation) {
    case Orientation::Auto:
        return "auto";
    case Orientation::Vertical:
        return "vertical";
    case Orientation::Horizontal:
        return "horizontal";
    }
    return "unknown";
}

const char* centerMethodName(CenterMethod method) {
    switch (method) {
    case CenterMethod::BackgroundSubtractedCentroid:
        return "background_centroid";
    case CenterMethod::GaussianDerivativeTaylor:
        return "gaussian_derivative_taylor";
    case CenterMethod::SaturatedHalfHeightMidpoint:
        return "saturated_half_height_midpoint";
    }
    return "unknown";
}

const char* algorithmVersion() {
    return "quality-v2-local-mad-dp-gaussian-taylor";
}

std::string rejectReasonNames(std::uint32_t flags) {
    if (flags == REJECT_NONE) {
        return "NONE";
    }
    const std::pair<std::uint32_t, const char*> names[] = {
        {REJECT_LOW_PROMINENCE, "LOW_PROMINENCE"},
        {REJECT_WIDTH_OUT_OF_RANGE, "WIDTH_OUT_OF_RANGE"},
        {REJECT_SATURATED_WIDE_PLATEAU,
         "SATURATED_WIDE_PLATEAU"},
        {REJECT_SATURATED_ASYMMETRIC,
         "SATURATED_ASYMMETRIC"},
        {REJECT_MULTI_PEAK_AMBIGUOUS,
         "MULTI_PEAK_AMBIGUOUS"},
        {REJECT_PROFILE_ASYMMETRIC, "PROFILE_ASYMMETRIC"},
        {REJECT_FIT_RESIDUAL_HIGH, "FIT_RESIDUAL_HIGH"},
        {REJECT_QUALITY_LOW, "QUALITY_LOW"},
        {REJECT_OUTSIDE_ROI, "OUTSIDE_ROI"},
        {REJECT_OUTSIDE_VALIDITY_MASK,
         "OUTSIDE_VALIDITY_MASK"},
        {REJECT_PATH_JUMP, "PATH_JUMP"},
        {REJECT_PATH_AMBIGUOUS, "PATH_AMBIGUOUS"}
    };
    std::ostringstream output;
    bool first = true;
    for (const std::pair<std::uint32_t, const char*>& item : names) {
        if ((flags & item.first) == 0U) {
            continue;
        }
        if (!first) {
            output << '|';
        }
        output << item.second;
        first = false;
    }
    return output.str();
}

bool hasRejectReason(const Candidate& candidate, RejectReason reason) {
    return (candidate.rejectFlags &
            static_cast<std::uint32_t>(reason)) != 0U;
}

}  // namespace hik_stripe
