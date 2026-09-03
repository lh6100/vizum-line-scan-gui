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
    // A clipped or slightly asymmetric physical ridge can create several
    // adjacent discrete local maxima. Treat those maxima as one optical
    // hypothesis before computing the competing-peak ratio; otherwise the
    // duplicate maxima can erase every real branch on the scanline.
    std::sort(
        line->values.begin(), line->values.end(),
        [](const Candidate& first, const Candidate& second) {
            if (first.prominence != second.prominence) {
                return first.prominence > second.prominence;
            }
            return first.peakIndex < second.peakIndex;
        });
    std::vector<Candidate> clustered;
    clustered.reserve(line->values.size());
    for (const Candidate& candidate : line->values) {
        bool duplicate = false;
        for (const Candidate& representative : clustered) {
            const double dx = candidate.pixel.x - representative.pixel.x;
            const double dy = candidate.pixel.y - representative.pixel.y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            const double mergeDistance = std::max(
                options.peakMergeMinimumDistancePx,
                options.peakMergeFwhmScale *
                    std::min(candidate.fwhmPx, representative.fwhmPx));
            if (distance <= mergeDistance) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            clustered.push_back(candidate);
        }
    }
    line->values.swap(clustered);
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
            // This is deliberately a soft lattice flag. usableForPath()
            // excludes only fatal optical/mask failures, so continuity can
            // resolve a multi-peak scanline instead of turning it into GAP.
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
            if (candidate.usableForPath()) {
                ++diagnostics->pathUsableCandidateCount;
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
    int currentLine{-1};
    int currentCandidate{-1};
    int previousLine{-1};
    int previousCandidate{-1};
    int bestPreviousState{-1};
    int bestNextState{-1};
    double forwardCost{std::numeric_limits<double>::infinity()};
    double backwardCost{std::numeric_limits<double>::infinity()};
    double lastSlope{0.0};
    bool hasSlope{false};
    int pointCount{0};
};

struct EdgeLookup {
    int previousLine{-1};
    int previousCandidate{-1};
    int stateId{-1};
};

struct LocalAlternative {
    std::size_t diagnosticIndex{0U};
    int alternateState{-1};
    double totalCost{std::numeric_limits<double>::infinity()};
    int differenceCount{0};
};

struct IntervalHypothesis {
    double pathCost{std::numeric_limits<double>::infinity()};
    std::vector<std::pair<int, int>> path;
};

double pathNodeCost(const Candidate& candidate, const Options& options) {
    return -options.pathCandidateReward * candidate.quality;
}

bool transitionIncrement(const std::vector<LineCandidates>& lines,
                         const PathState& previousState,
                         int currentLine,
                         int currentCandidate,
                         Orientation orientation,
                         const Options& options,
                         double* increment) {
    if (!increment || previousState.currentLine < 0 ||
        previousState.currentCandidate < 0) {
        return false;
    }
    const Candidate& previous =
        lines[static_cast<std::size_t>(previousState.currentLine)]
             .values[static_cast<std::size_t>(
                 previousState.currentCandidate)];
    const Candidate& current =
        lines[static_cast<std::size_t>(currentLine)]
             .values[static_cast<std::size_t>(currentCandidate)];
    const int scanDelta =
        current.scanIndex - previous.scanIndex;
    if (scanDelta <= 0) {
        return false;
    }
    const int gap = scanDelta - 1;
    if (gap > options.pathMaximumGap) {
        return false;
    }
    const double displacement =
        minorCoordinate(current, orientation) -
        minorCoordinate(previous, orientation);
    const double signedSlope =
        displacement / static_cast<double>(scanDelta);
    const double step = std::fabs(signedSlope);
    if (step > options.pathMaximumStepPx) {
        return false;
    }
    if (gap > 0) {
        // Missing/ambiguous columns must not make a branch jump appear safe
        // merely because the displacement was divided by scanDelta.
        const double predicted = minorCoordinate(previous, orientation) +
            (previousState.hasSlope
                 ? previousState.lastSlope *
                       static_cast<double>(scanDelta)
                 : 0.0);
        if (std::fabs(
                minorCoordinate(current, orientation) - predicted) >
                options.pathMaximumPredictionResidualPx ||
            std::fabs(displacement) >
                options.pathMaximumStepPx +
                    options.pathMaximumPredictionResidualPx) {
            return false;
        }
    }

    double transitionCost =
        options.pathPositionPenalty * huber(step, 1.0);
    if (previousState.hasSlope) {
        transitionCost += options.pathCurvaturePenalty *
            huber(
                std::fabs(signedSlope - previousState.lastSlope),
                0.5);
    }
    if (gap > 0) {
        transitionCost += options.pathGapOpenPenalty +
            options.pathGapExtendPenalty * static_cast<double>(gap);
    }
    *increment = transitionCost + pathNodeCost(current, options);
    return std::isfinite(*increment);
}

std::vector<std::pair<int, int>> tracePrefix(
        const std::vector<PathState>& states,
        int stateId) {
    std::vector<std::pair<int, int>> reversed;
    while (stateId >= 0) {
        const PathState& state =
            states[static_cast<std::size_t>(stateId)];
        reversed.push_back(std::make_pair(
            state.currentLine, state.currentCandidate));
        stateId = state.bestPreviousState;
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

std::vector<std::pair<int, int>> traceComplete(
        const std::vector<PathState>& states,
        int stateId) {
    std::vector<std::pair<int, int>> path =
        tracePrefix(states, stateId);
    int nextState =
        states[static_cast<std::size_t>(stateId)].bestNextState;
    while (nextState >= 0) {
        const PathState& state =
            states[static_cast<std::size_t>(nextState)];
        path.push_back(std::make_pair(
            state.currentLine, state.currentCandidate));
        nextState = state.bestNextState;
    }
    return path;
}

int pathDifferenceCount(
        const std::vector<std::pair<int, int>>& first,
        const std::vector<std::pair<int, int>>& second) {
    std::map<int, int> firstByLine;
    std::map<int, int> secondByLine;
    for (const std::pair<int, int>& node : first) {
        firstByLine[node.first] = node.second;
    }
    for (const std::pair<int, int>& node : second) {
        secondByLine[node.first] = node.second;
    }
    std::map<int, int>::const_iterator firstIt = firstByLine.begin();
    std::map<int, int>::const_iterator secondIt = secondByLine.begin();
    int different = 0;
    while (firstIt != firstByLine.end() ||
           secondIt != secondByLine.end()) {
        if (secondIt == secondByLine.end() ||
            (firstIt != firstByLine.end() &&
             firstIt->first < secondIt->first)) {
            ++different;
            ++firstIt;
        } else if (firstIt == firstByLine.end() ||
                   secondIt->first < firstIt->first) {
            ++different;
            ++secondIt;
        } else {
            if (firstIt->second != secondIt->second) {
                ++different;
            }
            ++firstIt;
            ++secondIt;
        }
    }
    return different;
}

void findDivergenceBoundaries(
        const std::vector<std::pair<int, int>>& bestPath,
        const std::vector<std::pair<int, int>>& alternatePath,
        int evidenceLine,
        int lineCount,
        int* coreFirstLine,
        int* coreLastLine,
        int* leftAnchorLine,
        int* rightAnchorLine) {
    std::map<int, int> bestByLine;
    std::map<int, int> alternateByLine;
    for (const std::pair<int, int>& node : bestPath) {
        bestByLine[node.first] = node.second;
    }
    for (const std::pair<int, int>& node : alternatePath) {
        alternateByLine[node.first] = node.second;
    }
    const auto isCommonCandidate =
        [&bestByLine, &alternateByLine](int line) {
            const std::map<int, int>::const_iterator best =
                bestByLine.find(line);
            const std::map<int, int>::const_iterator alternate =
                alternateByLine.find(line);
            return best != bestByLine.end() &&
                   alternate != alternateByLine.end() &&
                   best->second == alternate->second;
        };

    int leftAnchor = -1;
    for (int line = evidenceLine - 1; line >= 0; --line) {
        if (isCommonCandidate(line)) {
            leftAnchor = line;
            break;
        }
    }
    int rightAnchor = -1;
    for (int line = evidenceLine + 1;
         line < lineCount; ++line) {
        if (isCommonCandidate(line)) {
            rightAnchor = line;
            break;
        }
    }
    if (leftAnchorLine) {
        *leftAnchorLine = leftAnchor;
    }
    if (rightAnchorLine) {
        *rightAnchorLine = rightAnchor;
    }
    if (coreFirstLine) {
        *coreFirstLine = leftAnchor >= 0
            ? leftAnchor + 1 : 0;
    }
    if (coreLastLine) {
        *coreLastLine = rightAnchor >= 0
            ? rightAnchor - 1 : std::max(0, lineCount - 1);
    }
}

int findEdgeState(
        const std::vector<EdgeLookup>& lookup,
        int previousLine,
        int previousCandidate) {
    for (const EdgeLookup& entry : lookup) {
        if (entry.previousLine == previousLine &&
            entry.previousCandidate == previousCandidate) {
            return entry.stateId;
        }
    }
    return -1;
}

Candidate candidateAt(
        const std::vector<LineCandidates>& lines,
        const std::pair<int, int>& node) {
    return lines[static_cast<std::size_t>(node.first)]
                .values[static_cast<std::size_t>(node.second)];
}

bool optimizePath(std::vector<LineCandidates>* lines,
                  Orientation orientation,
                  const Options& options,
                  Result* result) {
    if (!lines || !result || lines->empty()) {
        return false;
    }
    std::vector<PathState> states;
    std::vector<std::vector<std::vector<int>>> statesEndingAt(
        lines->size());
    std::vector<std::vector<std::vector<EdgeLookup>>> edgeLookup(
        lines->size());
    for (std::size_t lineOffset = 0U;
         lineOffset < lines->size(); ++lineOffset) {
        LineCandidates& line = (*lines)[lineOffset];
        statesEndingAt[lineOffset].resize(line.values.size());
        edgeLookup[lineOffset].resize(line.values.size());
        for (std::size_t candidateIndex = 0U;
             candidateIndex < line.values.size(); ++candidateIndex) {
            Candidate& candidate = line.values[candidateIndex];
            if (!candidate.usableForPath()) {
                continue;
            }

            PathState start;
            start.currentLine = static_cast<int>(lineOffset);
            start.currentCandidate =
                static_cast<int>(candidateIndex);
            start.forwardCost =
                static_cast<double>(lineOffset) *
                    options.pathGapExtendPenalty +
                pathNodeCost(candidate, options);
            start.pointCount = 1;
            const int startId = static_cast<int>(states.size());
            states.push_back(start);
            statesEndingAt[lineOffset][candidateIndex].push_back(
                startId);

            const std::size_t earliest =
                lineOffset > static_cast<std::size_t>(
                    options.pathMaximumGap + 1)
                ? lineOffset - static_cast<std::size_t>(
                      options.pathMaximumGap + 1)
                : 0U;
            for (std::size_t previousLine = earliest;
                 previousLine < lineOffset; ++previousLine) {
                for (std::size_t previousCandidate = 0U;
                     previousCandidate <
                         (*lines)[previousLine].values.size();
                     ++previousCandidate) {
                    if (!(*lines)[previousLine]
                             .values[previousCandidate]
                             .usableForPath()) {
                        continue;
                    }
                    double bestCost =
                        std::numeric_limits<double>::infinity();
                    int bestPreviousState = -1;
                    double edgeSlope = 0.0;
                    for (const int sourceId :
                         statesEndingAt[previousLine]
                                       [previousCandidate]) {
                        double increment = 0.0;
                        if (!transitionIncrement(
                                *lines,
                                states[static_cast<std::size_t>(
                                    sourceId)],
                                static_cast<int>(lineOffset),
                                static_cast<int>(candidateIndex),
                                orientation, options, &increment)) {
                            continue;
                        }
                        const double cost =
                            states[static_cast<std::size_t>(
                                sourceId)].forwardCost +
                            increment;
                        if (cost < bestCost) {
                            bestCost = cost;
                            bestPreviousState = sourceId;
                        }
                    }
                    if (bestPreviousState < 0) {
                        continue;
                    }
                    const Candidate& previous =
                        (*lines)[previousLine]
                            .values[previousCandidate];
                    const int scanDelta =
                        candidate.scanIndex - previous.scanIndex;
                    edgeSlope =
                        (minorCoordinate(candidate, orientation) -
                         minorCoordinate(previous, orientation)) /
                        static_cast<double>(scanDelta);

                    PathState edge;
                    edge.currentLine =
                        static_cast<int>(lineOffset);
                    edge.currentCandidate =
                        static_cast<int>(candidateIndex);
                    edge.previousLine =
                        static_cast<int>(previousLine);
                    edge.previousCandidate =
                        static_cast<int>(previousCandidate);
                    edge.bestPreviousState = bestPreviousState;
                    edge.forwardCost = bestCost;
                    edge.lastSlope = edgeSlope;
                    edge.hasSlope = true;
                    edge.pointCount =
                        states[static_cast<std::size_t>(
                            bestPreviousState)].pointCount + 1;
                    const int edgeId =
                        static_cast<int>(states.size());
                    states.push_back(edge);
                    statesEndingAt[lineOffset][candidateIndex]
                        .push_back(edgeId);
                    EdgeLookup lookup;
                    lookup.previousLine =
                        static_cast<int>(previousLine);
                    lookup.previousCandidate =
                        static_cast<int>(previousCandidate);
                    lookup.stateId = edgeId;
                    edgeLookup[lineOffset][candidateIndex]
                        .push_back(lookup);
                }
            }
        }
    }

    const double allGapCost =
        static_cast<double>(lines->size()) *
        options.pathGapExtendPenalty;
    double bestCost = allGapCost;
    int bestEndingState = -1;
    int bestPointCount = 0;
    for (std::size_t stateIndex = 0U;
         stateIndex < states.size(); ++stateIndex) {
        const PathState& state = states[stateIndex];
        const double trailing =
            static_cast<double>(
                lines->size() -
                static_cast<std::size_t>(state.currentLine) - 1U) *
            options.pathGapExtendPenalty;
        const double total = state.forwardCost + trailing;
        if (total < bestCost ||
            (total == bestCost &&
             state.pointCount > bestPointCount)) {
            bestCost = total;
            bestEndingState = static_cast<int>(stateIndex);
            bestPointCount = state.pointCount;
        }
    }
    if (bestEndingState < 0) {
        result->error =
            "quality path optimizer selected only GAP states";
        result->diagnostics.bestPathCost = allGapCost;
        return false;
    }

    // Backward cost on the same second-order edge graph. Re-enumerating
    // outgoing transitions avoids storing a very large adjacency list.
    for (std::vector<PathState>::reverse_iterator stateIt =
             states.rbegin();
         stateIt != states.rend(); ++stateIt) {
        PathState& state = *stateIt;
        state.backwardCost =
            static_cast<double>(
                lines->size() -
                static_cast<std::size_t>(state.currentLine) - 1U) *
            options.pathGapExtendPenalty;
        const std::size_t latest = std::min(
            lines->size(),
            static_cast<std::size_t>(state.currentLine) +
                static_cast<std::size_t>(
                    options.pathMaximumGap + 2));
        for (std::size_t nextLine =
                 static_cast<std::size_t>(state.currentLine + 1);
             nextLine < latest; ++nextLine) {
            for (std::size_t nextCandidate = 0U;
                 nextCandidate <
                     (*lines)[nextLine].values.size();
                 ++nextCandidate) {
                const int nextState = findEdgeState(
                    edgeLookup[nextLine][nextCandidate],
                    state.currentLine, state.currentCandidate);
                if (nextState < 0) {
                    continue;
                }
                double increment = 0.0;
                if (!transitionIncrement(
                        *lines, state,
                        static_cast<int>(nextLine),
                        static_cast<int>(nextCandidate),
                        orientation, options, &increment)) {
                    continue;
                }
                const double suffix =
                    increment +
                    states[static_cast<std::size_t>(
                        nextState)].backwardCost;
                if (suffix < state.backwardCost) {
                    state.backwardCost = suffix;
                    state.bestNextState = nextState;
                }
            }
        }
    }

    const std::vector<std::pair<int, int>> bestPath =
        tracePrefix(states, bestEndingState);
    result->provisionalSelected.reserve(bestPath.size());
    for (const std::pair<int, int>& node : bestPath) {
        result->provisionalSelected.push_back(
            candidateAt(*lines, node));
    }

    std::map<int, int> bestCandidateByLine;
    for (const std::pair<int, int>& node : bestPath) {
        bestCandidateByLine[node.first] = node.second;
    }
    std::vector<LocalAlternative> alternatives;
    double secondCost =
        std::numeric_limits<double>::infinity();
    double minimumRawMargin =
        std::numeric_limits<double>::infinity();
    double minimumLocalMargin =
        std::numeric_limits<double>::infinity();
    for (const std::pair<int, int>& bestNode : bestPath) {
        const Candidate& selected =
            (*lines)[static_cast<std::size_t>(bestNode.first)]
                .values[static_cast<std::size_t>(bestNode.second)];
        PathScanlineDiagnostic diagnostic;
        diagnostic.scanIndex = selected.scanIndex;
        diagnostic.hasSelected = true;
        diagnostic.selectedPixel = selected.pixel;

        int alternateState = -1;
        double alternateTotal =
            std::numeric_limits<double>::infinity();
        const std::vector<LineCandidates>::const_reference line =
            (*lines)[static_cast<std::size_t>(bestNode.first)];
        for (std::size_t candidateIndex = 0U;
             candidateIndex < line.values.size(); ++candidateIndex) {
            if (static_cast<int>(candidateIndex) ==
                    bestNode.second ||
                !line.values[candidateIndex].usableForPath()) {
                continue;
            }
            const double separation = std::fabs(
                minorCoordinate(
                    line.values[candidateIndex], orientation) -
                minorCoordinate(selected, orientation));
            if (separation <
                options.pathAmbiguityMinimumSeparationPx) {
                continue;
            }
            for (const int stateId :
                 statesEndingAt[static_cast<std::size_t>(
                     bestNode.first)][candidateIndex]) {
                const PathState& state =
                    states[static_cast<std::size_t>(stateId)];
                const double total =
                    state.forwardCost + state.backwardCost;
                if (total < alternateTotal) {
                    alternateTotal = total;
                    alternateState = stateId;
                }
            }
        }
        if (alternateState >= 0) {
            const PathState& alternate =
                states[static_cast<std::size_t>(alternateState)];
            const Candidate& alternateCandidate =
                (*lines)[static_cast<std::size_t>(
                    alternate.currentLine)]
                    .values[static_cast<std::size_t>(
                        alternate.currentCandidate)];
            const std::vector<std::pair<int, int>> alternatePath =
                traceComplete(states, alternateState);
            const int differenceCount = std::max(
                1, pathDifferenceCount(bestPath, alternatePath));
            const double rawMargin = std::max(
                0.0, alternateTotal - bestCost);
            const double localMargin =
                rawMargin / static_cast<double>(differenceCount);
            diagnostic.hasAlternate = true;
            diagnostic.alternatePixel = alternateCandidate.pixel;
            diagnostic.separationPx = std::fabs(
                minorCoordinate(alternateCandidate, orientation) -
                minorCoordinate(selected, orientation));
            diagnostic.localCostMargin = localMargin;
            secondCost = std::min(secondCost, alternateTotal);
            minimumRawMargin =
                std::min(minimumRawMargin, rawMargin);
            minimumLocalMargin =
                std::min(minimumLocalMargin, localMargin);
            if (localMargin <=
                    options.pathAmbiguityMarginPerPoint +
                        kEpsilon) {
                LocalAlternative evidence;
                evidence.diagnosticIndex =
                    result->pathDiagnostics.size();
                evidence.alternateState = alternateState;
                evidence.totalCost = alternateTotal;
                evidence.differenceCount = differenceCount;
                alternatives.push_back(evidence);
            }
        }
        result->pathDiagnostics.push_back(diagnostic);
    }

    result->diagnostics.bestPathCost = bestCost;
    result->diagnostics.secondPathCost = secondCost;
    result->diagnostics.pathCostMargin = minimumRawMargin;
    result->diagnostics.pathCostMarginPerPoint =
        minimumLocalMargin;

    // Merge consecutive low-margin evidence into divergence/reconvergence
    // intervals, then pad them before publishing any points.
    std::vector<std::vector<IntervalHypothesis>>
        intervalHypotheses;
    std::size_t evidenceBegin = 0U;
    while (evidenceBegin < alternatives.size()) {
        std::size_t evidenceEnd = evidenceBegin;
        int previousScan =
            result->pathDiagnostics[
                alternatives[evidenceBegin].diagnosticIndex]
                .scanIndex;
        while (evidenceEnd + 1U < alternatives.size()) {
            const int nextScan =
                result->pathDiagnostics[
                    alternatives[evidenceEnd + 1U]
                        .diagnosticIndex].scanIndex;
            if (nextScan - previousScan >
                std::max(
                    1,
                    options.pathAmbiguityPaddingScanlines + 1)) {
                break;
            }
            ++evidenceEnd;
            previousScan = nextScan;
        }

        std::size_t representative = evidenceBegin;
        double intervalMinimumMargin =
            std::numeric_limits<double>::infinity();
        double intervalMaximumSeparation = 0.0;
        for (std::size_t index = evidenceBegin;
             index <= evidenceEnd; ++index) {
            const PathScanlineDiagnostic& diagnostic =
                result->pathDiagnostics[
                    alternatives[index].diagnosticIndex];
            intervalMinimumMargin = std::min(
                intervalMinimumMargin,
                diagnostic.localCostMargin);
            intervalMaximumSeparation = std::max(
                intervalMaximumSeparation,
                diagnostic.separationPx);
            if (diagnostic.localCostMargin <
                result->pathDiagnostics[
                    alternatives[representative]
                        .diagnosticIndex].localCostMargin) {
                representative = index;
            }
        }

        const std::vector<std::pair<int, int>> alternatePath =
            traceComplete(
                states,
                alternatives[representative].alternateState);
        int coreFirstLine = 0;
        int coreLastLine =
            static_cast<int>(lines->size()) - 1;
        int leftAnchorLine = -1;
        int rightAnchorLine = -1;
        findDivergenceBoundaries(
            bestPath, alternatePath,
            states[static_cast<std::size_t>(
                alternatives[representative].alternateState)]
                .currentLine,
            static_cast<int>(lines->size()),
            &coreFirstLine, &coreLastLine,
            &leftAnchorLine, &rightAnchorLine);

        MultipathInterval interval;
        interval.intervalId =
            static_cast<int>(result->multipathIntervals.size());
        interval.minimumLocalCostMargin =
            intervalMinimumMargin;
        interval.maximumSeparationPx =
            intervalMaximumSeparation;
        interval.coreFirstScanIndex =
            (*lines)[static_cast<std::size_t>(
                coreFirstLine)].scanIndex;
        interval.coreLastScanIndex =
            (*lines)[static_cast<std::size_t>(
                coreLastLine)].scanIndex;
        interval.leftBoundaryOpen = leftAnchorLine < 0;
        interval.rightBoundaryOpen = rightAnchorLine < 0;
        interval.leftAnchorScanIndex = leftAnchorLine >= 0
            ? (*lines)[static_cast<std::size_t>(
                  leftAnchorLine)].scanIndex
            : -1;
        interval.rightAnchorScanIndex = rightAnchorLine >= 0
            ? (*lines)[static_cast<std::size_t>(
                  rightAnchorLine)].scanIndex
            : -1;
        interval.firstScanIndex = interval.leftBoundaryOpen
            ? lines->front().scanIndex
            : std::min(
                  interval.leftAnchorScanIndex,
                  std::max(
                      lines->front().scanIndex,
                      interval.coreFirstScanIndex -
                          options
                              .pathAmbiguityPaddingScanlines));
        interval.lastScanIndex = interval.rightBoundaryOpen
            ? lines->back().scanIndex
            : std::max(
                  interval.rightAnchorScanIndex,
                  std::min(
                      lines->back().scanIndex,
                      interval.coreLastScanIndex +
                          options
                              .pathAmbiguityPaddingScanlines));

        IntervalHypothesis hypothesis;
        hypothesis.pathCost =
            alternatives[representative].totalCost;
        hypothesis.path = alternatePath;
        result->multipathIntervals.push_back(interval);
        intervalHypotheses.push_back(
            std::vector<IntervalHypothesis>(1U, hypothesis));
        evidenceBegin = evidenceEnd + 1U;
    }

    if (!result->multipathIntervals.empty()) {
        std::vector<MultipathInterval> mergedIntervals;
        std::vector<std::vector<IntervalHypothesis>>
            mergedHypotheses;
        std::vector<std::size_t> intervalOrder(
            result->multipathIntervals.size());
        std::iota(
            intervalOrder.begin(), intervalOrder.end(), 0U);
        std::sort(
            intervalOrder.begin(), intervalOrder.end(),
            [result](std::size_t first, std::size_t second) {
                const MultipathInterval& firstInterval =
                    result->multipathIntervals[first];
                const MultipathInterval& secondInterval =
                    result->multipathIntervals[second];
                if (firstInterval.firstScanIndex !=
                    secondInterval.firstScanIndex) {
                    return firstInterval.firstScanIndex <
                           secondInterval.firstScanIndex;
                }
                return firstInterval.lastScanIndex <
                       secondInterval.lastScanIndex;
            });
        const int mergeDistance = std::max(
            1, 2 * options.pathAmbiguityPaddingScanlines + 1);
        for (const std::size_t index : intervalOrder) {
            const MultipathInterval& next =
                result->multipathIntervals[index];
            if (mergedIntervals.empty() ||
                next.firstScanIndex >
                    mergedIntervals.back().lastScanIndex +
                        mergeDistance) {
                mergedIntervals.push_back(next);
                mergedHypotheses.push_back(
                    intervalHypotheses[index]);
                continue;
            }

            MultipathInterval& merged =
                mergedIntervals.back();
            merged.firstScanIndex = std::min(
                merged.firstScanIndex, next.firstScanIndex);
            merged.lastScanIndex = std::max(
                merged.lastScanIndex, next.lastScanIndex);
            merged.coreFirstScanIndex = std::min(
                merged.coreFirstScanIndex,
                next.coreFirstScanIndex);
            merged.coreLastScanIndex = std::max(
                merged.coreLastScanIndex,
                next.coreLastScanIndex);
            merged.minimumLocalCostMargin = std::min(
                merged.minimumLocalCostMargin,
                next.minimumLocalCostMargin);
            merged.maximumSeparationPx = std::max(
                merged.maximumSeparationPx,
                next.maximumSeparationPx);
            if (next.lastScanIndex >=
                merged.lastScanIndex) {
                merged.rightAnchorScanIndex =
                    next.rightAnchorScanIndex;
                merged.rightBoundaryOpen =
                    next.rightBoundaryOpen;
            }
            mergedHypotheses.back().insert(
                mergedHypotheses.back().end(),
                intervalHypotheses[index].begin(),
                intervalHypotheses[index].end());
        }
        result->multipathIntervals.swap(mergedIntervals);
        intervalHypotheses.swap(mergedHypotheses);

        for (std::size_t intervalIndex = 0U;
             intervalIndex <
                 result->multipathIntervals.size();
             ++intervalIndex) {
            MultipathInterval& interval =
                result->multipathIntervals[intervalIndex];
            interval.intervalId =
                static_cast<int>(intervalIndex);
            interval.branches.clear();

            const auto appendBranch =
                [&interval, lines](
                    const std::vector<std::pair<int, int>>& path,
                    double pathCost) {
                    MultipathBranch branch;
                    branch.branchId =
                        static_cast<int>(
                            interval.branches.size());
                    branch.pathCost = pathCost;
                    for (const std::pair<int, int>& node : path) {
                        Candidate candidate =
                            candidateAt(*lines, node);
                        if (candidate.scanIndex <
                                interval.firstScanIndex ||
                            candidate.scanIndex >
                                interval.lastScanIndex) {
                            continue;
                        }
                        candidate.rejectFlags |=
                            REJECT_PATH_AMBIGUOUS |
                            REJECT_AMBIGUOUS_MULTIPATH;
                        candidate.ambiguityIntervalId =
                            interval.intervalId;
                        candidate.ambiguityBranchId =
                            branch.branchId;
                        branch.candidates.push_back(candidate);
                    }
                    if (branch.candidates.empty()) {
                        return;
                    }
                    for (const MultipathBranch& existing :
                         interval.branches) {
                        if (existing.candidates.size() !=
                            branch.candidates.size()) {
                            continue;
                        }
                        bool same = true;
                        for (std::size_t candidateIndex = 0U;
                             candidateIndex <
                                 branch.candidates.size();
                             ++candidateIndex) {
                            if (existing.candidates[candidateIndex]
                                    .scanIndex !=
                                    branch.candidates[candidateIndex]
                                        .scanIndex ||
                                existing.candidates[candidateIndex]
                                    .peakIndex !=
                                    branch.candidates[candidateIndex]
                                        .peakIndex) {
                                same = false;
                                break;
                            }
                        }
                        if (same) {
                            return;
                        }
                    }
                    interval.branches.push_back(branch);
                };
            appendBranch(bestPath, bestCost);
            for (const IntervalHypothesis& hypothesis :
                 intervalHypotheses[intervalIndex]) {
                appendBranch(
                    hypothesis.path, hypothesis.pathCost);
            }
        }
    }

    for (PathScanlineDiagnostic& diagnostic :
         result->pathDiagnostics) {
        for (MultipathInterval& interval :
             result->multipathIntervals) {
            if (diagnostic.scanIndex >= interval.firstScanIndex &&
                diagnostic.scanIndex <= interval.lastScanIndex) {
                diagnostic.ambiguityIntervalId =
                    interval.intervalId;
                if (diagnostic.hasAlternate) {
                    interval.maximumSeparationPx = std::max(
                        interval.maximumSeparationPx,
                        diagnostic.separationPx);
                }
                break;
            }
        }
    }
    for (Candidate& candidate : result->provisionalSelected) {
        for (const MultipathInterval& interval :
             result->multipathIntervals) {
            if (candidate.scanIndex >= interval.firstScanIndex &&
                candidate.scanIndex <= interval.lastScanIndex) {
                candidate.rejectFlags |=
                    REJECT_PATH_AMBIGUOUS |
                    REJECT_AMBIGUOUS_MULTIPATH;
                candidate.ambiguityIntervalId =
                    interval.intervalId;
                candidate.ambiguityBranchId = 0;
                break;
            }
        }
        if (candidate.ambiguityIntervalId >= 0) {
            continue;
        }
        Candidate publishable = candidate;
        publishable.rejectFlags &=
            ~static_cast<std::uint32_t>(
                REJECT_MULTI_PEAK_AMBIGUOUS);
        publishable.ambiguityIntervalId = -1;
        publishable.ambiguityBranchId = -1;
        if (publishable.accepted()) {
            result->selected.push_back(publishable);
        }
    }
    result->diagnostics.ambiguousPathPointCount =
        result->provisionalSelected.size() -
        result->selected.size();
    std::size_t protectedScanlineCount = 0U;
    for (const LineCandidates& line : *lines) {
        if (std::any_of(
                result->multipathIntervals.begin(),
                result->multipathIntervals.end(),
                [&line](const MultipathInterval& interval) {
                    return line.scanIndex >= interval.firstScanIndex &&
                           line.scanIndex <= interval.lastScanIndex;
                })) {
            ++protectedScanlineCount;
        }
    }
    result->diagnostics.multipathAmbiguousScanlineCount =
        protectedScanlineCount;
    result->diagnostics.multipathIntervalCount =
        result->multipathIntervals.size();
    return !result->provisionalSelected.empty();
}

bool lineHasUsableCandidate(const LineCandidates& line) {
    return std::any_of(
        line.values.begin(), line.values.end(),
        [](const Candidate& candidate) {
            return candidate.usableForPath();
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
    std::size_t lastUsable = 0U;
    bool insideSpan = false;
    for (std::size_t index = 0U; index < lines.size(); ++index) {
        if (!lineHasUsableCandidate(lines[index])) {
            continue;
        }
        if (!insideSpan) {
            spanBegin = index;
            lastUsable = index;
            insideSpan = true;
            continue;
        }
        const int scanGap =
            lines[index].scanIndex -
            lines[lastUsable].scanIndex - 1;
        if (scanGap > options.pathMaximumGap) {
            spans.push_back(
                std::make_pair(spanBegin, lastUsable));
            spanBegin = index;
        }
        lastUsable = index;
    }
    if (insideSpan) {
        spans.push_back(std::make_pair(spanBegin, lastUsable));
    }
    if (spans.empty()) {
        result->error =
            "no scanline contains a path-usable stripe candidate";
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
    std::size_t multipathScanlineCount = 0U;
    std::size_t multipathIntervalCount = 0U;
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
        const int intervalOffset =
            static_cast<int>(result->multipathIntervals.size());
        for (Candidate& candidate :
             segmentResult.provisionalSelected) {
            if (candidate.ambiguityIntervalId >= 0) {
                candidate.ambiguityIntervalId += intervalOffset;
            }
        }
        for (PathScanlineDiagnostic& diagnostic :
             segmentResult.pathDiagnostics) {
            if (diagnostic.ambiguityIntervalId >= 0) {
                diagnostic.ambiguityIntervalId += intervalOffset;
            }
        }
        for (MultipathInterval& interval :
             segmentResult.multipathIntervals) {
            interval.intervalId += intervalOffset;
            for (MultipathBranch& branch : interval.branches) {
                for (Candidate& candidate : branch.candidates) {
                    candidate.ambiguityIntervalId =
                        interval.intervalId;
                }
            }
        }
        result->provisionalSelected.insert(
            result->provisionalSelected.end(),
            segmentResult.provisionalSelected.begin(),
            segmentResult.provisionalSelected.end());
        result->selected.insert(
            result->selected.end(),
            segmentResult.selected.begin(),
            segmentResult.selected.end());
        result->pathDiagnostics.insert(
            result->pathDiagnostics.end(),
            segmentResult.pathDiagnostics.begin(),
            segmentResult.pathDiagnostics.end());
        result->multipathIntervals.insert(
            result->multipathIntervals.end(),
            segmentResult.multipathIntervals.begin(),
            segmentResult.multipathIntervals.end());
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
        multipathScanlineCount +=
            segmentResult.diagnostics
                .multipathAmbiguousScanlineCount;
        multipathIntervalCount +=
            segmentResult.diagnostics.multipathIntervalCount;
    }
    if (successfulSegments == 0U ||
        result->provisionalSelected.empty()) {
        result->error =
            "no quality path segment survived optimization";
        return false;
    }
    std::sort(
        result->provisionalSelected.begin(),
        result->provisionalSelected.end(),
        [](const Candidate& first, const Candidate& second) {
            if (first.scanIndex != second.scanIndex) {
                return first.scanIndex < second.scanIndex;
            }
            return first.peakIndex < second.peakIndex;
        });
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
    result->diagnostics.multipathAmbiguousScanlineCount =
        multipathScanlineCount;
    result->diagnostics.multipathIntervalCount =
        multipathIntervalCount;
    return true;
}

void propagateMultipathAnnotations(Result* result) {
    if (!result) {
        return;
    }
    struct Annotation {
        std::uint32_t flags{0U};
        int intervalId{-1};
        int branchId{-1};
    };
    std::map<std::pair<int, int>, Annotation> annotations;
    for (const MultipathInterval& interval :
         result->multipathIntervals) {
        for (const MultipathBranch& branch : interval.branches) {
            for (const Candidate& candidate : branch.candidates) {
                Annotation& annotation = annotations[
                    std::make_pair(
                        candidate.scanIndex,
                        candidate.peakIndex)];
                annotation.flags |=
                    REJECT_PATH_AMBIGUOUS |
                    REJECT_AMBIGUOUS_MULTIPATH;
                if (annotation.intervalId < 0) {
                    annotation.intervalId = interval.intervalId;
                    annotation.branchId = branch.branchId;
                } else if (annotation.intervalId !=
                               interval.intervalId ||
                           annotation.branchId !=
                               branch.branchId) {
                    annotation.branchId = -1;
                }
            }
        }
    }
    for (Candidate& candidate : result->candidates) {
        const std::map<std::pair<int, int>, Annotation>::const_iterator
            found = annotations.find(std::make_pair(
                candidate.scanIndex, candidate.peakIndex));
        if (found == annotations.end()) {
            continue;
        }
        candidate.rejectFlags |= found->second.flags;
        candidate.ambiguityIntervalId =
            found->second.intervalId;
        candidate.ambiguityBranchId =
            found->second.branchId;
    }
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
    propagateMultipathAnnotations(result);

    result->diagnostics.provisionalSelectedPointCount =
        result->provisionalSelected.size();
    result->diagnostics.publishableSelectedPointCount =
        result->selected.size();
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
      pathAmbiguityMarginPerPoint(2.0),
      peakMergeMinimumDistancePx(1.25),
      peakMergeFwhmScale(0.45),
      pathAmbiguityMinimumSeparationPx(3.0),
      pathAmbiguityPaddingScanlines(2),
      pathMaximumPredictionResidualPx(4.0) {}

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
        pathAmbiguityMarginPerPoint < 0.0 ||
        !std::isfinite(peakMergeMinimumDistancePx) ||
        peakMergeMinimumDistancePx < 0.0 ||
        !std::isfinite(peakMergeFwhmScale) ||
        peakMergeFwhmScale < 0.0 ||
        !std::isfinite(pathAmbiguityMinimumSeparationPx) ||
        pathAmbiguityMinimumSeparationPx <= 0.0 ||
        pathAmbiguityPaddingScanlines < 0 ||
        !std::isfinite(pathMaximumPredictionResidualPx) ||
        pathMaximumPredictionResidualPx <= 0.0) {
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
      smoothedSecondDerivative(0.0), rejectFlags(REJECT_NONE),
      ambiguityIntervalId(-1), ambiguityBranchId(-1) {}

bool Candidate::usableForPath() const {
    const std::uint32_t fatalMask =
        REJECT_LOW_PROMINENCE |
        REJECT_WIDTH_OUT_OF_RANGE |
        REJECT_SATURATED_WIDE_PLATEAU |
        REJECT_SATURATED_ASYMMETRIC |
        REJECT_PROFILE_ASYMMETRIC |
        REJECT_FIT_RESIDUAL_HIGH |
        REJECT_QUALITY_LOW |
        REJECT_OUTSIDE_ROI |
        REJECT_OUTSIDE_VALIDITY_MASK |
        REJECT_PATH_JUMP |
        REJECT_PATH_AMBIGUOUS |
        REJECT_AMBIGUOUS_MULTIPATH;
    return (rejectFlags & fatalMask) == 0U;
}

bool Candidate::accepted() const {
    return rejectFlags == REJECT_NONE;
}

PathScanlineDiagnostic::PathScanlineDiagnostic()
    : scanIndex(-1), hasSelected(false), selectedPixel(0.0, 0.0),
      hasAlternate(false), alternatePixel(0.0, 0.0),
      separationPx(0.0),
      localCostMargin(std::numeric_limits<double>::infinity()),
      ambiguityIntervalId(-1) {}

MultipathBranch::MultipathBranch()
    : branchId(-1),
      pathCost(std::numeric_limits<double>::infinity()) {}

MultipathInterval::MultipathInterval()
    : intervalId(-1), firstScanIndex(-1), lastScanIndex(-1),
      coreFirstScanIndex(-1), coreLastScanIndex(-1),
      leftAnchorScanIndex(-1), rightAnchorScanIndex(-1),
      leftBoundaryOpen(true), rightBoundaryOpen(true),
      minimumLocalCostMargin(
          std::numeric_limits<double>::infinity()),
      maximumSeparationPx(0.0) {}

Diagnostics::Diagnostics()
    : requestedOrientation(Orientation::Auto),
      selectedOrientation(Orientation::Auto),
      scanlineCount(0U), scanlinesWithCandidates(0U),
      totalCandidateCount(0U), acceptedCandidateCount(0U),
      pathUsableCandidateCount(0U),
      provisionalSelectedPointCount(0U),
      publishableSelectedPointCount(0U),
      selectedPointCount(0U), selectedGapCount(0U),
      saturatedCandidateCount(0U), multiPeakScanlineCount(0U),
      ambiguousPathPointCount(0U),
      multipathAmbiguousScanlineCount(0U),
      multipathIntervalCount(0U),
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
    return "quality-v3-edge-dp-local-multipath";
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
        {REJECT_PATH_AMBIGUOUS, "PATH_AMBIGUOUS"},
        {REJECT_AMBIGUOUS_MULTIPATH,
         "AMBIGUOUS_MULTIPATH"}
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
