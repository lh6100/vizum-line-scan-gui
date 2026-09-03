#ifndef MYLINE_HIK_STRIPE_CENTERLINE_EXTRACTOR_H
#define MYLINE_HIK_STRIPE_CENTERLINE_EXTRACTOR_H

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hik_stripe {

enum class Orientation {
    Auto,
    Vertical,
    Horizontal
};

// The final sub-pixel estimator is recorded per candidate so saved quality
// data can distinguish an ordinary fallback from the curvature-based centre
// and from the deliberately separate saturated-ridge rule.
enum class CenterMethod {
    BackgroundSubtractedCentroid,
    GaussianDerivativeTaylor,
    SaturatedHalfHeightMidpoint
};

enum RejectReason : std::uint32_t {
    REJECT_NONE = 0U,
    REJECT_LOW_PROMINENCE = 1U << 0U,
    REJECT_WIDTH_OUT_OF_RANGE = 1U << 1U,
    REJECT_SATURATED_WIDE_PLATEAU = 1U << 2U,
    REJECT_SATURATED_ASYMMETRIC = 1U << 3U,
    REJECT_MULTI_PEAK_AMBIGUOUS = 1U << 4U,
    REJECT_PROFILE_ASYMMETRIC = 1U << 5U,
    REJECT_FIT_RESIDUAL_HIGH = 1U << 6U,
    REJECT_QUALITY_LOW = 1U << 7U,
    REJECT_OUTSIDE_ROI = 1U << 8U,
    REJECT_OUTSIDE_VALIDITY_MASK = 1U << 9U,
    REJECT_PATH_JUMP = 1U << 10U,
    REJECT_PATH_AMBIGUOUS = 1U << 11U,
    REJECT_AMBIGUOUS_MULTIPATH = 1U << 12U
};

struct Options {
    Orientation orientation;
    // Empty means the full image. Coordinates use the original calibrated
    // image; this is a software ROI and never changes the camera matrix.
    cv::Rect roi;
    int localBaselineRadius;
    int baselineExclusionRadius;
    int maximumCandidatesPerScanline;
    double minimumProminence;
    double thresholdMadScale;
    double minimumFwhmPx;
    double maximumFwhmPx;
    int rawSaturationThreshold;
    double maximumSaturatedFraction;
    int maximumSaturatedPlateauWidthPx;
    double maximumSecondPeakRatio;
    double maximumGradientAsymmetry;
    double maximumFitResidual;
    double minimumQuality;
    double pathCandidateReward;
    double pathPositionPenalty;
    double pathCurvaturePenalty;
    double pathGapOpenPenalty;
    double pathGapExtendPenalty;
    double pathMaximumStepPx;
    int pathMaximumGap;
    double pathAmbiguityMarginPerPoint;
    double peakMergeMinimumDistancePx;
    double peakMergeFwhmScale;
    double pathAmbiguityMinimumSeparationPx;
    int pathAmbiguityPaddingScanlines;
    double pathMaximumPredictionResidualPx;

    Options();
    bool validate(const cv::Size& imageSize, std::string* error = nullptr) const;
};

struct Candidate {
    cv::Point2d pixel;
    int scanIndex;
    int peakIndex;
    double rawPeak;
    double responsePeak;
    double localBaseline;
    double localNoiseMad;
    double prominence;
    double snr;
    double fwhmPx;
    double saturatedFraction;
    int saturatedPlateauWidthPx;
    double secondPeakRatio;
    double gradientAsymmetry;
    double fitResidual;
    double quality;
    double centerSigmaPx;
    CenterMethod centerMethod;
    double taylorOffsetPx;
    double smoothedFirstDerivative;
    double smoothedSecondDerivative;
    std::uint32_t rejectFlags;
    int ambiguityIntervalId;
    int ambiguityBranchId;

    Candidate();
    bool usableForPath() const;
    bool accepted() const;
};

struct PathScanlineDiagnostic {
    int scanIndex;
    bool hasSelected;
    cv::Point2d selectedPixel;
    bool hasAlternate;
    cv::Point2d alternatePixel;
    double separationPx;
    double localCostMargin;
    int ambiguityIntervalId;

    PathScanlineDiagnostic();
};

struct MultipathBranch {
    int branchId;
    double pathCost;
    std::vector<Candidate> candidates;

    MultipathBranch();
};

struct MultipathInterval {
    int intervalId;
    // Inclusive hard-rejection/protection range.
    int firstScanIndex;
    int lastScanIndex;
    // Inclusive range over which the representative hypotheses differ.
    int coreFirstScanIndex;
    int coreLastScanIndex;
    // Common best/alternate candidates immediately outside the core. -1 plus
    // an open flag means no common anchor exists on that side and the
    // protection range is extended to the path-segment boundary.
    int leftAnchorScanIndex;
    int rightAnchorScanIndex;
    bool leftBoundaryOpen;
    bool rightBoundaryOpen;
    double minimumLocalCostMargin;
    double maximumSeparationPx;
    // Ordered, mutually exclusive path hypotheses over first..last. Candidate
    // copies carry this intervalId and their branchId for direct 3D rebuild.
    std::vector<MultipathBranch> branches;

    MultipathInterval();
};

struct Diagnostics {
    Orientation requestedOrientation;
    Orientation selectedOrientation;
    cv::Rect appliedRoi;
    std::size_t scanlineCount;
    std::size_t scanlinesWithCandidates;
    std::size_t totalCandidateCount;
    std::size_t acceptedCandidateCount;
    std::size_t pathUsableCandidateCount;
    std::size_t provisionalSelectedPointCount;
    std::size_t publishableSelectedPointCount;
    std::size_t selectedPointCount;
    std::size_t selectedGapCount;
    std::size_t saturatedCandidateCount;
    std::size_t multiPeakScanlineCount;
    std::size_t ambiguousPathPointCount;
    std::size_t multipathAmbiguousScanlineCount;
    std::size_t multipathIntervalCount;
    std::size_t rejectedLowProminenceCount;
    std::size_t rejectedWidthCount;
    std::size_t rejectedSaturationCount;
    std::size_t rejectedMultiPeakCount;
    std::size_t rejectedAsymmetryCount;
    std::size_t rejectedFitCount;
    std::size_t rejectedQualityCount;
    std::size_t rejectedMaskCount;
    double meanSelectedQuality;
    double meanSelectedFwhmPx;
    double meanSelectedSnr;
    double meanSelectedGradientAsymmetry;
    double meanSelectedFitResidual;
    double meanSelectedSecondPeakRatio;
    double selectedSaturatedRatio;
    double bestPathCost;
    double secondPathCost;
    double pathCostMargin;
    double pathCostMarginPerPoint;

    Diagnostics();
};

struct Result {
    bool ok;
    std::string error;
    Orientation orientation;
    std::vector<Candidate> candidates;
    // The globally optimal path before ambiguous multipath intervals are
    // removed. It is diagnostic-only and must never be reconstructed as one
    // physical stripe without inspecting multipathIntervals.
    std::vector<Candidate> provisionalSelected;
    // Backward-compatible name for the publishable path. Ambiguous intervals
    // are removed and never interpolated.
    std::vector<Candidate> selected;
    std::vector<PathScanlineDiagnostic> pathDiagnostics;
    std::vector<MultipathInterval> multipathIntervals;
    Diagnostics diagnostics;

    Result();
};

// Extracts a single quality-gated centerline. response8 and raw8 must be
// CV_8UC1 with the same size. validityMask may be empty; otherwise non-zero
// pixels are eligible candidates. The extractor deliberately permits gaps.
bool extractCenterline(const cv::Mat& response8,
                       const cv::Mat& raw8,
                       const Options& options,
                       Result* result,
                       const cv::Mat& validityMask = cv::Mat());

const char* orientationName(Orientation orientation);
const char* centerMethodName(CenterMethod method);
const char* algorithmVersion();
std::string rejectReasonNames(std::uint32_t flags);
bool hasRejectReason(const Candidate& candidate, RejectReason reason);

}  // namespace hik_stripe

#endif  // MYLINE_HIK_STRIPE_CENTERLINE_EXTRACTOR_H
