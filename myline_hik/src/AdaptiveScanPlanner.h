#ifndef MYLINE_HIK_ADAPTIVE_SCAN_PLANNER_H
#define MYLINE_HIK_ADAPTIVE_SCAN_PLANNER_H

#include "HikScanCore.h"

#include <opencv2/core.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace hik_adaptive {

enum class MotionPrimitive {
    Line,
    Arc
};

enum class VerificationState {
    Unknown,
    Passed,
    Failed
};

struct RobotPathEvaluation {
    VerificationState ik;
    VerificationState collision;
    VerificationState singularity;
    double minimumSingularValue;
    double minimumJointLimitMarginDeg;
    double jointTravelDeg;
    double estimatedExecutionTimeS;
    std::vector<std::array<double, 6> > jointSamplesDeg;
    MotionPrimitive evaluatedPrimitive;
    bool usedLineFallback;
    std::string primaryPrimitiveFailureDetail;
    std::string detail;

    RobotPathEvaluation();
    bool fullyVerified() const;
    bool failed() const;
};

enum class SegmentKind {
    Measurement,
    Transition
};

struct ScanSegment {
    int segmentId;
    int roiId;
    SegmentKind kind;
    // start/end are the measurement interval retained for compatibility with
    // existing straight-scan code. motionStart/motionEnd add explicit
    // acceleration lead-in/out; frames outside start..end are never formal
    // measurement evidence.
    hik_scan::Pose6D start;
    hik_scan::Pose6D end;
    hik_scan::Pose6D motionStart;
    hik_scan::Pose6D motionEnd;
    MotionPrimitive primitive;
    hik_scan::Pose6D arcVia;
    double arcRadiusMm;
    double blendRadiusMm;
    bool allowLineFallback;
    bool usedLineFallback;
    std::string fallbackReason;
    double speedMmS;
    double accelerationMmS2;
    double exposureUs;
    double leadInMm;
    double leadOutMm;
    bool reverseDirection;
    RobotPathEvaluation robot;
    // Zero means the segment has not been physically timed. Planning code
    // must never substitute the estimated duration for this field.
    double actualExecutionTimeS;

    ScanSegment();
};

struct ScanPlan {
    std::string profileId;
    std::vector<ScanSegment> segments;
    double measurementLengthMm;
    double transitionLengthMm;
    double estimatedExecutionTimeS;
    double actualExecutionTimeS;
    bool safetyVerified;
    bool executable;
    std::string safetyDetail;

    ScanPlan();
};

struct SerpentineOptions {
    hik_scan::Pose6D firstLaneStart;
    hik_scan::Pose6D firstLaneEnd;
    cv::Vec3d laneOffsetMm;
    int laneCount;
    double measurementSpeedMmS;
    double transitionSpeedMmS;
    double accelerationMmS2;
    double exposureUs;
    double leadInMm;
    double leadOutMm;
    bool enableArcTransitions;
    double minimumArcRadiusMm;
    double maximumArcRadiusMm;
    double transitionBlendRadiusMm;
    double maximumArcTangentErrorDeg;
    int maximumSegmentCount;
    double maximumTotalLengthMm;

    SerpentineOptions();
};

double estimateTrapezoidalMoveTime(double lengthMm,
                                   double speedMmS,
                                   double accelerationMmS2);

bool buildSerpentinePlan(const SerpentineOptions& options,
                         ScanPlan* plan,
                         std::string* error = nullptr);

double segmentMotionLengthMm(const ScanSegment& segment);

bool sampleSegmentMotion(
    const ScanSegment& segment,
    double maximumCartesianStepMm,
    double maximumAngularStepDeg,
    std::size_t maximumSampleCount,
    std::vector<hik_scan::Pose6D>* samples,
    std::string* error = nullptr);

struct VoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator<(const VoxelKey& other) const;
    bool operator==(const VoxelKey& other) const;
};

enum QualityVoxelFlag : std::uint32_t {
    QUALITY_VOXEL_NONE = 0U,
    QUALITY_VOXEL_UNOBSERVED_EXPECTED = 1U << 0U,
    QUALITY_VOXEL_REJECTED_DOMINANT = 1U << 1U,
    QUALITY_VOXEL_LOW_CONFIDENCE = 1U << 2U,
    QUALITY_VOXEL_MULTIPATH = 1U << 3U,
    QUALITY_VOXEL_SATURATED = 1U << 4U,
    QUALITY_VOXEL_PATH_AMBIGUOUS = 1U << 5U,
    QUALITY_VOXEL_INSUFFICIENT_SUPPORT = 1U << 6U,
    QUALITY_VOXEL_INVALID_GEOMETRY = 1U << 7U,
    QUALITY_VOXEL_NEEDS_RESCAN = 1U << 8U,
    QUALITY_VOXEL_OPTICAL_UNKNOWN = 1U << 9U
};

struct QualityVoxel {
    VoxelKey key;
    cv::Point3d centerMm;
    cv::Point3d evidenceCentroidMm;
    std::uint64_t formalAcceptedObservationCount;
    std::uint64_t qualityAcceptedObservationCount;
    std::uint64_t rejectedObservationCount;
    std::uint64_t expectedObservationCount;
    std::set<int> formalAcceptedProfiles;
    std::set<int> qualityAcceptedProfiles;
    std::set<int> rejectedProfiles;
    double meanConfidence;
    double meanSnr;
    double meanSaturatedFraction;
    double meanFwhmPx;
    cv::Vec3d meanObservedViewDirectionBase;
    std::uint64_t viewObservationCount;
    std::uint32_t cloudQualityFlags;
    std::uint32_t stripeRejectFlags;
    std::array<std::uint64_t, 32> cloudFlagCounts;
    std::array<std::uint64_t, 32> stripeRejectCounts;
    std::uint32_t stateFlags;
    double severity;

    QualityVoxel();
    bool needsRescan() const;
};

struct QualityMapOptions {
    double voxelSizeMm;
    double minimumMeanConfidence;
    double rejectedRatioThreshold;
    std::uint64_t minimumAcceptedObservations;
    bool rescanAnyRejectedEvidence;
    bool rescanUnknownOpticalEvidence;
    std::uint32_t severeCloudQualityMask;
    std::uint32_t severeStripeRejectMask;

    QualityMapOptions();
};

struct QualityMap {
    double voxelSizeMm;
    std::map<VoxelKey, QualityVoxel> voxels;
    std::size_t formalAcceptedPointCount;
    std::size_t qualityAcceptedPointCount;
    std::size_t rejectedPointCount;
    std::size_t expectedPointCount;
    std::size_t rescanVoxelCount;

    QualityMap();
};

enum class ObservationRole {
    FormalAccepted,
    QualityAccepted,
    Rejected
};

struct QualityObservation {
    hik_scan::CloudPoint point;
    ObservationRole role;
    std::uint64_t frameId;
    cv::Point3d cameraOriginBaseMm;
    bool hasCameraOrigin;

    QualityObservation();
};

bool buildQualityMap(
    const std::vector<QualityObservation>& observations,
    const std::vector<cv::Point3d>& expectedSurfacePoints,
    const QualityMapOptions& options,
    QualityMap* map,
    std::string* error = nullptr);

struct RescanRoi {
    int roiId;
    std::vector<VoxelKey> voxelKeys;
    cv::Point3d minimumMm;
    cv::Point3d maximumMm;
    cv::Point3d centerMm;
    cv::Vec3d principalAxis;
    cv::Vec3d secondaryAxis;
    cv::Vec3d estimatedNormal;
    double severity;
    std::uint32_t stateFlags;
    std::uint32_t cloudQualityFlags;
    std::uint32_t stripeRejectFlags;
    std::uint64_t formalAcceptedObservationCount;
    std::uint64_t qualityAcceptedObservationCount;
    std::uint64_t rejectedObservationCount;
    std::uint64_t expectedObservationCount;

    RescanRoi();
};

struct RoiClusteringOptions {
    int connectivityRadiusVoxels;
    std::size_t minimumVoxelCount;
    double paddingMm;
    double maximumClusterExtentMm;

    RoiClusteringOptions();
};

bool clusterRescanRois(const QualityMap& map,
                       const RoiClusteringOptions& options,
                       std::vector<RescanRoi>* rois,
                       std::string* error = nullptr);

struct CandidateAction {
    int actionId;
    int roiId;
    ScanSegment measurement;
    double yawOffsetDeg;
    double pitchOffsetDeg;
    double workingDistanceMm;
    double predictedCoverageGain;
    double predictedUncertaintyGain;
    double predictedQualityGain;
    double predictedViewDiversityGain;
    double predictedInformationGain;
    double transitionTimeS;
    double riskPenalty;
    double utility;
    std::vector<VoxelKey> coveredVoxels;
    VerificationState fov;
    VerificationState laserSweep;
    VerificationState occlusion;
    std::string observabilityDetail;
    RobotPathEvaluation robot;

    CandidateAction();
};

struct CandidateLibraryOptions {
    std::vector<double> yawOffsetsDeg;
    std::vector<double> pitchOffsetsDeg;
    std::vector<double> workingDistanceScales;
    std::vector<double> speedsMmS;
    std::vector<double> exposureUs;
    bool includeReverseDirection;
    double roiPaddingMm;
    double accelerationMmS2;
    std::size_t maximumCandidateCount;

    CandidateLibraryOptions();
};

struct CandidateGenerationContext {
    cv::Matx44d baseFromReferenceFlange;
    cv::Matx44d flangeFromCamera;
    cv::Vec3d nominalScanDirectionBase;
    cv::Vec3d cameraForwardAxis;
    cv::Matx33d cameraMatrix;
    cv::Size calibratedImageSize;
    cv::Rect softwareRoi;
    cv::Vec3d laserPlaneNormalCamera;
    double laserPlaneDMm;
    double validDepthMinimumMm;
    double validDepthMaximumMm;
    double laserPlaneToleranceMm;
    bool calibratedObservabilityAvailable;
    bool occlusionModelAvailable;

    CandidateGenerationContext();
};

bool generateCandidateLibrary(
    const QualityMap& map,
    const std::vector<RescanRoi>& rois,
    const CandidateGenerationContext& context,
    const CandidateLibraryOptions& options,
    std::vector<CandidateAction>* candidates,
    std::string* error = nullptr);

struct PlanningWeights {
    double coverage;
    double uncertainty;
    double quality;
    double viewDiversity;
    double executionTime;
    double transitionTime;
    double risk;
    double unverifiedSafetyPenalty;

    PlanningWeights();
};

struct SearchOptions {
    PlanningWeights weights;
    int horizon;
    std::size_t beamWidth;
    bool allowUnverifiedForDryRun;
    bool requireFullyVerifiedForExecution;

    SearchOptions();
};

typedef std::function<RobotPathEvaluation(
    const hik_scan::Pose6D&,
    const ScanSegment&)>
    CandidateSafetyEvaluator;

bool evaluateAndRankCandidates(
    const hik_scan::Pose6D& currentPose,
    const SearchOptions& options,
    const CandidateSafetyEvaluator& evaluator,
    std::vector<CandidateAction>* candidates,
    std::string* error = nullptr);

bool selectGreedyAction(const std::vector<CandidateAction>& candidates,
                        const SearchOptions& options,
                        CandidateAction* selected,
                        std::string* error = nullptr);

struct PlannedActionSequence {
    std::vector<CandidateAction> actions;
    double totalUtility;
    double totalInformationGain;
    double totalEstimatedTimeS;
    bool fullyVerified;
    bool executable;
    std::string detail;

    PlannedActionSequence();
};

bool beamSearchActions(const hik_scan::Pose6D& currentPose,
                       const std::vector<CandidateAction>& candidates,
                       const SearchOptions& options,
                       const CandidateSafetyEvaluator& evaluator,
                       PlannedActionSequence* sequence,
                       std::string* error = nullptr);

bool loadCloudPly(const std::string& path,
                  std::vector<hik_scan::CloudPoint>* cloud,
                  std::string* error = nullptr);

bool saveAdaptivePlanJson(const std::string& path,
                          const ScanPlan& globalPlan,
                          const QualityMap& qualityMap,
                          const std::vector<RescanRoi>& rois,
                          const PlannedActionSequence& localPlan,
                          std::string* error = nullptr);

const char* verificationStateName(VerificationState state);
const char* motionPrimitiveName(MotionPrimitive primitive);

}  // namespace hik_adaptive

#endif  // MYLINE_HIK_ADAPTIVE_SCAN_PLANNER_H
