#ifndef MYLINE_HIK_HIK_SCAN_CORE_H
#define MYLINE_HIK_HIK_SCAN_CORE_H

#include "HikCalibrationCore.h"

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hik_scan {

struct Pose6D {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;

    Pose6D();
};

// Reverse a taught linear scan without changing its active tool orientation.
// The two XYZ endpoints are exchanged, while both resulting poses use the
// previous start RPY because buildLinearFlangePath intentionally ignores the
// taught end RPY. This makes repeated forward/reverse scans free of an
// unexpected wrist rotation at the turnaround point.
bool reverseLinearFlangePath(Pose6D* start,
                             Pose6D* end,
                             std::string* error = 0);

bool buildLinearFlangePath(const Pose6D& start,
                           const Pose6D& end,
                           double stepMm,
                           int maximumPointCount,
                           std::vector<Pose6D>* targets,
                           std::string* error = 0);

struct HandEyeFile {
    bool ok;
    std::string error;
    std::string mode;
    std::string parentFrame;
    std::string childFrame;
    std::string cameraSerial;
    std::string intrinsicsSha256;
    cv::Matx44d flangeFromCamera;

    HandEyeFile();
};

bool loadHandEyeYaml(const std::string& path,
                     HandEyeFile* handEye,
                     std::string* error = 0);

struct CloudPoint {
    cv::Point3d basePointMm;
    cv::Point3d cameraPointMm;
    double confidence;
    double response;
    int profileIndex;
    double pixelU;
    double pixelV;
    // Point-local optical evidence is preserved for the adaptive quality map.
    // Legacy scanner points leave opticalMetricsValid=false; absence of these
    // values must be treated as UNKNOWN rather than as a good observation.
    bool opticalMetricsValid;
    double snr;
    double fwhmPx;
    double saturatedFraction;
    double secondPeakRatio;
    double gradientAsymmetry;
    double fitResidual;
    double centerSigmaPx;
    std::uint32_t stripeRejectFlags;
    std::uint32_t qualityFlags;
    std::uint32_t observationCount;

    CloudPoint();
};

enum CloudPointQualityFlag : std::uint32_t {
    CLOUD_QUALITY_NONE = 0U,
    CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED = 1U << 0U,
    CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT = 1U << 1U,
    CLOUD_QUALITY_REJECTED_INVALID_BASE_POINT = 1U << 2U,
    CLOUD_QUALITY_VOXEL_AGGREGATED = 1U << 3U,
    // The 2-D observation passed the local optical hard gates and the
    // dynamic-programming publication gate. Multi-peak is soft lattice
    // evidence; unresolved multipath observations use the dedicated
    // CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE provenance instead.
    CLOUD_QUALITY_OPTICAL_ACCEPTED = 1U << 4U,
    CLOUD_QUALITY_V_GROOVE_GEOMETRY_VALIDATED = 1U << 5U,
    CLOUD_QUALITY_V_GROOVE_CANDIDATE_PROMOTED = 1U << 6U,
    CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS = 1U << 7U,
    CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT = 1U << 8U,
    CLOUD_QUALITY_REJECTED_V_GROOVE_GEOMETRY = 1U << 9U,
    CLOUD_QUALITY_REJECTED_V_GROOVE_ALTERNATE_BRANCH = 1U << 10U,
    CLOUD_QUALITY_REJECTED_V_GROOVE_OUTLIER = 1U << 11U,
    CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE = 1U << 12U,
    // Shadow preserves the calibrated Legacy pixel definition, but any
    // Legacy observation inside a 2-D multipath protection interval is
    // withheld from the formal cloud and retained only for rejected audit.
    CLOUD_QUALITY_REJECTED_SHADOW_LEGACY_MULTIPATH = 1U << 13U,
    // The point itself was reconstructable, but its complete profile failed
    // the configured formal publication count gate. No sparse fragment from
    // that profile may be reintroduced by a later stage.
    CLOUD_QUALITY_REJECTED_PROFILE_PUBLICATION_GATE = 1U << 14U,
    // Set by tolerant offline readers when an older PLY has no quality
    // provenance columns. UNKNOWN must never be silently interpreted as GOOD.
    CLOUD_QUALITY_UNKNOWN_SCHEMA = 1U << 15U
};

bool cloudPointHasQualityFlag(const CloudPoint& point,
                              CloudPointQualityFlag flag);

bool appendProfileInBase(const hik_calibration::StaticProfileResult& profile,
                         const cv::Matx44d& baseFromFlange,
                         const cv::Matx44d& flangeFromCamera,
                         int profileIndex,
                         std::vector<CloudPoint>* cloud,
                         std::string* error = 0);

// Continuous acquisition already computes T_base_camera at the exposure
// midpoint. Use it directly so reconstruction cannot accidentally fall back to
// a before/after or callback-time robot pose.
bool appendProfileUsingBaseFromCamera(
    const hik_calibration::StaticProfileResult& profile,
    const cv::Matx44d& baseFromCamera,
    int profileIndex,
    std::vector<CloudPoint>* cloud,
    std::string* error = 0);

// Appends an explicitly selected profile-point set. This keeps cloud assembly
// independent from whether the caller selected legacy, shadow-quality or
// quality-gated points. The append is transactional: either every source point
// is transformed and appended, or the output cloud is left unchanged.
bool appendProfilePointsUsingBaseFromCamera(
    const std::vector<hik_calibration::StaticProfilePoint>& points,
    const cv::Matx44d& baseFromCamera,
    int profileIndex,
    std::vector<CloudPoint>* cloud,
    std::string* error = 0);

struct VoxelDownsampleOptions {
    double voxelSizeMm;
    bool confidenceWeighted;
    // A zero-confidence observation still receives this small positive
    // geometry weight. This prevents a voxel containing only zero-confidence
    // observations from becoming undefined.
    double minimumConfidenceWeight;

    VoxelDownsampleOptions();
};

struct VoxelDownsampleStatistics {
    std::size_t inputPointCount;
    std::size_t finitePointCount;
    std::size_t rejectedNonFinitePointCount;
    std::size_t outputPointCount;
    bool confidenceWeighted;

    VoxelDownsampleStatistics();
};

// Extended voxel reducer. When confidenceWeighted is true, base/camera
// coordinates, response and source pixels use confidence as their weight.
// Confidence itself remains an arithmetic mean, observationCount is summed
// (with uint32 saturation), and qualityFlags are OR-combined.
std::vector<CloudPoint> voxelDownsample(
    const std::vector<CloudPoint>& cloud,
    const VoxelDownsampleOptions& options,
    VoxelDownsampleStatistics* statistics = 0);

// Compatibility wrapper: preserves the original equal-per-input-point voxel
// averaging semantics.
std::vector<CloudPoint> voxelDownsample(const std::vector<CloudPoint>& cloud,
                                        double voxelSizeMm);

struct AdjacentProfileSupportOptions {
    // Disabled by default so adding this quality stage cannot silently change
    // an existing production cloud.
    bool enabled;
    double radiusMm;
    int minimumSupportingProfiles;
    int maximumProfileGap;

    AdjacentProfileSupportOptions();
};

struct AdjacentProfileSupportStatistics {
    std::size_t inputPointCount;
    std::size_t validPointCount;
    std::size_t keptPointCount;
    std::size_t rejectedPointCount;
    std::size_t invalidPointCount;
    std::size_t insufficientSupportPointCount;

    AdjacentProfileSupportStatistics();
};

struct AdjacentProfileSupportResult {
    bool applied;
    std::vector<CloudPoint> kept;
    std::vector<CloudPoint> rejected;
    AdjacentProfileSupportStatistics statistics;

    AdjacentProfileSupportResult();
};

// Keeps a point only when at least minimumSupportingProfiles distinct, other
// profile indices contain a base-frame point within radiusMm and within
// maximumProfileGap. Run this on raw per-profile points before voxelization;
// a voxel stores only one representative profile index. This filter removes
// sparse/isolated observations, but a coherent false surface repeated across
// adjacent profiles will intentionally pass and needs optical/temporal quality
// evidence at a different layer.
bool filterByAdjacentProfileSupport(
    const std::vector<CloudPoint>& cloud,
    const AdjacentProfileSupportOptions& options,
    AdjacentProfileSupportResult* result,
    std::string* error = 0);

// One complete, caller-assembled interpretation of all ambiguous observations
// that belong together. Points from different branches are mutually exclusive.
// The validator deliberately does not cluster loose candidates into branches:
// the 2-D extractor/integration layer must preserve that evidence and assign a
// stable branchId before calling this API.
struct VGrooveCandidateBranch {
    std::uint64_t ambiguityGroupId;
    int branchId;
    // False means that an earlier per-profile hard gate retained this branch
    // for rejected/audit output only. Temporal geometry may still inspect it,
    // but must never promote any of its points into a formal cloud.
    bool formalPublicationEligible;
    std::vector<CloudPoint> points;

    VGrooveCandidateBranch();
};

struct VGrooveTemporalValidationOptions {
    int halfWindowProfiles;
    std::size_t minimumPointsPerPlane;
    int minimumProfilesPerPlane;
    int minimumRootSupportingProfiles;
    std::size_t maximumPlaneSamplePoints;
    std::size_t maximumPlaneCandidates;
    double pointToPlaneInlierMm;
    double maximumPlaneRmsMm;
    double minimumInlierFraction;
    double minimumPlaneSpreadMm;
    double minimumPlaneAngleDeg;
    double maximumPlaneAngleDeg;
    double maximumRootGapMm;
    std::size_t minimumCandidatePointsPerPlane;
    std::size_t minimumCandidateRootPointCount;
    // Only publishable points in this base_link radius of any candidate in
    // the current ambiguity group can support that group's local V model.
    double maximumSupportDistanceMm;
    double minimumOneSidedFraction;
    double rootSideToleranceMm;
    double equivalentPlaneAngleDeg;
    double equivalentPlaneOffsetMm;

    VGrooveTemporalValidationOptions();
};

enum class VGrooveProfileStatus {
    NotEvaluated,
    PromotedUnique,
    RejectedAmbiguous,
    RejectedInsufficientEvidence,
    RejectedInvalidGeometry
};

struct VGrooveProfileValidation {
    int profileIndex;
    VGrooveProfileStatus status;
    std::string reason;
    int selectedBranchId;
    std::size_t evaluatedHypothesisCount;
    std::size_t validHypothesisCount;
    std::size_t validModelCount;
    std::size_t windowPointCount;
    std::size_t planeOnePointCount;
    std::size_t planeTwoPointCount;
    int planeOneProfileCount;
    int planeTwoProfileCount;
    int rootSupportingProfileCount;
    double planeOneRmsMm;
    double planeTwoRmsMm;
    double planeAngleDeg;

    VGrooveProfileValidation();
};

struct VGrooveAmbiguityGroupValidation {
    int profileIndex;
    std::uint64_t ambiguityGroupId;
    VGrooveProfileStatus status;
    std::string reason;
    int selectedBranchId;
    std::size_t evaluatedBranchCount;
    std::size_t validBranchCount;
    std::size_t validModelCount;
    std::size_t localSupportPointCount;
    std::size_t promotedCandidatePointCount;
    std::size_t rejectedCandidatePointCount;
    std::size_t planeOnePointCount;
    std::size_t planeTwoPointCount;
    int planeOneProfileCount;
    int planeTwoProfileCount;
    int rootSupportingProfileCount;
    double planeOneRmsMm;
    double planeTwoRmsMm;
    double planeAngleDeg;

    VGrooveAmbiguityGroupValidation();
};

struct VGrooveTemporalValidationStatistics {
    std::size_t publishableInputPointCount;
    std::size_t candidateInputPointCount;
    std::size_t profileCount;
    std::size_t promotedProfileCount;
    std::size_t ambiguousProfileCount;
    std::size_t insufficientProfileCount;
    std::size_t invalidGeometryProfileCount;
    std::size_t passThroughPublishablePointCount;
    std::size_t promotedCandidatePointCount;
    std::size_t rejectedCandidatePointCount;
    std::size_t rejectedAmbiguousCandidatePointCount;
    std::size_t rejectedInsufficientCandidatePointCount;
    std::size_t rejectedGeometryCandidatePointCount;
    std::size_t rejectedAlternateBranchPointCount;
    std::size_t rejectedOutlierPointCount;

    VGrooveTemporalValidationStatistics();
};

struct VGrooveTemporalValidationResult {
    // Existing formal points are never removed or relabeled by this candidate
    // selector. They remain formal input but are NOT thereby V-validated.
    std::vector<CloudPoint> passThroughPublishable;
    std::vector<CloudPoint> promotedCandidates;
    std::vector<CloudPoint> rejectedCandidates;
    std::vector<VGrooveProfileValidation> profiles;
    std::vector<VGrooveAmbiguityGroupValidation> ambiguityGroups;
    VGrooveTemporalValidationStatistics statistics;
};

// Conservative base_link-only validation. Each center profile is evaluated
// with a sliding profile-index window. Common publishable points are combined
// with exactly one explicit candidate branch from one ambiguityGroupId at a
// time. Groups are independent, and branchId only needs to be unique within a
// group. Its local support is limited in base_link to publishable points near
// that group's candidate envelope. Candidate promotion requires one and only
// one branch hypothesis and one and only one robust two-plane model satisfying
// all support, residual, angle, intersection and root checks. A failed group
// never discards or relabels existing publishable points. Ambiguous, incomplete,
// ungrouped or invalid candidate evidence is rejected. This API never snaps to
// CAD, fills holes, synthesizes points, clusters loose candidates into branches,
// or guesses branches.
bool validateVGrooveTemporalGeometry(
    const std::vector<CloudPoint>& publishablePoints,
    const std::vector<VGrooveCandidateBranch>& ambiguousCandidateBranches,
    const VGrooveTemporalValidationOptions& options,
    VGrooveTemporalValidationResult* result,
    std::string* error = 0);

enum class PlyColorScalar {
    Uniform,
    CameraDepth,
    BaseZ
};

// Presentation-only PLY coloring. It never changes point coordinates or any
// reconstruction/quality field. Turbo limits use robust percentiles so a few
// isolated depth outliers do not collapse the useful color range.
struct PlyColorOptions {
    PlyColorScalar scalar;
    double lowerPercentile;
    double upperPercentile;
    bool invert;

    PlyColorOptions();
};

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 std::string* error = 0);

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 const PlyColorOptions& colorOptions,
                 std::string* error = 0);

// Same schema as saveScanPly(), encoded as binary_little_endian. CloudCompare
// and other PLY readers see identical properties while large continuous scans
// require less formatting, disk space and write time.
bool saveScanPlyBinary(const std::string& path,
                       const std::vector<CloudPoint>& cloud,
                       const std::string& frameId,
                       std::string* error = 0);

bool saveScanPlyBinary(const std::string& path,
                       const std::vector<CloudPoint>& cloud,
                       const std::string& frameId,
                       const PlyColorOptions& colorOptions,
                       std::string* error = 0);

}  // namespace hik_scan

#endif  // MYLINE_HIK_HIK_SCAN_CORE_H
