#include "HikScanCore.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace {

int gFailures = 0;

void fail(const std::string& message, int line) {
    ++gFailures;
    std::cerr << "FAIL line " << line << ": " << message << std::endl;
}

#define CHECK_TRUE(condition, message) \
    do {                                \
        if (!(condition)) {             \
            fail((message), __LINE__);  \
        }                               \
    } while (false)

bool nearlyEqual(double first, double second, double tolerance = 1.0e-12) {
    return std::isfinite(first) && std::isfinite(second) &&
           std::fabs(first - second) <= tolerance;
}

hik_scan::CloudPoint pointAt(double x,
                             double y,
                             double z,
                             int profile,
                             double confidence = 0.8) {
    hik_scan::CloudPoint point;
    point.basePointMm = cv::Point3d(x, y, z);
    point.cameraPointMm = point.basePointMm;
    point.profileIndex = profile;
    point.confidence = confidence;
    point.response = 100.0;
    point.pixelU = x;
    point.pixelV = y;
    return point;
}

std::vector<hik_scan::CloudPoint> makeVGrooveCloud(
        int firstProfile,
        int profileCount,
        bool includeOutliers = false) {
    std::vector<hik_scan::CloudPoint> cloud;
    for (int profileOffset = 0;
         profileOffset < profileCount; ++profileOffset) {
        const int profile = firstProfile + profileOffset;
        const double y = 2.0 * static_cast<double>(profileOffset);
        for (int sample = 1; sample <= 9; ++sample) {
            const double distance = 0.4 * static_cast<double>(sample);
            cloud.push_back(pointAt(
                -distance, y, distance, profile, 0.9));
            cloud.push_back(pointAt(
                distance, y, distance, profile, 0.9));
        }
        if (includeOutliers) {
            cloud.push_back(pointAt(
                8.0 + profileOffset, y, -6.0, profile, 0.2));
            cloud.push_back(pointAt(
                -7.0, y, 9.0 + profileOffset, profile, 0.2));
        }
    }
    return cloud;
}

void testReverseLinearFlangePathPreservesOrientation() {
    hik_scan::Pose6D start;
    start.x = 1.0;
    start.y = 2.0;
    start.z = 3.0;
    start.rx = 171.0;
    start.ry = -2.0;
    start.rz = 43.0;
    hik_scan::Pose6D end;
    end.x = 101.0;
    end.y = 202.0;
    end.z = 303.0;
    end.rx = -10.0;
    end.ry = 20.0;
    end.rz = -30.0;

    std::string error;
    CHECK_TRUE(hik_scan::reverseLinearFlangePath(
                   &start, &end, &error),
               std::string("path reversal failed: ") + error);
    CHECK_TRUE(nearlyEqual(start.x, 101.0) &&
                   nearlyEqual(start.y, 202.0) &&
                   nearlyEqual(start.z, 303.0) &&
                   nearlyEqual(end.x, 1.0) &&
                   nearlyEqual(end.y, 2.0) &&
                   nearlyEqual(end.z, 3.0),
               "path reversal must exchange the two XYZ endpoints");
    CHECK_TRUE(nearlyEqual(start.rx, 171.0) &&
                   nearlyEqual(start.ry, -2.0) &&
                   nearlyEqual(start.rz, 43.0) &&
                   nearlyEqual(end.rx, 171.0) &&
                   nearlyEqual(end.ry, -2.0) &&
                   nearlyEqual(end.rz, 43.0),
               "path reversal must preserve the active start orientation "
               "instead of adopting the ignored end RPY");

    std::vector<hik_scan::Pose6D> targets;
    CHECK_TRUE(hik_scan::buildLinearFlangePath(
                   start, end, 50.0, 100, &targets, &error),
               std::string("reversed path generation failed: ") + error);
    bool allOrientationsPreserved = !targets.empty();
    for (const hik_scan::Pose6D& target : targets) {
        allOrientationsPreserved =
            allOrientationsPreserved &&
            nearlyEqual(target.rx, 171.0) &&
            nearlyEqual(target.ry, -2.0) &&
            nearlyEqual(target.rz, 43.0);
    }
    CHECK_TRUE(allOrientationsPreserved,
               "every reversed path target must retain the original scan RPY");

    CHECK_TRUE(hik_scan::reverseLinearFlangePath(
                   &start, &end, &error),
               std::string("second path reversal failed: ") + error);
    CHECK_TRUE(nearlyEqual(start.x, 1.0) &&
                   nearlyEqual(start.y, 2.0) &&
                   nearlyEqual(start.z, 3.0) &&
                   nearlyEqual(end.x, 101.0) &&
                   nearlyEqual(end.y, 202.0) &&
                   nearlyEqual(end.z, 303.0),
               "two reversals must restore the original scan direction");
}

hik_scan::VGrooveCandidateBranch makeVGrooveBranch(
        std::uint64_t groupId,
        int branchId,
        int firstProfile,
        int profileCount,
        bool followsGroove) {
    hik_scan::VGrooveCandidateBranch branch;
    branch.ambiguityGroupId = groupId;
    branch.branchId = branchId;
    for (int offset = 0; offset < profileCount; ++offset) {
        const int profile = firstProfile + offset;
        const double y = 2.0 * static_cast<double>(offset);
        const cv::Point3d samples[] = {
            followsGroove
                ? cv::Point3d(-0.8, y, 0.8)
                : cv::Point3d(-0.8, y, 7.0),
            followsGroove
                ? cv::Point3d(-0.4, y, 0.4)
                : cv::Point3d(-0.4, y, 7.0),
            followsGroove
                ? cv::Point3d(0.0, y, 0.0)
                : cv::Point3d(0.0, y, 7.0),
            followsGroove
                ? cv::Point3d(0.4, y, 0.4)
                : cv::Point3d(0.4, y, 7.0),
            followsGroove
                ? cv::Point3d(0.8, y, 0.8)
                : cv::Point3d(0.8, y, 7.0)
        };
        for (const cv::Point3d& sample : samples) {
            hik_scan::CloudPoint point = pointAt(
                sample.x, sample.y, sample.z, profile,
                followsGroove ? 0.75 : 0.25);
            point.qualityFlags |=
                hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE;
            branch.points.push_back(point);
        }
    }
    return branch;
}

hik_scan::VGrooveCandidateBranch makeOneWingBranch(
        std::uint64_t groupId,
        int branchId,
        int firstProfile,
        int profileCount) {
    hik_scan::VGrooveCandidateBranch branch;
    branch.ambiguityGroupId = groupId;
    branch.branchId = branchId;
    for (int offset = 0; offset < profileCount; ++offset) {
        const int profile = firstProfile + offset;
        const double y = 2.0 * static_cast<double>(offset);
        const cv::Point3d samples[] = {
            cv::Point3d(0.0, y, 0.0),
            cv::Point3d(0.4, y, 0.4),
            cv::Point3d(0.8, y, 0.8),
            cv::Point3d(1.2, y, 1.2),
            cv::Point3d(1.6, y, 1.6)
        };
        for (const cv::Point3d& sample : samples) {
            hik_scan::CloudPoint point = pointAt(
                sample.x, sample.y, sample.z, profile, 0.75);
            point.qualityFlags |=
                hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE;
            branch.points.push_back(point);
        }
    }
    return branch;
}

hik_scan::VGrooveCandidateBranch makeBackwardExtensionBranch(
        std::uint64_t groupId,
        int branchId,
        int firstProfile,
        int profile) {
    hik_scan::VGrooveCandidateBranch branch;
    branch.ambiguityGroupId = groupId;
    branch.branchId = branchId;
    const double y =
        2.0 * static_cast<double>(profile - firstProfile);
    const cv::Point3d samples[] = {
        cv::Point3d(0.8, y, -0.8),
        cv::Point3d(0.4, y, -0.4),
        cv::Point3d(0.0, y, 0.0),
        cv::Point3d(-0.4, y, -0.4),
        cv::Point3d(-0.8, y, -0.8)
    };
    for (const cv::Point3d& sample : samples) {
        hik_scan::CloudPoint point = pointAt(
            sample.x, sample.y, sample.z, profile, 0.75);
        point.qualityFlags |=
            hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE;
        branch.points.push_back(point);
    }
    return branch;
}

hik_scan::VGrooveTemporalValidationOptions vGrooveTestOptions() {
    hik_scan::VGrooveTemporalValidationOptions options;
    options.halfWindowProfiles = 2;
    options.minimumPointsPerPlane = 8U;
    options.minimumProfilesPerPlane = 3;
    options.minimumRootSupportingProfiles = 2;
    options.maximumPlaneSamplePoints = 14U;
    options.maximumPlaneCandidates = 12U;
    options.pointToPlaneInlierMm = 0.12;
    options.maximumPlaneRmsMm = 0.06;
    options.minimumInlierFraction = 0.75;
    options.minimumPlaneSpreadMm = 0.20;
    options.minimumPlaneAngleDeg = 30.0;
    options.maximumPlaneAngleDeg = 90.0;
    options.maximumRootGapMm = 0.75;
    options.minimumOneSidedFraction = 0.95;
    options.rootSideToleranceMm = 0.05;
    options.equivalentPlaneAngleDeg = 1.0;
    options.equivalentPlaneOffsetMm = 0.10;
    return options;
}

void testFilterDisabledByDefault() {
    std::vector<hik_scan::CloudPoint> cloud;
    cloud.push_back(pointAt(100.0, 100.0, 100.0, 10));

    hik_scan::AdjacentProfileSupportOptions options;
    CHECK_TRUE(!options.enabled,
               "adjacent-profile filtering must be disabled by default");
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   cloud, options, &result, &error),
               std::string("disabled filter failed: ") + error);
    CHECK_TRUE(!result.applied && result.kept.size() == 1U &&
                   result.rejected.empty(),
               "disabled filter must preserve the complete input cloud");
    CHECK_TRUE(result.kept.front().qualityFlags ==
                   cloud.front().qualityFlags,
               "disabled filter must not mutate quality flags");
}

void testSparseIsolatedPointRejected() {
    std::vector<hik_scan::CloudPoint> cloud;
    cloud.push_back(pointAt(0.0, 0.0, 0.0, 9));
    cloud.push_back(pointAt(0.1, 0.0, 0.0, 10));
    cloud.push_back(pointAt(0.2, 0.0, 0.0, 11));
    cloud.push_back(pointAt(20.0, 20.0, 20.0, 10));

    hik_scan::AdjacentProfileSupportOptions options;
    options.enabled = true;
    options.radiusMm = 0.25;
    options.minimumSupportingProfiles = 1;
    options.maximumProfileGap = 1;
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   cloud, options, &result, &error),
               std::string("support filter failed: ") + error);
    CHECK_TRUE(result.kept.size() == 3U && result.rejected.size() == 1U,
               "three adjacent-profile points must remain and one isolated "
               "point must be rejected");
    CHECK_TRUE(result.statistics.insufficientSupportPointCount == 1U &&
                   result.statistics.invalidPointCount == 0U,
               "support rejection counters must distinguish isolation");
    CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                   result.rejected.front(),
                   hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT),
               "isolated point must retain an explicit rejection flag");
    for (std::size_t index = 0U; index < result.kept.size(); ++index) {
        CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                       result.kept[index],
                       hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED),
                   "kept points must record adjacent-profile support");
    }
}

void testDistinctProfilesAndMaximumGap() {
    std::vector<hik_scan::CloudPoint> oneNeighborProfile;
    oneNeighborProfile.push_back(pointAt(0.0, 0.0, 0.0, 10));
    oneNeighborProfile.push_back(pointAt(0.05, 0.0, 0.0, 9));
    oneNeighborProfile.push_back(pointAt(0.10, 0.0, 0.0, 9));

    hik_scan::AdjacentProfileSupportOptions options;
    options.enabled = true;
    options.radiusMm = 0.5;
    options.minimumSupportingProfiles = 2;
    options.maximumProfileGap = 1;
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   oneNeighborProfile, options, &result, &error),
               std::string("distinct-profile filter failed: ") + error);
    bool targetRejected = false;
    for (std::size_t index = 0U; index < result.rejected.size(); ++index) {
        if (result.rejected[index].profileIndex == 10) {
            targetRejected = true;
        }
    }
    CHECK_TRUE(targetRejected,
               "multiple points from one profile must count as one supporting "
               "profile, not multiple observations");

    std::vector<hik_scan::CloudPoint> enoughProfiles = oneNeighborProfile;
    enoughProfiles.push_back(pointAt(0.15, 0.0, 0.0, 11));
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   enoughProfiles, options, &result, &error),
               std::string("two-profile support filter failed: ") + error);
    bool targetKept = false;
    for (std::size_t index = 0U; index < result.kept.size(); ++index) {
        if (result.kept[index].profileIndex == 10) {
            targetKept = true;
        }
    }
    CHECK_TRUE(targetKept,
               "two distinct neighboring profiles must satisfy a two-profile "
               "support requirement");

    std::vector<hik_scan::CloudPoint> excessiveGap;
    excessiveGap.push_back(pointAt(0.0, 0.0, 0.0, 1));
    excessiveGap.push_back(pointAt(0.1, 0.0, 0.0, 4));
    options.minimumSupportingProfiles = 1;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   excessiveGap, options, &result, &error),
               std::string("maximum-gap filter failed: ") + error);
    CHECK_TRUE(result.kept.empty() && result.rejected.size() == 2U,
               "spatial neighbors outside maximumProfileGap must not provide "
               "support");
}

void testCoherentFalseSurfaceLimitation() {
    std::vector<hik_scan::CloudPoint> coherentFalseSurface;
    for (int profile = 20; profile <= 22; ++profile) {
        for (int sample = 0; sample < 5; ++sample) {
            coherentFalseSurface.push_back(pointAt(
                0.1 * static_cast<double>(profile - 20),
                0.1 * static_cast<double>(sample),
                500.0,
                profile));
        }
    }

    hik_scan::AdjacentProfileSupportOptions options;
    options.enabled = true;
    options.radiusMm = 0.16;
    options.minimumSupportingProfiles = 1;
    options.maximumProfileGap = 1;
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   coherentFalseSurface, options, &result, &error),
               std::string("coherent-surface filter failed: ") + error);
    CHECK_TRUE(result.kept.size() == coherentFalseSurface.size() &&
                   result.rejected.empty(),
               "a coherent false surface repeated across profiles must pass "
               "this isolation-only filter; optical/temporal gates are still "
               "required");
}

void testConfidenceWeightedVoxelDownsample() {
    hik_scan::CloudPoint strong = pointAt(0.0, 0.0, 0.0, 3, 0.9);
    strong.response = 100.0;
    strong.pixelU = 10.0;
    strong.observationCount = 2U;
    strong.qualityFlags = hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED;

    hik_scan::CloudPoint weak = pointAt(0.8, 0.0, 0.0, 4, 0.1);
    weak.response = 0.0;
    weak.pixelU = 20.0;
    weak.observationCount = 3U;
    weak.qualityFlags =
        hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT;
    const std::vector<hik_scan::CloudPoint> cloud{strong, weak};

    const std::vector<hik_scan::CloudPoint> legacy =
        hik_scan::voxelDownsample(cloud, 1.0);
    CHECK_TRUE(legacy.size() == 1U &&
                   nearlyEqual(legacy.front().basePointMm.x, 0.4),
               "legacy voxel wrapper must retain equal-input averaging");

    hik_scan::VoxelDownsampleOptions options;
    options.voxelSizeMm = 1.0;
    options.confidenceWeighted = true;
    hik_scan::VoxelDownsampleStatistics statistics;
    const std::vector<hik_scan::CloudPoint> weighted =
        hik_scan::voxelDownsample(cloud, options, &statistics);
    CHECK_TRUE(weighted.size() == 1U &&
                   nearlyEqual(weighted.front().basePointMm.x, 0.08),
               "confidence weighting must move voxel geometry toward the "
               "high-confidence observation");
    CHECK_TRUE(nearlyEqual(weighted.front().response, 90.0) &&
                   nearlyEqual(weighted.front().pixelU, 11.0),
               "response and source pixel must use the same confidence weights");
    CHECK_TRUE(nearlyEqual(weighted.front().confidence, 0.5),
               "voxel confidence must remain an arithmetic source mean");
    CHECK_TRUE(weighted.front().profileIndex == 3,
               "weighted voxel must retain the strongest source profile");
    CHECK_TRUE(weighted.front().observationCount == 5U,
               "voxel must sum represented observation counts");
    CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                   weighted.front(),
                   hik_scan::CLOUD_QUALITY_VOXEL_AGGREGATED) &&
                   hik_scan::cloudPointHasQualityFlag(
                       weighted.front(),
                       hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED) &&
                   hik_scan::cloudPointHasQualityFlag(
                       weighted.front(),
                       hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT),
               "voxel must OR source quality flags and mark aggregation");
    CHECK_TRUE(statistics.inputPointCount == 2U &&
                   statistics.finitePointCount == 2U &&
                   statistics.outputPointCount == 1U &&
                   statistics.confidenceWeighted,
               "voxel statistics must report weighted reduction counts");
}

void testQualityProfilePointsCarryOpticalAcceptance() {
    hik_calibration::StaticProfilePoint source;
    source.cameraPointMm = cv::Point3d(1.0, 2.0, 500.0);
    source.stripe.pixel = cv::Point2d(20.25, 30.5);
    source.stripe.confidence = 0.8;
    source.stripe.peakDifference = 120.0;
    source.stripe.qualityExtractor = true;
    source.stripe.rejectFlags = hik_stripe::REJECT_NONE;

    std::vector<hik_scan::CloudPoint> cloud;
    std::string error;
    CHECK_TRUE(hik_scan::appendProfilePointsUsingBaseFromCamera(
                   std::vector<hik_calibration::StaticProfilePoint>{source},
                   cv::Matx44d::eye(), 7, &cloud, &error),
               std::string("quality profile append failed: ") + error);
    CHECK_TRUE(cloud.size() == 1U &&
                   hik_scan::cloudPointHasQualityFlag(
                       cloud.front(),
                       hik_scan::CLOUD_QUALITY_OPTICAL_ACCEPTED),
               "a hard-gated quality center must retain an explicit optical "
               "acceptance flag in the point cloud");
}

void testProfileTransformIsTransactional() {
    hik_calibration::StaticProfilePoint finite;
    finite.cameraPointMm = cv::Point3d(1.0, 2.0, 3.0);
    hik_calibration::StaticProfilePoint invalid = finite;
    invalid.cameraPointMm.x =
        std::numeric_limits<double>::quiet_NaN();
    std::vector<hik_scan::CloudPoint> cloud{
        pointAt(9.0, 8.0, 7.0, 1)};
    std::string error;
    CHECK_TRUE(!hik_scan::appendProfilePointsUsingBaseFromCamera(
                   std::vector<hik_calibration::StaticProfilePoint>{
                       finite, invalid},
                   cv::Matx44d::eye(), 2, &cloud, &error),
               "a non-finite source point must fail the complete transform");
    CHECK_TRUE(cloud.size() == 1U &&
                   nearlyEqual(cloud.front().basePointMm.x, 9.0),
               "a failed profile transform must not leave a partially "
               "appended formal or rejected point set");
}

void testTrueVGrooveIsValidatedAcrossProfiles() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(20, 5);
    hik_scan::VGrooveCandidateBranch trueBranch =
        makeVGrooveBranch(901U, 0, 20, 5, true);
    hik_scan::CloudPoint backward =
        pointAt(0.4, 4.0, -0.4, 22, 0.75);
    backward.qualityFlags |=
        hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE;
    trueBranch.points.push_back(backward);
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        trueBranch,
        makeVGrooveBranch(901U, 1, 20, 5, false)};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches,
                   vGrooveTestOptions(), &result, &error),
               std::string("true V validation failed: ") + error);
    CHECK_TRUE(result.passThroughPublishable.size() == cloud.size() &&
                   result.promotedCandidates.size() == 25U &&
                   result.rejectedCandidates.size() == 26U &&
                   result.statistics.rejectedOutlierPointCount == 1U,
               "a clean multi-profile V must preserve formal points and "
               "promote only the outward points of its unique branch");
    CHECK_TRUE(result.profiles.size() == 5U &&
                   result.statistics.promotedProfileCount == 5U,
               "each profile must use its own sliding-window V validation");
    for (std::size_t index = 0U;
         index < result.promotedCandidates.size(); ++index) {
        CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                       result.promotedCandidates[index],
                       hik_scan::CLOUD_QUALITY_V_GROOVE_GEOMETRY_VALIDATED),
                   "promoted candidates must carry a V geometry flag");
        CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                       result.promotedCandidates[index],
                       hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE),
                   "promotion must preserve the 2-D multipath provenance flag");
    }
}

void testVGrooveAmbiguousBranchesAreRejected() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(30, 5);
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        makeVGrooveBranch(1001U, 0, 30, 5, true),
        makeVGrooveBranch(1001U, 1, 30, 5, true)};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches, vGrooveTestOptions(),
                   &result, &error),
               std::string("ambiguous branch validation failed: ") + error);
    CHECK_TRUE(result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 50U,
               "two geometrically valid explicit branches must both be "
               "rejected, never score-selected");
    CHECK_TRUE(result.passThroughPublishable.size() == cloud.size(),
               "a local candidate ambiguity must not discard unrelated "
               "publishable V points");
    CHECK_TRUE(result.ambiguityGroups.size() == 5U,
               "the ambiguity result must be localized per center profile");
    for (std::size_t index = 0U;
         index < result.rejectedCandidates.size(); ++index) {
        CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                       result.rejectedCandidates[index],
                       hik_scan::CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS),
                   "double explanations must carry the ambiguity rejection flag");
    }
}

void testIndependentAmbiguityGroupsDoNotContaminateEachOther() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(40, 5);
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        makeVGrooveBranch(2001U, 0, 40, 5, true),
        makeVGrooveBranch(2001U, 1, 40, 5, false),
        makeVGrooveBranch(2002U, 0, 40, 5, true),
        makeVGrooveBranch(2002U, 1, 40, 5, true)};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches, vGrooveTestOptions(),
                   &result, &error),
               std::string("independent group validation failed: ") + error);
    CHECK_TRUE(result.promotedCandidates.size() == 25U,
               "the uniquely valid group must promote exactly its true branch");
    CHECK_TRUE(result.rejectedCandidates.size() == 75U,
               "the false alternate and both ambiguous-group branches must be "
               "reported as rejected candidates");
    std::size_t promotedGroups = 0U;
    std::size_t ambiguousGroups = 0U;
    for (std::size_t index = 0U;
         index < result.ambiguityGroups.size(); ++index) {
        if (result.ambiguityGroups[index].status ==
            hik_scan::VGrooveProfileStatus::PromotedUnique) {
            ++promotedGroups;
        } else if (result.ambiguityGroups[index].status ==
                   hik_scan::VGrooveProfileStatus::RejectedAmbiguous) {
            ++ambiguousGroups;
        }
    }
    CHECK_TRUE(promotedGroups == 5U && ambiguousGroups == 5U,
               "group-local status must preserve one unique and one ambiguous "
               "decision in every profile");
}

void testSinglePlaneAndInsufficientProfilesFailClosed() {
    std::vector<hik_scan::CloudPoint> singlePlane;
    for (int profile = 0; profile < 5; ++profile) {
        for (int sample = -8; sample <= 8; ++sample) {
            singlePlane.push_back(pointAt(
                0.4 * sample, 2.0 * profile, 0.0, profile));
        }
    }
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    const std::vector<hik_scan::VGrooveCandidateBranch> singlePlaneBranch{
        makeVGrooveBranch(2999U, 0, 0, 5, true),
        makeVGrooveBranch(2999U, 1, 0, 5, false)};
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   singlePlane, singlePlaneBranch,
                   vGrooveTestOptions(), &result, &error),
               std::string("single-plane validation failed: ") + error);
    CHECK_TRUE(result.passThroughPublishable.size() ==
                       singlePlane.size() &&
                   result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 50U &&
                   result.statistics.invalidGeometryProfileCount == 5U,
               "a single plane is not a V: candidate points must fail while "
               "formal publishable input remains untouched");

    const std::vector<hik_scan::CloudPoint> twoProfiles =
        makeVGrooveCloud(60, 2);
    const std::vector<hik_scan::VGrooveCandidateBranch> branch{
        makeVGrooveBranch(3001U, 0, 60, 2, true)};
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   twoProfiles, branch, vGrooveTestOptions(),
                   &result, &error),
               std::string("insufficient-profile validation failed: ") +
                   error);
    CHECK_TRUE(result.passThroughPublishable.size() ==
                       twoProfiles.size() &&
                   result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 10U &&
                   result.statistics.insufficientProfileCount == 2U,
               "fewer than the required profiles must never promote an "
               "ambiguous candidate branch");
}

void testVGrooveRobustFitRejectsOutliers() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(70, 5, true);
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        makeVGrooveBranch(4001U, 0, 70, 5, true),
        makeVGrooveBranch(4001U, 1, 70, 5, false)};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches,
                   vGrooveTestOptions(), &result, &error),
               std::string("outlier V validation failed: ") + error);
    CHECK_TRUE(result.passThroughPublishable.size() == cloud.size() &&
                   result.promotedCandidates.size() == 25U &&
                   result.rejectedCandidates.size() == 25U,
               "local robust fitting must tolerate synthetic support outliers "
               "without deleting existing formal points");
}

void testSingleBranchAmbiguityGroupFailsClosed() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(75, 5);
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        makeVGrooveBranch(4501U, 0, 75, 5, true)};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches, vGrooveTestOptions(),
                   &result, &error),
               std::string("single-branch group validation failed: ") +
                   error);
    CHECK_TRUE(result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 25U &&
                   result.statistics.insufficientProfileCount == 5U,
               "an ambiguity group missing its alternate branch is "
               "incomplete and must never be treated as uniquely solved");
}

void testOneWingReflectionCannotBePromoted() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(80, 5);
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        makeOneWingBranch(5001U, 0, 80, 5),
        makeVGrooveBranch(5001U, 1, 80, 5, false)};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches, vGrooveTestOptions(),
                   &result, &error),
               std::string("one-wing validation failed: ") + error);
    CHECK_TRUE(result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 50U &&
                   result.statistics.invalidGeometryProfileCount == 5U,
               "a reflection branch following only one V face must fail the "
               "two-face/root support gate");
}

void testBackwardPlaneExtensionsCannotBePromoted() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(100, 5);
    hik_scan::VGrooveCandidateBranch alternate =
        makeBackwardExtensionBranch(5251U, 1, 100, 102);
    for (hik_scan::CloudPoint& point : alternate.points) {
        point.basePointMm.z = 7.0;
        point.cameraPointMm.z = 7.0;
    }
    const std::vector<hik_scan::VGrooveCandidateBranch> branches{
        makeBackwardExtensionBranch(5251U, 0, 100, 102),
        alternate};
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches, vGrooveTestOptions(),
                   &result, &error),
               std::string("backward-extension validation failed: ") +
                   error);
    CHECK_TRUE(result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 10U,
               "points on the two planes behind their shared root must not "
               "masquerade as the two outward V faces");
}

void testAuditOnlyBranchCannotBePromoted() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(85, 5);
    hik_scan::VGrooveCandidateBranch branch =
        makeVGrooveBranch(5501U, 0, 85, 5, true);
    branch.formalPublicationEligible = false;
    const hik_scan::VGrooveCandidateBranch alternate =
        makeVGrooveBranch(5501U, 1, 85, 5, false);
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud,
                   std::vector<hik_scan::VGrooveCandidateBranch>{
                       branch, alternate},
                   vGrooveTestOptions(), &result, &error),
               std::string("audit-only V validation failed: ") + error);
    CHECK_TRUE(result.promotedCandidates.empty() &&
                   result.rejectedCandidates.size() == 50U &&
                   result.statistics
                           .rejectedInsufficientCandidatePointCount == 50U,
               "a branch withheld by an earlier formal count gate must stay "
               "rejected even when it perfectly follows a unique V");
}

void testPerProfileGroupsRetainTemporalVSupport() {
    const std::vector<hik_scan::CloudPoint> cloud =
        makeVGrooveCloud(90, 5);
    std::vector<hik_scan::VGrooveCandidateBranch> branches;
    for (int offset = 0; offset < 5; ++offset) {
        const int profile = 90 + offset;
        const std::uint64_t groupId =
            6001U + static_cast<std::uint64_t>(offset);
        branches.push_back(
            makeVGrooveBranch(groupId, 0, profile, 1, true));
        branches.push_back(
            makeVGrooveBranch(groupId, 1, profile, 1, false));
    }
    hik_scan::VGrooveTemporalValidationResult result;
    std::string error;
    CHECK_TRUE(hik_scan::validateVGrooveTemporalGeometry(
                   cloud, branches, vGrooveTestOptions(),
                   &result, &error),
               std::string("per-profile group validation failed: ") + error);
    CHECK_TRUE(result.promotedCandidates.size() == 25U &&
                   result.rejectedCandidates.size() == 25U &&
                   result.ambiguityGroups.size() == 5U,
               "profile-local 2-D interval IDs must still use neighboring "
               "base_link profiles to validate one unique physical V branch");
}

void testPlyQualityProperties() {
    hik_scan::CloudPoint point = pointAt(1.0, 2.0, 3.0, 5, 0.7);
    point.qualityFlags =
        hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED |
        hik_scan::CLOUD_QUALITY_VOXEL_AGGREGATED;
    point.observationCount = 4U;
    const std::vector<hik_scan::CloudPoint> cloud{point};

    std::ostringstream pathStream;
    pathStream << "/tmp/myline_hik_scan_core_test_"
               << static_cast<long long>(::getpid()) << ".ply";
    const std::string path = pathStream.str();
    std::remove(path.c_str());
    std::string error;
    CHECK_TRUE(hik_scan::saveScanPly(path, cloud, "base_link", &error),
               std::string("quality PLY save failed: ") + error);
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    std::remove(path.c_str());
    CHECK_TRUE(contents.find("property float pixel_v\n"
                             "property uint quality_flags\n"
                             "property uint observation_count\n") !=
                   std::string::npos,
               "new PLY properties must be appended after all legacy fields");
    const std::size_t headerEnd = contents.find("end_header\n");
    std::istringstream vertex(
        headerEnd == std::string::npos
            ? std::string()
            : contents.substr(headerEnd + std::string("end_header\n").size()));
    std::vector<std::string> fields;
    std::string field;
    while (vertex >> field) fields.push_back(field);
    CHECK_TRUE(fields.size() == 22U &&
                   fields[11] == "9" && fields[12] == "4",
               "PLY vertex must retain quality flags and observation count "
               "before appended optical evidence");
}

void testAsciiPlyCameraDepthColoring() {
    hik_scan::CloudPoint nearPoint = pointAt(1.0, 2.0, 100.0, 1);
    hik_scan::CloudPoint farPoint = pointAt(4.0, 5.0, 200.0, 2);
    const std::vector<hik_scan::CloudPoint> cloud{nearPoint, farPoint};
    hik_scan::PlyColorOptions colorOptions;
    colorOptions.scalar = hik_scan::PlyColorScalar::CameraDepth;
    colorOptions.lowerPercentile = 0.0;
    colorOptions.upperPercentile = 100.0;

    std::ostringstream pathStream;
    pathStream << "/tmp/myline_hik_scan_core_depth_ascii_test_"
               << static_cast<long long>(::getpid()) << ".ply";
    const std::string path = pathStream.str();
    std::remove(path.c_str());
    std::string error;
    CHECK_TRUE(hik_scan::saveScanPly(
                   path, cloud, "base_link", colorOptions, &error),
               std::string("depth-colored ASCII PLY save failed: ") +
                   error);
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    std::remove(path.c_str());
    const std::size_t headerEnd = contents.find("end_header\n");
    std::istringstream vertices(
        headerEnd == std::string::npos
            ? std::string()
            : contents.substr(
                  headerEnd + std::string("end_header\n").size()));
    std::string nearLine;
    std::string farLine;
    std::getline(vertices, nearLine);
    std::getline(vertices, farLine);
    CHECK_TRUE(
        contents.find("comment color_map turbo\n") != std::string::npos &&
            contents.find("comment color_scalar camera_depth_mm\n") !=
                std::string::npos &&
            nearLine.find(" 48 18 59 ") != std::string::npos &&
            farLine.find(" 122 4 3 ") != std::string::npos,
        "ASCII PLY must map near/far camera depth to different Turbo RGB "
        "values and describe the scalar in its header");
}

void testAsciiPlyBaseZHeightColoring() {
    hik_scan::CloudPoint lowPoint = pointAt(1.0, 2.0, -50.0, 1);
    hik_scan::CloudPoint highPoint = pointAt(4.0, 5.0, 50.0, 2);
    // Keep camera depth equal so this test proves the palette is driven by
    // base_link Z rather than the old camera-local depth scalar.
    lowPoint.cameraPointMm.z = 550.0;
    highPoint.cameraPointMm.z = 550.0;
    const std::vector<hik_scan::CloudPoint> cloud{lowPoint, highPoint};
    hik_scan::PlyColorOptions colorOptions;
    colorOptions.scalar = hik_scan::PlyColorScalar::BaseZ;
    colorOptions.lowerPercentile = 0.0;
    colorOptions.upperPercentile = 100.0;

    std::ostringstream pathStream;
    pathStream << "/tmp/myline_hik_scan_core_base_z_ascii_test_"
               << static_cast<long long>(::getpid()) << ".ply";
    const std::string path = pathStream.str();
    std::remove(path.c_str());
    std::string error;
    CHECK_TRUE(hik_scan::saveScanPly(
                   path, cloud, "base_link", colorOptions, &error),
               std::string("base-Z-colored ASCII PLY save failed: ") +
                   error);
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    std::remove(path.c_str());
    const std::size_t headerEnd = contents.find("end_header\n");
    std::istringstream vertices(
        headerEnd == std::string::npos
            ? std::string()
            : contents.substr(
                  headerEnd + std::string("end_header\n").size()));
    std::string lowLine;
    std::string highLine;
    std::getline(vertices, lowLine);
    std::getline(vertices, highLine);
    CHECK_TRUE(
        contents.find("comment color_scalar base_z_mm\n") !=
                std::string::npos &&
            lowLine.find(" 48 18 59 ") != std::string::npos &&
            highLine.find(" 122 4 3 ") != std::string::npos,
        "ASCII PLY must map low/high base_link Z to low/high Turbo colors");
}

void testBinaryPlyUsesSameSchemaAndFixedVertexSize() {
    hik_scan::CloudPoint first = pointAt(1.0, 2.0, 3.0, 5, 0.7);
    first.qualityFlags =
        hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED;
    first.observationCount = 4U;
    first.opticalMetricsValid = true;
    first.snr = 12.5;
    first.fwhmPx = 2.25;
    hik_scan::CloudPoint second = first;
    second.basePointMm = cv::Point3d(4.0, 5.0, 6.0);
    second.cameraPointMm = second.basePointMm;
    const std::vector<hik_scan::CloudPoint> cloud{first, second};

    std::ostringstream pathStream;
    pathStream << "/tmp/myline_hik_scan_core_binary_test_"
               << static_cast<long long>(::getpid()) << ".ply";
    const std::string path = pathStream.str();
    std::remove(path.c_str());
    std::string error;
    hik_scan::PlyColorOptions colorOptions;
    colorOptions.scalar = hik_scan::PlyColorScalar::CameraDepth;
    colorOptions.lowerPercentile = 0.0;
    colorOptions.upperPercentile = 100.0;
    CHECK_TRUE(hik_scan::saveScanPlyBinary(
                   path, cloud, "base_link", colorOptions, &error),
               std::string("binary PLY save failed: ") + error);
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    std::remove(path.c_str());
    const std::string marker = "end_header\n";
    const std::size_t headerEnd = contents.find(marker);
    CHECK_TRUE(
        contents.find("format binary_little_endian 1.0\n") !=
                std::string::npos &&
            contents.find("element vertex 2\n") != std::string::npos &&
            contents.find("comment color_scalar camera_depth_mm\n") !=
                std::string::npos &&
            contents.find("property uint stripe_reject_flags\n") !=
                std::string::npos,
        "binary PLY must expose the same quality schema");
    constexpr std::size_t kBinaryVertexBytes = 88U;
    const std::size_t vertexDataOffset = headerEnd + marker.size();
    CHECK_TRUE(
        headerEnd != std::string::npos &&
            contents.size() ==
                vertexDataOffset +
                    cloud.size() * kBinaryVertexBytes,
        "binary PLY vertex records must have the declared fixed-width layout");
    CHECK_TRUE(
        headerEnd != std::string::npos &&
            static_cast<unsigned char>(contents[vertexDataOffset + 24U]) ==
                48U &&
            static_cast<unsigned char>(contents[vertexDataOffset + 25U]) ==
                18U &&
            static_cast<unsigned char>(contents[vertexDataOffset + 26U]) ==
                59U &&
            static_cast<unsigned char>(
                contents[vertexDataOffset + kBinaryVertexBytes + 24U]) ==
                122U,
        "binary PLY must store Turbo camera-depth RGB in each vertex record");
}

}  // namespace

int main() {
    testReverseLinearFlangePathPreservesOrientation();
    testFilterDisabledByDefault();
    testSparseIsolatedPointRejected();
    testDistinctProfilesAndMaximumGap();
    testCoherentFalseSurfaceLimitation();
    testConfidenceWeightedVoxelDownsample();
    testQualityProfilePointsCarryOpticalAcceptance();
    testProfileTransformIsTransactional();
    testTrueVGrooveIsValidatedAcrossProfiles();
    testVGrooveAmbiguousBranchesAreRejected();
    testIndependentAmbiguityGroupsDoNotContaminateEachOther();
    testSinglePlaneAndInsufficientProfilesFailClosed();
    testVGrooveRobustFitRejectsOutliers();
    testSingleBranchAmbiguityGroupFailsClosed();
    testOneWingReflectionCannotBePromoted();
    testBackwardPlaneExtensionsCannotBePromoted();
    testAuditOnlyBranchCannotBePromoted();
    testPerProfileGroupsRetainTemporalVSupport();
    testPlyQualityProperties();
    testAsciiPlyCameraDepthColoring();
    testAsciiPlyBaseZHeightColoring();
    testBinaryPlyUsesSameSchemaAndFixedVertexSize();

    if (gFailures != 0) {
        std::cerr << "HikScanCore tests failed: " << gFailures
                  << " assertion(s)" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "HikScanCore tests passed" << std::endl;
    return EXIT_SUCCESS;
}
