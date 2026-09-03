#include "AdaptiveScanPlanner.h"
#include "HandEyeCalibrationCore.h"
#include "StripeCenterlineExtractor.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

#define CHECK_TRUE(condition, message)                                      \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::cerr << "FAIL: " << (message) << " [" << __FILE__ << ':'  \
                      << __LINE__ << "]" << std::endl;                       \
            ++gFailures;                                                    \
        }                                                                   \
    } while (false)

bool near(double first, double second, double tolerance = 1.0e-9) {
    return std::abs(first - second) <= tolerance;
}

hik_scan::Pose6D pose(double x, double y, double z = 300.0) {
    hik_scan::Pose6D output;
    output.x = x;
    output.y = y;
    output.z = z;
    output.rx = 180.0;
    return output;
}

hik_scan::CloudPoint cloudPoint(double x, double y, double z,
                                int profile, double confidence) {
    hik_scan::CloudPoint point;
    point.basePointMm = cv::Point3d(x, y, z);
    point.cameraPointMm = cv::Point3d(x, y, z);
    point.profileIndex = profile;
    point.confidence = confidence;
    point.response = 100.0;
    return point;
}

hik_adaptive::QualityObservation observation(
        const hik_scan::CloudPoint& point,
        hik_adaptive::ObservationRole role) {
    hik_adaptive::QualityObservation output;
    output.point = point;
    output.role = role;
    return output;
}

void testSerpentinePath() {
    hik_adaptive::SerpentineOptions options;
    options.firstLaneStart = pose(0.0, 0.0);
    options.firstLaneEnd = pose(100.0, 0.0);
    options.laneOffsetMm = cv::Vec3d(0.0, 10.0, 0.0);
    options.laneCount = 3;
    options.leadInMm = 5.0;
    options.leadOutMm = 5.0;
    hik_adaptive::ScanPlan plan;
    std::string error;
    CHECK_TRUE(hik_adaptive::buildSerpentinePlan(
                   options, &plan, &error),
               std::string("serpentine build failed: ") + error);
    CHECK_TRUE(plan.segments.size() == 5U,
               "three lanes have two explicit transitions");
    CHECK_TRUE(
        plan.segments[0].kind ==
            hik_adaptive::SegmentKind::Measurement &&
        near(plan.segments[0].start.x, 0.0) &&
        near(plan.segments[0].end.x, 100.0) &&
        near(plan.segments[0].motionStart.x, -5.0) &&
        near(plan.segments[0].motionEnd.x, 105.0),
        "first lane separates motion and measurement intervals");
    CHECK_TRUE(
        plan.segments[2].kind ==
            hik_adaptive::SegmentKind::Measurement &&
        near(plan.segments[2].start.x, 100.0) &&
        near(plan.segments[2].end.x, 0.0) &&
        near(plan.segments[2].start.y, 10.0),
        "odd snake lane reverses direction");
    CHECK_TRUE(
        plan.segments[1].kind ==
            hik_adaptive::SegmentKind::Transition &&
        near(plan.segments[1].start.x, 105.0) &&
        near(plan.segments[1].end.x, 105.0) &&
        near(plan.segments[1].end.y, 10.0) &&
        plan.segments[1].primitive ==
            hik_adaptive::MotionPrimitive::Arc &&
        near(plan.segments[1].arcRadiusMm, 5.0) &&
        near(plan.segments[1].arcVia.x, 110.0) &&
        near(plan.segments[1].arcVia.y, 5.0) &&
        plan.segments[0].blendRadiusMm > 0.0 &&
        plan.segments[1].blendRadiusMm > 0.0,
        "lane transition is a tangent blended MoveC semicircle");
    CHECK_TRUE(near(plan.transitionLengthMm, 10.0 * 3.14159265358979323846),
               "two transition lengths use semicircle arc length");
    std::vector<hik_scan::Pose6D> arcSamples;
    CHECK_TRUE(hik_adaptive::sampleSegmentMotion(
                   plan.segments[1], 1.0, 5.0, 128U,
                   &arcSamples, &error) &&
               arcSamples.size() > 10U &&
               near(arcSamples.front().x, 105.0) &&
               near(arcSamples.back().x, 105.0) &&
               near(arcSamples.back().y, 10.0),
               "MoveC evaluator samples the actual circular path");
    CHECK_TRUE(near(plan.measurementLengthMm, 300.0),
               "measurement length excludes lead-in/out");
    CHECK_TRUE(!plan.safetyVerified && !plan.executable,
               "geometry generation alone never authorizes motion");

    options.maximumTotalLengthMm = 100.0;
    CHECK_TRUE(!hik_adaptive::buildSerpentinePlan(
                   options, &plan, &error),
               "physical path length guard includes all lanes and leads");

    options.maximumTotalLengthMm = 5000.0;
    options.minimumArcRadiusMm = 6.0;
    CHECK_TRUE(hik_adaptive::buildSerpentinePlan(
                   options, &plan, &error) &&
               plan.segments[1].primitive ==
                   hik_adaptive::MotionPrimitive::Line &&
               plan.segments[1].usedLineFallback &&
               plan.segments[0].blendRadiusMm == 0.0,
               "insufficient turn radius falls back to a stopped line");
}

void testTrapezoidalTime() {
    CHECK_TRUE(near(hik_adaptive::estimateTrapezoidalMoveTime(
                        100.0, 10.0, 100.0),
                    10.1, 1.0e-12),
               "long move uses trapezoidal time");
    CHECK_TRUE(near(hik_adaptive::estimateTrapezoidalMoveTime(
                        0.25, 10.0, 100.0),
                    0.1, 1.0e-12),
               "short move uses triangular time");
}

void testQualityMapAndRois() {
    hik_scan::CloudPoint formal =
        cloudPoint(-0.1, 0.1, 0.1, 1, 0.9);
    hik_scan::CloudPoint quality = formal;
    quality.opticalMetricsValid = true;
    quality.snr = 12.0;
    quality.fwhmPx = 2.0;
    quality.saturatedFraction = 0.0;
    hik_scan::CloudPoint rejected =
        cloudPoint(-1.9, 0.1, 0.1, 2, 0.1);
    rejected.opticalMetricsValid = true;
    rejected.snr = 1.0;
    rejected.fwhmPx = 9.0;
    rejected.saturatedFraction = 0.8;
    rejected.qualityFlags =
        hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE;
    rejected.stripeRejectFlags =
        hik_stripe::REJECT_SATURATED_WIDE_PLATEAU |
        hik_stripe::REJECT_AMBIGUOUS_MULTIPATH;

    std::vector<hik_adaptive::QualityObservation> observations;
    hik_adaptive::QualityObservation formalObservation =
        observation(
            formal, hik_adaptive::ObservationRole::FormalAccepted);
    formalObservation.hasCameraOrigin = true;
    formalObservation.cameraOriginBaseMm =
        cv::Point3d(-0.1, 0.1, 100.1);
    observations.push_back(formalObservation);
    observations.push_back(observation(
        quality, hik_adaptive::ObservationRole::QualityAccepted));
    observations.push_back(observation(
        rejected, hik_adaptive::ObservationRole::Rejected));
    hik_adaptive::QualityMapOptions options;
    options.voxelSizeMm = 2.0;
    options.rejectedRatioThreshold = 0.25;
    hik_adaptive::QualityMap map;
    std::string error;
    CHECK_TRUE(hik_adaptive::buildQualityMap(
                   observations, {cv::Point3d(4.1, 0.1, 0.1)},
                   options, &map, &error),
               std::string("quality map failed: ") + error);
    const hik_adaptive::VoxelKey negative{-1, 0, 0};
    const hik_adaptive::VoxelKey expected{2, 0, 0};
    CHECK_TRUE(map.voxels.count(negative) == 1U,
               "negative coordinates use floor, not truncation");
    const hik_adaptive::QualityVoxel& disputed =
        map.voxels.at(negative);
    CHECK_TRUE(
        disputed.formalAcceptedObservationCount == 1U &&
        disputed.qualityAcceptedObservationCount == 1U &&
        disputed.rejectedObservationCount == 1U,
        "formal, quality and rejected evidence remain separate");
    CHECK_TRUE(near(disputed.evidenceCentroidMm.x, -0.1),
               "rejected multipath branch never changes surface centroid");
    CHECK_TRUE(
        disputed.viewObservationCount == 1U &&
        near(disputed.meanObservedViewDirectionBase[2], 1.0),
        "formal viewpoint is retained in base_link for NBS view diversity");
    CHECK_TRUE(
        (disputed.stateFlags &
         hik_adaptive::QUALITY_VOXEL_MULTIPATH) != 0U &&
        (disputed.stateFlags &
         hik_adaptive::QUALITY_VOXEL_SATURATED) != 0U &&
        disputed.needsRescan(),
        "multipath and saturation become spatial rescan evidence");
    CHECK_TRUE(
        map.voxels.at(expected).needsRescan() &&
        (map.voxels.at(expected).stateFlags &
         hik_adaptive::QUALITY_VOXEL_UNOBSERVED_EXPECTED) != 0U,
        "target mask exposes completely unobserved cells");

    hik_adaptive::RoiClusteringOptions cluster;
    cluster.paddingMm = 1.0;
    std::vector<hik_adaptive::RescanRoi> rois;
    CHECK_TRUE(hik_adaptive::clusterRescanRois(
                   map, cluster, &rois, &error),
               std::string("ROI clustering failed: ") + error);
    CHECK_TRUE(rois.size() == 2U,
               "spatially separated defects remain separate ROIs");
}

hik_adaptive::RobotPathEvaluation passedRobot(
        const hik_scan::Pose6D&,
        const hik_adaptive::ScanSegment& segment) {
    hik_adaptive::RobotPathEvaluation result;
    result.ik = hik_adaptive::VerificationState::Passed;
    result.collision = hik_adaptive::VerificationState::Passed;
    result.singularity = hik_adaptive::VerificationState::Passed;
    result.minimumSingularValue = 0.2;
    result.minimumJointLimitMarginDeg = 20.0;
    result.estimatedExecutionTimeS =
        hik_adaptive::estimateTrapezoidalMoveTime(
            cv::norm(cv::Vec3d(
                segment.end.x - segment.start.x,
                segment.end.y - segment.start.y,
                segment.end.z - segment.start.z)),
            segment.speedMmS, segment.accelerationMmS2);
    result.detail = "synthetic verified robot path";
    return result;
}

hik_adaptive::CandidateAction action(
        int id, int roiId, double gain, double x) {
    hik_adaptive::CandidateAction output;
    output.actionId = id;
    output.roiId = roiId;
    output.measurement.start = pose(x, 0.0);
    output.measurement.end = pose(x + 10.0, 0.0);
    output.measurement.motionStart = output.measurement.start;
    output.measurement.motionEnd = output.measurement.end;
    output.measurement.speedMmS = 10.0;
    output.measurement.accelerationMmS2 = 100.0;
    output.predictedCoverageGain = gain;
    output.coveredVoxels.push_back(
        hik_adaptive::VoxelKey{roiId, 0, 0});
    output.fov = hik_adaptive::VerificationState::Passed;
    output.laserSweep = hik_adaptive::VerificationState::Passed;
    output.occlusion = hik_adaptive::VerificationState::Passed;
    return output;
}

void testGreedyAndBeamSafety() {
    std::vector<hik_adaptive::CandidateAction> candidates{
        action(0, 0, 10.0, 20.0),
        action(1, 1, 8.0, 1.0),
        action(2, 2, 7.0, 12.0)};
    hik_adaptive::SearchOptions options;
    options.allowUnverifiedForDryRun = false;
    options.requireFullyVerifiedForExecution = true;
    options.horizon = 2;
    options.beamWidth = 8U;
    std::string error;
    CHECK_TRUE(hik_adaptive::evaluateAndRankCandidates(
                   pose(0.0, 0.0), options, passedRobot,
                   &candidates, &error),
               std::string("candidate ranking failed: ") + error);
    hik_adaptive::CandidateAction greedy;
    CHECK_TRUE(hik_adaptive::selectGreedyAction(
                   candidates, options, &greedy, &error),
               std::string("greedy selection failed: ") + error);
    CHECK_TRUE(greedy.robot.fullyVerified(),
               "greedy output carries explicit robot verification");

    hik_adaptive::PlannedActionSequence sequence;
    CHECK_TRUE(hik_adaptive::beamSearchActions(
                   pose(0.0, 0.0), candidates, options,
                   passedRobot, &sequence, &error),
               std::string("beam search failed: ") + error);
    CHECK_TRUE(sequence.actions.size() == 2U &&
               sequence.fullyVerified && sequence.executable,
               "two-step beam returns only fully verified actions");
    CHECK_TRUE(
        sequence.actions[0].roiId != sequence.actions[1].roiId,
        "beam uses marginal coverage and does not repeat one ROI");

    for (hik_adaptive::CandidateAction& candidate : candidates) {
        candidate.robot = hik_adaptive::RobotPathEvaluation();
    }
    CHECK_TRUE(!hik_adaptive::selectGreedyAction(
                   candidates, options, &greedy, &error),
               "UNKNOWN robot safety is fail-closed for execution");
    options.allowUnverifiedForDryRun = true;
    CHECK_TRUE(hik_adaptive::selectGreedyAction(
                   candidates, options, &greedy, &error),
               "UNKNOWN safety remains inspectable in explicit dry-run");
}

void testCandidateLibraryAndUnknownObservability() {
    hik_adaptive::QualityMap map;
    map.voxelSizeMm = 2.0;
    hik_adaptive::QualityVoxel voxel;
    voxel.key = hik_adaptive::VoxelKey{0, 0, 0};
    voxel.centerMm = cv::Point3d(1.0, 1.0, 1.0);
    voxel.evidenceCentroidMm = voxel.centerMm;
    voxel.stateFlags =
        hik_adaptive::QUALITY_VOXEL_NEEDS_RESCAN;
    voxel.severity = 2.0;
    map.voxels[voxel.key] = voxel;
    hik_adaptive::RescanRoi roi;
    roi.roiId = 0;
    roi.voxelKeys.push_back(voxel.key);
    roi.minimumMm = cv::Point3d(0.0, 0.0, 0.0);
    roi.maximumMm = cv::Point3d(2.0, 2.0, 2.0);
    roi.centerMm = cv::Point3d(1.0, 1.0, 1.0);
    roi.severity = 2.0;
    hik_adaptive::CandidateGenerationContext context;
    context.baseFromReferenceFlange =
        hik_calibration::fairinoBaseFromFlange(
            1.0, 1.0, -299.0, 0.0, 0.0, 0.0);
    hik_adaptive::CandidateLibraryOptions options;
    options.yawOffsetsDeg = {0.0};
    options.pitchOffsetsDeg = {0.0};
    options.workingDistanceScales = {1.0};
    options.speedsMmS = {10.0};
    options.exposureUs = {1000.0};
    options.includeReverseDirection = false;
    std::vector<hik_adaptive::CandidateAction> candidates;
    std::string error;
    CHECK_TRUE(hik_adaptive::generateCandidateLibrary(
                   map, {roi}, context, options,
                   &candidates, &error),
               std::string("candidate generation failed: ") + error);
    CHECK_TRUE(candidates.size() == 1U,
               "finite candidate template is generated");
    CHECK_TRUE(
        candidates.front().fov ==
            hik_adaptive::VerificationState::Unknown &&
        candidates.front().laserSweep ==
            hik_adaptive::VerificationState::Unknown &&
        candidates.front().occlusion ==
            hik_adaptive::VerificationState::Unknown,
        "missing optical/occlusion model is explicit UNKNOWN");
}

void testPlyRoundTripAndOldSchema() {
    const std::string path = "/tmp/myline_adaptive_roundtrip.ply";
    hik_scan::CloudPoint input =
        cloudPoint(1.0, 2.0, 3.0, 7, 0.8);
    input.opticalMetricsValid = true;
    input.snr = 9.0;
    input.fwhmPx = 2.5;
    input.saturatedFraction = 0.1;
    input.secondPeakRatio = 0.2;
    input.gradientAsymmetry = 0.3;
    input.fitResidual = 0.4;
    input.centerSigmaPx = 0.05;
    input.stripeRejectFlags = hik_stripe::REJECT_PATH_AMBIGUOUS;
    std::string error;
    CHECK_TRUE(hik_scan::saveScanPly(
                   path, {input}, "base_link", &error),
               std::string("test PLY save failed: ") + error);
    std::vector<hik_scan::CloudPoint> loaded;
    CHECK_TRUE(hik_adaptive::loadCloudPly(
                   path, &loaded, &error),
               std::string("test PLY load failed: ") + error);
    CHECK_TRUE(loaded.size() == 1U &&
               loaded[0].opticalMetricsValid &&
               near(loaded[0].snr, 9.0) &&
               loaded[0].stripeRejectFlags ==
                   hik_stripe::REJECT_PATH_AMBIGUOUS,
               "adaptive PLY preserves point-local optical evidence");
    std::remove(path.c_str());

    const std::string oldPath =
        "/tmp/myline_adaptive_old_schema.ply";
    {
        std::ofstream output(oldPath.c_str());
        output << "ply\nformat ascii 1.0\n"
               << "element vertex 1\n"
               << "property double x\nproperty double y\n"
               << "property double z\nend_header\n"
               << "1 2 3\n";
    }
    CHECK_TRUE(hik_adaptive::loadCloudPly(
                   oldPath, &loaded, &error),
               std::string("old PLY load failed: ") + error);
    CHECK_TRUE(
        loaded.size() == 1U &&
        (loaded[0].qualityFlags &
         hik_scan::CLOUD_QUALITY_UNKNOWN_SCHEMA) != 0U,
        "old PLY missing provenance is UNKNOWN, never silently GOOD");
    std::remove(oldPath.c_str());
}

void testAdaptivePlanArtifactContract() {
    hik_adaptive::SerpentineOptions options;
    options.firstLaneStart = pose(0.0, 0.0);
    options.firstLaneEnd = pose(20.0, 0.0);
    options.laneOffsetMm = cv::Vec3d(0.0, 10.0, 0.0);
    options.laneCount = 2;
    options.leadInMm = 3.0;
    options.leadOutMm = 3.0;
    hik_adaptive::ScanPlan global;
    std::string error;
    CHECK_TRUE(hik_adaptive::buildSerpentinePlan(
                   options, &global, &error),
               std::string("artifact plan build failed: ") + error);
    global.segments.front().robot.ik =
        hik_adaptive::VerificationState::Passed;
    global.segments.front().robot.collision =
        hik_adaptive::VerificationState::Unknown;
    global.segments.front().robot.singularity =
        hik_adaptive::VerificationState::Passed;
    global.segments.front().robot.minimumJointLimitMarginDeg = 8.5;
    global.segments.front().robot.minimumSingularValue = 0.12;
    global.segments.front().robot.estimatedExecutionTimeS = 2.25;
    global.segments.front().actualExecutionTimeS = 2.5;
    hik_adaptive::QualityMap map;
    map.voxelSizeMm = 2.0;
    hik_adaptive::QualityVoxel voxel;
    voxel.key = hik_adaptive::VoxelKey{1, 2, 3};
    voxel.centerMm = cv::Point3d(3.0, 5.0, 7.0);
    voxel.evidenceCentroidMm = voxel.centerMm;
    voxel.rejectedObservationCount = 2U;
    voxel.stateFlags =
        hik_adaptive::QUALITY_VOXEL_NEEDS_RESCAN;
    voxel.severity = 2.0;
    map.voxels[voxel.key] = voxel;
    map.rejectedPointCount = 2U;
    map.rescanVoxelCount = 1U;
    hik_adaptive::PlannedActionSequence noLocalPlan;
    const std::string path =
        "/tmp/myline_adaptive_plan_contract.json";
    CHECK_TRUE(hik_adaptive::saveAdaptivePlanJson(
                   path, global, map, {}, noLocalPlan, &error),
               std::string("adaptive artifact save failed: ") + error);
    std::ifstream input(path.c_str());
    std::ostringstream text;
    text << input.rdbuf();
    const std::string json = text.str();
    CHECK_TRUE(
        json.find("\"motion_start\"") != std::string::npos &&
        json.find("\"motion_end\"") != std::string::npos &&
        json.find("\"primitive\":\"ARC\"") != std::string::npos &&
        json.find("\"arc_via\"") != std::string::npos &&
        json.find("\"blend_radius_mm\"") != std::string::npos &&
        json.find("\"lead_in_mm\":3") != std::string::npos &&
        json.find("\"lead_out_mm\":3") != std::string::npos &&
        json.find("\"ik\":\"PASSED\"") != std::string::npos &&
        json.find("\"collision\":\"UNKNOWN\"") !=
            std::string::npos &&
        json.find("\"actual_execution_time_s\":2.5") !=
            std::string::npos,
        "global artifact contains LINE/ARC geometry, FR5 evidence and timing");
    CHECK_TRUE(
        json.find("\"voxels\"") != std::string::npos &&
        json.find("\"stripe_reject_flags\"") !=
            std::string::npos &&
        json.find("\"total_utility\":null") !=
            std::string::npos &&
        json.find("-inf") == std::string::npos,
        "quality map is auditable and an empty local plan remains valid JSON");
    if (!std::getenv("MYLINE_KEEP_ADAPTIVE_PLAN_JSON")) {
        std::remove(path.c_str());
    }
}

}  // namespace

int main() {
    testSerpentinePath();
    testTrapezoidalTime();
    testQualityMapAndRois();
    testGreedyAndBeamSafety();
    testCandidateLibraryAndUnknownObservability();
    testPlyRoundTripAndOldSchema();
    testAdaptivePlanArtifactContract();
    if (gFailures != 0) {
        std::cerr << "AdaptiveScanPlanner tests failed: "
                  << gFailures << std::endl;
        return 1;
    }
    std::cout << "AdaptiveScanPlanner tests passed" << std::endl;
    return 0;
}
