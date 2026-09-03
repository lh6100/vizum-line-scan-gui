#include "AdaptiveScanExecutor.h"

#include <iostream>
#include <string>

namespace {

int failures = 0;

#define CHECK_TRUE(condition, message)                                  \
    do {                                                                \
        if (!(condition)) {                                             \
            std::cerr << "FAIL: " << (message) << " [" << __LINE__      \
                      << "]" << std::endl;                               \
            ++failures;                                                 \
        }                                                               \
    } while (false)

hik_scan::Pose6D pose(double x, double y) {
    hik_scan::Pose6D output;
    output.x = x;
    output.y = y;
    output.z = 300.0;
    output.rx = 180.0;
    return output;
}

hik_adaptive::ScanPlan plan() {
    hik_adaptive::SerpentineOptions options;
    options.firstLaneStart = pose(0.0, 0.0);
    options.firstLaneEnd = pose(100.0, 0.0);
    options.laneOffsetMm = cv::Vec3d(0.0, 10.0, 0.0);
    options.laneCount = 2;
    options.leadInMm = 5.0;
    options.leadOutMm = 5.0;
    hik_adaptive::ScanPlan output;
    std::string error;
    CHECK_TRUE(hik_adaptive::buildSerpentinePlan(
                   options, &output, &error),
               std::string("unable to build test plan: ") + error);
    return output;
}

void testCompleteSequence() {
    hik_adaptive::AdaptiveScanExecutor executor;
    std::string error;
    CHECK_TRUE(executor.load(plan(), &error), error);
    auto command = executor.begin();
    CHECK_TRUE(
        command.kind == hik_adaptive::AdaptiveScanExecutor::CommandKind::
                            MoveToPlanStart &&
        command.target.x == -5.0,
        "executor first moves to the lead-in start");
    command = executor.motionFinished(true);
    CHECK_TRUE(
        command.kind == hik_adaptive::AdaptiveScanExecutor::CommandKind::
                            StartAcquisition,
        "acquisition starts only after reaching plan start");
    command = executor.acquisitionStarted(true);
    CHECK_TRUE(
        command.kind ==
            hik_adaptive::AdaptiveScanExecutor::CommandKind::
                ExecuteTrajectory &&
        command.trajectory.size() == 3U &&
        command.trajectory[0].primitive ==
            hik_adaptive::MotionPrimitive::Line &&
        command.trajectory[1].primitive ==
            hik_adaptive::MotionPrimitive::Arc &&
        command.trajectory[2].primitive ==
            hik_adaptive::MotionPrimitive::Line,
        "verified tangent snake is pre-submitted as LINE/ARC/LINE");
    command = executor.motionFinished(true);
    CHECK_TRUE(
        command.kind == hik_adaptive::AdaptiveScanExecutor::CommandKind::
                            StopAcquisition &&
        command.completed,
        "camera stops after the final measurement lane");
    command = executor.acquisitionStopped(true);
    CHECK_TRUE(
        command.kind == hik_adaptive::AdaptiveScanExecutor::CommandKind::
                            Finalize &&
        command.completed,
        "confirmed camera stop completes the frozen plan");
}

void testLineFallbackUsesStoppedSequentialExecution() {
    hik_adaptive::ScanPlan fallback = plan();
    fallback.segments[0].blendRadiusMm = 0.0;
    fallback.segments[1].primitive =
        hik_adaptive::MotionPrimitive::Line;
    fallback.segments[1].blendRadiusMm = 0.0;
    fallback.segments[1].usedLineFallback = true;
    hik_adaptive::AdaptiveScanExecutor executor;
    std::string error;
    CHECK_TRUE(executor.load(fallback, &error), error);
    CHECK_TRUE(!executor.usesPreSubmittedTrajectory(),
               "one line fallback disables controller batch blending");
    (void)executor.begin();
    (void)executor.motionFinished(true);
    auto command = executor.acquisitionStarted(true);
    CHECK_TRUE(
        command.kind ==
            hik_adaptive::AdaptiveScanExecutor::CommandKind::
                ExecuteSegment &&
        command.gate == hik_adaptive::AdaptiveScanExecutor::
                            MeasurementGate::Configure,
        "fallback starts the first stopped measurement MoveL");
    command = executor.motionFinished(true);
    CHECK_TRUE(
        command.kind ==
            hik_adaptive::AdaptiveScanExecutor::CommandKind::
                ExecuteSegment &&
        command.gate == hik_adaptive::AdaptiveScanExecutor::
                            MeasurementGate::Suspend &&
        command.segment.primitive ==
            hik_adaptive::MotionPrimitive::Line,
        "fallback transition is a stopped line with measurement suspended");
}

void testMixedFallbackPlanIsRejected() {
    hik_adaptive::ScanPlan mixed = plan();
    mixed.segments[0].blendRadiusMm = 0.0;
    mixed.segments[1].blendRadiusMm = 0.0;
    hik_adaptive::AdaptiveScanExecutor executor;
    std::string error;
    CHECK_TRUE(!executor.load(mixed, &error) &&
               error.find("mixed ARC/LINE") != std::string::npos,
               "an ARC may not enter the stopped sequential fallback path");
}

void testFailureIsFailClosed() {
    hik_adaptive::AdaptiveScanExecutor executor;
    std::string error;
    CHECK_TRUE(executor.load(plan(), &error), error);
    (void)executor.begin();
    (void)executor.motionFinished(true);
    (void)executor.acquisitionStarted(true);
    auto command = executor.motionFinished(false, "synthetic motion fault");
    CHECK_TRUE(
        command.kind == hik_adaptive::AdaptiveScanExecutor::CommandKind::
                            StopAcquisition &&
        !command.completed,
        "motion fault requests acquisition stop");
    command = executor.acquisitionStopped(false, "stop not confirmed");
    CHECK_TRUE(
        command.kind == hik_adaptive::AdaptiveScanExecutor::CommandKind::
                            Finalize &&
        !command.completed &&
        executor.state() ==
            hik_adaptive::AdaptiveScanExecutor::State::Failed,
        "unconfirmed stop can never publish a completed plan");
}

}  // namespace

int main() {
    testCompleteSequence();
    testLineFallbackUsesStoppedSequentialExecution();
    testMixedFallbackPlanIsRejected();
    testFailureIsFailClosed();
    if (failures == 0) {
        std::cout << "Adaptive scan executor tests passed." << std::endl;
        return 0;
    }
    std::cerr << failures << " adaptive executor test(s) failed."
              << std::endl;
    return 1;
}
