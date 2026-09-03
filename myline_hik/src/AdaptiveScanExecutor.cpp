#include "AdaptiveScanExecutor.h"

#include <cmath>

namespace hik_adaptive {
namespace {

bool finitePose(const hik_scan::Pose6D& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.z) && std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) && std::isfinite(pose.rz);
}

bool validSegment(const ScanSegment& segment) {
    const bool primitiveValid =
        segment.primitive == MotionPrimitive::Line ||
        (segment.primitive == MotionPrimitive::Arc &&
         finitePose(segment.arcVia) &&
         std::isfinite(segment.arcRadiusMm) &&
         segment.arcRadiusMm > 0.0);
    return segment.segmentId >= 0 && primitiveValid &&
           finitePose(segment.start) && finitePose(segment.end) &&
           finitePose(segment.motionStart) &&
           finitePose(segment.motionEnd) &&
           std::isfinite(segment.speedMmS) &&
           segment.speedMmS > 0.0 &&
           std::isfinite(segment.accelerationMmS2) &&
           segment.accelerationMmS2 > 0.0;
}

void setError(const std::string& message, std::string* error) {
    if (error) *error = message;
}

}  // namespace

bool AdaptiveScanExecutor::load(const ScanPlan& plan, std::string* error) {
    reset();
    if (plan.profileId != "scanner_650") {
        setError("adaptive v1 accepts scanner_650 plans only", error);
        return false;
    }
    if (plan.segments.empty()) {
        setError("adaptive plan has no segments", error);
        return false;
    }
    if (plan.segments.front().kind != SegmentKind::Measurement) {
        setError("adaptive plan must start with a measurement segment", error);
        return false;
    }
    SegmentKind previous = SegmentKind::Transition;
    for (const ScanSegment& segment : plan.segments) {
        if (!validSegment(segment)) {
            setError("adaptive plan contains an invalid segment", error);
            return false;
        }
        if (segment.kind == SegmentKind::Measurement &&
            segment.primitive != MotionPrimitive::Line) {
            setError("adaptive v1 measurement segments must be LINE",
                     error);
            return false;
        }
        if (segment.kind == previous) {
            setError("measurement and transition segments must alternate",
                     error);
            return false;
        }
        previous = segment.kind;
    }
    if (plan.segments.back().kind != SegmentKind::Measurement) {
        setError("adaptive plan must end with a measurement segment", error);
        return false;
    }
    plan_ = plan;
    preSubmittedTrajectory_ = plan_.segments.size() > 1U;
    for (std::size_t index = 0U;
         index < plan_.segments.size(); ++index) {
        const ScanSegment& segment = plan_.segments[index];
        if (segment.kind == SegmentKind::Measurement &&
            segment.primitive != MotionPrimitive::Line) {
            preSubmittedTrajectory_ = false;
        }
        if (segment.kind == SegmentKind::Transition &&
            segment.primitive != MotionPrimitive::Arc) {
            preSubmittedTrajectory_ = false;
        }
        if (index + 1U < plan_.segments.size() &&
            segment.blendRadiusMm <= 0.0) {
            preSubmittedTrajectory_ = false;
        }
    }
    if (!preSubmittedTrajectory_) {
        for (const ScanSegment& segment : plan_.segments) {
            if (segment.kind == SegmentKind::Transition &&
                segment.primitive == MotionPrimitive::Arc) {
                reset();
                setError(
                    "mixed ARC/LINE fallback plan is forbidden; "
                    "all transitions must be converted to stopped LINE",
                    error);
                return false;
            }
        }
    }
    state_ = State::Idle;
    if (error) error->clear();
    return true;
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::begin() {
    if (state_ != State::Idle || plan_.segments.empty()) {
        return finalize(false, "adaptive executor is not ready");
    }
    state_ = State::MovingToPlanStart;
    segmentIndex_ = 0U;
    Command command;
    command.kind = CommandKind::MoveToPlanStart;
    command.segmentIndex = 0U;
    command.segment = plan_.segments.front();
    command.target = plan_.segments.front().motionStart;
    command.detail = "move to first lane lead-in start";
    return command;
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::motionFinished(
        bool targetReached, const std::string& detail) {
    if (state_ == State::MovingToPlanStart) {
        if (!targetReached) {
            return finalize(false,
                detail.empty() ? "failed to reach plan start" : detail);
        }
        state_ = State::WaitingForAcquisition;
        Command command;
        command.kind = CommandKind::StartAcquisition;
        command.segmentIndex = 0U;
        command.detail = "plan start reached";
        return command;
    }
    if (state_ != State::ExecutingSegment) {
        if (state_ == State::ExecutingTrajectory) {
            if (!targetReached) {
                terminalDetail_ = detail.empty()
                    ? "pre-submitted adaptive trajectory failed"
                    : detail;
                return stopAcquisition(false, terminalDetail_);
            }
            return stopAcquisition(
                true,
                "all pre-submitted adaptive segments completed");
        }
        return Command{};
    }
    if (!targetReached) {
        terminalDetail_ =
            detail.empty() ? "adaptive segment motion failed" : detail;
        return stopAcquisition(false, terminalDetail_);
    }
    ++segmentIndex_;
    if (segmentIndex_ >= plan_.segments.size()) {
        return stopAcquisition(true, "all adaptive segments completed");
    }
    return executeCurrentSegment();
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::acquisitionStarted(
        bool success, const std::string& detail) {
    if (state_ != State::WaitingForAcquisition) return Command{};
    if (!success) {
        return finalize(false,
            detail.empty() ? "continuous acquisition failed to start"
                           : detail);
    }
    acquisitionRunning_ = true;
    if (preSubmittedTrajectory_) {
        state_ = State::ExecutingTrajectory;
        Command command;
        command.kind = CommandKind::ExecuteTrajectory;
        command.trajectory = plan_.segments;
        command.segmentIndex = 0U;
        command.detail =
            "execute controller-pre-submitted LINE/ARC trajectory";
        return command;
    }
    return executeCurrentSegment();
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::acquisitionStopped(
        bool confirmed, const std::string& detail) {
    if (state_ != State::WaitingForAcquisitionStop) return Command{};
    acquisitionRunning_ = false;
    if (!confirmed) {
        completionRequested_ = false;
        terminalDetail_ =
            detail.empty() ? "continuous acquisition stop was not confirmed"
                           : detail;
    }
    return finalize(completionRequested_,
                    terminalDetail_.empty() ? detail : terminalDetail_);
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::requestStop(
        const std::string& reason) {
    if (!active()) return Command{};
    terminalDetail_ =
        reason.empty() ? "adaptive execution stopped" : reason;
    completionRequested_ = false;
    preSubmittedTrajectory_ = false;
    if (acquisitionRunning_) {
        return stopAcquisition(false, terminalDetail_);
    }
    return finalize(false, terminalDetail_);
}

void AdaptiveScanExecutor::reset() {
    plan_ = ScanPlan();
    state_ = State::Idle;
    segmentIndex_ = 0U;
    acquisitionRunning_ = false;
    completionRequested_ = false;
    terminalDetail_.clear();
}

bool AdaptiveScanExecutor::active() const {
    return state_ != State::Idle &&
           state_ != State::Completed &&
           state_ != State::Failed;
}

AdaptiveScanExecutor::Command
AdaptiveScanExecutor::executeCurrentSegment() {
    if (segmentIndex_ >= plan_.segments.size()) {
        return stopAcquisition(true, "all adaptive segments completed");
    }
    state_ = State::ExecutingSegment;
    const ScanSegment& segment = plan_.segments[segmentIndex_];
    Command command;
    command.kind = CommandKind::ExecuteSegment;
    command.segmentIndex = segmentIndex_;
    command.segment = segment;
    command.target = segment.motionEnd;
    command.gate = segment.kind == SegmentKind::Measurement
        ? MeasurementGate::Configure
        : MeasurementGate::Suspend;
    command.detail = segment.kind == SegmentKind::Measurement
        ? "execute measurement lane with an active spatial gate"
        : "execute lane transition with measurement suspended";
    return command;
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::stopAcquisition(
        bool completed, const std::string& detail) {
    completionRequested_ = completed;
    terminalDetail_ = detail;
    if (!acquisitionRunning_) {
        return finalize(completed, detail);
    }
    state_ = State::WaitingForAcquisitionStop;
    Command command;
    command.kind = CommandKind::StopAcquisition;
    command.completed = completed;
    command.detail = detail;
    return command;
}

AdaptiveScanExecutor::Command AdaptiveScanExecutor::finalize(
        bool completed, const std::string& detail) {
    state_ = completed ? State::Completed : State::Failed;
    acquisitionRunning_ = false;
    completionRequested_ = completed;
    terminalDetail_ = detail;
    Command command;
    command.kind = CommandKind::Finalize;
    command.completed = completed;
    command.detail = detail;
    return command;
}

const char* adaptiveExecutorStateName(AdaptiveScanExecutor::State state) {
    switch (state) {
    case AdaptiveScanExecutor::State::Idle: return "IDLE";
    case AdaptiveScanExecutor::State::MovingToPlanStart:
        return "MOVING_TO_PLAN_START";
    case AdaptiveScanExecutor::State::WaitingForAcquisition:
        return "WAITING_FOR_ACQUISITION";
    case AdaptiveScanExecutor::State::ExecutingSegment:
        return "EXECUTING_SEGMENT";
    case AdaptiveScanExecutor::State::ExecutingTrajectory:
        return "EXECUTING_TRAJECTORY";
    case AdaptiveScanExecutor::State::WaitingForAcquisitionStop:
        return "WAITING_FOR_ACQUISITION_STOP";
    case AdaptiveScanExecutor::State::Completed: return "COMPLETED";
    case AdaptiveScanExecutor::State::Failed: return "FAILED";
    }
    return "UNKNOWN";
}

}  // namespace hik_adaptive
