#ifndef MYLINE_HIK_ADAPTIVE_SCAN_EXECUTOR_H
#define MYLINE_HIK_ADAPTIVE_SCAN_EXECUTOR_H

#include "AdaptiveScanPlanner.h"

#include <cstddef>
#include <string>

namespace hik_adaptive {

// Pure state machine for a frozen multi-segment plan. It owns no camera,
// robot, TTL or synchronization object; the GUI translates each command into
// the existing single shared hardware sessions.
class AdaptiveScanExecutor {
public:
    enum class State {
        Idle,
        MovingToPlanStart,
        WaitingForAcquisition,
        ExecutingSegment,
        ExecutingTrajectory,
        WaitingForAcquisitionStop,
        Completed,
        Failed
    };

    enum class CommandKind {
        None,
        MoveToPlanStart,
        StartAcquisition,
        ExecuteSegment,
        ExecuteTrajectory,
        StopAcquisition,
        Finalize
    };

    enum class MeasurementGate {
        Unchanged,
        Configure,
        Suspend
    };

    struct Command {
        CommandKind kind{CommandKind::None};
        MeasurementGate gate{MeasurementGate::Unchanged};
        std::size_t segmentIndex{0U};
        ScanSegment segment;
        std::vector<ScanSegment> trajectory;
        hik_scan::Pose6D target;
        bool completed{false};
        std::string detail;
    };

    bool load(const ScanPlan& plan, std::string* error = nullptr);
    Command begin();
    Command acquisitionStarted(bool success,
                               const std::string& detail = std::string());
    Command motionFinished(bool targetReached,
                           const std::string& detail = std::string());
    Command acquisitionStopped(bool confirmed,
                               const std::string& detail = std::string());
    Command requestStop(const std::string& reason);
    void reset();

    State state() const { return state_; }
    bool active() const;
    const ScanPlan& plan() const { return plan_; }
    std::size_t segmentIndex() const { return segmentIndex_; }
    bool usesPreSubmittedTrajectory() const {
        return preSubmittedTrajectory_;
    }

private:
    Command executeCurrentSegment();
    Command stopAcquisition(bool completed, const std::string& detail);
    Command finalize(bool completed, const std::string& detail);

    ScanPlan plan_;
    State state_{State::Idle};
    std::size_t segmentIndex_{0U};
    bool acquisitionRunning_{false};
    bool completionRequested_{false};
    bool preSubmittedTrajectory_{false};
    std::string terminalDetail_;
};

const char* adaptiveExecutorStateName(AdaptiveScanExecutor::State state);

}  // namespace hik_adaptive

#endif  // MYLINE_HIK_ADAPTIVE_SCAN_EXECUTOR_H
