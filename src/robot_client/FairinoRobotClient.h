#ifndef FAIRINO_ROBOT_CLIENT_H
#define FAIRINO_ROBOT_CLIENT_H

#include "../geometry/TransformUtils.h"
#include "../welding/WeldPathPlanner.h"

#include <functional>
#include <mutex>
#include <string>

class FRRobot;

namespace fairino_client {

struct RobotConfig {
    std::string ip;
    bool connectRobot;
    bool enableRobotMotion;
    bool autoEnable;
    bool autoMode;
};

RobotConfig defaultRobotConfig();
RobotConfig loadRobotConfig(const std::string& path);
std::string formatErrorCode(int code);

struct RobotStateSnapshot {
    int programState{0};
    int robotState{0};
    int robotMode{0};
    int mainCode{0};
    int subCode{0};
    int toolId{0};
    int userId{0};
    int emergencyStop{0};
    int safetyStop0{0};
    int safetyStop1{0};
    int collisionState{0};
    int motionDone{0};
    int motionQueueLength{0};
    int enableState{0};
};

std::string formatRobotState(const RobotStateSnapshot& state);

struct MotionTraceSample {
    double elapsedSec;
    weld_geometry::Pose6D tcpPose;
};

typedef std::function<void(const MotionTraceSample&)> MotionTraceCallback;

class FairinoRobotClient {
public:
    FairinoRobotClient();
    ~FairinoRobotClient();

    bool connectRobot(const RobotConfig& config);
    void disconnectRobot();
    bool isConnected() const;

    bool prepareForMotion(const RobotConfig& config);
    bool enableRobot(bool enabled);
    bool setAutoMode();
    bool stopMotion();
    bool pauseMotion();
    bool resumeMotion();
    bool resetAllError();
    bool getRobotMotionDone(bool* done);
    int waitRobotMotionDone(int timeoutMs = 120000,
                            int pollIntervalMs = 100,
                            const std::function<void()>& pollCallback = std::function<void()>());
    bool getCurrentFlangePose(weld_geometry::Pose6D* pose);
    bool getCurrentToolPose(weld_geometry::Pose6D* pose);
    bool getCurrentToolId(int* toolId);
    bool getCurrentUserId(int* userId);
    bool getRobotErrorCode(int* mainCode, int* subCode);
    bool getRobotStateSnapshot(RobotStateSnapshot* state);
    bool getToolCoord(int toolId, weld_geometry::Pose6D* pose);

    weld_geometry::Matrix4 getTBaseFlange(const weld_geometry::Pose6D& fallbackFlangePose);
    weld_geometry::Matrix4 getTFlangeTool(int toolId, const weld_motion::ToolConfig& fallbackToolConfig);
    weld_geometry::Matrix4 getTBaseTool(const weld_geometry::Pose6D& fallbackFlangePose, const weld_motion::ToolConfig& fallbackToolConfig);

    int moveL(const weld_geometry::Pose6D& target, int toolId, int userId,
              const weld_motion::WeldMotionConfig& motionConfig, double velocity);

private:
    FRRobot* robot_;
    bool connected_;
    std::mutex mutex_;
};

int executeLinearWeldMove(
    FairinoRobotClient& robot,
    const weld_geometry::Vec3& startCameraMm,
    const weld_geometry::Vec3& endCameraMm,
    const weld_geometry::Pose6D& flangePose,
    const weld_geometry::Pose6D& currentTcpPose,
    const weld_motion::HandEyeConfig& handEyeConfig,
    const weld_motion::ToolConfig& toolConfig,
    const weld_motion::WeldMotionConfig& motionConfig,
    const std::function<bool()>& shouldStop = std::function<bool()>(),
    const MotionTraceCallback& traceCallback = MotionTraceCallback());

int executeLinearWeldPlan(
    FairinoRobotClient& robot,
    const weld_motion::WeldLinePlan& plan,
    const weld_motion::ToolConfig& toolConfig,
    const weld_motion::WeldMotionConfig& motionConfig,
    const std::function<bool()>& shouldStop = std::function<bool()>(),
    const MotionTraceCallback& traceCallback = MotionTraceCallback());

} // namespace fairino_client

#endif // FAIRINO_ROBOT_CLIENT_H
