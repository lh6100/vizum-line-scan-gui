#ifndef FAIRINO_ROBOT_CLIENT_H
#define FAIRINO_ROBOT_CLIENT_H

#include "../geometry/TransformUtils.h"
#include "../welding/WeldPathPlanner.h"

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
    bool getCurrentFlangePose(weld_geometry::Pose6D* pose);
    bool getCurrentToolPose(weld_geometry::Pose6D* pose);
    bool getToolCoord(int toolId, weld_geometry::Pose6D* pose);

    weld_geometry::Matrix4 getTBaseFlange(const weld_geometry::Pose6D& fallbackFlangePose);
    weld_geometry::Matrix4 getTFlangeTool(int toolId, const weld_motion::ToolConfig& fallbackToolConfig);
    weld_geometry::Matrix4 getTBaseTool(const weld_geometry::Pose6D& fallbackFlangePose, const weld_motion::ToolConfig& fallbackToolConfig);

    int moveL(const weld_geometry::Pose6D& target, int toolId, int userId, const weld_motion::WeldMotionConfig& motionConfig);

private:
    FRRobot* robot_;
    bool connected_;
};

int executeLinearWeldMove(
    FairinoRobotClient& robot,
    const weld_geometry::Vec3& startCameraMm,
    const weld_geometry::Vec3& endCameraMm,
    const weld_geometry::Pose6D& flangePose,
    const weld_geometry::Pose6D& currentTcpPose,
    const weld_motion::HandEyeConfig& handEyeConfig,
    const weld_motion::ToolConfig& toolConfig,
    const weld_motion::WeldMotionConfig& motionConfig);

} // namespace fairino_client

#endif // FAIRINO_ROBOT_CLIENT_H
