#ifndef WELD_PATH_PLANNER_H
#define WELD_PATH_PLANNER_H

#include "../geometry/TransformUtils.h"

#include <string>

namespace weld_motion {

struct HandEyeConfig {
    weld_geometry::Matrix4 matrix;
    std::string mode;
};

struct ToolConfig {
    int toolId;
    int userId;
    weld_geometry::Pose6D flangeToTool;
};

struct ProcessOffset {
    double x;
    double y;
    double z;
};

struct WeldMotionConfig {
    bool dryRun;
    bool enableRobotMotion;
    bool enableArc;
    bool physicalSpeedMode;
    double safeHeightMm;
    double retractHeightMm;
    double travelVel;
    double weldVel;
    double acc;
    double ovl;
    double blendR;
    ProcessOffset processOffset;
};

struct WeldLinePlan {
    weld_geometry::Vec3 startBase;
    weld_geometry::Vec3 endBase;
    weld_geometry::Pose6D startTcpTarget;
    weld_geometry::Pose6D approachTcpTarget;
    weld_geometry::Pose6D endTcpTarget;
    weld_geometry::Pose6D retractTcpTarget;
    double lineLengthMm;
};

HandEyeConfig defaultHandEyeConfig();
ToolConfig defaultToolConfig();
WeldMotionConfig defaultWeldMotionConfig();

HandEyeConfig loadHandEyeConfig(const std::string& path);
ToolConfig loadToolConfig(const std::string& path);
WeldMotionConfig loadWeldMotionConfig(const std::string& path);

weld_geometry::Matrix4 getTFlangeCamera(const HandEyeConfig& config);
weld_geometry::Matrix4 getTFlangeTool(const ToolConfig& config);
weld_geometry::Matrix4 getTBaseFlange(const weld_geometry::Pose6D& flangePose);
weld_geometry::Matrix4 getTBaseTool(const weld_geometry::Pose6D& flangePose, const ToolConfig& toolConfig);

weld_geometry::Vec3 cameraPointToWeldPointBase(
    const weld_geometry::Vec3& cameraPointMm,
    const weld_geometry::Pose6D& flangePose,
    const HandEyeConfig& handEyeConfig);

weld_geometry::Pose6D weldPointToTcpTarget(
    const weld_geometry::Vec3& weldPointBase,
    const ProcessOffset& processOffset,
    const weld_geometry::Pose6D& targetOrientation);

WeldLinePlan planLinearWeldMove(
    const weld_geometry::Vec3& startCameraMm,
    const weld_geometry::Vec3& endCameraMm,
    const weld_geometry::Pose6D& flangePose,
    const weld_geometry::Pose6D& currentTcpPose,
    const HandEyeConfig& handEyeConfig,
    const WeldMotionConfig& motionConfig);

bool isSafePose(const weld_geometry::Pose6D& pose, std::string* reason);
void printWeldLinePlan(const WeldLinePlan& plan);

} // namespace weld_motion

#endif // WELD_PATH_PLANNER_H
