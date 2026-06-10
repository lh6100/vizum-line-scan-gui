#include "WeldPathPlanner.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace weld_motion {

namespace {

std::string trim(const std::string& text) {
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::string stripComment(const std::string& line) {
    const size_t pos = line.find('#');
    return pos == std::string::npos ? line : line.substr(0, pos);
}

bool parseScalar(const std::string& path, const std::string& key, std::string* value) {
    std::ifstream input(path.c_str());
    if (!input.good()) {
        return false;
    }

    std::string line;
    while (std::getline(input, line)) {
        line = trim(stripComment(line));
        if (line.empty()) {
            continue;
        }
        const size_t sep = line.find_first_of(":=");
        if (sep == std::string::npos) {
            continue;
        }
        const std::string k = trim(line.substr(0, sep));
        if (k == key) {
            *value = trim(line.substr(sep + 1));
            return true;
        }
    }
    return false;
}

double parseDoubleValue(const std::string& path, const std::string& key, double fallback) {
    std::string value;
    if (parseScalar(path, key, &value)) {
        std::istringstream in(value);
        double parsed = fallback;
        if (in >> parsed) {
            return parsed;
        }
    }
    return fallback;
}

int parseIntValue(const std::string& path, const std::string& key, int fallback) {
    return static_cast<int>(parseDoubleValue(path, key, static_cast<double>(fallback)));
}

bool parseBoolValue(const std::string& path, const std::string& key, bool fallback) {
    std::string value;
    if (!parseScalar(path, key, &value)) {
        return fallback;
    }
    return value == "true" || value == "1" || value == "yes" || value == "on";
}

weld_geometry::Matrix4 defaultHandEyeMatrix() {
    weld_geometry::Matrix4 matrix;
    matrix.m = {{-0.98,  0.00, -0.21, -114.12,
                 -0.03, -0.99,  0.10,  -44.02,
                 -0.21,  0.10,  0.97,  125.38,
                  0.00,  0.00,  0.00,    1.00}};
    return matrix;
}

} // namespace

HandEyeConfig defaultHandEyeConfig() {
    HandEyeConfig config;
    config.matrix = defaultHandEyeMatrix();
    config.mode = "camera_to_flange";
    return config;
}

ToolConfig defaultToolConfig() {
    ToolConfig config;
    config.toolId = 3;
    config.userId = 0;
    config.flangeToTool = {0.326, -194.553, 368.943, -169.517, 8.355, 1.904};
    return config;
}

WeldMotionConfig defaultWeldMotionConfig() {
    WeldMotionConfig config;
    config.dryRun = true;
    config.enableRobotMotion = false;
    config.enableArc = false;
    config.safeHeightMm = 50.0;
    config.vel = 5.0;
    config.acc = 20.0;
    config.ovl = 10.0;
    config.blendR = -1.0;
    config.processOffset = {0.0, 0.0, 0.0};
    return config;
}

HandEyeConfig loadHandEyeConfig(const std::string& path) {
    HandEyeConfig config = defaultHandEyeConfig();
    std::string mode;
    if (parseScalar(path, "mode", &mode)) {
        config.mode = mode;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            std::ostringstream key;
            key << "m" << row << col;
            config.matrix.m[static_cast<size_t>(row * 4 + col)] =
                parseDoubleValue(path, key.str(), config.matrix.m[static_cast<size_t>(row * 4 + col)]);
        }
    }
    return config;
}

ToolConfig loadToolConfig(const std::string& path) {
    ToolConfig config = defaultToolConfig();
    config.toolId = parseIntValue(path, "tool_id", config.toolId);
    config.userId = parseIntValue(path, "user_id", config.userId);
    config.flangeToTool.x = parseDoubleValue(path, "x", config.flangeToTool.x);
    config.flangeToTool.y = parseDoubleValue(path, "y", config.flangeToTool.y);
    config.flangeToTool.z = parseDoubleValue(path, "z", config.flangeToTool.z);
    config.flangeToTool.rx = parseDoubleValue(path, "rx", config.flangeToTool.rx);
    config.flangeToTool.ry = parseDoubleValue(path, "ry", config.flangeToTool.ry);
    config.flangeToTool.rz = parseDoubleValue(path, "rz", config.flangeToTool.rz);
    return config;
}

WeldMotionConfig loadWeldMotionConfig(const std::string& path) {
    WeldMotionConfig config = defaultWeldMotionConfig();
    config.dryRun = parseBoolValue(path, "dry_run", config.dryRun);
    config.enableRobotMotion = parseBoolValue(path, "enable_robot_motion", config.enableRobotMotion);
    config.enableArc = parseBoolValue(path, "enable_arc", config.enableArc);
    config.safeHeightMm = parseDoubleValue(path, "safe_height_mm", config.safeHeightMm);
    config.vel = parseDoubleValue(path, "vel", config.vel);
    config.acc = parseDoubleValue(path, "acc", config.acc);
    config.ovl = parseDoubleValue(path, "ovl", config.ovl);
    config.blendR = parseDoubleValue(path, "blend_r", config.blendR);
    config.processOffset.x = parseDoubleValue(path, "process_offset_x", config.processOffset.x);
    config.processOffset.y = parseDoubleValue(path, "process_offset_y", config.processOffset.y);
    config.processOffset.z = parseDoubleValue(path, "process_offset_z", config.processOffset.z);
    return config;
}

weld_geometry::Matrix4 getTFlangeCamera(const HandEyeConfig& config) {
    if (config.mode == "flange_to_camera") {
        return weld_geometry::inverseRigid(config.matrix);
    }
    return config.matrix;
}

weld_geometry::Matrix4 getTFlangeTool(const ToolConfig& config) {
    return weld_geometry::poseToMatrix(config.flangeToTool);
}

weld_geometry::Matrix4 getTBaseFlange(const weld_geometry::Pose6D& flangePose) {
    return weld_geometry::poseToMatrix(flangePose);
}

weld_geometry::Matrix4 getTBaseTool(const weld_geometry::Pose6D& flangePose, const ToolConfig& toolConfig) {
    return weld_geometry::multiply(getTBaseFlange(flangePose), getTFlangeTool(toolConfig));
}

weld_geometry::Vec3 cameraPointToWeldPointBase(
    const weld_geometry::Vec3& cameraPointMm,
    const weld_geometry::Pose6D& flangePose,
    const HandEyeConfig& handEyeConfig) {
    const weld_geometry::Matrix4 tBaseFlange = getTBaseFlange(flangePose);
    const weld_geometry::Matrix4 tFlangeCamera = getTFlangeCamera(handEyeConfig);
    const weld_geometry::Matrix4 tBaseCamera = weld_geometry::multiply(tBaseFlange, tFlangeCamera);
    return weld_geometry::transformPoint(tBaseCamera, cameraPointMm);
}

weld_geometry::Pose6D weldPointToTcpTarget(
    const weld_geometry::Vec3& weldPointBase,
    const ProcessOffset& processOffset,
    const weld_geometry::Pose6D& targetOrientation) {
    weld_geometry::Pose6D pose = targetOrientation;
    pose.x = weldPointBase.x + processOffset.x;
    pose.y = weldPointBase.y + processOffset.y;
    pose.z = weldPointBase.z + processOffset.z;
    return pose;
}

WeldLinePlan planLinearWeldMove(
    const weld_geometry::Vec3& startCameraMm,
    const weld_geometry::Vec3& endCameraMm,
    const weld_geometry::Pose6D& flangePose,
    const weld_geometry::Pose6D& currentTcpPose,
    const HandEyeConfig& handEyeConfig,
    const WeldMotionConfig& motionConfig) {
    WeldLinePlan plan;
    plan.startBase = cameraPointToWeldPointBase(startCameraMm, flangePose, handEyeConfig);
    plan.endBase = cameraPointToWeldPointBase(endCameraMm, flangePose, handEyeConfig);
    plan.startTcpTarget = weldPointToTcpTarget(plan.startBase, motionConfig.processOffset, currentTcpPose);
    plan.endTcpTarget = weldPointToTcpTarget(plan.endBase, motionConfig.processOffset, currentTcpPose);
    plan.approachTcpTarget = plan.startTcpTarget;
    plan.approachTcpTarget.z += motionConfig.safeHeightMm;
    plan.lineLengthMm = weld_geometry::distance(plan.startBase, plan.endBase);
    return plan;
}

bool isSafePose(const weld_geometry::Pose6D& pose, std::string* reason) {
    const weld_geometry::Vec3 point = {pose.x, pose.y, pose.z};
    if (!weld_geometry::isFinite(point)) {
        if (reason) *reason = "目标点包含 NaN 或 inf";
        return false;
    }
    if (std::fabs(pose.x) > 2000.0 || std::fabs(pose.y) > 2000.0 || pose.z < -1000.0 || pose.z > 2000.0) {
        if (reason) *reason = "目标点超出默认软件安全范围";
        return false;
    }
    if (!std::isfinite(pose.rx) || !std::isfinite(pose.ry) || !std::isfinite(pose.rz)) {
        if (reason) *reason = "目标姿态包含 NaN 或 inf";
        return false;
    }
    return true;
}

void printWeldLinePlan(const WeldLinePlan& plan) {
    std::cout << "Start_base(mm): " << weld_geometry::formatVec3(plan.startBase) << std::endl;
    std::cout << "End_base(mm):   " << weld_geometry::formatVec3(plan.endBase) << std::endl;
    std::cout << "Line length(mm): " << plan.lineLengthMm << std::endl;
    std::cout << "Approach TCP: " << weld_geometry::formatPose(plan.approachTcpTarget) << std::endl;
    std::cout << "Start TCP:    " << weld_geometry::formatPose(plan.startTcpTarget) << std::endl;
    std::cout << "End TCP:      " << weld_geometry::formatPose(plan.endTcpTarget) << std::endl;
}

} // namespace weld_motion
