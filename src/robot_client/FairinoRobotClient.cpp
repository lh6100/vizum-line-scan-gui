#include "FairinoRobotClient.h"

#if defined(HAVE_FAIRINO_SDK)
#include "robot.h"
#endif

#include <fstream>
#include <iostream>
#include <sstream>

namespace fairino_client {

namespace {

std::string trim(const std::string& text) {
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

bool parseScalar(const std::string& path, const std::string& key, std::string* value) {
    std::ifstream input(path.c_str());
    if (!input.good()) {
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        const size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line = trim(line);
        const size_t sep = line.find_first_of(":=");
        if (sep == std::string::npos) {
            continue;
        }
        if (trim(line.substr(0, sep)) == key) {
            *value = trim(line.substr(sep + 1));
            return true;
        }
    }
    return false;
}

bool parseBool(const std::string& path, const std::string& key, bool fallback) {
    std::string value;
    if (!parseScalar(path, key, &value)) {
        return fallback;
    }
    return value == "true" || value == "1" || value == "yes" || value == "on";
}

#if defined(HAVE_FAIRINO_SDK)
weld_geometry::Pose6D fromDescPose(const DescPose& pose) {
    return {pose.tran.x, pose.tran.y, pose.tran.z, pose.rpy.rx, pose.rpy.ry, pose.rpy.rz};
}

DescPose toDescPose(const weld_geometry::Pose6D& pose) {
    return DescPose(pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
}
#endif

} // namespace

RobotConfig defaultRobotConfig() {
    RobotConfig config;
    config.ip = "192.168.1.200";
    config.connectRobot = false;
    config.enableRobotMotion = false;
    config.autoEnable = false;
    config.autoMode = false;
    return config;
}

RobotConfig loadRobotConfig(const std::string& path) {
    RobotConfig config = defaultRobotConfig();
    std::string ip;
    if (parseScalar(path, "robot_ip", &ip)) {
        config.ip = ip;
    }
    config.connectRobot = parseBool(path, "connect_robot", config.connectRobot);
    config.enableRobotMotion = parseBool(path, "enable_robot_motion", config.enableRobotMotion);
    config.autoEnable = parseBool(path, "auto_enable", config.autoEnable);
    config.autoMode = parseBool(path, "auto_mode", config.autoMode);
    return config;
}

FairinoRobotClient::FairinoRobotClient()
    : robot_(0),
      connected_(false) {
#if defined(HAVE_FAIRINO_SDK)
    robot_ = new FRRobot();
#endif
}

FairinoRobotClient::~FairinoRobotClient() {
    disconnectRobot();
#if defined(HAVE_FAIRINO_SDK)
    delete robot_;
#endif
}

bool FairinoRobotClient::connectRobot(const RobotConfig& config) {
    if (!config.connectRobot && !config.enableRobotMotion) {
        std::cout << "Robot connection disabled; Fairino client stays offline." << std::endl;
        return false;
    }
#if defined(HAVE_FAIRINO_SDK)
    const errno_t err = robot_->RPC(config.ip.c_str());
    connected_ = (err == 0);
    if (!connected_) {
        std::cerr << "FRRobot::RPC failed, err=" << err << std::endl;
        return false;
    }
    std::cout << "Connected to Fairino robot at " << config.ip << std::endl;
    return prepareForMotion(config);
#else
    std::cerr << "Fairino SDK is not linked in this build." << std::endl;
    return false;
#endif
}

void FairinoRobotClient::disconnectRobot() {
#if defined(HAVE_FAIRINO_SDK)
    if (connected_ && robot_) {
        robot_->CloseRPC();
    }
#endif
    connected_ = false;
}

bool FairinoRobotClient::isConnected() const {
    return connected_;
}

bool FairinoRobotClient::prepareForMotion(const RobotConfig& config) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        return false;
    }
    if (config.autoEnable) {
        const errno_t err = robot_->RobotEnable(1);
        if (err != 0) {
            std::cerr << "RobotEnable(1) failed, err=" << err << std::endl;
            return false;
        }
        std::cout << "RobotEnable(1) sent." << std::endl;
    }
    if (config.autoMode) {
        const errno_t err = robot_->Mode(0);
        if (err != 0) {
            std::cerr << "Mode(0) failed, err=" << err << std::endl;
            return false;
        }
        std::cout << "Mode(0) auto mode sent." << std::endl;
    }
    return true;
#else
    (void)config;
    return false;
#endif
}

bool FairinoRobotClient::enableRobot(bool enabled) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return false;
    }
    const errno_t err = robot_->RobotEnable(enabled ? 1 : 0);
    if (err != 0) {
        std::cerr << "RobotEnable(" << (enabled ? 1 : 0) << ") failed, err=" << err << std::endl;
        return false;
    }
    return true;
#else
    (void)enabled;
    return false;
#endif
}

bool FairinoRobotClient::setAutoMode() {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return false;
    }
    const errno_t err = robot_->Mode(0);
    if (err != 0) {
        std::cerr << "Mode(0) failed, err=" << err << std::endl;
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool FairinoRobotClient::stopMotion() {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return false;
    }
    const errno_t err = robot_->StopMotion();
    if (err != 0) {
        std::cerr << "StopMotion failed, err=" << err << std::endl;
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool FairinoRobotClient::pauseMotion() {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return false;
    }
    const errno_t err = robot_->PauseMotion();
    if (err != 0) {
        std::cerr << "PauseMotion failed, err=" << err << std::endl;
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool FairinoRobotClient::resumeMotion() {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return false;
    }
    const errno_t err = robot_->ResumeMotion();
    if (err != 0) {
        std::cerr << "ResumeMotion failed, err=" << err << std::endl;
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool FairinoRobotClient::resetAllError() {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return false;
    }
    const errno_t err = robot_->ResetAllError();
    if (err != 0) {
        std::cerr << "ResetAllError failed, err=" << err << std::endl;
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool FairinoRobotClient::getCurrentFlangePose(weld_geometry::Pose6D* pose) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !pose) {
        return false;
    }
    DescPose desc;
    const errno_t err = robot_->GetActualToolFlangePose(0, &desc);
    if (err != 0) {
        std::cerr << "GetActualToolFlangePose failed, err=" << err << std::endl;
        return false;
    }
    *pose = fromDescPose(desc);
    return true;
#else
    (void)pose;
    return false;
#endif
}

bool FairinoRobotClient::getCurrentToolPose(weld_geometry::Pose6D* pose) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !pose) {
        return false;
    }
    DescPose desc;
    const errno_t err = robot_->GetActualTCPPose(0, &desc);
    if (err != 0) {
        std::cerr << "GetActualTCPPose failed, err=" << err << std::endl;
        return false;
    }
    *pose = fromDescPose(desc);
    return true;
#else
    (void)pose;
    return false;
#endif
}

bool FairinoRobotClient::getToolCoord(int toolId, weld_geometry::Pose6D* pose) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !pose) {
        return false;
    }
    DescPose desc;
    const errno_t err = robot_->GetToolCoordWithID(toolId, desc);
    if (err != 0) {
        std::cerr << "GetToolCoordWithID(" << toolId << ") failed, err=" << err << std::endl;
        return false;
    }
    *pose = fromDescPose(desc);
    return true;
#else
    (void)toolId;
    (void)pose;
    return false;
#endif
}

weld_geometry::Matrix4 FairinoRobotClient::getTBaseFlange(const weld_geometry::Pose6D& fallbackFlangePose) {
    weld_geometry::Pose6D pose = fallbackFlangePose;
    getCurrentFlangePose(&pose);
    return weld_geometry::poseToMatrix(pose);
}

weld_geometry::Matrix4 FairinoRobotClient::getTFlangeTool(int toolId, const weld_motion::ToolConfig& fallbackToolConfig) {
    weld_geometry::Pose6D pose = fallbackToolConfig.flangeToTool;
    getToolCoord(toolId, &pose);
    return weld_geometry::poseToMatrix(pose);
}

weld_geometry::Matrix4 FairinoRobotClient::getTBaseTool(const weld_geometry::Pose6D& fallbackFlangePose, const weld_motion::ToolConfig& fallbackToolConfig) {
    return weld_geometry::multiply(getTBaseFlange(fallbackFlangePose), getTFlangeTool(fallbackToolConfig.toolId, fallbackToolConfig));
}

int FairinoRobotClient::moveL(const weld_geometry::Pose6D& target, int toolId, int userId, const weld_motion::WeldMotionConfig& motionConfig) {
    std::cout << "MoveL target TCP: " << weld_geometry::formatPose(target)
              << ", tool=" << toolId << ", user=" << userId
              << ", vel=" << motionConfig.vel << ", acc=" << motionConfig.acc
              << ", ovl=" << motionConfig.ovl << std::endl;

    if (motionConfig.dryRun || !motionConfig.enableRobotMotion) {
        std::cout << "Dry-run: no robot motion sent." << std::endl;
        return 0;
    }
    std::string reason;
    if (!weld_motion::isSafePose(target, &reason)) {
        std::cerr << "Unsafe target blocked: " << reason << std::endl;
        return -1;
    }
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        std::cerr << "Robot is not connected." << std::endl;
        return -2;
    }
    DescPose desc = toDescPose(target);
    ExaxisPos epos;
    DescPose offset;
    return robot_->MoveL(&desc, toolId, userId,
                         static_cast<float>(motionConfig.vel),
                         static_cast<float>(motionConfig.acc),
                         static_cast<float>(motionConfig.ovl),
                         static_cast<float>(motionConfig.blendR),
                         0, &epos, 0, 0, &offset);
#else
    std::cerr << "Fairino SDK is not linked in this build." << std::endl;
    return -3;
#endif
}

int executeLinearWeldMove(
    FairinoRobotClient& robot,
    const weld_geometry::Vec3& startCameraMm,
    const weld_geometry::Vec3& endCameraMm,
    const weld_geometry::Pose6D& flangePose,
    const weld_geometry::Pose6D& currentTcpPose,
    const weld_motion::HandEyeConfig& handEyeConfig,
    const weld_motion::ToolConfig& toolConfig,
    const weld_motion::WeldMotionConfig& motionConfig) {
    const weld_motion::WeldLinePlan plan = weld_motion::planLinearWeldMove(
        startCameraMm, endCameraMm, flangePose, currentTcpPose, handEyeConfig, motionConfig);
    weld_motion::printWeldLinePlan(plan);

    std::string reason;
    if (!weld_motion::isSafePose(plan.approachTcpTarget, &reason) ||
        !weld_motion::isSafePose(plan.startTcpTarget, &reason) ||
        !weld_motion::isSafePose(plan.endTcpTarget, &reason)) {
        std::cerr << "Linear weld move blocked by safety check: " << reason << std::endl;
        return -10;
    }

    int err = robot.moveL(plan.approachTcpTarget, toolConfig.toolId, toolConfig.userId, motionConfig);
    if (err != 0) {
        return err;
    }
    err = robot.moveL(plan.startTcpTarget, toolConfig.toolId, toolConfig.userId, motionConfig);
    if (err != 0) {
        return err;
    }
    return robot.moveL(plan.endTcpTarget, toolConfig.toolId, toolConfig.userId, motionConfig);
}

} // namespace fairino_client
