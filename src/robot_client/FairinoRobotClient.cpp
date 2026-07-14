#include "FairinoRobotClient.h"

#if defined(HAVE_FAIRINO_SDK)
#include "robot.h"
#endif

#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>

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

std::string formatErrorCode(int code) {
    switch (code) {
    case -32:
        return "-32 左右目拍照保存失败";
    case -31:
        return "-31 偏移方向计算失败";
    case -20:
        return "-20 用户停止";
    case -11:
        return "-11 运动参数无效";
    case -10:
        return "-10 安全检查未通过";
    case -5:
        return "-5 等待运动到位超时";
    case -4:
        return "-4 读取运动到位状态失败";
    case -3:
        return "-3 当前构建未链接 Fairino SDK";
    case -2:
        return "-2 机器人未连接";
    case 0:
        return "0 成功";
    case 3:
        return "3 参数个数不一致";
    case 4:
        return "4 参数值不在合理范围";
    case 14:
        return "14 指令执行失败";
    case 18:
        return "18 程序正在运行";
    case 25:
        return "25 计算失败";
    case 28:
        return "28 逆运动学计算失败";
    case 29:
        return "29 关节值超限";
    case 30:
        return "30 不可复位故障，请断电重启控制箱";
    case 34:
        return "34 工件号不对";
    case 38:
        return "38 奇异位姿";
    case 64:
        return "64 未加入指令队列";
    case 74:
        return "74 直线目标点错误";
    case 75:
        return "75 通道错误";
    case 76:
        return "76 等待超时";
    case 99:
        return "99 安全停止已触发";
    case 112:
        return "112 目标位姿无法到达";
    default:
        break;
    }
    std::ostringstream out;
    out << code << " 未知错误码";
    return out.str();
}

std::string formatRobotState(const RobotStateSnapshot& state) {
    auto programText = [](int value) -> const char* {
        switch (value) {
        case 1:
            return "停止";
        case 2:
            return "运行";
        case 3:
            return "暂停";
        default:
            return "未知";
        }
    };
    auto robotText = [](int value) -> const char* {
        switch (value) {
        case 1:
            return "停止";
        case 2:
            return "运行";
        case 3:
            return "暂停";
        case 4:
            return "拖动";
        default:
            return "未知";
        }
    };
    auto modeText = [](int value) -> const char* {
        switch (value) {
        case 0:
            return "自动";
        case 1:
            return "手动";
        default:
            return "未知";
        }
    };

    std::ostringstream out;
    out << "program=" << state.programState << "(" << programText(state.programState) << ")"
        << ", robot=" << state.robotState << "(" << robotText(state.robotState) << ")"
        << ", mode=" << state.robotMode << "(" << modeText(state.robotMode) << ")"
        << ", enable=" << state.enableState
        << ", tool=" << state.toolId
        << ", user=" << state.userId
        << ", main=" << state.mainCode
        << ", sub=" << state.subCode
        << ", EStop=" << state.emergencyStop
        << ", SI0=" << state.safetyStop0
        << ", SI1=" << state.safetyStop1
        << ", collision=" << state.collisionState
        << ", motionDone=" << state.motionDone
        << ", queue=" << state.motionQueueLength;
    return out.str();
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
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->RPC(config.ip.c_str());
    }
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
        std::lock_guard<std::mutex> lock(mutex_);
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
        std::lock_guard<std::mutex> lock(mutex_);
        const errno_t err = robot_->RobotEnable(1);
        if (err != 0) {
            std::cerr << "RobotEnable(1) failed, err=" << err << std::endl;
            return false;
        }
        std::cout << "RobotEnable(1) sent." << std::endl;
    }
    if (config.autoMode) {
        std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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
    std::lock_guard<std::mutex> lock(mutex_);
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

bool FairinoRobotClient::getRobotMotionDone(bool* done) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !done) {
        return false;
    }
    uint8_t state = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const errno_t err = robot_->GetRobotMotionDone(&state);
        if (err != 0) {
            std::cerr << "GetRobotMotionDone failed, err=" << err << std::endl;
            return false;
        }
    }
    *done = (state != 0);
    return true;
#else
    (void)done;
    return false;
#endif
}

int FairinoRobotClient::waitRobotMotionDone(int timeoutMs,
                                            int pollIntervalMs,
                                            const std::function<void()>& pollCallback) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_) {
        return -2;
    }
    const int loops = timeoutMs <= 0 ? 1 : (timeoutMs + pollIntervalMs - 1) / pollIntervalMs;
    for (int i = 0; i < loops; ++i) {
        if (pollCallback) {
            pollCallback();
        }
        bool done = false;
        if (!getRobotMotionDone(&done)) {
            return -4;
        }
        if (done) {
            if (pollCallback) {
                pollCallback();
            }
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }
    std::cerr << "waitRobotMotionDone timeout." << std::endl;
    return -5;
#else
    (void)timeoutMs;
    (void)pollIntervalMs;
    (void)pollCallback;
    return -3;
#endif
}

bool FairinoRobotClient::getCurrentFlangePose(weld_geometry::Pose6D* pose) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !pose) {
        return false;
    }
    DescPose desc;
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetActualToolFlangePose(0, &desc);
    }
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
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetActualTCPPose(0, &desc);
    }
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

bool FairinoRobotClient::getCurrentToolId(int* toolId) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !toolId) {
        return false;
    }
    int id = 0;
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetActualTCPNum(0, &id);
    }
    if (err != 0) {
        std::cerr << "GetActualTCPNum failed, err=" << err << std::endl;
        return false;
    }
    *toolId = id;
    return true;
#else
    (void)toolId;
    return false;
#endif
}

bool FairinoRobotClient::getCurrentUserId(int* userId) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !userId) {
        return false;
    }
    int id = 0;
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetActualWObjNum(0, &id);
    }
    if (err != 0) {
        std::cerr << "GetActualWObjNum failed, err=" << err << std::endl;
        return false;
    }
    *userId = id;
    return true;
#else
    (void)userId;
    return false;
#endif
}

bool FairinoRobotClient::getRobotErrorCode(int* mainCode, int* subCode) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !mainCode || !subCode) {
        return false;
    }
    int main = 0;
    int sub = 0;
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetRobotErrorCode(&main, &sub);
    }
    if (err != 0) {
        std::cerr << "GetRobotErrorCode failed, err=" << err << std::endl;
        return false;
    }
    *mainCode = main;
    *subCode = sub;
    return true;
#else
    (void)mainCode;
    (void)subCode;
    return false;
#endif
}

bool FairinoRobotClient::getRobotStateSnapshot(RobotStateSnapshot* state) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !state) {
        return false;
    }
    ROBOT_STATE_PKG pkg{};
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetRobotRealTimeState(&pkg);
    }
    if (err != 0) {
        std::cerr << "GetRobotRealTimeState failed, err=" << err << std::endl;
        return false;
    }
    state->programState = static_cast<int>(pkg.program_state);
    state->robotState = static_cast<int>(pkg.robot_state);
    state->robotMode = static_cast<int>(pkg.robot_mode);
    state->mainCode = pkg.main_code;
    state->subCode = pkg.sub_code;
    state->toolId = pkg.tool;
    state->userId = pkg.user;
    state->emergencyStop = static_cast<int>(pkg.EmergencyStop);
    state->safetyStop0 = static_cast<int>(pkg.safety_stop0_state);
    state->safetyStop1 = static_cast<int>(pkg.safety_stop1_state);
    state->collisionState = static_cast<int>(pkg.collisionState);
    state->motionDone = pkg.motion_done;
    state->motionQueueLength = pkg.mc_queue_len;
    state->enableState = pkg.rbtEnableState;
    return true;
#else
    (void)state;
    return false;
#endif
}

bool FairinoRobotClient::getToolCoord(int toolId, weld_geometry::Pose6D* pose) {
#if defined(HAVE_FAIRINO_SDK)
    if (!connected_ || !pose) {
        return false;
    }
    DescPose desc;
    errno_t err = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        err = robot_->GetToolCoordWithID(toolId, desc);
    }
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

int FairinoRobotClient::moveL(const weld_geometry::Pose6D& target, int toolId, int userId,
                              const weld_motion::WeldMotionConfig& motionConfig, double velocity) {
    const double effectiveBlendR = motionConfig.blendR < 0.0 ? 0.0 : motionConfig.blendR;
    const double sentOvl = motionConfig.physicalSpeedMode ? velocity * motionConfig.ovl / 100.0 : motionConfig.ovl;
    const double sentVel = motionConfig.physicalSpeedMode ? 100.0 : velocity;
    const double sentAcc = motionConfig.physicalSpeedMode ? 100.0 : motionConfig.acc;
    const double sentOacc = motionConfig.physicalSpeedMode ? motionConfig.acc : motionConfig.ovl;
    std::cout << "MoveL target TCP: " << weld_geometry::formatPose(target)
              << ", tool=" << toolId << ", user=" << userId
              << ", vel=" << velocity << (motionConfig.physicalSpeedMode ? "mm/s" : "%")
              << ", acc=" << motionConfig.acc << (motionConfig.physicalSpeedMode ? "mm/s2" : "%")
              << ", ovl=" << motionConfig.ovl << "%"
              << ", sentVel=" << sentVel << "%"
              << ", sentAcc=" << sentAcc << "%"
              << ", sentOvl=" << sentOvl << (motionConfig.physicalSpeedMode ? "mm/s" : "%")
              << ", sentOacc=" << sentOacc << (motionConfig.physicalSpeedMode ? "mm/s2" : "%")
              << ", blendR=" << effectiveBlendR << std::endl;

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
    if (velocity <= 0.0 || motionConfig.acc <= 0.0 || motionConfig.ovl <= 0.0 || sentVel <= 0.0 || sentOvl <= 0.0) {
        std::cerr << "MoveL speed parameters must be positive." << std::endl;
        return -11;
    }
    DescPose desc = toDescPose(target);
    ExaxisPos epos;
    DescPose offset;
    JointPos currentJoint;
    JointPos targetJoint;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        errno_t err = robot_->GetActualJointPosDegree(0, &currentJoint);
        if (err != 0) {
            std::cerr << "GetActualJointPosDegree failed, err=" << err << std::endl;
            return err;
        }
        err = robot_->GetInverseKinRef(0, &desc, &currentJoint, &targetJoint);
        if (err != 0) {
            std::cerr << "GetInverseKinRef failed for MoveL target, err=" << err << std::endl;
            return err;
        }
        return robot_->MoveL(&targetJoint, &desc, toolId, userId,
                             static_cast<float>(sentVel),
                             static_cast<float>(sentAcc),
                             static_cast<float>(sentOvl),
                             static_cast<float>(effectiveBlendR),
                             0, &epos, 0, 0, &offset,
                             static_cast<float>(sentOacc),
                             motionConfig.physicalSpeedMode ? 1 : 0);
    }
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
    const weld_motion::WeldMotionConfig& motionConfig,
    const std::function<bool()>& shouldStop,
    const MotionTraceCallback& traceCallback) {
    const weld_motion::WeldLinePlan plan = weld_motion::planLinearWeldMove(
        startCameraMm, endCameraMm, flangePose, currentTcpPose, handEyeConfig, motionConfig);
    return executeLinearWeldPlan(robot, plan, toolConfig, motionConfig, shouldStop, traceCallback);
}

int executeLinearWeldPlan(
    FairinoRobotClient& robot,
    const weld_motion::WeldLinePlan& plan,
    const weld_motion::ToolConfig& toolConfig,
    const weld_motion::WeldMotionConfig& motionConfig,
    const std::function<bool()>& shouldStop,
    const MotionTraceCallback& traceCallback) {
    weld_motion::printWeldLinePlan(plan);

    std::string reason;
    if (!weld_motion::isSafePose(plan.approachTcpTarget, &reason) ||
        !weld_motion::isSafePose(plan.startTcpTarget, &reason) ||
        !weld_motion::isSafePose(plan.endTcpTarget, &reason) ||
        !weld_motion::isSafePose(plan.retractTcpTarget, &reason)) {
        std::cerr << "Linear weld move blocked by safety check: " << reason << std::endl;
        return -10;
    }

    const bool realMotion = !motionConfig.dryRun && motionConfig.enableRobotMotion;
    const std::chrono::steady_clock::time_point traceStart = std::chrono::steady_clock::now();
    auto stopRequested = [&shouldStop]() {
        return shouldStop && shouldStop();
    };
    auto sampleTrace = [&]() {
        if (!realMotion || !traceCallback) {
            return;
        }
        weld_geometry::Pose6D tcpPose;
        if (!robot.getCurrentToolPose(&tcpPose)) {
            return;
        }
        const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        const double elapsedSec = std::chrono::duration_cast<std::chrono::duration<double>>(now - traceStart).count();
        MotionTraceSample sample;
        sample.elapsedSec = elapsedSec;
        sample.tcpPose = tcpPose;
        traceCallback(sample);
    };
    auto runSegment = [&](const char* name, const weld_geometry::Pose6D& target, double speed) {
        if (stopRequested()) {
            std::cerr << "MoveL sequence canceled before segment " << name << "." << std::endl;
            return -20;
        }
        std::cout << "MoveL segment: " << name << std::endl;
        const int err = robot.moveL(target, toolConfig.toolId, toolConfig.userId,
                                    motionConfig, speed);
        if (err != 0) {
            std::cerr << "MoveL segment " << name << " failed, err=" << err << std::endl;
            return err;
        }
        if (realMotion) {
            const int waitErr = robot.waitRobotMotionDone(120000, 100, sampleTrace);
            if (waitErr != 0) {
                std::cerr << "MoveL segment " << name << " wait failed, err=" << waitErr << std::endl;
                return waitErr;
            }
            if (stopRequested()) {
                std::cerr << "MoveL sequence canceled after segment " << name << "." << std::endl;
                return -20;
            }
        }
        return 0;
    };

    sampleTrace();
    int err = runSegment("Approach", plan.approachTcpTarget, motionConfig.travelVel);
    if (err != 0) {
        return err;
    }
    err = runSegment("Start", plan.startTcpTarget, motionConfig.travelVel);
    if (err != 0) {
        return err;
    }
    err = runSegment("Weld", plan.endTcpTarget, motionConfig.weldVel);
    if (err != 0) {
        return err;
    }
    err = runSegment("Retract", plan.retractTcpTarget, motionConfig.travelVel);
    if (err != 0) {
        return err;
    }
    return 0;
}

} // namespace fairino_client
