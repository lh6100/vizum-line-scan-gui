#include "../src/robot_client/FairinoRobotClient.h"
#include "../src/welding/WeldPathPlanner.h"

#include <cstdlib>
#include <iostream>

namespace {

void printUsage(const char* program) {
    std::cout << "Usage:\n"
              << "  " << program << "\n"
              << "  " << program << " sx sy sz ex ey ez\n\n"
              << "Camera point unit: mm. Default points are the built-in sample weld line.\n";
}

bool parseCameraPoints(int argc, char** argv, weld_geometry::Vec3* start, weld_geometry::Vec3* end) {
    *start = {35.615, 61.376, 376.638};
    *end = {49.096, 76.015, 374.845};
    if (argc == 1) {
        return true;
    }
    if (argc != 7) {
        printUsage(argv[0]);
        return false;
    }
    *start = {std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3])};
    *end = {std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6])};
    return true;
}

} // namespace

int main(int argc, char** argv) {
    weld_geometry::Vec3 startCamera;
    weld_geometry::Vec3 endCamera;
    if (!parseCameraPoints(argc, argv, &startCamera, &endCamera)) {
        return 2;
    }

    fairino_client::RobotConfig robotConfig = fairino_client::loadRobotConfig("config/robot_config.yaml");
    weld_motion::HandEyeConfig handEye = weld_motion::loadHandEyeConfig("config/handeye_config.yaml");
    weld_motion::ToolConfig tool = weld_motion::loadToolConfig("config/tool_config.yaml");
    weld_motion::WeldMotionConfig motion = weld_motion::loadWeldMotionConfig("config/weld_motion_config.yaml");

    fairino_client::FairinoRobotClient robot;
    if (!robot.connectRobot(robotConfig)) {
        std::cerr << "Robot is not connected. Set connect_robot: true in config/robot_config.yaml first." << std::endl;
        return 1;
    }

    weld_geometry::Pose6D currentFlangePose;
    weld_geometry::Pose6D currentTcpPose;
    if (!robot.getCurrentFlangePose(&currentFlangePose)) {
        std::cerr << "Failed to read current flange pose." << std::endl;
        return 1;
    }
    if (!robot.getCurrentToolPose(&currentTcpPose)) {
        std::cerr << "Failed to read current TCP pose." << std::endl;
        return 1;
    }

    weld_geometry::Pose6D sdkToolCoord = tool.flangeToTool;
    if (robot.getToolCoord(tool.toolId, &sdkToolCoord)) {
        tool.flangeToTool = sdkToolCoord;
    }

    std::cout << "Current flange pose: " << weld_geometry::formatPose(currentFlangePose) << std::endl;
    std::cout << "Current TCP pose:    " << weld_geometry::formatPose(currentTcpPose) << std::endl;
    std::cout << "Tool" << tool.toolId << " coord:       " << weld_geometry::formatPose(tool.flangeToTool) << std::endl;

    const int err = fairino_client::executeLinearWeldMove(
        robot, startCamera, endCamera, currentFlangePose, currentTcpPose, handEye, tool, motion);
    if (err != 0) {
        std::cerr << "executeLinearWeldMove failed, err=" << err << std::endl;
        return 1;
    }

    std::cout << "Fairino weld move demo finished." << std::endl;
    return 0;
}
