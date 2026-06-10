#include "../src/robot_client/FairinoRobotClient.h"
#include "../src/welding/WeldPathPlanner.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool nearlyEqual(double a, double b, double tolerance) {
    return std::fabs(a - b) <= tolerance;
}

int checkVec(const weld_geometry::Vec3& actual, const weld_geometry::Vec3& expected, double tolerance, const char* name) {
    const bool ok = nearlyEqual(actual.x, expected.x, tolerance) &&
                    nearlyEqual(actual.y, expected.y, tolerance) &&
                    nearlyEqual(actual.z, expected.z, tolerance);
    if (!ok) {
        std::cerr << name << " mismatch. actual=" << weld_geometry::formatVec3(actual)
                  << ", expected=" << weld_geometry::formatVec3(expected)
                  << ", tolerance=" << tolerance << std::endl;
        return 1;
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const weld_geometry::Vec3 startCamera = {35.615, 61.376, 376.638};
    const weld_geometry::Vec3 endCamera = {49.096, 76.015, 374.845};
    const weld_geometry::Pose6D exampleFlangePose = {
        136.008, -469.288, 475.017, -175.958, -13.814, -151.632
    };

    weld_motion::HandEyeConfig handEye = weld_motion::loadHandEyeConfig("config/handeye_config.yaml");
    weld_motion::ToolConfig tool = weld_motion::loadToolConfig("config/tool_config.yaml");
    weld_motion::WeldMotionConfig motion = weld_motion::loadWeldMotionConfig("config/weld_motion_config.yaml");

    weld_geometry::Pose6D currentTcpPose = exampleFlangePose;
    if (argc >= 7) {
        currentTcpPose = {
            std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]),
            std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6])
        };
    }

    const weld_motion::WeldLinePlan plan = weld_motion::planLinearWeldMove(
        startCamera, endCamera, exampleFlangePose, currentTcpPose, handEye, motion);

    weld_motion::printWeldLinePlan(plan);
    std::cout << "Tool coord fallback: id=" << tool.toolId
              << ", flange->TCP=" << weld_geometry::formatPose(tool.flangeToTool) << std::endl;

    int failures = 0;
    failures += checkVec(plan.startBase, {278.11, -509.62, -49.85}, 1.5, "Start_base");
    failures += checkVec(plan.endBase, {296.91, -516.32, -48.89}, 1.5, "End_base");
    if (!nearlyEqual(plan.lineLengthMm, 19.98, 1.5)) {
        std::cerr << "Line length mismatch. actual=" << plan.lineLengthMm
                  << ", expected=19.98, tolerance=1.5" << std::endl;
        ++failures;
    }

    fairino_client::RobotConfig robotConfig = fairino_client::loadRobotConfig("config/robot_config.yaml");
    fairino_client::FairinoRobotClient robot;
    robot.connectRobot(robotConfig);
    fairino_client::executeLinearWeldMove(
        robot, startCamera, endCamera, exampleFlangePose, currentTcpPose, handEye, tool, motion);

    if (failures != 0) {
        std::cerr << "Camera-to-base test failed." << std::endl;
        return 1;
    }
    std::cout << "Camera-to-base test passed." << std::endl;
    return 0;
}
