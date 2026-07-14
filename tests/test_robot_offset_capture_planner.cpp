#include "../src/robot_capture/RobotOffsetCapturePlanner.h"
#include "../src/vision/VizumEyeCaptureOptions.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool near(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

bool checkSamples(const std::vector<double>& got, const std::vector<double>& expected) {
    if (got.size() != expected.size()) {
        std::cerr << "sample size mismatch got=" << got.size()
                  << " expected=" << expected.size() << std::endl;
        return false;
    }
    for (size_t i = 0; i < got.size(); ++i) {
        if (!near(got[i], expected[i])) {
            std::cerr << "sample mismatch at " << i << " got=" << got[i]
                      << " expected=" << expected[i] << std::endl;
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    using robot_capture::CaptureAxis;

    bool ok = true;
    ok = checkSamples(robot_capture::buildOffsetSamples(5.0, 2.0),
                      std::vector<double>{0.0, 2.0, 4.0, 5.0}) && ok;
    ok = checkSamples(robot_capture::buildOffsetSamples(-5.0, 2.0),
                      std::vector<double>{0.0, -2.0, -4.0, -5.0}) && ok;

    const weld_geometry::Pose6D flangeIdentity{10.0, 20.0, 30.0, 0.0, 0.0, 0.0};
    const weld_geometry::Vec3 xDir =
        robot_capture::flangeAxisDirectionInBase(flangeIdentity, CaptureAxis::X);
    const weld_geometry::Vec3 yDir =
        robot_capture::flangeAxisDirectionInBase(flangeIdentity, CaptureAxis::Y);
    ok = near(xDir.x, 1.0) && near(xDir.y, 0.0) && near(xDir.z, 0.0) && ok;
    ok = near(yDir.x, 0.0) && near(yDir.y, 1.0) && near(yDir.z, 0.0) && ok;

    const weld_geometry::Pose6D tcp{100.0, 200.0, 300.0, 1.0, 2.0, 3.0};
    const weld_geometry::Pose6D target =
        robot_capture::offsetTcpPose(tcp, {0.0, 0.0, 1.0}, 12.5);
    ok = near(target.x, 100.0) && near(target.y, 200.0) && near(target.z, 312.5) && ok;
    ok = near(target.rx, tcp.rx) && near(target.ry, tcp.ry) && near(target.rz, tcp.rz) && ok;

    const std::string stem =
        robot_capture::formatCaptureStem(7, {1.25, -2.5, 3.75, 0.0, 0.0, 0.0}, "20260708_170000_123");
    ok = stem == "000007_X1.250_Y-2.500_Z3.750_20260708_170000_123" && ok;
    ok = !vizum_capture::defaultRobotOffsetUseCalibImage() && ok;

    if (!ok) {
        std::cerr << "RobotOffsetCapturePlannerTest failed." << std::endl;
        return 1;
    }
    std::cout << "RobotOffsetCapturePlannerTest passed." << std::endl;
    return 0;
}
