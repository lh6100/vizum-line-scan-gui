#include "RobotOffsetCapturePlanner.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace robot_capture {

namespace {

const double kEpsilon = 1e-9;

weld_geometry::Vec3 vectorBetween(const weld_geometry::Vec3& a,
                                  const weld_geometry::Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

weld_geometry::Vec3 normalized(const weld_geometry::Vec3& v) {
    const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= kEpsilon || !std::isfinite(len)) {
        return {0.0, 0.0, 0.0};
    }
    return {v.x / len, v.y / len, v.z / len};
}

} // namespace

std::vector<double> buildOffsetSamples(double totalOffsetMm, double stepMm) {
    std::vector<double> samples;
    samples.push_back(0.0);

    if (!std::isfinite(totalOffsetMm) || !std::isfinite(stepMm) || std::fabs(totalOffsetMm) <= kEpsilon) {
        return samples;
    }

    const double step = std::fabs(stepMm);
    if (step <= kEpsilon) {
        samples.push_back(totalOffsetMm);
        return samples;
    }

    const double sign = totalOffsetMm >= 0.0 ? 1.0 : -1.0;
    const double total = std::fabs(totalOffsetMm);
    for (double d = step; d < total - kEpsilon; d += step) {
        samples.push_back(sign * d);
    }
    samples.push_back(totalOffsetMm);
    return samples;
}

weld_geometry::Vec3 flangeAxisDirectionInBase(const weld_geometry::Pose6D& flangePose,
                                              CaptureAxis axis) {
    const weld_geometry::Matrix4 tBaseFlange = weld_geometry::poseToMatrix(flangePose);
    const weld_geometry::Vec3 origin = weld_geometry::transformPoint(tBaseFlange, {0.0, 0.0, 0.0});

    weld_geometry::Vec3 localAxis{1.0, 0.0, 0.0};
    if (axis == CaptureAxis::Y) {
        localAxis = {0.0, 1.0, 0.0};
    } else if (axis == CaptureAxis::Z) {
        localAxis = {0.0, 0.0, 1.0};
    }

    const weld_geometry::Vec3 axisPoint = weld_geometry::transformPoint(tBaseFlange, localAxis);
    return normalized(vectorBetween(axisPoint, origin));
}

weld_geometry::Pose6D offsetTcpPose(const weld_geometry::Pose6D& startTcp,
                                    const weld_geometry::Vec3& directionInBase,
                                    double offsetMm) {
    weld_geometry::Pose6D target = startTcp;
    target.x = startTcp.x + directionInBase.x * offsetMm;
    target.y = startTcp.y + directionInBase.y * offsetMm;
    target.z = startTcp.z + directionInBase.z * offsetMm;
    target.rx = startTcp.rx;
    target.ry = startTcp.ry;
    target.rz = startTcp.rz;
    return target;
}

std::string formatCaptureStem(int index,
                              const weld_geometry::Pose6D& flangePose,
                              const std::string& timestamp) {
    std::ostringstream out;
    out << std::setfill('0') << std::setw(6) << index << std::setfill(' ')
        << std::fixed << std::setprecision(3)
        << "_X" << flangePose.x
        << "_Y" << flangePose.y
        << "_Z" << flangePose.z
        << "_" << timestamp;
    return out.str();
}

} // namespace robot_capture
