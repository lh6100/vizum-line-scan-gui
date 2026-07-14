#ifndef ROBOT_OFFSET_CAPTURE_PLANNER_H
#define ROBOT_OFFSET_CAPTURE_PLANNER_H

#include "../geometry/TransformUtils.h"

#include <string>
#include <vector>

namespace robot_capture {

enum class CaptureAxis {
    X,
    Y,
    Z
};

std::vector<double> buildOffsetSamples(double totalOffsetMm, double stepMm);
weld_geometry::Vec3 flangeAxisDirectionInBase(const weld_geometry::Pose6D& flangePose,
                                              CaptureAxis axis);
weld_geometry::Pose6D offsetTcpPose(const weld_geometry::Pose6D& startTcp,
                                    const weld_geometry::Vec3& directionInBase,
                                    double offsetMm);
std::string formatCaptureStem(int index,
                              const weld_geometry::Pose6D& flangePose,
                              const std::string& timestamp);

} // namespace robot_capture

#endif // ROBOT_OFFSET_CAPTURE_PLANNER_H
