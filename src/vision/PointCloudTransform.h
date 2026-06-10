#ifndef POINT_CLOUD_TRANSFORM_H
#define POINT_CLOUD_TRANSFORM_H

#include "../geometry/TransformUtils.h"

#include <string>

namespace pointcloud_transform {

bool convertPlyCameraToBase(
    const std::string& inputPly,
    const std::string& outputPly,
    const weld_geometry::Matrix4& tBaseCamera);

} // namespace pointcloud_transform

#endif // POINT_CLOUD_TRANSFORM_H
