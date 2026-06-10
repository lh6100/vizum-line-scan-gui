#include "PointCloudTransform.h"

#include <iostream>

#if defined(HAVE_PCL_IO)
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

namespace pointcloud_transform {

bool convertPlyCameraToBase(
    const std::string& inputPly,
    const std::string& outputPly,
    const weld_geometry::Matrix4& tBaseCamera) {
#if defined(HAVE_PCL_IO)
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    if (pcl::io::loadPLYFile(inputPly, cloud) < 0) {
        std::cerr << "Failed to load PLY: " << inputPly << std::endl;
        return false;
    }
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        pcl::PointXYZRGBA& p = cloud.points[i];
        const weld_geometry::Vec3 base = weld_geometry::transformPoint(tBaseCamera, {p.x, p.y, p.z});
        p.x = static_cast<float>(base.x);
        p.y = static_cast<float>(base.y);
        p.z = static_cast<float>(base.z);
    }
    if (pcl::io::savePLYFileBinary(outputPly, cloud) < 0) {
        std::cerr << "Failed to save PLY: " << outputPly << std::endl;
        return false;
    }
    return true;
#else
    (void)inputPly;
    (void)outputPly;
    (void)tBaseCamera;
    std::cerr << "PCL IO is not enabled in this build." << std::endl;
    return false;
#endif
}

} // namespace pointcloud_transform
