#ifndef MYLINE_HIK_HIK_SCAN_CORE_H
#define MYLINE_HIK_HIK_SCAN_CORE_H

#include "HikCalibrationCore.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace hik_scan {

struct Pose6D {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;

    Pose6D();
};

bool buildLinearFlangePath(const Pose6D& start,
                           const Pose6D& end,
                           double stepMm,
                           int maximumPointCount,
                           std::vector<Pose6D>* targets,
                           std::string* error = 0);

struct HandEyeFile {
    bool ok;
    std::string error;
    std::string mode;
    std::string parentFrame;
    std::string childFrame;
    std::string cameraSerial;
    std::string intrinsicsSha256;
    cv::Matx44d flangeFromCamera;

    HandEyeFile();
};

bool loadHandEyeYaml(const std::string& path,
                     HandEyeFile* handEye,
                     std::string* error = 0);

struct CloudPoint {
    cv::Point3d basePointMm;
    cv::Point3d cameraPointMm;
    double confidence;
    double response;
    int profileIndex;
    double pixelU;
    double pixelV;

    CloudPoint();
};

bool appendProfileInBase(const hik_calibration::StaticProfileResult& profile,
                         const cv::Matx44d& baseFromFlange,
                         const cv::Matx44d& flangeFromCamera,
                         int profileIndex,
                         std::vector<CloudPoint>* cloud,
                         std::string* error = 0);

std::vector<CloudPoint> voxelDownsample(const std::vector<CloudPoint>& cloud,
                                        double voxelSizeMm);

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 std::string* error = 0);

}  // namespace hik_scan

#endif  // MYLINE_HIK_HIK_SCAN_CORE_H
