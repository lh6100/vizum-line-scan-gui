#include "HikScanCore.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>

namespace hik_scan {
namespace {

void setError(const std::string& message, std::string* error) {
    if (error) {
        *error = message;
    }
}

std::string trim(const std::string& input) {
    const std::string::size_type begin = input.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return std::string();
    }
    const std::string::size_type end = input.find_last_not_of(" \t\r\n");
    return input.substr(begin, end - begin + 1U);
}

std::string unquote(const std::string& input) {
    const std::string value = trim(input);
    if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
        return value.substr(1U, value.size() - 2U);
    }
    return value;
}

bool finitePose(const Pose6D& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.z) && std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) && std::isfinite(pose.rz);
}

bool finiteTransform(const cv::Matx44d& transform) {
    for (int index = 0; index < 16; ++index) {
        if (!std::isfinite(transform.val[index])) {
            return false;
        }
    }
    return true;
}

bool parseMatrix16(const std::string& value, cv::Matx44d* output) {
    if (!output) {
        return false;
    }
    const std::string::size_type begin = value.find('[');
    const std::string::size_type end = value.rfind(']');
    if (begin == std::string::npos || end == std::string::npos || end <= begin) {
        return false;
    }
    std::string contents = value.substr(begin + 1U, end - begin - 1U);
    std::replace(contents.begin(), contents.end(), ',', ' ');
    std::istringstream stream(contents);
    cv::Matx44d matrix;
    for (int index = 0; index < 16; ++index) {
        if (!(stream >> matrix.val[index])) {
            return false;
        }
    }
    double extra = 0.0;
    if (stream >> extra || !finiteTransform(matrix)) {
        return false;
    }
    *output = matrix;
    return true;
}

struct VoxelKey {
    long long x;
    long long y;
    long long z;

    bool operator<(const VoxelKey& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

struct VoxelAccumulator {
    cv::Vec3d baseSum;
    cv::Vec3d cameraSum;
    double confidenceSum;
    double responseSum;
    double pixelUSum;
    double pixelVSum;
    int profileIndex;
    int count;

    VoxelAccumulator()
        : baseSum(0.0, 0.0, 0.0), cameraSum(0.0, 0.0, 0.0),
          confidenceSum(0.0), responseSum(0.0), pixelUSum(0.0),
          pixelVSum(0.0), profileIndex(0), count(0) {}
};

}  // namespace

Pose6D::Pose6D()
    : x(0.0), y(0.0), z(0.0), rx(0.0), ry(0.0), rz(0.0) {}

bool buildLinearFlangePath(const Pose6D& start,
                           const Pose6D& end,
                           double stepMm,
                           int maximumPointCount,
                           std::vector<Pose6D>* targets,
                           std::string* error) {
    if (!targets) {
        setError("scan target output is null", error);
        return false;
    }
    targets->clear();
    if (!finitePose(start) || !finitePose(end) || !std::isfinite(stepMm) ||
        stepMm <= 0.0 || maximumPointCount < 2) {
        setError("linear scan path options are invalid", error);
        return false;
    }
    const cv::Vec3d delta(end.x - start.x, end.y - start.y, end.z - start.z);
    const double length = cv::norm(delta);
    if (!std::isfinite(length) || length < 0.5) {
        setError("scan start and end are too close", error);
        return false;
    }
    const int intervals = std::max(1, static_cast<int>(std::ceil(length / stepMm)));
    if (intervals + 1 > maximumPointCount) {
        std::ostringstream message;
        message << "scan path requires " << intervals + 1
                << " points, exceeding limit " << maximumPointCount;
        setError(message.str(), error);
        return false;
    }
    targets->reserve(static_cast<std::size_t>(intervals + 1));
    for (int index = 0; index <= intervals; ++index) {
        const double ratio = static_cast<double>(index) /
                             static_cast<double>(intervals);
        Pose6D target = start;
        target.x = start.x + ratio * delta[0];
        target.y = start.y + ratio * delta[1];
        target.z = start.z + ratio * delta[2];
        // Scan orientation is intentionally locked to the start pose. End RPY
        // is a teaching aid only and never interpolated into a verification scan.
        targets->push_back(target);
    }
    return true;
}

HandEyeFile::HandEyeFile()
    : ok(false), flangeFromCamera(cv::Matx44d::eye()) {}

bool loadHandEyeYaml(const std::string& path,
                     HandEyeFile* handEye,
                     std::string* error) {
    if (!handEye) {
        setError("hand-eye output is null", error);
        return false;
    }
    *handEye = HandEyeFile();
    std::ifstream input(path.c_str());
    if (!input) {
        handEye->error = "cannot open hand-eye YAML: " + path;
        setError(handEye->error, error);
        return false;
    }
    std::string section;
    bool foundMatrix = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string::size_type comment = line.find('#');
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        const std::string value = trim(line);
        if (value.empty()) {
            continue;
        }
        const std::string::size_type colon = value.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        const std::string key = trim(value.substr(0, colon));
        const std::string scalar = trim(value.substr(colon + 1U));
        if (scalar.empty()) {
            section = key;
            continue;
        }
        if (key == "mode") handEye->mode = unquote(scalar);
        else if (key == "parent_frame") handEye->parentFrame = unquote(scalar);
        else if (key == "child_frame") handEye->childFrame = unquote(scalar);
        else if (key == "T_flange_camera") {
            foundMatrix = parseMatrix16(scalar, &handEye->flangeFromCamera);
        } else if (section == "camera" && key == "serial") {
            handEye->cameraSerial = unquote(scalar);
        } else if (section == "sources" && key == "intrinsics_sha256") {
            handEye->intrinsicsSha256 = unquote(scalar);
        }
    }
    const cv::Matx33d rotation(
        handEye->flangeFromCamera(0, 0), handEye->flangeFromCamera(0, 1), handEye->flangeFromCamera(0, 2),
        handEye->flangeFromCamera(1, 0), handEye->flangeFromCamera(1, 1), handEye->flangeFromCamera(1, 2),
        handEye->flangeFromCamera(2, 0), handEye->flangeFromCamera(2, 1), handEye->flangeFromCamera(2, 2));
    const cv::Matx33d orthogonality = rotation.t() * rotation - cv::Matx33d::eye();
    double maximumError = 0.0;
    for (int index = 0; index < 9; ++index) {
        maximumError = std::max(maximumError, std::fabs(orthogonality.val[index]));
    }
    const double determinant = cv::determinant(cv::Mat(rotation));
    if (!foundMatrix || handEye->mode != "camera_to_flange" ||
        handEye->parentFrame.empty() || handEye->childFrame.empty() ||
        handEye->cameraSerial.empty() || handEye->intrinsicsSha256.empty() ||
        maximumError > 1.0e-6 || std::fabs(determinant - 1.0) > 1.0e-6) {
        handEye->error = "hand-eye YAML is incomplete or T_flange_camera is not rigid";
        setError(handEye->error, error);
        return false;
    }
    handEye->ok = true;
    return true;
}

CloudPoint::CloudPoint()
    : basePointMm(0.0, 0.0, 0.0), cameraPointMm(0.0, 0.0, 0.0),
      confidence(0.0), response(0.0), profileIndex(0), pixelU(0.0), pixelV(0.0) {}

bool appendProfileInBase(const hik_calibration::StaticProfileResult& profile,
                         const cv::Matx44d& baseFromFlange,
                         const cv::Matx44d& flangeFromCamera,
                         int profileIndex,
                         std::vector<CloudPoint>* cloud,
                         std::string* error) {
    if (!finiteTransform(baseFromFlange) || !finiteTransform(flangeFromCamera)) {
        setError("cannot append invalid profile or transform", error);
        return false;
    }
    const cv::Matx44d baseFromCamera = baseFromFlange * flangeFromCamera;
    return appendProfileUsingBaseFromCamera(
        profile, baseFromCamera, profileIndex, cloud, error);
}

bool appendProfileUsingBaseFromCamera(
        const hik_calibration::StaticProfileResult& profile,
        const cv::Matx44d& baseFromCamera,
        int profileIndex,
        std::vector<CloudPoint>* cloud,
        std::string* error) {
    if (!cloud || !profile.ok || profile.points.empty() ||
        !finiteTransform(baseFromCamera)) {
        setError("cannot append invalid profile or T_base_camera", error);
        return false;
    }
    for (std::size_t index = 0; index < profile.points.size(); ++index) {
        const hik_calibration::StaticProfilePoint& input = profile.points[index];
        const cv::Vec4d camera(input.cameraPointMm.x, input.cameraPointMm.y,
                              input.cameraPointMm.z, 1.0);
        const cv::Vec4d base = baseFromCamera * camera;
        if (!std::isfinite(base[0]) || !std::isfinite(base[1]) ||
            !std::isfinite(base[2])) {
            continue;
        }
        CloudPoint output;
        output.basePointMm = cv::Point3d(base[0], base[1], base[2]);
        output.cameraPointMm = input.cameraPointMm;
        output.confidence = input.stripe.confidence;
        output.response = input.stripe.peakDifference;
        output.profileIndex = profileIndex;
        output.pixelU = input.stripe.pixel.x;
        output.pixelV = input.stripe.pixel.y;
        cloud->push_back(output);
    }
    return true;
}

std::vector<CloudPoint> voxelDownsample(const std::vector<CloudPoint>& cloud,
                                        double voxelSizeMm) {
    if (!std::isfinite(voxelSizeMm) || voxelSizeMm <= 0.0) {
        return cloud;
    }
    std::map<VoxelKey, VoxelAccumulator> voxels;
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        const CloudPoint& point = cloud[index];
        VoxelKey key;
        key.x = static_cast<long long>(std::floor(point.basePointMm.x / voxelSizeMm));
        key.y = static_cast<long long>(std::floor(point.basePointMm.y / voxelSizeMm));
        key.z = static_cast<long long>(std::floor(point.basePointMm.z / voxelSizeMm));
        VoxelAccumulator& accumulator = voxels[key];
        accumulator.baseSum += cv::Vec3d(point.basePointMm.x, point.basePointMm.y, point.basePointMm.z);
        accumulator.cameraSum += cv::Vec3d(point.cameraPointMm.x, point.cameraPointMm.y, point.cameraPointMm.z);
        accumulator.confidenceSum += point.confidence;
        accumulator.responseSum += point.response;
        accumulator.pixelUSum += point.pixelU;
        accumulator.pixelVSum += point.pixelV;
        accumulator.profileIndex = point.profileIndex;
        ++accumulator.count;
    }
    std::vector<CloudPoint> result;
    result.reserve(voxels.size());
    for (std::map<VoxelKey, VoxelAccumulator>::const_iterator iterator = voxels.begin();
         iterator != voxels.end(); ++iterator) {
        const VoxelAccumulator& accumulator = iterator->second;
        const double inverse = 1.0 / static_cast<double>(accumulator.count);
        CloudPoint point;
        const cv::Vec3d base = accumulator.baseSum * inverse;
        const cv::Vec3d camera = accumulator.cameraSum * inverse;
        point.basePointMm = cv::Point3d(base[0], base[1], base[2]);
        point.cameraPointMm = cv::Point3d(camera[0], camera[1], camera[2]);
        point.confidence = accumulator.confidenceSum * inverse;
        point.response = accumulator.responseSum * inverse;
        point.pixelU = accumulator.pixelUSum * inverse;
        point.pixelV = accumulator.pixelVSum * inverse;
        point.profileIndex = accumulator.profileIndex;
        result.push_back(point);
    }
    return result;
}

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 std::string* error) {
    if (cloud.empty()) {
        setError("cannot save an empty scan cloud", error);
        return false;
    }
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output) {
        setError("cannot open scan PLY: " + path, error);
        return false;
    }
    output << "ply\nformat ascii 1.0\n";
    output << "comment frame_id " << frameId << "\n";
    output << "comment units millimeter\n";
    output << "element vertex " << cloud.size() << "\n";
    output << "property double x\nproperty double y\nproperty double z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "property float confidence\nproperty float response\n";
    output << "property int profile_index\nproperty float pixel_u\nproperty float pixel_v\n";
    output << "end_header\n" << std::setprecision(12);
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        const CloudPoint& point = cloud[index];
        output << point.basePointMm.x << ' ' << point.basePointMm.y << ' '
               << point.basePointMm.z << " 0 200 255 "
               << point.confidence << ' ' << point.response << ' '
               << point.profileIndex << ' ' << point.pixelU << ' '
               << point.pixelV << '\n';
    }
    if (!output.good()) {
        setError("failed while writing scan PLY: " + path, error);
        return false;
    }
    return true;
}

}  // namespace hik_scan
