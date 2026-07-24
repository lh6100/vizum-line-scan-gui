#include "HikScanCore.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

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
    cv::Vec3d weightedBaseSum;
    cv::Vec3d weightedCameraSum;
    double confidenceSum;
    double weightedResponseSum;
    double weightedPixelUSum;
    double weightedPixelVSum;
    double geometryWeightSum;
    double representativeWeight;
    int profileIndex;
    std::size_t count;
    std::uint64_t observationCount;
    std::uint32_t qualityFlags;

    VoxelAccumulator()
        : weightedBaseSum(0.0, 0.0, 0.0),
          weightedCameraSum(0.0, 0.0, 0.0), confidenceSum(0.0),
          weightedResponseSum(0.0), weightedPixelUSum(0.0),
          weightedPixelVSum(0.0), geometryWeightSum(0.0),
          representativeWeight(-1.0), profileIndex(0), count(0U),
          observationCount(0U), qualityFlags(CLOUD_QUALITY_NONE) {}
};

struct SpatialCellKey {
    long long x;
    long long y;
    long long z;

    bool operator==(const SpatialCellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct SpatialCellKeyHash {
    std::size_t operator()(const SpatialCellKey& key) const {
        const std::size_t first = std::hash<long long>()(key.x);
        const std::size_t second = std::hash<long long>()(key.y);
        const std::size_t third = std::hash<long long>()(key.z);
        std::size_t combined = first;
        combined ^= second + static_cast<std::size_t>(0x9e3779b9U) +
                    (combined << 6U) + (combined >> 2U);
        combined ^= third + static_cast<std::size_t>(0x9e3779b9U) +
                    (combined << 6U) + (combined >> 2U);
        return combined;
    }
};

bool finitePoint(const cv::Point3d& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

bool safeCellCoordinate(double value,
                        double cellSize,
                        long long* coordinate) {
    if (!coordinate || !std::isfinite(value) || !std::isfinite(cellSize) ||
        cellSize <= 0.0) {
        return false;
    }
    const double scaled = std::floor(value / cellSize);
    const double minimum = static_cast<double>(
        std::numeric_limits<long long>::min() + 1LL);
    const double maximum = static_cast<double>(
        std::numeric_limits<long long>::max() - 1LL);
    if (!std::isfinite(scaled) || scaled < minimum || scaled > maximum) {
        return false;
    }
    *coordinate = static_cast<long long>(scaled);
    return true;
}

bool spatialCellForPoint(const cv::Point3d& point,
                         double cellSize,
                         SpatialCellKey* key) {
    return key && finitePoint(point) &&
           safeCellCoordinate(point.x, cellSize, &key->x) &&
           safeCellCoordinate(point.y, cellSize, &key->y) &&
           safeCellCoordinate(point.z, cellSize, &key->z);
}

std::uint32_t saturatedObservationCount(std::uint64_t count) {
    return count >= static_cast<std::uint64_t>(
                        std::numeric_limits<std::uint32_t>::max())
        ? std::numeric_limits<std::uint32_t>::max()
        : static_cast<std::uint32_t>(count);
}

double boundedConfidence(double confidence) {
    if (!std::isfinite(confidence)) {
        return 0.0;
    }
    return std::max(0.0, std::min(1.0, confidence));
}

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
      confidence(0.0), response(0.0), profileIndex(0), pixelU(0.0),
      pixelV(0.0), qualityFlags(CLOUD_QUALITY_NONE), observationCount(1U) {}

bool cloudPointHasQualityFlag(const CloudPoint& point,
                              CloudPointQualityFlag flag) {
    return (point.qualityFlags & static_cast<std::uint32_t>(flag)) != 0U;
}

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
    if (!profile.ok) {
        setError("cannot append an unsuccessful profile", error);
        return false;
    }
    return appendProfilePointsUsingBaseFromCamera(
        profile.points, baseFromCamera, profileIndex, cloud, error);
}

bool appendProfilePointsUsingBaseFromCamera(
        const std::vector<hik_calibration::StaticProfilePoint>& points,
        const cv::Matx44d& baseFromCamera,
        int profileIndex,
        std::vector<CloudPoint>* cloud,
        std::string* error) {
    if (!cloud || points.empty() || !finiteTransform(baseFromCamera)) {
        setError("cannot append invalid profile or T_base_camera", error);
        return false;
    }
    for (std::size_t index = 0; index < points.size(); ++index) {
        const hik_calibration::StaticProfilePoint& input = points[index];
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
        if (input.stripe.qualityExtractor &&
            input.stripe.rejectFlags == 0U) {
            output.qualityFlags |= CLOUD_QUALITY_OPTICAL_ACCEPTED;
        }
        cloud->push_back(output);
    }
    return true;
}

VoxelDownsampleOptions::VoxelDownsampleOptions()
    : voxelSizeMm(0.0), confidenceWeighted(false),
      minimumConfidenceWeight(1.0e-6) {}

VoxelDownsampleStatistics::VoxelDownsampleStatistics()
    : inputPointCount(0U), finitePointCount(0U),
      rejectedNonFinitePointCount(0U), outputPointCount(0U),
      confidenceWeighted(false) {}

std::vector<CloudPoint> voxelDownsample(
        const std::vector<CloudPoint>& cloud,
        const VoxelDownsampleOptions& options,
        VoxelDownsampleStatistics* statistics) {
    VoxelDownsampleStatistics localStatistics;
    localStatistics.inputPointCount = cloud.size();
    localStatistics.confidenceWeighted = options.confidenceWeighted;
    if (!std::isfinite(options.voxelSizeMm) || options.voxelSizeMm <= 0.0) {
        localStatistics.finitePointCount = cloud.size();
        localStatistics.outputPointCount = cloud.size();
        if (statistics) {
            *statistics = localStatistics;
        }
        return cloud;
    }
    const double minimumConfidenceWeight =
        std::isfinite(options.minimumConfidenceWeight) &&
                options.minimumConfidenceWeight > 0.0
            ? options.minimumConfidenceWeight
            : 1.0e-6;

    std::map<VoxelKey, VoxelAccumulator> voxels;
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        const CloudPoint& point = cloud[index];
        if (!finitePoint(point.basePointMm) ||
            !finitePoint(point.cameraPointMm)) {
            ++localStatistics.rejectedNonFinitePointCount;
            continue;
        }
        VoxelKey key;
        if (!safeCellCoordinate(
                point.basePointMm.x, options.voxelSizeMm, &key.x) ||
            !safeCellCoordinate(
                point.basePointMm.y, options.voxelSizeMm, &key.y) ||
            !safeCellCoordinate(
                point.basePointMm.z, options.voxelSizeMm, &key.z)) {
            ++localStatistics.rejectedNonFinitePointCount;
            continue;
        }
        ++localStatistics.finitePointCount;

        const double confidence = boundedConfidence(point.confidence);
        const double geometryWeight = options.confidenceWeighted
            ? std::max(minimumConfidenceWeight, confidence)
            : 1.0;
        VoxelAccumulator& accumulator = voxels[key];
        accumulator.weightedBaseSum += geometryWeight * cv::Vec3d(
            point.basePointMm.x, point.basePointMm.y, point.basePointMm.z);
        accumulator.weightedCameraSum += geometryWeight * cv::Vec3d(
            point.cameraPointMm.x, point.cameraPointMm.y, point.cameraPointMm.z);
        accumulator.confidenceSum += confidence;
        accumulator.weightedResponseSum += geometryWeight * point.response;
        accumulator.weightedPixelUSum += geometryWeight * point.pixelU;
        accumulator.weightedPixelVSum += geometryWeight * point.pixelV;
        accumulator.geometryWeightSum += geometryWeight;
        // Preserve the legacy "last point wins" representative profile for
        // equal-weight input. Confidence-weighted mode instead retains the
        // profile that contributes the greatest geometry weight.
        if (!options.confidenceWeighted ||
            geometryWeight >= accumulator.representativeWeight) {
            accumulator.representativeWeight = geometryWeight;
            accumulator.profileIndex = point.profileIndex;
        }
        accumulator.observationCount +=
            std::max<std::uint32_t>(1U, point.observationCount);
        accumulator.qualityFlags |= point.qualityFlags;
        ++accumulator.count;
    }

    std::vector<CloudPoint> result;
    result.reserve(voxels.size());
    for (std::map<VoxelKey, VoxelAccumulator>::const_iterator iterator = voxels.begin();
         iterator != voxels.end(); ++iterator) {
        const VoxelAccumulator& accumulator = iterator->second;
        if (accumulator.count == 0U ||
            !std::isfinite(accumulator.geometryWeightSum) ||
            accumulator.geometryWeightSum <= 0.0) {
            continue;
        }
        const double geometryInverse = 1.0 / accumulator.geometryWeightSum;
        const double countInverse =
            1.0 / static_cast<double>(accumulator.count);
        CloudPoint point;
        const cv::Vec3d base =
            accumulator.weightedBaseSum * geometryInverse;
        const cv::Vec3d camera =
            accumulator.weightedCameraSum * geometryInverse;
        point.basePointMm = cv::Point3d(base[0], base[1], base[2]);
        point.cameraPointMm = cv::Point3d(camera[0], camera[1], camera[2]);
        point.confidence = accumulator.confidenceSum * countInverse;
        point.response =
            accumulator.weightedResponseSum * geometryInverse;
        point.pixelU =
            accumulator.weightedPixelUSum * geometryInverse;
        point.pixelV =
            accumulator.weightedPixelVSum * geometryInverse;
        point.profileIndex = accumulator.profileIndex;
        point.observationCount =
            saturatedObservationCount(accumulator.observationCount);
        point.qualityFlags = accumulator.qualityFlags;
        if (accumulator.count > 1U ||
            accumulator.observationCount > 1U) {
            point.qualityFlags |= CLOUD_QUALITY_VOXEL_AGGREGATED;
        }
        result.push_back(point);
    }
    localStatistics.outputPointCount = result.size();
    if (statistics) {
        *statistics = localStatistics;
    }
    return result;
}

std::vector<CloudPoint> voxelDownsample(const std::vector<CloudPoint>& cloud,
                                        double voxelSizeMm) {
    VoxelDownsampleOptions options;
    options.voxelSizeMm = voxelSizeMm;
    return voxelDownsample(cloud, options, 0);
}

AdjacentProfileSupportOptions::AdjacentProfileSupportOptions()
    : enabled(false), radiusMm(1.0), minimumSupportingProfiles(1),
      maximumProfileGap(1) {}

AdjacentProfileSupportStatistics::AdjacentProfileSupportStatistics()
    : inputPointCount(0U), validPointCount(0U), keptPointCount(0U),
      rejectedPointCount(0U), invalidPointCount(0U),
      insufficientSupportPointCount(0U) {}

AdjacentProfileSupportResult::AdjacentProfileSupportResult()
    : applied(false) {}

bool filterByAdjacentProfileSupport(
        const std::vector<CloudPoint>& cloud,
        const AdjacentProfileSupportOptions& options,
        AdjacentProfileSupportResult* result,
        std::string* error) {
    if (!result) {
        setError("adjacent-profile support output is null", error);
        return false;
    }
    *result = AdjacentProfileSupportResult();
    result->statistics.inputPointCount = cloud.size();

    if (!options.enabled) {
        result->kept = cloud;
        result->statistics.validPointCount = cloud.size();
        result->statistics.keptPointCount = cloud.size();
        if (error) {
            error->clear();
        }
        return true;
    }
    if (!std::isfinite(options.radiusMm) || options.radiusMm <= 0.0 ||
        options.minimumSupportingProfiles < 1 ||
        options.maximumProfileGap < 1) {
        setError("adjacent-profile support options are invalid", error);
        return false;
    }
    result->applied = true;
    result->kept.reserve(cloud.size());
    result->rejected.reserve(cloud.size());

    typedef std::unordered_map<
        SpatialCellKey, std::vector<std::size_t>, SpatialCellKeyHash> Grid;
    Grid grid;
    grid.reserve(cloud.size());
    std::vector<SpatialCellKey> pointCells(cloud.size());
    std::vector<unsigned char> pointValid(cloud.size(), 0U);
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        SpatialCellKey key;
        if (!spatialCellForPoint(
                cloud[index].basePointMm, options.radiusMm, &key)) {
            CloudPoint rejected = cloud[index];
            rejected.qualityFlags &=
                ~static_cast<std::uint32_t>(
                    CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED);
            rejected.qualityFlags |=
                CLOUD_QUALITY_REJECTED_INVALID_BASE_POINT;
            result->rejected.push_back(rejected);
            ++result->statistics.invalidPointCount;
            continue;
        }
        pointValid[index] = 1U;
        pointCells[index] = key;
        grid[key].push_back(index);
        ++result->statistics.validPointCount;
    }

    const double radiusSquared = options.radiusMm * options.radiusMm;
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        if (pointValid[index] == 0U) {
            continue;
        }
        const CloudPoint& point = cloud[index];
        const SpatialCellKey& center = pointCells[index];
        std::unordered_set<int> supportingProfiles;
        for (long long deltaZ = -1LL; deltaZ <= 1LL; ++deltaZ) {
            for (long long deltaY = -1LL; deltaY <= 1LL; ++deltaY) {
                for (long long deltaX = -1LL; deltaX <= 1LL; ++deltaX) {
                    SpatialCellKey neighborKey;
                    neighborKey.x = center.x + deltaX;
                    neighborKey.y = center.y + deltaY;
                    neighborKey.z = center.z + deltaZ;
                    const Grid::const_iterator found = grid.find(neighborKey);
                    if (found == grid.end()) {
                        continue;
                    }
                    const std::vector<std::size_t>& candidates = found->second;
                    for (std::size_t candidateOffset = 0U;
                         candidateOffset < candidates.size();
                         ++candidateOffset) {
                        const std::size_t candidateIndex =
                            candidates[candidateOffset];
                        if (candidateIndex == index) {
                            continue;
                        }
                        const CloudPoint& candidate = cloud[candidateIndex];
                        if (candidate.profileIndex == point.profileIndex) {
                            continue;
                        }
                        const long long profileDelta =
                            static_cast<long long>(candidate.profileIndex) -
                            static_cast<long long>(point.profileIndex);
                        if (std::llabs(profileDelta) >
                            static_cast<long long>(options.maximumProfileGap)) {
                            continue;
                        }
                        const cv::Point3d difference =
                            candidate.basePointMm - point.basePointMm;
                        const double distanceSquared =
                            difference.x * difference.x +
                            difference.y * difference.y +
                            difference.z * difference.z;
                        if (std::isfinite(distanceSquared) &&
                            distanceSquared <= radiusSquared) {
                            supportingProfiles.insert(
                                candidate.profileIndex);
                        }
                    }
                }
            }
        }

        CloudPoint classified = point;
        classified.qualityFlags &=
            ~(static_cast<std::uint32_t>(
                  CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED) |
              static_cast<std::uint32_t>(
                  CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT) |
              static_cast<std::uint32_t>(
                  CLOUD_QUALITY_REJECTED_INVALID_BASE_POINT));
        if (supportingProfiles.size() >= static_cast<std::size_t>(
                options.minimumSupportingProfiles)) {
            classified.qualityFlags |=
                CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED;
            result->kept.push_back(classified);
        } else {
            classified.qualityFlags |=
                CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT;
            result->rejected.push_back(classified);
            ++result->statistics.insufficientSupportPointCount;
        }
    }

    result->statistics.keptPointCount = result->kept.size();
    result->statistics.rejectedPointCount = result->rejected.size();
    if (error) {
        error->clear();
    }
    return true;
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
    // Append new fields after the legacy property sequence so readers that
    // consume the established leading columns remain compatible.
    output << "property uint quality_flags\nproperty uint observation_count\n";
    output << "end_header\n" << std::setprecision(12);
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        const CloudPoint& point = cloud[index];
        output << point.basePointMm.x << ' ' << point.basePointMm.y << ' '
               << point.basePointMm.z << " 0 200 255 "
               << point.confidence << ' ' << point.response << ' '
               << point.profileIndex << ' ' << point.pixelU << ' '
               << point.pixelV << ' ' << point.qualityFlags << ' '
               << point.observationCount << '\n';
    }
    if (!output.good()) {
        setError("failed while writing scan PLY: " + path, error);
        return false;
    }
    return true;
}

}  // namespace hik_scan
