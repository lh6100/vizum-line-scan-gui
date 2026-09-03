#include "HikScanCore.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
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

bool hostIsLittleEndian() {
    const std::uint16_t value = 1U;
    return *reinterpret_cast<const std::uint8_t*>(&value) == 1U;
}

template <typename Value>
bool writeLittleEndian(std::ostream* output, Value value) {
    if (!output) return false;
    std::array<char, sizeof(Value)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(Value));
    if (!hostIsLittleEndian()) {
        std::reverse(bytes.begin(), bytes.end());
    }
    output->write(bytes.data(),
                  static_cast<std::streamsize>(bytes.size()));
    return output->good();
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
    double snrSum;
    double fwhmSum;
    double saturatedFractionSum;
    double secondPeakRatioSum;
    double gradientAsymmetrySum;
    double fitResidualSum;
    double centerSigmaSum;
    double geometryWeightSum;
    double representativeWeight;
    int profileIndex;
    std::size_t count;
    std::uint64_t observationCount;
    std::size_t opticalMetricCount;
    std::uint32_t stripeRejectFlags;
    std::uint32_t qualityFlags;

    VoxelAccumulator()
        : weightedBaseSum(0.0, 0.0, 0.0),
          weightedCameraSum(0.0, 0.0, 0.0), confidenceSum(0.0),
          weightedResponseSum(0.0), weightedPixelUSum(0.0),
          weightedPixelVSum(0.0), snrSum(0.0), fwhmSum(0.0),
          saturatedFractionSum(0.0), secondPeakRatioSum(0.0),
          gradientAsymmetrySum(0.0), fitResidualSum(0.0),
          centerSigmaSum(0.0), geometryWeightSum(0.0),
          representativeWeight(-1.0), profileIndex(0), count(0U),
          observationCount(0U), opticalMetricCount(0U),
          stripeRejectFlags(0U), qualityFlags(CLOUD_QUALITY_NONE) {}
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

const double kRadiansToDegrees =
    180.0 / 3.141592653589793238462643383279502884;

struct VGroovePlane {
    cv::Vec3d normal;
    double offset;
    double rmsMm;
    double spreadMm;
    std::vector<std::size_t> inliers;
    int supportingProfileCount;

    VGroovePlane()
        : normal(0.0, 0.0, 1.0), offset(0.0),
          rmsMm(std::numeric_limits<double>::infinity()), spreadMm(0.0),
          supportingProfileCount(0) {}
};

struct VGrooveModel {
    VGroovePlane first;
    VGroovePlane second;
    cv::Point3d rootPointMm;
    cv::Vec3d rootDirection;
    cv::Vec3d firstOutwardDirection;
    cv::Vec3d secondOutwardDirection;
    double planeAngleDeg;
    int rootSupportingProfileCount;
    std::vector<int> assignment;

    VGrooveModel()
        : rootPointMm(0.0, 0.0, 0.0), rootDirection(0.0, 0.0, 1.0),
          firstOutwardDirection(1.0, 0.0, 0.0),
          secondOutwardDirection(-1.0, 0.0, 0.0),
          planeAngleDeg(0.0), rootSupportingProfileCount(0) {}
};

struct VGrooveModelSearch {
    std::vector<VGrooveModel> models;
    bool insufficientEvidence;
    std::string reason;

    VGrooveModelSearch() : insufficientEvidence(false) {}
};

double clampUnit(double value) {
    return std::max(-1.0, std::min(1.0, value));
}

cv::Vec3d pointVector(const cv::Point3d& point) {
    return cv::Vec3d(point.x, point.y, point.z);
}

void canonicalizePlane(cv::Vec3d* normal, double* offset) {
    if (!normal || !offset) return;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs((*normal)[axis]) <= 1.0e-12) continue;
        if ((*normal)[axis] < 0.0) {
            *normal *= -1.0;
            *offset *= -1.0;
        }
        break;
    }
}

double planeDistance(const VGroovePlane& plane,
                     const CloudPoint& point) {
    return std::fabs(
        plane.normal.dot(pointVector(point.basePointMm)) + plane.offset);
}

int distinctProfileCount(const std::vector<CloudPoint>& points,
                         const std::vector<std::size_t>& indices) {
    std::set<int> profiles;
    for (std::size_t offset = 0U; offset < indices.size(); ++offset) {
        profiles.insert(points[indices[offset]].profileIndex);
    }
    return static_cast<int>(profiles.size());
}

bool fitPlaneLeastSquares(
        const std::vector<CloudPoint>& points,
        const std::vector<std::size_t>& indices,
        double minimumPlaneSpreadMm,
        VGroovePlane* plane) {
    if (!plane || indices.size() < 3U) return false;
    cv::Vec3d centroid(0.0, 0.0, 0.0);
    for (std::size_t offset = 0U; offset < indices.size(); ++offset) {
        const cv::Point3d& point = points[indices[offset]].basePointMm;
        if (!finitePoint(point)) return false;
        centroid += pointVector(point);
    }
    centroid *= 1.0 / static_cast<double>(indices.size());

    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (std::size_t offset = 0U; offset < indices.size(); ++offset) {
        const cv::Vec3d centered =
            pointVector(points[indices[offset]].basePointMm) - centroid;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                covariance(row, column) +=
                    centered[row] * centered[column];
            }
        }
    }
    covariance *= 1.0 / static_cast<double>(indices.size());
    cv::Mat eigenvalues;
    cv::Mat eigenvectors;
    if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors) ||
        eigenvalues.rows < 3 || eigenvectors.rows < 3 ||
        eigenvectors.cols < 3) {
        return false;
    }
    cv::Vec3d normal(
        eigenvectors.at<double>(2, 0),
        eigenvectors.at<double>(2, 1),
        eigenvectors.at<double>(2, 2));
    const double normalNorm = cv::norm(normal);
    const double secondEigenvalue = eigenvalues.at<double>(1, 0);
    if (!std::isfinite(normalNorm) || normalNorm <= 1.0e-12 ||
        !std::isfinite(secondEigenvalue) ||
        secondEigenvalue <
            minimumPlaneSpreadMm * minimumPlaneSpreadMm) {
        return false;
    }
    normal *= 1.0 / normalNorm;
    double offset = -normal.dot(centroid);
    canonicalizePlane(&normal, &offset);

    double squaredResidualSum = 0.0;
    for (std::size_t index = 0U; index < indices.size(); ++index) {
        const double residual =
            normal.dot(pointVector(
                points[indices[index]].basePointMm)) + offset;
        squaredResidualSum += residual * residual;
    }
    const double rms = std::sqrt(
        squaredResidualSum / static_cast<double>(indices.size()));
    if (!std::isfinite(rms)) return false;

    plane->normal = normal;
    plane->offset = offset;
    plane->rmsMm = rms;
    plane->spreadMm = std::sqrt(std::max(0.0, secondEigenvalue));
    plane->inliers = indices;
    plane->supportingProfileCount =
        distinctProfileCount(points, indices);
    return true;
}

bool refinePlane(
        const std::vector<CloudPoint>& points,
        const VGroovePlane& seed,
        const VGrooveTemporalValidationOptions& options,
        VGroovePlane* refined) {
    if (!refined) return false;
    VGroovePlane current = seed;
    for (int iteration = 0; iteration < 3; ++iteration) {
        std::vector<std::size_t> inliers;
        inliers.reserve(points.size());
        for (std::size_t index = 0U; index < points.size(); ++index) {
            if (finitePoint(points[index].basePointMm) &&
                planeDistance(current, points[index]) <=
                    options.pointToPlaneInlierMm) {
                inliers.push_back(index);
            }
        }
        if (inliers.size() < options.minimumPointsPerPlane ||
            distinctProfileCount(points, inliers) <
                options.minimumProfilesPerPlane ||
            !fitPlaneLeastSquares(
                points, inliers, options.minimumPlaneSpreadMm, &current)) {
            return false;
        }
    }
    std::vector<std::size_t> finalInliers;
    finalInliers.reserve(points.size());
    for (std::size_t index = 0U; index < points.size(); ++index) {
        if (finitePoint(points[index].basePointMm) &&
            planeDistance(current, points[index]) <=
                options.pointToPlaneInlierMm) {
            finalInliers.push_back(index);
        }
    }
    if (finalInliers.size() < options.minimumPointsPerPlane ||
        distinctProfileCount(points, finalInliers) <
            options.minimumProfilesPerPlane ||
        !fitPlaneLeastSquares(
            points, finalInliers, options.minimumPlaneSpreadMm, &current) ||
        current.rmsMm > options.maximumPlaneRmsMm) {
        return false;
    }
    *refined = current;
    return true;
}

bool equivalentPlane(const VGroovePlane& first,
                     const VGroovePlane& second,
                     const VGrooveTemporalValidationOptions& options) {
    const double angle = std::acos(clampUnit(std::fabs(
        first.normal.dot(second.normal)))) * kRadiansToDegrees;
    return angle <= options.equivalentPlaneAngleDeg &&
           std::fabs(first.offset - second.offset) <=
               options.equivalentPlaneOffsetMm;
}

bool betterPlane(const VGroovePlane& first,
                 const VGroovePlane& second) {
    if (first.inliers.size() != second.inliers.size()) {
        return first.inliers.size() > second.inliers.size();
    }
    return first.rmsMm < second.rmsMm;
}

std::vector<std::size_t> evenlySampledFiniteIndices(
        const std::vector<CloudPoint>& points,
        std::size_t maximumCount) {
    std::vector<std::size_t> finite;
    finite.reserve(points.size());
    for (std::size_t index = 0U; index < points.size(); ++index) {
        if (finitePoint(points[index].basePointMm)) {
            finite.push_back(index);
        }
    }
    if (finite.size() <= maximumCount) return finite;
    std::vector<std::size_t> sampled;
    sampled.reserve(maximumCount);
    for (std::size_t index = 0U; index < maximumCount; ++index) {
        const std::size_t binBegin =
            index * finite.size() / maximumCount;
        const std::size_t binEnd =
            (index + 1U) * finite.size() / maximumCount;
        // Alternate bin ends so a regular left/right or scanline ordering
        // cannot alias to only one parity when N is close to 2 * maximumCount.
        const std::size_t source =
            (index & 1U) == 0U
                ? binBegin
                : std::max(binBegin, binEnd - 1U);
        if (sampled.empty() || sampled.back() != finite[source]) {
            sampled.push_back(finite[source]);
        }
    }
    return sampled;
}

std::vector<VGroovePlane> generatePlaneCandidates(
        const std::vector<CloudPoint>& points,
        const VGrooveTemporalValidationOptions& options) {
    const std::vector<std::size_t> sampled =
        evenlySampledFiniteIndices(
            points, options.maximumPlaneSamplePoints);
    std::vector<VGroovePlane> candidates;
    if (sampled.size() < 3U) return candidates;
    for (std::size_t firstIndex = 0U;
         firstIndex + 2U < sampled.size(); ++firstIndex) {
        const cv::Vec3d first = pointVector(
            points[sampled[firstIndex]].basePointMm);
        for (std::size_t secondIndex = firstIndex + 1U;
             secondIndex + 1U < sampled.size(); ++secondIndex) {
            const cv::Vec3d second = pointVector(
                points[sampled[secondIndex]].basePointMm);
            for (std::size_t thirdIndex = secondIndex + 1U;
                 thirdIndex < sampled.size(); ++thirdIndex) {
                const cv::Vec3d third = pointVector(
                    points[sampled[thirdIndex]].basePointMm);
                cv::Vec3d normal = (second - first).cross(third - first);
                const double normalNorm = cv::norm(normal);
                if (!std::isfinite(normalNorm) ||
                    normalNorm <= 1.0e-9) {
                    continue;
                }
                normal *= 1.0 / normalNorm;
                double offset = -normal.dot(first);
                canonicalizePlane(&normal, &offset);
                VGroovePlane seed;
                seed.normal = normal;
                seed.offset = offset;
                VGroovePlane candidate;
                if (!refinePlane(points, seed, options, &candidate)) {
                    continue;
                }

                bool merged = false;
                for (std::size_t existing = 0U;
                     existing < candidates.size(); ++existing) {
                    if (!equivalentPlane(
                            candidate, candidates[existing], options)) {
                        continue;
                    }
                    if (betterPlane(candidate, candidates[existing])) {
                        candidates[existing] = candidate;
                    }
                    merged = true;
                    break;
                }
                if (!merged) candidates.push_back(candidate);
                if (candidates.size() >
                    4U * options.maximumPlaneCandidates) {
                    std::sort(
                        candidates.begin(), candidates.end(), betterPlane);
                    candidates.resize(
                        2U * options.maximumPlaneCandidates);
                }
            }
        }
    }
    std::sort(candidates.begin(), candidates.end(), betterPlane);
    if (candidates.size() > options.maximumPlaneCandidates) {
        candidates.resize(options.maximumPlaneCandidates);
    }
    return candidates;
}

double medianValue(std::vector<double> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if ((values.size() & 1U) != 0U) return values[middle];
    return 0.5 * (values[middle - 1U] + values[middle]);
}

bool orientAndCheckOneSided(
        const std::vector<CloudPoint>& points,
        const std::vector<std::size_t>& indices,
        const cv::Point3d& rootPoint,
        const cv::Vec3d& rootDirection,
        const cv::Vec3d& normal,
        const VGrooveTemporalValidationOptions& options,
        cv::Vec3d* outwardDirection) {
    if (!outwardDirection || indices.empty()) return false;
    cv::Vec3d side = rootDirection.cross(normal);
    const double sideNorm = cv::norm(side);
    if (!std::isfinite(sideNorm) || sideNorm <= 1.0e-12) {
        return false;
    }
    side *= 1.0 / sideNorm;
    const cv::Vec3d root = pointVector(rootPoint);
    std::vector<double> signedDistances;
    signedDistances.reserve(indices.size());
    for (std::size_t index = 0U; index < indices.size(); ++index) {
        signedDistances.push_back(
            (pointVector(points[indices[index]].basePointMm) - root)
                .dot(side));
    }
    const double median = medianValue(signedDistances);
    if (!std::isfinite(median)) return false;
    if (median < 0.0) {
        side *= -1.0;
        for (std::size_t index = 0U;
             index < signedDistances.size(); ++index) {
            signedDistances[index] *= -1.0;
        }
    }
    std::size_t oneSidedCount = 0U;
    for (std::size_t index = 0U;
         index < signedDistances.size(); ++index) {
        if (signedDistances[index] >=
            -options.rootSideToleranceMm) {
            ++oneSidedCount;
        }
    }
    const double fraction =
        static_cast<double>(oneSidedCount) /
        static_cast<double>(signedDistances.size());
    if (fraction < options.minimumOneSidedFraction) return false;
    *outwardDirection = side;
    return true;
}

bool buildVGrooveModel(
        const std::vector<CloudPoint>& points,
        const VGroovePlane& firstSeed,
        const VGroovePlane& secondSeed,
        const VGrooveTemporalValidationOptions& options,
        VGrooveModel* model) {
    if (!model) return false;
    VGroovePlane first = firstSeed;
    VGroovePlane second = secondSeed;
    std::vector<int> assignment(points.size(), -1);
    for (int iteration = 0; iteration < 4; ++iteration) {
        std::vector<std::size_t> firstIndices;
        std::vector<std::size_t> secondIndices;
        for (std::size_t index = 0U; index < points.size(); ++index) {
            if (!finitePoint(points[index].basePointMm)) continue;
            const double firstDistance =
                planeDistance(first, points[index]);
            const double secondDistance =
                planeDistance(second, points[index]);
            const double bestDistance =
                std::min(firstDistance, secondDistance);
            if (bestDistance > options.pointToPlaneInlierMm) continue;
            if (firstDistance <= secondDistance) {
                firstIndices.push_back(index);
            } else {
                secondIndices.push_back(index);
            }
        }
        if (firstIndices.size() < options.minimumPointsPerPlane ||
            secondIndices.size() < options.minimumPointsPerPlane ||
            distinctProfileCount(points, firstIndices) <
                options.minimumProfilesPerPlane ||
            distinctProfileCount(points, secondIndices) <
                options.minimumProfilesPerPlane ||
            !fitPlaneLeastSquares(
                points, firstIndices, options.minimumPlaneSpreadMm,
                &first) ||
            !fitPlaneLeastSquares(
                points, secondIndices, options.minimumPlaneSpreadMm,
                &second)) {
            return false;
        }
    }

    std::vector<std::size_t> firstIndices;
    std::vector<std::size_t> secondIndices;
    for (std::size_t index = 0U; index < points.size(); ++index) {
        if (!finitePoint(points[index].basePointMm)) continue;
        const double firstDistance = planeDistance(first, points[index]);
        const double secondDistance = planeDistance(second, points[index]);
        const double bestDistance = std::min(firstDistance, secondDistance);
        if (bestDistance > options.pointToPlaneInlierMm) continue;
        if (firstDistance <= secondDistance) {
            assignment[index] = 0;
            firstIndices.push_back(index);
        } else {
            assignment[index] = 1;
            secondIndices.push_back(index);
        }
    }
    if (firstIndices.size() < options.minimumPointsPerPlane ||
        secondIndices.size() < options.minimumPointsPerPlane ||
        !fitPlaneLeastSquares(
            points, firstIndices, options.minimumPlaneSpreadMm, &first) ||
        !fitPlaneLeastSquares(
            points, secondIndices, options.minimumPlaneSpreadMm, &second) ||
        first.rmsMm > options.maximumPlaneRmsMm ||
        second.rmsMm > options.maximumPlaneRmsMm) {
        return false;
    }
    const std::size_t finiteCount = static_cast<std::size_t>(
        std::count_if(
            points.begin(), points.end(),
            [](const CloudPoint& point) {
                return finitePoint(point.basePointMm);
            }));
    const double inlierFraction = finiteCount == 0U
        ? 0.0
        : static_cast<double>(
              firstIndices.size() + secondIndices.size()) /
              static_cast<double>(finiteCount);
    if (inlierFraction < options.minimumInlierFraction ||
        first.supportingProfileCount <
            options.minimumProfilesPerPlane ||
        second.supportingProfileCount <
            options.minimumProfilesPerPlane) {
        return false;
    }

    const double normalDot =
        std::fabs(first.normal.dot(second.normal));
    const double planeAngle =
        std::acos(clampUnit(normalDot)) * kRadiansToDegrees;
    if (planeAngle < options.minimumPlaneAngleDeg ||
        planeAngle > options.maximumPlaneAngleDeg) {
        return false;
    }
    cv::Vec3d rootDirection =
        first.normal.cross(second.normal);
    const double rootDirectionNorm = cv::norm(rootDirection);
    if (!std::isfinite(rootDirectionNorm) ||
        rootDirectionNorm <= 1.0e-9) {
        return false;
    }
    rootDirection *= 1.0 / rootDirectionNorm;
    const cv::Vec3d offsetCombination =
        second.offset * first.normal -
        first.offset * second.normal;
    const cv::Vec3d root =
        offsetCombination.cross(rootDirection) / rootDirectionNorm;
    const cv::Point3d rootPoint(root[0], root[1], root[2]);
    if (!finitePoint(rootPoint)) return false;

    cv::Vec3d firstOutward;
    cv::Vec3d secondOutward;
    if (!orientAndCheckOneSided(
            points, firstIndices, rootPoint, rootDirection,
            first.normal, options, &firstOutward) ||
        !orientAndCheckOneSided(
            points, secondIndices, rootPoint, rootDirection,
            second.normal, options, &secondOutward)) {
        return false;
    }

    std::map<int, std::pair<double, double> > rootDistances;
    const double infinity = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < firstIndices.size(); ++index) {
        const CloudPoint& point = points[firstIndices[index]];
        std::pair<double, double>& distances =
            rootDistances[point.profileIndex];
        if (distances.first == 0.0 && distances.second == 0.0) {
            distances = std::make_pair(infinity, infinity);
        }
        const double distance = cv::norm(
            (pointVector(point.basePointMm) - root)
                .cross(rootDirection));
        distances.first = std::min(distances.first, distance);
    }
    for (std::size_t index = 0U; index < secondIndices.size(); ++index) {
        const CloudPoint& point = points[secondIndices[index]];
        std::pair<double, double>& distances =
            rootDistances[point.profileIndex];
        if (distances.first == 0.0 && distances.second == 0.0) {
            distances = std::make_pair(infinity, infinity);
        }
        const double distance = cv::norm(
            (pointVector(point.basePointMm) - root)
                .cross(rootDirection));
        distances.second = std::min(distances.second, distance);
    }
    int rootSupportingProfiles = 0;
    for (std::map<int, std::pair<double, double> >::const_iterator it =
             rootDistances.begin();
         it != rootDistances.end(); ++it) {
        if (it->second.first <= options.maximumRootGapMm &&
            it->second.second <= options.maximumRootGapMm) {
            ++rootSupportingProfiles;
        }
    }
    if (rootSupportingProfiles <
        options.minimumRootSupportingProfiles) {
        return false;
    }

    model->first = first;
    model->second = second;
    model->rootPointMm = rootPoint;
    model->rootDirection = rootDirection;
    model->firstOutwardDirection = firstOutward;
    model->secondOutwardDirection = secondOutward;
    model->planeAngleDeg = planeAngle;
    model->rootSupportingProfileCount = rootSupportingProfiles;
    model->assignment.swap(assignment);
    return true;
}

bool equivalentVGrooveModel(
        const VGrooveModel& first,
        const VGrooveModel& second,
        const VGrooveTemporalValidationOptions& options) {
    const bool direct =
        equivalentPlane(first.first, second.first, options) &&
        equivalentPlane(first.second, second.second, options);
    const bool swapped =
        equivalentPlane(first.first, second.second, options) &&
        equivalentPlane(first.second, second.first, options);
    return direct || swapped;
}

VGrooveModelSearch findVGrooveModels(
        const std::vector<CloudPoint>& points,
        const VGrooveTemporalValidationOptions& options) {
    VGrooveModelSearch search;
    std::set<int> profiles;
    std::size_t finiteCount = 0U;
    for (std::size_t index = 0U; index < points.size(); ++index) {
        if (!finitePoint(points[index].basePointMm)) continue;
        ++finiteCount;
        profiles.insert(points[index].profileIndex);
    }
    if (finiteCount < 2U * options.minimumPointsPerPlane ||
        profiles.size() <
            static_cast<std::size_t>(
                options.minimumProfilesPerPlane)) {
        search.insufficientEvidence = true;
        search.reason =
            "insufficient finite points or distinct profiles for two planes";
        return search;
    }
    const std::vector<VGroovePlane> candidates =
        generatePlaneCandidates(points, options);
    if (candidates.size() < 2U) {
        search.reason =
            "no two non-degenerate local plane candidates";
        return search;
    }
    for (std::size_t first = 0U;
         first + 1U < candidates.size(); ++first) {
        for (std::size_t second = first + 1U;
             second < candidates.size(); ++second) {
            VGrooveModel model;
            if (!buildVGrooveModel(
                    points, candidates[first], candidates[second],
                    options, &model)) {
                continue;
            }
            bool duplicate = false;
            for (std::size_t existing = 0U;
                 existing < search.models.size(); ++existing) {
                if (equivalentVGrooveModel(
                        model, search.models[existing], options)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) search.models.push_back(model);
        }
    }
    if (search.models.empty()) {
        search.reason =
            "two-plane candidates failed residual, profile, angle or root gates";
    } else if (search.models.size() > 1U) {
        search.reason = "multiple distinct two-plane models remain valid";
    } else {
        search.reason = "unique two-plane model";
    }
    return search;
}

std::uint32_t vGrooveQualityMask() {
    return
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_V_GROOVE_GEOMETRY_VALIDATED) |
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_V_GROOVE_CANDIDATE_PROMOTED) |
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS) |
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT) |
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_REJECTED_V_GROOVE_GEOMETRY) |
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_REJECTED_V_GROOVE_ALTERNATE_BRANCH) |
        static_cast<std::uint32_t>(
            CLOUD_QUALITY_REJECTED_V_GROOVE_OUTLIER);
}

CloudPoint withVGrooveFlag(const CloudPoint& source,
                           CloudPointQualityFlag flag) {
    CloudPoint point = source;
    point.qualityFlags &= ~vGrooveQualityMask();
    point.qualityFlags |= static_cast<std::uint32_t>(flag);
    return point;
}

int outwardVGrooveFace(
        const CloudPoint& point,
        const VGrooveModel& model,
        const VGrooveTemporalValidationOptions& options) {
    if (!finitePoint(point.basePointMm)) return -1;
    const double firstDistance =
        planeDistance(model.first, point);
    const double secondDistance =
        planeDistance(model.second, point);
    if (std::min(firstDistance, secondDistance) >
        options.pointToPlaneInlierMm) {
        return -1;
    }
    const cv::Vec3d fromRoot =
        pointVector(point.basePointMm) -
        pointVector(model.rootPointMm);
    if (firstDistance <= secondDistance) {
        return fromRoot.dot(model.firstOutwardDirection) >=
                       -options.rootSideToleranceMm
            ? 0
            : -1;
    }
    return fromRoot.dot(model.secondOutwardDirection) >=
                   -options.rootSideToleranceMm
        ? 1
        : -1;
}

bool pointFitsVGrooveModel(
        const CloudPoint& point,
        const VGrooveModel& model,
        const VGrooveTemporalValidationOptions& options) {
    return outwardVGrooveFace(point, model, options) >= 0;
}

bool candidateBranchSupportsBothFacesAndRoot(
        const std::vector<CloudPoint>& candidatePoints,
        const VGrooveModel& model,
        const VGrooveTemporalValidationOptions& options) {
    struct ProfileTopology {
        std::size_t firstFaceCount{0U};
        std::size_t secondFaceCount{0U};
        std::size_t rootCount{0U};
    };
    std::size_t finiteCount = 0U;
    std::size_t fittingCount = 0U;
    std::size_t firstPlaneCount = 0U;
    std::size_t secondPlaneCount = 0U;
    std::size_t rootPointCount = 0U;
    std::map<int, ProfileTopology> topologyByProfile;
    const cv::Vec3d root = pointVector(model.rootPointMm);
    for (const CloudPoint& point : candidatePoints) {
        if (!finitePoint(point.basePointMm)) {
            continue;
        }
        ++finiteCount;
        const int face =
            outwardVGrooveFace(point, model, options);
        if (face < 0) {
            continue;
        }
        const cv::Vec3d fromRoot =
            pointVector(point.basePointMm) - root;
        ProfileTopology& topology =
            topologyByProfile[point.profileIndex];
        if (face == 0) {
            ++firstPlaneCount;
            ++topology.firstFaceCount;
        } else {
            ++secondPlaneCount;
            ++topology.secondFaceCount;
        }
        ++fittingCount;
        const double rootDistance = cv::norm(
            fromRoot.cross(model.rootDirection));
        if (std::isfinite(rootDistance) &&
            rootDistance <= options.maximumRootGapMm) {
            ++rootPointCount;
            ++topology.rootCount;
        }
    }
    const double inlierFraction = finiteCount == 0U
        ? 0.0
        : static_cast<double>(fittingCount) /
              static_cast<double>(finiteCount);
    const bool oneProfileSpansBothFacesAndRoot =
        std::any_of(
            topologyByProfile.begin(), topologyByProfile.end(),
            [](const std::pair<const int, ProfileTopology>& entry) {
                return entry.second.firstFaceCount > 0U &&
                       entry.second.secondFaceCount > 0U &&
                       entry.second.rootCount > 0U;
            });
    return inlierFraction >= options.minimumInlierFraction &&
           firstPlaneCount >=
               options.minimumCandidatePointsPerPlane &&
           secondPlaneCount >=
               options.minimumCandidatePointsPerPlane &&
           rootPointCount >=
               options.minimumCandidateRootPointCount &&
           oneProfileSpansBothFacesAndRoot;
}

bool pointInProfileWindow(int pointProfile,
                          int centerProfile,
                          int halfWindowProfiles) {
    const long long delta =
        static_cast<long long>(pointProfile) -
        static_cast<long long>(centerProfile);
    return std::llabs(delta) <=
        static_cast<long long>(halfWindowProfiles);
}

std::vector<CloudPoint> localVGrooveSupport(
        const std::vector<CloudPoint>& publishableWindow,
        const std::vector<CloudPoint>& candidateEnvelope,
        double maximumSupportDistanceMm) {
    std::vector<CloudPoint> support;
    if (candidateEnvelope.empty()) return support;
    const double maximumSquared =
        maximumSupportDistanceMm * maximumSupportDistanceMm;
    support.reserve(publishableWindow.size());
    for (std::size_t pointIndex = 0U;
         pointIndex < publishableWindow.size(); ++pointIndex) {
        if (!finitePoint(
                publishableWindow[pointIndex].basePointMm)) {
            continue;
        }
        bool nearGroup = false;
        for (std::size_t candidateIndex = 0U;
             candidateIndex < candidateEnvelope.size();
             ++candidateIndex) {
            if (!finitePoint(
                    candidateEnvelope[candidateIndex].basePointMm)) {
                continue;
            }
            const cv::Point3d difference =
                publishableWindow[pointIndex].basePointMm -
                candidateEnvelope[candidateIndex].basePointMm;
            const double squared =
                difference.x * difference.x +
                difference.y * difference.y +
                difference.z * difference.z;
            if (std::isfinite(squared) && squared <= maximumSquared) {
                nearGroup = true;
                break;
            }
        }
        if (nearGroup) support.push_back(publishableWindow[pointIndex]);
    }
    return support;
}

}  // namespace

Pose6D::Pose6D()
    : x(0.0), y(0.0), z(0.0), rx(0.0), ry(0.0), rz(0.0) {}

bool reverseLinearFlangePath(Pose6D* start,
                             Pose6D* end,
                             std::string* error) {
    if (!start || !end) {
        setError("scan start/end pose is null", error);
        return false;
    }
    if (!finitePose(*start) || !finitePose(*end)) {
        setError("scan start/end pose contains a non-finite value", error);
        return false;
    }

    const Pose6D previousStart = *start;
    const Pose6D previousEnd = *end;
    start->x = previousEnd.x;
    start->y = previousEnd.y;
    start->z = previousEnd.z;
    start->rx = previousStart.rx;
    start->ry = previousStart.ry;
    start->rz = previousStart.rz;

    *end = previousStart;
    return true;
}

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
      pixelV(0.0), opticalMetricsValid(false), snr(0.0), fwhmPx(0.0),
      saturatedFraction(0.0), secondPeakRatio(0.0),
      gradientAsymmetry(0.0), fitResidual(0.0), centerSigmaPx(0.0),
      stripeRejectFlags(0U), qualityFlags(CLOUD_QUALITY_NONE),
      observationCount(1U) {}

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
    std::vector<CloudPoint> transformed;
    transformed.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
        const hik_calibration::StaticProfilePoint& input = points[index];
        if (!finitePoint(input.cameraPointMm)) {
            std::ostringstream message;
            message << "profile camera point " << index
                    << " is not finite";
            setError(message.str(), error);
            return false;
        }
        const cv::Vec4d camera(input.cameraPointMm.x, input.cameraPointMm.y,
                              input.cameraPointMm.z, 1.0);
        const cv::Vec4d base = baseFromCamera * camera;
        if (!std::isfinite(base[0]) || !std::isfinite(base[1]) ||
            !std::isfinite(base[2])) {
            std::ostringstream message;
            message << "profile base point " << index
                    << " is not finite";
            setError(message.str(), error);
            return false;
        }
        CloudPoint output;
        output.basePointMm = cv::Point3d(base[0], base[1], base[2]);
        output.cameraPointMm = input.cameraPointMm;
        output.confidence = input.stripe.confidence;
        output.response = input.stripe.peakDifference;
        output.profileIndex = profileIndex;
        output.pixelU = input.stripe.pixel.x;
        output.pixelV = input.stripe.pixel.y;
        output.opticalMetricsValid = input.stripe.qualityExtractor;
        output.snr = input.stripe.snr;
        output.fwhmPx = input.stripe.widthPx;
        output.saturatedFraction = input.stripe.saturatedFraction;
        output.secondPeakRatio = input.stripe.secondPeakRatio;
        output.gradientAsymmetry = input.stripe.gradientAsymmetry;
        output.fitResidual = input.stripe.fitResidual;
        output.centerSigmaPx = input.stripe.centerSigmaPx;
        output.stripeRejectFlags = input.stripe.rejectFlags;
        if (input.stripe.qualityExtractor &&
            input.stripe.rejectFlags == 0U) {
            output.qualityFlags |= CLOUD_QUALITY_OPTICAL_ACCEPTED;
        }
        transformed.push_back(output);
    }
    cloud->insert(
        cloud->end(),
        std::make_move_iterator(transformed.begin()),
        std::make_move_iterator(transformed.end()));
    if (error) error->clear();
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
        if (point.opticalMetricsValid &&
            std::isfinite(point.snr) && std::isfinite(point.fwhmPx) &&
            std::isfinite(point.saturatedFraction) &&
            std::isfinite(point.secondPeakRatio) &&
            std::isfinite(point.gradientAsymmetry) &&
            std::isfinite(point.fitResidual) &&
            std::isfinite(point.centerSigmaPx)) {
            accumulator.snrSum += point.snr;
            accumulator.fwhmSum += point.fwhmPx;
            accumulator.saturatedFractionSum += point.saturatedFraction;
            accumulator.secondPeakRatioSum += point.secondPeakRatio;
            accumulator.gradientAsymmetrySum +=
                point.gradientAsymmetry;
            accumulator.fitResidualSum += point.fitResidual;
            accumulator.centerSigmaSum += point.centerSigmaPx;
            ++accumulator.opticalMetricCount;
        }
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
        accumulator.stripeRejectFlags |= point.stripeRejectFlags;
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
        point.opticalMetricsValid =
            accumulator.opticalMetricCount > 0U;
        if (point.opticalMetricsValid) {
            const double opticalInverse =
                1.0 / static_cast<double>(
                    accumulator.opticalMetricCount);
            point.snr = accumulator.snrSum * opticalInverse;
            point.fwhmPx = accumulator.fwhmSum * opticalInverse;
            point.saturatedFraction =
                accumulator.saturatedFractionSum * opticalInverse;
            point.secondPeakRatio =
                accumulator.secondPeakRatioSum * opticalInverse;
            point.gradientAsymmetry =
                accumulator.gradientAsymmetrySum * opticalInverse;
            point.fitResidual =
                accumulator.fitResidualSum * opticalInverse;
            point.centerSigmaPx =
                accumulator.centerSigmaSum * opticalInverse;
        }
        point.stripeRejectFlags = accumulator.stripeRejectFlags;
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

VGrooveCandidateBranch::VGrooveCandidateBranch()
    : ambiguityGroupId(0U), branchId(0),
      formalPublicationEligible(true) {}

VGrooveTemporalValidationOptions::VGrooveTemporalValidationOptions()
    : halfWindowProfiles(2),
      minimumPointsPerPlane(8U),
      minimumProfilesPerPlane(3),
      minimumRootSupportingProfiles(2),
      maximumPlaneSamplePoints(14U),
      maximumPlaneCandidates(12U),
      pointToPlaneInlierMm(0.35),
      maximumPlaneRmsMm(0.20),
      minimumInlierFraction(0.75),
      minimumPlaneSpreadMm(0.25),
      minimumPlaneAngleDeg(15.0),
      maximumPlaneAngleDeg(90.0),
      maximumRootGapMm(1.5),
      minimumCandidatePointsPerPlane(2U),
      minimumCandidateRootPointCount(1U),
      maximumSupportDistanceMm(8.0),
      minimumOneSidedFraction(0.90),
      rootSideToleranceMm(0.20),
      equivalentPlaneAngleDeg(2.0),
      equivalentPlaneOffsetMm(0.25) {}

VGrooveProfileValidation::VGrooveProfileValidation()
    : profileIndex(0),
      status(VGrooveProfileStatus::NotEvaluated),
      selectedBranchId(-1),
      evaluatedHypothesisCount(0U),
      validHypothesisCount(0U),
      validModelCount(0U),
      windowPointCount(0U),
      planeOnePointCount(0U),
      planeTwoPointCount(0U),
      planeOneProfileCount(0),
      planeTwoProfileCount(0),
      rootSupportingProfileCount(0),
      planeOneRmsMm(std::numeric_limits<double>::quiet_NaN()),
      planeTwoRmsMm(std::numeric_limits<double>::quiet_NaN()),
      planeAngleDeg(std::numeric_limits<double>::quiet_NaN()) {}

VGrooveAmbiguityGroupValidation::VGrooveAmbiguityGroupValidation()
    : profileIndex(0),
      ambiguityGroupId(0U),
      status(VGrooveProfileStatus::RejectedInsufficientEvidence),
      selectedBranchId(-1),
      evaluatedBranchCount(0U),
      validBranchCount(0U),
      validModelCount(0U),
      localSupportPointCount(0U),
      promotedCandidatePointCount(0U),
      rejectedCandidatePointCount(0U),
      planeOnePointCount(0U),
      planeTwoPointCount(0U),
      planeOneProfileCount(0),
      planeTwoProfileCount(0),
      rootSupportingProfileCount(0),
      planeOneRmsMm(std::numeric_limits<double>::quiet_NaN()),
      planeTwoRmsMm(std::numeric_limits<double>::quiet_NaN()),
      planeAngleDeg(std::numeric_limits<double>::quiet_NaN()) {}

VGrooveTemporalValidationStatistics::
VGrooveTemporalValidationStatistics()
    : publishableInputPointCount(0U),
      candidateInputPointCount(0U),
      profileCount(0U),
      promotedProfileCount(0U),
      ambiguousProfileCount(0U),
      insufficientProfileCount(0U),
      invalidGeometryProfileCount(0U),
      passThroughPublishablePointCount(0U),
      promotedCandidatePointCount(0U),
      rejectedCandidatePointCount(0U),
      rejectedAmbiguousCandidatePointCount(0U),
      rejectedInsufficientCandidatePointCount(0U),
      rejectedGeometryCandidatePointCount(0U),
      rejectedAlternateBranchPointCount(0U),
      rejectedOutlierPointCount(0U) {}

bool validateVGrooveTemporalGeometry(
        const std::vector<CloudPoint>& publishablePoints,
        const std::vector<VGrooveCandidateBranch>&
            ambiguousCandidateBranches,
        const VGrooveTemporalValidationOptions& options,
        VGrooveTemporalValidationResult* result,
        std::string* error) {
    if (!result) {
        setError("V-groove validation output is null", error);
        return false;
    }
    *result = VGrooveTemporalValidationResult();
    if (options.halfWindowProfiles < 1 ||
        options.minimumPointsPerPlane < 3U ||
        options.minimumProfilesPerPlane < 2 ||
        options.minimumRootSupportingProfiles < 1 ||
        options.maximumPlaneSamplePoints < 3U ||
        options.maximumPlaneCandidates < 2U ||
        !std::isfinite(options.pointToPlaneInlierMm) ||
        options.pointToPlaneInlierMm <= 0.0 ||
        !std::isfinite(options.maximumPlaneRmsMm) ||
        options.maximumPlaneRmsMm <= 0.0 ||
        !std::isfinite(options.minimumInlierFraction) ||
        options.minimumInlierFraction <= 0.0 ||
        options.minimumInlierFraction > 1.0 ||
        !std::isfinite(options.minimumPlaneSpreadMm) ||
        options.minimumPlaneSpreadMm <= 0.0 ||
        !std::isfinite(options.minimumPlaneAngleDeg) ||
        !std::isfinite(options.maximumPlaneAngleDeg) ||
        options.minimumPlaneAngleDeg <= 0.0 ||
        options.maximumPlaneAngleDeg <= options.minimumPlaneAngleDeg ||
        options.maximumPlaneAngleDeg > 90.0 ||
        !std::isfinite(options.maximumRootGapMm) ||
        options.maximumRootGapMm <= 0.0 ||
        options.minimumCandidatePointsPerPlane < 1U ||
        options.minimumCandidateRootPointCount < 1U ||
        !std::isfinite(options.maximumSupportDistanceMm) ||
        options.maximumSupportDistanceMm <= 0.0 ||
        !std::isfinite(options.minimumOneSidedFraction) ||
        options.minimumOneSidedFraction <= 0.5 ||
        options.minimumOneSidedFraction > 1.0 ||
        !std::isfinite(options.rootSideToleranceMm) ||
        options.rootSideToleranceMm < 0.0 ||
        !std::isfinite(options.equivalentPlaneAngleDeg) ||
        options.equivalentPlaneAngleDeg <= 0.0 ||
        !std::isfinite(options.equivalentPlaneOffsetMm) ||
        options.equivalentPlaneOffsetMm <= 0.0) {
        setError("V-groove validation options are invalid", error);
        return false;
    }

    VGrooveTemporalValidationStatistics& statistics =
        result->statistics;
    statistics.publishableInputPointCount =
        publishablePoints.size();
    std::set<std::pair<std::uint64_t, int> > branchKeys;
    std::map<std::uint64_t, std::vector<std::set<int> > >
        groupBranchProfiles;
    std::set<int> profileIndices;
    for (std::size_t index = 0U;
         index < publishablePoints.size(); ++index) {
        profileIndices.insert(publishablePoints[index].profileIndex);
    }
    for (std::size_t branchIndex = 0U;
         branchIndex < ambiguousCandidateBranches.size();
         ++branchIndex) {
        const VGrooveCandidateBranch& branch =
            ambiguousCandidateBranches[branchIndex];
        if (branch.branchId < 0 || branch.points.empty() ||
            !branchKeys.insert(std::make_pair(
                branch.ambiguityGroupId, branch.branchId)).second) {
            setError(
                "V-groove candidate branches require a non-negative "
                "branchId, non-empty points, and a unique "
                "(ambiguityGroupId, branchId) pair",
                error);
            return false;
        }
        for (std::size_t pointIndex = 0U;
             pointIndex < branch.points.size(); ++pointIndex) {
            if (!cloudPointHasQualityFlag(
                    branch.points[pointIndex],
                    CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE)) {
                setError(
                    "every V-groove branch point must carry "
                    "CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE provenance",
                    error);
                return false;
            }
        }
        statistics.candidateInputPointCount +=
            branch.points.size();
        std::set<int> branchProfiles;
        for (std::size_t pointIndex = 0U;
             pointIndex < branch.points.size(); ++pointIndex) {
            profileIndices.insert(
                branch.points[pointIndex].profileIndex);
            branchProfiles.insert(
                branch.points[pointIndex].profileIndex);
        }
        groupBranchProfiles[branch.ambiguityGroupId].push_back(
            branchProfiles);
    }
    std::set<std::uint64_t> incompleteGroups;
    for (std::map<
             std::uint64_t,
             std::vector<std::set<int> > >::const_iterator groupIt =
             groupBranchProfiles.begin();
         groupIt != groupBranchProfiles.end(); ++groupIt) {
        const std::vector<std::set<int> >& coverages =
            groupIt->second;
        if (coverages.size() < 2U) {
            incompleteGroups.insert(groupIt->first);
            continue;
        }
        for (std::size_t branchIndex = 1U;
             branchIndex < coverages.size(); ++branchIndex) {
            if (coverages[branchIndex] != coverages.front()) {
                incompleteGroups.insert(groupIt->first);
                break;
            }
        }
    }
    statistics.profileCount = profileIndices.size();

    for (std::set<int>::const_iterator profileIt =
             profileIndices.begin();
         profileIt != profileIndices.end(); ++profileIt) {
        const int centerProfile = *profileIt;
        std::vector<CloudPoint> commonWindow;
        std::vector<const CloudPoint*> currentPublishable;
        for (std::size_t index = 0U;
             index < publishablePoints.size(); ++index) {
            const CloudPoint& point = publishablePoints[index];
            if (pointInProfileWindow(
                    point.profileIndex, centerProfile,
                    options.halfWindowProfiles)) {
                commonWindow.push_back(point);
            }
            if (point.profileIndex == centerProfile) {
                currentPublishable.push_back(&point);
            }
        }

        VGrooveProfileValidation profileValidation;
        profileValidation.profileIndex = centerProfile;
        profileValidation.windowPointCount = commonWindow.size();
        profileValidation.reason =
            "no ambiguity group was evaluated for this profile";
        for (std::size_t index = 0U;
             index < currentPublishable.size(); ++index) {
            result->passThroughPublishable.push_back(
                *currentPublishable[index]);
        }

        std::set<std::uint64_t> activeGroups;
        for (std::size_t branchIndex = 0U;
             branchIndex < ambiguousCandidateBranches.size();
             ++branchIndex) {
            const VGrooveCandidateBranch& branch =
                ambiguousCandidateBranches[branchIndex];
            for (std::size_t pointIndex = 0U;
                 pointIndex < branch.points.size(); ++pointIndex) {
                if (branch.points[pointIndex].profileIndex ==
                    centerProfile) {
                    activeGroups.insert(branch.ambiguityGroupId);
                    break;
                }
            }
        }

        for (std::set<std::uint64_t>::const_iterator groupIt =
                 activeGroups.begin();
             groupIt != activeGroups.end(); ++groupIt) {
            const std::uint64_t groupId = *groupIt;
            VGrooveAmbiguityGroupValidation groupValidation;
            groupValidation.profileIndex = centerProfile;
            groupValidation.ambiguityGroupId = groupId;
            std::vector<const VGrooveCandidateBranch*> groupBranches;
            for (std::size_t branchIndex = 0U;
                 branchIndex < ambiguousCandidateBranches.size();
                 ++branchIndex) {
                const VGrooveCandidateBranch& branch =
                    ambiguousCandidateBranches[branchIndex];
                if (branch.ambiguityGroupId != groupId) continue;
                bool hasCenterPoint = false;
                for (std::size_t pointIndex = 0U;
                     pointIndex < branch.points.size(); ++pointIndex) {
                    if (branch.points[pointIndex].profileIndex ==
                        centerProfile) {
                        hasCenterPoint = true;
                        break;
                    }
                }
                if (hasCenterPoint) groupBranches.push_back(&branch);
            }
            groupValidation.evaluatedBranchCount =
                groupBranches.size();
            if (incompleteGroups.find(groupId) !=
                incompleteGroups.end()) {
                groupValidation.status =
                    VGrooveProfileStatus::RejectedInsufficientEvidence;
                groupValidation.reason =
                    "the ambiguity group has fewer than two complete branches "
                    "with identical profile coverage";
                for (std::size_t branchIndex = 0U;
                     branchIndex < groupBranches.size(); ++branchIndex) {
                    const VGrooveCandidateBranch& branch =
                        *groupBranches[branchIndex];
                    for (std::size_t pointIndex = 0U;
                         pointIndex < branch.points.size(); ++pointIndex) {
                        if (branch.points[pointIndex].profileIndex !=
                            centerProfile) {
                            continue;
                        }
                        result->rejectedCandidates.push_back(
                            withVGrooveFlag(
                                branch.points[pointIndex],
                                CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT));
                        ++groupValidation.rejectedCandidatePointCount;
                        ++statistics
                              .rejectedInsufficientCandidatePointCount;
                    }
                }
                result->ambiguityGroups.push_back(groupValidation);
                continue;
            }
            std::vector<CloudPoint> groupCandidateEnvelope;
            for (std::size_t branchIndex = 0U;
                 branchIndex < groupBranches.size(); ++branchIndex) {
                const VGrooveCandidateBranch& branch =
                    *groupBranches[branchIndex];
                for (std::size_t pointIndex = 0U;
                     pointIndex < branch.points.size(); ++pointIndex) {
                    if (pointInProfileWindow(
                            branch.points[pointIndex].profileIndex,
                            centerProfile,
                            options.halfWindowProfiles)) {
                        groupCandidateEnvelope.push_back(
                            branch.points[pointIndex]);
                    }
                }
            }
            const std::vector<CloudPoint> localSupport =
                localVGrooveSupport(
                    commonWindow, groupCandidateEnvelope,
                    options.maximumSupportDistanceMm);
            groupValidation.localSupportPointCount =
                localSupport.size();
            const VGrooveModelSearch localBaseline =
                findVGrooveModels(localSupport, options);
            const VGrooveModel* localBaselineModel =
                localBaseline.models.size() == 1U
                    ? &localBaseline.models.front()
                    : 0;
            if (localBaselineModel) {
                groupValidation.planeOnePointCount =
                    localBaselineModel->first.inliers.size();
                groupValidation.planeTwoPointCount =
                    localBaselineModel->second.inliers.size();
                groupValidation.planeOneProfileCount =
                    localBaselineModel->first.supportingProfileCount;
                groupValidation.planeTwoProfileCount =
                    localBaselineModel->second.supportingProfileCount;
                groupValidation.rootSupportingProfileCount =
                    localBaselineModel->rootSupportingProfileCount;
                groupValidation.planeOneRmsMm =
                    localBaselineModel->first.rmsMm;
                groupValidation.planeTwoRmsMm =
                    localBaselineModel->second.rmsMm;
                groupValidation.planeAngleDeg =
                    localBaselineModel->planeAngleDeg;
            }

            CloudPointQualityFlag rejectionFlag =
                CLOUD_QUALITY_REJECTED_V_GROOVE_GEOMETRY;
            if (!localBaselineModel) {
                if (localBaseline.models.size() > 1U) {
                    groupValidation.status =
                        VGrooveProfileStatus::RejectedAmbiguous;
                    groupValidation.reason =
                        "local publishable support admits multiple V models";
                    rejectionFlag =
                        CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS;
                } else if (localBaseline.insufficientEvidence) {
                    groupValidation.status =
                        VGrooveProfileStatus::
                            RejectedInsufficientEvidence;
                    groupValidation.reason =
                        "candidate group lacks sufficient nearby publishable "
                        "multi-profile support";
                    rejectionFlag =
                        CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT;
                } else {
                    groupValidation.status =
                        VGrooveProfileStatus::RejectedInvalidGeometry;
                    groupValidation.reason =
                        "nearby publishable support is not a valid V";
                }
                for (std::size_t branchIndex = 0U;
                     branchIndex < groupBranches.size(); ++branchIndex) {
                    const VGrooveCandidateBranch& branch =
                        *groupBranches[branchIndex];
                    for (std::size_t pointIndex = 0U;
                         pointIndex < branch.points.size(); ++pointIndex) {
                        if (branch.points[pointIndex].profileIndex !=
                            centerProfile) {
                            continue;
                        }
                        result->rejectedCandidates.push_back(
                            withVGrooveFlag(
                                branch.points[pointIndex],
                                rejectionFlag));
                        ++groupValidation.rejectedCandidatePointCount;
                        if (rejectionFlag ==
                            CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS) {
                            ++statistics
                                  .rejectedAmbiguousCandidatePointCount;
                        } else if (rejectionFlag ==
                            CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT) {
                            ++statistics
                                  .rejectedInsufficientCandidatePointCount;
                        } else {
                            ++statistics
                                  .rejectedGeometryCandidatePointCount;
                        }
                    }
                }
                result->ambiguityGroups.push_back(groupValidation);
                continue;
            }

            struct QualifyingCandidateModel {
                const VGrooveCandidateBranch* branch;
                VGrooveModel model;
            };
            std::vector<QualifyingCandidateModel> qualifyingModels;
            std::set<int> validBranchIds;
            bool anyInsufficientBranch = false;
            bool anyPublicationIneligibleBranch = false;
            for (std::size_t branchIndex = 0U;
                 branchIndex < groupBranches.size(); ++branchIndex) {
                const VGrooveCandidateBranch& branch =
                    *groupBranches[branchIndex];
                if (!branch.formalPublicationEligible) {
                    anyInsufficientBranch = true;
                    anyPublicationIneligibleBranch = true;
                    continue;
                }
                std::vector<CloudPoint> hypothesis = localSupport;
                std::vector<CloudPoint> candidateWindow;
                for (std::size_t pointIndex = 0U;
                     pointIndex < branch.points.size(); ++pointIndex) {
                    if (pointInProfileWindow(
                            branch.points[pointIndex].profileIndex,
                            centerProfile,
                            options.halfWindowProfiles)) {
                        candidateWindow.push_back(
                            branch.points[pointIndex]);
                        hypothesis.push_back(
                            branch.points[pointIndex]);
                    }
                }
                const VGrooveModelSearch candidateSearch =
                    findVGrooveModels(hypothesis, options);
                anyInsufficientBranch =
                    anyInsufficientBranch ||
                    candidateSearch.insufficientEvidence;
                for (std::size_t modelIndex = 0U;
                     modelIndex < candidateSearch.models.size();
                     ++modelIndex) {
                    const VGrooveModel& candidateModel =
                        candidateSearch.models[modelIndex];
                    if (!candidateBranchSupportsBothFacesAndRoot(
                            candidateWindow, candidateModel,
                            options)) {
                        continue;
                    }
                    QualifyingCandidateModel qualifying;
                    qualifying.branch = &branch;
                    qualifying.model = candidateModel;
                    qualifyingModels.push_back(qualifying);
                    validBranchIds.insert(branch.branchId);
                }
            }
            groupValidation.validBranchCount =
                validBranchIds.size();
            groupValidation.validModelCount =
                qualifyingModels.size();

            const QualifyingCandidateModel* selected = 0;
            if (!anyPublicationIneligibleBranch &&
                qualifyingModels.size() == 1U &&
                equivalentVGrooveModel(
                    qualifyingModels.front().model,
                    *localBaselineModel, options)) {
                selected = &qualifyingModels.front();
            }
            if (selected) {
                groupValidation.status =
                    VGrooveProfileStatus::PromotedUnique;
                groupValidation.reason =
                    "exactly one explicit branch preserves the unique V model";
                groupValidation.selectedBranchId =
                    selected->branch->branchId;
                for (std::size_t branchIndex = 0U;
                     branchIndex < groupBranches.size(); ++branchIndex) {
                    const VGrooveCandidateBranch& branch =
                        *groupBranches[branchIndex];
                    for (std::size_t pointIndex = 0U;
                         pointIndex < branch.points.size(); ++pointIndex) {
                        const CloudPoint& source =
                            branch.points[pointIndex];
                        if (source.profileIndex != centerProfile) {
                            continue;
                        }
                        if (&branch == selected->branch &&
                            pointFitsVGrooveModel(
                                source, selected->model,
                                options)) {
                            CloudPoint promoted = withVGrooveFlag(
                                source,
                                CLOUD_QUALITY_V_GROOVE_GEOMETRY_VALIDATED);
                            promoted.qualityFlags |=
                                CLOUD_QUALITY_V_GROOVE_CANDIDATE_PROMOTED;
                            result->promotedCandidates.push_back(promoted);
                            ++groupValidation
                                  .promotedCandidatePointCount;
                        } else {
                            const CloudPointQualityFlag flag =
                                !branch.formalPublicationEligible
                                ? CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT
                                : (&branch == selected->branch
                                   ? CLOUD_QUALITY_REJECTED_V_GROOVE_OUTLIER
                                   : CLOUD_QUALITY_REJECTED_V_GROOVE_ALTERNATE_BRANCH);
                            result->rejectedCandidates.push_back(
                                withVGrooveFlag(source, flag));
                            ++groupValidation
                                  .rejectedCandidatePointCount;
                            if (flag ==
                                CLOUD_QUALITY_REJECTED_V_GROOVE_OUTLIER) {
                                ++statistics.rejectedOutlierPointCount;
                            } else if (flag ==
                                       CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT) {
                                ++statistics
                                      .rejectedInsufficientCandidatePointCount;
                            } else {
                                ++statistics
                                      .rejectedAlternateBranchPointCount;
                            }
                        }
                    }
                }
            } else {
                if (anyPublicationIneligibleBranch) {
                    groupValidation.status =
                        VGrooveProfileStatus::
                            RejectedInsufficientEvidence;
                    groupValidation.reason =
                        "an earlier profile publication gate marked this "
                        "ambiguity group audit-only";
                    rejectionFlag =
                        CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT;
                } else if (qualifyingModels.size() > 1U) {
                    groupValidation.status =
                        VGrooveProfileStatus::RejectedAmbiguous;
                    groupValidation.reason =
                        "multiple candidate branches or V models remain valid";
                    rejectionFlag =
                        CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS;
                } else if (qualifyingModels.empty() &&
                           anyInsufficientBranch) {
                    groupValidation.status =
                        VGrooveProfileStatus::
                            RejectedInsufficientEvidence;
                    groupValidation.reason =
                        "candidate branches do not provide sufficient "
                        "multi-profile evidence";
                    rejectionFlag =
                        CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT;
                } else {
                    groupValidation.status =
                        VGrooveProfileStatus::RejectedInvalidGeometry;
                    groupValidation.reason =
                        qualifyingModels.size() == 1U
                        ? "candidate branch conflicts with the publishable V model"
                        : "no candidate branch satisfies the V geometry gates";
                    rejectionFlag =
                        CLOUD_QUALITY_REJECTED_V_GROOVE_GEOMETRY;
                }
                for (std::size_t branchIndex = 0U;
                     branchIndex < groupBranches.size(); ++branchIndex) {
                    const VGrooveCandidateBranch& branch =
                        *groupBranches[branchIndex];
                    for (std::size_t pointIndex = 0U;
                         pointIndex < branch.points.size(); ++pointIndex) {
                        if (branch.points[pointIndex].profileIndex !=
                            centerProfile) {
                            continue;
                        }
                        result->rejectedCandidates.push_back(
                            withVGrooveFlag(
                                branch.points[pointIndex],
                                rejectionFlag));
                        ++groupValidation.rejectedCandidatePointCount;
                        if (rejectionFlag ==
                            CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS) {
                            ++statistics
                                  .rejectedAmbiguousCandidatePointCount;
                        } else if (rejectionFlag ==
                            CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT) {
                            ++statistics
                                  .rejectedInsufficientCandidatePointCount;
                        } else {
                            ++statistics
                                  .rejectedGeometryCandidatePointCount;
                        }
                    }
                }
            }
            result->ambiguityGroups.push_back(groupValidation);
        }

        bool sawGroup = false;
        bool sawAmbiguous = false;
        bool sawInsufficient = false;
        bool sawInvalid = false;
        bool allGroupsPromoted = true;
        for (std::size_t groupIndex = 0U;
             groupIndex < result->ambiguityGroups.size(); ++groupIndex) {
            const VGrooveAmbiguityGroupValidation& group =
                result->ambiguityGroups[groupIndex];
            if (group.profileIndex != centerProfile) continue;
            sawGroup = true;
            profileValidation.evaluatedHypothesisCount +=
                group.evaluatedBranchCount;
            profileValidation.validHypothesisCount +=
                group.validBranchCount;
            profileValidation.validModelCount +=
                group.validModelCount;
            if (group.status !=
                VGrooveProfileStatus::PromotedUnique) {
                allGroupsPromoted = false;
            }
            sawAmbiguous = sawAmbiguous ||
                group.status ==
                    VGrooveProfileStatus::RejectedAmbiguous;
            sawInsufficient = sawInsufficient ||
                group.status ==
                    VGrooveProfileStatus::
                        RejectedInsufficientEvidence;
            sawInvalid = sawInvalid ||
                group.status ==
                    VGrooveProfileStatus::RejectedInvalidGeometry;
        }
        if (!sawGroup) {
            profileValidation.status =
                VGrooveProfileStatus::NotEvaluated;
        } else if (sawAmbiguous) {
            profileValidation.status =
                VGrooveProfileStatus::RejectedAmbiguous;
            profileValidation.reason =
                "one or more local ambiguity groups remain ambiguous";
            ++statistics.ambiguousProfileCount;
        } else if (sawInsufficient) {
            profileValidation.status =
                VGrooveProfileStatus::RejectedInsufficientEvidence;
            profileValidation.reason =
                "one or more local ambiguity groups lack support";
            ++statistics.insufficientProfileCount;
        } else if (sawInvalid) {
            profileValidation.status =
                VGrooveProfileStatus::RejectedInvalidGeometry;
            profileValidation.reason =
                "one or more local ambiguity groups fail V geometry";
            ++statistics.invalidGeometryProfileCount;
        } else if (allGroupsPromoted) {
            profileValidation.status =
                VGrooveProfileStatus::PromotedUnique;
            profileValidation.reason =
                "all local ambiguity groups have unique branches";
            ++statistics.promotedProfileCount;
        }
        result->profiles.push_back(profileValidation);
    }

    statistics.passThroughPublishablePointCount =
        result->passThroughPublishable.size();
    statistics.promotedCandidatePointCount =
        result->promotedCandidates.size();
    statistics.rejectedCandidatePointCount =
        result->rejectedCandidates.size();
    if (error) error->clear();
    return true;
}

PlyColorOptions::PlyColorOptions()
    : scalar(PlyColorScalar::Uniform), lowerPercentile(1.0),
      upperPercentile(99.0), invert(false) {}

namespace {

struct PlyRgb {
    std::uint8_t red{0U};
    std::uint8_t green{200U};
    std::uint8_t blue{255U};
};

struct PlyColorScale {
    PlyColorOptions options;
    double lower{0.0};
    double upper{0.0};
    bool hasFiniteValues{false};
};

double plyColorScalarValue(const CloudPoint& point,
                           PlyColorScalar scalar) {
    switch (scalar) {
    case PlyColorScalar::Uniform:
        break;
    case PlyColorScalar::CameraDepth:
        return point.cameraPointMm.z;
    case PlyColorScalar::BaseZ:
        return point.basePointMm.z;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

double sortedPercentile(const std::vector<double>& sorted,
                        double percentile) {
    if (sorted.empty()) return 0.0;
    const double position =
        static_cast<double>(sorted.size() - 1U) * percentile / 100.0;
    const std::size_t lowerIndex =
        static_cast<std::size_t>(std::floor(position));
    const std::size_t upperIndex =
        static_cast<std::size_t>(std::ceil(position));
    if (lowerIndex == upperIndex) return sorted[lowerIndex];
    const double fraction = position - static_cast<double>(lowerIndex);
    return sorted[lowerIndex] * (1.0 - fraction) +
           sorted[upperIndex] * fraction;
}

bool buildPlyColorScale(const std::vector<CloudPoint>& cloud,
                        const PlyColorOptions& options,
                        PlyColorScale* scale,
                        std::string* error) {
    if (!scale || !std::isfinite(options.lowerPercentile) ||
        !std::isfinite(options.upperPercentile) ||
        options.lowerPercentile < 0.0 ||
        options.lowerPercentile >= options.upperPercentile ||
        options.upperPercentile > 100.0) {
        setError("invalid PLY color percentile range", error);
        return false;
    }
    scale->options = options;
    if (options.scalar == PlyColorScalar::Uniform) return true;
    std::vector<double> values;
    values.reserve(cloud.size());
    for (const CloudPoint& point : cloud) {
        const double value = plyColorScalarValue(point, options.scalar);
        if (std::isfinite(value)) values.push_back(value);
    }
    if (values.empty()) return true;
    std::sort(values.begin(), values.end());
    scale->lower = sortedPercentile(values, options.lowerPercentile);
    scale->upper = sortedPercentile(values, options.upperPercentile);
    scale->hasFiniteValues = true;
    return true;
}

PlyRgb turboColor(double normalized) {
    struct Stop {
        double position;
        PlyRgb color;
    };
    static const std::array<Stop, 9> stops{{
        {0.00, {48U, 18U, 59U}},
        {0.13, {67U, 97U, 209U}},
        {0.25, {32U, 183U, 233U}},
        {0.38, {47U, 238U, 174U}},
        {0.50, {164U, 252U, 60U}},
        {0.63, {238U, 208U, 35U}},
        {0.75, {251U, 126U, 32U}},
        {0.88, {204U, 45U, 12U}},
        {1.00, {122U, 4U, 3U}}
    }};
    normalized = std::max(0.0, std::min(1.0, normalized));
    for (std::size_t index = 1U; index < stops.size(); ++index) {
        if (normalized > stops[index].position) continue;
        const Stop& first = stops[index - 1U];
        const Stop& second = stops[index];
        const double fraction =
            (normalized - first.position) /
            (second.position - first.position);
        const auto interpolate =
            [fraction](std::uint8_t firstValue,
                       std::uint8_t secondValue) {
                return static_cast<std::uint8_t>(std::lround(
                    static_cast<double>(firstValue) +
                    (static_cast<double>(secondValue) -
                     static_cast<double>(firstValue)) * fraction));
            };
        return PlyRgb{
            interpolate(first.color.red, second.color.red),
            interpolate(first.color.green, second.color.green),
            interpolate(first.color.blue, second.color.blue)};
    }
    return stops.back().color;
}

PlyRgb plyPointColor(const CloudPoint& point,
                     const PlyColorScale& scale) {
    if (scale.options.scalar == PlyColorScalar::Uniform) return PlyRgb{};
    const double value =
        plyColorScalarValue(point, scale.options.scalar);
    if (!std::isfinite(value)) return PlyRgb{128U, 128U, 128U};
    double normalized = 0.5;
    if (scale.hasFiniteValues && scale.upper > scale.lower) {
        normalized =
            (value - scale.lower) / (scale.upper - scale.lower);
    }
    if (scale.options.invert) normalized = 1.0 - normalized;
    return turboColor(normalized);
}

const char* plyColorScalarName(PlyColorScalar scalar) {
    switch (scalar) {
    case PlyColorScalar::Uniform:
        return "uniform";
    case PlyColorScalar::CameraDepth:
        return "camera_depth_mm";
    case PlyColorScalar::BaseZ:
        return "base_z_mm";
    }
    return "unknown";
}

void writePlyColorComments(std::ostream* output,
                           const PlyColorScale& scale) {
    if (!output || scale.options.scalar == PlyColorScalar::Uniform) return;
    *output << "comment color_map turbo\n";
    *output << "comment color_scalar "
            << plyColorScalarName(scale.options.scalar) << "\n";
    *output << "comment color_percentile "
            << scale.options.lowerPercentile << ' '
            << scale.options.upperPercentile << "\n";
    if (scale.hasFiniteValues) {
        *output << "comment color_range_mm " << scale.lower << ' '
                << scale.upper << "\n";
    }
}

}  // namespace

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 std::string* error) {
    return saveScanPly(
        path, cloud, frameId, PlyColorOptions(), error);
}

bool saveScanPly(const std::string& path,
                 const std::vector<CloudPoint>& cloud,
                 const std::string& frameId,
                 const PlyColorOptions& colorOptions,
                 std::string* error) {
    PlyColorScale colorScale;
    if (!buildPlyColorScale(cloud, colorOptions, &colorScale, error)) {
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
    writePlyColorComments(&output, colorScale);
    output << "element vertex " << cloud.size() << "\n";
    output << "property double x\nproperty double y\nproperty double z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "property float confidence\nproperty float response\n";
    output << "property int profile_index\nproperty float pixel_u\nproperty float pixel_v\n";
    // Append new fields after the legacy property sequence so readers that
    // consume the established leading columns remain compatible.
    output << "property uint quality_flags\nproperty uint observation_count\n";
    output << "property uchar optical_metrics_valid\n";
    output << "property float snr\nproperty float fwhm_px\n";
    output << "property float saturated_fraction\n";
    output << "property float second_peak_ratio\n";
    output << "property float gradient_asymmetry\n";
    output << "property float fit_residual\n";
    output << "property float center_sigma_px\n";
    output << "property uint stripe_reject_flags\n";
    output << "end_header\n" << std::setprecision(12);
    for (std::size_t index = 0; index < cloud.size(); ++index) {
        const CloudPoint& point = cloud[index];
        const PlyRgb color = plyPointColor(point, colorScale);
        output << point.basePointMm.x << ' ' << point.basePointMm.y << ' '
               << point.basePointMm.z << ' '
               << static_cast<unsigned int>(color.red) << ' '
               << static_cast<unsigned int>(color.green) << ' '
               << static_cast<unsigned int>(color.blue) << ' '
               << point.confidence << ' ' << point.response << ' '
               << point.profileIndex << ' ' << point.pixelU << ' '
               << point.pixelV << ' ' << point.qualityFlags << ' '
               << point.observationCount << ' '
               << (point.opticalMetricsValid ? 1 : 0) << ' '
               << point.snr << ' ' << point.fwhmPx << ' '
               << point.saturatedFraction << ' '
               << point.secondPeakRatio << ' '
               << point.gradientAsymmetry << ' '
               << point.fitResidual << ' '
               << point.centerSigmaPx << ' '
               << point.stripeRejectFlags << '\n';
    }
    if (!output.good()) {
        setError("failed while writing scan PLY: " + path, error);
        return false;
    }
    return true;
}

bool saveScanPlyBinary(const std::string& path,
                       const std::vector<CloudPoint>& cloud,
                       const std::string& frameId,
                       std::string* error) {
    return saveScanPlyBinary(
        path, cloud, frameId, PlyColorOptions(), error);
}

bool saveScanPlyBinary(const std::string& path,
                       const std::vector<CloudPoint>& cloud,
                       const std::string& frameId,
                       const PlyColorOptions& colorOptions,
                       std::string* error) {
    static_assert(sizeof(float) == 4U, "PLY float must be 32-bit");
    static_assert(sizeof(double) == 8U, "PLY double must be 64-bit");
    static_assert(sizeof(std::int32_t) == 4U, "PLY int must be 32-bit");
    static_assert(sizeof(std::uint32_t) == 4U, "PLY uint must be 32-bit");

    PlyColorScale colorScale;
    if (!buildPlyColorScale(cloud, colorOptions, &colorScale, error)) {
        return false;
    }
    std::ofstream output(
        path.c_str(),
        std::ios::out | std::ios::binary | std::ios::trunc);
    if (!output) {
        setError("cannot open binary scan PLY: " + path, error);
        return false;
    }
    output << "ply\nformat binary_little_endian 1.0\n";
    output << "comment frame_id " << frameId << "\n";
    output << "comment units millimeter\n";
    writePlyColorComments(&output, colorScale);
    output << "element vertex " << cloud.size() << "\n";
    output << "property double x\nproperty double y\nproperty double z\n";
    output << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    output << "property float confidence\nproperty float response\n";
    output << "property int profile_index\nproperty float pixel_u\nproperty float pixel_v\n";
    output << "property uint quality_flags\nproperty uint observation_count\n";
    output << "property uchar optical_metrics_valid\n";
    output << "property float snr\nproperty float fwhm_px\n";
    output << "property float saturated_fraction\n";
    output << "property float second_peak_ratio\n";
    output << "property float gradient_asymmetry\n";
    output << "property float fit_residual\n";
    output << "property float center_sigma_px\n";
    output << "property uint stripe_reject_flags\n";
    output << "end_header\n";
    if (!output.good()) {
        setError("failed while writing binary scan PLY header: " + path,
                 error);
        return false;
    }

    for (const CloudPoint& point : cloud) {
        const PlyRgb color = plyPointColor(point, colorScale);
        const float confidence = static_cast<float>(point.confidence);
        const float response = static_cast<float>(point.response);
        const std::int32_t profileIndex =
            static_cast<std::int32_t>(point.profileIndex);
        const float pixelU = static_cast<float>(point.pixelU);
        const float pixelV = static_cast<float>(point.pixelV);
        const std::uint32_t qualityFlags =
            static_cast<std::uint32_t>(point.qualityFlags);
        const std::uint32_t observationCount =
            static_cast<std::uint32_t>(point.observationCount);
        const std::uint8_t opticalMetricsValid =
            point.opticalMetricsValid ? 1U : 0U;
        const float snr = static_cast<float>(point.snr);
        const float fwhmPx = static_cast<float>(point.fwhmPx);
        const float saturatedFraction =
            static_cast<float>(point.saturatedFraction);
        const float secondPeakRatio =
            static_cast<float>(point.secondPeakRatio);
        const float gradientAsymmetry =
            static_cast<float>(point.gradientAsymmetry);
        const float fitResidual =
            static_cast<float>(point.fitResidual);
        const float centerSigmaPx =
            static_cast<float>(point.centerSigmaPx);
        const std::uint32_t stripeRejectFlags =
            static_cast<std::uint32_t>(point.stripeRejectFlags);
        if (!writeLittleEndian(&output, point.basePointMm.x) ||
            !writeLittleEndian(&output, point.basePointMm.y) ||
            !writeLittleEndian(&output, point.basePointMm.z) ||
            !writeLittleEndian(&output, color.red) ||
            !writeLittleEndian(&output, color.green) ||
            !writeLittleEndian(&output, color.blue) ||
            !writeLittleEndian(&output, confidence) ||
            !writeLittleEndian(&output, response) ||
            !writeLittleEndian(&output, profileIndex) ||
            !writeLittleEndian(&output, pixelU) ||
            !writeLittleEndian(&output, pixelV) ||
            !writeLittleEndian(&output, qualityFlags) ||
            !writeLittleEndian(&output, observationCount) ||
            !writeLittleEndian(&output, opticalMetricsValid) ||
            !writeLittleEndian(&output, snr) ||
            !writeLittleEndian(&output, fwhmPx) ||
            !writeLittleEndian(&output, saturatedFraction) ||
            !writeLittleEndian(&output, secondPeakRatio) ||
            !writeLittleEndian(&output, gradientAsymmetry) ||
            !writeLittleEndian(&output, fitResidual) ||
            !writeLittleEndian(&output, centerSigmaPx) ||
            !writeLittleEndian(&output, stripeRejectFlags)) {
            setError("failed while writing binary scan PLY: " + path,
                     error);
            return false;
        }
    }
    return true;
}

}  // namespace hik_scan
