#include "AdaptiveScanPlanner.h"

#include "HandEyeCalibrationCore.h"
#include "StripeCenterlineExtractor.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace hik_adaptive {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumSegmentLengthMm = 0.5;

void setError(const std::string& message, std::string* error) {
    if (error) *error = message;
}

bool finiteNumber(double value) {
    return std::isfinite(value);
}

bool finitePoint(const cv::Point3d& point) {
    return finiteNumber(point.x) && finiteNumber(point.y) &&
           finiteNumber(point.z);
}

bool finiteVector(const cv::Vec3d& value) {
    return finiteNumber(value[0]) && finiteNumber(value[1]) &&
           finiteNumber(value[2]);
}

bool finitePose(const hik_scan::Pose6D& pose) {
    return finiteNumber(pose.x) && finiteNumber(pose.y) &&
           finiteNumber(pose.z) && finiteNumber(pose.rx) &&
           finiteNumber(pose.ry) && finiteNumber(pose.rz);
}

double poseDistanceMm(const hik_scan::Pose6D& first,
                      const hik_scan::Pose6D& second) {
    return cv::norm(cv::Vec3d(second.x - first.x,
                             second.y - first.y,
                             second.z - first.z));
}

cv::Vec3d normalized(const cv::Vec3d& input,
                     const cv::Vec3d& fallback);
double poseRotationDistanceDeg(const hik_scan::Pose6D& first,
                               const hik_scan::Pose6D& second);

struct ArcGeometry {
    cv::Vec3d center{0.0, 0.0, 0.0};
    cv::Vec3d radialStart{1.0, 0.0, 0.0};
    cv::Vec3d radialQuarter{0.0, 1.0, 0.0};
    double radiusMm{0.0};
    double sweepRad{0.0};
};

double normalizedPositiveAngle(double angle) {
    const double period = 2.0 * kPi;
    while (angle < 0.0) angle += period;
    while (angle >= period) angle -= period;
    return angle;
}

bool computeArcGeometry(const hik_scan::Pose6D& start,
                        const hik_scan::Pose6D& via,
                        const hik_scan::Pose6D& end,
                        ArcGeometry* geometry) {
    if (!geometry || !finitePose(start) ||
        !finitePose(via) || !finitePose(end)) {
        return false;
    }
    const cv::Vec3d a(start.x, start.y, start.z);
    const cv::Vec3d b(via.x, via.y, via.z);
    const cv::Vec3d c(end.x, end.y, end.z);
    const cv::Vec3d u = b - a;
    const cv::Vec3d v = c - a;
    const cv::Vec3d normal = u.cross(v);
    const double normalSquared = normal.dot(normal);
    if (!finiteNumber(normalSquared) || normalSquared < 1.0e-12) {
        return false;
    }
    const cv::Vec3d center =
        a + (u.dot(u) * v.cross(normal) +
             v.dot(v) * normal.cross(u)) *
                (0.5 / normalSquared);
    const cv::Vec3d startRadius = a - center;
    const double radius = cv::norm(startRadius);
    if (!finiteVector(center) || !finiteNumber(radius) ||
        radius < 0.25) {
        return false;
    }
    const cv::Vec3d normalUnit =
        normal * (1.0 / std::sqrt(normalSquared));
    const cv::Vec3d e1 = startRadius * (1.0 / radius);
    const cv::Vec3d e2 =
        normalized(normalUnit.cross(e1),
                   cv::Vec3d(0.0, 1.0, 0.0));
    const auto angleFor =
        [&](const cv::Vec3d& point) {
            const cv::Vec3d radial = point - center;
            return normalizedPositiveAngle(
                std::atan2(radial.dot(e2), radial.dot(e1)));
        };
    const double viaAngle = angleFor(b);
    const double endAngle = angleFor(c);
    double selectedSweep =
        std::numeric_limits<double>::infinity();
    for (int endTurn = -1; endTurn <= 1; ++endTurn) {
        const double sweep =
            endAngle + 2.0 * kPi * endTurn;
        if (std::abs(sweep) < 1.0e-8 ||
            std::abs(sweep) > 2.0 * kPi + 1.0e-8) {
            continue;
        }
        for (int viaTurn = -1; viaTurn <= 1; ++viaTurn) {
            const double through =
                viaAngle + 2.0 * kPi * viaTurn;
            const bool contained = sweep > 0.0
                ? through > 0.0 && through < sweep
                : through < 0.0 && through > sweep;
            if (contained &&
                std::abs(sweep) < std::abs(selectedSweep)) {
                selectedSweep = sweep;
            }
        }
    }
    if (!finiteNumber(selectedSweep)) return false;
    geometry->center = center;
    geometry->radialStart = e1;
    geometry->radialQuarter = e2;
    geometry->radiusMm = radius;
    geometry->sweepRad = selectedSweep;
    return true;
}

double wrapDegrees(double value) {
    while (value > 180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return value;
}

hik_scan::Pose6D interpolatePose(
        const hik_scan::Pose6D& first,
        const hik_scan::Pose6D& second,
        double ratio) {
    hik_scan::Pose6D pose = first;
    pose.x += ratio * (second.x - first.x);
    pose.y += ratio * (second.y - first.y);
    pose.z += ratio * (second.z - first.z);
    pose.rx += ratio * wrapDegrees(second.rx - first.rx);
    pose.ry += ratio * wrapDegrees(second.ry - first.ry);
    pose.rz += ratio * wrapDegrees(second.rz - first.rz);
    return pose;
}

bool appendLinearPoseSamples(
        const hik_scan::Pose6D& first,
        const hik_scan::Pose6D& second,
        double maximumCartesianStepMm,
        double maximumAngularStepDeg,
        std::size_t maximumSampleCount,
        std::vector<hik_scan::Pose6D>* samples) {
    if (!samples || !finitePose(first) || !finitePose(second)) {
        return false;
    }
    const std::size_t translationIntervals =
        static_cast<std::size_t>(std::ceil(
            poseDistanceMm(first, second) /
            maximumCartesianStepMm));
    const std::size_t rotationIntervals =
        static_cast<std::size_t>(std::ceil(
            poseRotationDistanceDeg(first, second) /
            maximumAngularStepDeg));
    const std::size_t intervals = std::max<std::size_t>(
        1U, std::max(translationIntervals, rotationIntervals));
    if (samples->size() + intervals > maximumSampleCount) {
        return false;
    }
    for (std::size_t index = 1U; index <= intervals; ++index) {
        samples->push_back(interpolatePose(
            first, second,
            static_cast<double>(index) /
                static_cast<double>(intervals)));
    }
    return true;
}

double poseRotationDistanceDeg(const hik_scan::Pose6D& first,
                               const hik_scan::Pose6D& second) {
    const cv::Matx44d firstTransform =
        hik_calibration::fairinoBaseFromFlange(
            0.0, 0.0, 0.0, first.rx, first.ry, first.rz);
    const cv::Matx44d secondTransform =
        hik_calibration::fairinoBaseFromFlange(
            0.0, 0.0, 0.0, second.rx, second.ry, second.rz);
    return hik_calibration::rigidRotationDistanceDeg(
        firstTransform, secondTransform);
}

double transitionTime(const hik_scan::Pose6D& first,
                      const hik_scan::Pose6D& second) {
    const double translationSeconds = poseDistanceMm(first, second) / 100.0;
    const double rotationSeconds =
        poseRotationDistanceDeg(first, second) / 30.0;
    return std::max(translationSeconds, rotationSeconds) + 0.20;
}

bool safeCoordinate(double value, double voxelSize,
                    std::int64_t* coordinate) {
    if (!coordinate || !finiteNumber(value) ||
        !finiteNumber(voxelSize) || voxelSize <= 0.0) {
        return false;
    }
    const double scaled = std::floor(value / voxelSize);
    const double minimum = static_cast<double>(
        std::numeric_limits<std::int64_t>::min() + 1);
    const double maximum = static_cast<double>(
        std::numeric_limits<std::int64_t>::max() - 1);
    if (!finiteNumber(scaled) || scaled < minimum || scaled > maximum) {
        return false;
    }
    *coordinate = static_cast<std::int64_t>(scaled);
    return true;
}

bool keyForPoint(const cv::Point3d& point, double voxelSize,
                 VoxelKey* key) {
    return key && finitePoint(point) &&
           safeCoordinate(point.x, voxelSize, &key->x) &&
           safeCoordinate(point.y, voxelSize, &key->y) &&
           safeCoordinate(point.z, voxelSize, &key->z);
}

cv::Point3d centerForKey(const VoxelKey& key, double voxelSize) {
    return cv::Point3d(
        (static_cast<double>(key.x) + 0.5) * voxelSize,
        (static_cast<double>(key.y) + 0.5) * voxelSize,
        (static_cast<double>(key.z) + 0.5) * voxelSize);
}

std::uint64_t observationWeight(const hik_scan::CloudPoint& point) {
    return std::max<std::uint32_t>(1U, point.observationCount);
}

bool hasCloudFlag(std::uint32_t flags,
                  hik_scan::CloudPointQualityFlag flag) {
    return (flags & static_cast<std::uint32_t>(flag)) != 0U;
}

void accumulateBitCounts(
        std::uint32_t flags,
        std::uint64_t weight,
        std::array<std::uint64_t, 32>* counts) {
    if (!counts) return;
    for (std::size_t bit = 0U; bit < counts->size(); ++bit) {
        if ((flags & (1U << bit)) != 0U) {
            (*counts)[bit] += weight;
        }
    }
}

cv::Vec3d normalized(const cv::Vec3d& input,
                     const cv::Vec3d& fallback) {
    const double length = cv::norm(input);
    if (!finiteNumber(length) || length < 1.0e-9) return fallback;
    return input * (1.0 / length);
}

cv::Vec3d rotateAroundAxis(const cv::Vec3d& vector,
                           const cv::Vec3d& axis,
                           double angleDeg) {
    const cv::Vec3d unitAxis =
        normalized(axis, cv::Vec3d(0.0, 0.0, 1.0));
    const double angle = angleDeg * kPi / 180.0;
    return vector * std::cos(angle) +
           unitAxis.cross(vector) * std::sin(angle) +
           unitAxis * unitAxis.dot(vector) *
               (1.0 - std::cos(angle));
}

cv::Matx33d rotationOf(const cv::Matx44d& transform) {
    return cv::Matx33d(
        transform(0, 0), transform(0, 1), transform(0, 2),
        transform(1, 0), transform(1, 1), transform(1, 2),
        transform(2, 0), transform(2, 1), transform(2, 2));
}

cv::Vec3d translationOf(const cv::Matx44d& transform) {
    return cv::Vec3d(transform(0, 3), transform(1, 3),
                     transform(2, 3));
}

cv::Matx44d rigidInverse(const cv::Matx44d& transform) {
    const cv::Matx33d rotation = rotationOf(transform);
    const cv::Matx33d inverseRotation = rotation.t();
    const cv::Vec3d translation = translationOf(transform);
    const cv::Vec3d inverseTranslation =
        -(inverseRotation * translation);
    return cv::Matx44d(
        inverseRotation(0, 0), inverseRotation(0, 1),
        inverseRotation(0, 2), inverseTranslation[0],
        inverseRotation(1, 0), inverseRotation(1, 1),
        inverseRotation(1, 2), inverseTranslation[1],
        inverseRotation(2, 0), inverseRotation(2, 1),
        inverseRotation(2, 2), inverseTranslation[2],
        0.0, 0.0, 0.0, 1.0);
}

cv::Matx44d transformFromRotationTranslation(
        const cv::Matx33d& rotation,
        const cv::Vec3d& translation) {
    return cv::Matx44d(
        rotation(0, 0), rotation(0, 1), rotation(0, 2),
        translation[0],
        rotation(1, 0), rotation(1, 1), rotation(1, 2),
        translation[1],
        rotation(2, 0), rotation(2, 1), rotation(2, 2),
        translation[2],
        0.0, 0.0, 0.0, 1.0);
}

bool poseFromTransform(const cv::Matx44d& transform,
                       hik_scan::Pose6D* pose) {
    if (!pose) return false;
    for (int index = 0; index < 16; ++index) {
        if (!finiteNumber(transform.val[index])) return false;
    }
    const double r20 = transform(2, 0);
    const double clamped = std::max(-1.0, std::min(1.0, -r20));
    const double ry = std::asin(clamped);
    const double cy = std::cos(ry);
    double rx = 0.0;
    double rz = 0.0;
    if (std::abs(cy) > 1.0e-8) {
        rx = std::atan2(transform(2, 1), transform(2, 2));
        rz = std::atan2(transform(1, 0), transform(0, 0));
    } else {
        rx = 0.0;
        rz = std::atan2(-transform(0, 1), transform(1, 1));
    }
    pose->x = transform(0, 3);
    pose->y = transform(1, 3);
    pose->z = transform(2, 3);
    pose->rx = wrapDegrees(rx * 180.0 / kPi);
    pose->ry = wrapDegrees(ry * 180.0 / kPi);
    pose->rz = wrapDegrees(rz * 180.0 / kPi);
    return finitePose(*pose);
}

cv::Matx33d lookAtRotation(const cv::Vec3d& forward,
                           const cv::Vec3d& preferredX) {
    const cv::Vec3d z =
        normalized(forward, cv::Vec3d(0.0, 0.0, 1.0));
    cv::Vec3d x = preferredX - z * preferredX.dot(z);
    if (cv::norm(x) < 1.0e-8) {
        const cv::Vec3d alternate =
            std::abs(z[2]) < 0.9
                ? cv::Vec3d(0.0, 0.0, 1.0)
                : cv::Vec3d(1.0, 0.0, 0.0);
        x = alternate - z * alternate.dot(z);
    }
    x = normalized(x, cv::Vec3d(1.0, 0.0, 0.0));
    const cv::Vec3d y =
        normalized(z.cross(x), cv::Vec3d(0.0, 1.0, 0.0));
    x = normalized(y.cross(z), x);
    return cv::Matx33d(
        x[0], y[0], z[0],
        x[1], y[1], z[1],
        x[2], y[2], z[2]);
}

double roiExtentAlong(const RescanRoi& roi,
                      const cv::Vec3d& direction) {
    const cv::Vec3d unit =
        normalized(direction, cv::Vec3d(1.0, 0.0, 0.0));
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();
    for (int mask = 0; mask < 8; ++mask) {
        const cv::Vec3d corner(
            (mask & 1) ? roi.maximumMm.x : roi.minimumMm.x,
            (mask & 2) ? roi.maximumMm.y : roi.minimumMm.y,
            (mask & 4) ? roi.maximumMm.z : roi.minimumMm.z);
        const double projection = unit.dot(corner);
        minimum = std::min(minimum, projection);
        maximum = std::max(maximum, projection);
    }
    return maximum - minimum;
}

double informationGain(const CandidateAction& action,
                       const PlanningWeights& weights) {
    return weights.coverage * action.predictedCoverageGain +
           weights.uncertainty * action.predictedUncertaintyGain +
           weights.quality * action.predictedQualityGain +
           weights.viewDiversity *
               action.predictedViewDiversityGain;
}

bool safetyAllowed(const CandidateAction& action,
                   const SearchOptions& options) {
    const bool opticalFailed =
        action.fov == VerificationState::Failed ||
        action.laserSweep == VerificationState::Failed ||
        action.occlusion == VerificationState::Failed;
    const bool opticalVerified =
        action.fov == VerificationState::Passed &&
        action.laserSweep == VerificationState::Passed &&
        action.occlusion == VerificationState::Passed;
    if (action.robot.failed() || opticalFailed) return false;
    if (options.requireFullyVerifiedForExecution &&
        !options.allowUnverifiedForDryRun &&
        (!action.robot.fullyVerified() || !opticalVerified)) {
        return false;
    }
    return (action.robot.fullyVerified() && opticalVerified) ||
           options.allowUnverifiedForDryRun;
}

bool observabilityFullyVerified(const CandidateAction& action) {
    return action.fov == VerificationState::Passed &&
           action.laserSweep == VerificationState::Passed &&
           action.occlusion == VerificationState::Passed;
}

bool pointObservableDuringSegment(
        const cv::Point3d& basePoint,
        const ScanSegment& segment,
        const CandidateGenerationContext& context,
        bool* fovVisible,
        bool* laserVisible) {
    if (!fovVisible || !laserVisible) return false;
    *fovVisible = false;
    *laserVisible = false;
    const cv::Rect roi = context.softwareRoi.empty()
        ? cv::Rect(0, 0, context.calibratedImageSize.width,
                   context.calibratedImageSize.height)
        : context.softwareRoi;
    const int sampleCount = 41;
    for (int sample = 0; sample < sampleCount; ++sample) {
        const double ratio = static_cast<double>(sample) /
            static_cast<double>(sampleCount - 1);
        hik_scan::Pose6D flange = segment.start;
        flange.x = segment.start.x +
            ratio * (segment.end.x - segment.start.x);
        flange.y = segment.start.y +
            ratio * (segment.end.y - segment.start.y);
        flange.z = segment.start.z +
            ratio * (segment.end.z - segment.start.z);
        const cv::Matx44d baseFromFlange =
            hik_calibration::fairinoBaseFromFlange(
                flange.x, flange.y, flange.z,
                flange.rx, flange.ry, flange.rz);
        const cv::Matx44d cameraFromBase =
            rigidInverse(baseFromFlange *
                         context.flangeFromCamera);
        const cv::Vec4d pointBase(
            basePoint.x, basePoint.y, basePoint.z, 1.0);
        const cv::Vec4d pointCamera = cameraFromBase * pointBase;
        if (!finiteNumber(pointCamera[0]) ||
            !finiteNumber(pointCamera[1]) ||
            !finiteNumber(pointCamera[2]) ||
            pointCamera[2] < context.validDepthMinimumMm ||
            pointCamera[2] > context.validDepthMaximumMm) {
            continue;
        }
        const double normalizedX =
            pointCamera[0] / pointCamera[2];
        const double normalizedY =
            pointCamera[1] / pointCamera[2];
        const double u =
            context.cameraMatrix(0, 0) * normalizedX +
            context.cameraMatrix(0, 1) * normalizedY +
            context.cameraMatrix(0, 2);
        const double v =
            context.cameraMatrix(1, 0) * normalizedX +
            context.cameraMatrix(1, 1) * normalizedY +
            context.cameraMatrix(1, 2);
        if (!finiteNumber(u) || !finiteNumber(v) ||
            u < roi.x || v < roi.y ||
            u >= roi.x + roi.width ||
            v >= roi.y + roi.height) {
            continue;
        }
        *fovVisible = true;
        const double planeDistance = std::abs(
            context.laserPlaneNormalCamera.dot(
                cv::Vec3d(pointCamera[0],
                          pointCamera[1],
                          pointCamera[2])) +
            context.laserPlaneDMm);
        if (finiteNumber(planeDistance) &&
            planeDistance <= context.laserPlaneToleranceMm) {
            *laserVisible = true;
            return true;
        }
    }
    return true;
}

std::string jsonEscape(const std::string& input) {
    std::ostringstream output;
    for (char character : input) {
        switch (character) {
        case '\\': output << "\\\\"; break;
        case '"': output << "\\\""; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (static_cast<unsigned char>(character) < 0x20U) {
                output << "\\u"
                       << std::hex << std::setw(4)
                       << std::setfill('0')
                       << static_cast<int>(
                              static_cast<unsigned char>(character))
                       << std::dec << std::setfill(' ');
            } else {
                output << character;
            }
        }
    }
    return output.str();
}

std::string jsonNumber(double value) {
    if (!std::isfinite(value)) return "null";
    std::ostringstream output;
    output << std::setprecision(12) << value;
    return output.str();
}

}  // namespace

ScanSegment::ScanSegment()
    : segmentId(-1), roiId(-1), kind(SegmentKind::Measurement),
      primitive(MotionPrimitive::Line), arcRadiusMm(0.0),
      blendRadiusMm(0.0), allowLineFallback(false),
      usedLineFallback(false),
      speedMmS(0.0), accelerationMmS2(0.0), exposureUs(0.0),
      leadInMm(0.0), leadOutMm(0.0),
      reverseDirection(false), actualExecutionTimeS(0.0) {}

ScanPlan::ScanPlan()
    : profileId("scanner_650"), measurementLengthMm(0.0),
      transitionLengthMm(0.0), estimatedExecutionTimeS(0.0),
      actualExecutionTimeS(0.0),
      safetyVerified(false), executable(false),
      safetyDetail("robot path safety has not been evaluated") {}

SerpentineOptions::SerpentineOptions()
    : laneOffsetMm(0.0, 10.0, 0.0), laneCount(1),
      measurementSpeedMmS(10.0), transitionSpeedMmS(20.0),
      accelerationMmS2(100.0), exposureUs(1825.0),
      leadInMm(30.0), leadOutMm(30.0),
      enableArcTransitions(true), minimumArcRadiusMm(5.0),
      maximumArcRadiusMm(100.0), transitionBlendRadiusMm(2.0),
      maximumArcTangentErrorDeg(5.0),
      maximumSegmentCount(199), maximumTotalLengthMm(5000.0) {}

double estimateTrapezoidalMoveTime(double lengthMm,
                                   double speedMmS,
                                   double accelerationMmS2) {
    if (!finiteNumber(lengthMm) || !finiteNumber(speedMmS) ||
        !finiteNumber(accelerationMmS2) || lengthMm < 0.0 ||
        speedMmS <= 0.0 || accelerationMmS2 <= 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    if (lengthMm == 0.0) return 0.0;
    const double accelerationDistance =
        speedMmS * speedMmS / accelerationMmS2;
    if (lengthMm <= accelerationDistance) {
        return 2.0 * std::sqrt(lengthMm / accelerationMmS2);
    }
    return 2.0 * speedMmS / accelerationMmS2 +
           (lengthMm - accelerationDistance) / speedMmS;
}

double segmentMotionLengthMm(const ScanSegment& segment) {
    if (!finitePose(segment.motionStart) ||
        !finitePose(segment.motionEnd)) {
        return std::numeric_limits<double>::infinity();
    }
    if (segment.primitive == MotionPrimitive::Line) {
        return poseDistanceMm(
            segment.motionStart, segment.motionEnd);
    }
    ArcGeometry geometry;
    if (!computeArcGeometry(
            segment.motionStart, segment.arcVia,
            segment.motionEnd, &geometry)) {
        return std::numeric_limits<double>::infinity();
    }
    return geometry.radiusMm * std::abs(geometry.sweepRad);
}

bool sampleSegmentMotion(
        const ScanSegment& segment,
        double maximumCartesianStepMm,
        double maximumAngularStepDeg,
        std::size_t maximumSampleCount,
        std::vector<hik_scan::Pose6D>* samples,
        std::string* error) {
    if (!samples) {
        setError("segment sample output is null", error);
        return false;
    }
    samples->clear();
    if (!finitePose(segment.motionStart) ||
        !finitePose(segment.motionEnd) ||
        !finiteNumber(maximumCartesianStepMm) ||
        maximumCartesianStepMm <= 0.0 ||
        !finiteNumber(maximumAngularStepDeg) ||
        maximumAngularStepDeg <= 0.0 ||
        maximumSampleCount < 2U) {
        setError("segment sampling parameters are invalid", error);
        return false;
    }
    samples->push_back(segment.motionStart);
    if (segment.primitive == MotionPrimitive::Line) {
        bool sampled = false;
        if (segment.kind == SegmentKind::Measurement &&
            finitePose(segment.start) && finitePose(segment.end)) {
            sampled = appendLinearPoseSamples(
                segment.motionStart, segment.start,
                maximumCartesianStepMm, maximumAngularStepDeg,
                maximumSampleCount, samples) &&
                appendLinearPoseSamples(
                    segment.start, segment.end,
                    maximumCartesianStepMm, maximumAngularStepDeg,
                    maximumSampleCount, samples) &&
                appendLinearPoseSamples(
                    segment.end, segment.motionEnd,
                    maximumCartesianStepMm, maximumAngularStepDeg,
                    maximumSampleCount, samples);
        } else {
            sampled = appendLinearPoseSamples(
                segment.motionStart, segment.motionEnd,
                maximumCartesianStepMm, maximumAngularStepDeg,
                maximumSampleCount, samples);
        }
        if (!sampled) {
            samples->clear();
            setError("line sampling exceeds configured sample limit",
                     error);
            return false;
        }
        if (error) error->clear();
        return true;
    }
    if (!finitePose(segment.arcVia)) {
        samples->clear();
        setError("arc via pose is invalid", error);
        return false;
    }
    ArcGeometry geometry;
    if (!computeArcGeometry(
            segment.motionStart, segment.arcVia,
            segment.motionEnd, &geometry)) {
        samples->clear();
        setError("arc start/via/end do not define a valid circle",
                 error);
        return false;
    }
    const double arcLength =
        geometry.radiusMm * std::abs(geometry.sweepRad);
    const std::size_t translationIntervals =
        static_cast<std::size_t>(std::ceil(
            arcLength / maximumCartesianStepMm));
    const std::size_t rotationIntervals =
        static_cast<std::size_t>(std::ceil(
            poseRotationDistanceDeg(
                segment.motionStart, segment.motionEnd) /
            maximumAngularStepDeg));
    const std::size_t intervals = std::max<std::size_t>(
        2U, std::max(translationIntervals, rotationIntervals));
    if (1U + intervals > maximumSampleCount) {
        samples->clear();
        setError("arc sampling exceeds configured sample limit",
                 error);
        return false;
    }
    for (std::size_t index = 1U; index <= intervals; ++index) {
        const double ratio =
            static_cast<double>(index) /
            static_cast<double>(intervals);
        const double angle = geometry.sweepRad * ratio;
        const cv::Vec3d position =
            geometry.center +
            geometry.radiusMm *
                (geometry.radialStart * std::cos(angle) +
                 geometry.radialQuarter * std::sin(angle));
        hik_scan::Pose6D pose = interpolatePose(
            segment.motionStart, segment.motionEnd, ratio);
        pose.x = position[0];
        pose.y = position[1];
        pose.z = position[2];
        samples->push_back(pose);
    }
    if (error) error->clear();
    return true;
}

bool buildSerpentinePlan(const SerpentineOptions& options,
                         ScanPlan* plan,
                         std::string* error) {
    if (!plan) {
        setError("serpentine plan output is null", error);
        return false;
    }
    *plan = ScanPlan();
    if (!finitePose(options.firstLaneStart) ||
        !finitePose(options.firstLaneEnd) ||
        !finiteVector(options.laneOffsetMm) ||
        options.laneCount < 1 ||
        !finiteNumber(options.measurementSpeedMmS) ||
        options.measurementSpeedMmS <= 0.0 ||
        !finiteNumber(options.transitionSpeedMmS) ||
        options.transitionSpeedMmS <= 0.0 ||
        !finiteNumber(options.accelerationMmS2) ||
        options.accelerationMmS2 <= 0.0 ||
        !finiteNumber(options.exposureUs) || options.exposureUs <= 0.0 ||
        !finiteNumber(options.leadInMm) || options.leadInMm < 0.0 ||
        !finiteNumber(options.leadOutMm) || options.leadOutMm < 0.0 ||
        !finiteNumber(options.minimumArcRadiusMm) ||
        options.minimumArcRadiusMm <= 0.0 ||
        !finiteNumber(options.maximumArcRadiusMm) ||
        options.maximumArcRadiusMm < options.minimumArcRadiusMm ||
        !finiteNumber(options.transitionBlendRadiusMm) ||
        options.transitionBlendRadiusMm < 0.0 ||
        !finiteNumber(options.maximumArcTangentErrorDeg) ||
        options.maximumArcTangentErrorDeg < 0.0 ||
        options.maximumArcTangentErrorDeg > 30.0 ||
        options.maximumSegmentCount < 1 ||
        !finiteNumber(options.maximumTotalLengthMm) ||
        options.maximumTotalLengthMm <= 0.0) {
        setError("serpentine options are invalid", error);
        return false;
    }
    const double laneLength =
        poseDistanceMm(options.firstLaneStart,
                       options.firstLaneEnd);
    const double offsetLength = cv::norm(options.laneOffsetMm);
    if (laneLength < kMinimumSegmentLengthMm) {
        setError("serpentine first lane is too short", error);
        return false;
    }
    if (options.laneCount > 1 &&
        (!finiteNumber(offsetLength) ||
         offsetLength < kMinimumSegmentLengthMm)) {
        setError("serpentine lane offset is too small", error);
        return false;
    }
    const int requiredSegments =
        options.laneCount + std::max(0, options.laneCount - 1);
    if (requiredSegments > options.maximumSegmentCount) {
        setError("serpentine segment count exceeds configured limit",
                 error);
        return false;
    }

    plan->segments.reserve(
        static_cast<std::size_t>(requiredSegments));
    hik_scan::Pose6D previousEnd;
    cv::Vec3d previousDirection(1.0, 0.0, 0.0);
    bool hasPrevious = false;
    int segmentId = 0;
    for (int lane = 0; lane < options.laneCount; ++lane) {
        const cv::Vec3d offset =
            static_cast<double>(lane) * options.laneOffsetMm;
        hik_scan::Pose6D low = options.firstLaneStart;
        hik_scan::Pose6D high = options.firstLaneEnd;
        low.x += offset[0]; low.y += offset[1]; low.z += offset[2];
        high.x += offset[0]; high.y += offset[1]; high.z += offset[2];
        low.rx = high.rx = options.firstLaneStart.rx;
        low.ry = high.ry = options.firstLaneStart.ry;
        low.rz = high.rz = options.firstLaneStart.rz;
        const bool reverse = (lane % 2) != 0;
        const hik_scan::Pose6D laneStart = reverse ? high : low;
        const hik_scan::Pose6D laneEnd = reverse ? low : high;

        const cv::Vec3d measurementDirection = normalized(
            cv::Vec3d(laneEnd.x - laneStart.x,
                      laneEnd.y - laneStart.y,
                      laneEnd.z - laneStart.z),
            cv::Vec3d(1.0, 0.0, 0.0));
        hik_scan::Pose6D motionStart = laneStart;
        hik_scan::Pose6D motionEnd = laneEnd;
        motionStart.x -= options.leadInMm * measurementDirection[0];
        motionStart.y -= options.leadInMm * measurementDirection[1];
        motionStart.z -= options.leadInMm * measurementDirection[2];
        motionEnd.x += options.leadOutMm * measurementDirection[0];
        motionEnd.y += options.leadOutMm * measurementDirection[1];
        motionEnd.z += options.leadOutMm * measurementDirection[2];

        if (hasPrevious) {
            ScanSegment transition;
            transition.segmentId = segmentId++;
            transition.kind = SegmentKind::Transition;
            transition.start = previousEnd;
            transition.end = motionStart;
            transition.motionStart = transition.start;
            transition.motionEnd = transition.end;
            transition.allowLineFallback =
                options.enableArcTransitions;
            transition.speedMmS = options.transitionSpeedMmS;
            transition.accelerationMmS2 = options.accelerationMmS2;
            transition.exposureUs = options.exposureUs;
            const cv::Vec3d chord(
                transition.end.x - transition.start.x,
                transition.end.y - transition.start.y,
                transition.end.z - transition.start.z);
            const double chordLength = cv::norm(chord);
            const double turnRadius = 0.5 * chordLength;
            const cv::Vec3d chordDirection =
                normalized(chord, cv::Vec3d(0.0, 1.0, 0.0));
            const double tangentTolerance =
                std::sin(options.maximumArcTangentErrorDeg *
                         kPi / 180.0);
            const double reverseTolerance =
                std::cos(options.maximumArcTangentErrorDeg *
                         kPi / 180.0);
            const bool tangentGeometry =
                chordLength >= 2.0 * options.minimumArcRadiusMm &&
                turnRadius <= options.maximumArcRadiusMm &&
                std::abs(previousDirection.dot(chordDirection)) <=
                    tangentTolerance &&
                std::abs(measurementDirection.dot(chordDirection)) <=
                    tangentTolerance &&
                previousDirection.dot(measurementDirection) <=
                    -reverseTolerance;
            if (options.enableArcTransitions && tangentGeometry) {
                transition.primitive = MotionPrimitive::Arc;
                transition.arcRadiusMm = turnRadius;
                transition.arcVia = interpolatePose(
                    transition.motionStart,
                    transition.motionEnd, 0.5);
                transition.arcVia.x +=
                    turnRadius * previousDirection[0];
                transition.arcVia.y +=
                    turnRadius * previousDirection[1];
                transition.arcVia.z +=
                    turnRadius * previousDirection[2];
                transition.blendRadiusMm = std::min(
                    options.transitionBlendRadiusMm,
                    0.5 * turnRadius);
            } else {
                transition.primitive = MotionPrimitive::Line;
                transition.usedLineFallback =
                    options.enableArcTransitions;
                transition.fallbackReason =
                    options.enableArcTransitions
                    ? "arc radius/tangency geometry is invalid; "
                      "use stopped line transition"
                    : "arc transitions are disabled";
            }
            const double transitionLength =
                segmentMotionLengthMm(transition);
            plan->transitionLengthMm +=
                transitionLength;
            plan->estimatedExecutionTimeS +=
                estimateTrapezoidalMoveTime(
                    transitionLength,
                    transition.speedMmS,
                    transition.accelerationMmS2);
            plan->segments.push_back(transition);
        }

        ScanSegment measurement;
        measurement.segmentId = segmentId++;
        measurement.kind = SegmentKind::Measurement;
        measurement.start = laneStart;
        measurement.end = laneEnd;
        measurement.motionStart = motionStart;
        measurement.motionEnd = motionEnd;
        measurement.speedMmS = options.measurementSpeedMmS;
        measurement.accelerationMmS2 = options.accelerationMmS2;
        measurement.exposureUs = options.exposureUs;
        measurement.leadInMm = options.leadInMm;
        measurement.leadOutMm = options.leadOutMm;
        measurement.reverseDirection = reverse;
        plan->measurementLengthMm += laneLength;
        plan->estimatedExecutionTimeS +=
            estimateTrapezoidalMoveTime(
                laneLength + options.leadInMm + options.leadOutMm,
                measurement.speedMmS,
                measurement.accelerationMmS2);
        plan->segments.push_back(measurement);
        previousEnd = motionEnd;
        previousDirection = measurementDirection;
        hasPrevious = true;
    }
    bool hasArcTransition = false;
    bool hasLineTransition = false;
    for (std::size_t index = 1U;
         index + 1U < plan->segments.size(); index += 2U) {
        if (plan->segments[index].primitive == MotionPrimitive::Arc) {
            hasArcTransition = true;
        } else {
            hasLineTransition = true;
        }
    }
    if (hasArcTransition && hasLineTransition) {
        for (std::size_t index = 1U;
             index + 1U < plan->segments.size(); index += 2U) {
            ScanSegment& transition = plan->segments[index];
            if (transition.primitive != MotionPrimitive::Arc) continue;
            const double arcLength =
                segmentMotionLengthMm(transition);
            const double arcTime = estimateTrapezoidalMoveTime(
                arcLength, transition.speedMmS,
                transition.accelerationMmS2);
            transition.primitive = MotionPrimitive::Line;
            transition.arcRadiusMm = 0.0;
            transition.blendRadiusMm = 0.0;
            transition.usedLineFallback = true;
            transition.fallbackReason =
                "another lane transition failed arc geometry; "
                "use one uniform stopped-line trajectory";
            const double lineLength =
                segmentMotionLengthMm(transition);
            const double lineTime = estimateTrapezoidalMoveTime(
                lineLength, transition.speedMmS,
                transition.accelerationMmS2);
            plan->transitionLengthMm += lineLength - arcLength;
            plan->estimatedExecutionTimeS += lineTime - arcTime;
        }
    }
    for (std::size_t index = 1U;
         index + 1U < plan->segments.size(); index += 2U) {
        ScanSegment& transition = plan->segments[index];
        if (transition.primitive != MotionPrimitive::Arc ||
            transition.blendRadiusMm <= 0.0) {
            transition.blendRadiusMm = 0.0;
            plan->segments[index - 1U].blendRadiusMm = 0.0;
            continue;
        }
        plan->segments[index - 1U].blendRadiusMm =
            transition.blendRadiusMm;
    }
    if (!plan->segments.empty()) {
        plan->segments.back().blendRadiusMm = 0.0;
    }
    const double totalLength =
        plan->measurementLengthMm + plan->transitionLengthMm +
        static_cast<double>(options.laneCount) *
            (options.leadInMm + options.leadOutMm);
    if (totalLength > options.maximumTotalLengthMm) {
        *plan = ScanPlan();
        setError("serpentine total length exceeds configured limit",
                 error);
        return false;
    }
    plan->safetyDetail =
        "geometry generated; IK/collision/singularity are not verified";
    if (error) error->clear();
    return true;
}

bool VoxelKey::operator<(const VoxelKey& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
}

bool VoxelKey::operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
}

QualityVoxel::QualityVoxel()
    : key{0, 0, 0}, centerMm(0.0, 0.0, 0.0),
      evidenceCentroidMm(0.0, 0.0, 0.0),
      formalAcceptedObservationCount(0U),
      qualityAcceptedObservationCount(0U),
      rejectedObservationCount(0U), expectedObservationCount(0U),
      meanConfidence(0.0), meanSnr(0.0),
      meanSaturatedFraction(0.0), meanFwhmPx(0.0),
      meanObservedViewDirectionBase(0.0, 0.0, 0.0),
      viewObservationCount(0U),
      cloudQualityFlags(0U), stripeRejectFlags(0U),
      cloudFlagCounts{}, stripeRejectCounts{}, stateFlags(0U),
      severity(0.0) {}

bool QualityVoxel::needsRescan() const {
    return (stateFlags & QUALITY_VOXEL_NEEDS_RESCAN) != 0U;
}

QualityMapOptions::QualityMapOptions()
    : voxelSizeMm(2.0), minimumMeanConfidence(0.35),
      rejectedRatioThreshold(0.30), minimumAcceptedObservations(1U),
      rescanAnyRejectedEvidence(false),
      rescanUnknownOpticalEvidence(false),
      severeCloudQualityMask(
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT) |
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_REJECTED_V_GROOVE_AMBIGUOUS) |
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT) |
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_REJECTED_V_GROOVE_GEOMETRY) |
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE) |
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_REJECTED_SHADOW_LEGACY_MULTIPATH) |
          static_cast<std::uint32_t>(
              hik_scan::CLOUD_QUALITY_REJECTED_PROFILE_PUBLICATION_GATE)),
      severeStripeRejectMask(
          hik_stripe::REJECT_SATURATED_WIDE_PLATEAU |
          hik_stripe::REJECT_SATURATED_ASYMMETRIC |
          hik_stripe::REJECT_PATH_AMBIGUOUS |
          hik_stripe::REJECT_AMBIGUOUS_MULTIPATH) {}

QualityMap::QualityMap()
    : voxelSizeMm(0.0), formalAcceptedPointCount(0U),
      qualityAcceptedPointCount(0U), rejectedPointCount(0U),
      expectedPointCount(0U), rescanVoxelCount(0U) {}

QualityObservation::QualityObservation()
    : role(ObservationRole::FormalAccepted), frameId(0U),
      cameraOriginBaseMm(0.0, 0.0, 0.0),
      hasCameraOrigin(false) {}

bool buildQualityMap(
        const std::vector<QualityObservation>& observations,
        const std::vector<cv::Point3d>& expectedSurfacePoints,
        const QualityMapOptions& options,
        QualityMap* map,
        std::string* error) {
    if (!map) {
        setError("quality map output is null", error);
        return false;
    }
    *map = QualityMap();
    if (!finiteNumber(options.voxelSizeMm) ||
        options.voxelSizeMm <= 0.0 ||
        !finiteNumber(options.minimumMeanConfidence) ||
        options.minimumMeanConfidence < 0.0 ||
        options.minimumMeanConfidence > 1.0 ||
        !finiteNumber(options.rejectedRatioThreshold) ||
        options.rejectedRatioThreshold < 0.0 ||
        options.rejectedRatioThreshold > 1.0 ||
        options.minimumAcceptedObservations < 1U) {
        setError("quality map options are invalid", error);
        return false;
    }
    map->voxelSizeMm = options.voxelSizeMm;

    struct Accumulator {
        cv::Vec3d formalPositionSum{0.0, 0.0, 0.0};
        cv::Vec3d fallbackPositionSum{0.0, 0.0, 0.0};
        std::uint64_t fallbackPositionCount{0U};
        long double confidenceSum{0.0L};
        std::uint64_t confidenceCount{0U};
        long double snrSum{0.0L};
        long double saturationSum{0.0L};
        long double fwhmSum{0.0L};
        std::uint64_t opticalCount{0U};
        cv::Vec3d viewDirectionSum{0.0, 0.0, 0.0};
        std::uint64_t viewDirectionCount{0U};
    };
    std::map<VoxelKey, Accumulator> accumulators;

    for (const QualityObservation& observation : observations) {
        const hik_scan::CloudPoint& point = observation.point;
        VoxelKey key;
        if (!keyForPoint(
                point.basePointMm, options.voxelSizeMm, &key)) {
            setError("quality observation contains an invalid base point",
                     error);
            *map = QualityMap();
            return false;
        }
        QualityVoxel& voxel = map->voxels[key];
        voxel.key = key;
        voxel.centerMm = centerForKey(key, options.voxelSizeMm);
        Accumulator& accumulator = accumulators[key];
        const std::uint64_t weight = observationWeight(point);
        accumulator.fallbackPositionSum +=
            static_cast<double>(weight) * cv::Vec3d(
                point.basePointMm.x, point.basePointMm.y,
                point.basePointMm.z);
        accumulator.fallbackPositionCount += weight;
        voxel.cloudQualityFlags |= point.qualityFlags;
        voxel.stripeRejectFlags |= point.stripeRejectFlags;
        accumulateBitCounts(point.qualityFlags, weight,
                            &voxel.cloudFlagCounts);
        accumulateBitCounts(point.stripeRejectFlags, weight,
                            &voxel.stripeRejectCounts);
        if (finiteNumber(point.confidence)) {
            accumulator.confidenceSum +=
                static_cast<long double>(point.confidence) * weight;
            accumulator.confidenceCount += weight;
        }
        if (point.opticalMetricsValid && finiteNumber(point.snr) &&
            finiteNumber(point.saturatedFraction) &&
            finiteNumber(point.fwhmPx)) {
            accumulator.snrSum +=
                static_cast<long double>(point.snr) * weight;
            accumulator.saturationSum +=
                static_cast<long double>(
                    point.saturatedFraction) * weight;
            accumulator.fwhmSum +=
                static_cast<long double>(point.fwhmPx) * weight;
            accumulator.opticalCount += weight;
        }
        switch (observation.role) {
        case ObservationRole::FormalAccepted:
            voxel.formalAcceptedObservationCount += weight;
            voxel.formalAcceptedProfiles.insert(point.profileIndex);
            accumulator.formalPositionSum +=
                static_cast<double>(weight) * cv::Vec3d(
                    point.basePointMm.x, point.basePointMm.y,
                    point.basePointMm.z);
            map->formalAcceptedPointCount +=
                static_cast<std::size_t>(weight);
            if (observation.hasCameraOrigin &&
                finitePoint(observation.cameraOriginBaseMm)) {
                const cv::Vec3d viewDirection =
                    cv::Vec3d(
                        observation.cameraOriginBaseMm.x -
                            point.basePointMm.x,
                        observation.cameraOriginBaseMm.y -
                            point.basePointMm.y,
                        observation.cameraOriginBaseMm.z -
                            point.basePointMm.z);
                if (cv::norm(viewDirection) > 1.0e-9) {
                    accumulator.viewDirectionSum +=
                        static_cast<double>(weight) *
                        normalized(
                            viewDirection,
                            cv::Vec3d(0.0, 0.0, 1.0));
                    accumulator.viewDirectionCount += weight;
                }
            }
            break;
        case ObservationRole::QualityAccepted:
            voxel.qualityAcceptedObservationCount += weight;
            voxel.qualityAcceptedProfiles.insert(point.profileIndex);
            map->qualityAcceptedPointCount +=
                static_cast<std::size_t>(weight);
            break;
        case ObservationRole::Rejected:
            voxel.rejectedObservationCount += weight;
            voxel.rejectedProfiles.insert(point.profileIndex);
            map->rejectedPointCount +=
                static_cast<std::size_t>(weight);
            break;
        }
    }

    for (const cv::Point3d& expected : expectedSurfacePoints) {
        VoxelKey key;
        if (!keyForPoint(expected, options.voxelSizeMm, &key)) {
            setError("expected surface mask contains an invalid point",
                     error);
            *map = QualityMap();
            return false;
        }
        QualityVoxel& voxel = map->voxels[key];
        voxel.key = key;
        voxel.centerMm = centerForKey(key, options.voxelSizeMm);
        ++voxel.expectedObservationCount;
        ++map->expectedPointCount;
    }

    for (std::pair<const VoxelKey, QualityVoxel>& entry :
         map->voxels) {
        QualityVoxel& voxel = entry.second;
        const Accumulator& accumulator = accumulators[entry.first];
        if (voxel.formalAcceptedObservationCount > 0U) {
            const cv::Vec3d centroid =
                accumulator.formalPositionSum *
                (1.0 / static_cast<double>(
                    voxel.formalAcceptedObservationCount));
            voxel.evidenceCentroidMm =
                cv::Point3d(centroid[0], centroid[1], centroid[2]);
        } else if (accumulator.fallbackPositionCount > 0U) {
            const cv::Vec3d centroid =
                accumulator.fallbackPositionSum *
                (1.0 / static_cast<double>(
                    accumulator.fallbackPositionCount));
            voxel.evidenceCentroidMm =
                cv::Point3d(centroid[0], centroid[1], centroid[2]);
        } else {
            voxel.evidenceCentroidMm = voxel.centerMm;
        }
        if (accumulator.confidenceCount > 0U) {
            voxel.meanConfidence = static_cast<double>(
                accumulator.confidenceSum /
                accumulator.confidenceCount);
        }
        if (accumulator.opticalCount > 0U) {
            voxel.meanSnr = static_cast<double>(
                accumulator.snrSum / accumulator.opticalCount);
            voxel.meanSaturatedFraction = static_cast<double>(
                accumulator.saturationSum /
                accumulator.opticalCount);
            voxel.meanFwhmPx = static_cast<double>(
                accumulator.fwhmSum / accumulator.opticalCount);
        } else if (voxel.formalAcceptedObservationCount > 0U) {
            voxel.stateFlags |= QUALITY_VOXEL_OPTICAL_UNKNOWN;
        }
        if (accumulator.viewDirectionCount > 0U) {
            voxel.meanObservedViewDirectionBase = normalized(
                accumulator.viewDirectionSum,
                cv::Vec3d(0.0, 0.0, 1.0));
            voxel.viewObservationCount =
                accumulator.viewDirectionCount;
        }

        const std::uint64_t accepted =
            voxel.formalAcceptedObservationCount;
        const std::uint64_t classified =
            accepted + voxel.rejectedObservationCount;
        const double rejectedRatio = classified == 0U ? 0.0 :
            static_cast<double>(voxel.rejectedObservationCount) /
            static_cast<double>(classified);
        if (voxel.expectedObservationCount > 0U &&
            accepted == 0U) {
            voxel.stateFlags |= QUALITY_VOXEL_UNOBSERVED_EXPECTED;
        }
        if (voxel.rejectedObservationCount > 0U &&
            (options.rescanAnyRejectedEvidence ||
             rejectedRatio >= options.rejectedRatioThreshold)) {
            voxel.stateFlags |= QUALITY_VOXEL_REJECTED_DOMINANT;
        }
        if (accepted > 0U &&
            accumulator.confidenceCount > 0U &&
            voxel.meanConfidence < options.minimumMeanConfidence) {
            voxel.stateFlags |= QUALITY_VOXEL_LOW_CONFIDENCE;
        }
        if ((voxel.cloudQualityFlags &
             static_cast<std::uint32_t>(
                 hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE)) != 0U ||
            (voxel.cloudQualityFlags &
             static_cast<std::uint32_t>(
                 hik_scan::CLOUD_QUALITY_REJECTED_SHADOW_LEGACY_MULTIPATH)) != 0U ||
            (voxel.stripeRejectFlags &
             hik_stripe::REJECT_AMBIGUOUS_MULTIPATH) != 0U) {
            voxel.stateFlags |= QUALITY_VOXEL_MULTIPATH;
        }
        if ((voxel.stripeRejectFlags &
             (hik_stripe::REJECT_SATURATED_WIDE_PLATEAU |
              hik_stripe::REJECT_SATURATED_ASYMMETRIC)) != 0U) {
            voxel.stateFlags |= QUALITY_VOXEL_SATURATED;
        }
        if ((voxel.stripeRejectFlags &
             hik_stripe::REJECT_PATH_AMBIGUOUS) != 0U) {
            voxel.stateFlags |= QUALITY_VOXEL_PATH_AMBIGUOUS;
        }
        if (hasCloudFlag(
                voxel.cloudQualityFlags,
                hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT) ||
            hasCloudFlag(
                voxel.cloudQualityFlags,
                hik_scan::CLOUD_QUALITY_REJECTED_PROFILE_PUBLICATION_GATE)) {
            voxel.stateFlags |=
                QUALITY_VOXEL_INSUFFICIENT_SUPPORT;
        }
        if (hasCloudFlag(
                voxel.cloudQualityFlags,
                hik_scan::CLOUD_QUALITY_REJECTED_V_GROOVE_GEOMETRY)) {
            voxel.stateFlags |= QUALITY_VOXEL_INVALID_GEOMETRY;
        }
        bool needsRescan =
            accepted < options.minimumAcceptedObservations ||
            (voxel.stateFlags &
             (QUALITY_VOXEL_UNOBSERVED_EXPECTED |
              QUALITY_VOXEL_REJECTED_DOMINANT |
              QUALITY_VOXEL_LOW_CONFIDENCE)) != 0U ||
            (voxel.cloudQualityFlags &
             options.severeCloudQualityMask) != 0U ||
            (voxel.stripeRejectFlags &
             options.severeStripeRejectMask) != 0U;
        if ((voxel.stateFlags & QUALITY_VOXEL_OPTICAL_UNKNOWN) != 0U &&
            options.rescanUnknownOpticalEvidence) {
            needsRescan = true;
        }
        // A formal-only point without target-mask or rejected evidence is not
        // automatically a defect merely because old files lack optical data.
        if (accepted > 0U && voxel.expectedObservationCount == 0U &&
            voxel.rejectedObservationCount == 0U &&
            !options.rescanUnknownOpticalEvidence &&
            (voxel.cloudQualityFlags &
             options.severeCloudQualityMask) == 0U &&
            (voxel.stripeRejectFlags &
             options.severeStripeRejectMask) == 0U &&
            voxel.meanConfidence >= options.minimumMeanConfidence) {
            needsRescan = false;
        }
        if (needsRescan) {
            voxel.stateFlags |= QUALITY_VOXEL_NEEDS_RESCAN;
            ++map->rescanVoxelCount;
        }
        voxel.severity =
            ((voxel.stateFlags &
              QUALITY_VOXEL_UNOBSERVED_EXPECTED) ? 2.0 : 0.0) +
            ((voxel.stateFlags &
              QUALITY_VOXEL_REJECTED_DOMINANT) ? 1.5 : 0.0) +
            ((voxel.stateFlags &
              QUALITY_VOXEL_MULTIPATH) ? 2.0 : 0.0) +
            ((voxel.stateFlags &
              QUALITY_VOXEL_SATURATED) ? 1.5 : 0.0) +
            ((voxel.stateFlags &
              QUALITY_VOXEL_PATH_AMBIGUOUS) ? 1.5 : 0.0) +
            ((voxel.stateFlags &
              QUALITY_VOXEL_INSUFFICIENT_SUPPORT) ? 1.0 : 0.0) +
            ((voxel.stateFlags &
              QUALITY_VOXEL_INVALID_GEOMETRY) ? 1.5 : 0.0) +
            std::max(0.0, 1.0 - voxel.meanConfidence) +
            rejectedRatio;
    }
    if (error) error->clear();
    return true;
}

RescanRoi::RescanRoi()
    : roiId(-1), minimumMm(0.0, 0.0, 0.0),
      maximumMm(0.0, 0.0, 0.0), centerMm(0.0, 0.0, 0.0),
      principalAxis(1.0, 0.0, 0.0),
      secondaryAxis(0.0, 1.0, 0.0),
      estimatedNormal(0.0, 0.0, 1.0), severity(0.0),
      stateFlags(0U), cloudQualityFlags(0U),
      stripeRejectFlags(0U), formalAcceptedObservationCount(0U),
      qualityAcceptedObservationCount(0U),
      rejectedObservationCount(0U), expectedObservationCount(0U) {}

RoiClusteringOptions::RoiClusteringOptions()
    : connectivityRadiusVoxels(1), minimumVoxelCount(1U),
      paddingMm(3.0), maximumClusterExtentMm(250.0) {}

bool clusterRescanRois(const QualityMap& map,
                       const RoiClusteringOptions& options,
                       std::vector<RescanRoi>* rois,
                       std::string* error) {
    if (!rois) {
        setError("ROI output is null", error);
        return false;
    }
    rois->clear();
    if (!finiteNumber(map.voxelSizeMm) || map.voxelSizeMm <= 0.0 ||
        options.connectivityRadiusVoxels < 1 ||
        options.minimumVoxelCount < 1U ||
        !finiteNumber(options.paddingMm) || options.paddingMm < 0.0 ||
        !finiteNumber(options.maximumClusterExtentMm) ||
        options.maximumClusterExtentMm <= 0.0) {
        setError("ROI clustering options are invalid", error);
        return false;
    }
    std::set<VoxelKey> remaining;
    for (const std::pair<const VoxelKey, QualityVoxel>& entry :
         map.voxels) {
        if (entry.second.needsRescan()) remaining.insert(entry.first);
    }
    int nextRoiId = 0;
    while (!remaining.empty()) {
        const VoxelKey seed = *remaining.begin();
        remaining.erase(remaining.begin());
        std::queue<VoxelKey> frontier;
        frontier.push(seed);
        std::vector<VoxelKey> component;
        while (!frontier.empty()) {
            const VoxelKey current = frontier.front();
            frontier.pop();
            component.push_back(current);
            for (int dx = -options.connectivityRadiusVoxels;
                 dx <= options.connectivityRadiusVoxels; ++dx) {
                for (int dy = -options.connectivityRadiusVoxels;
                     dy <= options.connectivityRadiusVoxels; ++dy) {
                    for (int dz = -options.connectivityRadiusVoxels;
                         dz <= options.connectivityRadiusVoxels; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        const VoxelKey neighbor{
                            current.x + dx,
                            current.y + dy,
                            current.z + dz};
                        const std::set<VoxelKey>::iterator found =
                            remaining.find(neighbor);
                        if (found != remaining.end()) {
                            frontier.push(*found);
                            remaining.erase(found);
                        }
                    }
                }
            }
        }
        if (component.size() < options.minimumVoxelCount) continue;

        RescanRoi roi;
        roi.roiId = nextRoiId++;
        roi.voxelKeys = component;
        roi.minimumMm = cv::Point3d(
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity());
        roi.maximumMm = cv::Point3d(
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity());
        cv::Vec3d centroidSum(0.0, 0.0, 0.0);
        std::vector<cv::Point3d> formalCenters;
        for (const VoxelKey& key : component) {
            const QualityVoxel& voxel = map.voxels.at(key);
            const cv::Point3d point = voxel.evidenceCentroidMm;
            roi.minimumMm.x = std::min(roi.minimumMm.x, point.x);
            roi.minimumMm.y = std::min(roi.minimumMm.y, point.y);
            roi.minimumMm.z = std::min(roi.minimumMm.z, point.z);
            roi.maximumMm.x = std::max(roi.maximumMm.x, point.x);
            roi.maximumMm.y = std::max(roi.maximumMm.y, point.y);
            roi.maximumMm.z = std::max(roi.maximumMm.z, point.z);
            centroidSum += cv::Vec3d(point.x, point.y, point.z);
            roi.severity += voxel.severity;
            roi.stateFlags |= voxel.stateFlags;
            roi.cloudQualityFlags |= voxel.cloudQualityFlags;
            roi.stripeRejectFlags |= voxel.stripeRejectFlags;
            roi.formalAcceptedObservationCount +=
                voxel.formalAcceptedObservationCount;
            roi.qualityAcceptedObservationCount +=
                voxel.qualityAcceptedObservationCount;
            roi.rejectedObservationCount +=
                voxel.rejectedObservationCount;
            roi.expectedObservationCount +=
                voxel.expectedObservationCount;
            if (voxel.formalAcceptedObservationCount > 0U) {
                formalCenters.push_back(point);
            }
        }
        roi.centerMm = cv::Point3d(
            centroidSum[0] / component.size(),
            centroidSum[1] / component.size(),
            centroidSum[2] / component.size());
        roi.minimumMm -= cv::Point3d(
            options.paddingMm, options.paddingMm, options.paddingMm);
        roi.maximumMm += cv::Point3d(
            options.paddingMm, options.paddingMm, options.paddingMm);
        roi.severity /= static_cast<double>(component.size());
        const cv::Vec3d extent(
            roi.maximumMm.x - roi.minimumMm.x,
            roi.maximumMm.y - roi.minimumMm.y,
            roi.maximumMm.z - roi.minimumMm.z);
        if (extent[0] > options.maximumClusterExtentMm ||
            extent[1] > options.maximumClusterExtentMm ||
            extent[2] > options.maximumClusterExtentMm) {
            // Large components are unsafe to treat as one local rescan.
            continue;
        }

        // Only formal accepted geometry contributes to the local surface PCA.
        if (formalCenters.size() >= 3U) {
            cv::Mat covariance = cv::Mat::zeros(3, 3, CV_64F);
            cv::Vec3d mean(0.0, 0.0, 0.0);
            for (const cv::Point3d& point : formalCenters) {
                mean += cv::Vec3d(point.x, point.y, point.z);
            }
            mean *= 1.0 / static_cast<double>(formalCenters.size());
            for (const cv::Point3d& point : formalCenters) {
                const cv::Vec3d delta =
                    cv::Vec3d(point.x, point.y, point.z) - mean;
                covariance += cv::Mat(delta) * cv::Mat(delta).t();
            }
            covariance *= 1.0 / static_cast<double>(
                formalCenters.size() - 1U);
            cv::Mat eigenvalues;
            cv::Mat eigenvectors;
            if (cv::eigen(covariance, eigenvalues, eigenvectors) &&
                eigenvectors.rows == 3 && eigenvectors.cols == 3) {
                roi.principalAxis = normalized(
                    cv::Vec3d(eigenvectors.at<double>(0, 0),
                              eigenvectors.at<double>(0, 1),
                              eigenvectors.at<double>(0, 2)),
                    roi.principalAxis);
                roi.secondaryAxis = normalized(
                    cv::Vec3d(eigenvectors.at<double>(1, 0),
                              eigenvectors.at<double>(1, 1),
                              eigenvectors.at<double>(1, 2)),
                    roi.secondaryAxis);
                roi.estimatedNormal = normalized(
                    cv::Vec3d(eigenvectors.at<double>(2, 0),
                              eigenvectors.at<double>(2, 1),
                              eigenvectors.at<double>(2, 2)),
                    roi.estimatedNormal);
            }
        }
        rois->push_back(roi);
    }
    if (error) error->clear();
    return true;
}

RobotPathEvaluation::RobotPathEvaluation()
    : ik(VerificationState::Unknown),
      collision(VerificationState::Unknown),
      singularity(VerificationState::Unknown),
      minimumSingularValue(0.0),
      minimumJointLimitMarginDeg(0.0), jointTravelDeg(0.0),
      estimatedExecutionTimeS(0.0),
      evaluatedPrimitive(MotionPrimitive::Line),
      usedLineFallback(false),
      detail("robot feasibility has not been evaluated") {}

bool RobotPathEvaluation::fullyVerified() const {
    return ik == VerificationState::Passed &&
           collision == VerificationState::Passed &&
           singularity == VerificationState::Passed;
}

bool RobotPathEvaluation::failed() const {
    return ik == VerificationState::Failed ||
           collision == VerificationState::Failed ||
           singularity == VerificationState::Failed;
}

CandidateAction::CandidateAction()
    : actionId(-1), roiId(-1), yawOffsetDeg(0.0),
      pitchOffsetDeg(0.0), workingDistanceMm(0.0),
      predictedCoverageGain(0.0),
      predictedUncertaintyGain(0.0),
      predictedQualityGain(0.0),
      predictedViewDiversityGain(0.0),
      predictedInformationGain(0.0), transitionTimeS(0.0),
      riskPenalty(0.0), utility(
          -std::numeric_limits<double>::infinity()),
      fov(VerificationState::Unknown),
      laserSweep(VerificationState::Unknown),
      occlusion(VerificationState::Unknown),
      observabilityDetail(
          "FOV/laser sweep/occlusion have not been evaluated") {}

CandidateLibraryOptions::CandidateLibraryOptions()
    : yawOffsetsDeg{-25.0, 0.0, 25.0},
      pitchOffsetsDeg{-15.0, 0.0, 15.0},
      workingDistanceScales{0.9, 1.0, 1.1},
      speedsMmS{10.0, 20.0, 30.0},
      exposureUs{900.0, 1825.0}, includeReverseDirection(true),
      roiPaddingMm(5.0), accelerationMmS2(100.0),
      maximumCandidateCount(4096U) {}

CandidateGenerationContext::CandidateGenerationContext()
    : baseFromReferenceFlange(cv::Matx44d::eye()),
      flangeFromCamera(cv::Matx44d::eye()),
      nominalScanDirectionBase(1.0, 0.0, 0.0),
      cameraForwardAxis(0.0, 0.0, 1.0),
      cameraMatrix(cv::Matx33d::eye()),
      calibratedImageSize(0, 0), softwareRoi(),
      laserPlaneNormalCamera(0.0, 0.0, 1.0),
      laserPlaneDMm(0.0), validDepthMinimumMm(0.0),
      validDepthMaximumMm(0.0), laserPlaneToleranceMm(1.0),
      calibratedObservabilityAvailable(false),
      occlusionModelAvailable(false) {}

bool generateCandidateLibrary(
        const QualityMap& map,
        const std::vector<RescanRoi>& rois,
        const CandidateGenerationContext& context,
        const CandidateLibraryOptions& options,
        std::vector<CandidateAction>* candidates,
        std::string* error) {
    if (!candidates) {
        setError("candidate output is null", error);
        return false;
    }
    candidates->clear();
    if (rois.empty()) {
        if (error) error->clear();
        return true;
    }
    if (options.yawOffsetsDeg.empty() ||
        options.pitchOffsetsDeg.empty() ||
        options.workingDistanceScales.empty() ||
        options.speedsMmS.empty() || options.exposureUs.empty() ||
        !finiteNumber(options.roiPaddingMm) ||
        options.roiPaddingMm < 0.0 ||
        !finiteNumber(options.accelerationMmS2) ||
        options.accelerationMmS2 <= 0.0 ||
        options.maximumCandidateCount < 1U ||
        !finiteVector(context.nominalScanDirectionBase) ||
        !finiteVector(context.cameraForwardAxis)) {
        setError("candidate library options are invalid", error);
        return false;
    }
    if (context.calibratedObservabilityAvailable &&
        (context.calibratedImageSize.width < 1 ||
         context.calibratedImageSize.height < 1 ||
         !finiteNumber(context.cameraMatrix(0, 0)) ||
         !finiteNumber(context.cameraMatrix(1, 1)) ||
         context.cameraMatrix(0, 0) <= 0.0 ||
         context.cameraMatrix(1, 1) <= 0.0 ||
         !finiteVector(context.laserPlaneNormalCamera) ||
         cv::norm(context.laserPlaneNormalCamera) < 0.9 ||
         !finiteNumber(context.laserPlaneDMm) ||
         !finiteNumber(context.validDepthMinimumMm) ||
         !finiteNumber(context.validDepthMaximumMm) ||
         context.validDepthMinimumMm <= 0.0 ||
         context.validDepthMaximumMm <=
             context.validDepthMinimumMm ||
         !finiteNumber(context.laserPlaneToleranceMm) ||
         context.laserPlaneToleranceMm <= 0.0)) {
        setError("calibrated observability model is invalid", error);
        return false;
    }
    const cv::Matx44d baseFromReferenceCamera =
        context.baseFromReferenceFlange * context.flangeFromCamera;
    const cv::Matx33d referenceCameraRotation =
        rotationOf(baseFromReferenceCamera);
    const cv::Vec3d referenceCameraPosition =
        translationOf(baseFromReferenceCamera);
    const cv::Vec3d referenceForward = normalized(
        referenceCameraRotation * context.cameraForwardAxis,
        cv::Vec3d(0.0, 0.0, 1.0));
    const cv::Vec3d referenceX(
        referenceCameraRotation(0, 0),
        referenceCameraRotation(1, 0),
        referenceCameraRotation(2, 0));
    const cv::Matx44d cameraFromFlange =
        rigidInverse(context.flangeFromCamera);
    int nextActionId = 0;
    for (const RescanRoi& roi : rois) {
        cv::Vec3d observedViewSum(0.0, 0.0, 0.0);
        std::uint64_t observedViewCount = 0U;
        for (const VoxelKey& key : roi.voxelKeys) {
            const auto found = map.voxels.find(key);
            if (found == map.voxels.end() ||
                found->second.viewObservationCount == 0U) {
                continue;
            }
            observedViewSum +=
                static_cast<double>(
                    found->second.viewObservationCount) *
                found->second.meanObservedViewDirectionBase;
            observedViewCount +=
                found->second.viewObservationCount;
        }
        const cv::Vec3d observedViewDirection = normalized(
            observedViewSum, cv::Vec3d(0.0, 0.0, 1.0));
        const cv::Vec3d roiCenter(
            roi.centerMm.x, roi.centerMm.y, roi.centerMm.z);
        cv::Vec3d cameraFromSurface =
            referenceCameraPosition - roiCenter;
        double nominalDistance = cv::norm(cameraFromSurface);
        if (!finiteNumber(nominalDistance) ||
            nominalDistance < 20.0) {
            nominalDistance = 300.0;
            cameraFromSurface = -referenceForward * nominalDistance;
        }
        cameraFromSurface = normalized(
            cameraFromSurface, -referenceForward);
        cv::Vec3d yawAxis = normalized(
            roi.estimatedNormal, cv::Vec3d(0.0, 0.0, 1.0));
        cv::Vec3d pitchAxis = normalized(
            roi.secondaryAxis, cv::Vec3d(0.0, 1.0, 0.0));
        for (double yaw : options.yawOffsetsDeg) {
            for (double pitch : options.pitchOffsetsDeg) {
                cv::Vec3d orbitDirection =
                    rotateAroundAxis(cameraFromSurface, yawAxis, yaw);
                orbitDirection =
                    rotateAroundAxis(orbitDirection, pitchAxis, pitch);
                orbitDirection = normalized(
                    orbitDirection, cameraFromSurface);
                for (double distanceScale :
                     options.workingDistanceScales) {
                    if (!finiteNumber(distanceScale) ||
                        distanceScale <= 0.0) {
                        continue;
                    }
                    const double distance =
                        nominalDistance * distanceScale;
                    const cv::Vec3d cameraPosition =
                        roiCenter + orbitDirection * distance;
                    const cv::Vec3d viewDirection =
                        normalized(roiCenter - cameraPosition,
                                   referenceForward);
                    const cv::Matx33d cameraRotation =
                        lookAtRotation(viewDirection, referenceX);
                    const cv::Matx44d baseFromCamera =
                        transformFromRotationTranslation(
                            cameraRotation, cameraPosition);
                    const cv::Matx44d baseFromFlange =
                        baseFromCamera * cameraFromFlange;
                    cv::Vec3d scanDirection =
                        context.nominalScanDirectionBase -
                        viewDirection *
                        context.nominalScanDirectionBase.dot(
                            viewDirection);
                    if (cv::norm(scanDirection) < 1.0e-8) {
                        scanDirection = roi.principalAxis -
                            viewDirection *
                            roi.principalAxis.dot(viewDirection);
                    }
                    scanDirection = normalized(
                        scanDirection, roi.principalAxis);
                    const double length =
                        std::max(kMinimumSegmentLengthMm,
                            roiExtentAlong(roi, scanDirection) +
                            2.0 * options.roiPaddingMm);
                    const cv::Vec3d halfOffset =
                        0.5 * length * scanDirection;
                    cv::Matx44d startTransform = baseFromFlange;
                    cv::Matx44d endTransform = baseFromFlange;
                    startTransform(0, 3) -= halfOffset[0];
                    startTransform(1, 3) -= halfOffset[1];
                    startTransform(2, 3) -= halfOffset[2];
                    endTransform(0, 3) += halfOffset[0];
                    endTransform(1, 3) += halfOffset[1];
                    endTransform(2, 3) += halfOffset[2];
                    hik_scan::Pose6D forwardStart;
                    hik_scan::Pose6D forwardEnd;
                    if (!poseFromTransform(startTransform,
                                           &forwardStart) ||
                        !poseFromTransform(endTransform,
                                           &forwardEnd)) {
                        continue;
                    }
                    for (double speed : options.speedsMmS) {
                        for (double exposure : options.exposureUs) {
                            if (!finiteNumber(speed) || speed <= 0.0 ||
                                !finiteNumber(exposure) ||
                                exposure <= 0.0) {
                                continue;
                            }
                            const int directionCount =
                                options.includeReverseDirection ? 2 : 1;
                            for (int direction = 0;
                                 direction < directionCount; ++direction) {
                                if (candidates->size() >=
                                    options.maximumCandidateCount) {
                                    setError(
                                        "candidate count exceeds configured limit",
                                        error);
                                    candidates->clear();
                                    return false;
                                }
                                CandidateAction action;
                                action.actionId = nextActionId++;
                                action.roiId = roi.roiId;
                                action.yawOffsetDeg = yaw;
                                action.pitchOffsetDeg = pitch;
                                action.workingDistanceMm = distance;
                                action.measurement.segmentId =
                                    action.actionId;
                                action.measurement.roiId = roi.roiId;
                                action.measurement.kind =
                                    SegmentKind::Measurement;
                                action.measurement.start =
                                    direction == 0
                                        ? forwardStart : forwardEnd;
                                action.measurement.end =
                                    direction == 0
                                        ? forwardEnd : forwardStart;
                                action.measurement.motionStart =
                                    action.measurement.start;
                                action.measurement.motionEnd =
                                    action.measurement.end;
                                action.measurement.speedMmS = speed;
                                action.measurement.accelerationMmS2 =
                                    options.accelerationMmS2;
                                action.measurement.exposureUs = exposure;
                                action.measurement.reverseDirection =
                                    direction != 0;
                                action.coveredVoxels.clear();
                                if (context
                                        .calibratedObservabilityAvailable) {
                                    bool anyFov = false;
                                    bool anyLaser = false;
                                    for (const VoxelKey& key :
                                         roi.voxelKeys) {
                                        const std::map<
                                            VoxelKey,
                                            QualityVoxel>::
                                            const_iterator voxel =
                                                map.voxels.find(key);
                                        if (voxel == map.voxels.end()) {
                                            continue;
                                        }
                                        bool fovVisible = false;
                                        bool laserVisible = false;
                                        if (!pointObservableDuringSegment(
                                                voxel->second
                                                    .evidenceCentroidMm,
                                                action.measurement,
                                                context,
                                                &fovVisible,
                                                &laserVisible)) {
                                            continue;
                                        }
                                        anyFov = anyFov || fovVisible;
                                        anyLaser =
                                            anyLaser || laserVisible;
                                        if (fovVisible && laserVisible) {
                                            action.coveredVoxels
                                                .push_back(key);
                                        }
                                    }
                                    action.fov = anyFov
                                        ? VerificationState::Passed
                                        : VerificationState::Failed;
                                    action.laserSweep = anyLaser
                                        ? VerificationState::Passed
                                        : VerificationState::Failed;
                                    action.occlusion =
                                        context.occlusionModelAvailable
                                        ? VerificationState::Passed
                                        : VerificationState::Unknown;
                                    action.observabilityDetail =
                                        context.occlusionModelAvailable
                                        ? "calibrated FOV/laser sweep and "
                                          "occlusion model passed"
                                        : "calibrated FOV/laser sweep "
                                          "passed; occlusion is UNKNOWN";
                                    if (action.coveredVoxels.empty()) {
                                        continue;
                                    }
                                } else {
                                    action.coveredVoxels =
                                        roi.voxelKeys;
                                    action.fov =
                                        VerificationState::Unknown;
                                    action.laserSweep =
                                        VerificationState::Unknown;
                                    action.occlusion =
                                        VerificationState::Unknown;
                                    action.observabilityDetail =
                                        "calibrated FOV/laser sweep model "
                                        "is unavailable";
                                }
                                action.predictedCoverageGain =
                                    static_cast<double>(
                                        action.coveredVoxels.size());
                                action.predictedUncertaintyGain =
                                    roi.severity *
                                    static_cast<double>(
                                        action.coveredVoxels.size());
                                const double angularDiversity =
                                    observedViewCount > 0U
                                    ? std::max(
                                          0.0,
                                          std::min(
                                              1.0,
                                              0.5 *
                                              (1.0 -
                                               observedViewDirection.dot(
                                                   orbitDirection))))
                                    : std::min(
                                          1.0,
                                          (std::abs(yaw) +
                                           std::abs(pitch)) / 35.0);
                                action.predictedViewDiversityGain =
                                    angularDiversity *
                                    static_cast<double>(
                                        action.coveredVoxels.size());
                                const bool reflectiveOrMultipath =
                                    (roi.stateFlags &
                                     (QUALITY_VOXEL_MULTIPATH |
                                      QUALITY_VOXEL_SATURATED |
                                      QUALITY_VOXEL_PATH_AMBIGUOUS)) != 0U;
                                const double exposureFactor =
                                    reflectiveOrMultipath
                                        ? 1825.0 /
                                          std::max(1825.0, exposure)
                                        : 1.0;
                                action.predictedQualityGain =
                                    roi.severity *
                                    (0.5 + angularDiversity) *
                                    exposureFactor;
                                action.predictedInformationGain =
                                    action.predictedCoverageGain +
                                    action.predictedUncertaintyGain +
                                    action.predictedQualityGain +
                                    action.predictedViewDiversityGain;
                                candidates->push_back(action);
                            }
                        }
                    }
                }
            }
        }
    }
    if (error) error->clear();
    return true;
}

PlanningWeights::PlanningWeights()
    : coverage(1.0), uncertainty(1.0), quality(1.0),
      viewDiversity(0.5), executionTime(1.0),
      transitionTime(1.0), risk(2.0),
      unverifiedSafetyPenalty(1000.0) {}

SearchOptions::SearchOptions()
    : horizon(1), beamWidth(12U),
      allowUnverifiedForDryRun(true),
      requireFullyVerifiedForExecution(true) {}

bool evaluateAndRankCandidates(
        const hik_scan::Pose6D& currentPose,
        const SearchOptions& options,
        const CandidateSafetyEvaluator& evaluator,
        std::vector<CandidateAction>* candidates,
        std::string* error) {
    if (!candidates || !finitePose(currentPose) ||
        options.horizon < 1 || options.horizon > 3 ||
        options.beamWidth < 1U) {
        setError("candidate evaluation options are invalid", error);
        return false;
    }
    for (CandidateAction& action : *candidates) {
        if (evaluator) {
            action.robot =
                evaluator(currentPose, action.measurement);
        }
        action.transitionTimeS =
            transitionTime(currentPose,
                           action.measurement.motionStart);
        const double scanTime =
            action.robot.estimatedExecutionTimeS > 0.0
                ? action.robot.estimatedExecutionTimeS
                : estimateTrapezoidalMoveTime(
                      poseDistanceMm(action.measurement.start,
                                     action.measurement.end),
                      action.measurement.speedMmS,
                      action.measurement.accelerationMmS2);
        action.predictedInformationGain =
            informationGain(action, options.weights);
        const double unknownPenalty =
            action.robot.fullyVerified() ? 0.0 :
            action.robot.failed()
                ? std::numeric_limits<double>::infinity()
                : options.weights.unverifiedSafetyPenalty;
        action.utility =
            action.predictedInformationGain -
            options.weights.executionTime * scanTime -
            options.weights.transitionTime *
                action.transitionTimeS -
            options.weights.risk * action.riskPenalty -
            unknownPenalty;
    }
    std::stable_sort(
        candidates->begin(), candidates->end(),
        [](const CandidateAction& first,
           const CandidateAction& second) {
            if (first.utility != second.utility) {
                return first.utility > second.utility;
            }
            return first.actionId < second.actionId;
        });
    if (error) error->clear();
    return true;
}

bool selectGreedyAction(const std::vector<CandidateAction>& candidates,
                        const SearchOptions& options,
                        CandidateAction* selected,
                        std::string* error) {
    if (!selected) {
        setError("greedy action output is null", error);
        return false;
    }
    for (const CandidateAction& candidate : candidates) {
        if (!safetyAllowed(candidate, options)) continue;
        *selected = candidate;
        if (error) error->clear();
        return true;
    }
    setError("no candidate satisfies the configured robot safety gates",
             error);
    return false;
}

PlannedActionSequence::PlannedActionSequence()
    : totalUtility(-std::numeric_limits<double>::infinity()),
      totalInformationGain(0.0), totalEstimatedTimeS(0.0),
      fullyVerified(false), executable(false),
      detail("no local plan") {}

bool beamSearchActions(const hik_scan::Pose6D& currentPose,
                       const std::vector<CandidateAction>& candidates,
                       const SearchOptions& options,
                       const CandidateSafetyEvaluator& evaluator,
                       PlannedActionSequence* sequence,
                       std::string* error) {
    if (!sequence) {
        setError("beam-search output is null", error);
        return false;
    }
    *sequence = PlannedActionSequence();
    if (!finitePose(currentPose) || options.horizon < 1 ||
        options.horizon > 3 || options.beamWidth < 1U) {
        setError("beam-search options are invalid", error);
        return false;
    }
    struct BeamState {
        std::vector<CandidateAction> actions;
        std::set<int> roiIds;
        std::set<VoxelKey> covered;
        hik_scan::Pose6D lastPose;
        double utility{0.0};
        double information{0.0};
        double time{0.0};
        bool verified{true};
    };
    std::vector<BeamState> beam(1U);
    beam.front().lastPose = currentPose;
    for (int depth = 0; depth < options.horizon; ++depth) {
        std::vector<BeamState> expanded;
        for (const BeamState& state : beam) {
            for (const CandidateAction& candidate : candidates) {
                if (state.roiIds.count(candidate.roiId) > 0U) {
                    continue;
                }
                std::size_t newVoxelCount = 0U;
                for (const VoxelKey& key :
                     candidate.coveredVoxels) {
                    if (state.covered.count(key) == 0U) {
                        ++newVoxelCount;
                    }
                }
                if (newVoxelCount == 0U) continue;
                const double newFraction =
                    candidate.coveredVoxels.empty() ? 0.0 :
                    static_cast<double>(newVoxelCount) /
                    static_cast<double>(
                        candidate.coveredVoxels.size());
                const double gain =
                    informationGain(candidate, options.weights) *
                    newFraction;
                const double moveTime =
                    transitionTime(state.lastPose,
                                   candidate.measurement.motionStart);
                CandidateAction evaluatedCandidate = candidate;
                if (evaluator) {
                    evaluatedCandidate.robot = evaluator(
                        state.lastPose,
                        evaluatedCandidate.measurement);
                }
                if (!safetyAllowed(evaluatedCandidate, options)) {
                    continue;
                }
                const double scanTime =
                    evaluatedCandidate.robot
                                .estimatedExecutionTimeS > 0.0
                        ? evaluatedCandidate.robot
                              .estimatedExecutionTimeS
                        : estimateTrapezoidalMoveTime(
                              poseDistanceMm(
                                  evaluatedCandidate.measurement.start,
                                  evaluatedCandidate.measurement.end),
                              evaluatedCandidate.measurement.speedMmS,
                              evaluatedCandidate.measurement
                                  .accelerationMmS2);
                const double riskCost =
                    options.weights.risk *
                    evaluatedCandidate.riskPenalty;
                const double unknownCost =
                    (evaluatedCandidate.robot.fullyVerified() &&
                     observabilityFullyVerified(
                         evaluatedCandidate)) ? 0.0 :
                    options.weights.unverifiedSafetyPenalty;
                BeamState next = state;
                next.actions.push_back(evaluatedCandidate);
                next.roiIds.insert(evaluatedCandidate.roiId);
                next.covered.insert(
                    evaluatedCandidate.coveredVoxels.begin(),
                    evaluatedCandidate.coveredVoxels.end());
                next.lastPose =
                    evaluatedCandidate.measurement.motionEnd;
                next.information += gain;
                next.time += moveTime + scanTime;
                next.utility += gain -
                    options.weights.transitionTime * moveTime -
                    options.weights.executionTime * scanTime -
                    riskCost - unknownCost;
                next.verified =
                    next.verified &&
                    evaluatedCandidate.robot.fullyVerified() &&
                    observabilityFullyVerified(
                        evaluatedCandidate);
                expanded.push_back(std::move(next));
            }
        }
        if (expanded.empty()) break;
        std::stable_sort(
            expanded.begin(), expanded.end(),
            [](const BeamState& first,
               const BeamState& second) {
                if (first.utility != second.utility) {
                    return first.utility > second.utility;
                }
                if (first.time != second.time) {
                    return first.time < second.time;
                }
                return first.actions.size() <
                       second.actions.size();
            });
        if (expanded.size() > options.beamWidth) {
            expanded.resize(options.beamWidth);
        }
        beam = std::move(expanded);
    }
    if (beam.empty() || beam.front().actions.empty()) {
        setError("beam search found no safety-admissible action sequence",
                 error);
        return false;
    }
    const BeamState& best = beam.front();
    sequence->actions = best.actions;
    sequence->totalUtility = best.utility;
    sequence->totalInformationGain = best.information;
    sequence->totalEstimatedTimeS = best.time;
    sequence->fullyVerified = best.verified;
    sequence->executable =
        best.verified && options.requireFullyVerifiedForExecution;
    if (!options.requireFullyVerifiedForExecution) {
        sequence->executable = best.verified;
    }
    sequence->detail = best.verified
        ? "IK, collision and singularity are all verified"
        : "dry-run only: at least one robot safety gate is UNKNOWN";
    if (error) error->clear();
    return true;
}

const char* verificationStateName(VerificationState state) {
    switch (state) {
    case VerificationState::Unknown: return "UNKNOWN";
    case VerificationState::Passed: return "PASSED";
    case VerificationState::Failed: return "FAILED";
    }
    return "UNKNOWN";
}

const char* motionPrimitiveName(MotionPrimitive primitive) {
    switch (primitive) {
    case MotionPrimitive::Line: return "LINE";
    case MotionPrimitive::Arc: return "ARC";
    }
    return "LINE";
}

bool loadCloudPly(const std::string& path,
                  std::vector<hik_scan::CloudPoint>* cloud,
                  std::string* error) {
    if (!cloud) {
        setError("PLY cloud output is null", error);
        return false;
    }
    cloud->clear();
    std::ifstream input(path.c_str());
    if (!input) {
        setError("cannot open scan PLY: " + path, error);
        return false;
    }
    std::string line;
    if (!std::getline(input, line) || line != "ply") {
        setError("scan PLY has no ply signature: " + path, error);
        return false;
    }
    std::vector<std::string> properties;
    std::size_t vertexCount = 0U;
    bool ascii = false;
    bool headerEnded = false;
    while (std::getline(input, line)) {
        std::istringstream stream(line);
        std::string key;
        stream >> key;
        if (key == "format") {
            std::string format;
            stream >> format;
            ascii = format == "ascii";
        } else if (key == "element") {
            std::string element;
            stream >> element;
            if (element == "vertex") stream >> vertexCount;
        } else if (key == "property") {
            std::string type;
            std::string name;
            stream >> type >> name;
            if (!name.empty()) properties.push_back(name);
        } else if (key == "end_header") {
            headerEnded = true;
            break;
        }
    }
    if (!ascii || !headerEnded || properties.empty()) {
        setError("only scalar ASCII scan PLY is supported: " + path,
                 error);
        return false;
    }
    std::map<std::string, std::size_t> indexByName;
    for (std::size_t index = 0U; index < properties.size(); ++index) {
        indexByName[properties[index]] = index;
    }
    if (indexByName.count("x") == 0U ||
        indexByName.count("y") == 0U ||
        indexByName.count("z") == 0U) {
        setError("scan PLY is missing x/y/z properties: " + path,
                 error);
        return false;
    }
    const bool hasQualitySchema =
        indexByName.count("quality_flags") > 0U &&
        indexByName.count("observation_count") > 0U;
    cloud->reserve(vertexCount);
    for (std::size_t row = 0U; row < vertexCount; ++row) {
        if (!std::getline(input, line)) {
            setError("scan PLY ended before all vertices: " + path,
                     error);
            cloud->clear();
            return false;
        }
        std::istringstream stream(line);
        std::vector<double> values;
        double value = 0.0;
        while (stream >> value) values.push_back(value);
        if (values.size() < properties.size()) {
            setError("scan PLY vertex has too few fields: " + path,
                     error);
            cloud->clear();
            return false;
        }
        const auto scalar = [&](const char* name, double fallback) {
            const std::map<std::string, std::size_t>::const_iterator found =
                indexByName.find(name);
            return found == indexByName.end()
                ? fallback : values[found->second];
        };
        hik_scan::CloudPoint point;
        point.basePointMm = cv::Point3d(
            scalar("x", 0.0), scalar("y", 0.0),
            scalar("z", 0.0));
        point.confidence = scalar("confidence", 0.0);
        point.response = scalar("response", 0.0);
        point.profileIndex = static_cast<int>(
            scalar("profile_index", 0.0));
        point.pixelU = scalar("pixel_u", 0.0);
        point.pixelV = scalar("pixel_v", 0.0);
        point.qualityFlags = hasQualitySchema
            ? static_cast<std::uint32_t>(
                  scalar("quality_flags", 0.0))
            : static_cast<std::uint32_t>(
                  hik_scan::CLOUD_QUALITY_UNKNOWN_SCHEMA);
        point.observationCount = hasQualitySchema
            ? static_cast<std::uint32_t>(
                  std::max(1.0,
                      scalar("observation_count", 1.0)))
            : 1U;
        point.opticalMetricsValid =
            scalar("optical_metrics_valid", 0.0) != 0.0;
        point.snr = scalar("snr", 0.0);
        point.fwhmPx = scalar("fwhm_px", 0.0);
        point.saturatedFraction =
            scalar("saturated_fraction", 0.0);
        point.secondPeakRatio =
            scalar("second_peak_ratio", 0.0);
        point.gradientAsymmetry =
            scalar("gradient_asymmetry", 0.0);
        point.fitResidual = scalar("fit_residual", 0.0);
        point.centerSigmaPx = scalar("center_sigma_px", 0.0);
        point.stripeRejectFlags =
            static_cast<std::uint32_t>(
                scalar("stripe_reject_flags", 0.0));
        if (!finitePoint(point.basePointMm)) {
            setError("scan PLY contains non-finite geometry: " + path,
                     error);
            cloud->clear();
            return false;
        }
        cloud->push_back(point);
    }
    if (error) error->clear();
    return true;
}

bool saveAdaptivePlanJson(const std::string& path,
                          const ScanPlan& globalPlan,
                          const QualityMap& qualityMap,
                          const std::vector<RescanRoi>& rois,
                          const PlannedActionSequence& localPlan,
                          std::string* error) {
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output) {
        setError("cannot open adaptive plan JSON: " + path, error);
        return false;
    }
    output << std::setprecision(12);
    output << "{\n";
    output << "  \"schema\": \"myline_hik_adaptive_scan_v1\",\n";
    output << "  \"profile\": \""
           << jsonEscape(globalPlan.profileId) << "\",\n";
    output << "  \"global\": {\n";
    output << "    \"segment_count\": "
           << globalPlan.segments.size() << ",\n";
    output << "    \"measurement_length_mm\": "
           << globalPlan.measurementLengthMm << ",\n";
    output << "    \"transition_length_mm\": "
           << globalPlan.transitionLengthMm << ",\n";
    output << "    \"estimated_execution_time_s\": "
           << globalPlan.estimatedExecutionTimeS << ",\n";
    output << "    \"actual_execution_time_s\": "
           << (globalPlan.actualExecutionTimeS > 0.0
                   ? jsonNumber(globalPlan.actualExecutionTimeS)
                   : std::string("null"))
           << ",\n";
    output << "    \"safety_verified\": "
           << (globalPlan.safetyVerified ? "true" : "false") << ",\n";
    output << "    \"executable\": "
           << (globalPlan.executable ? "true" : "false") << ",\n";
    output << "    \"safety_detail\": \""
           << jsonEscape(globalPlan.safetyDetail) << "\",\n";
    output << "    \"segments\": [\n";
    for (std::size_t index = 0U;
         index < globalPlan.segments.size(); ++index) {
        const ScanSegment& segment = globalPlan.segments[index];
        output << "      {\"id\":" << segment.segmentId
               << ",\"kind\":\""
               << (segment.kind == SegmentKind::Measurement
                       ? "MEASUREMENT" : "TRANSITION")
               << "\",\"primitive\":\""
               << motionPrimitiveName(segment.primitive)
               << "\",\"start\":["
               << segment.start.x << ',' << segment.start.y << ','
               << segment.start.z << ',' << segment.start.rx << ','
               << segment.start.ry << ',' << segment.start.rz
               << "],\"end\":["
               << segment.end.x << ',' << segment.end.y << ','
               << segment.end.z << ',' << segment.end.rx << ','
               << segment.end.ry << ',' << segment.end.rz
               << "],\"motion_start\":["
               << segment.motionStart.x << ','
               << segment.motionStart.y << ','
               << segment.motionStart.z << ','
               << segment.motionStart.rx << ','
               << segment.motionStart.ry << ','
               << segment.motionStart.rz
               << "],\"motion_end\":["
               << segment.motionEnd.x << ','
               << segment.motionEnd.y << ','
               << segment.motionEnd.z << ','
               << segment.motionEnd.rx << ','
               << segment.motionEnd.ry << ','
               << segment.motionEnd.rz
               << "],\"arc_via\":["
               << segment.arcVia.x << ','
               << segment.arcVia.y << ','
               << segment.arcVia.z << ','
               << segment.arcVia.rx << ','
               << segment.arcVia.ry << ','
               << segment.arcVia.rz
               << "],\"arc_radius_mm\":"
               << segment.arcRadiusMm
               << ",\"blend_radius_mm\":"
               << segment.blendRadiusMm
               << ",\"allow_line_fallback\":"
               << (segment.allowLineFallback ? "true" : "false")
               << ",\"used_line_fallback\":"
               << (segment.usedLineFallback ? "true" : "false")
               << ",\"fallback_reason\":\""
               << jsonEscape(segment.fallbackReason) << '"'
               << ",\"speed_mm_s\":" << segment.speedMmS
               << ",\"acceleration_mm_s2\":"
               << segment.accelerationMmS2
               << ",\"exposure_us\":" << segment.exposureUs
               << ",\"lead_in_mm\":" << segment.leadInMm
               << ",\"lead_out_mm\":" << segment.leadOutMm
               << ",\"ik\":\""
               << verificationStateName(segment.robot.ik)
               << "\",\"collision\":\""
               << verificationStateName(segment.robot.collision)
               << "\",\"singularity\":\""
               << verificationStateName(segment.robot.singularity)
               << "\",\"minimum_joint_limit_margin_deg\":"
               << jsonNumber(
                      segment.robot.minimumJointLimitMarginDeg)
               << ",\"minimum_normalized_singular_value\":"
               << jsonNumber(segment.robot.minimumSingularValue)
               << ",\"joint_travel_deg\":"
               << jsonNumber(segment.robot.jointTravelDeg)
               << ",\"estimated_robot_time_s\":"
               << jsonNumber(
                      segment.robot.estimatedExecutionTimeS)
               << ",\"evaluated_primitive\":\""
               << motionPrimitiveName(
                      segment.robot.evaluatedPrimitive)
               << "\",\"evaluation_used_line_fallback\":"
               << (segment.robot.usedLineFallback
                       ? "true" : "false")
               << ",\"primary_primitive_failure_detail\":\""
               << jsonEscape(
                      segment.robot.primaryPrimitiveFailureDetail)
               << '"'
               << ",\"actual_execution_time_s\":"
               << (segment.actualExecutionTimeS > 0.0
                       ? jsonNumber(segment.actualExecutionTimeS)
                       : std::string("null"))
               << ",\"robot_detail\":\""
               << jsonEscape(segment.robot.detail) << '"'
               << '}';
        if (index + 1U < globalPlan.segments.size()) output << ',';
        output << '\n';
    }
    output << "    ]\n  },\n";
    output << "  \"quality_map\": {\"voxel_size_mm\":"
           << qualityMap.voxelSizeMm
           << ",\"voxel_count\":" << qualityMap.voxels.size()
           << ",\"rescan_voxel_count\":"
           << qualityMap.rescanVoxelCount
           << ",\"formal_accepted_points\":"
           << qualityMap.formalAcceptedPointCount
           << ",\"quality_accepted_points\":"
           << qualityMap.qualityAcceptedPointCount
           << ",\"rejected_points\":"
           << qualityMap.rejectedPointCount
           << ",\"expected_points\":"
           << qualityMap.expectedPointCount
           << ",\"voxels\":[\n";
    std::size_t voxelIndex = 0U;
    for (const std::pair<const VoxelKey, QualityVoxel>& entry :
         qualityMap.voxels) {
        const QualityVoxel& voxel = entry.second;
        output << "    {\"key\":[" << voxel.key.x << ','
               << voxel.key.y << ',' << voxel.key.z
               << "],\"center_mm\":[" << voxel.centerMm.x << ','
               << voxel.centerMm.y << ',' << voxel.centerMm.z
               << "],\"evidence_centroid_mm\":["
               << voxel.evidenceCentroidMm.x << ','
               << voxel.evidenceCentroidMm.y << ','
               << voxel.evidenceCentroidMm.z
               << "],\"formal_count\":"
               << voxel.formalAcceptedObservationCount
               << ",\"quality_count\":"
               << voxel.qualityAcceptedObservationCount
               << ",\"rejected_count\":"
               << voxel.rejectedObservationCount
               << ",\"expected_count\":"
               << voxel.expectedObservationCount
               << ",\"mean_confidence\":"
               << jsonNumber(voxel.meanConfidence)
               << ",\"mean_snr\":" << jsonNumber(voxel.meanSnr)
               << ",\"mean_fwhm_px\":"
               << jsonNumber(voxel.meanFwhmPx)
               << ",\"mean_saturated_fraction\":"
               << jsonNumber(voxel.meanSaturatedFraction)
               << ",\"mean_observed_view_direction_base\":["
               << voxel.meanObservedViewDirectionBase[0] << ','
               << voxel.meanObservedViewDirectionBase[1] << ','
               << voxel.meanObservedViewDirectionBase[2]
               << "],\"view_observation_count\":"
               << voxel.viewObservationCount
               << ",\"cloud_quality_flags\":"
               << voxel.cloudQualityFlags
               << ",\"stripe_reject_flags\":"
               << voxel.stripeRejectFlags
               << ",\"state_flags\":" << voxel.stateFlags
               << ",\"severity\":" << jsonNumber(voxel.severity)
               << '}';
        if (++voxelIndex < qualityMap.voxels.size()) output << ',';
        output << '\n';
    }
    output << "  ]},\n";
    output << "  \"rois\": [\n";
    for (std::size_t index = 0U; index < rois.size(); ++index) {
        const RescanRoi& roi = rois[index];
        output << "    {\"id\":" << roi.roiId
               << ",\"voxel_count\":" << roi.voxelKeys.size()
               << ",\"center_mm\":[" << roi.centerMm.x << ','
               << roi.centerMm.y << ',' << roi.centerMm.z
               << "],\"min_mm\":[" << roi.minimumMm.x << ','
               << roi.minimumMm.y << ',' << roi.minimumMm.z
               << "],\"max_mm\":[" << roi.maximumMm.x << ','
               << roi.maximumMm.y << ',' << roi.maximumMm.z
               << "],\"severity\":" << roi.severity
               << ",\"state_flags\":" << roi.stateFlags << '}';
        if (index + 1U < rois.size()) output << ',';
        output << '\n';
    }
    output << "  ],\n";
    output << "  \"local\": {\n";
    output << "    \"action_count\":" << localPlan.actions.size()
           << ",\"total_utility\":"
           << jsonNumber(localPlan.totalUtility)
           << ",\"total_information_gain\":"
           << localPlan.totalInformationGain
           << ",\"total_estimated_time_s\":"
           << localPlan.totalEstimatedTimeS
           << ",\"fully_verified\":"
           << (localPlan.fullyVerified ? "true" : "false")
           << ",\"executable\":"
           << (localPlan.executable ? "true" : "false")
           << ",\"detail\":\"" << jsonEscape(localPlan.detail)
           << "\",\n    \"actions\": [\n";
    for (std::size_t index = 0U;
         index < localPlan.actions.size(); ++index) {
        const CandidateAction& action = localPlan.actions[index];
        output << "      {\"id\":" << action.actionId
               << ",\"roi_id\":" << action.roiId
               << ",\"yaw_deg\":" << action.yawOffsetDeg
               << ",\"pitch_deg\":" << action.pitchOffsetDeg
               << ",\"working_distance_mm\":"
               << action.workingDistanceMm
               << ",\"utility\":" << jsonNumber(action.utility)
               << ",\"measurement_start\":["
               << action.measurement.start.x << ','
               << action.measurement.start.y << ','
               << action.measurement.start.z << ','
               << action.measurement.start.rx << ','
               << action.measurement.start.ry << ','
               << action.measurement.start.rz
               << "],\"measurement_end\":["
               << action.measurement.end.x << ','
               << action.measurement.end.y << ','
               << action.measurement.end.z << ','
               << action.measurement.end.rx << ','
               << action.measurement.end.ry << ','
               << action.measurement.end.rz
               << "],\"motion_start\":["
               << action.measurement.motionStart.x << ','
               << action.measurement.motionStart.y << ','
               << action.measurement.motionStart.z << ','
               << action.measurement.motionStart.rx << ','
               << action.measurement.motionStart.ry << ','
               << action.measurement.motionStart.rz
               << "],\"motion_end\":["
               << action.measurement.motionEnd.x << ','
               << action.measurement.motionEnd.y << ','
               << action.measurement.motionEnd.z << ','
               << action.measurement.motionEnd.rx << ','
               << action.measurement.motionEnd.ry << ','
               << action.measurement.motionEnd.rz
               << "],\"speed_mm_s\":"
               << action.measurement.speedMmS
               << ",\"exposure_us\":"
               << action.measurement.exposureUs
               << ",\"fov\":\""
               << verificationStateName(action.fov)
               << "\",\"laser_sweep\":\""
               << verificationStateName(action.laserSweep)
               << "\",\"occlusion\":\""
               << verificationStateName(action.occlusion)
               << "\",\"ik\":\""
               << verificationStateName(action.robot.ik)
               << "\",\"collision\":\""
               << verificationStateName(action.robot.collision)
               << "\",\"singularity\":\""
               << verificationStateName(action.robot.singularity)
               << "\",\"minimum_joint_limit_margin_deg\":"
               << jsonNumber(
                      action.robot.minimumJointLimitMarginDeg)
               << ",\"minimum_normalized_singular_value\":"
               << jsonNumber(
                      action.robot.minimumSingularValue)
               << ",\"estimated_execution_time_s\":"
               << jsonNumber(
                      action.robot.estimatedExecutionTimeS)
               << ",\"observability_detail\":\""
               << jsonEscape(action.observabilityDetail)
               << "\",\"robot_detail\":\""
               << jsonEscape(action.robot.detail)
               << "\"}";
        if (index + 1U < localPlan.actions.size()) output << ',';
        output << '\n';
    }
    output << "    ]\n  }\n}\n";
    if (!output.good()) {
        setError("failed while writing adaptive plan JSON: " + path,
                 error);
        return false;
    }
    if (error) error->clear();
    return true;
}

}  // namespace hik_adaptive
