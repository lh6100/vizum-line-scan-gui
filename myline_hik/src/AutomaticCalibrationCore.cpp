#include "AutomaticCalibrationCore.h"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

namespace hik_calibration {
namespace {

constexpr double kPi = 3.14159265358979323846;

void setError(const std::string& message, std::string* error) {
    if (error) *error = message;
}

bool finiteNumber(double value) {
    return std::isfinite(value);
}

cv::Matx33d rotationOf(const cv::Matx44d& transform) {
    return cv::Matx33d(
        transform(0, 0), transform(0, 1), transform(0, 2),
        transform(1, 0), transform(1, 1), transform(1, 2),
        transform(2, 0), transform(2, 1), transform(2, 2));
}

cv::Vec3d translationOf(const cv::Matx44d& transform) {
    return cv::Vec3d(transform(0, 3), transform(1, 3), transform(2, 3));
}

cv::Matx44d transformFromRt(const cv::Matx33d& rotation,
                            const cv::Vec3d& translation) {
    return cv::Matx44d(
        rotation(0, 0), rotation(0, 1), rotation(0, 2), translation[0],
        rotation(1, 0), rotation(1, 1), rotation(1, 2), translation[1],
        rotation(2, 0), rotation(2, 1), rotation(2, 2), translation[2],
        0.0, 0.0, 0.0, 1.0);
}

cv::Matx44d rigidInverse(const cv::Matx44d& transform) {
    const cv::Matx33d rotation = rotationOf(transform);
    const cv::Matx33d inverseRotation = rotation.t();
    return transformFromRt(
        inverseRotation, -(inverseRotation * translationOf(transform)));
}

bool rigidTransform(const cv::Matx44d& transform) {
    for (int index = 0; index < 16; ++index) {
        if (!finiteNumber(transform.val[index])) return false;
    }
    if (std::abs(transform(3, 0)) > 1.0e-8 ||
        std::abs(transform(3, 1)) > 1.0e-8 ||
        std::abs(transform(3, 2)) > 1.0e-8 ||
        std::abs(transform(3, 3) - 1.0) > 1.0e-8) {
        return false;
    }
    const cv::Matx33d rotation = rotationOf(transform);
    const cv::Matx33d orthogonality =
        rotation.t() * rotation - cv::Matx33d::eye();
    double maximumError = 0.0;
    for (int index = 0; index < 9; ++index) {
        maximumError = std::max(maximumError,
                                std::abs(orthogonality.val[index]));
    }
    return maximumError <= 1.0e-5 &&
           std::abs(cv::determinant(cv::Mat(rotation)) - 1.0) <= 1.0e-5;
}

cv::Matx33d axisRotation(double xDeg, double yDeg, double zDeg) {
    const double x = xDeg * kPi / 180.0;
    const double y = yDeg * kPi / 180.0;
    const double z = zDeg * kPi / 180.0;
    const cv::Matx33d rx(
        1.0, 0.0, 0.0,
        0.0, std::cos(x), -std::sin(x),
        0.0, std::sin(x), std::cos(x));
    const cv::Matx33d ry(
        std::cos(y), 0.0, std::sin(y),
        0.0, 1.0, 0.0,
        -std::sin(y), 0.0, std::cos(y));
    const cv::Matx33d rz(
        std::cos(z), -std::sin(z), 0.0,
        std::sin(z), std::cos(z), 0.0,
        0.0, 0.0, 1.0);
    return rz * ry * rx;
}

double rotationDistanceDeg(const cv::Matx33d& first,
                           const cv::Matx33d& second) {
    const cv::Matx33d delta = first.t() * second;
    const double cosine = std::max(
        -1.0, std::min(1.0, (cv::trace(cv::Mat(delta))[0] - 1.0) * 0.5));
    return std::acos(cosine) * 180.0 / kPi;
}

double percentile(std::vector<double> values, double probability) {
    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const double position = std::max(0.0, std::min(1.0, probability)) *
                            static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

ErrorMetrics metrics(const std::vector<double>& values) {
    ErrorMetrics result;
    if (values.empty()) return result;
    result.mean = std::accumulate(values.begin(), values.end(), 0.0) /
                  static_cast<double>(values.size());
    double squared = 0.0;
    for (double value : values) squared += value * value;
    result.rms = std::sqrt(squared / static_cast<double>(values.size()));
    result.p95 = percentile(values, 0.95);
    result.maximum = *std::max_element(values.begin(), values.end());
    return result;
}

std::string yamlQuote(const std::string& input) {
    std::string output = "\"";
    for (char value : input) {
        if (value == '\\' || value == '"') output.push_back('\\');
        output.push_back(value);
    }
    output.push_back('"');
    return output;
}

void writeTransform(std::ostream& output, const cv::Matx44d& transform) {
    output << '[';
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            if (row != 0 || column != 0) output << ", ";
            output << transform(row, column);
        }
    }
    output << ']';
}

bool boardFitsImage(const cv::Matx44d& cameraFromBoard,
                    const BoardSpec& board,
                    const cv::Mat& cameraMatrix,
                    const cv::Mat& distCoeffs,
                    const cv::Size& imageSize,
                    double marginPx) {
    const std::vector<cv::Point3f> boardCorners = {
        cv::Point3f(0.0F, 0.0F, 0.0F),
        cv::Point3f(static_cast<float>(board.widthMm()), 0.0F, 0.0F),
        cv::Point3f(static_cast<float>(board.widthMm()),
                    static_cast<float>(board.heightMm()), 0.0F),
        cv::Point3f(0.0F, static_cast<float>(board.heightMm()), 0.0F)
    };
    const cv::Matx33d rotation = rotationOf(cameraFromBoard);
    const cv::Vec3d translation = translationOf(cameraFromBoard);
    for (const cv::Point3f& point : boardCorners) {
        const cv::Vec3d cameraPoint = rotation *
            cv::Vec3d(point.x, point.y, point.z) + translation;
        if (!finiteNumber(cameraPoint[2]) || cameraPoint[2] <= 1.0) {
            return false;
        }
    }
    cv::Vec3d rvec;
    cv::Rodrigues(cv::Mat(rotation), rvec);
    std::vector<cv::Point2f> pixels;
    try {
        cv::projectPoints(boardCorners, rvec, translation,
                          cameraMatrix, distCoeffs, pixels);
    } catch (const cv::Exception&) {
        return false;
    }
    if (pixels.size() != boardCorners.size()) return false;
    for (const cv::Point2f& pixel : pixels) {
        if (!finiteNumber(pixel.x) || !finiteNumber(pixel.y) ||
            pixel.x < marginPx || pixel.y < marginPx ||
            pixel.x > static_cast<double>(imageSize.width - 1) - marginPx ||
            pixel.y > static_cast<double>(imageSize.height - 1) - marginPx) {
            return false;
        }
    }
    return true;
}

}  // namespace

AutomaticCalibrationPlanOptions::AutomaticCalibrationPlanOptions()
    : nearDepthMm(425.0), middleDepthMm(550.0), farDepthMm(675.0),
      maximumTranslationFromStartMm(420.0),
      maximumRotationFromStartDeg(55.0), speedMmS(20.0),
      accelerationMmS2(100.0) {}

AutomaticCalibrationTarget::AutomaticCalibrationTarget()
    : index(-1), holdout(false), requestedBoardDepthMm(0.0),
      requestedBoardCenterXmm(0.0), requestedBoardCenterYmm(0.0),
      requestedTiltXDeg(0.0), requestedTiltYDeg(0.0),
      requestedRollDeg(0.0), cameraFromBoard(cv::Matx44d::eye()) {}

AutomaticCalibrationPlan::AutomaticCalibrationPlan()
    : ok(false), baseFromBoardSeed(cv::Matx44d::eye()),
      trainingTargetCount(0), holdoutTargetCount(0),
      maximumTranslationFromStartMm(0.0),
      maximumRotationFromStartDeg(0.0) {}

bool poseFromFairinoTransform(const cv::Matx44d& transform,
                              hik_scan::Pose6D* pose,
                              std::string* error) {
    if (!pose || !rigidTransform(transform)) {
        setError("base-from-flange transform is not rigid", error);
        return false;
    }
    const double ry = std::asin(std::max(
        -1.0, std::min(1.0, -transform(2, 0))));
    const double cy = std::cos(ry);
    double rx = 0.0;
    double rz = 0.0;
    if (std::abs(cy) > 1.0e-8) {
        rx = std::atan2(transform(2, 1), transform(2, 2));
        rz = std::atan2(transform(1, 0), transform(0, 0));
    } else {
        rz = std::atan2(-transform(0, 1), transform(1, 1));
    }
    pose->x = transform(0, 3);
    pose->y = transform(1, 3);
    pose->z = transform(2, 3);
    pose->rx = rx * 180.0 / kPi;
    pose->ry = ry * 180.0 / kPi;
    pose->rz = rz * 180.0 / kPi;
    return true;
}

bool buildAutomaticCalibrationPlan(
        const cv::Matx44d& baseFromFlangeStart,
        const cv::Matx44d& flangeFromCameraSeed,
        const cv::Matx44d& cameraFromBoardSeed,
        const BoardSpec& board,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        const cv::Size& imageSize,
        const AutomaticCalibrationPlanOptions& options,
        AutomaticCalibrationPlan* plan) {
    if (!plan) return false;
    *plan = AutomaticCalibrationPlan();
    std::string boardError;
    if (!validateBoardSpec(board, &boardError) ||
        !rigidTransform(baseFromFlangeStart) ||
        !rigidTransform(flangeFromCameraSeed) ||
        !rigidTransform(cameraFromBoardSeed) ||
        cameraMatrix.rows != 3 || cameraMatrix.cols != 3 ||
        cameraMatrix.empty() || !cv::checkRange(cameraMatrix) ||
        distCoeffs.empty() || !cv::checkRange(distCoeffs) ||
        imageSize.width <= 0 || imageSize.height <= 0 ||
        !finiteNumber(options.nearDepthMm) ||
        !finiteNumber(options.middleDepthMm) ||
        !finiteNumber(options.farDepthMm) ||
        options.nearDepthMm < 350.0 ||
        options.nearDepthMm >= options.middleDepthMm ||
        options.middleDepthMm >= options.farDepthMm ||
        options.farDepthMm > 750.0 ||
        !finiteNumber(options.maximumTranslationFromStartMm) ||
        options.maximumTranslationFromStartMm < 100.0 ||
        !finiteNumber(options.maximumRotationFromStartDeg) ||
        options.maximumRotationFromStartDeg < 15.0 ||
        !finiteNumber(options.speedMmS) || options.speedMmS < 5.0 ||
        options.speedMmS > 50.0 ||
        !finiteNumber(options.accelerationMmS2) ||
        options.accelerationMmS2 <= 0.0 || options.accelerationMmS2 > 500.0) {
        plan->error = boardError.empty()
            ? "automatic calibration plan inputs are invalid" : boardError;
        return false;
    }
    const cv::Vec3d seedCenter =
        rotationOf(cameraFromBoardSeed) *
            cv::Vec3d(board.widthMm() * 0.5,
                      board.heightMm() * 0.5, 0.0) +
        translationOf(cameraFromBoardSeed);
    if (!finiteNumber(seedCenter[2]) || seedCenter[2] < 300.0 ||
        seedCenter[2] > 900.0) {
        plan->error = "seed image places the board outside 300-900 mm";
        return false;
    }

    plan->baseFromBoardSeed = baseFromFlangeStart *
        flangeFromCameraSeed * cameraFromBoardSeed;
    if (!poseFromFairinoTransform(baseFromFlangeStart, &plan->homePose,
                                  &plan->error)) {
        return false;
    }

    // Ten training patterns and two holdout patterns at each depth. The
    // translations are expressed as fractions of depth so the board center
    // visits the image center, edges and corners for both camera resolutions.
    struct Pattern {
        double xRatio, yRatio, tiltX, tiltY, roll;
        bool holdout;
    };
    const Pattern patterns[] = {
        { 0.00,  0.00,   0.0,   0.0,   0.0, false},
        {-0.13, -0.09, -14.0,  10.0,  -7.0, false},
        { 0.13, -0.09,  14.0, -10.0,   7.0, false},
        {-0.13,  0.09,  12.0,  13.0,   5.0, false},
        { 0.13,  0.09, -12.0, -13.0,  -5.0, false},
        { 0.00, -0.11, -16.0,   0.0,  10.0, false},
        { 0.00,  0.11,  16.0,   0.0, -10.0, false},
        {-0.15,  0.00,   0.0, -16.0,   8.0, false},
        { 0.15,  0.00,   0.0,  16.0,  -8.0, false},
        { 0.06, -0.05,  -9.0,   9.0,  12.0, false},
        {-0.07,  0.06,  10.0,  -8.0, -11.0, true},
        { 0.08,  0.05, -11.0, -10.0,   6.0, true}
    };
    const double depths[] = {
        options.nearDepthMm,
        options.middleDepthMm,
        options.farDepthMm
    };
    const cv::Matx33d seedRotation = rotationOf(cameraFromBoardSeed);
    const cv::Vec3d boardCenter(
        board.widthMm() * 0.5, board.heightMm() * 0.5, 0.0);
    int targetIndex = 0;
    for (double depth : depths) {
        for (const Pattern& pattern : patterns) {
            AutomaticCalibrationTarget target;
            target.index = targetIndex++;
            target.holdout = pattern.holdout;
            target.requestedBoardDepthMm = depth;
            bool visible = false;
            // Near-field visibility is lens dependent. Start with the full
            // coverage request, then shrink offset and tilt together until
            // all four physical board corners retain a 12 px image margin.
            for (int step = 20; step >= 0; --step) {
                const double scale = static_cast<double>(step) / 20.0;
                target.requestedBoardCenterXmm =
                    pattern.xRatio * depth * scale;
                target.requestedBoardCenterYmm =
                    pattern.yRatio * depth * scale;
                target.requestedTiltXDeg = pattern.tiltX * scale;
                target.requestedTiltYDeg = pattern.tiltY * scale;
                target.requestedRollDeg = pattern.roll * scale;
                const cv::Matx33d targetRotation = axisRotation(
                    target.requestedTiltXDeg,
                    target.requestedTiltYDeg,
                    target.requestedRollDeg) * seedRotation;
                const cv::Vec3d desiredCenter(
                    target.requestedBoardCenterXmm,
                    target.requestedBoardCenterYmm, depth);
                target.cameraFromBoard = transformFromRt(
                    targetRotation,
                    desiredCenter - targetRotation * boardCenter);
                if (boardFitsImage(target.cameraFromBoard, board,
                                   cameraMatrix, distCoeffs, imageSize,
                                   12.0)) {
                    visible = true;
                    break;
                }
            }
            if (!visible) {
                std::ostringstream message;
                message << "target " << target.index
                        << " cannot keep the complete board inside the calibrated image at "
                        << depth << " mm";
                plan->error = message.str();
                plan->targets.clear();
                return false;
            }
            const cv::Matx44d baseFromFlangeTarget =
                plan->baseFromBoardSeed *
                rigidInverse(target.cameraFromBoard) *
                rigidInverse(flangeFromCameraSeed);
            if (!poseFromFairinoTransform(
                    baseFromFlangeTarget, &target.baseFromFlangePose,
                    &plan->error)) {
                plan->targets.clear();
                return false;
            }
            const double translation = cv::norm(
                translationOf(baseFromFlangeTarget) -
                translationOf(baseFromFlangeStart));
            const double rotation = rotationDistanceDeg(
                rotationOf(baseFromFlangeStart),
                rotationOf(baseFromFlangeTarget));
            plan->maximumTranslationFromStartMm = std::max(
                plan->maximumTranslationFromStartMm, translation);
            plan->maximumRotationFromStartDeg = std::max(
                plan->maximumRotationFromStartDeg, rotation);
            if (translation > options.maximumTranslationFromStartMm ||
                rotation > options.maximumRotationFromStartDeg) {
                std::ostringstream message;
                message << "target " << target.index
                        << " exceeds configured motion envelope: "
                        << translation << " mm / " << rotation << " deg";
                plan->error = message.str();
                plan->targets.clear();
                return false;
            }
            if (target.holdout) ++plan->holdoutTargetCount;
            else ++plan->trainingTargetCount;
            plan->targets.push_back(target);
        }
    }
    if (plan->trainingTargetCount != 30 ||
        plan->holdoutTargetCount != 6 || plan->targets.size() != 36U) {
        plan->error = "internal automatic calibration pattern count mismatch";
        plan->targets.clear();
        return false;
    }
    plan->ok = true;
    return true;
}

AutomaticHoldoutSample::AutomaticHoldoutSample()
    : baseFromFlange(cv::Matx44d::eye()),
      cameraFromBoard(cv::Matx44d::eye()), reprojectionRmsPx(0.0) {}

AutomaticCalibrationValidation::AutomaticCalibrationValidation()
    : ok(false), passed(false), sampleCount(0) {}

bool validateAutomaticCalibrationHoldout(
        const std::vector<AutomaticHoldoutSample>& samples,
        const cv::Matx44d& flangeFromCamera,
        const cv::Matx44d& baseFromBoardReference,
        double maximumReprojectionRmsPx,
        double maximumTranslationRmsMm,
        double maximumRotationRmsDeg,
        AutomaticCalibrationValidation* validation) {
    if (!validation) return false;
    *validation = AutomaticCalibrationValidation();
    if (samples.size() < 4U || !rigidTransform(flangeFromCamera) ||
        !rigidTransform(baseFromBoardReference) ||
        !finiteNumber(maximumReprojectionRmsPx) ||
        maximumReprojectionRmsPx <= 0.0 ||
        !finiteNumber(maximumTranslationRmsMm) ||
        maximumTranslationRmsMm <= 0.0 ||
        !finiteNumber(maximumRotationRmsDeg) ||
        maximumRotationRmsDeg <= 0.0) {
        validation->error = "automatic calibration holdout inputs are invalid";
        return false;
    }
    std::vector<double> reprojection;
    std::vector<double> translation;
    std::vector<double> rotation;
    for (const AutomaticHoldoutSample& sample : samples) {
        if (!rigidTransform(sample.baseFromFlange) ||
            !rigidTransform(sample.cameraFromBoard) ||
            !finiteNumber(sample.reprojectionRmsPx) ||
            sample.reprojectionRmsPx < 0.0) {
            validation->error = "holdout sample contains invalid data: " +
                                sample.sampleId;
            return false;
        }
        const cv::Matx44d baseFromBoard = sample.baseFromFlange *
            flangeFromCamera * sample.cameraFromBoard;
        reprojection.push_back(sample.reprojectionRmsPx);
        translation.push_back(cv::norm(
            translationOf(baseFromBoard) -
            translationOf(baseFromBoardReference)));
        rotation.push_back(rotationDistanceDeg(
            rotationOf(baseFromBoardReference), rotationOf(baseFromBoard)));
    }
    validation->sampleCount = static_cast<int>(samples.size());
    validation->reprojectionPx = metrics(reprojection);
    validation->baseBoardTranslationMm = metrics(translation);
    validation->baseBoardRotationDeg = metrics(rotation);
    validation->passed =
        validation->reprojectionPx.rms <= maximumReprojectionRmsPx &&
        validation->baseBoardTranslationMm.rms <= maximumTranslationRmsMm &&
        validation->baseBoardRotationDeg.rms <= maximumRotationRmsDeg;
    validation->ok = true;
    return true;
}

DualCameraExtrinsics::DualCameraExtrinsics()
    : ok(false), firstCameraFromSecondCamera(cv::Matx44d::eye()),
      baselineMm(0.0), relativeRotationDeg(0.0) {}

bool computeDualCameraExtrinsics(
        const cv::Matx44d& flangeFromFirstCamera,
        const cv::Matx44d& flangeFromSecondCamera,
        DualCameraExtrinsics* extrinsics) {
    if (!extrinsics) return false;
    *extrinsics = DualCameraExtrinsics();
    if (!rigidTransform(flangeFromFirstCamera) ||
        !rigidTransform(flangeFromSecondCamera)) {
        extrinsics->error = "dual-camera hand-eye transform is not rigid";
        return false;
    }
    extrinsics->firstCameraFromSecondCamera =
        rigidInverse(flangeFromFirstCamera) * flangeFromSecondCamera;
    extrinsics->baselineMm = cv::norm(
        translationOf(extrinsics->firstCameraFromSecondCamera));
    extrinsics->relativeRotationDeg = rotationDistanceDeg(
        cv::Matx33d::eye(),
        rotationOf(extrinsics->firstCameraFromSecondCamera));
    if (!finiteNumber(extrinsics->baselineMm) ||
        extrinsics->baselineMm <= 1.0 ||
        !finiteNumber(extrinsics->relativeRotationDeg)) {
        extrinsics->error = "dual-camera baseline or rotation is invalid";
        return false;
    }
    extrinsics->ok = true;
    return true;
}

bool saveDualCameraExtrinsicsYaml(
        const std::string& path,
        const DualCameraExtrinsics& extrinsics,
        const DualCameraYamlMetadata& metadata,
        std::string* error) {
    if (!extrinsics.ok || !rigidTransform(
            extrinsics.firstCameraFromSecondCamera) ||
        metadata.firstCameraFrame.empty() ||
        metadata.secondCameraFrame.empty() ||
        metadata.firstHandEyeFile.empty() ||
        metadata.firstHandEyeSha256.empty() ||
        metadata.secondHandEyeFile.empty() ||
        metadata.secondHandEyeSha256.empty()) {
        setError("dual-camera result or metadata is incomplete", error);
        return false;
    }
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output) {
        setError("cannot open dual-camera YAML for writing: " + path, error);
        return false;
    }
    output << std::setprecision(12);
    output << "schema_version: 1\n";
    output << "calibration_type: dual_camera_extrinsics\n";
    output << "transform_convention: \"T_parent_child maps child coordinates into parent coordinates\"\n";
    output << "parent_frame: " << yamlQuote(metadata.firstCameraFrame) << "\n";
    output << "child_frame: " << yamlQuote(metadata.secondCameraFrame) << "\n";
    output << "translation_unit: mm\n";
    output << "T_first_camera_second_camera: ";
    writeTransform(output, extrinsics.firstCameraFromSecondCamera);
    output << "\nquality:\n";
    output << "  baseline_mm: " << extrinsics.baselineMm << "\n";
    output << "  relative_rotation_deg: "
           << extrinsics.relativeRotationDeg << "\n";
    output << "  derivation: common_flange_hand_eye\n";
    output << "sources:\n";
    output << "  first_handeye_file: "
           << yamlQuote(metadata.firstHandEyeFile) << "\n";
    output << "  first_handeye_sha256: "
           << yamlQuote(metadata.firstHandEyeSha256) << "\n";
    output << "  second_handeye_file: "
           << yamlQuote(metadata.secondHandEyeFile) << "\n";
    output << "  second_handeye_sha256: "
           << yamlQuote(metadata.secondHandEyeSha256) << "\n";
    output << "generated_at: " << yamlQuote(metadata.generatedAt) << "\n";
    if (!output) {
        setError("failed while writing dual-camera YAML: " + path, error);
        return false;
    }
    return true;
}

}  // namespace hik_calibration
