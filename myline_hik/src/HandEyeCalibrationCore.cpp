#include "HandEyeCalibrationCore.h"

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

const double kPi = 3.14159265358979323846;

cv::Matx33d rotationOf(const cv::Matx44d& transform) {
    return cv::Matx33d(transform(0, 0), transform(0, 1), transform(0, 2),
                       transform(1, 0), transform(1, 1), transform(1, 2),
                       transform(2, 0), transform(2, 1), transform(2, 2));
}

cv::Vec3d translationOf(const cv::Matx44d& transform) {
    return cv::Vec3d(transform(0, 3), transform(1, 3), transform(2, 3));
}

cv::Matx44d transformFromRt(const cv::Matx33d& rotation,
                            const cv::Vec3d& translation) {
    cv::Matx44d value = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            value(row, column) = rotation(row, column);
        }
        value(row, 3) = translation[row];
    }
    return value;
}

cv::Matx33d matToMatx33(const cv::Mat& input) {
    cv::Mat value;
    input.convertTo(value, CV_64F);
    return cv::Matx33d(
        value.at<double>(0, 0), value.at<double>(0, 1), value.at<double>(0, 2),
        value.at<double>(1, 0), value.at<double>(1, 1), value.at<double>(1, 2),
        value.at<double>(2, 0), value.at<double>(2, 1), value.at<double>(2, 2));
}

bool finiteTransform(const cv::Matx44d& transform) {
    for (int index = 0; index < 16; ++index) {
        if (!std::isfinite(transform.val[index])) {
            return false;
        }
    }
    return true;
}

cv::Matx33d projectedRotation(const cv::Matx33d& input) {
    cv::Mat singularValues;
    cv::Mat left;
    cv::Mat rightTranspose;
    cv::SVD::compute(cv::Mat(input), singularValues, left, rightTranspose);
    cv::Mat rotation = left * rightTranspose;
    if (cv::determinant(rotation) < 0.0) {
        left.col(2) *= -1.0;
        rotation = left * rightTranspose;
    }
    return matToMatx33(rotation);
}

cv::Matx33d meanRotation(const std::vector<cv::Matx44d>& transforms) {
    cv::Matx33d sum = cv::Matx33d::zeros();
    for (std::size_t index = 0; index < transforms.size(); ++index) {
        sum += rotationOf(transforms[index]);
    }
    return projectedRotation(sum);
}

double rotationDistanceDeg(const cv::Matx33d& first,
                           const cv::Matx33d& second) {
    const cv::Matx33d delta = first.t() * second;
    const double cosine = std::max(-1.0, std::min(
        1.0, (cv::trace(cv::Mat(delta))[0] - 1.0) * 0.5));
    return std::acos(cosine) * 180.0 / kPi;
}

double percentile(std::vector<double> values, double probability) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
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
    if (values.empty()) {
        return result;
    }
    double sum = 0.0;
    double sumSquares = 0.0;
    for (std::size_t index = 0; index < values.size(); ++index) {
        sum += values[index];
        sumSquares += values[index] * values[index];
        result.maximum = std::max(result.maximum, values[index]);
    }
    result.mean = sum / static_cast<double>(values.size());
    result.rms = std::sqrt(sumSquares / static_cast<double>(values.size()));
    result.p95 = percentile(values, 0.95);
    return result;
}

double robustThreshold(const std::vector<double>& values,
                       double floor,
                       double madScale) {
    const double median = percentile(values, 0.5);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (std::size_t index = 0; index < values.size(); ++index) {
        deviations.push_back(std::fabs(values[index] - median));
    }
    const double mad = percentile(deviations, 0.5);
    return std::max(floor, median + madScale * 1.4826 * mad);
}

std::string yamlQuote(const std::string& input) {
    std::string escaped;
    escaped.reserve(input.size() + 2U);
    escaped.push_back('"');
    for (std::size_t index = 0; index < input.size(); ++index) {
        const char c = input[index];
        if (c == '\\' || c == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(c);
    }
    escaped.push_back('"');
    return escaped;
}

struct MethodCandidate {
    bool ok;
    std::string name;
    cv::HandEyeCalibrationMethod method;
    cv::Matx44d flangeFromCamera;
    cv::Matx44d baseFromBoardMean;
    std::vector<cv::Matx44d> baseFromBoards;
    std::vector<double> translations;
    std::vector<double> rotations;
    ErrorMetrics translationMetrics;
    ErrorMetrics rotationMetrics;
    double score;

    MethodCandidate()
        : ok(false), method(cv::CALIB_HAND_EYE_TSAI),
          flangeFromCamera(cv::Matx44d::eye()),
          baseFromBoardMean(cv::Matx44d::eye()),
          score(std::numeric_limits<double>::infinity()) {}
};

MethodCandidate solveMethod(const std::vector<HandEyeSample>& samples,
                            const std::vector<int>& indices,
                            cv::HandEyeCalibrationMethod method,
                            const std::string& name) {
    MethodCandidate candidate;
    candidate.name = name;
    candidate.method = method;
    std::vector<cv::Mat> rotationsGripperToBase;
    std::vector<cv::Mat> translationsGripperToBase;
    std::vector<cv::Mat> rotationsTargetToCamera;
    std::vector<cv::Mat> translationsTargetToCamera;
    rotationsGripperToBase.reserve(indices.size());
    translationsGripperToBase.reserve(indices.size());
    rotationsTargetToCamera.reserve(indices.size());
    translationsTargetToCamera.reserve(indices.size());
    for (std::size_t activeIndex = 0; activeIndex < indices.size(); ++activeIndex) {
        const HandEyeSample& sample = samples[static_cast<std::size_t>(indices[activeIndex])];
        rotationsGripperToBase.push_back(cv::Mat(rotationOf(sample.baseFromFlange)).clone());
        translationsGripperToBase.push_back(cv::Mat(translationOf(sample.baseFromFlange)).clone());
        rotationsTargetToCamera.push_back(cv::Mat(rotationOf(sample.cameraFromBoard)).clone());
        translationsTargetToCamera.push_back(cv::Mat(translationOf(sample.cameraFromBoard)).clone());
    }

    cv::Mat rotationCameraToGripper;
    cv::Mat translationCameraToGripper;
    try {
        cv::calibrateHandEye(rotationsGripperToBase,
                             translationsGripperToBase,
                             rotationsTargetToCamera,
                             translationsTargetToCamera,
                             rotationCameraToGripper,
                             translationCameraToGripper,
                             method);
    } catch (const cv::Exception&) {
        return candidate;
    }
    if (rotationCameraToGripper.empty() || translationCameraToGripper.empty()) {
        return candidate;
    }
    cv::Mat rotation64;
    cv::Mat translation64;
    rotationCameraToGripper.convertTo(rotation64, CV_64F);
    translationCameraToGripper.reshape(1, 3).convertTo(translation64, CV_64F);
    cv::Matx33d rotation;
    cv::Vec3d translation;
    rotation = matToMatx33(rotation64);
    for (int row = 0; row < 3; ++row) {
        translation[row] = translation64.at<double>(row, 0);
    }
    candidate.flangeFromCamera = transformFromRt(projectedRotation(rotation), translation);
    if (!finiteTransform(candidate.flangeFromCamera)) {
        return candidate;
    }

    candidate.baseFromBoards.reserve(indices.size());
    cv::Vec3d meanTranslation(0.0, 0.0, 0.0);
    for (std::size_t activeIndex = 0; activeIndex < indices.size(); ++activeIndex) {
        const HandEyeSample& sample = samples[static_cast<std::size_t>(indices[activeIndex])];
        const cv::Matx44d board = sample.baseFromFlange *
                                  candidate.flangeFromCamera *
                                  sample.cameraFromBoard;
        candidate.baseFromBoards.push_back(board);
        meanTranslation += translationOf(board);
    }
    meanTranslation *= 1.0 / static_cast<double>(indices.size());
    const cv::Matx33d rotationMean = meanRotation(candidate.baseFromBoards);
    candidate.baseFromBoardMean = transformFromRt(rotationMean, meanTranslation);

    candidate.translations.reserve(indices.size());
    candidate.rotations.reserve(indices.size());
    for (std::size_t activeIndex = 0; activeIndex < indices.size(); ++activeIndex) {
        candidate.translations.push_back(cv::norm(
            translationOf(candidate.baseFromBoards[activeIndex]) - meanTranslation));
        candidate.rotations.push_back(rotationDistanceDeg(
            rotationMean, rotationOf(candidate.baseFromBoards[activeIndex])));
    }
    candidate.translationMetrics = metrics(candidate.translations);
    candidate.rotationMetrics = metrics(candidate.rotations);
    candidate.score = candidate.translationMetrics.rms +
                      2.0 * candidate.rotationMetrics.rms;
    candidate.ok = std::isfinite(candidate.score);
    return candidate;
}

MethodCandidate bestMethod(const std::vector<HandEyeSample>& samples,
                           const std::vector<int>& indices) {
    const cv::HandEyeCalibrationMethod methods[] = {
        cv::CALIB_HAND_EYE_TSAI,
        cv::CALIB_HAND_EYE_PARK,
        cv::CALIB_HAND_EYE_HORAUD,
        cv::CALIB_HAND_EYE_ANDREFF,
        cv::CALIB_HAND_EYE_DANIILIDIS
    };
    const char* names[] = {"TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"};
    MethodCandidate best;
    for (int methodIndex = 0; methodIndex < 5; ++methodIndex) {
        const MethodCandidate candidate = solveMethod(
            samples, indices, methods[methodIndex], names[methodIndex]);
        if (candidate.ok && (!best.ok || candidate.score < best.score)) {
            best = candidate;
        }
    }
    return best;
}

void motionSpans(const std::vector<HandEyeSample>& samples,
                 const std::vector<int>& indices,
                 double* translationSpan,
                 double* rotationSpan,
                 double* secondaryRotationSpread) {
    *translationSpan = 0.0;
    *rotationSpan = 0.0;
    for (std::size_t left = 0; left < indices.size(); ++left) {
        for (std::size_t right = left + 1U; right < indices.size(); ++right) {
            const cv::Matx44d& first = samples[static_cast<std::size_t>(indices[left])].baseFromFlange;
            const cv::Matx44d& second = samples[static_cast<std::size_t>(indices[right])].baseFromFlange;
            *translationSpan = std::max(*translationSpan,
                rigidTranslationDistanceMm(first, second));
            *rotationSpan = std::max(*rotationSpan,
                rigidRotationDistanceDeg(first, second));
        }
    }
    *secondaryRotationSpread = 0.0;
    if (indices.size() < 3U) {
        return;
    }
    cv::Mat excitation(static_cast<int>(indices.size() - 1U), 3, CV_64F);
    const cv::Matx33d reference = rotationOf(
        samples[static_cast<std::size_t>(indices.front())].baseFromFlange);
    for (std::size_t index = 1; index < indices.size(); ++index) {
        const cv::Matx33d current = rotationOf(
            samples[static_cast<std::size_t>(indices[index])].baseFromFlange);
        cv::Vec3d rotationVector;
        cv::Rodrigues(cv::Mat(reference.t() * current), rotationVector);
        for (int axis = 0; axis < 3; ++axis) {
            excitation.at<double>(static_cast<int>(index - 1U), axis) =
                rotationVector[axis] * 180.0 / kPi;
        }
    }
    cv::SVD svd(excitation, cv::SVD::NO_UV);
    if (svd.w.total() >= 2U) {
        *secondaryRotationSpread = svd.w.at<double>(1, 0) /
            std::sqrt(static_cast<double>(indices.size() - 1U));
    }
}

}  // namespace

cv::Matx44d fairinoBaseFromFlange(double xMm,
                                  double yMm,
                                  double zMm,
                                  double rxDeg,
                                  double ryDeg,
                                  double rzDeg) {
    const double rx = rxDeg * kPi / 180.0;
    const double ry = ryDeg * kPi / 180.0;
    const double rz = rzDeg * kPi / 180.0;
    const cv::Matx33d x(1.0, 0.0, 0.0,
                        0.0, std::cos(rx), -std::sin(rx),
                        0.0, std::sin(rx), std::cos(rx));
    const cv::Matx33d y(std::cos(ry), 0.0, std::sin(ry),
                        0.0, 1.0, 0.0,
                        -std::sin(ry), 0.0, std::cos(ry));
    const cv::Matx33d z(std::cos(rz), -std::sin(rz), 0.0,
                        std::sin(rz), std::cos(rz), 0.0,
                        0.0, 0.0, 1.0);
    return transformFromRt(z * y * x, cv::Vec3d(xMm, yMm, zMm));
}

double rigidTranslationDistanceMm(const cv::Matx44d& first,
                                  const cv::Matx44d& second) {
    return cv::norm(translationOf(first) - translationOf(second));
}

double rigidRotationDistanceDeg(const cv::Matx44d& first,
                                const cv::Matx44d& second) {
    return rotationDistanceDeg(rotationOf(first), rotationOf(second));
}

cv::Matx44d interpolateRigidHalf(const cv::Matx44d& first,
                                 const cv::Matx44d& second) {
    const cv::Vec3d translation = (translationOf(first) + translationOf(second)) * 0.5;
    const cv::Matx33d firstRotation = rotationOf(first);
    const cv::Matx33d delta = firstRotation.t() * rotationOf(second);
    cv::Vec3d deltaVector;
    cv::Rodrigues(cv::Mat(delta), deltaVector);
    cv::Mat halfRotationMat;
    cv::Rodrigues(deltaVector * 0.5, halfRotationMat);
    const cv::Matx33d halfRotation = matToMatx33(halfRotationMat);
    return transformFromRt(firstRotation * halfRotation, translation);
}

HandEyeSample::HandEyeSample()
    : baseFromFlange(cv::Matx44d::eye()),
      cameraFromBoard(cv::Matx44d::eye()),
      boardPoseRmsPx(0.0) {}

HandEyeOptions::HandEyeOptions()
    : minSamples(15), maxOutlierRounds(3), minTranslationSpanMm(50.0),
      minRotationSpanDeg(20.0), minSecondaryRotationSpreadDeg(5.0),
      outlierTranslationFloorMm(2.0),
      outlierRotationFloorDeg(0.5), madScale(3.5) {}

HandEyeSampleResult::HandEyeSampleResult()
    : accepted(false), baseBoardTranslationResidualMm(0.0),
      baseBoardRotationResidualDeg(0.0) {}

HandEyeCalibrationResult::HandEyeCalibrationResult()
    : ok(false), flangeFromCamera(cv::Matx44d::eye()),
      baseFromBoardMean(cv::Matx44d::eye()), inputSampleCount(0),
      acceptedSampleCount(0), rejectedSampleCount(0),
      translationSpanMm(0.0), rotationSpanDeg(0.0),
      secondaryRotationSpreadDeg(0.0) {}

bool calibrateHandEyeRobust(const std::vector<HandEyeSample>& samples,
                            const HandEyeOptions& options,
                            HandEyeCalibrationResult* result) {
    if (!result) {
        return false;
    }
    *result = HandEyeCalibrationResult();
    result->inputSampleCount = static_cast<int>(samples.size());
    result->samples.resize(samples.size());
    if (options.minSamples < 3 || options.maxOutlierRounds < 0) {
        result->error = "invalid hand-eye options";
        return false;
    }
    if (static_cast<int>(samples.size()) < options.minSamples) {
        std::ostringstream message;
        message << "too few hand-eye samples: " << samples.size()
                << " < " << options.minSamples;
        result->error = message.str();
        return false;
    }
    std::vector<int> active;
    active.reserve(samples.size());
    for (std::size_t index = 0; index < samples.size(); ++index) {
        result->samples[index].sampleId = samples[index].sampleId;
        if (!finiteTransform(samples[index].baseFromFlange) ||
            !finiteTransform(samples[index].cameraFromBoard)) {
            result->samples[index].rejectReason = "non-finite transform";
        } else {
            active.push_back(static_cast<int>(index));
        }
    }
    if (static_cast<int>(active.size()) < options.minSamples) {
        result->error = "too few finite hand-eye samples";
        return false;
    }
    motionSpans(samples, active, &result->translationSpanMm, &result->rotationSpanDeg,
                &result->secondaryRotationSpreadDeg);
    if (result->translationSpanMm < options.minTranslationSpanMm ||
        result->rotationSpanDeg < options.minRotationSpanDeg ||
        result->secondaryRotationSpreadDeg < options.minSecondaryRotationSpreadDeg) {
        std::ostringstream message;
        message << "insufficient robot pose diversity: translation span="
                << result->translationSpanMm << " mm, rotation span="
                << result->rotationSpanDeg << " deg, secondary-axis spread="
                << result->secondaryRotationSpreadDeg << " deg";
        result->error = message.str();
        return false;
    }

    MethodCandidate selected;
    for (int round = 0; round <= options.maxOutlierRounds; ++round) {
        selected = bestMethod(samples, active);
        if (!selected.ok) {
            result->error = "all OpenCV hand-eye methods failed";
            return false;
        }
        if (round == options.maxOutlierRounds ||
            static_cast<int>(active.size()) <= options.minSamples) {
            break;
        }
        const double translationThreshold = robustThreshold(
            selected.translations, options.outlierTranslationFloorMm, options.madScale);
        const double rotationThreshold = robustThreshold(
            selected.rotations, options.outlierRotationFloorDeg, options.madScale);
        int worstActiveIndex = -1;
        double worstScore = 1.0;
        for (std::size_t index = 0; index < active.size(); ++index) {
            const double score = std::max(
                selected.translations[index] / translationThreshold,
                selected.rotations[index] / rotationThreshold);
            if (score > worstScore) {
                worstScore = score;
                worstActiveIndex = static_cast<int>(index);
            }
        }
        if (worstActiveIndex < 0) {
            break;
        }
        const int removed = active[static_cast<std::size_t>(worstActiveIndex)];
        result->samples[static_cast<std::size_t>(removed)].rejectReason =
            "robust base-board consistency outlier";
        active.erase(active.begin() + worstActiveIndex);
    }

    // Re-solve after the last removal so output metrics always match active.
    selected = bestMethod(samples, active);
    if (!selected.ok) {
        result->error = "final OpenCV hand-eye solve failed";
        return false;
    }
    motionSpans(samples, active, &result->translationSpanMm, &result->rotationSpanDeg,
                &result->secondaryRotationSpreadDeg);
    if (result->translationSpanMm < options.minTranslationSpanMm ||
        result->rotationSpanDeg < options.minRotationSpanDeg ||
        result->secondaryRotationSpreadDeg < options.minSecondaryRotationSpreadDeg) {
        result->error = "pose diversity became insufficient after outlier rejection";
        return false;
    }
    result->method = selected.name;
    result->flangeFromCamera = selected.flangeFromCamera;
    result->baseFromBoardMean = selected.baseFromBoardMean;
    result->translationConsistencyMm = selected.translationMetrics;
    result->rotationConsistencyDeg = selected.rotationMetrics;
    result->acceptedSampleCount = static_cast<int>(active.size());
    result->rejectedSampleCount = result->inputSampleCount - result->acceptedSampleCount;
    for (std::size_t activeIndex = 0; activeIndex < active.size(); ++activeIndex) {
        HandEyeSampleResult& sampleResult =
            result->samples[static_cast<std::size_t>(active[activeIndex])];
        sampleResult.accepted = true;
        sampleResult.baseBoardTranslationResidualMm = selected.translations[activeIndex];
        sampleResult.baseBoardRotationResidualDeg = selected.rotations[activeIndex];
    }
    result->ok = true;
    return true;
}

HandEyeYamlMetadata::HandEyeYamlMetadata()
    : flangeFrame("fairino_flange_reported"), baseFrame("base_link") {}

bool saveHandEyeYaml(const std::string& path,
                     const HandEyeCalibrationResult& calibration,
                     const HandEyeYamlMetadata& metadata,
                     std::string* error) {
    if (error) {
        error->clear();
    }
    if (!calibration.ok || !finiteTransform(calibration.flangeFromCamera)) {
        if (error) {
            *error = "hand-eye result is not valid";
        }
        return false;
    }
    std::ofstream stream(path.c_str(), std::ios::out | std::ios::trunc);
    if (!stream) {
        if (error) {
            *error = "cannot open hand-eye YAML for writing: " + path;
        }
        return false;
    }
    stream << std::setprecision(12);
    stream << "schema_version: 1\n";
    stream << "calibration_type: eye_in_hand\n";
    stream << "mode: camera_to_flange\n";
    stream << "transform_convention: "
           << yamlQuote("T_parent_child maps child coordinates into parent coordinates") << "\n";
    stream << "parent_frame: " << yamlQuote(metadata.flangeFrame) << "\n";
    stream << "child_frame: " << yamlQuote(metadata.cameraFrame) << "\n";
    stream << "base_frame: " << yamlQuote(metadata.baseFrame) << "\n";
    stream << "translation_unit: mm\n";
    stream << "rotation_unit: dimensionless\n";
    stream << "T_flange_camera: [";
    for (int index = 0; index < 16; ++index) {
        if (index) {
            stream << ", ";
        }
        stream << calibration.flangeFromCamera.val[index];
    }
    stream << "]\n";
    stream << "solver:\n";
    stream << "  selected_opencv_method: " << yamlQuote(calibration.method) << "\n";
    stream << "  input_samples: " << calibration.inputSampleCount << "\n";
    stream << "  accepted_samples: " << calibration.acceptedSampleCount << "\n";
    stream << "  rejected_samples: " << calibration.rejectedSampleCount << "\n";
    stream << "  translation_span_mm: " << calibration.translationSpanMm << "\n";
    stream << "  rotation_span_deg: " << calibration.rotationSpanDeg << "\n";
    stream << "  secondary_rotation_spread_deg: "
           << calibration.secondaryRotationSpreadDeg << "\n";
    stream << "quality:\n";
    stream << "  base_board_translation_rms_mm: "
           << calibration.translationConsistencyMm.rms << "\n";
    stream << "  base_board_translation_p95_mm: "
           << calibration.translationConsistencyMm.p95 << "\n";
    stream << "  base_board_translation_max_mm: "
           << calibration.translationConsistencyMm.maximum << "\n";
    stream << "  base_board_rotation_rms_deg: "
           << calibration.rotationConsistencyDeg.rms << "\n";
    stream << "  base_board_rotation_p95_deg: "
           << calibration.rotationConsistencyDeg.p95 << "\n";
    stream << "  base_board_rotation_max_deg: "
           << calibration.rotationConsistencyDeg.maximum << "\n";
    stream << "camera:\n";
    stream << "  model: " << yamlQuote(metadata.cameraModel) << "\n";
    stream << "  serial: " << yamlQuote(metadata.cameraSerial) << "\n";
    stream << "sources:\n";
    stream << "  intrinsics_file: " << yamlQuote(metadata.intrinsicsFile) << "\n";
    stream << "  intrinsics_sha256: " << yamlQuote(metadata.intrinsicsSha256) << "\n";
    stream << "  dataset_manifest: " << yamlQuote(metadata.datasetManifest) << "\n";
    stream << "  dataset_manifest_sha256: "
           << yamlQuote(metadata.datasetManifestSha256) << "\n";
    stream << "generated_at: " << yamlQuote(metadata.generatedAt) << "\n";
    stream << "samples:\n";
    for (std::size_t index = 0; index < calibration.samples.size(); ++index) {
        const HandEyeSampleResult& sample = calibration.samples[index];
        stream << "  - id: " << yamlQuote(sample.sampleId) << "\n";
        stream << "    accepted: " << (sample.accepted ? "true" : "false") << "\n";
        if (sample.accepted) {
            stream << "    base_board_translation_residual_mm: "
                   << sample.baseBoardTranslationResidualMm << "\n";
            stream << "    base_board_rotation_residual_deg: "
                   << sample.baseBoardRotationResidualDeg << "\n";
        } else {
            stream << "    reason: " << yamlQuote(sample.rejectReason) << "\n";
        }
    }
    if (!stream.good()) {
        if (error) {
            *error = "failed while writing hand-eye YAML: " + path;
        }
        return false;
    }
    return true;
}

}  // namespace hik_calibration
