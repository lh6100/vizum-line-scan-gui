#include "HikCalibrationCore.h"
#include "StripeCenterlineExtractor.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

struct CommandLine {
    fs::path input;
    fs::path output;
    hik_stripe::Orientation orientation{hik_stripe::Orientation::Auto};
    cv::Rect roi;
    bool roiSet{false};
    bool morphologyResponse{true};
    int backgroundWidth{31};
    int backgroundHeight{3};
    int minimumRawIntensity{60};
    double ambiguityMarginPerPoint{-1.0};
    double ambiguityMinimumSeparationPx{-1.0};
    int ambiguityPaddingScanlines{-1};
    bool writeOverlays{true};
};

struct Statistics {
    double p50{std::numeric_limits<double>::quiet_NaN()};
    double p95{std::numeric_limits<double>::quiet_NaN()};
    double maximum{std::numeric_limits<double>::quiet_NaN()};
};

struct RobustOffsetStatistics {
    std::size_t inlierCount{0U};
    std::size_t grossMismatchCount{0U};
    double signedMedian{std::numeric_limits<double>::quiet_NaN()};
    double robustSignedMean{std::numeric_limits<double>::quiet_NaN()};
    double gate{std::numeric_limits<double>::quiet_NaN()};
};

struct RejectDescriptor {
    hik_stripe::RejectReason flag;
    const char* csvName;
};

const std::array<RejectDescriptor, 13> kRejectDescriptors{{
    {hik_stripe::REJECT_LOW_PROMINENCE, "reject_low_prominence"},
    {hik_stripe::REJECT_WIDTH_OUT_OF_RANGE, "reject_width"},
    {hik_stripe::REJECT_SATURATED_WIDE_PLATEAU, "reject_saturated_wide"},
    {hik_stripe::REJECT_SATURATED_ASYMMETRIC, "reject_saturated_asymmetric"},
    {hik_stripe::REJECT_MULTI_PEAK_AMBIGUOUS, "reject_multi_peak"},
    {hik_stripe::REJECT_PROFILE_ASYMMETRIC, "reject_profile_asymmetric"},
    {hik_stripe::REJECT_FIT_RESIDUAL_HIGH, "reject_fit"},
    {hik_stripe::REJECT_QUALITY_LOW, "reject_quality"},
    {hik_stripe::REJECT_OUTSIDE_ROI, "reject_outside_roi"},
    {hik_stripe::REJECT_OUTSIDE_VALIDITY_MASK, "reject_mask"},
    {hik_stripe::REJECT_PATH_JUMP, "reject_path_jump"},
    {hik_stripe::REJECT_PATH_AMBIGUOUS, "reject_path_ambiguous"},
    {hik_stripe::REJECT_AMBIGUOUS_MULTIPATH,
     "reject_ambiguous_multipath"}
}};

void printUsage(const char* executable) {
    std::cerr
        << "Usage: " << executable
        << " --input <png-or-directory> --output <directory> [options]\n"
        << "Options:\n"
        << "  --orientation auto|horizontal|vertical\n"
        << "  --roi x,y,width,height          Pixel ROI in calibrated image coordinates\n"
        << "  --response-mode morphology|identity\n"
        << "  --background-width <odd>        Default: 31\n"
        << "  --background-height <odd>       Default: 3\n"
        << "  --minimum-raw <0..255>           Default: 60\n"
        << "  --ambiguity-margin <cost/point>  Override local multipath margin\n"
        << "  --ambiguity-min-separation <px>  Override branch separation gate\n"
        << "  --ambiguity-padding <scanlines>  Override interval protection band\n"
        << "  --no-overlay                    Do not write diagnostic PNG overlays\n"
        << "\nThe input PNG files are opened read-only. All generated files are placed\n"
        << "under the explicitly supplied output directory.\n";
}

bool parseInteger(const std::string& text, int* value) {
    if (!value || text.empty()) {
        return false;
    }
    std::size_t consumed = 0;
    try {
        const int parsed = std::stoi(text, &consumed);
        if (consumed != text.size()) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parseDouble(const std::string& text, double* value) {
    if (!value || text.empty()) {
        return false;
    }
    std::size_t consumed = 0U;
    try {
        const double parsed = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(parsed)) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parseRoi(const std::string& text, cv::Rect* roi) {
    if (!roi) {
        return false;
    }
    std::array<int, 4> values{};
    std::stringstream stream(text);
    std::string token;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!std::getline(stream, token, ',') ||
            !parseInteger(token, &values[index])) {
            return false;
        }
    }
    if (std::getline(stream, token, ',')) {
        return false;
    }
    if (values[2] <= 0 || values[3] <= 0) {
        return false;
    }
    *roi = cv::Rect(values[0], values[1], values[2], values[3]);
    return true;
}

bool parseOrientation(const std::string& text,
                      hik_stripe::Orientation* orientation) {
    if (!orientation) {
        return false;
    }
    if (text == "auto") {
        *orientation = hik_stripe::Orientation::Auto;
        return true;
    }
    if (text == "horizontal") {
        *orientation = hik_stripe::Orientation::Horizontal;
        return true;
    }
    if (text == "vertical") {
        *orientation = hik_stripe::Orientation::Vertical;
        return true;
    }
    return false;
}

bool parseCommandLine(int argc,
                      char** argv,
                      CommandLine* command,
                      std::string* error) {
    if (!command) {
        return false;
    }
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        const auto requireValue = [&](const char* option, std::string* value) {
            if (index + 1 >= argc) {
                if (error) {
                    *error = std::string(option) + " requires a value";
                }
                return false;
            }
            *value = argv[++index];
            return true;
        };

        std::string value;
        if (argument == "--input") {
            if (!requireValue("--input", &value)) {
                return false;
            }
            command->input = fs::path(value);
        } else if (argument == "--output") {
            if (!requireValue("--output", &value)) {
                return false;
            }
            command->output = fs::path(value);
        } else if (argument == "--orientation") {
            if (!requireValue("--orientation", &value) ||
                !parseOrientation(value, &command->orientation)) {
                if (error && error->empty()) {
                    *error = "invalid --orientation value";
                }
                return false;
            }
        } else if (argument == "--roi") {
            if (!requireValue("--roi", &value) ||
                !parseRoi(value, &command->roi)) {
                if (error && error->empty()) {
                    *error = "invalid --roi; expected x,y,width,height";
                }
                return false;
            }
            command->roiSet = true;
        } else if (argument == "--response-mode") {
            if (!requireValue("--response-mode", &value)) {
                return false;
            }
            if (value == "morphology") {
                command->morphologyResponse = true;
            } else if (value == "identity") {
                command->morphologyResponse = false;
            } else {
                if (error) {
                    *error = "--response-mode must be morphology or identity";
                }
                return false;
            }
        } else if (argument == "--background-width") {
            if (!requireValue("--background-width", &value) ||
                !parseInteger(value, &command->backgroundWidth)) {
                if (error && error->empty()) {
                    *error = "invalid --background-width";
                }
                return false;
            }
        } else if (argument == "--background-height") {
            if (!requireValue("--background-height", &value) ||
                !parseInteger(value, &command->backgroundHeight)) {
                if (error && error->empty()) {
                    *error = "invalid --background-height";
                }
                return false;
            }
        } else if (argument == "--minimum-raw") {
            if (!requireValue("--minimum-raw", &value) ||
                !parseInteger(value, &command->minimumRawIntensity)) {
                if (error && error->empty()) {
                    *error = "invalid --minimum-raw";
                }
                return false;
            }
        } else if (argument == "--ambiguity-margin") {
            if (!requireValue("--ambiguity-margin", &value) ||
                !parseDouble(value, &command->ambiguityMarginPerPoint) ||
                command->ambiguityMarginPerPoint < 0.0) {
                if (error && error->empty()) {
                    *error = "invalid --ambiguity-margin";
                }
                return false;
            }
        } else if (argument == "--ambiguity-min-separation") {
            if (!requireValue(
                    "--ambiguity-min-separation", &value) ||
                !parseDouble(
                    value,
                    &command->ambiguityMinimumSeparationPx) ||
                command->ambiguityMinimumSeparationPx <= 0.0) {
                if (error && error->empty()) {
                    *error =
                        "invalid --ambiguity-min-separation";
                }
                return false;
            }
        } else if (argument == "--ambiguity-padding") {
            if (!requireValue("--ambiguity-padding", &value) ||
                !parseInteger(
                    value,
                    &command->ambiguityPaddingScanlines) ||
                command->ambiguityPaddingScanlines < 0) {
                if (error && error->empty()) {
                    *error = "invalid --ambiguity-padding";
                }
                return false;
            }
        } else if (argument == "--no-overlay") {
            command->writeOverlays = false;
        } else if (argument == "--help" || argument == "-h") {
            return false;
        } else {
            if (error) {
                *error = "unknown argument: " + argument;
            }
            return false;
        }
    }

    if (command->input.empty() || command->output.empty()) {
        if (error) {
            *error = "--input and --output are required";
        }
        return false;
    }
    if (command->backgroundWidth < 5 ||
        command->backgroundWidth % 2 == 0 ||
        command->backgroundHeight < 1 ||
        command->backgroundHeight % 2 == 0 ||
        command->minimumRawIntensity < 0 ||
        command->minimumRawIntensity > 255) {
        if (error) {
            *error = "morphology dimensions must be positive odd values and "
                     "--minimum-raw must be in 0..255";
        }
        return false;
    }
    return true;
}

std::string lowerExtension(const fs::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });
    return extension;
}

bool pathIsInside(const fs::path& path, const fs::path& possibleParent) {
    const fs::path relative = path.lexically_relative(possibleParent);
    if (relative.empty() || relative == ".") {
        return true;
    }
    const std::string relativeText = relative.generic_string();
    return relativeText != ".." &&
           relativeText.rfind("../", 0) != 0;
}

bool collectInputFiles(const CommandLine& command,
                       std::vector<fs::path>* files,
                       std::string* error) {
    if (!files) {
        return false;
    }
    files->clear();
    std::error_code statusError;
    const fs::file_status inputStatus =
        fs::status(command.input, statusError);
    if (statusError) {
        if (error) {
            *error = "cannot inspect input: " + statusError.message();
        }
        return false;
    }
    if (fs::is_regular_file(inputStatus)) {
        if (lowerExtension(command.input) != ".png") {
            if (error) {
                *error = "input file is not PNG: " + command.input.string();
            }
            return false;
        }
        files->push_back(command.input);
    } else if (fs::is_directory(inputStatus)) {
        std::error_code canonicalError;
        const fs::path outputAbsolute =
            fs::weakly_canonical(command.output, canonicalError);
        if (canonicalError) {
            canonicalError.clear();
        }
        const fs::recursive_directory_iterator end;
        for (fs::recursive_directory_iterator iterator(
                 command.input, fs::directory_options::skip_permission_denied,
                 statusError);
             iterator != end;
             iterator.increment(statusError)) {
            if (statusError) {
                statusError.clear();
                continue;
            }
            if (!iterator->is_regular_file(statusError) ||
                lowerExtension(iterator->path()) != ".png") {
                statusError.clear();
                continue;
            }
            const fs::path candidateAbsolute =
                fs::weakly_canonical(iterator->path(), canonicalError);
            if (!canonicalError && !outputAbsolute.empty() &&
                pathIsInside(candidateAbsolute, outputAbsolute)) {
                continue;
            }
            canonicalError.clear();
            files->push_back(iterator->path());
        }
    } else {
        if (error) {
            *error = "input is neither a PNG file nor a directory";
        }
        return false;
    }
    std::sort(files->begin(), files->end());
    if (files->empty()) {
        if (error) {
            *error = "no PNG files found under input";
        }
        return false;
    }
    return true;
}

cv::Mat buildResponse(const cv::Mat& raw, const CommandLine& command) {
    if (!command.morphologyResponse) {
        return raw.clone();
    }
    const cv::Mat horizontalKernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(command.backgroundWidth, command.backgroundHeight));
    const cv::Mat verticalKernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(command.backgroundHeight, command.backgroundWidth));
    cv::Mat horizontalBackground;
    cv::Mat verticalBackground;
    cv::morphologyEx(raw, horizontalBackground, cv::MORPH_OPEN,
                     horizontalKernel, cv::Point(-1, -1), 1,
                     cv::BORDER_REPLICATE);
    cv::morphologyEx(raw, verticalBackground, cv::MORPH_OPEN,
                     verticalKernel, cv::Point(-1, -1), 1,
                     cv::BORDER_REPLICATE);
    cv::Mat background;
    cv::min(horizontalBackground, verticalBackground, background);
    cv::Mat response;
    cv::subtract(raw, background, response);
    if (command.minimumRawIntensity > 0) {
        cv::Mat darkMask;
        cv::compare(raw, command.minimumRawIntensity, darkMask, cv::CMP_LT);
        response.setTo(0, darkMask);
    }
    return response;
}

Statistics statistics(std::vector<double> values) {
    Statistics output;
    if (values.empty()) {
        return output;
    }
    std::sort(values.begin(), values.end());
    const auto percentile = [&values](double fraction) {
        const double position = fraction *
            static_cast<double>(values.size() - 1);
        const std::size_t lower =
            static_cast<std::size_t>(std::floor(position));
        const std::size_t upper =
            static_cast<std::size_t>(std::ceil(position));
        const double alpha = position - static_cast<double>(lower);
        return values[lower] * (1.0 - alpha) + values[upper] * alpha;
    };
    output.p50 = percentile(0.50);
    output.p95 = percentile(0.95);
    output.maximum = values.back();
    return output;
}

double medianValue(std::vector<double> values) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return (values.size() & 1U) != 0U
        ? values[middle]
        : 0.5 * (values[middle - 1U] + values[middle]);
}

RobustOffsetStatistics robustOffsetStatistics(
        const std::vector<double>& signedOffsets) {
    RobustOffsetStatistics result;
    if (signedOffsets.empty()) {
        return result;
    }
    result.signedMedian = medianValue(signedOffsets);
    std::vector<double> deviations;
    deviations.reserve(signedOffsets.size());
    for (const double offset : signedOffsets) {
        deviations.push_back(
            std::fabs(offset - result.signedMedian));
    }
    const double scaledMad =
        1.4826 * medianValue(std::move(deviations));
    result.gate = std::max(0.5, 4.0 * scaledMad);
    const double grossGate = std::max(2.0, result.gate);
    long double robustSum = 0.0L;
    for (const double offset : signedOffsets) {
        const double centered =
            std::fabs(offset - result.signedMedian);
        if (centered <= result.gate) {
            robustSum += offset;
            ++result.inlierCount;
        }
        if (centered > grossGate) {
            ++result.grossMismatchCount;
        }
    }
    if (result.inlierCount > 0U) {
        result.robustSignedMean = static_cast<double>(
            robustSum /
            static_cast<long double>(result.inlierCount));
    }
    return result;
}

std::string csvQuote(const std::string& text) {
    std::string escaped;
    escaped.reserve(text.size() + 2);
    escaped.push_back('"');
    for (const char character : text) {
        if (character == '"') {
            escaped.push_back('"');
        }
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

double normalCoordinate(const hik_stripe::Candidate& point,
                        hik_stripe::Orientation orientation) {
    return orientation == hik_stripe::Orientation::Horizontal
        ? point.pixel.y : point.pixel.x;
}

int oldScanIndex(const hik_calibration::StripePoint& point,
                 hik_stripe::Orientation orientation) {
    return orientation == hik_stripe::Orientation::Horizontal
        ? static_cast<int>(std::lround(point.pixel.x))
        : static_cast<int>(std::lround(point.pixel.y));
}

double oldNormalCoordinate(const hik_calibration::StripePoint& point,
                           hik_stripe::Orientation orientation) {
    return orientation == hik_stripe::Orientation::Horizontal
        ? point.pixel.y : point.pixel.x;
}

std::vector<hik_calibration::StripePoint> extractLegacy(
        const cv::Mat& response,
        hik_stripe::Orientation orientation,
        std::string* warning) {
    hik_calibration::StripeExtractionOptions options;
    options.minimumDifference = 10;
    options.thresholdStddevScale = 2.0;
    options.minPointCount = 1;
    std::vector<hik_calibration::StripePoint> points;
    std::string error;
    bool ok = false;
    if (orientation == hik_stripe::Orientation::Horizontal) {
        ok = hik_calibration::extractHorizontalLaserStripe(
            response, options, &points, &error);
    } else if (orientation == hik_stripe::Orientation::Vertical) {
        ok = hik_calibration::extractVerticalLaserStripe(
            response, options, &points, &error);
    } else {
        ok = hik_calibration::extractLaserStripe(
            response, options, &points, &error);
    }
    if (!ok && warning) {
        *warning = error;
    }
    return points;
}

bool writeCandidateCsv(
        const fs::path& path,
        const hik_stripe::Result& result,
        const std::map<int, double>& oldCenters,
        std::string* error) {
    std::ofstream output(path);
    if (!output) {
        if (error) {
            *error = "cannot create candidate CSV: " + path.string();
        }
        return false;
    }
    output << "scan_index,peak_index,pixel_x,pixel_y,path_usable,"
              "provisional_selected,publishable_selected,"
              "ambiguity_interval_id,ambiguity_branch_id,"
              "branch_memberships,has_alternate,alternate_pixel_x,"
              "alternate_pixel_y,branch_separation_px,local_cost_margin,"
              "raw_peak,response_peak,local_baseline,local_noise_mad,"
              "prominence,snr,fwhm_px,saturated_fraction,"
              "saturated_plateau_width_px,second_peak_ratio,"
              "gradient_asymmetry,fit_residual,quality,center_sigma_px,"
              "center_method,taylor_offset_px,smoothed_first_derivative,"
              "smoothed_second_derivative,"
              "reject_flags,reject_reasons,legacy_center,absolute_offset_px\n";

    using CandidateKey = std::pair<int, int>;
    std::map<CandidateKey, hik_stripe::Candidate> provisional;
    for (const hik_stripe::Candidate& point :
         result.provisionalSelected) {
        provisional[std::make_pair(
            point.scanIndex, point.peakIndex)] = point;
    }
    std::map<CandidateKey, hik_stripe::Candidate> selected;
    for (const hik_stripe::Candidate& point : result.selected) {
        selected[std::make_pair(point.scanIndex, point.peakIndex)] =
            point;
    }
    std::map<int, hik_stripe::PathScanlineDiagnostic> pathDiagnostics;
    for (const hik_stripe::PathScanlineDiagnostic& diagnostic :
         result.pathDiagnostics) {
        pathDiagnostics[diagnostic.scanIndex] = diagnostic;
    }
    std::map<CandidateKey, std::vector<std::pair<int, int>>>
        branchMemberships;
    for (const hik_stripe::MultipathInterval& interval :
         result.multipathIntervals) {
        for (const hik_stripe::MultipathBranch& branch :
             interval.branches) {
            for (const hik_stripe::Candidate& candidate :
                 branch.candidates) {
                branchMemberships[std::make_pair(
                    candidate.scanIndex,
                    candidate.peakIndex)].push_back(
                        std::make_pair(
                            interval.intervalId,
                            branch.branchId));
            }
        }
    }
    output << std::setprecision(12);
    for (const hik_stripe::Candidate& candidate : result.candidates) {
        const CandidateKey candidateKey =
            std::make_pair(candidate.scanIndex, candidate.peakIndex);
        const std::map<CandidateKey, hik_stripe::Candidate>::const_iterator
            provisionalEntry = provisional.find(candidateKey);
        const std::map<CandidateKey, hik_stripe::Candidate>::const_iterator
            selectedEntry = selected.find(candidateKey);
        const std::uint32_t effectiveFlags =
            candidate.rejectFlags |
            (provisionalEntry == provisional.end()
                 ? 0U : provisionalEntry->second.rejectFlags) |
            (selectedEntry == selected.end()
                 ? 0U : selectedEntry->second.rejectFlags);
        const hik_stripe::Candidate* classified =
            provisionalEntry != provisional.end()
            ? &provisionalEntry->second
            : &candidate;
        const std::map<int, hik_stripe::PathScanlineDiagnostic>::
            const_iterator diagnostic =
                pathDiagnostics.find(candidate.scanIndex);
        std::ostringstream memberships;
        const std::map<CandidateKey,
                       std::vector<std::pair<int, int>>>::const_iterator
            membershipEntry = branchMemberships.find(candidateKey);
        if (membershipEntry != branchMemberships.end()) {
            for (std::size_t index = 0U;
                 index < membershipEntry->second.size(); ++index) {
                if (index > 0U) {
                    memberships << '|';
                }
                memberships
                    << membershipEntry->second[index].first
                    << ':'
                    << membershipEntry->second[index].second;
            }
        }
        const std::map<int, double>::const_iterator old =
            oldCenters.find(candidate.scanIndex);
        const double coordinate =
            normalCoordinate(candidate, result.orientation);
        output << candidate.scanIndex << ','
               << candidate.peakIndex << ','
               << candidate.pixel.x << ','
               << candidate.pixel.y << ','
               << (candidate.usableForPath() ? 1 : 0) << ','
               << (provisionalEntry != provisional.end() ? 1 : 0)
               << ','
               << (selectedEntry != selected.end() ? 1 : 0)
               << ',' << classified->ambiguityIntervalId
               << ',' << classified->ambiguityBranchId
               << ',' << csvQuote(memberships.str())
               << ','
               << (diagnostic != pathDiagnostics.end() &&
                           diagnostic->second.hasAlternate
                       ? 1 : 0);
        if (diagnostic != pathDiagnostics.end() &&
            diagnostic->second.hasAlternate) {
            output << ',' << diagnostic->second.alternatePixel.x
                   << ',' << diagnostic->second.alternatePixel.y
                   << ',' << diagnostic->second.separationPx
                   << ',' << diagnostic->second.localCostMargin;
        } else {
            output << ",,,,";
        }
        output
               << ',' << candidate.rawPeak
               << ',' << candidate.responsePeak
               << ',' << candidate.localBaseline
               << ',' << candidate.localNoiseMad
               << ',' << candidate.prominence
               << ',' << candidate.snr
               << ',' << candidate.fwhmPx
               << ',' << candidate.saturatedFraction
               << ',' << candidate.saturatedPlateauWidthPx
               << ',' << candidate.secondPeakRatio
               << ',' << candidate.gradientAsymmetry
               << ',' << candidate.fitResidual
               << ',' << candidate.quality
               << ',' << candidate.centerSigmaPx
               << ',' << hik_stripe::centerMethodName(
                              candidate.centerMethod)
               << ',' << candidate.taylorOffsetPx
               << ',' << candidate.smoothedFirstDerivative
               << ',' << candidate.smoothedSecondDerivative
               << ',' << effectiveFlags
               << ',' << csvQuote(hik_stripe::rejectReasonNames(
                                   effectiveFlags));
        if (old == oldCenters.end()) {
            output << ",,";
        } else {
            output << ',' << old->second
                   << ',' << std::fabs(coordinate - old->second);
        }
        output << '\n';
    }
    if (!output) {
        if (error) {
            *error = "failed while writing candidate CSV: " + path.string();
        }
        return false;
    }
    return true;
}

bool writeMultipathCsv(
        const fs::path& path,
        const hik_stripe::Result& result,
        std::string* error) {
    std::ofstream output(path);
    if (!output) {
        if (error) {
            *error = "cannot create multipath CSV: " + path.string();
        }
        return false;
    }
    output
        << "interval_id,first_scan_index,last_scan_index,"
           "core_first_scan_index,core_last_scan_index,"
           "left_anchor_scan_index,right_anchor_scan_index,"
           "left_boundary_open,right_boundary_open,"
           "minimum_local_cost_margin,maximum_separation_px,"
           "branch_id,branch_path_cost,branch_candidate_count,"
           "branch_first_scan_index,branch_last_scan_index\n";
    output << std::setprecision(12);
    for (const hik_stripe::MultipathInterval& interval :
         result.multipathIntervals) {
        for (const hik_stripe::MultipathBranch& branch :
             interval.branches) {
            int branchFirst = -1;
            int branchLast = -1;
            for (const hik_stripe::Candidate& candidate :
                 branch.candidates) {
                if (branchFirst < 0 ||
                    candidate.scanIndex < branchFirst) {
                    branchFirst = candidate.scanIndex;
                }
                branchLast = std::max(
                    branchLast, candidate.scanIndex);
            }
            output << interval.intervalId << ','
                   << interval.firstScanIndex << ','
                   << interval.lastScanIndex << ','
                   << interval.coreFirstScanIndex << ','
                   << interval.coreLastScanIndex << ','
                   << interval.leftAnchorScanIndex << ','
                   << interval.rightAnchorScanIndex << ','
                   << (interval.leftBoundaryOpen ? 1 : 0) << ','
                   << (interval.rightBoundaryOpen ? 1 : 0) << ','
                   << interval.minimumLocalCostMargin << ','
                   << interval.maximumSeparationPx << ','
                   << branch.branchId << ','
                   << branch.pathCost << ','
                   << branch.candidates.size() << ','
                   << branchFirst << ','
                   << branchLast << '\n';
        }
    }
    if (!output) {
        if (error) {
            *error =
                "failed while writing multipath CSV: " +
                path.string();
        }
        return false;
    }
    return true;
}

void drawCross(cv::Mat* image,
               const cv::Point2d& point,
               const cv::Scalar& color,
               int radius,
               int thickness) {
    if (!image) {
        return;
    }
    const cv::Point center(
        static_cast<int>(std::lround(point.x)),
        static_cast<int>(std::lround(point.y)));
    cv::line(*image, center + cv::Point(-radius, 0),
             center + cv::Point(radius, 0), color, thickness, cv::LINE_AA);
    cv::line(*image, center + cv::Point(0, -radius),
             center + cv::Point(0, radius), color, thickness, cv::LINE_AA);
}

bool writeOverlay(const fs::path& path,
                  const cv::Mat& raw,
                  const std::vector<hik_calibration::StripePoint>& legacy,
                  const hik_stripe::Result& quality,
                  std::string* error) {
    cv::Mat overlay;
    cv::cvtColor(raw, overlay, cv::COLOR_GRAY2BGR);
    for (const hik_stripe::Candidate& candidate : quality.candidates) {
        if (!candidate.usableForPath()) {
            drawCross(&overlay, candidate.pixel, cv::Scalar(0, 0, 190), 1, 1);
        }
    }
    for (const hik_calibration::StripePoint& point : legacy) {
        drawCross(&overlay, point.pixel, cv::Scalar(0, 220, 0), 1, 1);
    }
    for (const hik_stripe::Candidate& point : quality.selected) {
        drawCross(&overlay, point.pixel, cv::Scalar(0, 220, 255), 2, 1);
    }
    for (const hik_stripe::MultipathInterval& interval :
         quality.multipathIntervals) {
        cv::Rect band;
        if (quality.orientation == hik_stripe::Orientation::Horizontal) {
            band = cv::Rect(
                interval.firstScanIndex,
                quality.diagnostics.appliedRoi.y,
                interval.lastScanIndex -
                    interval.firstScanIndex + 1,
                quality.diagnostics.appliedRoi.height);
        } else {
            band = cv::Rect(
                quality.diagnostics.appliedRoi.x,
                interval.firstScanIndex,
                quality.diagnostics.appliedRoi.width,
                interval.lastScanIndex -
                    interval.firstScanIndex + 1);
        }
        band &= cv::Rect(0, 0, overlay.cols, overlay.rows);
        if (!band.empty()) {
            cv::Mat tinted = overlay.clone();
            cv::rectangle(
                tinted, band, cv::Scalar(180, 0, 180),
                cv::FILLED);
            cv::addWeighted(
                tinted, 0.18, overlay, 0.82, 0.0, overlay);
            cv::rectangle(
                overlay, band, cv::Scalar(255, 0, 255),
                1, cv::LINE_AA);
        }
        for (const hik_stripe::MultipathBranch& branch :
             interval.branches) {
            for (const hik_stripe::Candidate& point :
                 branch.candidates) {
                drawCross(
                    &overlay, point.pixel,
                    cv::Scalar(255, 0, 255), 2, 1);
            }
        }
    }
    cv::rectangle(overlay, quality.diagnostics.appliedRoi,
                  cv::Scalar(255, 120, 0), 1, cv::LINE_AA);
    const std::string legend =
        "legacy=green publishable=yellow multipath=magenta fatal=red";
    cv::putText(overlay, legend, cv::Point(12, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    if (!cv::imwrite(path.string(), overlay)) {
        if (error) {
            *error = "cannot write overlay PNG: " + path.string();
        }
        return false;
    }
    return true;
}

std::string finiteOrEmpty(double value) {
    if (!std::isfinite(value)) {
        return std::string();
    }
    std::ostringstream stream;
    stream << std::setprecision(12) << value;
    return stream.str();
}

}  // namespace

int main(int argc, char** argv) {
    CommandLine command;
    std::string error;
    if (!parseCommandLine(argc, argv, &command, &error)) {
        if (!error.empty()) {
            std::cerr << "Error: " << error << "\n\n";
        }
        printUsage(argv[0]);
        return error.empty() ? 0 : 2;
    }

    std::vector<fs::path> inputFiles;
    if (!collectInputFiles(command, &inputFiles, &error)) {
        std::cerr << "Error: " << error << '\n';
        return 2;
    }

    std::error_code directoryError;
    fs::create_directories(command.output / "csv", directoryError);
    if (!directoryError && command.writeOverlays) {
        fs::create_directories(command.output / "overlay", directoryError);
    }
    if (directoryError) {
        std::cerr << "Error: cannot create output directory: "
                  << directoryError.message() << '\n';
        return 2;
    }

    const fs::path summaryPath = command.output / "stripe_quality_summary.csv";
    std::ofstream summary(summaryPath);
    if (!summary) {
        std::cerr << "Error: cannot create " << summaryPath << '\n';
        return 2;
    }
    summary
        << "input,width,height,orientation,legacy_points,quality_points,"
           "provisional_points,publishable_points,"
           "matched_points,offset_signed_mean_px,offset_p50_px,"
           "offset_p95_px,offset_max_px,offset_robust_matched_points,"
           "offset_gross_mismatch_points,offset_signed_median_px,"
           "offset_robust_signed_mean_px,offset_robust_gate_px,"
           "total_candidates,path_usable_candidates,"
           "fatal_rejected_candidates,multipath_interval_count,"
           "multipath_ambiguous_scanlines,"
           "multipath_branch_candidate_count,"
           "minimum_multipath_local_margin,"
           "maximum_multipath_separation_px,"
           "saturated_candidate_ratio,"
           "multi_peak_scanline_ratio,selected_saturated_ratio,selected_gaps,"
           "mean_selected_quality,mean_selected_fwhm_px,mean_selected_snr,"
           "mean_selected_gradient_asymmetry,mean_selected_fit_residual,"
           "mean_selected_second_peak_ratio,path_margin_per_point";
    for (const RejectDescriptor& descriptor : kRejectDescriptors) {
        summary << ',' << descriptor.csvName;
    }
    summary << ",legacy_warning,quality_error\n";

    std::vector<double> allOffsets;
    std::vector<double> allSignedOffsets;
    long double allSignedOffsetSum = 0.0L;
    std::size_t allSignedOffsetCount = 0U;
    std::size_t processedCount = 0;
    std::size_t failedCount = 0;
    for (std::size_t fileIndex = 0; fileIndex < inputFiles.size(); ++fileIndex) {
        const fs::path& input = inputFiles[fileIndex];
        const cv::Mat raw = cv::imread(input.string(), cv::IMREAD_GRAYSCALE);
        if (raw.empty()) {
            std::cerr << "Warning: cannot decode " << input << '\n';
            ++failedCount;
            continue;
        }
        const cv::Mat response = buildResponse(raw, command);

        hik_stripe::Options qualityOptions;
        qualityOptions.orientation = command.orientation;
        qualityOptions.roi = command.roiSet
            ? command.roi : cv::Rect(0, 0, raw.cols, raw.rows);
        if (command.ambiguityMarginPerPoint >= 0.0) {
            qualityOptions.pathAmbiguityMarginPerPoint =
                command.ambiguityMarginPerPoint;
        }
        if (command.ambiguityMinimumSeparationPx > 0.0) {
            qualityOptions.pathAmbiguityMinimumSeparationPx =
                command.ambiguityMinimumSeparationPx;
        }
        if (command.ambiguityPaddingScanlines >= 0) {
            qualityOptions.pathAmbiguityPaddingScanlines =
                command.ambiguityPaddingScanlines;
        }
        hik_stripe::Result quality;
        const bool qualityOk = hik_stripe::extractCenterline(
            response, raw, qualityOptions, &quality);

        hik_stripe::Orientation comparisonOrientation = quality.orientation;
        if (comparisonOrientation == hik_stripe::Orientation::Auto) {
            comparisonOrientation = command.orientation;
        }
        std::string legacyWarning;
        const std::vector<hik_calibration::StripePoint> legacy =
            extractLegacy(response, comparisonOrientation, &legacyWarning);
        std::map<int, double> oldCenters;
        for (const hik_calibration::StripePoint& point : legacy) {
            oldCenters[oldScanIndex(point, comparisonOrientation)] =
                oldNormalCoordinate(point, comparisonOrientation);
        }

        std::vector<double> offsets;
        std::vector<double> signedOffsets;
        long double signedOffsetSum = 0.0L;
        for (const hik_stripe::Candidate& point : quality.selected) {
            const std::map<int, double>::const_iterator old =
                oldCenters.find(point.scanIndex);
            if (old == oldCenters.end()) {
                continue;
            }
            const double signedOffset =
                normalCoordinate(point, comparisonOrientation) -
                old->second;
            const double absoluteOffset = std::fabs(signedOffset);
            signedOffsetSum += signedOffset;
            allSignedOffsetSum += signedOffset;
            ++allSignedOffsetCount;
            signedOffsets.push_back(signedOffset);
            allSignedOffsets.push_back(signedOffset);
            offsets.push_back(absoluteOffset);
            allOffsets.push_back(absoluteOffset);
        }
        const Statistics offsetStats = statistics(offsets);
        const RobustOffsetStatistics robustStats =
            robustOffsetStatistics(signedOffsets);
        const double signedMeanOffset = offsets.empty()
            ? std::numeric_limits<double>::quiet_NaN()
            : static_cast<double>(
                  signedOffsetSum /
                  static_cast<long double>(offsets.size()));

        std::array<std::size_t, kRejectDescriptors.size()> rejectCounts{};
        std::size_t fatalRejectedCandidates = 0U;
        for (const hik_stripe::Candidate& candidate : quality.candidates) {
            if (!candidate.usableForPath()) {
                ++fatalRejectedCandidates;
            }
            for (std::size_t reasonIndex = 0;
                 reasonIndex < kRejectDescriptors.size(); ++reasonIndex) {
                if (hik_stripe::hasRejectReason(
                        candidate, kRejectDescriptors[reasonIndex].flag)) {
                    ++rejectCounts[reasonIndex];
                }
            }
        }
        for (const hik_stripe::Candidate& selected :
             quality.provisionalSelected) {
            for (std::size_t reasonIndex = 0;
                 reasonIndex < kRejectDescriptors.size(); ++reasonIndex) {
                const hik_stripe::RejectReason flag =
                    kRejectDescriptors[reasonIndex].flag;
                if ((flag == hik_stripe::REJECT_PATH_JUMP ||
                     flag == hik_stripe::REJECT_PATH_AMBIGUOUS ||
                     flag ==
                         hik_stripe::REJECT_AMBIGUOUS_MULTIPATH) &&
                    hik_stripe::hasRejectReason(selected, flag)) {
                    ++rejectCounts[reasonIndex];
                }
            }
        }
        std::size_t multipathBranchCandidateCount = 0U;
        double minimumMultipathMargin =
            std::numeric_limits<double>::infinity();
        double maximumMultipathSeparation = 0.0;
        for (const hik_stripe::MultipathInterval& interval :
             quality.multipathIntervals) {
            minimumMultipathMargin = std::min(
                minimumMultipathMargin,
                interval.minimumLocalCostMargin);
            maximumMultipathSeparation = std::max(
                maximumMultipathSeparation,
                interval.maximumSeparationPx);
            for (const hik_stripe::MultipathBranch& branch :
                 interval.branches) {
                multipathBranchCandidateCount +=
                    branch.candidates.size();
            }
        }

        std::ostringstream prefix;
        prefix << std::setw(6) << std::setfill('0') << fileIndex
               << '_' << input.stem().string();
        const fs::path candidatePath =
            command.output / "csv" / (prefix.str() + "_quality.csv");
        if (!writeCandidateCsv(
                candidatePath, quality, oldCenters, &error)) {
            std::cerr << "Warning: " << error << '\n';
            ++failedCount;
            continue;
        }
        const fs::path multipathPath =
            command.output / "csv" /
            (prefix.str() + "_multipath.csv");
        if (!writeMultipathCsv(
                multipathPath, quality, &error)) {
            std::cerr << "Warning: " << error << '\n';
            ++failedCount;
            continue;
        }
        if (command.writeOverlays) {
            const fs::path overlayPath =
                command.output / "overlay" / (prefix.str() + "_overlay.png");
            if (!writeOverlay(
                    overlayPath, raw, legacy, quality, &error)) {
                std::cerr << "Warning: " << error << '\n';
                ++failedCount;
                continue;
            }
        }

        const double saturatedCandidateRatio =
            quality.diagnostics.totalCandidateCount > 0
            ? static_cast<double>(
                  quality.diagnostics.saturatedCandidateCount) /
                  static_cast<double>(
                      quality.diagnostics.totalCandidateCount)
            : 0.0;
        const double multiPeakRatio =
            quality.diagnostics.scanlineCount > 0
            ? static_cast<double>(
                  quality.diagnostics.multiPeakScanlineCount) /
                  static_cast<double>(quality.diagnostics.scanlineCount)
            : 0.0;
        summary << csvQuote(input.string())
                << ',' << raw.cols
                << ',' << raw.rows
                << ',' << hik_stripe::orientationName(comparisonOrientation)
                << ',' << legacy.size()
                << ',' << quality.selected.size()
                << ',' << quality.provisionalSelected.size()
                << ',' << quality.selected.size()
                << ',' << offsets.size()
                << ',' << finiteOrEmpty(signedMeanOffset)
                << ',' << finiteOrEmpty(offsetStats.p50)
                << ',' << finiteOrEmpty(offsetStats.p95)
                << ',' << finiteOrEmpty(offsetStats.maximum)
                << ',' << robustStats.inlierCount
                << ',' << robustStats.grossMismatchCount
                << ',' << finiteOrEmpty(robustStats.signedMedian)
                << ',' << finiteOrEmpty(robustStats.robustSignedMean)
                << ',' << finiteOrEmpty(robustStats.gate)
                << ',' << quality.candidates.size()
                << ',' << quality.diagnostics.pathUsableCandidateCount
                << ',' << fatalRejectedCandidates
                << ',' << quality.multipathIntervals.size()
                << ',' << quality.diagnostics
                               .multipathAmbiguousScanlineCount
                << ',' << multipathBranchCandidateCount
                << ',' << finiteOrEmpty(minimumMultipathMargin)
                << ',' << maximumMultipathSeparation
                << ',' << saturatedCandidateRatio
                << ',' << multiPeakRatio
                << ',' << quality.diagnostics.selectedSaturatedRatio
                << ',' << quality.diagnostics.selectedGapCount
                << ',' << quality.diagnostics.meanSelectedQuality
                << ',' << quality.diagnostics.meanSelectedFwhmPx
                << ',' << quality.diagnostics.meanSelectedSnr
                << ',' << quality.diagnostics
                               .meanSelectedGradientAsymmetry
                << ',' << quality.diagnostics.meanSelectedFitResidual
                << ',' << quality.diagnostics
                               .meanSelectedSecondPeakRatio
                << ',' << quality.diagnostics.pathCostMarginPerPoint;
        for (const std::size_t count : rejectCounts) {
            summary << ',' << count;
        }
        summary << ',' << csvQuote(legacyWarning)
                << ',' << csvQuote(qualityOk ? std::string() : quality.error)
                << '\n';
        ++processedCount;
    }

    if (!summary) {
        std::cerr << "Error: failed while writing " << summaryPath << '\n';
        return 2;
    }

    const Statistics aggregate = statistics(allOffsets);
    const RobustOffsetStatistics aggregateRobust =
        robustOffsetStatistics(allSignedOffsets);
    const double aggregateSignedMean =
        allSignedOffsetCount > 0U
        ? static_cast<double>(
              allSignedOffsetSum /
              static_cast<long double>(allSignedOffsetCount))
        : std::numeric_limits<double>::quiet_NaN();
    std::cout << "Processed " << processedCount << " PNG file(s); "
              << failedCount << " failed. Algorithm="
              << hik_stripe::algorithmVersion()
              << ", matched=" << allOffsets.size()
              << ", signed mean(new-legacy)="
              << finiteOrEmpty(aggregateSignedMean)
              << " px"
              << ", robust matched/gross="
              << aggregateRobust.inlierCount << '/'
              << aggregateRobust.grossMismatchCount
              << ", robust signed median/mean="
              << finiteOrEmpty(aggregateRobust.signedMedian) << '/'
              << finiteOrEmpty(aggregateRobust.robustSignedMean)
              << " px (gate="
              << finiteOrEmpty(aggregateRobust.gate) << " px)"
              << ", |new-legacy| P50=" << finiteOrEmpty(aggregate.p50)
              << " px, P95=" << finiteOrEmpty(aggregate.p95)
              << " px, max=" << finiteOrEmpty(aggregate.maximum)
              << " px.\nSummary: " << summaryPath << '\n';
    return failedCount == 0 ? 0 : 1;
}
