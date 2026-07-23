#include "HikSynchronizationCore.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <thread>

#include <time.h>

namespace hik_sync {
namespace {

constexpr double kNsPerMs = 1000000.0;
constexpr double kNsPerSecond = 1000000000.0;

template <typename T>
T clampValue(T value, T low, T high) {
    return std::max(low, std::min(high, value));
}

std::string trim(const std::string& value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return std::string();
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1U);
}

std::string unquote(const std::string& value) {
    const std::string clean = trim(value);
    if (clean.size() >= 2U &&
        ((clean.front() == '"' && clean.back() == '"') ||
         (clean.front() == '\'' && clean.back() == '\''))) {
        return clean.substr(1U, clean.size() - 2U);
    }
    return clean;
}

bool parseDouble(const std::string& text, double* value) {
    if (!value) return false;
    try {
        std::size_t used = 0U;
        const double parsed = std::stod(trim(text), &used);
        if (used != trim(text).size() || !std::isfinite(parsed)) return false;
        *value = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

bool parseInteger(const std::string& text, int* value) {
    if (!value) return false;
    try {
        std::size_t used = 0U;
        const long parsed = std::stol(trim(text), &used, 10);
        if (used != trim(text).size() ||
            parsed < std::numeric_limits<int>::min() ||
            parsed > std::numeric_limits<int>::max()) return false;
        *value = static_cast<int>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseSize(const std::string& text, std::size_t* value) {
    if (!value) return false;
    if (!trim(text).empty() && trim(text).front() == '-') return false;
    try {
        std::size_t used = 0U;
        const unsigned long long parsed = std::stoull(trim(text), &used, 10);
        if (used != trim(text).size() || parsed == 0U ||
            parsed > std::numeric_limits<std::size_t>::max()) return false;
        *value = static_cast<std::size_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseBool(const std::string& text, bool* value) {
    if (!value) return false;
    const std::string clean = unquote(text);
    if (clean == "true" || clean == "True" || clean == "1") {
        *value = true;
        return true;
    }
    if (clean == "false" || clean == "False" || clean == "0") {
        *value = false;
        return true;
    }
    return false;
}

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if ((values.size() % 2U) == 0U) {
        const double lower = *std::max_element(values.begin(), values.begin() + middle);
        result = 0.5 * (lower + result);
    }
    return result;
}

ClockFitReport fitAffine(const std::vector<std::pair<double, double>>& input,
                         std::size_t minimumSamples,
                         double maximumResidualNs,
                         bool lowDelayEnvelope) {
    ClockFitReport report;
    report.sampleCount = input.size();
    if (input.size() < 2U) return report;

    auto fitIndices = [&input](const std::vector<std::size_t>& indices,
                               double* scale, double* offset) -> bool {
        if (!scale || !offset || indices.size() < 2U) return false;
        long double meanX = 0.0L;
        long double meanY = 0.0L;
        for (const std::size_t index : indices) {
            meanX += input[index].first;
            meanY += input[index].second;
        }
        meanX /= static_cast<long double>(indices.size());
        meanY /= static_cast<long double>(indices.size());
        long double covariance = 0.0L;
        long double variance = 0.0L;
        for (const std::size_t index : indices) {
            const long double dx = input[index].first - meanX;
            covariance += dx * (input[index].second - meanY);
            variance += dx * dx;
        }
        if (variance <= std::numeric_limits<long double>::epsilon()) return false;
        *scale = static_cast<double>(covariance / variance);
        *offset = static_cast<double>(meanY - (*scale) * meanX);
        return std::isfinite(*scale) && std::isfinite(*offset);
    };

    std::vector<std::size_t> indices(input.size());
    std::iota(indices.begin(), indices.end(), 0U);
    double scale = 1.0;
    double offset = 0.0;
    if (!fitIndices(indices, &scale, &offset)) return report;

    for (int iteration = 0; iteration < 2; ++iteration) {
        std::vector<double> residuals;
        residuals.reserve(indices.size());
        for (const std::size_t index : indices) {
            residuals.push_back(input[index].second -
                                (scale * input[index].first + offset));
        }
        const double center = median(residuals);
        std::vector<double> deviations;
        deviations.reserve(residuals.size());
        for (const double residual : residuals) {
            deviations.push_back(std::abs(residual - center));
        }
        const double mad = median(deviations);
        const double gate = std::max(50000.0, 4.5 * 1.4826 * mad);
        std::vector<std::size_t> accepted;
        for (std::size_t i = 0; i < indices.size(); ++i) {
            if (std::abs(residuals[i] - center) <= gate) {
                accepted.push_back(indices[i]);
            }
        }
        if (accepted.size() < 2U || accepted.size() == indices.size()) break;
        indices.swap(accepted);
        if (!fitIndices(indices, &scale, &offset)) return report;
    }

    std::vector<double> finalResiduals;
    finalResiduals.reserve(indices.size());
    for (const std::size_t index : indices) {
        finalResiduals.push_back(input[index].second -
                                 (scale * input[index].first + offset));
    }
    if (lowDelayEnvelope && finalResiduals.size() >= 10U) {
        std::vector<double> ordered = finalResiduals;
        const std::size_t lowIndex = ordered.size() / 10U;
        std::nth_element(ordered.begin(), ordered.begin() + lowIndex, ordered.end());
        offset += ordered[lowIndex];
    }

    const double residualCenter = median(finalResiduals);
    long double squared = 0.0L;
    for (const double residual : finalResiduals) {
        const long double centered = residual - residualCenter;
        squared += centered * centered;
    }
    report.scale = scale;
    report.offsetNs = offset;
    report.residualRmsNs = std::sqrt(static_cast<double>(
        squared / static_cast<long double>(finalResiduals.size())));
    report.sampleCount = indices.size();
    report.stable = report.sampleCount >= minimumSamples &&
                    std::isfinite(report.scale) && report.scale > 0.0 &&
                    std::isfinite(report.residualRmsNs) &&
                    report.residualRmsNs <= maximumResidualNs;
    return report;
}

std::string jsonEscape(const std::string& text) {
    std::ostringstream out;
    for (const unsigned char character : text) {
        switch (character) {
        case '"': out << "\\\""; break;
        case '\\': out << "\\\\"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (character < 0x20U) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(character) << std::dec;
            } else {
                out << character;
            }
        }
    }
    return out.str();
}

std::string isoUtcNow() {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch());
    const std::time_t seconds = static_cast<std::time_t>(milliseconds.count() / 1000);
    std::tm utc{};
    gmtime_r(&seconds, &utc);
    std::ostringstream text;
    text << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << (milliseconds.count() % 1000)
         << 'Z';
    return text.str();
}

std::string frameFilename(uint64_t frameId, bool saveImages) {
    if (!saveImages) return std::string();
    std::ostringstream name;
    name << "images/frame_" << std::setw(12) << std::setfill('0')
         << frameId << ".png";
    return name.str();
}

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}

    bool tryPush(T value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || queue_.size() >= capacity_) return false;
        queue_.push_back(std::move(value));
        condition_.notify_one();
        return true;
    }

    bool pop(T* value) {
        if (!value) return false;
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return false;
        *value = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
        condition_.notify_all();
    }

private:
    const std::size_t capacity_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<T> queue_;
    bool closed_{false};
};

}  // namespace

int64_t getMonotonicRawNs() {
    timespec value{};
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &value) != 0) return 0;
    return static_cast<int64_t>(value.tv_sec) * 1000000000LL +
           static_cast<int64_t>(value.tv_nsec);
}

struct ImageBufferPool::State {
    explicit State(std::size_t requestedCapacity) : capacity(requestedCapacity) {}
    std::size_t capacity;
    std::size_t created{0U};
    mutable std::mutex mutex;
    std::vector<std::unique_ptr<ImageBuffer>> available;
};

ImageBufferPool::ImageBufferPool(std::size_t capacity)
    : state_(std::make_shared<State>(std::max<std::size_t>(1U, capacity))) {}

std::shared_ptr<ImageBuffer> ImageBufferPool::acquire(std::size_t byteCount) {
    std::unique_ptr<ImageBuffer> owned;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->available.empty()) {
            owned = std::move(state_->available.back());
            state_->available.pop_back();
        } else if (state_->created < state_->capacity) {
            owned = std::make_unique<ImageBuffer>();
            ++state_->created;
        }
    }
    if (!owned) return {};
    try {
        owned->bytes.resize(byteCount);
    } catch (...) {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->available.push_back(std::move(owned));
        return {};
    }
    ImageBuffer* raw = owned.release();
    const std::shared_ptr<State> state = state_;
    return std::shared_ptr<ImageBuffer>(raw, [state](ImageBuffer* buffer) {
        if (!buffer) return;
        buffer->width = 0;
        buffer->height = 0;
        buffer->stride = 0;
        buffer->pixelFormat = 0;
        std::lock_guard<std::mutex> lock(state->mutex);
        state->available.emplace_back(buffer);
    });
}

std::size_t ImageBufferPool::capacity() const { return state_->capacity; }

std::size_t ImageBufferPool::available() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->available.size() + (state_->capacity - state_->created);
}

CyclicCounterUnwrapper::CyclicCounterUnwrapper(unsigned int bits) {
    if (bits == 0U || bits > 63U) bits = 32U;
    modulus_ = uint64_t{1} << bits;
    mask_ = modulus_ - 1U;
}

CounterUpdate CyclicCounterUnwrapper::update(uint64_t rawCounter) {
    CounterUpdate result;
    rawCounter &= mask_;
    if (!initialized_) {
        initialized_ = true;
        lastRaw_ = rawCounter;
        sequence_ = rawCounter;
        result.sequence = sequence_;
        result.first = true;
        return result;
    }
    const uint64_t delta = (rawCounter - lastRaw_) & mask_;
    if (delta == 0U) {
        result.sequence = sequence_;
        result.duplicate = true;
        return result;
    }
    if (delta > modulus_ / 2U) {
        result.sequence = sequence_;
        result.continuous = false;
        result.outOfOrder = true;
        return result;
    }
    sequence_ += delta;
    lastRaw_ = rawCounter;
    result.sequence = sequence_;
    result.dropped = delta > 1U ? delta - 1U : 0U;
    result.continuous = delta == 1U;
    return result;
}

void CyclicCounterUnwrapper::reset() {
    initialized_ = false;
    lastRaw_ = 0U;
    sequence_ = 0U;
}

const char* syncQualityName(SyncQuality quality) {
    switch (quality) {
    case SyncQuality::VALID: return "VALID";
    case SyncQuality::ROBOT_DATA_NOT_READY: return "ROBOT_DATA_NOT_READY";
    case SyncQuality::ROBOT_BRACKET_NOT_FOUND: return "ROBOT_BRACKET_NOT_FOUND";
    case SyncQuality::ROBOT_PACKET_SEQUENCE_GAP: return "ROBOT_PACKET_SEQUENCE_GAP";
    case SyncQuality::ROBOT_GAP_TOO_LARGE: return "ROBOT_GAP_TOO_LARGE";
    case SyncQuality::CAMERA_TIMESTAMP_INVALID: return "CAMERA_TIMESTAMP_INVALID";
    case SyncQuality::CAMERA_FRAME_DROPPED: return "CAMERA_FRAME_DROPPED";
    case SyncQuality::OUTSIDE_VALID_SCAN_SEGMENT: return "OUTSIDE_VALID_SCAN_SEGMENT";
    case SyncQuality::SPEED_NOT_STABLE: return "SPEED_NOT_STABLE";
    }
    return "UNKNOWN";
}

const char* motionPhaseName(MotionPhase phase) {
    switch (phase) {
    case MotionPhase::WARMUP: return "WARMUP";
    case MotionPhase::STOPPED: return "STOPPED";
    case MotionPhase::ACCELERATING: return "ACCELERATING";
    case MotionPhase::CONSTANT_SPEED: return "CONSTANT_SPEED";
    case MotionPhase::DECELERATING: return "DECELERATING";
    case MotionPhase::UNSTABLE: return "UNSTABLE";
    }
    return "UNKNOWN";
}

const char* cameraTimestampReferenceName(CameraTimestampReference reference) {
    switch (reference) {
    case CameraTimestampReference::ExposureStart: return "exposure_start";
    case CameraTimestampReference::ExposureEnd: return "exposure_end";
    case CameraTimestampReference::Callback: return "callback";
    }
    return "unknown";
}

const char* robotTimeModeName(RobotTimeMode mode) {
    switch (mode) {
    case RobotTimeMode::ControllerTimestamp: return "CONTROLLER_TIMESTAMP";
    case RobotTimeMode::PeriodFit: return "PERIOD_FIT";
    case RobotTimeMode::HostReceive: return "HOST_RECEIVE";
    }
    return "UNKNOWN";
}

bool SynchronizationConfig::validate(std::string* error) const {
    auto fail = [error](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (!std::isfinite(scanSpeedMmS) || scanSpeedMmS < scanMinSpeedMmS ||
        scanSpeedMmS > scanMaxSpeedMmS) {
        std::ostringstream message;
        message << "scan.speed_mm_s must be in [" << scanMinSpeedMmS << ", "
                << scanMaxSpeedMmS << "] mm/s, got " << scanSpeedMmS;
        return fail(message.str());
    }
    if (!(cameraTargetFps > 0.0) || !(cameraExposureUs > 0.0) ||
        cameraQueueCapacity == 0U || cameraClockMappingWindow < 10U) {
        return fail("invalid camera synchronization configuration");
    }
    if (!(robotPeriodMs > 0.0) || !(robotExpectedFeedbackPeriodMs > 0.0) ||
        !(robotPoseBufferDurationS >= 5.0) ||
        robotBracketWaitTimeoutMs < 1 ||
        robotNormalGapMinMs < 0.0 ||
        robotNormalGapMinMs > robotNormalGapMaxMs ||
        robotNormalGapMaxMs > robotWarningGapMs ||
        robotWarningGapMs > robotInvalidGapMs) {
        return fail("invalid robot synchronization configuration");
    }
    const double capacityPeriodMs =
        std::min(robotPeriodMs, robotExpectedFeedbackPeriodMs);
    const std::size_t minimumRobotQueueCapacity = static_cast<std::size_t>(
        std::ceil(robotPoseBufferDurationS * 1000.0 / capacityPeriodMs)) + 2U;
    if (robotQueueCapacity < minimumRobotQueueCapacity) {
        return fail("robot.queue_capacity cannot cover pose_buffer_duration_s at the "
                    "configured request/feedback period");
    }
    if (!(scanSpeedTolerancePercent >= 0.0) || !(scanAccelerationMmS2 > 0.0) ||
        !std::isfinite(scanAccelerationMmS2) || scanStableSampleCount < 1 ||
        scanSpeedFilterWindowSamples < scanStableSampleCount ||
        imageWriterThreads < 1U || imageWriterThreads > 16U ||
        writerQueueCapacity < cameraQueueCapacity) {
        return fail("invalid scan/writer synchronization configuration");
    }
    if (error) error->clear();
    return true;
}

bool SynchronizationConfig::loadYaml(const std::string& path,
                                     SynchronizationConfig* config,
                                     std::string* error) {
    if (!config) {
        if (error) *error = "configuration output is null";
        return false;
    }
    std::ifstream input(path);
    if (!input) {
        if (error) *error = "cannot open synchronization YAML: " + path;
        return false;
    }
    SynchronizationConfig parsed;
    std::string section;
    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        if (trim(line).empty()) continue;
        const bool indented = !line.empty() && (line.front() == ' ' || line.front() == '\t');
        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) {
            if (error) *error = "invalid synchronization YAML line " +
                                std::to_string(lineNumber);
            return false;
        }
        const std::string key = trim(line.substr(0U, colon));
        const std::string value = trim(line.substr(colon + 1U));
        if (!indented && value.empty()) {
            section = key;
            continue;
        }
        const std::string full = section.empty() ? key : section + "." + key;
        bool ok = true;
        if (full == "camera.target_fps") ok = parseDouble(value, &parsed.cameraTargetFps);
        else if (full == "camera.exposure_us") ok = parseDouble(value, &parsed.cameraExposureUs);
        else if (full == "camera.fixed_time_offset_us") ok = parseDouble(value, &parsed.cameraFixedTimeOffsetUs);
        else if (full == "camera.transport_delay_us") ok = parseDouble(value, &parsed.cameraTransportDelayUs);
        else if (full == "camera.queue_capacity") ok = parseSize(value, &parsed.cameraQueueCapacity);
        else if (full == "camera.clock_mapping_window") ok = parseSize(value, &parsed.cameraClockMappingWindow);
        else if (full == "camera.mapping_max_residual_us") ok = parseDouble(value, &parsed.cameraMappingMaxResidualUs);
        else if (full == "camera.timestamp_reference") {
            const std::string mode = unquote(value);
            if (mode == "exposure_start") parsed.cameraTimestampReference = CameraTimestampReference::ExposureStart;
            else if (mode == "exposure_end") parsed.cameraTimestampReference = CameraTimestampReference::ExposureEnd;
            else if (mode == "callback") parsed.cameraTimestampReference = CameraTimestampReference::Callback;
            else ok = false;
        } else if (full == "robot.cn_de_period_ms") ok = parseDouble(value, &parsed.robotPeriodMs);
        else if (full == "robot.expected_feedback_period_ms") {
            ok = parseDouble(value, &parsed.robotExpectedFeedbackPeriodMs);
        }
        else if (full == "robot.pose_buffer_duration_s") ok = parseDouble(value, &parsed.robotPoseBufferDurationS);
        else if (full == "robot.queue_capacity") ok = parseSize(value, &parsed.robotQueueCapacity);
        else if (full == "robot.normal_gap_min_ms") ok = parseDouble(value, &parsed.robotNormalGapMinMs);
        else if (full == "robot.normal_gap_max_ms") ok = parseDouble(value, &parsed.robotNormalGapMaxMs);
        else if (full == "robot.warning_gap_ms") ok = parseDouble(value, &parsed.robotWarningGapMs);
        else if (full == "robot.invalid_gap_ms") ok = parseDouble(value, &parsed.robotInvalidGapMs);
        else if (full == "robot.bracket_wait_timeout_ms") ok = parseInteger(value, &parsed.robotBracketWaitTimeoutMs);
        else if (full == "robot.time_mode") {
            const std::string mode = unquote(value);
            if (mode == "controller_timestamp") parsed.robotTimeMode = RobotTimeMode::ControllerTimestamp;
            else if (mode == "period_fit") parsed.robotTimeMode = RobotTimeMode::PeriodFit;
            else if (mode == "host_receive") parsed.robotTimeMode = RobotTimeMode::HostReceive;
            else ok = false;
        } else if (full == "scan.speed_mm_s") ok = parseDouble(value, &parsed.scanSpeedMmS);
        else if (full == "scan.min_speed_mm_s") ok = parseDouble(value, &parsed.scanMinSpeedMmS);
        else if (full == "scan.max_speed_mm_s") ok = parseDouble(value, &parsed.scanMaxSpeedMmS);
        else if (full == "scan.speed_tolerance_percent") ok = parseDouble(value, &parsed.scanSpeedTolerancePercent);
        else if (full == "scan.acceleration_mm_s2") ok = parseDouble(value, &parsed.scanAccelerationMmS2);
        else if (full == "scan.stable_sample_count") ok = parseInteger(value, &parsed.scanStableSampleCount);
        else if (full == "scan.speed_filter_window_samples") ok = parseInteger(value, &parsed.scanSpeedFilterWindowSamples);
        else if (full == "scan.discard_acceleration_segment") ok = parseBool(value, &parsed.discardAccelerationSegment);
        else if (full == "scan.discard_deceleration_segment") ok = parseBool(value, &parsed.discardDecelerationSegment);
        else if (full == "scan.pre_scan_distance_mm") ok = parseDouble(value, &parsed.preScanDistanceMm);
        else if (full == "scan.post_scan_distance_mm") ok = parseDouble(value, &parsed.postScanDistanceMm);
        else if (full == "output.save_images") ok = parseBool(value, &parsed.saveImages);
        else if (full == "output.save_robot_raw_csv") ok = parseBool(value, &parsed.saveRobotRawCsv);
        else if (full == "output.save_camera_raw_csv") ok = parseBool(value, &parsed.saveCameraRawCsv);
        else if (full == "output.save_sync_csv") ok = parseBool(value, &parsed.saveSyncCsv);
        else if (full == "output.output_directory") parsed.outputDirectory = unquote(value);
        else if (full == "output.writer_queue_capacity") ok = parseSize(value, &parsed.writerQueueCapacity);
        else if (full == "output.image_writer_threads") ok = parseSize(value, &parsed.imageWriterThreads);
        if (!ok) {
            if (error) *error = "invalid value for " + full + " at line " +
                                std::to_string(lineNumber);
            return false;
        }
    }
    std::string validationError;
    if (!parsed.validate(&validationError)) {
        if (error) *error = validationError;
        return false;
    }
    *config = parsed;
    if (error) error->clear();
    return true;
}

Eigen::Quaterniond fairinoFixedAxisRpyDegrees(double rxDeg,
                                              double ryDeg,
                                              double rzDeg) {
    constexpr double radiansPerDegree = 3.14159265358979323846 / 180.0;
    const Eigen::AngleAxisd rx(rxDeg * radiansPerDegree, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(ryDeg * radiansPerDegree, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(rzDeg * radiansPerDegree, Eigen::Vector3d::UnitZ());
    return Eigen::Quaterniond(rz * ry * rx).normalized();
}

Eigen::Matrix4d makeTransform(const Eigen::Vector3d& positionMm,
                              const Eigen::Quaterniond& orientation) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
    transform.block<3, 1>(0, 3) = positionMm;
    return transform;
}

bool interpolateRobotPose(const RobotSample& before,
                          const RobotSample& after,
                          int64_t targetTimestampNs,
                          Eigen::Vector3d* positionMm,
                          Eigen::Quaterniond* orientation,
                          double* alpha) {
    if (!positionMm || !orientation || !alpha || !before.valid || !after.valid ||
        after.alignedTimestampNs <= before.alignedTimestampNs ||
        targetTimestampNs < before.alignedTimestampNs ||
        targetTimestampNs > after.alignedTimestampNs) return false;
    const long double numerator = targetTimestampNs - before.alignedTimestampNs;
    const long double denominator = after.alignedTimestampNs - before.alignedTimestampNs;
    const double fraction = clampValue(static_cast<double>(numerator / denominator), 0.0, 1.0);
    *positionMm = (1.0 - fraction) * before.flangePositionMm +
                  fraction * after.flangePositionMm;
    *orientation = before.flangeOrientation.slerp(
        fraction, after.flangeOrientation).normalized();
    *alpha = fraction;
    return positionMm->allFinite() && orientation->coeffs().allFinite();
}

int64_t cameraTicksToNs(uint64_t rawTicks, uint64_t frequencyHz, bool* ok) {
    const bool valid = frequencyHz != 0U;
    if (ok) *ok = valid;
    if (!valid) return 0;
    const long double nanoseconds = static_cast<long double>(rawTicks) *
                                    kNsPerSecond /
                                    static_cast<long double>(frequencyHz);
    if (nanoseconds < 0.0L ||
        nanoseconds > static_cast<long double>(std::numeric_limits<int64_t>::max())) {
        if (ok) *ok = false;
        return 0;
    }
    return static_cast<int64_t>(std::llround(nanoseconds));
}

ExposureTiming computeExposureTiming(int64_t referenceHostNs,
                                     double exposureUs,
                                     double fixedOffsetUs,
                                     CameraTimestampReference reference) {
    ExposureTiming result;
    if (referenceHostNs <= 0 || !std::isfinite(exposureUs) || exposureUs <= 0.0 ||
        !std::isfinite(fixedOffsetUs)) return result;
    const int64_t exposureNs = static_cast<int64_t>(std::llround(exposureUs * 1000.0));
    if (reference == CameraTimestampReference::ExposureStart) {
        result.exposureStartHostNs = referenceHostNs;
        result.exposureMidHostNs = referenceHostNs + exposureNs / 2;
    } else {
        // For an exposure-end timestamp, or callback fallback, the reference
        // is after integration. Transport delay is removed before this call.
        result.exposureStartHostNs = referenceHostNs - exposureNs;
        result.exposureMidHostNs = referenceHostNs - exposureNs / 2;
    }
    result.alignedTimestampNs = result.exposureMidHostNs +
        static_cast<int64_t>(std::llround(fixedOffsetUs * 1000.0));
    result.valid = result.exposureStartHostNs > 0 && result.exposureMidHostNs > 0 &&
                   result.alignedTimestampNs > 0;
    return result;
}

CameraClockMapper::CameraClockMapper(std::size_t windowSize,
                                     double maximumResidualNs)
    : windowSize_(std::max<std::size_t>(10U, windowSize)),
      maximumResidualNs_(std::max(1.0, maximumResidualNs)) {}

void CameraClockMapper::addSample(int64_t cameraTimestampNs, int64_t hostCallbackNs) {
    if (cameraTimestampNs <= 0 || hostCallbackNs <= 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.emplace_back(cameraTimestampNs, hostCallbackNs);
    while (samples_.size() > windowSize_) samples_.pop_front();
    refitLocked();
}

void CameraClockMapper::refitLocked() {
    std::vector<std::pair<double, double>> samples;
    samples.reserve(samples_.size());
    for (const auto& sample : samples_) {
        samples.emplace_back(static_cast<double>(sample.first),
                             static_cast<double>(sample.second));
    }
    report_ = fitAffine(samples, std::min<std::size_t>(30U, windowSize_),
                        maximumResidualNs_, true);
    if (report_.scale < 0.98 || report_.scale > 1.02) report_.stable = false;
}

bool CameraClockMapper::mapToHost(int64_t cameraTimestampNs,
                                  int64_t* hostTimestampNs) const {
    if (!hostTimestampNs || cameraTimestampNs <= 0) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!report_.stable) return false;
    const long double mapped = report_.scale *
        static_cast<long double>(cameraTimestampNs) + report_.offsetNs;
    if (mapped <= 0.0L || mapped > std::numeric_limits<int64_t>::max()) return false;
    *hostTimestampNs = static_cast<int64_t>(std::llround(mapped));
    return true;
}

ClockFitReport CameraClockMapper::report() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return report_;
}

void CameraClockMapper::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    report_ = ClockFitReport{};
}

RobotClockMapper::RobotClockMapper(RobotTimeMode requestedMode,
                                   double nominalPeriodMs,
                                   std::size_t windowSize)
    : requestedMode_(requestedMode), activeMode_(RobotTimeMode::HostReceive),
      nominalPeriodNs_(nominalPeriodMs * kNsPerMs),
      windowSize_(std::max<std::size_t>(20U, windowSize)) {}

void RobotClockMapper::align(RobotSample* sample) {
    if (!sample || sample->hostReceiveNs <= 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    periodSamples_.emplace_back(static_cast<double>(sample->sequence),
                                static_cast<double>(sample->hostReceiveNs));
    while (periodSamples_.size() > windowSize_) periodSamples_.pop_front();
    std::vector<std::pair<double, double>> period(periodSamples_.begin(), periodSamples_.end());
    periodReport_ = fitAffine(period, 20U, 2000000.0, true);
    if (periodReport_.scale < nominalPeriodNs_ * 0.75 ||
        periodReport_.scale > nominalPeriodNs_ * 1.25) {
        periodReport_.stable = false;
    }

    if (sample->hasRobotTimestamp && sample->robotTimestampNs > 0) {
        controllerSamples_.emplace_back(static_cast<double>(sample->robotTimestampNs),
                                        static_cast<double>(sample->hostReceiveNs));
        while (controllerSamples_.size() > windowSize_) controllerSamples_.pop_front();
        std::vector<std::pair<double, double>> controller(
            controllerSamples_.begin(), controllerSamples_.end());
        controllerReport_ = fitAffine(controller, 30U, 2500000.0, true);
        if (controllerReport_.scale < 0.98 || controllerReport_.scale > 1.02) {
            controllerReport_.stable = false;
        }
    }

    activeMode_ = RobotTimeMode::HostReceive;
    sample->alignedTimestampNs = sample->hostReceiveNs;
    if (requestedMode_ == RobotTimeMode::ControllerTimestamp &&
        sample->hasRobotTimestamp && controllerReport_.stable) {
        const long double mapped = controllerReport_.scale *
            static_cast<long double>(sample->robotTimestampNs) +
            controllerReport_.offsetNs;
        sample->alignedTimestampNs = static_cast<int64_t>(std::llround(mapped));
        activeMode_ = RobotTimeMode::ControllerTimestamp;
    } else if (requestedMode_ != RobotTimeMode::HostReceive && periodReport_.stable) {
        const long double mapped = periodReport_.scale *
            static_cast<long double>(sample->sequence) + periodReport_.offsetNs;
        sample->alignedTimestampNs = static_cast<int64_t>(std::llround(mapped));
        activeMode_ = RobotTimeMode::PeriodFit;
    }
}

RobotTimeMode RobotClockMapper::activeMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return activeMode_;
}

ClockFitReport RobotClockMapper::report() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (activeMode_ == RobotTimeMode::ControllerTimestamp) return controllerReport_;
    if (activeMode_ == RobotTimeMode::PeriodFit) return periodReport_;
    ClockFitReport direct;
    direct.sampleCount = periodSamples_.size();
    direct.stable = !periodSamples_.empty();
    return direct;
}

void RobotClockMapper::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    periodSamples_.clear();
    controllerSamples_.clear();
    periodReport_ = ClockFitReport{};
    controllerReport_ = ClockFitReport{};
    activeMode_ = RobotTimeMode::HostReceive;
}

SpeedStabilityDetector::SpeedStabilityDetector(double targetSpeedMmS,
                                               double tolerancePercent,
                                               int requiredSamples,
                                               int filterWindowSamples)
    : targetSpeedMmS_(targetSpeedMmS),
      toleranceFraction_(std::max(0.0, tolerancePercent) / 100.0),
      requiredSamples_(std::max(1, requiredSamples)),
      filterWindowSamples_(static_cast<std::size_t>(
          std::max(std::max(5, requiredSamples), filterWindowSamples))) {}

MotionPhase SpeedStabilityDetector::update(
        int64_t timestampNs, const Eigen::Vector3d& positionMm,
        double actualSpeedMmS, double* filteredSpeedMmS,
        double* positionFitSpeedMmS) {
    if (filteredSpeedMmS) *filteredSpeedMmS = 0.0;
    if (positionFitSpeedMmS) *positionFitSpeedMmS = 0.0;
    if (timestampNs <= 0 || !positionMm.allFinite() ||
        !std::isfinite(actualSpeedMmS)) {
        enterConsecutive_ = 0;
        return MotionPhase::UNSTABLE;
    }

    observations_.push_back({timestampNs, positionMm, actualSpeedMmS});
    while (observations_.size() > filterWindowSamples_) observations_.pop_front();

    std::vector<double> reportedSpeeds;
    reportedSpeeds.reserve(observations_.size());
    for (const Observation& observation : observations_) {
        reportedSpeeds.push_back(observation.reportedSpeedMmS);
    }
    const double reportedMedian = median(std::move(reportedSpeeds));

    double fittedSpeed = reportedMedian;
    if (observations_.size() >= 3U) {
        const int64_t referenceNs = observations_.back().timestampNs;
        long double meanTime = 0.0L;
        Eigen::Vector3d meanPosition = Eigen::Vector3d::Zero();
        for (const Observation& observation : observations_) {
            meanTime += static_cast<long double>(
                observation.timestampNs - referenceNs) / kNsPerSecond;
            meanPosition += observation.positionMm;
        }
        meanTime /= static_cast<long double>(observations_.size());
        meanPosition /= static_cast<double>(observations_.size());
        long double denominator = 0.0L;
        Eigen::Vector3d numerator = Eigen::Vector3d::Zero();
        for (const Observation& observation : observations_) {
            const long double time = static_cast<long double>(
                observation.timestampNs - referenceNs) / kNsPerSecond;
            const double centeredTime = static_cast<double>(time - meanTime);
            denominator += static_cast<long double>(centeredTime) * centeredTime;
            numerator += centeredTime * (observation.positionMm - meanPosition);
        }
        if (denominator > 1.0e-12L) {
            fittedSpeed = (numerator / static_cast<double>(denominator)).norm();
        }
    }

    // The position fit suppresses sample-to-sample TCP-speed noise, while the
    // median feedback keeps the estimate responsive during acceleration.
    const double filtered = observations_.size() >= filterWindowSamples_
        ? 0.75 * fittedSpeed + 0.25 * reportedMedian
        : reportedMedian;
    if (filteredSpeedMmS) *filteredSpeedMmS = filtered;
    if (positionFitSpeedMmS) *positionFitSpeedMmS = fittedSpeed;

    if (observations_.size() < filterWindowSamples_) return MotionPhase::WARMUP;

    const double enterTolerance = targetSpeedMmS_ * toleranceFraction_;
    const double exitTolerance = std::max(enterTolerance * 2.0,
                                          targetSpeedMmS_ * 0.02);
    const bool insideEnter = std::abs(filtered - targetSpeedMmS_) <= enterTolerance;
    const bool insideExit = std::abs(filtered - targetSpeedMmS_) <= exitTolerance;

    if (constantSpeed_) {
        if (insideExit) {
            exitConsecutive_ = 0;
            return MotionPhase::CONSTANT_SPEED;
        }
        ++exitConsecutive_;
        if (exitConsecutive_ < requiredSamples_) return MotionPhase::CONSTANT_SPEED;
        constantSpeed_ = false;
        enterConsecutive_ = 0;
    }

    if (insideEnter) {
        enterConsecutive_ = std::min(requiredSamples_, enterConsecutive_ + 1);
        if (enterConsecutive_ >= requiredSamples_) {
            constantSpeed_ = true;
            reachedConstantSpeed_ = true;
            exitConsecutive_ = 0;
            return MotionPhase::CONSTANT_SPEED;
        }
    } else {
        enterConsecutive_ = 0;
    }

    if (filtered <= targetSpeedMmS_ * 0.10) return MotionPhase::STOPPED;
    if (!reachedConstantSpeed_ && filtered < targetSpeedMmS_ + exitTolerance) {
        return MotionPhase::ACCELERATING;
    }
    if (reachedConstantSpeed_ && filtered < targetSpeedMmS_ - enterTolerance) {
        return MotionPhase::DECELERATING;
    }
    return MotionPhase::UNSTABLE;
}

void SpeedStabilityDetector::reset() {
    observations_.clear();
    enterConsecutive_ = 0;
    exitConsecutive_ = 0;
    constantSpeed_ = false;
    reachedConstantSpeed_ = false;
}

RobotPoseBuffer::RobotPoseBuffer(std::size_t capacity, double durationSeconds)
    : capacity_(std::max<std::size_t>(2U, capacity)),
      durationNs_(static_cast<int64_t>(std::llround(
          std::max(0.001, durationSeconds) * kNsPerSecond))) {}

void RobotPoseBuffer::push(const RobotSample& sample) {
    if (!sample.valid || sample.alignedTimestampNs <= 0) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto position = std::lower_bound(
        samples_.begin(), samples_.end(), sample.alignedTimestampNs,
        [](const RobotSample& value, int64_t timestamp) {
            return value.alignedTimestampNs < timestamp;
        });
    if (position != samples_.end() &&
        position->alignedTimestampNs == sample.alignedTimestampNs) {
        *position = sample;
    } else {
        samples_.insert(position, sample);
    }
    while (samples_.size() > capacity_) samples_.pop_front();
    if (!samples_.empty()) {
        const int64_t cutoff = samples_.back().alignedTimestampNs - durationNs_;
        while (samples_.size() > 2U && samples_.front().alignedTimestampNs < cutoff) {
            samples_.pop_front();
        }
    }
    condition_.notify_all();
}

bool RobotPoseBuffer::getBracketingSamplesLocked(
        int64_t targetTimestampNs, RobotSample& before, RobotSample& after) const {
    if (samples_.size() < 2U || targetTimestampNs < samples_.front().alignedTimestampNs ||
        targetTimestampNs > samples_.back().alignedTimestampNs) return false;
    const auto upper = std::lower_bound(
        samples_.begin(), samples_.end(), targetTimestampNs,
        [](const RobotSample& value, int64_t timestamp) {
            return value.alignedTimestampNs < timestamp;
        });
    if (upper == samples_.end()) return false;
    if (upper->alignedTimestampNs == targetTimestampNs) {
        if (upper == samples_.begin()) {
            before = *upper;
            after = *(upper + 1);
        } else {
            before = *(upper - 1);
            after = *upper;
        }
        return true;
    }
    if (upper == samples_.begin()) return false;
    before = *(upper - 1);
    after = *upper;
    return true;
}

bool RobotPoseBuffer::getBracketingSamples(int64_t targetTimestampNs,
                                           RobotSample& before,
                                           RobotSample& after) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return getBracketingSamplesLocked(targetTimestampNs, before, after);
}

bool RobotPoseBuffer::waitForBracketingSamples(
        int64_t targetTimestampNs, std::chrono::milliseconds timeout,
        RobotSample& before, RobotSample& after) const {
    std::unique_lock<std::mutex> lock(mutex_);
    if (getBracketingSamplesLocked(targetTimestampNs, before, after)) return true;
    condition_.wait_for(lock, timeout, [this, targetTimestampNs] {
        return samples_.size() >= 2U &&
               samples_.front().alignedTimestampNs <= targetTimestampNs &&
               samples_.back().alignedTimestampNs >= targetTimestampNs;
    });
    return getBracketingSamplesLocked(targetTimestampNs, before, after);
}

bool RobotPoseBuffer::bounds(int64_t* earliestNs, int64_t* latestNs) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) return false;
    if (earliestNs) *earliestNs = samples_.front().alignedTimestampNs;
    if (latestNs) *latestNs = samples_.back().alignedTimestampNs;
    return true;
}

std::size_t RobotPoseBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

void RobotPoseBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    condition_.notify_all();
}

void RobotPoseBuffer::notifyAll() { condition_.notify_all(); }

namespace {

struct WriterEvent {
    enum class Type { Robot, Camera, Synchronized };
    Type type{Type::Robot};
    RobotSample robot;
    CameraFrame camera;
    SynchronizedFrame synchronized;
    std::string imageFilename;
};

struct ImageWriteTask {
    std::shared_ptr<ImageBuffer> image;
    std::string imageFilename;
};

}  // namespace

struct SynchronizationSession::Impl {
    SynchronizationConfig config;
    std::string directory;
    Eigen::Matrix4d flangeFromCamera{Eigen::Matrix4d::Identity()};
    bool hasFlangeFromCamera{false};
    std::unique_ptr<BoundedQueue<CameraFrame>> cameraQueue;
    std::unique_ptr<BoundedQueue<WriterEvent>> writerQueue;
    std::unique_ptr<BoundedQueue<ImageWriteTask>> imageQueue;
    RobotPoseBuffer robotBuffer;
    CameraClockMapper cameraMapper;
    RobotClockMapper robotMapper;
    SpeedStabilityDetector speedDetector;
    CyclicCounterUnwrapper robotCounter{8U};
    CyclicCounterUnwrapper cameraCounter{32U};
    std::thread synchronizationThread;
    std::thread writerThread;
    std::vector<std::thread> imageWriterThreads;
    std::atomic<bool> accepting{false};
    mutable std::mutex stateMutex;
    mutable std::mutex statisticsMutex;
    mutable std::mutex callbackMutex;
    PipelineStatistics stats;
    std::function<void(const SynchronizedFrame&)> synchronizedCallback;
    int64_t firstCameraNs{0};
    int64_t lastCameraNs{0};
    int64_t firstCameraDeviceNs{0};
    int64_t lastCameraDeviceNs{0};
    uint64_t firstCameraDeviceSequence{0};
    uint64_t lastCameraDeviceSequence{0};
    uint64_t firstCameraSequence{0};
    uint64_t lastCameraSequence{0};
    bool haveCameraSequence{false};
    int64_t firstRobotNs{0};
    int64_t lastRobotNs{0};
    int64_t lastRobotReceiveNs{0};
    int64_t lastRobotAlignedNs{0};
    uint64_t observedRobotSequence{0};
    uint64_t lastSdkReceiveSequence{0};
    long double robotGapSumMs{0.0L};
    uint64_t robotGapCount{0U};
    long double robotGetterDurationSumNs{0.0L};
    std::vector<int64_t> robotGetterDurationsNs;
    std::string startUtc;
    std::string endUtc;
    std::ofstream robotCsv;
    std::ofstream cameraCsv;
    std::ofstream syncCsv;

    Impl(const SynchronizationConfig& requestedConfig,
         const std::string& requestedDirectory)
        : config(requestedConfig), directory(requestedDirectory),
          cameraQueue(std::make_unique<BoundedQueue<CameraFrame>>(
              requestedConfig.cameraQueueCapacity)),
          writerQueue(std::make_unique<BoundedQueue<WriterEvent>>(
              requestedConfig.writerQueueCapacity)),
          imageQueue(std::make_unique<BoundedQueue<ImageWriteTask>>(
              requestedConfig.cameraQueueCapacity)),
          robotBuffer(requestedConfig.robotQueueCapacity,
                      requestedConfig.robotPoseBufferDurationS),
          cameraMapper(requestedConfig.cameraClockMappingWindow,
                       requestedConfig.cameraMappingMaxResidualUs * 1000.0),
          robotMapper(requestedConfig.robotTimeMode,
                      requestedConfig.robotExpectedFeedbackPeriodMs),
          speedDetector(requestedConfig.scanSpeedMmS,
                        requestedConfig.scanSpeedTolerancePercent,
                        requestedConfig.scanStableSampleCount,
                        requestedConfig.scanSpeedFilterWindowSamples) {}

    bool queueWriter(WriterEvent event) {
        if (writerQueue->tryPush(std::move(event))) return true;
        std::lock_guard<std::mutex> lock(statisticsMutex);
        ++stats.writerQueueOverflows;
        return false;
    }

    void synchronizationLoop() {
        CameraFrame camera;
        while (cameraQueue->pop(&camera)) {
            SynchronizedFrame output;
            output.frameId = camera.frameId;
            output.cameraTimestampRaw = camera.cameraTimestampRaw;
            output.cameraDeviceTimestampNs = camera.cameraTimestampNs;
            output.hostCallbackNs = camera.hostCallbackNs;
            output.image = camera.image;
            output.imageFilename = camera.imageFilename;

            int64_t referenceHostNs = 0;
            bool mappedDeviceTimestamp = false;
            if (camera.timestampValid && camera.cameraTimestampNs > 0) {
                cameraMapper.addSample(camera.cameraTimestampNs,
                                       camera.hostCallbackNs);
                mappedDeviceTimestamp = cameraMapper.mapToHost(
                    camera.cameraTimestampNs, &referenceHostNs);
            }
            if (!mappedDeviceTimestamp) {
                referenceHostNs = camera.hostCallbackNs - static_cast<int64_t>(
                    std::llround(config.cameraTransportDelayUs * 1000.0));
            }
            CameraTimestampReference timingReference = config.cameraTimestampReference;
            if (!mappedDeviceTimestamp) timingReference = CameraTimestampReference::Callback;
            const ExposureTiming timing = computeExposureTiming(
                referenceHostNs, camera.exposureUs,
                config.cameraFixedTimeOffsetUs, timingReference);
            output.exposureStartHostNs = timing.exposureStartHostNs;
            output.exposureMidHostNs = timing.exposureMidHostNs;
            output.alignedTimestampNs = timing.alignedTimestampNs;
            if (!timing.valid) {
                output.quality = SyncQuality::CAMERA_TIMESTAMP_INVALID;
            } else {
                RobotSample before;
                RobotSample after;
                const bool bracketed = robotBuffer.waitForBracketingSamples(
                    timing.alignedTimestampNs,
                    std::chrono::milliseconds(config.robotBracketWaitTimeoutMs),
                    before, after);
                if (!bracketed) {
                    int64_t earliest = 0;
                    int64_t latest = 0;
                    const bool hasBounds = robotBuffer.bounds(&earliest, &latest);
                    output.quality = !hasBounds || timing.alignedTimestampNs > latest
                        ? SyncQuality::ROBOT_DATA_NOT_READY
                        : SyncQuality::ROBOT_BRACKET_NOT_FOUND;
                } else {
                    output.robotSequenceBefore = before.sequence;
                    output.robotSequenceAfter = after.sequence;
                    output.robotSdkSequenceBefore = before.sdkReceiveSequence;
                    output.robotSdkSequenceAfter = after.sdkReceiveSequence;
                    output.robotTimeBeforeNs = before.alignedTimestampNs;
                    output.robotTimeAfterNs = after.alignedTimestampNs;
                    output.robotGapMs = static_cast<double>(
                        after.alignedTimestampNs - before.alignedTimestampNs) / kNsPerMs;
                    if (!interpolateRobotPose(before, after,
                                              timing.alignedTimestampNs,
                                              &output.flangePositionMm,
                                              &output.flangeOrientation,
                                              &output.interpolationAlpha)) {
                        output.quality = SyncQuality::ROBOT_BRACKET_NOT_FOUND;
                    } else {
                        output.baseFromFlange = makeTransform(
                            output.flangePositionMm, output.flangeOrientation);
                        if (hasFlangeFromCamera) {
                            output.baseFromCamera = output.baseFromFlange * flangeFromCamera;
                            output.hasBaseFromCamera = true;
                        }
                        output.actualScanSpeedMmS =
                            (1.0 - output.interpolationAlpha) * before.actualLinearSpeedMmS +
                            output.interpolationAlpha * after.actualLinearSpeedMmS;
                        output.filteredScanSpeedMmS =
                            (1.0 - output.interpolationAlpha) * before.filteredLinearSpeedMmS +
                            output.interpolationAlpha * after.filteredLinearSpeedMmS;
                        output.motionPhase = before.motionPhase == after.motionPhase
                            ? before.motionPhase : MotionPhase::UNSTABLE;
                        const bool sdkSequenceKnown =
                            output.robotSdkSequenceBefore > 0U &&
                            output.robotSdkSequenceAfter > 0U;
                        if (sdkSequenceKnown &&
                            output.robotSdkSequenceAfter !=
                                output.robotSdkSequenceBefore + 1U) {
                            output.quality =
                                SyncQuality::ROBOT_PACKET_SEQUENCE_GAP;
                        } else if (output.robotGapMs > config.robotInvalidGapMs) {
                            output.quality = SyncQuality::ROBOT_GAP_TOO_LARGE;
                        } else if (!camera.frameIdContinuous ||
                                   !camera.imageWriteQueued) {
                            output.quality = SyncQuality::CAMERA_FRAME_DROPPED;
                        } else if ((!before.speedStable || !after.speedStable) &&
                                   !((!config.discardAccelerationSegment &&
                                      before.motionPhase == MotionPhase::ACCELERATING &&
                                      after.motionPhase == MotionPhase::ACCELERATING) ||
                                     (!config.discardDecelerationSegment &&
                                      before.motionPhase == MotionPhase::DECELERATING &&
                                      after.motionPhase == MotionPhase::DECELERATING))) {
                            output.quality = SyncQuality::SPEED_NOT_STABLE;
                        } else {
                            output.quality = SyncQuality::VALID;
                        }
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(statisticsMutex);
                if (output.motionPhase == MotionPhase::CONSTANT_SPEED) {
                    ++stats.stableSpeedFrames;
                }
                if (output.quality == SyncQuality::VALID) {
                    ++stats.successfulSyncFrames;
                } else {
                    ++stats.invalidSyncFrames;
                }
            }
            WriterEvent event;
            event.type = WriterEvent::Type::Synchronized;
            event.synchronized = output;
            event.synchronized.image.reset();
            queueWriter(std::move(event));
            std::function<void(const SynchronizedFrame&)> callback;
            {
                std::lock_guard<std::mutex> lock(callbackMutex);
                callback = synchronizedCallback;
            }
            if (callback) callback(output);
        }
    }

    void writeSummary() {
        const PipelineStatistics snapshot = statisticsSnapshot();
        std::ofstream summary(directory + "/session_summary.json",
                              std::ios::out | std::ios::trunc);
        if (!summary) return;
        summary << std::setprecision(12)
                << "{\n"
                << "  \"configuration\": {\n"
                << "    \"camera_target_fps\": " << config.cameraTargetFps << ",\n"
                << "    \"camera_exposure_us\": " << config.cameraExposureUs << ",\n"
                << "    \"camera_fixed_time_offset_us\": " << config.cameraFixedTimeOffsetUs << ",\n"
                << "    \"camera_timestamp_reference\": \""
                << cameraTimestampReferenceName(config.cameraTimestampReference) << "\",\n"
                << "    \"robot_period_ms\": " << config.robotPeriodMs << ",\n"
                << "    \"robot_cn_de_requested_period_ms\": "
                << config.robotPeriodMs << ",\n"
                << "    \"robot_expected_feedback_period_ms\": "
                << config.robotExpectedFeedbackPeriodMs << ",\n"
                << "    \"robot_normal_gap_min_ms\": "
                << config.robotNormalGapMinMs << ",\n"
                << "    \"robot_normal_gap_max_ms\": "
                << config.robotNormalGapMaxMs << ",\n"
                << "    \"robot_warning_gap_ms\": "
                << config.robotWarningGapMs << ",\n"
                << "    \"robot_invalid_gap_ms\": "
                << config.robotInvalidGapMs << ",\n"
                << "    \"robot_time_mode_requested\": \""
                << robotTimeModeName(config.robotTimeMode) << "\",\n"
                << "    \"robot_time_mode_active\": \""
                << robotTimeModeName(robotMapper.activeMode()) << "\",\n"
                << "    \"scan_speed_mm_s\": " << config.scanSpeedMmS << ",\n"
                << "    \"scan_acceleration_mm_s2\": " << config.scanAccelerationMmS2 << ",\n"
                << "    \"speed_tolerance_percent\": "
                << config.scanSpeedTolerancePercent << ",\n"
                << "    \"speed_filter_window_samples\": "
                << config.scanSpeedFilterWindowSamples << ",\n"
                << "    \"image_writer_threads\": " << config.imageWriterThreads << "\n"
                << "  },\n"
                << "  \"actual_camera_device_fps\": " << snapshot.actualCameraFps << ",\n"
                << "  \"accepted_camera_callback_fps\": "
                << snapshot.acceptedCameraFps << ",\n"
                << "  \"actual_robot_average_feedback_hz\": " << snapshot.actualRobotHz << ",\n"
                << "  \"total_camera_frames_accepted\": " << snapshot.totalCameraFrames << ",\n"
                << "  \"camera_frame_id_skips\": " << snapshot.cameraFramesDropped << ",\n"
                << "  \"camera_queue_overflows\": " << snapshot.cameraQueueOverflows << ",\n"
                << "  \"image_pool_exhaustions\": " << snapshot.imagePoolExhaustions << ",\n"
                << "  \"image_queue_overflows\": " << snapshot.imageQueueOverflows << ",\n"
                << "  \"image_frames_written\": " << snapshot.imageFramesWritten << ",\n"
                << "  \"image_write_failures\": " << snapshot.imageWriteFailures << ",\n"
                << "  \"total_robot_samples\": " << snapshot.totalRobotSamples << ",\n"
                << "  \"robot_receive_sequence_gaps\": "
                << snapshot.robotReceiveSequenceGaps << ",\n"
                << "  \"robot_frame_counter_skips\": "
                << snapshot.robotFrameCounterSkips << ",\n"
                << "  \"robot_frame_counter_duplicates\": "
                << snapshot.robotFrameCounterDuplicates << ",\n"
                << "  \"robot_frame_counter_out_of_order\": "
                << snapshot.robotFrameCounterOutOfOrder << ",\n"
                << "  \"robot_abnormal_intervals\": " << snapshot.robotAbnormalIntervals << ",\n"
                << "  \"robot_getter_errors\": " << snapshot.robotGetterErrors << ",\n"
                << "  \"robot_getter_duration_samples\": "
                << snapshot.robotGetterDurationSamples << ",\n"
                << "  \"robot_getter_mean_us\": " << snapshot.robotGetterMeanUs << ",\n"
                << "  \"robot_getter_p99_us\": " << snapshot.robotGetterP99Us << ",\n"
                << "  \"robot_getter_max_us\": " << snapshot.robotGetterMaximumUs << ",\n"
                << "  \"successful_sync_frames\": " << snapshot.successfulSyncFrames << ",\n"
                << "  \"invalid_sync_frames\": " << snapshot.invalidSyncFrames << ",\n"
                << "  \"constant_speed_frames\": " << snapshot.stableSpeedFrames << ",\n"
                << "  \"robot_gap_mean_ms\": " << snapshot.meanRobotGapMs << ",\n"
                << "  \"robot_gap_max_ms\": " << snapshot.maximumRobotGapMs << ",\n"
                << "  \"camera_clock_scale\": " << snapshot.cameraClockFit.scale << ",\n"
                << "  \"camera_clock_offset_ns\": " << snapshot.cameraClockFit.offsetNs << ",\n"
                << "  \"camera_clock_residual_rms_ns\": "
                << snapshot.cameraClockFit.residualRmsNs << ",\n"
                << "  \"camera_clock_mapping_stable\": "
                << (snapshot.cameraClockFit.stable ? "true" : "false") << ",\n"
                << "  \"robot_clock_scale\": " << snapshot.robotClockFit.scale << ",\n"
                << "  \"robot_clock_offset_ns\": " << snapshot.robotClockFit.offsetNs << ",\n"
                << "  \"robot_clock_residual_rms_ns\": "
                << snapshot.robotClockFit.residualRmsNs << ",\n"
                << "  \"robot_clock_mapping_stable\": "
                << (snapshot.robotClockFit.stable ? "true" : "false") << ",\n"
                << "  \"writer_queue_overflows\": " << snapshot.writerQueueOverflows << ",\n"
                << "  \"run_started_utc\": \"" << jsonEscape(startUtc) << "\",\n"
                << "  \"run_ended_utc\": \"" << jsonEscape(endUtc) << "\"\n"
                << "}\n";
    }

    void imageWriterLoop() {
        ImageWriteTask task;
        while (imageQueue->pop(&task)) {
            bool written = false;
            if (task.image && !task.image->bytes.empty() &&
                !task.imageFilename.empty()) {
                const ImageBuffer& buffer = *task.image;
                const cv::Mat image(buffer.height, buffer.width, CV_8UC1,
                                    const_cast<uint8_t*>(buffer.bytes.data()),
                                    static_cast<std::size_t>(buffer.stride));
                try {
                    written = cv::imwrite(
                        directory + "/" + task.imageFilename, image,
                        {cv::IMWRITE_PNG_COMPRESSION, 1});
                } catch (const cv::Exception&) {
                    written = false;
                }
            }
            std::lock_guard<std::mutex> lock(statisticsMutex);
            if (written) {
                ++stats.imageFramesWritten;
            } else {
                ++stats.imageWriteFailures;
            }
        }
    }

    void writerLoop() {
        WriterEvent event;
        std::size_t pendingFlush = 0U;
        while (writerQueue->pop(&event)) {
            if (event.type == WriterEvent::Type::Robot && robotCsv) {
                const RobotSample& value = event.robot;
                robotCsv << value.sequence << ',' << value.sdkReceiveSequence << ','
                         << value.rawFrameCount << ',' << value.hostReceiveNs << ','
                         << value.getterDurationNs << ',' << value.getterResult << ','
                         << value.robotTimestampNs;
                for (const double pose : value.flangePoseRaw) robotCsv << ',' << pose;
                for (const double joint : value.jointPositionDeg) robotCsv << ',' << joint;
                robotCsv << ',' << value.actualLinearSpeedMmS << ','
                         << (value.valid ? 1 : 0) << ',' << value.rawFrameSequence << ','
                         << value.rawFrameAdvance << ',' << value.filteredLinearSpeedMmS << ','
                         << value.positionFitSpeedMmS << ','
                         << motionPhaseName(value.motionPhase) << '\n';
            } else if (event.type == WriterEvent::Type::Camera) {
                if (cameraCsv) {
                    const CameraFrame& value = event.camera;
                    cameraCsv << value.frameId << ',' << value.cameraTimestampRaw << ','
                              << value.cameraTimestampNs << ',' << value.hostCallbackNs << ','
                              << value.exposureUs << ',' << value.width << ',' << value.height
                              << ',' << (value.frameIdContinuous ? 1 : 0) << ','
                              << (value.timestampValid ? 1 : 0) << ','
                              << event.imageFilename << ','
                              << (value.imageWriteQueued ? 1 : 0) << '\n';
                }
            } else if (event.type == WriterEvent::Type::Synchronized && syncCsv) {
                const SynchronizedFrame& value = event.synchronized;
                syncCsv << value.frameId << ',' << value.cameraDeviceTimestampNs << ','
                        << value.hostCallbackNs << ',' << value.exposureMidHostNs << ','
                        << value.alignedTimestampNs << ',' << value.robotSequenceBefore << ','
                        << value.robotSequenceAfter << ',' << value.robotTimeBeforeNs << ','
                        << value.robotTimeAfterNs << ',' << value.interpolationAlpha << ','
                        << value.robotGapMs << ',' << value.flangePositionMm.x() << ','
                        << value.flangePositionMm.y() << ',' << value.flangePositionMm.z() << ','
                        << value.flangeOrientation.x() << ',' << value.flangeOrientation.y() << ','
                        << value.flangeOrientation.z() << ',' << value.flangeOrientation.w() << ','
                        << value.actualScanSpeedMmS << ',' << syncQualityName(value.quality) << ','
                        << value.imageFilename << ',' << value.filteredScanSpeedMmS << ','
                        << motionPhaseName(value.motionPhase) << ','
                        << value.robotSdkSequenceBefore << ','
                        << value.robotSdkSequenceAfter << '\n';
            }
            if (++pendingFlush >= 128U) {
                if (robotCsv) robotCsv.flush();
                if (cameraCsv) cameraCsv.flush();
                if (syncCsv) syncCsv.flush();
                pendingFlush = 0U;
            }
        }
        if (robotCsv) robotCsv.flush();
        if (cameraCsv) cameraCsv.flush();
        if (syncCsv) syncCsv.flush();
    }

    PipelineStatistics statisticsSnapshot() const {
        std::lock_guard<std::mutex> lock(statisticsMutex);
        PipelineStatistics snapshot = stats;
        if (lastCameraNs > firstCameraNs && stats.totalCameraFrames > 1U) {
            snapshot.acceptedCameraFps =
                static_cast<double>(stats.totalCameraFrames - 1U) * kNsPerSecond /
                static_cast<double>(lastCameraNs - firstCameraNs);
        }
        if (lastCameraDeviceNs > firstCameraDeviceNs &&
            lastCameraDeviceSequence > firstCameraDeviceSequence) {
            snapshot.actualCameraFps = static_cast<double>(
                lastCameraDeviceSequence - firstCameraDeviceSequence) * kNsPerSecond /
                static_cast<double>(lastCameraDeviceNs - firstCameraDeviceNs);
        } else if (lastCameraNs > firstCameraNs && haveCameraSequence &&
                   lastCameraSequence > firstCameraSequence) {
            snapshot.actualCameraFps = static_cast<double>(
                lastCameraSequence - firstCameraSequence) * kNsPerSecond /
                static_cast<double>(lastCameraNs - firstCameraNs);
        }
        if (lastRobotNs > firstRobotNs && stats.totalRobotSamples > 1U) {
            snapshot.actualRobotHz =
                static_cast<double>(stats.totalRobotSamples - 1U) * kNsPerSecond /
                static_cast<double>(lastRobotNs - firstRobotNs);
        }
        snapshot.meanRobotGapMs = robotGapCount == 0U ? 0.0
            : static_cast<double>(robotGapSumMs / robotGapCount);
        snapshot.robotGetterDurationSamples = robotGetterDurationsNs.size();
        if (!robotGetterDurationsNs.empty()) {
            snapshot.robotGetterMeanUs = static_cast<double>(
                robotGetterDurationSumNs /
                static_cast<long double>(robotGetterDurationsNs.size()) / 1000.0L);
            snapshot.robotGetterMaximumUs = static_cast<double>(
                *std::max_element(robotGetterDurationsNs.begin(),
                                  robotGetterDurationsNs.end())) / 1000.0;
            std::vector<int64_t> sortedDurations = robotGetterDurationsNs;
            std::sort(sortedDurations.begin(), sortedDurations.end());
            const std::size_t percentileIndex = std::min(
                sortedDurations.size() - 1U,
                static_cast<std::size_t>(
                    std::ceil(0.99 * static_cast<double>(sortedDurations.size()))) - 1U);
            snapshot.robotGetterP99Us =
                static_cast<double>(sortedDurations[percentileIndex]) / 1000.0;
        }
        snapshot.cameraClockFit = cameraMapper.report();
        snapshot.robotClockFit = robotMapper.report();
        return snapshot;
    }
};

SynchronizationSession::SynchronizationSession() = default;

SynchronizationSession::~SynchronizationSession() { stop(); }

bool SynchronizationSession::start(const SynchronizationConfig& config,
                                   const std::string& sessionDirectory,
                                   const Eigen::Matrix4d* flangeFromCamera,
                                   std::string* error) {
    stop();
    std::string validationError;
    if (!config.validate(&validationError)) {
        if (error) *error = validationError;
        return false;
    }
    if (sessionDirectory.empty()) {
        if (error) *error = "session directory is empty";
        return false;
    }
    std::error_code filesystemError;
    std::filesystem::create_directories(sessionDirectory, filesystemError);
    if (!filesystemError && config.saveImages) {
        std::filesystem::create_directories(
            std::filesystem::path(sessionDirectory) / "images", filesystemError);
    }
    if (filesystemError) {
        if (error) *error = "cannot create synchronization output directory: " +
                            filesystemError.message();
        return false;
    }

    std::unique_ptr<Impl> created = std::make_unique<Impl>(config, sessionDirectory);
    if (flangeFromCamera && flangeFromCamera->allFinite()) {
        created->flangeFromCamera = *flangeFromCamera;
        created->hasFlangeFromCamera = true;
    }
    if (config.saveRobotRawCsv) {
        created->robotCsv.open(sessionDirectory + "/robot_raw.csv",
                               std::ios::out | std::ios::trunc);
        if (!created->robotCsv) {
            if (error) *error = "cannot create robot_raw.csv";
            return false;
        }
        created->robotCsv << "sequence,sdk_receive_sequence,raw_frame_count,host_receive_ns,"
            "getter_duration_ns,getter_result,robot_timestamp_ns,"
            "flange_x_mm,flange_y_mm,flange_z_mm,flange_rx_deg,flange_ry_deg,flange_rz_deg,"
            "joint_1_deg,joint_2_deg,joint_3_deg,joint_4_deg,joint_5_deg,joint_6_deg,"
            "actual_linear_speed_mm_s,valid,raw_frame_sequence,raw_frame_advance,"
            "filtered_linear_speed_mm_s,position_fit_speed_mm_s,motion_phase\n";
    }
    if (config.saveCameraRawCsv) {
        created->cameraCsv.open(sessionDirectory + "/camera_raw.csv",
                                std::ios::out | std::ios::trunc);
        if (!created->cameraCsv) {
            if (error) *error = "cannot create camera_raw.csv";
            return false;
        }
        created->cameraCsv << "frame_id,camera_timestamp_raw,camera_timestamp_ns,host_callback_ns,"
            "exposure_us,width,height,frame_id_continuous,timestamp_valid,image_filename,"
            "image_write_queued\n";
    }
    if (config.saveSyncCsv) {
        created->syncCsv.open(sessionDirectory + "/synchronization.csv",
                              std::ios::out | std::ios::trunc);
        if (!created->syncCsv) {
            if (error) *error = "cannot create synchronization.csv";
            return false;
        }
        created->syncCsv << "frame_id,camera_timestamp_ns,host_callback_ns,exposure_mid_host_ns,"
            "aligned_timestamp_ns,robot_sequence_before,robot_sequence_after,robot_time_before_ns,"
            "robot_time_after_ns,interpolation_alpha,robot_gap_ms,flange_x_mm,flange_y_mm,"
            "flange_z_mm,flange_qx,flange_qy,flange_qz,flange_qw,actual_scan_speed_mm_s,"
            "sync_quality,image_filename,filtered_scan_speed_mm_s,motion_phase,"
            "robot_sdk_sequence_before,robot_sdk_sequence_after\n";
    }
    created->startUtc = isoUtcNow();
    created->accepting.store(true);
    created->writerThread = std::thread([pointer = created.get()] {
        pointer->writerLoop();
    });
    if (config.saveImages) {
        created->imageWriterThreads.reserve(config.imageWriterThreads);
        for (std::size_t index = 0; index < config.imageWriterThreads; ++index) {
            created->imageWriterThreads.emplace_back([pointer = created.get()] {
                pointer->imageWriterLoop();
            });
        }
    }
    created->synchronizationThread = std::thread([pointer = created.get()] {
        pointer->synchronizationLoop();
    });
    {
        std::lock_guard<std::mutex> lock(lifecycleMutex_);
        impl_ = std::move(created);
        lastStatistics_ = PipelineStatistics{};
        lastSessionDirectory_ = sessionDirectory;
        lastClockModeDescription_.clear();
    }
    if (error) error->clear();
    return true;
}

bool SynchronizationSession::pushCamera(CameraFrame frame) {
    std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
    Impl* implementation = impl_.get();
    if (!implementation || !implementation->accepting.load()) return false;
    const CounterUpdate update = implementation->cameraCounter.update(frame.frameId);
    frame.frameIdContinuous = update.first || update.continuous;
    frame.imageFilename = frameFilename(frame.frameId,
                                        implementation->config.saveImages);
    if (implementation->config.saveImages) {
        ImageWriteTask imageTask;
        imageTask.image = frame.image;
        imageTask.imageFilename = frame.imageFilename;
        if (!imageTask.image ||
            !implementation->imageQueue->tryPush(std::move(imageTask))) {
            std::lock_guard<std::mutex> lock(implementation->statisticsMutex);
            ++implementation->stats.imageQueueOverflows;
            frame.imageFilename.clear();
            frame.imageWriteQueued = false;
        }
    }
    {
        std::lock_guard<std::mutex> lock(implementation->statisticsMutex);
        ++implementation->stats.totalCameraFrames;
        implementation->stats.cameraFramesDropped += update.dropped;
        if (implementation->firstCameraNs == 0) {
            implementation->firstCameraNs = frame.hostCallbackNs;
        }
        implementation->lastCameraNs = frame.hostCallbackNs;
        if (!implementation->haveCameraSequence) {
            implementation->firstCameraSequence = update.sequence;
            implementation->haveCameraSequence = true;
        }
        implementation->lastCameraSequence = update.sequence;
        if (frame.timestampValid && frame.cameraTimestampNs > 0) {
            if (implementation->firstCameraDeviceNs == 0) {
                implementation->firstCameraDeviceNs = frame.cameraTimestampNs;
                implementation->firstCameraDeviceSequence = update.sequence;
            }
            implementation->lastCameraDeviceNs = frame.cameraTimestampNs;
            implementation->lastCameraDeviceSequence = update.sequence;
        }
    }
    WriterEvent rawEvent;
    rawEvent.type = WriterEvent::Type::Camera;
    rawEvent.camera = frame;
    rawEvent.camera.image.reset();
    rawEvent.imageFilename = frame.imageFilename;
    implementation->queueWriter(std::move(rawEvent));
    if (!implementation->cameraQueue->tryPush(frame)) {
        {
            std::lock_guard<std::mutex> lock(implementation->statisticsMutex);
            ++implementation->stats.cameraQueueOverflows;
            ++implementation->stats.invalidSyncFrames;
        }
        SynchronizedFrame dropped;
        dropped.frameId = frame.frameId;
        dropped.cameraTimestampRaw = frame.cameraTimestampRaw;
        dropped.cameraDeviceTimestampNs = frame.cameraTimestampNs;
        dropped.hostCallbackNs = frame.hostCallbackNs;
        dropped.quality = SyncQuality::CAMERA_FRAME_DROPPED;
        dropped.imageFilename = frame.imageFilename;
        WriterEvent droppedEvent;
        droppedEvent.type = WriterEvent::Type::Synchronized;
        droppedEvent.synchronized = std::move(dropped);
        implementation->queueWriter(std::move(droppedEvent));
        return false;
    }
    return true;
}

bool SynchronizationSession::pushRobot(RobotSample sample) {
    std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
    Impl* implementation = impl_.get();
    if (!implementation || !implementation->accepting.load()) return false;
    const CounterUpdate update = implementation->robotCounter.update(sample.rawFrameCount);
    sample.sequence = ++implementation->observedRobotSequence;
    sample.rawFrameSequence = update.sequence;
    sample.rawFrameAdvance =
        update.first || update.duplicate || update.outOfOrder
            ? 0U : update.dropped + 1U;
    uint64_t receiveSequenceGap = 0U;
    if (sample.sdkReceiveSequence > 0U) {
        if (implementation->lastSdkReceiveSequence > 0U &&
            sample.sdkReceiveSequence >
                implementation->lastSdkReceiveSequence + 1U) {
            receiveSequenceGap = sample.sdkReceiveSequence -
                implementation->lastSdkReceiveSequence - 1U;
        }
        implementation->lastSdkReceiveSequence = std::max(
            implementation->lastSdkReceiveSequence, sample.sdkReceiveSequence);
    }
    if (implementation->lastRobotReceiveNs > 0) {
        sample.receiveGapMs = static_cast<double>(
            sample.hostReceiveNs - implementation->lastRobotReceiveNs) / kNsPerMs;
    }
    implementation->lastRobotReceiveNs = sample.hostReceiveNs;
    implementation->robotMapper.align(&sample);
    if (sample.alignedTimestampNs <= implementation->lastRobotAlignedNs) {
        sample.alignedTimestampNs = implementation->lastRobotAlignedNs + 1;
    }
    implementation->lastRobotAlignedNs = sample.alignedTimestampNs;
    const int64_t motionTimestampNs = sample.hasRobotTimestamp &&
        sample.robotTimestampNs > 0 ? sample.robotTimestampNs : sample.hostReceiveNs;
    sample.motionPhase = implementation->speedDetector.update(
        motionTimestampNs, sample.flangePositionMm,
        sample.actualLinearSpeedMmS, &sample.filteredLinearSpeedMmS,
        &sample.positionFitSpeedMmS);
    sample.speedStable = sample.motionPhase == MotionPhase::CONSTANT_SPEED;
    implementation->robotBuffer.push(sample);
    {
        std::lock_guard<std::mutex> lock(implementation->statisticsMutex);
        ++implementation->stats.totalRobotSamples;
        implementation->stats.robotReceiveSequenceGaps += receiveSequenceGap;
        implementation->stats.robotFrameCounterSkips += update.dropped;
        if (update.duplicate) {
            ++implementation->stats.robotFrameCounterDuplicates;
        }
        if (update.outOfOrder) {
            ++implementation->stats.robotFrameCounterOutOfOrder;
        }
        if (sample.getterResult != 0) {
            ++implementation->stats.robotGetterErrors;
        }
        if (sample.getterDurationNs >= 0) {
            implementation->robotGetterDurationSumNs +=
                static_cast<long double>(sample.getterDurationNs);
            implementation->robotGetterDurationsNs.push_back(
                sample.getterDurationNs);
        }
        if (sample.receiveGapMs > implementation->config.robotWarningGapMs) {
            ++implementation->stats.robotAbnormalIntervals;
        }
        if (sample.receiveGapMs > 0.0) {
            implementation->robotGapSumMs += sample.receiveGapMs;
            ++implementation->robotGapCount;
            implementation->stats.maximumRobotGapMs = std::max(
                implementation->stats.maximumRobotGapMs, sample.receiveGapMs);
        }
        if (implementation->firstRobotNs == 0) {
            implementation->firstRobotNs = sample.hostReceiveNs;
        }
        implementation->lastRobotNs = sample.hostReceiveNs;
    }
    WriterEvent event;
    event.type = WriterEvent::Type::Robot;
    event.robot = sample;
    return implementation->queueWriter(std::move(event));
}

void SynchronizationSession::noteImagePoolExhaustion() {
    std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
    Impl* implementation = impl_.get();
    if (!implementation) return;
    std::lock_guard<std::mutex> lock(implementation->statisticsMutex);
    ++implementation->stats.imagePoolExhaustions;
}

void SynchronizationSession::stop() {
    std::unique_lock<std::mutex> lifecycleLock(lifecycleMutex_);
    std::unique_ptr<Impl> stopping = std::move(impl_);
    if (!stopping) return;
    stopping->accepting.store(false);
    stopping->cameraQueue->close();
    stopping->robotBuffer.notifyAll();
    if (stopping->synchronizationThread.joinable()) {
        stopping->synchronizationThread.join();
    }
    stopping->writerQueue->close();
    stopping->imageQueue->close();
    if (stopping->writerThread.joinable()) stopping->writerThread.join();
    for (std::thread& writer : stopping->imageWriterThreads) {
        if (writer.joinable()) writer.join();
    }
    stopping->endUtc = isoUtcNow();
    stopping->writeSummary();
    lastStatistics_ = stopping->statisticsSnapshot();
    lastSessionDirectory_ = stopping->directory;
    const ClockFitReport camera = stopping->cameraMapper.report();
    std::ostringstream description;
    description << "camera=" << (camera.stable
        ? "DEVICE_TIMESTAMP_MAPPING" : "HOST_CALLBACK_FALLBACK")
        << ", robot=" << robotTimeModeName(stopping->robotMapper.activeMode());
    lastClockModeDescription_ = description.str();
}

bool SynchronizationSession::running() const {
    std::lock_guard<std::mutex> lock(lifecycleMutex_);
    return impl_ && impl_->accepting.load();
}

PipelineStatistics SynchronizationSession::statistics() const {
    std::lock_guard<std::mutex> lock(lifecycleMutex_);
    return impl_ ? impl_->statisticsSnapshot() : lastStatistics_;
}

std::string SynchronizationSession::sessionDirectory() const {
    std::lock_guard<std::mutex> lock(lifecycleMutex_);
    return impl_ ? impl_->directory : lastSessionDirectory_;
}

std::string SynchronizationSession::clockModeDescription() const {
    std::lock_guard<std::mutex> lock(lifecycleMutex_);
    if (!impl_) return lastClockModeDescription_.empty()
        ? "not_running" : lastClockModeDescription_;
    const ClockFitReport camera = impl_->cameraMapper.report();
    std::ostringstream description;
    description << "camera=" << (camera.stable
        ? "DEVICE_TIMESTAMP_MAPPING" : "HOST_CALLBACK_FALLBACK")
        << ", robot=" << robotTimeModeName(impl_->robotMapper.activeMode());
    return description.str();
}

void SynchronizationSession::setSynchronizedFrameCallback(
        std::function<void(const SynchronizedFrame&)> callback) {
    std::lock_guard<std::mutex> lifecycleLock(lifecycleMutex_);
    if (!impl_) return;
    std::lock_guard<std::mutex> lock(impl_->callbackMutex);
    impl_->synchronizedCallback = std::move(callback);
}

int64_t controllerCalendarToNs(int year, int month, int day,
                               int hour, int minute, int second,
                               int millisecond, bool* ok) {
    const bool valid = year >= 1970 && year <= 9999 && month >= 1 && month <= 12 &&
                       day >= 1 && day <= 31 && hour >= 0 && hour <= 23 &&
                       minute >= 0 && minute <= 59 && second >= 0 && second <= 60 &&
                       millisecond >= 0 && millisecond <= 999;
    if (ok) *ok = valid;
    if (!valid) return 0;
    // Howard Hinnant's civil calendar conversion, with 1970-01-01 as day 0.
    int adjustedYear = year;
    adjustedYear -= month <= 2;
    const int era = (adjustedYear >= 0 ? adjustedYear : adjustedYear - 399) / 400;
    const unsigned int yearOfEra = static_cast<unsigned int>(adjustedYear - era * 400);
    const unsigned int adjustedMonth = static_cast<unsigned int>(
        month + (month > 2 ? -3 : 9));
    const unsigned int dayOfYear = (153U * adjustedMonth + 2U) / 5U +
                                   static_cast<unsigned int>(day - 1);
    const unsigned int dayOfEra = yearOfEra * 365U + yearOfEra / 4U -
                                  yearOfEra / 100U + dayOfYear;
    const int64_t days = static_cast<int64_t>(era) * 146097LL +
                         static_cast<int64_t>(dayOfEra) - 719468LL;
    const int64_t totalMilliseconds =
        (((days * 24LL + hour) * 60LL + minute) * 60LL + second) * 1000LL +
        millisecond;
    return totalMilliseconds * 1000000LL;
}

}  // namespace hik_sync
