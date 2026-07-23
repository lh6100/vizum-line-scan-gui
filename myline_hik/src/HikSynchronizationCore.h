#ifndef MYLINE_HIK_HIK_SYNCHRONIZATION_CORE_H
#define MYLINE_HIK_HIK_SYNCHRONIZATION_CORE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace hik_sync {

// CLOCK_MONOTONIC_RAW is the only host clock used by the synchronization
// path. Calendar/system time is used only for human-readable session metadata.
int64_t getMonotonicRawNs();

struct ImageBuffer {
    int width{0};
    int height{0};
    int stride{0};
    int pixelFormat{0};
    std::vector<uint8_t> bytes;
};

// Fixed-capacity reusable image storage. If every pooled buffer is in flight,
// acquire() returns an empty pointer instead of allocating without a bound.
class ImageBufferPool {
public:
    explicit ImageBufferPool(std::size_t capacity = 256U);
    std::shared_ptr<ImageBuffer> acquire(std::size_t byteCount);
    std::size_t capacity() const;
    std::size_t available() const;

private:
    struct State;
    std::shared_ptr<State> state_;
};

struct CounterUpdate {
    uint64_t sequence{0};
    uint64_t dropped{0};
    bool first{false};
    bool duplicate{false};
    bool continuous{true};
    bool outOfOrder{false};
};

enum class MotionPhase {
    WARMUP,
    STOPPED,
    ACCELERATING,
    CONSTANT_SPEED,
    DECELERATING,
    UNSTABLE
};

const char* motionPhaseName(MotionPhase phase);

// Unwraps an N-bit cyclic hardware counter while retaining gaps. Sequence
// advances by the raw counter delta, so missing packets remain observable.
class CyclicCounterUnwrapper {
public:
    explicit CyclicCounterUnwrapper(unsigned int bits);
    CounterUpdate update(uint64_t rawCounter);
    void reset();

private:
    uint64_t modulus_{0};
    uint64_t mask_{0};
    bool initialized_{false};
    uint64_t lastRaw_{0};
    uint64_t sequence_{0};
};

struct RobotSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t sequence{0};
    uint64_t sdkReceiveSequence{0};
    uint32_t rawFrameCount{0};
    uint64_t rawFrameSequence{0};
    uint64_t rawFrameAdvance{0};
    int64_t hostReceiveNs{0};
    int64_t getterDurationNs{-1};
    int getterResult{0};
    int64_t robotTimestampNs{0};
    bool hasRobotTimestamp{false};
    int64_t alignedTimestampNs{0};

    Eigen::Vector3d flangePositionMm{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond flangeOrientation{Eigen::Quaterniond::Identity()};
    std::array<double, 6> flangePoseRaw{};
    std::array<double, 6> jointPositionDeg{};
    std::array<double, 6> actualTcpSpeed{};
    double actualLinearSpeedMmS{0.0};
    double filteredLinearSpeedMmS{0.0};
    double positionFitSpeedMmS{0.0};

    double receiveGapMs{0.0};
    MotionPhase motionPhase{MotionPhase::WARMUP};
    bool speedStable{false};
    bool valid{false};
};

struct CameraFrame {
    uint64_t frameId{0};
    uint64_t cameraTimestampRaw{0};
    int64_t cameraTimestampNs{0};
    uint64_t cameraTimestampFrequencyHz{0};
    int64_t hostCallbackNs{0};
    double exposureUs{0.0};
    int width{0};
    int height{0};
    int pixelFormat{0};
    std::shared_ptr<ImageBuffer> image;
    std::string imageFilename;
    bool frameIdContinuous{true};
    bool imageWriteQueued{true};
    bool timestampValid{false};
};

enum class SyncQuality {
    VALID,
    ROBOT_DATA_NOT_READY,
    ROBOT_BRACKET_NOT_FOUND,
    ROBOT_PACKET_SEQUENCE_GAP,
    ROBOT_GAP_TOO_LARGE,
    CAMERA_TIMESTAMP_INVALID,
    CAMERA_FRAME_DROPPED,
    OUTSIDE_VALID_SCAN_SEGMENT,
    SPEED_NOT_STABLE
};

const char* syncQualityName(SyncQuality quality);

struct SynchronizedFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t frameId{0};
    uint64_t cameraTimestampRaw{0};
    int64_t cameraDeviceTimestampNs{0};
    int64_t hostCallbackNs{0};
    int64_t exposureStartHostNs{0};
    int64_t exposureMidHostNs{0};
    int64_t alignedTimestampNs{0};

    uint64_t robotSequenceBefore{0};
    uint64_t robotSequenceAfter{0};
    uint64_t robotSdkSequenceBefore{0};
    uint64_t robotSdkSequenceAfter{0};
    int64_t robotTimeBeforeNs{0};
    int64_t robotTimeAfterNs{0};
    double interpolationAlpha{0.0};
    double robotGapMs{0.0};

    Eigen::Vector3d flangePositionMm{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond flangeOrientation{Eigen::Quaterniond::Identity()};
    Eigen::Matrix4d baseFromFlange{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d baseFromCamera{Eigen::Matrix4d::Identity()};
    bool hasBaseFromCamera{false};
    double actualScanSpeedMmS{0.0};
    double filteredScanSpeedMmS{0.0};
    MotionPhase motionPhase{MotionPhase::WARMUP};
    SyncQuality quality{SyncQuality::ROBOT_DATA_NOT_READY};
    std::shared_ptr<ImageBuffer> image;
    std::string imageFilename;
};

enum class CameraTimestampReference { ExposureStart, ExposureEnd, Callback };
enum class RobotTimeMode { ControllerTimestamp, PeriodFit, HostReceive };

const char* cameraTimestampReferenceName(CameraTimestampReference reference);
const char* robotTimeModeName(RobotTimeMode mode);

struct SynchronizationConfig {
    double cameraTargetFps{60.0};
    double cameraExposureUs{1825.0};
    double cameraFixedTimeOffsetUs{0.0};
    double cameraTransportDelayUs{0.0};
    CameraTimestampReference cameraTimestampReference{
        CameraTimestampReference::ExposureStart};
    std::size_t cameraQueueCapacity{256U};
    std::size_t cameraClockMappingWindow{600U};
    double cameraMappingMaxResidualUs{2000.0};

    double robotPeriodMs{10.0};
    double robotExpectedFeedbackPeriodMs{12.0};
    double robotPoseBufferDurationS{5.0};
    std::size_t robotQueueCapacity{2048U};
    RobotTimeMode robotTimeMode{RobotTimeMode::ControllerTimestamp};
    double robotNormalGapMinMs{10.0};
    double robotNormalGapMaxMs{14.0};
    double robotWarningGapMs{18.0};
    double robotInvalidGapMs{25.0};
    int robotBracketWaitTimeoutMs{50};

    double scanSpeedMmS{10.0};
    double scanMinSpeedMmS{10.0};
    double scanMaxSpeedMmS{50.0};
    double scanSpeedTolerancePercent{5.0};
    double scanAccelerationMmS2{100.0};
    int scanStableSampleCount{5};
    int scanSpeedFilterWindowSamples{15};
    bool discardAccelerationSegment{true};
    bool discardDecelerationSegment{true};
    double preScanDistanceMm{30.0};
    double postScanDistanceMm{30.0};

    bool saveImages{true};
    bool saveRobotRawCsv{true};
    bool saveCameraRawCsv{true};
    bool saveSyncCsv{true};
    std::string outputDirectory{"./data/scan"};
    std::size_t writerQueueCapacity{8192U};
    std::size_t imageWriterThreads{2U};

    bool validate(std::string* error = nullptr) const;
    static bool loadYaml(const std::string& path,
                         SynchronizationConfig* config,
                         std::string* error = nullptr);
};

// Fairino SDK 3.9.4 robot_types.h defines RPY as rotations about fixed X, Y,
// Z axes in degrees. Applying those extrinsic rotations in X/Y/Z order gives
// R = Rz(rz) * Ry(ry) * Rx(rx), matching fairinoBaseFromFlange() already used
// by the hand-eye calibration path.
Eigen::Quaterniond fairinoFixedAxisRpyDegrees(double rxDeg,
                                              double ryDeg,
                                              double rzDeg);

Eigen::Matrix4d makeTransform(const Eigen::Vector3d& positionMm,
                              const Eigen::Quaterniond& orientation);

bool interpolateRobotPose(const RobotSample& before,
                          const RobotSample& after,
                          int64_t targetTimestampNs,
                          Eigen::Vector3d* positionMm,
                          Eigen::Quaterniond* orientation,
                          double* alpha);

int64_t cameraTicksToNs(uint64_t rawTicks, uint64_t frequencyHz, bool* ok = nullptr);

struct ExposureTiming {
    int64_t exposureStartHostNs{0};
    int64_t exposureMidHostNs{0};
    int64_t alignedTimestampNs{0};
    bool valid{false};
};

ExposureTiming computeExposureTiming(int64_t referenceHostNs,
                                     double exposureUs,
                                     double fixedOffsetUs,
                                     CameraTimestampReference reference);

struct ClockFitReport {
    double scale{1.0};
    double offsetNs{0.0};
    double residualRmsNs{0.0};
    std::size_t sampleCount{0U};
    bool stable{false};
};

class CameraClockMapper {
public:
    CameraClockMapper(std::size_t windowSize = 600U,
                      double maximumResidualNs = 2000000.0);
    void addSample(int64_t cameraTimestampNs, int64_t hostCallbackNs);
    bool mapToHost(int64_t cameraTimestampNs, int64_t* hostTimestampNs) const;
    ClockFitReport report() const;
    void reset();

private:
    void refitLocked();
    std::size_t windowSize_;
    double maximumResidualNs_;
    mutable std::mutex mutex_;
    std::deque<std::pair<int64_t, int64_t>> samples_;
    ClockFitReport report_;
};

class RobotClockMapper {
public:
    RobotClockMapper(RobotTimeMode requestedMode,
                     double nominalPeriodMs,
                     std::size_t windowSize = 625U);
    void align(RobotSample* sample);
    RobotTimeMode activeMode() const;
    ClockFitReport report() const;
    void reset();

private:
    RobotTimeMode requestedMode_;
    RobotTimeMode activeMode_;
    double nominalPeriodNs_;
    std::size_t windowSize_;
    mutable std::mutex mutex_;
    std::deque<std::pair<double, double>> periodSamples_;
    std::deque<std::pair<double, double>> controllerSamples_;
    ClockFitReport periodReport_;
    ClockFitReport controllerReport_;
};

class SpeedStabilityDetector {
public:
    SpeedStabilityDetector(double targetSpeedMmS,
                           double tolerancePercent,
                           int requiredSamples,
                           int filterWindowSamples = 15);
    MotionPhase update(int64_t timestampNs,
                       const Eigen::Vector3d& positionMm,
                       double actualSpeedMmS,
                       double* filteredSpeedMmS = nullptr,
                       double* positionFitSpeedMmS = nullptr);
    void reset();

private:
    struct Observation {
        int64_t timestampNs{0};
        Eigen::Vector3d positionMm{Eigen::Vector3d::Zero()};
        double reportedSpeedMmS{0.0};
    };

    double targetSpeedMmS_;
    double toleranceFraction_;
    int requiredSamples_;
    std::size_t filterWindowSamples_;
    std::deque<Observation> observations_;
    int enterConsecutive_{0};
    int exitConsecutive_{0};
    bool constantSpeed_{false};
    bool reachedConstantSpeed_{false};
};

class RobotPoseBuffer {
public:
    RobotPoseBuffer(std::size_t capacity = 2048U,
                    double durationSeconds = 5.0);
    void push(const RobotSample& sample);
    bool getBracketingSamples(int64_t targetTimestampNs,
                              RobotSample& before,
                              RobotSample& after) const;
    bool waitForBracketingSamples(int64_t targetTimestampNs,
                                  std::chrono::milliseconds timeout,
                                  RobotSample& before,
                                  RobotSample& after) const;
    bool bounds(int64_t* earliestNs, int64_t* latestNs) const;
    std::size_t size() const;
    void clear();
    void notifyAll();

private:
    bool getBracketingSamplesLocked(int64_t targetTimestampNs,
                                    RobotSample& before,
                                    RobotSample& after) const;
    std::size_t capacity_;
    int64_t durationNs_;
    mutable std::mutex mutex_;
    mutable std::condition_variable condition_;
    std::deque<RobotSample, Eigen::aligned_allocator<RobotSample>> samples_;
};

struct PipelineStatistics {
    uint64_t totalCameraFrames{0};
    uint64_t cameraFramesDropped{0};
    uint64_t cameraQueueOverflows{0};
    uint64_t imagePoolExhaustions{0};
    uint64_t imageQueueOverflows{0};
    uint64_t imageFramesWritten{0};
    uint64_t imageWriteFailures{0};
    uint64_t totalRobotSamples{0};
    uint64_t robotFrameCounterSkips{0};
    uint64_t robotFrameCounterDuplicates{0};
    uint64_t robotFrameCounterOutOfOrder{0};
    uint64_t robotReceiveSequenceGaps{0};
    uint64_t robotAbnormalIntervals{0};
    uint64_t robotGetterErrors{0};
    uint64_t robotGetterDurationSamples{0};
    uint64_t successfulSyncFrames{0};
    uint64_t invalidSyncFrames{0};
    uint64_t stableSpeedFrames{0};
    uint64_t writerQueueOverflows{0};
    double actualCameraFps{0.0};
    double acceptedCameraFps{0.0};
    double actualRobotHz{0.0};
    double meanRobotGapMs{0.0};
    double maximumRobotGapMs{0.0};
    double robotGetterMeanUs{0.0};
    double robotGetterP99Us{0.0};
    double robotGetterMaximumUs{0.0};
    ClockFitReport cameraClockFit;
    ClockFitReport robotClockFit;
};

// Threaded camera/robot association and writer pipeline. pushCamera() and
// pushRobot() are bounded, non-blocking callback entry points.
class SynchronizationSession {
public:
    SynchronizationSession();
    ~SynchronizationSession();

    bool start(const SynchronizationConfig& config,
               const std::string& sessionDirectory,
               const Eigen::Matrix4d* flangeFromCamera,
               std::string* error = nullptr);
    bool pushCamera(CameraFrame frame);
    bool pushRobot(RobotSample sample);
    void noteImagePoolExhaustion();
    void stop();
    bool running() const;
    PipelineStatistics statistics() const;
    std::string sessionDirectory() const;
    std::string clockModeDescription() const;
    void setSynchronizedFrameCallback(
        std::function<void(const SynchronizedFrame&)> callback);

private:
    struct Impl;
    mutable std::mutex lifecycleMutex_;
    std::unique_ptr<Impl> impl_;
    PipelineStatistics lastStatistics_;
    std::string lastSessionDirectory_;
    std::string lastClockModeDescription_;
};

// Converts the calendar fields from FAIRINO RobotTime into a monotonically
// comparable millisecond-resolution scalar. The timezone is intentionally not
// applied because only controller-clock differences and affine mapping matter.
int64_t controllerCalendarToNs(int year, int month, int day,
                               int hour, int minute, int second,
                               int millisecond, bool* ok = nullptr);

}  // namespace hik_sync

#endif  // MYLINE_HIK_HIK_SYNCHRONIZATION_CORE_H
