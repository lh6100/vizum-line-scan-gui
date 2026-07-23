#include "HikSynchronizationCore.h"

#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

int failures = 0;

#define CHECK_TRUE(condition, message)                                           \
    do {                                                                          \
        if (!(condition)) {                                                        \
            std::cerr << "FAIL: " << (message) << " (line " << __LINE__ << ")\n"; \
            ++failures;                                                            \
        }                                                                          \
    } while (false)

bool near(double first, double second, double tolerance) {
    return std::abs(first - second) <= tolerance;
}

hik_sync::RobotSample sampleAt(uint64_t sequence, int64_t timestampNs,
                               double xMm, double speedMmS = 20.0) {
    hik_sync::RobotSample sample;
    sample.sequence = sequence;
    sample.rawFrameCount = static_cast<uint32_t>(sequence & 0xffU);
    sample.hostReceiveNs = timestampNs;
    sample.alignedTimestampNs = timestampNs;
    sample.flangePositionMm = Eigen::Vector3d(xMm, 2.0, 3.0);
    sample.flangeOrientation = Eigen::Quaterniond::Identity();
    sample.flangePoseRaw = {xMm, 2.0, 3.0, 0.0, 0.0, 0.0};
    sample.actualLinearSpeedMmS = speedMmS;
    sample.actualTcpSpeed[0] = speedMmS;
    sample.valid = true;
    sample.speedStable = true;
    return sample;
}

void testMonotonicClock() {
    const int64_t first = hik_sync::getMonotonicRawNs();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const int64_t second = hik_sync::getMonotonicRawNs();
    CHECK_TRUE(first > 0 && second > first,
               "CLOCK_MONOTONIC_RAW wrapper must increase");
}

void testCountersAndDrops() {
    hik_sync::CyclicCounterUnwrapper robot(8U);
    CHECK_TRUE(robot.update(254U).sequence == 254U, "first raw count retained");
    CHECK_TRUE(robot.update(255U).sequence == 255U, "normal robot counter step");
    const hik_sync::CounterUpdate wrapped = robot.update(0U);
    CHECK_TRUE(wrapped.sequence == 256U && wrapped.continuous,
               "8-bit robot frame counter unwrap");
    const hik_sync::CounterUpdate lost = robot.update(3U);
    CHECK_TRUE(lost.sequence == 259U && lost.dropped == 2U && !lost.continuous,
               "robot packet loss retained by unwrapped sequence");
    CHECK_TRUE(robot.update(3U).duplicate, "duplicate robot state detected");

    hik_sync::CyclicCounterUnwrapper camera(32U);
    camera.update(100U);
    const hik_sync::CounterUpdate cameraLost = camera.update(103U);
    CHECK_TRUE(cameraLost.dropped == 2U && !cameraLost.continuous,
               "camera FrameID jump detected");
}

void testPoseBufferAndInterpolation() {
    hik_sync::RobotPoseBuffer buffer(2048U, 5.0);
    hik_sync::RobotSample first = sampleAt(1U, 1000, 10.0);
    hik_sync::RobotSample second = sampleAt(2U, 2000, 20.0);
    second.flangeOrientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    buffer.push(first);
    buffer.push(second);
    hik_sync::RobotSample before;
    hik_sync::RobotSample after;
    CHECK_TRUE(buffer.getBracketingSamples(1500, before, after),
               "binary bracketing finds surrounding samples");
    CHECK_TRUE(before.sequence == 1U && after.sequence == 2U,
               "bracketing order is correct");
    CHECK_TRUE(!buffer.getBracketingSamples(999, before, after) &&
               !buffer.getBracketingSamples(2001, before, after),
               "out-of-range timestamps rejected");

    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double alpha = -1.0;
    CHECK_TRUE(hik_sync::interpolateRobotPose(
                   first, second, 1500, &position, &orientation, &alpha),
               "pose interpolation succeeds");
    CHECK_TRUE(near(alpha, 0.5, 1e-12) && near(position.x(), 15.0, 1e-12),
               "linear translation and alpha are correct");
    const Eigen::Vector3d rotated = orientation * Eigen::Vector3d::UnitX();
    CHECK_TRUE(near(rotated.x(), 0.0, 1e-9) && near(std::abs(rotated.y()), 1.0, 1e-9),
               "quaternion SLERP produces the halfway rotation");
    CHECK_TRUE(hik_sync::interpolateRobotPose(
                   first, second, 1000, &position, &orientation, &alpha) &&
               near(alpha, 0.0, 0.0), "alpha lower boundary");
    CHECK_TRUE(hik_sync::interpolateRobotPose(
                   first, second, 2000, &position, &orientation, &alpha) &&
               near(alpha, 1.0, 0.0), "alpha upper boundary");
}

void testFairinoOrientation() {
    const Eigen::Quaterniond orientation =
        hik_sync::fairinoFixedAxisRpyDegrees(0.0, 0.0, 90.0);
    const Eigen::Vector3d rotated = orientation * Eigen::Vector3d::UnitX();
    CHECK_TRUE(near(rotated.x(), 0.0, 1e-12) && near(rotated.y(), 1.0, 1e-12),
               "Fairino fixed-axis Rz*Ry*Rx conversion");
}

void testCameraClockMapping() {
    hik_sync::CameraClockMapper mapper(100U, 500000.0);
    constexpr int64_t cameraBase = 1000000000000LL;
    constexpr int64_t offset = 4000000000LL;
    for (int index = 0; index < 100; ++index) {
        const int64_t camera = cameraBase + index * 16666667LL;
        const int64_t jitter = (index % 20 == 0) ? 8000000LL : (index % 5) * 10000LL;
        mapper.addSample(camera, camera + offset + jitter);
    }
    const hik_sync::ClockFitReport report = mapper.report();
    CHECK_TRUE(report.stable && near(report.scale, 1.0, 1e-5),
               "robust camera affine mapping becomes stable");
    int64_t mapped = 0;
    CHECK_TRUE(mapper.mapToHost(cameraBase + 2000000000LL, &mapped),
               "stable camera mapping maps timestamps");
    CHECK_TRUE(std::abs(mapped - (cameraBase + 2000000000LL + offset)) < 200000,
               "camera mapping rejects positive network-delay outliers");

    bool converted = false;
    CHECK_TRUE(hik_sync::cameraTicksToNs(125000000U, 125000000U, &converted) ==
                   1000000000LL && converted,
               "device timestamp tick frequency conversion");
    (void)hik_sync::cameraTicksToNs(1U, 0U, &converted);
    CHECK_TRUE(!converted, "unknown timestamp frequency rejected");
}

void testExposureAndSpeed() {
    const hik_sync::ExposureTiming start = hik_sync::computeExposureTiming(
        1000000000LL, 1825.0, 0.0,
        hik_sync::CameraTimestampReference::ExposureStart);
    CHECK_TRUE(start.valid && start.exposureStartHostNs == 1000000000LL &&
               start.exposureMidHostNs == 1000912500LL,
               "exposure-start midpoint calculation");
    const hik_sync::ExposureTiming end = hik_sync::computeExposureTiming(
        1002000000LL, 2000.0, 50.0,
        hik_sync::CameraTimestampReference::ExposureEnd);
    CHECK_TRUE(end.exposureStartHostNs == 1000000000LL &&
               end.exposureMidHostNs == 1001000000LL &&
               end.alignedTimestampNs == 1001050000LL,
               "exposure-end and fixed-offset calculation");

    hik_sync::SpeedStabilityDetector detector(20.0, 5.0, 3, 7);
    hik_sync::MotionPhase phase = hik_sync::MotionPhase::WARMUP;
    double filtered = 0.0;
    double fitted = 0.0;
    for (int index = 0; index < 12; ++index) {
        const int64_t timestamp = 1000000000LL + index * 10000000LL;
        const double position = index * 0.010 * 20.0;
        const double noisyFeedback = index % 2 == 0 ? 18.5 : 21.5;
        phase = detector.update(timestamp, Eigen::Vector3d(position, 0.0, 0.0),
                                noisyFeedback, &filtered, &fitted);
    }
    CHECK_TRUE(phase == hik_sync::MotionPhase::CONSTANT_SPEED,
               "position fit and median filter recognize constant speed");
    CHECK_TRUE(near(fitted, 20.0, 1e-9) && near(filtered, 20.375, 0.5),
               "filtered speed suppresses noisy instantaneous feedback");
    for (int index = 12; index < 28; ++index) {
        phase = detector.update(1000000000LL + index * 10000000LL,
                                Eigen::Vector3d(12 * 0.010 * 20.0, 0.0, 0.0),
                                0.0, &filtered, &fitted);
    }
    CHECK_TRUE(phase == hik_sync::MotionPhase::STOPPED ||
               phase == hik_sync::MotionPhase::DECELERATING,
               "filtered state machine leaves constant phase after deceleration");
}

void testConfigurationAndControllerTime() {
    hik_sync::SynchronizationConfig config;
    CHECK_TRUE(config.robotTimeMode == hik_sync::RobotTimeMode::ControllerTimestamp,
               "controller RobotTime is the default robot clock");
    CHECK_TRUE(near(config.robotPeriodMs, 10.0, 0.0) &&
               near(config.robotExpectedFeedbackPeriodMs, 12.0, 0.0) &&
               near(config.robotWarningGapMs, 18.0, 0.0) &&
               near(config.robotInvalidGapMs, 25.0, 0.0) &&
               config.reconstructionQueueCapacity == 64U &&
               config.reconstructionThreads == 2U,
               "10 ms CNDE request uses measured 12 ms feedback thresholds");
    std::string error;
    config.scanSpeedMmS = 10.0;
    CHECK_TRUE(config.validate(&error), "minimum scan speed accepted");
    config.scanSpeedMmS = 50.0;
    CHECK_TRUE(config.validate(&error), "maximum scan speed accepted");
    config.scanSpeedMmS = 50.01;
    CHECK_TRUE(!config.validate(&error), "out-of-range scan speed rejected");
    bool valid = false;
    const int64_t first = hik_sync::controllerCalendarToNs(
        2026, 7, 22, 12, 0, 0, 0, &valid);
    const int64_t second = hik_sync::controllerCalendarToNs(
        2026, 7, 22, 12, 0, 0, 10, &valid);
    CHECK_TRUE(valid && second - first == 10000000LL,
               "FAIRINO RobotTime millisecond conversion");

    hik_sync::RobotClockMapper mapper(
        hik_sync::RobotTimeMode::ControllerTimestamp, 10.0, 100U);
    for (uint64_t index = 0; index < 100U; ++index) {
        hik_sync::RobotSample sample;
        sample.sequence = index + 1U;
        sample.robotTimestampNs = first + static_cast<int64_t>(index) * 10000000LL;
        sample.hostReceiveNs = sample.robotTimestampNs + 500000000LL +
            static_cast<int64_t>(index % 3U) * 10000LL;
        sample.hasRobotTimestamp = true;
        mapper.align(&sample);
    }
    CHECK_TRUE(mapper.activeMode() == hik_sync::RobotTimeMode::ControllerTimestamp &&
               mapper.report().stable,
               "controller RobotTime affine mapping becomes active");
}

void testSyntheticPipeline() {
    hik_sync::SynchronizationConfig config;
    config.robotTimeMode = hik_sync::RobotTimeMode::HostReceive;
    config.scanSpeedMmS = 20.0;
    config.scanStableSampleCount = 5;
    config.saveImages = true;
    config.saveRobotRawCsv = true;
    config.saveCameraRawCsv = true;
    config.saveSyncCsv = true;
    config.cameraQueueCapacity = 256U;
    config.writerQueueCapacity = 4096U;
    const std::filesystem::path output = std::filesystem::temp_directory_path() /
        ("hik_sync_test_" + std::to_string(hik_sync::getMonotonicRawNs()));

    hik_sync::SynchronizationSession session;
    std::string error;
    CHECK_TRUE(session.start(config, output.string(), nullptr, &error),
               std::string("synthetic session starts: ") + error);
    std::mutex outputMutex;
    std::vector<hik_sync::SynchronizedFrame,
                Eigen::aligned_allocator<hik_sync::SynchronizedFrame>> synchronized;
    session.setSynchronizedFrameCallback(
        [&outputMutex, &synchronized](const hik_sync::SynchronizedFrame& frame) {
            std::lock_guard<std::mutex> lock(outputMutex);
            synchronized.push_back(frame);
        });

    const int64_t base = hik_sync::getMonotonicRawNs() + 100000000LL;
    // 100 Hz robot: raw frame-counter skips remain diagnostic; two adjacent
    // missing observations create a time gap larger than the invalid limit.
    for (uint64_t index = 0U; index <= 150U; ++index) {
        if (index == 50U || index == 90U || index == 91U) continue;
        const int64_t timestamp = base + static_cast<int64_t>(index) * 10000000LL;
        hik_sync::RobotSample robot = sampleAt(
            index, timestamp, 20.0 * (timestamp - base) / 1.0e9, 20.0);
        session.pushRobot(robot);
    }
    // 60 Hz camera. Callback is 1 ms after the desired exposure midpoint;
    // exposure=2 ms makes HOST_CALLBACK_FALLBACK recover that midpoint.
    for (uint64_t index = 0U; index < 60U; ++index) {
        hik_sync::CameraFrame camera;
        camera.frameId = index < 30U ? index : index + 1U;  // one FrameID jump
        const int64_t midpoint = base + 50000000LL +
            static_cast<int64_t>(std::llround(index * (1.0e9 / 60.0)));
        camera.hostCallbackNs = midpoint + 1000000LL;
        camera.exposureUs = 2000.0;
        camera.width = 1;
        camera.height = 1;
        camera.image = std::make_shared<hik_sync::ImageBuffer>();
        camera.image->width = 1;
        camera.image->height = 1;
        camera.image->stride = 1;
        camera.image->bytes.assign(1U, static_cast<uint8_t>(index));
        session.pushCamera(camera);
    }
    session.stop();
    const hik_sync::PipelineStatistics stats = session.statistics();

    CHECK_TRUE(synchronized.size() == 60U,
               "every simulated camera frame produces an association result");
    bool sawRobotGapTooLarge = false;
    bool sawCameraDrop = false;
    double maximumValidPositionError = 0.0;
    for (const hik_sync::SynchronizedFrame& frame : synchronized) {
        sawRobotGapTooLarge = sawRobotGapTooLarge ||
            frame.quality == hik_sync::SyncQuality::ROBOT_GAP_TOO_LARGE;
        sawCameraDrop = sawCameraDrop ||
            frame.quality == hik_sync::SyncQuality::CAMERA_FRAME_DROPPED;
        CHECK_TRUE(frame.interpolationAlpha >= 0.0 && frame.interpolationAlpha <= 1.0,
                   "simulated interpolation alpha remains bounded");
        if (frame.robotTimeAfterNs > frame.robotTimeBeforeNs) {
            const double expected = 20.0 * (frame.alignedTimestampNs - base) / 1.0e9;
            maximumValidPositionError = std::max(
                maximumValidPositionError,
                std::abs(frame.flangePositionMm.x() - expected));
        }
    }
    CHECK_TRUE(maximumValidPositionError < 1e-9,
               "100 Hz to 60 Hz position interpolation matches theory");
    CHECK_TRUE(sawRobotGapTooLarge,
               "robot gap above invalid threshold is marked too large");
    CHECK_TRUE(sawCameraDrop, "simulated camera FrameID jump changes quality");
    CHECK_TRUE(std::filesystem::exists(output / "robot_raw.csv") &&
               std::filesystem::exists(output / "camera_raw.csv") &&
               std::filesystem::exists(output / "synchronization.csv") &&
               std::filesystem::exists(output / "session_summary.json"),
               "writer produces all raw, synchronized, and summary files");
    CHECK_TRUE(stats.robotFrameCounterSkips == 3U,
               "raw robot frame counter skips remain diagnostic");
    CHECK_TRUE(stats.imageFramesWritten == 60U &&
               stats.imageWriteFailures == 0U &&
               std::filesystem::exists(output / "images/frame_000000000000.png"),
               "parallel image writers drain all queued images");
    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

void testRobotPacketsDoNotDependOnFrameCounter() {
    hik_sync::SynchronizationConfig config;
    config.robotTimeMode = hik_sync::RobotTimeMode::HostReceive;
    config.saveImages = false;
    config.saveRobotRawCsv = false;
    config.saveCameraRawCsv = false;
    config.saveSyncCsv = false;
    const std::filesystem::path output = std::filesystem::temp_directory_path() /
        ("hik_sync_packet_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));

    hik_sync::SynchronizationSession session;
    std::string error;
    CHECK_TRUE(session.start(config, output.string(), nullptr, &error),
               std::string("packet session starts: ") + error);
    const int64_t base = hik_sync::getMonotonicRawNs();
    const std::array<uint64_t, 3> receiveSequences{{1U, 2U, 4U}};
    const std::array<int64_t, 3> getterDurations{{1000, 2000, 100000}};
    for (std::size_t index = 0; index < receiveSequences.size(); ++index) {
        hik_sync::RobotSample robot = sampleAt(
            7U, base + static_cast<int64_t>(index) * 10000000LL,
            static_cast<double>(index), 10.0);
        robot.sdkReceiveSequence = receiveSequences[index];
        robot.getterDurationNs = getterDurations[index];
        session.pushRobot(robot);
    }
    session.stop();
    const hik_sync::PipelineStatistics stats = session.statistics();
    CHECK_TRUE(stats.totalRobotSamples == 3U,
               "every SDK packet is accepted even when frame_cnt repeats");
    CHECK_TRUE(stats.robotFrameCounterDuplicates == 2U,
               "repeated frame_cnt values remain diagnostic");
    CHECK_TRUE(stats.robotReceiveSequenceGaps == 1U,
               "SDK receive sequence reports a real callback/SPSC gap");
    CHECK_TRUE(stats.robotGetterDurationSamples == 3U &&
               std::abs(stats.robotGetterMeanUs - 34.3333333333) < 1e-6 &&
               std::abs(stats.robotGetterP99Us - 100.0) < 1e-9 &&
               std::abs(stats.robotGetterMaximumUs - 100.0) < 1e-9,
               "getter mean, P99 and maximum are reported");
    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

void testSdkPacketSequenceGapInvalidatesInterpolation() {
    hik_sync::SynchronizationConfig config;
    config.robotTimeMode = hik_sync::RobotTimeMode::HostReceive;
    config.saveImages = false;
    config.saveRobotRawCsv = false;
    config.saveCameraRawCsv = false;
    config.saveSyncCsv = false;
    const std::filesystem::path output = std::filesystem::temp_directory_path() /
        ("hik_sync_sequence_quality_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));

    hik_sync::SynchronizationSession session;
    std::string error;
    CHECK_TRUE(session.start(config, output.string(), nullptr, &error),
               std::string("sequence-quality session starts: ") + error);
    std::mutex outputMutex;
    std::vector<hik_sync::SynchronizedFrame,
                Eigen::aligned_allocator<hik_sync::SynchronizedFrame>> synchronized;
    session.setSynchronizedFrameCallback(
        [&outputMutex, &synchronized](const hik_sync::SynchronizedFrame& frame) {
            std::lock_guard<std::mutex> lock(outputMutex);
            synchronized.push_back(frame);
        });

    const int64_t base = hik_sync::getMonotonicRawNs() + 100000000LL;
    const std::array<uint64_t, 3> receiveSequences{{1U, 2U, 4U}};
    for (std::size_t index = 0; index < receiveSequences.size(); ++index) {
        hik_sync::RobotSample robot = sampleAt(
            static_cast<uint64_t>(index + 1U),
            base + static_cast<int64_t>(index) * 12000000LL,
            static_cast<double>(index), 10.0);
        robot.sdkReceiveSequence = receiveSequences[index];
        session.pushRobot(robot);
    }

    hik_sync::CameraFrame camera;
    camera.frameId = 1U;
    camera.hostCallbackNs = base + 18000000LL + 1000000LL;
    camera.exposureUs = 2000.0;
    session.pushCamera(camera);
    session.stop();

    CHECK_TRUE(synchronized.size() == 1U,
               "sequence-quality camera frame produces one association");
    if (synchronized.size() == 1U) {
        const hik_sync::SynchronizedFrame& frame = synchronized.front();
        CHECK_TRUE(frame.robotSdkSequenceBefore == 2U &&
                   frame.robotSdkSequenceAfter == 4U,
                   "bracketing SDK packet sequences are retained");
        CHECK_TRUE(
            frame.quality == hik_sync::SyncQuality::ROBOT_PACKET_SEQUENCE_GAP,
            "non-contiguous SDK packet sequence invalidates interpolation");
    }
    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

void testContiguousSdkPacketsAllow21MsControllerGap() {
    hik_sync::SynchronizationConfig config;
    config.robotTimeMode = hik_sync::RobotTimeMode::HostReceive;
    config.saveImages = false;
    config.saveRobotRawCsv = false;
    config.saveCameraRawCsv = false;
    config.saveSyncCsv = false;
    const std::filesystem::path output = std::filesystem::temp_directory_path() /
        ("hik_sync_contiguous_gap_test_" +
         std::to_string(hik_sync::getMonotonicRawNs()));

    hik_sync::SynchronizationSession session;
    std::string error;
    CHECK_TRUE(session.start(config, output.string(), nullptr, &error),
               std::string("contiguous-gap session starts: ") + error);
    std::mutex outputMutex;
    std::vector<hik_sync::SynchronizedFrame,
                Eigen::aligned_allocator<hik_sync::SynchronizedFrame>> synchronized;
    session.setSynchronizedFrameCallback(
        [&outputMutex, &synchronized](const hik_sync::SynchronizedFrame& frame) {
            std::lock_guard<std::mutex> lock(outputMutex);
            synchronized.push_back(frame);
        });

    const int64_t base = hik_sync::getMonotonicRawNs() + 100000000LL;
    int64_t timestamp = base;
    for (uint64_t index = 0U; index <= 21U; ++index) {
        if (index > 0U) timestamp += index == 21U ? 21000000LL : 12000000LL;
        const double xMm = 10.0 * static_cast<double>(timestamp - base) / 1.0e9;
        hik_sync::RobotSample robot = sampleAt(index + 1U, timestamp, xMm, 10.0);
        robot.sdkReceiveSequence = index + 1U;
        session.pushRobot(robot);
    }

    const int64_t gapStart = base + 20LL * 12000000LL;
    const int64_t exposureMidpoint = gapStart + 10500000LL;
    hik_sync::CameraFrame camera;
    camera.frameId = 1U;
    camera.hostCallbackNs = exposureMidpoint + 1000000LL;
    camera.exposureUs = 2000.0;
    session.pushCamera(camera);
    session.stop();

    CHECK_TRUE(synchronized.size() == 1U,
               "21 ms contiguous-gap camera frame produces one association");
    if (synchronized.size() == 1U) {
        const hik_sync::SynchronizedFrame& frame = synchronized.front();
        CHECK_TRUE(frame.robotSdkSequenceBefore == 21U &&
                   frame.robotSdkSequenceAfter == 22U,
                   "21 ms gap retains contiguous SDK packet sequences");
        CHECK_TRUE(near(frame.robotGapMs, 21.0, 1e-9),
                   "controller gap is measured as 21 ms");
        CHECK_TRUE(frame.quality == hik_sync::SyncQuality::VALID,
                   "contiguous 21 ms controller gap remains valid below 25 ms");
    }
    std::error_code removeError;
    std::filesystem::remove_all(output, removeError);
}

}  // namespace

int main() {
    testMonotonicClock();
    testCountersAndDrops();
    testPoseBufferAndInterpolation();
    testFairinoOrientation();
    testCameraClockMapping();
    testExposureAndSpeed();
    testConfigurationAndControllerTime();
    testSyntheticPipeline();
    testRobotPacketsDoNotDependOnFrameCounter();
    testSdkPacketSequenceGapInvalidatesInterpolation();
    testContiguousSdkPacketsAllow21MsControllerGap();
    if (failures != 0) {
        std::cerr << failures << " synchronization test(s) failed\n";
        return 1;
    }
    std::cout << "All Hik synchronization tests passed\n";
    return 0;
}
