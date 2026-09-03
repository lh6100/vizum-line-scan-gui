#ifndef MYLINE_HIK_STEREO_FRAME_SYNCHRONIZER_H
#define MYLINE_HIK_STEREO_FRAME_SYNCHRONIZER_H

#include "HikSynchronizationCore.h"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace hik_stereo {

struct StereoFramePair {
    hik_sync::CameraFrame left;
    hik_sync::CameraFrame right;
    std::int64_t midpointHostNs{0};
    double skewMs{0.0};
};

struct StereoFramePairerStatistics {
    std::uint64_t leftFrames{0};
    std::uint64_t rightFrames{0};
    std::uint64_t pairedFrames{0};
    std::uint64_t leftDropped{0};
    std::uint64_t rightDropped{0};
};

// Pairs monotonic camera callbacks without allocating image memory. Frames
// retain shared ownership of the camera buffer until processing completes.
class StereoFramePairer {
public:
    explicit StereoFramePairer(double maximumSkewMs = 5.0,
                               std::size_t queueCapacity = 8U);

    void configure(double maximumSkewMs, std::size_t queueCapacity);
    void pushLeft(hik_sync::CameraFrame frame);
    void pushRight(hik_sync::CameraFrame frame);
    bool takePair(StereoFramePair* pair);
    void clear();
    StereoFramePairerStatistics statistics() const { return statistics_; }

private:
    void pairAvailable();
    double maximumSkewMs_{5.0};
    std::size_t queueCapacity_{8U};
    std::deque<hik_sync::CameraFrame> left_;
    std::deque<hik_sync::CameraFrame> right_;
    std::deque<StereoFramePair> pairs_;
    StereoFramePairerStatistics statistics_;
};

struct SynchronizedStereoFrame {
    StereoFramePair pair;
    Eigen::Matrix4d baseFromFlange{Eigen::Matrix4d::Identity()};
    double robotInterpolationAlpha{0.0};
    double robotGapMs{0.0};
    double actualLinearSpeedMmS{0.0};
};

struct StereoPoseSynchronizerStatistics {
    std::uint64_t robotSamples{0};
    std::uint64_t pairsQueued{0};
    std::uint64_t pairsSynchronized{0};
    std::uint64_t pairsDroppedNoBracket{0};
    std::uint64_t pairsDroppedRobotGap{0};
};

// Associates paired exposures with FR5 pose samples. hostReceiveNs is used as
// the common monotonic domain; an explicit offset absorbs measured camera
// callback/transport delay. Pending pairs wait for the next robot packet so
// interpolation never silently extrapolates.
class StereoPoseSynchronizer {
public:
    StereoPoseSynchronizer();

    void configure(double cameraToRobotOffsetUs,
                   double maximumRobotGapMs,
                   double historySeconds = 5.0);
    void pushRobot(hik_sync::RobotSample sample);
    void pushPair(StereoFramePair pair);
    bool takeSynchronized(SynchronizedStereoFrame* frame,
                          std::string* reason = nullptr);
    void clear();
    StereoPoseSynchronizerStatistics statistics() const { return statistics_; }

private:
    void prune();
    double cameraToRobotOffsetUs_{0.0};
    double maximumRobotGapMs_{25.0};
    std::int64_t historyNs_{5000000000LL};
    std::deque<hik_sync::RobotSample> robots_;
    std::deque<StereoFramePair> pairs_;
    StereoPoseSynchronizerStatistics statistics_;
};

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_FRAME_SYNCHRONIZER_H
