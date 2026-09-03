#include "stereo/core/StereoFrameSynchronizer.h"

#include <algorithm>
#include <cmath>

namespace hik_stereo {

StereoFramePairer::StereoFramePairer(double maximumSkewMs,
                                     std::size_t queueCapacity) {
    configure(maximumSkewMs, queueCapacity);
}

void StereoFramePairer::configure(double maximumSkewMs,
                                  std::size_t queueCapacity) {
    maximumSkewMs_ = std::max(0.1, std::min(100.0, maximumSkewMs));
    queueCapacity_ = std::max<std::size_t>(2U,
        std::min<std::size_t>(128U, queueCapacity));
    clear();
}

void StereoFramePairer::pushLeft(hik_sync::CameraFrame frame) {
    ++statistics_.leftFrames;
    left_.push_back(std::move(frame));
    while (left_.size() > queueCapacity_) {
        left_.pop_front();
        ++statistics_.leftDropped;
    }
    pairAvailable();
}

void StereoFramePairer::pushRight(hik_sync::CameraFrame frame) {
    ++statistics_.rightFrames;
    right_.push_back(std::move(frame));
    while (right_.size() > queueCapacity_) {
        right_.pop_front();
        ++statistics_.rightDropped;
    }
    pairAvailable();
}

void StereoFramePairer::pairAvailable() {
    const std::int64_t maximumSkewNs = static_cast<std::int64_t>(
        std::llround(maximumSkewMs_ * 1000000.0));
    while (!left_.empty() && !right_.empty()) {
        const std::int64_t leftTime = left_.front().hostCallbackNs;
        const std::int64_t rightTime = right_.front().hostCallbackNs;
        if (leftTime <= 0 || rightTime <= 0) {
            if (leftTime <= rightTime) {
                left_.pop_front();
                ++statistics_.leftDropped;
            } else {
                right_.pop_front();
                ++statistics_.rightDropped;
            }
            continue;
        }
        const std::int64_t difference = leftTime - rightTime;
        if (std::llabs(difference) <= maximumSkewNs) {
            StereoFramePair pair;
            pair.left = std::move(left_.front());
            pair.right = std::move(right_.front());
            left_.pop_front();
            right_.pop_front();
            pair.midpointHostNs = leftTime / 2 + rightTime / 2 +
                (leftTime % 2 + rightTime % 2) / 2;
            pair.skewMs = std::abs(static_cast<double>(difference)) / 1.0e6;
            pairs_.push_back(std::move(pair));
            while (pairs_.size() > queueCapacity_) pairs_.pop_front();
            ++statistics_.pairedFrames;
        } else if (difference < 0) {
            left_.pop_front();
            ++statistics_.leftDropped;
        } else {
            right_.pop_front();
            ++statistics_.rightDropped;
        }
    }
}

bool StereoFramePairer::takePair(StereoFramePair* pair) {
    if (!pair || pairs_.empty()) return false;
    *pair = std::move(pairs_.front());
    pairs_.pop_front();
    return true;
}

void StereoFramePairer::clear() {
    left_.clear();
    right_.clear();
    pairs_.clear();
    statistics_ = StereoFramePairerStatistics();
}

StereoPoseSynchronizer::StereoPoseSynchronizer() = default;

void StereoPoseSynchronizer::configure(double cameraToRobotOffsetUs,
                                       double maximumRobotGapMs,
                                       double historySeconds) {
    cameraToRobotOffsetUs_ = std::max(
        -100000.0, std::min(100000.0, cameraToRobotOffsetUs));
    maximumRobotGapMs_ = std::max(
        1.0, std::min(1000.0, maximumRobotGapMs));
    historyNs_ = static_cast<std::int64_t>(std::llround(
        std::max(1.0, std::min(60.0, historySeconds)) * 1.0e9));
    clear();
}

void StereoPoseSynchronizer::pushRobot(hik_sync::RobotSample sample) {
    ++statistics_.robotSamples;
    if (!sample.valid || sample.hostReceiveNs <= 0) return;
    sample.alignedTimestampNs = sample.hostReceiveNs;
    if (!robots_.empty() &&
        sample.alignedTimestampNs <= robots_.back().alignedTimestampNs) {
        return;
    }
    robots_.push_back(std::move(sample));
    prune();
}

void StereoPoseSynchronizer::pushPair(StereoFramePair pair) {
    ++statistics_.pairsQueued;
    const std::int64_t exposureHalfNs = static_cast<std::int64_t>(
        std::llround(std::max(pair.left.exposureUs,
                             pair.right.exposureUs) * 500.0));
    pair.midpointHostNs -= exposureHalfNs;
    pair.midpointHostNs += static_cast<std::int64_t>(
        std::llround(cameraToRobotOffsetUs_ * 1000.0));
    pairs_.push_back(std::move(pair));
    prune();
}

bool StereoPoseSynchronizer::takeSynchronized(
        SynchronizedStereoFrame* frame,
        std::string* reason) {
    if (!frame || pairs_.empty() || robots_.size() < 2U) return false;
    const std::int64_t target = pairs_.front().midpointHostNs;
    if (target > robots_.back().alignedTimestampNs) return false;
    if (target < robots_.front().alignedTimestampNs) {
        pairs_.pop_front();
        ++statistics_.pairsDroppedNoBracket;
        if (reason) *reason = "stereo exposure predates robot pose history";
        return false;
    }
    auto after = std::lower_bound(
        robots_.begin(), robots_.end(), target,
        [](const hik_sync::RobotSample& sample, std::int64_t value) {
            return sample.alignedTimestampNs < value;
        });
    if (after == robots_.begin() || after == robots_.end()) return false;
    const auto before = std::prev(after);
    const double gapMs = static_cast<double>(
        after->alignedTimestampNs - before->alignedTimestampNs) / 1.0e6;
    if (gapMs <= 0.0 || gapMs > maximumRobotGapMs_) {
        pairs_.pop_front();
        ++statistics_.pairsDroppedRobotGap;
        if (reason) *reason = "robot bracket gap exceeds stereo mapping limit";
        return false;
    }
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double alpha = 0.0;
    if (!hik_sync::interpolateRobotPose(
            *before, *after, target, &position, &orientation, &alpha)) {
        pairs_.pop_front();
        ++statistics_.pairsDroppedNoBracket;
        if (reason) *reason = "robot pose interpolation failed";
        return false;
    }
    frame->pair = std::move(pairs_.front());
    pairs_.pop_front();
    frame->baseFromFlange = hik_sync::makeTransform(position, orientation);
    frame->robotInterpolationAlpha = alpha;
    frame->robotGapMs = gapMs;
    frame->actualLinearSpeedMmS =
        (1.0 - alpha) * before->actualLinearSpeedMmS +
        alpha * after->actualLinearSpeedMmS;
    ++statistics_.pairsSynchronized;
    return true;
}

void StereoPoseSynchronizer::prune() {
    if (!robots_.empty()) {
        const std::int64_t cutoff = robots_.back().alignedTimestampNs -
                                    historyNs_;
        while (robots_.size() > 2U &&
               robots_[1].alignedTimestampNs < cutoff) {
            robots_.pop_front();
        }
        while (!pairs_.empty() && pairs_.front().midpointHostNs < cutoff) {
            pairs_.pop_front();
            ++statistics_.pairsDroppedNoBracket;
        }
    }
    while (pairs_.size() > 32U) {
        pairs_.pop_front();
        ++statistics_.pairsDroppedNoBracket;
    }
}

void StereoPoseSynchronizer::clear() {
    robots_.clear();
    pairs_.clear();
    statistics_ = StereoPoseSynchronizerStatistics();
}

}  // namespace hik_stereo
