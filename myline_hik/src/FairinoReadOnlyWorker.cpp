#include "FairinoReadOnlyWorker.h"

#include <QTimer>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef HAVE_FAIRINO_SDK
#include "robot.h"
#endif

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <sstream>

namespace {

constexpr qint64 kStopConfirmationTimeoutMs = 5000;
constexpr qint64 kRealtimeFreshnessLimitNs = 500000000LL;
constexpr int kRequiredStationaryStopSamples = 3;
constexpr double kStoppedLinearSpeedMmS = 0.10;
constexpr double kStoppedAngularSpeedDegS = 0.10;
constexpr double kPi = 3.14159265358979323846;

bool finitePose(const hik_scan::Pose6D& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.z) && std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) && std::isfinite(pose.rz);
}

double wrappedDeltaDeg(double from, double to) {
    double delta = to - from;
    while (delta > 180.0) delta -= 360.0;
    while (delta < -180.0) delta += 360.0;
    return delta;
}

double poseDistanceMm(const hik_scan::Pose6D& first,
                      const hik_scan::Pose6D& second) {
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    const double dz = second.z - first.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double poseRotationDeltaDeg(const hik_scan::Pose6D& first,
                            const hik_scan::Pose6D& second) {
    const double dx = wrappedDeltaDeg(first.rx, second.rx);
    const double dy = wrappedDeltaDeg(first.ry, second.ry);
    const double dz = wrappedDeltaDeg(first.rz, second.rz);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool appendPoseSamples(
        const hik_scan::Pose6D& first,
        const hik_scan::Pose6D& second,
        const hik_fr5::PathEvaluationOptions& options,
        std::vector<hik_scan::Pose6D>* samples) {
    if (!samples || !finitePose(first) || !finitePose(second)) {
        return false;
    }
    const double translation = poseDistanceMm(first, second);
    const double rotation = poseRotationDeltaDeg(first, second);
    const std::size_t translationIntervals =
        static_cast<std::size_t>(std::ceil(
            translation / options.maximumCartesianSampleStepMm));
    const std::size_t rotationIntervals =
        static_cast<std::size_t>(std::ceil(
            rotation / options.maximumAngularSampleStepDeg));
    const std::size_t intervals =
        std::max<std::size_t>(
            1U, std::max(translationIntervals, rotationIntervals));
    if (samples->size() + intervals >
        options.maximumPathSamples) {
        return false;
    }
    for (std::size_t index = 1U; index <= intervals; ++index) {
        const double ratio =
            static_cast<double>(index) /
            static_cast<double>(intervals);
        hik_scan::Pose6D pose = first;
        pose.x += ratio * (second.x - first.x);
        pose.y += ratio * (second.y - first.y);
        pose.z += ratio * (second.z - first.z);
        pose.rx += ratio * wrappedDeltaDeg(first.rx, second.rx);
        pose.ry += ratio * wrappedDeltaDeg(first.ry, second.ry);
        pose.rz += ratio * wrappedDeltaDeg(first.rz, second.rz);
        samples->push_back(pose);
    }
    return true;
}

double estimateMoveTime(double lengthMm,
                        double speedMmS,
                        double accelerationMmS2) {
    if (!std::isfinite(lengthMm) || !std::isfinite(speedMmS) ||
        !std::isfinite(accelerationMmS2) || lengthMm < 0.0 ||
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

Eigen::Matrix3d fixedAxisRpy(double rxDeg,
                             double ryDeg,
                             double rzDeg) {
    const double scale = kPi / 180.0;
    return (Eigen::AngleAxisd(rzDeg * scale,
                             Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(ryDeg * scale,
                             Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rxDeg * scale,
                             Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

}  // namespace

#ifdef HAVE_FAIRINO_SDK
class FairinoRealtimeReceiverState {
public:
    struct Packet {
        ROBOT_STATE_PKG state{};
        int64_t hostReceiveNs{0};
        uint64_t receiveSequence{0};
    };

    static constexpr std::size_t kCapacity = 1024U;

    bool push(const ROBOT_STATE_PKG* state,
              int64_t hostReceiveNs,
              uint64_t receiveSequence) noexcept {
        if (!state) return false;
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1U) % kCapacity;
        if (next == tail_.load(std::memory_order_acquire)) {
            droppedPackets.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }
        slots_[head].state = *state;
        slots_[head].hostReceiveNs = hostReceiveNs;
        slots_[head].receiveSequence = receiveSequence;
        head_.store(next, std::memory_order_release);
        condition.notify_one();
        return true;
    }

    bool pop(Packet* packet) noexcept {
        if (!packet) return false;
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        *packet = slots_[tail];
        tail_.store((tail + 1U) % kCapacity, std::memory_order_release);
        return true;
    }

    bool empty() const noexcept {
        return tail_.load(std::memory_order_acquire) ==
               head_.load(std::memory_order_acquire);
    }

    void reset() {
        head_.store(0U, std::memory_order_relaxed);
        tail_.store(0U, std::memory_order_relaxed);
        droppedPackets.store(0U, std::memory_order_relaxed);
    }

    std::mutex waitMutex;
    std::condition_variable condition;
    std::atomic<uint64_t> droppedPackets{0U};

private:
    std::array<Packet, kCapacity> slots_{};
    std::atomic<std::size_t> head_{0U};
    std::atomic<std::size_t> tail_{0U};
};

namespace {

void fairinoRealtimeStateCallback(const ROBOT_STATE_PKG* state,
                                  int64_t hostReceiveNs,
                                  uint64_t receiveSequence,
                                  void* userData) {
    auto* receiver = static_cast<FairinoRealtimeReceiverState*>(userData);
    if (receiver) {
        (void)receiver->push(state, hostReceiveNs, receiveSequence);
    }
}

}  // namespace
#else
class FairinoRealtimeReceiverState {};
#endif

FairinoReadOnlyWorker::FairinoReadOnlyWorker(QObject* parent)
    : QObject(parent) {
#ifdef HAVE_FAIRINO_SDK
    robot_ = new FRRobot;
    realtimeReceiver_ = std::make_unique<FairinoRealtimeReceiverState>();
#endif
    motionPollTimer_ = new QTimer(this);
    motionPollTimer_->setInterval(100);
    connect(motionPollTimer_, &QTimer::timeout,
            this, &FairinoReadOnlyWorker::pollMotionDone);
}

FairinoReadOnlyWorker::~FairinoReadOnlyWorker() {
#ifdef HAVE_FAIRINO_SDK
    stopRealtimeReceiving();
    if (connected_ && robot_ && !motionStopUnconfirmed_) {
        (void)robot_->CloseRPC();
    }
    // If mechanical stop was not confirmed, deliberately leave the RPC
    // object untouched until process death instead of calling CloseRPC
    // before the safety condition has been observed.
    // Fairino 3.9.4 owns detached SDK threads and exposes no join operation.
    // Releasing this one object at process teardown avoids a shutdown-only
    // use-after-free; the operating system reclaims it immediately afterward.
    robot_ = nullptr;
#endif
}

void FairinoReadOnlyWorker::beginStopConfirmation(
        const QString& context) {
    if (!motionActive_) return;
    stopConfirmationPending_ = true;
    stopConfirmationStartMs_ =
        hik_sync::getMonotonicRawNs() / 1000000LL;
    {
        const std::lock_guard<std::mutex> lock(
            realtimeStopSnapshotMutex_);
        lastStopSampleReceiveNs_ = latestRealtimeHostReceiveNs_;
    }
    stationaryStopSamples_ = 0;
    if (!motionPollTimer_->isActive()) motionPollTimer_->start();
    emit log(QStringLiteral(
        "%1；等待至少 %2 个新鲜 FR5 20004 状态包同时确认 "
        "motion_done=1、TCP线速度≤%3 mm/s、角速度≤%4 deg/s。")
        .arg(context)
        .arg(kRequiredStationaryStopSamples)
        .arg(kStoppedLinearSpeedMmS, 0, 'f', 2)
        .arg(kStoppedAngularSpeedDegS, 0, 'f', 2));
}

bool FairinoReadOnlyWorker::confirmStoppedBlocking(int timeoutMs) {
#ifndef HAVE_FAIRINO_SDK
    (void)timeoutMs;
    return false;
#else
    qint64 lastSampleReceiveNs = 0;
    {
        const std::lock_guard<std::mutex> lock(
            realtimeStopSnapshotMutex_);
        lastSampleReceiveNs = latestRealtimeHostReceiveNs_;
    }
    int consecutiveStationarySamples = 0;
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(std::max(1, timeoutMs));
    while (std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        qint64 sampleReceiveNs = 0;
        int sampleMotionDone = 0;
        double sampleLinearSpeed = 0.0;
        double sampleAngularSpeed = 0.0;
        {
            const std::lock_guard<std::mutex> lock(
                realtimeStopSnapshotMutex_);
            sampleReceiveNs = latestRealtimeHostReceiveNs_;
            sampleMotionDone = latestRealtimeMotionDone_;
            sampleLinearSpeed = latestRealtimeLinearSpeedMmS_;
            sampleAngularSpeed = latestRealtimeAngularSpeedDegS_;
        }
        if (sampleReceiveNs <= lastSampleReceiveNs) continue;
        lastSampleReceiveNs = sampleReceiveNs;
        const qint64 nowNs = hik_sync::getMonotonicRawNs();
        const bool fresh =
            sampleReceiveNs > 0 && nowNs >= sampleReceiveNs &&
            nowNs - sampleReceiveNs <= kRealtimeFreshnessLimitNs;
        const double linearSpeed = std::abs(sampleLinearSpeed);
        const double angularSpeed = std::abs(sampleAngularSpeed);
        if (fresh && sampleMotionDone != 0 &&
            std::isfinite(linearSpeed) &&
            std::isfinite(angularSpeed) &&
            linearSpeed <= kStoppedLinearSpeedMmS &&
            angularSpeed <= kStoppedAngularSpeedDegS) {
            ++consecutiveStationarySamples;
            if (consecutiveStationarySamples >=
                kRequiredStationaryStopSamples) {
                return true;
            }
        } else {
            consecutiveStationarySamples = 0;
        }
    }
    return false;
#endif
}

void FairinoReadOnlyWorker::finishMotion(
        bool targetReached,
        bool motionStoppedConfirmed,
        const QString& description) {
    motionPollTimer_->stop();
    const int request = motionRequestId_;
    const qint64 finishedMs =
        hik_sync::getMonotonicRawNs() / 1000000LL;
    const qint64 elapsedMs =
        motionStartMs_ > 0 && finishedMs >= motionStartMs_
        ? finishedMs - motionStartMs_ : -1;
    motionActive_ = false;
    motionStopUnconfirmed_ = !motionStoppedConfirmed;
    stopConfirmationPending_ = false;
    motionRequestId_ = -1;
    motionStartMs_ = 0;
    motionTimeoutMs_ = 0;
    stopConfirmationStartMs_ = 0;
    lastStopSampleReceiveNs_ = 0;
    stationaryStopSamples_ = 0;
    if (request >= 0) {
        if (elapsedMs >= 0) {
            emit motionTimingMeasured(
                request, elapsedMs, targetReached);
        }
        emit motionFinished(request, targetReached,
                            motionStoppedConfirmed, description);
    }
}

void FairinoReadOnlyWorker::connectRobot(QString ipAddress) {
#ifndef HAVE_FAIRINO_SDK
    emit error(-1, QStringLiteral(
        "构建时未找到 Fairino SDK，FR5 连接不可用。请设置 FAIRINO_SDK_DIR 后重新运行脚本。"));
    emit connectionChanged(false, QStringLiteral("Fairino SDK 不可用"));
    return;
#else
    if (connected_) {
        emit connectionChanged(true, QStringLiteral("FR5 已连接"));
        return;
    }
    if (attempted_ || terminal_ || !robot_) {
        emit error(-1, QStringLiteral(
            "Fairino SDK 本进程已结束一次 RPC 会话，不能安全重连；请关闭并重新启动当前程序。"));
        return;
    }
    attempted_ = true;
    emit busyChanged(true);
    emit log(QStringLiteral("正在连接 FR5 %1 ...").arg(ipAddress));
    const QByteArray ip = ipAddress.toLatin1();
    const errno_t result = robot_->RPC(ip.constData());
    emit busyChanged(false);
    if (result != 0) {
        (void)robot_->CloseRPC();
        terminal_ = true;
        emit connectionChanged(false, QStringLiteral("FR5 连接失败"));
        emit error(-1, QStringLiteral(
            "FRRobot::RPC(%1) 失败，err=%2。请检查网络，并停止占用机器人的 ROS2/其他 SDK 进程后重启本工具。")
            .arg(ipAddress).arg(result));
        return;
    }
    connected_ = true;
    int actualPeriodMs = 0;
    errno_t periodResult = robot_->GetRobotRealtimeStateSamplePeriod(actualPeriodMs);
    if (periodResult != 0 || actualPeriodMs != 10) {
        periodResult = robot_->SetRobotRealtimeStateSamplePeriod(10);
        if (periodResult == 0) {
            periodResult = robot_->GetRobotRealtimeStateSamplePeriod(actualPeriodMs);
        }
    }
    if (periodResult != 0 || actualPeriodMs != 10) {
        (void)robot_->CloseRPC();
        connected_ = false;
        terminal_ = true;
        emit connectionChanged(false, QStringLiteral("FR5 实时反馈周期配置失败"));
        emit error(-1, QStringLiteral(
            "FR5 20004 实时状态周期必须为 10 ms；Set/GetRobotRealtimeStateSamplePeriod 失败，err=%1，读回=%2 ms。")
            .arg(periodResult).arg(actualPeriodMs));
        return;
    }
    if (!startRealtimeReceiving()) {
        (void)robot_->CloseRPC();
        connected_ = false;
        terminal_ = true;
        emit connectionChanged(false, QStringLiteral("FR5 20004逐包接收启动失败"));
        return;
    }
    emit realtimePeriodConfigured(actualPeriodMs);
    emit connectionChanged(true, QStringLiteral("FR5 连接成功：%1").arg(ipAddress));
    emit log(QStringLiteral(
        "FR5 SDK 会话已建立；20004 实时状态周期已读取/配置为 %1 ms；"
        "已启用SDK接收线程 CLOCK_MONOTONIC_RAW 逐包时间戳和SPSC消费线程；"
        "程序不会自动使能或切换控制器模式。")
        .arg(actualPeriodMs));
#endif
}

void FairinoReadOnlyWorker::disconnectRobot() {
#ifdef HAVE_FAIRINO_SDK
    if ((motionActive_ || motionStopUnconfirmed_) && robot_) {
        const errno_t stopResult = robot_->StopMotion();
        const bool stoppedConfirmed =
            stopResult == 0 &&
            confirmStoppedBlocking(
                static_cast<int>(kStopConfirmationTimeoutMs));
        const QString stopDescription =
            stopResult != 0
                ? QStringLiteral(
                      "退出前 StopMotion 失败，err=%1；"
                      "未关闭 RPC，请使用物理急停")
                      .arg(stopResult)
                : stoppedConfirmed
                    ? QStringLiteral(
                          "退出前 StopMotion 已由新鲜 20004 状态"
                          "连续确认机械停止")
                    : QStringLiteral(
                          "退出前 StopMotion 已提交，但 %1 ms 内未由"
                          "新鲜 20004 状态确认机械停止；未关闭 RPC，"
                          "请使用物理急停")
                          .arg(kStopConfirmationTimeoutMs);
        if (motionActive_) {
            finishMotion(false, stoppedConfirmed,
                         stopDescription);
        } else {
            motionStopUnconfirmed_ = !stoppedConfirmed;
            emit log(stopDescription);
        }
        if (!stoppedConfirmed) {
            motionPollTimer_->stop();
            stopRealtimeReceiving();
            connected_ = false;
            terminal_ = true;
            emit connectionChanged(
                false,
                QStringLiteral(
                    "FR5 停止未确认；为避免失去软件停止通道，"
                    "未调用 CloseRPC，进程将退出"));
            emit error(-1, stopDescription);
            return;
        }
    }
    stopRealtimeReceiving();
    if (connected_ && robot_) {
        emit busyChanged(true);
        const errno_t result = robot_->CloseRPC();
        emit busyChanged(false);
        connected_ = false;
        terminal_ = true;
        emit connectionChanged(false, result == 0
            ? QStringLiteral("FR5 已断开；再次连接需重启工具")
            : QStringLiteral("FR5 已断开，CloseRPC err=%1；再次连接需重启工具").arg(result));
    }
#else
    emit connectionChanged(false, QStringLiteral("Fairino SDK 不可用"));
#endif
}

void FairinoReadOnlyWorker::readFlangePose(int requestId) {
#ifndef HAVE_FAIRINO_SDK
    emit error(requestId, QStringLiteral("Fairino SDK 不可用。"));
#else
    if (!connected_ || !robot_) {
        emit error(requestId, QStringLiteral("FR5 尚未连接。"));
        return;
    }
    emit busyChanged(true);
    DescPose pose;
    const errno_t result = robot_->GetActualToolFlangePose(0, &pose);
    emit busyChanged(false);
    if (result != 0) {
        emit error(requestId, QStringLiteral(
            "GetActualToolFlangePose 失败，err=%1。为避免 SDK 重连竞态，请重启工具后再试。")
            .arg(result));
        return;
    }
    const double values[] = {
        pose.tran.x, pose.tran.y, pose.tran.z,
        pose.rpy.rx, pose.rpy.ry, pose.rpy.rz
    };
    for (int index = 0; index < 6; ++index) {
        if (!std::isfinite(values[index])) {
            emit error(requestId, QStringLiteral("FR5 返回了非有限法兰位姿，样本已拒绝。"));
            return;
        }
    }
    emit flangePoseReady(requestId,
                         values[0], values[1], values[2],
                         values[3], values[4], values[5],
                         hik_sync::getMonotonicRawNs() / 1000000LL);
#endif
}

void FairinoReadOnlyWorker::evaluateKinematicPaths(
        int requestId,
        std::vector<hik_fr5::PathEvaluationRequest> requests,
        hik_fr5::PathEvaluationOptions options) {
#ifndef HAVE_FAIRINO_SDK
    (void)requests;
    (void)options;
    emit kinematicPathBatchFinished(
        requestId, false,
        QStringLiteral("构建时未找到 Fairino SDK，路径评估不可用。"));
    return;
#else
    const bool optionsValid =
        std::isfinite(options.maximumCartesianSampleStepMm) &&
        options.maximumCartesianSampleStepMm >= 1.0 &&
        std::isfinite(options.maximumAngularSampleStepDeg) &&
        options.maximumAngularSampleStepDeg >= 0.5 &&
        std::isfinite(options.minimumJointLimitMarginDeg) &&
        options.minimumJointLimitMarginDeg >= 0.0 &&
        std::isfinite(options.minimumNormalizedSingularValue) &&
        options.minimumNormalizedSingularValue > 0.0 &&
        std::isfinite(options.jacobianCharacteristicLengthMm) &&
        options.jacobianCharacteristicLengthMm > 0.0 &&
        options.maximumJacobianSamples >= 2U &&
        options.maximumJacobianSamples <= 128U &&
        options.maximumPathSamples >= 4U &&
        options.maximumPathSamples <= 4096U;
    if (!optionsValid || requests.empty() ||
        requests.size() > 256U) {
        emit kinematicPathBatchFinished(
            requestId, false,
            QStringLiteral("FR5 路径评估参数无效或候选数超过 256。"));
        return;
    }
    if (!connected_ || !robot_) {
        emit kinematicPathBatchFinished(
            requestId, false,
            QStringLiteral("FR5 尚未连接，IK/奇异性为 UNKNOWN。"));
        return;
    }
    if (motionActive_ || motionStopUnconfirmed_) {
        emit kinematicPathBatchFinished(
            requestId, false,
            QStringLiteral("FR5 正在运动或机械停止未确认，拒绝路径评估。"));
        return;
    }

    JointPos actualJoint;
    float negativeLimit[6] = {};
    float positiveLimit[6] = {};
    errno_t setupResult =
        robot_->GetActualJointPosDegree(0, &actualJoint);
    if (setupResult == 0) {
        setupResult = robot_->GetJointSoftLimitDeg(
            0, negativeLimit, positiveLimit);
    }
    if (setupResult != 0) {
        emit kinematicPathBatchFinished(
            requestId, false,
            QStringLiteral(
                "FR5 当前关节/软限位读取失败，err=%1；"
                "所有候选保持 UNKNOWN。")
                .arg(setupResult));
        return;
    }

    emit busyChanged(true);
    std::size_t completedCount = 0U;
    JointPos chainedReference = actualJoint;
    hik_scan::Pose6D chainedEndPose;
    bool chainedReferenceValid = false;
    for (const hik_fr5::PathEvaluationRequest& request :
         requests) {
        hik_adaptive::RobotPathEvaluation evaluation;
        evaluation.collision =
            hik_adaptive::VerificationState::Unknown;
        evaluation.detail =
            "geometric collision model is unavailable";
        const hik_adaptive::ScanSegment& segment =
            request.segment;
        evaluation.evaluatedPrimitive = segment.primitive;
        const hik_scan::Pose6D motionStart =
            finitePose(segment.motionStart)
                ? segment.motionStart : segment.start;
        const hik_scan::Pose6D motionEnd =
            finitePose(segment.motionEnd)
                ? segment.motionEnd : segment.end;
        if (request.chainWithPrevious &&
            (!chainedReferenceValid ||
             poseDistanceMm(
                 request.currentPose, chainedEndPose) > 1.0e-3 ||
             poseRotationDeltaDeg(
                 request.currentPose, chainedEndPose) > 1.0e-3)) {
            evaluation.ik =
                hik_adaptive::VerificationState::Failed;
            evaluation.singularity =
                hik_adaptive::VerificationState::Unknown;
            evaluation.detail =
                "cross-segment IK chain is unavailable or discontinuous; "
                "collision remains UNKNOWN";
            chainedReferenceValid = false;
            emit kinematicPathEvaluated(
                requestId, request.actionId, evaluation);
            ++completedCount;
            continue;
        }
        std::vector<hik_scan::Pose6D> poses;
        poses.reserve(std::min<std::size_t>(
            options.maximumPathSamples, 128U));
        poses.push_back(request.currentPose);
        std::vector<hik_scan::Pose6D> segmentPoses;
        std::string segmentSampleError;
        bool pathValid =
            finitePose(request.currentPose) &&
            finitePose(segment.start) &&
            finitePose(segment.end) &&
            finitePose(motionStart) &&
            finitePose(motionEnd) &&
            std::isfinite(segment.speedMmS) &&
            segment.speedMmS > 0.0 &&
            std::isfinite(segment.accelerationMmS2) &&
            segment.accelerationMmS2 > 0.0 &&
            appendPoseSamples(
                request.currentPose, motionStart,
                options, &poses);
        const std::size_t remainingSamples =
            poses.size() < options.maximumPathSamples
            ? options.maximumPathSamples - poses.size() + 1U
            : 0U;
        pathValid = pathValid && remainingSamples >= 2U &&
            hik_adaptive::sampleSegmentMotion(
                segment,
                options.maximumCartesianSampleStepMm,
                options.maximumAngularSampleStepDeg,
                remainingSamples, &segmentPoses,
                &segmentSampleError);
        if (pathValid) {
            poses.insert(poses.end(),
                         segmentPoses.begin() + 1,
                         segmentPoses.end());
        }
        if (!pathValid) {
            evaluation.ik =
                hik_adaptive::VerificationState::Failed;
            evaluation.singularity =
                hik_adaptive::VerificationState::Unknown;
            evaluation.detail =
                "candidate path is invalid or exceeds sample limit: " +
                segmentSampleError +
                "; collision remains UNKNOWN";
            chainedReferenceValid = false;
            emit kinematicPathEvaluated(
                requestId, request.actionId, evaluation);
            ++completedCount;
            continue;
        }

        JointPos reference = request.chainWithPrevious
            ? chainedReference : actualJoint;
        std::vector<JointPos> jointSolutions;
        jointSolutions.reserve(poses.size());
        evaluation.jointSamplesDeg.reserve(poses.size());
        evaluation.minimumJointLimitMarginDeg =
            std::numeric_limits<double>::infinity();
        bool ikPassed = true;
        bool jointMarginPassed = true;
        errno_t ikError = 0;
        for (std::size_t poseIndex = 0U;
             poseIndex < poses.size(); ++poseIndex) {
            const hik_scan::Pose6D& pose = poses[poseIndex];
            DescPose target(
                pose.x, pose.y, pose.z,
                pose.rx, pose.ry, pose.rz);
            JointPos solution;
            ikError = robot_->GetInverseKinRef(
                0, &target, &reference, &solution);
            if (ikError != 0) {
                ikPassed = false;
                break;
            }
            std::array<double, 6> sample{};
            for (int joint = 0; joint < 6; ++joint) {
                const double value = solution.jPos[joint];
                if (!std::isfinite(value)) {
                    ikPassed = false;
                    break;
                }
                sample[static_cast<std::size_t>(joint)] = value;
                const double margin = std::min(
                    value - negativeLimit[joint],
                    positiveLimit[joint] - value);
                evaluation.minimumJointLimitMarginDeg =
                    std::min(
                        evaluation.minimumJointLimitMarginDeg,
                        margin);
                if (!std::isfinite(margin) ||
                    margin <
                        options.minimumJointLimitMarginDeg) {
                    jointMarginPassed = false;
                }
            }
            if (!ikPassed) break;
            if (!jointSolutions.empty()) {
                for (int joint = 0; joint < 6; ++joint) {
                    evaluation.jointTravelDeg += std::abs(
                        solution.jPos[joint] -
                        jointSolutions.back().jPos[joint]);
                }
            }
            evaluation.jointSamplesDeg.push_back(sample);
            jointSolutions.push_back(solution);
            reference = solution;
        }
        if (!ikPassed || jointSolutions.size() != poses.size()) {
            evaluation.ik =
                hik_adaptive::VerificationState::Failed;
            evaluation.singularity =
                hik_adaptive::VerificationState::Unknown;
            evaluation.detail =
                QStringLiteral(
                    "FR5 sampled path IK failed at %1/%2, err=%3; "
                    "collision remains UNKNOWN")
                    .arg(jointSolutions.size())
                    .arg(poses.size())
                    .arg(ikError)
                    .toStdString();
            chainedReferenceValid = false;
            emit kinematicPathEvaluated(
                requestId, request.actionId, evaluation);
            ++completedCount;
            continue;
        }
        chainedReference = jointSolutions.back();
        chainedEndPose = motionEnd;
        chainedReferenceValid = true;
        evaluation.ik = jointMarginPassed
            ? hik_adaptive::VerificationState::Passed
            : hik_adaptive::VerificationState::Failed;

        const std::size_t jacobianCount = std::min(
            options.maximumJacobianSamples,
            jointSolutions.size());
        evaluation.minimumSingularValue =
            std::numeric_limits<double>::infinity();
        bool jacobianValid = true;
        errno_t fkError = 0;
        for (std::size_t sampleIndex = 0U;
             sampleIndex < jacobianCount; ++sampleIndex) {
            const std::size_t jointIndex =
                jacobianCount == 1U ? 0U :
                sampleIndex * (jointSolutions.size() - 1U) /
                (jacobianCount - 1U);
            JointPos baselineJoint = jointSolutions[jointIndex];
            DescPose baselinePose;
            fkError = robot_->GetForwardKin(
                &baselineJoint, &baselinePose);
            if (fkError != 0) {
                jacobianValid = false;
                break;
            }
            const Eigen::Vector3d baselineTranslation(
                baselinePose.tran.x,
                baselinePose.tran.y,
                baselinePose.tran.z);
            const Eigen::Matrix3d baselineRotation =
                fixedAxisRpy(
                    baselinePose.rpy.rx,
                    baselinePose.rpy.ry,
                    baselinePose.rpy.rz);
            Eigen::Matrix<double, 6, 6> jacobian;
            jacobian.setZero();
            const double epsilonRad = 1.0e-4;
            const double epsilonDeg =
                epsilonRad * 180.0 / kPi;
            for (int joint = 0; joint < 6; ++joint) {
                JointPos perturbed = baselineJoint;
                double signedEpsilonDeg = epsilonDeg;
                if (perturbed.jPos[joint] + epsilonDeg >=
                    positiveLimit[joint]) {
                    signedEpsilonDeg = -epsilonDeg;
                }
                perturbed.jPos[joint] += signedEpsilonDeg;
                DescPose perturbedPose;
                fkError = robot_->GetForwardKin(
                    &perturbed, &perturbedPose);
                if (fkError != 0) {
                    jacobianValid = false;
                    break;
                }
                const double signedEpsilonRad =
                    signedEpsilonDeg * kPi / 180.0;
                const Eigen::Vector3d translation(
                    perturbedPose.tran.x,
                    perturbedPose.tran.y,
                    perturbedPose.tran.z);
                jacobian.block<3, 1>(0, joint) =
                    (translation - baselineTranslation) /
                    signedEpsilonRad /
                    options.jacobianCharacteristicLengthMm;
                const Eigen::Matrix3d rotation =
                    fixedAxisRpy(
                        perturbedPose.rpy.rx,
                        perturbedPose.rpy.ry,
                        perturbedPose.rpy.rz);
                const Eigen::Matrix3d delta =
                    baselineRotation.transpose() * rotation;
                const Eigen::AngleAxisd angleAxis(delta);
                jacobian.block<3, 1>(3, joint) =
                    angleAxis.axis() * angleAxis.angle() /
                    signedEpsilonRad;
            }
            if (!jacobianValid) break;
            const Eigen::JacobiSVD<
                Eigen::Matrix<double, 6, 6>> svd(
                    jacobian,
                    Eigen::ComputeFullU |
                    Eigen::ComputeFullV);
            const Eigen::Matrix<double, 6, 1> singular =
                svd.singularValues();
            const double minimum = singular.minCoeff();
            if (!std::isfinite(minimum)) {
                jacobianValid = false;
                break;
            }
            evaluation.minimumSingularValue =
                std::min(
                    evaluation.minimumSingularValue,
                    minimum);
        }
        if (!jacobianValid) {
            evaluation.singularity =
                hik_adaptive::VerificationState::Unknown;
        } else {
            evaluation.singularity =
                evaluation.minimumSingularValue >=
                    options.minimumNormalizedSingularValue
                ? hik_adaptive::VerificationState::Passed
                : hik_adaptive::VerificationState::Failed;
        }

        const double physicalLength =
            poseDistanceMm(request.currentPose, motionStart) +
            hik_adaptive::segmentMotionLengthMm(segment);
        evaluation.estimatedExecutionTimeS =
            estimateMoveTime(
                physicalLength,
                segment.speedMmS,
                segment.accelerationMmS2) + 0.20;
        std::ostringstream detail;
        detail << "sampled_poses=" << poses.size()
               << ", primitive="
               << hik_adaptive::motionPrimitiveName(
                      segment.primitive)
               << ", jacobian_samples=" << jacobianCount
               << ", min_joint_margin_deg="
               << evaluation.minimumJointLimitMarginDeg
               << ", min_normalized_sigma="
               << evaluation.minimumSingularValue
               << ", fk_error=" << fkError
               << "; collision=UNKNOWN (no validated geometric "
                  "workcell model)";
        evaluation.detail = detail.str();
        emit kinematicPathEvaluated(
            requestId, request.actionId, evaluation);
        ++completedCount;
    }
    emit busyChanged(false);
    emit kinematicPathBatchFinished(
        requestId, completedCount == requests.size(),
        QStringLiteral(
            "FR5 路径评估完成 %1/%2；IK/关节余量/数值Jacobian"
            "来自控制器模型，几何碰撞仍为 UNKNOWN。")
            .arg(completedCount)
            .arg(requests.size()));
#endif
}

void FairinoReadOnlyWorker::moveLinear(int requestId,
                                       double xMm,
                                       double yMm,
                                       double zMm,
                                       double rxDeg,
                                       double ryDeg,
                                       double rzDeg,
                                       double velocityPercent,
                                       double accelerationPercent,
                                       int timeoutMs) {
    moveLinearImpl(requestId, xMm, yMm, zMm, rxDeg, ryDeg, rzDeg,
                   velocityPercent, accelerationPercent, timeoutMs, false);
}

void FairinoReadOnlyWorker::moveLinearPhysical(
        int requestId, double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg,
        double speedMmS, double accelerationMmS2, int timeoutMs) {
    moveLinearImpl(requestId, xMm, yMm, zMm, rxDeg, ryDeg, rzDeg,
                   speedMmS, accelerationMmS2, timeoutMs, true);
}

void FairinoReadOnlyWorker::executeAdaptiveTrajectory(
        int requestId,
        std::vector<hik_adaptive::ScanSegment> segments,
        int timeoutMs) {
#ifndef HAVE_FAIRINO_SDK
    (void)segments;
    (void)timeoutMs;
    emit motionFinished(requestId, false, true,
                        QStringLiteral("Fairino SDK 不可用。"));
#else
    if (!connected_ || !robot_) {
        emit motionFinished(requestId, false, false,
                            QStringLiteral("FR5 尚未连接。"));
        return;
    }
    if (motionActive_ || motionStopUnconfirmed_) {
        emit motionFinished(
            requestId, false, false,
            motionActive_
                ? QStringLiteral("已有 FR5 运动正在执行。")
                : QStringLiteral(
                      "上一运动尚未确认机械停止；拒绝预提交轨迹。"));
        return;
    }
    if (segments.size() < 3U || segments.size() > 199U ||
        timeoutMs < 1000 || timeoutMs > 300000) {
        emit motionFinished(
            requestId, false, true,
            QStringLiteral("预提交轨迹段数或超时参数无效。"));
        return;
    }
    for (std::size_t index = 0U;
         index < segments.size(); ++index) {
        const hik_adaptive::ScanSegment& segment = segments[index];
        const bool expectedMeasurement = (index % 2U) == 0U;
        const bool primitiveValid = expectedMeasurement
            ? segment.kind ==
                  hik_adaptive::SegmentKind::Measurement &&
              segment.primitive ==
                  hik_adaptive::MotionPrimitive::Line
            : segment.kind ==
                  hik_adaptive::SegmentKind::Transition &&
              segment.primitive ==
                  hik_adaptive::MotionPrimitive::Arc &&
              finitePose(segment.arcVia);
        const bool blendValid =
            index + 1U == segments.size()
            ? segment.blendRadiusMm == 0.0
            : std::isfinite(segment.blendRadiusMm) &&
              segment.blendRadiusMm > 0.0 &&
              segment.blendRadiusMm <= 1000.0;
        if (!primitiveValid || !blendValid ||
            !finitePose(segment.motionEnd) ||
            !std::isfinite(segment.speedMmS) ||
            segment.speedMmS < 10.0 ||
            segment.speedMmS > 50.0 ||
            !std::isfinite(segment.accelerationMmS2) ||
            segment.accelerationMmS2 <= 0.0 ||
            segment.accelerationMmS2 > 1000.0) {
            emit motionFinished(
                requestId, false, true,
                QStringLiteral(
                    "预提交轨迹第 %1 段不是已验证的融合 "
                    "LINE/ARC，或速度/加速度/blendR 越界。")
                    .arg(index));
            return;
        }
    }

    struct PreparedCommand {
        hik_adaptive::ScanSegment segment;
        JointPos viaJoint;
        JointPos targetJoint;
    };
    JointPos reference;
    errno_t result =
        robot_->GetActualJointPosDegree(0, &reference);
    std::vector<PreparedCommand> prepared;
    prepared.reserve(segments.size());
    for (const hik_adaptive::ScanSegment& segment : segments) {
        if (result != 0) break;
        PreparedCommand command;
        command.segment = segment;
        if (segment.primitive ==
            hik_adaptive::MotionPrimitive::Arc) {
            DescPose via(
                segment.arcVia.x, segment.arcVia.y,
                segment.arcVia.z, segment.arcVia.rx,
                segment.arcVia.ry, segment.arcVia.rz);
            result = robot_->GetInverseKinRef(
                0, &via, &reference, &command.viaJoint);
            if (result == 0) reference = command.viaJoint;
        }
        if (result == 0) {
            const hik_scan::Pose6D& pose = segment.motionEnd;
            DescPose target(
                pose.x, pose.y, pose.z,
                pose.rx, pose.ry, pose.rz);
            result = robot_->GetInverseKinRef(
                0, &target, &reference, &command.targetJoint);
            if (result == 0) reference = command.targetJoint;
        }
        if (result == 0) prepared.push_back(command);
    }
    if (result != 0 || prepared.size() != segments.size()) {
        emit motionFinished(
            requestId, false, true,
            QStringLiteral(
                "预提交前端点/圆弧中间点链式逆解失败，err=%1；"
                "未发送任何运动。")
                .arg(result));
        return;
    }

    ExaxisPos externalAxis(0.0, 0.0, 0.0, 0.0);
    DescPose offset;
    bool firstSubmitted = false;
    for (std::size_t index = 0U;
         index < prepared.size(); ++index) {
        PreparedCommand& command = prepared[index];
        const hik_adaptive::ScanSegment& segment =
            command.segment;
        const float blendRadius =
            static_cast<float>(segment.blendRadiusMm);
        const hik_scan::Pose6D& end = segment.motionEnd;
        DescPose target(
            end.x, end.y, end.z,
            end.rx, end.ry, end.rz);
        if (segment.primitive ==
            hik_adaptive::MotionPrimitive::Arc) {
            const hik_scan::Pose6D& middle = segment.arcVia;
            DescPose via(
                middle.x, middle.y, middle.z,
                middle.rx, middle.ry, middle.rz);
            result = robot_->MoveC(
                &command.viaJoint, &via,
                0, 0, 100.0F, 100.0F,
                &externalAxis, 0, &offset,
                &command.targetJoint, &target,
                0, 0, 100.0F, 100.0F,
                &externalAxis, 0, &offset,
                static_cast<float>(segment.speedMmS),
                blendRadius,
                static_cast<float>(segment.accelerationMmS2), 1);
        } else {
            result = robot_->MoveL(
                &command.targetJoint, &target,
                0, 0, 100.0F, 100.0F,
                static_cast<float>(segment.speedMmS),
                blendRadius, 0, &externalAxis, 0, 0, &offset,
                static_cast<float>(segment.accelerationMmS2), 1);
        }
        if (result != 0) break;
        if (!firstSubmitted) {
            firstSubmitted = true;
            motionActive_ = true;
            motionStopUnconfirmed_ = true;
            stopConfirmationPending_ = false;
            motionRequestId_ = requestId;
            motionStartMs_ =
                hik_sync::getMonotonicRawNs() / 1000000LL;
            motionTimeoutMs_ = timeoutMs;
            stopConfirmationStartMs_ = 0;
            lastStopSampleReceiveNs_ = 0;
            stationaryStopSamples_ = 0;
            motionPollTimer_->start();
            emit motionStarted(
                requestId,
                QStringLiteral(
                    "FR5 融合轨迹开始预提交：%1 段，"
                    "物理速度模式，LINE/MoveC，非末段 blendR>0。")
                    .arg(segments.size()));
        }
    }
    if (result != 0) {
        if (!firstSubmitted) {
            emit motionFinished(
                requestId, false, true,
                QStringLiteral(
                    "首段轨迹提交失败，err=%1；未开始运动。")
                    .arg(result));
            return;
        }
        const errno_t stopResult = robot_->StopMotion();
        if (stopResult == 0) {
            beginStopConfirmation(QStringLiteral(
                "轨迹预提交中途失败，err=%1；StopMotion 已提交")
                .arg(result));
        } else {
            finishMotion(
                false, false,
                QStringLiteral(
                    "轨迹预提交中途失败，err=%1；StopMotion "
                    "失败，err=%2，请使用物理急停")
                    .arg(result)
                    .arg(stopResult));
        }
        return;
    }
    emit log(QStringLiteral(
        "FR5 控制器已接受全部 %1 段 LINE/MoveC；"
        "最终到位由20004状态与GetRobotMotionDone监控。")
        .arg(segments.size()));
#endif
}

void FairinoReadOnlyWorker::moveLinearImpl(
        int requestId, double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg,
        double speedValue, double accelerationValue,
        int timeoutMs, bool physicalMode) {
#ifndef HAVE_FAIRINO_SDK
    emit motionFinished(requestId, false, true,
                        QStringLiteral("Fairino SDK 不可用。"));
#else
    if (!connected_ || !robot_) {
        emit motionFinished(requestId, false, false,
                            QStringLiteral("FR5 尚未连接。"));
        return;
    }
    if (motionActive_ || motionStopUnconfirmed_) {
        emit motionFinished(requestId, false, false,
                            motionActive_
                                ? QStringLiteral("已有 MoveL 正在执行。")
                                : QStringLiteral(
                                      "上一运动尚未确认机械停止；"
                                      "拒绝新的 MoveL，请使用物理急停并重启程序。"));
        return;
    }
    const double values[] = {xMm, yMm, zMm, rxDeg, ryDeg, rzDeg,
                             speedValue, accelerationValue};
    for (int index = 0; index < 8; ++index) {
        if (!std::isfinite(values[index])) {
            emit motionFinished(requestId, false, true,
                                QStringLiteral("MoveL 参数包含非有限数。"));
            return;
        }
    }
    const bool motionParametersValid = physicalMode
        ? (speedValue >= 10.0 && speedValue <= 50.0 &&
           accelerationValue > 0.0 && accelerationValue <= 1000.0)
        : (speedValue > 0.0 && speedValue <= 20.0 &&
           accelerationValue > 0.0 && accelerationValue <= 50.0);
    if (!motionParametersValid ||
        timeoutMs < 1000 || timeoutMs > 300000) {
        emit motionFinished(requestId, false, true, physicalMode
            ? QStringLiteral(
                "连续扫描限制：物理速度必须在 [10,50] mm/s，加速度在 (0,1000] mm/s²，超时 1–300 s。")
            : QStringLiteral(
                "验证扫描限制：速度必须在 (0,20]% ，加速度在 (0,50]% ，超时 1–300 s。"));
        return;
    }

    DescPose target(xMm, yMm, zMm, rxDeg, ryDeg, rzDeg);
    JointPos currentJoint;
    JointPos targetJoint;
    ExaxisPos externalAxis(0.0, 0.0, 0.0, 0.0);
    DescPose offset;
    errno_t result = robot_->GetActualJointPosDegree(0, &currentJoint);
    if (result == 0) {
        result = robot_->GetInverseKinRef(0, &target, &currentJoint, &targetJoint);
    }
    if (result == 0) {
        // tool=0,user=0 makes the target an explicit base-frame flange pose.
        // blendR=0 keeps MoveL non-blocking so this worker can service StopMotion.
        if (physicalMode) {
            // FAIRINO 3.9.4 robot.h: velAccParamMode=1 changes ovl/oacc
            // to physical linear speed (mm/s) and acceleration (mm/s^2).
            result = robot_->MoveL(
                &targetJoint, &target, 0, 0,
                100.0F, 100.0F,
                static_cast<float>(speedValue), 0.0F, 0,
                &externalAxis, 0, 0, &offset,
                static_cast<float>(accelerationValue), 1);
        } else {
            result = robot_->MoveL(
                &targetJoint, &target, 0, 0,
                static_cast<float>(speedValue),
                static_cast<float>(accelerationValue),
                100.0F, 0.0F, 0, &externalAxis, 0, 0, &offset,
                100.0F, 0);
        }
    }
    if (result != 0) {
        emit motionFinished(requestId, false, true,
                            QStringLiteral("FR5 MoveL/逆解失败，err=%1").arg(result));
        return;
    }
    motionActive_ = true;
    motionStopUnconfirmed_ = true;
    stopConfirmationPending_ = false;
    motionRequestId_ = requestId;
    motionStartMs_ = hik_sync::getMonotonicRawNs() / 1000000LL;
    motionTimeoutMs_ = timeoutMs;
    stopConfirmationStartMs_ = 0;
    lastStopSampleReceiveNs_ = 0;
    stationaryStopSamples_ = 0;
    motionPollTimer_->start();
    emit motionStarted(requestId, physicalMode
        ? QStringLiteral(
            "MoveL 已发送：tool=0,user=0,physical_speed=%1 mm/s,physical_acc=%2 mm/s²,velAccParamMode=1,blendR=0")
              .arg(speedValue, 0, 'f', 3).arg(accelerationValue, 0, 'f', 3)
        : QStringLiteral(
            "MoveL 已发送：tool=0,user=0,vel=%1%,acc=%2%,velAccParamMode=0,blendR=0")
              .arg(speedValue, 0, 'f', 2).arg(accelerationValue, 0, 'f', 2));
#endif
}

void FairinoReadOnlyWorker::stopMotion(int requestId) {
#ifndef HAVE_FAIRINO_SDK
    emit motionFinished(requestId, false, false,
                        QStringLiteral("Fairino SDK 不可用，无法确认停止。"));
#else
    if (!connected_ || !robot_) {
        if (motionActive_) {
            finishMotion(false, false,
                         QStringLiteral(
                             "FR5 尚未连接，无法发送 StopMotion 或确认机械停止。"));
        } else {
            emit motionFinished(
                requestId, false, false,
                QStringLiteral("FR5 尚未连接，无法软件停止。"));
        }
        return;
    }
    if (stopConfirmationPending_) {
        emit log(QStringLiteral(
            "StopMotion 已提交，正在等待机械停止确认；忽略重复停止请求。"));
        return;
    }
    const errno_t result = robot_->StopMotion();
    if (motionActive_) {
        if (result != 0) {
            finishMotion(
                false, false,
                QStringLiteral("StopMotion 失败，err=%1；请使用物理急停")
                    .arg(result));
        } else {
            beginStopConfirmation(QStringLiteral(
                "控制器已接受 StopMotion 请求"));
        }
    } else if (motionStopUnconfirmed_) {
        const bool stoppedConfirmed =
            result == 0 &&
            confirmStoppedBlocking(
                static_cast<int>(kStopConfirmationTimeoutMs));
        motionStopUnconfirmed_ = !stoppedConfirmed;
        emit motionFinished(
            requestId, false, stoppedConfirmed,
            result != 0
                ? QStringLiteral(
                      "StopMotion 重试失败，err=%1；请使用物理急停")
                      .arg(result)
                : stoppedConfirmed
                    ? QStringLiteral(
                          "StopMotion 重试后已由新鲜 20004 状态"
                          "连续确认机械停止")
                    : QStringLiteral(
                          "StopMotion 重试后仍未确认机械停止；"
                          "请使用物理急停"));
    } else {
        emit log(result == 0 ? QStringLiteral("StopMotion 已发送（当前无活动 MoveL）")
                             : QStringLiteral("StopMotion 失败，err=%1；请使用物理急停").arg(result));
    }
#endif
}

void FairinoReadOnlyWorker::pollMotionDone() {
#ifdef HAVE_FAIRINO_SDK
    if (!motionActive_ || !connected_ || !robot_) {
        motionPollTimer_->stop();
        return;
    }
    const qint64 nowNs = hik_sync::getMonotonicRawNs();
    const qint64 nowMs = nowNs / 1000000LL;
    if (stopConfirmationPending_) {
        qint64 sampleReceiveNs = 0;
        int sampleMotionDone = 0;
        double sampleLinearSpeed = 0.0;
        double sampleAngularSpeed = 0.0;
        {
            const std::lock_guard<std::mutex> lock(
                realtimeStopSnapshotMutex_);
            sampleReceiveNs = latestRealtimeHostReceiveNs_;
            sampleMotionDone = latestRealtimeMotionDone_;
            sampleLinearSpeed = latestRealtimeLinearSpeedMmS_;
            sampleAngularSpeed = latestRealtimeAngularSpeedDegS_;
        }
        if (sampleReceiveNs > lastStopSampleReceiveNs_) {
            lastStopSampleReceiveNs_ = sampleReceiveNs;
            const bool fresh =
                sampleReceiveNs > 0 && nowNs >= sampleReceiveNs &&
                nowNs - sampleReceiveNs <= kRealtimeFreshnessLimitNs;
            const bool motionDone =
                sampleMotionDone != 0;
            const double linearSpeed = std::abs(sampleLinearSpeed);
            const double angularSpeed = std::abs(sampleAngularSpeed);
            if (fresh && motionDone &&
                std::isfinite(linearSpeed) &&
                std::isfinite(angularSpeed) &&
                linearSpeed <= kStoppedLinearSpeedMmS &&
                angularSpeed <= kStoppedAngularSpeedDegS) {
                ++stationaryStopSamples_;
            } else {
                stationaryStopSamples_ = 0;
            }
            if (stationaryStopSamples_ >=
                kRequiredStationaryStopSamples) {
                finishMotion(
                    false, true,
                    QStringLiteral(
                        "StopMotion 后已由新鲜 20004 状态连续确认机械停止"));
                return;
            }
        }
        if (nowMs - stopConfirmationStartMs_ >
            kStopConfirmationTimeoutMs) {
            finishMotion(
                false, false,
                QStringLiteral(
                    "StopMotion 已提交，但 %1 ms 内未由新鲜 20004 状态"
                    "确认机械停止；请使用物理急停")
                    .arg(kStopConfirmationTimeoutMs));
        }
        return;
    }
    uint8_t done = 0;
    const errno_t result = robot_->GetRobotMotionDone(&done);
    if (result != 0) {
        const errno_t stopResult = robot_->StopMotion();
        if (stopResult != 0) {
            finishMotion(
                false, false,
                QStringLiteral(
                    "GetRobotMotionDone 失败，err=%1；"
                    "StopMotion 失败，err=%2；请使用物理急停")
                    .arg(result)
                    .arg(stopResult));
        } else {
            beginStopConfirmation(QStringLiteral(
                "GetRobotMotionDone 失败，err=%1；StopMotion 已提交")
                .arg(result));
        }
        return;
    }
    if (done != 0U) {
        finishMotion(true, true, QStringLiteral("FR5 运动已到位"));
        return;
    }
    if (nowMs - motionStartMs_ > motionTimeoutMs_) {
        const errno_t stopResult = robot_->StopMotion();
        if (stopResult != 0) {
            finishMotion(
                false, false,
                QStringLiteral(
                    "FR5 运动等待超时；StopMotion 失败，err=%1；"
                    "请使用物理急停")
                    .arg(stopResult));
        } else {
            beginStopConfirmation(QStringLiteral(
                "FR5 运动等待超时；StopMotion 已提交"));
        }
    }
#endif
}

bool FairinoReadOnlyWorker::startRealtimeReceiving() {
#ifdef HAVE_FAIRINO_SDK
    stopRealtimeReceiving();
    if (!connected_ || !robot_ || !realtimeReceiver_) return false;
    realtimeReceiver_->reset();
    {
        const std::lock_guard<std::mutex> lock(
            realtimeStopSnapshotMutex_);
        latestRealtimeHostReceiveNs_ = 0;
        latestRealtimeMotionDone_ = 0;
        latestRealtimeLinearSpeedMmS_ = 0.0;
        latestRealtimeAngularSpeedDegS_ = 0.0;
    }
    realtimeReceiving_.store(true, std::memory_order_release);
    const errno_t callbackResult = robot_->SetRobotRealtimeStateCallback(
        &fairinoRealtimeStateCallback, realtimeReceiver_.get());
    if (callbackResult != 0) {
        realtimeReceiving_.store(false, std::memory_order_release);
        emit error(-1, QStringLiteral(
            "注册FR5 20004逐包回调失败，err=%1。").arg(callbackResult));
        return false;
    }
    try {
        realtimeThread_ = std::thread([this] { realtimeConsumerLoop(); });
    } catch (const std::exception& exception) {
        (void)robot_->SetRobotRealtimeStateCallback(nullptr, nullptr);
        realtimeReceiving_.store(false, std::memory_order_release);
        emit error(-1, QStringLiteral(
            "启动FR5 SPSC消费线程失败：%1").arg(exception.what()));
        return false;
    }
    return true;
#else
    return false;
#endif
}

void FairinoReadOnlyWorker::stopRealtimeReceiving() {
#ifdef HAVE_FAIRINO_SDK
    if (robot_) {
        // The patched SDK waits for an in-flight callback before unregistering.
        (void)robot_->SetRobotRealtimeStateCallback(nullptr, nullptr);
    }
    realtimeReceiving_.store(false, std::memory_order_release);
    if (realtimeReceiver_) realtimeReceiver_->condition.notify_all();
    if (realtimeThread_.joinable() &&
        realtimeThread_.get_id() != std::this_thread::get_id()) {
        realtimeThread_.join();
    }
    {
        const std::lock_guard<std::mutex> lock(
            realtimeStopSnapshotMutex_);
        latestRealtimeHostReceiveNs_ = 0;
    }
    if (realtimeReceiver_) {
        const uint64_t dropped = realtimeReceiver_->droppedPackets.load(
            std::memory_order_relaxed);
        if (dropped > 0U) {
            emit log(QStringLiteral(
                "警告：FR5 20004 SPSC队列累计丢包=%1。").arg(dropped));
        }
    }
#endif
}

void FairinoReadOnlyWorker::realtimeConsumerLoop() {
#ifdef HAVE_FAIRINO_SDK
    while (realtimeReceiver_) {
        FairinoRealtimeReceiverState::Packet packet;
        if (!realtimeReceiver_->pop(&packet)) {
            if (!realtimeReceiving_.load(std::memory_order_acquire)) break;
            std::unique_lock<std::mutex> lock(realtimeReceiver_->waitMutex);
            realtimeReceiver_->condition.wait_for(
                lock, std::chrono::milliseconds(100), [this] {
                    return !realtimeReceiving_.load(std::memory_order_acquire) ||
                           !realtimeReceiver_->empty();
                });
            continue;
        }

        const ROBOT_STATE_PKG& state = packet.state;
        ROBOT_STATE_PKG getterSnapshot{};
        const int64_t getterStartNs = hik_sync::getMonotonicRawNs();
        const errno_t getterResult =
            robot_->GetRobotRealTimeState(&getterSnapshot);
        const int64_t getterEndNs = hik_sync::getMonotonicRawNs();
        const int64_t getterDurationNs = std::max<int64_t>(
            0, getterEndNs - getterStartNs);

        hik_sync::RobotSample sample;
        sample.sdkReceiveSequence = packet.receiveSequence;
        sample.rawFrameCount = state.frame_cnt;
        sample.hostReceiveNs = packet.hostReceiveNs;
        sample.getterDurationNs = getterDurationNs;
        sample.getterResult = getterResult;
        bool controllerTimeValid = false;
        sample.robotTimestampNs = hik_sync::controllerCalendarToNs(
            state.robotTime.year, state.robotTime.mouth, state.robotTime.day,
            state.robotTime.hour, state.robotTime.minute, state.robotTime.second,
            state.robotTime.millisecond, &controllerTimeValid);
        sample.hasRobotTimestamp = controllerTimeValid;
        bool finite = state.frame_head == 0x5A5AU &&
                      packet.hostReceiveNs > 0;
        for (int index = 0; index < 6; ++index) {
            sample.flangePoseRaw[static_cast<std::size_t>(index)] =
                state.flange_cur_pos[index];
            sample.jointPositionDeg[static_cast<std::size_t>(index)] =
                state.jt_cur_pos[index];
            sample.actualTcpSpeed[static_cast<std::size_t>(index)] =
                state.actual_TCP_Speed[index];
            finite = finite && std::isfinite(state.flange_cur_pos[index]) &&
                     std::isfinite(state.jt_cur_pos[index]) &&
                     std::isfinite(state.actual_TCP_Speed[index]);
        }
        sample.flangePositionMm = Eigen::Vector3d(
            state.flange_cur_pos[0], state.flange_cur_pos[1],
            state.flange_cur_pos[2]);
        sample.flangeOrientation = hik_sync::fairinoFixedAxisRpyDegrees(
            state.flange_cur_pos[3], state.flange_cur_pos[4],
            state.flange_cur_pos[5]);
        sample.actualLinearSpeedMmS = state.actual_TCP_CmpSpeed[0];
        finite = finite && std::isfinite(sample.actualLinearSpeedMmS) &&
                 std::isfinite(state.actual_TCP_CmpSpeed[1]) &&
                 sample.flangeOrientation.coeffs().allFinite();
        sample.valid = finite;
        if (finite) {
            const std::lock_guard<std::mutex> lock(
                realtimeStopSnapshotMutex_);
            latestRealtimeMotionDone_ = state.motion_done;
            latestRealtimeLinearSpeedMmS_ =
                state.actual_TCP_CmpSpeed[0];
            latestRealtimeAngularSpeedDegS_ =
                state.actual_TCP_CmpSpeed[1];
            latestRealtimeHostReceiveNs_ = packet.hostReceiveNs;
        }
        emit robotSampleReady(std::move(sample));
    }
#endif
}
