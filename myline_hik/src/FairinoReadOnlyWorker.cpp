#include "FairinoReadOnlyWorker.h"

#include <QTimer>

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
#include <mutex>

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
    if (connected_ && robot_) {
        (void)robot_->CloseRPC();
    }
    // Fairino 3.9.4 owns detached SDK threads and exposes no join operation.
    // Releasing this one object at process teardown avoids a shutdown-only
    // use-after-free; the operating system reclaims it immediately afterward.
    robot_ = nullptr;
#endif
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
            "Fairino SDK 本进程已结束一次 RPC 会话，不能安全重连；请关闭并重新启动标定工具。"));
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
    stopRealtimeReceiving();
    if (motionActive_ && robot_) {
        (void)robot_->StopMotion();
        motionPollTimer_->stop();
        motionActive_ = false;
        emit motionFinished(motionRequestId_, false,
                            QStringLiteral("断开连接前已发送 StopMotion"));
        motionRequestId_ = -1;
    }
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

void FairinoReadOnlyWorker::moveLinearImpl(
        int requestId, double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg,
        double speedValue, double accelerationValue,
        int timeoutMs, bool physicalMode) {
#ifndef HAVE_FAIRINO_SDK
    emit motionFinished(requestId, false, QStringLiteral("Fairino SDK 不可用。"));
#else
    if (!connected_ || !robot_) {
        emit motionFinished(requestId, false, QStringLiteral("FR5 尚未连接。"));
        return;
    }
    if (motionActive_) {
        emit motionFinished(requestId, false, QStringLiteral("已有 MoveL 正在执行。"));
        return;
    }
    const double values[] = {xMm, yMm, zMm, rxDeg, ryDeg, rzDeg,
                             speedValue, accelerationValue};
    for (int index = 0; index < 8; ++index) {
        if (!std::isfinite(values[index])) {
            emit motionFinished(requestId, false, QStringLiteral("MoveL 参数包含非有限数。"));
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
        emit motionFinished(requestId, false, physicalMode
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
        emit motionFinished(requestId, false,
                            QStringLiteral("FR5 MoveL/逆解失败，err=%1").arg(result));
        return;
    }
    motionActive_ = true;
    motionRequestId_ = requestId;
    motionStartMs_ = hik_sync::getMonotonicRawNs() / 1000000LL;
    motionTimeoutMs_ = timeoutMs;
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
    emit motionFinished(requestId, false, QStringLiteral("Fairino SDK 不可用。"));
#else
    if (!connected_ || !robot_) {
        emit motionFinished(requestId, false, QStringLiteral("FR5 尚未连接，无法软件停止。"));
        return;
    }
    const errno_t result = robot_->StopMotion();
    if (motionActive_) {
        motionPollTimer_->stop();
        const int activeRequest = motionRequestId_;
        motionActive_ = false;
        motionRequestId_ = -1;
        emit motionFinished(activeRequest, false,
                            result == 0 ? QStringLiteral("用户已发送 StopMotion")
                                        : QStringLiteral("StopMotion 失败，err=%1；请使用物理急停").arg(result));
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
    uint8_t done = 0;
    const errno_t result = robot_->GetRobotMotionDone(&done);
    if (result != 0) {
        (void)robot_->StopMotion();
        motionPollTimer_->stop();
        const int request = motionRequestId_;
        motionActive_ = false;
        motionRequestId_ = -1;
        emit motionFinished(request, false,
                            QStringLiteral("GetRobotMotionDone 失败，err=%1；已请求 StopMotion").arg(result));
        return;
    }
    if (done != 0U) {
        motionPollTimer_->stop();
        const int request = motionRequestId_;
        motionActive_ = false;
        motionRequestId_ = -1;
        emit motionFinished(request, true, QStringLiteral("FR5 MoveL 已到位"));
        return;
    }
    if (hik_sync::getMonotonicRawNs() / 1000000LL - motionStartMs_ > motionTimeoutMs_) {
        const errno_t stopResult = robot_->StopMotion();
        motionPollTimer_->stop();
        const int request = motionRequestId_;
        motionActive_ = false;
        motionRequestId_ = -1;
        emit motionFinished(request, false,
            QStringLiteral("MoveL 等待超时；StopMotion err=%1").arg(stopResult));
    }
#endif
}

bool FairinoReadOnlyWorker::startRealtimeReceiving() {
#ifdef HAVE_FAIRINO_SDK
    stopRealtimeReceiving();
    if (!connected_ || !robot_ || !realtimeReceiver_) return false;
    realtimeReceiver_->reset();
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

        ROBOT_STATE_PKG getterSnapshot{};
        const int64_t getterStartNs = hik_sync::getMonotonicRawNs();
        const errno_t getterResult =
            robot_->GetRobotRealTimeState(&getterSnapshot);
        const int64_t getterEndNs = hik_sync::getMonotonicRawNs();
        const int64_t getterDurationNs = std::max<int64_t>(
            0, getterEndNs - getterStartNs);

        const ROBOT_STATE_PKG& state = packet.state;
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
                 sample.flangeOrientation.coeffs().allFinite();
        sample.valid = finite;
        emit robotSampleReady(std::move(sample));
    }
#endif
}
