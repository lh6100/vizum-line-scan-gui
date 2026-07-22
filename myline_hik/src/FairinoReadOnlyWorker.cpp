#include "FairinoReadOnlyWorker.h"

#include <QTimer>

#ifdef HAVE_FAIRINO_SDK
#include "robot.h"
#endif

#include <cmath>

FairinoReadOnlyWorker::FairinoReadOnlyWorker(QObject* parent)
    : QObject(parent) {
#ifdef HAVE_FAIRINO_SDK
    robot_ = new FRRobot;
#endif
    motionPollTimer_ = new QTimer(this);
    motionPollTimer_->setInterval(100);
    connect(motionPollTimer_, &QTimer::timeout,
            this, &FairinoReadOnlyWorker::pollMotionDone);
    realtimePollTimer_ = new QTimer(this);
    realtimePollTimer_->setTimerType(Qt::PreciseTimer);
    // The SDK owns the blocking TCP receive thread and exposes no per-packet
    // callback. Poll its lock-free snapshot faster than the configured 8 ms
    // period, then publish only when frame_cnt changes.
    realtimePollTimer_->setInterval(1);
    connect(realtimePollTimer_, &QTimer::timeout,
            this, &FairinoReadOnlyWorker::pollRealtimeState);
}

FairinoReadOnlyWorker::~FairinoReadOnlyWorker() {
#ifdef HAVE_FAIRINO_SDK
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
    if (periodResult != 0 || actualPeriodMs != 8) {
        periodResult = robot_->SetRobotRealtimeStateSamplePeriod(8);
        if (periodResult == 0) {
            periodResult = robot_->GetRobotRealtimeStateSamplePeriod(actualPeriodMs);
        }
    }
    if (periodResult != 0 || actualPeriodMs != 8) {
        (void)robot_->CloseRPC();
        connected_ = false;
        terminal_ = true;
        emit connectionChanged(false, QStringLiteral("FR5 实时反馈周期配置失败"));
        emit error(-1, QStringLiteral(
            "FR5 20004 实时状态周期必须为 8 ms；Set/GetRobotRealtimeStateSamplePeriod 失败，err=%1，读回=%2 ms。")
            .arg(periodResult).arg(actualPeriodMs));
        return;
    }
    haveRealtimeFrame_ = false;
    realtimePollTimer_->start();
    emit realtimePeriodConfigured(actualPeriodMs);
    emit connectionChanged(true, QStringLiteral("FR5 连接成功：%1").arg(ipAddress));
    emit log(QStringLiteral(
        "FR5 SDK 会话已建立；20004 实时状态周期已读取/配置为 %1 ms；程序不会自动使能或切换控制器模式。")
        .arg(actualPeriodMs));
#endif
}

void FairinoReadOnlyWorker::disconnectRobot() {
#ifdef HAVE_FAIRINO_SDK
    realtimePollTimer_->stop();
    haveRealtimeFrame_ = false;
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

void FairinoReadOnlyWorker::pollRealtimeState() {
#ifdef HAVE_FAIRINO_SDK
    if (!connected_ || !robot_) return;
    ROBOT_STATE_PKG state{};
    const errno_t result = robot_->GetRobotRealTimeState(&state);
    // This is the earliest host-side instant available through SDK 3.9.4: the
    // public getter copies the most recently received 20004 packet but exposes
    // no receive callback or receive timestamp.
    const int64_t receiveNs = hik_sync::getMonotonicRawNs();
    if (result != 0) {
        realtimePollTimer_->stop();
        emit error(-1, QStringLiteral(
            "GetRobotRealTimeState 失败，err=%1；实时同步流已停止。").arg(result));
        return;
    }
    if (haveRealtimeFrame_ && state.frame_cnt == lastRealtimeFrame_) return;
    haveRealtimeFrame_ = true;
    lastRealtimeFrame_ = state.frame_cnt;

    hik_sync::RobotSample sample;
    sample.rawFrameCount = state.frame_cnt;
    sample.hostReceiveNs = receiveNs;
    bool controllerTimeValid = false;
    sample.robotTimestampNs = hik_sync::controllerCalendarToNs(
        state.robotTime.year, state.robotTime.mouth, state.robotTime.day,
        state.robotTime.hour, state.robotTime.minute, state.robotTime.second,
        state.robotTime.millisecond, &controllerTimeValid);
    sample.hasRobotTimestamp = controllerTimeValid;
    bool finite = state.frame_head == 0x5A5AU;
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
#endif
}
