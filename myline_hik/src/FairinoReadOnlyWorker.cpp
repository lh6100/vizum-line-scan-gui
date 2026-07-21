#include "FairinoReadOnlyWorker.h"

#include <QDateTime>
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
    emit connectionChanged(true, QStringLiteral("FR5 连接成功：%1").arg(ipAddress));
    emit log(QStringLiteral("FR5 SDK 会话已建立；程序不会自动使能或切换控制器模式。"));
#endif
}

void FairinoReadOnlyWorker::disconnectRobot() {
#ifdef HAVE_FAIRINO_SDK
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
                         QDateTime::currentMSecsSinceEpoch());
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
                             velocityPercent, accelerationPercent};
    for (int index = 0; index < 8; ++index) {
        if (!std::isfinite(values[index])) {
            emit motionFinished(requestId, false, QStringLiteral("MoveL 参数包含非有限数。"));
            return;
        }
    }
    if (velocityPercent <= 0.0 || velocityPercent > 20.0 ||
        accelerationPercent <= 0.0 || accelerationPercent > 50.0 ||
        timeoutMs < 1000 || timeoutMs > 300000) {
        emit motionFinished(requestId, false, QStringLiteral(
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
        result = robot_->MoveL(
            &targetJoint, &target, 0, 0,
            static_cast<float>(velocityPercent),
            static_cast<float>(accelerationPercent),
            100.0F, 0.0F, 0, &externalAxis, 0, 0, &offset,
            100.0F, 0);
    }
    if (result != 0) {
        emit motionFinished(requestId, false,
                            QStringLiteral("FR5 MoveL/逆解失败，err=%1").arg(result));
        return;
    }
    motionActive_ = true;
    motionRequestId_ = requestId;
    motionStartMs_ = QDateTime::currentMSecsSinceEpoch();
    motionTimeoutMs_ = timeoutMs;
    motionPollTimer_->start();
    emit motionStarted(requestId, QStringLiteral(
        "MoveL 已发送：tool=0,user=0,vel=%1%,acc=%2%,blendR=0")
        .arg(velocityPercent, 0, 'f', 2)
        .arg(accelerationPercent, 0, 'f', 2));
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
    if (QDateTime::currentMSecsSinceEpoch() - motionStartMs_ > motionTimeoutMs_) {
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
