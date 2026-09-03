#include "stereo/app/StereoCameraRig.h"

#include "HikCameraWorker.h"

#include <QMetaObject>

namespace hik_stereo {

StereoCameraRig::StereoCameraRig(QObject* parent)
    : QObject(parent), pairer_(5.0, 8U) {
    qRegisterMetaType<hik_sync::CameraFrame>("hik_sync::CameraFrame");
    qRegisterMetaType<StereoCameraSide>("hik_stereo::StereoCameraSide");
    qRegisterMetaType<StereoFramePair>("hik_stereo::StereoFramePair");
    leftThread_.setObjectName(QStringLiteral("StereoLeftCamera"));
    rightThread_.setObjectName(QStringLiteral("StereoRightCamera"));
    setupCamera(&leftWorker_, &leftThread_, StereoCameraSide::Left);
    setupCamera(&rightWorker_, &rightThread_, StereoCameraSide::Right);
    leftThread_.start();
    rightThread_.start();
}

StereoCameraRig::~StereoCameraRig() {
    shutdownWorker(leftWorker_, &leftThread_);
    shutdownWorker(rightWorker_, &rightThread_);
    leftWorker_ = nullptr;
    rightWorker_ = nullptr;
}

void StereoCameraRig::setupCamera(HikCameraWorker** output,
                                  QThread* thread,
                                  StereoCameraSide side) {
    HikCameraWorker* worker = new HikCameraWorker;
    *output = worker;
    worker->moveToThread(thread);
    connect(thread, &QThread::finished, worker, &QObject::deleteLater);
    connect(worker, &HikCameraWorker::connectionChanged,
            this, [this, side](bool connected, const QString& description) {
                if (side == StereoCameraSide::Left)
                    leftConnected_ = connected;
                else
                    rightConnected_ = connected;
                emit connectionChanged(side, connected, description);
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::identityChanged,
            this, [this, side](const QString& model,
                              const QString& serial,
                              const QString& ip) {
                emit identityChanged(side, model, serial, ip);
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::continuousFrameReady,
            this, [this, side](hik_sync::CameraFrame frame) {
                handleFrame(side, std::move(frame));
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::continuousStarted,
            this, [this, side](double, double, quint64, const QString&) {
                handleContinuousStarted(side);
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::continuousStopped,
            this, [this, side](bool confirmed, const QString& description) {
                handleContinuousStopped(side, confirmed, description);
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::continuousFrameRejected,
            this, [this, side](quint64 frame, const QString& reason) {
                emit log(QStringLiteral("%1 相机拒绝帧 %2：%3")
                    .arg(side == StereoCameraSide::Left
                             ? QStringLiteral("左") : QStringLiteral("右"))
                    .arg(frame).arg(reason));
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::imagePoolExhausted,
            this, [this, side]() {
                emit error(QStringLiteral("%1相机图像池耗尽；已丢帧，降低帧率。")
                    .arg(side == StereoCameraSide::Left
                             ? QStringLiteral("左") : QStringLiteral("右")));
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::error,
            this, [this, side](int, const QString& message) {
                emit error(QStringLiteral("%1相机：%2")
                    .arg(side == StereoCameraSide::Left
                             ? QStringLiteral("左") : QStringLiteral("右"),
                         message));
            }, Qt::QueuedConnection);
    connect(worker, &HikCameraWorker::log,
            this, &StereoCameraRig::log, Qt::QueuedConnection);
}

void StereoCameraRig::connectCameras(QString leftIp, QString rightIp) {
    if (leftIp.trimmed().isEmpty() || rightIp.trimmed().isEmpty() ||
        leftIp.trimmed() == rightIp.trimmed()) {
        emit error(QStringLiteral("双目相机 IP 必须非空且互不相同。"));
        return;
    }
    QMetaObject::invokeMethod(leftWorker_, "connectCamera",
        Qt::QueuedConnection, Q_ARG(QString, leftIp.trimmed()));
    QMetaObject::invokeMethod(rightWorker_, "connectCamera",
        Qt::QueuedConnection, Q_ARG(QString, rightIp.trimmed()));
}

void StereoCameraRig::disconnectCameras() {
    if (running_ || stopping_) stop();
    QMetaObject::invokeMethod(leftWorker_, "disconnectCamera",
                             Qt::QueuedConnection);
    QMetaObject::invokeMethod(rightWorker_, "disconnectCamera",
                             Qt::QueuedConnection);
}

void StereoCameraRig::start(double leftExposureUs,
                            double rightExposureUs,
                            double gainDb,
                            double framesPerSecond,
                            double maximumPairSkewMs) {
    if (!leftConnected_ || !rightConnected_) {
        emit error(QStringLiteral("两台相机尚未全部连接。"));
        return;
    }
    if (running_ || stopping_) {
        emit error(QStringLiteral("双目连续采集已经运行或正在停止。"));
        return;
    }
    pairer_.configure(maximumPairSkewMs, 8U);
    leftStarted_ = false;
    rightStarted_ = false;
    leftStoppedConfirmed_ = false;
    rightStoppedConfirmed_ = false;
    QMetaObject::invokeMethod(leftWorker_, "startContinuous",
        Qt::QueuedConnection,
        Q_ARG(double, leftExposureUs), Q_ARG(double, gainDb),
        Q_ARG(double, framesPerSecond), Q_ARG(int, 32));
    QMetaObject::invokeMethod(rightWorker_, "startContinuous",
        Qt::QueuedConnection,
        Q_ARG(double, rightExposureUs), Q_ARG(double, gainDb),
        Q_ARG(double, framesPerSecond), Q_ARG(int, 32));
}

void StereoCameraRig::stop() {
    if (stopping_) return;
    if (!running_ && !leftStarted_ && !rightStarted_) {
        emit stopped(true, QStringLiteral("双目采集未运行。"));
        return;
    }
    stopping_ = true;
    QMetaObject::invokeMethod(leftWorker_, "stopContinuous",
                             Qt::QueuedConnection);
    QMetaObject::invokeMethod(rightWorker_, "stopContinuous",
                             Qt::QueuedConnection);
}

void StereoCameraRig::handleFrame(StereoCameraSide side,
                                  hik_sync::CameraFrame frame) {
    if (!running_ || stopping_) return;
    if (side == StereoCameraSide::Left)
        pairer_.pushLeft(std::move(frame));
    else
        pairer_.pushRight(std::move(frame));
    StereoFramePair pair;
    while (pairer_.takePair(&pair)) emit pairReady(std::move(pair));
}

void StereoCameraRig::handleContinuousStarted(StereoCameraSide side) {
    if (side == StereoCameraSide::Left) leftStarted_ = true;
    else rightStarted_ = true;
    if (leftStarted_ && rightStarted_ && !running_) {
        running_ = true;
        emit started();
    }
}

void StereoCameraRig::handleContinuousStopped(
        StereoCameraSide side, bool confirmed,
        const QString& description) {
    if (side == StereoCameraSide::Left) {
        leftStarted_ = false;
        leftStoppedConfirmed_ = confirmed;
    } else {
        rightStarted_ = false;
        rightStoppedConfirmed_ = confirmed;
    }
    emit log(QStringLiteral("%1相机停止：%2")
        .arg(side == StereoCameraSide::Left
                 ? QStringLiteral("左") : QStringLiteral("右"),
             description));
    if (!leftStarted_ && !rightStarted_) {
        const bool allConfirmed = leftStoppedConfirmed_ &&
                                  rightStoppedConfirmed_;
        running_ = false;
        stopping_ = false;
        pairer_.clear();
        emit stopped(allConfirmed,
            allConfirmed ? QStringLiteral("两台相机均已停止。")
                         : QStringLiteral("至少一台相机停止未确认。"));
    }
}

StereoFramePairerStatistics StereoCameraRig::pairerStatistics() const {
    return pairer_.statistics();
}

void StereoCameraRig::shutdownWorker(HikCameraWorker* worker,
                                     QThread* thread) {
    if (!worker || !thread || !thread->isRunning()) return;
    QMetaObject::invokeMethod(worker, "stopContinuous",
                             Qt::BlockingQueuedConnection);
    QMetaObject::invokeMethod(worker, "disconnectCamera",
                             Qt::BlockingQueuedConnection);
    thread->quit();
    thread->wait();
}

}  // namespace hik_stereo
