#include "stereo/app/StereoMapperWindow.h"

#include "FairinoRobotSession.h"
#include "ImageView.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <stdexcept>

namespace hik_stereo {

StereoMapperWindow::StereoMapperWindow(
        LineLaserController* laserController,
        FairinoRobotSession* robotSession,
        QWidget* parent)
    : QWidget(parent),
      laserController_(laserController),
      robotSession_(robotSession),
      sourceDirectory_(QStringLiteral(HIK_CALIBRATION_SOURCE_DIR)) {
    // Keep reconstruction on the same physical convention as calibration.
    // Profile names identify existing calibration records, not stereo sides.
    leftProfile_ = findLineLaserDeviceProfile(QStringLiteral("scanner_650"));
    rightProfile_ = findLineLaserDeviceProfile(QStringLiteral("scanner_450"));
    if (!laserController_ || !robotSession_ ||
        !leftProfile_ || !rightProfile_) {
        throw std::runtime_error(
            "stereo mapper requires scanner_650 as left and scanner_450 as right");
    }
    qRegisterMetaType<hik_sync::RobotSample>("hik_sync::RobotSample");
    qRegisterMetaType<StereoProcessingRequest>(
        "hik_stereo::StereoProcessingRequest");
    qRegisterMetaType<StereoUiFrame>("hik_stereo::StereoUiFrame");

    buildUi();
    appendLog(QStringLiteral(
        "物理双目顺序：左=160万 DA8784601（192.168.7.45），"
        "右=130万 DB0403208（192.168.1.46）。"));
    cameraRig_ = new StereoCameraRig(this);
    processingWorker_ = new StereoMappingWorker;
    processingThread_.setObjectName(QStringLiteral("StereoDepthAndMap"));
    processingWorker_->moveToThread(&processingThread_);
    connect(&processingThread_, &QThread::finished,
            processingWorker_, &QObject::deleteLater);
    processingThread_.start();

    connect(connectCamerasButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::connectCameras);
    connect(connectRobotButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::connectRobot);
    connect(loadCalibrationButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::loadCalibration);
    connect(startButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::startMapping);
    connect(stopButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::stopMapping);
    connect(resetButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::resetMap);
    connect(exportButton_, &QPushButton::clicked,
            this, &StereoMapperWindow::exportMap);

    connect(cameraRig_, &StereoCameraRig::connectionChanged,
            this, &StereoMapperWindow::onCameraConnection);
    connect(cameraRig_, &StereoCameraRig::identityChanged,
            this, &StereoMapperWindow::onCameraIdentity);
    connect(cameraRig_, &StereoCameraRig::pairReady,
            this, &StereoMapperWindow::onStereoPair);
    connect(cameraRig_, &StereoCameraRig::started,
            this, [this]() {
                mappingActive_ = true;
                mappingStatusLabel_->setText(
                    QStringLiteral("双目连续采集运行中；正在等待帧/机器人位姿配对。"));
                appendLog(QStringLiteral("双目采集已启动。"));
                updateUi();
            });
    connect(cameraRig_, &StereoCameraRig::stopped,
            this, &StereoMapperWindow::onCameraRigStopped);
    connect(cameraRig_, &StereoCameraRig::error,
            this, [this](const QString& message) {
                appendLog(QStringLiteral("错误：%1").arg(message));
                if (mappingRequested_) stopMapping();
            });
    connect(cameraRig_, &StereoCameraRig::log,
            this, &StereoMapperWindow::appendLog);
    connect(processingWorker_, &StereoMappingWorker::frameProcessed,
            this, &StereoMapperWindow::onFrameProcessed,
            Qt::QueuedConnection);
    connect(processingWorker_, &StereoMappingWorker::mapCleared,
            this, [this]() {
                processedFrames_ = 0;
                appendLog(QStringLiteral("占据地图已清空。"));
                updateUi();
            }, Qt::QueuedConnection);
    connect(processingWorker_, &StereoMappingWorker::mapExported,
            this, [this](bool ok, const QString& directory,
                         const QString& description) {
                appendLog(QStringLiteral("地图导出%1：%2｜%3")
                    .arg(ok ? QStringLiteral("成功") : QStringLiteral("失败"),
                         directory, description));
                if (!ok) QMessageBox::critical(
                    this, QStringLiteral("地图导出失败"), description);
                updateUi();
            }, Qt::QueuedConnection);

    robotClientId_ = robotSession_->registerClient(
        QStringLiteral("stereo_environment_mapper"));
    if (robotClientId_ <= 0 || !robotSession_->isRunning()) {
        throw std::runtime_error("unable to register stereo mapper with FR5 session");
    }
    connect(robotSession_, &FairinoRobotSession::connectionChanged,
            this, [this](bool connected, const QString& description) {
                robotConnected_ = connected;
                robotStatusLabel_->setText(description);
                updateUi();
            }, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::robotSampleReady,
            this, &StereoMapperWindow::onRobotSample,
            Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::error,
            this, [this](int requestId, const QString& message) {
                if (robotSession_->requestBelongsToClient(
                        requestId, robotClientId_)) {
                    appendLog(QStringLiteral("FR5 错误：%1").arg(message));
                    if (mappingRequested_) stopMapping();
                }
            }, Qt::QueuedConnection);
    connect(laserController_, &LineLaserController::statusChanged,
            this, &StereoMapperWindow::onLaserStatus,
            Qt::QueuedConnection);
    connect(laserController_, &LineLaserController::faultOccurred,
            this, [this](const QString& message) {
                appendLog(QStringLiteral("TTL 故障：%1").arg(message));
                if (mappingRequested_) stopMapping();
            }, Qt::QueuedConnection);

    laserWatchdog_ = new QTimer(this);
    laserWatchdog_->setInterval(250);
    connect(laserWatchdog_, &QTimer::timeout, this, [this]() {
        const qint64 now = hik_sync::getMonotonicRawNs();
        if (waitingLaserOff_ && laserOffRequestedNs_ > 0 &&
            now - laserOffRequestedNs_ > 3000000000LL) {
            appendLog(QStringLiteral(
                "3 s 内未收到两路 TTL LOW 的因果 ACK，取消建图。"));
            stopMapping();
            return;
        }
        if (!mappingActive_) return;
        const bool fresh = latestLaserStatus_.sourceMonotonicNs > 0 &&
            now >= latestLaserStatus_.sourceMonotonicNs &&
            now - latestLaserStatus_.sourceMonotonicNs <= 1500000000LL;
        if (!fresh) {
            appendLog(QStringLiteral(
                "TTL 状态超过 1.5 s 未更新；按激光状态未知处理并停止建图。"));
            stopMapping();
        }
    });
    laserWatchdog_->start();

    loadCalibration();
    updateUi();
}

StereoMapperWindow::~StereoMapperWindow() {
    if (cameraRig_ && cameraRig_->running()) cameraRig_->stop();
    if (robotSession_ && robotSession_->isExclusiveOwner(robotClientId_))
        robotSession_->releaseExclusive(robotClientId_);
    if (laserController_) laserController_->off();
    processingThread_.quit();
    processingThread_.wait();
    processingWorker_ = nullptr;
}

void StereoMapperWindow::loadCalibration() {
    if (mappingRequested_) return;
    const QByteArray leftIntrinsics =
        leftProfile_->intrinsicsConfigPath(sourceDirectory_).toLocal8Bit();
    const QByteArray leftHandEye =
        leftProfile_->handEyeConfigPath(sourceDirectory_).toLocal8Bit();
    const QByteArray rightIntrinsics =
        rightProfile_->intrinsicsConfigPath(sourceDirectory_).toLocal8Bit();
    const QByteArray rightHandEye =
        rightProfile_->handEyeConfigPath(sourceDirectory_).toLocal8Bit();
    const QString stereoPath = QDir(sourceDirectory_).absoluteFilePath(
        QStringLiteral("config/hik_stereo.yaml"));
    std::string error;
    calibrationLoaded_ = QFileInfo::exists(stereoPath) &&
        loadStereoRigFromStereoYaml(
            stereoPath.toLocal8Bit().constData(),
            leftIntrinsics.constData(), leftHandEye.constData(),
            rightIntrinsics.constData(), rightHandEye.constData(),
            &rigCalibration_, &error);
    if (!calibrationLoaded_) {
        calibrationStatusLabel_->setText(QFileInfo::exists(stereoPath)
            ? QStringLiteral("正式双目外参加载失败：%1")
                  .arg(QString::fromStdString(error))
            : QStringLiteral(
                  "缺少 config/hik_stereo.yaml；请先运行 HikStereoCalibration。"
                  "两份独立手眼不会被当作双目 R/T。"));
        appendLog(calibrationStatusLabel_->text());
    } else {
        calibrationStatusLabel_->setText(QStringLiteral(
            "正式 stereoCalibrate R/T｜基线 %1 mm｜相对转角 %2°")
            .arg(rigCalibration_.baselineMm, 0, 'f', 2)
            .arg(rigCalibration_.relativeRotationDeg, 0, 'f', 2));
        appendLog(QStringLiteral("正式双目几何已加载：%1")
                      .arg(calibrationStatusLabel_->text()));
    }
    updateUi();
}

void StereoMapperWindow::connectCameras() {
    cameraRig_->connectCameras(leftIpEdit_->text(), rightIpEdit_->text());
}

void StereoMapperWindow::connectRobot() {
    robotSession_->connectRobot(robotClientId_, robotIpEdit_->text().trimmed());
}

void StereoMapperWindow::startMapping() {
    if (mappingRequested_) return;
    if (!calibrationLoaded_ || !leftConnected_ || !rightConnected_ ||
        !cameraIdentityAccepted(StereoCameraSide::Left) ||
        !cameraIdentityAccepted(StereoCameraSide::Right) ||
        !robotConnected_) {
        QMessageBox::warning(this, QStringLiteral("无法开始"),
            QStringLiteral("请先加载正式标定，连接并验证两台相机身份，同时连接 FR5。"));
        return;
    }
    if (maximumDepthSpin_->value() <= minimumDepthSpin_->value() ||
        gridMaximumHeightSpin_->value() <= gridMinimumHeightSpin_->value()) {
        QMessageBox::warning(this, QStringLiteral("参数无效"),
            QStringLiteral("最大深度/栅格最高高度必须大于对应最小值。"));
        return;
    }
    QString leaseError;
    if (!robotSession_->acquireExclusive(
            robotClientId_, QStringLiteral("双目环境建图（只读实时位姿）"),
            &leaseError)) {
        QMessageBox::critical(this, QStringLiteral("FR5 被占用"), leaseError);
        return;
    }
    robotLeaseEpoch_ = robotSession_->exclusiveLeaseEpochFor(robotClientId_);
    poseSynchronizer_.configure(
        robotOffsetSpin_->value(), robotGapSpin_->value());
    processedFrames_ = 0;
    busyDrops_ = 0;
    consecutiveProcessingErrors_ = 0;
    processingBusy_ = false;
    ++mappingGeneration_;
    mappingRequested_ = true;
    waitingLaserOff_ = true;
    pendingLaserOffToken_ = laserController_->requestOffTracked();
    laserOffRequestedNs_ = hik_sync::getMonotonicRawNs();
    mappingStatusLabel_->setText(
        QStringLiteral("正在等待两路激光 TTL LOW 的因果 ACK…"));
    appendLog(QStringLiteral("已取得 FR5 独占只读位姿租约；请求关闭两路线激光。"));
    updateUi();
}

void StereoMapperWindow::beginMappingAfterLaserOff() {
    if (!mappingRequested_ || !waitingLaserOff_) return;
    waitingLaserOff_ = false;
    bool configured = false;
    QString configureError;
    const StereoRigCalibration rig = rigCalibration_;
    const StereoDepthOptions depth = depthOptionsFromUi();
    const OccupancyMapOptions map = mapOptionsFromUi();
    QMetaObject::invokeMethod(
        processingWorker_,
        [this, &configured, &configureError, rig, depth, map]() {
            configured = processingWorker_->configure(
                rig, depth, map, &configureError);
        }, Qt::BlockingQueuedConnection);
    if (!configured) {
        appendLog(QStringLiteral("双目处理器配置失败：%1")
                      .arg(configureError));
        QMessageBox::critical(this, QStringLiteral("双目配置失败"),
                              configureError);
        finishMapping(configureError);
        return;
    }
    appendLog(QStringLiteral(
        "双目处理器已配置：%1×%2，depth=%3–%4 mm，voxel=%5 mm。")
        .arg(depth.processingSize.width).arg(depth.processingSize.height)
        .arg(depth.minimumDepthMm).arg(depth.maximumDepthMm)
        .arg(map.voxelSizeMm));
    cameraRig_->start(exposureSpin_->value(), exposureSpin_->value(),
                      gainSpin_->value(),
                      fpsSpin_->value(), pairSkewSpin_->value());
}

void StereoMapperWindow::stopMapping() {
    if (!mappingRequested_) return;
    waitingLaserOff_ = false;
    laserController_->off();
    if (cameraRig_->running()) {
        mappingStatusLabel_->setText(QStringLiteral("正在停止两台相机…"));
        cameraRig_->stop();
    } else {
        finishMapping(QStringLiteral("建图已停止。"));
    }
    updateUi();
}

void StereoMapperWindow::finishMapping(const QString& description) {
    mappingRequested_ = false;
    mappingActive_ = false;
    waitingLaserOff_ = false;
    processingBusy_ = false;
    poseSynchronizer_.clear();
    pendingLaserOffToken_ = 0;
    laserOffRequestedNs_ = 0;
    if (robotSession_->isExclusiveOwner(robotClientId_))
        robotSession_->releaseExclusive(robotClientId_);
    robotLeaseEpoch_ = 0;
    mappingStatusLabel_->setText(description);
    appendLog(description);
    updateUi();
}

void StereoMapperWindow::resetMap() {
    if (mappingRequested_ || processingBusy_) return;
    QMetaObject::invokeMethod(processingWorker_, "clearMap",
                             Qt::QueuedConnection);
}

void StereoMapperWindow::exportMap() {
    if (mappingRequested_ || processingBusy_ || processedFrames_ == 0) return;
    const QString directory = QDir(sourceDirectory_).absoluteFilePath(
        QStringLiteral("data/stereo_maps/session_%1")
            .arg(QDateTime::currentDateTime().toString(
                QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    QMetaObject::invokeMethod(processingWorker_, "exportMap",
        Qt::QueuedConnection,
        Q_ARG(QString, directory),
        Q_ARG(double, gridMinimumHeightSpin_->value()),
        Q_ARG(double, gridMaximumHeightSpin_->value()));
    appendLog(QStringLiteral("正在导出地图：%1").arg(directory));
}

void StereoMapperWindow::onLaserStatus(LineLaserStatus status) {
    latestLaserStatus_ = status;
    const bool off = status.reachable && !status.ttl450High &&
                     !status.ttl650High &&
                     status.state == LineLaserState::Off;
    laserStatusLabel_->setText(off
        ? QStringLiteral("TTL：两路线激光 LOW")
        : QStringLiteral("TTL：未确认关闭｜%1").arg(status.fault));
    if (mappingActive_ && !off) {
        appendLog(QStringLiteral(
            "建图期间检测到激光 TTL 非 LOW/不可达，停止双目采集。"));
        stopMapping();
        return;
    }
    if (waitingLaserOff_ && off && pendingLaserOffToken_ > 0 &&
        status.acknowledgedOffCommandToken >= pendingLaserOffToken_) {
        beginMappingAfterLaserOff();
    }
}

void StereoMapperWindow::onCameraConnection(
        StereoCameraSide side, bool connected, QString description) {
    if (side == StereoCameraSide::Left) {
        leftConnected_ = connected;
        leftStatusLabel_->setText(description);
    } else {
        rightConnected_ = connected;
        rightStatusLabel_->setText(description);
    }
    updateUi();
}

void StereoMapperWindow::onCameraIdentity(
        StereoCameraSide side, QString model,
        QString serial, QString) {
    if (side == StereoCameraSide::Left) {
        leftModel_ = model;
        leftSerial_ = serial;
        leftStatusLabel_->setText(QStringLiteral("%1｜SN %2")
                                  .arg(model, serial));
    } else {
        rightModel_ = model;
        rightSerial_ = serial;
        rightStatusLabel_->setText(QStringLiteral("%1｜SN %2")
                                   .arg(model, serial));
    }
    if (!cameraIdentityAccepted(side)) {
        appendLog(QStringLiteral("拒绝相机身份：%1侧 model=%2 serial=%3。")
            .arg(side == StereoCameraSide::Left
                     ? QStringLiteral("左") : QStringLiteral("右"),
                 model, serial));
    }
    updateUi();
}

void StereoMapperWindow::onStereoPair(StereoFramePair pair) {
    if (!mappingActive_) return;
    if (processingBusy_) {
        ++busyDrops_;
        return;
    }
    poseSynchronizer_.pushPair(std::move(pair));
    drainSynchronizedPairs();
}

void StereoMapperWindow::onRobotSample(
        int clientId, quint64 leaseEpoch,
        hik_sync::RobotSample sample) {
    if (!mappingRequested_ || clientId != robotClientId_ ||
        leaseEpoch != robotLeaseEpoch_) return;
    poseSynchronizer_.pushRobot(std::move(sample));
    drainSynchronizedPairs();
}

void StereoMapperWindow::drainSynchronizedPairs() {
    if (!mappingActive_ || processingBusy_) return;
    SynchronizedStereoFrame synchronized;
    std::string reason;
    if (!poseSynchronizer_.takeSynchronized(&synchronized, &reason)) {
        if (!reason.empty()) appendLog(QString::fromStdString(reason));
        return;
    }
    StereoProcessingRequest request;
    request.generation = mappingGeneration_;
    request.pair = std::move(synchronized.pair);
    request.baseFromFlange = eigenToCv(synchronized.baseFromFlange);
    request.robotGapMs = synchronized.robotGapMs;
    request.robotSpeedMmS = synchronized.actualLinearSpeedMmS;
    processingBusy_ = true;
    QMetaObject::invokeMethod(processingWorker_, "process",
        Qt::QueuedConnection,
        Q_ARG(hik_stereo::StereoProcessingRequest, request));
}

void StereoMapperWindow::onFrameProcessed(StereoUiFrame frame) {
    if (frame.generation != mappingGeneration_) return;
    processingBusy_ = false;
    if (!mappingRequested_) return;
    if (!frame.ok) {
        ++consecutiveProcessingErrors_;
        appendLog(QStringLiteral("深度帧失败：%1").arg(frame.error));
        if (consecutiveProcessingErrors_ >= 3) {
            appendLog(QStringLiteral("连续 3 帧深度失败，自动停止。"));
            stopMapping();
            return;
        }
    } else {
        consecutiveProcessingErrors_ = 0;
        ++processedFrames_;
        leftView_->setImage(frame.rectifiedLeft, true);
        disparityView_->setImage(frame.disparity, true);
        depthView_->setImage(frame.depth, true);
        statisticsLabel_->setText(QStringLiteral(
            "帧 L%1/R%2｜偏差 %3 ms｜处理 %4 ms｜有效 %5%｜中值深度 %6 mm｜"
            "机器人包 %7 ms / TCP %8 mm/s｜地图帧 %9｜占据体素 %10｜忙丢帧 %11")
            .arg(frame.leftFrameId).arg(frame.rightFrameId)
            .arg(frame.pairSkewMs, 0, 'f', 2)
            .arg(frame.depthStatistics.processingMs, 0, 'f', 1)
            .arg(frame.depthStatistics.validFraction * 100.0, 0, 'f', 1)
            .arg(frame.depthStatistics.medianDepthMm, 0, 'f', 1)
            .arg(frame.robotGapMs, 0, 'f', 2)
            .arg(frame.robotSpeedMmS, 0, 'f', 2)
            .arg(frame.mapStatistics.integratedFrames)
            .arg(frame.mapStatistics.occupiedVoxels)
            .arg(busyDrops_));
    }
    drainSynchronizedPairs();
}

void StereoMapperWindow::onCameraRigStopped(
        bool confirmed, QString description) {
    finishMapping(confirmed ? description
        : QStringLiteral("相机停止未全部确认：%1").arg(description));
}

bool StereoMapperWindow::cameraIdentityAccepted(
        StereoCameraSide side) const {
    const LineLaserDeviceProfile* profile =
        side == StereoCameraSide::Left ? leftProfile_ : rightProfile_;
    const QString model = side == StereoCameraSide::Left
        ? leftModel_ : rightModel_;
    const QString serial = side == StereoCameraSide::Left
        ? leftSerial_ : rightSerial_;
    return !model.isEmpty() && !serial.isEmpty() &&
           (profile->expectedCameraModel.isEmpty() ||
            profile->expectedCameraModel == model) &&
           (profile->expectedCameraSerial.isEmpty() ||
            profile->expectedCameraSerial == serial);
}

void StereoMapperWindow::appendLog(const QString& message) {
    logView_->append(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(
                 QStringLiteral("HH:mm:ss.zzz")), message));
}

void StereoMapperWindow::closeEvent(QCloseEvent* event) {
    if (mappingRequested_) stopMapping();
    QWidget::closeEvent(event);
}

}  // namespace hik_stereo
