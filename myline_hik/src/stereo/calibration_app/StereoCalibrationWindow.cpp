#include "stereo/calibration_app/StereoCalibrationWindow.h"

#include "FairinoRobotSession.h"
#include "ImageView.h"
#include "stereo/core/StereoDepthEngine.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QTextEdit>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hik_stereo {
namespace {

// FR5 actual_TCP_CmpSpeed[0] retains a small non-zero floor while the arm is
// physically stationary. Require a longer quiet window instead of using the
// controller noise floor as an unrealistically tight instantaneous limit.
constexpr double kStationaryLinearSpeedMmS = 1.0;
constexpr double kStationaryAngularSpeedDegS = 0.2;
constexpr int kRequiredStationarySamples = 15;
constexpr qint64 kCameraDeliveryGuardNs = 20000000LL;

}  // namespace

StereoCalibrationWindow::StereoCalibrationWindow(
        LineLaserController* laserController,
        FairinoRobotSession* robotSession,
        QWidget* parent)
    : QWidget(parent),
      laserController_(laserController),
      robotSession_(robotSession),
      sourceDirectory_(QStringLiteral(HIK_CALIBRATION_SOURCE_DIR)) {
    // Physical stereo convention (viewed from behind the sensor head):
    // left  = 1.6 MP / DA8784601 (stored in scanner_650),
    // right = 1.3 MP / DB0403208 (stored in scanner_450).
    // Do not infer stereo sides from the line-laser profile array order.
    leftProfile_ = findLineLaserDeviceProfile(QStringLiteral("scanner_650"));
    rightProfile_ = findLineLaserDeviceProfile(QStringLiteral("scanner_450"));
    if (!laserController_ || !robotSession_ ||
        !leftProfile_ || !rightProfile_) {
        throw std::runtime_error(
            "stereo calibration requires scanner_650 as left and scanner_450 as right");
    }
    qRegisterMetaType<hik_sync::RobotSample>("hik_sync::RobotSample");
    buildUi();
    appendLog(QStringLiteral(
        "物理双目顺序：左=160万 DA8784601（192.168.7.45），"
        "右=130万 DB0403208（192.168.1.46）。"));
    cameraRig_ = new StereoCameraRig(this);
    robotClientId_ = robotSession_->registerClient(
        QStringLiteral("stereo_calibration"));
    if (robotClientId_ <= 0 || !robotSession_->isRunning())
        throw std::runtime_error("unable to register stereo calibration FR5 client");

    connect(connectCamerasButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::connectCameras);
    connect(connectRobotButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::connectRobot);
    connect(startButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::startPreview);
    connect(stopButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::stopPreview);
    connect(captureButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::capturePair);
    connect(importSessionButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::importSession);
    connect(solveButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::solveCalibration);
    connect(approveButton_, &QPushButton::clicked,
            this, &StereoCalibrationWindow::approveCalibration);
    connect(cameraRig_, &StereoCameraRig::connectionChanged,
            this, [this](StereoCameraSide side, bool connected,
                         const QString& description) {
                if (side == StereoCameraSide::Left) {
                    leftConnected_ = connected;
                    leftStatus_->setText(description);
                } else {
                    rightConnected_ = connected;
                    rightStatus_->setText(description);
                }
                updateUi();
            });
    connect(cameraRig_, &StereoCameraRig::identityChanged,
            this, [this](StereoCameraSide side, const QString& model,
                         const QString& serial, const QString&) {
                if (side == StereoCameraSide::Left) {
                    leftModel_ = model;
                    leftSerial_ = serial;
                    leftStatus_->setText(QStringLiteral("%1｜SN %2").arg(model, serial));
                } else {
                    rightModel_ = model;
                    rightSerial_ = serial;
                    rightStatus_->setText(QStringLiteral("%1｜SN %2").arg(model, serial));
                }
                updateUi();
            });
    connect(cameraRig_, &StereoCameraRig::pairReady,
            this, &StereoCalibrationWindow::onPair);
    connect(cameraRig_, &StereoCameraRig::started, this, [this]() {
        previewActive_ = true;
        appendLog(QStringLiteral("双目同步预览已启动。"));
        updateUi();
    });
    connect(cameraRig_, &StereoCameraRig::stopped,
            this, [this](bool confirmed, const QString& description) {
                finishPreview(confirmed ? description
                    : QStringLiteral("至少一台相机停止未确认：%1").arg(description));
            });
    connect(cameraRig_, &StereoCameraRig::error,
            this, [this](const QString& message) {
                appendLog(QStringLiteral("相机错误：%1").arg(message));
                if (previewRequested_) stopPreview();
            });
    connect(cameraRig_, &StereoCameraRig::log,
            this, &StereoCalibrationWindow::appendLog);
    connect(robotSession_, &FairinoRobotSession::connectionChanged,
            this, [this](bool connected, const QString& description) {
                robotConnected_ = connected;
                robotStatus_->setText(description);
                updateUi();
            }, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::robotSampleReady,
            this, &StereoCalibrationWindow::onRobotSample,
            Qt::QueuedConnection);
    connect(laserController_, &LineLaserController::statusChanged,
            this, &StereoCalibrationWindow::onLaserStatus,
            Qt::QueuedConnection);

    watchdog_ = new QTimer(this);
    watchdog_->setInterval(250);
    connect(watchdog_, &QTimer::timeout, this, [this]() {
        const qint64 now = hik_sync::getMonotonicRawNs();
        if (waitingLaserOff_ && laserOffRequestedNs_ > 0 &&
            now - laserOffRequestedNs_ > 3000000000LL) {
            appendLog(QStringLiteral("3 s 内未收到 TTL LOW 因果 ACK，取消预览。"));
            stopPreview();
            return;
        }
        if (!previewActive_) return;
        updateCaptureReadiness();
        if (laserStatus_.sourceMonotonicNs <= 0 ||
            now - laserStatus_.sourceMonotonicNs > 1500000000LL) {
            appendLog(QStringLiteral("TTL 状态超时，停止双目标定预览。"));
            stopPreview();
        }
    });
    watchdog_->start();
    QString error;
    seedLoaded_ = loadIndependentCalibration(&error);
    if (!seedLoaded_) appendLog(QStringLiteral("正式内参/手眼加载失败：%1").arg(error));
    updateUi();
}

StereoCalibrationWindow::~StereoCalibrationWindow() {
    if (cameraRig_ && cameraRig_->running()) cameraRig_->stop();
    if (robotSession_->isExclusiveOwner(robotClientId_))
        robotSession_->releaseExclusive(robotClientId_);
    laserController_->off();
}

bool StereoCalibrationWindow::loadIndependentCalibration(QString* error) {
    const QByteArray leftIntrinsics =
        leftProfile_->intrinsicsConfigPath(sourceDirectory_).toLocal8Bit();
    const QByteArray leftHandEye =
        leftProfile_->handEyeConfigPath(sourceDirectory_).toLocal8Bit();
    const QByteArray rightIntrinsics =
        rightProfile_->intrinsicsConfigPath(sourceDirectory_).toLocal8Bit();
    const QByteArray rightHandEye =
        rightProfile_->handEyeConfigPath(sourceDirectory_).toLocal8Bit();
    std::string detail;
    const bool ok = loadStereoRigFromHandEye(
        leftIntrinsics.constData(), leftHandEye.constData(),
        rightIntrinsics.constData(), rightHandEye.constData(),
        &seedRig_, &detail);
    if (!ok && error) *error = QString::fromStdString(detail);
    return ok;
}

void StereoCalibrationWindow::connectCameras() {
    cameraRig_->connectCameras(leftIpEdit_->text(), rightIpEdit_->text());
}

void StereoCalibrationWindow::connectRobot() {
    robotSession_->connectRobot(robotClientId_, robotIpEdit_->text().trimmed());
}

void StereoCalibrationWindow::startPreview() {
    if (previewRequested_) return;
    if (!seedLoaded_ || !leftConnected_ || !rightConnected_ ||
        !robotConnected_ || !cameraIdentitiesAccepted()) {
        QMessageBox::warning(this, QStringLiteral("无法开始"),
            QStringLiteral("请先加载两台正式内参/手眼，连接正确的两台相机和 FR5。"));
        return;
    }
    QString leaseError;
    if (!robotSession_->acquireExclusive(
            robotClientId_, QStringLiteral("双目标定停稳校验"), &leaseError)) {
        QMessageBox::critical(this, QStringLiteral("FR5 被占用"), leaseError);
        return;
    }
    robotLeaseEpoch_ = robotSession_->exclusiveLeaseEpochFor(robotClientId_);
    sessionDirectory_ = QDir(sourceDirectory_).absoluteFilePath(
        QStringLiteral("data/calibration/stereo/session_%1")
            .arg(QDateTime::currentDateTime().toString(
                QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    if (!QDir().mkpath(sessionDirectory_)) {
        robotSession_->releaseExclusive(robotClientId_);
        QMessageBox::critical(this, QStringLiteral("无法开始"),
            QStringLiteral("无法创建双目标定 session 目录。"));
        return;
    }
    samples_.clear();
    boardRvecs_.clear();
    boardTvecs_.clear();
    result_ = StereoCalibrationResult();
    candidatePath_.clear();
    latestPair_ = StereoFramePair();
    latestRobotSample_ = hik_sync::RobotSample();
    stationaryRobotSamples_ = 0;
    stationaryConfirmedNs_ = 0;
    previewRequested_ = true;
    waitingLaserOff_ = true;
    laserOffToken_ = laserController_->requestOffTracked();
    laserOffRequestedNs_ = hik_sync::getMonotonicRawNs();
    appendLog(QStringLiteral("等待两路线激光 TTL LOW 后启动预览。"));
    updateUi();
}

void StereoCalibrationWindow::beginPreviewAfterLaserOff() {
    if (!previewRequested_ || !waitingLaserOff_) return;
    waitingLaserOff_ = false;
    cameraRig_->start(exposureSpin_->value(), exposureSpin_->value(),
                      gainSpin_->value(),
                      fpsSpin_->value(), pairSkewSpin_->value());
}

void StereoCalibrationWindow::stopPreview() {
    if (!previewRequested_) return;
    waitingLaserOff_ = false;
    laserController_->off();
    if (cameraRig_->running()) cameraRig_->stop();
    else finishPreview(QStringLiteral("双目标定预览已停止。"));
}

void StereoCalibrationWindow::finishPreview(const QString& description) {
    previewRequested_ = false;
    previewActive_ = false;
    waitingLaserOff_ = false;
    laserOffToken_ = 0;
    laserOffRequestedNs_ = 0;
    if (robotSession_->isExclusiveOwner(robotClientId_))
        robotSession_->releaseExclusive(robotClientId_);
    robotLeaseEpoch_ = 0;
    appendLog(description);
    updateUi();
}

void StereoCalibrationWindow::onPair(StereoFramePair pair) {
    if (!previewActive_) return;
    latestPair_ = std::move(pair);
    leftView_->setImage(bufferImage(latestPair_.left), true);
    rightView_->setImage(bufferImage(latestPair_.right), true);
}

void StereoCalibrationWindow::onRobotSample(
        int clientId, quint64 leaseEpoch,
        hik_sync::RobotSample sample) {
    if (!previewRequested_ || clientId != robotClientId_ ||
        leaseEpoch != robotLeaseEpoch_) return;
    double maximumAngular = 0.0;
    for (int index = 3; index < 6; ++index)
        maximumAngular = std::max(maximumAngular,
            std::abs(sample.actualTcpSpeed[static_cast<std::size_t>(index)]));
    const bool stationary = sample.valid &&
        std::abs(sample.actualLinearSpeedMmS) <= kStationaryLinearSpeedMmS &&
        maximumAngular <= kStationaryAngularSpeedDegS;
    if (stationary) {
        stationaryRobotSamples_ = std::min(1000, stationaryRobotSamples_ + 1);
        if (stationaryRobotSamples_ == kRequiredStationarySamples)
            stationaryConfirmedNs_ = sample.hostReceiveNs;
    } else {
        stationaryRobotSamples_ = 0;
        stationaryConfirmedNs_ = 0;
    }
    latestRobotSample_ = std::move(sample);
}

void StereoCalibrationWindow::capturePair() {
    if (!previewActive_) return;
    QString stationaryReason;
    if (!robotStationary(&stationaryReason)) {
        appendLog(QStringLiteral("采集拒绝：%1").arg(stationaryReason));
        QMessageBox::warning(this, QStringLiteral("机器人未停稳"), stationaryReason);
        return;
    }
    QString postStationaryReason;
    if (!pairWasCapturedAfterStationary(&postStationaryReason)) {
        appendLog(QStringLiteral("采集拒绝：%1").arg(postStationaryReason));
        QMessageBox::information(
            this, QStringLiteral("等待停稳后的新双目帧"), postStationaryReason);
        return;
    }
    const qint64 now = hik_sync::getMonotonicRawNs();
    if (latestPair_.midpointHostNs <= 0 ||
        now - latestPair_.midpointHostNs > 500000000LL) {
        appendLog(QStringLiteral("采集拒绝：最近双目配对帧超过 500 ms。"));
        QMessageBox::warning(this, QStringLiteral("没有新鲜双目帧"),
            QStringLiteral("最近双目配对帧超过 500 ms，请检查同步/网络。"));
        return;
    }
    cv::Mat left;
    cv::Mat right;
    QString imageError;
    if (!pairToGray(latestPair_, &left, &right, &imageError)) {
        QMessageBox::critical(this, QStringLiteral("图像无效"), imageError);
        return;
    }
    const QString id = QStringLiteral("stereo_%1")
        .arg(QDateTime::currentDateTime().toString(
            QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    StereoCalibrationSample sample;
    std::string detectionError;
    if (!detectStereoCharucoSample(
            left, right, id.toStdString(), seedRig_,
            &sample, &detectionError)) {
        const QString rejectedDirectory =
            QDir(sessionDirectory_).absoluteFilePath(QStringLiteral("rejected"));
        QDir().mkpath(rejectedDirectory);
        const QString rejectedLeft = QDir(rejectedDirectory).absoluteFilePath(
            id + QStringLiteral("_left.png"));
        const QString rejectedRight = QDir(rejectedDirectory).absoluteFilePath(
            id + QStringLiteral("_right.png"));
        cv::imwrite(rejectedLeft.toLocal8Bit().constData(), left);
        cv::imwrite(rejectedRight.toLocal8Bit().constData(), right);
        appendLog(QStringLiteral("采集拒绝：标定板检测失败：%1；原图保存在 %2")
            .arg(QString::fromStdString(detectionError), rejectedDirectory));
        QMessageBox::warning(this, QStringLiteral("标定板检测失败"),
            QStringLiteral("%1\n\n本次左右原图已保存到：\n%2")
                .arg(QString::fromStdString(detectionError), rejectedDirectory));
        return;
    }
    hik_calibration::BoardPoseOptions poseOptions;
    poseOptions.minCorners = 12;
    poseOptions.maxRmsErrorPx = 0.6;
    poseOptions.maxPointErrorPx = 1.5;
    hik_calibration::BoardPoseResult pose;
    if (!hik_calibration::estimateBoardPose(
            sample.left, seedRig_.left.intrinsics.board,
            seedRig_.left.intrinsics.cameraMatrix,
            seedRig_.left.intrinsics.distCoeffs,
            poseOptions, &pose)) {
        appendLog(QStringLiteral("采集拒绝：左相机板位姿无效：%1")
            .arg(QString::fromStdString(pose.error)));
        QMessageBox::warning(this, QStringLiteral("板位姿无效"),
            QString::fromStdString(pose.error));
        return;
    }
    if (!boardTvecs_.empty()) {
        const double translation = cv::norm(pose.tvec - boardTvecs_.back());
        cv::Mat currentRotation;
        cv::Mat previousRotation;
        cv::Rodrigues(pose.rvec, currentRotation);
        cv::Rodrigues(boardRvecs_.back(), previousRotation);
        const cv::Mat delta = previousRotation.t() * currentRotation;
        const double rotation = std::acos(std::max(-1.0, std::min(
            1.0, (cv::trace(delta)[0] - 1.0) * 0.5))) * 180.0 / CV_PI;
        if (translation < 15.0 && rotation < 4.0) {
            appendLog(QStringLiteral(
                "采集拒绝：与上一组仅平移 %1 mm、旋转 %2°。")
                .arg(translation, 0, 'f', 2).arg(rotation, 0, 'f', 2));
            QMessageBox::information(this, QStringLiteral("姿态过于相似"),
                QStringLiteral("请让相机相对标定板平移至少 15 mm 或旋转至少 4°。"));
            return;
        }
    }
    const QString leftPath = QDir(sessionDirectory_).absoluteFilePath(id + "_left.png");
    const QString rightPath = QDir(sessionDirectory_).absoluteFilePath(id + "_right.png");
    if (!cv::imwrite(leftPath.toLocal8Bit().constData(), left) ||
        !cv::imwrite(rightPath.toLocal8Bit().constData(), right)) {
        QMessageBox::critical(this, QStringLiteral("保存失败"),
            QStringLiteral("无法保存双目标定原图。"));
        return;
    }
    samples_.push_back(std::move(sample));
    boardRvecs_.push_back(pose.rvec);
    boardTvecs_.push_back(pose.tvec);
    sampleStatus_->setText(QStringLiteral(
        "已接受 %1 组｜最近帧偏差 %2 ms｜左板 RMS %3 px｜原图已保存")
        .arg(samples_.size()).arg(latestPair_.skewMs, 0, 'f', 2)
        .arg(pose.reprojection.rms, 0, 'f', 3));
    appendLog(sampleStatus_->text());
    updateUi();
}

void StereoCalibrationWindow::importSession() {
    if (previewRequested_ || !seedLoaded_) return;
    const QString calibrationRoot = QDir(sourceDirectory_).absoluteFilePath(
        QStringLiteral("data/calibration/stereo"));
    const QString selected = QFileDialog::getExistingDirectory(
        this, QStringLiteral("选择已有双目标定 session"), calibrationRoot,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selected.isEmpty()) return;
    QDir directory(selected);
    const QStringList leftFiles = directory.entryList(
        QStringList() << QStringLiteral("stereo_*_left.png"),
        QDir::Files, QDir::Name);
    if (leftFiles.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("无法加载"),
            QStringLiteral("所选目录没有 stereo_*_left.png。"));
        return;
    }

    std::vector<StereoCalibrationSample> importedSamples;
    std::vector<cv::Vec3d> importedRvecs;
    std::vector<cv::Vec3d> importedTvecs;
    int rejected = 0;
    QString firstError;
    for (const QString& leftFile : leftFiles) {
        QString id = leftFile;
        id.chop(QStringLiteral("_left.png").size());
        const QString rightFile = id + QStringLiteral("_right.png");
        const cv::Mat left = cv::imread(
            directory.absoluteFilePath(leftFile).toLocal8Bit().constData(),
            cv::IMREAD_GRAYSCALE);
        const cv::Mat right = cv::imread(
            directory.absoluteFilePath(rightFile).toLocal8Bit().constData(),
            cv::IMREAD_GRAYSCALE);
        StereoCalibrationSample sample;
        std::string detectionError;
        if (left.empty() || right.empty() ||
            !detectStereoCharucoSample(
                left, right, id.toStdString(), seedRig_,
                &sample, &detectionError)) {
            ++rejected;
            if (firstError.isEmpty()) {
                firstError = detectionError.empty()
                    ? QStringLiteral("左右图缺失或无法读取")
                    : QString::fromStdString(detectionError);
            }
            continue;
        }
        hik_calibration::BoardPoseOptions poseOptions;
        poseOptions.minCorners = 12;
        poseOptions.maxRmsErrorPx = 0.6;
        poseOptions.maxPointErrorPx = 1.5;
        hik_calibration::BoardPoseResult pose;
        if (!hik_calibration::estimateBoardPose(
                sample.left, seedRig_.left.intrinsics.board,
                seedRig_.left.intrinsics.cameraMatrix,
                seedRig_.left.intrinsics.distCoeffs,
                poseOptions, &pose)) {
            ++rejected;
            if (firstError.isEmpty())
                firstError = QString::fromStdString(pose.error);
            continue;
        }
        importedSamples.push_back(std::move(sample));
        importedRvecs.push_back(pose.rvec);
        importedTvecs.push_back(pose.tvec);
    }

    samples_ = std::move(importedSamples);
    boardRvecs_ = std::move(importedRvecs);
    boardTvecs_ = std::move(importedTvecs);
    result_ = StereoCalibrationResult();
    candidatePath_.clear();
    sessionDirectory_ = directory.absolutePath();
    sampleStatus_->setText(QStringLiteral(
        "已从 session 加载 %1 组，拒绝 %2 组｜%3")
        .arg(samples_.size()).arg(rejected).arg(sessionDirectory_));
    appendLog(sampleStatus_->text());
    if (!firstError.isEmpty())
        appendLog(QStringLiteral("首个导入拒绝原因：%1").arg(firstError));
    updateUi();
    QMessageBox::information(this, QStringLiteral("session 加载完成"),
        QStringLiteral("有效 %1 组，拒绝 %2 组。%3")
            .arg(samples_.size()).arg(rejected)
            .arg(samples_.size() >= 15U
                ? QStringLiteral("现在可以直接求解。")
                : QStringLiteral("有效组数不足 15，不能求解。")));
}

void StereoCalibrationWindow::solveCalibration() {
    if (previewRequested_ || samples_.size() < 15U) return;
    cv::Vec3d minimum = boardTvecs_.front();
    cv::Vec3d maximum = boardTvecs_.front();
    for (const cv::Vec3d& translation : boardTvecs_) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], translation[axis]);
            maximum[axis] = std::max(maximum[axis], translation[axis]);
        }
    }
    if (maximum[0] - minimum[0] < 50.0 ||
        maximum[1] - minimum[1] < 40.0 ||
        maximum[2] - minimum[2] < 100.0) {
        QMessageBox::warning(this, QStringLiteral("姿态覆盖不足"),
            QStringLiteral(
                "左相机观测到的标定板覆盖不足：需要 X≥50 mm、Y≥40 mm、深度≥100 mm。"
                "请重新开始并采集更丰富的近/中/远和画面边缘姿态。"));
        return;
    }
    StereoCalibrationOptions options;
    if (!calibrateStereoFixedIntrinsics(
            samples_, seedRig_, options, &result_)) {
        resultStatus_->setText(QStringLiteral("求解失败：%1")
            .arg(QString::fromStdString(result_.error)));
        updateUi();
        return;
    }
    StereoRigCalibration candidateRig = seedRig_;
    candidateRig.rotationRightFromLeft = result_.rotationRightFromLeft.clone();
    candidateRig.translationRightFromLeft = result_.translationRightFromLeft.clone();
    candidateRig.rightFromLeft = cv::Matx44d::eye();
    for (int row = 0; row < 3; ++row) {
        candidateRig.rightFromLeft(row, 3) =
            result_.translationRightFromLeft.at<double>(row);
        for (int column = 0; column < 3; ++column)
            candidateRig.rightFromLeft(row, column) =
                result_.rotationRightFromLeft.at<double>(row, column);
    }
    candidateRig.baselineMm = result_.baselineMm;
    candidateRig.relativeRotationDeg = result_.relativeRotationDeg;
    candidateRig.derivedFromIndependentHandEye = false;
    StereoDepthOptions depthOptions;
    depthOptions.minimumDepthMm = 450.0;
    depthOptions.maximumDepthMm = 3000.0;
    std::string geometryError;
    for (const cv::Size size : {cv::Size(612, 512), cv::Size(1224, 1024)}) {
        depthOptions.processingSize = size;
        StereoDepthEngine engine;
        if (!engine.configure(candidateRig, depthOptions, &geometryError)) {
            result_.passed = false;
            result_.error = "depth rectification validation failed: " + geometryError;
            break;
        }
    }
    resultStatus_->setText(QStringLiteral(
        "%1｜接受 %2/%3｜stereo RMS %4 px｜极线 RMS %5 px｜基线 %6 mm｜转角 %7°")
        .arg(result_.passed ? QStringLiteral("通过") : QStringLiteral("未通过"))
        .arg(result_.acceptedSamples).arg(result_.inputSamples)
        .arg(result_.stereoRmsPx, 0, 'f', 4)
        .arg(result_.epipolarRmsPx, 0, 'f', 4)
        .arg(result_.baselineMm, 0, 'f', 3)
        .arg(result_.relativeRotationDeg, 0, 'f', 3));
    if (!result_.error.empty())
        resultStatus_->setText(resultStatus_->text() + QStringLiteral("｜%1")
            .arg(QString::fromStdString(result_.error)));
    if (result_.passed) {
        candidatePath_ = QDir(sessionDirectory_).absoluteFilePath(
            QStringLiteral("hik_stereo_candidate.yaml"));
        std::string saveError;
        if (!saveStereoCalibrationYaml(
                candidatePath_.toLocal8Bit().constData(), result_, seedRig_,
                QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString(),
                &saveError)) {
            result_.passed = false;
            resultStatus_->setText(QStringLiteral("候选保存失败：%1")
                .arg(QString::fromStdString(saveError)));
        }
    }
    appendLog(resultStatus_->text());
    updateUi();
}

void StereoCalibrationWindow::approveCalibration() {
    if (!result_.ok || !result_.passed || candidatePath_.isEmpty()) return;
    if (QMessageBox::question(this, QStringLiteral("批准正式双目外参"),
            QStringLiteral("将原子更新 config/hik_stereo.yaml。确认两台相机和夹具此后不会移动？"))
        != QMessageBox::Yes) return;
    QFile candidate(candidatePath_);
    if (!candidate.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, QStringLiteral("批准失败"),
                              QStringLiteral("无法读取候选文件。"));
        return;
    }
    const QByteArray content = candidate.readAll();
    const QString formalPath = QDir(sourceDirectory_).absoluteFilePath(
        QStringLiteral("config/hik_stereo.yaml"));
    QSaveFile formal(formalPath);
    if (!formal.open(QIODevice::WriteOnly) || formal.write(content) != content.size() ||
        !formal.commit()) {
        QMessageBox::critical(this, QStringLiteral("批准失败"),
                              QStringLiteral("正式双目 YAML 原子写入失败。"));
        return;
    }
    appendLog(QStringLiteral("已批准正式双目外参：%1").arg(formalPath));
    QMessageBox::information(this, QStringLiteral("批准成功"),
        QStringLiteral("HikStereoMapper 现在可以加载正式双目 R/T。"));
}

void StereoCalibrationWindow::onLaserStatus(LineLaserStatus status) {
    laserStatus_ = status;
    const bool off = status.reachable && status.state == LineLaserState::Off &&
        !status.ttl450High && !status.ttl650High;
    laserStatusLabel_->setText(off ? QStringLiteral("TTL：两路线激光 LOW")
        : QStringLiteral("TTL：未确认关闭｜%1").arg(status.fault));
    if (previewActive_ && !off) {
        appendLog(QStringLiteral("预览期间激光状态不安全，停止采集。"));
        stopPreview();
        return;
    }
    if (waitingLaserOff_ && off && laserOffToken_ > 0 &&
        status.acknowledgedOffCommandToken >= laserOffToken_)
        beginPreviewAfterLaserOff();
}

bool StereoCalibrationWindow::cameraIdentitiesAccepted() const {
    return leftModel_ == leftProfile_->expectedCameraModel &&
        leftSerial_ == leftProfile_->expectedCameraSerial &&
        rightModel_ == rightProfile_->expectedCameraModel &&
        rightSerial_ == rightProfile_->expectedCameraSerial;
}

bool StereoCalibrationWindow::robotStationary(QString* reason) const {
    const qint64 now = hik_sync::getMonotonicRawNs();
    if (!latestRobotSample_.valid || latestRobotSample_.hostReceiveNs <= 0 ||
        now - latestRobotSample_.hostReceiveNs > 200000000LL) {
        if (reason) *reason = QStringLiteral("没有 200 ms 内的新鲜 FR5 20004 状态包。");
        return false;
    }
    double maximumAngular = 0.0;
    for (int index = 3; index < 6; ++index)
        maximumAngular = std::max(maximumAngular,
            std::abs(latestRobotSample_.actualTcpSpeed[static_cast<std::size_t>(index)]));
    if (std::abs(latestRobotSample_.actualLinearSpeedMmS) >
            kStationaryLinearSpeedMmS ||
        maximumAngular > kStationaryAngularSpeedDegS ||
        stationaryRobotSamples_ < kRequiredStationarySamples) {
        if (reason) *reason = QStringLiteral(
            "FR5 尚未连续 %1 个状态包停稳：线速度=%2 mm/s（要求≤%3），"
            "角速度分量最大=%4°/s（要求≤%5），连续包=%6。")
            .arg(kRequiredStationarySamples)
            .arg(latestRobotSample_.actualLinearSpeedMmS, 0, 'f', 3)
            .arg(kStationaryLinearSpeedMmS, 0, 'f', 1)
            .arg(maximumAngular, 0, 'f', 3)
            .arg(kStationaryAngularSpeedDegS, 0, 'f', 1)
            .arg(stationaryRobotSamples_);
        return false;
    }
    return true;
}

bool StereoCalibrationWindow::pairWasCapturedAfterStationary(
        QString* reason) const {
    if (stationaryConfirmedNs_ <= 0 ||
        latestPair_.left.hostCallbackNs <= 0 ||
        latestPair_.right.hostCallbackNs <= 0) {
        if (reason) *reason = QStringLiteral(
            "尚未收到连续停稳确认之后的新双目帧，请稍候。");
        return false;
    }
    const double maximumExposureUs = std::max(
        latestPair_.left.exposureUs, latestPair_.right.exposureUs);
    const qint64 requiredCallbackNs = stationaryConfirmedNs_ +
        static_cast<qint64>(std::llround(maximumExposureUs * 1000.0)) +
        kCameraDeliveryGuardNs;
    const qint64 earliestCallbackNs = std::min(
        latestPair_.left.hostCallbackNs,
        latestPair_.right.hostCallbackNs);
    if (earliestCallbackNs <= requiredCallbackNs) {
        if (reason) {
            const double remainingMs = static_cast<double>(
                requiredCallbackNs - earliestCallbackNs) / 1.0e6;
            *reason = QStringLiteral(
                "当前配对帧早于完整停稳窗口；等待下一组双目帧（尚差约 %1 ms）。")
                .arg(std::max(0.0, remainingMs), 0, 'f', 1);
        }
        return false;
    }
    return true;
}

bool StereoCalibrationWindow::pairToGray(
        const StereoFramePair& pair, cv::Mat* left,
        cv::Mat* right, QString* error) const {
    if (!left || !right || !pair.left.image || !pair.right.image) {
        if (error) *error = QStringLiteral("双目图像缓冲为空。");
        return false;
    }
    const auto convert = [](const hik_sync::ImageBuffer& buffer, cv::Mat* output) {
        if (buffer.width <= 0 || buffer.height <= 0 ||
            buffer.stride < buffer.width ||
            buffer.bytes.size() < static_cast<std::size_t>(buffer.stride) * buffer.height)
            return false;
        cv::Mat wrapped(buffer.height, buffer.width, CV_8UC1,
                        const_cast<std::uint8_t*>(buffer.bytes.data()),
                        static_cast<std::size_t>(buffer.stride));
        *output = wrapped.clone();
        return true;
    };
    if (!convert(*pair.left.image, left) || !convert(*pair.right.image, right)) {
        if (error) *error = QStringLiteral("双目图像尺寸或步长无效。");
        return false;
    }
    return true;
}

QImage StereoCalibrationWindow::bufferImage(const hik_sync::CameraFrame& frame) {
    if (!frame.image || frame.image->width <= 0 || frame.image->height <= 0 ||
        frame.image->stride < frame.image->width) return QImage();
    return QImage(frame.image->bytes.data(), frame.image->width,
                  frame.image->height, frame.image->stride,
                  QImage::Format_Grayscale8).copy();
}

void StereoCalibrationWindow::updateUi() {
    const bool idle = !previewRequested_;
    connectCamerasButton_->setEnabled(idle);
    connectRobotButton_->setEnabled(idle);
    startButton_->setEnabled(idle && seedLoaded_ && leftConnected_ &&
        rightConnected_ && robotConnected_ && cameraIdentitiesAccepted());
    stopButton_->setEnabled(previewRequested_);
    captureButton_->setEnabled(previewActive_);
    importSessionButton_->setEnabled(idle && seedLoaded_);
    solveButton_->setEnabled(idle && samples_.size() >= 15U);
    approveButton_->setEnabled(idle && result_.ok && result_.passed &&
                               !candidatePath_.isEmpty());
    exposureSpin_->setEnabled(idle);
    gainSpin_->setEnabled(idle);
    fpsSpin_->setEnabled(idle);
    pairSkewSpin_->setEnabled(idle);
    updateCaptureReadiness();
}

void StereoCalibrationWindow::updateCaptureReadiness() {
    if (!previewRequested_) {
        captureReadinessLabel_->setText(
            QStringLiteral("采集状态：请开始同步预览。"));
        return;
    }
    if (!previewActive_) {
        captureReadinessLabel_->setText(
            QStringLiteral("采集状态：正在等待 TTL LOW 和双相机启动。"));
        return;
    }
    QString reason;
    const bool stationary = robotStationary(&reason);
    const qint64 now = hik_sync::getMonotonicRawNs();
    const bool freshPair = latestPair_.midpointHostNs > 0 &&
        now - latestPair_.midpointHostNs <= 500000000LL;
    QString postStationaryReason;
    const bool postStationaryPair =
        pairWasCapturedAfterStationary(&postStationaryReason);
    if (!stationary) {
        captureReadinessLabel_->setText(
            QStringLiteral("采集状态：暂不可采集｜%1").arg(reason));
    } else if (!freshPair) {
        const StereoFramePairerStatistics statistics =
            cameraRig_->pairerStatistics();
        captureReadinessLabel_->setText(
            QStringLiteral(
                "采集状态：暂不可采集｜没有 500 ms 内的新鲜双目配对帧｜"
                "左帧 %1、右帧 %2、已配对 %3、左丢弃 %4、右丢弃 %5｜"
                "当前门槛≤%6 ms。")
                .arg(statistics.leftFrames)
                .arg(statistics.rightFrames)
                .arg(statistics.pairedFrames)
                .arg(statistics.leftDropped)
                .arg(statistics.rightDropped)
                .arg(pairSkewSpin_->value(), 0, 'f', 1));
    } else if (!postStationaryPair) {
        captureReadinessLabel_->setText(
            QStringLiteral("采集状态：暂不可采集｜%1")
                .arg(postStationaryReason));
    } else {
        captureReadinessLabel_->setText(QStringLiteral(
            "采集状态：可以采集｜FR5 已连续停稳｜帧偏差 %1 ms｜"
            "曝光 左 %2 / 右 %3 us")
            .arg(latestPair_.skewMs, 0, 'f', 2)
            .arg(latestPair_.left.exposureUs, 0, 'f', 0)
            .arg(latestPair_.right.exposureUs, 0, 'f', 0));
    }
}

void StereoCalibrationWindow::appendLog(const QString& message) {
    logView_->append(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(
                 QStringLiteral("HH:mm:ss.zzz")), message));
}

}  // namespace hik_stereo
