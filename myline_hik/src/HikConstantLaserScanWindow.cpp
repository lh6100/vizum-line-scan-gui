#include "HikConstantLaserScanWindow.h"

#include "FairinoReadOnlyWorker.h"
#include "HandEyeCalibrationCore.h"
#include "HikCameraWorker.h"
#include "ImageView.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QCloseEvent>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#ifndef HIK_CALIBRATION_SOURCE_DIR
#define HIK_CALIBRATION_SOURCE_DIR "."
#endif

namespace {

const double kStillTranslationMm = 0.10;
const double kStillRotationDeg = 0.05;

bool validIpv4(const QString& value) {
    const QStringList parts = value.trimmed().split(QLatin1Char('.'));
    if (parts.size() != 4) return false;
    for (int index = 0; index < 4; ++index) {
        bool ok = false;
        const int number = parts[index].toInt(&ok);
        if (!ok || number < 0 || number > 255 || parts[index].isEmpty()) return false;
    }
    return true;
}

QString sha256File(const QString& path, QString* error = nullptr) {
    if (error) error->clear();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) *error = QStringLiteral("无法读取文件: %1").arg(path);
        return QString();
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while (!file.atEnd()) {
        const QByteArray data = file.read(1024 * 1024);
        if (data.isEmpty() && file.error() != QFileDevice::NoError) {
            if (error) *error = QStringLiteral("读取文件失败: %1").arg(path);
            return QString();
        }
        hash.addData(data);
    }
    return QString::fromLatin1(hash.result().toHex());
}

QString poseText(const hik_scan::Pose6D& pose) {
    return QStringLiteral("XYZ=[%1, %2, %3] mm, RPY=[%4, %5, %6] deg")
        .arg(pose.x, 0, 'f', 3).arg(pose.y, 0, 'f', 3).arg(pose.z, 0, 'f', 3)
        .arg(pose.rx, 0, 'f', 3).arg(pose.ry, 0, 'f', 3).arg(pose.rz, 0, 'f', 3);
}

QString uniqueSession(const QString& root, const QString& prefix = QStringLiteral("scan")) {
    const QString base = QStringLiteral("%1_%2").arg(
        prefix,
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    for (int index = 0; index < 1000; ++index) {
        const QString name = index == 0 ? base
            : base + QStringLiteral("_%1").arg(index, 3, 10, QLatin1Char('0'));
        const QString path = QDir(root).absoluteFilePath(name);
        if (!QFileInfo::exists(path)) return path;
    }
    return QString();
}

QByteArray csvQuoted(const QString& text) {
    QString escaped = text;
    escaped.replace(QLatin1Char('"'), QStringLiteral("\"\""));
    return QStringLiteral("\"%1\"").arg(escaped).toUtf8();
}

}  // namespace

HikConstantLaserScanWindow::HikConstantLaserScanWindow(
        QWidget* parent, double scanSpeedOverrideMmS)
    : QMainWindow(parent),
      sourceDir_(QDir::cleanPath(QString::fromUtf8(HIK_CALIBRATION_SOURCE_DIR))) {
    configDir_ = QDir(sourceDir_).absoluteFilePath(QStringLiteral("config"));
    synchronizationConfigPath_ = QDir(configDir_).absoluteFilePath(
        QStringLiteral("synchronization.yaml"));
    std::string synchronizationError;
    synchronizationConfigReady_ = hik_sync::SynchronizationConfig::loadYaml(
        localPath(synchronizationConfigPath_), &synchronizationConfig_,
        &synchronizationError);
    if (synchronizationConfigReady_ && scanSpeedOverrideMmS >= 0.0) {
        synchronizationConfig_.scanSpeedMmS = scanSpeedOverrideMmS;
        synchronizationConfigReady_ = synchronizationConfig_.validate(
            &synchronizationError);
    }
    profileOptions_.reconstruction.stripe.minimumDifference = 10;
    profileOptions_.reconstruction.stripe.thresholdStddevScale = 2.0;
    profileOptions_.reconstruction.stripe.minPointCount = 80;
    profileOptions_.reconstruction.minReconstructedPoints = 80;
    profileOptions_.reconstruction.maxLineRmsMm = 0.50;
    buildUi();
    setupWorkers();
    if (synchronizationConfigReady_) {
        appendLog(QStringLiteral(
            "同步配置已加载：%1；目标=%2 fps，曝光=%3 us，扫描速度=%4 mm/s，机器人周期=%5 ms。")
            .arg(synchronizationConfigPath_)
            .arg(synchronizationConfig_.cameraTargetFps, 0, 'f', 3)
            .arg(synchronizationConfig_.cameraExposureUs, 0, 'f', 3)
            .arg(synchronizationConfig_.scanSpeedMmS, 0, 'f', 3)
            .arg(synchronizationConfig_.robotPeriodMs, 0, 'f', 3));
    } else {
        appendLog(QStringLiteral("同步配置不可用：%1")
            .arg(QString::fromStdString(synchronizationError)));
    }
    QString error;
    if (loadFormalCalibration(&error)) {
        appendLog(QStringLiteral("正式内参、激光平面和手眼标定已校验。"));
    } else {
        appendLog(QStringLiteral("正式标定未就绪: %1").arg(error));
    }
    appendLog(QStringLiteral(
        "常亮模式使用单帧形态学背景抑制；原图始终保存，错误条纹必须通过预览人工复核。"));
    updateUi();
}

HikConstantLaserScanWindow::~HikConstantLaserScanWindow() {
    shutdownWorkers();
}

void HikConstantLaserScanWindow::buildUi() {
    setWindowTitle(QStringLiteral("FR5 海康常亮线激光停稳扫描验证"));
    resize(1620, 960);
    QWidget* central = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(central);

    QLabel* warning = new QLabel(QStringLiteral(
        "验证流程会真实发送 FR5 MoveL。默认 dry-run；真运动前必须确认控制器已使能/自动模式、路径无碰撞、速度足够低且人员守在物理急停旁。"
        "本页软件停止不是控制柜物理急停。线激光保持常亮。"), central);
    warning->setWordWrap(true);
    warning->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    root->addWidget(warning);

    QGroupBox* devices = new QGroupBox(QStringLiteral("设备连接"), central);
    QGridLayout* deviceLayout = new QGridLayout(devices);
    cameraIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.56"), devices);
    exposureSpin_ = new QDoubleSpinBox(devices);
    exposureSpin_->setRange(1.0, 10000000.0);
    exposureSpin_->setDecimals(3);
    exposureSpin_->setValue(synchronizationConfig_.cameraExposureUs);
    exposureSpin_->setSuffix(QStringLiteral(" us"));
    gainSpin_ = new QDoubleSpinBox(devices);
    gainSpin_->setRange(0.0, 48.0);
    gainSpin_->setDecimals(3);
    gainSpin_->setValue(0.0);
    gainSpin_->setSuffix(QStringLiteral(" dB"));
    cameraTimeoutSpin_ = new QSpinBox(devices);
    cameraTimeoutSpin_->setRange(100, 10000);
    cameraTimeoutSpin_->setValue(3000);
    connectCameraButton_ = new QPushButton(QStringLiteral("连接相机"), devices);
    disconnectCameraButton_ = new QPushButton(QStringLiteral("断开相机"), devices);
    cameraStatusLabel_ = new QLabel(QStringLiteral("相机未连接"), devices);

    robotIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.200"), devices);
    connectRobotButton_ = new QPushButton(QStringLiteral("连接 FR5"), devices);
    disconnectRobotButton_ = new QPushButton(QStringLiteral("断开 FR5"), devices);
    readPoseButton_ = new QPushButton(QStringLiteral("读取法兰"), devices);
    robotStatusLabel_ = new QLabel(QStringLiteral("FR5 未连接"), devices);
    currentPoseLabel_ = new QLabel(QStringLiteral("当前法兰: -"), devices);
    currentPoseLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);

    deviceLayout->addWidget(new QLabel(QStringLiteral("相机 IP"), devices), 0, 0);
    deviceLayout->addWidget(cameraIpEdit_, 0, 1);
    deviceLayout->addWidget(new QLabel(QStringLiteral("曝光"), devices), 0, 2);
    deviceLayout->addWidget(exposureSpin_, 0, 3);
    deviceLayout->addWidget(new QLabel(QStringLiteral("增益"), devices), 0, 4);
    deviceLayout->addWidget(gainSpin_, 0, 5);
    deviceLayout->addWidget(new QLabel(QStringLiteral("超时 ms"), devices), 0, 6);
    deviceLayout->addWidget(cameraTimeoutSpin_, 0, 7);
    deviceLayout->addWidget(connectCameraButton_, 0, 8);
    deviceLayout->addWidget(disconnectCameraButton_, 0, 9);
    deviceLayout->addWidget(cameraStatusLabel_, 0, 10);
    deviceLayout->addWidget(new QLabel(QStringLiteral("机器人 IP"), devices), 1, 0);
    deviceLayout->addWidget(robotIpEdit_, 1, 1);
    deviceLayout->addWidget(connectRobotButton_, 1, 2);
    deviceLayout->addWidget(disconnectRobotButton_, 1, 3);
    deviceLayout->addWidget(readPoseButton_, 1, 4);
    deviceLayout->addWidget(robotStatusLabel_, 1, 5, 1, 2);
    deviceLayout->addWidget(currentPoseLabel_, 1, 7, 1, 4);
    deviceLayout->setColumnStretch(1, 1);
    root->addWidget(devices);

    QGroupBox* calibration = new QGroupBox(QStringLiteral("正式标定"), central);
    QHBoxLayout* calibrationLayout = new QHBoxLayout(calibration);
    reloadCalibrationButton_ = new QPushButton(QStringLiteral("重新加载 config"), calibration);
    calibrationStatusLabel_ = new QLabel(QStringLiteral("尚未加载"), calibration);
    calibrationStatusLabel_->setWordWrap(true);
    calibrationLayout->addWidget(reloadCalibrationButton_);
    calibrationLayout->addWidget(calibrationStatusLabel_, 1);
    root->addWidget(calibration);

    QGroupBox* path = new QGroupBox(QStringLiteral("示教直线扫描路径（姿态锁定为起点 RPY）"), central);
    QGridLayout* pathLayout = new QGridLayout(path);
    teachStartButton_ = new QPushButton(QStringLiteral("读取并设为起点"), path);
    teachEndButton_ = new QPushButton(QStringLiteral("读取并设为终点"), path);
    editStartButton_ = new QPushButton(QStringLiteral("编辑起点数值"), path);
    editEndButton_ = new QPushButton(QStringLiteral("编辑终点数值"), path);
    startPoseLabel_ = new QLabel(QStringLiteral("起点: -"), path);
    endPoseLabel_ = new QLabel(QStringLiteral("终点: -"), path);
    stepSpin_ = new QDoubleSpinBox(path);
    stepSpin_->setRange(0.5, 20.0);
    stepSpin_->setDecimals(2);
    stepSpin_->setValue(2.0);
    stepSpin_->setSuffix(QStringLiteral(" mm"));
    velocitySpin_ = new QDoubleSpinBox(path);
    velocitySpin_->setRange(1.0, 20.0);
    velocitySpin_->setValue(5.0);
    velocitySpin_->setSuffix(QStringLiteral(" %"));
    scanSpeedSpin_ = new QDoubleSpinBox(path);
    scanSpeedSpin_->setRange(10.0, 50.0);
    scanSpeedSpin_->setDecimals(2);
    scanSpeedSpin_->setSingleStep(10.0);
    scanSpeedSpin_->setValue(synchronizationConfig_.scanSpeedMmS);
    scanSpeedSpin_->setSuffix(QStringLiteral(" mm/s"));
    scanSpeedSpin_->setToolTip(QStringLiteral(
        "连续起点→终点段使用 FAIRINO MoveL velAccParamMode=1，按物理速度 mm/s 下发；"
        "同时用于实际 TCP 速度质量判定。"));
    accelerationSpin_ = new QDoubleSpinBox(path);
    accelerationSpin_->setRange(1.0, 50.0);
    accelerationSpin_->setValue(20.0);
    accelerationSpin_->setSuffix(QStringLiteral(" %"));
    settleSpin_ = new QSpinBox(path);
    settleSpin_->setRange(100, 2000);
    settleSpin_->setValue(250);
    settleSpin_->setSuffix(QStringLiteral(" ms"));
    motionTimeoutSpin_ = new QSpinBox(path);
    motionTimeoutSpin_->setRange(5, 300);
    motionTimeoutSpin_->setValue(120);
    motionTimeoutSpin_->setSuffix(QStringLiteral(" s"));
    voxelSpin_ = new QDoubleSpinBox(path);
    voxelSpin_->setRange(0.0, 10.0);
    voxelSpin_->setDecimals(2);
    voxelSpin_->setValue(0.5);
    voxelSpin_->setSuffix(QStringLiteral(" mm"));
    flatTargetGateCheck_ = new QCheckBox(
        QStringLiteral("标准平板模式：直线 RMS 超限时停止"), path);
    flatTargetGateCheck_->setChecked(false);
    lineRmsLimitSpin_ = new QDoubleSpinBox(path);
    lineRmsLimitSpin_->setRange(0.05, 5.0);
    lineRmsLimitSpin_->setDecimals(3);
    lineRmsLimitSpin_->setValue(0.5);
    lineRmsLimitSpin_->setSuffix(QStringLiteral(" mm"));
    pathLengthLimitCheck_ = new QCheckBox(
        QStringLiteral("启用验证路径长度保护"), path);
    pathLengthLimitCheck_->setChecked(true);
    pathLengthLimitSpin_ = new QDoubleSpinBox(path);
    pathLengthLimitSpin_->setRange(20.0, 1000.0);
    pathLengthLimitSpin_->setDecimals(1);
    pathLengthLimitSpin_->setValue(100.0);
    pathLengthLimitSpin_->setSuffix(QStringLiteral(" mm"));
    targetCountLimitCheck_ = new QCheckBox(
        QStringLiteral("启用目标数量保护"), path);
    targetCountLimitCheck_->setChecked(true);
    targetCountLimitSpin_ = new QSpinBox(path);
    targetCountLimitSpin_->setRange(2, 10000);
    targetCountLimitSpin_->setValue(201);
    targetCountLimitSpin_->setSuffix(QStringLiteral(" 个"));
    pathLayout->addWidget(teachStartButton_, 0, 0);
    pathLayout->addWidget(editStartButton_, 0, 1);
    pathLayout->addWidget(startPoseLabel_, 0, 2, 1, 4);
    pathLayout->addWidget(teachEndButton_, 1, 0);
    pathLayout->addWidget(editEndButton_, 1, 1);
    pathLayout->addWidget(endPoseLabel_, 1, 2, 1, 4);
    pathLayout->addWidget(new QLabel(QStringLiteral("步距"), path), 2, 0);
    pathLayout->addWidget(stepSpin_, 2, 1);
    pathLayout->addWidget(new QLabel(QStringLiteral("速度"), path), 2, 2);
    pathLayout->addWidget(velocitySpin_, 2, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("加速度"), path), 2, 4);
    pathLayout->addWidget(accelerationSpin_, 2, 5);
    pathLayout->addWidget(new QLabel(QStringLiteral("停稳等待"), path), 3, 0);
    pathLayout->addWidget(settleSpin_, 3, 1);
    pathLayout->addWidget(new QLabel(QStringLiteral("运动超时"), path), 3, 2);
    pathLayout->addWidget(motionTimeoutSpin_, 3, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("体素"), path), 3, 4);
    pathLayout->addWidget(voxelSpin_, 3, 5);
    pathLayout->addWidget(flatTargetGateCheck_, 4, 0, 1, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("直线 RMS 门槛"), path), 4, 3);
    pathLayout->addWidget(lineRmsLimitSpin_, 4, 4, 1, 2);
    pathLayout->addWidget(pathLengthLimitCheck_, 5, 0, 1, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("最大验证路径"), path), 5, 3);
    pathLayout->addWidget(pathLengthLimitSpin_, 5, 4, 1, 2);
    pathLayout->addWidget(targetCountLimitCheck_, 6, 0, 1, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("最大目标数量"), path), 6, 3);
    pathLayout->addWidget(targetCountLimitSpin_, 6, 4, 1, 2);
    pathLayout->addWidget(new QLabel(QStringLiteral("连续同步目标速度"), path), 7, 0);
    pathLayout->addWidget(scanSpeedSpin_, 7, 1);
    pathLayout->addWidget(new QLabel(
        QStringLiteral("连续段按物理速度下发；上方速度百分比只用于移到起点/原停稳模式"), path),
        7, 2, 1, 4);
    root->addWidget(path);

    QHBoxLayout* actions = new QHBoxLayout;
    dryRunCheck_ = new QCheckBox(QStringLiteral("dry-run（不运动、不拍照）"), central);
    dryRunCheck_->setChecked(true);
    safetyConfirmCheck_ = new QCheckBox(QStringLiteral("我确认真运动路径安全且物理急停可用"), central);
    dryRunButton_ = new QPushButton(QStringLiteral("生成并打印目标列表"), central);
    captureCurrentButton_ = new QPushButton(QStringLiteral("单点常亮验证（不移动）"), central);
    startScanButton_ = new QPushButton(QStringLiteral("开始停稳扫描"), central);
    startContinuousButton_ = new QPushButton(
        QStringLiteral("开始 60fps 连续同步扫描"), central);
    stopButton_ = new QPushButton(QStringLiteral("停止扫描 / StopMotion"), central);
    stopButton_->setStyleSheet(QStringLiteral("background:#b00020;color:white;font-weight:bold;"));
    actions->addWidget(dryRunCheck_);
    actions->addWidget(safetyConfirmCheck_);
    actions->addWidget(dryRunButton_);
    actions->addWidget(captureCurrentButton_);
    actions->addWidget(startScanButton_);
    actions->addWidget(startContinuousButton_);
    actions->addWidget(stopButton_);
    root->addLayout(actions);
    scanStatusLabel_ = new QLabel(QStringLiteral("等待操作。"), central);
    scanStatusLabel_->setWordWrap(true);
    root->addWidget(scanStatusLabel_);

    QSplitter* splitter = new QSplitter(Qt::Horizontal, central);
    profileTable_ = new QTableWidget(0, 11, splitter);
    profileTable_->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("序号") << QStringLiteral("图像") << QStringLiteral("点数")
        << QStringLiteral("Z最小") << QStringLiteral("Z最大") << QStringLiteral("直线RMS")
        << QStringLiteral("条纹饱和")
        << QStringLiteral("静止Δmm") << QStringLiteral("静止Δdeg")
        << QStringLiteral("法兰X") << QStringLiteral("法兰Y/Z"));
    profileTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    profileTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    profileTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    profileTable_->horizontalHeader()->setStretchLastSection(true);
    QWidget* preview = new QWidget(splitter);
    QVBoxLayout* previewLayout = new QVBoxLayout(preview);
    imageView_ = new ImageView(preview);
    imageView_->setEmptyText(QStringLiteral("暂无常亮激光图像"));
    logView_ = new QPlainTextEdit(preview);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(4000);
    logView_->setMinimumHeight(190);
    previewLayout->addWidget(imageView_, 1);
    previewLayout->addWidget(new QLabel(QStringLiteral("扫描日志"), preview));
    previewLayout->addWidget(logView_);
    splitter->addWidget(profileTable_);
    splitter->addWidget(preview);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 4);
    root->addWidget(splitter, 1);
    setCentralWidget(central);

    connect(connectCameraButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::connectCamera);
    connect(disconnectCameraButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::disconnectCamera);
    connect(connectRobotButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::connectRobot);
    connect(disconnectRobotButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::disconnectRobot);
    connect(readPoseButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::readCurrentPose);
    connect(teachStartButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::teachStart);
    connect(teachEndButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::teachEnd);
    connect(editStartButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::editStartPose);
    connect(editEndButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::editEndPose);
    connect(dryRunButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::generateDryRun);
    connect(captureCurrentButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::captureCurrentProfile);
    connect(startScanButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::startScan);
    connect(startContinuousButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::startContinuousScan);
    connect(stopButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::stopScan);
    connect(reloadCalibrationButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::reloadCalibration);
    connect(dryRunCheck_, &QCheckBox::toggled, this, [this](bool checked) {
        if (checked) safetyConfirmCheck_->setChecked(false);
        updateUi();
    });
    connect(flatTargetGateCheck_, &QCheckBox::toggled,
            this, [this](bool) { updateUi(); });
    connect(pathLengthLimitCheck_, &QCheckBox::toggled,
            this, [this](bool) { updateUi(); });
    connect(targetCountLimitCheck_, &QCheckBox::toggled,
            this, [this](bool) { updateUi(); });
}

void HikConstantLaserScanWindow::setupWorkers() {
    qRegisterMetaType<hik_sync::CameraFrame>("hik_sync::CameraFrame");
    qRegisterMetaType<hik_sync::RobotSample>("hik_sync::RobotSample");
    cameraWorker_ = new HikCameraWorker;
    cameraWorker_->moveToThread(&cameraThread_);
    connect(&cameraThread_, &QThread::finished, cameraWorker_, &QObject::deleteLater);
    connect(this, &HikConstantLaserScanWindow::requestConnectCamera,
            cameraWorker_, &HikCameraWorker::connectCamera, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestDisconnectCamera,
            cameraWorker_, &HikCameraWorker::disconnectCamera, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestCaptureSingle,
            cameraWorker_, &HikCameraWorker::captureSingle, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestStartContinuous,
            cameraWorker_, &HikCameraWorker::startContinuous, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestStopContinuous,
            cameraWorker_, &HikCameraWorker::stopContinuous, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::connectionChanged,
            this, &HikConstantLaserScanWindow::onCameraConnectionChanged, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::identityChanged,
            this, &HikConstantLaserScanWindow::onCameraIdentityChanged, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::busyChanged,
            this, &HikConstantLaserScanWindow::onCameraBusyChanged, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::frameReady,
            this, &HikConstantLaserScanWindow::onCameraFrameReady, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::log,
            this, &HikConstantLaserScanWindow::onCameraLog, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::error,
            this, &HikConstantLaserScanWindow::onCameraError, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::continuousStarted,
            this, &HikConstantLaserScanWindow::onContinuousCameraStarted,
            Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::continuousStopped,
            this, &HikConstantLaserScanWindow::onContinuousCameraStopped,
            Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::continuousFrameRejected,
            this, &HikConstantLaserScanWindow::onContinuousFrameRejected,
            Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::continuousFrameReady,
            this, [this](hik_sync::CameraFrame frame) {
                (void)synchronizationSession_.pushCamera(std::move(frame));
            }, Qt::DirectConnection);
    connect(cameraWorker_, &HikCameraWorker::imagePoolExhausted,
            this, [this]() {
                synchronizationSession_.noteImagePoolExhaustion();
            }, Qt::DirectConnection);
    cameraThread_.start();

    robotWorker_ = new FairinoReadOnlyWorker;
    robotWorker_->moveToThread(&robotThread_);
    connect(&robotThread_, &QThread::finished, robotWorker_, &QObject::deleteLater);
    connect(this, &HikConstantLaserScanWindow::requestConnectRobot,
            robotWorker_, &FairinoReadOnlyWorker::connectRobot, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestDisconnectRobot,
            robotWorker_, &FairinoReadOnlyWorker::disconnectRobot, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestReadFlangePose,
            robotWorker_, &FairinoReadOnlyWorker::readFlangePose, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestMoveLinear,
            robotWorker_, &FairinoReadOnlyWorker::moveLinear, Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestMoveLinearPhysical,
            robotWorker_, &FairinoReadOnlyWorker::moveLinearPhysical,
            Qt::QueuedConnection);
    connect(this, &HikConstantLaserScanWindow::requestStopMotion,
            robotWorker_, &FairinoReadOnlyWorker::stopMotion, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::connectionChanged,
            this, &HikConstantLaserScanWindow::onRobotConnectionChanged, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::busyChanged,
            this, &HikConstantLaserScanWindow::onRobotBusyChanged, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::flangePoseReady,
            this, &HikConstantLaserScanWindow::onRobotFlangePoseReady, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::motionStarted,
            this, &HikConstantLaserScanWindow::onRobotMotionStarted, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::motionFinished,
            this, &HikConstantLaserScanWindow::onRobotMotionFinished, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::log,
            this, &HikConstantLaserScanWindow::onRobotLog, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::error,
            this, &HikConstantLaserScanWindow::onRobotError, Qt::QueuedConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::robotSampleReady,
            this, [this](hik_sync::RobotSample sample) {
                (void)synchronizationSession_.pushRobot(std::move(sample));
            }, Qt::DirectConnection);
    connect(robotWorker_, &FairinoReadOnlyWorker::realtimePeriodConfigured,
            this, [this](int periodMs) {
                appendLog(QStringLiteral("FR5 20004 实时反馈周期确认：%1 ms。")
                          .arg(periodMs));
            }, Qt::QueuedConnection);
    robotThread_.start();
}

void HikConstantLaserScanWindow::shutdownWorkers() {
    if (shuttingDown_) return;
    shuttingDown_ = true;
    if (cameraWorker_ && cameraThread_.isRunning() &&
        continuousState_ != ContinuousState::Idle) {
        QMetaObject::invokeMethod(cameraWorker_, "stopContinuous",
                                  Qt::BlockingQueuedConnection);
    }
    synchronizationSession_.stop();
    continuousState_ = ContinuousState::Idle;
    if (robotWorker_ && robotThread_.isRunning()) {
        QMetaObject::invokeMethod(robotWorker_, "disconnectRobot", Qt::BlockingQueuedConnection);
        robotThread_.quit();
        robotThread_.wait();
    }
    robotWorker_ = nullptr;
    if (cameraWorker_ && cameraThread_.isRunning()) {
        QMetaObject::invokeMethod(cameraWorker_, "disconnectCamera", Qt::BlockingQueuedConnection);
        cameraThread_.quit();
        cameraThread_.wait();
    }
    cameraWorker_ = nullptr;
}

void HikConstantLaserScanWindow::closeEvent(QCloseEvent* event) {
    shutdownWorkers();
    event->accept();
}

void HikConstantLaserScanWindow::appendLog(const QString& message) {
    logView_->appendPlainText(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), message));
}

void HikConstantLaserScanWindow::showError(const QString& title, const QString& message) {
    appendLog(QStringLiteral("%1: %2").arg(title, message));
    if (!shuttingDown_) QMessageBox::warning(this, title, message);
}

void HikConstantLaserScanWindow::updateUi() {
    const bool idle = !shuttingDown_ && scanState_ == ScanState::Idle &&
                      continuousState_ == ContinuousState::Idle &&
                      pendingRobotRequestId_ < 0 && pendingCameraRequestId_ < 0;
    connectCameraButton_->setEnabled(idle && !cameraConnected_ && !cameraBusy_);
    disconnectCameraButton_->setEnabled(idle && cameraConnected_ && !cameraBusy_);
    cameraIpEdit_->setEnabled(idle && !cameraConnected_);
    exposureSpin_->setEnabled(idle && !cameraBusy_);
    gainSpin_->setEnabled(idle && !cameraBusy_);
    cameraTimeoutSpin_->setEnabled(idle && !cameraBusy_);
    connectRobotButton_->setEnabled(idle && !robotConnected_ && !robotBusy_);
    disconnectRobotButton_->setEnabled(idle && robotConnected_ && !robotBusy_);
    robotIpEdit_->setEnabled(idle && !robotConnected_);
    readPoseButton_->setEnabled(idle && robotConnected_ && !robotBusy_);
    teachStartButton_->setEnabled(idle && robotConnected_ && !robotBusy_);
    teachEndButton_->setEnabled(idle && robotConnected_ && !robotBusy_);
    editStartButton_->setEnabled(idle && startTaught_);
    editEndButton_->setEnabled(idle && endTaught_);
    reloadCalibrationButton_->setEnabled(idle);
    dryRunButton_->setEnabled(idle && startTaught_ && endTaught_);
    captureCurrentButton_->setEnabled(idle && cameraConnected_ && robotConnected_ && calibrationReady_);
    startScanButton_->setEnabled(idle && startTaught_ && endTaught_ &&
                                 (dryRunCheck_->isChecked() ||
                                  (cameraConnected_ && robotConnected_ && calibrationReady_)));
    startContinuousButton_->setEnabled(
        idle && synchronizationConfigReady_ && startTaught_ && endTaught_ &&
        cameraConnected_ && robotConnected_ && calibrationReady_);
    stopButton_->setEnabled(!idle && robotConnected_);
    safetyConfirmCheck_->setEnabled(idle && !dryRunCheck_->isChecked());
    stepSpin_->setEnabled(idle);
    velocitySpin_->setEnabled(idle);
    scanSpeedSpin_->setEnabled(idle && synchronizationConfigReady_);
    accelerationSpin_->setEnabled(idle);
    settleSpin_->setEnabled(idle);
    motionTimeoutSpin_->setEnabled(idle);
    voxelSpin_->setEnabled(idle);
    flatTargetGateCheck_->setEnabled(idle);
    lineRmsLimitSpin_->setEnabled(idle && flatTargetGateCheck_->isChecked());
    pathLengthLimitCheck_->setEnabled(idle);
    pathLengthLimitSpin_->setEnabled(idle && pathLengthLimitCheck_->isChecked());
    targetCountLimitCheck_->setEnabled(idle);
    targetCountLimitSpin_->setEnabled(idle && targetCountLimitCheck_->isChecked());
}

void HikConstantLaserScanWindow::connectCamera() {
    const QString ip = cameraIpEdit_->text().trimmed();
    if (!validIpv4(ip)) { showError(QStringLiteral("相机连接"), QStringLiteral("相机 IP 无效。")); return; }
    emit requestConnectCamera(ip);
}

void HikConstantLaserScanWindow::disconnectCamera() { emit requestDisconnectCamera(); }

void HikConstantLaserScanWindow::connectRobot() {
    const QString ip = robotIpEdit_->text().trimmed();
    if (!validIpv4(ip)) { showError(QStringLiteral("FR5 连接"), QStringLiteral("机器人 IP 无效。")); return; }
    emit requestConnectRobot(ip);
}

void HikConstantLaserScanWindow::disconnectRobot() { emit requestDisconnectRobot(); }

void HikConstantLaserScanWindow::readCurrentPose() { issueRobotRead(ReadRole::Manual); }
void HikConstantLaserScanWindow::teachStart() { issueRobotRead(ReadRole::TeachStart); }
void HikConstantLaserScanWindow::teachEnd() { issueRobotRead(ReadRole::TeachEnd); }

void HikConstantLaserScanWindow::editStartPose() {
    if (!startTaught_) return;
    const hik_scan::Pose6D before = startPose_;
    if (!editPoseDialog(&startPose_, QStringLiteral("编辑扫描起点"), false)) return;
    startPoseLabel_->setText(QStringLiteral("起点: %1").arg(poseText(startPose_)));
    appendLog(QStringLiteral("起点已手动修改：%1 → %2；请重新执行 dry-run。")
              .arg(poseText(before), poseText(startPose_)));
}

void HikConstantLaserScanWindow::editEndPose() {
    if (!endTaught_) return;
    const hik_scan::Pose6D before = endPose_;
    if (!editPoseDialog(&endPose_, QStringLiteral("编辑扫描终点"), true)) return;
    endPoseLabel_->setText(QStringLiteral("终点位置: %1（扫描时忽略终点 RPY）")
                           .arg(poseText(endPose_)));
    appendLog(QStringLiteral("终点已手动修改：%1 → %2；终点 RPY 不参与扫描，请重新执行 dry-run。")
              .arg(poseText(before), poseText(endPose_)));
}

bool HikConstantLaserScanWindow::editPoseDialog(
        hik_scan::Pose6D* pose, const QString& title, bool orientationIgnored) {
    if (!pose) return false;
    QDialog dialog(this);
    dialog.setWindowTitle(title);
    QVBoxLayout* root = new QVBoxLayout(&dialog);
    if (orientationIgnored) {
        QLabel* note = new QLabel(QStringLiteral(
            "终点 RPY 仅用于记录；生成扫描目标时，所有姿态锁定为起点 RPY。"), &dialog);
        note->setWordWrap(true);
        note->setStyleSheet(QStringLiteral("color:#b00020;"));
        root->addWidget(note);
    }
    QFormLayout* form = new QFormLayout;
    const QStringList names = QStringList()
        << QStringLiteral("X (mm)") << QStringLiteral("Y (mm)") << QStringLiteral("Z (mm)")
        << QStringLiteral("Rx (deg)") << QStringLiteral("Ry (deg)") << QStringLiteral("Rz (deg)");
    const double original[] = {pose->x, pose->y, pose->z, pose->rx, pose->ry, pose->rz};
    QDoubleSpinBox* fields[6] = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    for (int index = 0; index < 6; ++index) {
        fields[index] = new QDoubleSpinBox(&dialog);
        fields[index]->setRange(index < 3 ? -10000.0 : -360.0,
                                index < 3 ? 10000.0 : 360.0);
        fields[index]->setDecimals(3);
        fields[index]->setSingleStep(index < 3 ? 0.1 : 0.01);
        fields[index]->setValue(original[index]);
        form->addRow(names[index], fields[index]);
    }
    root->addLayout(form);
    QDialogButtonBox* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    root->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted) return false;
    pose->x = fields[0]->value(); pose->y = fields[1]->value(); pose->z = fields[2]->value();
    pose->rx = fields[3]->value(); pose->ry = fields[4]->value(); pose->rz = fields[5]->value();
    return true;
}

void HikConstantLaserScanWindow::reloadCalibration() {
    QString error;
    if (!loadFormalCalibration(&error)) showError(QStringLiteral("标定加载失败"), error);
    else appendLog(QStringLiteral("已重新加载三份正式标定。"));
    updateUi();
}

bool HikConstantLaserScanWindow::loadFormalCalibration(QString* error) {
    if (error) error->clear();
    calibrationReady_ = false;
    intrinsicsPath_ = QDir(configDir_).absoluteFilePath(QStringLiteral("hik_intrinsics.yaml"));
    laserPlanePath_ = QDir(configDir_).absoluteFilePath(QStringLiteral("hik_laser_plane.yaml"));
    handEyePath_ = QDir(configDir_).absoluteFilePath(QStringLiteral("hik_handeye.yaml"));
    std::string coreError;
    if (!hik_calibration::loadIntrinsicsYaml(localPath(intrinsicsPath_), &intrinsics_,
                                              &intrinsicsMetadata_, &coreError)) {
        if (error) *error = QStringLiteral("内参: %1").arg(QString::fromStdString(coreError));
        return false;
    }
    if (!hik_calibration::loadLaserPlaneYaml(localPath(laserPlanePath_), &laserPlane_,
                                              &laserBoard_, &laserMetadata_, &coreError)) {
        if (error) *error = QStringLiteral("激光平面: %1").arg(QString::fromStdString(coreError));
        return false;
    }
    if (!hik_scan::loadHandEyeYaml(localPath(handEyePath_), &handEye_, &coreError)) {
        if (error) *error = QStringLiteral("手眼: %1").arg(QString::fromStdString(coreError));
        return false;
    }
    QString hashError;
    intrinsicsSha256_ = sha256File(intrinsicsPath_, &hashError);
    laserPlaneSha256_ = sha256File(laserPlanePath_, &hashError);
    handEyeSha256_ = sha256File(handEyePath_, &hashError);
    if (intrinsicsSha256_.isEmpty() || laserPlaneSha256_.isEmpty() || handEyeSha256_.isEmpty()) {
        if (error) *error = hashError;
        return false;
    }
    const QString laserIntrinsicHash = QString::fromStdString(laserMetadata_.intrinsicsSha256);
    const QString handEyeIntrinsicHash = QString::fromStdString(handEye_.intrinsicsSha256);
    if (laserIntrinsicHash.compare(intrinsicsSha256_, Qt::CaseInsensitive) != 0 ||
        handEyeIntrinsicHash.compare(intrinsicsSha256_, Qt::CaseInsensitive) != 0) {
        if (error) *error = QStringLiteral("激光平面或手眼文件绑定的内参 SHA-256 与当前正式内参不一致。");
        return false;
    }
    const QString intrinsicFrame = QString::fromStdString(intrinsicsMetadata_.frameId).trimmed();
    const QString laserFrame = QString::fromStdString(laserMetadata_.cameraFrame).trimmed();
    const QString handEyeParent = QString::fromStdString(handEye_.parentFrame).trimmed();
    const QString handEyeChild = QString::fromStdString(handEye_.childFrame).trimmed();
    const QString intrinsicSerial = QString::fromStdString(intrinsicsMetadata_.cameraSerial).trimmed();
    const QString handEyeSerial = QString::fromStdString(handEye_.cameraSerial).trimmed();
    if (intrinsicFrame.isEmpty() || laserFrame != intrinsicFrame ||
        handEyeChild != intrinsicFrame ||
        handEyeParent != QStringLiteral("fairino_flange_reported")) {
        if (error) *error = QStringLiteral(
            "标定坐标系不一致：内参=%1，激光=%2，手眼=%3→%4；要求 fairino_flange_reported→内参相机坐标系。")
            .arg(intrinsicFrame, laserFrame, handEyeParent, handEyeChild);
        return false;
    }
    if (intrinsicSerial.isEmpty() || handEyeSerial != intrinsicSerial) {
        if (error) *error = QStringLiteral("内参与手眼相机序列号不一致：%1 / %2。")
            .arg(intrinsicSerial, handEyeSerial);
        return false;
    }
    profileOptions_.reconstruction.minimumDepthMm = laserMetadata_.validCameraZMinMm;
    profileOptions_.reconstruction.maximumDepthMm = laserMetadata_.validCameraZMaxMm;
    calibrationReady_ = true;
    calibrationStatusLabel_->setText(QStringLiteral(
        "已校验：相机 SN=%1，图像=%2×%3，Z=%4–%5 mm，输出 base_link；常亮背景核=%6×%7。")
        .arg(QString::fromStdString(intrinsicsMetadata_.cameraSerial))
        .arg(intrinsics_.imageSize.width).arg(intrinsics_.imageSize.height)
        .arg(profileOptions_.reconstruction.minimumDepthMm, 0, 'f', 2)
        .arg(profileOptions_.reconstruction.maximumDepthMm, 0, 'f', 2)
        .arg(profileOptions_.backgroundKernelWidth)
        .arg(profileOptions_.backgroundKernelHeight));
    calibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#087f23;"));
    return true;
}

bool HikConstantLaserScanWindow::formalCalibrationFilesUnchanged(QString* error) const {
    if (error) error->clear();
    QString hashError;
    const QString intrinsicsNow = sha256File(intrinsicsPath_, &hashError);
    if (intrinsicsNow.isEmpty()) { if (error) *error = hashError; return false; }
    const QString laserNow = sha256File(laserPlanePath_, &hashError);
    if (laserNow.isEmpty()) { if (error) *error = hashError; return false; }
    const QString handEyeNow = sha256File(handEyePath_, &hashError);
    if (handEyeNow.isEmpty()) { if (error) *error = hashError; return false; }
    if (intrinsicsNow.compare(intrinsicsSha256_, Qt::CaseInsensitive) != 0 ||
        laserNow.compare(laserPlaneSha256_, Qt::CaseInsensitive) != 0 ||
        handEyeNow.compare(handEyeSha256_, Qt::CaseInsensitive) != 0) {
        if (error) *error = QStringLiteral(
            "正式标定文件在加载后发生变化；请点击“重新加载 config”并重新确认路径。");
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::calibrationIdentityMatches(QString* error) const {
    const QString intrinsicSerial = QString::fromStdString(intrinsicsMetadata_.cameraSerial).trimmed();
    const QString handEyeSerial = QString::fromStdString(handEye_.cameraSerial).trimmed();
    if (cameraSerial_.isEmpty() || cameraSerial_ != intrinsicSerial || cameraSerial_ != handEyeSerial) {
        if (error) *error = QStringLiteral("当前相机 SN=%1，内参 SN=%2，手眼 SN=%3。")
            .arg(cameraSerial_, intrinsicSerial, handEyeSerial);
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::buildTargets(std::vector<hik_scan::Pose6D>* targets,
                                               QString* error) const {
    if (!startTaught_ || !endTaught_) {
        if (error) *error = QStringLiteral("请先示教起点和终点。");
        return false;
    }
    const double pathLength = cv::norm(cv::Vec3d(endPose_.x - startPose_.x,
                                                 endPose_.y - startPose_.y,
                                                 endPose_.z - startPose_.z));
    if (pathLengthLimitCheck_->isChecked() &&
        pathLength > pathLengthLimitSpin_->value()) {
        if (error) *error = QStringLiteral(
            "验证模式路径长度 %1 mm 超过当前 %2 mm 保护值；可提高保护值，或明确取消“启用验证路径长度保护”。")
            .arg(pathLength, 0, 'f', 3)
            .arg(pathLengthLimitSpin_->value(), 0, 'f', 1);
        return false;
    }
    const int requiredTargetCount =
        static_cast<int>(std::ceil(pathLength / stepSpin_->value())) + 1;
    if (targetCountLimitCheck_->isChecked() &&
        requiredTargetCount > targetCountLimitSpin_->value()) {
        if (error) *error = QStringLiteral(
            "路径需要 %1 个目标，超过当前 %2 个保护值；可提高“最大目标数量”，或明确取消“启用目标数量保护”。")
            .arg(requiredTargetCount).arg(targetCountLimitSpin_->value());
        return false;
    }
    std::string coreError;
    const int maximumPointCount = targetCountLimitCheck_->isChecked()
        ? targetCountLimitSpin_->value() : 100000;
    if (!hik_scan::buildLinearFlangePath(startPose_, endPose_, stepSpin_->value(),
                                          maximumPointCount, targets, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    return true;
}

void HikConstantLaserScanWindow::generateDryRun() {
    std::vector<hik_scan::Pose6D> targets;
    QString error;
    if (!buildTargets(&targets, &error)) { showError(QStringLiteral("路径生成失败"), error); return; }
    appendLog(QStringLiteral("dry-run：%1 个目标，姿态全部锁定为起点 RPY；未发送任何运动。")
              .arg(static_cast<int>(targets.size())));
    if (!pathLengthLimitCheck_->isChecked()) {
        appendLog(QStringLiteral("警告：验证路径长度保护已关闭；必须逐项核对目标和完整直线路径。"));
    }
    if (!targetCountLimitCheck_->isChecked()) {
        appendLog(QStringLiteral("警告：目标数量保护已关闭；大量停稳采集会显著增加扫描时间和文件数量。"));
    }
    for (std::size_t index = 0; index < targets.size(); ++index) {
        appendLog(QStringLiteral("目标 %1/%2: %3")
            .arg(static_cast<int>(index + 1U)).arg(static_cast<int>(targets.size()))
            .arg(poseText(targets[index])));
    }
}

void HikConstantLaserScanWindow::startScan() {
    QString error;
    profileOptions_.reconstruction.maxLineRmsMm = lineRmsLimitSpin_->value();
    if (!buildTargets(&targets_, &error)) { showError(QStringLiteral("路径生成失败"), error); return; }
    generateDryRun();
    if (dryRunCheck_->isChecked()) {
        scanStatusLabel_->setText(QStringLiteral("dry-run 完成，没有运动和拍照。取消 dry-run 后才能真扫描。"));
        return;
    }
    if (!cameraConnected_ || !robotConnected_ || !calibrationReady_) {
        showError(QStringLiteral("无法开始"), QStringLiteral("请连接相机、FR5并加载正式标定。"));
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error)) {
        showError(QStringLiteral("标定文件已变化"), error);
        return;
    }
    if (!calibrationIdentityMatches(&error)) { showError(QStringLiteral("相机身份不匹配"), error); return; }
    if (!safetyConfirmCheck_->isChecked()) {
        showError(QStringLiteral("真运动未授权"), QStringLiteral("请确认路径、控制器状态和物理急停后勾选安全确认。"));
        return;
    }
    const double pathLength = cv::norm(cv::Vec3d(endPose_.x - startPose_.x,
                                                 endPose_.y - startPose_.y,
                                                 endPose_.z - startPose_.z));
    const QString lengthProtection = pathLengthLimitCheck_->isChecked()
        ? QStringLiteral("开启（%1 mm）").arg(pathLengthLimitSpin_->value(), 0, 'f', 1)
        : QStringLiteral("已关闭");
    if (QMessageBox::warning(this, QStringLiteral("确认真实 FR5 扫描"),
        QStringLiteral("将真实移动 FR5 到 %1 个法兰目标。\n起点：%2\n终点位置：%3\n路径长度=%4 mm，长度保护=%5。\n速度=%6%，步距≈%7 mm。\n\n确认完整直线路径无碰撞并开始？")
            .arg(static_cast<int>(targets_.size())).arg(poseText(startPose_))
            .arg(poseText(endPose_)).arg(pathLength, 0, 'f', 3)
            .arg(lengthProtection).arg(velocitySpin_->value(), 0, 'f', 1)
            .arg(stepSpin_->value(), 0, 'f', 2),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
    if (!createScanSession(&error)) { showError(QStringLiteral("创建扫描会话失败"), error); return; }
    cloud_.clear();
    profileRows_.clear();
    refreshTable();
    currentTargetIndex_ = 0;
    singlePointMode_ = false;
    stopRequested_ = false;
    appendLog(QStringLiteral("真实停稳扫描开始，输出目录: %1").arg(scanSessionDir_));
    issueMoveForCurrentTarget();
}

void HikConstantLaserScanWindow::startContinuousScan() {
    QString error;
    if (!synchronizationConfigReady_) {
        showError(QStringLiteral("无法开始连续同步"),
                  QStringLiteral("同步配置未通过校验：%1")
                      .arg(synchronizationConfigPath_));
        return;
    }
    if (scanState_ != ScanState::Idle || continuousState_ != ContinuousState::Idle ||
        pendingMotionRequestId_ >= 0) {
        showError(QStringLiteral("无法开始连续同步"), QStringLiteral("当前已有任务在运行。"));
        return;
    }
    if (dryRunCheck_->isChecked()) {
        showError(QStringLiteral("无法开始连续同步"),
                  QStringLiteral("连续扫描会真实执行一次 MoveL；请先完成 dry-run，再取消 dry-run。"));
        return;
    }
    if (!cameraConnected_ || !robotConnected_ || !calibrationReady_ ||
        !startTaught_ || !endTaught_) {
        showError(QStringLiteral("无法开始连续同步"),
                  QStringLiteral("请连接相机/FR5、加载正式标定并示教起点和终点。"));
        return;
    }
    if (!safetyConfirmCheck_->isChecked()) {
        showError(QStringLiteral("真运动未授权"),
                  QStringLiteral("请确认路径、控制器状态和物理急停后勾选安全确认。"));
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error) ||
        !calibrationIdentityMatches(&error)) {
        showError(QStringLiteral("标定检查失败"), error);
        return;
    }
    synchronizationConfig_.scanSpeedMmS = scanSpeedSpin_->value();
    synchronizationConfig_.cameraExposureUs = exposureSpin_->value();
    std::string validationError;
    if (!synchronizationConfig_.validate(&validationError)) {
        showError(QStringLiteral("同步参数无效"),
                  QString::fromStdString(validationError));
        return;
    }
    const double pathLength = cv::norm(cv::Vec3d(
        endPose_.x - startPose_.x, endPose_.y - startPose_.y,
        endPose_.z - startPose_.z));
    if (pathLengthLimitCheck_->isChecked() &&
        pathLength > pathLengthLimitSpin_->value()) {
        showError(QStringLiteral("路径长度保护"),
                  QStringLiteral("连续路径 %1 mm 超过保护门槛 %2 mm。")
                      .arg(pathLength, 0, 'f', 3)
                      .arg(pathLengthLimitSpin_->value(), 0, 'f', 3));
        return;
    }
    const QString confirmation = QStringLiteral(
        "将先移动到起点，再以自由运行 %1 fps 采图，并执行一条起点→终点 MoveL。\n"
        "连续段使用 FAIRINO MoveL velAccParamMode=1：物理速度=%2 mm/s，物理加速度=%3 mm/s²。\n"
        "移到起点仍使用低速 %4%%；若连续段实际 TCP 速度不在目标 ±%5%%，"
        "图像仍保存但标为 SPEED_NOT_STABLE。\n路径长度=%6 mm。\n\n确认完整路径安全并开始？")
        .arg(synchronizationConfig_.cameraTargetFps, 0, 'f', 3)
        .arg(synchronizationConfig_.scanSpeedMmS, 0, 'f', 3)
        .arg(synchronizationConfig_.scanAccelerationMmS2, 0, 'f', 3)
        .arg(velocitySpin_->value(), 0, 'f', 2)
        .arg(synchronizationConfig_.scanSpeedTolerancePercent, 0, 'f', 2)
        .arg(pathLength, 0, 'f', 3);
    if (QMessageBox::warning(this, QStringLiteral("确认连续同步真实扫描"),
                             confirmation, QMessageBox::Yes | QMessageBox::No,
                             QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    continuousAbortRequested_ = false;
    synchronizationSessionDir_.clear();
    continuousState_ = ContinuousState::MovingToStart;
    pendingMotionRequestId_ = ++nextRobotRequestId_;
    scanStatusLabel_->setText(QStringLiteral("正在移动到连续扫描起点：%1")
                              .arg(poseText(startPose_)));
    appendLog(scanStatusLabel_->text());
    emit requestMoveLinear(pendingMotionRequestId_,
                           startPose_.x, startPose_.y, startPose_.z,
                           startPose_.rx, startPose_.ry, startPose_.rz,
                           velocitySpin_->value(), accelerationSpin_->value(),
                           motionTimeoutSpin_->value() * 1000);
    updateUi();
}

bool HikConstantLaserScanWindow::createSynchronizationSession(QString* error) {
    QString outputRoot = QString::fromStdString(
        synchronizationConfig_.outputDirectory);
    if (QDir::isRelativePath(outputRoot)) {
        outputRoot = QDir(sourceDir_).absoluteFilePath(outputRoot);
    }
    synchronizationSessionDir_ = uniqueSession(outputRoot, QStringLiteral("sync_scan"));
    if (synchronizationSessionDir_.isEmpty()) {
        if (error) *error = QStringLiteral("无法生成唯一同步会话目录。根目录=%1").arg(outputRoot);
        return false;
    }
    Eigen::Matrix4d flangeFromCamera = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            flangeFromCamera(row, column) = handEye_.flangeFromCamera(row, column);
        }
    }
    std::string coreError;
    if (!synchronizationSession_.start(
            synchronizationConfig_, localPath(synchronizationSessionDir_),
            &flangeFromCamera, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    const double spacing = synchronizationConfig_.scanSpeedMmS /
                           synchronizationConfig_.cameraTargetFps;
    const double motionDuringExposure = synchronizationConfig_.scanSpeedMmS *
        synchronizationConfig_.cameraExposureUs / 1.0e6;
    appendLog(QStringLiteral(
        "同步会话=%1；相机初始模式=HOST_CALLBACK_FALLBACK（设备映射稳定后自动切换）；"
        "机器人初始实际模式=HOST_RECEIVE、请求模式=%2（拟合稳定后自动切换）；"
        "理论线间距=%3 mm/帧；曝光运动量=%4 mm；PNG并行写线程=%5。")
        .arg(synchronizationSessionDir_)
        .arg(QString::fromLatin1(hik_sync::robotTimeModeName(
            synchronizationConfig_.robotTimeMode)))
        .arg(spacing, 0, 'f', 6)
        .arg(motionDuringExposure, 0, 'f', 6)
        .arg(synchronizationConfig_.imageWriterThreads));
    return true;
}

void HikConstantLaserScanWindow::onContinuousCameraStarted(
        double actualExposureUs, double actualFps,
        quint64 timestampFrequencyHz, QString timestampDescription) {
    if (continuousState_ != ContinuousState::StartingCamera) {
        if (continuousState_ == ContinuousState::Stopping) emit requestStopContinuous();
        return;
    }
    appendLog(QStringLiteral(
        "连续相机已就绪：actual exposure=%1 us, actual fps=%2, timestamp=%3, frequency=%4 Hz。")
        .arg(actualExposureUs, 0, 'f', 3).arg(actualFps, 0, 'f', 3)
        .arg(timestampDescription).arg(timestampFrequencyHz));
    QTimer::singleShot(100, this, [this]() {
        if (continuousState_ != ContinuousState::StartingCamera ||
            continuousAbortRequested_) return;
        continuousState_ = ContinuousState::Scanning;
        pendingMotionRequestId_ = ++nextRobotRequestId_;
        scanStatusLabel_->setText(QStringLiteral(
            "60fps 连续同步采集中：MoveL 到终点；加减速帧会保留并标记。"));
        appendLog(scanStatusLabel_->text());
        emit requestMoveLinearPhysical(
            pendingMotionRequestId_,
            endPose_.x, endPose_.y, endPose_.z,
            startPose_.rx, startPose_.ry, startPose_.rz,
            synchronizationConfig_.scanSpeedMmS,
            synchronizationConfig_.scanAccelerationMmS2,
            motionTimeoutSpin_->value() * 1000);
        updateUi();
    });
}

void HikConstantLaserScanWindow::onContinuousCameraStopped() {
    if (continuousState_ != ContinuousState::Stopping) return;
    QTimer::singleShot(60, this, [this]() {
        if (continuousState_ != ContinuousState::Stopping) return;
        finalizeContinuousScan(!continuousAbortRequested_,
            continuousAbortRequested_ ? QStringLiteral("扫描中止，已保存已采数据")
                                      : QStringLiteral("连续扫描完成"));
    });
}

void HikConstantLaserScanWindow::onContinuousFrameRejected(
        quint64 frameNo, QString reason) {
    appendLog(QStringLiteral("相机连续帧 %1 被拒绝：%2").arg(frameNo).arg(reason));
}

void HikConstantLaserScanWindow::abortContinuousScan(
        const QString& reason, bool requestStop) {
    if (continuousState_ == ContinuousState::Idle) return;
    appendLog(QStringLiteral("连续同步扫描终止请求：%1").arg(reason));
    continuousAbortRequested_ = true;
    const ContinuousState previous = continuousState_;
    if (requestStop && robotConnected_ && pendingMotionRequestId_ >= 0) {
        emit requestStopMotion(++nextRobotRequestId_);
    }
    if (previous == ContinuousState::StartingCamera ||
        previous == ContinuousState::Scanning ||
        previous == ContinuousState::Stopping) {
        continuousState_ = ContinuousState::Stopping;
        emit requestStopContinuous();
    } else if (previous == ContinuousState::MovingToStart &&
               (!requestStop || pendingMotionRequestId_ < 0)) {
        pendingMotionRequestId_ = -1;
        finalizeContinuousScan(false, reason);
    }
    scanStatusLabel_->setText(QStringLiteral("正在安全停止连续同步扫描：%1").arg(reason));
    updateUi();
}

void HikConstantLaserScanWindow::finalizeContinuousScan(
        bool completed, const QString& reason) {
    const bool sessionStarted = synchronizationSession_.running();
    if (sessionStarted) synchronizationSession_.stop();
    const hik_sync::PipelineStatistics stats = sessionStarted
        ? synchronizationSession_.statistics() : hik_sync::PipelineStatistics{};
    const QString mode = sessionStarted
        ? QString::fromStdString(synchronizationSession_.clockModeDescription())
        : QStringLiteral("同步会话未启动");
    continuousState_ = ContinuousState::Idle;
    pendingMotionRequestId_ = -1;
    safetyConfirmCheck_->setChecked(false);
    scanStatusLabel_->setText(QStringLiteral(
        "%1：frames=%2, frame_id_skip=%3, frame_cnt_skip=%4(仅诊断), valid=%5, invalid=%6, stable=%7；"
        "pool_exhaust=%8, image_queue_overflow=%9, images_written=%10, "
        "image_write_fail=%11, writer_overflow=%12；%13；目录=%14")
        .arg(completed ? QStringLiteral("连续同步完成") : QStringLiteral("连续同步已终止"))
        .arg(stats.totalCameraFrames).arg(stats.cameraFramesDropped)
        .arg(stats.robotFrameCounterSkips).arg(stats.successfulSyncFrames)
        .arg(stats.invalidSyncFrames).arg(stats.stableSpeedFrames)
        .arg(stats.imagePoolExhaustions).arg(stats.imageQueueOverflows)
        .arg(stats.imageFramesWritten).arg(stats.imageWriteFailures)
        .arg(stats.writerQueueOverflows).arg(mode, synchronizationSessionDir_));
    appendLog(QStringLiteral("%1；%2").arg(scanStatusLabel_->text(), reason));
    if (stats.actualCameraFps > 0.0 &&
        std::abs(stats.actualCameraFps - synchronizationConfig_.cameraTargetFps) >
            synchronizationConfig_.cameraTargetFps * 0.05) {
        appendLog(QStringLiteral(
            "警告：相机设备时间戳/FrameID 实测帧率=%1 fps，偏离目标 %2 fps 超过 5%。")
            .arg(stats.actualCameraFps, 0, 'f', 3)
            .arg(synchronizationConfig_.cameraTargetFps, 0, 'f', 3));
    }
    if (stats.acceptedCameraFps > 0.0 &&
        stats.acceptedCameraFps < synchronizationConfig_.cameraTargetFps * 0.95) {
        appendLog(QStringLiteral(
            "警告：软件接受帧率=%1 fps；请检查图像池/图像队列统计。")
            .arg(stats.acceptedCameraFps, 0, 'f', 3));
    }
    if (stats.maximumRobotGapMs > synchronizationConfig_.robotWarningGapMs) {
        appendLog(QStringLiteral(
            "警告：FR5 最大主机接收间隔=%1 ms，超过警告门槛 %2 ms；异常间隔=%3。")
            .arg(stats.maximumRobotGapMs, 0, 'f', 3)
            .arg(synchronizationConfig_.robotWarningGapMs, 0, 'f', 3)
            .arg(stats.robotAbnormalIntervals));
    }
    appendLog(QStringLiteral(
        "FR5 20004接收诊断：packet_sequence_gap=%1，frame_cnt_skip=%2，"
        "frame_cnt_duplicate=%3，frame_cnt_out_of_order=%4；"
        "getter mean=%5 us，P99=%6 us，max=%7 us，errors=%8。")
        .arg(stats.robotReceiveSequenceGaps)
        .arg(stats.robotFrameCounterSkips)
        .arg(stats.robotFrameCounterDuplicates)
        .arg(stats.robotFrameCounterOutOfOrder)
        .arg(stats.robotGetterMeanUs, 0, 'f', 3)
        .arg(stats.robotGetterP99Us, 0, 'f', 3)
        .arg(stats.robotGetterMaximumUs, 0, 'f', 3)
        .arg(stats.robotGetterErrors));
    const double requestedRobotHz = 1000.0 / synchronizationConfig_.robotPeriodMs;
    if (stats.actualRobotHz > 0.0 &&
        std::abs(stats.actualRobotHz - requestedRobotHz) > requestedRobotHz * 0.10) {
        appendLog(QStringLiteral(
            "警告：FR5 20004完整包实测=%1 Hz，与请求周期 %2 ms（%3 Hz）偏差超过 10%；"
            "到达时刻来自SDK RecvPkg成功返回处的 CLOCK_MONOTONIC_RAW；"
            "frame_cnt仅作控制器内部计数诊断。")
            .arg(stats.actualRobotHz, 0, 'f', 3)
            .arg(synchronizationConfig_.robotPeriodMs, 0, 'f', 3)
            .arg(requestedRobotHz, 0, 'f', 3));
    }
    continuousAbortRequested_ = false;
    updateUi();
}

void HikConstantLaserScanWindow::captureCurrentProfile() {
    QString error;
    profileOptions_.reconstruction.maxLineRmsMm = lineRmsLimitSpin_->value();
    if (!cameraConnected_ || !robotConnected_ || !calibrationReady_) {
        showError(QStringLiteral("无法单点采集"), QStringLiteral("请连接相机、FR5并加载正式标定。"));
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error)) {
        showError(QStringLiteral("标定文件已变化"), error);
        return;
    }
    if (!calibrationIdentityMatches(&error)) { showError(QStringLiteral("相机身份不匹配"), error); return; }
    if (!createScanSession(&error)) { showError(QStringLiteral("创建扫描会话失败"), error); return; }
    cloud_.clear();
    profileRows_.clear();
    refreshTable();
    targets_.clear();
    currentTargetIndex_ = 0;
    singlePointMode_ = true;
    stopRequested_ = false;
    scanState_ = ScanState::ReadingBefore;
    issueRobotRead(ReadRole::ScanBefore);
    appendLog(QStringLiteral("开始单点常亮验证，不发送机器人运动。"));
}

void HikConstantLaserScanWindow::stopScan() {
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("用户停止连续同步扫描。"),
                            pendingMotionRequestId_ >= 0);
        return;
    }
    if (scanState_ == ScanState::Idle) return;
    const bool moving = pendingMotionRequestId_ >= 0;
    abortScan(moving
        ? QStringLiteral("用户停止扫描；已向活动 MoveL 发送 StopMotion。")
        : QStringLiteral("用户停止扫描；当前无活动 MoveL，已立即终止采集状态机。"),
        moving);
}

void HikConstantLaserScanWindow::issueRobotRead(ReadRole role) {
    if (!robotConnected_ || pendingRobotRequestId_ >= 0) return;
    readRole_ = role;
    pendingRobotRequestId_ = ++nextRobotRequestId_;
    emit requestReadFlangePose(pendingRobotRequestId_);
    updateUi();
}

void HikConstantLaserScanWindow::issueMoveForCurrentTarget() {
    if (currentTargetIndex_ < 0 || currentTargetIndex_ >= static_cast<int>(targets_.size())) {
        abortScan(QStringLiteral("当前扫描目标索引无效。"), false);
        return;
    }
    const hik_scan::Pose6D& target = targets_[static_cast<std::size_t>(currentTargetIndex_)];
    scanState_ = ScanState::Moving;
    pendingMotionRequestId_ = ++nextRobotRequestId_;
    scanStatusLabel_->setText(QStringLiteral("移动到 %1/%2：%3")
        .arg(currentTargetIndex_ + 1).arg(static_cast<int>(targets_.size())).arg(poseText(target)));
    appendLog(scanStatusLabel_->text());
    emit requestMoveLinear(pendingMotionRequestId_, target.x, target.y, target.z,
                           target.rx, target.ry, target.rz,
                           velocitySpin_->value(), accelerationSpin_->value(),
                           motionTimeoutSpin_->value() * 1000);
    updateUi();
}

void HikConstantLaserScanWindow::beginSettledCapture() {
    if (scanState_ != ScanState::Settling || stopRequested_) {
        if (stopRequested_) abortScan(QStringLiteral("扫描已由用户停止。"), false);
        return;
    }
    scanState_ = ScanState::ReadingBefore;
    issueRobotRead(ReadRole::ScanBefore);
}

void HikConstantLaserScanWindow::onCameraConnectionChanged(bool connected, QString description) {
    cameraConnected_ = connected;
    cameraStatusLabel_->setText(description);
    cameraStatusLabel_->setStyleSheet(connected ? QStringLiteral("color:#087f23;") : QStringLiteral("color:#b00020;"));
    if (!connected && continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("连续同步扫描中相机断开。"), true);
    } else if (!connected && scanState_ != ScanState::Idle) {
        abortScan(QStringLiteral("扫描中相机断开。"), true);
    }
    updateUi();
}

void HikConstantLaserScanWindow::onCameraIdentityChanged(QString model, QString serial, QString ipAddress) {
    cameraModel_ = model; cameraSerial_ = serial; cameraIp_ = ipAddress;
    appendLog(QStringLiteral("相机身份：model=%1, SN=%2, IP=%3").arg(model, serial, ipAddress));
}

void HikConstantLaserScanWindow::onCameraBusyChanged(bool busy) { cameraBusy_ = busy; updateUi(); }
void HikConstantLaserScanWindow::onCameraLog(QString message) { appendLog(QStringLiteral("相机: %1").arg(message)); }

void HikConstantLaserScanWindow::onCameraError(int requestId, QString message) {
    if (requestId == pendingCameraRequestId_) pendingCameraRequestId_ = -1;
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("相机错误: %1").arg(message), true);
    } else if (scanState_ != ScanState::Idle) abortScan(QStringLiteral("相机错误: %1").arg(message), false);
    else showError(QStringLiteral("相机错误"), message);
    updateUi();
}

void HikConstantLaserScanWindow::onRobotConnectionChanged(bool connected, QString description) {
    robotConnected_ = connected;
    robotStatusLabel_->setText(description);
    robotStatusLabel_->setStyleSheet(connected ? QStringLiteral("color:#087f23;") : QStringLiteral("color:#b00020;"));
    if (!connected && continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("连续同步扫描中 FR5 断开。"), false);
    } else if (!connected && scanState_ != ScanState::Idle) {
        abortScan(QStringLiteral("扫描中 FR5 断开。"), false);
    }
    updateUi();
}

void HikConstantLaserScanWindow::onRobotBusyChanged(bool busy) { robotBusy_ = busy; updateUi(); }
void HikConstantLaserScanWindow::onRobotLog(QString message) { appendLog(QStringLiteral("FR5: %1").arg(message)); }

void HikConstantLaserScanWindow::onRobotError(int requestId, QString message) {
    if (requestId == pendingRobotRequestId_) { pendingRobotRequestId_ = -1; readRole_ = ReadRole::None; }
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("FR5 错误: %1").arg(message), true);
    } else if (scanState_ != ScanState::Idle) abortScan(QStringLiteral("FR5 错误: %1").arg(message), true);
    else showError(QStringLiteral("FR5 错误"), message);
    updateUi();
}

void HikConstantLaserScanWindow::onRobotFlangePoseReady(
        int requestId, double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg, qint64 hostTimestampMs) {
    if (requestId != pendingRobotRequestId_) return;
    pendingRobotRequestId_ = -1;
    PoseReading reading;
    reading.valid = true;
    reading.pose.x = xMm; reading.pose.y = yMm; reading.pose.z = zMm;
    reading.pose.rx = rxDeg; reading.pose.ry = ryDeg; reading.pose.rz = rzDeg;
    reading.baseFromFlange = hik_calibration::fairinoBaseFromFlange(
        xMm, yMm, zMm, rxDeg, ryDeg, rzDeg);
    reading.hostTimestampMs = hostTimestampMs;
    currentPose_ = reading;
    currentPoseLabel_->setText(QStringLiteral("当前法兰: %1").arg(poseText(reading.pose)));
    const ReadRole role = readRole_;
    readRole_ = ReadRole::None;
    if (role == ReadRole::TeachStart) {
        startPose_ = reading.pose; startTaught_ = true;
        startPoseLabel_->setText(QStringLiteral("起点: %1").arg(poseText(startPose_)));
        appendLog(QStringLiteral("已示教扫描起点。"));
    } else if (role == ReadRole::TeachEnd) {
        endPose_ = reading.pose; endTaught_ = true;
        endPoseLabel_->setText(QStringLiteral("终点位置: %1（扫描时忽略终点 RPY）").arg(poseText(endPose_)));
        appendLog(QStringLiteral("已示教扫描终点位置；扫描姿态仍使用起点 RPY。"));
    } else if (role == ReadRole::ScanBefore && scanState_ == ScanState::ReadingBefore) {
        beforePose_ = reading;
        scanState_ = ScanState::Capturing;
        pendingCameraRequestId_ = ++nextCameraRequestId_;
        emit requestCaptureSingle(pendingCameraRequestId_, exposureSpin_->value(),
                                  gainSpin_->value(), cameraTimeoutSpin_->value());
    } else if (role == ReadRole::ScanAfter && scanState_ == ScanState::ReadingAfter) {
        finishProfile(reading);
    }
    updateUi();
}

void HikConstantLaserScanWindow::onRobotMotionStarted(int requestId, QString description) {
    if (requestId == pendingMotionRequestId_) appendLog(QStringLiteral("FR5: %1").arg(description));
}

void HikConstantLaserScanWindow::onRobotMotionFinished(int requestId, bool success, QString description) {
    if (requestId != pendingMotionRequestId_) {
        if (stopRequested_ && !success) abortScan(description, false);
        return;
    }
    pendingMotionRequestId_ = -1;
    appendLog(QStringLiteral("FR5: %1").arg(description));
    if (continuousState_ == ContinuousState::MovingToStart) {
        if (!success || continuousAbortRequested_) {
            finalizeContinuousScan(false, description);
            return;
        }
        QString sessionError;
        if (!createSynchronizationSession(&sessionError)) {
            continuousState_ = ContinuousState::Idle;
            safetyConfirmCheck_->setChecked(false);
            showError(QStringLiteral("创建同步会话失败"), sessionError);
            updateUi();
            return;
        }
        continuousState_ = ContinuousState::StartingCamera;
        scanStatusLabel_->setText(QStringLiteral(
            "已到连续扫描起点；预采机器人状态后启动相机。"));
        QTimer::singleShot(100, this, [this]() {
            if (continuousState_ != ContinuousState::StartingCamera) return;
            emit requestStartContinuous(
                synchronizationConfig_.cameraExposureUs,
                gainSpin_->value(),
                synchronizationConfig_.cameraTargetFps,
                static_cast<int>(synchronizationConfig_.cameraQueueCapacity));
        });
        updateUi();
        return;
    }
    if (continuousState_ == ContinuousState::Scanning) {
        continuousState_ = ContinuousState::Stopping;
        continuousAbortRequested_ = !success || continuousAbortRequested_;
        emit requestStopContinuous();
        scanStatusLabel_->setText(success
            ? QStringLiteral("连续 MoveL 已完成，正在停止相机并清空同步/写入队列。")
            : QStringLiteral("连续 MoveL 失败，正在停止相机并保存已采数据。"));
        updateUi();
        return;
    }
    if (continuousState_ == ContinuousState::Stopping) {
        // StopMotion reports completion for the original active MoveL request.
        // Camera shutdown owns final queue draining in this state.
        return;
    }
    if (!success || stopRequested_) {
        abortScan(description, false);
        return;
    }
    scanState_ = ScanState::Settling;
    scanStatusLabel_->setText(QStringLiteral("已到位，等待结构停稳 %1 ms。")
                              .arg(settleSpin_->value()));
    QTimer::singleShot(settleSpin_->value(), this, &HikConstantLaserScanWindow::beginSettledCapture);
    updateUi();
}

void HikConstantLaserScanWindow::onCameraFrameReady(
        int requestId, QImage image, quint64 frameNo, quint64 deviceTimestamp,
        qint64 hostTimestamp, double actualExposure, double actualGain,
        QString description) {
    if (requestId != pendingCameraRequestId_ || scanState_ != ScanState::Capturing) return;
    pendingCameraRequestId_ = -1;
    const int profileIndex = static_cast<int>(profileRows_.size());
    const QString imagePath = QDir(imageDir_).absoluteFilePath(
        QStringLiteral("profile_%1_%2.png")
            .arg(profileIndex, 6, 10, QLatin1Char('0'))
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
    if (image.isNull() || !image.save(imagePath, "PNG")) {
        abortScan(QStringLiteral("无法保存相机原图: %1").arg(imagePath), false);
        return;
    }
    const cv::Mat gray = qImageToGray(image);
    hik_calibration::StaticProfileResult profile;
    const std::string id = std::string("scan_profile_") + std::to_string(profileIndex);
    if (!hik_calibration::reconstructSingleFrameProfile(
            gray, id, intrinsics_, laserPlane_, profileOptions_, &profile) || !profile.ok) {
        imageView_->setImage(profile.differenceImage.empty()
            ? image : cvMatToQImage(profile.differenceImage));
        abortScan(QStringLiteral("常亮单帧重建失败：%1；原图=%2")
                  .arg(QString::fromStdString(profile.error), imagePath), false);
        return;
    }
    int saturatedStripeCount = 0;
    for (std::size_t index = 0; index < profile.stripe.size(); ++index) {
        const int row = profile.stripe[index].row;
        const int column = profile.stripe[index].peakX;
        if (row >= 0 && row < gray.rows && column >= 0 && column < gray.cols &&
            gray.at<unsigned char>(row, column) >= 250) {
            ++saturatedStripeCount;
        }
    }
    pendingStripeSaturatedRatio_ = profile.stripe.empty() ? 0.0
        : static_cast<double>(saturatedStripeCount) /
          static_cast<double>(profile.stripe.size());
    if (pendingStripeSaturatedRatio_ > 0.30) {
        appendLog(QStringLiteral(
            "警告：%1% 的条纹中心达到灰度 250–255，亚像素中心可能受饱和平台影响；建议降低曝光或激光功率。")
            .arg(100.0 * pendingStripeSaturatedRatio_, 0, 'f', 1));
    }
    if (!profile.lineQualityPassed) {
        const QString message = QStringLiteral(
            "轮廓直线 RMS=%1 mm 超过参考门槛 %2 mm。该指标描述被测轮廓几何，不是通用条纹可信度。")
            .arg(profile.lineDistanceMm.rms, 0, 'f', 4)
            .arg(profileOptions_.reconstruction.maxLineRmsMm, 0, 'f', 4);
        if (flatTargetGateCheck_->isChecked()) {
            imageView_->setImage(drawStripeOverlay(gray, profile));
            abortScan(QStringLiteral("标准平板模式拒绝：%1 原图=%2")
                      .arg(message, imagePath), false);
            return;
        }
        appendLog(QStringLiteral("警告：%1 普通扫描模式继续累计。")
                  .arg(message));
    }
    pendingProfile_ = profile;
    pendingImagePath_ = imagePath;
    pendingFrameNo_ = frameNo;
    pendingDeviceTimestamp_ = deviceTimestamp;
    pendingCameraHostTimestamp_ = hostTimestamp;
    pendingExposureUs_ = actualExposure;
    pendingGainDb_ = actualGain;
    imageView_->setImage(drawStripeOverlay(gray, profile));
    appendLog(QStringLiteral("常亮轮廓提取通过：%1 点，Z=%2–%3 mm，line RMS=%4 mm，饱和=%5%，%6")
        .arg(static_cast<int>(profile.points.size()))
        .arg(profile.minimumDepthMm, 0, 'f', 3).arg(profile.maximumDepthMm, 0, 'f', 3)
        .arg(profile.lineDistanceMm.rms, 0, 'f', 4)
        .arg(100.0 * pendingStripeSaturatedRatio_, 0, 'f', 1).arg(description));
    scanState_ = ScanState::ReadingAfter;
    issueRobotRead(ReadRole::ScanAfter);
    updateUi();
}

void HikConstantLaserScanWindow::finishProfile(const PoseReading& after) {
    const double translationDelta = hik_calibration::rigidTranslationDistanceMm(
        beforePose_.baseFromFlange, after.baseFromFlange);
    const double rotationDelta = hik_calibration::rigidRotationDistanceDeg(
        beforePose_.baseFromFlange, after.baseFromFlange);
    if (translationDelta > kStillTranslationMm || rotationDelta > kStillRotationDeg) {
        abortScan(QStringLiteral("采图前后法兰变化 %1 mm/%2°，静止检查不通过。")
            .arg(translationDelta, 0, 'f', 4).arg(rotationDelta, 0, 'f', 4), false);
        return;
    }
    const cv::Matx44d baseFromFlange = hik_calibration::interpolateRigidHalf(
        beforePose_.baseFromFlange, after.baseFromFlange);
    const std::size_t oldCloudSize = cloud_.size();
    std::string coreError;
    const int profileIndex = static_cast<int>(profileRows_.size());
    if (!hik_scan::appendProfileInBase(pendingProfile_, baseFromFlange,
                                       handEye_.flangeFromCamera, profileIndex,
                                       &cloud_, &coreError)) {
        abortScan(QString::fromStdString(coreError), false);
        return;
    }
    ProfileRow row;
    row.index = profileIndex;
    row.imagePath = pendingImagePath_;
    row.before = beforePose_;
    row.after = after;
    row.cameraPointCount = static_cast<int>(pendingProfile_.points.size());
    row.minimumDepthMm = pendingProfile_.minimumDepthMm;
    row.maximumDepthMm = pendingProfile_.maximumDepthMm;
    row.lineRmsMm = pendingProfile_.lineDistanceMm.rms;
    row.lineRmsLimitMm = profileOptions_.reconstruction.maxLineRmsMm;
    row.flatTargetGate = flatTargetGateCheck_->isChecked();
    row.stripeSaturatedRatio = pendingStripeSaturatedRatio_;
    row.translationDeltaMm = translationDelta;
    row.rotationDeltaDeg = rotationDelta;
    QString error;
    if (!appendManifest(row, pendingFrameNo_, pendingDeviceTimestamp_,
                        pendingCameraHostTimestamp_, pendingExposureUs_,
                        pendingGainDb_, &error)) {
        cloud_.resize(oldCloudSize);
        abortScan(error, false);
        return;
    }
    profileRows_.push_back(row);
    refreshTable();
    scanStatusLabel_->setText(QStringLiteral("轮廓 %1 已累计，总点数=%2。")
        .arg(profileIndex).arg(static_cast<qulonglong>(cloud_.size())));
    continueOrFinish();
}

void HikConstantLaserScanWindow::continueOrFinish() {
    if (singlePointMode_ || currentTargetIndex_ + 1 >= static_cast<int>(targets_.size())) {
        QString error;
        if (!saveCloudOutputs(&error)) {
            abortScan(error, false);
            return;
        }
        scanState_ = ScanState::Idle;
        safetyConfirmCheck_->setChecked(false);
        scanStatusLabel_->setText(QStringLiteral("扫描完成：%1 条轮廓，%2 点。raw=%3，voxel=%4")
            .arg(static_cast<int>(profileRows_.size()))
            .arg(static_cast<qulonglong>(cloud_.size())).arg(rawPlyPath_, voxelPlyPath_));
        appendLog(scanStatusLabel_->text());
        updateUi();
        return;
    }
    ++currentTargetIndex_;
    issueMoveForCurrentTarget();
}

void HikConstantLaserScanWindow::abortScan(const QString& reason, bool requestStop) {
    appendLog(QStringLiteral("扫描终止: %1").arg(reason));
    stopRequested_ = true;
    if (requestStop && robotConnected_ && pendingMotionRequestId_ >= 0) {
        emit requestStopMotion(++nextRobotRequestId_);
    }
    QString saveError;
    if (!cloud_.empty()) saveCloudOutputs(&saveError);
    scanState_ = ScanState::Idle;
    pendingCameraRequestId_ = -1;
    pendingRobotRequestId_ = -1;
    pendingMotionRequestId_ = -1;
    readRole_ = ReadRole::None;
    safetyConfirmCheck_->setChecked(false);
    scanStatusLabel_->setText(QStringLiteral("扫描已终止：%1%2")
        .arg(reason, saveError.isEmpty() ? QString() : QStringLiteral("；部分 PLY 保存失败：") + saveError));
    updateUi();
}

bool HikConstantLaserScanWindow::createScanSession(QString* error) {
    const QString root = QDir(sourceDir_).absoluteFilePath(QStringLiteral("data/scans"));
    if (!QDir().mkpath(root)) { if (error) *error = QStringLiteral("无法创建 %1").arg(root); return false; }
    scanSessionDir_ = uniqueSession(root);
    imageDir_ = QDir(scanSessionDir_).absoluteFilePath(QStringLiteral("images"));
    if (scanSessionDir_.isEmpty() || !QDir().mkpath(imageDir_)) {
        if (error) *error = QStringLiteral("无法创建唯一扫描会话目录。");
        return false;
    }
    manifestPath_ = QDir(scanSessionDir_).absoluteFilePath(QStringLiteral("scan_manifest.csv"));
    rawPlyPath_ = QDir(scanSessionDir_).absoluteFilePath(QStringLiteral("scan_raw.ply"));
    voxelPlyPath_ = QDir(scanSessionDir_).absoluteFilePath(QStringLiteral("scan_voxel.ply"));
    QSaveFile file(manifestPath_);
    const QByteArray header =
        "profile_index,image_path,frame_no,device_timestamp,camera_host_timestamp_raw,"
        "exposure_us,gain_db,before_host_ms,after_host_ms,"
        "before_x_mm,before_y_mm,before_z_mm,before_rx_deg,before_ry_deg,before_rz_deg,"
        "after_x_mm,after_y_mm,after_z_mm,after_rx_deg,after_ry_deg,after_rz_deg,"
        "translation_delta_mm,rotation_delta_deg,point_count,z_min_mm,z_max_mm,line_rms_mm,"
        "line_rms_limit_mm,flat_target_gate,stripe_saturated_ratio,"
        "intrinsics_sha256,laser_plane_sha256,handeye_sha256\n";
    if (!file.open(QIODevice::WriteOnly) || file.write(header) != header.size() || !file.commit()) {
        if (error) *error = QStringLiteral("无法创建 %1").arg(manifestPath_);
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::appendManifest(
        const ProfileRow& row, quint64 frameNo, quint64 deviceTimestamp,
        qint64 cameraHostTimestampRaw, double exposureUs, double gainDb,
        QString* error) const {
    QFile file(manifestPath_);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        if (error) *error = QStringLiteral("无法追加 %1").arg(manifestPath_);
        return false;
    }
    QByteArray line;
    line += QByteArray::number(row.index);
    line += ','; line += csvQuoted(row.imagePath);
    line += ','; line += QByteArray::number(frameNo);
    line += ','; line += QByteArray::number(deviceTimestamp);
    line += ','; line += QByteArray::number(cameraHostTimestampRaw);
    line += ','; line += QByteArray::number(exposureUs, 'f', 6);
    line += ','; line += QByteArray::number(gainDb, 'f', 6);
    line += ','; line += QByteArray::number(row.before.hostTimestampMs);
    line += ','; line += QByteArray::number(row.after.hostTimestampMs);
    const hik_scan::Pose6D& before = row.before.pose;
    const hik_scan::Pose6D& after = row.after.pose;
    const double values[] = {before.x, before.y, before.z,
                             before.rx, before.ry, before.rz,
                             after.x, after.y, after.z,
                             after.rx, after.ry, after.rz,
                             row.translationDeltaMm, row.rotationDeltaDeg,
                             static_cast<double>(row.cameraPointCount), row.minimumDepthMm,
                             row.maximumDepthMm, row.lineRmsMm};
    for (int index = 0; index < 18; ++index) {
        line += ','; line += QByteArray::number(values[index], 'f', 9);
    }
    line += ','; line += QByteArray::number(row.lineRmsLimitMm, 'f', 9);
    line += ','; line += (row.flatTargetGate ? "1" : "0");
    line += ','; line += QByteArray::number(row.stripeSaturatedRatio, 'f', 9);
    line += ','; line += csvQuoted(intrinsicsSha256_);
    line += ','; line += csvQuoted(laserPlaneSha256_);
    line += ','; line += csvQuoted(handEyeSha256_);
    line += '\n';
    if (file.write(line) != line.size() || !file.flush()) {
        if (error) *error = QStringLiteral("写入扫描清单失败: %1").arg(file.errorString());
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::saveCloudOutputs(QString* error) {
    std::string coreError;
    if (!hik_scan::saveScanPly(localPath(rawPlyPath_), cloud_, "base_link", &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    const std::vector<hik_scan::CloudPoint> voxel =
        hik_scan::voxelDownsample(cloud_, voxelSpin_->value());
    if (!hik_scan::saveScanPly(localPath(voxelPlyPath_), voxel, "base_link", &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    return true;
}

void HikConstantLaserScanWindow::refreshTable() {
    profileTable_->setRowCount(static_cast<int>(profileRows_.size()));
    for (int rowIndex = 0; rowIndex < static_cast<int>(profileRows_.size()); ++rowIndex) {
        const ProfileRow& row = profileRows_[static_cast<std::size_t>(rowIndex)];
        const QStringList values = QStringList()
            << QString::number(row.index) << QFileInfo(row.imagePath).fileName()
            << QString::number(row.cameraPointCount)
            << QString::number(row.minimumDepthMm, 'f', 3)
            << QString::number(row.maximumDepthMm, 'f', 3)
            << QString::number(row.lineRmsMm, 'f', 4)
            << QStringLiteral("%1%").arg(100.0 * row.stripeSaturatedRatio, 0, 'f', 1)
            << QString::number(row.translationDeltaMm, 'f', 4)
            << QString::number(row.rotationDeltaDeg, 'f', 4)
            << QString::number(row.before.pose.x, 'f', 3)
            << QStringLiteral("%1 / %2").arg(row.before.pose.y, 0, 'f', 3)
                                         .arg(row.before.pose.z, 0, 'f', 3);
        for (int column = 0; column < values.size(); ++column)
            profileTable_->setItem(rowIndex, column, new QTableWidgetItem(values[column]));
    }
}

QImage HikConstantLaserScanWindow::drawStripeOverlay(
        const cv::Mat& gray, const hik_calibration::StaticProfileResult& profile) const {
    cv::Mat overlay;
    cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);
    for (std::size_t index = 0; index < profile.stripe.size(); ++index) {
        cv::circle(overlay, cv::Point(static_cast<int>(std::lround(profile.stripe[index].pixel.x)),
                                     static_cast<int>(std::lround(profile.stripe[index].pixel.y))),
                   1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
    std::ostringstream caption;
    caption << "single-frame: " << profile.points.size() << " pts, Z="
            << std::fixed << std::setprecision(1) << profile.minimumDepthMm << ".."
            << profile.maximumDepthMm << " mm";
    cv::putText(overlay, caption.str(), cv::Point(20, 36), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    return cvMatToQImage(overlay);
}

cv::Mat HikConstantLaserScanWindow::qImageToGray(const QImage& image) {
    const QImage gray = image.convertToFormat(QImage::Format_Grayscale8);
    return cv::Mat(gray.height(), gray.width(), CV_8UC1,
                   const_cast<uchar*>(gray.constBits()),
                   static_cast<std::size_t>(gray.bytesPerLine())).clone();
}

QImage HikConstantLaserScanWindow::cvMatToQImage(const cv::Mat& image) {
    if (image.type() == CV_8UC1)
        return QImage(image.data, image.cols, image.rows, static_cast<int>(image.step),
                      QImage::Format_Grayscale8).copy();
    if (image.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                      QImage::Format_RGB888).copy();
    }
    return QImage();
}

std::string HikConstantLaserScanWindow::localPath(const QString& path) {
    const QByteArray encoded = QFile::encodeName(path);
    return std::string(encoded.constData(), static_cast<std::size_t>(encoded.size()));
}
