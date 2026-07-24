#include "HikConstantLaserScanWindow.h"

#include "FairinoRobotSession.h"
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
#include <QJsonDocument>
#include <QJsonObject>
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
#include <condition_variable>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>

#ifndef HIK_CALIBRATION_SOURCE_DIR
#define HIK_CALIBRATION_SOURCE_DIR "."
#endif

class DirectCallbackGate final {
public:
    bool enter() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!accepting_) return false;
        ++activeCallbacks_;
        return true;
    }

    void leave() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (activeCallbacks_ > 0) --activeCallbacks_;
        if (activeCallbacks_ == 0) condition_.notify_all();
    }

    void closeAndWait() {
        std::unique_lock<std::mutex> lock(mutex_);
        accepting_ = false;
        condition_.wait(lock, [this] {
            return activeCallbacks_ == 0;
        });
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    bool accepting_{true};
    int activeCallbacks_{0};
};

namespace {

class DirectCallbackUse final {
public:
    explicit DirectCallbackUse(
            const std::shared_ptr<DirectCallbackGate>& gate)
        : gate_(gate),
          entered_(gate_ && gate_->enter()) {}

    ~DirectCallbackUse() {
        if (entered_) gate_->leave();
    }

    explicit operator bool() const { return entered_; }

private:
    std::shared_ptr<DirectCallbackGate> gate_;
    bool entered_{false};
};

const double kStillTranslationMm = 0.10;
const double kStillRotationDeg = 0.05;
const qint64 kLaserStatusFreshnessLimitMs = 1250;
const int kQualitySupportMinimumProfiles = 1;
const int kQualitySupportMaximumProfileGap = 2;
const double kQualitySupportRadiusFloorMm = 0.5;
const double kQualitySupportStepFactor = 1.75;
const double kQualitySupportVoxelFactor = 2.0;

// Scan state-machine ownership stays process-wide.  The FAIRINO SDK client and
// command lease are owned separately by FairinoRobotSession.
HikConstantLaserScanWindow* gScanActivityOwner = nullptr;

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

bool saveCloudPlyAllowEmpty(
        const QString& path,
        const std::vector<hik_scan::CloudPoint>& cloud,
        const QString& frameId,
        QString* error) {
    if (error) error->clear();
    if (!cloud.empty()) {
        const QByteArray encoded = QFile::encodeName(path);
        std::string coreError;
        if (!hik_scan::saveScanPly(
                std::string(
                    encoded.constData(),
                    static_cast<std::size_t>(encoded.size())),
                cloud, frameId.toStdString(), &coreError)) {
            if (error) *error = QString::fromStdString(coreError);
            return false;
        }
        return true;
    }

    // An empty rejected set is a valid quality result. HikScanCore deliberately
    // rejects generic empty clouds, so write the same schema here with zero
    // vertices instead of inventing a placeholder measurement.
    QByteArray payload;
    payload += "ply\nformat ascii 1.0\n";
    payload += "comment frame_id " + frameId.toUtf8() + "\n";
    payload += "comment units millimeter\n";
    payload += "element vertex 0\n";
    payload += "property double x\nproperty double y\nproperty double z\n";
    payload += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    payload += "property float confidence\nproperty float response\n";
    payload += "property int profile_index\n";
    payload += "property float pixel_u\nproperty float pixel_v\n";
    payload += "property uint quality_flags\n";
    payload += "property uint observation_count\n";
    payload += "end_header\n";
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(payload) != payload.size() ||
        !file.commit()) {
        if (error) {
            *error = QStringLiteral("无法写入空质量点云：%1").arg(path);
        }
        return false;
    }
    return true;
}

QString laserStateText(LineLaserState state) {
    switch (state) {
        case LineLaserState::Off:
            return QStringLiteral("两路 LOW");
        case LineLaserState::Laser450:
            return QStringLiteral("450 nm TTL HIGH");
        case LineLaserState::Laser650:
            return QStringLiteral("650 nm TTL HIGH");
        case LineLaserState::Unknown:
            break;
    }
    return QStringLiteral("未知");
}

QString laserStateProtocolName(LineLaserState state) {
    switch (state) {
        case LineLaserState::Off:
            return QStringLiteral("off");
        case LineLaserState::Laser450:
            return QStringLiteral("laser450");
        case LineLaserState::Laser650:
            return QStringLiteral("laser650");
        case LineLaserState::Unknown:
            break;
    }
    return QStringLiteral("unknown");
}

hik_stripe::Orientation stripeOrientation(
        LineLaserStripeOrientation orientation) {
    switch (orientation) {
    case LineLaserStripeOrientation::Auto:
        return hik_stripe::Orientation::Auto;
    case LineLaserStripeOrientation::Horizontal:
        return hik_stripe::Orientation::Horizontal;
    case LineLaserStripeOrientation::Vertical:
        return hik_stripe::Orientation::Vertical;
    }
    return hik_stripe::Orientation::Auto;
}

hik_calibration::StripeExtractionMode stripeMode(
        LineLaserCenterlinePolicy policy) {
    switch (policy) {
    case LineLaserCenterlinePolicy::Legacy:
        return hik_calibration::StripeExtractionMode::Legacy;
    case LineLaserCenterlinePolicy::Shadow:
        return hik_calibration::StripeExtractionMode::Shadow;
    case LineLaserCenterlinePolicy::Quality:
        return hik_calibration::StripeExtractionMode::Quality;
    }
    return hik_calibration::StripeExtractionMode::Legacy;
}

cv::Rect stripeRoi(const LineLaserDeviceProfile& profile,
                   const cv::Size& imageSize) {
    const int x = std::max(
        0, std::min(
            imageSize.width - 1,
            static_cast<int>(std::floor(
                profile.stripeRoiX * imageSize.width))));
    const int y = std::max(
        0, std::min(
            imageSize.height - 1,
            static_cast<int>(std::floor(
                profile.stripeRoiY * imageSize.height))));
    const int right = std::max(
        x + 1, std::min(
            imageSize.width,
            static_cast<int>(std::ceil(
                (profile.stripeRoiX + profile.stripeRoiWidth) *
                imageSize.width))));
    const int bottom = std::max(
        y + 1, std::min(
            imageSize.height,
            static_cast<int>(std::ceil(
                (profile.stripeRoiY + profile.stripeRoiHeight) *
                imageSize.height))));
    return cv::Rect(x, y, right - x, bottom - y);
}

}  // namespace

HikConstantLaserScanWindow::HikConstantLaserScanWindow(
        const LineLaserDeviceProfile& profile,
        LineLaserController* laserController,
        FairinoRobotSession* robotSession,
        QWidget* parent, double scanSpeedOverrideMmS)
    : QMainWindow(parent),
      profile_(profile),
      laserController_(laserController),
      robotSession_(robotSession),
      sourceDir_(QDir::cleanPath(QString::fromUtf8(HIK_CALIBRATION_SOURCE_DIR))) {
    QString profileError;
    if (!profile_.isValid(&profileError)) {
        throw std::invalid_argument(profileError.toStdString());
    }
    if (!laserController_) {
        throw std::invalid_argument(
            "LineLaserController must be shared with the scan window");
    }
    if (!robotSession_) {
        throw std::invalid_argument(
            "FairinoRobotSession must be shared with the scan window");
    }
    synchronizationConfigPath_ =
        profile_.synchronizationConfigPath(sourceDir_);
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
    profileOptions_.reconstruction.stripe.mode =
        stripeMode(profile_.scanCenterlinePolicy);
    profileOptions_.reconstruction.stripe.quality.orientation =
        stripeOrientation(profile_.stripeOrientation);
    profileOptions_.reconstruction.minReconstructedPoints = 80;
    profileOptions_.reconstruction.maxLineRmsMm = 0.50;
    buildUi();
    setupWorkers();
    appendLog(QStringLiteral(
        "条纹策略：方向=%1，模式=%2，归一化ROI=[%3,%4,%5,%6]；"
        "ROI还会与正式激光平面的有效深度走廊取交集。")
        .arg(lineLaserStripeOrientationName(profile_.stripeOrientation),
             lineLaserCenterlinePolicyName(profile_.scanCenterlinePolicy))
        .arg(profile_.stripeRoiX, 0, 'f', 4)
        .arg(profile_.stripeRoiY, 0, 'f', 4)
        .arg(profile_.stripeRoiWidth, 0, 'f', 4)
        .arg(profile_.stripeRoiHeight, 0, 'f', 4));
    if (profile_.scanCenterlinePolicy ==
        LineLaserCenterlinePolicy::Shadow) {
        appendLog(QStringLiteral(
            "shadow 模式只并行生成质量诊断点云；正式 scan_raw/scan_voxel "
            "仍严格使用 legacy 中心，不会被质量中心替换。"));
    }
    if (synchronizationConfigReady_) {
        appendLog(QStringLiteral(
            "同步配置已加载：%1；目标=%2 fps，曝光=%3 us，扫描速度=%4 mm/s，"
            "CNDE请求周期=%5 ms，预期实际反馈周期=%6 ms；"
            "连续重建线程=%7、非阻塞队列=%8。")
            .arg(synchronizationConfigPath_)
            .arg(synchronizationConfig_.cameraTargetFps, 0, 'f', 3)
            .arg(synchronizationConfig_.cameraExposureUs, 0, 'f', 3)
            .arg(synchronizationConfig_.scanSpeedMmS, 0, 'f', 3)
            .arg(synchronizationConfig_.robotPeriodMs, 0, 'f', 3)
            .arg(synchronizationConfig_.robotExpectedFeedbackPeriodMs, 0, 'f', 3)
            .arg(synchronizationConfig_.reconstructionThreads)
            .arg(synchronizationConfig_.reconstructionQueueCapacity));
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
        "设备组=%1（%2，TTL 物理 Pin %3）；常亮模式使用单帧形态学背景抑制；"
        "原图始终保存，错误条纹必须通过预览人工复核。")
        .arg(profile_.id, profile_.displayName)
        .arg(profile_.ttlPhysicalPin));
    appendLog(QStringLiteral(
        "本页只把“板端 ACK + GPIO 回读”为 HIGH 记为 TTL 就绪；"
        "这不等同于对激光器实际光功率的测量。"));
    updateUi();
}

HikConstantLaserScanWindow::~HikConstantLaserScanWindow() {
    shutdownWorkers();
}

void HikConstantLaserScanWindow::buildUi() {
    setWindowTitle(QStringLiteral("FR5 线扫｜%1｜%2")
                   .arg(profile_.displayName, profile_.id));
    resize(1620, 960);
    QWidget* central = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(central);

    QLabel* warning = new QLabel(QStringLiteral(
        "当前设备组：%1（%2 nm，TTL 物理 Pin %3）。验证流程会真实发送 FR5 MoveL。"
        "默认 dry-run；真运动前必须确认控制器已使能/自动模式、路径无碰撞、速度足够低且人员守在物理急停旁。"
        "本页软件停止不是控制柜物理急停；两个设备页共享唯一 FR5 连接，"
        "但同一时刻只允许一组持有运动/扫描命令租约。")
        .arg(profile_.displayName)
        .arg(profile_.wavelengthNm)
        .arg(profile_.ttlPhysicalPin), central);
    warning->setWordWrap(true);
    warning->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    root->addWidget(warning);

    QGroupBox* devices = new QGroupBox(QStringLiteral("设备连接"), central);
    QGridLayout* deviceLayout = new QGridLayout(devices);
    cameraIpEdit_ = new QLineEdit(profile_.defaultCameraIp, devices);
    if (profile_.defaultCameraIp.trimmed().isEmpty()) {
        cameraIpEdit_->setPlaceholderText(QStringLiteral("请输入此设备组的相机 IPv4 地址"));
    }
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
    connectCameraButton_ = new QPushButton(QStringLiteral("连接本组相机"), devices);
    disconnectCameraButton_ = new QPushButton(QStringLiteral("断开相机"), devices);
    cameraStatusLabel_ = new QLabel(
        QStringLiteral("%1：相机未连接").arg(profile_.id), devices);

    robotIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.200"), devices);
    connectRobotButton_ = new QPushButton(
        QStringLiteral("连接共享 FR5"), devices);
    disconnectRobotButton_ = new QPushButton(
        QStringLiteral("断开共享 FR5"), devices);
    readPoseButton_ = new QPushButton(QStringLiteral("读取法兰"), devices);
    robotStatusLabel_ = new QLabel(QStringLiteral("FR5 未连接"), devices);
    currentPoseLabel_ = new QLabel(QStringLiteral("当前法兰: -"), devices);
    currentPoseLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);

    connectLaserButton_ = new QPushButton(
        QStringLiteral("连接鲁班猫 TTL"), devices);
    enableProfileLaserButton_ = new QPushButton(
        QStringLiteral("开启本组 %1 nm TTL").arg(profile_.wavelengthNm),
        devices);
    disableAllLasersButton_ = new QPushButton(
        QStringLiteral("关闭两路 TTL"), devices);
    enableProfileLaserButton_->setStyleSheet(QStringLiteral(
        "background:#f9a825;color:#111;font-weight:bold;"));
    disableAllLasersButton_->setStyleSheet(QStringLiteral(
        "background:#2e7d32;color:white;font-weight:bold;"));
    enableProfileLaserButton_->setToolTip(QStringLiteral(
        "请求鲁班猫将本组 TTL 持续置 HIGH；只有板端 ACK 和 GPIO 回读一致后状态才会变绿。"));
    disableAllLasersButton_->setToolTip(QStringLiteral(
        "无条件请求 Pin 11/GPIO15 与 Pin 7/GPIO16 同时置 LOW。"));
    laserStatusLabel_ = new QLabel(
        QStringLiteral("鲁班猫 TTL 未连接（192.168.1.12）"), devices);
    laserStatusLabel_->setWordWrap(true);

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
    deviceLayout->addWidget(new QLabel(
        QStringLiteral("激光 TTL"), devices), 2, 0);
    deviceLayout->addWidget(connectLaserButton_, 2, 1);
    deviceLayout->addWidget(enableProfileLaserButton_, 2, 2, 1, 2);
    deviceLayout->addWidget(disableAllLasersButton_, 2, 4, 1, 2);
    deviceLayout->addWidget(laserStatusLabel_, 2, 6, 1, 5);
    deviceLayout->setColumnStretch(1, 1);
    root->addWidget(devices);

    QGroupBox* calibration = new QGroupBox(
        QStringLiteral("正式标定｜%1").arg(profile_.displayName), central);
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
    imageView_->setEmptyText(QStringLiteral("%1：暂无常亮激光图像")
                             .arg(profile_.displayName));
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
    connect(connectLaserButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::connectLaserController);
    connect(enableProfileLaserButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::enableProfileLaser);
    connect(disableAllLasersButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::disableAllLasers);
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
    connect(laserController_, &LineLaserController::connectionStateChanged,
            this,
            &HikConstantLaserScanWindow::onLaserConnectionStateChanged);
    connect(laserController_, &LineLaserController::statusChanged,
            this, &HikConstantLaserScanWindow::onLaserStatusChanged);
    connect(laserController_, &LineLaserController::commandFinished,
            this, &HikConstantLaserScanWindow::onLaserCommandFinished);
    connect(laserController_, &LineLaserController::faultOccurred,
            this, &HikConstantLaserScanWindow::onLaserFault);
    connect(laserController_, &LineLaserController::logMessage,
            this, [this](const QString& message) {
                appendLog(QStringLiteral("TTL控制：%1").arg(message));
            });
    laserFreshnessTimer_ = new QTimer(this);
    laserFreshnessTimer_->setInterval(250);
    connect(laserFreshnessTimer_, &QTimer::timeout, this, [this]() {
        if (terminalBarrierActive_) {
            tryCompleteTerminalBarrier();
        } else if (scanActivityHeld_ && !laserStatusFresh()) {
            abortForLaserSafety(QStringLiteral(
                "超过 %1 ms 未收到新鲜的板端 TTL 状态，"
                "按失光处理并请求停止 FR5。")
                .arg(kLaserStatusFreshnessLimitMs));
        }
    });
    laserFreshnessTimer_->start();
}

void HikConstantLaserScanWindow::setupWorkers() {
    qRegisterMetaType<hik_sync::CameraFrame>("hik_sync::CameraFrame");
    qRegisterMetaType<hik_sync::RobotSample>("hik_sync::RobotSample");
    directCallbackGate_ = std::make_shared<DirectCallbackGate>();
    robotClientId_ = robotSession_->registerClient(profile_.id);
    if (robotClientId_ <= 0 || !robotSession_->isRunning()) {
        throw std::runtime_error(
            "unable to register with the shared FairinoRobotSession");
    }
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
    continuousFrameConnection_ = connect(
            cameraWorker_, &HikCameraWorker::continuousFrameReady,
            this,
            [this, gate = directCallbackGate_](
                    hik_sync::CameraFrame frame) {
                DirectCallbackUse callback(gate);
                if (!callback) return;
                (void)synchronizationSession_.pushCamera(std::move(frame));
            }, Qt::DirectConnection);
    imagePoolConnection_ = connect(
            cameraWorker_, &HikCameraWorker::imagePoolExhausted,
            this, [this, gate = directCallbackGate_]() {
                DirectCallbackUse callback(gate);
                if (!callback) return;
                synchronizationSession_.noteImagePoolExhaustion();
            }, Qt::DirectConnection);
    cameraThread_.start();

    connect(this, &HikConstantLaserScanWindow::requestConnectRobot,
            robotSession_, &FairinoRobotSession::connectRobot);
    connect(this, &HikConstantLaserScanWindow::requestDisconnectRobot,
            robotSession_, &FairinoRobotSession::disconnectRobot);
    connect(this, &HikConstantLaserScanWindow::requestReadFlangePose,
            robotSession_, &FairinoRobotSession::readFlangePose);
    connect(this, &HikConstantLaserScanWindow::requestMoveLinear,
            robotSession_, &FairinoRobotSession::moveLinear);
    connect(this, &HikConstantLaserScanWindow::requestMoveLinearPhysical,
            robotSession_, &FairinoRobotSession::moveLinearPhysical);
    connect(this, &HikConstantLaserScanWindow::requestStopMotion,
            robotSession_, &FairinoRobotSession::stopMotion);
    connect(robotSession_, &FairinoRobotSession::connectionChanged,
            this, &HikConstantLaserScanWindow::onRobotConnectionChanged, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::busyChanged,
            this, &HikConstantLaserScanWindow::onRobotBusyChanged, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::flangePoseReady,
            this, &HikConstantLaserScanWindow::onRobotFlangePoseReady, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::motionStarted,
            this, &HikConstantLaserScanWindow::onRobotMotionStarted, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::motionFinished,
            this, &HikConstantLaserScanWindow::onRobotMotionFinished, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::log,
            this, &HikConstantLaserScanWindow::onRobotLog, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::error,
            this, &HikConstantLaserScanWindow::onRobotError, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::clientError,
            this, &HikConstantLaserScanWindow::onRobotClientError,
            Qt::QueuedConnection);
    robotSampleConnection_ = connect(
            robotSession_, &FairinoRobotSession::robotSampleReady,
            this,
            [this, gate = directCallbackGate_](
                    int clientId,
                    quint64 leaseEpoch,
                    hik_sync::RobotSample sample) {
                DirectCallbackUse callback(gate);
                if (!callback) return;
                if (clientId != robotClientId_ ||
                    leaseEpoch != robotLeaseEpoch_.load(
                        std::memory_order_acquire)) {
                    return;
                }
                (void)synchronizationSession_.pushRobot(std::move(sample));
            }, Qt::DirectConnection);
    connect(robotSession_, &FairinoRobotSession::realtimePeriodConfigured,
            this, [this](int periodMs) {
                appendLog(QStringLiteral("FR5 20004 实时反馈周期确认：%1 ms。")
                          .arg(periodMs));
            }, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::exclusiveOwnerChanged,
            this, [this](int, const QString&) {
                updateUi();
            }, Qt::QueuedConnection);
    robotConnected_ = robotSession_->isConnected();
    robotBusy_ = robotSession_->isBusy();
}

void HikConstantLaserScanWindow::shutdownWorkers() {
    if (shuttingDown_) return;
    shuttingDown_ = true;
    if (directCallbackGate_) {
        directCallbackGate_->closeAndWait();
    }
    QObject::disconnect(continuousFrameConnection_);
    QObject::disconnect(imagePoolConnection_);
    QObject::disconnect(robotSampleConnection_);
    requestLaserOff();
    if (cameraWorker_ && cameraThread_.isRunning() &&
        continuousState_ != ContinuousState::Idle) {
        QMetaObject::invokeMethod(cameraWorker_, "stopContinuous",
                                  Qt::BlockingQueuedConnection);
    }
    synchronizationSession_.stop();
    if (continuousReconstruction_.running()) {
        hik_scan::ContinuousReconstructionStatistics ignoredStatistics;
        std::string ignoredError;
        (void)continuousReconstruction_.stopAndSave(
            &ignoredStatistics, &ignoredError);
    }
    continuousState_ = ContinuousState::Idle;
    if (cameraWorker_ && cameraThread_.isRunning()) {
        QMetaObject::invokeMethod(cameraWorker_, "disconnectCamera", Qt::BlockingQueuedConnection);
        cameraThread_.quit();
        cameraThread_.wait();
    }
    cameraWorker_ = nullptr;
    robotLeaseEpoch_.store(0, std::memory_order_release);
    if (scanActivityHeld_) {
        // Process teardown must not hand an unresolved motion/terminal lease
        // to another page.  FairinoRobotSession keeps it until its sole worker
        // has sent StopMotion and confirmed fresh 20004 stationary samples.
        scanActivityHeld_ = false;
        if (gScanActivityOwner == this) gScanActivityOwner = nullptr;
        emit scanActivityChanged(false);
    } else if (robotSession_ && robotClientId_ > 0) {
        robotSession_->releaseExclusive(robotClientId_);
    }
}

void HikConstantLaserScanWindow::closeEvent(QCloseEvent* event) {
    shutdownWorkers();
    event->accept();
}

void HikConstantLaserScanWindow::appendLog(const QString& message) {
    logView_->appendPlainText(QStringLiteral("[%1][%2] %3")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")),
             profile_.id, message));
}

void HikConstantLaserScanWindow::showError(const QString& title, const QString& message) {
    appendLog(QStringLiteral("%1: %2").arg(title, message));
    if (!shuttingDown_) QMessageBox::warning(this, title, message);
}

void HikConstantLaserScanWindow::setProfileTabActive(bool active) {
    if (profileTabActive_ == active) return;
    profileTabActive_ = active;
    if (!active && scanActivityHeld_) {
        appendLog(QStringLiteral(
            "本设备组仍在执行扫描安全收尾，继续持有共享 FR5 命令租约。"));
    } else if (active &&
               !robotSession_->commandAvailableTo(robotClientId_)) {
        appendLog(QStringLiteral(
            "另一设备组当前持有共享 FR5 命令租约；连接状态仍然共享，"
            "但租约释放前本页不能发送机器人命令。"));
        if (scanState_ == ScanState::Idle &&
            continuousState_ == ContinuousState::Idle) {
            scanStatusLabel_->setText(QStringLiteral(
                "共享 FR5 正由另一设备组执行扫描/安全收尾。"));
        }
    }
    updateUi();
}

bool HikConstantLaserScanWindow::acquireScanActivity(QString* error) {
    if (scanActivityHeld_) return true;
    if (!profileTabActive_) {
        if (error) *error = QStringLiteral("当前设备组标签页未激活。");
        return false;
    }
    if (gScanActivityOwner && gScanActivityOwner != this) {
        if (error) {
            *error = QStringLiteral("设备组 %1 正在执行真实采集或运动。")
                .arg(gScanActivityOwner->deviceProfile().displayName);
        }
        return false;
    }
    QString leaseError;
    if (!robotSession_->acquireExclusive(
            robotClientId_,
            QStringLiteral("%1 扫描/安全收尾").arg(profile_.id),
            &leaseError)) {
        if (error) *error = leaseError;
        return false;
    }
    const quint64 leaseEpoch =
        robotSession_->exclusiveLeaseEpochFor(robotClientId_);
    if (leaseEpoch == 0) {
        robotSession_->releaseExclusive(robotClientId_);
        if (error) {
            *error = QStringLiteral(
                "共享 FR5 已授予租约，但未生成有效代次。");
        }
        return false;
    }
    robotLeaseEpoch_.store(leaseEpoch, std::memory_order_release);
    gScanActivityOwner = this;
    scanActivityHeld_ = true;
    laserSafetyAbortIssued_ = false;
    terminalBarrierActive_ = false;
    terminalCompleted_ = false;
    terminalMotionFault_ = false;
    terminalCameraFault_ = false;
    terminalReason_.clear();
    terminalMode_.clear();
    terminalSessionDirectory_.clear();
    terminalEndedUtc_.clear();
    terminalMotionFaultDetail_.clear();
    terminalCameraFaultDetail_.clear();
    terminalStatistics_ = QJsonObject();
    ++scanGeneration_;
    emit scanActivityChanged(true);
    return true;
}

void HikConstantLaserScanWindow::releaseScanActivity() {
    if (!scanActivityHeld_) return;
    scanActivityHeld_ = false;
    if (gScanActivityOwner == this) gScanActivityOwner = nullptr;
    robotLeaseEpoch_.store(0, std::memory_order_release);
    robotSession_->releaseExclusive(robotClientId_);
    emit scanActivityChanged(false);
}

void HikConstantLaserScanWindow::updateUi() {
    const bool stateIdle = !shuttingDown_ && scanState_ == ScanState::Idle &&
                           continuousState_ == ContinuousState::Idle &&
                           !terminalBarrierActive_ &&
                           pendingMotionRequestId_ < 0 &&
                           pendingRobotRequestId_ < 0 &&
                           pendingCameraRequestId_ < 0 &&
                           !robotBusy_ && !cameraBusy_;
    const bool idle = profileTabActive_ && stateIdle;
    const bool robotAvailable =
        robotSession_->commandAvailableTo(robotClientId_);
    const bool laserTransportReady =
        laserConnectionState_ == LineLaserConnectionState::Ready;
    const bool profileLaserReady = laserReadyForProfile();
    connectCameraButton_->setEnabled(idle && !cameraConnected_ && !cameraBusy_);
    disconnectCameraButton_->setEnabled(idle && cameraConnected_ && !cameraBusy_);
    cameraIpEdit_->setEnabled(idle && !cameraConnected_);
    exposureSpin_->setEnabled(idle && !cameraBusy_);
    gainSpin_->setEnabled(idle && !cameraBusy_);
    cameraTimeoutSpin_->setEnabled(idle && !cameraBusy_);
    connectLaserButton_->setEnabled(
        profileTabActive_ &&
        (laserConnectionState_ == LineLaserConnectionState::Disconnected ||
         laserConnectionState_ == LineLaserConnectionState::Fault));
    enableProfileLaserButton_->setEnabled(
        idle && laserTransportReady && !profileLaserReady);
    disableAllLasersButton_->setEnabled(
        profileTabActive_ && laserTransportReady &&
        laserStatus_.state != LineLaserState::Off);
    connectRobotButton_->setEnabled(
        idle && robotAvailable && !robotConnected_ && !robotBusy_ &&
        !robotSession_->connectionAttempted());
    disconnectRobotButton_->setEnabled(
        idle && robotAvailable && robotConnected_ && !robotBusy_);
    robotIpEdit_->setEnabled(
        idle && !robotConnected_ && !robotSession_->connectionAttempted());
    readPoseButton_->setEnabled(
        idle && robotAvailable && robotConnected_ && !robotBusy_);
    teachStartButton_->setEnabled(
        idle && robotAvailable && robotConnected_ && !robotBusy_);
    teachEndButton_->setEnabled(
        idle && robotAvailable && robotConnected_ && !robotBusy_);
    editStartButton_->setEnabled(idle && startTaught_);
    editEndButton_->setEnabled(idle && endTaught_);
    reloadCalibrationButton_->setEnabled(idle);
    dryRunButton_->setEnabled(idle && startTaught_ && endTaught_);
    captureCurrentButton_->setEnabled(
        idle && robotAvailable && cameraConnected_ && robotConnected_ &&
        calibrationReady_ && profileLaserReady);
    startScanButton_->setEnabled(idle && robotAvailable &&
                                 startTaught_ && endTaught_ &&
                                 (dryRunCheck_->isChecked() ||
                                  (cameraConnected_ && robotConnected_ &&
                                   calibrationReady_ && profileLaserReady)));
    startContinuousButton_->setEnabled(
        idle && robotAvailable && synchronizationConfigReady_ &&
        startTaught_ && endTaught_ && cameraConnected_ && robotConnected_ &&
        calibrationReady_ && profileLaserReady);
    const bool scanStateMachineActive =
        scanState_ != ScanState::Idle ||
        continuousState_ != ContinuousState::Idle;
    stopButton_->setEnabled(
        profileTabActive_ && scanStateMachineActive && robotConnected_);
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

bool HikConstantLaserScanWindow::laserReadyForProfile(QString* error) const {
    if (error) error->clear();
    if (laserConnectionState_ != LineLaserConnectionState::Ready ||
        !laserStatus_.reachable || !laserStatus_.leaseActive) {
        if (error) {
            *error = QStringLiteral(
                "鲁班猫 TTL 控制尚未就绪或控制租约无效。");
        }
        return false;
    }
    if (!laserStatusFresh()) {
        if (error) {
            *error = QStringLiteral(
                "板端 TTL 状态已超过 %1 ms 未刷新。")
                         .arg(kLaserStatusFreshnessLimitMs);
        }
        return false;
    }
    if (!laserStatus_.fault.trimmed().isEmpty()) {
        if (error) {
            *error = QStringLiteral("板端 GPIO 故障：%1")
                         .arg(laserStatus_.fault);
        }
        return false;
    }
    const bool expected450 = profile_.wavelengthNm == 450;
    const bool expected650 = profile_.wavelengthNm == 650;
    const LineLaserState expectedState = expected450
        ? LineLaserState::Laser450
        : expected650 ? LineLaserState::Laser650
                      : LineLaserState::Unknown;
    if (expectedState == LineLaserState::Unknown ||
        laserStatus_.state != expectedState ||
        laserStatus_.ttl450High != expected450 ||
        laserStatus_.ttl650High != expected650) {
        if (error) {
            *error = QStringLiteral(
                "本组 %1 nm TTL 尚未由板端确认 HIGH，或另一路未确认 LOW。")
                         .arg(profile_.wavelengthNm);
        }
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::laserStatusFresh() const {
    if (laserStatusReceivedMonotonicMs_ <= 0) return false;
    const qint64 nowMs = hik_sync::getMonotonicRawNs() / 1000000LL;
    const qint64 ageMs = nowMs - laserStatusReceivedMonotonicMs_;
    return ageMs >= 0 && ageMs <= kLaserStatusFreshnessLimitMs;
}

void HikConstantLaserScanWindow::connectLaserController() {
    appendLog(QStringLiteral(
        "请求连接鲁班猫 192.168.1.12；使用受限密钥，不允许密码交互。"));
    laserController_->connectController();
}

void HikConstantLaserScanWindow::enableProfileLaser() {
    if (!profileTabActive_) {
        showError(QStringLiteral("TTL 控制"),
                  QStringLiteral("请先切换到本设备组。"));
        return;
    }
    if (gScanActivityOwner) {
        showError(QStringLiteral("TTL 控制"),
                  QStringLiteral(
                      "真实采集、运动或安全收尾期间禁止开启任何激光；"
                      "只能请求关闭两路 TTL。"));
        return;
    }
    if (scanState_ != ScanState::Idle ||
        continuousState_ != ContinuousState::Idle ||
        terminalBarrierActive_) {
        showError(QStringLiteral("TTL 控制"),
                  QStringLiteral("扫描期间不能切换激光通道。"));
        return;
    }
    appendLog(QStringLiteral(
        "请求开启 %1 nm：TTL 将持续 HIGH，直到关光、租约失效或连接断开；"
        "另一通道会保持 LOW。")
        .arg(profile_.wavelengthNm));
    if (profile_.wavelengthNm == 450) {
        laserController_->set450();
    } else if (profile_.wavelengthNm == 650) {
        laserController_->set650();
    } else {
        showError(QStringLiteral("TTL 控制"),
                  QStringLiteral("不支持的激光波长：%1 nm")
                      .arg(profile_.wavelengthNm));
    }
}

void HikConstantLaserScanWindow::disableAllLasers() {
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(
            QStringLiteral("用户关闭两路 TTL，连续扫描同步终止。"),
            pendingMotionRequestId_ >= 0);
        return;
    }
    if (scanState_ != ScanState::Idle) {
        abortScan(
            QStringLiteral("用户关闭两路 TTL，停稳扫描同步终止。"),
            pendingMotionRequestId_ >= 0);
        return;
    }
    appendLog(QStringLiteral("请求关闭两路 TTL。"));
    requestLaserOff();
}

quint64 HikConstantLaserScanWindow::requestLaserOff() {
    return laserController_
        ? laserController_->requestOffTracked()
        : 0;
}

void HikConstantLaserScanWindow::beginTerminalBarrier(
        bool completed,
        const QString& reason,
        const QString& mode,
        const QString& sessionDirectory,
        const QJsonObject& statistics) {
    terminalBarrierActive_ = true;
    terminalCompleted_ = completed;
    terminalReason_ = reason;
    terminalMode_ = mode;
    terminalSessionDirectory_ = sessionDirectory;
    terminalEndedUtc_ = QDateTime::currentDateTimeUtc().toString(
        Qt::ISODateWithMs);
    terminalStatistics_ = statistics;
    terminalLaserOffRequestedNs_ = hik_sync::getMonotonicRawNs();
    terminalLaserOffCommandToken_ = requestLaserOff();
    QString resultError;
    if (!writeSessionResult(&resultError)) {
        appendLog(QStringLiteral("警告：无法写入会话最终状态：%1")
                      .arg(resultError));
    }
    scanStatusLabel_->setText(QStringLiteral(
        "%1；正在等待板端确认两路 LOW%2。")
        .arg(reason,
             pendingMotionRequestId_ >= 0
                ? QStringLiteral("并等待 FR5 运动终止")
                : QString()));
    tryCompleteTerminalBarrier();
    updateUi();
}

void HikConstantLaserScanWindow::tryCompleteTerminalBarrier() {
    if (!terminalBarrierActive_) return;
    const bool laserOffConfirmed =
        laserConnectionState_ == LineLaserConnectionState::Ready &&
        laserStatusFresh() && laserStatus_.reachable &&
        terminalLaserOffRequestedNs_ > 0 &&
        laserStatus_.sourceMonotonicNs >= terminalLaserOffRequestedNs_ &&
        terminalLaserOffCommandToken_ > 0 &&
        laserAcknowledgedOffCommandToken_ >=
            terminalLaserOffCommandToken_ &&
        laserStatus_.leaseActive &&
        laserStatus_.state == LineLaserState::Off &&
        !laserStatus_.ttl450High && !laserStatus_.ttl650High &&
        laserStatus_.fault.trimmed().isEmpty();
    const bool motionConfirmed =
        pendingMotionRequestId_ < 0 && !robotBusy_ &&
        !terminalMotionFault_;
    const bool cameraIdleConfirmed =
        !cameraBusy_ && pendingCameraRequestId_ < 0 &&
        !terminalCameraFault_;

    if (terminalMotionFault_) {
        scanStatusLabel_->setText(QStringLiteral(
            "%1；FR5 停止确认失败，保持设备组锁止。"
            "请使用物理急停并重启程序：%2")
            .arg(terminalReason_, terminalMotionFaultDetail_));
        scanStatusLabel_->setStyleSheet(QStringLiteral(
            "color:#b00020;font-weight:bold;"));
        return;
    }
    if (terminalCameraFault_) {
        scanStatusLabel_->setText(QStringLiteral(
            "%1；相机停流未确认，保持设备组锁止。"
            "请关闭相机电源/网络并重启程序：%2")
            .arg(terminalReason_, terminalCameraFaultDetail_));
        scanStatusLabel_->setStyleSheet(QStringLiteral(
            "color:#b00020;font-weight:bold;"));
        return;
    }
    if (!laserOffConfirmed || !motionConfirmed ||
        !cameraIdleConfirmed) {
        scanStatusLabel_->setText(QStringLiteral(
            "%1；安全收尾等待：TTL LOW=%2，FR5停止=%3，相机空闲=%4。")
            .arg(terminalReason_,
                 laserOffConfirmed ? QStringLiteral("已确认")
                                   : QStringLiteral("等待"),
                 motionConfirmed ? QStringLiteral("已确认")
                                 : QStringLiteral("等待"),
                 cameraIdleConfirmed ? QStringLiteral("已确认")
                                     : QStringLiteral("等待")));
        return;
    }

    terminalBarrierActive_ = false;
    scanStatusLabel_->setStyleSheet(QString());
    scanStatusLabel_->setText(QStringLiteral(
        "%1；板端已确认两路 TTL LOW，FR5 已确认停止，相机已空闲。")
        .arg(terminalReason_));
    QString resultError;
    if (!writeSessionResult(&resultError)) {
        appendLog(QStringLiteral("警告：无法更新会话最终状态：%1")
                      .arg(resultError));
    }
    releaseScanActivity();
    updateUi();
}

bool HikConstantLaserScanWindow::writeSessionResult(
        QString* error) const {
    if (error) error->clear();
    if (terminalSessionDirectory_.trimmed().isEmpty()) return true;
    if (!QDir().mkpath(terminalSessionDirectory_)) {
        if (error) {
            *error = QStringLiteral("无法创建会话结果目录：%1")
                         .arg(terminalSessionDirectory_);
        }
        return false;
    }
    const bool laserOffConfirmed =
        laserConnectionState_ == LineLaserConnectionState::Ready &&
        laserStatusFresh() && laserStatus_.reachable &&
        terminalLaserOffRequestedNs_ > 0 &&
        laserStatus_.sourceMonotonicNs >= terminalLaserOffRequestedNs_ &&
        terminalLaserOffCommandToken_ > 0 &&
        laserAcknowledgedOffCommandToken_ >=
            terminalLaserOffCommandToken_ &&
        laserStatus_.leaseActive &&
        laserStatus_.state == LineLaserState::Off &&
        !laserStatus_.ttl450High && !laserStatus_.ttl650High &&
        laserStatus_.fault.trimmed().isEmpty();
    const bool motionConfirmed =
        pendingMotionRequestId_ < 0 && !robotBusy_ &&
        !terminalMotionFault_;
    const bool cameraIdleConfirmed =
        !cameraBusy_ && pendingCameraRequestId_ < 0 &&
        !terminalCameraFault_;
    const qint64 nowMonotonicMs =
        hik_sync::getMonotonicRawNs() / 1000000LL;
    const qint64 laserStatusAgeMs =
        laserStatusReceivedMonotonicMs_ > 0
            ? nowMonotonicMs - laserStatusReceivedMonotonicMs_
            : -1;

    QJsonObject root;
    root.insert(QStringLiteral("schema_version"), 1);
    root.insert(QStringLiteral("profile_id"), profile_.id);
    root.insert(QStringLiteral("wavelength_nm"), profile_.wavelengthNm);
    root.insert(QStringLiteral("ttl_physical_pin"),
                profile_.ttlPhysicalPin);
    root.insert(QStringLiteral("mode"), terminalMode_);
    root.insert(QStringLiteral("completed"), terminalCompleted_);
    root.insert(QStringLiteral("reason"), terminalReason_);
    root.insert(QStringLiteral("ended_utc"), terminalEndedUtc_);
    root.insert(QStringLiteral("laser_off_confirmed"),
                laserOffConfirmed);
    root.insert(QStringLiteral("laser_status_fresh"),
                laserStatusFresh());
    root.insert(QStringLiteral("laser_status_age_ms"),
                static_cast<double>(laserStatusAgeMs));
    root.insert(QStringLiteral("laser_status_source_monotonic_ns"),
                static_cast<double>(laserStatus_.sourceMonotonicNs));
    root.insert(QStringLiteral("laser_status_event_sequence"),
                static_cast<double>(laserStatus_.eventSequence));
    root.insert(QStringLiteral("laser_transport_generation"),
                static_cast<double>(laserStatus_.transportGeneration));
    root.insert(QStringLiteral("terminal_laser_off_requested_ns"),
                static_cast<double>(terminalLaserOffRequestedNs_));
    root.insert(QStringLiteral("terminal_laser_off_command_token"),
                static_cast<double>(terminalLaserOffCommandToken_));
    root.insert(QStringLiteral("laser_acknowledged_off_command_token"),
                static_cast<double>(
                    laserAcknowledgedOffCommandToken_));
    root.insert(QStringLiteral(
                    "laser_current_status_off_command_token"),
                static_cast<double>(
                    laserStatus_.acknowledgedOffCommandToken));
    root.insert(QStringLiteral("laser_state"),
                laserStateText(laserStatus_.state));
    root.insert(QStringLiteral("ttl_450_high"),
                laserStatus_.ttl450High);
    root.insert(QStringLiteral("ttl_650_high"),
                laserStatus_.ttl650High);
    root.insert(QStringLiteral("laser_lease_active"),
                laserStatus_.leaseActive);
    root.insert(QStringLiteral("laser_fault"), laserStatus_.fault);
    root.insert(QStringLiteral("robot_motion_confirmed"),
                motionConfirmed);
    root.insert(QStringLiteral("robot_busy"), robotBusy_);
    root.insert(QStringLiteral("camera_idle_confirmed"),
                cameraIdleConfirmed);
    root.insert(QStringLiteral("camera_busy"), cameraBusy_);
    root.insert(QStringLiteral("camera_stop_fault"),
                terminalCameraFault_);
    root.insert(QStringLiteral("camera_stop_fault_detail"),
                terminalCameraFaultDetail_);
    root.insert(QStringLiteral("interlock_complete"),
                laserOffConfirmed && motionConfirmed &&
                cameraIdleConfirmed);
    root.insert(QStringLiteral("motion_stop_fault"),
                terminalMotionFault_);
    root.insert(QStringLiteral("motion_stop_fault_detail"),
                terminalMotionFaultDetail_);
    root.insert(QStringLiteral("laser_daemon_generation"),
                laserStatus_.daemonGeneration);
    root.insert(QStringLiteral("terminal_barrier_active"),
                terminalBarrierActive_);
    root.insert(QStringLiteral("statistics"), terminalStatistics_);

    const QByteArray payload =
        QJsonDocument(root).toJson(QJsonDocument::Indented);
    const QString path = QDir(terminalSessionDirectory_).absoluteFilePath(
        QStringLiteral("session_result.json"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(payload) != payload.size() ||
        !file.commit()) {
        if (error) {
            *error = QStringLiteral("无法原子写入 %1").arg(path);
        }
        return false;
    }
    return true;
}

void HikConstantLaserScanWindow::onLaserConnectionStateChanged(
        LineLaserConnectionState state, QString detail) {
    laserConnectionState_ = state;
    if (state == LineLaserConnectionState::Disconnected) {
        connectLaserButton_->setText(QStringLiteral("连接鲁班猫 TTL"));
    } else if (state == LineLaserConnectionState::Fault) {
        connectLaserButton_->setText(QStringLiteral("重连鲁班猫 TTL"));
    } else {
        connectLaserButton_->setText(QStringLiteral("TTL 通道已连接"));
    }
    appendLog(QStringLiteral("TTL连接状态：%1").arg(detail));
    if (scanActivityHeld_ &&
        !terminalBarrierActive_ &&
        state != LineLaserConnectionState::Ready &&
        state != LineLaserConnectionState::CommandPending) {
        abortForLaserSafety(QStringLiteral("TTL 控制连接失效：%1").arg(detail));
    }
    if (terminalBarrierActive_) {
        const bool currentOffAcknowledged =
            terminalLaserOffCommandToken_ > 0 &&
            laserAcknowledgedOffCommandToken_ >=
                terminalLaserOffCommandToken_;
        if (state == LineLaserConnectionState::Ready &&
            !currentOffAcknowledged) {
            terminalLaserOffRequestedNs_ =
                hik_sync::getMonotonicRawNs();
            terminalLaserOffCommandToken_ = requestLaserOff();
        }
        tryCompleteTerminalBarrier();
    }
    updateUi();
}

void HikConstantLaserScanWindow::onLaserStatusChanged(
        LineLaserStatus status) {
    if (status.sourceMonotonicNs <= 0 || status.eventSequence == 0) {
        appendLog(QStringLiteral(
            "忽略缺少源单调时间或事件序号的 TTL 状态。"));
        return;
    }
    if (status.transportGeneration < laserTransportGeneration_ ||
        status.eventSequence <= laserStatusEventSequence_) {
        appendLog(QStringLiteral(
            "忽略过期 TTL 状态：transport=%1, sequence=%2；"
            "当前 transport=%3, sequence=%4。")
            .arg(status.transportGeneration)
            .arg(status.eventSequence)
            .arg(laserTransportGeneration_)
            .arg(laserStatusEventSequence_));
        return;
    }
    if (status.transportGeneration > laserTransportGeneration_) {
        // An ACK is meaningful only within the SSH transport that carried its
        // request/reply pair.  Reconnect starts a fresh causal domain.
        laserAcknowledgedOffCommandToken_ = 0;
    }
    laserTransportGeneration_ = status.transportGeneration;
    laserStatusEventSequence_ = status.eventSequence;
    laserAcknowledgedOffCommandToken_ = std::max(
        laserAcknowledgedOffCommandToken_,
        status.acknowledgedOffCommandToken);
    laserStatus_ = status;
    laserStatusReceivedMonotonicMs_ =
        status.sourceMonotonicNs / 1000000LL;
    if (!status.reachable) {
        laserStatusLabel_->setText(QStringLiteral(
            "鲁班猫 TTL 状态未知；断线租约会强制两路 LOW"));
        laserStatusLabel_->setStyleSheet(QStringLiteral(
            "color:#b00020;font-weight:bold;"));
    } else {
        const bool ready = laserReadyForProfile();
        laserStatusLabel_->setText(QStringLiteral(
            "%1｜本组 Pin %2｜租约 %3 ms｜daemon=%4")
            .arg(laserStateText(status.state))
            .arg(profile_.ttlPhysicalPin)
            .arg(status.leaseRemainingMs)
            .arg(status.daemonGeneration.left(12)));
        laserStatusLabel_->setStyleSheet(ready
            ? QStringLiteral("color:#1b5e20;font-weight:bold;")
            : status.state == LineLaserState::Off
                ? QStringLiteral("color:#455a64;font-weight:bold;")
                : QStringLiteral("color:#e65100;font-weight:bold;"));
    }
    const bool laserHighRequired =
        scanState_ != ScanState::Idle ||
        continuousState_ == ContinuousState::MovingToStart ||
        continuousState_ == ContinuousState::StartingCamera ||
        continuousState_ == ContinuousState::Scanning;
    if (scanActivityHeld_ && !terminalBarrierActive_ &&
        laserHighRequired && !laserReadyForProfile()) {
        abortForLaserSafety(QStringLiteral(
            "板端不再确认本组 TTL HIGH 且另一路 LOW。"));
    }
    if (terminalBarrierActive_) tryCompleteTerminalBarrier();
    updateUi();
}

void HikConstantLaserScanWindow::onLaserCommandFinished(
        QString command, bool success, QString detail) {
    appendLog(QStringLiteral("TTL命令 %1：%2；%3")
        .arg(command, success ? QStringLiteral("成功")
                              : QStringLiteral("失败"),
             detail));
    updateUi();
}

void HikConstantLaserScanWindow::onLaserFault(QString detail) {
    appendLog(QStringLiteral("TTL控制故障：%1").arg(detail));
    if (scanActivityHeld_ && !terminalBarrierActive_) {
        abortForLaserSafety(QStringLiteral("TTL 控制故障：%1").arg(detail));
    }
}

void HikConstantLaserScanWindow::abortForLaserSafety(
        const QString& reason) {
    if (laserSafetyAbortIssued_) return;
    laserSafetyAbortIssued_ = true;
    requestLaserOff();
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(reason, pendingMotionRequestId_ >= 0);
    } else if (scanState_ != ScanState::Idle) {
        abortScan(reason, pendingMotionRequestId_ >= 0);
    }
}

void HikConstantLaserScanWindow::connectCamera() {
    if (!profileTabActive_) {
        showError(QStringLiteral("相机连接"),
                  QStringLiteral("请先切换到设备组 %1。").arg(profile_.displayName));
        return;
    }
    const QString ip = cameraIpEdit_->text().trimmed();
    if (!validIpv4(ip)) {
        showError(QStringLiteral("相机连接"),
                  QStringLiteral("设备组 %1 的相机 IP 无效。")
                      .arg(profile_.displayName));
        return;
    }
    emit requestConnectCamera(ip);
}

void HikConstantLaserScanWindow::disconnectCamera() { emit requestDisconnectCamera(); }

void HikConstantLaserScanWindow::connectRobot() {
    if (!profileTabActive_) {
        showError(QStringLiteral("FR5 连接"),
                  QStringLiteral("请先切换到设备组 %1。").arg(profile_.displayName));
        return;
    }
    const QString ip = robotIpEdit_->text().trimmed();
    if (!validIpv4(ip)) { showError(QStringLiteral("FR5 连接"), QStringLiteral("机器人 IP 无效。")); return; }
    if (!robotSession_ || !robotSession_->isRunning()) {
        showError(QStringLiteral("FR5 连接"),
                  QStringLiteral("共享 FR5 会话不可用。"));
        return;
    }
    robotBusy_ = true;
    robotStatusLabel_->setText(
        QStringLiteral("正在建立共享连接 %1 …").arg(ip));
    emit requestConnectRobot(robotClientId_, ip);
    updateUi();
}

void HikConstantLaserScanWindow::disconnectRobot() {
    emit requestDisconnectRobot(robotClientId_);
}

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
    intrinsicsPath_ = profile_.intrinsicsConfigPath(sourceDir_);
    laserPlanePath_ = profile_.laserPlaneConfigPath(sourceDir_);
    handEyePath_ = profile_.handEyeConfigPath(sourceDir_);
    QStringList missingFiles;
    const QStringList formalFiles = {
        intrinsicsPath_, laserPlanePath_, handEyePath_
    };
    for (const QString& path : formalFiles) {
        if (!QFileInfo(path).isFile()) missingFiles.push_back(path);
    }
    if (!missingFiles.isEmpty()) {
        const QString message = QStringLiteral(
            "设备组 %1 的三维标定未就绪，缺少独立正式文件：%2。"
            "本组仍可连接相机确认身份和连接状态，但单点、停稳和连续建图均已禁用。")
            .arg(profile_.displayName, missingFiles.join(QStringLiteral("；")));
        calibrationStatusLabel_->setText(message);
        calibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        if (error) *error = message;
        return false;
    }
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
    if (intrinsicFrame != profile_.cameraFrame.trimmed()) {
        if (error) {
            *error = QStringLiteral(
                "正式标定 frame=%1，不属于当前 profile 要求的 frame=%2。")
                .arg(intrinsicFrame, profile_.cameraFrame);
        }
        return false;
    }
    if (intrinsicSerial.isEmpty() || handEyeSerial != intrinsicSerial) {
        if (error) *error = QStringLiteral("内参与手眼相机序列号不一致：%1 / %2。")
            .arg(intrinsicSerial, handEyeSerial);
        return false;
    }
    if (!profile_.expectedCameraSerial.trimmed().isEmpty() &&
        intrinsicSerial != profile_.expectedCameraSerial.trimmed()) {
        if (error) {
            *error = QStringLiteral(
                "设备 profile 期望相机 SN=%1，但正式内参/手眼属于 SN=%2。")
                .arg(profile_.expectedCameraSerial, intrinsicSerial);
        }
        return false;
    }
    const QString intrinsicModel =
        QString::fromStdString(intrinsicsMetadata_.cameraModel).trimmed();
    if (!profile_.expectedCameraModel.trimmed().isEmpty() &&
        intrinsicModel != profile_.expectedCameraModel.trimmed()) {
        if (error) {
            *error = QStringLiteral(
                "设备 profile 期望相机型号=%1，但正式内参属于型号=%2。")
                .arg(profile_.expectedCameraModel, intrinsicModel);
        }
        return false;
    }
    profileOptions_.reconstruction.minimumDepthMm = laserMetadata_.validCameraZMinMm;
    profileOptions_.reconstruction.maximumDepthMm = laserMetadata_.validCameraZMaxMm;
    profileOptions_.reconstruction.stripe.quality.roi =
        stripeRoi(profile_, intrinsics_.imageSize);
    profileOptions_.reconstruction.stripeValidityMask.release();
    if (profileOptions_.reconstruction.stripe.mode !=
            hik_calibration::StripeExtractionMode::Legacy &&
        !hik_calibration::buildLaserPlaneValidityMask(
            intrinsics_.imageSize, intrinsics_, laserPlane_,
            profileOptions_.reconstruction.minimumDepthMm,
            profileOptions_.reconstruction.maximumDepthMm,
            profileOptions_.reconstruction.stripe.quality.roi,
            &profileOptions_.reconstruction.stripeValidityMask,
            &coreError)) {
        if (error) {
            *error = QStringLiteral("无法建立条纹有效深度走廊: %1")
                .arg(QString::fromStdString(coreError));
        }
        return false;
    }
    calibrationReady_ = true;
    calibrationStatusLabel_->setText(QStringLiteral(
        "%1 已校验：相机 SN=%2，图像=%3×%4，Z=%5–%6 mm，输出 base_link；"
        "常亮背景核=%7×%8；质量走廊=%9 px。")
        .arg(profile_.id)
        .arg(QString::fromStdString(intrinsicsMetadata_.cameraSerial))
        .arg(intrinsics_.imageSize.width).arg(intrinsics_.imageSize.height)
        .arg(profileOptions_.reconstruction.minimumDepthMm, 0, 'f', 2)
        .arg(profileOptions_.reconstruction.maximumDepthMm, 0, 'f', 2)
        .arg(profileOptions_.backgroundKernelWidth)
        .arg(profileOptions_.backgroundKernelHeight)
        .arg(profileOptions_.reconstruction.stripeValidityMask.empty()
             ? 0
             : cv::countNonZero(
                   profileOptions_.reconstruction.stripeValidityMask)));
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
    const QString expectedSerial = profile_.expectedCameraSerial.trimmed();
    const QString expectedModel = profile_.expectedCameraModel.trimmed();
    if ((!expectedSerial.isEmpty() && cameraSerial_ != expectedSerial) ||
        (!expectedModel.isEmpty() && cameraModel_ != expectedModel)) {
        if (error) {
            *error = QStringLiteral(
                "当前设备与 profile 不匹配：model=%1（期望 %2），SN=%3（期望 %4）。")
                .arg(cameraModel_,
                     expectedModel.isEmpty() ? QStringLiteral("未限定") : expectedModel,
                     cameraSerial_,
                     expectedSerial.isEmpty() ? QStringLiteral("未限定") : expectedSerial);
        }
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
    if (!laserReadyForProfile(&error)) {
        showError(QStringLiteral("激光 TTL 未就绪"), error);
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
    if (!cameraConnected_ || !robotConnected_) {
        showError(QStringLiteral("设备状态已变化"),
                  QStringLiteral("确认期间相机或 FR5 已断开。"));
        return;
    }
    if (!laserReadyForProfile(&error)) {
        showError(QStringLiteral("设备状态已变化"), error);
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error) ||
        !calibrationIdentityMatches(&error)) {
        showError(QStringLiteral("标定状态已变化"), error);
        return;
    }
    if (!acquireScanActivity(&error)) {
        showError(QStringLiteral("设备组互斥"), error);
        return;
    }
    scanSessionDir_.clear();
    if (!createScanSession(&error)) {
        scanState_ = ScanState::Idle;
        safetyConfirmCheck_->setChecked(false);
        beginTerminalBarrier(
            false,
            QStringLiteral("创建停稳扫描会话失败：%1").arg(error),
            QStringLiteral("stop_and_shoot_setup"),
            scanSessionDir_);
        showError(QStringLiteral("创建扫描会话失败"), error);
        return;
    }
    cloud_.clear();
    qualityCloud_.clear();
    qualitySupportResult_ =
        hik_scan::AdjacentProfileSupportResult();
    qualityVoxelPointCount_ = 0U;
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
    if (!laserReadyForProfile(&error)) {
        showError(QStringLiteral("激光 TTL 未就绪"), error);
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
    profileOptions_.reconstruction.maxLineRmsMm =
        lineRmsLimitSpin_->value();
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
    if (!cameraConnected_ || !robotConnected_) {
        showError(QStringLiteral("设备状态已变化"),
                  QStringLiteral("确认期间相机或 FR5 已断开。"));
        return;
    }
    if (!laserReadyForProfile(&error)) {
        showError(QStringLiteral("设备状态已变化"), error);
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error) ||
        !calibrationIdentityMatches(&error)) {
        showError(QStringLiteral("标定状态已变化"), error);
        return;
    }
    if (!acquireScanActivity(&error)) {
        showError(QStringLiteral("设备组互斥"), error);
        return;
    }
    continuousAbortRequested_ = false;
    synchronizationSessionDir_.clear();
    continuousState_ = ContinuousState::MovingToStart;
    pendingMotionRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
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
    outputRoot = QDir(outputRoot).absoluteFilePath(profile_.id);
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
    if (!writeSessionMetadata(synchronizationSessionDir_, error)) {
        synchronizationSession_.stop();
        return false;
    }
    hik_scan::ContinuousReconstructionOptions reconstructionOptions;
    reconstructionOptions.queueCapacity =
        synchronizationConfig_.reconstructionQueueCapacity;
    reconstructionOptions.workerThreads =
        synchronizationConfig_.reconstructionThreads;
    reconstructionOptions.voxelSizeMm = voxelSpin_->value();
    reconstructionOptions.outputDirectory =
        localPath(synchronizationSessionDir_);
    reconstructionOptions.intrinsicsSha256 =
        localPath(intrinsicsSha256_);
    reconstructionOptions.laserPlaneSha256 =
        localPath(laserPlaneSha256_);
    reconstructionOptions.handEyeSha256 =
        localPath(handEyeSha256_);
    const bool qualityCenterlineEnabled =
        profileOptions_.reconstruction.stripe.mode !=
        hik_calibration::StripeExtractionMode::Legacy;
    const double nominalProfileSpacingMm =
        synchronizationConfig_.scanSpeedMmS /
        synchronizationConfig_.cameraTargetFps;
    reconstructionOptions.saveQualityCloud =
        qualityCenterlineEnabled;
    reconstructionOptions.enableAdjacentProfileSupport =
        qualityCenterlineEnabled;
    reconstructionOptions.adjacentSupportRadiusMm = std::max(
        kQualitySupportRadiusFloorMm,
        std::max(
            kQualitySupportStepFactor * nominalProfileSpacingMm,
            kQualitySupportVoxelFactor * voxelSpin_->value()));
    reconstructionOptions.adjacentMinimumSupportingProfiles =
        kQualitySupportMinimumProfiles;
    reconstructionOptions.adjacentMaximumProfileGap =
        kQualitySupportMaximumProfileGap;
    if (!continuousReconstruction_.start(
            reconstructionOptions, intrinsics_, laserPlane_,
            profileOptions_, &coreError)) {
        synchronizationSession_.stop();
        if (error) {
            *error = QStringLiteral("连续点云后台流水线启动失败：%1")
                .arg(QString::fromStdString(coreError));
        }
        return false;
    }
    synchronizationSession_.setSynchronizedFrameCallback(
        [this](const hik_sync::SynchronizedFrame& frame) {
            // Deliberately best-effort and non-blocking. Reconstruction queue
            // pressure must never propagate to camera or FR5 receive threads.
            (void)continuousReconstruction_.tryEnqueue(frame);
        });
    const double spacing = synchronizationConfig_.scanSpeedMmS /
                           synchronizationConfig_.cameraTargetFps;
    const double motionDuringExposure = synchronizationConfig_.scanSpeedMmS *
        synchronizationConfig_.cameraExposureUs / 1.0e6;
    appendLog(QStringLiteral(
        "同步会话=%1；相机初始模式=HOST_CALLBACK_FALLBACK（设备映射稳定后自动切换）；"
        "机器人初始实际模式=HOST_RECEIVE、请求模式=%2（拟合稳定后自动切换）；"
        "CNDE请求周期=%3 ms、预期实际反馈周期=%4 ms；"
        "理论线间距=%5 mm/帧；曝光运动量=%6 mm；PNG并行写线程=%7；"
        "连续重建线程=%8、非阻塞队列=%9。")
        .arg(synchronizationSessionDir_)
        .arg(QString::fromLatin1(hik_sync::robotTimeModeName(
            synchronizationConfig_.robotTimeMode)))
        .arg(synchronizationConfig_.robotPeriodMs, 0, 'f', 3)
        .arg(synchronizationConfig_.robotExpectedFeedbackPeriodMs, 0, 'f', 3)
        .arg(spacing, 0, 'f', 6)
        .arg(motionDuringExposure, 0, 'f', 6)
        .arg(synchronizationConfig_.imageWriterThreads)
        .arg(synchronizationConfig_.reconstructionThreads)
        .arg(synchronizationConfig_.reconstructionQueueCapacity));
    if (qualityCenterlineEnabled) {
        appendLog(QStringLiteral(
            "连续质量点云并行开启：二维硬门控后保存 optical，"
            "再以 radius=%1 mm、min_profiles=%2、max_gap=%3 做相邻 "
            "profile 支持；不会覆盖正式 continuous_raw/voxel。")
            .arg(reconstructionOptions.adjacentSupportRadiusMm, 0, 'f', 3)
            .arg(reconstructionOptions.adjacentMinimumSupportingProfiles)
            .arg(reconstructionOptions.adjacentMaximumProfileGap));
    }
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
    const quint64 generation = scanGeneration_;
    QTimer::singleShot(100, this, [this, generation]() {
        if (generation != scanGeneration_ ||
            continuousState_ != ContinuousState::StartingCamera ||
            continuousAbortRequested_) return;
        continuousState_ = ContinuousState::Scanning;
        pendingMotionRequestId_ =
            robotSession_->allocateRequestId(robotClientId_);
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

void HikConstantLaserScanWindow::onContinuousCameraStopped(
        bool confirmed, QString description) {
    if (continuousState_ != ContinuousState::Stopping) return;
    if (!confirmed) {
        terminalCameraFault_ = true;
        terminalCameraFaultDetail_ = description;
        continuousAbortRequested_ = true;
        finalizeContinuousScan(
            false,
            QStringLiteral("相机连续停流未确认：%1")
                .arg(description));
        return;
    }
    const quint64 generation = scanGeneration_;
    QTimer::singleShot(60, this, [this, generation]() {
        if (generation != scanGeneration_ ||
            continuousState_ != ContinuousState::Stopping) return;
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
    requestLaserOff();
    appendLog(QStringLiteral("连续同步扫描终止请求：%1").arg(reason));
    continuousAbortRequested_ = true;
    const ContinuousState previous = continuousState_;
    const bool motionWasPending = pendingMotionRequestId_ >= 0;
    if (motionWasPending) {
        if (requestStop && robotConnected_) {
            emit requestStopMotion(
                robotSession_->allocateRequestId(robotClientId_));
        } else {
            terminalMotionFault_ = true;
            terminalMotionFaultDetail_ = QStringLiteral(
                "连续扫描运动仍在活动，但当前无法确认 StopMotion：%1")
                .arg(reason);
            pendingMotionRequestId_ = -1;
        }
    }
    if (previous == ContinuousState::StartingCamera ||
        previous == ContinuousState::Scanning ||
        previous == ContinuousState::Stopping) {
        continuousState_ = ContinuousState::Stopping;
        emit requestStopContinuous();
    } else if (previous == ContinuousState::MovingToStart &&
               pendingMotionRequestId_ < 0) {
        finalizeContinuousScan(false, reason);
    }
    scanStatusLabel_->setText(QStringLiteral("正在安全停止连续同步扫描：%1").arg(reason));
    updateUi();
}

void HikConstantLaserScanWindow::finalizeContinuousScan(
        bool completed, const QString& reason) {
    requestLaserOff();
    const bool sessionStarted = synchronizationSession_.running();
    if (sessionStarted) synchronizationSession_.stop();
    const hik_sync::PipelineStatistics stats = sessionStarted
        ? synchronizationSession_.statistics() : hik_sync::PipelineStatistics{};
    const QString mode = sessionStarted
        ? QString::fromStdString(synchronizationSession_.clockModeDescription())
        : QStringLiteral("同步会话未启动");
    hik_scan::ContinuousReconstructionStatistics reconstructionStats;
    std::string reconstructionError;
    const bool reconstructionWasRunning = continuousReconstruction_.running();
    bool cloudSaved = false;
    if (reconstructionWasRunning) {
        scanStatusLabel_->setText(QStringLiteral(
            "同步数据已清空，正在等待后台条纹重建完成并保存连续点云……"));
        appendLog(scanStatusLabel_->text());
        cloudSaved = continuousReconstruction_.stopAndSave(
            &reconstructionStats, &reconstructionError);
    }
    continuousState_ = ContinuousState::Idle;
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
    if (reconstructionStats.synchronizedFramesSeen > 0U) {
        appendLog(QStringLiteral(
            "连续点云：seen=%1，sync_invalid_skip=%2，reconstructed=%3，"
            "reconstruct_fail=%4，line_quality_warning=%5，"
            "missing_image_skip=%6，invalid_transform_skip=%7，"
            "queue_full_drop=%8，queue_contention_drop=%9，"
            "reconstruct_mean=%10 ms，max=%11 ms，"
            "raw_points=%12，voxel_points=%13；raw=%14；voxel=%15。")
            .arg(reconstructionStats.synchronizedFramesSeen)
            .arg(reconstructionStats.invalidSyncFramesSkipped)
            .arg(reconstructionStats.reconstructedFrames)
            .arg(reconstructionStats.reconstructionFailures)
            .arg(reconstructionStats.lineQualityWarnings)
            .arg(reconstructionStats.missingImageFramesSkipped)
            .arg(reconstructionStats.invalidTransformFramesSkipped)
            .arg(reconstructionStats.queueFullDrops)
            .arg(reconstructionStats.queueContentionDrops)
            .arg(reconstructionStats.meanReconstructionMs, 0, 'f', 3)
            .arg(reconstructionStats.maximumReconstructionMs, 0, 'f', 3)
            .arg(reconstructionStats.rawPointCount)
            .arg(reconstructionStats.voxelPointCount)
            .arg(QString::fromStdString(reconstructionStats.rawPlyPath))
            .arg(QString::fromStdString(reconstructionStats.voxelPlyPath)));
        if (reconstructionStats.qualityFramesPassed > 0U ||
            reconstructionStats.qualityFramesRejected > 0U) {
            appendLog(QStringLiteral(
                "连续质量结果：frames passed/rejected=%1/%2，"
                "optical/filtered/rejected/voxel 点=%3/%4/%5/%6；"
                "optical=%7；filtered=%8；rejected=%9；voxel=%10。")
                .arg(reconstructionStats.qualityFramesPassed)
                .arg(reconstructionStats.qualityFramesRejected)
                .arg(reconstructionStats.qualityOpticalPointCount)
                .arg(reconstructionStats.qualityFilteredPointCount)
                .arg(reconstructionStats.qualityRejectedPointCount)
                .arg(reconstructionStats.qualityVoxelPointCount)
                .arg(QString::fromStdString(
                    reconstructionStats.qualityOpticalPlyPath))
                .arg(QString::fromStdString(
                    reconstructionStats.qualityPlyPath))
                .arg(QString::fromStdString(
                    reconstructionStats.qualityRejectedPlyPath))
                .arg(QString::fromStdString(
                    reconstructionStats.qualityVoxelPlyPath)));
            if (profile_.scanCenterlinePolicy ==
                LineLaserCenterlinePolicy::Shadow) {
                appendLog(QStringLiteral(
                    "650/shadow 保护：连续质量文件仅供并行比较；"
                    "正式建图仍使用 continuous_raw/continuous_voxel。"));
            }
        }
        if (!cloudSaved) {
            appendLog(QStringLiteral("警告：连续点云保存未完整成功：%1；详情=%2")
                .arg(QString::fromStdString(reconstructionError),
                     QString::fromStdString(reconstructionStats.detailCsvPath)));
        }
        if (reconstructionStats.queueFullDrops > 0U ||
            reconstructionStats.queueContentionDrops > 0U) {
            appendLog(QStringLiteral(
                "警告：连续重建采用非阻塞丢弃策略，共跳过%1帧；"
                "相机采集、FR5接收、同步CSV和原图保存未被反压。")
                .arg(reconstructionStats.queueFullDrops +
                     reconstructionStats.queueContentionDrops));
        }
    }
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
    const double expectedRobotHz =
        1000.0 / synchronizationConfig_.robotExpectedFeedbackPeriodMs;
    if (stats.actualRobotHz > 0.0 &&
        std::abs(stats.actualRobotHz - expectedRobotHz) > expectedRobotHz * 0.10) {
        appendLog(QStringLiteral(
            "警告：FR5 20004完整包实测=%1 Hz，与预期实际反馈周期 %2 ms（%3 Hz）"
            "偏差超过 10%；CNDE请求周期=%4 ms；"
            "到达时刻来自SDK RecvPkg成功返回处的 CLOCK_MONOTONIC_RAW；"
            "frame_cnt仅作控制器内部计数诊断。")
            .arg(stats.actualRobotHz, 0, 'f', 3)
            .arg(synchronizationConfig_.robotExpectedFeedbackPeriodMs, 0, 'f', 3)
            .arg(expectedRobotHz, 0, 'f', 3)
            .arg(synchronizationConfig_.robotPeriodMs, 0, 'f', 3));
    } else if (stats.actualRobotHz > 0.0) {
        appendLog(QStringLiteral(
            "FR5 20004完整包实测=%1 Hz，符合当前控制器预期实际反馈周期 %2 ms"
            "（%3 Hz）；CNDE请求周期=%4 ms，frame_cnt仅作诊断。")
            .arg(stats.actualRobotHz, 0, 'f', 3)
            .arg(synchronizationConfig_.robotExpectedFeedbackPeriodMs, 0, 'f', 3)
            .arg(expectedRobotHz, 0, 'f', 3)
            .arg(synchronizationConfig_.robotPeriodMs, 0, 'f', 3));
    }
    QJsonObject terminalStats;
    terminalStats.insert(QStringLiteral("total_camera_frames"),
                         static_cast<double>(stats.totalCameraFrames));
    terminalStats.insert(QStringLiteral("camera_frames_dropped"),
                         static_cast<double>(stats.cameraFramesDropped));
    terminalStats.insert(QStringLiteral("successful_sync_frames"),
                         static_cast<double>(stats.successfulSyncFrames));
    terminalStats.insert(QStringLiteral("invalid_sync_frames"),
                         static_cast<double>(stats.invalidSyncFrames));
    terminalStats.insert(QStringLiteral("stable_speed_frames"),
                         static_cast<double>(stats.stableSpeedFrames));
    terminalStats.insert(QStringLiteral("image_frames_written"),
                         static_cast<double>(stats.imageFramesWritten));
    terminalStats.insert(QStringLiteral("image_write_failures"),
                         static_cast<double>(stats.imageWriteFailures));
    terminalStats.insert(QStringLiteral("reconstructed_frames"),
                         static_cast<double>(
                             reconstructionStats.reconstructedFrames));
    terminalStats.insert(QStringLiteral("raw_point_count"),
                         static_cast<double>(
                             reconstructionStats.rawPointCount));
    terminalStats.insert(QStringLiteral("voxel_point_count"),
                         static_cast<double>(
                             reconstructionStats.voxelPointCount));
    terminalStats.insert(QStringLiteral("cloud_saved"),
                         !reconstructionWasRunning || cloudSaved);

    const bool finalCompleted =
        completed && (!reconstructionWasRunning || cloudSaved);
    QString finalReason = reason;
    if (completed && reconstructionWasRunning && !cloudSaved) {
        finalReason += QStringLiteral("；连续点云保存失败：%1")
            .arg(QString::fromStdString(reconstructionError));
    }
    continuousAbortRequested_ = false;
    beginTerminalBarrier(
        finalCompleted, finalReason, QStringLiteral("continuous"),
        synchronizationSessionDir_, terminalStats);
}

void HikConstantLaserScanWindow::captureCurrentProfile() {
    QString error;
    profileOptions_.reconstruction.maxLineRmsMm = lineRmsLimitSpin_->value();
    if (!cameraConnected_ || !robotConnected_ || !calibrationReady_) {
        showError(QStringLiteral("无法单点采集"), QStringLiteral("请连接相机、FR5并加载正式标定。"));
        return;
    }
    if (!laserReadyForProfile(&error)) {
        showError(QStringLiteral("激光 TTL 未就绪"), error);
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error)) {
        showError(QStringLiteral("标定文件已变化"), error);
        return;
    }
    if (!calibrationIdentityMatches(&error)) { showError(QStringLiteral("相机身份不匹配"), error); return; }
    if (!acquireScanActivity(&error)) {
        showError(QStringLiteral("设备组互斥"), error);
        return;
    }
    scanSessionDir_.clear();
    if (!createScanSession(&error)) {
        scanState_ = ScanState::Idle;
        safetyConfirmCheck_->setChecked(false);
        beginTerminalBarrier(
            false,
            QStringLiteral("创建单点扫描会话失败：%1").arg(error),
            QStringLiteral("single_point_setup"),
            scanSessionDir_);
        showError(QStringLiteral("创建扫描会话失败"), error);
        return;
    }
    cloud_.clear();
    qualityCloud_.clear();
    qualitySupportResult_ =
        hik_scan::AdjacentProfileSupportResult();
    qualityVoxelPointCount_ = 0U;
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
    pendingRobotRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
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
    pendingMotionRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
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
    if (scanState_ != ScanState::Settling || stopRequested_) return;
    scanState_ = ScanState::ReadingBefore;
    issueRobotRead(ReadRole::ScanBefore);
}

void HikConstantLaserScanWindow::onCameraConnectionChanged(bool connected, QString description) {
    cameraConnected_ = connected;
    cameraStatusLabel_->setText(QStringLiteral("%1：%2").arg(profile_.id, description));
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
    appendLog(QStringLiteral("相机身份：model=%1, SN=%2, IP=%3；profile=%4")
              .arg(model, serial, ipAddress, profile_.id));
    const QString expectedModel = profile_.expectedCameraModel.trimmed();
    const QString expectedSerial = profile_.expectedCameraSerial.trimmed();
    if ((!expectedModel.isEmpty() && model != expectedModel) ||
        (!expectedSerial.isEmpty() && serial != expectedSerial)) {
        appendLog(QStringLiteral(
            "警告：相机身份不属于当前设备 profile；三维扫描会被硬性禁止。"));
    }
}

void HikConstantLaserScanWindow::onCameraBusyChanged(bool busy) {
    cameraBusy_ = busy;
    if (terminalBarrierActive_) tryCompleteTerminalBarrier();
    updateUi();
}
void HikConstantLaserScanWindow::onCameraLog(QString message) { appendLog(QStringLiteral("相机: %1").arg(message)); }

void HikConstantLaserScanWindow::onCameraError(int requestId, QString message) {
    const bool pendingSingleCapture =
        requestId == pendingCameraRequestId_;
    if (pendingSingleCapture) pendingCameraRequestId_ = -1;
    if (pendingSingleCapture && terminalBarrierActive_ &&
        scanState_ == ScanState::Idle &&
        continuousState_ == ContinuousState::Idle) {
        appendLog(QStringLiteral(
            "终止后的单帧请求已返回错误 ACK：%1").arg(message));
        tryCompleteTerminalBarrier();
        updateUi();
        return;
    }
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("相机错误: %1").arg(message), true);
    } else if (scanState_ != ScanState::Idle) {
        abortScan(QStringLiteral("相机错误: %1").arg(message),
                  pendingMotionRequestId_ >= 0);
    }
    else showError(QStringLiteral("相机错误"), message);
    updateUi();
}

void HikConstantLaserScanWindow::onRobotConnectionChanged(bool connected, QString description) {
    robotConnected_ = connected;
    robotStatusLabel_->setText(QStringLiteral("%1：%2").arg(profile_.id, description));
    robotStatusLabel_->setStyleSheet(connected ? QStringLiteral("color:#087f23;") : QStringLiteral("color:#b00020;"));
    if (!connected && robotSession_->connectionAttempted()) {
        appendLog(QStringLiteral(
            "共享 FAIRINO RPC 会话已结束；受 SDK 3.9.4 生命周期限制，"
            "本进程不再创建第二个 FRRobot，重新连接需重启程序。"));
    }
    if (!connected && terminalBarrierActive_ &&
        pendingMotionRequestId_ >= 0) {
        terminalMotionFault_ = true;
        terminalMotionFaultDetail_ = QStringLiteral(
            "等待运动停止确认时 FR5 连接断开：%1").arg(description);
        pendingMotionRequestId_ = -1;
        QString resultError;
        if (!writeSessionResult(&resultError) && !resultError.isEmpty()) {
            appendLog(QStringLiteral("警告：无法记录 FR5 断连故障：%1")
                          .arg(resultError));
        }
        tryCompleteTerminalBarrier();
    }
    if (!connected && continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("连续同步扫描中 FR5 断开。"), false);
    } else if (!connected && scanState_ != ScanState::Idle) {
        abortScan(QStringLiteral("扫描中 FR5 断开。"), false);
    }
    updateUi();
}

void HikConstantLaserScanWindow::onRobotBusyChanged(bool busy) {
    robotBusy_ = busy;
    if (terminalBarrierActive_) tryCompleteTerminalBarrier();
    updateUi();
}
void HikConstantLaserScanWindow::onRobotLog(QString message) { appendLog(QStringLiteral("FR5: %1").arg(message)); }

void HikConstantLaserScanWindow::onRobotError(int requestId, QString message) {
    if (requestId >= 0 &&
        !robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    if (requestId < 0 && !profileTabActive_) {
        appendLog(QStringLiteral("共享 FR5 错误：%1").arg(message));
        return;
    }
    if (requestId == pendingRobotRequestId_) { pendingRobotRequestId_ = -1; readRole_ = ReadRole::None; }
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(QStringLiteral("FR5 错误: %1").arg(message), true);
    } else if (scanState_ != ScanState::Idle) abortScan(QStringLiteral("FR5 错误: %1").arg(message), true);
    else {
        showError(QStringLiteral("FR5 错误"), message);
    }
    updateUi();
}

void HikConstantLaserScanWindow::onRobotClientError(
        int clientId, QString message) {
    if (clientId != robotClientId_) return;
    robotBusy_ = false;
    if (continuousState_ != ContinuousState::Idle) {
        abortContinuousScan(
            QStringLiteral("共享 FR5 错误: %1").arg(message), true);
    } else if (scanState_ != ScanState::Idle) {
        abortScan(QStringLiteral("共享 FR5 错误: %1").arg(message), true);
    } else {
        showError(QStringLiteral("共享 FR5 错误"), message);
    }
    updateUi();
}

void HikConstantLaserScanWindow::onRobotFlangePoseReady(
        int requestId, double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg, qint64 hostTimestampMs) {
    if (!robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
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
    if (!robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    if (requestId == pendingMotionRequestId_) appendLog(QStringLiteral("FR5: %1").arg(description));
}

void HikConstantLaserScanWindow::onRobotMotionFinished(
        int requestId,
        bool targetReached,
        bool motionStoppedConfirmed,
        QString description) {
    if (!robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    if (requestId != pendingMotionRequestId_) {
        appendLog(QStringLiteral(
            "忽略非当前运动请求的完成信号：request=%1，current=%2，%3")
            .arg(requestId).arg(pendingMotionRequestId_).arg(description));
        return;
    }
    pendingMotionRequestId_ = -1;
    appendLog(QStringLiteral("FR5: %1").arg(description));
    if (!motionStoppedConfirmed) {
        terminalMotionFault_ = true;
        terminalMotionFaultDetail_ = description;
        QString resultError;
        if (!writeSessionResult(&resultError) && !resultError.isEmpty()) {
            appendLog(QStringLiteral("警告：无法更新运动停止故障：%1")
                          .arg(resultError));
        }
    }
    if (terminalBarrierActive_) {
        tryCompleteTerminalBarrier();
        updateUi();
        return;
    }
    if (continuousState_ == ContinuousState::MovingToStart) {
        if (!targetReached || continuousAbortRequested_) {
            finalizeContinuousScan(false, description);
            return;
        }
        QString readinessError;
        if (!laserReadyForProfile(&readinessError) ||
            !formalCalibrationFilesUnchanged(&readinessError) ||
            !calibrationIdentityMatches(&readinessError)) {
            continuousState_ = ContinuousState::Idle;
            safetyConfirmCheck_->setChecked(false);
            beginTerminalBarrier(
                false,
                QStringLiteral("到达连续扫描起点后安全复查失败：%1")
                    .arg(readinessError),
                QStringLiteral("continuous_setup"),
                synchronizationSessionDir_);
            showError(QStringLiteral("到达起点后安全复查失败"),
                      readinessError);
            return;
        }
        QString sessionError;
        if (!createSynchronizationSession(&sessionError)) {
            continuousState_ = ContinuousState::Idle;
            safetyConfirmCheck_->setChecked(false);
            beginTerminalBarrier(
                false,
                QStringLiteral("创建连续同步会话失败：%1")
                    .arg(sessionError),
                QStringLiteral("continuous_setup"),
                synchronizationSessionDir_);
            showError(QStringLiteral("创建同步会话失败"), sessionError);
            return;
        }
        continuousState_ = ContinuousState::StartingCamera;
        scanStatusLabel_->setText(QStringLiteral(
            "已到连续扫描起点；预采机器人状态后启动相机。"));
        const quint64 generation = scanGeneration_;
        QTimer::singleShot(100, this, [this, generation]() {
            if (generation != scanGeneration_ ||
                continuousState_ != ContinuousState::StartingCamera) return;
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
        requestLaserOff();
        continuousState_ = ContinuousState::Stopping;
        continuousAbortRequested_ =
            !targetReached || continuousAbortRequested_;
        emit requestStopContinuous();
        scanStatusLabel_->setText(targetReached
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
    if (!targetReached || stopRequested_) {
        abortScan(description, false);
        return;
    }
    scanState_ = ScanState::Settling;
    scanStatusLabel_->setText(QStringLiteral("已到位，等待结构停稳 %1 ms。")
                              .arg(settleSpin_->value()));
    const quint64 generation = scanGeneration_;
    QTimer::singleShot(settleSpin_->value(), this,
        [this, generation]() {
            if (generation != scanGeneration_) return;
            beginSettledCapture();
        });
    updateUi();
}

void HikConstantLaserScanWindow::onCameraFrameReady(
        int requestId, QImage image, quint64 frameNo, quint64 deviceTimestamp,
        qint64 hostTimestamp, double actualExposure, double actualGain,
        QString description) {
    if (requestId != pendingCameraRequestId_) return;
    pendingCameraRequestId_ = -1;
    if (scanState_ != ScanState::Capturing) {
        appendLog(QStringLiteral(
            "终止后的单帧请求已返回 frame ACK，图像不进入点云。"));
        if (terminalBarrierActive_) tryCompleteTerminalBarrier();
        updateUi();
        return;
    }
    QString laserError;
    if (!laserReadyForProfile(&laserError)) {
        abortScan(QStringLiteral(
            "相机帧回调时 TTL 状态不再满足本组曝光条件：%1")
            .arg(laserError), false);
        return;
    }
    pendingLaserStatus_ = laserStatus_;
    const qint64 frameCallbackMonotonicMs =
        hik_sync::getMonotonicRawNs() / 1000000LL;
    pendingLaserStatusAgeMs_ =
        laserStatusReceivedMonotonicMs_ > 0
            ? frameCallbackMonotonicMs -
                  laserStatusReceivedMonotonicMs_
            : -1;
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
    if (profileOptions_.reconstruction.stripe.mode !=
        hik_calibration::StripeExtractionMode::Legacy) {
        const hik_stripe::Diagnostics& diagnostics =
            profile.qualityDiagnostics;
        const std::size_t uniqueRejected =
            diagnostics.totalCandidateCount >=
                    diagnostics.acceptedCandidateCount
                ? diagnostics.totalCandidateCount -
                      diagnostics.acceptedCandidateCount
                : 0U;
        const QString centerOffset =
            profile.shadowComparison.matchedPointCount > 0U
                ? QStringLiteral(
                      "匹配/robust/gross=%1/%2/%3，"
                      "signed mean/median/robust mean=%4/%5/%6 px，"
                      "robust gate=%7 px，"
                      "|Δ| median/P95/max=%8/%9/%10 px")
                      .arg(static_cast<qulonglong>(
                          profile.shadowComparison.matchedPointCount))
                      .arg(static_cast<qulonglong>(
                          profile.shadowComparison
                              .robustMatchedPointCount))
                      .arg(static_cast<qulonglong>(
                          profile.shadowComparison
                              .grossMismatchPointCount))
                      .arg(
                          profile.shadowComparison.signedMeanOffsetPx,
                          0, 'f', 4)
                      .arg(
                          profile.shadowComparison.signedMedianOffsetPx,
                          0, 'f', 4)
                      .arg(
                          profile.shadowComparison.robustSignedMeanOffsetPx,
                          0, 'f', 4)
                      .arg(
                          profile.shadowComparison.robustGatePx,
                          0, 'f', 4)
                      .arg(
                          profile.shadowComparison.absoluteMedianOffsetPx,
                          0, 'f', 4)
                      .arg(
                          profile.shadowComparison.absoluteP95OffsetPx,
                          0, 'f', 4)
                      .arg(
                          profile.shadowComparison.absoluteMaximumOffsetPx,
                          0, 'f', 4)
                : QStringLiteral("无可匹配 legacy/quality 中心");
        appendLog(QStringLiteral(
            "质量中心线：passed=%1，legacy/quality 3D点=%2/%3，"
            "候选 accepted/total/rejected=%4/%5/%6；"
            "拒绝 low/width/saturation/multipeak/asymmetry/fit/quality/mask="
            "%7/%8/%9/%10/%11/%12/%13/%14；新旧中心：%15；算法=%16%17")
            .arg(profile.qualityExtractionPassed
                     ? QStringLiteral("yes")
                     : QStringLiteral("no"))
            .arg(static_cast<qulonglong>(
                profile.legacyPoints.size()))
            .arg(static_cast<qulonglong>(
                profile.qualityPoints.size()))
            .arg(static_cast<qulonglong>(
                diagnostics.acceptedCandidateCount))
            .arg(static_cast<qulonglong>(
                diagnostics.totalCandidateCount))
            .arg(static_cast<qulonglong>(uniqueRejected))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedLowProminenceCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedWidthCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedSaturationCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedMultiPeakCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedAsymmetryCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedFitCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedQualityCount))
            .arg(static_cast<qulonglong>(
                diagnostics.rejectedMaskCount))
            .arg(centerOffset)
            .arg(QString::fromStdString(
                profile.centerlineAlgorithmVersion))
            .arg(profile.qualityExtractionError.empty()
                     ? QString()
                     : QStringLiteral("；错误=") +
                           QString::fromStdString(
                               profile.qualityExtractionError)));
        appendLog(QStringLiteral(
            "质量指标：selected/gap=%1/%2，mean SNR/FWHM=%3/%4 px，"
            "selected saturation=%5%，multipeak scanlines=%6，"
            "mean asymmetry/fit/second-peak=%7/%8/%9，"
            "path margin/point=%10。")
            .arg(static_cast<qulonglong>(
                diagnostics.selectedPointCount))
            .arg(static_cast<qulonglong>(
                diagnostics.selectedGapCount))
            .arg(diagnostics.meanSelectedSnr, 0, 'f', 3)
            .arg(diagnostics.meanSelectedFwhmPx, 0, 'f', 3)
            .arg(
                100.0 * diagnostics.selectedSaturatedRatio,
                0, 'f', 2)
            .arg(static_cast<qulonglong>(
                diagnostics.multiPeakScanlineCount))
            .arg(
                diagnostics.meanSelectedGradientAsymmetry,
                0, 'f', 4)
            .arg(
                diagnostics.meanSelectedFitResidual,
                0, 'f', 4)
            .arg(
                diagnostics.meanSelectedSecondPeakRatio,
                0, 'f', 4)
            .arg(
                diagnostics.pathCostMarginPerPoint,
                0, 'f', 6));
    }
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
    const std::size_t oldQualityCloudSize = qualityCloud_.size();
    std::string coreError;
    const int profileIndex = static_cast<int>(profileRows_.size());
    if (!hik_scan::appendProfileInBase(pendingProfile_, baseFromFlange,
                                       handEye_.flangeFromCamera, profileIndex,
                                       &cloud_, &coreError)) {
        abortScan(QString::fromStdString(coreError), false);
        return;
    }
    QString qualityAppendError;
    if (pendingProfile_.qualityExtractionPassed &&
        !pendingProfile_.qualityPoints.empty()) {
        const cv::Matx44d baseFromCamera =
            baseFromFlange * handEye_.flangeFromCamera;
        if (!hik_scan::appendProfilePointsUsingBaseFromCamera(
                pendingProfile_.qualityPoints, baseFromCamera,
                profileIndex, &qualityCloud_, &coreError)) {
            qualityAppendError = QString::fromStdString(coreError);
            appendLog(QStringLiteral(
                "警告：轮廓 %1 的质量点并行累计失败；"
                "现有正式/legacy 点云不受影响：%2")
                .arg(profileIndex)
                .arg(qualityAppendError));
        }
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
    row.qualityExtractionPassed =
        pendingProfile_.qualityExtractionPassed &&
        qualityAppendError.isEmpty();
    row.legacyPointCount =
        static_cast<int>(pendingProfile_.legacyPoints.size());
    row.qualityPointCount =
        static_cast<int>(pendingProfile_.qualityPoints.size());
    const hik_stripe::Diagnostics& qualityDiagnostics =
        pendingProfile_.qualityDiagnostics;
    row.qualityCandidateCount =
        static_cast<int>(qualityDiagnostics.totalCandidateCount);
    row.qualityAcceptedCandidateCount =
        static_cast<int>(qualityDiagnostics.acceptedCandidateCount);
    row.qualityRejectedCandidateCount =
        std::max(0, row.qualityCandidateCount -
                       row.qualityAcceptedCandidateCount);
    row.qualitySelectedPointCount =
        static_cast<int>(qualityDiagnostics.selectedPointCount);
    row.qualitySelectedGapCount =
        static_cast<int>(qualityDiagnostics.selectedGapCount);
    row.qualityMultiPeakScanlineCount =
        static_cast<int>(qualityDiagnostics.multiPeakScanlineCount);
    row.qualityAmbiguousPathPointCount =
        static_cast<int>(qualityDiagnostics.ambiguousPathPointCount);
    row.qualityRejectedLowProminenceCount =
        static_cast<int>(
            qualityDiagnostics.rejectedLowProminenceCount);
    row.qualityRejectedWidthCount =
        static_cast<int>(qualityDiagnostics.rejectedWidthCount);
    row.qualityRejectedSaturationCount =
        static_cast<int>(
            qualityDiagnostics.rejectedSaturationCount);
    row.qualityRejectedMultiPeakCount =
        static_cast<int>(
            qualityDiagnostics.rejectedMultiPeakCount);
    row.qualityRejectedAsymmetryCount =
        static_cast<int>(
            qualityDiagnostics.rejectedAsymmetryCount);
    row.qualityRejectedFitCount =
        static_cast<int>(qualityDiagnostics.rejectedFitCount);
    row.qualityRejectedQualityCount =
        static_cast<int>(
            qualityDiagnostics.rejectedQualityCount);
    row.qualityRejectedMaskCount =
        static_cast<int>(qualityDiagnostics.rejectedMaskCount);
    row.qualityMeanSelectedQuality =
        qualityDiagnostics.meanSelectedQuality;
    row.qualityMeanSelectedFwhmPx =
        qualityDiagnostics.meanSelectedFwhmPx;
    row.qualityMeanSelectedSnr =
        qualityDiagnostics.meanSelectedSnr;
    row.qualitySelectedSaturatedRatio =
        qualityDiagnostics.selectedSaturatedRatio;
    row.qualityMeanSelectedGradientAsymmetry =
        qualityDiagnostics.meanSelectedGradientAsymmetry;
    row.qualityMeanSelectedFitResidual =
        qualityDiagnostics.meanSelectedFitResidual;
    row.qualityMeanSelectedSecondPeakRatio =
        qualityDiagnostics.meanSelectedSecondPeakRatio;
    row.qualityPathCostMarginPerPoint =
        qualityDiagnostics.pathCostMarginPerPoint;
    row.centerMatchedPointCount =
        static_cast<int>(
            pendingProfile_.shadowComparison.matchedPointCount);
    row.centerRobustMatchedPointCount =
        static_cast<int>(
            pendingProfile_.shadowComparison.robustMatchedPointCount);
    row.centerGrossMismatchPointCount =
        static_cast<int>(
            pendingProfile_.shadowComparison.grossMismatchPointCount);
    row.centerSignedMeanOffsetPx =
        pendingProfile_.shadowComparison.signedMeanOffsetPx;
    row.centerSignedMedianOffsetPx =
        pendingProfile_.shadowComparison.signedMedianOffsetPx;
    row.centerRobustSignedMeanOffsetPx =
        pendingProfile_.shadowComparison.robustSignedMeanOffsetPx;
    row.centerRobustGatePx =
        pendingProfile_.shadowComparison.robustGatePx;
    row.centerAbsoluteMedianOffsetPx =
        pendingProfile_.shadowComparison.absoluteMedianOffsetPx;
    row.centerAbsoluteP95OffsetPx =
        pendingProfile_.shadowComparison.absoluteP95OffsetPx;
    row.centerAbsoluteMaximumOffsetPx =
        pendingProfile_.shadowComparison.absoluteMaximumOffsetPx;
    row.centerlineAlgorithmVersion =
        QString::fromStdString(
            pendingProfile_.centerlineAlgorithmVersion);
    row.qualityExtractionError = qualityAppendError.isEmpty()
        ? QString::fromStdString(
              pendingProfile_.qualityExtractionError)
        : qualityAppendError;
    QString error;
    if (!appendManifest(row, pendingFrameNo_, pendingDeviceTimestamp_,
                        pendingCameraHostTimestamp_, pendingExposureUs_,
                        pendingGainDb_, &error)) {
        cloud_.resize(oldCloudSize);
        qualityCloud_.resize(oldQualityCloudSize);
        abortScan(error, false);
        return;
    }
    profileRows_.push_back(row);
    refreshTable();
    scanStatusLabel_->setText(QStringLiteral(
        "轮廓 %1 已累计，正式/legacy 点=%2，质量并行点=%3。")
        .arg(profileIndex)
        .arg(static_cast<qulonglong>(cloud_.size()))
        .arg(static_cast<qulonglong>(qualityCloud_.size())));
    continueOrFinish();
}

void HikConstantLaserScanWindow::continueOrFinish() {
    if (singlePointMode_ || currentTargetIndex_ + 1 >= static_cast<int>(targets_.size())) {
        requestLaserOff();
        QString error;
        if (!saveCloudOutputs(&error)) {
            abortScan(error, false);
            return;
        }
        scanState_ = ScanState::Idle;
        safetyConfirmCheck_->setChecked(false);
        scanStatusLabel_->setText(QStringLiteral(
            "扫描完成：%1 条轮廓，正式点=%2；"
            "质量 kept/rejected/voxel=%3/%4/%5。"
            "raw=%6，voxel=%7")
            .arg(static_cast<int>(profileRows_.size()))
            .arg(static_cast<qulonglong>(cloud_.size()))
            .arg(static_cast<qulonglong>(
                qualitySupportResult_.kept.size()))
            .arg(static_cast<qulonglong>(
                qualitySupportResult_.rejected.size()))
            .arg(static_cast<qulonglong>(
                qualityVoxelPointCount_))
            .arg(rawPlyPath_, voxelPlyPath_));
        appendLog(scanStatusLabel_->text());
        QJsonObject terminalStats;
        terminalStats.insert(QStringLiteral("profile_count"),
                             static_cast<int>(profileRows_.size()));
        terminalStats.insert(QStringLiteral("raw_point_count"),
                             static_cast<double>(cloud_.size()));
        terminalStats.insert(
            QStringLiteral("quality_input_point_count"),
            static_cast<double>(
                qualitySupportResult_.statistics.inputPointCount));
        terminalStats.insert(
            QStringLiteral("quality_support_filter_applied"),
            qualitySupportResult_.applied);
        terminalStats.insert(
            QStringLiteral("quality_filtered_point_count"),
            static_cast<double>(
                qualitySupportResult_.statistics.keptPointCount));
        terminalStats.insert(
            QStringLiteral("quality_rejected_point_count"),
            static_cast<double>(
                qualitySupportResult_.statistics.rejectedPointCount));
        terminalStats.insert(
            QStringLiteral("quality_invalid_point_count"),
            static_cast<double>(
                qualitySupportResult_.statistics.invalidPointCount));
        terminalStats.insert(
            QStringLiteral("quality_insufficient_support_point_count"),
            static_cast<double>(
                qualitySupportResult_.statistics
                    .insufficientSupportPointCount));
        terminalStats.insert(
            QStringLiteral("quality_voxel_point_count"),
            static_cast<double>(qualityVoxelPointCount_));
        beginTerminalBarrier(
            true, scanStatusLabel_->text(),
            singlePointMode_ ? QStringLiteral("single_point")
                             : QStringLiteral("stop_and_shoot"),
            scanSessionDir_, terminalStats);
        return;
    }
    ++currentTargetIndex_;
    issueMoveForCurrentTarget();
}

void HikConstantLaserScanWindow::abortScan(const QString& reason, bool requestStop) {
    requestLaserOff();
    appendLog(QStringLiteral("扫描终止: %1").arg(reason));
    stopRequested_ = true;
    const bool motionWasPending = pendingMotionRequestId_ >= 0;
    if (motionWasPending) {
        if (requestStop && robotConnected_) {
            emit requestStopMotion(
                robotSession_->allocateRequestId(robotClientId_));
        } else {
            terminalMotionFault_ = true;
            terminalMotionFaultDetail_ = QStringLiteral(
                "FR5 运动仍在活动，但当前无法确认 StopMotion：%1")
                .arg(reason);
            pendingMotionRequestId_ = -1;
        }
    }
    QString saveError;
    if (!cloud_.empty()) saveCloudOutputs(&saveError);
    scanState_ = ScanState::Idle;
    pendingRobotRequestId_ = -1;
    readRole_ = ReadRole::None;
    safetyConfirmCheck_->setChecked(false);
    const QString finalReason = QStringLiteral("扫描已终止：%1%2")
        .arg(reason,
             saveError.isEmpty()
                 ? QString()
                 : QStringLiteral("；部分 PLY 保存失败：") + saveError);
    scanStatusLabel_->setText(finalReason);
    QJsonObject terminalStats;
    terminalStats.insert(QStringLiteral("profile_count"),
                         static_cast<int>(profileRows_.size()));
    terminalStats.insert(QStringLiteral("raw_point_count"),
                         static_cast<double>(cloud_.size()));
    terminalStats.insert(
        QStringLiteral("quality_input_point_count"),
        static_cast<double>(
            qualitySupportResult_.statistics.inputPointCount));
    terminalStats.insert(
        QStringLiteral("quality_support_filter_applied"),
        qualitySupportResult_.applied);
    terminalStats.insert(
        QStringLiteral("quality_filtered_point_count"),
        static_cast<double>(
            qualitySupportResult_.statistics.keptPointCount));
    terminalStats.insert(
        QStringLiteral("quality_rejected_point_count"),
        static_cast<double>(
            qualitySupportResult_.statistics.rejectedPointCount));
    terminalStats.insert(
        QStringLiteral("quality_invalid_point_count"),
        static_cast<double>(
            qualitySupportResult_.statistics.invalidPointCount));
    terminalStats.insert(
        QStringLiteral("quality_insufficient_support_point_count"),
        static_cast<double>(
            qualitySupportResult_.statistics
                .insufficientSupportPointCount));
    terminalStats.insert(
        QStringLiteral("quality_voxel_point_count"),
        static_cast<double>(qualityVoxelPointCount_));
    beginTerminalBarrier(
        false, finalReason,
        singlePointMode_ ? QStringLiteral("single_point")
                         : QStringLiteral("stop_and_shoot"),
        scanSessionDir_, terminalStats);
}

bool HikConstantLaserScanWindow::createScanSession(QString* error) {
    const QString root = profile_.scanSessionRoot(sourceDir_);
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
    qualityOpticalPlyPath_ = QDir(scanSessionDir_).absoluteFilePath(
        QStringLiteral("scan_quality_optical.ply"));
    qualityFilteredPlyPath_ = QDir(scanSessionDir_).absoluteFilePath(
        QStringLiteral("scan_quality_filtered.ply"));
    qualityRejectedPlyPath_ = QDir(scanSessionDir_).absoluteFilePath(
        QStringLiteral("scan_quality_rejected.ply"));
    qualityVoxelPlyPath_ = QDir(scanSessionDir_).absoluteFilePath(
        QStringLiteral("scan_quality_voxel.ply"));
    if (!writeSessionMetadata(scanSessionDir_, error)) return false;
    QSaveFile file(manifestPath_);
    const QByteArray header =
        "device_profile,wavelength_nm,ttl_physical_pin,laser_daemon_generation,"
        "laser_state,laser_status_age_ms,ttl450_high,ttl650_high,"
        "laser_lease_active,laser_fault,camera_model,camera_serial,camera_ip,"
        "profile_index,image_path,frame_no,device_timestamp,camera_host_timestamp_raw,"
        "exposure_us,gain_db,before_host_ms,after_host_ms,"
        "before_x_mm,before_y_mm,before_z_mm,before_rx_deg,before_ry_deg,before_rz_deg,"
        "after_x_mm,after_y_mm,after_z_mm,after_rx_deg,after_ry_deg,after_rz_deg,"
        "translation_delta_mm,rotation_delta_deg,point_count,z_min_mm,z_max_mm,line_rms_mm,"
        "line_rms_limit_mm,flat_target_gate,stripe_saturated_ratio,"
        "quality_extraction_passed,legacy_point_count,quality_point_count,"
        "quality_candidate_count,quality_accepted_candidate_count,"
        "quality_rejected_candidate_count,"
        "quality_selected_point_count,quality_selected_gap_count,"
        "quality_multipeak_scanline_count,"
        "quality_ambiguous_path_point_count,"
        "quality_reject_low_prominence_count,quality_reject_width_count,"
        "quality_reject_saturation_count,quality_reject_multipeak_count,"
        "quality_reject_asymmetry_count,quality_reject_fit_count,"
        "quality_reject_quality_count,quality_reject_mask_count,"
        "quality_mean_selected_quality,quality_mean_selected_fwhm_px,"
        "quality_mean_selected_snr,quality_selected_saturated_ratio,"
        "quality_mean_selected_gradient_asymmetry,"
        "quality_mean_selected_fit_residual,"
        "quality_mean_selected_second_peak_ratio,"
        "quality_path_cost_margin_per_point,"
        "center_matched_point_count,center_robust_matched_point_count,"
        "center_gross_mismatch_point_count,center_signed_mean_offset_px,"
        "center_signed_median_offset_px,"
        "center_robust_signed_mean_offset_px,center_robust_gate_px,"
        "center_absolute_median_offset_px,center_absolute_p95_offset_px,"
        "center_absolute_maximum_offset_px,centerline_algorithm_version,"
        "quality_extraction_error,"
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
    line += csvQuoted(profile_.id);
    line += ','; line += QByteArray::number(profile_.wavelengthNm);
    line += ','; line += QByteArray::number(profile_.ttlPhysicalPin);
    line += ','; line += csvQuoted(pendingLaserStatus_.daemonGeneration);
    line += ','; line += csvQuoted(
        laserStateProtocolName(pendingLaserStatus_.state));
    line += ','; line += QByteArray::number(pendingLaserStatusAgeMs_);
    line += ','; line += pendingLaserStatus_.ttl450High ? "1" : "0";
    line += ','; line += pendingLaserStatus_.ttl650High ? "1" : "0";
    line += ','; line += pendingLaserStatus_.leaseActive ? "1" : "0";
    line += ','; line += csvQuoted(pendingLaserStatus_.fault);
    line += ','; line += csvQuoted(cameraModel_);
    line += ','; line += csvQuoted(cameraSerial_);
    line += ','; line += csvQuoted(cameraIp_);
    line += ','; line += QByteArray::number(row.index);
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
    const double poseAndMotionValues[] = {
        before.x, before.y, before.z,
        before.rx, before.ry, before.rz,
        after.x, after.y, after.z,
        after.rx, after.ry, after.rz,
        row.translationDeltaMm, row.rotationDeltaDeg};
    for (double value : poseAndMotionValues) {
        line += ','; line += QByteArray::number(value, 'f', 9);
    }
    line += ','; line += QByteArray::number(row.cameraPointCount);
    const double reconstructionValues[] = {
        row.minimumDepthMm, row.maximumDepthMm, row.lineRmsMm};
    for (double value : reconstructionValues) {
        line += ','; line += QByteArray::number(value, 'f', 9);
    }
    line += ','; line += QByteArray::number(row.lineRmsLimitMm, 'f', 9);
    line += ','; line += (row.flatTargetGate ? "1" : "0");
    line += ','; line += QByteArray::number(row.stripeSaturatedRatio, 'f', 9);
    line += ',';
    line += (row.qualityExtractionPassed ? "1" : "0");
    const int qualityCounts[] = {
        row.legacyPointCount,
        row.qualityPointCount,
        row.qualityCandidateCount,
        row.qualityAcceptedCandidateCount,
        row.qualityRejectedCandidateCount,
        row.qualitySelectedPointCount,
        row.qualitySelectedGapCount,
        row.qualityMultiPeakScanlineCount,
        row.qualityAmbiguousPathPointCount,
        row.qualityRejectedLowProminenceCount,
        row.qualityRejectedWidthCount,
        row.qualityRejectedSaturationCount,
        row.qualityRejectedMultiPeakCount,
        row.qualityRejectedAsymmetryCount,
        row.qualityRejectedFitCount,
        row.qualityRejectedQualityCount,
        row.qualityRejectedMaskCount};
    for (int value : qualityCounts) {
        line += ','; line += QByteArray::number(value);
    }
    const double qualityMetrics[] = {
        row.qualityMeanSelectedQuality,
        row.qualityMeanSelectedFwhmPx,
        row.qualityMeanSelectedSnr,
        row.qualitySelectedSaturatedRatio,
        row.qualityMeanSelectedGradientAsymmetry,
        row.qualityMeanSelectedFitResidual,
        row.qualityMeanSelectedSecondPeakRatio,
        row.qualityPathCostMarginPerPoint};
    for (double value : qualityMetrics) {
        line += ','; line += QByteArray::number(value, 'f', 12);
    }
    line += ','; line += QByteArray::number(row.centerMatchedPointCount);
    line += ','; line += QByteArray::number(
        row.centerRobustMatchedPointCount);
    line += ','; line += QByteArray::number(
        row.centerGrossMismatchPointCount);
    const double centerOffsets[] = {
        row.centerSignedMeanOffsetPx,
        row.centerSignedMedianOffsetPx,
        row.centerRobustSignedMeanOffsetPx,
        row.centerRobustGatePx,
        row.centerAbsoluteMedianOffsetPx,
        row.centerAbsoluteP95OffsetPx,
        row.centerAbsoluteMaximumOffsetPx};
    for (double value : centerOffsets) {
        line += ','; line += QByteArray::number(value, 'f', 9);
    }
    line += ','; line += csvQuoted(row.centerlineAlgorithmVersion);
    line += ','; line += csvQuoted(row.qualityExtractionError);
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

bool HikConstantLaserScanWindow::writeSessionMetadata(
        const QString& directory, QString* error) const {
    if (error) error->clear();
    if (!QDir().mkpath(directory)) {
        if (error) {
            *error = QStringLiteral("无法创建会话目录：%1").arg(directory);
        }
        return false;
    }
    QJsonObject profileObject;
    profileObject.insert(QStringLiteral("id"), profile_.id);
    profileObject.insert(QStringLiteral("display_name"), profile_.displayName);
    profileObject.insert(QStringLiteral("wavelength_nm"),
                         profile_.wavelengthNm);
    profileObject.insert(QStringLiteral("ttl_physical_pin"),
                         profile_.ttlPhysicalPin);
    profileObject.insert(QStringLiteral("camera_frame"),
                         profile_.cameraFrame);
    profileObject.insert(QStringLiteral("camera_ip"), cameraIp_);
    profileObject.insert(QStringLiteral("camera_model"), cameraModel_);
    profileObject.insert(QStringLiteral("camera_serial"), cameraSerial_);
    profileObject.insert(
        QStringLiteral("stripe_orientation"),
        lineLaserStripeOrientationName(profile_.stripeOrientation));
    profileObject.insert(
        QStringLiteral("centerline_policy"),
        lineLaserCenterlinePolicyName(profile_.scanCenterlinePolicy));
    QJsonObject stripeRoi;
    stripeRoi.insert(QStringLiteral("x"), profile_.stripeRoiX);
    stripeRoi.insert(QStringLiteral("y"), profile_.stripeRoiY);
    stripeRoi.insert(QStringLiteral("width"), profile_.stripeRoiWidth);
    stripeRoi.insert(QStringLiteral("height"), profile_.stripeRoiHeight);
    profileObject.insert(QStringLiteral("stripe_roi_normalized"), stripeRoi);

    QJsonObject laserObject;
    laserObject.insert(QStringLiteral("state"),
                       laserStateProtocolName(laserStatus_.state));
    laserObject.insert(QStringLiteral("ttl450_high"),
                       laserStatus_.ttl450High);
    laserObject.insert(QStringLiteral("ttl650_high"),
                       laserStatus_.ttl650High);
    laserObject.insert(QStringLiteral("lease_active"),
                       laserStatus_.leaseActive);
    laserObject.insert(QStringLiteral("daemon_generation"),
                       laserStatus_.daemonGeneration);
    laserObject.insert(QStringLiteral("board_model"),
                       laserStatus_.boardModel);
    laserObject.insert(QStringLiteral("meaning"),
        QStringLiteral("TTL GPIO readback only; not an optical-power measurement"));

    QJsonObject calibrationObject;
    calibrationObject.insert(QStringLiteral("intrinsics_path"),
                             intrinsicsPath_);
    calibrationObject.insert(QStringLiteral("laser_plane_path"),
                             laserPlanePath_);
    calibrationObject.insert(QStringLiteral("handeye_path"),
                             handEyePath_);
    calibrationObject.insert(QStringLiteral("intrinsics_sha256"),
                             intrinsicsSha256_);
    calibrationObject.insert(QStringLiteral("laser_plane_sha256"),
                             laserPlaneSha256_);
    calibrationObject.insert(QStringLiteral("handeye_sha256"),
                             handEyeSha256_);

    QJsonObject root;
    root.insert(QStringLiteral("schema_version"), 1);
    root.insert(QStringLiteral("created_utc"),
                QDateTime::currentDateTimeUtc().toString(
                    Qt::ISODateWithMs));
    root.insert(QStringLiteral("device_profile"), profileObject);
    root.insert(QStringLiteral("laser_control"), laserObject);
    root.insert(QStringLiteral("calibration"), calibrationObject);

    const QString path = QDir(directory).absoluteFilePath(
        QStringLiteral("session_metadata.json"));
    const QByteArray payload =
        QJsonDocument(root).toJson(QJsonDocument::Indented);
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(payload) != payload.size() ||
        !file.commit()) {
        if (error) {
            *error = QStringLiteral("无法写入会话元数据：%1")
                         .arg(path);
        }
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::saveCloudOutputs(QString* error) {
    if (error) error->clear();
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

    hik_scan::AdjacentProfileSupportOptions supportOptions;
    // A single-point validation has no neighboring profile by definition.
    // Preserve all quality points in that mode, but still emit the same three
    // quality files so downstream tooling has a stable contract.
    supportOptions.enabled =
        profileRows_.size() > 1U && !qualityCloud_.empty();
    supportOptions.minimumSupportingProfiles =
        kQualitySupportMinimumProfiles;
    supportOptions.maximumProfileGap =
        kQualitySupportMaximumProfileGap;
    supportOptions.radiusMm = std::max(
        kQualitySupportRadiusFloorMm,
        std::max(
            kQualitySupportStepFactor * stepSpin_->value(),
            kQualitySupportVoxelFactor * voxelSpin_->value()));

    qualitySupportResult_ =
        hik_scan::AdjacentProfileSupportResult();
    if (!hik_scan::filterByAdjacentProfileSupport(
            qualityCloud_, supportOptions, &qualitySupportResult_,
            &coreError)) {
        if (error) {
            *error = QStringLiteral("质量点相邻 profile 支持过滤失败：%1")
                .arg(QString::fromStdString(coreError));
        }
        return false;
    }

    hik_scan::VoxelDownsampleOptions qualityVoxelOptions;
    qualityVoxelOptions.voxelSizeMm = voxelSpin_->value();
    qualityVoxelOptions.confidenceWeighted = true;
    hik_scan::VoxelDownsampleStatistics qualityVoxelStatistics;
    const std::vector<hik_scan::CloudPoint> qualityVoxel =
        hik_scan::voxelDownsample(
            qualitySupportResult_.kept, qualityVoxelOptions,
            &qualityVoxelStatistics);
    qualityVoxelPointCount_ = qualityVoxel.size();

    QString qualitySaveError;
    if (!saveCloudPlyAllowEmpty(
            qualityOpticalPlyPath_, qualityCloud_,
            QStringLiteral("base_link"), &qualitySaveError) ||
        !saveCloudPlyAllowEmpty(
            qualityFilteredPlyPath_, qualitySupportResult_.kept,
            QStringLiteral("base_link"), &qualitySaveError) ||
        !saveCloudPlyAllowEmpty(
            qualityRejectedPlyPath_, qualitySupportResult_.rejected,
            QStringLiteral("base_link"), &qualitySaveError) ||
        !saveCloudPlyAllowEmpty(
            qualityVoxelPlyPath_, qualityVoxel,
            QStringLiteral("base_link"), &qualitySaveError)) {
        if (error) {
            *error = QStringLiteral("质量点云保存失败：%1")
                .arg(qualitySaveError);
        }
        return false;
    }

    const hik_scan::AdjacentProfileSupportStatistics& qualityStatistics =
        qualitySupportResult_.statistics;
    appendLog(QStringLiteral(
        "质量点云：input=%1，support_filter=%2（radius=%3 mm，"
        "min_profiles=%4，max_gap=%5），kept/rejected=%6/%7，"
        "invalid/insufficient=%8/%9，confidence-weighted voxel=%10；"
        "optical=%11，filtered=%12，rejected=%13，voxel=%14。")
        .arg(static_cast<qulonglong>(
            qualityStatistics.inputPointCount))
        .arg(qualitySupportResult_.applied
                 ? QStringLiteral("on")
                 : QStringLiteral("off"))
        .arg(supportOptions.radiusMm, 0, 'f', 3)
        .arg(supportOptions.minimumSupportingProfiles)
        .arg(supportOptions.maximumProfileGap)
        .arg(static_cast<qulonglong>(
            qualityStatistics.keptPointCount))
        .arg(static_cast<qulonglong>(
            qualityStatistics.rejectedPointCount))
        .arg(static_cast<qulonglong>(
            qualityStatistics.invalidPointCount))
        .arg(static_cast<qulonglong>(
            qualityStatistics.insufficientSupportPointCount))
        .arg(static_cast<qulonglong>(
            qualityVoxelStatistics.outputPointCount))
        .arg(qualityOpticalPlyPath_, qualityFilteredPlyPath_,
             qualityRejectedPlyPath_, qualityVoxelPlyPath_));
    if (profile_.scanCenterlinePolicy ==
        LineLaserCenterlinePolicy::Shadow) {
        appendLog(QStringLiteral(
            "650/shadow 保护：上述质量文件仅供比较；正式输出仍为 %1 和 %2。")
            .arg(rawPlyPath_, voxelPlyPath_));
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
    const std::vector<hik_calibration::StripePoint>& legacy =
        profile.legacyStripe.empty() &&
                profile.qualityStripe.empty()
            ? profile.stripe
            : profile.legacyStripe;
    for (const hik_calibration::StripePoint& point : legacy) {
        cv::circle(overlay, cv::Point(
                       static_cast<int>(std::lround(point.pixel.x)),
                       static_cast<int>(std::lround(point.pixel.y))),
                   1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
    for (const hik_calibration::StripePoint& point :
         profile.qualityStripe) {
        cv::circle(overlay, cv::Point(
                       static_cast<int>(std::lround(point.pixel.x)),
                       static_cast<int>(std::lround(point.pixel.y))),
                   1, cv::Scalar(0, 220, 255), -1, cv::LINE_AA);
    }
    if (!profile.qualityStripe.empty() &&
        profile.qualityDiagnostics.appliedRoi.width > 0 &&
        profile.qualityDiagnostics.appliedRoi.height > 0) {
        cv::rectangle(
            overlay, profile.qualityDiagnostics.appliedRoi,
            cv::Scalar(255, 120, 0), 1, cv::LINE_AA);
        cv::putText(
            overlay, "legacy=green quality=yellow ROI=blue",
            cv::Point(20, 66), cv::FONT_HERSHEY_SIMPLEX,
            0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    std::ostringstream caption;
    caption << "official=" << profile.points.size()
            << " legacy=" << profile.legacyPoints.size()
            << " quality=" << profile.qualityPoints.size()
            << ", Z="
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
