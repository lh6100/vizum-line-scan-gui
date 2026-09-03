#include "HikConstantLaserScanWindow.h"

#include "FairinoRobotSession.h"
#include "HandEyeCalibrationCore.h"
#include "HikCameraWorker.h"
#include "ImageView.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDebug>
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
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>

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

std::uint64_t stopAndShootAmbiguityGroupId(
        int profileIndex, int intervalId) {
    return
        (static_cast<std::uint64_t>(
             static_cast<std::uint32_t>(profileIndex)) << 32U) |
        static_cast<std::uint64_t>(
            static_cast<std::uint32_t>(intervalId));
}

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
        case LineLaserState::Both:
            return QStringLiteral("450 nm + 650 nm TTL HIGH");
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
        case LineLaserState::Both:
            return QStringLiteral("both");
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
        QWidget* parent, double scanSpeedOverrideMmS,
        bool useDeviceCalibrationAsIs)
    : QMainWindow(parent),
      profile_(profile),
      laserController_(laserController),
      robotSession_(robotSession),
      useDeviceCalibrationAsIs_(useDeviceCalibrationAsIs),
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
    QString adaptiveConfigError;
    if (!profile_.adaptiveScanConfigRelativePath.trimmed().isEmpty()) {
        adaptiveConfigPath_ = profile_.adaptiveScanConfigPath(sourceDir_);
        adaptiveConfigReady_ =
            loadAdaptiveScanConfig(&adaptiveConfigError);
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
    applySelectedScanCenterlinePolicy(false);
    setupWorkers();
    appendLog(QStringLiteral(
        "条纹策略：方向=%1，模式=%2，归一化ROI=[%3,%4,%5,%6]；"
        "ROI还会与正式激光平面的有效深度走廊取交集。")
        .arg(lineLaserStripeOrientationName(profile_.stripeOrientation),
             lineLaserCenterlinePolicyName(
                 selectedScanCenterlinePolicy()))
        .arg(profile_.stripeRoiX, 0, 'f', 4)
        .arg(profile_.stripeRoiY, 0, 'f', 4)
        .arg(profile_.stripeRoiWidth, 0, 'f', 4)
        .arg(profile_.stripeRoiHeight, 0, 'f', 4));
    if (selectedScanCenterlinePolicy() ==
        LineLaserCenterlinePolicy::Shadow) {
        appendLog(QStringLiteral(
            "Shadow 模式仍严格使用 Legacy 中心，不会被 Quality 中心替换；"
            "但 Quality 判定的多路径歧义区会从正式 raw/voxel 中硬遮罩。"));
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
    if (profile_.id == QStringLiteral("scanner_650")) {
        appendLog(adaptiveConfigReady_
            ? QStringLiteral("自适应扫描配置已加载：%1")
                  .arg(adaptiveConfigPath_)
            : QStringLiteral("自适应扫描配置不可用：%1；"
                             "自适应按钮已禁用。")
                  .arg(adaptiveConfigError));
    }
    QString error;
    if (loadFormalCalibration(&error)) {
        appendLog(calibrationProvenanceOverrideActive_
            ? QStringLiteral(
                "设备目录三份参数已加载；按启动选项原样采用手眼数值，"
                "手眼内参来源差异未被改写。")
            : QStringLiteral("正式内参、激光平面和手眼标定已校验。"));
        const QString readyMessage = calibrationProvenanceOverrideActive_
            ? QStringLiteral(
                "[%1] 已按用户要求原样使用设备目录参数；手眼声明的内参哈希=%2，"
                "当前内参哈希=%3。该差异将写入每个建图 session。")
                  .arg(profile_.id, handEyeDeclaredIntrinsicsSha256_,
                       intrinsicsSha256_)
            : QStringLiteral("[%1] 正式标定已就绪。").arg(profile_.id);
        if (calibrationProvenanceOverrideActive_) {
            appendLog(readyMessage);
            qWarning().noquote() << readyMessage;
        } else {
            qInfo().noquote() << readyMessage;
        }
        qInfo().noquote() << QStringLiteral(
            "  内参：%1\n  激光平面：%2\n  手眼：%3")
            .arg(intrinsicsPath_, laserPlanePath_, handEyePath_);
    } else {
        const QString message = QStringLiteral("正式标定未就绪：%1").arg(error);
        calibrationStatusLabel_->setText(message);
        calibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        appendLog(message);
        qWarning().noquote() << QStringLiteral(
            "[%1] %2\n  内参：%3\n  激光平面：%4\n  手眼：%5")
            .arg(profile_.id, message, intrinsicsPath_, laserPlanePath_,
                 handEyePath_);
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
    QWidget* central = new QWidget;
    central->setObjectName(QStringLiteral("scanScrollContent"));
    central->setMinimumWidth(1220);
    QVBoxLayout* root = new QVBoxLayout(central);
    root->setSizeConstraint(QLayout::SetMinimumSize);

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
    cameraStatusLabel_->setWordWrap(true);

    robotIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.200"), devices);
    connectRobotButton_ = new QPushButton(
        QStringLiteral("连接共享 FR5"), devices);
    disconnectRobotButton_ = new QPushButton(
        QStringLiteral("断开共享 FR5"), devices);
    readPoseButton_ = new QPushButton(QStringLiteral("读取法兰"), devices);
    robotStatusLabel_ = new QLabel(QStringLiteral("FR5 未连接"), devices);
    currentPoseLabel_ = new QLabel(QStringLiteral("当前法兰: -"), devices);
    currentPoseLabel_->setWordWrap(true);
    currentPoseLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);

    connectLaserButton_ = new QPushButton(
        QStringLiteral("连接鲁班猫 TTL"), devices);
    enableProfileLaserButton_ = new QPushButton(
        QStringLiteral("开启本组 %1 nm TTL").arg(profile_.wavelengthNm),
        devices);
    enableBothLasersButton_ = new QPushButton(
        QStringLiteral("同时开启 450 + 650 nm"), devices);
    disableAllLasersButton_ = new QPushButton(
        QStringLiteral("关闭两路 TTL"), devices);
    enableProfileLaserButton_->setStyleSheet(QStringLiteral(
        "background:#f9a825;color:#111;font-weight:bold;"));
    enableBothLasersButton_->setStyleSheet(QStringLiteral(
        "background:#ef6c00;color:white;font-weight:bold;"));
    disableAllLasersButton_->setStyleSheet(QStringLiteral(
        "background:#2e7d32;color:white;font-weight:bold;"));
    enableProfileLaserButton_->setToolTip(QStringLiteral(
        "请求鲁班猫将本组 TTL 持续置 HIGH；只有板端 ACK 和 GPIO 回读一致后状态才会变绿。"));
    enableBothLasersButton_->setToolTip(QStringLiteral(
        "手动将 Pin 11/GPIO15 与 Pin 7/GPIO16 同时置 HIGH；"
        "双开状态只用于人工观察，扫描仍要求当前 profile 单路开启。"));
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
    deviceLayout->addWidget(enableBothLasersButton_, 2, 4, 1, 2);
    deviceLayout->addWidget(disableAllLasersButton_, 2, 6, 1, 2);
    deviceLayout->addWidget(laserStatusLabel_, 2, 8, 1, 3);
    deviceLayout->setColumnStretch(1, 1);
    root->addWidget(devices);

    QGroupBox* calibration = new QGroupBox(
        QStringLiteral("正式标定与重建范围｜%1").arg(profile_.displayName), central);
    QHBoxLayout* calibrationLayout = new QHBoxLayout(calibration);
    reloadCalibrationButton_ = new QPushButton(QStringLiteral("重新加载 config"), calibration);
    reconstructionDepthMinimumSpin_ = new QDoubleSpinBox(calibration);
    reconstructionDepthMinimumSpin_->setObjectName(
        QStringLiteral("reconstructionDepthMinimumSpin"));
    reconstructionDepthMinimumSpin_->setRange(1.0, 9999.0);
    reconstructionDepthMinimumSpin_->setDecimals(1);
    reconstructionDepthMinimumSpin_->setSingleStep(10.0);
    reconstructionDepthMinimumSpin_->setValue(100.0);
    reconstructionDepthMinimumSpin_->setSuffix(QStringLiteral(" mm"));
    reconstructionDepthMinimumSpin_->setToolTip(QStringLiteral(
        "相机光学坐标系 +Z 的重建下限；小于该值的点会被丢弃。"));
    reconstructionDepthMaximumSpin_ = new QDoubleSpinBox(calibration);
    reconstructionDepthMaximumSpin_->setObjectName(
        QStringLiteral("reconstructionDepthMaximumSpin"));
    reconstructionDepthMaximumSpin_->setRange(2.0, 10000.0);
    reconstructionDepthMaximumSpin_->setDecimals(1);
    reconstructionDepthMaximumSpin_->setSingleStep(10.0);
    reconstructionDepthMaximumSpin_->setValue(1000.0);
    reconstructionDepthMaximumSpin_->setSuffix(QStringLiteral(" mm"));
    reconstructionDepthMaximumSpin_->setToolTip(QStringLiteral(
        "相机光学坐标系 +Z 的重建上限；大于该值的点会被丢弃。"
        "超出标定验证范围的点可以输出，但精度没有标定保证。"));
    calibrationStatusLabel_ = new QLabel(QStringLiteral("尚未加载"), calibration);
    calibrationStatusLabel_->setWordWrap(true);
    calibrationLayout->addWidget(reloadCalibrationButton_);
    calibrationLayout->addWidget(new QLabel(QStringLiteral("相机 Z 最小"), calibration));
    calibrationLayout->addWidget(reconstructionDepthMinimumSpin_);
    calibrationLayout->addWidget(new QLabel(QStringLiteral("相机 Z 最大"), calibration));
    calibrationLayout->addWidget(reconstructionDepthMaximumSpin_);
    calibrationLayout->addWidget(calibrationStatusLabel_, 1);
    root->addWidget(calibration);

    QGroupBox* path = new QGroupBox(QStringLiteral("示教直线扫描路径（姿态锁定为起点 RPY）"), central);
    QGridLayout* pathLayout = new QGridLayout(path);
    teachStartButton_ = new QPushButton(QStringLiteral("读取并设为起点"), path);
    teachEndButton_ = new QPushButton(QStringLiteral("读取并设为终点"), path);
    editStartButton_ = new QPushButton(QStringLiteral("编辑起点数值"), path);
    editEndButton_ = new QPushButton(QStringLiteral("编辑终点数值"), path);
    swapPathButton_ = new QPushButton(
        QStringLiteral("⇄ 交换起点/终点（反向扫描）"), path);
    swapPathButton_->setToolTip(QStringLiteral(
        "交换两端 XYZ，但保持当前起点 RPY 不变。适合一次扫描结束后从旧终点"
        "直接反向重复扫描；点击本按钮不会立即移动机械臂。"));
    startPoseLabel_ = new QLabel(QStringLiteral("起点: -"), path);
    endPoseLabel_ = new QLabel(QStringLiteral("终点: -"), path);
    startPoseLabel_->setWordWrap(true);
    endPoseLabel_->setWordWrap(true);
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
    binaryPlyCheck_ = new QCheckBox(
        QStringLiteral("连续点云使用二进制 PLY（推荐）"), path);
    binaryPlyCheck_->setChecked(true);
    binaryPlyCheck_->setToolTip(QStringLiteral(
        "只改变 continuous_raw/continuous_voxel 的编码，不改变点坐标、"
        "属性或体素精度；CloudCompare 可直接打开。"));
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
    pathLayout->addWidget(swapPathButton_, 2, 0, 1, 2);
    QLabel* reversePathHint = new QLabel(QStringLiteral(
        "只交换 XYZ，保持当前起点 RPY；交换本身不会运动"), path);
    reversePathHint->setWordWrap(true);
    pathLayout->addWidget(reversePathHint, 2, 2, 1, 4);
    pathLayout->addWidget(new QLabel(QStringLiteral("步距"), path), 3, 0);
    pathLayout->addWidget(stepSpin_, 3, 1);
    pathLayout->addWidget(new QLabel(QStringLiteral("速度"), path), 3, 2);
    pathLayout->addWidget(velocitySpin_, 3, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("加速度"), path), 3, 4);
    pathLayout->addWidget(accelerationSpin_, 3, 5);
    pathLayout->addWidget(new QLabel(QStringLiteral("停稳等待"), path), 4, 0);
    pathLayout->addWidget(settleSpin_, 4, 1);
    pathLayout->addWidget(new QLabel(QStringLiteral("运动超时"), path), 4, 2);
    pathLayout->addWidget(motionTimeoutSpin_, 4, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("体素"), path), 4, 4);
    pathLayout->addWidget(voxelSpin_, 4, 5);
    pathLayout->addWidget(flatTargetGateCheck_, 5, 0, 1, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("直线 RMS 门槛"), path), 5, 3);
    pathLayout->addWidget(lineRmsLimitSpin_, 5, 4, 1, 2);
    pathLayout->addWidget(pathLengthLimitCheck_, 6, 0, 1, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("最大验证路径"), path), 6, 3);
    pathLayout->addWidget(pathLengthLimitSpin_, 6, 4, 1, 2);
    pathLayout->addWidget(targetCountLimitCheck_, 7, 0, 1, 3);
    pathLayout->addWidget(new QLabel(QStringLiteral("最大目标数量"), path), 7, 3);
    pathLayout->addWidget(targetCountLimitSpin_, 7, 4, 1, 2);
    pathLayout->addWidget(new QLabel(QStringLiteral("连续同步目标速度"), path), 8, 0);
    pathLayout->addWidget(scanSpeedSpin_, 8, 1);
    pathLayout->addWidget(new QLabel(
        QStringLiteral("连续段按物理速度下发；上方速度百分比只用于移到起点/原停稳模式"), path),
        8, 2, 1, 4);
    pathLayout->addWidget(binaryPlyCheck_, 9, 0, 1, 3);
    pathLayout->addWidget(new QLabel(
        QStringLiteral("需要更快/更小点云时，可把体素从 0.5 mm 调为 1.0 mm"), path),
        9, 3, 1, 3);
    QLabel* centerlinePolicyLabel = new QLabel(
        QStringLiteral("正式中心线策略"), path);
    centerlinePolicyCombo_ = new QComboBox(path);
    centerlinePolicyCombo_->setObjectName(
        QStringLiteral("centerlinePolicyCombo"));
    centerlinePolicyCombo_->addItem(
        QStringLiteral("Legacy｜连续优先，不屏蔽多路径歧义点"),
        static_cast<int>(LineLaserCenterlinePolicy::Legacy));
    centerlinePolicyCombo_->addItem(
        QStringLiteral("Shadow｜反射安全，屏蔽多路径歧义点"),
        static_cast<int>(LineLaserCenterlinePolicy::Shadow));
    const int configuredCenterlineIndex =
        centerlinePolicyCombo_->findData(
            static_cast<int>(profile_.scanCenterlinePolicy));
    centerlinePolicyCombo_->setCurrentIndex(
        configuredCenterlineIndex >= 0 ? configuredCenterlineIndex : 0);
    centerlinePolicyCombo_->setToolTip(QStringLiteral(
        "Legacy 保留旧中心线的全部正式点，连续性优先；"
        "Shadow 仍使用 Legacy 坐标，但会硬屏蔽质量算法判定的多路径区段。"
        "该选项同时作用于单点、停稳扫描和连续同步扫描，"
        "并写入 session_metadata.json。"));
    const bool configurableCenterline =
        profile_.id == QStringLiteral("scanner_650");
    centerlinePolicyLabel->setVisible(configurableCenterline);
    centerlinePolicyCombo_->setVisible(configurableCenterline);
    pathLayout->addWidget(centerlinePolicyLabel, 10, 0);
    pathLayout->addWidget(centerlinePolicyCombo_, 10, 1, 1, 5);
    root->addWidget(path);

    adaptive650Group_ = new QGroupBox(
        QStringLiteral("scanner_650 自适应扫描 v1｜固定全局蛇形 + 局部 NBS dry-run"),
        central);
    QGridLayout* adaptiveLayout = new QGridLayout(adaptive650Group_);
    adaptiveLaneOffsetXSpin_ = new QDoubleSpinBox(adaptive650Group_);
    adaptiveLaneOffsetYSpin_ = new QDoubleSpinBox(adaptive650Group_);
    adaptiveLaneOffsetZSpin_ = new QDoubleSpinBox(adaptive650Group_);
    for (QDoubleSpinBox* spin : {
             adaptiveLaneOffsetXSpin_,
             adaptiveLaneOffsetYSpin_,
             adaptiveLaneOffsetZSpin_}) {
        spin->setRange(-500.0, 500.0);
        spin->setDecimals(2);
        spin->setSingleStep(1.0);
        spin->setSuffix(QStringLiteral(" mm"));
    }
    adaptiveLaneOffsetXSpin_->setValue(
        adaptiveSerpentineDefaults_.laneOffsetMm[0]);
    adaptiveLaneOffsetYSpin_->setValue(
        adaptiveSerpentineDefaults_.laneOffsetMm[1]);
    adaptiveLaneOffsetZSpin_->setValue(
        adaptiveSerpentineDefaults_.laneOffsetMm[2]);
    adaptiveLaneCountSpin_ = new QSpinBox(adaptive650Group_);
    adaptiveLaneCountSpin_->setRange(1, 50);
    adaptiveLaneCountSpin_->setValue(
        adaptiveSerpentineDefaults_.laneCount);
    adaptiveLaneCountSpin_->setSuffix(QStringLiteral(" 条"));
    adaptiveTransitionSpeedSpin_ = new QDoubleSpinBox(adaptive650Group_);
    adaptiveTransitionSpeedSpin_->setRange(10.0, 50.0);
    adaptiveTransitionSpeedSpin_->setDecimals(2);
    adaptiveTransitionSpeedSpin_->setValue(
        adaptiveSerpentineDefaults_.transitionSpeedMmS);
    adaptiveTransitionSpeedSpin_->setSuffix(QStringLiteral(" mm/s"));
    adaptiveLeadInSpin_ = new QDoubleSpinBox(adaptive650Group_);
    adaptiveLeadOutSpin_ = new QDoubleSpinBox(adaptive650Group_);
    for (QDoubleSpinBox* spin :
         {adaptiveLeadInSpin_, adaptiveLeadOutSpin_}) {
        spin->setRange(0.0, 100.0);
        spin->setDecimals(2);
        spin->setSuffix(QStringLiteral(" mm"));
    }
    adaptiveLeadInSpin_->setValue(
        adaptiveSerpentineDefaults_.leadInMm);
    adaptiveLeadOutSpin_->setValue(
        adaptiveSerpentineDefaults_.leadOutMm);
    adaptiveQualityVoxelSpin_ = new QDoubleSpinBox(adaptive650Group_);
    adaptiveQualityVoxelSpin_->setRange(0.2, 20.0);
    adaptiveQualityVoxelSpin_->setDecimals(2);
    adaptiveQualityVoxelSpin_->setValue(
        adaptiveQualityMapOptions_.voxelSizeMm);
    adaptiveQualityVoxelSpin_->setSuffix(QStringLiteral(" mm"));
    adaptiveBeamHorizonSpin_ = new QSpinBox(adaptive650Group_);
    adaptiveBeamHorizonSpin_->setRange(2, 3);
    adaptiveBeamHorizonSpin_->setValue(
        adaptiveSearchOptions_.horizon);
    adaptiveBeamWidthSpin_ = new QSpinBox(adaptive650Group_);
    adaptiveBeamWidthSpin_->setRange(2, 32);
    adaptiveBeamWidthSpin_->setValue(
        static_cast<int>(adaptiveSearchOptions_.beamWidth));
    adaptiveMappingModeCombo_ = new QComboBox(adaptive650Group_);
    adaptiveMappingModeCombo_->addItem(
        QStringLiteral("普通快速建图（仅 raw / voxel）"), false);
    adaptiveMappingModeCombo_->addItem(
        QStringLiteral("自适应质量建图（保留局部 NBS 证据）"), true);
    adaptiveMappingModeCombo_->setCurrentIndex(0);
    adaptiveMappingModeCombo_->setToolTip(QStringLiteral(
        "普通模式跳过650nm质量候选、V槽时序验证和相邻轮廓过滤；"
        "只有需要随后生成局部NBS补扫计划时才选择自适应质量模式。"));
    adaptiveGenerateGlobalButton_ = new QPushButton(
        QStringLiteral("1. 生成全局蛇形 dry-run"), adaptive650Group_);
    adaptiveEvaluateGlobalButton_ = new QPushButton(
        QStringLiteral("2. FR5 评估全局路径"), adaptive650Group_);
    adaptiveStartGlobalButton_ = new QPushButton(
        QStringLiteral("3. 执行固定全局粗扫"), adaptive650Group_);
    adaptivePlanLocalButton_ = new QPushButton(
        QStringLiteral("4. 质量地图 + 局部 greedy/beam"), adaptive650Group_);
    adaptiveStatusLabel_ = new QLabel(
        QStringLiteral(
            "尚未生成计划。碰撞模型为 UNKNOWN；局部自适应计划只输出 dry-run，"
            "不会自动运动。"),
        adaptive650Group_);
    adaptiveStatusLabel_->setWordWrap(true);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("带间偏移 X/Y/Z"), adaptive650Group_),
        0, 0);
    adaptiveLayout->addWidget(adaptiveLaneOffsetXSpin_, 0, 1);
    adaptiveLayout->addWidget(adaptiveLaneOffsetYSpin_, 0, 2);
    adaptiveLayout->addWidget(adaptiveLaneOffsetZSpin_, 0, 3);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("扫描带数"), adaptive650Group_), 0, 4);
    adaptiveLayout->addWidget(adaptiveLaneCountSpin_, 0, 5);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("转场速度"), adaptive650Group_), 1, 0);
    adaptiveLayout->addWidget(adaptiveTransitionSpeedSpin_, 1, 1);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("lead-in / lead-out"), adaptive650Group_),
        1, 2);
    adaptiveLayout->addWidget(adaptiveLeadInSpin_, 1, 3);
    adaptiveLayout->addWidget(adaptiveLeadOutSpin_, 1, 4);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("质量体素"), adaptive650Group_), 2, 0);
    adaptiveLayout->addWidget(adaptiveQualityVoxelSpin_, 2, 1);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("beam 时域 / 宽度"), adaptive650Group_),
        2, 2);
    adaptiveLayout->addWidget(adaptiveBeamHorizonSpin_, 2, 3);
    adaptiveLayout->addWidget(adaptiveBeamWidthSpin_, 2, 4);
    adaptiveLayout->addWidget(adaptiveGenerateGlobalButton_, 3, 0, 1, 2);
    adaptiveLayout->addWidget(adaptiveEvaluateGlobalButton_, 3, 2);
    adaptiveLayout->addWidget(adaptiveStartGlobalButton_, 3, 3);
    adaptiveLayout->addWidget(adaptivePlanLocalButton_, 3, 4, 1, 2);
    adaptiveLayout->addWidget(adaptiveStatusLabel_, 4, 0, 1, 6);
    adaptiveLayout->addWidget(
        new QLabel(QStringLiteral("连续建图模式"), adaptive650Group_),
        5, 0);
    adaptiveLayout->addWidget(adaptiveMappingModeCombo_, 5, 1, 1, 5);
    adaptive650Group_->setVisible(profile_.id == QStringLiteral("scanner_650"));
    root->addWidget(adaptive650Group_);

    QGroupBox* executionGroup = new QGroupBox(
        QStringLiteral("执行控制"), central);
    QGridLayout* actions = new QGridLayout(executionGroup);
    dryRunCheck_ = new QCheckBox(
        QStringLiteral("dry-run（不运动、不拍照）"), executionGroup);
    dryRunCheck_->setChecked(true);
    safetyConfirmCheck_ = new QCheckBox(
        QStringLiteral("我确认真运动路径安全且物理急停可用"),
        executionGroup);
    dryRunButton_ = new QPushButton(
        QStringLiteral("生成并打印目标列表"), executionGroup);
    captureCurrentButton_ = new QPushButton(
        QStringLiteral("单点常亮验证（不移动）"), executionGroup);
    startScanButton_ = new QPushButton(
        QStringLiteral("开始停稳扫描"), executionGroup);
    startContinuousButton_ = new QPushButton(
        QStringLiteral("开始 60fps 连续同步扫描"), executionGroup);
    stopButton_ = new QPushButton(
        QStringLiteral("停止扫描 / StopMotion"), executionGroup);
    stopButton_->setStyleSheet(QStringLiteral("background:#b00020;color:white;font-weight:bold;"));
    continuousFinalizeProgressLabel_ = new QLabel(
        QStringLiteral("点云后台收尾"), executionGroup);
    continuousFinalizeProgress_ = new QProgressBar(executionGroup);
    continuousFinalizeProgress_->setRange(0, 100);
    continuousFinalizeProgress_->setValue(0);
    continuousFinalizeProgress_->setTextVisible(true);
    continuousFinalizeProgressLabel_->setVisible(false);
    continuousFinalizeProgress_->setVisible(false);
    actions->addWidget(dryRunCheck_, 0, 0, 1, 2);
    actions->addWidget(safetyConfirmCheck_, 0, 2, 1, 3);
    actions->addWidget(dryRunButton_, 1, 0);
    actions->addWidget(captureCurrentButton_, 1, 1);
    actions->addWidget(startScanButton_, 1, 2);
    actions->addWidget(startContinuousButton_, 1, 3);
    actions->addWidget(stopButton_, 1, 4);
    actions->addWidget(continuousFinalizeProgressLabel_, 2, 0, 1, 2);
    actions->addWidget(continuousFinalizeProgress_, 2, 2, 1, 3);
    root->addWidget(executionGroup);
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
    profileTable_->setMinimumSize(610, 420);
    profileTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    profileTable_->horizontalHeader()->setStretchLastSection(true);
    QWidget* preview = new QWidget(splitter);
    QVBoxLayout* previewLayout = new QVBoxLayout(preview);
    imageView_ = new ImageView(preview);
    imageView_->setMinimumHeight(260);
    imageView_->setEmptyText(QStringLiteral("%1：暂无常亮激光图像")
                             .arg(profile_.displayName));
    logView_ = new QPlainTextEdit(preview);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(4000);
    logView_->setMinimumHeight(190);
    preview->setMinimumSize(560, 420);
    previewLayout->addWidget(imageView_, 1);
    previewLayout->addWidget(new QLabel(QStringLiteral("扫描日志"), preview));
    previewLayout->addWidget(logView_);
    splitter->addWidget(profileTable_);
    splitter->addWidget(preview);
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 4);
    splitter->setChildrenCollapsible(false);
    splitter->setMinimumHeight(440);
    root->addWidget(splitter, 1);

    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setObjectName(QStringLiteral("scanPageScrollArea"));
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setWidget(central);
    setCentralWidget(scrollArea);

    connect(connectCameraButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::connectCamera);
    connect(disconnectCameraButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::disconnectCamera);
    connect(connectLaserButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::connectLaserController);
    connect(enableProfileLaserButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::enableProfileLaser);
    connect(enableBothLasersButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::enableBothLasers);
    connect(disableAllLasersButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::disableAllLasers);
    connect(connectRobotButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::connectRobot);
    connect(disconnectRobotButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::disconnectRobot);
    connect(readPoseButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::readCurrentPose);
    connect(teachStartButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::teachStart);
    connect(teachEndButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::teachEnd);
    connect(editStartButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::editStartPose);
    connect(editEndButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::editEndPose);
    connect(swapPathButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::swapPathEndpoints);
    connect(dryRunButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::generateDryRun);
    connect(captureCurrentButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::captureCurrentProfile);
    connect(startScanButton_, &QPushButton::clicked, this, &HikConstantLaserScanWindow::startScan);
    connect(startContinuousButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::startContinuousScan);
    connect(adaptiveGenerateGlobalButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::generateAdaptiveGlobalPlan);
    connect(adaptiveEvaluateGlobalButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::evaluateAdaptiveGlobalPlan);
    connect(adaptiveStartGlobalButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::startAdaptiveGlobalScan);
    connect(adaptivePlanLocalButton_, &QPushButton::clicked,
            this, &HikConstantLaserScanWindow::planAdaptiveLocalRescan);
    connect(adaptiveMappingModeCombo_,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int) {
                if (!adaptiveQualityMappingSelected()) {
                    clearAdaptiveQualityArtifacts();
                    adaptiveStatusLabel_->setText(QStringLiteral(
                        "普通快速建图：只计算 raw/voxel；"
                        "局部 NBS 需要先切换到自适应质量建图并重新粗扫。"));
                } else {
                    const int shadowIndex =
                        centerlinePolicyCombo_->findData(
                            static_cast<int>(
                                LineLaserCenterlinePolicy::Shadow));
                    if (shadowIndex >= 0 &&
                        centerlinePolicyCombo_->currentIndex() !=
                            shadowIndex) {
                        centerlinePolicyCombo_->setCurrentIndex(
                            shadowIndex);
                        appendLog(QStringLiteral(
                            "自适应质量建图依赖多路径候选分析，"
                            "正式中心线策略已自动切换为 Shadow。"));
                    }
                    adaptiveStatusLabel_->setText(QStringLiteral(
                        "自适应质量建图已选择：粗扫结束后将执行质量候选、"
                        "V槽时序验证和相邻轮廓过滤，供局部 NBS 使用。"));
                }
                updateUi();
            });
    connect(centerlinePolicyCombo_,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this, [this](int) {
                applySelectedScanCenterlinePolicy(true);
                updateUi();
            });
    const auto invalidateAdaptiveGlobal =
        [this]() {
            adaptiveGlobalPlanReady_ = false;
            adaptiveGlobalKinematicsPassed_ = false;
            adaptiveGlobalEvaluationPoseValid_ = false;
            adaptiveGlobalEvaluations_.clear();
            if (profile_.id == QStringLiteral("scanner_650")) {
                adaptiveStatusLabel_->setText(QStringLiteral(
                    "蛇形几何参数已变化，请重新生成 dry-run 并重新做 FR5 评估。"));
            }
            updateUi();
        };
    for (QDoubleSpinBox* spin : {
             adaptiveLaneOffsetXSpin_,
             adaptiveLaneOffsetYSpin_,
             adaptiveLaneOffsetZSpin_,
             adaptiveTransitionSpeedSpin_,
             adaptiveLeadInSpin_,
             adaptiveLeadOutSpin_,
             scanSpeedSpin_,
             exposureSpin_}) {
        connect(spin, qOverload<double>(
                          &QDoubleSpinBox::valueChanged),
                this, [invalidateAdaptiveGlobal](double) {
                    invalidateAdaptiveGlobal();
                });
    }
    connect(adaptiveLaneCountSpin_,
            qOverload<int>(&QSpinBox::valueChanged),
            this, [invalidateAdaptiveGlobal](int) {
                invalidateAdaptiveGlobal();
            });
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
    connect(this,
            &HikConstantLaserScanWindow::
                requestExecuteAdaptiveTrajectory,
            robotSession_,
            &FairinoRobotSession::executeAdaptiveTrajectory);
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
    connect(robotSession_, &FairinoRobotSession::motionTimingMeasured,
            this,
            &HikConstantLaserScanWindow::onRobotMotionTimingMeasured,
            Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::kinematicPathEvaluated,
            this, &HikConstantLaserScanWindow::onKinematicPathEvaluated,
            Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::kinematicPathBatchFinished,
            this, &HikConstantLaserScanWindow::onKinematicPathBatchFinished,
            Qt::QueuedConnection);
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
    if (continuousFinalizationThread_.joinable()) {
        continuousFinalizationThread_.join();
    }
    continuousFinalizationActive_ = false;
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
    if (continuousFinalizationActive_) {
        closeAfterContinuousFinalization_ = true;
        scanStatusLabel_->setText(QStringLiteral(
            "关闭请求已记录：正在完成点云后台保存，完成后自动关闭；"
            "为避免生成损坏的 PLY，不会强制终止写盘。"));
        appendLog(scanStatusLabel_->text());
        event->ignore();
        return;
    }
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
    enableBothLasersButton_->setEnabled(
        idle && laserTransportReady &&
        laserStatus_.state != LineLaserState::Both);
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
    swapPathButton_->setEnabled(idle && startTaught_ && endTaught_);
    reloadCalibrationButton_->setEnabled(idle);
    reconstructionDepthMinimumSpin_->setEnabled(idle);
    reconstructionDepthMaximumSpin_->setEnabled(idle);
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
    const bool adaptive650 =
        profile_.id == QStringLiteral("scanner_650");
    adaptiveGenerateGlobalButton_->setEnabled(
        adaptive650 && adaptiveConfigReady_ && idle &&
        startTaught_ && endTaught_);
    adaptiveEvaluateGlobalButton_->setEnabled(
        adaptive650 && adaptiveConfigReady_ && idle &&
        robotAvailable && robotConnected_ &&
        adaptiveGlobalPlanReady_);
    adaptiveStartGlobalButton_->setEnabled(
        adaptive650 && adaptiveConfigReady_ && idle &&
        robotAvailable && robotConnected_ &&
        cameraConnected_ && calibrationReady_ && profileLaserReady &&
        adaptiveGlobalPlanReady_ && adaptiveGlobalKinematicsPassed_ &&
        !dryRunCheck_->isChecked());
    adaptivePlanLocalButton_->setEnabled(
        adaptive650 && adaptiveConfigReady_ && idle &&
        robotAvailable && robotConnected_ &&
        calibrationReady_ &&
        (!lastContinuousArtifacts_.formal.empty() ||
         !lastContinuousArtifacts_.rejected.empty()));
    for (QDoubleSpinBox* spin : {
             adaptiveLaneOffsetXSpin_,
             adaptiveLaneOffsetYSpin_,
             adaptiveLaneOffsetZSpin_,
             adaptiveTransitionSpeedSpin_,
             adaptiveLeadInSpin_,
             adaptiveLeadOutSpin_,
             adaptiveQualityVoxelSpin_}) {
        spin->setEnabled(adaptive650 && idle);
    }
    adaptiveLaneCountSpin_->setEnabled(adaptive650 && idle);
    adaptiveBeamHorizonSpin_->setEnabled(adaptive650 && idle);
    adaptiveBeamWidthSpin_->setEnabled(adaptive650 && idle);
    adaptiveMappingModeCombo_->setEnabled(adaptive650 && idle);
    centerlinePolicyCombo_->setEnabled(
        adaptive650 && idle && !adaptiveQualityMappingSelected());
    const bool scanStateMachineActive =
        scanState_ != ScanState::Idle ||
        (continuousState_ != ContinuousState::Idle &&
         continuousState_ != ContinuousState::Finalizing);
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
    binaryPlyCheck_->setEnabled(idle);
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

void HikConstantLaserScanWindow::enableBothLasers() {
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
    const QMessageBox::StandardButton answer = QMessageBox::warning(
        this,
        QStringLiteral("同时开启两路激光"),
        QStringLiteral(
            "将同时开启 450 nm 和 650 nm 激光。请确认防护眼镜、遮光、"
            "联锁和现场人员安全措施均已就绪。\n\n"
            "双开状态只用于人工观察；单点、停稳和连续扫描仍要求本组单路激光。"),
        QMessageBox::Yes | QMessageBox::Cancel,
        QMessageBox::Cancel);
    if (answer != QMessageBox::Yes) {
        return;
    }
    appendLog(QStringLiteral(
        "请求同时开启 450 nm（Pin 11）与 650 nm（Pin 7）；"
        "两路 TTL 将持续 HIGH，直到关光、租约失效或连接断开。"));
    laserController_->setBoth();
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
    root.insert(QStringLiteral("schema_version"), 2);
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

void HikConstantLaserScanWindow::swapPathEndpoints() {
    if (!startTaught_ || !endTaught_) {
        showError(QStringLiteral("无法交换路径"),
                  QStringLiteral("请先示教起点和终点。"));
        return;
    }
    const QString previousStart = poseText(startPose_);
    const QString previousEnd = poseText(endPose_);
    std::string coreError;
    if (!hik_scan::reverseLinearFlangePath(
            &startPose_, &endPose_, &coreError)) {
        showError(QStringLiteral("无法交换路径"),
                  QString::fromStdString(coreError));
        return;
    }

    targets_.clear();
    currentTargetIndex_ = -1;
    adaptiveGlobalPlanReady_ = false;
    adaptiveGlobalKinematicsPassed_ = false;
    adaptiveGlobalEvaluationPoseValid_ = false;
    adaptiveGlobalEvaluations_.clear();
    safetyConfirmCheck_->setChecked(false);
    startPoseLabel_->setText(
        QStringLiteral("起点: %1").arg(poseText(startPose_)));
    endPoseLabel_->setText(
        QStringLiteral("终点位置: %1（扫描时忽略终点 RPY）")
            .arg(poseText(endPose_)));
    scanStatusLabel_->setText(QStringLiteral(
        "扫描方向已反转；交换未触发运动，请重新核对路径并执行 dry-run。"));
    appendLog(QStringLiteral(
        "示教路径已反向：旧起点=%1，旧终点=%2；新起点=%3，新终点=%4。"
        "仅交换 XYZ，扫描 RPY 保持为 [%5, %6, %7] deg；"
        "已清除旧路径/FR5评估并取消安全确认，请重新执行 dry-run。")
        .arg(previousStart, previousEnd,
             poseText(startPose_), poseText(endPose_))
        .arg(startPose_.rx, 0, 'f', 3)
        .arg(startPose_.ry, 0, 'f', 3)
        .arg(startPose_.rz, 0, 'f', 3));
    if (profile_.id == QStringLiteral("scanner_650")) {
        adaptiveStatusLabel_->setText(QStringLiteral(
            "示教路径方向已反转；请重新生成全局蛇形 dry-run 并重新做 FR5 评估。"));
    }
    updateUi();
}

void HikConstantLaserScanWindow::editStartPose() {
    if (!startTaught_) return;
    const hik_scan::Pose6D before = startPose_;
    if (!editPoseDialog(&startPose_, QStringLiteral("编辑扫描起点"), false)) return;
    adaptiveGlobalPlanReady_ = false;
    adaptiveGlobalKinematicsPassed_ = false;
    adaptiveGlobalEvaluationPoseValid_ = false;
    startPoseLabel_->setText(QStringLiteral("起点: %1").arg(poseText(startPose_)));
    appendLog(QStringLiteral("起点已手动修改：%1 → %2；请重新执行 dry-run。")
              .arg(poseText(before), poseText(startPose_)));
}

void HikConstantLaserScanWindow::editEndPose() {
    if (!endTaught_) return;
    const hik_scan::Pose6D before = endPose_;
    if (!editPoseDialog(&endPose_, QStringLiteral("编辑扫描终点"), true)) return;
    adaptiveGlobalPlanReady_ = false;
    adaptiveGlobalKinematicsPassed_ = false;
    adaptiveGlobalEvaluationPoseValid_ = false;
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
    if (!loadFormalCalibration(&error)) {
        calibrationStatusLabel_->setText(
            QStringLiteral("正式标定未就绪：%1").arg(error));
        calibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        qWarning().noquote() << QStringLiteral("[%1] 重新加载标定失败：%2")
            .arg(profile_.id, error);
        showError(QStringLiteral("标定加载失败"), error);
    } else {
        appendLog(QStringLiteral("已重新加载三份正式标定。"));
        qInfo().noquote() << QStringLiteral("[%1] 三份正式标定已重新加载。")
            .arg(profile_.id);
    }
    updateUi();
}

bool HikConstantLaserScanWindow::applyConfiguredReconstructionDepthRange(
        QString* error) {
    if (error) error->clear();
    if (!reconstructionDepthMinimumSpin_ ||
        !reconstructionDepthMaximumSpin_) {
        if (error) *error = QStringLiteral("重建深度范围控件尚未初始化。");
        return false;
    }
    const double minimumDepthMm =
        reconstructionDepthMinimumSpin_->value();
    const double maximumDepthMm =
        reconstructionDepthMaximumSpin_->value();
    if (!std::isfinite(minimumDepthMm) ||
        !std::isfinite(maximumDepthMm) ||
        minimumDepthMm <= 0.0 ||
        maximumDepthMm <= minimumDepthMm) {
        if (error) {
            *error = QStringLiteral(
                "相机 Z 重建范围无效：要求 0 < 最小值 < 最大值，当前为 %1..%2 mm。")
                .arg(minimumDepthMm, 0, 'f', 1)
                .arg(maximumDepthMm, 0, 'f', 1);
        }
        return false;
    }
    if (!intrinsics_.ok || !laserPlane_.ok) {
        if (error) *error = QStringLiteral("正式内参或激光平面尚未加载。");
        return false;
    }

    cv::Mat configuredMask;
    std::string coreError;
    if (!hik_calibration::buildLaserPlaneValidityMask(
            intrinsics_.imageSize, intrinsics_, laserPlane_,
            minimumDepthMm, maximumDepthMm,
            profileOptions_.reconstruction.stripe.quality.roi,
            &configuredMask, &coreError)) {
        if (error) {
            *error = QStringLiteral("无法建立可配置条纹深度走廊: %1")
                .arg(QString::fromStdString(coreError));
        }
        return false;
    }
    profileOptions_.reconstruction.minimumDepthMm = minimumDepthMm;
    profileOptions_.reconstruction.maximumDepthMm = maximumDepthMm;
    profileOptions_.reconstruction.stripeValidityMask = configuredMask;
    return true;
}

void HikConstantLaserScanWindow::updateCalibrationStatusText() {
    const double configuredMinimum =
        profileOptions_.reconstruction.minimumDepthMm;
    const double configuredMaximum =
        profileOptions_.reconstruction.maximumDepthMm;
    const double validatedMinimum = laserMetadata_.validCameraZMinMm;
    const double validatedMaximum = laserMetadata_.validCameraZMaxMm;
    const bool outsideValidatedRange =
        configuredMinimum < validatedMinimum ||
        configuredMaximum > validatedMaximum;
    calibrationStatusLabel_->setText(QStringLiteral(
        "%1 %2：相机 SN=%3，图像=%4×%5；重建相机 Z=%6–%7 mm，"
        "标定验证 Z=%8–%9 mm%10；输出 base_link；"
        "常亮背景核=%11×%12；质量走廊=%13 px。")
        .arg(profile_.id)
        .arg(calibrationProvenanceOverrideActive_
                 ? QStringLiteral("设备参数原样模式（手眼内参来源不一致）")
                 : QStringLiteral("已加载"))
        .arg(QString::fromStdString(intrinsicsMetadata_.cameraSerial))
        .arg(intrinsics_.imageSize.width).arg(intrinsics_.imageSize.height)
        .arg(configuredMinimum, 0, 'f', 1)
        .arg(configuredMaximum, 0, 'f', 1)
        .arg(validatedMinimum, 0, 'f', 1)
        .arg(validatedMaximum, 0, 'f', 1)
        .arg(outsideValidatedRange
                 ? QStringLiteral("（扩展区未经正式标定精度验证）")
                 : QString())
        .arg(profileOptions_.backgroundKernelWidth)
        .arg(profileOptions_.backgroundKernelHeight)
        .arg(profileOptions_.reconstruction.stripeValidityMask.empty()
                 ? 0
                 : cv::countNonZero(
                       profileOptions_.reconstruction.stripeValidityMask)));
    calibrationStatusLabel_->setStyleSheet(
        calibrationProvenanceOverrideActive_ || outsideValidatedRange
            ? QStringLiteral("color:#b26a00;font-weight:bold;")
            : QStringLiteral("color:#087f23;"));
}

bool HikConstantLaserScanWindow::loadFormalCalibration(QString* error) {
    if (error) error->clear();
    calibrationReady_ = false;
    calibrationProvenanceOverrideActive_ = false;
    handEyeDeclaredIntrinsicsSha256_.clear();
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
    handEyeDeclaredIntrinsicsSha256_ = handEyeIntrinsicHash;
    const bool laserIntrinsicMismatch =
        laserIntrinsicHash.compare(intrinsicsSha256_, Qt::CaseInsensitive) != 0;
    const bool handEyeIntrinsicMismatch =
        handEyeIntrinsicHash.compare(intrinsicsSha256_, Qt::CaseInsensitive) != 0;
    if (laserIntrinsicMismatch ||
        (handEyeIntrinsicMismatch && !useDeviceCalibrationAsIs_)) {
        if (error) {
            *error = QStringLiteral(
                "设备组 %1 的内参依赖不一致：当前内参=%2，"
                "激光平面声明=%3，手眼声明=%4。不能通过手改哈希绕过，"
                "必须使用当前内参重新求解不一致的标定项。")
                .arg(profile_.id, intrinsicsSha256_,
                     laserIntrinsicHash.isEmpty()
                         ? QStringLiteral("<缺失>") : laserIntrinsicHash,
                     handEyeIntrinsicHash.isEmpty()
                         ? QStringLiteral("<缺失>") : handEyeIntrinsicHash);
        }
        return false;
    }
    // Explicit operator mode: use the numeric T_flange_camera exactly as it is
    // stored in this profile directory.  Laser-plane/intrinsics consistency is
    // never bypassed because it directly defines each reconstructed 3-D ray.
    calibrationProvenanceOverrideActive_ = handEyeIntrinsicMismatch;
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
    profileOptions_.reconstruction.stripe.quality.roi =
        stripeRoi(profile_, intrinsics_.imageSize);
    if (!applyConfiguredReconstructionDepthRange(error)) {
        return false;
    }
    calibrationReady_ = true;
    updateCalibrationStatusText();
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
    applySelectedScanCenterlinePolicy(false);
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
        QStringLiteral("将真实移动 FR5 到 %1 个法兰目标。\n起点：%2\n终点位置：%3\n路径长度=%4 mm，长度保护=%5。\n速度=%6%，步距≈%7 mm。\n正式中心线策略=%8。\n\n确认完整直线路径无碰撞并开始？")
            .arg(static_cast<int>(targets_.size())).arg(poseText(startPose_))
            .arg(poseText(endPose_)).arg(pathLength, 0, 'f', 3)
            .arg(lengthProtection).arg(velocitySpin_->value(), 0, 'f', 1)
            .arg(stepSpin_->value(), 0, 'f', 2)
            .arg(lineLaserCenterlinePolicyName(
                selectedScanCenterlinePolicy())),
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
    qualityAmbiguousBranches_.clear();
    shadowMaskedLegacyRejectedCloud_.clear();
    publicationGateRejectedCloud_.clear();
    opticalRejectedCandidateCloud_.clear();
    qualityVGrooveValidationResult_ =
        hik_scan::VGrooveTemporalValidationResult();
    qualityRejectedCloud_.clear();
    qualitySupportResult_ =
        hik_scan::AdjacentProfileSupportResult();
    qualityAdjacentRejectedPointCount_ = 0U;
    qualityVoxelPointCount_ = 0U;
    profileRows_.clear();
    refreshTable();
    currentTargetIndex_ = 0;
    singlePointMode_ = false;
    stopRequested_ = false;
    appendLog(QStringLiteral("真实停稳扫描开始，输出目录: %1").arg(scanSessionDir_));
    issueMoveForCurrentTarget();
}

bool HikConstantLaserScanWindow::loadAdaptiveScanConfig(
        QString* error) {
    if (error) error->clear();
    adaptiveSerpentineDefaults_ =
        hik_adaptive::SerpentineOptions();
    adaptiveSerpentineDefaults_.laneCount = 5;
    adaptiveQualityMapOptions_ =
        hik_adaptive::QualityMapOptions();
    adaptiveRoiOptions_ =
        hik_adaptive::RoiClusteringOptions();
    adaptiveCandidateOptions_ =
        hik_adaptive::CandidateLibraryOptions();
    adaptiveSearchOptions_ =
        hik_adaptive::SearchOptions();
    adaptiveSearchOptions_.horizon = 3;
    adaptiveSearchOptions_.beamWidth = 12U;
    adaptiveFr5Options_ = hik_fr5::PathEvaluationOptions();
    adaptiveExposureScales_ = {0.5, 1.0};
    adaptiveMaximumCandidateRois_ = 8U;
    adaptiveShortlistSize_ = 12U;
    adaptiveMaximumCandidatesPerRoi_ = 3;

    cv::FileStorage storage(
        localPath(adaptiveConfigPath_),
        cv::FileStorage::READ);
    if (!storage.isOpened()) {
        if (error) {
            *error = QStringLiteral("无法读取 %1")
                         .arg(adaptiveConfigPath_);
        }
        return false;
    }
    int schemaVersion = 0;
    std::string profileId;
    storage["schema_version"] >> schemaVersion;
    storage["profile_id"] >> profileId;
    if (schemaVersion != 1 || profileId != "scanner_650") {
        if (error) {
            *error = QStringLiteral(
                "配置 schema/profile 不匹配：schema=%1，profile=%2")
                .arg(schemaVersion)
                .arg(QString::fromStdString(profileId));
        }
        return false;
    }
    const auto readDouble =
        [error](const cv::FileNode& node,
                const char* key, double* value,
                double minimum, double maximum) {
            const cv::FileNode field = node[key];
            if (field.empty()) {
                if (error) {
                    *error = QStringLiteral("缺少配置项 %1")
                                 .arg(QString::fromLatin1(key));
                }
                return false;
            }
            double parsed = 0.0;
            field >> parsed;
            if (!std::isfinite(parsed) ||
                parsed < minimum || parsed > maximum) {
                if (error) {
                    *error = QStringLiteral("配置项 %1 超出范围")
                                 .arg(QString::fromLatin1(key));
                }
                return false;
            }
            *value = parsed;
            return true;
        };
    const auto readInt =
        [error](const cv::FileNode& node,
                const char* key, int* value,
                int minimum, int maximum) {
            const cv::FileNode field = node[key];
            if (field.empty()) {
                if (error) {
                    *error = QStringLiteral("缺少配置项 %1")
                                 .arg(QString::fromLatin1(key));
                }
                return false;
            }
            int parsed = 0;
            field >> parsed;
            if (parsed < minimum || parsed > maximum) {
                if (error) {
                    *error = QStringLiteral("配置项 %1 超出范围")
                                 .arg(QString::fromLatin1(key));
                }
                return false;
            }
            *value = parsed;
            return true;
        };
    const auto readVector =
        [error](const cv::FileNode& node,
                const char* key,
                std::vector<double>* values,
                double minimum, double maximum) {
            const cv::FileNode field = node[key];
            if (field.empty() || field.type() != cv::FileNode::SEQ) {
                if (error) {
                    *error = QStringLiteral("配置项 %1 必须是数组")
                                 .arg(QString::fromLatin1(key));
                }
                return false;
            }
            std::vector<double> parsed;
            field >> parsed;
            if (parsed.empty()) {
                if (error) {
                    *error = QStringLiteral("配置数组 %1 不能为空")
                                 .arg(QString::fromLatin1(key));
                }
                return false;
            }
            for (double value : parsed) {
                if (!std::isfinite(value) ||
                    value < minimum || value > maximum) {
                    if (error) {
                        *error = QStringLiteral(
                            "配置数组 %1 含越界值")
                            .arg(QString::fromLatin1(key));
                    }
                    return false;
                }
            }
            *values = std::move(parsed);
            return true;
        };

    const cv::FileNode global = storage["global"];
    std::vector<double> laneOffset;
    int maximumSegmentCount = 0;
    int enableArcTransitions = 0;
    if (global.empty() ||
        !readVector(global, "lane_offset_mm", &laneOffset,
                    -500.0, 500.0) ||
        laneOffset.size() != 3U ||
        !readInt(global, "lane_count",
                 &adaptiveSerpentineDefaults_.laneCount, 1, 50) ||
        !readDouble(global, "transition_speed_mm_s",
                    &adaptiveSerpentineDefaults_.transitionSpeedMmS,
                    10.0, 50.0) ||
        !readDouble(global, "lead_in_mm",
                    &adaptiveSerpentineDefaults_.leadInMm,
                    0.0, 200.0) ||
        !readDouble(global, "lead_out_mm",
                    &adaptiveSerpentineDefaults_.leadOutMm,
                    0.0, 200.0) ||
        !readInt(global, "enable_arc_transitions",
                 &enableArcTransitions, 0, 1) ||
        !readDouble(global, "minimum_arc_radius_mm",
                    &adaptiveSerpentineDefaults_.minimumArcRadiusMm,
                    0.5, 500.0) ||
        !readDouble(global, "maximum_arc_radius_mm",
                    &adaptiveSerpentineDefaults_.maximumArcRadiusMm,
                    0.5, 5000.0) ||
        !readDouble(global, "transition_blend_radius_mm",
                    &adaptiveSerpentineDefaults_
                         .transitionBlendRadiusMm,
                    0.0, 1000.0) ||
        !readDouble(global, "maximum_arc_tangent_error_deg",
                    &adaptiveSerpentineDefaults_
                         .maximumArcTangentErrorDeg,
                    0.0, 30.0) ||
        !readInt(global, "maximum_segment_count",
                 &maximumSegmentCount, 1, 999) ||
        !readDouble(global, "maximum_total_length_mm",
                    &adaptiveSerpentineDefaults_.maximumTotalLengthMm,
                    1.0, 100000.0)) {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("global 配置无效");
        }
        return false;
    }
    if (adaptiveSerpentineDefaults_.maximumArcRadiusMm <
        adaptiveSerpentineDefaults_.minimumArcRadiusMm) {
        if (error) {
            *error = QStringLiteral(
                "maximum_arc_radius_mm 不能小于 "
                "minimum_arc_radius_mm");
        }
        return false;
    }
    adaptiveSerpentineDefaults_.laneOffsetMm =
        cv::Vec3d(laneOffset[0], laneOffset[1], laneOffset[2]);
    adaptiveSerpentineDefaults_.enableArcTransitions =
        enableArcTransitions != 0;
    adaptiveSerpentineDefaults_.maximumSegmentCount =
        maximumSegmentCount;

    const cv::FileNode quality = storage["quality_map"];
    int minimumAccepted = 0;
    if (quality.empty() ||
        !readDouble(quality, "voxel_size_mm",
                    &adaptiveQualityMapOptions_.voxelSizeMm,
                    0.1, 50.0) ||
        !readDouble(quality, "minimum_mean_confidence",
                    &adaptiveQualityMapOptions_.minimumMeanConfidence,
                    0.0, 1.0) ||
        !readDouble(quality, "rejected_ratio_threshold",
                    &adaptiveQualityMapOptions_.rejectedRatioThreshold,
                    0.0, 1.0) ||
        !readInt(quality, "minimum_accepted_observations",
                 &minimumAccepted, 1, 1000000)) {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("quality_map 配置无效");
        }
        return false;
    }
    adaptiveQualityMapOptions_.minimumAcceptedObservations =
        static_cast<std::uint64_t>(minimumAccepted);

    const cv::FileNode roi = storage["roi"];
    int minimumVoxelCount = 0;
    int maximumCandidateRois = 0;
    if (roi.empty() ||
        !readInt(roi, "minimum_voxel_count",
                 &minimumVoxelCount, 1, 1000000) ||
        !readDouble(roi, "padding_mm",
                    &adaptiveRoiOptions_.paddingMm,
                    0.0, 100.0) ||
        !readDouble(roi, "maximum_extent_mm",
                    &adaptiveRoiOptions_.maximumClusterExtentMm,
                    1.0, 10000.0) ||
        !readInt(roi, "maximum_candidate_rois",
                 &maximumCandidateRois, 1, 64)) {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("roi 配置无效");
        }
        return false;
    }
    adaptiveRoiOptions_.minimumVoxelCount =
        static_cast<std::size_t>(minimumVoxelCount);
    adaptiveMaximumCandidateRois_ =
        static_cast<std::size_t>(maximumCandidateRois);

    const cv::FileNode candidate = storage["candidate_library"];
    int maximumCandidates = 0;
    int includeReverse = 0;
    if (candidate.empty() ||
        !readVector(candidate, "yaw_offsets_deg",
                    &adaptiveCandidateOptions_.yawOffsetsDeg,
                    -90.0, 90.0) ||
        !readVector(candidate, "pitch_offsets_deg",
                    &adaptiveCandidateOptions_.pitchOffsetsDeg,
                    -90.0, 90.0) ||
        !readVector(candidate, "working_distance_scales",
                    &adaptiveCandidateOptions_.workingDistanceScales,
                    0.1, 5.0) ||
        !readVector(candidate, "speeds_mm_s",
                    &adaptiveCandidateOptions_.speedsMmS,
                    1.0, 200.0) ||
        !readVector(candidate, "exposure_scales",
                    &adaptiveExposureScales_, 0.05, 5.0) ||
        !readInt(candidate, "include_reverse_direction",
                 &includeReverse, 0, 1) ||
        !readDouble(candidate, "roi_padding_mm",
                    &adaptiveCandidateOptions_.roiPaddingMm,
                    0.0, 100.0) ||
        !readInt(candidate, "maximum_candidate_count",
                 &maximumCandidates, 1, 100000)) {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("candidate_library 配置无效");
        }
        return false;
    }
    adaptiveCandidateOptions_.includeReverseDirection =
        includeReverse != 0;
    adaptiveCandidateOptions_.maximumCandidateCount =
        static_cast<std::size_t>(maximumCandidates);

    const cv::FileNode search = storage["search"];
    int beamWidth = 0;
    int shortlistSize = 0;
    if (search.empty() ||
        !readInt(search, "horizon",
                 &adaptiveSearchOptions_.horizon, 2, 3) ||
        !readInt(search, "beam_width",
                 &beamWidth, 1, 64) ||
        !readInt(search, "shortlist_size",
                 &shortlistSize, 1, 16) ||
        !readInt(search, "maximum_candidates_per_roi",
                 &adaptiveMaximumCandidatesPerRoi_, 1, 16)) {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("search 配置无效");
        }
        return false;
    }
    adaptiveSearchOptions_.beamWidth =
        static_cast<std::size_t>(beamWidth);
    adaptiveShortlistSize_ =
        static_cast<std::size_t>(shortlistSize);

    const cv::FileNode fr5 = storage["fr5"];
    int maximumJacobianSamples = 0;
    int maximumPathSamples = 0;
    if (fr5.empty() ||
        !readDouble(fr5, "maximum_cartesian_sample_step_mm",
                    &adaptiveFr5Options_
                         .maximumCartesianSampleStepMm,
                    1.0, 100.0) ||
        !readDouble(fr5, "maximum_angular_sample_step_deg",
                    &adaptiveFr5Options_
                         .maximumAngularSampleStepDeg,
                    0.5, 45.0) ||
        !readDouble(fr5, "minimum_joint_limit_margin_deg",
                    &adaptiveFr5Options_
                         .minimumJointLimitMarginDeg,
                    0.0, 90.0) ||
        !readDouble(fr5, "minimum_normalized_singular_value",
                    &adaptiveFr5Options_
                         .minimumNormalizedSingularValue,
                    1.0e-6, 1.0) ||
        !readDouble(fr5, "jacobian_characteristic_length_mm",
                    &adaptiveFr5Options_
                         .jacobianCharacteristicLengthMm,
                    1.0, 10000.0) ||
        !readInt(fr5, "maximum_jacobian_samples",
                 &maximumJacobianSamples, 2, 128) ||
        !readInt(fr5, "maximum_path_samples",
                 &maximumPathSamples, 4, 4096)) {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("fr5 配置无效");
        }
        return false;
    }
    adaptiveFr5Options_.maximumJacobianSamples =
        static_cast<std::size_t>(maximumJacobianSamples);
    adaptiveFr5Options_.maximumPathSamples =
        static_cast<std::size_t>(maximumPathSamples);
    const cv::FileNode safety = storage["safety"];
    int collisionValidated = 0;
    int localExecutionEnabled = 0;
    std::string collisionUnavailableState;
    safety["collision_state_when_unavailable"] >>
        collisionUnavailableState;
    if (safety.empty() ||
        !readInt(safety, "collision_model_validated",
                 &collisionValidated, 0, 1) ||
        !readInt(safety, "local_adaptive_execution_enabled",
                 &localExecutionEnabled, 0, 1) ||
        collisionUnavailableState != "UNKNOWN") {
        if (error && error->isEmpty()) {
            *error = QStringLiteral("safety 配置无效");
        }
        return false;
    }
    if (collisionValidated != 0 ||
        localExecutionEnabled != 0) {
        if (error) {
            *error = QStringLiteral(
                "当前版本没有已验证碰撞场景，"
                "safety 中 collision_model_validated 和 "
                "local_adaptive_execution_enabled 必须保持 0。");
        }
        return false;
    }
    return true;
}

bool HikConstantLaserScanWindow::buildAdaptiveSerpentinePlan(
        hik_adaptive::ScanPlan* plan, QString* error) const {
    if (error) error->clear();
    if (profile_.id != QStringLiteral("scanner_650")) {
        if (error) {
            *error = QStringLiteral(
                "自适应扫描 v1 只允许 scanner_650；scanner_450 暂不加入。");
        }
        return false;
    }
    if (!startTaught_ || !endTaught_) {
        if (error) *error = QStringLiteral("请先示教第一条扫描带的起点和终点。");
        return false;
    }
    if (!adaptiveConfigReady_) {
        if (error) {
            *error = QStringLiteral("自适应配置尚未通过校验：%1")
                         .arg(adaptiveConfigPath_);
        }
        return false;
    }
    hik_adaptive::SerpentineOptions options =
        adaptiveSerpentineDefaults_;
    options.firstLaneStart = startPose_;
    options.firstLaneEnd = endPose_;
    options.laneOffsetMm = cv::Vec3d(
        adaptiveLaneOffsetXSpin_->value(),
        adaptiveLaneOffsetYSpin_->value(),
        adaptiveLaneOffsetZSpin_->value());
    options.laneCount = adaptiveLaneCountSpin_->value();
    options.measurementSpeedMmS = scanSpeedSpin_->value();
    options.transitionSpeedMmS =
        adaptiveTransitionSpeedSpin_->value();
    options.accelerationMmS2 =
        synchronizationConfig_.scanAccelerationMmS2;
    options.exposureUs = exposureSpin_->value();
    options.leadInMm = adaptiveLeadInSpin_->value();
    options.leadOutMm = adaptiveLeadOutSpin_->value();
    std::string coreError;
    if (!hik_adaptive::buildSerpentinePlan(
            options, plan, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    return true;
}

void HikConstantLaserScanWindow::generateAdaptiveGlobalPlan() {
    hik_adaptive::ScanPlan plan;
    QString error;
    if (!buildAdaptiveSerpentinePlan(&plan, &error)) {
        showError(QStringLiteral("全局蛇形计划失败"), error);
        return;
    }
    adaptiveGlobalPlan_ = plan;
    adaptiveGlobalPlanReady_ = true;
    adaptiveGlobalKinematicsPassed_ = false;
    adaptiveGlobalEvaluationPoseValid_ = false;
    adaptiveArcFallbackEvaluationActive_ = false;
    adaptiveGlobalEvaluations_.clear();
    adaptiveLocalPlan_ = hik_adaptive::PlannedActionSequence();
    const int measurementSegments =
        (adaptiveLaneCountSpin_->value());
    int arcTransitions = 0;
    int lineFallbackTransitions = 0;
    for (const hik_adaptive::ScanSegment& segment :
         plan.segments) {
        if (segment.kind !=
            hik_adaptive::SegmentKind::Transition) {
            continue;
        }
        if (segment.primitive ==
            hik_adaptive::MotionPrimitive::Arc) {
            ++arcTransitions;
        } else {
            ++lineFallbackTransitions;
        }
    }
    adaptiveStatusLabel_->setText(QStringLiteral(
        "全局 dry-run 已冻结：测量带=%1，转场=%2，总测量=%3 mm，"
        "MoveC=%3，停稳直线回退=%4，总测量=%5 mm，"
        "转场=%6 mm，梯形模型预计=%7 s。下一步必须执行 FR5 全路径评估；"
        "碰撞仍为 UNKNOWN。")
        .arg(measurementSegments)
        .arg(std::max(0, measurementSegments - 1))
        .arg(arcTransitions)
        .arg(lineFallbackTransitions)
        .arg(plan.measurementLengthMm, 0, 'f', 2)
        .arg(plan.transitionLengthMm, 0, 'f', 2)
        .arg(plan.estimatedExecutionTimeS, 0, 'f', 2));
    appendLog(adaptiveStatusLabel_->text());
    for (std::size_t index = 0U; index < plan.segments.size();
         ++index) {
        const hik_adaptive::ScanSegment& segment =
            plan.segments[index];
        appendLog(QStringLiteral(
            "自适应段[%1] id=%2 type=%3 primitive=%4，"
            "measurement=%5 → %6，motion=%7 → %8，"
            "via=%9，radius=%10 mm，blendR=%11 mm，"
            "speed=%12 mm/s，fallback=%13。")
            .arg(index)
            .arg(segment.segmentId)
            .arg(segment.kind ==
                     hik_adaptive::SegmentKind::Measurement
                 ? QStringLiteral("MEASUREMENT")
                 : QStringLiteral("TRANSITION"))
            .arg(QString::fromLatin1(
                hik_adaptive::motionPrimitiveName(
                    segment.primitive)))
            .arg(poseText(segment.start))
            .arg(poseText(segment.end))
            .arg(poseText(segment.motionStart))
            .arg(poseText(segment.motionEnd))
            .arg(poseText(segment.arcVia))
            .arg(segment.arcRadiusMm, 0, 'f', 2)
            .arg(segment.blendRadiusMm, 0, 'f', 2)
            .arg(segment.speedMmS, 0, 'f', 2)
            .arg(QString::fromStdString(
                segment.fallbackReason)));
    }
    QString artifactPath;
    if (!saveAdaptivePlanArtifact(&artifactPath, &error)) {
        appendLog(QStringLiteral("警告：无法保存自适应 dry-run：%1")
                      .arg(error));
    } else {
        appendLog(QStringLiteral("自适应 dry-run 已保存：%1")
                      .arg(artifactPath));
    }
    updateUi();
}

void HikConstantLaserScanWindow::evaluateAdaptiveGlobalPlan() {
    if (!adaptiveGlobalPlanReady_ ||
        profile_.id != QStringLiteral("scanner_650")) {
        showError(QStringLiteral("无法评估全局路径"),
                  QStringLiteral("请先生成 scanner_650 全局蛇形 dry-run。"));
        return;
    }
    if (!robotConnected_ || robotBusy_ ||
        adaptiveKinematicRequestId_ >= 0) {
        showError(QStringLiteral("无法评估全局路径"),
                  QStringLiteral("FR5 未连接、正忙或已有评估任务。"));
        return;
    }
    QString error;
    if (!robotSession_->acquireExclusive(
            robotClientId_,
            QStringLiteral("scanner_650 全局路径预评估"),
            &error)) {
        showError(QStringLiteral("FR5 评估互斥"), error);
        return;
    }
    adaptiveEvaluationLeaseHeld_ = true;
    adaptiveEvaluationMode_ = AdaptiveEvaluationMode::Global;
    adaptiveGlobalKinematicsPassed_ = false;
    adaptiveArcFallbackEvaluationActive_ = false;
    adaptiveGlobalEvaluations_.clear();
    adaptiveStatusLabel_->setText(
        QStringLiteral("正在读取 FR5 当前法兰位姿，随后逐段采样 IK/奇异性。"));
    issueRobotRead(ReadRole::AdaptiveGlobalEvaluation);
}

void HikConstantLaserScanWindow::startAdaptiveGlobalEvaluation(
        const hik_scan::Pose6D& currentPose) {
    adaptivePlanningPose_ = currentPose;
    adaptiveGlobalEvaluationPose_ = currentPose;
    adaptiveGlobalEvaluationPoseValid_ = true;
    std::vector<hik_fr5::PathEvaluationRequest> requests;
    requests.reserve(adaptiveGlobalPlan_.segments.size());
    adaptiveEvaluationTokens_.clear();
    int token = 1;
    hik_scan::Pose6D from = currentPose;
    for (std::size_t index = 0U;
         index < adaptiveGlobalPlan_.segments.size(); ++index) {
        hik_fr5::PathEvaluationRequest request;
        request.actionId = token;
        request.chainWithPrevious = index > 0U;
        request.currentPose = from;
        request.segment = adaptiveGlobalPlan_.segments[index];
        requests.push_back(request);
        adaptiveEvaluationTokens_[token] =
            std::make_pair(static_cast<int>(index), -1);
        from = request.segment.motionEnd;
        ++token;
    }
    adaptiveEvaluationExpected_ = requests.size();
    adaptiveEvaluationReceived_ = 0U;
    adaptiveKinematicRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
    appendLog(QStringLiteral(
        "提交 %1 个全局段进行 FR5 采样评估；碰撞模型缺失时 collision 保持 UNKNOWN。")
        .arg(requests.size()));
    robotSession_->evaluateKinematicPaths(
        adaptiveKinematicRequestId_, std::move(requests),
        adaptiveFr5Options_);
    updateUi();
}

void HikConstantLaserScanWindow::startAdaptiveGlobalScan() {
    QString error;
    if (!adaptiveGlobalPlanReady_ ||
        !adaptiveGlobalKinematicsPassed_) {
        showError(QStringLiteral("无法执行全局粗扫"),
                  QStringLiteral(
                      "计划尚未生成，或 FR5 逐段 IK/限位/奇异性未全部通过。"));
        return;
    }
    if (dryRunCheck_->isChecked()) {
        showError(QStringLiteral("无法执行全局粗扫"),
                  QStringLiteral("请确认 dry-run 后取消 dry-run 选项。"));
        return;
    }
    if (!safetyConfirmCheck_->isChecked()) {
        showError(QStringLiteral("真运动未授权"),
                  QStringLiteral(
                      "请确认完整蛇形路径、转场、控制器状态和物理急停。"));
        return;
    }
    if (!cameraConnected_ || !robotConnected_ ||
        !calibrationReady_ || !synchronizationConfigReady_ ||
        !laserReadyForProfile(&error)) {
        showError(QStringLiteral("设备未就绪"),
                  error.isEmpty()
                      ? QStringLiteral(
                            "请连接相机/FR5/TTL并加载 scanner_650 正式标定。")
                      : error);
        return;
    }
    if (!formalCalibrationFilesUnchanged(&error) ||
        !calibrationIdentityMatches(&error)) {
        showError(QStringLiteral("标定检查失败"), error);
        return;
    }
    synchronizationConfig_.scanSpeedMmS = scanSpeedSpin_->value();
    synchronizationConfig_.cameraExposureUs = exposureSpin_->value();
    std::string synchronizationError;
    if (!synchronizationConfig_.validate(&synchronizationError)) {
        showError(QStringLiteral("同步参数无效"),
                  QString::fromStdString(synchronizationError));
        return;
    }
    if (!adaptiveGlobalEvaluationPoseValid_) {
        showError(QStringLiteral("全局评估已失效"),
                  QStringLiteral("缺少评估时的 FR5 起始位姿，请重新评估。"));
        return;
    }
    if (!acquireScanActivity(&error)) {
        showError(QStringLiteral("设备组互斥"), error);
        return;
    }
    adaptiveExecutionPreflightPending_ = true;
    scanStatusLabel_->setText(QStringLiteral(
        "执行前重新读取 FR5 位姿，验证机器人未在路径评估后被移动。"));
    issueRobotRead(ReadRole::AdaptiveExecutionPreflight);
}

void HikConstantLaserScanWindow::
beginAdaptiveGlobalExecutionAfterPreflight(
        const PoseReading& currentPose) {
    if (!adaptiveExecutionPreflightPending_) return;
    adaptiveExecutionPreflightPending_ = false;
    const cv::Matx44d evaluated =
        hik_calibration::fairinoBaseFromFlange(
            adaptiveGlobalEvaluationPose_.x,
            adaptiveGlobalEvaluationPose_.y,
            adaptiveGlobalEvaluationPose_.z,
            adaptiveGlobalEvaluationPose_.rx,
            adaptiveGlobalEvaluationPose_.ry,
            adaptiveGlobalEvaluationPose_.rz);
    const double translationDelta =
        hik_calibration::rigidTranslationDistanceMm(
            evaluated, currentPose.baseFromFlange);
    const double rotationDelta =
        hik_calibration::rigidRotationDistanceDeg(
            evaluated, currentPose.baseFromFlange);
    if (translationDelta > 0.20 || rotationDelta > 0.10) {
        adaptiveGlobalKinematicsPassed_ = false;
        adaptiveGlobalEvaluationPoseValid_ = false;
        safetyConfirmCheck_->setChecked(false);
        releaseScanActivity();
        showError(QStringLiteral("全局路径评估已失效"),
                  QStringLiteral(
                      "FR5 在评估后发生移动：Δ=%1 mm / %2 deg。"
                      "请重新做全局 FR5 路径评估。")
                  .arg(translationDelta, 0, 'f', 4)
                  .arg(rotationDelta, 0, 'f', 4));
        return;
    }
    const double physicalLength =
        adaptiveGlobalPlan_.measurementLengthMm +
        adaptiveGlobalPlan_.transitionLengthMm +
        adaptiveLaneCountSpin_->value() *
            (adaptiveLeadInSpin_->value() +
             adaptiveLeadOutSpin_->value());
    const QString confirmation = QStringLiteral(
        "将执行 scanner_650 固定全局蛇形：%1 条测量带、%2 个转场，"
        "单一连续相机流，转场帧标记为 OUTSIDE_VALID_SCAN_SEGMENT。\n"
        "物理路径约 %3 mm，预计 %4 s。\n\n"
        "全部转场为已通过采样检查的 MoveC 时，将通过 blendR>0 "
        "整条预提交；任一转场已回退为 MoveL 时，将采用停稳逐段执行。\n\n"
        "FR5 采样 IK、关节余量和奇异性已通过；"
        "但当前没有完整机器人/扫描头/线缆/夹具/工件碰撞模型，"
        "collision=UNKNOWN。本次只允许按人工确认的固定粗扫边界执行，"
        "不会自动执行局部 NBS。\n"
        "为保留局部 NBS 所需的多路径证据，本次正式中心线将强制使用 "
        "Shadow 门控。\n\n确认完整路径无碰撞并开始？")
        .arg(adaptiveLaneCountSpin_->value())
        .arg(std::max(0, adaptiveLaneCountSpin_->value() - 1))
        .arg(physicalLength, 0, 'f', 2)
        .arg(adaptiveGlobalPlan_.estimatedExecutionTimeS, 0, 'f', 2);
    if (QMessageBox::warning(
            this, QStringLiteral("确认 scanner_650 固定全局粗扫"),
            confirmation, QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) != QMessageBox::Yes) {
        releaseScanActivity();
        return;
    }
    std::string executorError;
    if (!adaptiveExecutor_.load(
            adaptiveGlobalPlan_, &executorError)) {
        releaseScanActivity();
        showError(QStringLiteral("执行器拒绝计划"),
                  QString::fromStdString(executorError));
        return;
    }
    continuousAbortRequested_ = false;
    // Executing the adaptive global coarse scan explicitly requests the
    // evidence needed by the following local NBS stage, regardless of the
    // ordinary continuous-scan selector.
    const int adaptiveModeIndex =
        adaptiveMappingModeCombo_->findData(true);
    if (adaptiveModeIndex >= 0) {
        adaptiveMappingModeCombo_->setCurrentIndex(
            adaptiveModeIndex);
    }
    continuousAdaptiveQualityModeActive_ = true;
    synchronizationSessionDir_.clear();
    clearAdaptiveQualityArtifacts();
    adaptiveMotionRequestSegments_.clear();
    adaptiveActualSegmentTimeS_.clear();
    adaptiveGlobalPlan_.actualExecutionTimeS = 0.0;
    for (hik_adaptive::ScanSegment& segment :
         adaptiveGlobalPlan_.segments) {
        segment.actualExecutionTimeS = 0.0;
    }
    adaptiveGlobalExecutionActive_ = true;
    dispatchAdaptiveCommand(adaptiveExecutor_.begin());
}

void HikConstantLaserScanWindow::dispatchAdaptiveCommand(
        const hik_adaptive::AdaptiveScanExecutor::Command& command) {
    using CommandKind =
        hik_adaptive::AdaptiveScanExecutor::CommandKind;
    if (command.kind == CommandKind::None) return;
    if (command.kind == CommandKind::MoveToPlanStart) {
        continuousState_ = ContinuousState::MovingToStart;
        pendingMotionRequestId_ =
            robotSession_->allocateRequestId(robotClientId_);
        adaptiveMotionRequestSegments_[pendingMotionRequestId_] = -1;
        scanStatusLabel_->setText(QStringLiteral(
            "scanner_650 蛇形：移动到第一带 lead-in 起点 %1")
            .arg(poseText(command.target)));
        appendLog(scanStatusLabel_->text());
        emit requestMoveLinear(
            pendingMotionRequestId_,
            command.target.x, command.target.y, command.target.z,
            command.target.rx, command.target.ry, command.target.rz,
            velocitySpin_->value(), accelerationSpin_->value(),
            motionTimeoutSpin_->value() * 1000);
    } else if (command.kind == CommandKind::StartAcquisition) {
        QString readinessError;
        if (!laserReadyForProfile(&readinessError) ||
            !formalCalibrationFilesUnchanged(&readinessError) ||
            !calibrationIdentityMatches(&readinessError) ||
            !createSynchronizationSession(&readinessError)) {
            dispatchAdaptiveCommand(adaptiveExecutor_.requestStop(
                localPath(QStringLiteral(
                    "自适应连续会话启动失败：%1").arg(readinessError))));
            showError(QStringLiteral("自适应连续会话启动失败"),
                      readinessError);
            return;
        }
        continuousState_ = ContinuousState::StartingCamera;
        scanStatusLabel_->setText(QStringLiteral(
            "蛇形起点已到达，正在启动一次连续相机流。"));
        emit requestStartContinuous(
            synchronizationConfig_.cameraExposureUs,
            gainSpin_->value(),
            synchronizationConfig_.cameraTargetFps,
            static_cast<int>(
                synchronizationConfig_.cameraQueueCapacity));
    } else if (command.kind == CommandKind::ExecuteTrajectory) {
        std::vector<hik_sync::MeasurementSegmentGate> gates;
        for (const hik_adaptive::ScanSegment& segment :
             command.trajectory) {
            if (segment.kind !=
                hik_adaptive::SegmentKind::Measurement) {
                continue;
            }
            hik_sync::MeasurementSegmentGate gate;
            gate.segmentId = segment.segmentId;
            gate.measurementStartMm = Eigen::Vector3d(
                segment.start.x, segment.start.y, segment.start.z);
            gate.measurementEndMm = Eigen::Vector3d(
                segment.end.x, segment.end.y, segment.end.z);
            gate.lateralToleranceMm = std::max(
                1.0, 0.5 * adaptiveQualityVoxelSpin_->value());
            gates.push_back(gate);
        }
        std::string gateError;
        if (!synchronizationSession_.configureMeasurementSegments(
                gates, &gateError)) {
            abortContinuousScan(
                QStringLiteral("多测量带门控失败：%1")
                    .arg(QString::fromStdString(gateError)),
                false);
            return;
        }
        continuousState_ = ContinuousState::Scanning;
        pendingMotionRequestId_ =
            robotSession_->allocateRequestId(robotClientId_);
        adaptiveMotionRequestSegments_[pendingMotionRequestId_] = -2;
        scanStatusLabel_->setText(QStringLiteral(
            "FR5 已开始预提交融合蛇形：%1 段 LINE/MoveC，"
            "%2 条直线有效测量带；圆弧和 lead-in/out 由空间门控拒绝。")
            .arg(command.trajectory.size())
            .arg(gates.size()));
        appendLog(scanStatusLabel_->text());
        emit requestExecuteAdaptiveTrajectory(
            pendingMotionRequestId_, command.trajectory,
            motionTimeoutSpin_->value() * 1000);
    } else if (command.kind == CommandKind::ExecuteSegment) {
        const hik_adaptive::ScanSegment& segment =
            command.segment;
        if (command.gate ==
            hik_adaptive::AdaptiveScanExecutor::MeasurementGate::
                Configure) {
            std::string gateError;
            if (!synchronizationSession_.configureMeasurementSegment(
                    segment.segmentId,
                    Eigen::Vector3d(
                        segment.start.x,
                        segment.start.y,
                        segment.start.z),
                    Eigen::Vector3d(
                        segment.end.x,
                        segment.end.y,
                        segment.end.z),
                    std::max(1.0,
                        0.5 * adaptiveQualityVoxelSpin_->value()),
                    &gateError)) {
                abortContinuousScan(
                    QStringLiteral("有效测量段门控失败：%1")
                        .arg(QString::fromStdString(gateError)),
                    true);
                return;
            }
        } else if (command.gate ==
                   hik_adaptive::AdaptiveScanExecutor::MeasurementGate::
                       Suspend) {
            synchronizationSession_.suspendMeasurementSegment();
        }
        continuousState_ = ContinuousState::Scanning;
        pendingMotionRequestId_ =
            robotSession_->allocateRequestId(robotClientId_);
        adaptiveMotionRequestSegments_[pendingMotionRequestId_] =
            segment.segmentId;
        const bool measurement =
            segment.kind == hik_adaptive::SegmentKind::Measurement;
        scanStatusLabel_->setText(QStringLiteral(
            "蛇形段 %1/%2：%3，segment_id=%4，speed=%5 mm/s。")
            .arg(command.segmentIndex + 1U)
            .arg(adaptiveGlobalPlan_.segments.size())
            .arg(measurement
                 ? QStringLiteral("有效测量")
                 : QStringLiteral("转场（不发布点云）"))
            .arg(segment.segmentId)
            .arg(segment.speedMmS, 0, 'f', 2));
        appendLog(scanStatusLabel_->text());
        emit requestMoveLinearPhysical(
            pendingMotionRequestId_,
            command.target.x, command.target.y, command.target.z,
            command.target.rx, command.target.ry, command.target.rz,
            segment.speedMmS, segment.accelerationMmS2,
            motionTimeoutSpin_->value() * 1000);
    } else if (command.kind == CommandKind::StopAcquisition) {
        synchronizationSession_.suspendMeasurementSegment();
        requestLaserOff();
        continuousAbortRequested_ = !command.completed;
        continuousState_ = ContinuousState::Stopping;
        emit requestStopContinuous();
        scanStatusLabel_->setText(command.completed
            ? QStringLiteral(
                  "全部蛇形测量带完成，正在停止相机并清空队列。")
            : QStringLiteral(
                  "蛇形扫描异常，正在停止相机并保存已采数据。"));
    } else if (command.kind == CommandKind::Finalize) {
        finalizeContinuousScan(
            command.completed,
            QString::fromStdString(command.detail));
    }
    updateUi();
}

bool HikConstantLaserScanWindow::buildAdaptiveQualityAndCandidates(
        QString* error) {
    if (error) error->clear();
    adaptiveQualityMap_ = hik_adaptive::QualityMap();
    adaptiveRois_.clear();
    adaptiveCandidates_.clear();
    adaptiveLocalPlan_ = hik_adaptive::PlannedActionSequence();
    if (profile_.id != QStringLiteral("scanner_650")) {
        if (error) *error = QStringLiteral("自适应质量地图仅属于 scanner_650。");
        return false;
    }
    std::map<int, cv::Point3d> origins;
    for (const hik_scan::ContinuousFrameViewpoint& viewpoint :
         lastContinuousArtifacts_.viewpoints) {
        origins[viewpoint.profileIndex] =
            viewpoint.cameraOriginBaseMm;
    }
    std::vector<hik_adaptive::QualityObservation> observations;
    observations.reserve(
        lastContinuousArtifacts_.formal.size() +
        lastContinuousArtifacts_.qualityAccepted.size() +
        lastContinuousArtifacts_.rejected.size());
    const auto appendObservations =
        [&observations, &origins](
            const std::vector<hik_scan::CloudPoint>& points,
            hik_adaptive::ObservationRole role) {
            for (const hik_scan::CloudPoint& point : points) {
                hik_adaptive::QualityObservation observation;
                observation.point = point;
                observation.role = role;
                observation.frameId =
                    point.profileIndex >= 0
                    ? static_cast<std::uint64_t>(point.profileIndex)
                    : 0U;
                const auto found = origins.find(point.profileIndex);
                if (found != origins.end()) {
                    observation.cameraOriginBaseMm = found->second;
                    observation.hasCameraOrigin = true;
                }
                observations.push_back(std::move(observation));
            }
        };
    appendObservations(
        lastContinuousArtifacts_.formal,
        hik_adaptive::ObservationRole::FormalAccepted);
    appendObservations(
        lastContinuousArtifacts_.qualityAccepted,
        hik_adaptive::ObservationRole::QualityAccepted);
    appendObservations(
        lastContinuousArtifacts_.rejected,
        hik_adaptive::ObservationRole::Rejected);
    if (observations.empty()) {
        if (error) *error = QStringLiteral("连续粗扫没有可用于质量地图的空间证据。");
        return false;
    }

    hik_adaptive::QualityMapOptions mapOptions =
        adaptiveQualityMapOptions_;
    mapOptions.voxelSizeMm = adaptiveQualityVoxelSpin_->value();
    // Old formal points without point-local optical metrics remain UNKNOWN,
    // but scanner_650 Shadow formal geometry must not be rejected solely for
    // lacking the parallel quality schema.
    mapOptions.rescanUnknownOpticalEvidence = false;
    std::string coreError;
    if (!hik_adaptive::buildQualityMap(
            observations,
            std::vector<cv::Point3d>(),
            mapOptions, &adaptiveQualityMap_, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    hik_adaptive::RoiClusteringOptions roiOptions =
        adaptiveRoiOptions_;
    if (!hik_adaptive::clusterRescanRois(
            adaptiveQualityMap_, roiOptions,
            &adaptiveRois_, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    if (adaptiveRois_.empty()) return true;

    std::vector<hik_adaptive::RescanRoi> candidateRois =
        adaptiveRois_;
    std::stable_sort(
        candidateRois.begin(), candidateRois.end(),
        [](const hik_adaptive::RescanRoi& first,
           const hik_adaptive::RescanRoi& second) {
            if (first.severity != second.severity) {
                return first.severity > second.severity;
            }
            return first.roiId < second.roiId;
        });
    if (candidateRois.size() >
        adaptiveMaximumCandidateRois_) {
        candidateRois.resize(adaptiveMaximumCandidateRois_);
    }

    hik_adaptive::CandidateGenerationContext context;
    const hik_scan::Pose6D referencePose =
        adaptiveGlobalPlan_.segments.empty()
        ? startPose_
        : adaptiveGlobalPlan_.segments.front().start;
    context.baseFromReferenceFlange =
        hik_calibration::fairinoBaseFromFlange(
            referencePose.x, referencePose.y, referencePose.z,
            referencePose.rx, referencePose.ry, referencePose.rz);
    context.flangeFromCamera = handEye_.flangeFromCamera;
    context.nominalScanDirectionBase = cv::Vec3d(
        endPose_.x - startPose_.x,
        endPose_.y - startPose_.y,
        endPose_.z - startPose_.z);
    context.cameraForwardAxis = cv::Vec3d(0.0, 0.0, 1.0);
    if (intrinsics_.cameraMatrix.rows == 3 &&
        intrinsics_.cameraMatrix.cols == 3 &&
        intrinsics_.cameraMatrix.type() == CV_64F) {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                context.cameraMatrix(row, column) =
                    intrinsics_.cameraMatrix.at<double>(row, column);
            }
        }
    } else {
        if (error) *error = QStringLiteral("scanner_650 内参矩阵格式无效。");
        return false;
    }
    context.calibratedImageSize = intrinsics_.imageSize;
    context.softwareRoi =
        profileOptions_.reconstruction.stripe.quality.roi;
    context.laserPlaneNormalCamera = laserPlane_.plane.normal;
    context.laserPlaneDMm = laserPlane_.plane.dMm;
    context.validDepthMinimumMm =
        profileOptions_.reconstruction.minimumDepthMm;
    context.validDepthMaximumMm =
        profileOptions_.reconstruction.maximumDepthMm;
    context.laserPlaneToleranceMm =
        std::max(1.0, adaptiveQualityVoxelSpin_->value());
    context.calibratedObservabilityAvailable = true;
    // No mesh/depth model is currently available for ray occlusion.
    context.occlusionModelAvailable = false;

    hik_adaptive::CandidateLibraryOptions candidateOptions =
        adaptiveCandidateOptions_;
    candidateOptions.exposureUs.clear();
    for (double scale : adaptiveExposureScales_) {
        candidateOptions.exposureUs.push_back(
            std::max(1.0, scale * exposureSpin_->value()));
    }
    candidateOptions.accelerationMmS2 =
        synchronizationConfig_.scanAccelerationMmS2;
    if (!hik_adaptive::generateCandidateLibrary(
            adaptiveQualityMap_, candidateRois, context,
            candidateOptions, &adaptiveCandidates_, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    return true;
}

void HikConstantLaserScanWindow::planAdaptiveLocalRescan() {
    QString error;
    if (adaptiveCandidates_.empty() &&
        !buildAdaptiveQualityAndCandidates(&error)) {
        showError(QStringLiteral("局部补扫规划失败"), error);
        return;
    }
    if (adaptiveRois_.empty() || adaptiveCandidates_.empty()) {
        adaptiveStatusLabel_->setText(QStringLiteral(
            "当前质量地图没有聚类出需要局部补扫的 ROI。"));
        appendLog(adaptiveStatusLabel_->text());
        return;
    }
    if (!robotConnected_ || robotBusy_ ||
        adaptiveKinematicRequestId_ >= 0) {
        showError(QStringLiteral("局部补扫评估不可用"),
                  QStringLiteral("FR5 未连接、正忙或已有评估任务。"));
        return;
    }
    if (!robotSession_->acquireExclusive(
            robotClientId_,
            QStringLiteral("scanner_650 局部 NBS 预评估"),
            &error)) {
        showError(QStringLiteral("FR5 评估互斥"), error);
        return;
    }
    adaptiveEvaluationLeaseHeld_ = true;
    adaptiveEvaluationMode_ = AdaptiveEvaluationMode::Local;
    adaptiveInitialEvaluations_.clear();
    adaptivePairEvaluations_.clear();
    adaptiveStatusLabel_->setText(QStringLiteral(
        "正在读取 FR5 当前法兰位姿，随后评估有限候选及候选间转场。"));
    issueRobotRead(ReadRole::AdaptiveLocalEvaluation);
}

void HikConstantLaserScanWindow::startAdaptiveLocalEvaluation(
        const hik_scan::Pose6D& currentPose) {
    adaptivePlanningPose_ = currentPose;
    hik_adaptive::SearchOptions preRank =
        adaptiveSearchOptions_;
    preRank.horizon = adaptiveBeamHorizonSpin_->value();
    preRank.beamWidth =
        static_cast<std::size_t>(adaptiveBeamWidthSpin_->value());
    preRank.allowUnverifiedForDryRun = true;
    std::string coreError;
    if (!hik_adaptive::evaluateAndRankCandidates(
            currentPose, preRank,
            hik_adaptive::CandidateSafetyEvaluator(),
            &adaptiveCandidates_, &coreError)) {
        if (adaptiveEvaluationLeaseHeld_) {
            robotSession_->releaseExclusive(robotClientId_);
            adaptiveEvaluationLeaseHeld_ = false;
        }
        adaptiveEvaluationMode_ = AdaptiveEvaluationMode::None;
        showError(QStringLiteral("候选预排序失败"),
                  QString::fromStdString(coreError));
        return;
    }

    std::vector<hik_adaptive::CandidateAction> shortlist;
    std::map<int, int> perRoi;
    const std::size_t maximumShortlist =
        adaptiveShortlistSize_;
    const int maximumPerRoi =
        adaptiveMaximumCandidatesPerRoi_;
    for (const hik_adaptive::CandidateAction& candidate :
         adaptiveCandidates_) {
        if (shortlist.size() >= maximumShortlist) break;
        if (perRoi[candidate.roiId] >= maximumPerRoi) continue;
        shortlist.push_back(candidate);
        ++perRoi[candidate.roiId];
    }
    adaptiveCandidates_ = std::move(shortlist);
    if (adaptiveCandidates_.empty()) {
        if (adaptiveEvaluationLeaseHeld_) {
            robotSession_->releaseExclusive(robotClientId_);
            adaptiveEvaluationLeaseHeld_ = false;
        }
        adaptiveEvaluationMode_ = AdaptiveEvaluationMode::None;
        showError(QStringLiteral("候选评估失败"),
                  QStringLiteral("FOV/激光扫掠预筛选后没有候选。"));
        return;
    }

    std::vector<hik_fr5::PathEvaluationRequest> requests;
    adaptiveEvaluationTokens_.clear();
    int token = 1;
    for (const hik_adaptive::CandidateAction& target :
         adaptiveCandidates_) {
        hik_fr5::PathEvaluationRequest request;
        request.actionId = token;
        request.currentPose = currentPose;
        request.segment = target.measurement;
        requests.push_back(request);
        adaptiveEvaluationTokens_[token] =
            std::make_pair(-1, target.actionId);
        ++token;
    }
    for (const hik_adaptive::CandidateAction& from :
         adaptiveCandidates_) {
        for (const hik_adaptive::CandidateAction& target :
             adaptiveCandidates_) {
            if (from.actionId == target.actionId ||
                from.roiId == target.roiId) {
                continue;
            }
            hik_fr5::PathEvaluationRequest request;
            request.actionId = token;
            request.currentPose = from.measurement.motionEnd;
            request.segment = target.measurement;
            requests.push_back(request);
            adaptiveEvaluationTokens_[token] =
                std::make_pair(from.actionId, target.actionId);
            ++token;
        }
    }
    adaptiveEvaluationExpected_ = requests.size();
    adaptiveEvaluationReceived_ = 0U;
    adaptiveKinematicRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
    appendLog(QStringLiteral(
        "局部 NBS 有限库：短名单=%1，提交初始/候选间路径=%2；"
        "beam horizon=%3，width=%4。")
        .arg(adaptiveCandidates_.size())
        .arg(requests.size())
        .arg(adaptiveBeamHorizonSpin_->value())
        .arg(adaptiveBeamWidthSpin_->value()));
    robotSession_->evaluateKinematicPaths(
        adaptiveKinematicRequestId_, std::move(requests),
        adaptiveFr5Options_);
    updateUi();
}

void HikConstantLaserScanWindow::finishAdaptiveLocalPlanning() {
    for (hik_adaptive::CandidateAction& candidate :
         adaptiveCandidates_) {
        const auto found =
            adaptiveInitialEvaluations_.find(candidate.actionId);
        if (found != adaptiveInitialEvaluations_.end()) {
            candidate.robot = found->second;
        }
    }
    hik_adaptive::SearchOptions options =
        adaptiveSearchOptions_;
    options.horizon = adaptiveBeamHorizonSpin_->value();
    options.beamWidth =
        static_cast<std::size_t>(adaptiveBeamWidthSpin_->value());
    options.allowUnverifiedForDryRun = true;
    options.requireFullyVerifiedForExecution = true;
    const auto evaluator =
        [this](const hik_scan::Pose6D& from,
               const hik_adaptive::ScanSegment& target) {
            int targetId = target.segmentId;
            int fromId = -1;
            for (const hik_adaptive::CandidateAction& candidate :
                 adaptiveCandidates_) {
                const hik_scan::Pose6D& end =
                    candidate.measurement.motionEnd;
                const double translation = cv::norm(cv::Vec3d(
                    end.x - from.x, end.y - from.y, end.z - from.z));
                const double rotation =
                    std::abs(end.rx - from.rx) +
                    std::abs(end.ry - from.ry) +
                    std::abs(end.rz - from.rz);
                if (translation < 1.0e-4 && rotation < 1.0e-4) {
                    fromId = candidate.actionId;
                    break;
                }
            }
            if (fromId < 0) {
                const auto initial =
                    adaptiveInitialEvaluations_.find(targetId);
                if (initial != adaptiveInitialEvaluations_.end()) {
                    return initial->second;
                }
            } else {
                const auto pair = adaptivePairEvaluations_.find(
                    std::make_pair(fromId, targetId));
                if (pair != adaptivePairEvaluations_.end()) {
                    return pair->second;
                }
            }
            return hik_adaptive::RobotPathEvaluation();
        };
    std::string coreError;
    if (!hik_adaptive::evaluateAndRankCandidates(
            adaptivePlanningPose_, options, evaluator,
            &adaptiveCandidates_, &coreError)) {
        showError(QStringLiteral("局部候选排序失败"),
                  QString::fromStdString(coreError));
        return;
    }
    hik_adaptive::CandidateAction greedy;
    const bool greedyOk =
        hik_adaptive::selectGreedyAction(
            adaptiveCandidates_, options, &greedy, &coreError);
    if (!hik_adaptive::beamSearchActions(
            adaptivePlanningPose_, adaptiveCandidates_, options,
            evaluator, &adaptiveLocalPlan_, &coreError)) {
        showError(QStringLiteral("局部 beam search 失败"),
                  QString::fromStdString(coreError));
        return;
    }
    QStringList actionIds;
    for (const hik_adaptive::CandidateAction& action :
         adaptiveLocalPlan_.actions) {
        actionIds.push_back(QString::number(action.actionId));
    }
    adaptiveStatusLabel_->setText(QStringLiteral(
        "局部规划完成：greedy=%1；beam=[%2]，预计=%3 s，"
        "fully_verified=%4，executable=%5。由于 collision/occlusion="
        "UNKNOWN，当前结果强制为 dry-run，不显示自动执行入口。")
        .arg(greedyOk ? QString::number(greedy.actionId)
                      : QStringLiteral("无"))
        .arg(actionIds.join(QStringLiteral(",")))
        .arg(adaptiveLocalPlan_.totalEstimatedTimeS, 0, 'f', 2)
        .arg(adaptiveLocalPlan_.fullyVerified
             ? QStringLiteral("true") : QStringLiteral("false"))
        .arg(adaptiveLocalPlan_.executable
             ? QStringLiteral("true") : QStringLiteral("false")));
    appendLog(adaptiveStatusLabel_->text());
    QString outputPath;
    QString saveError;
    if (!saveAdaptivePlanArtifact(&outputPath, &saveError)) {
        appendLog(QStringLiteral("警告：局部计划保存失败：%1")
                      .arg(saveError));
    } else {
        appendLog(QStringLiteral("局部 dry-run 计划已保存：%1")
                      .arg(outputPath));
    }
}

bool HikConstantLaserScanWindow::saveAdaptivePlanArtifact(
        QString* outputPath, QString* error) const {
    if (error) error->clear();
    QString directory = synchronizationSessionDir_;
    if (directory.isEmpty()) {
        directory = QDir(sourceDir_).absoluteFilePath(
            QStringLiteral("data/scan/scanner_650/adaptive_plans"));
    }
    if (!QDir().mkpath(directory)) {
        if (error) *error = QStringLiteral("无法创建目录：%1").arg(directory);
        return false;
    }
    const QString fileName = synchronizationSessionDir_.isEmpty()
        ? QStringLiteral("adaptive_plan_%1.json")
              .arg(QDateTime::currentDateTime()
                       .toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")))
        : QStringLiteral("adaptive_scan_plan.json");
    const QString path = QDir(directory).absoluteFilePath(fileName);
    std::string coreError;
    if (!hik_adaptive::saveAdaptivePlanJson(
            localPath(path), adaptiveGlobalPlan_,
            adaptiveQualityMap_, adaptiveRois_,
            adaptiveLocalPlan_, &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    if (outputPath) *outputPath = path;
    return true;
}

bool HikConstantLaserScanWindow::adaptiveQualityMappingSelected() const {
    return profile_.id == QStringLiteral("scanner_650") &&
           adaptiveMappingModeCombo_ &&
           adaptiveMappingModeCombo_->currentData().toBool();
}

LineLaserCenterlinePolicy
HikConstantLaserScanWindow::selectedScanCenterlinePolicy() const {
    if (profile_.id != QStringLiteral("scanner_650") ||
        !centerlinePolicyCombo_) {
        return profile_.scanCenterlinePolicy;
    }
    const int selected = centerlinePolicyCombo_->currentData().toInt();
    switch (static_cast<LineLaserCenterlinePolicy>(selected)) {
    case LineLaserCenterlinePolicy::Legacy:
        return LineLaserCenterlinePolicy::Legacy;
    case LineLaserCenterlinePolicy::Shadow:
        return LineLaserCenterlinePolicy::Shadow;
    case LineLaserCenterlinePolicy::Quality:
        break;
    }
    return profile_.scanCenterlinePolicy;
}

void HikConstantLaserScanWindow::applySelectedScanCenterlinePolicy(
        bool announce) {
    const hik_calibration::StripeExtractionMode previous =
        profileOptions_.reconstruction.stripe.mode;
    const LineLaserCenterlinePolicy policy =
        selectedScanCenterlinePolicy();
    profileOptions_.reconstruction.stripe.mode =
        stripeMode(policy);
    if (previous != profileOptions_.reconstruction.stripe.mode) {
        clearAdaptiveQualityArtifacts();
    }
    if (!announce) {
        return;
    }
    if (policy == LineLaserCenterlinePolicy::Legacy) {
        appendLog(QStringLiteral(
            "正式中心线策略=Legacy：保留全部 Legacy 中心线，"
            "不执行多路径歧义区硬遮罩；连续性优先。"));
    } else if (policy == LineLaserCenterlinePolicy::Shadow) {
        appendLog(QStringLiteral(
            "正式中心线策略=Shadow：坐标仍采用 Legacy，"
            "但会硬屏蔽质量算法判定的多路径歧义区；反射安全优先。"));
    }
}

void HikConstantLaserScanWindow::clearAdaptiveQualityArtifacts() {
    lastContinuousArtifacts_ =
        hik_scan::ContinuousReconstructionArtifacts();
    adaptiveQualityMap_ = hik_adaptive::QualityMap();
    adaptiveRois_.clear();
    adaptiveCandidates_.clear();
    adaptiveLocalPlan_ =
        hik_adaptive::PlannedActionSequence();
}

void HikConstantLaserScanWindow::startContinuousScan() {
    QString error;
    applySelectedScanCenterlinePolicy(false);
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
        "图像仍保存但标为 SPEED_NOT_STABLE。\n路径长度=%6 mm。\n"
        "正式中心线策略=%7。\n\n确认完整路径安全并开始？")
        .arg(synchronizationConfig_.cameraTargetFps, 0, 'f', 3)
        .arg(synchronizationConfig_.scanSpeedMmS, 0, 'f', 3)
        .arg(synchronizationConfig_.scanAccelerationMmS2, 0, 'f', 3)
        .arg(velocitySpin_->value(), 0, 'f', 2)
        .arg(synchronizationConfig_.scanSpeedTolerancePercent, 0, 'f', 2)
        .arg(pathLength, 0, 'f', 3)
        .arg(lineLaserCenterlinePolicyName(
            selectedScanCenterlinePolicy()));
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
    continuousAdaptiveQualityModeActive_ =
        adaptiveQualityMappingSelected();
    if (profile_.id == QStringLiteral("scanner_650") &&
        !continuousAdaptiveQualityModeActive_) {
        clearAdaptiveQualityArtifacts();
        appendLog(QStringLiteral(
            "scanner_650 普通快速建图：本次只计算 raw/voxel，"
            "跳过质量候选、V槽时序验证和相邻轮廓过滤；"
            "本次结果不能直接用于局部 NBS。"));
    }
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
    if (!applyConfiguredReconstructionDepthRange(error)) {
        return false;
    }
    updateCalibrationStatusText();
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
    reconstructionOptions.binaryPly = binaryPlyCheck_->isChecked();
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
    const bool retainAdaptiveQualityArtifacts =
        qualityCenterlineEnabled &&
        profile_.id == QStringLiteral("scanner_650") &&
        continuousAdaptiveQualityModeActive_;
    reconstructionOptions.saveQualityCloud = false;
    reconstructionOptions.retainQualityArtifacts =
        retainAdaptiveQualityArtifacts;
    reconstructionOptions.enableVGrooveTemporalValidation =
        retainAdaptiveQualityArtifacts;
    reconstructionOptions.enableAdjacentProfileSupport =
        retainAdaptiveQualityArtifacts;
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
    appendLog(QStringLiteral(
        "%1 连续扫描采用精简 PLY 输出：只生成 continuous_raw.ply 和 "
        "continuous_voxel.ply（按 base_link 世界 Z 高度自动 Turbo 着色），"
        "编码=%2；正式中心线=%3%4。")
        .arg(
            profile_.id,
            reconstructionOptions.binaryPly
                ? QStringLiteral("binary_little_endian")
                : QStringLiteral("ascii"),
            lineLaserCenterlinePolicyName(
                selectedScanCenterlinePolicy()),
            retainAdaptiveQualityArtifacts
                ? QStringLiteral(
                      "，质量 accepted/rejected 仅在内存中保留给自适应规划")
                : qualityCenterlineEnabled
                    ? QStringLiteral(
                          "，跳过附加质量点云、邻帧分类和 rejected 收集")
                    : QStringLiteral(
                          "，不运行 Quality 多路径分析或 Shadow 硬遮罩")));
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
    if (retainAdaptiveQualityArtifacts) {
        appendLog(QStringLiteral(
            "自适应质量证据已开启：二维硬门控后保留 optical，"
            "再以 radius=%1 mm、min_profiles=%2、max_gap=%3 做相邻 "
            "profile 支持；不会额外写质量 PLY。")
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
    if (adaptiveGlobalExecutionActive_) {
        dispatchAdaptiveCommand(
            adaptiveExecutor_.acquisitionStarted(
                true, "continuous camera started"));
        return;
    }
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
    if (adaptiveGlobalExecutionActive_) {
        if (!confirmed) {
            terminalCameraFault_ = true;
            terminalCameraFaultDetail_ = description;
        }
        dispatchAdaptiveCommand(
            adaptiveExecutor_.acquisitionStopped(
                confirmed, localPath(description)));
        return;
    }
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
    if (continuousState_ == ContinuousState::Finalizing) {
        appendLog(QStringLiteral(
            "点云已进入后台收尾，忽略运动/采集终止请求：%1；"
            "当前没有活动相机流或扫描运动。")
            .arg(reason));
        return;
    }
    if (adaptiveGlobalExecutionActive_) {
        requestLaserOff();
        appendLog(QStringLiteral(
            "scanner_650 蛇形终止请求：%1").arg(reason));
        continuousAbortRequested_ = true;
        if (pendingMotionRequestId_ >= 0 &&
            requestStop && robotConnected_) {
            emit requestStopMotion(
                robotSession_->allocateRequestId(robotClientId_));
        }
        dispatchAdaptiveCommand(
            adaptiveExecutor_.requestStop(localPath(reason)));
        return;
    }
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
    if (continuousFinalizationActive_) {
        appendLog(QStringLiteral(
            "忽略重复的连续点云收尾请求：后台任务仍在运行。"));
        return;
    }
    requestLaserOff();
    const bool sessionStarted = synchronizationSession_.running();
    if (sessionStarted) synchronizationSession_.stop();
    continuousFinalizationContext_ = ContinuousFinalizationContext();
    continuousFinalizationContext_.completed = completed;
    continuousFinalizationContext_.reason = reason;
    continuousFinalizationContext_.sessionStarted = sessionStarted;
    continuousFinalizationContext_.synchronizationStatistics = sessionStarted
        ? synchronizationSession_.statistics() : hik_sync::PipelineStatistics{};
    continuousFinalizationContext_.synchronizationMode = sessionStarted
        ? QString::fromStdString(synchronizationSession_.clockModeDescription())
        : QStringLiteral("同步会话未启动");
    continuousFinalizationContext_.reconstructionWasRunning =
        continuousReconstruction_.running();
    synchronizationSession_.clearMeasurementSegmentGate();
    safetyConfirmCheck_->setChecked(false);
    continuousFinalizationResult_ = ContinuousFinalizationResult();

    if (!continuousFinalizationContext_.reconstructionWasRunning) {
        completeContinuousFinalization();
        return;
    }

    continuousState_ = ContinuousState::Finalizing;
    continuousFinalizationActive_ = true;
    continuousFinalizeProgressLabel_->setVisible(true);
    continuousFinalizeProgress_->setVisible(true);
    continuousFinalizeProgress_->setValue(0);
    updateContinuousFinalizationProgress(
        0, QStringLiteral("准备后台点云收尾"));
    appendLog(QStringLiteral(
        "相机/同步队列已停止；点云计算和保存转入独立后台线程，"
        "GUI 将保持响应，新扫描在收尾完成前锁定。"));
    updateUi();

    if (continuousFinalizationThread_.joinable()) {
        continuousFinalizationThread_.join();
    }
    const bool retainArtifacts =
        profile_.id == QStringLiteral("scanner_650") &&
        continuousAdaptiveQualityModeActive_;
    try {
        continuousFinalizationThread_ = std::thread(
            [this, retainArtifacts]() {
                const hik_scan::ContinuousReconstructionProgressCallback
                    progress =
                        [this](int percent,
                               const std::string& stage) {
                            const QString message =
                                QString::fromStdString(stage);
                            QMetaObject::invokeMethod(
                                this,
                                [this, percent, message]() {
                                    updateContinuousFinalizationProgress(
                                        percent, message);
                                },
                                Qt::QueuedConnection);
                        };
                try {
                    continuousFinalizationResult_.cloudSaved =
                        continuousReconstruction_.stopAndSave(
                            &continuousFinalizationResult_.statistics,
                            &continuousFinalizationResult_.error,
                            retainArtifacts
                                ? &continuousFinalizationResult_.artifacts
                                : nullptr,
                            progress);
                } catch (const std::exception& exception) {
                    continuousFinalizationResult_.cloudSaved = false;
                    continuousFinalizationResult_.error =
                        std::string(
                            "continuous finalization exception: ") +
                        exception.what();
                } catch (...) {
                    continuousFinalizationResult_.cloudSaved = false;
                    continuousFinalizationResult_.error =
                        "continuous finalization unknown exception";
                }
                QMetaObject::invokeMethod(
                    this, "onContinuousFinalizationFinished",
                    Qt::QueuedConnection);
            });
    } catch (const std::exception& exception) {
        continuousFinalizationActive_ = false;
        appendLog(QStringLiteral(
            "警告：无法启动点云后台线程（%1），为避免丢失数据将同步完成保存。")
            .arg(QString::fromLocal8Bit(exception.what())));
        continuousFinalizationResult_.cloudSaved =
            continuousReconstruction_.stopAndSave(
                &continuousFinalizationResult_.statistics,
                &continuousFinalizationResult_.error,
                retainArtifacts
                    ? &continuousFinalizationResult_.artifacts
                    : nullptr);
        completeContinuousFinalization();
    }
}

void HikConstantLaserScanWindow::updateContinuousFinalizationProgress(
        int percent, const QString& stage) {
    if (!continuousFinalizationActive_ &&
        percent < 100) {
        return;
    }
    const int boundedPercent = std::max(0, std::min(100, percent));
    continuousFinalizeProgress_->setValue(boundedPercent);
    continuousFinalizeProgressLabel_->setText(
        QStringLiteral("点云后台收尾：%1").arg(stage));
    scanStatusLabel_->setText(QStringLiteral(
        "点云后台收尾 %1%：%2；GUI 可继续查看日志，请勿开始新扫描。")
        .arg(boundedPercent)
        .arg(stage));
}

void HikConstantLaserScanWindow::onContinuousFinalizationFinished() {
    if (continuousFinalizationThread_.joinable()) {
        continuousFinalizationThread_.join();
    }
    if (!continuousFinalizationActive_) return;
    continuousFinalizationActive_ = false;
    updateContinuousFinalizationProgress(
        100, QStringLiteral("计算与保存完成"));
    completeContinuousFinalization();
}

void HikConstantLaserScanWindow::completeContinuousFinalization() {
    const bool completed =
        continuousFinalizationContext_.completed;
    const QString reason =
        continuousFinalizationContext_.reason;
    const bool sessionStarted =
        continuousFinalizationContext_.sessionStarted;
    const hik_sync::PipelineStatistics& stats =
        continuousFinalizationContext_.synchronizationStatistics;
    const QString& mode =
        continuousFinalizationContext_.synchronizationMode;
    const bool reconstructionWasRunning =
        continuousFinalizationContext_.reconstructionWasRunning;
    hik_scan::ContinuousReconstructionStatistics& reconstructionStats =
        continuousFinalizationResult_.statistics;
    std::string& reconstructionError =
        continuousFinalizationResult_.error;
    const bool cloudSaved =
        continuousFinalizationResult_.cloudSaved;
    hik_scan::ContinuousReconstructionArtifacts& artifacts =
        continuousFinalizationResult_.artifacts;

    continuousState_ = ContinuousState::Idle;
    safetyConfirmCheck_->setChecked(false);
    continuousFinalizeProgressLabel_->setVisible(false);
    continuousFinalizeProgress_->setVisible(false);
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
            "raw_points=%12，voxel_points=%13；raw=%14；"
            "voxel=%15（base_link 世界 Z 高度 Turbo 着色）。")
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
        if (reconstructionStats.qualityOpticalPlySaved ||
            reconstructionStats.qualityPlySaved ||
            reconstructionStats.qualityRejectedPlySaved ||
            reconstructionStats.qualityVoxelPlySaved) {
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
            if (selectedScanCenterlinePolicy() ==
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
    double adaptiveActualMotionTimeS = 0.0;
    for (const auto& entry : adaptiveActualSegmentTimeS_) {
        adaptiveActualMotionTimeS += entry.second;
    }
    terminalStats.insert(
        QStringLiteral("adaptive_actual_motion_time_s"),
        adaptiveActualMotionTimeS);
    terminalStats.insert(
        QStringLiteral("adaptive_timed_segment_count"),
        static_cast<double>(adaptiveActualSegmentTimeS_.size()));

    const bool finalCompleted =
        completed && (!reconstructionWasRunning || cloudSaved);
    QString finalReason = reason;
    if (completed && reconstructionWasRunning && !cloudSaved) {
        finalReason += QStringLiteral("；连续点云保存失败：%1")
            .arg(QString::fromStdString(reconstructionError));
    }
    if (profile_.id == QStringLiteral("scanner_650") &&
        continuousAdaptiveQualityModeActive_ &&
        reconstructionWasRunning && cloudSaved) {
        lastContinuousArtifacts_ = std::move(artifacts);
        adaptiveStatusLabel_->setText(QStringLiteral(
            "自适应质量证据已就绪：formal=%1，quality=%2，rejected=%3。"
            "质量地图、ROI和候选将在点击“质量地图 + 局部 greedy/beam”"
            "时按需生成。")
            .arg(lastContinuousArtifacts_.formal.size())
            .arg(lastContinuousArtifacts_.qualityAccepted.size())
            .arg(lastContinuousArtifacts_.rejected.size()));
        appendLog(adaptiveStatusLabel_->text());
    }
    if (profile_.id == QStringLiteral("scanner_650") &&
        adaptiveGlobalPlanReady_) {
        QString artifactPath;
        QString artifactError;
        if (!saveAdaptivePlanArtifact(
                &artifactPath, &artifactError)) {
            appendLog(QStringLiteral(
                "警告：无法保存扫描后的自适应计划审计产物：%1")
                .arg(artifactError));
        } else {
            appendLog(QStringLiteral(
                "扫描后的逐段实测时间、质量地图和补扫候选已保存：%1")
                .arg(artifactPath));
        }
    }
    adaptiveGlobalExecutionActive_ = false;
    adaptiveExecutor_.reset();
    continuousAbortRequested_ = false;
    beginTerminalBarrier(
        finalCompleted, finalReason, QStringLiteral("continuous"),
        synchronizationSessionDir_, terminalStats);
    continuousAdaptiveQualityModeActive_ = false;
    continuousFinalizationContext_ = ContinuousFinalizationContext();
    continuousFinalizationResult_ = ContinuousFinalizationResult();
    updateUi();
    if (closeAfterContinuousFinalization_) {
        closeAfterContinuousFinalization_ = false;
        QTimer::singleShot(0, this, [this]() { close(); });
    }
}

void HikConstantLaserScanWindow::captureCurrentProfile() {
    QString error;
    applySelectedScanCenterlinePolicy(false);
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
    qualityAmbiguousBranches_.clear();
    shadowMaskedLegacyRejectedCloud_.clear();
    publicationGateRejectedCloud_.clear();
    opticalRejectedCandidateCloud_.clear();
    qualityVGrooveValidationResult_ =
        hik_scan::VGrooveTemporalValidationResult();
    qualityRejectedCloud_.clear();
    qualitySupportResult_ =
        hik_scan::AdjacentProfileSupportResult();
    qualityAdjacentRejectedPointCount_ = 0U;
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
    const bool adaptiveReadFailed =
        requestId == pendingRobotRequestId_ &&
        (readRole_ == ReadRole::AdaptiveGlobalEvaluation ||
         readRole_ == ReadRole::AdaptiveLocalEvaluation);
    const bool adaptivePreflightFailed =
        requestId == pendingRobotRequestId_ &&
        readRole_ == ReadRole::AdaptiveExecutionPreflight;
    const bool adaptiveBatchFailed =
        requestId == adaptiveKinematicRequestId_;
    if (requestId == pendingRobotRequestId_) {
        pendingRobotRequestId_ = -1;
        readRole_ = ReadRole::None;
    }
    if (adaptiveReadFailed || adaptiveBatchFailed) {
        adaptiveKinematicRequestId_ = -1;
        adaptiveEvaluationMode_ = AdaptiveEvaluationMode::None;
        adaptiveGlobalKinematicsPassed_ = false;
        if (adaptiveEvaluationLeaseHeld_) {
            robotSession_->releaseExclusive(robotClientId_);
            adaptiveEvaluationLeaseHeld_ = false;
        }
        adaptiveStatusLabel_->setText(QStringLiteral(
            "FR5 路径评估失败：%1").arg(message));
    }
    if (adaptivePreflightFailed) {
        adaptiveExecutionPreflightPending_ = false;
        adaptiveGlobalKinematicsPassed_ = false;
        safetyConfirmCheck_->setChecked(false);
        releaseScanActivity();
        adaptiveStatusLabel_->setText(QStringLiteral(
            "执行前 FR5 位姿复查失败：%1；请重新评估。").arg(message));
    }
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
    if (adaptiveEvaluationLeaseHeld_) {
        robotSession_->releaseExclusive(robotClientId_);
        adaptiveEvaluationLeaseHeld_ = false;
    }
    adaptiveKinematicRequestId_ = -1;
    adaptiveEvaluationMode_ = AdaptiveEvaluationMode::None;
    adaptiveGlobalKinematicsPassed_ = false;
    adaptiveGlobalEvaluationPoseValid_ = false;
    if (adaptiveExecutionPreflightPending_) {
        adaptiveExecutionPreflightPending_ = false;
        releaseScanActivity();
    }
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
        adaptiveGlobalPlanReady_ = false;
        adaptiveGlobalKinematicsPassed_ = false;
        adaptiveGlobalEvaluationPoseValid_ = false;
        startPoseLabel_->setText(QStringLiteral("起点: %1").arg(poseText(startPose_)));
        appendLog(QStringLiteral("已示教扫描起点。"));
    } else if (role == ReadRole::TeachEnd) {
        endPose_ = reading.pose; endTaught_ = true;
        adaptiveGlobalPlanReady_ = false;
        adaptiveGlobalKinematicsPassed_ = false;
        adaptiveGlobalEvaluationPoseValid_ = false;
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
    } else if (role == ReadRole::AdaptiveGlobalEvaluation) {
        startAdaptiveGlobalEvaluation(reading.pose);
    } else if (role == ReadRole::AdaptiveLocalEvaluation) {
        startAdaptiveLocalEvaluation(reading.pose);
    } else if (role == ReadRole::AdaptiveExecutionPreflight) {
        beginAdaptiveGlobalExecutionAfterPreflight(reading);
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

void HikConstantLaserScanWindow::onRobotMotionTimingMeasured(
        int requestId, qint64 elapsedMs, bool targetReached) {
    if (!robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    const auto found =
        adaptiveMotionRequestSegments_.find(requestId);
    if (found == adaptiveMotionRequestSegments_.end()) return;
    const int segmentId = found->second;
    adaptiveMotionRequestSegments_.erase(found);
    const double elapsedS =
        std::max<qint64>(0, elapsedMs) / 1000.0;
    if (segmentId >= 0) {
        adaptiveActualSegmentTimeS_[segmentId] = elapsedS;
        for (hik_adaptive::ScanSegment& segment :
             adaptiveGlobalPlan_.segments) {
            if (segment.segmentId == segmentId) {
                segment.actualExecutionTimeS = elapsedS;
                break;
            }
        }
    } else if (segmentId == -2) {
        adaptiveActualSegmentTimeS_[segmentId] = elapsedS;
        adaptiveGlobalPlan_.actualExecutionTimeS = elapsedS;
    }
    appendLog(QStringLiteral(
        "FR5 实测运动时间：segment_id=%1，elapsed=%2 s，reached=%3。"
        "该值来自控制器完成/停止确认的单调时钟，不是规划前估计。")
        .arg(segmentId)
        .arg(elapsedS, 0, 'f', 3)
        .arg(targetReached
             ? QStringLiteral("true") : QStringLiteral("false")));
}

void HikConstantLaserScanWindow::onKinematicPathEvaluated(
        int requestId, int actionId,
        hik_adaptive::RobotPathEvaluation evaluation) {
    if (requestId != adaptiveKinematicRequestId_ ||
        !robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    const auto token = adaptiveEvaluationTokens_.find(actionId);
    if (token == adaptiveEvaluationTokens_.end()) {
        appendLog(QStringLiteral(
            "忽略未知的 FR5 路径评估 token=%1。").arg(actionId));
        return;
    }
    if (adaptiveEvaluationMode_ ==
        AdaptiveEvaluationMode::Global) {
        const int segmentIndex = token->second.first;
        if (segmentIndex >= 0 &&
            static_cast<std::size_t>(segmentIndex) <
                adaptiveGlobalPlan_.segments.size()) {
            const hik_adaptive::ScanSegment& segment =
                adaptiveGlobalPlan_.segments[
                    static_cast<std::size_t>(segmentIndex)];
            if (segment.usedLineFallback) {
                evaluation.usedLineFallback = true;
                evaluation.primaryPrimitiveFailureDetail =
                    segment.fallbackReason;
            }
            adaptiveGlobalPlan_.segments[
                static_cast<std::size_t>(segmentIndex)].robot =
                    evaluation;
        }
        adaptiveGlobalEvaluations_[token->second.first] =
            evaluation;
    } else if (adaptiveEvaluationMode_ ==
               AdaptiveEvaluationMode::Local) {
        if (token->second.first < 0) {
            adaptiveInitialEvaluations_[token->second.second] =
                evaluation;
        } else {
            adaptivePairEvaluations_[token->second] = evaluation;
        }
    }
    ++adaptiveEvaluationReceived_;
    appendLog(QStringLiteral(
        "FR5 路径评估 %1/%2：token=%3，IK=%4，singularity=%5，"
        "collision=%6，min_joint_margin=%7 deg，min_sigma=%8，"
        "estimate=%9 s。")
        .arg(adaptiveEvaluationReceived_)
        .arg(adaptiveEvaluationExpected_)
        .arg(actionId)
        .arg(QString::fromLatin1(
            hik_adaptive::verificationStateName(evaluation.ik)))
        .arg(QString::fromLatin1(
            hik_adaptive::verificationStateName(
                evaluation.singularity)))
        .arg(QString::fromLatin1(
            hik_adaptive::verificationStateName(
                evaluation.collision)))
        .arg(evaluation.minimumJointLimitMarginDeg, 0, 'f', 3)
        .arg(evaluation.minimumSingularValue, 0, 'f', 6)
        .arg(evaluation.estimatedExecutionTimeS, 0, 'f', 3));
}

void HikConstantLaserScanWindow::onKinematicPathBatchFinished(
        int requestId, bool completed, QString description) {
    if (requestId != adaptiveKinematicRequestId_ ||
        !robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    adaptiveKinematicRequestId_ = -1;
    appendLog(QStringLiteral("FR5 路径评估批次：%1")
                  .arg(description));
    if (completed &&
        adaptiveEvaluationReceived_ ==
            adaptiveEvaluationExpected_ &&
        adaptiveEvaluationMode_ ==
            AdaptiveEvaluationMode::Global &&
        !adaptiveArcFallbackEvaluationActive_) {
        std::vector<std::size_t> fallbackIndexes;
        for (std::size_t index = 0U;
             index < adaptiveGlobalPlan_.segments.size(); ++index) {
            const hik_adaptive::ScanSegment& segment =
                adaptiveGlobalPlan_.segments[index];
            const auto found = adaptiveGlobalEvaluations_.find(
                static_cast<int>(index));
            if (segment.primitive ==
                    hik_adaptive::MotionPrimitive::Arc &&
                segment.allowLineFallback &&
                (found == adaptiveGlobalEvaluations_.end() ||
                 found->second.ik !=
                     hik_adaptive::VerificationState::Passed ||
                 found->second.singularity !=
                     hik_adaptive::VerificationState::Passed)) {
                fallbackIndexes.push_back(index);
            }
        }
        if (!fallbackIndexes.empty()) {
            std::set<std::size_t> failedArcIndexes(
                fallbackIndexes.begin(), fallbackIndexes.end());
            fallbackIndexes.clear();
            for (std::size_t index = 0U;
                 index < adaptiveGlobalPlan_.segments.size();
                 ++index) {
                if (adaptiveGlobalPlan_.segments[index].primitive ==
                    hik_adaptive::MotionPrimitive::Arc) {
                    fallbackIndexes.push_back(index);
                }
            }
            for (std::size_t index : fallbackIndexes) {
                hik_adaptive::ScanSegment& segment =
                    adaptiveGlobalPlan_.segments[index];
                const double oldLength =
                    hik_adaptive::segmentMotionLengthMm(segment);
                const auto found = adaptiveGlobalEvaluations_.find(
                    static_cast<int>(index));
                segment.primitive =
                    hik_adaptive::MotionPrimitive::Line;
                segment.usedLineFallback = true;
                segment.blendRadiusMm = 0.0;
                if (failedArcIndexes.count(index) != 0U) {
                    segment.fallbackReason =
                        found == adaptiveGlobalEvaluations_.end()
                        ? "arc evaluation result is missing"
                        : found->second.detail;
                } else {
                    segment.fallbackReason =
                        "another MoveC failed; entire trajectory "
                        "falls back to stopped line transitions";
                }
                if (index > 0U) {
                    adaptiveGlobalPlan_.segments[
                        index - 1U].blendRadiusMm = 0.0;
                }
                const double newLength =
                    hik_adaptive::segmentMotionLengthMm(segment);
                adaptiveGlobalPlan_.transitionLengthMm +=
                    newLength - oldLength;
                adaptiveGlobalPlan_.estimatedExecutionTimeS +=
                    hik_adaptive::estimateTrapezoidalMoveTime(
                        newLength, segment.speedMmS,
                        segment.accelerationMmS2) -
                    hik_adaptive::estimateTrapezoidalMoveTime(
                        oldLength, segment.speedMmS,
                        segment.accelerationMmS2);
            }
            adaptiveArcFallbackEvaluationActive_ = true;
            adaptiveGlobalEvaluations_.clear();
            appendLog(QStringLiteral(
                "%1 个 MoveC 转场未通过 IK/关节余量/奇异性检查；"
                "已改写为停稳 MoveL 转场，并自动重新评估完整冻结路径。")
                .arg(fallbackIndexes.size()));
            startAdaptiveGlobalEvaluation(
                adaptiveGlobalEvaluationPose_);
            return;
        }
    }
    if (adaptiveEvaluationLeaseHeld_) {
        robotSession_->releaseExclusive(robotClientId_);
        adaptiveEvaluationLeaseHeld_ = false;
    }
    if (!completed ||
        adaptiveEvaluationReceived_ !=
            adaptiveEvaluationExpected_) {
        adaptiveGlobalKinematicsPassed_ = false;
        if (adaptiveEvaluationMode_ ==
            AdaptiveEvaluationMode::Global) {
            adaptiveGlobalEvaluationPoseValid_ = false;
            adaptiveGlobalPlan_.safetyVerified = false;
            adaptiveGlobalPlan_.executable = false;
            adaptiveGlobalPlan_.safetyDetail =
                "FR5 evaluation batch was incomplete; partial per-segment "
                "evidence is retained for audit and execution is forbidden";
            QString artifactPath;
            QString artifactError;
            if (!saveAdaptivePlanArtifact(
                    &artifactPath, &artifactError)) {
                appendLog(QStringLiteral(
                    "警告：无法保存未完成的 FR5 评估证据：%1")
                    .arg(artifactError));
            } else {
                appendLog(QStringLiteral(
                    "未完成的 FR5 评估证据已保存，计划不可执行：%1")
                    .arg(artifactPath));
            }
        }
        adaptiveStatusLabel_->setText(QStringLiteral(
            "FR5 路径评估未完整结束：received=%1/%2。计划不可执行。")
            .arg(adaptiveEvaluationReceived_)
            .arg(adaptiveEvaluationExpected_));
        adaptiveEvaluationMode_ = AdaptiveEvaluationMode::None;
        adaptiveArcFallbackEvaluationActive_ = false;
        updateUi();
        return;
    }
    if (adaptiveEvaluationMode_ ==
        AdaptiveEvaluationMode::Global) {
        bool passed =
            adaptiveGlobalEvaluations_.size() ==
            adaptiveGlobalPlan_.segments.size();
        for (const auto& entry :
             adaptiveGlobalEvaluations_) {
            passed = passed &&
                entry.second.ik ==
                    hik_adaptive::VerificationState::Passed &&
                entry.second.singularity ==
                    hik_adaptive::VerificationState::Passed;
        }
        adaptiveGlobalKinematicsPassed_ = passed;
        if (!passed) adaptiveGlobalEvaluationPoseValid_ = false;
        adaptiveGlobalPlan_.safetyVerified = false;
        adaptiveGlobalPlan_.executable = false;
        adaptiveGlobalPlan_.safetyDetail = passed
            ? "IK, joint margin and numerical singularity checks passed; "
              "collision remains UNKNOWN and fixed global execution "
              "requires explicit operator confirmation"
            : "at least one segment failed or lacks complete IK, joint "
              "margin or numerical singularity verification";
        adaptiveStatusLabel_->setText(passed
            ? QStringLiteral(
                  "全局 FR5 采样 IK、关节余量和奇异性已通过；"
                  "collision=UNKNOWN。仅在人工确认完整固定路径后"
                  "允许执行全局粗扫。")
            : QStringLiteral(
                  "全局路径至少一段 IK/关节余量/奇异性未通过，"
                  "真实执行已禁用。"));
        QString artifactPath;
        QString artifactError;
        if (!saveAdaptivePlanArtifact(
                &artifactPath, &artifactError)) {
            appendLog(QStringLiteral(
                "警告：无法保存带 FR5 评估的全局计划：%1")
                .arg(artifactError));
        } else {
            appendLog(QStringLiteral(
                "带逐段 FR5 评估的全局计划已保存：%1")
                .arg(artifactPath));
        }
    } else if (adaptiveEvaluationMode_ ==
               AdaptiveEvaluationMode::Local) {
        finishAdaptiveLocalPlanning();
    }
    adaptiveEvaluationMode_ = AdaptiveEvaluationMode::None;
    adaptiveArcFallbackEvaluationActive_ = false;
    updateUi();
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
    if (adaptiveGlobalExecutionActive_) {
        dispatchAdaptiveCommand(
            adaptiveExecutor_.motionFinished(
                targetReached && !continuousAbortRequested_,
                localPath(description)));
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
    if (!profile.multipathAuditOnly &&
        !profile.lineQualityPassed) {
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
    if (profile.multipathAuditOnly) {
        appendLog(QStringLiteral(
            "本帧正式点数不足，已进入 multipath audit-only："
            "不发布任何稀疏正式点；完整分支和 Shadow 遮罩点仍将"
            "变换到 base_link 并只写入 rejected。"));
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
                    diagnostics.pathUsableCandidateCount
                ? diagnostics.totalCandidateCount -
                      diagnostics.pathUsableCandidateCount
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
            "质量中心线：analysis/passed=%1/%2，legacy/quality 3D点=%3/%4，"
            "候选 path-usable/total/fatal-rejected=%5/%6/%7；"
            "拒绝 low/width/saturation/multipeak/asymmetry/fit/quality/mask="
            "%8/%9/%10/%11/%12/%13/%14/%15；"
            "多路径 interval/scanline/branch-3D=%16/%17/%18，"
            "Shadow 遮罩 legacy=%19；新旧中心：%20；算法=%21%22")
            .arg(profile.qualityAnalysisCompleted
                     ? QStringLiteral("yes")
                     : QStringLiteral("no"))
            .arg(profile.qualityExtractionPassed
                     ? QStringLiteral("yes")
                     : QStringLiteral("no"))
            .arg(static_cast<qulonglong>(
                profile.legacyPoints.size()))
            .arg(static_cast<qulonglong>(
                profile.qualityPoints.size()))
            .arg(static_cast<qulonglong>(
                diagnostics.pathUsableCandidateCount))
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
            .arg(static_cast<qulonglong>(
                diagnostics.multipathIntervalCount))
            .arg(static_cast<qulonglong>(
                diagnostics.multipathAmbiguousScanlineCount))
            .arg(static_cast<qulonglong>([&profile] {
                std::size_t count = 0U;
                for (const hik_calibration::StaticProfileAmbiguousBranch&
                         branch : profile.qualityAmbiguousBranches) {
                    count += branch.points.size();
                }
                return count;
            }()))
            .arg(static_cast<qulonglong>(
                profile.ambiguityMaskedLegacyPoints.size()))
            .arg(centerOffset)
            .arg(QString::fromStdString(
                profile.centerlineAlgorithmVersion))
            .arg(profile.qualityExtractionError.empty()
                     ? QString()
                     : QStringLiteral("；错误=") +
                           QString::fromStdString(
                               profile.qualityExtractionError)));
        appendLog(QStringLiteral(
            "质量指标：provisional/publishable/gap=%1/%2/%3，"
            "mean SNR/FWHM=%4/%5 px，"
            "selected saturation=%6%，multipeak scanlines=%7，"
            "mean asymmetry/fit/second-peak=%8/%9/%10，"
            "path margin/point=%11。")
            .arg(static_cast<qulonglong>(
                diagnostics.provisionalSelectedPointCount))
            .arg(static_cast<qulonglong>(
                diagnostics.publishableSelectedPointCount))
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
    const cv::Matx44d baseFromCamera =
        baseFromFlange * handEye_.flangeFromCamera;
    const std::size_t oldCloudSize = cloud_.size();
    const std::size_t oldQualityCloudSize = qualityCloud_.size();
    const std::size_t oldAmbiguousBranchSize =
        qualityAmbiguousBranches_.size();
    const std::size_t oldShadowMaskedLegacySize =
        shadowMaskedLegacyRejectedCloud_.size();
    const std::size_t oldPublicationGateRejectedSize =
        publicationGateRejectedCloud_.size();
    const std::size_t oldOpticalRejectedCandidateSize =
        opticalRejectedCandidateCloud_.size();
    std::string coreError;
    const int profileIndex = static_cast<int>(profileRows_.size());
    if (!pendingProfile_.points.empty() &&
        !hik_scan::appendProfileInBase(
            pendingProfile_, baseFromFlange,
            handEye_.flangeFromCamera, profileIndex,
            &cloud_, &coreError)) {
        abortScan(QString::fromStdString(coreError), false);
        return;
    }
    if (pendingProfile_.points.empty() &&
        !pendingProfile_.multipathAuditOnly) {
        abortScan(
            QStringLiteral(
                "轮廓没有正式点且未声明 multipath audit-only，"
                "已禁止继续。"),
            false);
        return;
    }
    QString qualityAppendError;
    if (pendingProfile_.qualityExtractionPassed &&
        !pendingProfile_.multipathAuditOnly &&
        !pendingProfile_.qualityPoints.empty()) {
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
    if (!pendingProfile_.qualityRejectedCandidatePoints.empty()) {
        if (!hik_scan::appendProfilePointsUsingBaseFromCamera(
                pendingProfile_.qualityRejectedCandidatePoints,
                baseFromCamera, profileIndex,
                &opticalRejectedCandidateCloud_, &coreError) ||
            opticalRejectedCandidateCloud_.size() -
                    oldOpticalRejectedCandidateSize !=
                pendingProfile_.qualityRejectedCandidatePoints.size()) {
            cloud_.resize(oldCloudSize);
            qualityCloud_.resize(oldQualityCloudSize);
            qualityAmbiguousBranches_.resize(
                oldAmbiguousBranchSize);
            shadowMaskedLegacyRejectedCloud_.resize(
                oldShadowMaskedLegacySize);
            publicationGateRejectedCloud_.resize(
                oldPublicationGateRejectedSize);
            opticalRejectedCandidateCloud_.resize(
                oldOpticalRejectedCandidateSize);
            abortScan(
                QStringLiteral(
                    "轮廓 %1 的二维光学硬拒绝候选无法完整变换到 "
                    "base_link；为避免拒绝原因脱离空间位置已终止：%2")
                    .arg(profileIndex)
                    .arg(QString::fromStdString(
                        coreError.empty()
                            ? "optical rejected candidates were partially transformed"
                            : coreError)),
                false);
            return;
        }
        for (std::size_t index =
                 oldOpticalRejectedCandidateSize;
             index < opticalRejectedCandidateCloud_.size();
             ++index) {
            opticalRejectedCandidateCloud_[index].qualityFlags &=
                ~static_cast<std::uint32_t>(
                    hik_scan::CLOUD_QUALITY_OPTICAL_ACCEPTED);
        }
    }
    if (!pendingProfile_.ambiguityMaskedLegacyPoints.empty()) {
        if (!hik_scan::appendProfilePointsUsingBaseFromCamera(
                pendingProfile_.ambiguityMaskedLegacyPoints,
                baseFromCamera, profileIndex,
                &shadowMaskedLegacyRejectedCloud_, &coreError) ||
            shadowMaskedLegacyRejectedCloud_.size() ==
                oldShadowMaskedLegacySize) {
            cloud_.resize(oldCloudSize);
            qualityCloud_.resize(oldQualityCloudSize);
            qualityAmbiguousBranches_.resize(
                oldAmbiguousBranchSize);
            shadowMaskedLegacyRejectedCloud_.resize(
                oldShadowMaskedLegacySize);
            publicationGateRejectedCloud_.resize(
                oldPublicationGateRejectedSize);
            opticalRejectedCandidateCloud_.resize(
                oldOpticalRejectedCandidateSize);
            abortScan(
                QStringLiteral(
                    "轮廓 %1 的 Shadow 遮罩 Legacy 点无法变换到 "
                    "base_link；为避免丢失 rejected 审计证据已终止：%2")
                    .arg(profileIndex)
                    .arg(QString::fromStdString(
                        coreError.empty()
                            ? "masked legacy points produced no finite point"
                            : coreError)),
                false);
            return;
        }
        for (std::size_t index = oldShadowMaskedLegacySize;
             index < shadowMaskedLegacyRejectedCloud_.size();
             ++index) {
            shadowMaskedLegacyRejectedCloud_[index].qualityFlags |=
                static_cast<std::uint32_t>(
                    hik_scan::
                        CLOUD_QUALITY_REJECTED_SHADOW_LEGACY_MULTIPATH);
        }
    }
    if (!pendingProfile_.publicationGateRejectedPoints.empty()) {
        if (!hik_scan::appendProfilePointsUsingBaseFromCamera(
                pendingProfile_.publicationGateRejectedPoints,
                baseFromCamera, profileIndex,
                &publicationGateRejectedCloud_, &coreError) ||
            publicationGateRejectedCloud_.size() -
                    oldPublicationGateRejectedSize !=
                pendingProfile_.publicationGateRejectedPoints.size()) {
            cloud_.resize(oldCloudSize);
            qualityCloud_.resize(oldQualityCloudSize);
            qualityAmbiguousBranches_.resize(
                oldAmbiguousBranchSize);
            shadowMaskedLegacyRejectedCloud_.resize(
                oldShadowMaskedLegacySize);
            publicationGateRejectedCloud_.resize(
                oldPublicationGateRejectedSize);
            opticalRejectedCandidateCloud_.resize(
                oldOpticalRejectedCandidateSize);
            abortScan(
                QStringLiteral(
                    "轮廓 %1 的 profile publication-gate 拒绝点无法"
                    "完整变换到 base_link；为避免丢失审计证据已终止：%2")
                    .arg(profileIndex)
                    .arg(QString::fromStdString(
                        coreError.empty()
                            ? "publication-gate points were partially transformed"
                            : coreError)),
                false);
            return;
        }
        for (std::size_t index = oldPublicationGateRejectedSize;
             index < publicationGateRejectedCloud_.size();
             ++index) {
            publicationGateRejectedCloud_[index].qualityFlags |=
                static_cast<std::uint32_t>(
                    hik_scan::
                        CLOUD_QUALITY_REJECTED_PROFILE_PUBLICATION_GATE);
        }
    }
    const bool collectVGrooveEvidence =
        adaptiveQualityMappingSelected();
    for (const hik_calibration::StaticProfileAmbiguousBranch& source :
         pendingProfile_.qualityAmbiguousBranches) {
        if (!collectVGrooveEvidence) {
            break;
        }
        if (source.points.empty()) {
            continue;
        }
        if (source.intervalId < 0 || source.branchId < 0) {
            cloud_.resize(oldCloudSize);
            qualityCloud_.resize(oldQualityCloudSize);
            qualityAmbiguousBranches_.resize(
                oldAmbiguousBranchSize);
            shadowMaskedLegacyRejectedCloud_.resize(
                oldShadowMaskedLegacySize);
            publicationGateRejectedCloud_.resize(
                oldPublicationGateRejectedSize);
            opticalRejectedCandidateCloud_.resize(
                oldOpticalRejectedCandidateSize);
            abortScan(
                QStringLiteral(
                    "轮廓 %1 的多路径候选缺少稳定 interval/branch "
                    "标识，已禁止发布。")
                    .arg(profileIndex),
                false);
            return;
        }
        hik_scan::VGrooveCandidateBranch branch;
        branch.ambiguityGroupId =
            stopAndShootAmbiguityGroupId(
                profileIndex, source.intervalId);
        branch.branchId = source.branchId;
        branch.formalPublicationEligible =
            pendingProfile_.qualityExtractionPassed &&
            qualityAppendError.isEmpty() &&
            !pendingProfile_.multipathAuditOnly;
        if (!hik_scan::appendProfilePointsUsingBaseFromCamera(
                source.points, baseFromCamera, profileIndex,
                &branch.points, &coreError) ||
            branch.points.empty()) {
            cloud_.resize(oldCloudSize);
            qualityCloud_.resize(oldQualityCloudSize);
            qualityAmbiguousBranches_.resize(
                oldAmbiguousBranchSize);
            shadowMaskedLegacyRejectedCloud_.resize(
                oldShadowMaskedLegacySize);
            publicationGateRejectedCloud_.resize(
                oldPublicationGateRejectedSize);
            opticalRejectedCandidateCloud_.resize(
                oldOpticalRejectedCandidateSize);
            abortScan(
                QStringLiteral(
                    "轮廓 %1 的多路径候选无法变换到 base_link，"
                    "为避免丢失 rejected 审计证据已终止：%2")
                    .arg(profileIndex)
                    .arg(QString::fromStdString(
                        coreError.empty()
                            ? "candidate branch produced no finite point"
                            : coreError)),
                false);
            return;
        }
        for (hik_scan::CloudPoint& point : branch.points) {
            point.qualityFlags &=
                ~static_cast<std::uint32_t>(
                    hik_scan::CLOUD_QUALITY_OPTICAL_ACCEPTED);
            point.qualityFlags |=
                static_cast<std::uint32_t>(
                    hik_scan::CLOUD_QUALITY_2D_MULTIPATH_CANDIDATE);
        }
        qualityAmbiguousBranches_.push_back(
            std::move(branch));
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
    row.qualityAnalysisCompleted =
        pendingProfile_.qualityAnalysisCompleted;
    row.qualityExtractionPassed =
        pendingProfile_.qualityExtractionPassed &&
        qualityAppendError.isEmpty();
    row.multipathAuditOnly =
        pendingProfile_.multipathAuditOnly;
    row.legacyPointCount =
        static_cast<int>(pendingProfile_.legacyPoints.size());
    row.qualityPointCount =
        static_cast<int>(pendingProfile_.qualityPoints.size());
    row.ambiguityMaskedLegacyPointCount =
        static_cast<int>(
            pendingProfile_.ambiguityMaskedLegacyPoints.size());
    row.publicationGateRejectedPointCount =
        static_cast<int>(
            pendingProfile_.publicationGateRejectedPoints.size());
    const hik_stripe::Diagnostics& qualityDiagnostics =
        pendingProfile_.qualityDiagnostics;
    row.qualityCandidateCount =
        static_cast<int>(qualityDiagnostics.totalCandidateCount);
    row.qualityAcceptedCandidateCount =
        static_cast<int>(qualityDiagnostics.acceptedCandidateCount);
    row.qualityPathUsableCandidateCount =
        static_cast<int>(
            qualityDiagnostics.pathUsableCandidateCount);
    row.qualityRejectedCandidateCount =
        std::max(0, row.qualityCandidateCount -
                       row.qualityPathUsableCandidateCount);
    row.qualityProvisionalPointCount =
        static_cast<int>(
            qualityDiagnostics.provisionalSelectedPointCount);
    row.qualityPublishablePointCount =
        static_cast<int>(
            qualityDiagnostics.publishableSelectedPointCount);
    row.qualitySelectedPointCount =
        static_cast<int>(qualityDiagnostics.selectedPointCount);
    row.qualitySelectedGapCount =
        static_cast<int>(qualityDiagnostics.selectedGapCount);
    row.qualityMultiPeakScanlineCount =
        static_cast<int>(qualityDiagnostics.multiPeakScanlineCount);
    row.qualityAmbiguousPathPointCount =
        static_cast<int>(qualityDiagnostics.ambiguousPathPointCount);
    row.qualityMultipathIntervalCount =
        static_cast<int>(qualityDiagnostics.multipathIntervalCount);
    row.qualityMultipathAmbiguousScanlineCount =
        static_cast<int>(
            qualityDiagnostics.multipathAmbiguousScanlineCount);
    for (const hik_calibration::StaticProfileAmbiguousBranch& branch :
         pendingProfile_.qualityAmbiguousBranches) {
        row.qualityMultipathCandidatePointCount +=
            static_cast<int>(branch.points.size());
    }
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
        qualityAmbiguousBranches_.resize(
            oldAmbiguousBranchSize);
        shadowMaskedLegacyRejectedCloud_.resize(
            oldShadowMaskedLegacySize);
        publicationGateRejectedCloud_.resize(
            oldPublicationGateRejectedSize);
        opticalRejectedCandidateCloud_.resize(
            oldOpticalRejectedCandidateSize);
        abortScan(error, false);
        return;
    }
    profileRows_.push_back(row);
    refreshTable();
    if (adaptiveQualityMappingSelected()) {
        scanStatusLabel_->setText(QStringLiteral(
            "轮廓 %1 已累计，正式点=%2，质量 publishable=%3，"
            "多路径候选分支=%4（等待 base_link V 坡口验证）。")
            .arg(profileIndex)
            .arg(static_cast<qulonglong>(cloud_.size()))
            .arg(static_cast<qulonglong>(qualityCloud_.size()))
            .arg(static_cast<qulonglong>(
                qualityAmbiguousBranches_.size())));
    } else {
        scanStatusLabel_->setText(QStringLiteral(
            "轮廓 %1 已累计，正式点=%2；V 槽时序收尾=关闭。")
            .arg(profileIndex)
            .arg(static_cast<qulonglong>(cloud_.size())));
    }
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
                qualityRejectedCloud_.size()))
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
                qualityRejectedCloud_.size()));
        terminalStats.insert(
            QStringLiteral("quality_adjacent_rejected_point_count"),
            static_cast<double>(
                qualityAdjacentRejectedPointCount_));
        terminalStats.insert(
            QStringLiteral("quality_vgroove_promoted_point_count"),
            static_cast<double>(
                qualityVGrooveValidationResult_
                    .promotedCandidates.size()));
        terminalStats.insert(
            QStringLiteral("quality_vgroove_rejected_point_count"),
            static_cast<double>(
                qualityVGrooveValidationResult_
                    .rejectedCandidates.size()));
        terminalStats.insert(
            QStringLiteral(
                "quality_shadow_masked_legacy_rejected_point_count"),
            static_cast<double>(
                shadowMaskedLegacyRejectedCloud_.size()));
        terminalStats.insert(
            QStringLiteral(
                "quality_profile_gate_rejected_point_count"),
            static_cast<double>(
                publicationGateRejectedCloud_.size()));
        terminalStats.insert(
            QStringLiteral(
                "quality_optical_hard_rejected_point_count"),
            static_cast<double>(
                opticalRejectedCandidateCloud_.size()));
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
    const bool hasAuditableScanData =
        !profileRows_.empty() || !cloud_.empty() ||
        !qualityCloud_.empty() ||
        !qualityAmbiguousBranches_.empty() ||
        !shadowMaskedLegacyRejectedCloud_.empty() ||
        !publicationGateRejectedCloud_.empty() ||
        !opticalRejectedCandidateCloud_.empty();
    if (!scanSessionDir_.isEmpty() && hasAuditableScanData) {
        saveCloudOutputs(&saveError);
    }
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
            qualityRejectedCloud_.size()));
    terminalStats.insert(
        QStringLiteral("quality_adjacent_rejected_point_count"),
        static_cast<double>(
            qualityAdjacentRejectedPointCount_));
    terminalStats.insert(
        QStringLiteral("quality_vgroove_promoted_point_count"),
        static_cast<double>(
            qualityVGrooveValidationResult_
                .promotedCandidates.size()));
    terminalStats.insert(
        QStringLiteral("quality_vgroove_rejected_point_count"),
        static_cast<double>(
            qualityVGrooveValidationResult_
                .rejectedCandidates.size()));
    terminalStats.insert(
        QStringLiteral(
            "quality_shadow_masked_legacy_rejected_point_count"),
        static_cast<double>(
            shadowMaskedLegacyRejectedCloud_.size()));
    terminalStats.insert(
        QStringLiteral(
            "quality_profile_gate_rejected_point_count"),
        static_cast<double>(
            publicationGateRejectedCloud_.size()));
    terminalStats.insert(
        QStringLiteral(
            "quality_optical_hard_rejected_point_count"),
        static_cast<double>(
            opticalRejectedCandidateCloud_.size()));
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
    if (!applyConfiguredReconstructionDepthRange(error)) {
        return false;
    }
    updateCalibrationStatusText();
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
        "quality_analysis_completed,quality_extraction_passed,"
        "multipath_audit_only,"
        "legacy_point_count,quality_point_count,"
        "ambiguity_masked_legacy_point_count,"
        "publication_gate_rejected_point_count,"
        "quality_candidate_count,quality_accepted_candidate_count,"
        "quality_path_usable_candidate_count,"
        "quality_rejected_candidate_count,"
        "quality_provisional_point_count,quality_publishable_point_count,"
        "quality_selected_point_count,quality_selected_gap_count,"
        "quality_multipeak_scanline_count,"
        "quality_ambiguous_path_point_count,"
        "quality_multipath_interval_count,"
        "quality_multipath_ambiguous_scanline_count,"
        "quality_multipath_candidate_point_count,"
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
    if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) {
        if (error) *error = QStringLiteral("无法追加 %1").arg(manifestPath_);
        return false;
    }
    const qint64 originalSize = file.size();
    if (originalSize < 0 || !file.seek(originalSize)) {
        if (error) {
            *error = QStringLiteral("无法定位扫描清单末尾: %1")
                         .arg(file.errorString());
        }
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
    line += (row.qualityAnalysisCompleted ? "1" : "0");
    line += ',';
    line += (row.qualityExtractionPassed ? "1" : "0");
    line += ',';
    line += (row.multipathAuditOnly ? "1" : "0");
    const int qualityCounts[] = {
        row.legacyPointCount,
        row.qualityPointCount,
        row.ambiguityMaskedLegacyPointCount,
        row.publicationGateRejectedPointCount,
        row.qualityCandidateCount,
        row.qualityAcceptedCandidateCount,
        row.qualityPathUsableCandidateCount,
        row.qualityRejectedCandidateCount,
        row.qualityProvisionalPointCount,
        row.qualityPublishablePointCount,
        row.qualitySelectedPointCount,
        row.qualitySelectedGapCount,
        row.qualityMultiPeakScanlineCount,
        row.qualityAmbiguousPathPointCount,
        row.qualityMultipathIntervalCount,
        row.qualityMultipathAmbiguousScanlineCount,
        row.qualityMultipathCandidatePointCount,
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
        const QString writeError = file.errorString();
        const bool rolledBack =
            file.resize(originalSize) && file.flush();
        if (error) {
            *error = QStringLiteral(
                "写入扫描清单失败: %1；尾部回滚=%2")
                         .arg(writeError,
                              rolledBack
                                  ? QStringLiteral("成功")
                                  : QStringLiteral("失败"));
        }
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
        lineLaserCenterlinePolicyName(
            selectedScanCenterlinePolicy()));
    profileObject.insert(
        QStringLiteral("centerline_policy_source"),
        profile_.id == QStringLiteral("scanner_650")
            ? QStringLiteral("gui_session_option")
            : QStringLiteral("device_profile"));
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
    calibrationObject.insert(
        QStringLiteral("device_parameters_used_as_is"),
        useDeviceCalibrationAsIs_);
    calibrationObject.insert(
        QStringLiteral("provenance_override_active"),
        calibrationProvenanceOverrideActive_);
    calibrationObject.insert(
        QStringLiteral("handeye_declared_intrinsics_sha256"),
        handEyeDeclaredIntrinsicsSha256_);
    calibrationObject.insert(
        QStringLiteral("current_intrinsics_sha256"),
        intrinsicsSha256_);

    QJsonObject reconstructionObject;
    reconstructionObject.insert(
        QStringLiteral("mode"),
        continuousAdaptiveQualityModeActive_
            ? QStringLiteral("adaptive_quality")
            : QStringLiteral("fast_raw_voxel"));
    reconstructionObject.insert(
        QStringLiteral("formal_centerline_policy"),
        lineLaserCenterlinePolicyName(
            selectedScanCenterlinePolicy()));
    reconstructionObject.insert(
        QStringLiteral("v_groove_temporal_validation_enabled"),
        adaptiveQualityMappingSelected());
    reconstructionObject.insert(
        QStringLiteral("shadow_multipath_hard_gate"),
        selectedScanCenterlinePolicy() ==
            LineLaserCenterlinePolicy::Shadow);
    reconstructionObject.insert(
        QStringLiteral("ply_encoding"),
        binaryPlyCheck_ && binaryPlyCheck_->isChecked()
            ? QStringLiteral("binary_little_endian")
            : QStringLiteral("ascii"));
    reconstructionObject.insert(
        QStringLiteral("voxel_size_mm"),
        voxelSpin_ ? voxelSpin_->value() : 0.5);
    reconstructionObject.insert(
        QStringLiteral("camera_z_min_mm"),
        profileOptions_.reconstruction.minimumDepthMm);
    reconstructionObject.insert(
        QStringLiteral("camera_z_max_mm"),
        profileOptions_.reconstruction.maximumDepthMm);
    reconstructionObject.insert(
        QStringLiteral("calibration_valid_camera_z_min_mm"),
        laserMetadata_.validCameraZMinMm);
    reconstructionObject.insert(
        QStringLiteral("calibration_valid_camera_z_max_mm"),
        laserMetadata_.validCameraZMaxMm);
    reconstructionObject.insert(
        QStringLiteral("worker_threads"),
        static_cast<double>(
            synchronizationConfig_.reconstructionThreads));
    reconstructionObject.insert(
        QStringLiteral("queue_capacity"),
        static_cast<double>(
            synchronizationConfig_.reconstructionQueueCapacity));

    QJsonObject root;
    root.insert(QStringLiteral("schema_version"), 1);
    root.insert(QStringLiteral("created_utc"),
                QDateTime::currentDateTimeUtc().toString(
                    Qt::ISODateWithMs));
    root.insert(QStringLiteral("device_profile"), profileObject);
    root.insert(QStringLiteral("laser_control"), laserObject);
    root.insert(QStringLiteral("calibration"), calibrationObject);
    root.insert(QStringLiteral("continuous_reconstruction"),
                reconstructionObject);

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

    hik_scan::VGrooveTemporalValidationResult vGrooveResult;
    const bool vGrooveEnabled = adaptiveQualityMappingSelected();
    hik_scan::VGrooveTemporalValidationOptions vGrooveOptions;
    std::string vGrooveError;
    if (!vGrooveEnabled) {
        vGrooveResult.passThroughPublishable = qualityCloud_;
        vGrooveResult.statistics.publishableInputPointCount =
            qualityCloud_.size();
        vGrooveResult.statistics.passThroughPublishablePointCount =
            qualityCloud_.size();
        appendLog(QStringLiteral(
            "%1：V 槽时序收尾已关闭；正式 raw/voxel 仅使用常规重建点。")
            .arg(profile_.id));
    } else if (!hik_scan::validateVGrooveTemporalGeometry(
                   qualityCloud_, qualityAmbiguousBranches_,
                   vGrooveOptions, &vGrooveResult, &vGrooveError)) {
        // A validator/configuration failure is not permission to publish an
        // ambiguous branch. Preserve ordinary publishable points and send
        // every withheld branch point to rejected with an explicit reason.
        vGrooveResult =
            hik_scan::VGrooveTemporalValidationResult();
        vGrooveResult.passThroughPublishable = qualityCloud_;
        vGrooveResult.statistics.publishableInputPointCount =
            qualityCloud_.size();
        vGrooveResult.statistics
            .passThroughPublishablePointCount =
            qualityCloud_.size();
        for (const hik_scan::VGrooveCandidateBranch& branch :
             qualityAmbiguousBranches_) {
            vGrooveResult.statistics.candidateInputPointCount +=
                branch.points.size();
            for (const hik_scan::CloudPoint& source :
                 branch.points) {
                hik_scan::CloudPoint rejected = source;
                rejected.qualityFlags &=
                    ~static_cast<std::uint32_t>(
                        hik_scan::CLOUD_QUALITY_OPTICAL_ACCEPTED);
                rejected.qualityFlags |=
                    static_cast<std::uint32_t>(
                        hik_scan::
                            CLOUD_QUALITY_REJECTED_V_GROOVE_INSUFFICIENT);
                vGrooveResult.rejectedCandidates.push_back(
                    rejected);
            }
        }
        vGrooveResult.statistics.rejectedCandidatePointCount =
            vGrooveResult.rejectedCandidates.size();
        vGrooveResult.statistics
            .rejectedInsufficientCandidatePointCount =
            vGrooveResult.rejectedCandidates.size();
        appendLog(QStringLiteral(
            "警告：base_link V 坡口验证器失败；已 fail-closed，"
            "所有多路径候选仅进入 rejected：%1")
            .arg(QString::fromStdString(vGrooveError)));
    }

    for (hik_scan::CloudPoint& promoted :
         vGrooveResult.promotedCandidates) {
        promoted.qualityFlags |=
            static_cast<std::uint32_t>(
                hik_scan::CLOUD_QUALITY_OPTICAL_ACCEPTED);
    }
    std::vector<hik_scan::CloudPoint> qualityOpticalCloud =
        vGrooveResult.passThroughPublishable;
    qualityOpticalCloud.insert(
        qualityOpticalCloud.end(),
        vGrooveResult.promotedCandidates.begin(),
        vGrooveResult.promotedCandidates.end());

    // V-groove candidates remain quality evidence. Formal raw/voxel output
    // always uses the centerline selected during per-frame reconstruction.
    std::vector<hik_scan::CloudPoint> formalCloud = cloud_;

    std::string coreError;
    if (!hik_scan::saveScanPly(
            localPath(rawPlyPath_), formalCloud,
            "base_link", &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }
    const std::vector<hik_scan::CloudPoint> voxel =
        hik_scan::voxelDownsample(
            formalCloud, voxelSpin_->value());
    if (!hik_scan::saveScanPly(localPath(voxelPlyPath_), voxel, "base_link", &coreError)) {
        if (error) *error = QString::fromStdString(coreError);
        return false;
    }

    hik_scan::AdjacentProfileSupportOptions supportOptions;
    // A single-point validation has no neighboring profile by definition.
    // This only disables the adjacency filter. The preceding V-groove gate
    // still rejects every multipath branch with insufficient temporal proof.
    supportOptions.enabled =
        profileRows_.size() > 1U &&
        !qualityOpticalCloud.empty();
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
            qualityOpticalCloud, supportOptions,
            &qualitySupportResult_,
            &coreError)) {
        if (error) {
            *error = QStringLiteral("质量点相邻 profile 支持过滤失败：%1")
                .arg(QString::fromStdString(coreError));
        }
        return false;
    }
    std::vector<hik_scan::CloudPoint> rejectedCloud =
        vGrooveResult.rejectedCandidates;
    rejectedCloud.insert(
        rejectedCloud.end(),
        shadowMaskedLegacyRejectedCloud_.begin(),
        shadowMaskedLegacyRejectedCloud_.end());
    rejectedCloud.insert(
        rejectedCloud.end(),
        publicationGateRejectedCloud_.begin(),
        publicationGateRejectedCloud_.end());
    rejectedCloud.insert(
        rejectedCloud.end(),
        opticalRejectedCandidateCloud_.begin(),
        opticalRejectedCandidateCloud_.end());
    const std::size_t adjacentRejectedPointCount =
        qualitySupportResult_.rejected.size();
    rejectedCloud.insert(
        rejectedCloud.end(),
        qualitySupportResult_.rejected.begin(),
        qualitySupportResult_.rejected.end());

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
            qualityOpticalPlyPath_, qualityOpticalCloud,
            QStringLiteral("base_link"), &qualitySaveError) ||
        !saveCloudPlyAllowEmpty(
            qualityFilteredPlyPath_, qualitySupportResult_.kept,
            QStringLiteral("base_link"), &qualitySaveError) ||
        !saveCloudPlyAllowEmpty(
            qualityRejectedPlyPath_, rejectedCloud,
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
    const hik_scan::VGrooveTemporalValidationStatistics&
        vGrooveStatistics = vGrooveResult.statistics;
    if (vGrooveEnabled) {
        appendLog(QStringLiteral(
            "V 坡口门控：groups=%1，candidate=%2，promoted=%3，"
            "V-rejected=%4（ambiguous/insufficient/geometry/alternate/"
            "outlier=%5/%6/%7/%8/%9）；候选仅进入质量结果，不修改"
            "正式点云。")
        .arg(static_cast<qulonglong>(
            vGrooveResult.ambiguityGroups.size()))
        .arg(static_cast<qulonglong>([&] {
            std::size_t count = 0U;
            for (const hik_scan::VGrooveCandidateBranch& branch :
                 qualityAmbiguousBranches_) {
                count += branch.points.size();
            }
            return count;
        }()))
        .arg(static_cast<qulonglong>(
            vGrooveResult.promotedCandidates.size()))
        .arg(static_cast<qulonglong>(
            vGrooveResult.rejectedCandidates.size()))
        .arg(static_cast<qulonglong>(
            vGrooveStatistics
                .rejectedAmbiguousCandidatePointCount))
        .arg(static_cast<qulonglong>(
            vGrooveStatistics
                .rejectedInsufficientCandidatePointCount))
        .arg(static_cast<qulonglong>(
            vGrooveStatistics
                .rejectedGeometryCandidatePointCount))
        .arg(static_cast<qulonglong>(
            vGrooveStatistics
                .rejectedAlternateBranchPointCount))
        .arg(static_cast<qulonglong>(
            vGrooveStatistics.rejectedOutlierPointCount)));
    }
    appendLog(QStringLiteral(
        "质量点云：V-gated optical=%1，support_filter=%2"
        "（radius=%3 mm，min_profiles=%4，max_gap=%5），"
        "kept/adjacent-rejected/optical-hard-rejected/total-rejected="
        "%6/%7/%8/%9，invalid/insufficient=%10/%11，"
        "confidence-weighted voxel=%12；"
        "optical=%13；filtered=%14；rejected=%15；voxel=%16。")
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
            adjacentRejectedPointCount))
        .arg(static_cast<qulonglong>(
            opticalRejectedCandidateCloud_.size()))
        .arg(static_cast<qulonglong>(
            rejectedCloud.size()))
        .arg(static_cast<qulonglong>(
            qualityStatistics.invalidPointCount))
        .arg(static_cast<qulonglong>(
            qualityStatistics.insufficientSupportPointCount))
        .arg(static_cast<qulonglong>(
            qualityVoxelStatistics.outputPointCount))
        .arg(qualityOpticalPlyPath_, qualityFilteredPlyPath_,
             qualityRejectedPlyPath_, qualityVoxelPlyPath_));
    if (selectedScanCenterlinePolicy() ==
        LineLaserCenterlinePolicy::Shadow) {
        std::size_t maskedLegacyPointCount = 0U;
        for (const ProfileRow& row : profileRows_) {
            maskedLegacyPointCount +=
                static_cast<std::size_t>(
                    std::max(
                        0,
                        row.ambiguityMaskedLegacyPointCount));
        }
        appendLog(QStringLiteral(
            "Shadow 保护：正式 legacy 已硬遮罩 %1 个歧义区点；"
            "这些 Legacy 点只保存在 rejected；Quality/V 候选绝不"
            "替换正式坐标。正式输出=%2、%3。")
            .arg(static_cast<qulonglong>(
                maskedLegacyPointCount))
            .arg(rawPlyPath_, voxelPlyPath_));
    }

    cloud_ = std::move(formalCloud);
    qualityCloud_ = std::move(qualityOpticalCloud);
    qualityVGrooveValidationResult_ =
        std::move(vGrooveResult);
    qualityRejectedCloud_ = std::move(rejectedCloud);
    qualityAdjacentRejectedPointCount_ =
        adjacentRejectedPointCount;
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
    for (const hik_calibration::StripePoint& point :
         profile.ambiguityMaskedLegacyStripe) {
        cv::circle(overlay, cv::Point(
                       static_cast<int>(std::lround(point.pixel.x)),
                       static_cast<int>(std::lround(point.pixel.y))),
                   2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    }
    std::size_t ambiguousCandidateCount = 0U;
    for (const hik_calibration::StaticProfileAmbiguousBranch& branch :
         profile.qualityAmbiguousBranches) {
        ambiguousCandidateCount += branch.stripe.size();
        for (const hik_calibration::StripePoint& point :
             branch.stripe) {
            cv::drawMarker(
                overlay,
                cv::Point(
                    static_cast<int>(std::lround(point.pixel.x)),
                    static_cast<int>(std::lround(point.pixel.y))),
                cv::Scalar(255, 0, 255),
                cv::MARKER_CROSS, 4, 1, cv::LINE_AA);
        }
    }
    if (!profile.qualityStripe.empty() &&
        profile.qualityDiagnostics.appliedRoi.width > 0 &&
        profile.qualityDiagnostics.appliedRoi.height > 0) {
        cv::rectangle(
            overlay, profile.qualityDiagnostics.appliedRoi,
            cv::Scalar(255, 120, 0), 1, cv::LINE_AA);
        cv::putText(
            overlay,
            "legacy=green publishable=yellow masked=red "
            "multipath=magenta ROI=blue",
            cv::Point(20, 66), cv::FONT_HERSHEY_SIMPLEX,
            0.55, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    std::ostringstream caption;
    caption << "official=" << profile.points.size()
            << " legacy=" << profile.legacyPoints.size()
            << " quality=" << profile.qualityPoints.size()
            << " masked=" << profile.ambiguityMaskedLegacyPoints.size()
            << " ambiguous=" << ambiguousCandidateCount
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
