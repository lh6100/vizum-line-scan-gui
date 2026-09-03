#include "exposure_test/ExposureTestWindow.h"

#include "HikCameraWorker.h"
#include "ImageView.h"
#include "LineLaserDeviceProfile.h"
#include "exposure_test/ExposureTestStorage.h"

#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QSpinBox>
#include <QThread>
#include <QUrl>
#include <QVBoxLayout>

#include <algorithm>

namespace {

const int kCameraCount = 2;

QString boolText(bool value) {
    return value ? QStringLiteral("HIGH") : QStringLiteral("LOW");
}

} // namespace

struct ExposureTestWindow::CameraPanel {
    const LineLaserDeviceProfile* profile{nullptr};
    QThread thread;
    HikCameraWorker* worker{nullptr};
    QLineEdit* ipEdit{nullptr};
    QPushButton* connectButton{nullptr};
    QPushButton* disconnectButton{nullptr};
    QDoubleSpinBox* exposureSpin{nullptr};
    QDoubleSpinBox* gainSpin{nullptr};
    QSpinBox* timeoutSpin{nullptr};
    QPushButton* captureButton{nullptr};
    QLabel* statusLabel{nullptr};
    QLabel* identityLabel{nullptr};
    QLabel* imageInfoLabel{nullptr};
    QLabel* savedPathLabel{nullptr};
    ImageView* imageView{nullptr};
    bool connected{false};
    bool busy{false};
    int pendingRequestId{0};
    QString model;
    QString serial;
    QString ip;
};

ExposureTestWindow::ExposureTestWindow(LineLaserController* laserController,
                                       QWidget* parent)
    : QWidget(parent), laserController_(laserController) {
    const std::vector<LineLaserDeviceProfile>& profiles = lineLaserDeviceProfiles();
    if (profiles.size() < static_cast<std::size_t>(kCameraCount)) {
        throw std::runtime_error("two line-laser device profiles are required");
    }
    for (int index = 0; index < kCameraCount; ++index) {
        cameras_[index] = std::make_unique<CameraPanel>();
        cameras_[index]->profile = &profiles[static_cast<std::size_t>(index)];
    }

    buildUi();
    for (int index = 0; index < kCameraCount; ++index) setupCameraWorker(index);

    if (laserController_) {
        connect(laserController_, &LineLaserController::connectionStateChanged,
                this, [this](LineLaserConnectionState state, const QString& detail) {
                    laserConnectionState_ = state;
                    laserConnectionLabel_->setText(detail);
                    appendLog(QStringLiteral("TTL控制连接：%1").arg(detail));
                    updateUi();
                });
        connect(laserController_, &LineLaserController::statusChanged,
                this, [this](const LineLaserStatus& status) {
                    latestLaserStatus_ = status;
                    laserReadbackLabel_->setText(QStringLiteral(
                        "实际状态：%1｜450=%2｜650=%3｜租约=%4 ms")
                        .arg(laserStateName(status.state),
                             boolText(status.ttl450High),
                             boolText(status.ttl650High))
                        .arg(status.leaseRemainingMs));
                    updateUi();
                });
        connect(laserController_, &LineLaserController::commandFinished,
                this, [this](const QString& command, bool success,
                             const QString& detail) {
                    appendLog(QStringLiteral("TTL命令 %1：%2")
                                  .arg(command, detail),
                              !success);
                    updateUi();
                });
        connect(laserController_, &LineLaserController::faultOccurred,
                this, [this](const QString& detail) {
                    appendLog(QStringLiteral("TTL故障：%1").arg(detail), true);
                    updateUi();
                });
        connect(laserController_, &LineLaserController::logMessage,
                this, [this](const QString& message) {
                    appendLog(QStringLiteral("TTL：%1").arg(message));
                });
    }

    appendLog(QStringLiteral(
        "测试页已就绪。图片文件名和 captures.csv 均记录实际分辨率、曝光、增益与时间戳。"));
    updateUi();
}

ExposureTestWindow::~ExposureTestWindow() {
    if (laserController_) laserController_->requestOffTracked();
    shutdownCameraWorkers();
}

void ExposureTestWindow::buildUi() {
    setWindowTitle(QStringLiteral("海康双相机 / 双线激光曝光测试"));
    resize(1560, 940);

    auto* root = new QVBoxLayout(this);
    auto* title = new QLabel(QStringLiteral(
        "双相机曝光与增益测试｜每台相机使用独立参数，保存 PNG 与 captures.csv"), this);
    QFont titleFont = title->font();
    titleFont.setPointSize(titleFont.pointSize() + 3);
    titleFont.setBold(true);
    title->setFont(titleFont);
    root->addWidget(title);

    auto* outputGroup = new QGroupBox(QStringLiteral("保存位置与批量操作"), this);
    auto* outputLayout = new QGridLayout(outputGroup);
    outputDirectoryEdit_ = new QLineEdit(defaultOutputDirectory(), outputGroup);
    chooseOutputButton_ = new QPushButton(QStringLiteral("选择目录"), outputGroup);
    openOutputButton_ = new QPushButton(QStringLiteral("打开目录"), outputGroup);
    connectBothButton_ = new QPushButton(QStringLiteral("连接两台相机"), outputGroup);
    disconnectBothButton_ = new QPushButton(QStringLiteral("断开两台相机"), outputGroup);
    captureBothButton_ = new QPushButton(QStringLiteral("两台同时拍摄并保存"), outputGroup);
    captureBothButton_->setDefault(true);
    outputLayout->addWidget(new QLabel(QStringLiteral("输出目录"), outputGroup), 0, 0);
    outputLayout->addWidget(outputDirectoryEdit_, 0, 1, 1, 5);
    outputLayout->addWidget(chooseOutputButton_, 0, 6);
    outputLayout->addWidget(openOutputButton_, 0, 7);
    outputLayout->addWidget(connectBothButton_, 1, 1);
    outputLayout->addWidget(disconnectBothButton_, 1, 2);
    outputLayout->addWidget(captureBothButton_, 1, 3, 1, 2);
    outputLayout->addWidget(new QLabel(
        QStringLiteral("输出：PNG + captures.csv（记录相机身份、参数、帧号、TTL回读）"),
        outputGroup), 1, 5, 1, 3);
    root->addWidget(outputGroup);

    auto* laserGroup = new QGroupBox(QStringLiteral("双线激光 TTL 控制"), this);
    auto* laserLayout = new QGridLayout(laserGroup);
    laserConnectButton_ = new QPushButton(QStringLiteral("连接TTL控制器"), laserGroup);
    laserDisconnectButton_ = new QPushButton(QStringLiteral("断开TTL控制器"), laserGroup);
    laser450Check_ = new QCheckBox(QStringLiteral("开启 450 nm（Pin 11）"), laserGroup);
    laser650Check_ = new QCheckBox(QStringLiteral("开启 650 nm（Pin 7）"), laserGroup);
    applyLaserButton_ = new QPushButton(QStringLiteral("应用激光状态"), laserGroup);
    lasersOffButton_ = new QPushButton(QStringLiteral("紧急关闭两路TTL"), laserGroup);
    laserConnectionLabel_ = new QLabel(QStringLiteral("激光控制未连接"), laserGroup);
    laserReadbackLabel_ = new QLabel(QStringLiteral("实际状态：unknown"), laserGroup);
    auto* laserWarning = new QLabel(QStringLiteral(
        "注意：GPIO 回读只代表逻辑电平，不能替代实测出光、硬件下拉和急停。退出页面会请求两路关光。"),
        laserGroup);
    laserWarning->setStyleSheet(QStringLiteral("color:#a15c00;"));
    laserLayout->addWidget(laserConnectButton_, 0, 0);
    laserLayout->addWidget(laserDisconnectButton_, 0, 1);
    laserLayout->addWidget(laser450Check_, 0, 2);
    laserLayout->addWidget(laser650Check_, 0, 3);
    laserLayout->addWidget(applyLaserButton_, 0, 4);
    laserLayout->addWidget(lasersOffButton_, 0, 5);
    laserLayout->addWidget(laserConnectionLabel_, 1, 0, 1, 3);
    laserLayout->addWidget(laserReadbackLabel_, 1, 3, 1, 3);
    laserLayout->addWidget(laserWarning, 2, 0, 1, 6);
    root->addWidget(laserGroup);

    auto* cameraSplitter = new QSplitter(Qt::Horizontal, this);
    for (int index = 0; index < kCameraCount; ++index) {
        cameraSplitter->addWidget(buildCameraPanel(index));
        cameraSplitter->setStretchFactor(index, 1);
    }
    cameraSplitter->setChildrenCollapsible(false);
    root->addWidget(cameraSplitter, 1);

    auto* logGroup = new QGroupBox(QStringLiteral("运行日志"), this);
    auto* logLayout = new QVBoxLayout(logGroup);
    logEdit_ = new QPlainTextEdit(logGroup);
    logEdit_->setReadOnly(true);
    logEdit_->setMaximumBlockCount(1000);
    logEdit_->setMinimumHeight(115);
    logLayout->addWidget(logEdit_);
    root->addWidget(logGroup);

    connect(chooseOutputButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::chooseOutputDirectory);
    connect(openOutputButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::openOutputDirectory);
    connect(connectBothButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::connectBothCameras);
    connect(disconnectBothButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::disconnectBothCameras);
    connect(captureBothButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::captureBothCameras);
    connect(laserConnectButton_, &QPushButton::clicked,
            laserController_, &LineLaserController::connectController);
    connect(laserDisconnectButton_, &QPushButton::clicked,
            laserController_, &LineLaserController::disconnectController);
    connect(applyLaserButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::applyLaserSelection);
    connect(lasersOffButton_, &QPushButton::clicked,
            this, &ExposureTestWindow::turnLasersOff);
}

QWidget* ExposureTestWindow::buildCameraPanel(int cameraIndex) {
    CameraPanel& camera = *cameras_[cameraIndex];
    auto* group = new QGroupBox(
        QStringLiteral("%1｜%2")
            .arg(camera.profile->id, camera.profile->displayName), this);
    auto* root = new QVBoxLayout(group);
    auto* controls = new QGridLayout;

    camera.ipEdit = new QLineEdit(camera.profile->defaultCameraIp, group);
    camera.connectButton = new QPushButton(QStringLiteral("连接"), group);
    camera.disconnectButton = new QPushButton(QStringLiteral("断开"), group);
    camera.exposureSpin = new QDoubleSpinBox(group);
    camera.exposureSpin->setRange(1.0, 10000000.0);
    camera.exposureSpin->setDecimals(3);
    camera.exposureSpin->setValue(1825.0);
    camera.exposureSpin->setSuffix(QStringLiteral(" us"));
    camera.gainSpin = new QDoubleSpinBox(group);
    camera.gainSpin->setRange(0.0, 48.0);
    camera.gainSpin->setDecimals(3);
    camera.gainSpin->setValue(0.0);
    camera.gainSpin->setSuffix(QStringLiteral(" dB"));
    camera.timeoutSpin = new QSpinBox(group);
    camera.timeoutSpin->setRange(100, 10000);
    camera.timeoutSpin->setValue(3000);
    camera.timeoutSpin->setSuffix(QStringLiteral(" ms"));
    camera.captureButton = new QPushButton(QStringLiteral("拍摄并保存当前相机"), group);
    camera.statusLabel = new QLabel(QStringLiteral("未连接"), group);
    camera.identityLabel = new QLabel(
        QStringLiteral("期望：%1｜SN=%2｜%3 nm")
            .arg(camera.profile->expectedCameraModel,
                 camera.profile->expectedCameraSerial)
            .arg(camera.profile->wavelengthNm), group);
    camera.identityLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

    controls->addWidget(new QLabel(QStringLiteral("相机 IP"), group), 0, 0);
    controls->addWidget(camera.ipEdit, 0, 1, 1, 2);
    controls->addWidget(camera.connectButton, 0, 3);
    controls->addWidget(camera.disconnectButton, 0, 4);
    controls->addWidget(new QLabel(QStringLiteral("曝光时间"), group), 1, 0);
    controls->addWidget(camera.exposureSpin, 1, 1);
    controls->addWidget(new QLabel(QStringLiteral("增益"), group), 1, 2);
    controls->addWidget(camera.gainSpin, 1, 3);
    controls->addWidget(new QLabel(QStringLiteral("超时"), group), 2, 0);
    controls->addWidget(camera.timeoutSpin, 2, 1);
    controls->addWidget(camera.captureButton, 2, 2, 1, 3);
    controls->addWidget(camera.statusLabel, 3, 0, 1, 5);
    controls->addWidget(camera.identityLabel, 4, 0, 1, 5);
    root->addLayout(controls);

    camera.imageView = new ImageView(group);
    camera.imageView->setEmptyText(QStringLiteral("等待 %1 单帧图像").arg(camera.profile->id));
    camera.imageView->setMinimumSize(520, 330);
    root->addWidget(camera.imageView, 1);
    camera.imageInfoLabel = new QLabel(QStringLiteral("尚未拍摄"), group);
    camera.savedPathLabel = new QLabel(QStringLiteral("尚未保存图片"), group);
    camera.savedPathLabel->setWordWrap(true);
    camera.savedPathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(camera.imageInfoLabel);
    root->addWidget(camera.savedPathLabel);

    connect(camera.connectButton, &QPushButton::clicked,
            this, [this, cameraIndex]() { connectCamera(cameraIndex); });
    connect(camera.disconnectButton, &QPushButton::clicked,
            this, [this, cameraIndex]() { disconnectCamera(cameraIndex); });
    connect(camera.captureButton, &QPushButton::clicked,
            this, [this, cameraIndex]() { captureCamera(cameraIndex); });
    return group;
}

void ExposureTestWindow::setupCameraWorker(int cameraIndex) {
    CameraPanel& camera = *cameras_[cameraIndex];
    camera.thread.setObjectName(QStringLiteral("ExposureTestCamera%1").arg(cameraIndex + 1));
    camera.worker = new HikCameraWorker;
    camera.worker->moveToThread(&camera.thread);
    connect(&camera.thread, &QThread::finished,
            camera.worker, &QObject::deleteLater);
    connect(camera.worker, &HikCameraWorker::connectionChanged,
            this, [this, cameraIndex](bool connected, const QString& description) {
                CameraPanel& panel = *cameras_[cameraIndex];
                panel.connected = connected;
                if (!connected) panel.pendingRequestId = 0;
                panel.statusLabel->setText(description);
                appendLog(QStringLiteral("%1：%2")
                              .arg(panel.profile->id, description),
                          !connected && description.contains(QStringLiteral("失败")));
                updateUi();
            }, Qt::QueuedConnection);
    connect(camera.worker, &HikCameraWorker::identityChanged,
            this, [this, cameraIndex](const QString& model,
                                     const QString& serial,
                                     const QString& ip) {
                CameraPanel& panel = *cameras_[cameraIndex];
                panel.model = model;
                panel.serial = serial;
                panel.ip = ip;
                panel.identityLabel->setText(QStringLiteral(
                    "当前：%1｜SN=%2｜IP=%3｜本组激光=%4 nm")
                    .arg(model, serial, ip)
                    .arg(panel.profile->wavelengthNm));
            }, Qt::QueuedConnection);
    connect(camera.worker, &HikCameraWorker::busyChanged,
            this, [this, cameraIndex](bool busy) {
                cameras_[cameraIndex]->busy = busy;
                updateUi();
            }, Qt::QueuedConnection);
    connect(camera.worker, &HikCameraWorker::frameReady,
            this, [this, cameraIndex](int requestId,
                                     const QImage& image,
                                     quint64 frameNumber,
                                     quint64 deviceTimestamp,
                                     qint64 hostTimestamp,
                                     double actualExposure,
                                     double actualGain,
                                     const QString& description) {
                CameraPanel& panel = *cameras_[cameraIndex];
                if (panel.pendingRequestId != requestId) {
                    appendLog(QStringLiteral("%1 收到过期帧 request=%2，已忽略")
                                  .arg(panel.profile->id).arg(requestId), true);
                    return;
                }
                panel.pendingRequestId = 0;
                panel.imageView->setImage(image);
                panel.imageInfoLabel->setText(description);

                hik_exposure_test::CaptureRecord record;
                record.capturedAt = QDateTime::currentDateTime();
                record.profileId = panel.profile->id;
                record.wavelengthNm = panel.profile->wavelengthNm;
                record.cameraIp = panel.ip.isEmpty() ? panel.ipEdit->text().trimmed() : panel.ip;
                record.cameraModel = panel.model.isEmpty()
                    ? panel.profile->expectedCameraModel : panel.model;
                record.cameraSerial = panel.serial.isEmpty()
                    ? panel.profile->expectedCameraSerial : panel.serial;
                record.width = image.width();
                record.height = image.height();
                record.pixelFormat = QStringLiteral("Mono8");
                record.exposureUs = actualExposure;
                record.gainDb = actualGain;
                record.frameNumber = frameNumber;
                record.deviceTimestamp = deviceTimestamp;
                record.hostTimestamp = hostTimestamp;
                record.laserState = laserStateName(latestLaserStatus_.state);
                record.laserStatusReachable = latestLaserStatus_.reachable;
                record.laserStatusSourceMonotonicNs =
                    latestLaserStatus_.sourceMonotonicNs;
                record.ttl450High = latestLaserStatus_.ttl450High;
                record.ttl650High = latestLaserStatus_.ttl650High;

                QString savedPath;
                QString saveError;
                const bool saved = hik_exposure_test::saveCapture(
                    image, record, outputDirectoryEdit_->text().trimmed(),
                    &savedPath, &saveError);
                if (saved) {
                    panel.savedPathLabel->setText(
                        QStringLiteral("已保存：%1").arg(savedPath));
                    appendLog(QStringLiteral("%1 图片与参数已保存：%2")
                                  .arg(panel.profile->id, savedPath));
                } else {
                    panel.savedPathLabel->setText(
                        savedPath.isEmpty()
                            ? QStringLiteral("保存失败：%1").arg(saveError)
                            : QStringLiteral("图片已保存但参数清单失败：%1；%2")
                                  .arg(savedPath, saveError));
                    appendLog(QStringLiteral("%1 保存失败：%2")
                                  .arg(panel.profile->id, saveError), true);
                }
                updateUi();
            }, Qt::QueuedConnection);
    connect(camera.worker, &HikCameraWorker::log,
            this, [this, cameraIndex](const QString& message) {
                appendLog(QStringLiteral("%1相机：%2")
                              .arg(cameras_[cameraIndex]->profile->id, message));
            }, Qt::QueuedConnection);
    connect(camera.worker, &HikCameraWorker::error,
            this, [this, cameraIndex](int requestId, const QString& message) {
                CameraPanel& panel = *cameras_[cameraIndex];
                if (requestId > 0 && panel.pendingRequestId == requestId) {
                    panel.pendingRequestId = 0;
                }
                panel.statusLabel->setText(message);
                appendLog(QStringLiteral("%1错误：%2")
                              .arg(panel.profile->id, message), true);
                updateUi();
            }, Qt::QueuedConnection);
    camera.thread.start();
}

void ExposureTestWindow::shutdownCameraWorkers() {
    if (shuttingDown_) return;
    shuttingDown_ = true;
    updateUi();
    for (std::unique_ptr<CameraPanel>& camera : cameras_) {
        if (!camera || !camera->worker || !camera->thread.isRunning()) continue;
        QMetaObject::invokeMethod(camera->worker, "disconnectCamera",
                                  Qt::BlockingQueuedConnection);
        camera->thread.quit();
        if (!camera->thread.wait(5000)) camera->thread.wait();
        camera->worker = nullptr;
        camera->connected = false;
        camera->busy = false;
        camera->pendingRequestId = 0;
    }
}

void ExposureTestWindow::connectCamera(int cameraIndex) {
    CameraPanel& camera = *cameras_[cameraIndex];
    if (!camera.worker || !camera.thread.isRunning() || camera.busy || camera.connected) return;
    const QString ip = camera.ipEdit->text().trimmed();
    camera.statusLabel->setText(QStringLiteral("正在连接 %1…").arg(ip));
    QMetaObject::invokeMethod(camera.worker, "connectCamera", Qt::QueuedConnection,
                              Q_ARG(QString, ip));
    appendLog(QStringLiteral("%1 请求连接 %2").arg(camera.profile->id, ip));
}

void ExposureTestWindow::disconnectCamera(int cameraIndex) {
    CameraPanel& camera = *cameras_[cameraIndex];
    if (!camera.worker || !camera.thread.isRunning() || camera.busy) return;
    QMetaObject::invokeMethod(camera.worker, "disconnectCamera", Qt::QueuedConnection);
}

void ExposureTestWindow::captureCamera(int cameraIndex) {
    CameraPanel& camera = *cameras_[cameraIndex];
    if (!camera.worker || !camera.connected || camera.busy ||
        camera.pendingRequestId != 0) {
        return;
    }
    const QString output = outputDirectoryEdit_->text().trimmed();
    if (output.isEmpty()) {
        appendLog(QStringLiteral("输出目录为空，不能拍摄"), true);
        return;
    }
    if (!QDir(output).exists() && !QDir().mkpath(output)) {
        appendLog(QStringLiteral("无法创建输出目录：%1").arg(output), true);
        return;
    }

    camera.pendingRequestId = ++nextRequestId_;
    if (nextRequestId_ <= 0) {
        nextRequestId_ = 1;
        camera.pendingRequestId = 1;
    }
    const int requestId = camera.pendingRequestId;
    camera.statusLabel->setText(QStringLiteral(
        "正在拍摄：曝光 %1 us，增益 %2 dB…")
        .arg(camera.exposureSpin->value(), 0, 'f', 3)
        .arg(camera.gainSpin->value(), 0, 'f', 3));
    QMetaObject::invokeMethod(camera.worker, "captureSingle", Qt::QueuedConnection,
                              Q_ARG(int, requestId),
                              Q_ARG(double, camera.exposureSpin->value()),
                              Q_ARG(double, camera.gainSpin->value()),
                              Q_ARG(int, camera.timeoutSpin->value()));
    updateUi();
}

void ExposureTestWindow::connectBothCameras() {
    if (cameras_[0]->ipEdit->text().trimmed() ==
        cameras_[1]->ipEdit->text().trimmed()) {
        appendLog(QStringLiteral("两台相机 IP 不能相同"), true);
        return;
    }
    for (int index = 0; index < kCameraCount; ++index) connectCamera(index);
}

void ExposureTestWindow::disconnectBothCameras() {
    for (int index = 0; index < kCameraCount; ++index) disconnectCamera(index);
}

void ExposureTestWindow::captureBothCameras() {
    if (!cameras_[0]->connected || !cameras_[1]->connected) {
        appendLog(QStringLiteral("两台相机必须全部连接后才能同时拍摄"), true);
        return;
    }
    for (int index = 0; index < kCameraCount; ++index) captureCamera(index);
}

void ExposureTestWindow::chooseOutputDirectory() {
    const QString selected = QFileDialog::getExistingDirectory(
        this, QStringLiteral("选择曝光测试图片保存目录"),
        outputDirectoryEdit_->text().trimmed());
    if (!selected.isEmpty()) outputDirectoryEdit_->setText(QDir::cleanPath(selected));
}

void ExposureTestWindow::openOutputDirectory() {
    const QString output = outputDirectoryEdit_->text().trimmed();
    if (!QDir(output).exists() && !QDir().mkpath(output)) {
        appendLog(QStringLiteral("无法创建输出目录：%1").arg(output), true);
        return;
    }
    QDesktopServices::openUrl(QUrl::fromLocalFile(QDir(output).absolutePath()));
}

void ExposureTestWindow::applyLaserSelection() {
    if (!laserController_) return;
    const bool laser450 = laser450Check_->isChecked();
    const bool laser650 = laser650Check_->isChecked();
    if (laser450 && laser650) laserController_->setBoth();
    else if (laser450) laserController_->set450();
    else if (laser650) laserController_->set650();
    else laserController_->off();
    appendLog(QStringLiteral("已提交激光状态：450=%1，650=%2")
                  .arg(laser450 ? QStringLiteral("ON") : QStringLiteral("OFF"),
                       laser650 ? QStringLiteral("ON") : QStringLiteral("OFF")));
}

void ExposureTestWindow::turnLasersOff() {
    laser450Check_->setChecked(false);
    laser650Check_->setChecked(false);
    if (laserController_) laserController_->requestOffTracked();
    appendLog(QStringLiteral("已请求两路 TTL 关闭"));
}

void ExposureTestWindow::updateUi() {
    const bool active = !shuttingDown_;
    bool allConnected = true;
    bool anyConnected = false;
    bool anyBusy = false;
    for (std::unique_ptr<CameraPanel>& camera : cameras_) {
        if (!camera) continue;
        const bool pending = camera->pendingRequestId != 0;
        camera->ipEdit->setEnabled(active && !camera->connected && !camera->busy);
        camera->connectButton->setEnabled(
            active && !camera->connected && !camera->busy);
        camera->disconnectButton->setEnabled(
            active && camera->connected && !camera->busy && !pending);
        camera->exposureSpin->setEnabled(active && !camera->busy && !pending);
        camera->gainSpin->setEnabled(active && !camera->busy && !pending);
        camera->timeoutSpin->setEnabled(active && !camera->busy && !pending);
        camera->captureButton->setEnabled(
            active && camera->connected && !camera->busy && !pending);
        allConnected = allConnected && camera->connected;
        anyConnected = anyConnected || camera->connected;
        anyBusy = anyBusy || camera->busy || pending;
    }
    outputDirectoryEdit_->setEnabled(active && !anyBusy);
    chooseOutputButton_->setEnabled(active && !anyBusy);
    openOutputButton_->setEnabled(active);
    connectBothButton_->setEnabled(active && !allConnected && !anyBusy);
    disconnectBothButton_->setEnabled(active && anyConnected && !anyBusy);
    captureBothButton_->setEnabled(active && allConnected && !anyBusy);

    const bool laserReady =
        laserConnectionState_ == LineLaserConnectionState::Ready;
    const bool laserTransportActive =
        laserConnectionState_ == LineLaserConnectionState::Connecting ||
        laserConnectionState_ == LineLaserConnectionState::Ready ||
        laserConnectionState_ == LineLaserConnectionState::CommandPending ||
        laserConnectionState_ == LineLaserConnectionState::Disconnecting;
    laserConnectButton_->setEnabled(active && !laserTransportActive);
    laserDisconnectButton_->setEnabled(active && laserTransportActive);
    laser450Check_->setEnabled(active && laserReady);
    laser650Check_->setEnabled(active && laserReady);
    applyLaserButton_->setEnabled(active && laserReady);
    lasersOffButton_->setEnabled(active && laserTransportActive);
}

void ExposureTestWindow::appendLog(const QString& message, bool error) {
    if (!logEdit_) return;
    const QString prefix = error ? QStringLiteral("错误") : QStringLiteral("信息");
    logEdit_->appendPlainText(QStringLiteral("[%1] [%2] %3")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")),
             prefix, message));
}

QString ExposureTestWindow::defaultOutputDirectory() const {
#if defined(HIK_CALIBRATION_SOURCE_DIR)
    const QDir source(QStringLiteral(HIK_CALIBRATION_SOURCE_DIR));
#else
    const QDir source(QDir::currentPath());
#endif
    return source.absoluteFilePath(QStringLiteral("data/exposure_tests/session_%1")
        .arg(QDateTime::currentDateTime().toString(
            QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
}

QString ExposureTestWindow::laserStateName(LineLaserState state) const {
    switch (state) {
    case LineLaserState::Off: return QStringLiteral("off");
    case LineLaserState::Laser450: return QStringLiteral("laser450");
    case LineLaserState::Laser650: return QStringLiteral("laser650");
    case LineLaserState::Both: return QStringLiteral("both");
    case LineLaserState::Unknown: break;
    }
    return QStringLiteral("unknown");
}

void ExposureTestWindow::closeEvent(QCloseEvent* event) {
    if (laserController_) laserController_->requestOffTracked();
    shutdownCameraWorkers();
    event->accept();
}
