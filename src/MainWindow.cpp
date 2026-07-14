#include "MainWindow.h"
#include "ExtrinsicCalibrationWidget.h"
#include "RobotControlWidget.h"
#include "RgbImageLabel.h"
#include "ScanWorker.h"
#include "WeldSeamWidget.h"

#include <QPushButton>
#include <QPlainTextEdit>
#include <QLabel>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QDateTime>
#include <QDir>
#include <QCoreApplication>
#include <QCloseEvent>
#include <QFileInfo>
#include <QMessageBox>
#include <QMetaType>
#include <QTabWidget>
#include <QSizePolicy>
#include <QSplitter>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>");
    qRegisterMetaType<QVector<double>>("QVector<double>");

    setWindowTitle("Vizum 线扫建图");
    resize(1200, 760);

    auto* tabs = new QTabWidget(this);
    auto* scanTab = new QWidget(this);
    auto* root = new QVBoxLayout(scanTab);

    // Status bar (top)
    m_statusLabel = new QLabel("未连接", this);
    m_statusLabel->setStyleSheet("color:#b00; font-weight:bold;");
    root->addWidget(m_statusLabel);

    auto* group1 = new QGroupBox("设备", this);
    auto* l1 = new QHBoxLayout(group1);
    m_btnConnect = new QPushButton("连接设备", this);
    m_btnDisconnect = new QPushButton("关闭设备", this);
    m_btnSoftReset = new QPushButton("软复位", this);
    m_btnReboot = new QPushButton("重启设备", this);
    m_btnGetVersions = new QPushButton("获取版本号", this);
    l1->addWidget(m_btnConnect);
    l1->addWidget(m_btnDisconnect);
    l1->addWidget(m_btnSoftReset);
    l1->addWidget(m_btnReboot);
    l1->addWidget(m_btnGetVersions);
    root->addWidget(group1);

    auto* configGroup = new QGroupBox("Vizum配置", this);
    auto* configLayout = new QHBoxLayout(configGroup);
    m_vizumConfigCombo = new QComboBox(this);
    m_vizumConfigCombo->setMinimumContentsLength(36);
    m_btnRefreshVizumConfig = new QPushButton("刷新", this);
    m_btnLoadVizumConfig = new QPushButton("加载配置", this);
    m_autoLoadVizumConfigCheck = new QCheckBox("连接后自动加载", this);
    m_autoLoadVizumConfigCheck->setChecked(true);
    configLayout->addWidget(m_vizumConfigCombo, 1);
    configLayout->addWidget(m_btnRefreshVizumConfig);
    configLayout->addWidget(m_btnLoadVizumConfig);
    configLayout->addWidget(m_autoLoadVizumConfigCheck);
    root->addWidget(configGroup);

    auto* group2 = new QGroupBox("防尘盖", this);
    auto* l2 = new QHBoxLayout(group2);
    m_btnOpenCover = new QPushButton("开盖", this);
    m_btnCloseCover = new QPushButton("关盖", this);
    l2->addWidget(m_btnOpenCover);
    l2->addWidget(m_btnCloseCover);
    root->addWidget(group2);

    auto* group3 = new QGroupBox("扫描", this);
    auto* l3 = new QHBoxLayout(group3);
    m_btnScan = new QPushButton("线扫建图并保存 PLY", this);
    m_btnScan->setMinimumHeight(40);
    l3->addWidget(m_btnScan);
    root->addWidget(group3);

    auto* lowerSplitter = new QSplitter(Qt::Horizontal, this);

    auto* group4 = new QGroupBox("图像预览", this);
    auto* l4 = new QVBoxLayout(group4);
    auto* rgbTop = new QHBoxLayout;
    m_btnGrabRgb = new QPushButton("获取 RGB 图片", this);
    m_btnGrabLeftEye = new QPushButton("获取左目灰度图", this);
    m_btnCalibrateLeftEye = new QPushButton("左目 ChArUco 外参", this);
    m_rgbInfoLabel = new QLabel("未获取图像", this);
    m_rgbInfoLabel->setWordWrap(true);
    rgbTop->addWidget(m_btnGrabRgb);
    rgbTop->addWidget(m_btnGrabLeftEye);
    rgbTop->addWidget(m_btnCalibrateLeftEye);
    rgbTop->addWidget(m_rgbInfoLabel, 1);
    l4->addLayout(rgbTop);
    m_rgbImageLabel = new RgbImageLabel(this);
    m_rgbImageLabel->setText("图像显示区域");
    m_rgbImageLabel->setMinimumSize(520, 520);
    m_rgbImageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    l4->addWidget(m_rgbImageLabel, 1);
    lowerSplitter->addWidget(group4);

    auto* logGroup = new QGroupBox("日志", this);
    auto* logLayout = new QVBoxLayout(logGroup);
    m_logView = new QPlainTextEdit(this);
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(2000);
    logLayout->addWidget(m_logView);
    lowerSplitter->addWidget(logGroup);
    lowerSplitter->setStretchFactor(0, 3);
    lowerSplitter->setStretchFactor(1, 2);
    root->addWidget(lowerSplitter, 1);

    m_weldWidget = new WeldSeamWidget(this);
    m_calibWidget = new ExtrinsicCalibrationWidget(this);
    auto* robotWidget = new RobotControlWidget(this);
    tabs->addTab(scanTab, "线扫建图");
    tabs->addTab(m_weldWidget, "焊缝拟合");
    tabs->addTab(m_calibWidget, "外参标定");
    tabs->addTab(robotWidget, "机械臂控制");
    setCentralWidget(tabs);

    connect(m_weldWidget, &WeldSeamWidget::cameraLineChanged,
            robotWidget, &RobotControlWidget::setCameraLineFromFit);

    // --- Worker thread ---
    m_worker = new ScanWorker;            // do NOT pass parent: it gets moved to thread
    m_worker->moveToThread(&m_workerThread);
    m_workerThread.start();

    connect(&m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);

    connect(m_worker, &ScanWorker::log, this, &MainWindow::onLog, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::connectionChanged, this, &MainWindow::onConnectionChanged, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::scanFinished, this, &MainWindow::onScanFinished, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::busyChanged, this, &MainWindow::onBusyChanged, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::rgbImageReady, this, &MainWindow::onRgbImageReady, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::leftEyeImageReady, this, &MainWindow::onLeftEyeImageReady, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::leftRightEyeImagesSaved,
            robotWidget, &RobotControlWidget::onLeftRightEyeCaptureSaved, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::rgbLineMapped, this, &MainWindow::onRgbLineMapped, Qt::QueuedConnection);
    m_calibWidget->setWorker(m_worker);

    connect(this, &MainWindow::reqInit, m_worker, &ScanWorker::initSdk, Qt::QueuedConnection);
    connect(this, &MainWindow::reqConnect, m_worker, &ScanWorker::connectDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqDisconnect, m_worker, &ScanWorker::disconnectDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqSoftReset, m_worker, &ScanWorker::softResetDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqReboot, m_worker, &ScanWorker::rebootDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqDestroy, m_worker, &ScanWorker::destroySdk, Qt::QueuedConnection);
    connect(this, &MainWindow::reqGetVersions, m_worker, &ScanWorker::getVersions, Qt::QueuedConnection);
    connect(this, &MainWindow::reqOpenCover, m_worker, &ScanWorker::openCover, Qt::QueuedConnection);
    connect(this, &MainWindow::reqCloseCover, m_worker, &ScanWorker::closeCover, Qt::QueuedConnection);
    connect(this, &MainWindow::reqScan, m_worker, &ScanWorker::startScanAndSave, Qt::QueuedConnection);
    connect(this, &MainWindow::reqGrabRgb, m_worker, &ScanWorker::grabRgbImage, Qt::QueuedConnection);
    connect(this, &MainWindow::reqGrabLeftEye, m_worker, &ScanWorker::grabLeftEyeImage, Qt::QueuedConnection);
    connect(this, &MainWindow::reqCalibrateLeftEye, m_worker, &ScanWorker::calibrateLeftEyeCharuco, Qt::QueuedConnection);
    connect(this, &MainWindow::reqLoadVizumConfig, m_worker, &ScanWorker::loadVizumConfig, Qt::QueuedConnection);
    connect(this, &MainWindow::reqMapRgbLine, m_worker, &ScanWorker::mapRgbLineTo3D, Qt::QueuedConnection);
    connect(robotWidget, &RobotControlWidget::requestLeftRightEyeCapture,
            m_worker, &ScanWorker::saveLeftRightEyeImages, Qt::QueuedConnection);

    // --- Button signals ---
    connect(m_btnConnect, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
    connect(m_btnDisconnect, &QPushButton::clicked, this, &MainWindow::onDisconnectClicked);
    connect(m_btnSoftReset, &QPushButton::clicked, this, &MainWindow::onSoftResetClicked);
    connect(m_btnReboot, &QPushButton::clicked, this, &MainWindow::onRebootClicked);
    connect(m_btnGetVersions, &QPushButton::clicked, this, &MainWindow::onGetVersionsClicked);
    connect(m_btnOpenCover, &QPushButton::clicked, this, &MainWindow::onOpenCoverClicked);
    connect(m_btnCloseCover, &QPushButton::clicked, this, &MainWindow::onCloseCoverClicked);
    connect(m_btnScan, &QPushButton::clicked, this, &MainWindow::onScanClicked);
    connect(m_btnGrabRgb, &QPushButton::clicked, this, &MainWindow::onGrabRgbClicked);
    connect(m_btnGrabLeftEye, &QPushButton::clicked, this, &MainWindow::onGrabLeftEyeClicked);
    connect(m_btnCalibrateLeftEye, &QPushButton::clicked, this, &MainWindow::onCalibrateLeftEyeClicked);
    connect(m_btnRefreshVizumConfig, &QPushButton::clicked, this, &MainWindow::refreshVizumConfigList);
    connect(m_btnLoadVizumConfig, &QPushButton::clicked, this, &MainWindow::onLoadVizumConfigClicked);
    connect(m_rgbImageLabel, &RgbImageLabel::lineSelected, this, &MainWindow::onRgbLineSelected);

    refreshVizumConfigList();
    onConnectionChanged(false);

    // Auto-init SDK
    emit reqInit();
}

MainWindow::~MainWindow() {
    // Ensure worker stopped & destroyed cleanly
    if (m_workerThread.isRunning()) {
        // synchronously ask worker to release SDK resources
        QMetaObject::invokeMethod(m_worker, "destroySdk", Qt::BlockingQueuedConnection);
        m_workerThread.quit();
        m_workerThread.wait(3000);
    }
}

void MainWindow::closeEvent(QCloseEvent* ev) {
    // Stop worker before window dies
    if (m_workerThread.isRunning()) {
        QMetaObject::invokeMethod(m_worker, "destroySdk", Qt::BlockingQueuedConnection);
        m_workerThread.quit();
        m_workerThread.wait(3000);
    }
    ev->accept();
}

void MainWindow::onLog(QString msg) {
    m_logView->appendPlainText(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
        .arg(msg));
}

void MainWindow::onConnectionChanged(bool connected) {
    const bool becameConnected = connected && !m_connected;
    m_connected = connected;
    if (connected) {
        m_statusLabel->setText("已连接");
        m_statusLabel->setStyleSheet("color:#080; font-weight:bold;");
    } else {
        m_statusLabel->setText("未连接");
        m_statusLabel->setStyleSheet("color:#b00; font-weight:bold;");
    }
    m_btnConnect->setEnabled(!connected);
    m_btnDisconnect->setEnabled(connected);
    m_btnSoftReset->setEnabled(connected);
    m_btnReboot->setEnabled(connected);
    m_btnGetVersions->setEnabled(connected);
    m_btnOpenCover->setEnabled(connected);
    m_btnCloseCover->setEnabled(connected);
    m_btnScan->setEnabled(connected);
    m_btnGrabRgb->setEnabled(connected);
    m_btnGrabLeftEye->setEnabled(connected);
    m_btnCalibrateLeftEye->setEnabled(connected);
    m_btnLoadVizumConfig->setEnabled(connected && m_vizumConfigCombo->count() > 0);

    if (becameConnected && m_autoLoadVizumConfigCheck->isChecked()) {
        const QString path = currentVizumConfigPath();
        if (!path.isEmpty()) {
            onLog(QStringLiteral("连接后自动加载 Vizum 配置: %1").arg(QFileInfo(path).fileName()));
            emit reqLoadVizumConfig(path);
        }
    }
}

void MainWindow::onScanFinished(bool ok, QString path) {
    if (ok) {
        onLog(QStringLiteral("点云已保存: %1").arg(path));
        if (m_weldWidget) {
            m_weldWidget->loadCloudFile(path);
            onLog(QStringLiteral("已在焊缝拟合页展示点云: %1").arg(path));
        }
    } else {
        onLog(QStringLiteral("扫描未保存有效点云"));
    }
}

void MainWindow::onRgbImageReady(QImage image, QString desc) {
    if (image.isNull()) {
        onLog("RGB 图片为空");
        return;
    }
    const QString path = defaultRgbPath();
    if (image.save(path)) {
        m_rgbInfoLabel->setText(QStringLiteral("%1 -> %2").arg(desc, QFileInfo(path).fileName()));
        m_rgbInfoLabel->setToolTip(path);
        onLog(QStringLiteral("RGB 图片已保存: %1").arg(path));
    } else {
        m_rgbInfoLabel->setText(desc);
        m_rgbInfoLabel->setToolTip(QString());
        onLog(QStringLiteral("RGB 图片保存失败: %1").arg(path));
    }

    m_rgbImageLabel->setImage(image);
    onLog(QStringLiteral("RGB 图片已显示: %1").arg(desc));
}

void MainWindow::onLeftEyeImageReady(QImage image, QString desc) {
    if (image.isNull()) {
        onLog("左目图像为空");
        return;
    }
    const QString path = defaultLeftEyePath();
    if (image.save(path)) {
        m_rgbInfoLabel->setText(QStringLiteral("%1 -> %2").arg(desc, QFileInfo(path).fileName()));
        m_rgbInfoLabel->setToolTip(path);
        onLog(QStringLiteral("左目图像已保存: %1").arg(path));
    } else {
        m_rgbInfoLabel->setText(desc);
        m_rgbInfoLabel->setToolTip(QString());
        onLog(QStringLiteral("左目图像保存失败: %1").arg(path));
    }

    m_rgbImageLabel->setImage(image);
    onLog(QStringLiteral("左目图像已显示: %1").arg(desc));
}

void MainWindow::onRgbLineSelected(QPoint start, QPoint end) {
    onLog(QStringLiteral("RGB 图上线段: (%1,%2) -> (%3,%4)，开始映射到 3D")
          .arg(start.x()).arg(start.y()).arg(end.x()).arg(end.y()));
    emit reqMapRgbLine(start, end, 300);
}

void MainWindow::onRgbLineMapped(QVector<QVector3D> points, QString desc) {
    onLog(desc);
    if (!m_weldWidget) {
        return;
    }
    m_weldWidget->showMappedRgbLine(points, desc);
}

void MainWindow::setBusy(bool busy) {
    if (busy) {
        m_btnConnect->setEnabled(false);
        m_btnDisconnect->setEnabled(false);
        m_btnSoftReset->setEnabled(false);
        m_btnReboot->setEnabled(false);
        m_btnGetVersions->setEnabled(false);
        m_btnOpenCover->setEnabled(false);
        m_btnCloseCover->setEnabled(false);
        m_btnScan->setEnabled(false);
        m_btnGrabRgb->setEnabled(false);
        m_btnGrabLeftEye->setEnabled(false);
        m_btnCalibrateLeftEye->setEnabled(false);
        m_btnLoadVizumConfig->setEnabled(false);
    } else {
        // Re-evaluate based on connection state
        onConnectionChanged(m_connected);
    }
}

void MainWindow::onBusyChanged(bool busy) {
    setBusy(busy);
}

QString MainWindow::defaultPlyPath() {
    QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    return QStringLiteral("%1/scan_%2_%3.ply").arg(dataDirPath()).arg(stamp).arg(++m_scanCounter);
}

QString MainWindow::defaultRgbPath() {
    QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    return QStringLiteral("%1/rgb_%2_%3.png").arg(dataDirPath()).arg(stamp).arg(++m_rgbCounter);
}

QString MainWindow::defaultLeftEyePath() {
    QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    return QStringLiteral("%1/left_%2_%3.png").arg(dataDirPath()).arg(stamp).arg(++m_leftEyeCounter);
}

QString MainWindow::dataDirPath() {
    QDir dir(QCoreApplication::applicationDirPath());
    if (dir.dirName() == "build") {
        dir.cdUp();
    }
    if (!dir.exists("data")) {
        dir.mkdir("data");
    }
    dir.cd("data");
    return dir.absolutePath();
}

QString MainWindow::vizumConfigDirPath() {
    QDir dir(QCoreApplication::applicationDirPath());
    if (dir.dirName() == "build") {
        dir.cdUp();
    }
    return dir.absoluteFilePath(QStringLiteral("config/VizumConfig"));
}

QString MainWindow::currentVizumConfigPath() const {
    if (!m_vizumConfigCombo || m_vizumConfigCombo->currentIndex() < 0) {
        return {};
    }
    return m_vizumConfigCombo->currentData().toString();
}

void MainWindow::refreshVizumConfigList() {
    if (!m_vizumConfigCombo) {
        return;
    }

    const QString previous = currentVizumConfigPath();
    const QDir dir(vizumConfigDirPath());
    const QFileInfoList files = dir.entryInfoList(QStringList{QStringLiteral("*.ini")},
                                                  QDir::Files | QDir::Readable,
                                                  QDir::Name);

    m_vizumConfigCombo->clear();
    for (const QFileInfo& info : files) {
        m_vizumConfigCombo->addItem(info.fileName(), info.absoluteFilePath());
    }

    int index = previous.isEmpty() ? -1 : m_vizumConfigCombo->findData(previous);
    if (index < 0) {
        index = m_vizumConfigCombo->findText(QStringLiteral("SavedEnvironmentConfigPara_zhongshiye_tuxiangxia_shinei5.ini"));
    }
    if (index < 0 && m_vizumConfigCombo->count() > 0) {
        index = 0;
    }
    if (index >= 0) {
        m_vizumConfigCombo->setCurrentIndex(index);
    }

    m_btnLoadVizumConfig->setEnabled(m_connected && m_vizumConfigCombo->count() > 0);
    if (files.isEmpty()) {
        onLog(QStringLiteral("未找到 Vizum 配置文件: %1/*.ini").arg(dir.absolutePath()));
    }
}

void MainWindow::onConnectClicked() { emit reqConnect(); }
void MainWindow::onDisconnectClicked() { emit reqDisconnect(); }
void MainWindow::onSoftResetClicked() { emit reqSoftReset(); }
void MainWindow::onRebootClicked() { emit reqReboot(); }
void MainWindow::onGetVersionsClicked() { emit reqGetVersions(); }
void MainWindow::onOpenCoverClicked() { emit reqOpenCover(); }
void MainWindow::onCloseCoverClicked() { emit reqCloseCover(); }
void MainWindow::onGrabRgbClicked() { emit reqGrabRgb(); }
void MainWindow::onGrabLeftEyeClicked() { emit reqGrabLeftEye(); }
void MainWindow::onCalibrateLeftEyeClicked() { emit reqCalibrateLeftEye(); }

void MainWindow::onLoadVizumConfigClicked() {
    const QString path = currentVizumConfigPath();
    if (path.isEmpty()) {
        onLog("请先选择 Vizum 配置文件");
        return;
    }
    onLog(QStringLiteral("开始加载 Vizum 配置: %1").arg(QFileInfo(path).fileName()));
    emit reqLoadVizumConfig(path);
}

void MainWindow::onScanClicked() {
    const QString path = defaultPlyPath();
    onLog(QStringLiteral("开始扫描，PLY 将保存到: %1").arg(path));
    emit reqScan(path);
}
