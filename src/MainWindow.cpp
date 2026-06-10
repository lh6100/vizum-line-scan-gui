#include "MainWindow.h"
#include "RobotControlWidget.h"
#include "ScanWorker.h"
#include "WeldSeamWidget.h"

#include <QPushButton>
#include <QPlainTextEdit>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QDateTime>
#include <QDir>
#include <QCloseEvent>
#include <QMessageBox>
#include <QTabWidget>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
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
    m_btnReboot = new QPushButton("重启设备", this);
    m_btnGetVersions = new QPushButton("获取版本号", this);
    l1->addWidget(m_btnConnect);
    l1->addWidget(m_btnDisconnect);
    l1->addWidget(m_btnReboot);
    l1->addWidget(m_btnGetVersions);
    root->addWidget(group1);

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

    m_logView = new QPlainTextEdit(this);
    m_logView->setReadOnly(true);
    m_logView->setMaximumBlockCount(2000);
    root->addWidget(m_logView, 1);

    auto* weldWidget = new WeldSeamWidget(this);
    auto* robotWidget = new RobotControlWidget(this);
    tabs->addTab(scanTab, "线扫建图");
    tabs->addTab(weldWidget, "焊缝拟合");
    tabs->addTab(robotWidget, "机械臂控制");
    setCentralWidget(tabs);

    connect(weldWidget, &WeldSeamWidget::cameraLineChanged,
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

    connect(this, &MainWindow::reqInit, m_worker, &ScanWorker::initSdk, Qt::QueuedConnection);
    connect(this, &MainWindow::reqConnect, m_worker, &ScanWorker::connectDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqDisconnect, m_worker, &ScanWorker::disconnectDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqReboot, m_worker, &ScanWorker::rebootDevice, Qt::QueuedConnection);
    connect(this, &MainWindow::reqDestroy, m_worker, &ScanWorker::destroySdk, Qt::QueuedConnection);
    connect(this, &MainWindow::reqGetVersions, m_worker, &ScanWorker::getVersions, Qt::QueuedConnection);
    connect(this, &MainWindow::reqOpenCover, m_worker, &ScanWorker::openCover, Qt::QueuedConnection);
    connect(this, &MainWindow::reqCloseCover, m_worker, &ScanWorker::closeCover, Qt::QueuedConnection);
    connect(this, &MainWindow::reqScan, m_worker, &ScanWorker::startScanAndSave, Qt::QueuedConnection);

    // --- Button signals ---
    connect(m_btnConnect, &QPushButton::clicked, this, &MainWindow::onConnectClicked);
    connect(m_btnDisconnect, &QPushButton::clicked, this, &MainWindow::onDisconnectClicked);
    connect(m_btnReboot, &QPushButton::clicked, this, &MainWindow::onRebootClicked);
    connect(m_btnGetVersions, &QPushButton::clicked, this, &MainWindow::onGetVersionsClicked);
    connect(m_btnOpenCover, &QPushButton::clicked, this, &MainWindow::onOpenCoverClicked);
    connect(m_btnCloseCover, &QPushButton::clicked, this, &MainWindow::onCloseCoverClicked);
    connect(m_btnScan, &QPushButton::clicked, this, &MainWindow::onScanClicked);

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
    m_btnReboot->setEnabled(connected);
    m_btnGetVersions->setEnabled(connected);
    m_btnOpenCover->setEnabled(connected);
    m_btnCloseCover->setEnabled(connected);
    m_btnScan->setEnabled(connected);
}

void MainWindow::onScanFinished(bool ok, QString path) {
    if (ok) {
        onLog(QStringLiteral("点云已保存: %1").arg(path));
    } else {
        onLog(QStringLiteral("扫描未保存有效点云"));
    }
}

void MainWindow::setBusy(bool busy) {
    if (busy) {
        m_btnConnect->setEnabled(false);
        m_btnDisconnect->setEnabled(false);
        m_btnReboot->setEnabled(false);
        m_btnGetVersions->setEnabled(false);
        m_btnOpenCover->setEnabled(false);
        m_btnCloseCover->setEnabled(false);
        m_btnScan->setEnabled(false);
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
    QString dir = QDir::currentPath();
    return QStringLiteral("%1/scan_%2_%3.ply").arg(dir).arg(stamp).arg(++m_scanCounter);
}

void MainWindow::onConnectClicked() { emit reqConnect(); }
void MainWindow::onDisconnectClicked() { emit reqDisconnect(); }
void MainWindow::onRebootClicked() { emit reqReboot(); }
void MainWindow::onGetVersionsClicked() { emit reqGetVersions(); }
void MainWindow::onOpenCoverClicked() { emit reqOpenCover(); }
void MainWindow::onCloseCoverClicked() { emit reqCloseCover(); }

void MainWindow::onScanClicked() {
    QString suggested = defaultPlyPath();
    QString path = QFileDialog::getSaveFileName(this, "保存点云为 PLY", suggested, "PLY (*.ply)");
    if (path.isEmpty()) return;
    if (!path.endsWith(".ply", Qt::CaseInsensitive)) path += ".ply";
    emit reqScan(path);
}
