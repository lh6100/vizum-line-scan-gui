#include "RobotControlWidget.h"
#include "MotionTracePlotWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaType>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTextStream>
#include <QVBoxLayout>

#include <chrono>
#include <cmath>

namespace {

const double kMinTraceSampleIntervalSec = 0.02;

QVector<double> relativeSeries(const QVector<double>& values) {
    QVector<double> output = values;
    if (output.isEmpty()) {
        return output;
    }
    const double origin = output.front();
    for (double& value : output) {
        value -= origin;
    }
    return output;
}

QVector<double> derivativeSeries(const QVector<double>& time, const QVector<double>& values) {
    QVector<double> output(values.size(), 0.0);
    if (time.size() != values.size() || values.size() < 2) {
        return output;
    }
    for (int i = 1; i < values.size(); ++i) {
        const double dt = time[i] - time[i - 1];
        if (dt >= kMinTraceSampleIntervalSec && std::isfinite(dt)) {
            output[i] = (values[i] - values[i - 1]) / dt;
        } else {
            output[i] = output[i - 1];
        }
    }
    output[0] = output[1];
    return output;
}

} // namespace

RobotControlWidget::RobotControlWidget(QWidget* parent)
    : QWidget(parent) {
    qRegisterMetaType<QVector<double>>("QVector<double>");
    loadConfigs();

    auto* outer = new QVBoxLayout(this);
    m_pages = new QTabWidget(this);
    outer->addWidget(m_pages);

    auto* controlPage = new QWidget(this);
    auto* root = new QVBoxLayout(controlPage);

    auto* connGroup = new QGroupBox("机械臂连接与安全", this);
    auto* connGrid = new QGridLayout(connGroup);
    m_ipEdit = new QLineEdit(QString::fromStdString(m_robotConfig.ip), this);
    m_statusLabel = new QLabel("未连接", this);
    m_statusLabel->setStyleSheet("color:#b00; font-weight:bold;");
    m_dryRunCheck = new QCheckBox("干运行：只打印，不发运动", this);
    m_dryRunCheck->setChecked(m_motionConfig.dryRun);
    m_enableMotionCheck = new QCheckBox("允许真运动 MoveL", this);
    m_enableMotionCheck->setChecked(m_motionConfig.enableRobotMotion);

    m_btnConnect = new QPushButton("连接机械臂", this);
    m_btnDisconnect = new QPushButton("断开", this);
    m_btnReadPose = new QPushButton("读取当前法兰/TCP", this);
    m_btnServoOn = new QPushButton("上伺服", this);
    m_btnServoOff = new QPushButton("下伺服", this);
    m_btnAutoMode = new QPushButton("切自动模式", this);
    m_btnResetError = new QPushButton("复位错误", this);
    m_btnStop = new QPushButton("停止运动/急停移动", this);
    m_btnStop->setStyleSheet("background:#b00020; color:white; font-weight:bold;");
    m_btnPause = new QPushButton("暂停运动", this);
    m_btnResume = new QPushButton("继续运动", this);

    connGrid->addWidget(new QLabel("IP", this), 0, 0);
    connGrid->addWidget(m_ipEdit, 0, 1);
    connGrid->addWidget(m_statusLabel, 0, 2);
    connGrid->addWidget(m_btnConnect, 1, 0);
    connGrid->addWidget(m_btnDisconnect, 1, 1);
    connGrid->addWidget(m_btnReadPose, 1, 2);
    connGrid->addWidget(m_btnServoOn, 2, 0);
    connGrid->addWidget(m_btnServoOff, 2, 1);
    connGrid->addWidget(m_btnAutoMode, 2, 2);
    connGrid->addWidget(m_btnResetError, 3, 0);
    connGrid->addWidget(m_btnPause, 3, 1);
    connGrid->addWidget(m_btnResume, 3, 2);
    connGrid->addWidget(m_btnStop, 4, 0, 1, 2);
    connGrid->addWidget(m_dryRunCheck, 4, 2);
    connGrid->addWidget(m_enableMotionCheck, 4, 3);
    root->addWidget(connGroup);

    auto* lineGroup = new QGroupBox("焊缝 Camera 线段端点(mm)", this);
    auto* lineGrid = new QGridLayout(lineGroup);
    m_startX = makeCoordSpin(160.575);
    m_startY = makeCoordSpin(149.285);
    m_startZ = makeCoordSpin(374.518);
    m_endX = makeCoordSpin(140.663);
    m_endY = makeCoordSpin(148.809);
    m_endZ = makeCoordSpin(375.435);
    lineGrid->addWidget(new QLabel("Start X/Y/Z", this), 0, 0);
    lineGrid->addWidget(m_startX, 0, 1);
    lineGrid->addWidget(m_startY, 0, 2);
    lineGrid->addWidget(m_startZ, 0, 3);
    lineGrid->addWidget(new QLabel("End X/Y/Z", this), 1, 0);
    lineGrid->addWidget(m_endX, 1, 1);
    lineGrid->addWidget(m_endY, 1, 2);
    lineGrid->addWidget(m_endZ, 1, 3);
    root->addWidget(lineGroup);

    auto* poseGroup = new QGroupBox("当前机器人位姿(mm/deg)", this);
    auto* poseGrid = new QGridLayout(poseGroup);
    m_flangeX = makePoseSpin(136.008);
    m_flangeY = makePoseSpin(-469.288);
    m_flangeZ = makePoseSpin(475.017);
    m_flangeRx = makePoseSpin(-175.958);
    m_flangeRy = makePoseSpin(-13.814);
    m_flangeRz = makePoseSpin(-151.632);
    m_tcpX = makePoseSpin(165.856);
    m_tcpY = makePoseSpin(-703.278);
    m_tcpZ = makePoseSpin(131.029);
    m_tcpRx = makePoseSpin(15.206);
    m_tcpRy = makePoseSpin(-22.000);
    m_tcpRz = makePoseSpin(-154.292);
    poseGrid->addWidget(new QLabel("Flange", this), 0, 0);
    poseGrid->addWidget(m_flangeX, 0, 1);
    poseGrid->addWidget(m_flangeY, 0, 2);
    poseGrid->addWidget(m_flangeZ, 0, 3);
    poseGrid->addWidget(m_flangeRx, 0, 4);
    poseGrid->addWidget(m_flangeRy, 0, 5);
    poseGrid->addWidget(m_flangeRz, 0, 6);
    poseGrid->addWidget(new QLabel("TCP", this), 1, 0);
    poseGrid->addWidget(m_tcpX, 1, 1);
    poseGrid->addWidget(m_tcpY, 1, 2);
    poseGrid->addWidget(m_tcpZ, 1, 3);
    poseGrid->addWidget(m_tcpRx, 1, 4);
    poseGrid->addWidget(m_tcpRy, 1, 5);
    poseGrid->addWidget(m_tcpRz, 1, 6);
    root->addWidget(poseGroup);

    auto* motionGroup = new QGroupBox("运动参数与工艺偏移", this);
    auto* motionGrid = new QGridLayout(motionGroup);
    m_safeHeightSpin = makeCoordSpin(m_motionConfig.safeHeightMm);
    m_retractHeightSpin = makeCoordSpin(m_motionConfig.retractHeightMm);
    m_travelVelSpin = makeSpeedSpin(m_motionConfig.travelVel);
    m_weldVelSpin = makeSpeedSpin(m_motionConfig.weldVel);
    m_accSpin = makeSpeedSpin(m_motionConfig.acc);
    m_ovlSpin = makeSpeedSpin(m_motionConfig.ovl);
    m_offsetXSpin = makeCoordSpin(m_motionConfig.processOffset.x);
    m_offsetYSpin = makeCoordSpin(m_motionConfig.processOffset.y);
    m_offsetZSpin = makeCoordSpin(m_motionConfig.processOffset.z);
    m_physicalSpeedModeCheck = new QCheckBox("物理速度(mm/s)", this);
    m_physicalSpeedModeCheck->setChecked(m_motionConfig.physicalSpeedMode);
    m_btnCalculate = new QPushButton("计算并记录焊枪 TCP 点位", this);
    m_btnExecute = new QPushButton("执行当前 MoveL 轨迹", this);
    m_btnExecuteLast = new QPushButton("执行上次轨迹", this);
    m_lastPlanLabel = new QLabel("上次轨迹：未记录", this);
    m_lastPlanLabel->setWordWrap(true);
    motionGrid->addWidget(new QLabel("安全高度/结束抬高(mm)", this), 0, 0);
    motionGrid->addWidget(m_safeHeightSpin, 0, 1);
    motionGrid->addWidget(m_retractHeightSpin, 0, 2);
    motionGrid->addWidget(m_physicalSpeedModeCheck, 0, 3);
    motionGrid->addWidget(new QLabel("移动速度/焊接速度/加速度/倍率", this), 1, 0);
    motionGrid->addWidget(m_travelVelSpin, 1, 1);
    motionGrid->addWidget(m_weldVelSpin, 1, 2);
    motionGrid->addWidget(m_accSpin, 1, 3);
    motionGrid->addWidget(m_ovlSpin, 1, 4);
    motionGrid->addWidget(new QLabel("工艺偏移 X/Y/Z", this), 2, 0);
    motionGrid->addWidget(m_offsetXSpin, 2, 1);
    motionGrid->addWidget(m_offsetYSpin, 2, 2);
    motionGrid->addWidget(m_offsetZSpin, 2, 3);
    motionGrid->addWidget(m_btnCalculate, 3, 0, 1, 2);
    motionGrid->addWidget(m_btnExecute, 3, 2, 1, 2);
    motionGrid->addWidget(m_btnExecuteLast, 3, 4);
    motionGrid->addWidget(m_lastPlanLabel, 4, 0, 1, 5);
    root->addWidget(motionGroup);

    auto* captureGroup = new QGroupBox("线激光固定：机械臂偏移拍照", this);
    auto* captureGrid = new QGridLayout(captureGroup);
    m_captureAxisCombo = new QComboBox(this);
    m_captureAxisCombo->addItem("法兰 X", static_cast<int>(robot_capture::CaptureAxis::X));
    m_captureAxisCombo->addItem("法兰 Y", static_cast<int>(robot_capture::CaptureAxis::Y));
    m_captureAxisCombo->addItem("法兰 Z", static_cast<int>(robot_capture::CaptureAxis::Z));
    m_captureDistanceSpin = makeCoordSpin(20.0);
    m_captureDistanceSpin->setRange(-1000.0, 1000.0);
    m_captureDistanceSpin->setSuffix(" mm");
    m_captureStepSpin = makeCoordSpin(2.0);
    m_captureStepSpin->setRange(0.1, 100.0);
    m_captureStepSpin->setSuffix(" mm");
    m_captureVelSpin = makeSpeedSpin(m_motionConfig.travelVel);
    m_captureFrameRateSpin = makeSpeedSpin(30.0);
    m_captureFrameRateSpin->setRange(1.0, 240.0);
    m_captureFrameRateSpin->setDecimals(0);
    m_captureFrameRateSpin->setSuffix(" fps");
    m_captureExposureSpin = makeSpeedSpin(260.0);
    m_captureExposureSpin->setRange(0.0, 65535.0);
    m_captureExposureSpin->setDecimals(0);
    m_captureExposureSpin->setSuffix(" us");
    m_captureGainSpin = makeSpeedSpin(1.0);
    m_captureGainSpin->setRange(0.0, 255.0);
    m_captureGainSpin->setDecimals(0);
    m_captureKeepLaserOnCheck = new QCheckBox("拍照期间保持线激光开启", this);
    m_captureKeepLaserOnCheck->setChecked(true);
    m_btnOffsetCapture = new QPushButton("执行偏移拍照", this);
    captureGrid->addWidget(new QLabel("偏移方向/总距离/步距", this), 0, 0);
    captureGrid->addWidget(m_captureAxisCombo, 0, 1);
    captureGrid->addWidget(m_captureDistanceSpin, 0, 2);
    captureGrid->addWidget(m_captureStepSpin, 0, 3);
    captureGrid->addWidget(new QLabel("移动速度", this), 0, 4);
    captureGrid->addWidget(m_captureVelSpin, 0, 5);
    captureGrid->addWidget(new QLabel("双目 FPS/曝光/增益", this), 1, 0);
    captureGrid->addWidget(m_captureFrameRateSpin, 1, 1);
    captureGrid->addWidget(m_captureExposureSpin, 1, 2);
    captureGrid->addWidget(m_captureGainSpin, 1, 3);
    captureGrid->addWidget(m_captureKeepLaserOnCheck, 1, 4);
    captureGrid->addWidget(m_btnOffsetCapture, 1, 5);
    root->addWidget(captureGroup);

    m_log = new QPlainTextEdit(this);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(2000);
    root->addWidget(m_log, 1);

    m_tracePage = new QWidget(this);
    auto* traceRoot = new QVBoxLayout(m_tracePage);
    m_traceSummaryLabel = new QLabel("暂无运动轨迹记录", m_tracePage);
    m_traceSummaryLabel->setWordWrap(true);
    m_displacementPlot = new MotionTracePlotWidget("位移曲线", "位移(mm，相对首点)", m_tracePage);
    m_velocityPlot = new MotionTracePlotWidget("速度曲线", "速度(mm/s)", m_tracePage);
    m_accelerationPlot = new MotionTracePlotWidget("加速度曲线", "加速度(mm/s2)", m_tracePage);
    traceRoot->addWidget(m_traceSummaryLabel);
    traceRoot->addWidget(m_displacementPlot, 1);
    traceRoot->addWidget(m_velocityPlot, 1);
    traceRoot->addWidget(m_accelerationPlot, 1);

    m_pages->addTab(controlPage, "运动控制");
    m_pages->addTab(m_tracePage, "轨迹曲线");

    connect(m_btnConnect, &QPushButton::clicked, this, &RobotControlWidget::connectRobot);
    connect(m_btnDisconnect, &QPushButton::clicked, this, &RobotControlWidget::disconnectRobot);
    connect(m_btnReadPose, &QPushButton::clicked, this, &RobotControlWidget::readCurrentPose);
    connect(m_btnServoOn, &QPushButton::clicked, this, &RobotControlWidget::servoOn);
    connect(m_btnServoOff, &QPushButton::clicked, this, &RobotControlWidget::servoOff);
    connect(m_btnAutoMode, &QPushButton::clicked, this, &RobotControlWidget::setAutoMode);
    connect(m_btnResetError, &QPushButton::clicked, this, &RobotControlWidget::resetError);
    connect(m_btnStop, &QPushButton::clicked, this, &RobotControlWidget::stopMotion);
    connect(m_btnPause, &QPushButton::clicked, this, &RobotControlWidget::pauseMotion);
    connect(m_btnResume, &QPushButton::clicked, this, &RobotControlWidget::resumeMotion);
    connect(m_btnCalculate, &QPushButton::clicked, this, &RobotControlWidget::calculateTargets);
    connect(m_btnExecute, &QPushButton::clicked, this, &RobotControlWidget::executeMove);
    connect(m_btnExecuteLast, &QPushButton::clicked, this, &RobotControlWidget::executeLastMove);
    connect(m_btnOffsetCapture, &QPushButton::clicked, this, &RobotControlWidget::executeOffsetCapture);
    connect(m_physicalSpeedModeCheck, &QCheckBox::toggled, this, &RobotControlWidget::updateSpeedModeUi);
    connect(this, &RobotControlWidget::asyncLog, this, &RobotControlWidget::appendLog);
    connect(this, &RobotControlWidget::asyncMotionFinished, this, &RobotControlWidget::onMotionFinished);
    connect(this, &RobotControlWidget::asyncMotionTraceReady, this, &RobotControlWidget::onMotionTraceReady);

    setConnectedUi(false);
    updateSpeedModeUi();
    appendLog("默认干运行。真运动需要取消干运行并勾选允许真运动 MoveL。");
}

RobotControlWidget::~RobotControlWidget() {
    if (m_motionRunning.load()) {
        appendLog("窗口关闭，正在发送 StopMotion。");
        m_stopRequested.store(true);
        m_robot.stopMotion();
    }
    if (m_motionThread.joinable()) {
        m_motionThread.join();
    }
    m_robot.disconnectRobot();
}

void RobotControlWidget::loadConfigs() {
    m_robotConfig = fairino_client::loadRobotConfig("config/robot_config.yaml");
    m_handEyeConfig = weld_motion::loadHandEyeConfig("config/handeye_config.yaml");
    m_toolConfig = weld_motion::loadToolConfig("config/tool_config.yaml");
    m_motionConfig = weld_motion::loadWeldMotionConfig("config/weld_motion_config.yaml");
}

QDoubleSpinBox* RobotControlWidget::makeCoordSpin(double value) {
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(-3000.0, 3000.0);
    spin->setDecimals(3);
    spin->setSingleStep(1.0);
    spin->setValue(value);
    return spin;
}

QDoubleSpinBox* RobotControlWidget::makePoseSpin(double value) {
    auto* spin = makeCoordSpin(value);
    spin->setRange(-5000.0, 5000.0);
    return spin;
}

QDoubleSpinBox* RobotControlWidget::makeSpeedSpin(double value) {
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(0.0, 5000.0);
    spin->setDecimals(3);
    spin->setSingleStep(1.0);
    spin->setValue(value);
    return spin;
}

void RobotControlWidget::appendLog(const QString& msg) {
    m_log->appendPlainText(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
        .arg(msg));
}

void RobotControlWidget::setConnectedUi(bool connected) {
    m_statusLabel->setText(connected ? "已连接" : "未连接");
    m_statusLabel->setStyleSheet(connected ? "color:#080; font-weight:bold;" : "color:#b00; font-weight:bold;");
    m_btnConnect->setEnabled(!connected);
    m_btnDisconnect->setEnabled(connected);
    m_btnReadPose->setEnabled(connected);
    m_btnServoOn->setEnabled(connected);
    m_btnServoOff->setEnabled(connected);
    m_btnAutoMode->setEnabled(connected);
    m_btnResetError->setEnabled(connected);
    m_btnStop->setEnabled(connected);
    m_btnPause->setEnabled(connected);
    m_btnResume->setEnabled(connected);
    if (m_motionRunning.load()) {
        setMotionUi(true);
    }
}

void RobotControlWidget::setMotionUi(bool running) {
    m_btnConnect->setEnabled(!running && !m_robot.isConnected());
    m_btnDisconnect->setEnabled(!running && m_robot.isConnected());
    m_btnReadPose->setEnabled(!running && m_robot.isConnected());
    m_btnServoOn->setEnabled(!running && m_robot.isConnected());
    m_btnServoOff->setEnabled(!running && m_robot.isConnected());
    m_btnAutoMode->setEnabled(!running && m_robot.isConnected());
    m_btnResetError->setEnabled(!running && m_robot.isConnected());
    m_btnPause->setEnabled(!running && m_robot.isConnected());
    m_btnResume->setEnabled(!running && m_robot.isConnected());
    m_btnCalculate->setEnabled(!running);
    m_btnExecute->setEnabled(!running);
    m_btnExecuteLast->setEnabled(!running && m_hasLastPlan);
    m_btnOffsetCapture->setEnabled(!running);
    m_btnStop->setEnabled(m_robot.isConnected() || running);
    m_dryRunCheck->setEnabled(!running);
    m_enableMotionCheck->setEnabled(!running);
    m_physicalSpeedModeCheck->setEnabled(!running);
    m_captureAxisCombo->setEnabled(!running);
    m_captureDistanceSpin->setEnabled(!running);
    m_captureStepSpin->setEnabled(!running);
    m_captureVelSpin->setEnabled(!running);
    m_captureFrameRateSpin->setEnabled(!running);
    m_captureExposureSpin->setEnabled(!running);
    m_captureGainSpin->setEnabled(!running);
    m_captureKeepLaserOnCheck->setEnabled(!running);
}

void RobotControlWidget::syncMotionConfigFromUi() {
    m_motionConfig.dryRun = m_dryRunCheck->isChecked();
    m_motionConfig.enableRobotMotion = m_enableMotionCheck->isChecked();
    m_motionConfig.physicalSpeedMode = m_physicalSpeedModeCheck->isChecked();
    m_motionConfig.safeHeightMm = m_safeHeightSpin->value();
    m_motionConfig.retractHeightMm = m_retractHeightSpin->value();
    m_motionConfig.travelVel = m_travelVelSpin->value();
    m_motionConfig.weldVel = m_weldVelSpin->value();
    m_motionConfig.acc = m_accSpin->value();
    m_motionConfig.ovl = m_ovlSpin->value();
    m_motionConfig.processOffset = {m_offsetXSpin->value(), m_offsetYSpin->value(), m_offsetZSpin->value()};
}

void RobotControlWidget::updateSpeedModeUi() {
    const bool physical = m_physicalSpeedModeCheck->isChecked();
    if (physical) {
        m_travelVelSpin->setRange(1.0, 1000.0);
        m_weldVelSpin->setRange(1.0, 1000.0);
        m_accSpin->setRange(1.0, 5000.0);
        m_travelVelSpin->setSuffix(" mm/s");
        m_weldVelSpin->setSuffix(" mm/s");
        m_accSpin->setSuffix(" mm/s2");
        if (m_captureVelSpin) {
            m_captureVelSpin->setRange(1.0, 1000.0);
            m_captureVelSpin->setSuffix(" mm/s");
        }
    } else {
        m_travelVelSpin->setRange(1.0, 100.0);
        m_weldVelSpin->setRange(1.0, 100.0);
        m_accSpin->setRange(1.0, 100.0);
        m_travelVelSpin->setSuffix(" %");
        m_weldVelSpin->setSuffix(" %");
        m_accSpin->setSuffix(" %");
        if (m_captureVelSpin) {
            m_captureVelSpin->setRange(1.0, 100.0);
            m_captureVelSpin->setSuffix(" %");
        }
    }
    m_ovlSpin->setRange(1.0, 100.0);
    m_ovlSpin->setSuffix(" %");
}

weld_geometry::Vec3 RobotControlWidget::startCameraPoint() const {
    return {m_startX->value(), m_startY->value(), m_startZ->value()};
}

weld_geometry::Vec3 RobotControlWidget::endCameraPoint() const {
    return {m_endX->value(), m_endY->value(), m_endZ->value()};
}

weld_geometry::Pose6D RobotControlWidget::fallbackFlangePose() const {
    return {m_flangeX->value(), m_flangeY->value(), m_flangeZ->value(),
            m_flangeRx->value(), m_flangeRy->value(), m_flangeRz->value()};
}

weld_geometry::Pose6D RobotControlWidget::fallbackTcpPose() const {
    return {m_tcpX->value(), m_tcpY->value(), m_tcpZ->value(),
            m_tcpRx->value(), m_tcpRy->value(), m_tcpRz->value()};
}

void RobotControlWidget::setCameraLineFromFit(double sx, double sy, double sz, double ex, double ey, double ez) {
    m_startX->setValue(sx);
    m_startY->setValue(sy);
    m_startZ->setValue(sz);
    m_endX->setValue(ex);
    m_endY->setValue(ey);
    m_endZ->setValue(ez);
    appendLog("已接收焊缝拟合页的 Camera 线段端点。");
}

void RobotControlWidget::connectRobot() {
    m_robotConfig.ip = m_ipEdit->text().trimmed().toStdString();
    m_robotConfig.connectRobot = true;
    m_robotConfig.enableRobotMotion = m_enableMotionCheck->isChecked();
    m_robotConfig.autoEnable = false;
    m_robotConfig.autoMode = false;
    if (m_robot.connectRobot(m_robotConfig)) {
        setConnectedUi(true);
        appendLog(QStringLiteral("已连接机械臂: %1").arg(m_ipEdit->text()));
    } else {
        setConnectedUi(false);
        appendLog("连接机械臂失败。");
    }
}

void RobotControlWidget::disconnectRobot() {
    if (m_motionRunning.load()) {
        QMessageBox::warning(this, "正在运动", "请先点击停止运动/急停移动，等待运动线程结束后再断开。");
        return;
    }
    m_robot.disconnectRobot();
    m_hasCurrentPose = false;
    setConnectedUi(false);
    appendLog("已断开机械臂。");
}

void RobotControlWidget::readCurrentPose() {
    weld_geometry::Pose6D flange;
    weld_geometry::Pose6D tcp;
    if (!m_robot.getCurrentFlangePose(&flange) || !m_robot.getCurrentToolPose(&tcp)) {
        appendLog("读取当前法兰/TCP 失败。");
        return;
    }
    m_flangeX->setValue(flange.x);
    m_flangeY->setValue(flange.y);
    m_flangeZ->setValue(flange.z);
    m_flangeRx->setValue(flange.rx);
    m_flangeRy->setValue(flange.ry);
    m_flangeRz->setValue(flange.rz);
    m_tcpX->setValue(tcp.x);
    m_tcpY->setValue(tcp.y);
    m_tcpZ->setValue(tcp.z);
    m_tcpRx->setValue(tcp.rx);
    m_tcpRy->setValue(tcp.ry);
    m_tcpRz->setValue(tcp.rz);
    weld_geometry::Pose6D toolPose;
    if (m_robot.getToolCoord(m_toolConfig.toolId, &toolPose)) {
        m_toolConfig.flangeToTool = toolPose;
    }
    m_hasCurrentPose = true;
    appendLog(QStringLiteral("当前法兰: %1").arg(QString::fromStdString(weld_geometry::formatPose(flange))));
    appendLog(QStringLiteral("当前TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(tcp))));
}

void RobotControlWidget::servoOn() {
    appendLog(m_robot.enableRobot(true) ? "上伺服成功。" : "上伺服失败。");
}

void RobotControlWidget::servoOff() {
    appendLog(m_robot.enableRobot(false) ? "下伺服成功。" : "下伺服失败。");
}

void RobotControlWidget::setAutoMode() {
    appendLog(m_robot.setAutoMode() ? "已切自动模式。" : "切自动模式失败。");
}

void RobotControlWidget::resetError() {
    appendLog(m_robot.resetAllError() ? "复位错误成功。" : "复位错误失败。");
}

void RobotControlWidget::stopMotion() {
    m_stopRequested.store(true);
    appendLog("正在发送 StopMotion...");
    appendLog(m_robot.stopMotion() ? "StopMotion 已发送。" : "StopMotion 发送失败。请立即使用控制柜物理急停。");
}

void RobotControlWidget::pauseMotion() {
    appendLog(m_robot.pauseMotion() ? "PauseMotion 已发送。" : "PauseMotion 发送失败。");
}

void RobotControlWidget::resumeMotion() {
    appendLog(m_robot.resumeMotion() ? "ResumeMotion 已发送。" : "ResumeMotion 发送失败。");
}

void RobotControlWidget::onLeftRightEyeCaptureSaved(int requestId, bool ok,
                                                    QString leftPath, QString rightPath,
                                                    QString desc) {
    {
        std::lock_guard<std::mutex> lock(m_captureMutex);
        m_captureResultRequestId = requestId;
        m_captureResultReady = true;
        m_captureResultOk = ok;
        m_captureResultDesc = desc;
    }
    m_captureCv.notify_all();

    appendLog(ok
        ? QStringLiteral("偏移拍照保存完成 #%1: %2 / %3").arg(requestId).arg(leftPath, rightPath)
        : QStringLiteral("偏移拍照保存失败 #%1: %2").arg(requestId).arg(desc));
}

bool RobotControlWidget::requestCaptureBlocking(QString leftPath, QString rightPath,
                                                int frameRate, int exposure, int gain,
                                                bool keepLaserOn, int timeoutMs) {
    int requestId = 0;
    {
        std::lock_guard<std::mutex> lock(m_captureMutex);
        requestId = ++m_nextCaptureRequestId;
        m_captureResultRequestId = requestId;
        m_captureResultReady = false;
        m_captureResultOk = false;
        m_captureResultDesc.clear();
    }

    emit requestLeftRightEyeCapture(requestId, leftPath, rightPath,
                                    frameRate, exposure, gain, keepLaserOn);

    std::unique_lock<std::mutex> lock(m_captureMutex);
    const bool ready = m_captureCv.wait_for(
        lock,
        std::chrono::milliseconds(timeoutMs),
        [this, requestId]() {
            return m_captureResultReady && m_captureResultRequestId == requestId;
        });
    if (!ready) {
        m_captureResultDesc = QStringLiteral("等待相机左右目保存超时 requestId=%1").arg(requestId);
        return false;
    }
    return m_captureResultOk;
}

bool RobotControlWidget::ensureRobotReadyForRealMove(const QString& actionName) {
    fairino_client::RobotStateSnapshot state;
    if (!m_robot.getRobotStateSnapshot(&state)) {
        appendLog(QStringLiteral("%1 运动前状态读取失败，取消真实 MoveL。").arg(actionName));
        QMessageBox::warning(this, "状态读取失败", "无法读取机器人实时状态，取消真实 MoveL。");
        return false;
    }

    appendLog(QStringLiteral("%1 运动前机器人状态: %2")
        .arg(actionName)
        .arg(QString::fromStdString(fairino_client::formatRobotState(state))));

    if (state.mainCode != 0 || state.subCode != 0) {
        QMessageBox::warning(
            this,
            "机器人已有故障",
            QStringLiteral("控制器已有主/子故障码 main=%1, sub=%2，请先复位错误后再执行。")
                .arg(state.mainCode)
                .arg(state.subCode));
        return false;
    }
    if (state.robotMode != 0) {
        QMessageBox::warning(this, "当前不是自动模式", "请先点击“切自动模式”，确认控制器进入自动模式后再执行真实 MoveL。");
        return false;
    }
    if (state.emergencyStop != 0 || state.safetyStop0 != 0 || state.safetyStop1 != 0) {
        QMessageBox::warning(this, "安全停止触发", "急停或安全停止信号处于触发状态，请在控制柜确认后再执行。");
        return false;
    }
    if (state.collisionState != 0) {
        QMessageBox::warning(this, "碰撞状态触发", "机器人当前处于碰撞检测触发状态，请复位并确认现场安全后再执行。");
        return false;
    }
    if (state.programState == 2 || state.robotState == 2) {
        QMessageBox::warning(this, "控制器正在运行", "控制器当前已有运行中的程序/运动，请停止或等待完成后再执行。");
        return false;
    }
    if (state.robotState == 4) {
        QMessageBox::warning(this, "当前是拖动状态", "机器人当前处于拖动状态，请退出拖动并切到自动模式后再执行。");
        return false;
    }
    if (state.enableState == 0) {
        appendLog(QStringLiteral("%1 提醒：机器人使能状态 enable=0，若 MoveL 被拒绝，请先点击“上伺服”。")
            .arg(actionName));
    }
    return true;
}

QString RobotControlWidget::offsetCaptureDirPath() const {
    QDir dir("data");
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    const QString folder = QStringLiteral("robot_offset_capture_%1")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    return dir.filePath(folder);
}

weld_motion::WeldLinePlan RobotControlWidget::buildCurrentPlan() const {
    return weld_motion::planLinearWeldMove(
        startCameraPoint(), endCameraPoint(), fallbackFlangePose(), fallbackTcpPose(), m_handEyeConfig, m_motionConfig);
}

void RobotControlWidget::logPlan(const weld_motion::WeldLinePlan& plan) {
    appendLog(QStringLiteral("Start_base(mm): %1").arg(QString::fromStdString(weld_geometry::formatVec3(plan.startBase))));
    appendLog(QStringLiteral("End_base(mm): %1").arg(QString::fromStdString(weld_geometry::formatVec3(plan.endBase))));
    appendLog(QStringLiteral("Line length(mm): %1").arg(plan.lineLengthMm, 0, 'f', 3));
    appendLog(QStringLiteral("Approach TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.approachTcpTarget))));
    appendLog(QStringLiteral("Start TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.startTcpTarget))));
    appendLog(QStringLiteral("End TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.endTcpTarget))));
    appendLog(QStringLiteral("Retract TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.retractTcpTarget))));
}

void RobotControlWidget::rememberPlan(const weld_motion::WeldLinePlan& plan, const QString& source) {
    m_lastPlan = plan;
    m_hasLastPlan = true;
    m_lastPlanLabel->setText(QStringLiteral("上次轨迹：%1 %2，长度 %3 mm，结束抬高点 Z=%4")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
        .arg(source)
        .arg(plan.lineLengthMm, 0, 'f', 3)
        .arg(plan.retractTcpTarget.z, 0, 'f', 3));
    m_btnExecuteLast->setEnabled(!m_motionRunning.load());
}

void RobotControlWidget::calculateTargets() {
    syncMotionConfigFromUi();
    const weld_motion::WeldLinePlan plan = buildCurrentPlan();
    logPlan(plan);
    rememberPlan(plan, "已记录当前轨迹");
}

void RobotControlWidget::startMoveWithPlan(const weld_motion::WeldLinePlan& plan, const QString& source) {
    if (m_motionRunning.load()) {
        QMessageBox::warning(this, "正在运动", "当前 MoveL 流程尚未结束。");
        return;
    }
    if (m_motionConfig.enableRobotMotion && m_motionConfig.dryRun) {
        QMessageBox::warning(this, "仍是干运行", "当前勾选了干运行，不会发送真实 MoveL。");
    }
    if (m_motionConfig.enableRobotMotion && !m_robot.isConnected()) {
        QMessageBox::warning(this, "未连接", "请先连接机械臂并确认当前法兰/TCP。");
        return;
    }
    if (!m_hasCurrentPose && m_motionConfig.enableRobotMotion) {
        QMessageBox::warning(this, "未读取当前位姿", "真运动前请先读取当前法兰/TCP。");
        return;
    }
    const bool realMotion = m_motionConfig.enableRobotMotion && !m_motionConfig.dryRun;
    if (realMotion &&
        (m_motionConfig.travelVel <= 0.0 ||
         m_motionConfig.weldVel <= 0.0 ||
         m_motionConfig.acc <= 0.0 ||
         m_motionConfig.ovl <= 0.0)) {
        QMessageBox::warning(this, "运动参数无效", "真实运动时移动速度、焊接速度、加速度和倍率都必须大于 0。");
        return;
    }
    if (realMotion && !ensureRobotReadyForRealMove(source)) {
        return;
    }
    if (m_motionThread.joinable()) {
        m_motionThread.join();
    }

    const weld_motion::ToolConfig tool = m_toolConfig;
    const weld_motion::WeldMotionConfig motion = m_motionConfig;
    const weld_motion::WeldLinePlan planToRun = plan;

    m_motionRunning.store(true);
    m_stopRequested.store(false);
    setMotionUi(true);
    appendLog(QStringLiteral("%1 MoveL 流程已进入后台线程，UI 可继续响应。红色停止按钮保持可用。")
        .arg(source));

    m_motionThread = std::thread([this, planToRun, tool, motion]() {
        QVector<double> traceTime;
        QVector<double> traceX;
        QVector<double> traceY;
        QVector<double> traceZ;
        auto traceCallback = [&](const fairino_client::MotionTraceSample& sample) {
            if (!traceTime.isEmpty() &&
                sample.elapsedSec - traceTime.back() < kMinTraceSampleIntervalSec) {
                return;
            }
            traceTime.append(sample.elapsedSec);
            traceX.append(sample.tcpPose.x);
            traceY.append(sample.tcpPose.y);
            traceZ.append(sample.tcpPose.z);
        };
        const int err = fairino_client::executeLinearWeldPlan(
            m_robot, planToRun, tool, motion,
            [this]() { return m_stopRequested.load(); },
            traceCallback);
        emit asyncMotionTraceReady(traceTime, traceX, traceY, traceZ);
        emit asyncMotionFinished(err);
    });
}

void RobotControlWidget::executeMove() {
    syncMotionConfigFromUi();
    const weld_motion::WeldLinePlan plan = buildCurrentPlan();
    logPlan(plan);
    rememberPlan(plan, "执行当前轨迹");
    startMoveWithPlan(plan, "当前轨迹");
}

void RobotControlWidget::executeLastMove() {
    syncMotionConfigFromUi();
    if (!m_hasLastPlan) {
        QMessageBox::warning(this, "没有上次轨迹", "请先点击“计算并记录焊枪 TCP 点位”或执行一次当前轨迹。");
        return;
    }
    logPlan(m_lastPlan);
    startMoveWithPlan(m_lastPlan, "上次轨迹");
}

void RobotControlWidget::executeOffsetCapture() {
    syncMotionConfigFromUi();
    if (m_motionRunning.load()) {
        QMessageBox::warning(this, "正在运动", "当前运动流程尚未结束。");
        return;
    }
    if (m_motionConfig.enableRobotMotion && m_motionConfig.dryRun) {
        QMessageBox::warning(this, "仍是干运行", "当前勾选了干运行，不会发送真实 MoveL，也不会请求相机保存。");
    }
    if (m_motionConfig.enableRobotMotion && !m_robot.isConnected()) {
        QMessageBox::warning(this, "未连接", "请先连接机械臂并读取当前法兰/TCP。");
        return;
    }
    if (!m_hasCurrentPose && m_motionConfig.enableRobotMotion) {
        QMessageBox::warning(this, "未读取当前位姿", "真运动前请先读取当前法兰/TCP。");
        return;
    }

    const bool realMotion = m_motionConfig.enableRobotMotion && !m_motionConfig.dryRun;
    const double captureVelocity = m_captureVelSpin->value();
    if (realMotion &&
        (captureVelocity <= 0.0 || m_motionConfig.acc <= 0.0 || m_motionConfig.ovl <= 0.0)) {
        QMessageBox::warning(this, "运动参数无效", "真实运动时移动速度、加速度和倍率都必须大于 0。");
        return;
    }
    if (realMotion && !ensureRobotReadyForRealMove("偏移拍照")) {
        return;
    }

    weld_geometry::Pose6D startFlange = fallbackFlangePose();
    weld_geometry::Pose6D startTcp = fallbackTcpPose();
    if (realMotion) {
        if (!m_robot.getCurrentFlangePose(&startFlange) || !m_robot.getCurrentToolPose(&startTcp)) {
            QMessageBox::warning(this, "读取位姿失败", "无法读取当前法兰/TCP，取消偏移拍照。");
            return;
        }
        m_flangeX->setValue(startFlange.x);
        m_flangeY->setValue(startFlange.y);
        m_flangeZ->setValue(startFlange.z);
        m_flangeRx->setValue(startFlange.rx);
        m_flangeRy->setValue(startFlange.ry);
        m_flangeRz->setValue(startFlange.rz);
        m_tcpX->setValue(startTcp.x);
        m_tcpY->setValue(startTcp.y);
        m_tcpZ->setValue(startTcp.z);
        m_tcpRx->setValue(startTcp.rx);
        m_tcpRy->setValue(startTcp.ry);
        m_tcpRz->setValue(startTcp.rz);
        m_hasCurrentPose = true;
    }

    const robot_capture::CaptureAxis axis =
        static_cast<robot_capture::CaptureAxis>(m_captureAxisCombo->currentData().toInt());
    const QString axisName = m_captureAxisCombo->currentText();
    const double totalOffset = m_captureDistanceSpin->value();
    const double step = m_captureStepSpin->value();
    const std::vector<double> offsets = robot_capture::buildOffsetSamples(totalOffset, step);
    const int frameRate = static_cast<int>(std::lround(m_captureFrameRateSpin->value()));
    const int exposure = static_cast<int>(std::lround(m_captureExposureSpin->value()));
    const int gain = static_cast<int>(std::lround(m_captureGainSpin->value()));
    const bool keepLaserOn = m_captureKeepLaserOnCheck->isChecked();
    const QString outputDir = offsetCaptureDirPath();
    int moveToolId = m_toolConfig.toolId;
    int moveUserId = m_toolConfig.userId;
    if (realMotion) {
        if (!m_robot.getCurrentToolId(&moveToolId)) {
            appendLog(QStringLiteral("读取当前工具号失败，偏移拍照暂用配置 tool=%1。").arg(moveToolId));
        }
        if (!m_robot.getCurrentUserId(&moveUserId)) {
            appendLog(QStringLiteral("读取当前工件号失败，偏移拍照暂用配置 user=%1。").arg(moveUserId));
        }
    }

    if (!QDir().mkpath(outputDir)) {
        QMessageBox::warning(this, "创建目录失败", QStringLiteral("无法创建输出目录: %1").arg(outputDir));
        return;
    }
    if (m_motionThread.joinable()) {
        m_motionThread.join();
    }

    const weld_motion::WeldMotionConfig motion = m_motionConfig;

    m_motionRunning.store(true);
    m_stopRequested.store(false);
    setMotionUi(true);
    appendLog(QStringLiteral("偏移拍照流程开始：方向=%1，总距离=%2 mm，步距=%3 mm，采样=%4，输出目录=%5")
        .arg(axisName)
        .arg(totalOffset, 0, 'f', 3)
        .arg(step, 0, 'f', 3)
        .arg(offsets.size())
        .arg(outputDir));
    appendLog(QStringLiteral("偏移拍照 MoveL 使用当前控制器坐标号: tool=%1, user=%2。").arg(moveToolId).arg(moveUserId));
    appendLog("偏移方向按启动时当前法兰坐标系计算；MoveL 目标只改 TCP x/y/z，rx/ry/rz 保持启动时当前 TCP 姿态。");
    if (!realMotion) {
        appendLog("当前为干运行或未允许真运动：只打印偏移目标，不请求相机保存。");
    } else if (keepLaserOn) {
        appendLog("拍照过程中不反复关盖；线激光保持开启。");
    } else {
        appendLog("拍照过程中不反复关盖；每次拍照前开线激光，拍完关闭线激光。");
    }

    m_motionThread = std::thread([this, startFlange, startTcp, axis, axisName,
                                  offsets, outputDir, frameRate, exposure, gain,
                                  keepLaserOn, captureVelocity, moveToolId, moveUserId, motion, realMotion]() {
        QVector<double> traceTime;
        QVector<double> traceX;
        QVector<double> traceY;
        QVector<double> traceZ;
        const std::chrono::steady_clock::time_point traceStart = std::chrono::steady_clock::now();
        auto sampleTrace = [&]() {
            if (!realMotion) {
                return;
            }
            weld_geometry::Pose6D tcpPose;
            if (!m_robot.getCurrentToolPose(&tcpPose)) {
                return;
            }
            const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            const double elapsedSec =
                std::chrono::duration_cast<std::chrono::duration<double>>(now - traceStart).count();
            if (!traceTime.isEmpty() && elapsedSec - traceTime.back() < kMinTraceSampleIntervalSec) {
                return;
            }
            traceTime.append(elapsedSec);
            traceX.append(tcpPose.x);
            traceY.append(tcpPose.y);
            traceZ.append(tcpPose.z);
        };

        int err = 0;
        const weld_geometry::Vec3 direction = robot_capture::flangeAxisDirectionInBase(startFlange, axis);
        const double dirLen = std::sqrt(direction.x * direction.x +
                                        direction.y * direction.y +
                                        direction.z * direction.z);
        if (dirLen <= 1e-9 || !std::isfinite(dirLen)) {
            emit asyncLog(QStringLiteral("偏移拍照失败：无法由当前法兰位姿计算 %1 方向。").arg(axisName));
            emit asyncMotionTraceReady(traceTime, traceX, traceY, traceZ);
            emit asyncMotionFinished(-31);
            return;
        }

        sampleTrace();
        for (size_t i = 0; i < offsets.size(); ++i) {
            if (m_stopRequested.load()) {
                err = -20;
                break;
            }

            const double offset = offsets[i];
            const weld_geometry::Pose6D target =
                robot_capture::offsetTcpPose(startTcp, direction, offset);

            emit asyncLog(QStringLiteral("偏移拍照点 %1/%2: offset=%3 mm, TCP=%4")
                .arg(i + 1)
                .arg(offsets.size())
                .arg(offset, 0, 'f', 3)
                .arg(QString::fromStdString(weld_geometry::formatPose(target))));

            if (i > 0) {
                err = m_robot.moveL(target, moveToolId, moveUserId, motion, captureVelocity);
                if (err != 0) {
                    emit asyncLog(QStringLiteral("偏移 MoveL 失败: %1")
                        .arg(QString::fromStdString(fairino_client::formatErrorCode(err))));
                    fairino_client::RobotStateSnapshot failedState;
                    if (m_robot.getRobotStateSnapshot(&failedState)) {
                        emit asyncLog(QStringLiteral("MoveL 失败后机器人状态: %1")
                            .arg(QString::fromStdString(fairino_client::formatRobotState(failedState))));
                    }
                    break;
                }
                if (realMotion) {
                    bool stopSent = false;
                    const int waitErr = m_robot.waitRobotMotionDone(
                        120000, 100,
                        [&]() {
                            sampleTrace();
                            if (m_stopRequested.load() && !stopSent) {
                                m_robot.stopMotion();
                                stopSent = true;
                            }
                        });
                    if (waitErr != 0) {
                        err = waitErr;
                        emit asyncLog(QStringLiteral("偏移 MoveL 等待到位失败，错误码 %1").arg(waitErr));
                        break;
                    }
                    if (m_stopRequested.load()) {
                        err = -20;
                        break;
                    }
                }
            }

            sampleTrace();

            weld_geometry::Pose6D flangeForName = startFlange;
            if (realMotion && !m_robot.getCurrentFlangePose(&flangeForName)) {
                emit asyncLog("读取当前法兰位姿失败，本张图片文件名使用起始法兰值。");
            }

            const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
            const QString stem = QString::fromStdString(
                robot_capture::formatCaptureStem(static_cast<int>(i), flangeForName, stamp.toStdString()));
            const QString leftPath = QDir(outputDir).filePath(stem + QStringLiteral("_left.png"));
            const QString rightPath = QDir(outputDir).filePath(stem + QStringLiteral("_right.png"));

            if (realMotion) {
                if (!requestCaptureBlocking(leftPath, rightPath, frameRate, exposure, gain, keepLaserOn)) {
                    std::lock_guard<std::mutex> lock(m_captureMutex);
                    emit asyncLog(QStringLiteral("左右目拍照保存失败: %1").arg(m_captureResultDesc));
                    err = -32;
                    break;
                }
            } else {
                emit asyncLog(QStringLiteral("干运行：计划保存 %1 / %2").arg(leftPath, rightPath));
            }
        }

        if (err == 0 && !m_stopRequested.load()) {
            emit asyncLog(QStringLiteral("偏移拍照流程完成，输出目录: %1").arg(outputDir));
        }
        emit asyncMotionTraceReady(traceTime, traceX, traceY, traceZ);
        emit asyncMotionFinished(err);
    });
}

void RobotControlWidget::onMotionFinished(int err) {
    if (m_motionThread.joinable()) {
        m_motionThread.join();
    }
    m_motionRunning.store(false);
    setMotionUi(false);
    appendLog(err == 0
        ? "MoveL 流程完成。"
        : QStringLiteral("MoveL 流程失败或被停止: %1")
              .arg(QString::fromStdString(fairino_client::formatErrorCode(err))));
}

void RobotControlWidget::onMotionTraceReady(QVector<double> time,
                                            QVector<double> x,
                                            QVector<double> y,
                                            QVector<double> z) {
    updateMotionTracePlots(time, x, y, z);
}

void RobotControlWidget::updateMotionTracePlots(const QVector<double>& time,
                                                const QVector<double>& x,
                                                const QVector<double>& y,
                                                const QVector<double>& z) {
    if (time.size() < 2 || x.size() != time.size() ||
        y.size() != time.size() || z.size() != time.size()) {
        m_displacementPlot->clearSeries();
        m_velocityPlot->clearSeries();
        m_accelerationPlot->clearSeries();
        m_traceSummaryLabel->setText("本次没有记录到真实 TCP 采样数据。干运行或运动未开始时不会生成曲线。");
        return;
    }

    const QVector<double> dx = relativeSeries(x);
    const QVector<double> dy = relativeSeries(y);
    const QVector<double> dz = relativeSeries(z);
    const QVector<double> vx = derivativeSeries(time, dx);
    const QVector<double> vy = derivativeSeries(time, dy);
    const QVector<double> vz = derivativeSeries(time, dz);
    const QVector<double> ax = derivativeSeries(time, vx);
    const QVector<double> ay = derivativeSeries(time, vy);
    const QVector<double> az = derivativeSeries(time, vz);

    m_displacementPlot->setSeries(time, dx, dy, dz);
    m_velocityPlot->setSeries(time, vx, vy, vz);
    m_accelerationPlot->setSeries(time, ax, ay, az);

    const QString csvPath = saveMotionTraceCsv(time, x, y, z, dx, dy, dz, vx, vy, vz, ax, ay, az);
    const double duration = time.back() - time.front();
    m_traceSummaryLabel->setText(QStringLiteral("本次记录 %1 个 TCP 采样点，时长 %2 s，CSV: %3")
        .arg(time.size())
        .arg(duration, 0, 'f', 3)
        .arg(csvPath.isEmpty() ? QStringLiteral("保存失败") : csvPath));
    appendLog(QStringLiteral("已记录末端 TCP 轨迹采样 %1 点。").arg(time.size()));
    if (m_pages && m_tracePage) {
        m_pages->setCurrentWidget(m_tracePage);
    }
}

QString RobotControlWidget::saveMotionTraceCsv(const QVector<double>& time,
                                               const QVector<double>& x,
                                               const QVector<double>& y,
                                               const QVector<double>& z,
                                               const QVector<double>& dx,
                                               const QVector<double>& dy,
                                               const QVector<double>& dz,
                                               const QVector<double>& vx,
                                               const QVector<double>& vy,
                                               const QVector<double>& vz,
                                               const QVector<double>& ax,
                                               const QVector<double>& ay,
                                               const QVector<double>& az) {
    QDir dir("data");
    if (!dir.exists() && !dir.mkpath(".")) {
        return QString();
    }
    const QString fileName = QStringLiteral("robot_motion_trace_%1.csv")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz"));
    QFile file(dir.filePath(fileName));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return QString();
    }

    QTextStream out(&file);
    out << "time_s,x_mm,y_mm,z_mm,dx_mm,dy_mm,dz_mm,"
           "vx_mm_s,vy_mm_s,vz_mm_s,ax_mm_s2,ay_mm_s2,az_mm_s2\n";
    for (int i = 0; i < time.size(); ++i) {
        out << QString::number(time[i], 'f', 6) << ","
            << QString::number(x[i], 'f', 6) << ","
            << QString::number(y[i], 'f', 6) << ","
            << QString::number(z[i], 'f', 6) << ","
            << QString::number(dx[i], 'f', 6) << ","
            << QString::number(dy[i], 'f', 6) << ","
            << QString::number(dz[i], 'f', 6) << ","
            << QString::number(vx[i], 'f', 6) << ","
            << QString::number(vy[i], 'f', 6) << ","
            << QString::number(vz[i], 'f', 6) << ","
            << QString::number(ax[i], 'f', 6) << ","
            << QString::number(ay[i], 'f', 6) << ","
            << QString::number(az[i], 'f', 6) << "\n";
    }
    file.close();
    return QFileInfo(file).absoluteFilePath();
}
