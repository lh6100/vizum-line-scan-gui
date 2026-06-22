#include "RobotControlWidget.h"

#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

RobotControlWidget::RobotControlWidget(QWidget* parent)
    : QWidget(parent) {
    loadConfigs();

    auto* root = new QVBoxLayout(this);

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
    m_velSpin = makeCoordSpin(m_motionConfig.vel);
    m_accSpin = makeCoordSpin(m_motionConfig.acc);
    m_ovlSpin = makeCoordSpin(m_motionConfig.ovl);
    m_offsetXSpin = makeCoordSpin(m_motionConfig.processOffset.x);
    m_offsetYSpin = makeCoordSpin(m_motionConfig.processOffset.y);
    m_offsetZSpin = makeCoordSpin(m_motionConfig.processOffset.z);
    m_btnCalculate = new QPushButton("计算焊枪 TCP 点位", this);
    m_btnExecute = new QPushButton("执行 MoveL 三点", this);
    motionGrid->addWidget(new QLabel("安全高度/速度/加速度/倍率", this), 0, 0);
    motionGrid->addWidget(m_safeHeightSpin, 0, 1);
    motionGrid->addWidget(m_velSpin, 0, 2);
    motionGrid->addWidget(m_accSpin, 0, 3);
    motionGrid->addWidget(m_ovlSpin, 0, 4);
    motionGrid->addWidget(new QLabel("工艺偏移 X/Y/Z", this), 1, 0);
    motionGrid->addWidget(m_offsetXSpin, 1, 1);
    motionGrid->addWidget(m_offsetYSpin, 1, 2);
    motionGrid->addWidget(m_offsetZSpin, 1, 3);
    motionGrid->addWidget(m_btnCalculate, 2, 0, 1, 2);
    motionGrid->addWidget(m_btnExecute, 2, 2, 1, 2);
    root->addWidget(motionGroup);

    m_log = new QPlainTextEdit(this);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(2000);
    root->addWidget(m_log, 1);

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
    connect(this, &RobotControlWidget::asyncLog, this, &RobotControlWidget::appendLog);
    connect(this, &RobotControlWidget::asyncMotionFinished, this, &RobotControlWidget::onMotionFinished);

    setConnectedUi(false);
    appendLog("默认干运行。真运动需要取消干运行并勾选允许真运动 MoveL。");
}

RobotControlWidget::~RobotControlWidget() {
    if (m_motionRunning.load()) {
        appendLog("窗口关闭，正在发送 StopMotion。");
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
    m_btnStop->setEnabled(m_robot.isConnected() || running);
    m_dryRunCheck->setEnabled(!running);
    m_enableMotionCheck->setEnabled(!running);
}

void RobotControlWidget::syncMotionConfigFromUi() {
    m_motionConfig.dryRun = m_dryRunCheck->isChecked();
    m_motionConfig.enableRobotMotion = m_enableMotionCheck->isChecked();
    m_motionConfig.safeHeightMm = m_safeHeightSpin->value();
    m_motionConfig.vel = m_velSpin->value();
    m_motionConfig.acc = m_accSpin->value();
    m_motionConfig.ovl = m_ovlSpin->value();
    m_motionConfig.processOffset = {m_offsetXSpin->value(), m_offsetYSpin->value(), m_offsetZSpin->value()};
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
    appendLog("正在发送 StopMotion...");
    appendLog(m_robot.stopMotion() ? "StopMotion 已发送。" : "StopMotion 发送失败。请立即使用控制柜物理急停。");
}

void RobotControlWidget::pauseMotion() {
    appendLog(m_robot.pauseMotion() ? "PauseMotion 已发送。" : "PauseMotion 发送失败。");
}

void RobotControlWidget::resumeMotion() {
    appendLog(m_robot.resumeMotion() ? "ResumeMotion 已发送。" : "ResumeMotion 发送失败。");
}

void RobotControlWidget::calculateTargets() {
    syncMotionConfigFromUi();
    const weld_motion::WeldLinePlan plan = weld_motion::planLinearWeldMove(
        startCameraPoint(), endCameraPoint(), fallbackFlangePose(), fallbackTcpPose(), m_handEyeConfig, m_motionConfig);
    appendLog(QStringLiteral("Start_base(mm): %1").arg(QString::fromStdString(weld_geometry::formatVec3(plan.startBase))));
    appendLog(QStringLiteral("End_base(mm): %1").arg(QString::fromStdString(weld_geometry::formatVec3(plan.endBase))));
    appendLog(QStringLiteral("Line length(mm): %1").arg(plan.lineLengthMm, 0, 'f', 3));
    appendLog(QStringLiteral("Approach TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.approachTcpTarget))));
    appendLog(QStringLiteral("Start TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.startTcpTarget))));
    appendLog(QStringLiteral("End TCP: %1").arg(QString::fromStdString(weld_geometry::formatPose(plan.endTcpTarget))));
}

void RobotControlWidget::executeMove() {
    syncMotionConfigFromUi();
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
    if (m_motionThread.joinable()) {
        m_motionThread.join();
    }

    const weld_geometry::Vec3 start = startCameraPoint();
    const weld_geometry::Vec3 end = endCameraPoint();
    const weld_geometry::Pose6D flange = fallbackFlangePose();
    const weld_geometry::Pose6D tcp = fallbackTcpPose();
    const weld_motion::HandEyeConfig handEye = m_handEyeConfig;
    const weld_motion::ToolConfig tool = m_toolConfig;
    const weld_motion::WeldMotionConfig motion = m_motionConfig;

    m_motionRunning.store(true);
    setMotionUi(true);
    appendLog("MoveL 流程已进入后台线程，UI 可继续响应。红色停止按钮保持可用。");

    m_motionThread = std::thread([this, start, end, flange, tcp, handEye, tool, motion]() {
        const int err = fairino_client::executeLinearWeldMove(
            m_robot, start, end, flange, tcp, handEye, tool, motion);
        emit asyncMotionFinished(err);
    });
}

void RobotControlWidget::onMotionFinished(int err) {
    if (m_motionThread.joinable()) {
        m_motionThread.join();
    }
    m_motionRunning.store(false);
    setMotionUi(false);
    appendLog(err == 0 ? "MoveL 流程完成。" : QStringLiteral("MoveL 流程失败或被停止，错误码 %1").arg(err));
}
