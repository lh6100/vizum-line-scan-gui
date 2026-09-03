#include "stereo/calibration_app/StereoCalibrationWindow.h"

#include "ImageView.h"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTextEdit>
#include <QVBoxLayout>

namespace hik_stereo {

void StereoCalibrationWindow::buildUi() {
    setWindowTitle(QStringLiteral("海康双目专用外参标定"));
    resize(1500, 900);
    auto* root = new QVBoxLayout(this);
    auto* devices = new QGroupBox(QStringLiteral("设备"), this);
    auto* grid = new QGridLayout(devices);
    leftIpEdit_ = new QLineEdit(leftProfile_->defaultCameraIp, devices);
    rightIpEdit_ = new QLineEdit(rightProfile_->defaultCameraIp, devices);
    robotIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.200"), devices);
    leftStatus_ = new QLabel(QStringLiteral("左相机未连接"), devices);
    rightStatus_ = new QLabel(QStringLiteral("右相机未连接"), devices);
    robotStatus_ = new QLabel(QStringLiteral("FR5 未连接"), devices);
    laserStatusLabel_ = new QLabel(QStringLiteral("TTL 未确认"), devices);
    connectCamerasButton_ = new QPushButton(QStringLiteral("连接双相机"), devices);
    connectRobotButton_ = new QPushButton(QStringLiteral("连接 FR5"), devices);
    grid->addWidget(new QLabel(
        QStringLiteral("左相机 IP（160万｜DA8784601）")), 0, 0);
    grid->addWidget(leftIpEdit_, 0, 1);
    grid->addWidget(leftStatus_, 0, 2);
    grid->addWidget(new QLabel(
        QStringLiteral("右相机 IP（130万｜DB0403208）")), 1, 0);
    grid->addWidget(rightIpEdit_, 1, 1);
    grid->addWidget(rightStatus_, 1, 2);
    grid->addWidget(connectCamerasButton_, 0, 3, 2, 1);
    grid->addWidget(new QLabel(QStringLiteral("FR5 IP")), 2, 0);
    grid->addWidget(robotIpEdit_, 2, 1);
    grid->addWidget(robotStatus_, 2, 2);
    grid->addWidget(connectRobotButton_, 2, 3);
    grid->addWidget(laserStatusLabel_, 3, 0, 1, 4);
    grid->setColumnStretch(2, 1);
    root->addWidget(devices);

    auto* acquisition = new QGroupBox(QStringLiteral("双目标定采集参数"), this);
    auto* acquisitionLayout = new QHBoxLayout(acquisition);
    exposureSpin_ = new QDoubleSpinBox(acquisition);
    exposureSpin_->setRange(50.0, 100000.0);
    exposureSpin_->setDecimals(0);
    exposureSpin_->setSingleStep(1000.0);
    exposureSpin_->setValue(8000.0);
    exposureSpin_->setSuffix(QStringLiteral(" us"));
    gainSpin_ = new QDoubleSpinBox(acquisition);
    gainSpin_->setRange(0.0, 24.0);
    gainSpin_->setDecimals(1);
    gainSpin_->setSingleStep(0.5);
    gainSpin_->setValue(0.0);
    gainSpin_->setSuffix(QStringLiteral(" dB"));
    fpsSpin_ = new QDoubleSpinBox(acquisition);
    fpsSpin_->setRange(1.0, 30.0);
    fpsSpin_->setDecimals(1);
    fpsSpin_->setValue(5.0);
    fpsSpin_->setSuffix(QStringLiteral(" fps"));
    pairSkewSpin_ = new QDoubleSpinBox(acquisition);
    pairSkewSpin_->setRange(1.0, 100.0);
    pairSkewSpin_->setDecimals(1);
    pairSkewSpin_->setSingleStep(5.0);
    pairSkewSpin_->setValue(30.0);
    pairSkewSpin_->setSuffix(QStringLiteral(" ms"));
    acquisitionLayout->addWidget(new QLabel(QStringLiteral("曝光"), acquisition));
    acquisitionLayout->addWidget(exposureSpin_);
    acquisitionLayout->addWidget(new QLabel(QStringLiteral("增益"), acquisition));
    acquisitionLayout->addWidget(gainSpin_);
    acquisitionLayout->addWidget(new QLabel(QStringLiteral("帧率"), acquisition));
    acquisitionLayout->addWidget(fpsSpin_);
    acquisitionLayout->addWidget(new QLabel(
        QStringLiteral("最大配对偏差（仅静态标定）"), acquisition));
    acquisitionLayout->addWidget(pairSkewSpin_);
    acquisitionLayout->addStretch();
    root->addWidget(acquisition);

    auto* actions = new QHBoxLayout;
    startButton_ = new QPushButton(QStringLiteral("开始同步预览"), this);
    stopButton_ = new QPushButton(QStringLiteral("停止预览"), this);
    captureButton_ = new QPushButton(QStringLiteral("采集当前双目板"), this);
    importSessionButton_ = new QPushButton(
        QStringLiteral("加载已有 session"), this);
    solveButton_ = new QPushButton(QStringLiteral("求解 stereoCalibrate"), this);
    approveButton_ = new QPushButton(QStringLiteral("批准正式双目外参"), this);
    actions->addWidget(startButton_);
    actions->addWidget(stopButton_);
    actions->addWidget(captureButton_);
    actions->addWidget(importSessionButton_);
    actions->addWidget(solveButton_);
    actions->addWidget(approveButton_);
    actions->addStretch();
    root->addLayout(actions);
    captureReadinessLabel_ = new QLabel(
        QStringLiteral("采集状态：请先连接设备并开始同步预览。"), this);
    captureReadinessLabel_->setWordWrap(true);
    root->addWidget(captureReadinessLabel_);
    sampleStatus_ = new QLabel(
        QStringLiteral("请采集至少 15 组不同位置/倾角的同步 ChArUco 图像。"), this);
    resultStatus_ = new QLabel(QStringLiteral("尚未求解"), this);
    resultStatus_->setWordWrap(true);
    root->addWidget(sampleStatus_);
    root->addWidget(resultStatus_);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    leftView_ = new ImageView(splitter);
    rightView_ = new ImageView(splitter);
    leftView_->setEmptyText(QStringLiteral("等待左相机"));
    rightView_->setEmptyText(QStringLiteral("等待右相机"));
    splitter->addWidget(leftView_);
    splitter->addWidget(rightView_);
    root->addWidget(splitter, 1);
    logView_ = new QTextEdit(this);
    logView_->setReadOnly(true);
    logView_->setMaximumHeight(150);
    root->addWidget(logView_);
}

}  // namespace hik_stereo
