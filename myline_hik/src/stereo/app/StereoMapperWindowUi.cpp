#include "stereo/app/StereoMapperWindow.h"

#include "ImageView.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QList>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTextEdit>
#include <QVBoxLayout>

namespace hik_stereo {

void StereoMapperWindow::buildUi() {
    setWindowTitle(QStringLiteral("海康双目深度与环境栅格地图"));
    resize(1560, 960);
    auto* root = new QVBoxLayout(this);

    auto* devices = new QGroupBox(QStringLiteral("设备与正式标定"), this);
    auto* deviceGrid = new QGridLayout(devices);
    leftIpEdit_ = new QLineEdit(leftProfile_->defaultCameraIp, devices);
    rightIpEdit_ = new QLineEdit(rightProfile_->defaultCameraIp, devices);
    robotIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.200"), devices);
    leftStatusLabel_ = new QLabel(QStringLiteral("左相机未连接"), devices);
    rightStatusLabel_ = new QLabel(QStringLiteral("右相机未连接"), devices);
    robotStatusLabel_ = new QLabel(QStringLiteral("FR5 未连接"), devices);
    laserStatusLabel_ = new QLabel(QStringLiteral("TTL 状态未知"), devices);
    calibrationStatusLabel_ = new QLabel(
        QStringLiteral("双目标定未加载"), devices);
    calibrationStatusLabel_->setWordWrap(true);
    connectCamerasButton_ = new QPushButton(
        QStringLiteral("连接两台相机"), devices);
    connectRobotButton_ = new QPushButton(QStringLiteral("连接 FR5"), devices);
    loadCalibrationButton_ = new QPushButton(
        QStringLiteral("重新加载正式标定"), devices);
    deviceGrid->addWidget(new QLabel(
        QStringLiteral("左相机 IP（160万｜DA8784601）")), 0, 0);
    deviceGrid->addWidget(leftIpEdit_, 0, 1);
    deviceGrid->addWidget(leftStatusLabel_, 0, 2);
    deviceGrid->addWidget(new QLabel(
        QStringLiteral("右相机 IP（130万｜DB0403208）")), 1, 0);
    deviceGrid->addWidget(rightIpEdit_, 1, 1);
    deviceGrid->addWidget(rightStatusLabel_, 1, 2);
    deviceGrid->addWidget(connectCamerasButton_, 0, 3, 2, 1);
    deviceGrid->addWidget(new QLabel(QStringLiteral("FR5 IP")), 2, 0);
    deviceGrid->addWidget(robotIpEdit_, 2, 1);
    deviceGrid->addWidget(robotStatusLabel_, 2, 2);
    deviceGrid->addWidget(connectRobotButton_, 2, 3);
    deviceGrid->addWidget(laserStatusLabel_, 3, 0, 1, 2);
    deviceGrid->addWidget(calibrationStatusLabel_, 3, 2);
    deviceGrid->addWidget(loadCalibrationButton_, 3, 3);
    deviceGrid->setColumnStretch(2, 1);
    root->addWidget(devices);

    auto* parameters = new QGroupBox(QStringLiteral("深度、同步与地图参数"), this);
    auto* parameterGrid = new QGridLayout(parameters);
    resolutionCombo_ = new QComboBox(parameters);
    resolutionCombo_->addItem(QStringLiteral("快速 612×512"), QSize(612, 512));
    resolutionCombo_->addItem(QStringLiteral("精细 1224×1024"), QSize(1224, 1024));
    exposureSpin_ = new QDoubleSpinBox(parameters);
    exposureSpin_->setRange(50.0, 100000.0);
    exposureSpin_->setValue(2000.0);
    exposureSpin_->setSuffix(QStringLiteral(" us"));
    gainSpin_ = new QDoubleSpinBox(parameters);
    gainSpin_->setRange(0.0, 24.0);
    fpsSpin_ = new QDoubleSpinBox(parameters);
    fpsSpin_->setRange(1.0, 30.0);
    fpsSpin_->setValue(5.0);
    fpsSpin_->setSuffix(QStringLiteral(" fps"));
    pairSkewSpin_ = new QDoubleSpinBox(parameters);
    pairSkewSpin_->setRange(0.2, 30.0);
    pairSkewSpin_->setValue(5.0);
    pairSkewSpin_->setSuffix(QStringLiteral(" ms"));
    minimumDepthSpin_ = new QDoubleSpinBox(parameters);
    minimumDepthSpin_->setRange(200.0, 5000.0);
    minimumDepthSpin_->setValue(450.0);
    minimumDepthSpin_->setSuffix(QStringLiteral(" mm"));
    maximumDepthSpin_ = new QDoubleSpinBox(parameters);
    maximumDepthSpin_->setRange(300.0, 10000.0);
    maximumDepthSpin_->setValue(3000.0);
    maximumDepthSpin_->setSuffix(QStringLiteral(" mm"));
    blockSizeSpin_ = new QSpinBox(parameters);
    blockSizeSpin_->setRange(3, 11);
    blockSizeSpin_->setSingleStep(2);
    blockSizeSpin_->setValue(5);
    leftRightCheck_ = new QCheckBox(QStringLiteral("左右一致性过滤"), parameters);
    leftRightCheck_->setChecked(true);
    voxelSizeSpin_ = new QDoubleSpinBox(parameters);
    voxelSizeSpin_->setRange(5.0, 200.0);
    voxelSizeSpin_->setValue(25.0);
    voxelSizeSpin_->setSuffix(QStringLiteral(" mm"));
    pixelStrideSpin_ = new QSpinBox(parameters);
    pixelStrideSpin_->setRange(1, 16);
    pixelStrideSpin_->setValue(4);
    robotOffsetSpin_ = new QDoubleSpinBox(parameters);
    robotOffsetSpin_->setRange(-100000.0, 100000.0);
    robotOffsetSpin_->setValue(0.0);
    robotOffsetSpin_->setSuffix(QStringLiteral(" us"));
    robotGapSpin_ = new QDoubleSpinBox(parameters);
    robotGapSpin_->setRange(5.0, 100.0);
    robotGapSpin_->setValue(25.0);
    robotGapSpin_->setSuffix(QStringLiteral(" ms"));
    gridMinimumHeightSpin_ = new QDoubleSpinBox(parameters);
    gridMinimumHeightSpin_->setRange(-3000.0, 5000.0);
    gridMinimumHeightSpin_->setValue(-100.0);
    gridMinimumHeightSpin_->setSuffix(QStringLiteral(" mm"));
    gridMaximumHeightSpin_ = new QDoubleSpinBox(parameters);
    gridMaximumHeightSpin_->setRange(-2000.0, 8000.0);
    gridMaximumHeightSpin_->setValue(2000.0);
    gridMaximumHeightSpin_->setSuffix(QStringLiteral(" mm"));
    const QList<QPair<QString, QWidget*>> fields = {
        {QStringLiteral("处理分辨率"), resolutionCombo_},
        {QStringLiteral("曝光"), exposureSpin_},
        {QStringLiteral("增益"), gainSpin_},
        {QStringLiteral("采集频率"), fpsSpin_},
        {QStringLiteral("最大帧偏差"), pairSkewSpin_},
        {QStringLiteral("最小深度"), minimumDepthSpin_},
        {QStringLiteral("最大深度"), maximumDepthSpin_},
        {QStringLiteral("SGBM 块"), blockSizeSpin_},
        {QStringLiteral("地图体素"), voxelSizeSpin_},
        {QStringLiteral("深度采样步长"), pixelStrideSpin_},
        {QStringLiteral("相机→机器人时偏"), robotOffsetSpin_},
        {QStringLiteral("最大机器人包间隔"), robotGapSpin_},
        {QStringLiteral("2D 最低高度"), gridMinimumHeightSpin_},
        {QStringLiteral("2D 最高高度"), gridMaximumHeightSpin_}
    };
    for (int index = 0; index < fields.size(); ++index) {
        const int column = (index / 4) * 2;
        const int row = index % 4;
        parameterGrid->addWidget(new QLabel(fields[index].first), row, column);
        parameterGrid->addWidget(fields[index].second, row, column + 1);
    }
    parameterGrid->addWidget(leftRightCheck_, 3, 6, 1, 2);
    root->addWidget(parameters);

    auto* actions = new QHBoxLayout;
    startButton_ = new QPushButton(QStringLiteral("开始双目建图"), this);
    stopButton_ = new QPushButton(QStringLiteral("停止"), this);
    resetButton_ = new QPushButton(QStringLiteral("清空地图"), this);
    exportButton_ = new QPushButton(QStringLiteral("导出 PLY + 栅格"), this);
    mappingStatusLabel_ = new QLabel(QStringLiteral("未运行"), this);
    statisticsLabel_ = new QLabel(QStringLiteral("尚无深度帧"), this);
    statisticsLabel_->setWordWrap(true);
    actions->addWidget(startButton_);
    actions->addWidget(stopButton_);
    actions->addWidget(resetButton_);
    actions->addWidget(exportButton_);
    actions->addWidget(mappingStatusLabel_, 1);
    root->addLayout(actions);
    root->addWidget(statisticsLabel_);

    auto* views = new QTabWidget(this);
    leftView_ = new ImageView(views);
    disparityView_ = new ImageView(views);
    depthView_ = new ImageView(views);
    leftView_->setEmptyText(QStringLiteral("等待校正后的左图"));
    disparityView_->setEmptyText(QStringLiteral("等待视差图"));
    depthView_->setEmptyText(QStringLiteral("等待深度图"));
    views->addTab(leftView_, QStringLiteral("校正左图"));
    views->addTab(disparityView_, QStringLiteral("视差"));
    views->addTab(depthView_, QStringLiteral("深度"));
    root->addWidget(views, 1);
    logView_ = new QTextEdit(this);
    logView_->setReadOnly(true);
    logView_->setMaximumHeight(130);
    root->addWidget(logView_);
}


StereoDepthOptions StereoMapperWindow::depthOptionsFromUi() const {
    StereoDepthOptions options;
    const QSize size = resolutionCombo_->currentData().toSize();
    options.processingSize = cv::Size(size.width(), size.height());
    options.minimumDepthMm = minimumDepthSpin_->value();
    options.maximumDepthMm = maximumDepthSpin_->value();
    options.blockSize = blockSizeSpin_->value() % 2 == 0
        ? blockSizeSpin_->value() + 1 : blockSizeSpin_->value();
    options.enableLeftRightCheck = leftRightCheck_->isChecked();
    options.maximumNumDisparities = 512;
    return options;
}

OccupancyMapOptions StereoMapperWindow::mapOptionsFromUi() const {
    OccupancyMapOptions options;
    options.voxelSizeMm = voxelSizeSpin_->value();
    options.pixelStride = pixelStrideSpin_->value();
    return options;
}

cv::Matx44d StereoMapperWindow::eigenToCv(
        const Eigen::Matrix4d& transform) {
    cv::Matx44d result;
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            result(row, column) = transform(row, column);
    return result;
}

void StereoMapperWindow::updateUi() {
    const bool idle = !mappingRequested_ && !processingBusy_;
    connectCamerasButton_->setEnabled(idle);
    connectRobotButton_->setEnabled(idle);
    loadCalibrationButton_->setEnabled(idle);
    startButton_->setEnabled(
        idle && calibrationLoaded_ && leftConnected_ && rightConnected_ &&
        robotConnected_ && cameraIdentityAccepted(StereoCameraSide::Left) &&
        cameraIdentityAccepted(StereoCameraSide::Right));
    stopButton_->setEnabled(mappingRequested_);
    resetButton_->setEnabled(idle);
    exportButton_->setEnabled(idle && processedFrames_ > 0);
    resolutionCombo_->setEnabled(idle);
    exposureSpin_->setEnabled(idle);
    gainSpin_->setEnabled(idle);
    fpsSpin_->setEnabled(idle);
    pairSkewSpin_->setEnabled(idle);
    minimumDepthSpin_->setEnabled(idle);
    maximumDepthSpin_->setEnabled(idle);
    blockSizeSpin_->setEnabled(idle);
    leftRightCheck_->setEnabled(idle);
    voxelSizeSpin_->setEnabled(idle);
    pixelStrideSpin_->setEnabled(idle);
    robotOffsetSpin_->setEnabled(idle);
    robotGapSpin_->setEnabled(idle);
}


}  // namespace hik_stereo
