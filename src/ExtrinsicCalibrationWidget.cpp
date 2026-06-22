#include "ExtrinsicCalibrationWidget.h"

#include "RgbImageLabel.h"
#include "ScanWorker.h"

#include <QComboBox>
#include <QCheckBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QAbstractItemView>
#include <QFile>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QVBoxLayout>

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

const double kPi = 3.14159265358979323846;

cv::Mat cvMatFromWeldMatrix(const weld_geometry::Matrix4& matrix) {
    cv::Mat out(4, 4, CV_64F);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out.at<double>(r, c) = matrix.m[static_cast<size_t>(r * 4 + c)];
        }
    }
    return out;
}

cv::Mat cvMatFromPose(const weld_geometry::Pose6D& pose) {
    return cvMatFromWeldMatrix(weld_geometry::poseToMatrix(pose));
}

cv::Mat mat4FromVector(const QVector<double>& values) {
    if (values.size() != 16) {
        return {};
    }
    cv::Mat out(4, 4, CV_64F);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out.at<double>(r, c) = values[r * 4 + c];
        }
    }
    return out;
}

cv::Mat mat4FromRt(const cv::Mat& rotation, const cv::Mat& translation) {
    cv::Mat out = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(out(cv::Rect(0, 0, 3, 3)));
    out.at<double>(0, 3) = translation.at<double>(0);
    out.at<double>(1, 3) = translation.at<double>(1);
    out.at<double>(2, 3) = translation.at<double>(2);
    return out;
}

void appendRt(const cv::Mat& transform, std::vector<cv::Mat>* rotations, std::vector<cv::Mat>* translations) {
    rotations->push_back(transform(cv::Rect(0, 0, 3, 3)).clone());
    translations->push_back(transform(cv::Rect(3, 0, 1, 3)).clone());
}

double rotationAngleDeg(const cv::Mat& rotation) {
    double c = (cv::trace(rotation)[0] - 1.0) * 0.5;
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c) * 180.0 / kPi;
}

QString formatMatrix4(const cv::Mat& matrix, int decimals = 6) {
    QString out;
    for (int r = 0; r < 4; ++r) {
        QStringList row;
        for (int c = 0; c < 4; ++c) {
            row << QString::number(matrix.at<double>(r, c), 'f', decimals);
        }
        out += QStringLiteral("    [") + row.join(QStringLiteral(", ")) + QStringLiteral("]\n");
    }
    return out.trimmed();
}

QString formatPoseQt(const weld_geometry::Pose6D& pose) {
    return QStringLiteral("[%1, %2, %3, %4, %5, %6]")
        .arg(pose.x, 0, 'f', 3)
        .arg(pose.y, 0, 'f', 3)
        .arg(pose.z, 0, 'f', 3)
        .arg(pose.rx, 0, 'f', 3)
        .arg(pose.ry, 0, 'f', 3)
        .arg(pose.rz, 0, 'f', 3);
}

QImage stretchGrayscalePreview(const QImage& source) {
    QImage gray = source.convertToFormat(QImage::Format_Grayscale8);
    if (gray.isNull()) {
        return source;
    }

    quint64 hist[256] = {};
    quint64 count = 0;
    for (int y = 0; y < gray.height(); ++y) {
        const uchar* row = gray.constScanLine(y);
        for (int x = 0; x < gray.width(); ++x) {
            ++hist[row[x]];
            ++count;
        }
    }
    if (count == 0) {
        return source;
    }

    const quint64 lowTarget = static_cast<quint64>(std::floor(count * 0.01));
    const quint64 highTarget = static_cast<quint64>(std::ceil(count * 0.995));
    quint64 acc = 0;
    int low = 0;
    for (int i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc >= lowTarget) {
            low = i;
            break;
        }
    }
    acc = 0;
    int high = 255;
    for (int i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc >= highTarget) {
            high = i;
            break;
        }
    }
    if (high <= low) {
        return source;
    }

    QImage out(gray.size(), QImage::Format_Grayscale8);
    out.setColorTable(gray.colorTable());
    for (int y = 0; y < gray.height(); ++y) {
        const uchar* src = gray.constScanLine(y);
        uchar* dst = out.scanLine(y);
        for (int x = 0; x < gray.width(); ++x) {
            int v = (static_cast<int>(src[x]) - low) * 255 / (high - low);
            dst[x] = static_cast<uchar>(std::max(0, std::min(255, v)));
        }
    }
    return out;
}

} // namespace

ExtrinsicCalibrationWidget::ExtrinsicCalibrationWidget(QWidget* parent)
    : QWidget(parent) {
    m_robotConfig = fairino_client::loadRobotConfig("config/robot_config.yaml");

    auto* root = new QVBoxLayout(this);
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    root->addWidget(splitter, 1);

    auto* leftPanel = new QWidget(this);
    auto* controls = new QVBoxLayout(leftPanel);

    auto* cameraGroup = new QGroupBox("左目灰度相机", this);
    auto* cameraGrid = new QGridLayout(cameraGroup);
    m_frameRateSpin = makeIntSpin(1, 2000, 1, 1);
    m_exposureSpin = makeIntSpin(0, 65535, 9000, 100);
    m_gainSpin = makeIntSpin(0, 255, 3, 1);
    m_previewStretchCheck = new QCheckBox("预览增强", this);
    m_previewStretchCheck->setChecked(true);
    m_keepLaserCheck = new QCheckBox("拍照保持线激光", this);
    m_keepLaserCheck->setChecked(false);
    m_btnGrabLeft = new QPushButton("获取左目图", this);
    m_btnDetect = new QPushButton("检测标定板", this);
    cameraGrid->addWidget(new QLabel("帧率", this), 0, 0);
    cameraGrid->addWidget(m_frameRateSpin, 0, 1);
    cameraGrid->addWidget(new QLabel("曝光", this), 0, 2);
    cameraGrid->addWidget(m_exposureSpin, 0, 3);
    cameraGrid->addWidget(new QLabel("增益", this), 0, 4);
    cameraGrid->addWidget(m_gainSpin, 0, 5);
    cameraGrid->addWidget(m_previewStretchCheck, 1, 0, 1, 2);
    cameraGrid->addWidget(m_keepLaserCheck, 1, 2, 1, 2);
    cameraGrid->addWidget(m_btnGrabLeft, 2, 0, 1, 3);
    cameraGrid->addWidget(m_btnDetect, 2, 3, 1, 3);
    controls->addWidget(cameraGroup);

    auto* boardGroup = new QGroupBox("标定板", this);
    auto* boardGrid = new QGridLayout(boardGroup);
    m_squaresXSpin = makeIntSpin(2, 40, 7);
    m_squaresYSpin = makeIntSpin(2, 40, 5);
    m_squareMmSpin = makeDoubleSpin(0.1, 1000.0, 20.0, 3, 1.0);
    m_markerMmSpin = makeDoubleSpin(0.1, 1000.0, 15.0, 3, 1.0);
    m_dictionaryCombo = new QComboBox(this);
    m_dictionaryCombo->addItems({
        QStringLiteral("DICT_4X4_50"),
        QStringLiteral("DICT_4X4_100"),
        QStringLiteral("DICT_4X4_250"),
        QStringLiteral("DICT_4X4_1000"),
        QStringLiteral("DICT_5X5_50"),
        QStringLiteral("DICT_5X5_100")
    });
    boardGrid->addWidget(new QLabel("方格 X/Y", this), 0, 0);
    boardGrid->addWidget(m_squaresXSpin, 0, 1);
    boardGrid->addWidget(m_squaresYSpin, 0, 2);
    boardGrid->addWidget(new QLabel("字典", this), 0, 3);
    boardGrid->addWidget(m_dictionaryCombo, 0, 4);
    boardGrid->addWidget(new QLabel("方格边长 mm", this), 1, 0);
    boardGrid->addWidget(m_squareMmSpin, 1, 1);
    boardGrid->addWidget(new QLabel("Marker 边长 mm", this), 1, 2);
    boardGrid->addWidget(m_markerMmSpin, 1, 3, 1, 2);
    controls->addWidget(boardGroup);

    auto* robotGroup = new QGroupBox("法奥机械臂法兰坐标", this);
    auto* robotGrid = new QGridLayout(robotGroup);
    m_ipEdit = new QLineEdit(QString::fromStdString(m_robotConfig.ip), this);
    m_robotStatusLabel = new QLabel("未连接", this);
    m_robotStatusLabel->setStyleSheet("color:#b00; font-weight:bold;");
    m_btnConnectRobot = new QPushButton("连接机械臂", this);
    m_btnDisconnectRobot = new QPushButton("断开", this);
    m_btnReadFlange = new QPushButton("读取法兰", this);
    m_flangeX = makePoseSpin(0.0);
    m_flangeY = makePoseSpin(0.0);
    m_flangeZ = makePoseSpin(0.0);
    m_flangeRx = makePoseSpin(0.0);
    m_flangeRy = makePoseSpin(0.0);
    m_flangeRz = makePoseSpin(0.0);
    robotGrid->addWidget(new QLabel("IP", this), 0, 0);
    robotGrid->addWidget(m_ipEdit, 0, 1, 1, 3);
    robotGrid->addWidget(m_robotStatusLabel, 0, 4);
    robotGrid->addWidget(m_btnConnectRobot, 1, 0);
    robotGrid->addWidget(m_btnDisconnectRobot, 1, 1);
    robotGrid->addWidget(m_btnReadFlange, 1, 2);
    robotGrid->addWidget(new QLabel("X/Y/Z", this), 2, 0);
    robotGrid->addWidget(m_flangeX, 2, 1);
    robotGrid->addWidget(m_flangeY, 2, 2);
    robotGrid->addWidget(m_flangeZ, 2, 3);
    robotGrid->addWidget(new QLabel("Rx/Ry/Rz", this), 3, 0);
    robotGrid->addWidget(m_flangeRx, 3, 1);
    robotGrid->addWidget(m_flangeRy, 3, 2);
    robotGrid->addWidget(m_flangeRz, 3, 3);
    controls->addWidget(robotGroup);

    auto* sampleGroup = new QGroupBox("采样与解算", this);
    auto* sampleLayout = new QVBoxLayout(sampleGroup);
    auto* sampleButtons = new QHBoxLayout;
    m_btnAddSample = new QPushButton("添加样本", this);
    m_btnClearSamples = new QPushButton("清空样本", this);
    m_btnSolve = new QPushButton("解算矩阵", this);
    sampleButtons->addWidget(m_btnAddSample);
    sampleButtons->addWidget(m_btnClearSamples);
    sampleButtons->addWidget(m_btnSolve);
    sampleLayout->addLayout(sampleButtons);
    m_sampleTable = new QTableWidget(0, 8, this);
    m_sampleTable->setHorizontalHeaderLabels({
        QStringLiteral("#"), QStringLiteral("Fx"), QStringLiteral("Fy"), QStringLiteral("Fz"),
        QStringLiteral("Frx"), QStringLiteral("Fry"), QStringLiteral("Frz"), QStringLiteral("BoardZ")
    });
    m_sampleTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_sampleTable->horizontalHeader()->setStretchLastSection(true);
    m_sampleTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_sampleTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    sampleLayout->addWidget(m_sampleTable, 1);
    controls->addWidget(sampleGroup, 1);

    m_log = new QPlainTextEdit(this);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(2000);
    controls->addWidget(m_log, 1);

    splitter->addWidget(leftPanel);

    auto* previewGroup = new QGroupBox("左目预览", this);
    auto* previewLayout = new QVBoxLayout(previewGroup);
    m_imageInfoLabel = new QLabel("未获取左目图", this);
    m_imageInfoLabel->setWordWrap(true);
    previewLayout->addWidget(m_imageInfoLabel);
    m_imageLabel = new RgbImageLabel(this);
    m_imageLabel->setText("左目灰度图显示区域");
    m_imageLabel->setMinimumSize(520, 520);
    m_imageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    previewLayout->addWidget(m_imageLabel, 1);
    splitter->addWidget(previewGroup);
    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 3);

    connect(m_btnGrabLeft, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onGrabLeftEyeClicked);
    connect(m_btnDetect, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onDetectClicked);
    connect(m_btnAddSample, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onAddSampleClicked);
    connect(m_btnClearSamples, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onClearSamplesClicked);
    connect(m_btnSolve, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onSolveClicked);
    connect(m_btnConnectRobot, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onConnectRobotClicked);
    connect(m_btnDisconnectRobot, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onDisconnectRobotClicked);
    connect(m_btnReadFlange, &QPushButton::clicked, this, &ExtrinsicCalibrationWidget::onReadFlangeClicked);

    setRobotUi(false);
    setCameraUi(false);
    updateSampleActions();
    appendLog("先调曝光/增益让左目图看清，再至少采集 3 组不同姿态样本；建议 8 组以上。");
}

ExtrinsicCalibrationWidget::~ExtrinsicCalibrationWidget() {
    m_robot.disconnectRobot();
}

void ExtrinsicCalibrationWidget::setWorker(ScanWorker* worker) {
    if (m_worker) {
        disconnect(this, nullptr, m_worker, nullptr);
        disconnect(m_worker, nullptr, this, nullptr);
    }
    m_worker = worker;
    if (!m_worker) {
        return;
    }

    connect(this, &ExtrinsicCalibrationWidget::reqGrabLeftEye,
            m_worker, &ScanWorker::grabLeftEyeImageWithParams, Qt::QueuedConnection);
    connect(this, &ExtrinsicCalibrationWidget::reqDetectLeftEyeCharuco,
            m_worker, &ScanWorker::calibrateLeftEyeCharucoWithParams, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::leftEyeImageReady,
            this, &ExtrinsicCalibrationWidget::onLeftEyeImageReady, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::leftEyeCharucoResult,
            this, &ExtrinsicCalibrationWidget::onCharucoResult, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::log,
            this, &ExtrinsicCalibrationWidget::onWorkerLog, Qt::QueuedConnection);
    connect(m_worker, &ScanWorker::busyChanged,
            this, &ExtrinsicCalibrationWidget::onWorkerBusyChanged, Qt::QueuedConnection);
    setCameraUi(false);
}

QSpinBox* ExtrinsicCalibrationWidget::makeIntSpin(int minValue, int maxValue, int value, int step) {
    auto* spin = new QSpinBox(this);
    spin->setRange(minValue, maxValue);
    spin->setSingleStep(step);
    spin->setValue(value);
    return spin;
}

QDoubleSpinBox* ExtrinsicCalibrationWidget::makeDoubleSpin(double minValue, double maxValue, double value,
                                                           int decimals, double step) {
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(minValue, maxValue);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    spin->setValue(value);
    return spin;
}

QDoubleSpinBox* ExtrinsicCalibrationWidget::makePoseSpin(double value) {
    return makeDoubleSpin(-5000.0, 5000.0, value, 3, 1.0);
}

void ExtrinsicCalibrationWidget::appendLog(const QString& msg) {
    m_log->appendPlainText(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
        .arg(msg));
}

void ExtrinsicCalibrationWidget::setRobotUi(bool connected) {
    m_robotStatusLabel->setText(connected ? "已连接" : "未连接");
    m_robotStatusLabel->setStyleSheet(connected ? "color:#080; font-weight:bold;" : "color:#b00; font-weight:bold;");
    m_btnConnectRobot->setEnabled(!connected);
    m_btnDisconnectRobot->setEnabled(connected);
    m_btnReadFlange->setEnabled(connected);
}

void ExtrinsicCalibrationWidget::setCameraUi(bool busy) {
    m_cameraBusy = busy;
    const bool enabled = !busy && m_worker;
    m_btnGrabLeft->setEnabled(enabled);
    m_btnDetect->setEnabled(enabled);
    m_btnAddSample->setEnabled(enabled);
    updateSampleActions();
}

bool ExtrinsicCalibrationWidget::ensureWorker() const {
    if (m_worker) {
        return true;
    }
    QMessageBox::warning(const_cast<ExtrinsicCalibrationWidget*>(this), "相机未就绪", "ScanWorker 尚未初始化。");
    return false;
}

int ExtrinsicCalibrationWidget::exposureValue() const {
    return m_exposureSpin->value();
}

int ExtrinsicCalibrationWidget::frameRateValue() const {
    return m_frameRateSpin->value();
}

int ExtrinsicCalibrationWidget::gainValue() const {
    return m_gainSpin->value();
}

QString ExtrinsicCalibrationWidget::dictionaryName() const {
    return m_dictionaryCombo->currentText();
}

QImage ExtrinsicCalibrationWidget::displayImageForPreview(const QImage& image) const {
    if (!m_previewStretchCheck || !m_previewStretchCheck->isChecked()) {
        return image;
    }
    if (image.format() == QImage::Format_Grayscale8 || image.depth() == 8) {
        return stretchGrayscalePreview(image);
    }
    return image;
}

weld_geometry::Pose6D ExtrinsicCalibrationWidget::currentFlangePose() const {
    return {m_flangeX->value(), m_flangeY->value(), m_flangeZ->value(),
            m_flangeRx->value(), m_flangeRy->value(), m_flangeRz->value()};
}

void ExtrinsicCalibrationWidget::setFlangePose(const weld_geometry::Pose6D& pose) {
    m_flangeX->setValue(pose.x);
    m_flangeY->setValue(pose.y);
    m_flangeZ->setValue(pose.z);
    m_flangeRx->setValue(pose.rx);
    m_flangeRy->setValue(pose.ry);
    m_flangeRz->setValue(pose.rz);
}

bool ExtrinsicCalibrationWidget::readCurrentFlangePose() {
    if (!m_robot.isConnected()) {
        appendLog("机械臂未连接，无法读取法兰。");
        return false;
    }
    weld_geometry::Pose6D pose;
    if (!m_robot.getCurrentFlangePose(&pose)) {
        appendLog("读取法奥当前法兰坐标失败。");
        return false;
    }
    setFlangePose(pose);
    appendLog(QStringLiteral("当前法兰: %1").arg(formatPoseQt(pose)));
    return true;
}

void ExtrinsicCalibrationWidget::onGrabLeftEyeClicked() {
    if (!ensureWorker()) {
        return;
    }
    appendLog(QStringLiteral("获取左目图：fps=%1 exposure=%2 gain=%3")
              .arg(frameRateValue()).arg(exposureValue()).arg(gainValue()));
    emit reqGrabLeftEye(frameRateValue(), exposureValue(), gainValue(), m_keepLaserCheck->isChecked());
}

void ExtrinsicCalibrationWidget::onDetectClicked() {
    if (!ensureWorker()) {
        return;
    }
    m_pendingAddSample = false;
    appendLog(QStringLiteral("检测 ChArUco：%1x%2 square=%3mm marker=%4mm %5")
              .arg(m_squaresXSpin->value()).arg(m_squaresYSpin->value())
              .arg(m_squareMmSpin->value()).arg(m_markerMmSpin->value())
              .arg(dictionaryName()));
    emit reqDetectLeftEyeCharuco(frameRateValue(), exposureValue(), gainValue(),
                                 m_squaresXSpin->value(), m_squaresYSpin->value(),
                                 m_squareMmSpin->value(), m_markerMmSpin->value(),
                                 dictionaryName(), m_keepLaserCheck->isChecked());
}

void ExtrinsicCalibrationWidget::onAddSampleClicked() {
    if (!ensureWorker()) {
        return;
    }
    if (m_robot.isConnected()) {
        if (!readCurrentFlangePose()) {
            return;
        }
    } else {
        appendLog("机械臂未连接，使用界面当前法兰坐标采样。");
    }
    m_pendingFlangePose = currentFlangePose();
    m_pendingAddSample = true;
    appendLog(QStringLiteral("开始添加样本，法兰=%1").arg(formatPoseQt(m_pendingFlangePose)));
    emit reqDetectLeftEyeCharuco(frameRateValue(), exposureValue(), gainValue(),
                                 m_squaresXSpin->value(), m_squaresYSpin->value(),
                                 m_squareMmSpin->value(), m_markerMmSpin->value(),
                                 dictionaryName(), m_keepLaserCheck->isChecked());
}

void ExtrinsicCalibrationWidget::onClearSamplesClicked() {
    m_samples.clear();
    refreshSampleTable();
    updateSampleActions();
    appendLog("已清空标定样本。");
}

void ExtrinsicCalibrationWidget::onConnectRobotClicked() {
    m_robotConfig.ip = m_ipEdit->text().trimmed().toStdString();
    m_robotConfig.connectRobot = true;
    m_robotConfig.enableRobotMotion = false;
    m_robotConfig.autoEnable = false;
    m_robotConfig.autoMode = false;
    if (m_robot.connectRobot(m_robotConfig)) {
        setRobotUi(true);
        appendLog(QStringLiteral("已连接法奥机械臂: %1").arg(m_ipEdit->text()));
    } else {
        setRobotUi(false);
        appendLog("连接法奥机械臂失败。");
    }
}

void ExtrinsicCalibrationWidget::onDisconnectRobotClicked() {
    m_robot.disconnectRobot();
    setRobotUi(false);
    appendLog("已断开法奥机械臂。");
}

void ExtrinsicCalibrationWidget::onReadFlangeClicked() {
    readCurrentFlangePose();
}

void ExtrinsicCalibrationWidget::onLeftEyeImageReady(QImage image, QString desc) {
    if (!image.isNull()) {
        m_imageLabel->setImage(displayImageForPreview(image));
    }
    m_imageInfoLabel->setText(desc);
    appendLog(QStringLiteral("左目图像已更新: %1").arg(desc));
}

void ExtrinsicCalibrationWidget::onCharucoResult(bool ok, QImage image, QString desc,
                                                 QVector<double> tLeftBoard, QString report) {
    if (!image.isNull()) {
        m_imageLabel->setImage(displayImageForPreview(image));
    }
    m_imageInfoLabel->setText(desc);
    appendLog(report);

    if (!m_pendingAddSample) {
        return;
    }

    m_pendingAddSample = false;
    if (!ok) {
        appendLog("样本未添加：标定板检测失败。");
        return;
    }

    cv::Mat tLeftBoardMat = mat4FromVector(tLeftBoard);
    if (tLeftBoardMat.empty()) {
        appendLog("样本未添加：左目到标定板矩阵无效。");
        return;
    }
    addSample(m_pendingFlangePose, tLeftBoardMat, desc);
}

void ExtrinsicCalibrationWidget::onWorkerLog(QString msg) {
    appendLog(msg);
}

void ExtrinsicCalibrationWidget::onWorkerBusyChanged(bool busy) {
    setCameraUi(busy);
}

void ExtrinsicCalibrationWidget::addSample(const weld_geometry::Pose6D& pose,
                                           const cv::Mat& tLeftBoard,
                                           const QString& desc) {
    CalibrationSample sample;
    sample.flangePose = pose;
    sample.tLeftBoard = tLeftBoard.clone();
    sample.desc = desc;
    m_samples.push_back(sample);
    refreshSampleTable();
    updateSampleActions();
    appendLog(QStringLiteral("样本 #%1 已添加。").arg(m_samples.size()));
}

void ExtrinsicCalibrationWidget::refreshSampleTable() {
    m_sampleTable->setRowCount(m_samples.size());
    for (int row = 0; row < m_samples.size(); ++row) {
        const CalibrationSample& sample = m_samples[row];
        const weld_geometry::Pose6D& p = sample.flangePose;
        const double boardZ = sample.tLeftBoard.empty() ? 0.0 : sample.tLeftBoard.at<double>(2, 3);
        const QStringList values = {
            QString::number(row + 1),
            QString::number(p.x, 'f', 3),
            QString::number(p.y, 'f', 3),
            QString::number(p.z, 'f', 3),
            QString::number(p.rx, 'f', 3),
            QString::number(p.ry, 'f', 3),
            QString::number(p.rz, 'f', 3),
            QString::number(boardZ, 'f', 3)
        };
        for (int col = 0; col < values.size(); ++col) {
            m_sampleTable->setItem(row, col, new QTableWidgetItem(values[col]));
        }
    }
}

void ExtrinsicCalibrationWidget::updateSampleActions() {
    m_btnClearSamples->setEnabled(!m_samples.isEmpty() && !m_cameraBusy);
    m_btnSolve->setEnabled(m_samples.size() >= 3 && !m_cameraBusy);
}

void ExtrinsicCalibrationWidget::onSolveClicked() {
    if (m_samples.size() < 3) {
        QMessageBox::warning(this, "样本不足", "至少需要 3 组不同姿态样本，建议 8 组以上。");
        return;
    }

    std::vector<cv::Mat> rGripper2Base;
    std::vector<cv::Mat> tGripper2Base;
    std::vector<cv::Mat> rTarget2Cam;
    std::vector<cv::Mat> tTarget2Cam;
    rGripper2Base.reserve(static_cast<size_t>(m_samples.size()));
    tGripper2Base.reserve(static_cast<size_t>(m_samples.size()));
    rTarget2Cam.reserve(static_cast<size_t>(m_samples.size()));
    tTarget2Cam.reserve(static_cast<size_t>(m_samples.size()));

    for (int i = 0; i < m_samples.size(); ++i) {
        appendRt(cvMatFromPose(m_samples[i].flangePose), &rGripper2Base, &tGripper2Base);
        appendRt(m_samples[i].tLeftBoard, &rTarget2Cam, &tTarget2Cam);
    }

    cv::Mat rCam2Gripper;
    cv::Mat tCam2Gripper;
    try {
        cv::calibrateHandEye(rGripper2Base, tGripper2Base,
                             rTarget2Cam, tTarget2Cam,
                             rCam2Gripper, tCam2Gripper,
                             cv::CALIB_HAND_EYE_TSAI);
    } catch (const cv::Exception& e) {
        appendLog(QStringLiteral("手眼解算失败: %1").arg(QString::fromLocal8Bit(e.what())));
        return;
    }

    const cv::Mat tFlangeLeft = mat4FromRt(rCam2Gripper, tCam2Gripper);
    std::vector<cv::Mat> tBaseBoard;
    tBaseBoard.reserve(static_cast<size_t>(m_samples.size()));
    cv::Mat meanT = cv::Mat::zeros(3, 1, CV_64F);
    for (int i = 0; i < m_samples.size(); ++i) {
        const cv::Mat tBaseFlange = cvMatFromPose(m_samples[i].flangePose);
        const cv::Mat boardPose = tBaseFlange * tFlangeLeft * m_samples[i].tLeftBoard;
        tBaseBoard.push_back(boardPose);
        meanT += boardPose(cv::Rect(3, 0, 1, 3));
    }
    meanT /= static_cast<double>(tBaseBoard.size());

    double transSq = 0.0;
    double transMax = 0.0;
    double rotSq = 0.0;
    double rotMax = 0.0;
    const cv::Mat rRef = tBaseBoard.front()(cv::Rect(0, 0, 3, 3)).clone();
    for (size_t i = 0; i < tBaseBoard.size(); ++i) {
        const cv::Mat delta = tBaseBoard[i](cv::Rect(3, 0, 1, 3)) - meanT;
        const double trans = cv::norm(delta);
        transSq += trans * trans;
        transMax = std::max(transMax, trans);

        const cv::Mat rNow = tBaseBoard[i](cv::Rect(0, 0, 3, 3));
        const double rot = rotationAngleDeg(rRef.t() * rNow);
        rotSq += rot * rot;
        rotMax = std::max(rotMax, rot);
    }

    const double transRms = std::sqrt(transSq / static_cast<double>(tBaseBoard.size()));
    const double rotRms = std::sqrt(rotSq / static_cast<double>(tBaseBoard.size()));

    const QString summary = QStringLiteral(
        "样本数=%1\n"
        "输出矩阵: T_flange_left_camera，单位 mm，可用于 handeye_config.yaml 的 mode: camera_to_flange\n"
        "T_flange_left_camera:\n%2\n"
        "标定板 base 坐标一致性误差: 平移 RMS=%3 mm, Max=%4 mm; 旋转 RMS=%5 deg, Max=%6 deg")
        .arg(m_samples.size())
        .arg(formatMatrix4(tFlangeLeft))
        .arg(transRms, 0, 'f', 4)
        .arg(transMax, 0, 'f', 4)
        .arg(rotRms, 0, 'f', 4)
        .arg(rotMax, 0, 'f', 4);

    const QString path = saveResultYaml(tFlangeLeft, summary);
    appendLog(summary);
    if (!path.isEmpty()) {
        appendLog(QStringLiteral("标定结果已保存: %1").arg(path));
    }
}

QString ExtrinsicCalibrationWidget::saveResultYaml(const cv::Mat& tFlangeLeft, const QString& summary) const {
    QDir dir(dataDirPath());
    const QString path = dir.absoluteFilePath(QStringLiteral("left_handeye_%1.yaml")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")));
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return {};
    }
    QTextStream out(&file);
    out << "# Generated by VizumScanGUI left-eye extrinsic calibration\n";
    out << "# " << QDateTime::currentDateTime().toString(Qt::ISODate) << "\n";
    out << "# Matrix is T_flange_left_camera, units are mm.\n";
    out << "mode: camera_to_flange\n\n";
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out << QStringLiteral("m%1%2: %3\n")
                .arg(r).arg(c)
                .arg(tFlangeLeft.at<double>(r, c), 0, 'f', 9);
        }
        out << "\n";
    }
    out << "# Summary:\n";
    const QStringList lines = summary.split('\n');
    for (const QString& line : lines) {
        out << "# " << line << "\n";
    }
    return path;
}

QString ExtrinsicCalibrationWidget::dataDirPath() const {
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
