#include "HikCalibrationWindow.h"

#include "HikCameraWorker.h"
#include "FairinoRobotSession.h"
#include "ImageView.h"

#include <QAbstractItemView>
#include <QCloseEvent>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>

#ifndef HIK_CALIBRATION_SOURCE_DIR
#define HIK_CALIBRATION_SOURCE_DIR "."
#endif

namespace {

const double kApprovedIntrinsicRmsPx = 0.45;
const double kApprovedIntrinsicViewP95Px = 0.60;
const double kApprovedIntrinsicViewMaxPx = 0.80;
const double kApprovedPlaneRmsMm = 0.20;
const double kApprovedPlaneP95Mm = 0.30;
const double kApprovedHandEyeTranslationRmsMm = 1.0;
const double kApprovedHandEyeRotationRmsDeg = 0.30;
const double kMaximumRobotStillTranslationMm = 0.10;
const double kMaximumRobotStillRotationDeg = 0.05;
const double kLaserGuidanceMinimumXSpanMm = 20.0;
const double kLaserGuidanceMinimumYSpanMm = 20.0;
const double kLaserGuidanceTiltSignThresholdDeg = 2.0;
const double kLaserGuidanceMinimumStripeSpanFraction = 0.06;

hik_stripe::Orientation calibrationStripeOrientation(
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

hik_calibration::StripeExtractionMode calibrationStripeMode(
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

cv::Rect calibrationStripeRoi(
        const LineLaserDeviceProfile& profile,
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

struct LaserGuidanceBinSpec {
    double targetDepthMm;
    double minimumDepthMm;
    double maximumDepthMm;
    int targetCount;
};

const std::array<LaserGuidanceBinSpec, 6> kLaserGuidanceBins = {{
    {300.0, 260.0, 350.0, 5},
    {400.0, 350.0, 475.0, 4},
    {550.0, 475.0, 625.0, 4},
    {700.0, 625.0, 775.0, 4},
    {850.0, 775.0, 925.0, 4},
    {1000.0, 925.0, 1040.0, 5}
}};

struct LaserGuidanceObservation {
    bool valid{false};
    int binIndex{-1};
    double medianDepthMm{std::numeric_limits<double>::quiet_NaN()};
    double boardXmm{std::numeric_limits<double>::quiet_NaN()};
    double boardYmm{std::numeric_limits<double>::quiet_NaN()};
    double tiltXDeg{std::numeric_limits<double>::quiet_NaN()};
    double tiltYDeg{std::numeric_limits<double>::quiet_NaN()};
    double stripeUMedianPx{std::numeric_limits<double>::quiet_NaN()};
    double stripeVMedianPx{std::numeric_limits<double>::quiet_NaN()};
};

struct LaserGuidanceBinStatistics {
    int count{0};
    double minimumXmm{std::numeric_limits<double>::max()};
    double maximumXmm{-std::numeric_limits<double>::max()};
    double minimumYmm{std::numeric_limits<double>::max()};
    double maximumYmm{-std::numeric_limits<double>::max()};
    double minimumTiltXDeg{std::numeric_limits<double>::max()};
    double maximumTiltXDeg{-std::numeric_limits<double>::max()};
    double minimumTiltYDeg{std::numeric_limits<double>::max()};
    double maximumTiltYDeg{-std::numeric_limits<double>::max()};
    double minimumStripeUPx{std::numeric_limits<double>::max()};
    double maximumStripeUPx{-std::numeric_limits<double>::max()};
    double minimumStripeVPx{std::numeric_limits<double>::max()};
    double maximumStripeVPx{-std::numeric_limits<double>::max()};
};

struct LaserGuidanceStatistics {
    std::array<LaserGuidanceBinStatistics, 6> bins;
    int outsideDepthCount{0};
};

double percentileValue(std::vector<double> values, double probability) {
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    const double position = std::max(0.0, std::min(1.0, probability)) *
                            static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

int laserGuidanceBinIndex(double depthMm) {
    if (!std::isfinite(depthMm)) {
        return -1;
    }
    for (std::size_t index = 0; index < kLaserGuidanceBins.size(); ++index) {
        const LaserGuidanceBinSpec& bin = kLaserGuidanceBins[index];
        const bool insideUpperBound = index + 1U == kLaserGuidanceBins.size()
            ? depthMm <= bin.maximumDepthMm
            : depthMm < bin.maximumDepthMm;
        if (depthMm >= bin.minimumDepthMm && insideUpperBound) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

LaserGuidanceObservation laserGuidanceObservation(
        const hik_calibration::LaserCalibrationPairResult& pair,
        const hik_calibration::BoardSpec& board) {
    LaserGuidanceObservation observation;
    if (!pair.ok || !pair.laserOffPose.ok || pair.cameraPointsMm.empty()) {
        return observation;
    }

    std::vector<double> depths;
    depths.reserve(pair.cameraPointsMm.size());
    for (std::size_t index = 0; index < pair.cameraPointsMm.size(); ++index) {
        if (std::isfinite(pair.cameraPointsMm[index].z)) {
            depths.push_back(pair.cameraPointsMm[index].z);
        }
    }
    std::vector<double> stripeU;
    std::vector<double> stripeV;
    stripeU.reserve(pair.stripe.size());
    stripeV.reserve(pair.stripe.size());
    for (std::size_t index = 0; index < pair.stripe.size(); ++index) {
        if (std::isfinite(pair.stripe[index].pixel.x) &&
            std::isfinite(pair.stripe[index].pixel.y)) {
            stripeU.push_back(pair.stripe[index].pixel.x);
            stripeV.push_back(pair.stripe[index].pixel.y);
        }
    }
    if (depths.empty() || stripeU.empty() ||
        !std::isfinite(pair.laserOffPose.tvec[0]) ||
        !std::isfinite(pair.laserOffPose.tvec[1])) {
        return observation;
    }

    cv::Vec3d normal(pair.laserOffPose.rotation(0, 2),
                     pair.laserOffPose.rotation(1, 2),
                     pair.laserOffPose.rotation(2, 2));
    const double normalLength = cv::norm(normal);
    if (!std::isfinite(normalLength) || normalLength <= 1.0e-12) {
        return observation;
    }
    normal /= normalLength;
    if (normal[2] < 0.0) {
        normal = -normal;
    }

    observation.medianDepthMm = percentileValue(depths, 0.5);
    observation.binIndex = laserGuidanceBinIndex(observation.medianDepthMm);
    const cv::Vec3d boardCenterInCamera =
        pair.laserOffPose.rotation *
            cv::Vec3d(board.widthMm() * 0.5, board.heightMm() * 0.5, 0.0) +
        pair.laserOffPose.tvec;
    observation.boardXmm = boardCenterInCamera[0];
    observation.boardYmm = boardCenterInCamera[1];
    observation.tiltXDeg = std::atan2(normal[0], normal[2]) * 180.0 / CV_PI;
    observation.tiltYDeg = std::atan2(normal[1], normal[2]) * 180.0 / CV_PI;
    observation.stripeUMedianPx = percentileValue(stripeU, 0.5);
    observation.stripeVMedianPx = percentileValue(stripeV, 0.5);
    observation.valid = std::isfinite(observation.medianDepthMm) &&
                        std::isfinite(observation.tiltXDeg) &&
                        std::isfinite(observation.tiltYDeg) &&
                        std::isfinite(observation.stripeUMedianPx) &&
                        std::isfinite(observation.stripeVMedianPx);
    return observation;
}

void addLaserGuidanceObservation(
        const LaserGuidanceObservation& observation,
        LaserGuidanceBinStatistics* statistics) {
    if (!statistics || !observation.valid) {
        return;
    }
    ++statistics->count;
    statistics->minimumXmm = std::min(statistics->minimumXmm, observation.boardXmm);
    statistics->maximumXmm = std::max(statistics->maximumXmm, observation.boardXmm);
    statistics->minimumYmm = std::min(statistics->minimumYmm, observation.boardYmm);
    statistics->maximumYmm = std::max(statistics->maximumYmm, observation.boardYmm);
    statistics->minimumTiltXDeg =
        std::min(statistics->minimumTiltXDeg, observation.tiltXDeg);
    statistics->maximumTiltXDeg =
        std::max(statistics->maximumTiltXDeg, observation.tiltXDeg);
    statistics->minimumTiltYDeg =
        std::min(statistics->minimumTiltYDeg, observation.tiltYDeg);
    statistics->maximumTiltYDeg =
        std::max(statistics->maximumTiltYDeg, observation.tiltYDeg);
    statistics->minimumStripeUPx =
        std::min(statistics->minimumStripeUPx, observation.stripeUMedianPx);
    statistics->maximumStripeUPx =
        std::max(statistics->maximumStripeUPx, observation.stripeUMedianPx);
    statistics->minimumStripeVPx =
        std::min(statistics->minimumStripeVPx, observation.stripeVMedianPx);
    statistics->maximumStripeVPx =
        std::max(statistics->maximumStripeVPx, observation.stripeVMedianPx);
}

LaserGuidanceStatistics summarizeLaserGuidance(
        const std::vector<const hik_calibration::LaserCalibrationPairResult*>& pairs,
        const hik_calibration::BoardSpec& board) {
    LaserGuidanceStatistics statistics;
    for (std::size_t index = 0; index < pairs.size(); ++index) {
        if (!pairs[index] || !pairs[index]->ok) {
            continue;
        }
        const LaserGuidanceObservation observation =
            laserGuidanceObservation(*pairs[index], board);
        if (!observation.valid) {
            continue;
        }
        if (observation.binIndex < 0) {
            ++statistics.outsideDepthCount;
            continue;
        }
        addLaserGuidanceObservation(
            observation,
            &statistics.bins[static_cast<std::size_t>(observation.binIndex)]);
    }
    return statistics;
}

double laserGuidanceStripeSpanFraction(
        const LaserGuidanceBinStatistics& statistics,
        const cv::Size& imageSize) {
    if (statistics.count <= 0 || imageSize.width <= 0 || imageSize.height <= 0) {
        return 0.0;
    }
    const double uFraction =
        (statistics.maximumStripeUPx - statistics.minimumStripeUPx) /
        static_cast<double>(imageSize.width);
    const double vFraction =
        (statistics.maximumStripeVPx - statistics.minimumStripeVPx) /
        static_cast<double>(imageSize.height);
    return std::max(uFraction, vFraction);
}

QString laserGuidanceBinIssue(
        std::size_t binIndex,
        const LaserGuidanceBinStatistics& statistics,
        const cv::Size& imageSize) {
    const LaserGuidanceBinSpec& spec = kLaserGuidanceBins[binIndex];
    if (statistics.count < spec.targetCount) {
        return QStringLiteral("数量 %1/%2").arg(statistics.count).arg(spec.targetCount);
    }
    if (statistics.maximumXmm - statistics.minimumXmm <
        kLaserGuidanceMinimumXSpanMm) {
        return QStringLiteral("补左右(X)位置");
    }
    if (statistics.maximumYmm - statistics.minimumYmm <
        kLaserGuidanceMinimumYSpanMm) {
        return QStringLiteral("补上下(Y)位置");
    }
    if (statistics.minimumTiltXDeg > -kLaserGuidanceTiltSignThresholdDeg ||
        statistics.maximumTiltXDeg < kLaserGuidanceTiltSignThresholdDeg) {
        return QStringLiteral("补法向X正/负倾角");
    }
    if (statistics.minimumTiltYDeg > -kLaserGuidanceTiltSignThresholdDeg ||
        statistics.maximumTiltYDeg < kLaserGuidanceTiltSignThresholdDeg) {
        return QStringLiteral("补法向Y正/负倾角");
    }
    if (laserGuidanceStripeSpanFraction(statistics, imageSize) <
        kLaserGuidanceMinimumStripeSpanFraction) {
        return QStringLiteral("补激光线画面位置");
    }
    return QString();
}

double angleBetweenNormalsDeg(const cv::Vec3d& first, const cv::Vec3d& second) {
    const double denominator = cv::norm(first) * cv::norm(second);
    if (!std::isfinite(denominator) || denominator <= 1.0e-12) {
        return 0.0;
    }
    const double cosine = std::max(0.0, std::min(
        1.0, std::fabs(first.dot(second) / denominator)));
    return std::acos(cosine) * 180.0 / CV_PI;
}

bool isValidIpv4(const QString& text) {
    const QStringList parts = text.trimmed().split(QLatin1Char('.'));
    if (parts.size() != 4) {
        return false;
    }
    for (int index = 0; index < parts.size(); ++index) {
        bool ok = false;
        const int value = parts[index].toInt(&ok, 10);
        if (!ok || parts[index].isEmpty() || value < 0 || value > 255) {
            return false;
        }
    }
    return true;
}

QString fileSha256Hex(const QString& path, QString* error = nullptr) {
    if (error) {
        error->clear();
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error) {
            *error = QStringLiteral("无法读取文件以计算 SHA-256: %1").arg(path);
        }
        return QString();
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while (!file.atEnd()) {
        const QByteArray chunk = file.read(1024 * 1024);
        if (chunk.isEmpty() && file.error() != QFileDevice::NoError) {
            if (error) {
                *error = QStringLiteral("读取文件计算 SHA-256 失败: %1").arg(path);
            }
            return QString();
        }
        hash.addData(chunk);
    }
    return QString::fromLatin1(hash.result().toHex());
}

QByteArray csvField(const QString& value) {
    QString escaped = value;
    escaped.replace(QLatin1Char('"'), QStringLiteral("\"\""));
    return QStringLiteral("\"%1\"").arg(escaped).toUtf8();
}

QString formatNumber(double value, int decimals = 3) {
    return std::isfinite(value) ? QString::number(value, 'f', decimals)
                                : QStringLiteral("-");
}

double averageAngleDeg(double first, double second) {
    const double firstRad = first * CV_PI / 180.0;
    const double secondRad = second * CV_PI / 180.0;
    return std::atan2(std::sin(firstRad) + std::sin(secondRad),
                      std::cos(firstRad) + std::cos(secondRad)) * 180.0 / CV_PI;
}

QString formatTransform(const cv::Matx44d& transform) {
    QStringList rows;
    for (int row = 0; row < 4; ++row) {
        rows << QStringLiteral("[%1, %2, %3, %4]")
            .arg(transform(row, 0), 0, 'f', 9)
            .arg(transform(row, 1), 0, 'f', 9)
            .arg(transform(row, 2), 0, 'f', 9)
            .arg(transform(row, 3), 0, 'f', 9);
    }
    return rows.join(QLatin1Char('\n'));
}

QString fileNameOnly(const QString& path) {
    return QFileInfo(path).fileName();
}

cv::Mat gray8Clone(const cv::Mat& source) {
    if (source.empty()) {
        return cv::Mat();
    }

    cv::Mat gray;
    if (source.channels() == 1) {
        gray = source;
    } else if (source.channels() == 3) {
        cv::cvtColor(source, gray, cv::COLOR_BGR2GRAY);
    } else if (source.channels() == 4) {
        cv::cvtColor(source, gray, cv::COLOR_BGRA2GRAY);
    } else {
        return cv::Mat();
    }

    if (gray.depth() == CV_8U) {
        return gray.clone();
    }

    double minimum = 0.0;
    double maximum = 0.0;
    cv::minMaxLoc(gray, &minimum, &maximum);
    cv::Mat converted;
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || maximum <= minimum) {
        gray.convertTo(converted, CV_8U);
    } else {
        gray.convertTo(converted, CV_8U,
                       255.0 / (maximum - minimum),
                       -minimum * 255.0 / (maximum - minimum));
    }
    return converted.clone();
}

QString nextSessionDirectory(const QString& calibrationRoot) {
    const QString base = QStringLiteral("session_%1")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
    QDir root(calibrationRoot);
    for (int index = 0; index < 1000; ++index) {
        const QString name = index == 0
            ? base
            : base + QStringLiteral("_%1").arg(index, 3, 10, QLatin1Char('0'));
        if (!QFileInfo(root.absoluteFilePath(name)).exists()) {
            return root.absoluteFilePath(name);
        }
    }
    return QString();
}

} // namespace

HikCalibrationWindow::HikCalibrationWindow(
        const LineLaserDeviceProfile& profile,
        FairinoRobotSession* robotSession,
        QWidget* parent)
    : QMainWindow(parent),
      robotSession_(robotSession),
      profile_(profile),
      sourceDir_(QDir::cleanPath(QString::fromUtf8(HIK_CALIBRATION_SOURCE_DIR))) {
    QString profileError;
    if (!profile_.isValid(&profileError)) {
        throw std::invalid_argument(profileError.toStdString());
    }
    if (!robotSession_) {
        throw std::invalid_argument(
            "FairinoRobotSession must be shared with the calibration window");
    }
    // GUI approval is intentionally stricter than the numerical core.  A result
    // that can be solved may still be retained as a candidate without being
    // allowed to replace the formal calibration files.
    detectionOptions_.minLaplacianVariance = 30.0;
    intrinsicOptions_.minViews = 15;
    laserPairOptions_.detection = detectionOptions_;
    laserPairOptions_.stripe.mode =
        calibrationStripeMode(
            profile_.calibrationCenterlinePolicy);
    laserPairOptions_.stripe.quality.orientation =
        calibrationStripeOrientation(
            profile_.stripeOrientation);
    laserPairOptions_.pose.maxRmsErrorPx = 0.40;
    laserPairOptions_.pose.maxPointErrorPx = 0.80;
    planeOptions_.minPoseCount = 8;
    planeOptions_.minPointsPerPose = 60;
    planeOptions_.minInlierRatio = 0.90;
    profileOptions_.boardDetection = detectionOptions_;
    profileOptions_.boardPose.maxRmsErrorPx = 0.40;
    profileOptions_.boardPose.maxPointErrorPx = 0.80;
    handEyeBoardPoseOptions_.minCorners = 12;
    handEyeBoardPoseOptions_.maxRmsErrorPx = 0.40;
    handEyeBoardPoseOptions_.maxPointErrorPx = 0.80;
    handEyeOptions_.minSamples = 15;

    buildUi();
    refreshLaserGuidance();

    QString sessionError;
    sessionReady_ = createSessionDirectories(&sessionError);
    if (sessionReady_) {
        appendLog(QStringLiteral("本次标定设备=%1，会话=%2")
                  .arg(profile_.id, sessionDir_));
        appendLog(QStringLiteral(
            "条纹标定策略：方向=%1，模式=%2；算法定义变化会写入会话，"
            "不得与另一设备组或旧算法的激光平面混用。")
            .arg(lineLaserStripeOrientationName(profile_.stripeOrientation),
                 lineLaserCenterlinePolicyName(
                     profile_.calibrationCenterlinePolicy)));
    } else {
        appendLog(sessionError);
        imageInfoLabel_->setText(sessionError);
    }

    QString profileCalibrationError;
    if (loadFormalProfileCalibration(&profileCalibrationError)) {
        appendLog(QStringLiteral("%1 静态轮廓已加载正式内参与激光平面。")
                  .arg(profile_.id));
    } else {
        appendLog(QStringLiteral("静态轮廓正式标定未就绪: %1").arg(profileCalibrationError));
    }

    QString handEyeIntrinsicsError;
    if (loadFormalHandEyeIntrinsics(&handEyeIntrinsicsError)) {
        appendLog(QStringLiteral("%1 手眼标定已加载正式内参。")
                  .arg(profile_.id));
    } else {
        appendLog(QStringLiteral("手眼标定内参未就绪: %1").arg(handEyeIntrinsicsError));
    }

    setupRobotSession();
    setupCameraWorker();
    appendLog(QStringLiteral("板参数默认采用 OpenCV squaresX=5、squaresY=7、方格=22 mm、Marker=16 mm、DICT_4X4_50。"));
    appendLog(QStringLiteral("内参采样时请关闭 %1 nm 线激光；激光面采样必须严格保持 laser-off/on 两帧之间标定板不动。")
              .arg(profile_.wavelengthNm));
    updateAllUiStates();
}

HikCalibrationWindow::~HikCalibrationWindow() {
    shutdownRobotSession();
    shutdownCameraWorker();
}

void HikCalibrationWindow::buildUi() {
    setWindowTitle(QStringLiteral("海康线激光标定｜%1｜%2")
                   .arg(profile_.id, profile_.displayName));
    resize(1560, 920);

    QWidget* central = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(central);
    root->addWidget(buildCameraBar());
    root->addWidget(buildBoardBar());

    QSplitter* splitter = new QSplitter(Qt::Horizontal, central);
    calibrationTabs_ = new QTabWidget(splitter);
    calibrationTabs_->addTab(buildIntrinsicPage(), QStringLiteral("相机内参"));
    calibrationTabs_->addTab(buildLaserPage(), QStringLiteral("激光平面"));
    calibrationTabs_->addTab(buildProfilePage(), QStringLiteral("静态三维轮廓"));
    calibrationTabs_->addTab(buildHandEyePage(), QStringLiteral("FR5 手眼标定"));
    splitter->addWidget(calibrationTabs_);
    splitter->addWidget(buildPreviewPanel());
    splitter->setStretchFactor(0, 3);
    splitter->setStretchFactor(1, 4);
    root->addWidget(splitter, 1);
    setCentralWidget(central);

    connect(connectButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::connectCamera);
    connect(disconnectButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::disconnectCamera);
    connect(singleFrameButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureManualFrame);
    connect(expectedCameraModelEdit_, &QLineEdit::textChanged,
            this, &HikCalibrationWindow::updateAllUiStates);
    connect(expectedCameraSerialEdit_, &QLineEdit::textChanged,
            this, &HikCalibrationWindow::updateAllUiStates);
    connect(swapBoardButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::swapBoardAxes);

    connect(captureIntrinsicButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureIntrinsicSample);
    connect(importIntrinsicButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::importIntrinsicImages);
    connect(deleteIntrinsicButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::deleteSelectedIntrinsicSamples);
    connect(clearIntrinsicButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::clearIntrinsicSamples);
    connect(solveIntrinsicButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::solveIntrinsics);
    connect(saveIntrinsicCandidateButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::saveIntrinsicCandidate);
    connect(saveIntrinsicApprovedButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::saveApprovedIntrinsics);

    connect(loadLaserIntrinsicsButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::loadIntrinsicsForLaser);
    connect(useCurrentIntrinsicsButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::useCurrentIntrinsicsForLaser);
    connect(captureLaserOffButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureLaserOff);
    connect(captureLaserOnButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureLaserOn);
    connect(cancelLaserPairButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::cancelPendingLaserPair);
    connect(importLaserPairsButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::importLaserPairs);
    connect(deleteLaserPairButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::deleteSelectedLaserPairs);
    connect(clearLaserPairsButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::clearLaserPairs);
    connect(solveLaserButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::solveLaserPlane);
    connect(saveLaserCandidateButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::saveLaserCandidate);
    connect(saveLaserApprovedButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::saveApprovedLaserPlane);
    connect(reloadProfileCalibrationButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::reloadProfileCalibration);
    connect(captureProfileOffButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureProfileOff);
    connect(captureProfileOnButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureProfileOn);
    connect(cancelProfilePairButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::cancelPendingProfilePair);
    connect(importProfilePairButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::importProfilePair);
    connect(connectRobotButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::connectRobot);
    connect(disconnectRobotButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::disconnectRobot);
    connect(readRobotPoseButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::readRobotPose);
    connect(captureHandEyeButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::captureHandEyeSample);
    connect(deleteHandEyeButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::deleteSelectedHandEyeSamples);
    connect(clearHandEyeButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::clearHandEyeSamples);
    connect(solveHandEyeButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::solveHandEye);
    connect(saveHandEyeCandidateButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::saveHandEyeCandidate);
    connect(saveHandEyeApprovedButton_, &QPushButton::clicked,
            this, &HikCalibrationWindow::saveApprovedHandEye);
    connect(ransacThresholdSpin_,
            static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this, [this](double) {
                if (hasLaserPlaneResult_) {
                    invalidateLaserSolution();
                    appendLog(QStringLiteral("RANSAC 阈值已改变，需重新求解激光平面。"));
                    updateAllUiStates();
                }
            });

    connect(intrinsicTable_, &QTableWidget::itemSelectionChanged,
            this, &HikCalibrationWindow::updateAllUiStates);
    connect(laserTable_, &QTableWidget::itemSelectionChanged,
            this, &HikCalibrationWindow::updateAllUiStates);
    connect(handEyeTable_, &QTableWidget::itemSelectionChanged,
            this, &HikCalibrationWindow::updateAllUiStates);
}

QWidget* HikCalibrationWindow::buildCameraBar() {
    QGroupBox* group = new QGroupBox(
        QStringLiteral("海康相机｜%1").arg(profile_.displayName), this);
    QGridLayout* layout = new QGridLayout(group);

    cameraIpEdit_ = new QLineEdit(profile_.defaultCameraIp, group);
    cameraIpEdit_->setPlaceholderText(QStringLiteral("输入本设备的相机 IPv4 地址"));
    exposureSpin_ = new QDoubleSpinBox(group);
    exposureSpin_->setRange(1.0, 10000000.0);
    exposureSpin_->setDecimals(3);
    exposureSpin_->setValue(1825.0);
    exposureSpin_->setSuffix(QStringLiteral(" us"));
    gainSpin_ = new QDoubleSpinBox(group);
    gainSpin_->setRange(0.0, 48.0);
    gainSpin_->setDecimals(3);
    gainSpin_->setValue(0.0);
    gainSpin_->setSuffix(QStringLiteral(" dB"));
    timeoutSpin_ = new QSpinBox(group);
    timeoutSpin_->setRange(100, 10000);
    timeoutSpin_->setValue(3000);
    timeoutSpin_->setSuffix(QStringLiteral(" ms"));
    expectedCameraModelEdit_ = new QLineEdit(profile_.expectedCameraModel, group);
    expectedCameraModelEdit_->setPlaceholderText(
        QStringLiteral("可选；连接后按相机实际型号绑定"));
    expectedCameraModelEdit_->setReadOnly(!profile_.expectedCameraModel.isEmpty());
    expectedCameraSerialEdit_ = new QLineEdit(profile_.expectedCameraSerial, group);
    expectedCameraSerialEdit_->setPlaceholderText(
        QStringLiteral("建议填写；用于阻止两套相机串用"));
    expectedCameraSerialEdit_->setReadOnly(!profile_.expectedCameraSerial.isEmpty());

    cameraStatusLabel_ = new QLabel(QStringLiteral("未连接"), group);
    cameraStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    connectButton_ = new QPushButton(QStringLiteral("连接"), group);
    disconnectButton_ = new QPushButton(QStringLiteral("断开"), group);
    singleFrameButton_ = new QPushButton(QStringLiteral("单帧"), group);

    layout->addWidget(new QLabel(QStringLiteral("IP"), group), 0, 0);
    layout->addWidget(cameraIpEdit_, 0, 1);
    layout->addWidget(new QLabel(QStringLiteral("曝光"), group), 0, 2);
    layout->addWidget(exposureSpin_, 0, 3);
    layout->addWidget(new QLabel(QStringLiteral("增益"), group), 0, 4);
    layout->addWidget(gainSpin_, 0, 5);
    layout->addWidget(new QLabel(QStringLiteral("超时"), group), 0, 6);
    layout->addWidget(timeoutSpin_, 0, 7);
    layout->addWidget(connectButton_, 0, 8);
    layout->addWidget(disconnectButton_, 0, 9);
    layout->addWidget(singleFrameButton_, 0, 10);
    layout->addWidget(cameraStatusLabel_, 0, 11);
    layout->addWidget(new QLabel(QStringLiteral("预期型号"), group), 1, 0);
    layout->addWidget(expectedCameraModelEdit_, 1, 1, 1, 3);
    layout->addWidget(new QLabel(QStringLiteral("预期 SN"), group), 1, 4);
    layout->addWidget(expectedCameraSerialEdit_, 1, 5, 1, 3);
    QLabel* profileLabel = new QLabel(
        QStringLiteral("profile=%1｜frame=%2｜TTL=物理 Pin %3｜正式内参=%4")
            .arg(profile_.id, profile_.cameraFrame)
            .arg(profile_.ttlPhysicalPin)
            .arg(profile_.intrinsicsConfigRelativePath),
        group);
    profileLabel->setWordWrap(true);
    profileLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(profileLabel, 1, 8, 1, 4);
    layout->setColumnStretch(1, 1);
    return group;
}

QWidget* HikCalibrationWindow::buildBoardBar() {
    QGroupBox* group = new QGroupBox(QStringLiteral("ChArUco 标定板（OpenCV 方格数定义）"), this);
    QGridLayout* layout = new QGridLayout(group);

    squaresXSpin_ = new QSpinBox(group);
    squaresXSpin_->setRange(2, 40);
    squaresXSpin_->setValue(5);
    squaresYSpin_ = new QSpinBox(group);
    squaresYSpin_->setRange(2, 40);
    squaresYSpin_->setValue(7);
    squareLengthSpin_ = new QDoubleSpinBox(group);
    squareLengthSpin_->setRange(0.1, 1000.0);
    squareLengthSpin_->setDecimals(3);
    squareLengthSpin_->setValue(22.0);
    squareLengthSpin_->setSuffix(QStringLiteral(" mm"));
    markerLengthSpin_ = new QDoubleSpinBox(group);
    markerLengthSpin_->setRange(0.1, 1000.0);
    markerLengthSpin_->setDecimals(3);
    markerLengthSpin_->setValue(16.0);
    markerLengthSpin_->setSuffix(QStringLiteral(" mm"));
    dictionaryCombo_ = new QComboBox(group);
    dictionaryCombo_->addItem(QStringLiteral("DICT_4X4_50"), cv::aruco::DICT_4X4_50);
    dictionaryCombo_->addItem(QStringLiteral("DICT_4X4_100"), cv::aruco::DICT_4X4_100);
    dictionaryCombo_->addItem(QStringLiteral("DICT_5X5_50"), cv::aruco::DICT_5X5_50);
    swapBoardButton_ = new QPushButton(QStringLiteral("无样本时切换 5×7 / 7×5"), group);
    boardLockLabel_ = new QLabel(group);
    boardLockLabel_->setWordWrap(true);

    layout->addWidget(new QLabel(QStringLiteral("OpenCV squaresX"), group), 0, 0);
    layout->addWidget(squaresXSpin_, 0, 1);
    layout->addWidget(new QLabel(QStringLiteral("squaresY"), group), 0, 2);
    layout->addWidget(squaresYSpin_, 0, 3);
    layout->addWidget(new QLabel(QStringLiteral("方格边长"), group), 0, 4);
    layout->addWidget(squareLengthSpin_, 0, 5);
    layout->addWidget(new QLabel(QStringLiteral("Marker 边长"), group), 0, 6);
    layout->addWidget(markerLengthSpin_, 0, 7);
    layout->addWidget(new QLabel(QStringLiteral("字典"), group), 0, 8);
    layout->addWidget(dictionaryCombo_, 0, 9);
    layout->addWidget(swapBoardButton_, 0, 10);
    layout->addWidget(boardLockLabel_, 1, 0, 1, 11);
    return group;
}

QWidget* HikCalibrationWindow::buildIntrinsicPage() {
    QWidget* page = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(page);

    QLabel* warning = new QLabel(
        QStringLiteral("实时内参采样：请关闭 %1 nm 线激光，并从不同距离、角度和画面位置采集清晰标定板。")
            .arg(profile_.wavelengthNm),
        page);
    warning->setWordWrap(true);
    warning->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    root->addWidget(warning);

    QHBoxLayout* actions = new QHBoxLayout;
    captureIntrinsicButton_ = new QPushButton(QStringLiteral("实时采样（激光关闭）"), page);
    importIntrinsicButton_ = new QPushButton(QStringLiteral("批量导入图片"), page);
    deleteIntrinsicButton_ = new QPushButton(QStringLiteral("删除选中"), page);
    clearIntrinsicButton_ = new QPushButton(QStringLiteral("清空"), page);
    solveIntrinsicButton_ = new QPushButton(QStringLiteral("求解内参"), page);
    actions->addWidget(captureIntrinsicButton_);
    actions->addWidget(importIntrinsicButton_);
    actions->addWidget(deleteIntrinsicButton_);
    actions->addWidget(clearIntrinsicButton_);
    actions->addStretch(1);
    actions->addWidget(solveIntrinsicButton_);
    root->addLayout(actions);

    intrinsicGateLabel_ = new QLabel(page);
    intrinsicGateLabel_->setWordWrap(true);
    root->addWidget(intrinsicGateLabel_);

    intrinsicTable_ = new QTableWidget(0, 12, page);
    intrinsicTable_->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("样本") << QStringLiteral("文件") << QStringLiteral("尺寸")
        << QStringLiteral("Marker") << QStringLiteral("角点") << QStringLiteral("覆盖率%")
        << QStringLiteral("边距px") << QStringLiteral("均值") << QStringLiteral("饱和%")
        << QStringLiteral("清晰度") << QStringLiteral("重投影RMS") << QStringLiteral("状态"));
    intrinsicTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    intrinsicTable_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    intrinsicTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    intrinsicTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    intrinsicTable_->horizontalHeader()->setStretchLastSection(true);
    root->addWidget(intrinsicTable_, 1);

    intrinsicResultLabel_ = new QLabel(QStringLiteral("尚未求解"), page);
    intrinsicResultLabel_->setWordWrap(true);
    root->addWidget(intrinsicResultLabel_);

    QHBoxLayout* saveActions = new QHBoxLayout;
    saveIntrinsicCandidateButton_ = new QPushButton(QStringLiteral("保存 session 候选 YAML"), page);
    saveIntrinsicApprovedButton_ = new QPushButton(
        QStringLiteral("保存通过结果到 %1")
            .arg(profile_.intrinsicsConfigRelativePath),
        page);
    saveActions->addWidget(saveIntrinsicCandidateButton_);
    saveActions->addWidget(saveIntrinsicApprovedButton_);
    saveActions->addStretch(1);
    root->addLayout(saveActions);
    return page;
}

QWidget* HikCalibrationWindow::buildLaserPage() {
    QWidget* page = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(page);

    QGroupBox* intrinsicsGroup = new QGroupBox(QStringLiteral("激光面使用的内参"), page);
    QGridLayout* intrinsicsLayout = new QGridLayout(intrinsicsGroup);
    laserIntrinsicsPathEdit_ = new QLineEdit(intrinsicsGroup);
    laserIntrinsicsPathEdit_->setReadOnly(true);
    loadLaserIntrinsicsButton_ = new QPushButton(QStringLiteral("加载内参 YAML"), intrinsicsGroup);
    useCurrentIntrinsicsButton_ = new QPushButton(QStringLiteral("沿用本页刚求出的内参"), intrinsicsGroup);
    laserIntrinsicsStatusLabel_ = new QLabel(QStringLiteral("未加载"), intrinsicsGroup);
    laserIntrinsicsStatusLabel_->setWordWrap(true);
    intrinsicsLayout->addWidget(laserIntrinsicsPathEdit_, 0, 0, 1, 4);
    intrinsicsLayout->addWidget(loadLaserIntrinsicsButton_, 0, 4);
    intrinsicsLayout->addWidget(useCurrentIntrinsicsButton_, 0, 5);
    intrinsicsLayout->addWidget(laserIntrinsicsStatusLabel_, 1, 0, 1, 6);
    root->addWidget(intrinsicsGroup);

    QGroupBox* pairGroup = new QGroupBox(QStringLiteral("严格静止成对采集"), page);
    QGridLayout* pairLayout = new QGridLayout(pairGroup);
    captureLaserOffButton_ = new QPushButton(QStringLiteral("1. 采 laser-off"), pairGroup);
    captureLaserOnButton_ = new QPushButton(QStringLiteral("2. 板不动，采 laser-on"), pairGroup);
    cancelLaserPairButton_ = new QPushButton(QStringLiteral("取消当前配对"), pairGroup);
    laserPairStateLabel_ = new QLabel(pairGroup);
    laserPairStateLabel_->setWordWrap(true);
    pairLayout->addWidget(captureLaserOffButton_, 0, 0);
    pairLayout->addWidget(captureLaserOnButton_, 0, 1);
    pairLayout->addWidget(cancelLaserPairButton_, 0, 2);
    pairLayout->addWidget(laserPairStateLabel_, 1, 0, 1, 3);
    root->addWidget(pairGroup);

    QGroupBox* guidanceGroup = new QGroupBox(
        QStringLiteral("300–1000 mm 采集引导（正式保存门槛）"), page);
    QVBoxLayout* guidanceLayout = new QVBoxLayout(guidanceGroup);
    laserGuidanceSummaryLabel_ = new QLabel(guidanceGroup);
    laserGuidanceSummaryLabel_->setWordWrap(true);
    guidanceLayout->addWidget(laserGuidanceSummaryLabel_);
    laserGuidanceTable_ = new QTableWidget(
        static_cast<int>(kLaserGuidanceBins.size()), 8, guidanceGroup);
    laserGuidanceTable_->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("目标Z") << QStringLiteral("分档范围")
        << QStringLiteral("有效组/目标") << QStringLiteral("板中心X跨度")
        << QStringLiteral("板中心Y跨度") << QStringLiteral("法向X/Y范围")
        << QStringLiteral("条纹U/V跨度") << QStringLiteral("下一项"));
    laserGuidanceTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    laserGuidanceTable_->setSelectionMode(QAbstractItemView::NoSelection);
    laserGuidanceTable_->verticalHeader()->setVisible(false);
    laserGuidanceTable_->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    laserGuidanceTable_->horizontalHeader()->setStretchLastSection(true);
    laserGuidanceTable_->setMaximumHeight(205);
    guidanceLayout->addWidget(laserGuidanceTable_);
    root->addWidget(guidanceGroup);

    QHBoxLayout* actions = new QHBoxLayout;
    importLaserPairsButton_ = new QPushButton(QStringLiteral("离线导入成对图"), page);
    deleteLaserPairButton_ = new QPushButton(QStringLiteral("删除选中"), page);
    clearLaserPairsButton_ = new QPushButton(QStringLiteral("清空"), page);
    actions->addWidget(importLaserPairsButton_);
    actions->addWidget(deleteLaserPairButton_);
    actions->addWidget(clearLaserPairsButton_);
    actions->addStretch(1);
    root->addLayout(actions);

    laserTable_ = new QTableWidget(0, 16, page);
    laserTable_->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("样本") << QStringLiteral("laser-off") << QStringLiteral("laser-on")
        << QStringLiteral("深度档") << QStringLiteral("中位Z mm")
        << QStringLiteral("板中心X/Y mm") << QStringLiteral("法向偏角X/Y deg")
        << QStringLiteral("条纹中位U/V px")
        << QStringLiteral("共同角点") << QStringLiteral("位移均值px") << QStringLiteral("位移P95px")
        << QStringLiteral("板平移mm") << QStringLiteral("板旋转deg") << QStringLiteral("条纹点")
        << QStringLiteral("3D点") << QStringLiteral("状态"));
    laserTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    laserTable_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    laserTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    laserTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    laserTable_->horizontalHeader()->setStretchLastSection(true);
    root->addWidget(laserTable_, 1);

    QGroupBox* solveGroup = new QGroupBox(QStringLiteral("激光平面 RANSAC"), page);
    QGridLayout* solveLayout = new QGridLayout(solveGroup);
    ransacThresholdSpin_ = new QDoubleSpinBox(solveGroup);
    ransacThresholdSpin_->setRange(0.001, 2.0);
    ransacThresholdSpin_->setDecimals(4);
    ransacThresholdSpin_->setValue(planeOptions_.ransacThresholdMm);
    ransacThresholdSpin_->setSuffix(QStringLiteral(" mm"));
    solveLaserButton_ = new QPushButton(QStringLiteral("求解激光平面"), solveGroup);
    saveLaserCandidateButton_ = new QPushButton(QStringLiteral("保存 session 候选 YAML"), solveGroup);
    saveLaserApprovedButton_ = new QPushButton(
        QStringLiteral("保存通过结果到 %1")
            .arg(profile_.laserPlaneConfigRelativePath),
        solveGroup);
    laserGateLabel_ = new QLabel(solveGroup);
    laserGateLabel_->setWordWrap(true);
    laserResultLabel_ = new QLabel(QStringLiteral("尚未求解"), solveGroup);
    laserResultLabel_->setWordWrap(true);
    solveLayout->addWidget(new QLabel(QStringLiteral("RANSAC 阈值"), solveGroup), 0, 0);
    solveLayout->addWidget(ransacThresholdSpin_, 0, 1);
    solveLayout->addWidget(solveLaserButton_, 0, 2);
    solveLayout->addWidget(saveLaserCandidateButton_, 0, 3);
    solveLayout->addWidget(saveLaserApprovedButton_, 0, 4);
    solveLayout->addWidget(laserGateLabel_, 1, 0, 1, 5);
    solveLayout->addWidget(laserResultLabel_, 2, 0, 1, 5);
    root->addWidget(solveGroup);
    return page;
}

QWidget* HikCalibrationWindow::buildProfilePage() {
    QWidget* page = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(page);

    QLabel* explanation = new QLabel(
        QStringLiteral("机器人保持静止：同一姿态、同一曝光/增益依次采 laser-off 和 laser-on。"
                       "程序使用正式内参和激光平面重建 %1 下的三维轮廓，"
                       "并自动保存 PLY/CSV。单条轮廓只能检验直线度；检测到 ChArUco 板时才报告真正的平板残差。"),
        page);
    explanation->setText(explanation->text().arg(profile_.cameraFrame));
    explanation->setWordWrap(true);
    explanation->setStyleSheet(QStringLiteral("color:#333;"));
    root->addWidget(explanation);

    QGroupBox* calibrationGroup = new QGroupBox(QStringLiteral("正式标定配置"), page);
    QHBoxLayout* calibrationLayout = new QHBoxLayout(calibrationGroup);
    reloadProfileCalibrationButton_ = new QPushButton(
        QStringLiteral("重新加载 config 正式标定"), calibrationGroup);
    profileCalibrationStatusLabel_ = new QLabel(QStringLiteral("尚未加载"), calibrationGroup);
    profileCalibrationStatusLabel_->setWordWrap(true);
    calibrationLayout->addWidget(reloadProfileCalibrationButton_);
    calibrationLayout->addWidget(profileCalibrationStatusLabel_, 1);
    root->addWidget(calibrationGroup);

    QGroupBox* captureGroup = new QGroupBox(QStringLiteral("静止成对采集"), page);
    QGridLayout* captureLayout = new QGridLayout(captureGroup);
    captureProfileOffButton_ = new QPushButton(QStringLiteral("1. 关闭激光，采 off"), captureGroup);
    captureProfileOnButton_ = new QPushButton(QStringLiteral("2. 保持不动，打开激光采 on"), captureGroup);
    cancelProfilePairButton_ = new QPushButton(QStringLiteral("取消当前配对"), captureGroup);
    importProfilePairButton_ = new QPushButton(QStringLiteral("离线导入 off/on"), captureGroup);
    profilePairStateLabel_ = new QLabel(captureGroup);
    profilePairStateLabel_->setWordWrap(true);
    captureLayout->addWidget(captureProfileOffButton_, 0, 0);
    captureLayout->addWidget(captureProfileOnButton_, 0, 1);
    captureLayout->addWidget(cancelProfilePairButton_, 0, 2);
    captureLayout->addWidget(importProfilePairButton_, 0, 3);
    captureLayout->addWidget(profilePairStateLabel_, 1, 0, 1, 4);
    root->addWidget(captureGroup);

    profileResultLabel_ = new QLabel(QStringLiteral("尚未重建。建议分别在近、中、远距离各采至少 3 组。"), page);
    profileResultLabel_->setWordWrap(true);
    root->addWidget(profileResultLabel_);

    profileTable_ = new QTableWidget(0, 11, page);
    profileTable_->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("样本") << QStringLiteral("off") << QStringLiteral("on")
        << QStringLiteral("3D点") << QStringLiteral("Z最小mm") << QStringLiteral("Z最大mm")
        << QStringLiteral("置信度") << QStringLiteral("直线RMSmm")
        << QStringLiteral("平板点") << QStringLiteral("平板RMSmm")
        << QStringLiteral("结果/PLY"));
    profileTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    profileTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    profileTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    profileTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    profileTable_->horizontalHeader()->setStretchLastSection(true);
    root->addWidget(profileTable_, 1);
    return page;
}

QWidget* HikCalibrationWindow::buildHandEyePage() {
    QWidget* page = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(page);

    QLabel* instructions = new QLabel(
        QStringLiteral(
            "Eye-in-hand：将 ChArUco 板牢固固定在机器人基座/工作台，关闭 %1 nm 线激光。"
            "每个样本自动执行 法兰前读→相机采图→法兰后读；前后位姿变化超过 0.10 mm 或 0.05° 即拒绝。"
            "请手动移动机器人到 15–25 个不同位置和姿态，采集时机器人必须完全停止。")
            .arg(profile_.wavelengthNm),
        page);
    instructions->setWordWrap(true);
    instructions->setStyleSheet(QStringLiteral("color:#333;"));
    root->addWidget(instructions);

    QLabel* warning = new QLabel(
        QStringLiteral(
            "安全/连接：本工具只读取实际法兰位姿，不发送运动指令。"
            "两个设备页共用本进程唯一的 Fairino SDK 会话，切页无需重连；"
            "连接前请停止 ros2_fr5 的 fairino_state_publisher 或其他 FR5 SDK 程序。"
            "另外，静态平板重建 RMS 仍应单独调到 <0.3 mm，手眼标定不会修复激光三角测量误差。"),
        page);
    warning->setWordWrap(true);
    warning->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    root->addWidget(warning);

    QGroupBox* robotGroup = new QGroupBox(
        QStringLiteral("共享 FR5｜只读法兰位姿"), page);
    QGridLayout* robotLayout = new QGridLayout(robotGroup);
    robotIpEdit_ = new QLineEdit(QStringLiteral("192.168.1.200"), robotGroup);
    connectRobotButton_ = new QPushButton(
        QStringLiteral("连接共享 FR5（只读）"), robotGroup);
    disconnectRobotButton_ = new QPushButton(
        QStringLiteral("断开共享 FR5"), robotGroup);
    readRobotPoseButton_ = new QPushButton(QStringLiteral("读取当前法兰"), robotGroup);
    robotStatusLabel_ = new QLabel(QStringLiteral("未连接"), robotGroup);
    robotPoseLabel_ = new QLabel(QStringLiteral("XYZ/RPY: -"), robotGroup);
    robotPoseLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    robotLayout->addWidget(new QLabel(QStringLiteral("机器人 IP"), robotGroup), 0, 0);
    robotLayout->addWidget(robotIpEdit_, 0, 1);
    robotLayout->addWidget(connectRobotButton_, 0, 2);
    robotLayout->addWidget(disconnectRobotButton_, 0, 3);
    robotLayout->addWidget(readRobotPoseButton_, 0, 4);
    robotLayout->addWidget(robotStatusLabel_, 0, 5);
    robotLayout->addWidget(robotPoseLabel_, 1, 0, 1, 6);
    robotLayout->setColumnStretch(1, 1);
    root->addWidget(robotGroup);

    handEyeIntrinsicsStatusLabel_ = new QLabel(QStringLiteral("正式内参尚未加载"), page);
    handEyeIntrinsicsStatusLabel_->setWordWrap(true);
    root->addWidget(handEyeIntrinsicsStatusLabel_);

    QHBoxLayout* actions = new QHBoxLayout;
    captureHandEyeButton_ = new QPushButton(QStringLiteral("采集一个手眼样本（激光关闭）"), page);
    deleteHandEyeButton_ = new QPushButton(QStringLiteral("删除选中"), page);
    clearHandEyeButton_ = new QPushButton(QStringLiteral("清空"), page);
    solveHandEyeButton_ = new QPushButton(QStringLiteral("鲁棒求解 T_flange_camera"), page);
    actions->addWidget(captureHandEyeButton_);
    actions->addWidget(deleteHandEyeButton_);
    actions->addWidget(clearHandEyeButton_);
    actions->addStretch(1);
    actions->addWidget(solveHandEyeButton_);
    root->addLayout(actions);

    handEyeCaptureStateLabel_ = new QLabel(QStringLiteral("等待采集。"), page);
    handEyeCaptureStateLabel_->setWordWrap(true);
    root->addWidget(handEyeCaptureStateLabel_);
    handEyeGateLabel_ = new QLabel(page);
    handEyeGateLabel_->setWordWrap(true);
    root->addWidget(handEyeGateLabel_);

    handEyeTable_ = new QTableWidget(0, 14, page);
    handEyeTable_->setHorizontalHeaderLabels(QStringList()
        << QStringLiteral("样本") << QStringLiteral("图像")
        << QStringLiteral("X") << QStringLiteral("Y") << QStringLiteral("Z")
        << QStringLiteral("Rx") << QStringLiteral("Ry") << QStringLiteral("Rz")
        << QStringLiteral("角点") << QStringLiteral("板RMSpx")
        << QStringLiteral("静止Δmm") << QStringLiteral("静止Δdeg")
        << QStringLiteral("板一致Δmm") << QStringLiteral("状态"));
    handEyeTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    handEyeTable_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    handEyeTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    handEyeTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    handEyeTable_->horizontalHeader()->setStretchLastSection(true);
    root->addWidget(handEyeTable_, 1);

    handEyeResultLabel_ = new QLabel(QStringLiteral("尚未求解。"), page);
    handEyeResultLabel_->setWordWrap(true);
    handEyeResultLabel_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    root->addWidget(handEyeResultLabel_);
    QHBoxLayout* saveActions = new QHBoxLayout;
    saveHandEyeCandidateButton_ = new QPushButton(QStringLiteral("保存 session 候选 YAML"), page);
    saveHandEyeApprovedButton_ = new QPushButton(
        QStringLiteral("质量通过后写入 %1")
            .arg(profile_.handEyeConfigRelativePath),
        page);
    saveActions->addStretch(1);
    saveActions->addWidget(saveHandEyeCandidateButton_);
    saveActions->addWidget(saveHandEyeApprovedButton_);
    root->addLayout(saveActions);
    return page;
}

QWidget* HikCalibrationWindow::buildPreviewPanel() {
    QWidget* panel = new QWidget(this);
    QVBoxLayout* root = new QVBoxLayout(panel);
    imageInfoLabel_ = new QLabel(QStringLiteral("尚未采集图像"), panel);
    imageInfoLabel_->setWordWrap(true);
    imageView_ = new ImageView(panel);
    imageView_->setEmptyText(QStringLiteral("暂无海康图像"));
    imageView_->setKeepViewOnImageUpdate(true);
    logView_ = new QPlainTextEdit(panel);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(3000);
    logView_->setMinimumHeight(180);
    root->addWidget(imageInfoLabel_);
    root->addWidget(imageView_, 1);
    root->addWidget(new QLabel(QStringLiteral("日志"), panel));
    root->addWidget(logView_);
    return panel;
}

void HikCalibrationWindow::setupCameraWorker() {
    cameraWorker_ = new HikCameraWorker;
    cameraWorker_->moveToThread(&cameraThread_);
    connect(&cameraThread_, &QThread::finished,
            cameraWorker_, &QObject::deleteLater);
    connect(this, &HikCalibrationWindow::requestConnectCamera,
            cameraWorker_, &HikCameraWorker::connectCamera, Qt::QueuedConnection);
    connect(this, &HikCalibrationWindow::requestDisconnectCamera,
            cameraWorker_, &HikCameraWorker::disconnectCamera, Qt::QueuedConnection);
    connect(this, &HikCalibrationWindow::requestCaptureSingle,
            cameraWorker_, &HikCameraWorker::captureSingle, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::connectionChanged,
            this, &HikCalibrationWindow::onCameraConnectionChanged, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::identityChanged,
            this, &HikCalibrationWindow::onCameraIdentityChanged, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::busyChanged,
            this, &HikCalibrationWindow::onCameraBusyChanged, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::frameReady,
            this, &HikCalibrationWindow::onCameraFrameReady, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::log,
            this, &HikCalibrationWindow::onCameraLog, Qt::QueuedConnection);
    connect(cameraWorker_, &HikCameraWorker::error,
            this, &HikCalibrationWindow::onCameraError, Qt::QueuedConnection);
    cameraThread_.start();
}

void HikCalibrationWindow::setupRobotSession() {
    robotClientId_ = robotSession_->registerClient(profile_.id);
    if (robotClientId_ <= 0 || !robotSession_->isRunning()) {
        throw std::runtime_error(
            "unable to register with the shared FairinoRobotSession");
    }
    connect(this, &HikCalibrationWindow::requestConnectRobot,
            robotSession_, &FairinoRobotSession::connectRobot);
    connect(this, &HikCalibrationWindow::requestDisconnectRobot,
            robotSession_, &FairinoRobotSession::disconnectRobot);
    connect(this, &HikCalibrationWindow::requestReadFlangePose,
            robotSession_, &FairinoRobotSession::readFlangePose);
    connect(robotSession_, &FairinoRobotSession::connectionChanged,
            this, &HikCalibrationWindow::onRobotConnectionChanged, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::busyChanged,
            this, &HikCalibrationWindow::onRobotBusyChanged, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::flangePoseReady,
            this, &HikCalibrationWindow::onRobotFlangePoseReady, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::log,
            this, &HikCalibrationWindow::onRobotLog, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::error,
            this, &HikCalibrationWindow::onRobotError, Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::clientError,
            this, &HikCalibrationWindow::onRobotClientError,
            Qt::QueuedConnection);
    connect(robotSession_, &FairinoRobotSession::exclusiveOwnerChanged,
            this, [this](int, const QString&) {
                updateAllUiStates();
            }, Qt::QueuedConnection);
    robotConnected_ = robotSession_->isConnected();
    robotBusy_ = robotSession_->isBusy();
}

void HikCalibrationWindow::shutdownCameraWorker() {
    if (shuttingDown_) {
        return;
    }
    shuttingDown_ = true;
    updateAllUiStates();

    if (cameraWorker_ && cameraThread_.isRunning()) {
        QMetaObject::invokeMethod(cameraWorker_, "disconnectCamera",
                                  Qt::BlockingQueuedConnection);
        cameraThread_.quit();
        if (!cameraThread_.wait(5000)) {
            cameraThread_.wait();
        }
    }
    cameraWorker_ = nullptr;
    cameraConnected_ = false;
    cameraBusy_ = false;
    currentCameraModel_.clear();
    currentCameraSerial_.clear();
    currentCameraIp_.clear();
    resetPendingCapture();
}

void HikCalibrationWindow::shutdownRobotSession() {
    if (robotSession_ && robotClientId_ > 0) {
        robotSession_->releaseExclusive(robotClientId_);
    }
    robotConnected_ = false;
    robotBusy_ = false;
    resetPendingHandEye();
}

void HikCalibrationWindow::closeEvent(QCloseEvent* event) {
    shutdownRobotSession();
    shutdownCameraWorker();
    event->accept();
}

void HikCalibrationWindow::setProfileTabActive(bool active) {
    if (profileTabActive_ == active) return;
    profileTabActive_ = active;
    if (!active &&
        handEyeCaptureState_ != HandEyeCaptureState::Idle) {
        appendLog(QStringLiteral(
            "手眼采样事务仍由本页持有共享 FR5；事务结束前其他设备页不能发送机器人命令。"));
    }
    updateAllUiStates();
}

bool HikCalibrationWindow::createSessionDirectories(QString* error) {
    if (error) {
        error->clear();
    }
    const QString calibrationRoot = profile_.calibrationSessionRoot(sourceDir_);
    if (!QDir().mkpath(calibrationRoot)) {
        if (error) {
            *error = QStringLiteral("无法创建标定数据根目录: %1").arg(calibrationRoot);
        }
        return false;
    }
    sessionDir_ = nextSessionDirectory(calibrationRoot);
    if (sessionDir_.isEmpty() || !QDir().mkpath(sessionDir_)) {
        if (error) {
            *error = QStringLiteral("无法创建唯一标定会话目录: %1").arg(calibrationRoot);
        }
        return false;
    }
    intrinsicSessionDir_ = QDir(sessionDir_).absoluteFilePath(QStringLiteral("intrinsics"));
    laserSessionDir_ = QDir(sessionDir_).absoluteFilePath(QStringLiteral("laser"));
    profileSessionDir_ = QDir(sessionDir_).absoluteFilePath(QStringLiteral("profiles"));
    handEyeSessionDir_ = QDir(sessionDir_).absoluteFilePath(QStringLiteral("handeye"));
    if (!QDir().mkpath(intrinsicSessionDir_) || !QDir().mkpath(laserSessionDir_) ||
        !QDir().mkpath(profileSessionDir_) || !QDir().mkpath(handEyeSessionDir_)) {
        if (error) {
            *error = QStringLiteral("无法创建会话子目录: %1").arg(sessionDir_);
        }
        return false;
    }
    captureManifestPath_ = QDir(sessionDir_).absoluteFilePath(
        QStringLiteral("capture_manifest.csv"));
    QSaveFile manifest(captureManifestPath_);
    if (!manifest.open(QIODevice::WriteOnly | QIODevice::Text) ||
        manifest.write("utc_time,profile_id,wavelength_nm,ttl_physical_pin,camera_frame,"
                       "expected_camera_model,expected_camera_serial,"
                       "purpose,pair_id,image_path,width,height,frame_no,device_timestamp,"
                       "host_timestamp,exposure_us,gain_db,camera_model,camera_serial,"
                       "camera_ip,description\n") < 0 ||
        !manifest.commit()) {
        if (error) {
            *error = QStringLiteral("无法创建会话采集清单: %1").arg(captureManifestPath_);
        }
        return false;
    }
    profileManifestPath_ = QDir(profileSessionDir_).absoluteFilePath(
        QStringLiteral("profile_manifest.csv"));
    QSaveFile profileManifest(profileManifestPath_);
    if (!profileManifest.open(QIODevice::WriteOnly | QIODevice::Text) ||
        profileManifest.write(
            "profile_id,sample_id,laser_off_path,laser_on_path,ply_path,csv_path,point_count,"
            "z_min_mm,z_max_mm,line_rms_mm,board_validation_available,board_point_count,"
            "board_rms_mm,acquisition_parameters_verified,intrinsics_file,intrinsics_sha256,"
            "laser_plane_file,laser_plane_sha256,frame_id\n") < 0 ||
        !profileManifest.commit()) {
        if (error) {
            *error = QStringLiteral("无法创建静态轮廓清单: %1").arg(profileManifestPath_);
        }
        return false;
    }
    handEyeManifestPath_ = QDir(handEyeSessionDir_).absoluteFilePath(
        QStringLiteral("handeye_manifest.csv"));
    QSaveFile handEyeManifest(handEyeManifestPath_);
    if (!handEyeManifest.open(QIODevice::WriteOnly | QIODevice::Text) ||
        handEyeManifest.write(
            "profile_id,sample_id,image_path,before_host_timestamp_ms,camera_host_timestamp_raw,"
            "after_host_timestamp_ms,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg,corner_count,"
            "board_pose_rms_px,robot_translation_delta_mm,robot_rotation_delta_deg,"
            "intrinsics_file,intrinsics_sha256,camera_model,camera_serial\n") < 0 ||
        !handEyeManifest.commit()) {
        if (error) {
            *error = QStringLiteral("无法创建手眼标定清单: %1").arg(handEyeManifestPath_);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::appendCaptureManifest(
        CapturePurpose purpose,
        const QString& pairId,
        const QString& imagePath,
        const QImage& image,
        quint64 frameNo,
        quint64 deviceTimestamp,
        qint64 hostTimestamp,
        double actualExposure,
        double actualGain,
        const QString& description,
        QString* error) const {
    if (error) {
        error->clear();
    }
    QString purposeName;
    switch (purpose) {
    case CapturePurpose::Manual: purposeName = QStringLiteral("manual"); break;
    case CapturePurpose::Intrinsic: purposeName = QStringLiteral("intrinsic"); break;
    case CapturePurpose::LaserOff: purposeName = QStringLiteral("laser_off"); break;
    case CapturePurpose::LaserOn: purposeName = QStringLiteral("laser_on"); break;
    case CapturePurpose::ProfileOff: purposeName = QStringLiteral("profile_off"); break;
    case CapturePurpose::ProfileOn: purposeName = QStringLiteral("profile_on"); break;
    case CapturePurpose::HandEye: purposeName = QStringLiteral("handeye"); break;
    case CapturePurpose::None: purposeName = QStringLiteral("none"); break;
    }
    QFile manifest(captureManifestPath_);
    if (!manifest.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法追加会话采集清单: %1").arg(captureManifestPath_);
        }
        return false;
    }
    QByteArray line;
    line += csvField(QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    line += ','; line += csvField(profile_.id);
    line += ','; line += QByteArray::number(profile_.wavelengthNm);
    line += ','; line += QByteArray::number(profile_.ttlPhysicalPin);
    line += ','; line += csvField(profile_.cameraFrame);
    line += ','; line += csvField(expectedCameraModelFromUi());
    line += ','; line += csvField(expectedCameraSerialFromUi());
    line += ','; line += csvField(purposeName);
    line += ','; line += csvField(pairId);
    line += ','; line += csvField(imagePath);
    line += ','; line += QByteArray::number(image.width());
    line += ','; line += QByteArray::number(image.height());
    line += ','; line += QByteArray::number(frameNo);
    line += ','; line += QByteArray::number(deviceTimestamp);
    line += ','; line += QByteArray::number(hostTimestamp);
    line += ','; line += QByteArray::number(actualExposure, 'f', 6);
    line += ','; line += QByteArray::number(actualGain, 'f', 6);
    line += ','; line += csvField(currentCameraModel_);
    line += ','; line += csvField(currentCameraSerial_);
    line += ','; line += csvField(currentCameraIp_);
    line += ','; line += csvField(description);
    line += '\n';
    if (manifest.write(line) != line.size() || !manifest.flush()) {
        if (error) {
            *error = QStringLiteral("写入会话采集清单失败: %1").arg(captureManifestPath_);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::appendProfileManifest(const ProfileSample& sample,
                                                  QString* error) const {
    if (error) {
        error->clear();
    }
    QFile manifest(profileManifestPath_);
    if (!manifest.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法追加静态轮廓清单: %1").arg(profileManifestPath_);
        }
        return false;
    }
    const hik_calibration::StaticProfileResult& result = sample.result;
    QByteArray line;
    line += csvField(profile_.id);
    line += ','; line += csvField(sample.sampleId);
    line += ','; line += csvField(sample.laserOffPath);
    line += ','; line += csvField(sample.laserOnPath);
    line += ','; line += csvField(sample.plyPath);
    line += ','; line += csvField(sample.csvPath);
    line += ','; line += QByteArray::number(static_cast<qulonglong>(result.points.size()));
    line += ','; line += QByteArray::number(result.minimumDepthMm, 'f', 9);
    line += ','; line += QByteArray::number(result.maximumDepthMm, 'f', 9);
    line += ','; line += QByteArray::number(result.lineDistanceMm.rms, 'f', 9);
    line += ','; line += result.boardValidationAvailable ? "1" : "0";
    line += ','; line += QByteArray::number(result.boardValidationPointCount);
    line += ',';
    if (result.boardValidationAvailable) {
        line += QByteArray::number(result.boardPlaneDistanceMm.rms, 'f', 9);
    }
    line += ','; line += sample.acquisitionParametersVerified ? "1" : "0";
    line += ','; line += csvField(profileIntrinsicsPath_);
    line += ','; line += csvField(profileIntrinsicsSha256_);
    line += ','; line += csvField(profileLaserPlanePath_);
    line += ','; line += csvField(profileLaserPlaneSha256_);
    line += ','; line += csvField(QString::fromStdString(profileLaserMetadata_.cameraFrame));
    line += '\n';
    if (manifest.write(line) != line.size() || !manifest.flush()) {
        if (error) {
            *error = QStringLiteral("写入静态轮廓清单失败: %1").arg(profileManifestPath_);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::appendHandEyeManifest(
        const HandEyeSampleEntry& sample,
        const RobotPoseReading& before,
        const RobotPoseReading& after,
        QString* error) const {
    if (error) {
        error->clear();
    }
    QFile manifest(handEyeManifestPath_);
    if (!manifest.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法追加手眼标定清单: %1").arg(handEyeManifestPath_);
        }
        return false;
    }
    QByteArray line;
    line += csvField(profile_.id);
    line += ','; line += csvField(QString::fromStdString(sample.sample.sampleId));
    line += ','; line += csvField(sample.imagePath);
    line += ','; line += QByteArray::number(before.hostTimestampMs);
    line += ','; line += QByteArray::number(pendingHandEyeImage_.cameraHostTimestampRaw);
    line += ','; line += QByteArray::number(after.hostTimestampMs);
    for (int index = 0; index < 6; ++index) {
        line += ',';
        line += QByteArray::number(sample.flangeValues[index], 'f', 9);
    }
    line += ','; line += QByteArray::number(sample.cornerCount);
    line += ','; line += QByteArray::number(sample.sample.boardPoseRmsPx, 'f', 9);
    line += ','; line += QByteArray::number(sample.robotTranslationDeltaMm, 'f', 9);
    line += ','; line += QByteArray::number(sample.robotRotationDeltaDeg, 'f', 9);
    line += ','; line += csvField(handEyeIntrinsicsPath_);
    line += ','; line += csvField(handEyeIntrinsicsSha256_);
    line += ','; line += csvField(currentCameraModel_);
    line += ','; line += csvField(currentCameraSerial_);
    line += '\n';
    if (manifest.write(line) != line.size() || !manifest.flush()) {
        if (error) {
            *error = QStringLiteral("写入手眼标定清单失败: %1").arg(handEyeManifestPath_);
        }
        return false;
    }
    return true;
}

void HikCalibrationWindow::appendLog(const QString& message) {
    if (!logView_) {
        return;
    }
    logView_->appendPlainText(QStringLiteral("[%1] %2")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz")), message));
}

void HikCalibrationWindow::showError(const QString& title, const QString& message) {
    appendLog(QStringLiteral("%1: %2").arg(title, message));
    if (!shuttingDown_) {
        QMessageBox::warning(this, title, message);
    }
}

void HikCalibrationWindow::updateAllUiStates() {
    updateCameraUi();
    updateBoardUi();
    updateIntrinsicUi();
    updateLaserUi();
    updateProfileUi();
    updateHandEyeUi();
}

void HikCalibrationWindow::updateCameraUi() {
    const bool operationPending = pendingCapturePurpose_ != CapturePurpose::None;
    const bool available = !shuttingDown_ && !cameraBusy_ && !operationPending;
    const bool profileCameraReady = currentCameraMatchesProfile();
    connectButton_->setEnabled(available && !cameraConnected_);
    disconnectButton_->setEnabled(available && cameraConnected_);
    singleFrameButton_->setEnabled(available && profileCameraReady && sessionReady_);
    cameraIpEdit_->setEnabled(available && !cameraConnected_);
    expectedCameraModelEdit_->setEnabled(
        profile_.expectedCameraModel.isEmpty() && profileIdentityCanChange());
    expectedCameraSerialEdit_->setEnabled(
        profile_.expectedCameraSerial.isEmpty() && profileIdentityCanChange());
    // A pending laser-off is a transaction: the on frame must use exactly the
    // same acquisition controls.  The actual values are checked again when the
    // on frame returns from the camera worker.
    exposureSpin_->setEnabled(available && !pendingLaserOff_.valid &&
                              !pendingProfileOff_.valid);
    gainSpin_->setEnabled(available && !pendingLaserOff_.valid &&
                          !pendingProfileOff_.valid);
    timeoutSpin_->setEnabled(available);
}

void HikCalibrationWindow::updateBoardUi() {
    const bool enabled = boardCanChange() && !shuttingDown_ && !cameraBusy_ &&
                         pendingCapturePurpose_ == CapturePurpose::None;
    squaresXSpin_->setEnabled(enabled);
    squaresYSpin_->setEnabled(enabled);
    squareLengthSpin_->setEnabled(enabled);
    markerLengthSpin_->setEnabled(enabled);
    dictionaryCombo_->setEnabled(enabled);
    swapBoardButton_->setEnabled(enabled);
    boardLockLabel_->setText(enabled
        ? QStringLiteral("尚无样本，可切换为 OpenCV 7×5 或修改板参数。添加任一标定样本后参数锁定。")
        : QStringLiteral("板参数已锁定；清空全部内参、激光及手眼样本，并取消未完成配对后方可修改。"));
    boardLockLabel_->setStyleSheet(enabled ? QString() : QStringLiteral("color:#a06000;"));
}

void HikCalibrationWindow::updateIntrinsicUi() {
    const bool idle = !shuttingDown_ && !cameraBusy_ &&
                      pendingCapturePurpose_ == CapturePurpose::None;
    const bool profileCameraReady = currentCameraMatchesProfile();
    const int accepted = acceptedIntrinsicSampleCount();
    const int minimum = intrinsicOptions_.minViews;
    captureIntrinsicButton_->setEnabled(idle && profileCameraReady && sessionReady_);
    importIntrinsicButton_->setEnabled(idle && sessionReady_);
    deleteIntrinsicButton_->setEnabled(idle && intrinsicTable_->selectionModel() &&
        !intrinsicTable_->selectionModel()->selectedRows().isEmpty());
    clearIntrinsicButton_->setEnabled(idle && !intrinsicSamples_.empty());
    solveIntrinsicButton_->setEnabled(idle && accepted >= minimum);
    saveIntrinsicCandidateButton_->setEnabled(idle && intrinsicResultHasMatrices());
    saveIntrinsicApprovedButton_->setEnabled(idle && intrinsicResultPassesQuality());
    intrinsicGateLabel_->setText(QStringLiteral("Core 门槛：至少 %1 个通过检测质量的视图；当前通过 %2 / 总计 %3。%4")
        .arg(minimum).arg(accepted).arg(static_cast<int>(intrinsicSamples_.size()))
        .arg(accepted >= minimum ? QStringLiteral("可求解。") : QStringLiteral("尚不可求解。")));
    intrinsicGateLabel_->setStyleSheet(accepted >= minimum
        ? QStringLiteral("color:#087f23;") : QStringLiteral("color:#b00020;"));
    useCurrentIntrinsicsButton_->setEnabled(idle && intrinsicResultHasMatrices() && laserPairs_.empty() &&
                                            !pendingLaserOff_.valid);
}

void HikCalibrationWindow::updateLaserUi() {
    const bool idle = !shuttingDown_ && !cameraBusy_ &&
                      pendingCapturePurpose_ == CapturePurpose::None;
    const bool awaitingOff = laserCaptureState_ == LaserCaptureState::AwaitingOff;
    const bool boardCompatible = hasActiveLaserIntrinsics_ &&
        boardMatches(boardSpecFromUi(), activeLaserIntrinsics_.board);
    const bool profileCameraReady = currentCameraMatchesProfile();
    captureLaserOffButton_->setEnabled(idle && profileCameraReady && sessionReady_ &&
                                       boardCompatible && awaitingOff);
    captureLaserOnButton_->setEnabled(idle && profileCameraReady && sessionReady_ &&
                                      boardCompatible && !awaitingOff &&
                                      pendingLaserOff_.valid);
    cancelLaserPairButton_->setEnabled(idle && pendingLaserOff_.valid);
    loadLaserIntrinsicsButton_->setEnabled(idle && laserPairs_.empty() && !pendingLaserOff_.valid);
    importLaserPairsButton_->setEnabled(idle && sessionReady_ && boardCompatible &&
                                        awaitingOff);
    deleteLaserPairButton_->setEnabled(idle && laserTable_->selectionModel() &&
        !laserTable_->selectionModel()->selectedRows().isEmpty());
    clearLaserPairsButton_->setEnabled(idle && (!laserPairs_.empty() || pendingLaserOff_.valid));
    ransacThresholdSpin_->setEnabled(idle);

    const int accepted = acceptedLaserPairCount();
    const int minimum = planeOptions_.minPoseCount;
    solveLaserButton_->setEnabled(idle && accepted >= minimum && hasActiveLaserIntrinsics_);
    saveLaserCandidateButton_->setEnabled(idle && hasLaserPlaneResult_);
    saveLaserApprovedButton_->setEnabled(idle && laserResultPassesQuality());
    laserGateLabel_->setText(QStringLiteral("Core 门槛：至少 %1 个通过静止/条纹/3D质量检查的不同板姿态；当前通过 %2 / 总计 %3。")
        .arg(minimum).arg(accepted).arg(static_cast<int>(laserPairs_.size())));
    laserGateLabel_->setStyleSheet(accepted >= minimum
        ? QStringLiteral("color:#087f23;") : QStringLiteral("color:#b00020;"));

    if (awaitingOff) {
        laserPairStateLabel_->setText(QStringLiteral("等待 laser-off。确认激光关闭后采集；采完后才允许 laser-on。"));
        laserPairStateLabel_->setStyleSheet(QStringLiteral("color:#333;"));
    } else {
        laserPairStateLabel_->setText(QStringLiteral("laser-off 已保存。严禁移动标定板或相机；现在打开激光并采 laser-on。"));
        laserPairStateLabel_->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    }
}

void HikCalibrationWindow::updateProfileUi() {
    const bool idle = !shuttingDown_ && !cameraBusy_ &&
                      pendingCapturePurpose_ == CapturePurpose::None;
    const bool awaitingOff = profileCaptureState_ == LaserCaptureState::AwaitingOff;
    const bool profileCameraReady = currentCameraMatchesProfile();
    captureProfileOffButton_->setEnabled(idle && profileCameraReady && sessionReady_ &&
                                         hasProfileCalibration_ && awaitingOff);
    captureProfileOnButton_->setEnabled(idle && profileCameraReady && sessionReady_ &&
                                        hasProfileCalibration_ && !awaitingOff &&
                                        pendingProfileOff_.valid);
    cancelProfilePairButton_->setEnabled(idle && pendingProfileOff_.valid);
    importProfilePairButton_->setEnabled(idle && sessionReady_ &&
                                         hasProfileCalibration_ && awaitingOff);
    reloadProfileCalibrationButton_->setEnabled(idle && !pendingProfileOff_.valid);
    if (awaitingOff) {
        profilePairStateLabel_->setText(
            QStringLiteral("等待 laser-off。确认工件/平板、相机均静止并关闭激光。"));
        profilePairStateLabel_->setStyleSheet(QString());
    } else {
        profilePairStateLabel_->setText(
            QStringLiteral("laser-off 已锁定；严禁移动相机或平板、严禁修改曝光/增益，现在打开激光采 on。"));
        profilePairStateLabel_->setStyleSheet(QStringLiteral("color:#b00020;font-weight:bold;"));
    }
}

void HikCalibrationWindow::updateHandEyeUi() {
    const bool cameraIdle = !shuttingDown_ && !cameraBusy_ &&
                            pendingCapturePurpose_ == CapturePurpose::None;
    const bool stateIdle = cameraIdle && !robotBusy_ &&
                           pendingRobotRequestId_ < 0 &&
                           handEyeCaptureState_ == HandEyeCaptureState::Idle;
    const bool idle = profileTabActive_ && stateIdle;
    const bool robotAvailable =
        robotSession_ && robotSession_->commandAvailableTo(robotClientId_);
    connectRobotButton_->setEnabled(
        idle && robotAvailable && !robotConnected_ &&
        !robotSession_->connectionAttempted());
    disconnectRobotButton_->setEnabled(
        idle && robotAvailable && robotConnected_);
    robotIpEdit_->setEnabled(
        idle && !robotConnected_ && !robotSession_->connectionAttempted());
    readRobotPoseButton_->setEnabled(
        idle && robotAvailable && robotConnected_);

    const bool boardCompatible = hasHandEyeIntrinsics_ &&
        boardMatches(boardSpecFromUi(), handEyeIntrinsics_.board);
    const bool profileCameraReady = currentCameraMatchesProfile();
    captureHandEyeButton_->setEnabled(
        idle && robotAvailable && robotConnected_ && profileCameraReady &&
        sessionReady_ && boardCompatible);
    deleteHandEyeButton_->setEnabled(idle && handEyeTable_->selectionModel() &&
        !handEyeTable_->selectionModel()->selectedRows().isEmpty());
    clearHandEyeButton_->setEnabled(idle && !handEyeSamples_.empty());
    solveHandEyeButton_->setEnabled(idle &&
        static_cast<int>(handEyeSamples_.size()) >= handEyeOptions_.minSamples);
    saveHandEyeCandidateButton_->setEnabled(idle && hasHandEyeResult_);
    saveHandEyeApprovedButton_->setEnabled(idle && handEyeResultPassesQuality());

    handEyeGateLabel_->setText(QStringLiteral(
        "求解门槛：至少 %1 个有效样本，法兰平移跨度 ≥%2 mm、旋转跨度 ≥%3°、第二旋转轴离散度 ≥%4°；当前 %5 个。"
        "正式质量：固定板 base 坐标平移 RMS ≤%6 mm、旋转 RMS ≤%7°。")
        .arg(handEyeOptions_.minSamples)
        .arg(handEyeOptions_.minTranslationSpanMm, 0, 'f', 0)
        .arg(handEyeOptions_.minRotationSpanDeg, 0, 'f', 0)
        .arg(handEyeOptions_.minSecondaryRotationSpreadDeg, 0, 'f', 0)
        .arg(static_cast<int>(handEyeSamples_.size()))
        .arg(kApprovedHandEyeTranslationRmsMm, 0, 'f', 2)
        .arg(kApprovedHandEyeRotationRmsDeg, 0, 'f', 2));
    handEyeGateLabel_->setStyleSheet(
        static_cast<int>(handEyeSamples_.size()) >= handEyeOptions_.minSamples
            ? QStringLiteral("color:#087f23;") : QStringLiteral("color:#b00020;"));

    switch (handEyeCaptureState_) {
    case HandEyeCaptureState::Idle:
        handEyeCaptureStateLabel_->setText(QStringLiteral(
            "等待采集：固定标定板、关闭激光、手动调整 FR5 后等待完全停止。"));
        handEyeCaptureStateLabel_->setStyleSheet(QString());
        break;
    case HandEyeCaptureState::WaitingBeforePose:
        handEyeCaptureStateLabel_->setText(QStringLiteral("正在读取采图前实际法兰位姿…"));
        handEyeCaptureStateLabel_->setStyleSheet(QStringLiteral("color:#a06000;"));
        break;
    case HandEyeCaptureState::WaitingCameraFrame:
        handEyeCaptureStateLabel_->setText(QStringLiteral("法兰前读完成，正在采集 ChArUco 图像…"));
        handEyeCaptureStateLabel_->setStyleSheet(QStringLiteral("color:#a06000;"));
        break;
    case HandEyeCaptureState::WaitingAfterPose:
        handEyeCaptureStateLabel_->setText(QStringLiteral("图像检测通过，正在读取采图后法兰并检查静止性…"));
        handEyeCaptureStateLabel_->setStyleSheet(QStringLiteral("color:#a06000;"));
        break;
    }
}

void HikCalibrationWindow::resetPendingCapture() {
    pendingRequestId_ = -1;
    pendingCapturePurpose_ = CapturePurpose::None;
}

QString HikCalibrationWindow::expectedCameraModelFromUi() const {
    return expectedCameraModelEdit_
        ? expectedCameraModelEdit_->text().trimmed()
        : profile_.expectedCameraModel.trimmed();
}

QString HikCalibrationWindow::expectedCameraSerialFromUi() const {
    return expectedCameraSerialEdit_
        ? expectedCameraSerialEdit_->text().trimmed()
        : profile_.expectedCameraSerial.trimmed();
}

bool HikCalibrationWindow::currentCameraMatchesProfile(QString* error) const {
    if (error) {
        error->clear();
    }
    if (!cameraConnected_) {
        if (error) {
            *error = QStringLiteral("%1 尚未连接相机。").arg(profile_.id);
        }
        return false;
    }
    if (currentCameraSerial_.isEmpty()) {
        if (error) {
            *error = QStringLiteral("%1 尚未取得相机序列号。").arg(profile_.id);
        }
        return false;
    }
    const QString expectedModel = expectedCameraModelFromUi();
    const QString expectedSerial = expectedCameraSerialFromUi();
    if (!expectedModel.isEmpty() &&
        expectedModel.compare(currentCameraModel_, Qt::CaseInsensitive) != 0) {
        if (error) {
            *error = QStringLiteral("%1 预期型号=%2，当前型号=%3。")
                .arg(profile_.id, expectedModel,
                     currentCameraModel_.isEmpty()
                         ? QStringLiteral("未知") : currentCameraModel_);
        }
        return false;
    }
    if (!expectedSerial.isEmpty() &&
        expectedSerial.compare(currentCameraSerial_, Qt::CaseInsensitive) != 0) {
        if (error) {
            *error = QStringLiteral("%1 预期 SN=%2，当前 SN=%3。")
                .arg(profile_.id, expectedSerial, currentCameraSerial_);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::intrinsicsMetadataMatchesProfile(
        const hik_calibration::IntrinsicsYamlMetadata& metadata,
        QString* error) const {
    if (error) {
        error->clear();
    }
    const QString frame = QString::fromStdString(metadata.frameId).trimmed();
    if (frame != profile_.cameraFrame) {
        if (error) {
            *error = QStringLiteral("内参 frame=%1，不属于 %2（要求 %3）。")
                .arg(frame.isEmpty() ? QStringLiteral("未设置") : frame,
                     profile_.id, profile_.cameraFrame);
        }
        return false;
    }
    const QString serial =
        QString::fromStdString(metadata.cameraSerial).trimmed();
    QString expectedSerial = expectedCameraSerialFromUi();
    if (expectedSerial.isEmpty() && cameraConnected_) {
        expectedSerial = currentCameraSerial_;
    }
    if (expectedSerial.isEmpty()) {
        expectedSerial = intrinsicDatasetCameraSerial_;
    }
    if (!serial.isEmpty() && !expectedSerial.isEmpty() &&
        serial.compare(expectedSerial, Qt::CaseInsensitive) != 0) {
        if (error) {
            *error = QStringLiteral("内参 SN=%1，与 %2 绑定 SN=%3 不一致。")
                .arg(serial, profile_.id, expectedSerial);
        }
        return false;
    }
    const QString model = QString::fromStdString(metadata.cameraModel).trimmed();
    QString expectedModel = expectedCameraModelFromUi();
    if (expectedModel.isEmpty() && cameraConnected_) {
        expectedModel = currentCameraModel_;
    }
    if (!expectedModel.isEmpty() && !model.isEmpty() &&
        model.compare(expectedModel, Qt::CaseInsensitive) != 0) {
        if (error) {
            *error = QStringLiteral("内参型号=%1，与 %2 绑定型号=%3 不一致。")
                .arg(model, profile_.id, expectedModel);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::profileIdentityCanChange() const {
    return !shuttingDown_ && !cameraBusy_ &&
           pendingCapturePurpose_ == CapturePurpose::None &&
           intrinsicSamples_.empty() && laserPairs_.empty() &&
           profileSamples_.empty() && handEyeSamples_.empty() &&
           !pendingLaserOff_.valid && !pendingProfileOff_.valid &&
           handEyeCaptureState_ == HandEyeCaptureState::Idle;
}

void HikCalibrationWindow::connectCamera() {
    const QString ip = cameraIpEdit_->text().trimmed();
    if (!isValidIpv4(ip)) {
        showError(QStringLiteral("连接失败"),
                  QStringLiteral("请为 %1 输入有效的相机 IPv4 地址。")
                      .arg(profile_.id));
        return;
    }
    if (!cameraWorker_ || !cameraThread_.isRunning() || shuttingDown_) {
        showError(QStringLiteral("连接失败"), QStringLiteral("相机工作线程不可用。"));
        return;
    }
    cameraBusy_ = true;
    cameraStatusLabel_->setText(QStringLiteral("正在连接 %1 ...").arg(ip));
    cameraStatusLabel_->setStyleSheet(QStringLiteral("color:#a06000;font-weight:bold;"));
    appendLog(QStringLiteral("请求连接相机 %1").arg(ip));
    updateAllUiStates();
    emit requestConnectCamera(ip);
}

void HikCalibrationWindow::disconnectCamera() {
    if (!cameraWorker_ || !cameraThread_.isRunning() || shuttingDown_) {
        return;
    }
    cameraBusy_ = true;
    appendLog(QStringLiteral("请求断开相机"));
    updateAllUiStates();
    emit requestDisconnectCamera();
}

void HikCalibrationWindow::captureManualFrame() {
    beginCapture(CapturePurpose::Manual);
}

void HikCalibrationWindow::captureIntrinsicSample() {
    hik_calibration::BoardSpec board;
    QString error;
    if (!checkedBoardSpec(&board, &error)) {
        showError(QStringLiteral("板参数错误"), error);
        return;
    }
    beginCapture(CapturePurpose::Intrinsic);
}

void HikCalibrationWindow::captureLaserOff() {
    if (!hasActiveLaserIntrinsics_) {
        showError(QStringLiteral("无法采集"), QStringLiteral("请先为激光面标定加载或沿用内参。"));
        return;
    }
    const hik_calibration::BoardSpec board = boardSpecFromUi();
    if (!boardMatches(board, activeLaserIntrinsics_.board)) {
        showError(QStringLiteral("板参数不匹配"),
                  QStringLiteral("当前界面的 ChArUco 板参数与激光标定所用内参文件不一致。"));
        return;
    }
    if (laserCaptureState_ != LaserCaptureState::AwaitingOff || pendingLaserOff_.valid) {
        showError(QStringLiteral("采集顺序错误"),
                  QStringLiteral("已有待配对的 laser-off，请采 laser-on 或先取消当前配对。"));
        return;
    }
    pendingLaserCapturePairId_ = QStringLiteral("laser_%1").arg(captureTimestamp());
    appendLog(QStringLiteral("正在采集 laser-off：请确认线激光已关闭。配对 ID=%1")
              .arg(pendingLaserCapturePairId_));
    if (beginCapture(CapturePurpose::LaserOff) < 0) {
        pendingLaserCapturePairId_.clear();
    }
}

void HikCalibrationWindow::captureLaserOn() {
    if (laserCaptureState_ != LaserCaptureState::AwaitingOn || !pendingLaserOff_.valid) {
        showError(QStringLiteral("采集顺序错误"), QStringLiteral("请先采集合格的 laser-off。"));
        return;
    }
    appendLog(QStringLiteral("正在采集 laser-on：将使用已保存的 laser-off 做静止配对检查。配对 ID=%1")
              .arg(pendingLaserOff_.sampleId));
    beginCapture(CapturePurpose::LaserOn);
}

void HikCalibrationWindow::cancelPendingLaserPair() {
    if (!pendingLaserOff_.valid) {
        return;
    }
    appendLog(QStringLiteral("已取消待配对 laser-off（原图仍保留在 session）: %1")
              .arg(pendingLaserOff_.imagePath));
    resetPendingLaserOff();
    updateAllUiStates();
}

void HikCalibrationWindow::reloadProfileCalibration() {
    if (pendingProfileOff_.valid) {
        showError(QStringLiteral("无法重新加载"),
                  QStringLiteral("请先完成或取消当前静态轮廓 off/on 配对。"));
        return;
    }
    QString error;
    if (!loadFormalProfileCalibration(&error)) {
        showError(QStringLiteral("正式标定加载失败"), error);
    } else {
        appendLog(QStringLiteral("已重新加载静态轮廓正式标定配置。"));
    }
    updateAllUiStates();
}

void HikCalibrationWindow::captureProfileOff() {
    if (!hasProfileCalibration_) {
        showError(QStringLiteral("无法采集"), QStringLiteral("请先加载正式内参与激光平面。"));
        return;
    }
    if (profileCaptureState_ != LaserCaptureState::AwaitingOff || pendingProfileOff_.valid) {
        showError(QStringLiteral("采集顺序错误"),
                  QStringLiteral("已有待配对的 profile-off，请采 on 或先取消。"));
        return;
    }
    QString identityError;
    if (!profileCameraIdentityMatches(&identityError)) {
        showError(QStringLiteral("相机身份不匹配"), identityError);
        return;
    }
    pendingProfileCapturePairId_ = QStringLiteral("profile_%1").arg(captureTimestamp());
    appendLog(QStringLiteral("正在采静态轮廓 laser-off：配对 ID=%1")
              .arg(pendingProfileCapturePairId_));
    if (beginCapture(CapturePurpose::ProfileOff) < 0) {
        pendingProfileCapturePairId_.clear();
    }
}

void HikCalibrationWindow::captureProfileOn() {
    if (profileCaptureState_ != LaserCaptureState::AwaitingOn || !pendingProfileOff_.valid) {
        showError(QStringLiteral("采集顺序错误"), QStringLiteral("请先采集合格的 profile-off。"));
        return;
    }
    appendLog(QStringLiteral("正在采静态轮廓 laser-on：配对 ID=%1")
              .arg(pendingProfileOff_.sampleId));
    beginCapture(CapturePurpose::ProfileOn);
}

void HikCalibrationWindow::cancelPendingProfilePair() {
    if (!pendingProfileOff_.valid) {
        return;
    }
    appendLog(QStringLiteral("已取消静态轮廓待配对 off（原图保留）: %1")
              .arg(pendingProfileOff_.imagePath));
    resetPendingProfileOff();
    updateAllUiStates();
}

void HikCalibrationWindow::importProfilePair() {
    if (!hasProfileCalibration_) {
        showError(QStringLiteral("无法导入"), QStringLiteral("请先加载正式内参与激光平面。"));
        return;
    }
    const QString offSource = QFileDialog::getOpenFileName(
        this, QStringLiteral("选择静态轮廓 laser-off 图像"), sourceDir_,
        QStringLiteral("图像 (*.png *.bmp *.jpg *.jpeg *.tif *.tiff);;所有文件 (*)"));
    if (offSource.isEmpty()) {
        return;
    }
    const QString onSource = QFileDialog::getOpenFileName(
        this, QStringLiteral("选择同一姿态、同曝光的 laser-on 图像"),
        QFileInfo(offSource).absolutePath(),
        QStringLiteral("图像 (*.png *.bmp *.jpg *.jpeg *.tif *.tiff);;所有文件 (*)"));
    if (onSource.isEmpty()) {
        return;
    }
    const QString sampleId = QStringLiteral("profile_import_%1").arg(captureTimestamp());
    cv::Mat offImage;
    cv::Mat onImage;
    QString offPath;
    QString onPath;
    QString error;
    if (!copyImportedImageToSession(offSource, profileSessionDir_,
                                    sampleId + QStringLiteral("_off"),
                                    &offImage, &offPath, &error) ||
        !copyImportedImageToSession(onSource, profileSessionDir_,
                                    sampleId + QStringLiteral("_on"),
                                    &onImage, &onPath, &error)) {
        showError(QStringLiteral("静态轮廓导入失败"), error);
        return;
    }
    addOfflineProfilePair(offImage, offPath, onImage, onPath, sampleId);
    updateAllUiStates();
}

void HikCalibrationWindow::connectRobot() {
    if (!profileTabActive_) {
        showError(QStringLiteral("FR5 连接失败"),
                  QStringLiteral("请先切换到设备组 %1。")
                      .arg(profile_.displayName));
        return;
    }
    const QString ip = robotIpEdit_->text().trimmed();
    if (!isValidIpv4(ip)) {
        showError(QStringLiteral("FR5 连接失败"),
                  QStringLiteral("请输入有效的机器人 IPv4 地址，例如 192.168.1.200。"));
        return;
    }
    if (!robotSession_ || !robotSession_->isRunning() ||
        robotClientId_ <= 0 || shuttingDown_) {
        showError(QStringLiteral("FR5 连接失败"),
                  QStringLiteral("共享机器人会话不可用。"));
        return;
    }
    robotBusy_ = true;
    robotStatusLabel_->setText(
        QStringLiteral("正在建立共享连接 %1 …").arg(ip));
    robotStatusLabel_->setStyleSheet(QStringLiteral("color:#a06000;font-weight:bold;"));
    updateAllUiStates();
    emit requestConnectRobot(robotClientId_, ip);
}

void HikCalibrationWindow::disconnectRobot() {
    if (handEyeCaptureState_ != HandEyeCaptureState::Idle) {
        showError(QStringLiteral("无法断开"), QStringLiteral("手眼样本采集事务尚未结束。"));
        return;
    }
    if (robotSession_ && robotSession_->isRunning()) {
        robotBusy_ = true;
        updateAllUiStates();
        emit requestDisconnectRobot(robotClientId_);
    }
}

void HikCalibrationWindow::readRobotPose() {
    if (!robotConnected_ || robotBusy_ || pendingRobotRequestId_ >= 0) {
        return;
    }
    manualRobotReadPending_ = true;
    pendingRobotRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
    emit requestReadFlangePose(pendingRobotRequestId_);
    updateAllUiStates();
}

void HikCalibrationWindow::captureHandEyeSample() {
    if (!hasHandEyeIntrinsics_) {
        showError(QStringLiteral("无法采集手眼样本"),
                  QStringLiteral("正式内参 %1 未加载。")
                      .arg(profile_.intrinsicsConfigRelativePath));
        return;
    }
    QString identityError;
    if (!handEyeCameraIdentityMatches(&identityError)) {
        showError(QStringLiteral("相机身份不匹配"), identityError);
        return;
    }
    if (!boardMatches(boardSpecFromUi(), handEyeIntrinsics_.board)) {
        showError(QStringLiteral("标定板不匹配"),
                  QStringLiteral("当前 ChArUco 参数与正式内参记录不一致。"));
        return;
    }
    if (!robotConnected_ || !cameraConnected_ || robotBusy_ || cameraBusy_ ||
        handEyeCaptureState_ != HandEyeCaptureState::Idle) {
        showError(QStringLiteral("无法采集手眼样本"),
                  QStringLiteral("请先连接同一台相机和 FR5，并等待当前操作结束。"));
        return;
    }
    QString hashError;
    if (fileSha256Hex(handEyeIntrinsicsPath_, &hashError) != handEyeIntrinsicsSha256_) {
        showError(QStringLiteral("正式内参已变化"),
                  QStringLiteral("采集前发现 %1 已变化，请重新加载。")
                      .arg(profile_.intrinsicsConfigRelativePath));
        return;
    }
    QString leaseError;
    if (!robotSession_->acquireExclusive(
            robotClientId_,
            QStringLiteral("%1 手眼样本事务").arg(profile_.id),
            &leaseError)) {
        showError(QStringLiteral("无法采集手眼样本"), leaseError);
        return;
    }
    hasHandEyeResult_ = false;
    handEyeResult_ = hik_calibration::HandEyeCalibrationResult();
    lastHandEyeCandidatePath_.clear();
    pendingHandEyeSampleId_ = QStringLiteral("handeye_%1").arg(captureTimestamp());
    pendingHandEyeBeforePose_ = RobotPoseReading();
    pendingHandEyeImage_ = PendingHandEyeImage();
    handEyeCaptureState_ = HandEyeCaptureState::WaitingBeforePose;
    pendingRobotRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
    manualRobotReadPending_ = false;
    appendLog(QStringLiteral("开始手眼样本 %1：读取采图前实际法兰。")
              .arg(pendingHandEyeSampleId_));
    emit requestReadFlangePose(pendingRobotRequestId_);
    updateAllUiStates();
}

void HikCalibrationWindow::onRobotConnectionChanged(bool connected, QString description) {
    robotConnected_ = connected;
    robotBusy_ = false;
    if (!connected && robotSession_->connectionAttempted()) {
        appendLog(QStringLiteral(
            "共享 FAIRINO RPC 会话已结束；受 SDK 3.9.4 生命周期限制，"
            "本进程不再创建第二个 FRRobot，重新连接需重启标定程序。"));
    }
    if (!connected && handEyeCaptureState_ != HandEyeCaptureState::Idle) {
        appendLog(QStringLiteral("FR5 断开，未完成的手眼样本已取消。"));
        resetPendingHandEye();
    }
    robotStatusLabel_->setText(description);
    robotStatusLabel_->setStyleSheet(connected
        ? QStringLiteral("color:#087f23;font-weight:bold;")
        : QStringLiteral("color:#b00020;font-weight:bold;"));
    appendLog(QStringLiteral("机器人: %1").arg(description));
    updateAllUiStates();
}

void HikCalibrationWindow::onRobotBusyChanged(bool busy) {
    robotBusy_ = busy;
    updateAllUiStates();
}

void HikCalibrationWindow::onRobotFlangePoseReady(int requestId,
                                                   double xMm,
                                                   double yMm,
                                                   double zMm,
                                                   double rxDeg,
                                                   double ryDeg,
                                                   double rzDeg,
                                                   qint64 hostTimestampMs) {
    if (!robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    if (requestId != pendingRobotRequestId_) {
        appendLog(QStringLiteral("忽略过期 FR5 位姿：request=%1，当前=%2")
                  .arg(requestId).arg(pendingRobotRequestId_));
        return;
    }
    pendingRobotRequestId_ = -1;
    RobotPoseReading reading;
    reading.valid = true;
    reading.values[0] = xMm; reading.values[1] = yMm; reading.values[2] = zMm;
    reading.values[3] = rxDeg; reading.values[4] = ryDeg; reading.values[5] = rzDeg;
    reading.baseFromFlange = hik_calibration::fairinoBaseFromFlange(
        xMm, yMm, zMm, rxDeg, ryDeg, rzDeg);
    reading.hostTimestampMs = hostTimestampMs;
    currentRobotPose_ = reading;
    robotPoseLabel_->setText(QStringLiteral(
        "实际 T_base_flange：XYZ=[%1, %2, %3] mm，RPY=[%4, %5, %6] deg，host_ms=%7")
        .arg(xMm, 0, 'f', 3).arg(yMm, 0, 'f', 3).arg(zMm, 0, 'f', 3)
        .arg(rxDeg, 0, 'f', 3).arg(ryDeg, 0, 'f', 3).arg(rzDeg, 0, 'f', 3)
        .arg(hostTimestampMs));

    if (manualRobotReadPending_) {
        manualRobotReadPending_ = false;
        appendLog(QStringLiteral("FR5 当前法兰读取成功。"));
    } else if (handEyeCaptureState_ == HandEyeCaptureState::WaitingBeforePose) {
        pendingHandEyeBeforePose_ = reading;
        handEyeCaptureState_ = HandEyeCaptureState::WaitingCameraFrame;
        appendLog(QStringLiteral("手眼样本 %1：前法兰已记录，开始相机采图。")
                  .arg(pendingHandEyeSampleId_));
        if (beginCapture(CapturePurpose::HandEye) < 0) {
            resetPendingHandEye();
        }
    } else if (handEyeCaptureState_ == HandEyeCaptureState::WaitingAfterPose) {
        finishHandEyeSample(reading);
    }
    updateAllUiStates();
}

void HikCalibrationWindow::onRobotLog(QString message) {
    appendLog(QStringLiteral("机器人: %1").arg(message));
}

void HikCalibrationWindow::onRobotError(int requestId, QString message) {
    if (requestId >= 0 &&
        !robotSession_->requestBelongsToClient(
            requestId, robotClientId_)) {
        return;
    }
    if (requestId < 0 && !profileTabActive_) {
        appendLog(QStringLiteral("共享 FR5 错误：%1").arg(message));
        return;
    }
    if (requestId == pendingRobotRequestId_) {
        pendingRobotRequestId_ = -1;
        manualRobotReadPending_ = false;
        if (handEyeCaptureState_ != HandEyeCaptureState::Idle) {
            resetPendingHandEye();
        }
    }
    robotBusy_ = false;
    showError(QStringLiteral("FR5 只读错误"), message);
    updateAllUiStates();
}

void HikCalibrationWindow::onRobotClientError(
        int clientId, QString message) {
    if (clientId != robotClientId_) return;
    robotBusy_ = false;
    showError(QStringLiteral("共享 FR5 错误"), message);
    updateAllUiStates();
}

void HikCalibrationWindow::onCameraConnectionChanged(bool connected,
                                                      QString description) {
    cameraConnected_ = connected;
    if (!connected && pendingCapturePurpose_ != CapturePurpose::None) {
        const CapturePurpose interruptedPurpose = pendingCapturePurpose_;
        appendLog(QStringLiteral("相机断开，已取消未完成的单帧请求。"));
        resetPendingCapture();
        if (interruptedPurpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (interruptedPurpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (interruptedPurpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
    }
    if (!connected && handEyeCaptureState_ != HandEyeCaptureState::Idle) {
        resetPendingHandEye();
    }
    if (!connected) {
        currentCameraModel_.clear();
        currentCameraSerial_.clear();
        currentCameraIp_.clear();
    }
    cameraStatusLabel_->setText(connected
        ? QStringLiteral("已连接：%1").arg(description)
        : QStringLiteral("未连接：%1").arg(description));
    cameraStatusLabel_->setStyleSheet(connected
        ? QStringLiteral("color:#087f23;font-weight:bold;")
        : QStringLiteral("color:#b00020;font-weight:bold;"));
    appendLog(connected
        ? QStringLiteral("相机连接成功: %1").arg(description)
        : QStringLiteral("相机已断开: %1").arg(description));
    if (connected && !currentCameraSerial_.isEmpty()) {
        QString identityError;
        if (!currentCameraMatchesProfile(&identityError)) {
            cameraStatusLabel_->setText(QStringLiteral("设备身份不匹配：%1")
                                        .arg(identityError));
            cameraStatusLabel_->setStyleSheet(
                QStringLiteral("color:#b00020;font-weight:bold;"));
        }
    }
    updateAllUiStates();
}

void HikCalibrationWindow::onCameraIdentityChanged(QString model,
                                                   QString serial,
                                                   QString ipAddress) {
    currentCameraModel_ = model.trimmed();
    currentCameraSerial_ = serial.trimmed();
    currentCameraIp_ = ipAddress.trimmed();
    if (profileIdentityCanChange()) {
        if (profile_.expectedCameraModel.isEmpty() &&
            expectedCameraModelEdit_->text().trimmed().isEmpty() &&
            !currentCameraModel_.isEmpty()) {
            expectedCameraModelEdit_->setText(currentCameraModel_);
        }
        if (profile_.expectedCameraSerial.isEmpty() &&
            expectedCameraSerialEdit_->text().trimmed().isEmpty() &&
            !currentCameraSerial_.isEmpty()) {
            expectedCameraSerialEdit_->setText(currentCameraSerial_);
        }
    }
    appendLog(QStringLiteral("相机身份已锁定：model=%1，SN=%2，IP=%3")
              .arg(currentCameraModel_, currentCameraSerial_, currentCameraIp_));
    if (cameraConnected_) {
        QString identityError;
        if (currentCameraMatchesProfile(&identityError)) {
            cameraStatusLabel_->setText(
                QStringLiteral("%1 身份通过：%2 / %3 / %4")
                    .arg(profile_.id, currentCameraModel_,
                         currentCameraSerial_, currentCameraIp_));
            cameraStatusLabel_->setStyleSheet(
                QStringLiteral("color:#087f23;font-weight:bold;"));
        } else {
            cameraStatusLabel_->setText(
                QStringLiteral("设备身份不匹配：%1").arg(identityError));
            cameraStatusLabel_->setStyleSheet(
                QStringLiteral("color:#b00020;font-weight:bold;"));
            appendLog(QStringLiteral("设备身份拒绝：%1").arg(identityError));
        }
    }
    updateAllUiStates();
}

void HikCalibrationWindow::onCameraBusyChanged(bool busy) {
    cameraBusy_ = busy;
    updateAllUiStates();
}

void HikCalibrationWindow::onCameraFrameReady(int requestId,
                                               QImage image,
                                               quint64 frameNo,
                                               quint64 deviceTimestamp,
                                               qint64 hostTimestamp,
                                               double actualExposure,
                                               double actualGain,
                                               QString description) {
    if (requestId != pendingRequestId_ || pendingCapturePurpose_ == CapturePurpose::None) {
        appendLog(QStringLiteral("忽略过期单帧：request=%1，当前=%2")
                  .arg(requestId).arg(pendingRequestId_));
        return;
    }

    const CapturePurpose purpose = pendingCapturePurpose_;
    QString pairId;
    if (purpose == CapturePurpose::LaserOff) {
        pairId = pendingLaserCapturePairId_;
    } else if (purpose == CapturePurpose::LaserOn) {
        pairId = pendingLaserOff_.sampleId;
    } else if (purpose == CapturePurpose::ProfileOff) {
        pairId = pendingProfileCapturePairId_;
    } else if (purpose == CapturePurpose::ProfileOn) {
        pairId = pendingProfileOff_.sampleId;
    } else if (purpose == CapturePurpose::HandEye) {
        pairId = pendingHandEyeSampleId_;
    }
    resetPendingCapture();
    updateAllUiStates();

    QString imagePath;
    QString saveError;
    if (!saveCameraImageImmediately(image, purpose, pairId, &imagePath, &saveError)) {
        if (purpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (purpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (purpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
        showError(QStringLiteral("原图保存失败"), saveError);
        return;
    }
    appendLog(QStringLiteral("原图已保存: %1").arg(imagePath));
    QString manifestError;
    if (!appendCaptureManifest(purpose, pairId, imagePath, image, frameNo,
                               deviceTimestamp, hostTimestamp, actualExposure,
                               actualGain, description, &manifestError)) {
        if (purpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (purpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (purpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
        showError(QStringLiteral("采集清单写入失败"),
                  manifestError + QStringLiteral("\n原图已保存，但该帧不会进入标定数据集。"));
        return;
    }

    const cv::Mat grayImage = qImageToGrayClone(image);
    imageInfoLabel_->setText(
        QStringLiteral("%1 | %2×%3 | frame=%4 | device_ts=%5 | host_ts=%6 | exposure=%7 us | gain=%8 dB | %9")
            .arg(fileNameOnly(imagePath)).arg(image.width()).arg(image.height())
            .arg(frameNo).arg(deviceTimestamp).arg(hostTimestamp)
            .arg(actualExposure, 0, 'f', 3).arg(actualGain, 0, 'f', 3).arg(description));

    try {
        if (purpose == CapturePurpose::Manual) {
            imageView_->setImage(image.copy());
        } else if (grayImage.empty()) {
            if (purpose == CapturePurpose::LaserOff) {
                resetPendingLaserOff();
            } else if (purpose == CapturePurpose::ProfileOff) {
                resetPendingProfileOff();
            } else if (purpose == CapturePurpose::HandEye) {
                resetPendingHandEye();
            }
            showError(QStringLiteral("图像转换失败"),
                      QStringLiteral("原图已保存，但无法转为 Mono8 供标定使用。"));
        } else if (purpose == CapturePurpose::Intrinsic) {
            if (currentCameraSerial_.isEmpty()) {
                showError(QStringLiteral("内参样本拒绝"),
                          QStringLiteral("未取得相机序列号，原图已保存但不会进入正式数据集。"));
            } else if (!intrinsicDatasetCameraSerial_.isEmpty() &&
                       intrinsicDatasetCameraSerial_ != currentCameraSerial_) {
                showError(QStringLiteral("相机身份不一致"),
                          QStringLiteral("当前相机 SN=%1，数据集相机 SN=%2。请清空数据集后再切换相机。")
                              .arg(currentCameraSerial_, intrinsicDatasetCameraSerial_));
            } else {
                if (intrinsicDatasetCameraSerial_.isEmpty()) {
                    intrinsicDatasetCameraSerial_ = currentCameraSerial_;
                    intrinsicDatasetCameraModel_ = currentCameraModel_;
                }
                addIntrinsicImage(grayImage, imagePath,
                                  QStringLiteral("intrinsic_%1").arg(captureTimestamp()),
                                  true);
            }
        } else if (purpose == CapturePurpose::LaserOff) {
            acceptLaserOffImage(grayImage, imagePath,
                                pairId,
                                actualExposure, actualGain);
        } else if (purpose == CapturePurpose::LaserOn) {
            completeLaserPair(grayImage, imagePath, actualExposure, actualGain);
        } else if (purpose == CapturePurpose::ProfileOff) {
            acceptProfileOffImage(grayImage, imagePath, pairId,
                                  actualExposure, actualGain);
        } else if (purpose == CapturePurpose::ProfileOn) {
            completeProfilePair(grayImage, imagePath, actualExposure, actualGain, true);
        } else if (purpose == CapturePurpose::HandEye) {
            processHandEyeImage(grayImage, imagePath, pairId, hostTimestamp);
        }
    } catch (const cv::Exception& exception) {
        if (purpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (purpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (purpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
        showError(QStringLiteral("OpenCV 处理失败"), QString::fromUtf8(exception.what()));
    } catch (const std::exception& exception) {
        if (purpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (purpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (purpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
        showError(QStringLiteral("图像处理失败"), QString::fromUtf8(exception.what()));
    } catch (...) {
        if (purpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (purpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (purpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
        showError(QStringLiteral("图像处理失败"), QStringLiteral("未知异常。"));
    }
    updateAllUiStates();
}

void HikCalibrationWindow::onCameraLog(QString message) {
    appendLog(QStringLiteral("相机: %1").arg(message));
}

void HikCalibrationWindow::onCameraError(int requestId, QString message) {
    if (requestId == pendingRequestId_ && pendingCapturePurpose_ != CapturePurpose::None) {
        const CapturePurpose failedPurpose = pendingCapturePurpose_;
        resetPendingCapture();
        if (failedPurpose == CapturePurpose::LaserOn && pendingLaserOff_.valid) {
            appendLog(QStringLiteral("laser-on 采集失败，已保留 laser-off，可重试或取消。"));
        } else if (failedPurpose == CapturePurpose::LaserOff) {
            resetPendingLaserOff();
        } else if (failedPurpose == CapturePurpose::ProfileOn && pendingProfileOff_.valid) {
            appendLog(QStringLiteral("静态轮廓 laser-on 采集失败，已保留 off，可重试或取消。"));
        } else if (failedPurpose == CapturePurpose::ProfileOff) {
            resetPendingProfileOff();
        } else if (failedPurpose == CapturePurpose::HandEye) {
            resetPendingHandEye();
        }
    }
    if (requestId < 0) {
        cameraBusy_ = false;
    }
    showError(requestId >= 0 ? QStringLiteral("单帧采集失败")
                             : QStringLiteral("相机错误"), message);
    updateAllUiStates();
}

void HikCalibrationWindow::swapBoardAxes() {
    if (!boardCanChange()) {
        showError(QStringLiteral("无法切换标定板"),
                  QStringLiteral("请先清空全部内参和激光样本，并取消当前 laser-off。"));
        return;
    }
    const int oldX = squaresXSpin_->value();
    squaresXSpin_->setValue(squaresYSpin_->value());
    squaresYSpin_->setValue(oldX);
    appendLog(QStringLiteral("已切换 OpenCV 标定板方格数为 %1×%2。")
              .arg(squaresXSpin_->value()).arg(squaresYSpin_->value()));
    updateAllUiStates();
}

hik_calibration::BoardSpec HikCalibrationWindow::boardSpecFromUi() const {
    hik_calibration::BoardSpec board;
    board.squaresX = squaresXSpin_->value();
    board.squaresY = squaresYSpin_->value();
    board.squareLengthMm = squareLengthSpin_->value();
    board.markerLengthMm = markerLengthSpin_->value();
    board.dictionaryId = dictionaryCombo_->currentData().toInt();
    return board;
}

bool HikCalibrationWindow::checkedBoardSpec(hik_calibration::BoardSpec* board,
                                            QString* error) const {
    if (error) {
        error->clear();
    }
    if (!board) {
        if (error) {
            *error = QStringLiteral("内部错误：标定板输出为空。");
        }
        return false;
    }
    *board = boardSpecFromUi();
    std::string coreError;
    if (!hik_calibration::validateBoardSpec(*board, &coreError)) {
        if (error) {
            *error = fromStdString(coreError);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::boardCanChange() const {
    return intrinsicSamples_.empty() && laserPairs_.empty() && handEyeSamples_.empty() &&
           !pendingLaserOff_.valid && handEyeCaptureState_ == HandEyeCaptureState::Idle;
}

void HikCalibrationWindow::setBoardSpecUi(const hik_calibration::BoardSpec& board) {
    squaresXSpin_->setValue(board.squaresX);
    squaresYSpin_->setValue(board.squaresY);
    squareLengthSpin_->setValue(board.squareLengthMm);
    markerLengthSpin_->setValue(board.markerLengthMm);
    const int index = dictionaryCombo_->findData(board.dictionaryId);
    if (index >= 0) {
        dictionaryCombo_->setCurrentIndex(index);
    }
}

bool HikCalibrationWindow::boardMatches(const hik_calibration::BoardSpec& left,
                                        const hik_calibration::BoardSpec& right) const {
    const double tolerance = 1.0e-6;
    return left.squaresX == right.squaresX && left.squaresY == right.squaresY &&
           left.dictionaryId == right.dictionaryId &&
           std::fabs(left.squareLengthMm - right.squareLengthMm) <= tolerance &&
           std::fabs(left.markerLengthMm - right.markerLengthMm) <= tolerance;
}

int HikCalibrationWindow::beginCapture(CapturePurpose purpose) {
    if (shuttingDown_ || !cameraWorker_ || !cameraThread_.isRunning()) {
        showError(QStringLiteral("无法采集"), QStringLiteral("相机工作线程不可用。"));
        return -1;
    }
    if (!cameraConnected_) {
        showError(QStringLiteral("无法采集"), QStringLiteral("请先连接相机。"));
        return -1;
    }
    QString identityError;
    if (!currentCameraMatchesProfile(&identityError)) {
        showError(QStringLiteral("设备身份不匹配"), identityError);
        return -1;
    }
    if (!sessionReady_) {
        showError(QStringLiteral("无法采集"), QStringLiteral("标定 session 目录未就绪。"));
        return -1;
    }
    if (cameraBusy_ || pendingCapturePurpose_ != CapturePurpose::None) {
        showError(QStringLiteral("无法采集"), QStringLiteral("相机正忙，请等待当前操作完成。"));
        return -1;
    }
    pendingRequestId_ = ++nextRequestId_;
    pendingCapturePurpose_ = purpose;
    updateAllUiStates();
    emit requestCaptureSingle(pendingRequestId_, exposureSpin_->value(),
                              gainSpin_->value(), timeoutSpin_->value());
    return pendingRequestId_;
}

QString HikCalibrationWindow::captureTimestamp() const {
    return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
}

QString HikCalibrationWindow::uniqueFilePath(const QString& directory,
                                             const QString& stem,
                                             const QString& suffix) const {
    const QString cleanSuffix = suffix.startsWith(QLatin1Char('.'))
        ? suffix : QStringLiteral(".") + suffix;
    QDir dir(directory);
    for (int index = 0; index < 10000; ++index) {
        const QString name = index == 0
            ? stem + cleanSuffix
            : stem + QStringLiteral("_%1").arg(index, 3, 10, QLatin1Char('0')) + cleanSuffix;
        const QString path = dir.absoluteFilePath(name);
        if (!QFileInfo::exists(path)) {
            return path;
        }
    }
    return QString();
}

bool HikCalibrationWindow::saveCameraImageImmediately(const QImage& image,
                                                       CapturePurpose purpose,
                                                       const QString& pairId,
                                                       QString* path,
                                                       QString* error) const {
    if (path) {
        path->clear();
    }
    if (error) {
        error->clear();
    }
    if (!sessionReady_ || image.isNull()) {
        if (error) {
            *error = QStringLiteral("会话目录未就绪或相机返回空图像。");
        }
        return false;
    }
    const bool laser = purpose == CapturePurpose::LaserOff || purpose == CapturePurpose::LaserOn;
    const bool profile = purpose == CapturePurpose::ProfileOff || purpose == CapturePurpose::ProfileOn;
    const QString directory = purpose == CapturePurpose::HandEye
        ? handEyeSessionDir_
        : (profile ? profileSessionDir_
                   : (laser ? laserSessionDir_ : intrinsicSessionDir_));
    QString stem = QStringLiteral("preview_%1").arg(captureTimestamp());
    if (purpose == CapturePurpose::Intrinsic) {
        stem = QStringLiteral("intrinsic_%1").arg(captureTimestamp());
    } else if (purpose == CapturePurpose::LaserOff) {
        if (pairId.isEmpty()) {
            if (error) {
                *error = QStringLiteral("laser-off 缺少配对 ID，已拒绝保存以免产生无法对应的文件。");
            }
            return false;
        }
        stem = pairId + QStringLiteral("_off");
    } else if (purpose == CapturePurpose::LaserOn) {
        if (pairId.isEmpty()) {
            if (error) {
                *error = QStringLiteral("laser-on 缺少配对 ID，已拒绝保存以免产生无法对应的文件。");
            }
            return false;
        }
        stem = pairId + QStringLiteral("_on");
    } else if (purpose == CapturePurpose::ProfileOff) {
        if (pairId.isEmpty()) {
            if (error) {
                *error = QStringLiteral("profile-off 缺少配对 ID。");
            }
            return false;
        }
        stem = pairId + QStringLiteral("_off");
    } else if (purpose == CapturePurpose::ProfileOn) {
        if (pairId.isEmpty()) {
            if (error) {
                *error = QStringLiteral("profile-on 缺少配对 ID。");
            }
            return false;
        }
        stem = pairId + QStringLiteral("_on");
    } else if (purpose == CapturePurpose::HandEye) {
        if (pairId.isEmpty()) {
            if (error) {
                *error = QStringLiteral("手眼采图缺少样本 ID。");
            }
            return false;
        }
        stem = pairId;
    }
    const QString target = uniqueFilePath(directory, stem, QStringLiteral("png"));
    if (target.isEmpty() || !image.copy().save(target, "PNG")) {
        if (error) {
            *error = QStringLiteral("无法保存 PNG 原图: %1").arg(target);
        }
        return false;
    }
    if (path) {
        *path = target;
    }
    return true;
}

bool HikCalibrationWindow::copyImportedImageToSession(const QString& sourcePath,
                                                       const QString& targetDirectory,
                                                       const QString& stem,
                                                       cv::Mat* grayImage,
                                                       QString* targetPath,
                                                       QString* error) const {
    if (grayImage) {
        grayImage->release();
    }
    if (targetPath) {
        targetPath->clear();
    }
    if (error) {
        error->clear();
    }
    const QFileInfo sourceInfo(sourcePath);
    if (!sourceInfo.isFile() || !sourceInfo.isReadable()) {
        if (error) {
            *error = QStringLiteral("导入文件不可读: %1").arg(sourcePath);
        }
        return false;
    }
    QString suffix = sourceInfo.suffix();
    if (suffix.isEmpty()) {
        suffix = QStringLiteral("img");
    }
    const QString target = uniqueFilePath(targetDirectory, stem, suffix);
    if (target.isEmpty() || !QFile::copy(sourcePath, target)) {
        if (error) {
            *error = QStringLiteral("无法将导入原图原样复制到 session: %1").arg(sourcePath);
        }
        return false;
    }
    const cv::Mat decoded = cv::imread(toLocalPath(target), cv::IMREAD_UNCHANGED);
    const cv::Mat gray = gray8Clone(decoded);
    if (gray.empty()) {
        QFile::remove(target);
        if (error) {
            *error = QStringLiteral("OpenCV 无法解码导入图像: %1").arg(sourcePath);
        }
        return false;
    }
    if (grayImage) {
        *grayImage = gray.clone();
    }
    if (targetPath) {
        *targetPath = target;
    }
    return true;
}

void HikCalibrationWindow::importIntrinsicImages() {
    hik_calibration::BoardSpec board;
    QString boardError;
    if (!checkedBoardSpec(&board, &boardError)) {
        showError(QStringLiteral("板参数错误"), boardError);
        return;
    }
    const QStringList files = QFileDialog::getOpenFileNames(
        this, QStringLiteral("导入内参标定图像"), sourceDir_,
        QStringLiteral("图像 (*.png *.bmp *.jpg *.jpeg *.tif *.tiff);;所有文件 (*)"));
    if (files.isEmpty()) {
        return;
    }

    int imported = 0;
    for (int index = 0; index < files.size(); ++index) {
        cv::Mat gray;
        QString storedPath;
        QString error;
        const QString id = QStringLiteral("intrinsic_import_%1_%2")
            .arg(captureTimestamp()).arg(index + 1, 3, 10, QLatin1Char('0'));
        if (!copyImportedImageToSession(files.at(index), intrinsicSessionDir_, id,
                                        &gray, &storedPath, &error)) {
            appendLog(QStringLiteral("导入跳过: %1").arg(error));
            continue;
        }
        try {
            addIntrinsicImage(gray, storedPath, id, false);
            ++imported;
        } catch (const cv::Exception& exception) {
            appendLog(QStringLiteral("图像已复制但检测异常 %1: %2")
                      .arg(storedPath, QString::fromUtf8(exception.what())));
        } catch (const std::exception& exception) {
            appendLog(QStringLiteral("图像已复制但处理异常 %1: %2")
                      .arg(storedPath, QString::fromUtf8(exception.what())));
        }
    }
    appendLog(QStringLiteral("内参批量导入完成：%1 / %2 张进入样本表。")
              .arg(imported).arg(files.size()));
    updateAllUiStates();
}

void HikCalibrationWindow::deleteSelectedIntrinsicSamples() {
    if (!intrinsicTable_->selectionModel()) {
        return;
    }
    const QModelIndexList selected = intrinsicTable_->selectionModel()->selectedRows();
    std::vector<int> rows;
    rows.reserve(static_cast<std::size_t>(selected.size()));
    for (int index = 0; index < selected.size(); ++index) {
        rows.push_back(selected.at(index).row());
    }
    std::sort(rows.begin(), rows.end(), std::greater<int>());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    for (std::size_t index = 0; index < rows.size(); ++index) {
        const int row = rows[index];
        if (row >= 0 && row < static_cast<int>(intrinsicSamples_.size())) {
            intrinsicSamples_.erase(intrinsicSamples_.begin() + row);
        }
    }
    if (!rows.empty()) {
        const bool hasVerifiedLiveSample = std::any_of(
            intrinsicSamples_.begin(), intrinsicSamples_.end(),
            [](const IntrinsicSample& sample) {
                return sample.acquisitionIdentityVerified;
            });
        if (!hasVerifiedLiveSample) {
            intrinsicDatasetCameraSerial_.clear();
            intrinsicDatasetCameraModel_.clear();
        }
        invalidateIntrinsicSolution();
        refreshIntrinsicTable();
        appendLog(QStringLiteral("已从内参数据集移除 %1 个样本；原图仍保留在 session。")
                  .arg(static_cast<int>(rows.size())));
    }
    updateAllUiStates();
}

void HikCalibrationWindow::clearIntrinsicSamples() {
    if (intrinsicSamples_.empty()) {
        return;
    }
    if (QMessageBox::question(
            this, QStringLiteral("清空内参样本"),
            QStringLiteral("确定清空内参样本表和当前求解结果吗？\n"
                           "session 中已保存的原图不会删除。"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    intrinsicSamples_.clear();
    intrinsicDatasetCameraSerial_.clear();
    intrinsicDatasetCameraModel_.clear();
    invalidateIntrinsicSolution();
    refreshIntrinsicTable();
    appendLog(QStringLiteral("已清空内参样本表。"));
    updateAllUiStates();
}

void HikCalibrationWindow::addIntrinsicImage(const cv::Mat& grayImage,
                                             const QString& imagePath,
                                             const QString& sampleId,
                                             bool acquisitionIdentityVerified) {
    hik_calibration::BoardSpec board;
    QString boardError;
    if (!checkedBoardSpec(&board, &boardError)) {
        throw std::runtime_error(boardError.toStdString());
    }

    IntrinsicSample sample;
    sample.sampleId = sampleId;
    sample.imagePath = imagePath;
    sample.acquisitionIdentityVerified = acquisitionIdentityVerified;
    hik_calibration::detectCharuco(grayImage, sampleId.toStdString(), board,
                                  detectionOptions_, &sample.detection);
    sample.calibrationRmsPx = std::numeric_limits<double>::quiet_NaN();
    intrinsicSamples_.push_back(sample);
    invalidateIntrinsicSolution();
    refreshIntrinsicTable();

    const bool accepted = sample.detection.ok && sample.detection.observation.quality.accepted;
    const QString reason = accepted ? QStringLiteral("检测质量通过")
                                    : fromStdString(sample.detection.error);
    imageView_->setImage(drawCharucoOverlay(
        grayImage, sample.detection,
        accepted ? QStringLiteral("intrinsic detection OK")
                 : QStringLiteral("intrinsic detection REJECT")));
    imageInfoLabel_->setText(QStringLiteral("%1 | Marker=%2 | ChArUco=%3 | %4")
        .arg(fileNameOnly(imagePath))
        .arg(sample.detection.observation.quality.markerCount)
        .arg(sample.detection.observation.quality.cornerCount)
        .arg(reason));
    appendLog(QStringLiteral("内参样本 %1: %2").arg(sampleId, reason));
    updateAllUiStates();
}

void HikCalibrationWindow::refreshIntrinsicTable() {
    intrinsicTable_->setRowCount(static_cast<int>(intrinsicSamples_.size()));
    for (int row = 0; row < static_cast<int>(intrinsicSamples_.size()); ++row) {
        const IntrinsicSample& sample = intrinsicSamples_[static_cast<std::size_t>(row)];
        const hik_calibration::CharucoQuality& quality = sample.detection.observation.quality;
        const cv::Size size = sample.detection.observation.imageSize;
        QString status;
        if (!sample.detection.ok || !quality.accepted) {
            status = QStringLiteral("检测拒绝: %1").arg(fromStdString(sample.detection.error));
        } else if (sample.calibrationAccepted) {
            status = QStringLiteral("标定采用");
        } else if (!sample.calibrationReason.isEmpty()) {
            status = QStringLiteral("标定剔除: %1").arg(sample.calibrationReason);
        } else {
            status = QStringLiteral("检测通过，待求解");
        }
        const QStringList values = QStringList()
            << sample.sampleId
            << fileNameOnly(sample.imagePath)
            << (size.width > 0 ? QStringLiteral("%1×%2").arg(size.width).arg(size.height)
                               : QStringLiteral("-"))
            << QString::number(quality.markerCount)
            << QString::number(quality.cornerCount)
            << formatNumber(quality.hullAreaRatio * 100.0, 2)
            << formatNumber(quality.minimumBorderDistancePx, 1)
            << formatNumber(quality.meanIntensity, 1)
            << formatNumber(quality.saturationRatio * 100.0, 2)
            << formatNumber(quality.laplacianVariance, 1)
            << (sample.calibrationAccepted ? formatNumber(sample.calibrationRmsPx, 4)
                                           : QStringLiteral("-"))
            << status;
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            item->setToolTip(column == 1 ? sample.imagePath : status);
            intrinsicTable_->setItem(row, column, item);
        }
    }
}

int HikCalibrationWindow::acceptedIntrinsicSampleCount() const {
    return static_cast<int>(std::count_if(
        intrinsicSamples_.begin(), intrinsicSamples_.end(),
        [](const IntrinsicSample& sample) {
            return sample.detection.ok && sample.detection.observation.quality.accepted;
        }));
}

void HikCalibrationWindow::invalidateIntrinsicSolution() {
    intrinsicResult_ = hik_calibration::IntrinsicCalibrationResult();
    hasIntrinsicResult_ = false;
    lastIntrinsicCandidatePath_.clear();
    if (intrinsicResultLabel_) {
        intrinsicResultLabel_->setText(QStringLiteral("数据集已变更，需重新求解。"));
    }
    for (std::size_t index = 0; index < intrinsicSamples_.size(); ++index) {
        intrinsicSamples_[index].calibrationAccepted = false;
        intrinsicSamples_[index].calibrationRmsPx = std::numeric_limits<double>::quiet_NaN();
        intrinsicSamples_[index].calibrationReason.clear();
    }
}

void HikCalibrationWindow::solveIntrinsics() {
    hik_calibration::BoardSpec board;
    QString boardError;
    if (!checkedBoardSpec(&board, &boardError)) {
        showError(QStringLiteral("板参数错误"), boardError);
        return;
    }
    if (acceptedIntrinsicSampleCount() < intrinsicOptions_.minViews) {
        showError(QStringLiteral("内参数据不足"),
                  QStringLiteral("至少需要 %1 张通过检测质量的图像。")
                      .arg(intrinsicOptions_.minViews));
        return;
    }

    std::vector<hik_calibration::CharucoObservation> observations;
    observations.reserve(intrinsicSamples_.size());
    for (std::size_t index = 0; index < intrinsicSamples_.size(); ++index) {
        observations.push_back(intrinsicSamples_[index].detection.observation);
    }

    appendLog(QStringLiteral("开始内参求解，输入 %1 个视图。")
              .arg(static_cast<int>(observations.size())));
    hik_calibration::IntrinsicCalibrationResult result;
    bool solved = false;
    try {
        solved = hik_calibration::calibrateIntrinsics(
            observations, board, intrinsicOptions_, &result);
    } catch (const cv::Exception& exception) {
        result.error = std::string("OpenCV exception: ") + exception.what();
    } catch (const std::exception& exception) {
        result.error = std::string("exception: ") + exception.what();
    } catch (...) {
        result.error = "unknown exception";
    }
    intrinsicResult_ = result;
    hasIntrinsicResult_ = solved && result.ok && !result.cameraMatrix.empty() &&
                          !result.distCoeffs.empty();

    for (std::size_t index = 0; index < intrinsicSamples_.size(); ++index) {
        intrinsicSamples_[index].calibrationAccepted = false;
        intrinsicSamples_[index].calibrationRmsPx = std::numeric_limits<double>::quiet_NaN();
        intrinsicSamples_[index].calibrationReason.clear();
    }
    for (std::size_t viewIndex = 0; viewIndex < result.views.size(); ++viewIndex) {
        const hik_calibration::IntrinsicViewResult& view = result.views[viewIndex];
        for (std::size_t sampleIndex = 0; sampleIndex < intrinsicSamples_.size(); ++sampleIndex) {
            if (intrinsicSamples_[sampleIndex].sampleId.toStdString() == view.sampleId) {
                intrinsicSamples_[sampleIndex].calibrationAccepted = view.accepted;
                intrinsicSamples_[sampleIndex].calibrationRmsPx = view.reprojection.rms;
                intrinsicSamples_[sampleIndex].calibrationReason = fromStdString(view.rejectReason);
                break;
            }
        }
    }
    refreshIntrinsicTable();

    if (!hasIntrinsicResult_) {
        const QString reason = fromStdString(result.error);
        intrinsicResultLabel_->setText(QStringLiteral("求解失败: %1").arg(reason));
        appendLog(QStringLiteral("内参求解失败: %1").arg(reason));
        updateAllUiStates();
        return;
    }

    QString qualityReason;
    const bool qualityPass = intrinsicResultPassesQuality(&qualityReason);
    intrinsicResultLabel_->setText(
        QStringLiteral("内参求解完成：RMS=%1 px，采用=%2，剔除=%3。质量门槛：%4%5")
            .arg(result.calibrationRmsPx, 0, 'f', 5)
            .arg(result.acceptedViewCount).arg(result.rejectedViewCount)
            .arg(qualityPass ? QStringLiteral("通过") : QStringLiteral("不通过"))
            .arg(qualityReason.isEmpty() ? QString() : QStringLiteral("（%1）").arg(qualityReason)));
    appendLog(intrinsicResultLabel_->text());
    saveIntrinsicCandidate();
    updateAllUiStates();
}

bool HikCalibrationWindow::intrinsicResultHasMatrices() const {
    return hasIntrinsicResult_ && intrinsicResult_.ok &&
           intrinsicResult_.cameraMatrix.rows == 3 &&
           intrinsicResult_.cameraMatrix.cols == 3 &&
           !intrinsicResult_.distCoeffs.empty();
}

bool HikCalibrationWindow::intrinsicResultPassesQuality(QString* reason) const {
    if (reason) {
        reason->clear();
    }
    if (!intrinsicResultHasMatrices()) {
        if (reason) {
            *reason = QStringLiteral("没有有效内参矩阵。");
        }
        return false;
    }
    if (intrinsicResult_.acceptedViewCount < intrinsicOptions_.minViews) {
        if (reason) {
            *reason = QStringLiteral("通过视图数不足。");
        }
        return false;
    }
    if (!std::isfinite(intrinsicResult_.calibrationRmsPx) ||
        intrinsicResult_.calibrationRmsPx > kApprovedIntrinsicRmsPx) {
        if (reason) {
            *reason = QStringLiteral("总 RMS %1 px 超过正式门槛 %2 px。")
                .arg(intrinsicResult_.calibrationRmsPx, 0, 'f', 5)
                .arg(kApprovedIntrinsicRmsPx, 0, 'f', 2);
        }
        return false;
    }
    if (!cv::checkRange(intrinsicResult_.cameraMatrix) ||
        !cv::checkRange(intrinsicResult_.distCoeffs)) {
        if (reason) {
            *reason = QStringLiteral("矩阵包含非有限数。");
        }
        return false;
    }
    if (intrinsicDatasetCameraSerial_.isEmpty()) {
        if (reason) {
            *reason = QStringLiteral("数据集未绑定海康相机序列号；离线导入数据仅可保存候选。");
        }
        return false;
    }

    std::vector<double> viewRmsValues;
    std::vector<cv::Vec3d> boardNormals;
    std::vector<double> boardDepths;
    for (std::size_t index = 0; index < intrinsicResult_.views.size(); ++index) {
        const hik_calibration::IntrinsicViewResult& view = intrinsicResult_.views[index];
        if (!view.accepted) {
            continue;
        }
        if (!std::isfinite(view.reprojection.rms) || !std::isfinite(view.tvec[2])) {
            if (reason) {
                *reason = QStringLiteral("通过视图含非有限的重投影或位姿数据。");
            }
            return false;
        }
        viewRmsValues.push_back(view.reprojection.rms);
        boardDepths.push_back(view.tvec[2]);
        cv::Mat rotation;
        cv::Rodrigues(view.rvec, rotation);
        boardNormals.push_back(cv::Vec3d(
            rotation.at<double>(0, 2), rotation.at<double>(1, 2),
            rotation.at<double>(2, 2)));
    }
    if (viewRmsValues.size() < static_cast<std::size_t>(intrinsicOptions_.minViews)) {
        if (reason) {
            *reason = QStringLiteral("缺少足够的逐视图质量统计。");
        }
        return false;
    }
    const double viewP95 = percentileValue(viewRmsValues, 0.95);
    const double viewMaximum = *std::max_element(viewRmsValues.begin(), viewRmsValues.end());
    if (viewP95 > kApprovedIntrinsicViewP95Px ||
        viewMaximum > kApprovedIntrinsicViewMaxPx) {
        if (reason) {
            *reason = QStringLiteral("逐视图 RMS 的 P95=%1 px、最大=%2 px，要求不超过 %3/%4 px。")
                .arg(viewP95, 0, 'f', 4).arg(viewMaximum, 0, 'f', 4)
                .arg(kApprovedIntrinsicViewP95Px, 0, 'f', 2)
                .arg(kApprovedIntrinsicViewMaxPx, 0, 'f', 2);
        }
        return false;
    }

    if (intrinsicResult_.imageSize.width <= 0 || intrinsicResult_.imageSize.height <= 0) {
        if (reason) {
            *reason = QStringLiteral("内参结果缺少有效图像尺寸。");
        }
        return false;
    }
    double minimumX = std::numeric_limits<double>::max();
    double minimumY = std::numeric_limits<double>::max();
    double maximumX = -std::numeric_limits<double>::max();
    double maximumY = -std::numeric_limits<double>::max();
    double minimumHull = std::numeric_limits<double>::max();
    double maximumHull = 0.0;
    std::set<int> occupiedCenterCells;
    int acceptedObservationCount = 0;
    for (std::size_t index = 0;
         index < intrinsicResult_.acceptedObservationIndices.size(); ++index) {
        const int observationIndex = intrinsicResult_.acceptedObservationIndices[index];
        if (observationIndex < 0 ||
            observationIndex >= static_cast<int>(intrinsicSamples_.size())) {
            continue;
        }
        const IntrinsicSample& sample =
            intrinsicSamples_[static_cast<std::size_t>(observationIndex)];
        if (!sample.acquisitionIdentityVerified) {
            if (reason) {
                *reason = QStringLiteral("存在离线导入或未验证相机身份的采用视图；仅可保存候选。");
            }
            return false;
        }
        const hik_calibration::CharucoObservation& observation =
            sample.detection.observation;
        if (observation.corners.empty()) {
            continue;
        }
        ++acceptedObservationCount;
        cv::Point2d center(0.0, 0.0);
        for (std::size_t cornerIndex = 0;
             cornerIndex < observation.corners.size(); ++cornerIndex) {
            const cv::Point2f& corner = observation.corners[cornerIndex];
            minimumX = std::min(minimumX, static_cast<double>(corner.x));
            minimumY = std::min(minimumY, static_cast<double>(corner.y));
            maximumX = std::max(maximumX, static_cast<double>(corner.x));
            maximumY = std::max(maximumY, static_cast<double>(corner.y));
            center.x += corner.x;
            center.y += corner.y;
        }
        center.x /= static_cast<double>(observation.corners.size());
        center.y /= static_cast<double>(observation.corners.size());
        const int cellX = std::max(0, std::min(2, static_cast<int>(
            3.0 * center.x / intrinsicResult_.imageSize.width)));
        const int cellY = std::max(0, std::min(2, static_cast<int>(
            3.0 * center.y / intrinsicResult_.imageSize.height)));
        occupiedCenterCells.insert(cellY * 3 + cellX);
        minimumHull = std::min(minimumHull, observation.quality.hullAreaRatio);
        maximumHull = std::max(maximumHull, observation.quality.hullAreaRatio);
    }
    if (acceptedObservationCount < intrinsicOptions_.minViews ||
        minimumX == std::numeric_limits<double>::max()) {
        if (reason) {
            *reason = QStringLiteral("通过视图与原始观测的映射不完整。");
        }
        return false;
    }
    const double widthCoverage = (maximumX - minimumX) /
        static_cast<double>(intrinsicResult_.imageSize.width);
    const double heightCoverage = (maximumY - minimumY) /
        static_cast<double>(intrinsicResult_.imageSize.height);
    if (widthCoverage < 0.70 || heightCoverage < 0.70 ||
        occupiedCenterCells.size() < 5U) {
        if (reason) {
            *reason = QStringLiteral("画面覆盖不足：角点范围=%1%×%2%，板中心仅覆盖九宫格 %3 格；要求至少 70%×70% 且 5 格。")
                .arg(widthCoverage * 100.0, 0, 'f', 1)
                .arg(heightCoverage * 100.0, 0, 'f', 1)
                .arg(static_cast<int>(occupiedCenterCells.size()));
        }
        return false;
    }
    if (!std::isfinite(minimumHull) || minimumHull <= 0.0 ||
        maximumHull / minimumHull < 1.40) {
        if (reason) {
            *reason = QStringLiteral("标定板远近尺度变化不足（最大/最小角点面积比需至少 1.40）。");
        }
        return false;
    }

    double maximumNormalAngleDeg = 0.0;
    for (std::size_t first = 0; first < boardNormals.size(); ++first) {
        for (std::size_t second = first + 1; second < boardNormals.size(); ++second) {
            maximumNormalAngleDeg = std::max(
                maximumNormalAngleDeg,
                angleBetweenNormalsDeg(boardNormals[first], boardNormals[second]));
        }
    }
    const std::pair<std::vector<double>::const_iterator,
                    std::vector<double>::const_iterator> depthRange =
        std::minmax_element(boardDepths.begin(), boardDepths.end());
    const double meanDepth = std::accumulate(
        boardDepths.begin(), boardDepths.end(), 0.0) /
        static_cast<double>(boardDepths.size());
    const double depthSpan = *depthRange.second - *depthRange.first;
    const double requiredDepthSpan = std::max(30.0, std::fabs(meanDepth) * 0.06);
    if (maximumNormalAngleDeg < 12.0 || depthSpan < requiredDepthSpan) {
        if (reason) {
            *reason = QStringLiteral("姿态多样性不足：最大板倾角差=%1°、深度跨度=%2 mm；要求至少 12° 和 %3 mm。")
                .arg(maximumNormalAngleDeg, 0, 'f', 1)
                .arg(depthSpan, 0, 'f', 1)
                .arg(requiredDepthSpan, 0, 'f', 1);
        }
        return false;
    }
    return true;
}

hik_calibration::IntrinsicsYamlMetadata HikCalibrationWindow::intrinsicMetadata() const {
    hik_calibration::IntrinsicsYamlMetadata metadata;
    metadata.cameraName = profile_.id.toStdString();
    metadata.cameraModel = intrinsicDatasetCameraModel_.isEmpty()
        ? expectedCameraModelFromUi().toStdString()
        : intrinsicDatasetCameraModel_.toStdString();
    metadata.cameraSerial = intrinsicDatasetCameraSerial_.toStdString();
    metadata.frameId = profile_.cameraFrame.toStdString();
    metadata.pixelFormat = "Mono8";
    metadata.printedPatternSha256.clear();
    metadata.generatedAt = QDateTime::currentDateTimeUtc()
        .toString(Qt::ISODateWithMs).toStdString();
    return metadata;
}

bool HikCalibrationWindow::writeIntrinsicYaml(const QString& path, QString* error) const {
    if (error) {
        error->clear();
    }
    if (!intrinsicResultHasMatrices()) {
        if (error) {
            *error = QStringLiteral("没有可保存的内参求解结果。");
        }
        return false;
    }
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
        if (error) {
            *error = QStringLiteral("无法创建 YAML 所在目录: %1").arg(QFileInfo(path).absolutePath());
        }
        return false;
    }
    std::string coreError;
    if (!hik_calibration::saveIntrinsicsYaml(
            toLocalPath(path), intrinsicResult_, intrinsicMetadata(), &coreError)) {
        if (error) {
            *error = fromStdString(coreError);
        }
        return false;
    }
    return true;
}

void HikCalibrationWindow::saveIntrinsicCandidate() {
    if (!intrinsicResultHasMatrices()) {
        showError(QStringLiteral("无法保存内参"), QStringLiteral("请先成功求解内参。"));
        return;
    }
    const QString path = uniqueFilePath(
        intrinsicSessionDir_,
        QStringLiteral("hik_intrinsics_candidate_%1").arg(captureTimestamp()),
        QStringLiteral("yaml"));
    QString error;
    if (!writeIntrinsicYaml(path, &error)) {
        showError(QStringLiteral("候选内参保存失败"), error);
        return;
    }
    lastIntrinsicCandidatePath_ = path;
    appendLog(QStringLiteral("内参 session 候选 YAML 已保存: %1").arg(path));
    updateAllUiStates();
}

void HikCalibrationWindow::saveApprovedIntrinsics() {
    QString qualityReason;
    if (!intrinsicResultPassesQuality(&qualityReason)) {
        showError(QStringLiteral("禁止覆盖正式内参"),
                  QStringLiteral("当前结果未通过质量门槛：%1\n正式文件保持不变。")
                      .arg(qualityReason));
        return;
    }
    const QString approvedSessionPath = uniqueFilePath(
        intrinsicSessionDir_,
        QStringLiteral("hik_intrinsics_approved_%1").arg(captureTimestamp()),
        QStringLiteral("yaml"));
    QString error;
    if (!writeIntrinsicYaml(approvedSessionPath, &error)) {
        showError(QStringLiteral("内参保存失败"), error);
        return;
    }
    const QString formalPath = profile_.intrinsicsConfigPath(sourceDir_);
    const QString formalLaserPath = profile_.laserPlaneConfigPath(sourceDir_);
    const QString formalHandEyePath = profile_.handEyeConfigPath(sourceDir_);
    const QString staleSuffix = QStringLiteral(".stale_%1").arg(captureTimestamp());
    QString staleLaserPath;
    QString staleHandEyePath;
    if (QFileInfo(formalLaserPath).isFile()) {
        staleLaserPath = formalLaserPath + staleSuffix;
        if (!QFile::rename(formalLaserPath, staleLaserPath)) {
            showError(QStringLiteral("正式内参更新已取消"),
                      QStringLiteral("无法先将旧激光平面标记为失效：%1\n"
                                     "为避免新内参与旧激光平面混用，正式文件均保持不变。")
                          .arg(formalLaserPath));
            return;
        }
    }
    if (QFileInfo(formalHandEyePath).isFile()) {
        staleHandEyePath = formalHandEyePath + staleSuffix;
        if (!QFile::rename(formalHandEyePath, staleHandEyePath)) {
            if (!staleLaserPath.isEmpty()) {
                QFile::rename(staleLaserPath, formalLaserPath);
            }
            showError(QStringLiteral("正式内参更新已取消"),
                      QStringLiteral("无法先将旧手眼标定标记为失效：%1\n"
                                     "正式文件均保持不变。")
                          .arg(formalHandEyePath));
            return;
        }
    }
    if (!promoteFileAtomically(approvedSessionPath, formalPath, &error)) {
        if (!staleLaserPath.isEmpty()) {
            QFile::rename(staleLaserPath, formalLaserPath);
        }
        if (!staleHandEyePath.isEmpty()) {
            QFile::rename(staleHandEyePath, formalHandEyePath);
        }
        showError(QStringLiteral("正式内参更新失败"), error);
        return;
    }
    lastIntrinsicCandidatePath_ = approvedSessionPath;
    appendLog(QStringLiteral("内参质量通过，已原子更新正式文件: %1").arg(formalPath));
    if (!staleLaserPath.isEmpty()) {
        appendLog(QStringLiteral("旧激光平面已因内参变更标记为失效: %1")
                  .arg(staleLaserPath));
    }
    if (!staleHandEyePath.isEmpty()) {
        appendLog(QStringLiteral("旧手眼标定已因内参变更标记为失效: %1")
                  .arg(staleHandEyePath));
    }
    QMessageBox::information(this, QStringLiteral("内参已批准"),
                             QStringLiteral("已更新:\n%1").arg(formalPath));
}

void HikCalibrationWindow::loadIntrinsicsForLaser() {
    const QString formalIntrinsicsPath =
        profile_.intrinsicsConfigPath(sourceDir_);
    const QString initial = QFileInfo(formalIntrinsicsPath).exists()
        ? formalIntrinsicsPath
        : QFileInfo(formalIntrinsicsPath).absolutePath();
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("加载激光面所用内参"), initial,
        QStringLiteral("YAML (*.yaml *.yml);;所有文件 (*)"));
    if (path.isEmpty()) {
        return;
    }
    hik_calibration::IntrinsicCalibrationResult calibration;
    hik_calibration::IntrinsicsYamlMetadata metadata;
    std::string coreError;
    try {
        if (!hik_calibration::loadIntrinsicsYaml(
                toLocalPath(path), &calibration, &metadata, &coreError)) {
            showError(QStringLiteral("内参加载失败"), fromStdString(coreError));
            return;
        }
    } catch (const cv::Exception& exception) {
        showError(QStringLiteral("内参加载失败"), QString::fromUtf8(exception.what()));
        return;
    } catch (const std::exception& exception) {
        showError(QStringLiteral("内参加载失败"), QString::fromUtf8(exception.what()));
        return;
    }
    QString error;
    if (!setActiveLaserIntrinsics(calibration, metadata, path, &error)) {
        showError(QStringLiteral("内参不可用"), error);
        return;
    }
    appendLog(QStringLiteral("激光面内参已加载: %1").arg(path));
}

void HikCalibrationWindow::useCurrentIntrinsicsForLaser() {
    if (!intrinsicResultHasMatrices()) {
        showError(QStringLiteral("无法沿用内参"), QStringLiteral("当前没有成功的内参求解结果。"));
        return;
    }
    QString sourcePath = lastIntrinsicCandidatePath_;
    if (sourcePath.isEmpty()) {
        sourcePath = QStringLiteral("session://current_intrinsics");
    }
    QString error;
    if (!setActiveLaserIntrinsics(intrinsicResult_, intrinsicMetadata(), sourcePath, &error)) {
        showError(QStringLiteral("无法沿用内参"), error);
        return;
    }
    appendLog(QStringLiteral("激光面标定已沿用当前内参求解结果。"));
}

bool HikCalibrationWindow::setActiveLaserIntrinsics(
        const hik_calibration::IntrinsicCalibrationResult& calibration,
        const hik_calibration::IntrinsicsYamlMetadata& metadata,
        const QString& sourcePath,
        QString* error) {
    if (error) {
        error->clear();
    }
    if (!laserPairs_.empty() || pendingLaserOff_.valid) {
        if (error) {
            *error = QStringLiteral("已有激光样本或待配对 laser-off，不能中途更换内参。");
        }
        return false;
    }
    if (!intrinsicsMetadataMatchesProfile(metadata, error)) {
        return false;
    }
    std::string boardError;
    if (!calibration.ok || calibration.cameraMatrix.rows != 3 ||
        calibration.cameraMatrix.cols != 3 || calibration.distCoeffs.total() < 4 ||
        calibration.imageSize.width <= 0 || calibration.imageSize.height <= 0 ||
        !cv::checkRange(calibration.cameraMatrix) || !cv::checkRange(calibration.distCoeffs) ||
        !hik_calibration::validateBoardSpec(calibration.board, &boardError)) {
        if (error) {
            *error = boardError.empty()
                ? QStringLiteral("内参 YAML 缺少有效矩阵、图像尺寸或标定板信息。")
                : fromStdString(boardError);
        }
        return false;
    }
    if (boardCanChange()) {
        setBoardSpecUi(calibration.board);
    } else if (!boardMatches(boardSpecFromUi(), calibration.board)) {
        if (error) {
            *error = QStringLiteral("内参 YAML 的标定板与当前已锁定数据集不匹配。");
        }
        return false;
    }

    QString sourceSha256;
    if (QFileInfo(sourcePath).isFile()) {
        QString hashError;
        sourceSha256 = fileSha256Hex(sourcePath, &hashError);
        if (sourceSha256.isEmpty()) {
            if (error) {
                *error = hashError;
            }
            return false;
        }
    }

    activeLaserIntrinsics_ = calibration;
    activeLaserIntrinsics_.cameraMatrix = calibration.cameraMatrix.clone();
    activeLaserIntrinsics_.distCoeffs = calibration.distCoeffs.clone();
    activeLaserIntrinsics_.intrinsicStdDeviations =
        calibration.intrinsicStdDeviations.clone();
    activeLaserIntrinsicsMetadata_ = metadata;
    activeLaserIntrinsicsPath_ = sourcePath;
    activeLaserIntrinsicsSha256_ = sourceSha256;
    hasActiveLaserIntrinsics_ = true;
    invalidateLaserSolution();
    laserIntrinsicsPathEdit_->setText(sourcePath);
    laserIntrinsicsStatusLabel_->setText(
        QStringLiteral("已加载 %1×%2，RMS=%3 px，板=%4×%5 %6")
            .arg(calibration.imageSize.width).arg(calibration.imageSize.height)
            .arg(calibration.calibrationRmsPx, 0, 'f', 5)
            .arg(calibration.board.squaresX).arg(calibration.board.squaresY)
            .arg(fromStdString(hik_calibration::dictionaryName(calibration.board.dictionaryId))));
    laserIntrinsicsStatusLabel_->setStyleSheet(QStringLiteral("color:#087f23;"));
    updateAllUiStates();
    return true;
}

bool HikCalibrationWindow::loadFormalProfileCalibration(QString* error) {
    if (error) {
        error->clear();
    }
    hasProfileCalibration_ = false;
    profileIntrinsics_ = hik_calibration::IntrinsicCalibrationResult();
    profileLaserPlane_ = hik_calibration::LaserPlaneFitResult();
    profileIntrinsicsMetadata_ = hik_calibration::IntrinsicsYamlMetadata();
    profileLaserMetadata_ = hik_calibration::LaserPlaneYamlMetadata();
    profileIntrinsicsPath_ = profile_.intrinsicsConfigPath(sourceDir_);
    profileLaserPlanePath_ = profile_.laserPlaneConfigPath(sourceDir_);

    std::string coreError;
    if (!hik_calibration::loadIntrinsicsYaml(
            toLocalPath(profileIntrinsicsPath_), &profileIntrinsics_,
            &profileIntrinsicsMetadata_, &coreError)) {
        if (error) {
            *error = QStringLiteral("无法加载正式内参 %1: %2")
                .arg(profileIntrinsicsPath_, fromStdString(coreError));
        }
        profileCalibrationStatusLabel_->setText(error ? *error : QStringLiteral("正式内参加载失败"));
        profileCalibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    if (!intrinsicsMetadataMatchesProfile(profileIntrinsicsMetadata_, error)) {
        profileCalibrationStatusLabel_->setText(
            error ? *error : QStringLiteral("正式内参不属于当前设备"));
        profileCalibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    coreError.clear();
    if (!hik_calibration::loadLaserPlaneYaml(
            toLocalPath(profileLaserPlanePath_), &profileLaserPlane_, &profileBoard_,
            &profileLaserMetadata_, &coreError)) {
        if (error) {
            *error = QStringLiteral("无法加载正式激光平面 %1: %2")
                .arg(profileLaserPlanePath_, fromStdString(coreError));
        }
        profileCalibrationStatusLabel_->setText(error ? *error : QStringLiteral("正式激光平面加载失败"));
        profileCalibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    if (QString::fromStdString(profileLaserMetadata_.cameraFrame).trimmed() !=
        profile_.cameraFrame) {
        if (error) {
            *error = QStringLiteral("正式激光平面 frame 不属于 %1。")
                .arg(profile_.id);
        }
        profileCalibrationStatusLabel_->setText(
            error ? *error : QStringLiteral("激光平面 frame 不匹配"));
        profileCalibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    QString hashError;
    profileIntrinsicsSha256_ = fileSha256Hex(profileIntrinsicsPath_, &hashError);
    if (profileIntrinsicsSha256_.isEmpty()) {
        if (error) {
            *error = hashError;
        }
        return false;
    }
    profileLaserPlaneSha256_ = fileSha256Hex(profileLaserPlanePath_, &hashError);
    if (profileLaserPlaneSha256_.isEmpty()) {
        if (error) {
            *error = hashError;
        }
        return false;
    }
    const QString recordedIntrinsicsHash =
        QString::fromStdString(profileLaserMetadata_.intrinsicsSha256).trimmed();
    if (recordedIntrinsicsHash.isEmpty() ||
        recordedIntrinsicsHash.compare(profileIntrinsicsSha256_, Qt::CaseInsensitive) != 0) {
        if (error) {
            *error = QStringLiteral("激光平面绑定的内参 SHA-256 与当前 %1 不一致。请重新标定或批准激光平面。")
                .arg(profile_.intrinsicsConfigRelativePath);
        }
        profileCalibrationStatusLabel_->setText(error ? *error : QStringLiteral("标定哈希不一致"));
        profileCalibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    if (!boardMatches(profileIntrinsics_.board, profileBoard_)) {
        if (error) {
            *error = QStringLiteral("正式内参与激光平面的 ChArUco 板定义不一致。");
        }
        return false;
    }
    if (!std::isfinite(profileLaserMetadata_.validCameraZMinMm) ||
        !std::isfinite(profileLaserMetadata_.validCameraZMaxMm) ||
        profileLaserMetadata_.validCameraZMinMm <= 0.0 ||
        profileLaserMetadata_.validCameraZMaxMm <= profileLaserMetadata_.validCameraZMinMm) {
        if (error) {
            *error = QStringLiteral("激光平面 YAML 缺少有效的相机 Z 深度范围。");
        }
        return false;
    }
    profileOptions_.minimumDepthMm = profileLaserMetadata_.validCameraZMinMm;
    profileOptions_.maximumDepthMm = profileLaserMetadata_.validCameraZMaxMm;
    profileOptions_.stripe = laserPairOptions_.stripe;
    profileOptions_.stripe.quality.roi =
        calibrationStripeRoi(profile_, profileIntrinsics_.imageSize);
    profileOptions_.stripeValidityMask.release();
    if (profileOptions_.stripe.mode !=
            hik_calibration::StripeExtractionMode::Legacy &&
        !hik_calibration::buildLaserPlaneValidityMask(
            profileIntrinsics_.imageSize, profileIntrinsics_,
            profileLaserPlane_, profileOptions_.minimumDepthMm,
            profileOptions_.maximumDepthMm,
            profileOptions_.stripe.quality.roi,
            &profileOptions_.stripeValidityMask, &coreError)) {
        if (error) {
            *error = QStringLiteral("无法建立条纹有效深度走廊: %1")
                .arg(fromStdString(coreError));
        }
        return false;
    }
    profileOptions_.boardDetection = detectionOptions_;
    profileOptions_.boardPose = laserPairOptions_.pose;
    hasProfileCalibration_ = true;
    profileCalibrationStatusLabel_->setText(
        QStringLiteral("已校验：%1×%2，SN=%3，Z有效范围=%4–%5 mm；输出坐标系=%6。")
            .arg(profileIntrinsics_.imageSize.width)
            .arg(profileIntrinsics_.imageSize.height)
            .arg(QString::fromStdString(profileIntrinsicsMetadata_.cameraSerial))
            .arg(profileOptions_.minimumDepthMm, 0, 'f', 2)
            .arg(profileOptions_.maximumDepthMm, 0, 'f', 2)
            .arg(QString::fromStdString(profileLaserMetadata_.cameraFrame)));
    profileCalibrationStatusLabel_->setStyleSheet(QStringLiteral("color:#087f23;"));
    return true;
}

bool HikCalibrationWindow::loadFormalHandEyeIntrinsics(QString* error) {
    if (error) {
        error->clear();
    }
    hasHandEyeIntrinsics_ = false;
    handEyeIntrinsics_ = hik_calibration::IntrinsicCalibrationResult();
    handEyeIntrinsicsMetadata_ = hik_calibration::IntrinsicsYamlMetadata();
    handEyeIntrinsicsPath_ = profile_.intrinsicsConfigPath(sourceDir_);
    std::string coreError;
    if (!hik_calibration::loadIntrinsicsYaml(
            toLocalPath(handEyeIntrinsicsPath_), &handEyeIntrinsics_,
            &handEyeIntrinsicsMetadata_, &coreError)) {
        if (error) {
            *error = QStringLiteral("无法加载正式内参 %1: %2")
                .arg(handEyeIntrinsicsPath_, fromStdString(coreError));
        }
        handEyeIntrinsicsStatusLabel_->setText(
            error ? *error : QStringLiteral("手眼正式内参加载失败"));
        handEyeIntrinsicsStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    if (!intrinsicsMetadataMatchesProfile(handEyeIntrinsicsMetadata_, error)) {
        handEyeIntrinsicsStatusLabel_->setText(
            error ? *error : QStringLiteral("正式内参不属于当前设备"));
        handEyeIntrinsicsStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    QString hashError;
    handEyeIntrinsicsSha256_ = fileSha256Hex(handEyeIntrinsicsPath_, &hashError);
    if (handEyeIntrinsicsSha256_.isEmpty()) {
        if (error) {
            *error = hashError;
        }
        handEyeIntrinsicsStatusLabel_->setText(hashError);
        handEyeIntrinsicsStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    if (boardCanChange()) {
        setBoardSpecUi(handEyeIntrinsics_.board);
    } else if (!boardMatches(boardSpecFromUi(), handEyeIntrinsics_.board)) {
        if (error) {
            *error = QStringLiteral("正式内参的 ChArUco 板定义与当前数据集不一致。");
        }
        handEyeIntrinsicsStatusLabel_->setText(error ? *error : QStringLiteral("标定板不一致"));
        handEyeIntrinsicsStatusLabel_->setStyleSheet(QStringLiteral("color:#b00020;"));
        return false;
    }
    hasHandEyeIntrinsics_ = true;
    handEyeIntrinsicsStatusLabel_->setText(QStringLiteral(
        "正式内参已校验：%1×%2，SN=%3，RMS=%4 px，SHA-256=%5…")
        .arg(handEyeIntrinsics_.imageSize.width)
        .arg(handEyeIntrinsics_.imageSize.height)
        .arg(QString::fromStdString(handEyeIntrinsicsMetadata_.cameraSerial))
        .arg(handEyeIntrinsics_.calibrationRmsPx, 0, 'f', 5)
        .arg(handEyeIntrinsicsSha256_.left(12)));
    handEyeIntrinsicsStatusLabel_->setStyleSheet(QStringLiteral("color:#087f23;"));
    return true;
}

bool HikCalibrationWindow::handEyeCameraIdentityMatches(QString* error) const {
    if (error) {
        error->clear();
    }
    const QString calibratedSerial =
        QString::fromStdString(handEyeIntrinsicsMetadata_.cameraSerial).trimmed();
    if (calibratedSerial.isEmpty() || currentCameraSerial_.isEmpty() ||
        calibratedSerial != currentCameraSerial_) {
        if (error) {
            *error = QStringLiteral("正式内参绑定 SN=%1，当前相机 SN=%2；手眼必须使用同一台相机。")
                .arg(calibratedSerial.isEmpty() ? QStringLiteral("未绑定") : calibratedSerial,
                     currentCameraSerial_.isEmpty() ? QStringLiteral("未知") : currentCameraSerial_);
        }
        return false;
    }
    return true;
}

bool HikCalibrationWindow::profileCameraIdentityMatches(QString* error) const {
    if (error) {
        error->clear();
    }
    const QString calibratedSerial =
        QString::fromStdString(profileIntrinsicsMetadata_.cameraSerial).trimmed();
    if (calibratedSerial.isEmpty() || currentCameraSerial_.isEmpty() ||
        calibratedSerial != currentCameraSerial_) {
        if (error) {
            *error = QStringLiteral("正式内参绑定 SN=%1，当前相机 SN=%2。必须使用同一台相机。")
                .arg(calibratedSerial.isEmpty() ? QStringLiteral("未绑定") : calibratedSerial,
                     currentCameraSerial_.isEmpty() ? QStringLiteral("未知") : currentCameraSerial_);
        }
        return false;
    }
    return true;
}

void HikCalibrationWindow::acceptProfileOffImage(const cv::Mat& grayImage,
                                                  const QString& imagePath,
                                                  const QString& sampleId,
                                                  double actualExposureUs,
                                                  double actualGainDb) {
    QString error;
    if (!hasProfileCalibration_ || !profileCameraIdentityMatches(&error)) {
        showError(QStringLiteral("profile-off 拒绝"),
                  error.isEmpty() ? QStringLiteral("正式标定未加载。") : error);
        resetPendingProfileOff();
        return;
    }
    if (grayImage.size() != profileIntrinsics_.imageSize) {
        showError(QStringLiteral("profile-off 尺寸不匹配"),
                  QStringLiteral("本帧 %1×%2，内参 %3×%4。")
                      .arg(grayImage.cols).arg(grayImage.rows)
                      .arg(profileIntrinsics_.imageSize.width)
                      .arg(profileIntrinsics_.imageSize.height));
        resetPendingProfileOff();
        return;
    }
    pendingProfileOff_.valid = true;
    pendingProfileOff_.sampleId = sampleId;
    pendingProfileOff_.imagePath = imagePath;
    pendingProfileOff_.image = grayImage.clone();
    pendingProfileOff_.exposureUs = actualExposureUs;
    pendingProfileOff_.gainDb = actualGainDb;
    profileCaptureState_ = LaserCaptureState::AwaitingOn;
    imageView_->setImage(cvMatToQImageCopy(grayImage));
    profileResultLabel_->setText(
        QStringLiteral("off 已保存：%1。保持平板/相机不动，打开激光后采 on。")
            .arg(fileNameOnly(imagePath)));
    appendLog(QStringLiteral("静态轮廓 off 通过，等待 on；期间严禁移动。"));
}

void HikCalibrationWindow::completeProfilePair(const cv::Mat& laserOnImage,
                                                const QString& laserOnPath,
                                                double actualExposureUs,
                                                double actualGainDb,
                                                bool acquisitionParametersVerified) {
    if (!pendingProfileOff_.valid || !hasProfileCalibration_) {
        showError(QStringLiteral("静态轮廓拒绝"), QStringLiteral("没有可配对的 off 或正式标定未加载。"));
        return;
    }
    if (acquisitionParametersVerified) {
        const double exposureTolerance = std::max(
            1.0, std::fabs(pendingProfileOff_.exposureUs) * 0.001);
        if (!std::isfinite(actualExposureUs) || !std::isfinite(actualGainDb) ||
            std::fabs(actualExposureUs - pendingProfileOff_.exposureUs) > exposureTolerance ||
            std::fabs(actualGainDb - pendingProfileOff_.gainDb) > 0.01) {
            showError(QStringLiteral("off/on 参数不一致"),
                      QStringLiteral("曝光 %1/%2 us，增益 %3/%4 dB；off 已保留，可恢复参数后重试。")
                          .arg(pendingProfileOff_.exposureUs, 0, 'f', 3)
                          .arg(actualExposureUs, 0, 'f', 3)
                          .arg(pendingProfileOff_.gainDb, 0, 'f', 3)
                          .arg(actualGainDb, 0, 'f', 3));
            return;
        }
    }
    QString hashError;
    if (fileSha256Hex(profileIntrinsicsPath_, &hashError) != profileIntrinsicsSha256_ ||
        fileSha256Hex(profileLaserPlanePath_, &hashError) != profileLaserPlaneSha256_) {
        showError(QStringLiteral("标定文件已变化"),
                  QStringLiteral("采集期间正式内参或激光平面发生变化。请取消本对并重新加载正式标定。"));
        return;
    }

    hik_calibration::StaticProfileResult result;
    bool reconstructed = false;
    try {
        reconstructed = hik_calibration::reconstructStaticProfile(
            pendingProfileOff_.image, laserOnImage,
            pendingProfileOff_.sampleId.toStdString(), profileIntrinsics_,
            profileLaserPlane_, profileBoard_, profileOptions_, &result);
    } catch (const cv::Exception& exception) {
        result.error = std::string("OpenCV exception: ") + exception.what();
    } catch (const std::exception& exception) {
        result.error = std::string("exception: ") + exception.what();
    }
    if (!reconstructed || !result.ok) {
        imageView_->setImage(result.differenceImage.empty()
            ? cvMatToQImageCopy(laserOnImage)
            : cvMatToQImageCopy(result.differenceImage));
        showError(QStringLiteral("静态轮廓重建失败"),
                  fromStdString(result.error) + QStringLiteral("\noff 已保留，可在不移动平板的前提下重试 on。"));
        return;
    }

    const QString plyPath = uniqueFilePath(
        profileSessionDir_, pendingProfileOff_.sampleId, QStringLiteral("ply"));
    const QString csvPath = uniqueFilePath(
        profileSessionDir_, pendingProfileOff_.sampleId, QStringLiteral("csv"));
    std::string coreError;
    const std::string frameId = profileLaserMetadata_.cameraFrame.empty()
        ? profileIntrinsicsMetadata_.frameId : profileLaserMetadata_.cameraFrame;
    if (plyPath.isEmpty() || csvPath.isEmpty() ||
        !hik_calibration::saveStaticProfilePly(
            toLocalPath(plyPath), result, frameId, &coreError) ||
        !hik_calibration::saveStaticProfileCsv(
            toLocalPath(csvPath), result, &coreError)) {
        showError(QStringLiteral("轮廓结果保存失败"), fromStdString(coreError));
        return;
    }

    ProfileSample sample;
    sample.sampleId = pendingProfileOff_.sampleId;
    sample.laserOffPath = pendingProfileOff_.imagePath;
    sample.laserOnPath = laserOnPath;
    sample.plyPath = plyPath;
    sample.csvPath = csvPath;
    sample.acquisitionParametersVerified = acquisitionParametersVerified;
    sample.result = result;
    QString manifestError;
    if (!appendProfileManifest(sample, &manifestError)) {
        QFile::remove(plyPath);
        QFile::remove(csvPath);
        showError(QStringLiteral("静态轮廓清单写入失败"), manifestError);
        return;
    }
    profileSamples_.push_back(sample);
    resetPendingProfileOff();
    refreshProfileTable();
    imageView_->setImage(drawProfileOverlay(laserOnImage, result));
    if (result.boardValidationAvailable) {
        profileResultLabel_->setText(
            QStringLiteral("重建成功：%1 点，Z=%2–%3 mm，直线 RMS=%4 mm；ChArUco 平板 %5 点，RMS=%6 mm，%7。PLY=%8")
                .arg(static_cast<int>(result.points.size()))
                .arg(result.minimumDepthMm, 0, 'f', 3)
                .arg(result.maximumDepthMm, 0, 'f', 3)
                .arg(result.lineDistanceMm.rms, 0, 'f', 4)
                .arg(result.boardValidationPointCount)
                .arg(result.boardPlaneDistanceMm.rms, 0, 'f', 4)
                .arg(result.boardValidationPassed ? QStringLiteral("通过") : QStringLiteral("不通过"))
                .arg(plyPath));
    } else {
        profileResultLabel_->setText(
            QStringLiteral("重建成功：%1 点，Z=%2–%3 mm，直线 RMS=%4 mm。未检测到可验证的 ChArUco 平面，因此该 RMS 仅表示轮廓直线度，不是平板精度。PLY=%5")
                .arg(static_cast<int>(result.points.size()))
                .arg(result.minimumDepthMm, 0, 'f', 3)
                .arg(result.maximumDepthMm, 0, 'f', 3)
                .arg(result.lineDistanceMm.rms, 0, 'f', 4)
                .arg(plyPath));
    }
    appendLog(QStringLiteral("静态三维轮廓 %1 已保存：PLY=%2，CSV=%3")
              .arg(sample.sampleId, plyPath, csvPath));
}

void HikCalibrationWindow::addOfflineProfilePair(const cv::Mat& laserOffImage,
                                                  const QString& laserOffPath,
                                                  const cv::Mat& laserOnImage,
                                                  const QString& laserOnPath,
                                                  const QString& sampleId) {
    pendingProfileOff_.valid = true;
    pendingProfileOff_.sampleId = sampleId;
    pendingProfileOff_.imagePath = laserOffPath;
    pendingProfileOff_.image = laserOffImage.clone();
    pendingProfileOff_.exposureUs = 0.0;
    pendingProfileOff_.gainDb = 0.0;
    profileCaptureState_ = LaserCaptureState::AwaitingOn;
    completeProfilePair(laserOnImage, laserOnPath, 0.0, 0.0, false);
}

void HikCalibrationWindow::resetPendingProfileOff() {
    pendingProfileOff_ = PendingProfileOff();
    pendingProfileCapturePairId_.clear();
    profileCaptureState_ = LaserCaptureState::AwaitingOff;
}

void HikCalibrationWindow::refreshProfileTable() {
    profileTable_->setRowCount(static_cast<int>(profileSamples_.size()));
    for (int row = 0; row < static_cast<int>(profileSamples_.size()); ++row) {
        const ProfileSample& sample = profileSamples_[static_cast<std::size_t>(row)];
        const hik_calibration::StaticProfileResult& result = sample.result;
        const QString planeRms = result.boardValidationAvailable
            ? QString::number(result.boardPlaneDistanceMm.rms, 'f', 4)
            : QStringLiteral("-");
        const QString state = result.boardValidationAvailable
            ? (result.boardValidationPassed ? QStringLiteral("平板通过")
                                            : QStringLiteral("平板不通过"))
            : (result.lineQualityPassed ? QStringLiteral("仅直线度通过")
                                        : QStringLiteral("直线度不通过"));
        const QStringList values = QStringList()
            << sample.sampleId << fileNameOnly(sample.laserOffPath)
            << fileNameOnly(sample.laserOnPath)
            << QString::number(static_cast<int>(result.points.size()))
            << formatNumber(result.minimumDepthMm, 3)
            << formatNumber(result.maximumDepthMm, 3)
            << formatNumber(result.meanConfidence, 3)
            << formatNumber(result.lineDistanceMm.rms, 4)
            << QString::number(result.boardValidationPointCount)
            << planeRms
            << QStringLiteral("%1 | %2").arg(state, fileNameOnly(sample.plyPath));
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            if (column == 10) {
                item->setToolTip(sample.plyPath);
                item->setForeground((result.boardValidationAvailable
                    ? result.boardValidationPassed : result.lineQualityPassed)
                    ? QColor(0, 120, 30) : QColor(180, 0, 0));
            }
            profileTable_->setItem(row, column, item);
        }
    }
}

void HikCalibrationWindow::processHandEyeImage(const cv::Mat& grayImage,
                                                const QString& imagePath,
                                                const QString& sampleId,
                                                qint64 cameraHostTimestampRaw) {
    if (handEyeCaptureState_ != HandEyeCaptureState::WaitingCameraFrame ||
        !pendingHandEyeBeforePose_.valid || sampleId != pendingHandEyeSampleId_) {
        showError(QStringLiteral("手眼采图状态错误"),
                  QStringLiteral("相机帧与法兰前读事务不匹配，样本已拒绝。"));
        resetPendingHandEye();
        return;
    }
    QString identityError;
    if (!hasHandEyeIntrinsics_ || !handEyeCameraIdentityMatches(&identityError)) {
        showError(QStringLiteral("手眼样本拒绝"), identityError);
        resetPendingHandEye();
        return;
    }
    if (grayImage.size() != handEyeIntrinsics_.imageSize) {
        showError(QStringLiteral("手眼图像尺寸不匹配"),
                  QStringLiteral("本帧 %1×%2，正式内参 %3×%4。")
                      .arg(grayImage.cols).arg(grayImage.rows)
                      .arg(handEyeIntrinsics_.imageSize.width)
                      .arg(handEyeIntrinsics_.imageSize.height));
        resetPendingHandEye();
        return;
    }
    hik_calibration::CharucoDetectionResult detection;
    const hik_calibration::BoardSpec board = boardSpecFromUi();
    hik_calibration::detectCharuco(
        grayImage, sampleId.toStdString(), board, detectionOptions_, &detection,
        handEyeIntrinsics_.cameraMatrix, handEyeIntrinsics_.distCoeffs);
    hik_calibration::BoardPoseResult pose;
    const bool poseOk = detection.ok && hik_calibration::estimateBoardPose(
        detection.observation, board, handEyeIntrinsics_.cameraMatrix,
        handEyeIntrinsics_.distCoeffs, handEyeBoardPoseOptions_, &pose);
    imageView_->setImage(drawCharucoOverlay(
        grayImage, detection, poseOk ? QStringLiteral("hand-eye pose OK")
                                     : QStringLiteral("hand-eye REJECT")));
    if (!detection.ok || !poseOk) {
        const QString reason = !detection.ok ? fromStdString(detection.error)
                                             : fromStdString(pose.error);
        appendLog(QStringLiteral("手眼样本 %1 拒绝：%2（原图保留）")
                  .arg(sampleId, reason));
        imageInfoLabel_->setText(QStringLiteral("%1 | 手眼拒绝: %2")
                                 .arg(fileNameOnly(imagePath), reason));
        resetPendingHandEye();
        return;
    }
    pendingHandEyeImage_.valid = true;
    pendingHandEyeImage_.sampleId = sampleId;
    pendingHandEyeImage_.imagePath = imagePath;
    pendingHandEyeImage_.detection = detection;
    pendingHandEyeImage_.boardPose = pose;
    pendingHandEyeImage_.cameraHostTimestampRaw = cameraHostTimestampRaw;
    handEyeCaptureState_ = HandEyeCaptureState::WaitingAfterPose;
    pendingRobotRequestId_ =
        robotSession_->allocateRequestId(robotClientId_);
    appendLog(QStringLiteral("手眼样本 %1：ChArUco 位姿通过，RMS=%2 px；读取采图后法兰。")
              .arg(sampleId).arg(pose.reprojection.rms, 0, 'f', 4));
    emit requestReadFlangePose(pendingRobotRequestId_);
}

void HikCalibrationWindow::finishHandEyeSample(const RobotPoseReading& after) {
    if (handEyeCaptureState_ != HandEyeCaptureState::WaitingAfterPose ||
        !pendingHandEyeBeforePose_.valid || !pendingHandEyeImage_.valid || !after.valid) {
        showError(QStringLiteral("手眼样本状态错误"), QStringLiteral("缺少前/后法兰或板位姿。"));
        resetPendingHandEye();
        return;
    }
    const double translationDelta = hik_calibration::rigidTranslationDistanceMm(
        pendingHandEyeBeforePose_.baseFromFlange, after.baseFromFlange);
    const double rotationDelta = hik_calibration::rigidRotationDistanceDeg(
        pendingHandEyeBeforePose_.baseFromFlange, after.baseFromFlange);
    if (translationDelta > kMaximumRobotStillTranslationMm ||
        rotationDelta > kMaximumRobotStillRotationDeg) {
        const QString reason = QStringLiteral(
            "采图前后法兰变化 %1 mm / %2°，超过 %3 mm / %4°；机器人未稳定或被移动。")
            .arg(translationDelta, 0, 'f', 4).arg(rotationDelta, 0, 'f', 4)
            .arg(kMaximumRobotStillTranslationMm, 0, 'f', 2)
            .arg(kMaximumRobotStillRotationDeg, 0, 'f', 2);
        appendLog(QStringLiteral("手眼样本 %1 拒绝：%2（原图保留）")
                  .arg(pendingHandEyeSampleId_, reason));
        showError(QStringLiteral("机器人静止检查不通过"), reason);
        resetPendingHandEye();
        return;
    }

    HandEyeSampleEntry entry;
    entry.sample.sampleId = pendingHandEyeSampleId_.toStdString();
    entry.sample.baseFromFlange = hik_calibration::interpolateRigidHalf(
        pendingHandEyeBeforePose_.baseFromFlange, after.baseFromFlange);
    entry.sample.cameraFromBoard = pendingHandEyeImage_.boardPose.cameraFromBoard;
    entry.sample.boardPoseRmsPx = pendingHandEyeImage_.boardPose.reprojection.rms;
    entry.imagePath = pendingHandEyeImage_.imagePath;
    for (int index = 0; index < 3; ++index) {
        entry.flangeValues[index] =
            (pendingHandEyeBeforePose_.values[index] + after.values[index]) * 0.5;
    }
    for (int index = 3; index < 6; ++index) {
        entry.flangeValues[index] = averageAngleDeg(
            pendingHandEyeBeforePose_.values[index], after.values[index]);
    }
    entry.cornerCount = pendingHandEyeImage_.detection.observation.quality.cornerCount;
    entry.robotTranslationDeltaMm = translationDelta;
    entry.robotRotationDeltaDeg = rotationDelta;
    QString manifestError;
    if (!appendHandEyeManifest(entry, pendingHandEyeBeforePose_, after, &manifestError)) {
        showError(QStringLiteral("手眼清单写入失败"),
                  manifestError + QStringLiteral("\n原图保留，但样本不会进入求解。"));
        resetPendingHandEye();
        return;
    }
    handEyeSamples_.push_back(entry);
    hasHandEyeResult_ = false;
    handEyeResult_ = hik_calibration::HandEyeCalibrationResult();
    lastHandEyeCandidatePath_.clear();
    const QString acceptedId = pendingHandEyeSampleId_;
    resetPendingHandEye();
    refreshHandEyeTable();
    appendLog(QStringLiteral("手眼样本 %1 已加入：板 RMS=%2 px，静止 Δ=%3 mm/%4°，当前 %5 个。")
              .arg(acceptedId)
              .arg(entry.sample.boardPoseRmsPx, 0, 'f', 4)
              .arg(translationDelta, 0, 'f', 4)
              .arg(rotationDelta, 0, 'f', 4)
              .arg(static_cast<int>(handEyeSamples_.size())));
}

void HikCalibrationWindow::resetPendingHandEye() {
    if (robotSession_ && robotClientId_ > 0) {
        robotSession_->releaseExclusive(robotClientId_);
    }
    handEyeCaptureState_ = HandEyeCaptureState::Idle;
    pendingHandEyeBeforePose_ = RobotPoseReading();
    pendingHandEyeImage_ = PendingHandEyeImage();
    pendingHandEyeSampleId_.clear();
    pendingRobotRequestId_ = -1;
    manualRobotReadPending_ = false;
}

void HikCalibrationWindow::refreshHandEyeTable() {
    handEyeTable_->setRowCount(static_cast<int>(handEyeSamples_.size()));
    for (int row = 0; row < static_cast<int>(handEyeSamples_.size()); ++row) {
        const HandEyeSampleEntry& sample = handEyeSamples_[static_cast<std::size_t>(row)];
        QString state = QStringLiteral("待求解");
        if (hasHandEyeResult_) {
            state = sample.solverAccepted ? QStringLiteral("采用")
                                          : QStringLiteral("剔除: %1").arg(sample.solverReason);
        }
        const QStringList values = QStringList()
            << QString::fromStdString(sample.sample.sampleId)
            << fileNameOnly(sample.imagePath)
            << formatNumber(sample.flangeValues[0], 3)
            << formatNumber(sample.flangeValues[1], 3)
            << formatNumber(sample.flangeValues[2], 3)
            << formatNumber(sample.flangeValues[3], 3)
            << formatNumber(sample.flangeValues[4], 3)
            << formatNumber(sample.flangeValues[5], 3)
            << QString::number(sample.cornerCount)
            << formatNumber(sample.sample.boardPoseRmsPx, 4)
            << formatNumber(sample.robotTranslationDeltaMm, 4)
            << formatNumber(sample.robotRotationDeltaDeg, 4)
            << (hasHandEyeResult_ && sample.solverAccepted
                ? formatNumber(sample.translationResidualMm, 4) : QStringLiteral("-"))
            << state;
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            if (column == 13 && hasHandEyeResult_) {
                item->setForeground(sample.solverAccepted
                    ? QColor(0, 120, 30) : QColor(180, 0, 0));
                item->setToolTip(sample.solverAccepted
                    ? QStringLiteral("固定板旋转残差=%1°")
                        .arg(sample.rotationResidualDeg, 0, 'f', 4)
                    : sample.solverReason);
            }
            handEyeTable_->setItem(row, column, item);
        }
    }
}

void HikCalibrationWindow::deleteSelectedHandEyeSamples() {
    if (!handEyeTable_->selectionModel()) {
        return;
    }
    const QModelIndexList selected = handEyeTable_->selectionModel()->selectedRows();
    std::vector<int> rows;
    for (int index = 0; index < selected.size(); ++index) {
        rows.push_back(selected.at(index).row());
    }
    std::sort(rows.begin(), rows.end(), std::greater<int>());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    for (std::size_t index = 0; index < rows.size(); ++index) {
        if (rows[index] >= 0 && rows[index] < static_cast<int>(handEyeSamples_.size())) {
            handEyeSamples_.erase(handEyeSamples_.begin() + rows[index]);
        }
    }
    if (!rows.empty()) {
        hasHandEyeResult_ = false;
        handEyeResult_ = hik_calibration::HandEyeCalibrationResult();
        lastHandEyeCandidatePath_.clear();
        refreshHandEyeTable();
        appendLog(QStringLiteral("已从手眼数据集移除 %1 个样本；session 原图与清单保留。")
                  .arg(static_cast<int>(rows.size())));
    }
    updateAllUiStates();
}

void HikCalibrationWindow::clearHandEyeSamples() {
    if (handEyeSamples_.empty()) {
        return;
    }
    if (QMessageBox::question(
            this, QStringLiteral("清空手眼样本"),
            QStringLiteral("确定清空当前手眼样本表和求解结果吗？session 原图与清单不会删除。"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    handEyeSamples_.clear();
    hasHandEyeResult_ = false;
    handEyeResult_ = hik_calibration::HandEyeCalibrationResult();
    lastHandEyeCandidatePath_.clear();
    refreshHandEyeTable();
    handEyeResultLabel_->setText(QStringLiteral("尚未求解。"));
    appendLog(QStringLiteral("已清空手眼样本表。"));
    updateAllUiStates();
}

void HikCalibrationWindow::solveHandEye() {
    if (static_cast<int>(handEyeSamples_.size()) < handEyeOptions_.minSamples) {
        showError(QStringLiteral("手眼样本不足"),
                  QStringLiteral("至少需要 %1 个有效样本。")
                      .arg(handEyeOptions_.minSamples));
        return;
    }
    std::vector<hik_calibration::HandEyeSample> samples;
    samples.reserve(handEyeSamples_.size());
    for (std::size_t index = 0; index < handEyeSamples_.size(); ++index) {
        samples.push_back(handEyeSamples_[index].sample);
    }
    appendLog(QStringLiteral("开始手眼求解：%1 个样本，比较 OpenCV 五种方法并做鲁棒离群剔除。")
              .arg(static_cast<int>(samples.size())));
    hik_calibration::HandEyeCalibrationResult result;
    if (!hik_calibration::calibrateHandEyeRobust(samples, handEyeOptions_, &result) ||
        !result.ok) {
        hasHandEyeResult_ = false;
        handEyeResult_ = result;
        handEyeResultLabel_->setText(QStringLiteral("求解失败：%1")
                                     .arg(fromStdString(result.error)));
        refreshHandEyeTable();
        showError(QStringLiteral("手眼求解失败"), fromStdString(result.error));
        updateAllUiStates();
        return;
    }
    handEyeResult_ = result;
    hasHandEyeResult_ = true;
    for (std::size_t index = 0; index < handEyeSamples_.size(); ++index) {
        const hik_calibration::HandEyeSampleResult& sampleResult = result.samples[index];
        HandEyeSampleEntry& entry = handEyeSamples_[index];
        entry.solverAccepted = sampleResult.accepted;
        entry.translationResidualMm = sampleResult.baseBoardTranslationResidualMm;
        entry.rotationResidualDeg = sampleResult.baseBoardRotationResidualDeg;
        entry.solverReason = fromStdString(sampleResult.rejectReason);
    }
    QString qualityReason;
    const bool qualityPassed = handEyeResultPassesQuality(&qualityReason);
    handEyeResultLabel_->setText(QStringLiteral(
        "方法=%1，采用=%2/%3，法兰跨度=%4 mm/%5°，第二旋转轴离散度=%6°；固定板一致性："
        "平移 RMS=%7 mm，P95=%8 mm，Max=%9 mm；旋转 RMS=%10°，P95=%11°，Max=%12°。"
        "质量=%13。\nT_flange_camera（mm）：\n%14")
        .arg(QString::fromStdString(result.method))
        .arg(result.acceptedSampleCount).arg(result.inputSampleCount)
        .arg(result.translationSpanMm, 0, 'f', 2)
        .arg(result.rotationSpanDeg, 0, 'f', 2)
        .arg(result.secondaryRotationSpreadDeg, 0, 'f', 2)
        .arg(result.translationConsistencyMm.rms, 0, 'f', 4)
        .arg(result.translationConsistencyMm.p95, 0, 'f', 4)
        .arg(result.translationConsistencyMm.maximum, 0, 'f', 4)
        .arg(result.rotationConsistencyDeg.rms, 0, 'f', 4)
        .arg(result.rotationConsistencyDeg.p95, 0, 'f', 4)
        .arg(result.rotationConsistencyDeg.maximum, 0, 'f', 4)
        .arg(qualityPassed ? QStringLiteral("通过")
                           : QStringLiteral("不通过：%1").arg(qualityReason))
        .arg(formatTransform(result.flangeFromCamera)));
    refreshHandEyeTable();
    appendLog(QStringLiteral("手眼求解完成：method=%1，采用=%2，剔除=%3，平移 RMS=%4 mm，旋转 RMS=%5°，质量=%6。")
              .arg(QString::fromStdString(result.method))
              .arg(result.acceptedSampleCount).arg(result.rejectedSampleCount)
              .arg(result.translationConsistencyMm.rms, 0, 'f', 4)
              .arg(result.rotationConsistencyDeg.rms, 0, 'f', 4)
              .arg(qualityPassed ? QStringLiteral("通过") : QStringLiteral("不通过")));
    updateAllUiStates();
}

bool HikCalibrationWindow::handEyeResultPassesQuality(QString* reason) const {
    if (reason) {
        reason->clear();
    }
    if (!hasHandEyeResult_ || !handEyeResult_.ok) {
        if (reason) *reason = QStringLiteral("尚无有效求解结果");
        return false;
    }
    if (handEyeResult_.acceptedSampleCount < handEyeOptions_.minSamples) {
        if (reason) *reason = QStringLiteral("采用样本少于 %1").arg(handEyeOptions_.minSamples);
        return false;
    }
    if (!std::isfinite(handEyeResult_.translationConsistencyMm.rms) ||
        handEyeResult_.translationConsistencyMm.rms > kApprovedHandEyeTranslationRmsMm) {
        if (reason) *reason = QStringLiteral("平移 RMS=%1 mm > %2 mm")
            .arg(handEyeResult_.translationConsistencyMm.rms, 0, 'f', 4)
            .arg(kApprovedHandEyeTranslationRmsMm, 0, 'f', 2);
        return false;
    }
    if (!std::isfinite(handEyeResult_.rotationConsistencyDeg.rms) ||
        handEyeResult_.rotationConsistencyDeg.rms > kApprovedHandEyeRotationRmsDeg) {
        if (reason) *reason = QStringLiteral("旋转 RMS=%1° > %2°")
            .arg(handEyeResult_.rotationConsistencyDeg.rms, 0, 'f', 4)
            .arg(kApprovedHandEyeRotationRmsDeg, 0, 'f', 2);
        return false;
    }
    return true;
}

hik_calibration::HandEyeYamlMetadata HikCalibrationWindow::handEyeMetadata() const {
    hik_calibration::HandEyeYamlMetadata metadata;
    metadata.cameraModel = handEyeIntrinsicsMetadata_.cameraModel;
    metadata.cameraSerial = handEyeIntrinsicsMetadata_.cameraSerial;
    metadata.cameraFrame = profile_.cameraFrame.toStdString();
    metadata.flangeFrame = "fairino_flange_reported";
    metadata.baseFrame = "base_link";
    metadata.intrinsicsFile = handEyeIntrinsicsPath_.toStdString();
    metadata.intrinsicsSha256 = handEyeIntrinsicsSha256_.toStdString();
    metadata.datasetManifest = handEyeManifestPath_.toStdString();
    QString hashError;
    metadata.datasetManifestSha256 = fileSha256Hex(handEyeManifestPath_, &hashError).toStdString();
    metadata.generatedAt = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toStdString();
    return metadata;
}

bool HikCalibrationWindow::writeHandEyeYaml(const QString& path, QString* error) const {
    std::string coreError;
    const bool ok = hik_calibration::saveHandEyeYaml(
        toLocalPath(path), handEyeResult_, handEyeMetadata(), &coreError);
    if (!ok && error) {
        *error = fromStdString(coreError);
    }
    return ok;
}

void HikCalibrationWindow::saveHandEyeCandidate() {
    if (!hasHandEyeResult_ || !handEyeResult_.ok) {
        showError(QStringLiteral("无法保存手眼结果"), QStringLiteral("请先成功求解。"));
        return;
    }
    const QString path = uniqueFilePath(
        handEyeSessionDir_, QStringLiteral("hik_handeye_candidate_%1").arg(captureTimestamp()),
        QStringLiteral("yaml"));
    QString error;
    if (!writeHandEyeYaml(path, &error)) {
        showError(QStringLiteral("手眼候选 YAML 保存失败"), error);
        return;
    }
    lastHandEyeCandidatePath_ = path;
    appendLog(QStringLiteral("手眼 session 候选 YAML 已保存: %1").arg(path));
    updateAllUiStates();
}

void HikCalibrationWindow::saveApprovedHandEye() {
    QString qualityReason;
    if (!handEyeResultPassesQuality(&qualityReason)) {
        showError(QStringLiteral("禁止覆盖正式手眼标定"),
                  QStringLiteral("当前结果未通过质量门槛：%1\n正式文件保持不变。")
                      .arg(qualityReason));
        return;
    }
    QString identityError;
    if (!handEyeCameraIdentityMatches(&identityError)) {
        showError(QStringLiteral("禁止覆盖正式手眼标定"), identityError);
        return;
    }
    QString hashError;
    if (fileSha256Hex(handEyeIntrinsicsPath_, &hashError) != handEyeIntrinsicsSha256_) {
        showError(QStringLiteral("禁止覆盖正式手眼标定"),
                  QStringLiteral("正式内参在采集后发生变化，必须重新采集手眼数据。"));
        return;
    }
    const QString approvedPath = uniqueFilePath(
        handEyeSessionDir_, QStringLiteral("hik_handeye_approved_%1").arg(captureTimestamp()),
        QStringLiteral("yaml"));
    QString error;
    if (!writeHandEyeYaml(approvedPath, &error)) {
        showError(QStringLiteral("手眼 YAML 保存失败"), error);
        return;
    }
    const QString formalPath = profile_.handEyeConfigPath(sourceDir_);
    if (!promoteFileAtomically(approvedPath, formalPath, &error)) {
        showError(QStringLiteral("正式手眼标定更新失败"), error);
        return;
    }
    lastHandEyeCandidatePath_ = approvedPath;
    appendLog(QStringLiteral("手眼质量通过，已原子更新正式文件: %1").arg(formalPath));
    QMessageBox::information(this, QStringLiteral("手眼标定已批准"),
        QStringLiteral("已写入：\n%1\n\n方向为 T_flange_camera，平移单位 mm。")
            .arg(formalPath));
}

void HikCalibrationWindow::acceptLaserOffImage(const cv::Mat& grayImage,
                                               const QString& imagePath,
                                               const QString& sampleId,
                                               double actualExposureUs,
                                               double actualGainDb) {
    if (!hasActiveLaserIntrinsics_) {
        showError(QStringLiteral("laser-off 拒绝"), QStringLiteral("激光面内参未加载。"));
        return;
    }
    const QString calibratedSerial =
        QString::fromStdString(activeLaserIntrinsicsMetadata_.cameraSerial).trimmed();
    if (calibratedSerial.isEmpty() || currentCameraSerial_.isEmpty() ||
        calibratedSerial != currentCameraSerial_) {
        const QString reason = QStringLiteral(
            "内参相机 SN=%1，当前相机 SN=%2；必须使用同一台已绑定序列号的相机。")
            .arg(calibratedSerial.isEmpty() ? QStringLiteral("未绑定") : calibratedSerial,
                 currentCameraSerial_.isEmpty() ? QStringLiteral("未知") : currentCameraSerial_);
        imageView_->setImage(cvMatToQImageCopy(grayImage));
        imageInfoLabel_->setText(QStringLiteral("%1 | laser-off 拒绝: %2")
                                 .arg(fileNameOnly(imagePath), reason));
        appendLog(reason);
        resetPendingLaserOff();
        return;
    }
    if (grayImage.size() != activeLaserIntrinsics_.imageSize) {
        const QString reason = QStringLiteral("图像尺寸 %1×%2 与内参标定尺寸 %3×%4 不一致。")
            .arg(grayImage.cols).arg(grayImage.rows)
            .arg(activeLaserIntrinsics_.imageSize.width)
            .arg(activeLaserIntrinsics_.imageSize.height);
        imageView_->setImage(cvMatToQImageCopy(grayImage));
        imageInfoLabel_->setText(QStringLiteral("%1 | laser-off 拒绝: %2")
                                 .arg(fileNameOnly(imagePath), reason));
        appendLog(QStringLiteral("laser-off 原图已保存但尺寸不匹配: %1").arg(reason));
        resetPendingLaserOff();
        return;
    }
    const hik_calibration::BoardSpec board = boardSpecFromUi();
    hik_calibration::CharucoDetectionResult detection;
    hik_calibration::detectCharuco(
        grayImage, sampleId.toStdString() + "_off", board,
        laserPairOptions_.detection, &detection,
        activeLaserIntrinsics_.cameraMatrix, activeLaserIntrinsics_.distCoeffs);
    hik_calibration::BoardPoseResult pose;
    const bool poseOk = detection.ok && hik_calibration::estimateBoardPose(
        detection.observation, board, activeLaserIntrinsics_.cameraMatrix,
        activeLaserIntrinsics_.distCoeffs, laserPairOptions_.pose, &pose);
    double approximateBoardDepthMm = std::numeric_limits<double>::quiet_NaN();
    int approximateDepthBin = -1;
    if (poseOk) {
        const cv::Vec3d boardCenterInCamera =
            pose.rotation *
                cv::Vec3d(board.widthMm() * 0.5, board.heightMm() * 0.5, 0.0) +
            pose.tvec;
        approximateBoardDepthMm = boardCenterInCamera[2];
        approximateDepthBin = laserGuidanceBinIndex(approximateBoardDepthMm);
    }
    QString offOverlayCaption = QStringLiteral("laser-off REJECT");
    if (poseOk) {
        offOverlayCaption = approximateDepthBin >= 0
            ? QStringLiteral("laser-off OK: board Z=%1 mm, approx bin=%2 mm")
                .arg(approximateBoardDepthMm, 0, 'f', 1)
                .arg(kLaserGuidanceBins[static_cast<std::size_t>(
                    approximateDepthBin)].targetDepthMm, 0, 'f', 0)
            : QStringLiteral("laser-off OK: board Z=%1 mm, OUTSIDE guide")
                .arg(approximateBoardDepthMm, 0, 'f', 1);
    }
    imageView_->setImage(drawCharucoOverlay(
        grayImage, detection, offOverlayCaption));
    if (!detection.ok || !poseOk) {
        const QString reason = !detection.ok ? fromStdString(detection.error)
                                             : fromStdString(pose.error);
        imageInfoLabel_->setText(QStringLiteral("%1 | laser-off 拒绝: %2")
                                 .arg(fileNameOnly(imagePath), reason));
        appendLog(QStringLiteral("laser-off 原图已保存但质量不通过: %1").arg(reason));
        resetPendingLaserOff();
        return;
    }
    pendingLaserOff_.valid = true;
    pendingLaserOff_.sampleId = sampleId;
    pendingLaserOff_.imagePath = imagePath;
    pendingLaserOff_.image = grayImage.clone();
    pendingLaserOff_.exposureUs = actualExposureUs;
    pendingLaserOff_.gainDb = actualGainDb;
    pendingLaserOff_.detection = detection;
    laserCaptureState_ = LaserCaptureState::AwaitingOn;
    const QString approximateGuide = approximateDepthBin >= 0
        ? QStringLiteral("板中心 Z≈%1 mm，暂归 %2 mm 档")
            .arg(approximateBoardDepthMm, 0, 'f', 1)
            .arg(kLaserGuidanceBins[static_cast<std::size_t>(
                approximateDepthBin)].targetDepthMm, 0, 'f', 0)
        : QStringLiteral("板中心 Z≈%1 mm，超出 260–1040 mm 引导范围；"
                         "如非预期请取消当前配对并重新摆板")
            .arg(approximateBoardDepthMm, 0, 'f', 1);
    imageInfoLabel_->setText(
        QStringLiteral("%1 | laser-off 板位姿通过，RMS=%2 px；%3。"
                       "最终档位以 laser-on 后激光3D点中位Z为准；现在请保持板不动并开启激光。")
            .arg(fileNameOnly(imagePath))
            .arg(pose.reprojection.rms, 0, 'f', 4)
            .arg(approximateGuide));
    appendLog(QStringLiteral("laser-off 通过：%1；等待 laser-on，期间严禁移动板/相机。")
              .arg(approximateGuide));
}

void HikCalibrationWindow::completeLaserPair(const cv::Mat& laserOnImage,
                                             const QString& laserOnPath,
                                             double actualExposureUs,
                                             double actualGainDb) {
    if (!pendingLaserOff_.valid || !hasActiveLaserIntrinsics_) {
        showError(QStringLiteral("laser-on 拒绝"), QStringLiteral("没有可配对的 laser-off 或内参。"));
        return;
    }
    const QString calibratedSerial =
        QString::fromStdString(activeLaserIntrinsicsMetadata_.cameraSerial).trimmed();
    if (calibratedSerial.isEmpty() || currentCameraSerial_.isEmpty() ||
        calibratedSerial != currentCameraSerial_) {
        showError(QStringLiteral("laser-on 相机身份不一致"),
                  QStringLiteral("内参相机 SN=%1，当前相机 SN=%2。laser-off 已保留，请换回正确相机后重试。")
                      .arg(calibratedSerial.isEmpty() ? QStringLiteral("未绑定") : calibratedSerial,
                           currentCameraSerial_.isEmpty() ? QStringLiteral("未知") : currentCameraSerial_));
        return;
    }
    if (laserOnImage.size() != activeLaserIntrinsics_.imageSize ||
        laserOnImage.size() != pendingLaserOff_.image.size()) {
        showError(QStringLiteral("laser-on 尺寸不匹配"),
                  QStringLiteral("本帧 %1×%2，laser-off %3×%4，内参 %5×%6。\n"
                                 "laser-off 仍保留，恢复正确尺寸后可重试。")
                      .arg(laserOnImage.cols).arg(laserOnImage.rows)
                      .arg(pendingLaserOff_.image.cols).arg(pendingLaserOff_.image.rows)
                      .arg(activeLaserIntrinsics_.imageSize.width)
                      .arg(activeLaserIntrinsics_.imageSize.height));
        return;
    }
    const double exposureToleranceUs = std::max(
        1.0, std::fabs(pendingLaserOff_.exposureUs) * 0.001);
    const double gainToleranceDb = 0.01;
    if (!std::isfinite(actualExposureUs) || !std::isfinite(actualGainDb) ||
        std::fabs(actualExposureUs - pendingLaserOff_.exposureUs) > exposureToleranceUs ||
        std::fabs(actualGainDb - pendingLaserOff_.gainDb) > gainToleranceDb) {
        showError(QStringLiteral("laser-on 参数不一致"),
                  QStringLiteral("laser-off/on 实际曝光为 %1/%2 us，增益为 %3/%4 dB。\n"
                                 "本次 laser-on 已拒绝，laser-off 保留，可恢复参数后重试。")
                      .arg(pendingLaserOff_.exposureUs, 0, 'f', 3)
                      .arg(actualExposureUs, 0, 'f', 3)
                      .arg(pendingLaserOff_.gainDb, 0, 'f', 3)
                      .arg(actualGainDb, 0, 'f', 3));
        return;
    }
    LaserPairSample pair;
    pair.sampleId = pendingLaserOff_.sampleId;
    pair.laserOffPath = pendingLaserOff_.imagePath;
    pair.laserOnPath = laserOnPath;
    pair.acquisitionParametersVerified = true;
    laserPairOptions_.stripe.quality.roi =
        calibrationStripeRoi(profile_, laserOnImage.size());
    hik_calibration::processLaserCalibrationPair(
        pendingLaserOff_.image, laserOnImage, pair.sampleId.toStdString(),
        boardSpecFromUi(), activeLaserIntrinsics_.cameraMatrix,
        activeLaserIntrinsics_.distCoeffs, laserPairOptions_, &pair.result);
    laserPairs_.push_back(pair);
    resetPendingLaserOff();
    invalidateLaserSolution();
    refreshLaserTable();
    imageView_->setImage(drawLaserOverlay(laserOnImage, pair.result));
    QString state = pair.result.ok ? QStringLiteral("通过")
                                   : QStringLiteral("拒绝: %1").arg(fromStdString(pair.result.error));
    if (pair.result.ok &&
        pair.result.laserOnDetection.observation.quality.saturationRatio >
            laserPairOptions_.detection.maxSaturationRatio) {
        state += QStringLiteral("（laser-on 饱和=%1%，按激光专用≤%2%门槛接受）")
            .arg(pair.result.laserOnDetection.observation.quality.saturationRatio * 100.0,
                 0, 'f', 2)
            .arg(laserPairOptions_.maxLaserOnSaturationRatio * 100.0, 0, 'f', 1);
    }
    imageInfoLabel_->setText(QStringLiteral("%1 + %2 | %3")
        .arg(fileNameOnly(pair.laserOffPath), fileNameOnly(pair.laserOnPath), state));
    appendLog(QStringLiteral("激光配对 %1: %2").arg(pair.sampleId, state));
}

void HikCalibrationWindow::addOfflineLaserPair(const cv::Mat& laserOffImage,
                                               const QString& laserOffPath,
                                               const cv::Mat& laserOnImage,
                                               const QString& laserOnPath,
                                               const QString& sampleId) {
    LaserPairSample pair;
    pair.sampleId = sampleId;
    pair.laserOffPath = laserOffPath;
    pair.laserOnPath = laserOnPath;
    if (laserOffImage.size() != activeLaserIntrinsics_.imageSize ||
        laserOnImage.size() != activeLaserIntrinsics_.imageSize) {
        pair.result.sampleId = sampleId.toStdString();
        pair.result.error = "offline pair image size differs from intrinsic calibration size";
    } else {
        laserPairOptions_.stripe.quality.roi =
            calibrationStripeRoi(profile_, laserOnImage.size());
        hik_calibration::processLaserCalibrationPair(
            laserOffImage, laserOnImage, sampleId.toStdString(), boardSpecFromUi(),
            activeLaserIntrinsics_.cameraMatrix, activeLaserIntrinsics_.distCoeffs,
            laserPairOptions_, &pair.result);
    }
    laserPairs_.push_back(pair);
    invalidateLaserSolution();
    imageView_->setImage(drawLaserOverlay(laserOnImage, pair.result));
    appendLog(QStringLiteral("离线激光配对 %1: %2")
        .arg(sampleId, pair.result.ok ? QStringLiteral("通过")
                                     : QStringLiteral("拒绝: %1").arg(fromStdString(pair.result.error))));
}

void HikCalibrationWindow::importLaserPairs() {
    if (!hasActiveLaserIntrinsics_) {
        showError(QStringLiteral("无法导入"), QStringLiteral("请先加载激光面所用内参。"));
        return;
    }
    if (!boardMatches(boardSpecFromUi(), activeLaserIntrinsics_.board)) {
        showError(QStringLiteral("无法导入"), QStringLiteral("当前板参数与内参 YAML 不匹配。"));
        return;
    }
    QStringList offFiles = QFileDialog::getOpenFileNames(
        this, QStringLiteral("第一步：选择 laser-off 图像"), sourceDir_,
        QStringLiteral("图像 (*.png *.bmp *.jpg *.jpeg *.tif *.tiff);;所有文件 (*)"));
    if (offFiles.isEmpty()) {
        return;
    }
    QStringList onFiles = QFileDialog::getOpenFileNames(
        this, QStringLiteral("第二步：选择与上一列同序的 laser-on 图像"), sourceDir_,
        QStringLiteral("图像 (*.png *.bmp *.jpg *.jpeg *.tif *.tiff);;所有文件 (*)"));
    if (onFiles.isEmpty()) {
        return;
    }
    offFiles.sort();
    onFiles.sort();
    if (offFiles.size() != onFiles.size()) {
        showError(QStringLiteral("配对数量不一致"),
                  QStringLiteral("laser-off=%1，laser-on=%2。请按文件名排序后一一对应。")
                      .arg(offFiles.size()).arg(onFiles.size()));
        return;
    }
    int added = 0;
    for (int index = 0; index < offFiles.size(); ++index) {
        const QString id = QStringLiteral("laser_import_%1_%2")
            .arg(captureTimestamp()).arg(index + 1, 3, 10, QLatin1Char('0'));
        cv::Mat offImage;
        cv::Mat onImage;
        QString offPath;
        QString onPath;
        QString error;
        if (!copyImportedImageToSession(offFiles.at(index), laserSessionDir_,
                                        id + QStringLiteral("_off"), &offImage,
                                        &offPath, &error)) {
            appendLog(QStringLiteral("配对 %1 跳过: %2").arg(index + 1).arg(error));
            continue;
        }
        if (!copyImportedImageToSession(onFiles.at(index), laserSessionDir_,
                                        id + QStringLiteral("_on"), &onImage,
                                        &onPath, &error)) {
            appendLog(QStringLiteral("配对 %1 跳过: %2（off 原图已保留）")
                      .arg(index + 1).arg(error));
            continue;
        }
        try {
            addOfflineLaserPair(offImage, offPath, onImage, onPath, id);
            ++added;
        } catch (const cv::Exception& exception) {
            appendLog(QStringLiteral("配对 %1 处理异常: %2")
                      .arg(index + 1).arg(QString::fromUtf8(exception.what())));
        } catch (const std::exception& exception) {
            appendLog(QStringLiteral("配对 %1 处理异常: %2")
                      .arg(index + 1).arg(QString::fromUtf8(exception.what())));
        }
    }
    refreshLaserTable();
    appendLog(QStringLiteral("离线激光配对导入完成：%1 / %2。")
              .arg(added).arg(offFiles.size()));
    updateAllUiStates();
}

void HikCalibrationWindow::deleteSelectedLaserPairs() {
    if (!laserTable_->selectionModel()) {
        return;
    }
    const QModelIndexList selected = laserTable_->selectionModel()->selectedRows();
    std::vector<int> rows;
    for (int index = 0; index < selected.size(); ++index) {
        rows.push_back(selected.at(index).row());
    }
    std::sort(rows.begin(), rows.end(), std::greater<int>());
    rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
    for (std::size_t index = 0; index < rows.size(); ++index) {
        const int row = rows[index];
        if (row >= 0 && row < static_cast<int>(laserPairs_.size())) {
            laserPairs_.erase(laserPairs_.begin() + row);
        }
    }
    if (!rows.empty()) {
        invalidateLaserSolution();
        refreshLaserTable();
        appendLog(QStringLiteral("已从激光数据集移除 %1 对；原图仍保留。")
                  .arg(static_cast<int>(rows.size())));
    }
    updateAllUiStates();
}

void HikCalibrationWindow::clearLaserPairs() {
    if (laserPairs_.empty() && !pendingLaserOff_.valid) {
        return;
    }
    if (QMessageBox::question(
            this, QStringLiteral("清空激光样本"),
            QStringLiteral("确定清空激光配对表、待配对 laser-off 和当前平面结果吗？\n"
                           "session 原图不会删除。"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
        return;
    }
    laserPairs_.clear();
    resetPendingLaserOff();
    invalidateLaserSolution();
    refreshLaserTable();
    appendLog(QStringLiteral("已清空激光配对表。"));
    updateAllUiStates();
}

void HikCalibrationWindow::resetPendingLaserOff() {
    pendingLaserOff_ = PendingLaserOff();
    pendingLaserCapturePairId_.clear();
    laserCaptureState_ = LaserCaptureState::AwaitingOff;
}

void HikCalibrationWindow::refreshLaserTable() {
    laserTable_->setRowCount(static_cast<int>(laserPairs_.size()));
    const hik_calibration::BoardSpec guidanceBoard = hasActiveLaserIntrinsics_
        ? activeLaserIntrinsics_.board : boardSpecFromUi();
    for (int row = 0; row < static_cast<int>(laserPairs_.size()); ++row) {
        const LaserPairSample& sample = laserPairs_[static_cast<std::size_t>(row)];
        const hik_calibration::LaserCalibrationPairResult& result = sample.result;
        const LaserGuidanceObservation observation =
            laserGuidanceObservation(result, guidanceBoard);
        QString depthBin = QStringLiteral("-");
        if (observation.valid) {
            depthBin = observation.binIndex >= 0
                ? QStringLiteral("%1 mm").arg(
                    kLaserGuidanceBins[static_cast<std::size_t>(
                        observation.binIndex)].targetDepthMm, 0, 'f', 0)
                : QStringLiteral("范围外");
        }
        QString state = result.ok ? QStringLiteral("通过")
                                  : QStringLiteral("拒绝: %1").arg(fromStdString(result.error));
        if (result.ok &&
            result.laserOnDetection.observation.quality.saturationRatio >
                laserPairOptions_.detection.maxSaturationRatio) {
            state = QStringLiteral("通过（on饱和%1%，激光专用门槛）")
                .arg(result.laserOnDetection.observation.quality.saturationRatio * 100.0,
                     0, 'f', 2);
        }
        const QStringList values = QStringList()
            << sample.sampleId << fileNameOnly(sample.laserOffPath) << fileNameOnly(sample.laserOnPath)
            << depthBin
            << formatNumber(observation.medianDepthMm, 1)
            << (observation.valid
                ? QStringLiteral("%1 / %2")
                    .arg(observation.boardXmm, 0, 'f', 1)
                    .arg(observation.boardYmm, 0, 'f', 1)
                : QStringLiteral("-"))
            << (observation.valid
                ? QStringLiteral("%1 / %2")
                    .arg(observation.tiltXDeg, 0, 'f', 1)
                    .arg(observation.tiltYDeg, 0, 'f', 1)
                : QStringLiteral("-"))
            << (observation.valid
                ? QStringLiteral("%1 / %2")
                    .arg(observation.stripeUMedianPx, 0, 'f', 1)
                    .arg(observation.stripeVMedianPx, 0, 'f', 1)
                : QStringLiteral("-"))
            << QString::number(result.motion.commonCornerCount)
            << formatNumber(result.motion.cornerShiftPx.mean, 3)
            << formatNumber(result.motion.cornerShiftPx.p95, 3)
            << formatNumber(result.motion.poseTranslationDeltaMm, 3)
            << formatNumber(result.motion.poseRotationDeltaDeg, 3)
            << QString::number(static_cast<int>(result.stripe.size()))
            << QString::number(static_cast<int>(result.cameraPointsMm.size()))
            << state;
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            if (column == 1) {
                item->setToolTip(sample.laserOffPath);
            } else if (column == 2) {
                item->setToolTip(sample.laserOnPath);
            } else if (column == 15) {
                item->setToolTip(values.at(column));
            }
            laserTable_->setItem(row, column, item);
        }
    }
    refreshLaserGuidance();
}

void HikCalibrationWindow::refreshLaserGuidance() {
    if (!laserGuidanceTable_ || !laserGuidanceSummaryLabel_) {
        return;
    }
    std::vector<const hik_calibration::LaserCalibrationPairResult*> pairResults;
    pairResults.reserve(laserPairs_.size());
    for (std::size_t index = 0; index < laserPairs_.size(); ++index) {
        pairResults.push_back(&laserPairs_[index].result);
    }
    const LaserGuidanceStatistics statistics =
        summarizeLaserGuidance(
            pairResults, hasActiveLaserIntrinsics_
                ? activeLaserIntrinsics_.board : boardSpecFromUi());
    const cv::Size imageSize = hasActiveLaserIntrinsics_
        ? activeLaserIntrinsics_.imageSize : cv::Size();

    int targetTotal = 0;
    int inRangeCount = 0;
    int completedBins = 0;
    int nextBinIndex = -1;
    double nextCompletionRatio = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < kLaserGuidanceBins.size(); ++index) {
        const LaserGuidanceBinSpec& spec = kLaserGuidanceBins[index];
        const LaserGuidanceBinStatistics& bin = statistics.bins[index];
        const QString issue = laserGuidanceBinIssue(index, bin, imageSize);
        targetTotal += spec.targetCount;
        inRangeCount += bin.count;
        if (issue.isEmpty()) {
            ++completedBins;
        } else {
            const double completionRatio = std::min(
                1.0, static_cast<double>(bin.count) /
                     static_cast<double>(spec.targetCount));
            if (nextBinIndex < 0 || completionRatio < nextCompletionRatio) {
                nextBinIndex = static_cast<int>(index);
                nextCompletionRatio = completionRatio;
            }
        }

        const double xSpan = bin.count > 0
            ? bin.maximumXmm - bin.minimumXmm : 0.0;
        const double ySpan = bin.count > 0
            ? bin.maximumYmm - bin.minimumYmm : 0.0;
        const double uSpan = bin.count > 0
            ? bin.maximumStripeUPx - bin.minimumStripeUPx : 0.0;
        const double vSpan = bin.count > 0
            ? bin.maximumStripeVPx - bin.minimumStripeVPx : 0.0;
        const QString rangeText = index + 1U == kLaserGuidanceBins.size()
            ? QStringLiteral("[%1, %2]")
                .arg(spec.minimumDepthMm, 0, 'f', 0)
                .arg(spec.maximumDepthMm, 0, 'f', 0)
            : QStringLiteral("[%1, %2)")
                .arg(spec.minimumDepthMm, 0, 'f', 0)
                .arg(spec.maximumDepthMm, 0, 'f', 0);
        const QStringList values = QStringList()
            << QStringLiteral("%1 mm").arg(spec.targetDepthMm, 0, 'f', 0)
            << rangeText
            << QStringLiteral("%1 / %2").arg(bin.count).arg(spec.targetCount)
            << (bin.count > 0 ? QStringLiteral("%1 mm").arg(xSpan, 0, 'f', 1)
                              : QStringLiteral("-"))
            << (bin.count > 0 ? QStringLiteral("%1 mm").arg(ySpan, 0, 'f', 1)
                              : QStringLiteral("-"))
            << (bin.count > 0
                ? QStringLiteral("[%1,%2] / [%3,%4]°")
                    .arg(bin.minimumTiltXDeg, 0, 'f', 1)
                    .arg(bin.maximumTiltXDeg, 0, 'f', 1)
                    .arg(bin.minimumTiltYDeg, 0, 'f', 1)
                    .arg(bin.maximumTiltYDeg, 0, 'f', 1)
                : QStringLiteral("-"))
            << (bin.count > 0
                ? QStringLiteral("%1 / %2 px")
                    .arg(uSpan, 0, 'f', 0).arg(vSpan, 0, 'f', 0)
                : QStringLiteral("-"))
            << (issue.isEmpty() ? QStringLiteral("完成") : issue);
        for (int column = 0; column < values.size(); ++column) {
            QTableWidgetItem* item = new QTableWidgetItem(values.at(column));
            if (column == 5) {
                item->setToolTip(QStringLiteral(
                    "板法向相对相机 Z 轴在 X/Y 方向的有符号偏角范围；每档要求正负方向均超过 2°。"));
            } else if (column == 6) {
                item->setToolTip(QStringLiteral(
                    "各样本激光中心像素中位数的 U/V 跨度；两者至少一个达到图像尺寸的 6%。"));
            }
            laserGuidanceTable_->setItem(static_cast<int>(index), column, item);
        }
    }

    QString nextSuggestion;
    if (nextBinIndex >= 0) {
        const std::size_t index = static_cast<std::size_t>(nextBinIndex);
        const LaserGuidanceBinSpec& spec = kLaserGuidanceBins[index];
        const LaserGuidanceBinStatistics& bin = statistics.bins[index];
        const QString issue = laserGuidanceBinIssue(index, bin, imageSize);
        if (bin.count < spec.targetCount) {
            static const std::array<QString, 5> variations = {{
                QStringLiteral("板居中，使用X/Y组合小倾角"),
                QStringLiteral("板向画面左侧，使用X负/Y正倾角"),
                QStringLiteral("板向画面右侧，使用X正/Y负倾角"),
                QStringLiteral("板向画面上侧，改变激光线落点"),
                QStringLiteral("板向画面下侧，使用与前几组相反的组合倾角")
            }};
            nextSuggestion = QStringLiteral(
                "下一组：把激光交点深度放到约 %1 mm；%2。")
                .arg(spec.targetDepthMm, 0, 'f', 0)
                .arg(variations[static_cast<std::size_t>(bin.count) %
                                variations.size()]);
        } else {
            nextSuggestion = QStringLiteral(
                "下一组仍在约 %1 mm，%2。")
                .arg(spec.targetDepthMm, 0, 'f', 0).arg(issue);
        }
    } else {
        nextSuggestion = QStringLiteral("六个深度档的数量和姿态多样性均已完成，可以求解并检查残差。");
    }

    QString latestText;
    for (std::vector<LaserPairSample>::const_reverse_iterator iterator =
             laserPairs_.rbegin(); iterator != laserPairs_.rend(); ++iterator) {
        const LaserGuidanceObservation observation =
            laserGuidanceObservation(
                iterator->result, hasActiveLaserIntrinsics_
                    ? activeLaserIntrinsics_.board : boardSpecFromUi());
        if (!observation.valid) {
            continue;
        }
        const QString binText = observation.binIndex >= 0
            ? QStringLiteral("%1 mm档").arg(
                kLaserGuidanceBins[static_cast<std::size_t>(
                    observation.binIndex)].targetDepthMm, 0, 'f', 0)
            : QStringLiteral("范围外");
        latestText = QStringLiteral(
            "最近有效 %1：Z=%2 mm（%3），板XY=%4/%5 mm，法向偏角=%6/%7°，条纹UV=%8/%9 px。")
            .arg(iterator->sampleId)
            .arg(observation.medianDepthMm, 0, 'f', 1)
            .arg(binText)
            .arg(observation.boardXmm, 0, 'f', 1)
            .arg(observation.boardYmm, 0, 'f', 1)
            .arg(observation.tiltXDeg, 0, 'f', 1)
            .arg(observation.tiltYDeg, 0, 'f', 1)
            .arg(observation.stripeUMedianPx, 0, 'f', 1)
            .arg(observation.stripeVMedianPx, 0, 'f', 1);
        break;
    }
    if (latestText.isEmpty()) {
        latestText = QStringLiteral(
            "尚无有效配对；深度按每组有效激光3D点的中位相机Z自动分档。");
    }
    const QString outsideWarning = statistics.outsideDepthCount > 0
        ? QStringLiteral(" 范围外有效组仍会参加当前拟合并扩大 YAML 有效 Z；"
                         "正式保存前请删除或在正确深度重拍。")
        : QString();
    laserGuidanceSummaryLabel_->setText(
        QStringLiteral("完成 %1 / 6 档；范围内有效 %2 / 目标 %3，范围外 %4。%5%6 %7")
            .arg(completedBins).arg(inRangeCount).arg(targetTotal)
            .arg(statistics.outsideDepthCount)
            .arg(outsideWarning, nextSuggestion, latestText));
    laserGuidanceSummaryLabel_->setStyleSheet(
        completedBins == static_cast<int>(kLaserGuidanceBins.size())
            ? QStringLiteral("color:#087f23;font-weight:bold;")
            : QStringLiteral("color:#8a4b00;"));
}

int HikCalibrationWindow::acceptedLaserPairCount() const {
    return static_cast<int>(std::count_if(
        laserPairs_.begin(), laserPairs_.end(),
        [](const LaserPairSample& pair) { return pair.result.ok; }));
}

bool HikCalibrationWindow::laserGuidancePasses(QString* reason) const {
    if (reason) {
        reason->clear();
    }
    std::vector<const hik_calibration::LaserCalibrationPairResult*> pairResults;
    pairResults.reserve(laserPairs_.size());
    for (std::size_t index = 0; index < laserPairs_.size(); ++index) {
        pairResults.push_back(&laserPairs_[index].result);
    }
    const LaserGuidanceStatistics statistics =
        summarizeLaserGuidance(
            pairResults, hasActiveLaserIntrinsics_
                ? activeLaserIntrinsics_.board : boardSpecFromUi());
    const cv::Size imageSize = hasActiveLaserIntrinsics_
        ? activeLaserIntrinsics_.imageSize : cv::Size();
    if (statistics.outsideDepthCount > 0) {
        if (reason) {
            *reason = QStringLiteral(
                "存在 %1 个中位 Z 超出 260–1040 mm 引导范围的有效配对；"
                "这些配对会参与拟合并扩大 YAML 有效 Z，正式保存前请删除或重拍。")
                .arg(statistics.outsideDepthCount);
        }
        return false;
    }
    for (std::size_t index = 0; index < kLaserGuidanceBins.size(); ++index) {
        const QString issue =
            laserGuidanceBinIssue(index, statistics.bins[index], imageSize);
        if (!issue.isEmpty()) {
            if (reason) {
                *reason = QStringLiteral(
                    "%1 mm 深度档未完成：%2。每档还要求板中心 X 至少变化 %3 mm、"
                    "Y 至少变化 %4 mm、法向 X/Y 均覆盖 ±%5°，"
                    "且条纹位置跨度至少达到图像宽或高的 %6%。")
                    .arg(kLaserGuidanceBins[index].targetDepthMm, 0, 'f', 0)
                    .arg(issue)
                    .arg(kLaserGuidanceMinimumXSpanMm, 0, 'f', 0)
                    .arg(kLaserGuidanceMinimumYSpanMm, 0, 'f', 0)
                    .arg(kLaserGuidanceTiltSignThresholdDeg, 0, 'f', 0)
                    .arg(kLaserGuidanceMinimumStripeSpanFraction * 100.0, 0, 'f', 0);
            }
            return false;
        }
    }
    return true;
}

void HikCalibrationWindow::invalidateLaserSolution() {
    laserPlaneResult_ = hik_calibration::LaserPlaneFitResult();
    hasLaserPlaneResult_ = false;
    lastLaserCandidatePath_.clear();
    if (laserResultLabel_) {
        laserResultLabel_->setText(QStringLiteral("数据集或内参已变更，需重新求解。"));
    }
}

void HikCalibrationWindow::solveLaserPlane() {
    if (!hasActiveLaserIntrinsics_) {
        showError(QStringLiteral("无法求解"), QStringLiteral("请先加载内参。"));
        return;
    }
    if (acceptedLaserPairCount() < planeOptions_.minPoseCount) {
        showError(QStringLiteral("激光数据不足"),
                  QStringLiteral("至少需要 %1 个通过质量检查的不同板姿态。")
                      .arg(planeOptions_.minPoseCount));
        return;
    }
    planeOptions_.ransacThresholdMm = ransacThresholdSpin_->value();
    std::vector<hik_calibration::LaserPlaneSample> samples;
    for (std::size_t index = 0; index < laserPairs_.size(); ++index) {
        if (laserPairs_[index].result.ok) {
            samples.push_back(hik_calibration::planeSampleFromPair(laserPairs_[index].result));
        }
    }
    appendLog(QStringLiteral("开始激光平面求解：%1 个姿态，RANSAC=%2 mm。")
              .arg(static_cast<int>(samples.size()))
              .arg(planeOptions_.ransacThresholdMm, 0, 'f', 4));
    hik_calibration::LaserPlaneFitResult result;
    bool solved = false;
    try {
        solved = hik_calibration::fitLaserPlane(samples, planeOptions_, &result);
    } catch (const cv::Exception& exception) {
        result.error = std::string("OpenCV exception: ") + exception.what();
    } catch (const std::exception& exception) {
        result.error = std::string("exception: ") + exception.what();
    } catch (...) {
        result.error = "unknown exception";
    }
    laserPlaneResult_ = result;
    hasLaserPlaneResult_ = solved && result.ok;
    if (!hasLaserPlaneResult_) {
        laserResultLabel_->setText(QStringLiteral("激光平面求解失败: %1")
                                   .arg(fromStdString(result.error)));
        appendLog(laserResultLabel_->text());
        if (!result.poses.empty()) {
            std::vector<const hik_calibration::PlanePoseStatistics*> rankedPoses;
            rankedPoses.reserve(result.poses.size());
            for (std::size_t index = 0; index < result.poses.size(); ++index) {
                rankedPoses.push_back(&result.poses[index]);
            }
            std::sort(
                rankedPoses.begin(), rankedPoses.end(),
                [](const hik_calibration::PlanePoseStatistics* left,
                   const hik_calibration::PlanePoseStatistics* right) {
                    const double leftRatio = left->pointCount > 0
                        ? static_cast<double>(left->inlierCount) / left->pointCount : 0.0;
                    const double rightRatio = right->pointCount > 0
                        ? static_cast<double>(right->inlierCount) / right->pointCount : 0.0;
                    return leftRatio < rightRatio;
                });
            QStringList worst;
            const std::size_t count = std::min<std::size_t>(5, rankedPoses.size());
            for (std::size_t index = 0; index < count; ++index) {
                const hik_calibration::PlanePoseStatistics& pose = *rankedPoses[index];
                const double ratio = pose.pointCount > 0
                    ? static_cast<double>(pose.inlierCount) / pose.pointCount : 0.0;
                worst << QStringLiteral("%1=%2%(%3/%4)")
                    .arg(QString::fromStdString(pose.sampleId))
                    .arg(ratio * 100.0, 0, 'f', 1)
                    .arg(pose.inlierCount)
                    .arg(pose.pointCount);
            }
            appendLog(QStringLiteral("激光平面最差姿态（按内点率）：%1")
                      .arg(worst.join(QStringLiteral("；"))));
        }
        updateAllUiStates();
        return;
    }
    QString qualityReason;
    const bool pass = laserResultPassesQuality(&qualityReason);
    laserResultLabel_->setText(
        QStringLiteral("平面 [%1, %2, %3, %4] mm；姿态=%5，点=%6，内点=%7 (%8%)，RMS=%9 mm。质量：%10%11")
            .arg(result.plane.normal[0], 0, 'f', 9)
            .arg(result.plane.normal[1], 0, 'f', 9)
            .arg(result.plane.normal[2], 0, 'f', 9)
            .arg(result.plane.dMm, 0, 'f', 6)
            .arg(result.poseCount).arg(result.pointCount).arg(result.inlierCount)
            .arg(result.inlierRatio * 100.0, 0, 'f', 2)
            .arg(result.inlierDistanceMm.rms, 0, 'f', 4)
            .arg(pass ? QStringLiteral("通过") : QStringLiteral("不通过"))
            .arg(qualityReason.isEmpty() ? QString() : QStringLiteral("（%1）").arg(qualityReason)));
    appendLog(laserResultLabel_->text());
    saveLaserCandidate();
    updateAllUiStates();
}

bool HikCalibrationWindow::laserResultPassesQuality(QString* reason) const {
    if (reason) {
        reason->clear();
    }
    if (!hasLaserPlaneResult_ || !laserPlaneResult_.ok) {
        if (reason) {
            *reason = QStringLiteral("没有有效激光平面结果。");
        }
        return false;
    }
    if (laserPlaneResult_.poseCount < planeOptions_.minPoseCount) {
        if (reason) {
            *reason = QStringLiteral("有效板姿态数不足。");
        }
        return false;
    }
    QString guidanceReason;
    if (!laserGuidancePasses(&guidanceReason)) {
        if (reason) {
            *reason = QStringLiteral("300–1000 mm 正式采集覆盖未完成：%1")
                .arg(guidanceReason);
        }
        return false;
    }
    if (!hasActiveLaserIntrinsics_ ||
        activeLaserIntrinsics_.acceptedViewCount < intrinsicOptions_.minViews ||
        !std::isfinite(activeLaserIntrinsics_.calibrationRmsPx) ||
        activeLaserIntrinsics_.calibrationRmsPx > kApprovedIntrinsicRmsPx) {
        if (reason) {
            *reason = QStringLiteral("所用内参未达到正式准入：至少 %1 个采用视图且总 RMS 不超过 %2 px。")
                .arg(intrinsicOptions_.minViews)
                .arg(kApprovedIntrinsicRmsPx, 0, 'f', 2);
        }
        return false;
    }
    if (activeLaserIntrinsicsSha256_.isEmpty() ||
        !QFileInfo(activeLaserIntrinsicsPath_).isFile()) {
        if (reason) {
            *reason = QStringLiteral("所用内参没有可校验的源文件 SHA-256；仅可保存候选。");
        }
        return false;
    }
    const QString approvedIntrinsicsPath =
        profile_.intrinsicsConfigPath(sourceDir_);
    if (QFileInfo(activeLaserIntrinsicsPath_).canonicalFilePath() !=
        QFileInfo(approvedIntrinsicsPath).canonicalFilePath()) {
        if (reason) {
            *reason = QStringLiteral("正式激光平面必须加载 %1 已批准的内参 %2；当前内参仅可用于候选求解。")
                .arg(profile_.id, profile_.intrinsicsConfigRelativePath);
        }
        return false;
    }
    QString hashError;
    const QString currentIntrinsicsSha256 =
        fileSha256Hex(activeLaserIntrinsicsPath_, &hashError);
    if (currentIntrinsicsSha256.isEmpty() ||
        currentIntrinsicsSha256 != activeLaserIntrinsicsSha256_) {
        if (reason) {
            *reason = currentIntrinsicsSha256.isEmpty()
                ? hashError
                : QStringLiteral("内参源文件在激光数据采集后已发生变化，必须重新加载并重采激光数据。");
        }
        return false;
    }
    if (!QFileInfo(captureManifestPath_).isFile() ||
        fileSha256Hex(captureManifestPath_).isEmpty()) {
        if (reason) {
            *reason = QStringLiteral("会话采集清单缺失或无法校验；仅可保存候选。");
        }
        return false;
    }
    if (!std::isfinite(laserPlaneResult_.inlierRatio) ||
        laserPlaneResult_.inlierRatio < planeOptions_.minInlierRatio) {
        if (reason) {
            *reason = QStringLiteral("内点率低于 %1%。").arg(planeOptions_.minInlierRatio * 100.0, 0, 'f', 1);
        }
        return false;
    }
    const double norm = cv::norm(laserPlaneResult_.plane.normal);
    if (!std::isfinite(norm) || std::fabs(norm - 1.0) > 1.0e-6 ||
        !std::isfinite(laserPlaneResult_.plane.dMm) ||
        !std::isfinite(laserPlaneResult_.inlierDistanceMm.rms) ||
        !std::isfinite(laserPlaneResult_.inlierDistanceMm.p95)) {
        if (reason) {
            *reason = QStringLiteral("平面系数或距离统计非有限。");
        }
        return false;
    }
    if (laserPlaneResult_.inlierDistanceMm.rms > kApprovedPlaneRmsMm ||
        laserPlaneResult_.inlierDistanceMm.p95 > kApprovedPlaneP95Mm) {
        if (reason) {
            *reason = QStringLiteral("平面残差 RMS=%1 mm、P95=%2 mm，正式门槛为 %3/%4 mm。")
                .arg(laserPlaneResult_.inlierDistanceMm.rms, 0, 'f', 4)
                .arg(laserPlaneResult_.inlierDistanceMm.p95, 0, 'f', 4)
                .arg(kApprovedPlaneRmsMm, 0, 'f', 2)
                .arg(kApprovedPlaneP95Mm, 0, 'f', 2);
        }
        return false;
    }
    if (laserPlaneResult_.poses.size() <
        static_cast<std::size_t>(laserPlaneResult_.poseCount)) {
        if (reason) {
            *reason = QStringLiteral("缺少逐姿态平面残差统计。");
        }
        return false;
    }
    for (std::size_t index = 0; index < laserPlaneResult_.poses.size(); ++index) {
        const hik_calibration::PlanePoseStatistics& pose = laserPlaneResult_.poses[index];
        const double poseInlierRatio = pose.pointCount > 0
            ? static_cast<double>(pose.inlierCount) / pose.pointCount : 0.0;
        if (poseInlierRatio < 0.80 || !std::isfinite(pose.distanceMm.rms) ||
            !std::isfinite(pose.distanceMm.p95) || pose.distanceMm.rms > 0.30 ||
            pose.distanceMm.p95 > 0.50) {
            if (reason) {
                *reason = QStringLiteral("姿态 %1 的质量不足：内点率=%2%，RMS=%3 mm，P95=%4 mm。")
                    .arg(QString::fromStdString(pose.sampleId))
                    .arg(poseInlierRatio * 100.0, 0, 'f', 1)
                    .arg(pose.distanceMm.rms, 0, 'f', 4)
                    .arg(pose.distanceMm.p95, 0, 'f', 4);
            }
            return false;
        }
    }

    std::vector<cv::Vec3d> boardNormals;
    std::vector<double> boardDepths;
    std::vector<cv::Vec3d> uniqueTranslations;
    std::vector<cv::Vec3d> uniqueNormals;
    for (std::size_t index = 0; index < laserPairs_.size(); ++index) {
        const LaserPairSample& sample = laserPairs_[index];
        const hik_calibration::LaserCalibrationPairResult& pair = sample.result;
        if (pair.ok && !sample.acquisitionParametersVerified) {
            if (reason) {
                *reason = QStringLiteral("存在离线导入或缺少采集元数据的配对，无法验证 off/on 曝光与增益一致；仅可保存候选。");
            }
            return false;
        }
        if (!pair.ok || !pair.laserOffPose.ok ||
            !std::isfinite(pair.laserOffPose.tvec[2])) {
            continue;
        }
        boardNormals.push_back(cv::Vec3d(
            pair.laserOffPose.rotation(0, 2), pair.laserOffPose.rotation(1, 2),
            pair.laserOffPose.rotation(2, 2)));
        boardDepths.push_back(pair.laserOffPose.tvec[2]);
        bool duplicate = false;
        for (std::size_t uniqueIndex = 0;
             uniqueIndex < uniqueTranslations.size(); ++uniqueIndex) {
            if (cv::norm(pair.laserOffPose.tvec - uniqueTranslations[uniqueIndex]) < 5.0 &&
                angleBetweenNormalsDeg(boardNormals.back(), uniqueNormals[uniqueIndex]) < 3.0) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            uniqueTranslations.push_back(pair.laserOffPose.tvec);
            uniqueNormals.push_back(boardNormals.back());
        }
    }
    if (boardNormals.size() < static_cast<std::size_t>(planeOptions_.minPoseCount)) {
        if (reason) {
            *reason = QStringLiteral("缺少足够的板位姿统计。");
        }
        return false;
    }
    if (uniqueTranslations.size() < static_cast<std::size_t>(planeOptions_.minPoseCount)) {
        if (reason) {
            *reason = QStringLiteral("去除近重复位姿后仅剩 %1 个；要求至少 %2 个（平移差 5 mm 或法向差 3°）。")
                .arg(static_cast<int>(uniqueTranslations.size()))
                .arg(planeOptions_.minPoseCount);
        }
        return false;
    }
    double maximumNormalAngleDeg = 0.0;
    for (std::size_t first = 0; first < boardNormals.size(); ++first) {
        for (std::size_t second = first + 1; second < boardNormals.size(); ++second) {
            maximumNormalAngleDeg = std::max(
                maximumNormalAngleDeg,
                angleBetweenNormalsDeg(boardNormals[first], boardNormals[second]));
        }
    }
    const std::pair<std::vector<double>::const_iterator,
                    std::vector<double>::const_iterator> depthRange =
        std::minmax_element(boardDepths.begin(), boardDepths.end());
    const double meanDepth = std::accumulate(
        boardDepths.begin(), boardDepths.end(), 0.0) /
        static_cast<double>(boardDepths.size());
    const double depthSpan = *depthRange.second - *depthRange.first;
    const double requiredDepthSpan = std::max(30.0, std::fabs(meanDepth) * 0.06);
    if (maximumNormalAngleDeg < 12.0 || depthSpan < requiredDepthSpan) {
        if (reason) {
            *reason = QStringLiteral("激光姿态多样性不足：最大板倾角差=%1°、深度跨度=%2 mm；要求至少 12° 和 %3 mm。")
                .arg(maximumNormalAngleDeg, 0, 'f', 1)
                .arg(depthSpan, 0, 'f', 1)
                .arg(requiredDepthSpan, 0, 'f', 1);
        }
        return false;
    }
    return true;
}

hik_calibration::LaserPlaneYamlMetadata HikCalibrationWindow::laserMetadata() const {
    hik_calibration::LaserPlaneYamlMetadata metadata;
    metadata.cameraFrame = profile_.cameraFrame.toStdString();
    metadata.intrinsicsFile = activeLaserIntrinsicsPath_.toStdString();
    metadata.intrinsicsSha256 = activeLaserIntrinsicsSha256_.toStdString();
    metadata.printedPatternSha256 = activeLaserIntrinsicsMetadata_.printedPatternSha256;
    metadata.datasetManifest = captureManifestPath_.toStdString();
    metadata.datasetManifestSha256 =
        fileSha256Hex(captureManifestPath_).toStdString();
    metadata.generatedAt = QDateTime::currentDateTimeUtc()
        .toString(Qt::ISODateWithMs).toStdString();
    double minimumZ = std::numeric_limits<double>::max();
    double maximumZ = -std::numeric_limits<double>::max();
    for (std::size_t pairIndex = 0; pairIndex < laserPairs_.size(); ++pairIndex) {
        if (!laserPairs_[pairIndex].result.ok) {
            continue;
        }
        const std::vector<cv::Point3d>& points = laserPairs_[pairIndex].result.cameraPointsMm;
        for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
            if (std::isfinite(points[pointIndex].z)) {
                minimumZ = std::min(minimumZ, points[pointIndex].z);
                maximumZ = std::max(maximumZ, points[pointIndex].z);
            }
        }
    }
    metadata.validCameraZMinMm = minimumZ == std::numeric_limits<double>::max() ? 0.0 : minimumZ;
    metadata.validCameraZMaxMm = maximumZ == -std::numeric_limits<double>::max() ? 0.0 : maximumZ;
    return metadata;
}

bool HikCalibrationWindow::writeLaserYaml(const QString& path, QString* error) const {
    if (error) {
        error->clear();
    }
    if (!hasLaserPlaneResult_ || !laserPlaneResult_.ok || !hasActiveLaserIntrinsics_) {
        if (error) {
            *error = QStringLiteral("没有可保存的激光平面结果。");
        }
        return false;
    }
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
        if (error) {
            *error = QStringLiteral("无法创建 YAML 所在目录: %1").arg(QFileInfo(path).absolutePath());
        }
        return false;
    }
    std::string coreError;
    if (!hik_calibration::saveLaserPlaneYaml(
            toLocalPath(path), laserPlaneResult_, boardSpecFromUi(),
            laserMetadata(), &coreError)) {
        if (error) {
            *error = fromStdString(coreError);
        }
        return false;
    }
    return true;
}

void HikCalibrationWindow::saveLaserCandidate() {
    if (!hasLaserPlaneResult_ || !laserPlaneResult_.ok) {
        showError(QStringLiteral("无法保存激光平面"), QStringLiteral("请先成功求解。"));
        return;
    }
    const QString path = uniqueFilePath(
        laserSessionDir_, QStringLiteral("hik_laser_plane_candidate_%1").arg(captureTimestamp()),
        QStringLiteral("yaml"));
    QString error;
    if (!writeLaserYaml(path, &error)) {
        showError(QStringLiteral("候选激光平面保存失败"), error);
        return;
    }
    lastLaserCandidatePath_ = path;
    appendLog(QStringLiteral("激光平面 session 候选 YAML 已保存: %1").arg(path));
    updateAllUiStates();
}

void HikCalibrationWindow::saveApprovedLaserPlane() {
    QString qualityReason;
    if (!laserResultPassesQuality(&qualityReason)) {
        showError(QStringLiteral("禁止覆盖正式激光平面"),
                  QStringLiteral("当前结果未通过质量门槛：%1\n正式文件保持不变。")
                      .arg(qualityReason));
        return;
    }
    const QString approvedSessionPath = uniqueFilePath(
        laserSessionDir_, QStringLiteral("hik_laser_plane_approved_%1").arg(captureTimestamp()),
        QStringLiteral("yaml"));
    QString error;
    if (!writeLaserYaml(approvedSessionPath, &error)) {
        showError(QStringLiteral("激光平面保存失败"), error);
        return;
    }
    const QString formalPath = profile_.laserPlaneConfigPath(sourceDir_);
    if (!promoteFileAtomically(approvedSessionPath, formalPath, &error)) {
        showError(QStringLiteral("正式激光平面更新失败"), error);
        return;
    }
    lastLaserCandidatePath_ = approvedSessionPath;
    appendLog(QStringLiteral("激光平面质量通过，已原子更新正式文件: %1").arg(formalPath));
    QMessageBox::information(this, QStringLiteral("激光平面已批准"),
                             QStringLiteral("已更新:\n%1").arg(formalPath));
}

bool HikCalibrationWindow::promoteFileAtomically(const QString& source,
                                                 const QString& destination,
                                                 QString* error) const {
    if (error) {
        error->clear();
    }
    QFile input(source);
    if (!input.open(QIODevice::ReadOnly)) {
        if (error) {
            *error = QStringLiteral("无法读取 session 候选文件: %1").arg(source);
        }
        return false;
    }
    const QByteArray payload = input.readAll();
    if (input.error() != QFile::NoError || payload.isEmpty()) {
        if (error) {
            *error = QStringLiteral("读取 session 候选文件失败或文件为空: %1").arg(source);
        }
        return false;
    }
    if (!QDir().mkpath(QFileInfo(destination).absolutePath())) {
        if (error) {
            *error = QStringLiteral("无法创建正式配置目录: %1").arg(QFileInfo(destination).absolutePath());
        }
        return false;
    }
    QSaveFile output(destination);
    if (!output.open(QIODevice::WriteOnly) || output.write(payload) != payload.size()) {
        output.cancelWriting();
        if (error) {
            *error = QStringLiteral("无法写入正式配置的临时文件: %1").arg(destination);
        }
        return false;
    }
    if (!output.commit()) {
        if (error) {
            *error = QStringLiteral("无法原子替换正式配置: %1").arg(destination);
        }
        return false;
    }
    return true;
}

QImage HikCalibrationWindow::drawCharucoOverlay(
        const cv::Mat& grayImage,
        const hik_calibration::CharucoDetectionResult& detection,
        const QString& caption) const {
    cv::Mat gray = gray8Clone(grayImage);
    if (gray.empty()) {
        return QImage();
    }
    cv::Mat overlay;
    cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);
    if (!detection.markerIds.empty() && !detection.markerCorners.empty()) {
        cv::aruco::drawDetectedMarkers(overlay, detection.markerCorners,
                                       detection.markerIds, cv::Scalar(0, 190, 255));
    }
    if (!detection.observation.corners.empty() && !detection.observation.ids.empty()) {
        cv::aruco::drawDetectedCornersCharuco(
            overlay, detection.observation.corners, detection.observation.ids,
            detection.ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    }
    cv::putText(overlay, caption.toLatin1().constData(), cv::Point(20, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                detection.ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2,
                cv::LINE_AA);
    return cvMatToQImageCopy(overlay);
}

QImage HikCalibrationWindow::drawLaserOverlay(
        const cv::Mat& grayImage,
        const hik_calibration::LaserCalibrationPairResult& pair) const {
    cv::Mat gray = gray8Clone(grayImage);
    if (gray.empty()) {
        return QImage();
    }
    cv::Mat overlay;
    cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);
    if (!pair.laserOnDetection.markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(
            overlay, pair.laserOnDetection.markerCorners,
            pair.laserOnDetection.markerIds, cv::Scalar(0, 190, 255));
    }
    for (std::size_t index = 0; index < pair.stripe.size(); ++index) {
        const cv::Point point(static_cast<int>(std::lround(pair.stripe[index].pixel.x)),
                              static_cast<int>(std::lround(pair.stripe[index].pixel.y)));
        cv::circle(overlay, point, 1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
    const std::string caption = pair.ok
        ? std::string("laser pair OK: ") + std::to_string(pair.cameraPointsMm.size()) + " 3D points"
        : std::string("laser pair REJECT");
    cv::putText(overlay, caption, cv::Point(20, 36), cv::FONT_HERSHEY_SIMPLEX,
                0.8, pair.ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                2, cv::LINE_AA);
    const LaserGuidanceObservation guidance =
        laserGuidanceObservation(
            pair, hasActiveLaserIntrinsics_
                ? activeLaserIntrinsics_.board : boardSpecFromUi());
    if (guidance.valid) {
        std::ostringstream depthCaption;
        depthCaption << "Zmed=" << std::fixed << std::setprecision(1)
                     << guidance.medianDepthMm << " mm, bin=";
        if (guidance.binIndex >= 0) {
            depthCaption
                << kLaserGuidanceBins[static_cast<std::size_t>(
                       guidance.binIndex)].targetDepthMm
                << " mm";
        } else {
            depthCaption << "OUTSIDE";
        }
        cv::putText(overlay, depthCaption.str(), cv::Point(20, 68),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    guidance.binIndex >= 0 ? cv::Scalar(0, 255, 255)
                                           : cv::Scalar(0, 0, 255),
                    2, cv::LINE_AA);
        std::ostringstream poseCaption;
        poseCaption << "board XY=" << std::fixed << std::setprecision(1)
                    << guidance.boardXmm << "/" << guidance.boardYmm
                    << " mm, tilt XY=" << guidance.tiltXDeg << "/"
                    << guidance.tiltYDeg << " deg, stripe UV="
                    << guidance.stripeUMedianPx << "/"
                    << guidance.stripeVMedianPx << " px";
        cv::putText(overlay, poseCaption.str(), cv::Point(20, 98),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255),
                    1, cv::LINE_AA);
    }
    return cvMatToQImageCopy(overlay);
}

QImage HikCalibrationWindow::drawProfileOverlay(
        const cv::Mat& grayImage,
        const hik_calibration::StaticProfileResult& profile) const {
    cv::Mat gray = gray8Clone(grayImage);
    if (gray.empty()) {
        return QImage();
    }
    cv::Mat overlay;
    cv::cvtColor(gray, overlay, cv::COLOR_GRAY2BGR);
    for (std::size_t index = 0; index < profile.stripe.size(); ++index) {
        const cv::Point point(
            static_cast<int>(std::lround(profile.stripe[index].pixel.x)),
            static_cast<int>(std::lround(profile.stripe[index].pixel.y)));
        cv::circle(overlay, point, 1, cv::Scalar(0, 190, 255), -1, cv::LINE_AA);
    }
    for (std::size_t index = 0; index < profile.points.size(); ++index) {
        const cv::Point point(
            static_cast<int>(std::lround(profile.points[index].stripe.pixel.x)),
            static_cast<int>(std::lround(profile.points[index].stripe.pixel.y)));
        const cv::Scalar color = profile.points[index].insideBoard
            ? cv::Scalar(255, 255, 0) : cv::Scalar(0, 255, 0);
        cv::circle(overlay, point, 1, color, -1, cv::LINE_AA);
    }
    std::ostringstream caption;
    caption << "profile: " << profile.points.size() << " pts, line RMS="
            << std::fixed << std::setprecision(3) << profile.lineDistanceMm.rms << " mm";
    if (profile.boardValidationAvailable) {
        caption << ", board RMS=" << profile.boardPlaneDistanceMm.rms << " mm";
    }
    cv::putText(overlay, caption.str(), cv::Point(20, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.70,
                profile.boardValidationAvailable && !profile.boardValidationPassed
                    ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0),
                2, cv::LINE_AA);
    return cvMatToQImageCopy(overlay);
}

cv::Mat HikCalibrationWindow::qImageToGrayClone(const QImage& image) {
    if (image.isNull()) {
        return cv::Mat();
    }
    const QImage gray = image.convertToFormat(QImage::Format_Grayscale8);
    return cv::Mat(gray.height(), gray.width(), CV_8UC1,
                   const_cast<uchar*>(gray.constBits()),
                   static_cast<std::size_t>(gray.bytesPerLine())).clone();
}

QImage HikCalibrationWindow::cvMatToQImageCopy(const cv::Mat& image) {
    if (image.empty()) {
        return QImage();
    }
    if (image.type() == CV_8UC1) {
        return QImage(image.data, image.cols, image.rows,
                      static_cast<int>(image.step), QImage::Format_Grayscale8).copy();
    }
    if (image.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows,
                      static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    if (image.type() == CV_8UC4) {
        cv::Mat rgba;
        cv::cvtColor(image, rgba, cv::COLOR_BGRA2RGBA);
        return QImage(rgba.data, rgba.cols, rgba.rows,
                      static_cast<int>(rgba.step), QImage::Format_RGBA8888).copy();
    }
    return cvMatToQImageCopy(gray8Clone(image));
}

QString HikCalibrationWindow::fromStdString(const std::string& text) {
    return QString::fromUtf8(text.data(), static_cast<int>(text.size()));
}

std::string HikCalibrationWindow::toLocalPath(const QString& path) {
    const QByteArray encoded = QFile::encodeName(path);
    return std::string(encoded.constData(), static_cast<std::size_t>(encoded.size()));
}
