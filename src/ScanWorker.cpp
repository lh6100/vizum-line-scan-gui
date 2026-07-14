#include "ScanWorker.h"
#include "vision/VizumEyeCaptureOptions.h"
#include <QThread>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QElapsedTimer>
#include <QProcessEnvironment>
#include <QSettings>
#include <QImage>
#include <QDateTime>
#include <QSet>
#include <QStringList>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <unistd.h>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

static QString errString(int code) {
    char buf[256] = {0};
    VzNL_GetErrorInfo(code, buf);
    return QStringLiteral("[%1] %2").arg(code).arg(QString::fromLocal8Bit(buf));
}

static QImage imageFromVzImage(const SVzNLImageData* image) {
    if (!image || !image->pBuffer || image->nWidth == 0 || image->nHeight == 0) {
        return {};
    }

    const int width = static_cast<int>(image->nWidth);
    const int height = static_cast<int>(image->nHeight);
    const int channels = static_cast<int>(image->nChannels);
    const int stride = width * channels;
    if (stride <= 0 || image->nBufferSize < static_cast<unsigned int>(stride * height)) {
        return {};
    }

    switch (image->eImageType) {
    case keVzNLImageType_RGB888:
        return QImage(image->pBuffer, width, height, stride, QImage::Format_RGB888).copy();
    case keVzNLImageType_BGR888:
        return QImage(image->pBuffer, width, height, stride, QImage::Format_BGR888).copy();
    case keVzNLImageType_GRAY:
        return QImage(image->pBuffer, width, height, width, QImage::Format_Grayscale8).copy();
    case keVzNLImageType_BGRA8888:
        return QImage(image->pBuffer, width, height, width * 4, QImage::Format_ARGB32).copy();
    default:
        if (channels == 3) {
            return QImage(image->pBuffer, width, height, stride, QImage::Format_RGB888).copy();
        }
        if (channels == 1) {
            return QImage(image->pBuffer, width, height, width, QImage::Format_Grayscale8).copy();
        }
        return {};
    }
}

static QImage imageFromCvBgr(const cv::Mat& image) {
    if (image.empty()) {
        return {};
    }
    if (image.type() == CV_8UC3) {
        return QImage(image.data, image.cols, image.rows,
                      static_cast<int>(image.step), QImage::Format_BGR888).copy();
    }
    if (image.type() == CV_8UC1) {
        return QImage(image.data, image.cols, image.rows,
                      static_cast<int>(image.step), QImage::Format_Grayscale8).copy();
    }
    return {};
}

static cv::Mat grayMatFromQImage(const QImage& image) {
    QImage gray = image.convertToFormat(QImage::Format_Grayscale8);
    return cv::Mat(gray.height(), gray.width(), CV_8UC1,
                   const_cast<uchar*>(gray.constBits()), gray.bytesPerLine()).clone();
}

static int envInt(const char* name, int fallback, int minValue, int maxValue) {
    const QString value = QProcessEnvironment::systemEnvironment()
                              .value(QString::fromLatin1(name))
                              .trimmed();
    if (value.isEmpty()) {
        return fallback;
    }
    bool ok = false;
    const int parsed = value.toInt(&ok);
    if (!ok || parsed < minValue || parsed > maxValue) {
        return fallback;
    }
    return parsed;
}

static double envDouble(const char* name, double fallback, double minValue, double maxValue) {
    const QString value = QProcessEnvironment::systemEnvironment()
                              .value(QString::fromLatin1(name))
                              .trimmed();
    if (value.isEmpty()) {
        return fallback;
    }
    bool ok = false;
    const double parsed = value.toDouble(&ok);
    if (!ok || parsed < minValue || parsed > maxValue) {
        return fallback;
    }
    return parsed;
}

static cv::Ptr<cv::aruco::Dictionary> charucoDictionaryFromName(const QString& dictionaryName) {
    const QString name = dictionaryName.trimmed().toUpper();
    int dict = cv::aruco::DICT_4X4_50;
    if (name == QStringLiteral("DICT_4X4_100") || name == QStringLiteral("4X4_100")) {
        dict = cv::aruco::DICT_4X4_100;
    } else if (name == QStringLiteral("DICT_4X4_250") || name == QStringLiteral("4X4_250")) {
        dict = cv::aruco::DICT_4X4_250;
    } else if (name == QStringLiteral("DICT_4X4_1000") || name == QStringLiteral("4X4_1000")) {
        dict = cv::aruco::DICT_4X4_1000;
    } else if (name == QStringLiteral("DICT_5X5_50") || name == QStringLiteral("5X5_50")) {
        dict = cv::aruco::DICT_5X5_50;
    } else if (name == QStringLiteral("DICT_5X5_100") || name == QStringLiteral("5X5_100")) {
        dict = cv::aruco::DICT_5X5_100;
    }
    return cv::aruco::getPredefinedDictionary(dict);
}

static QString charucoDictionaryNameFromEnv() {
    return QProcessEnvironment::systemEnvironment()
        .value(QStringLiteral("VIZUM_CHARUCO_DICT"), QStringLiteral("DICT_4X4_50"))
        .trimmed();
}

static cv::Ptr<cv::aruco::Dictionary> charucoDictionaryFromEnv() {
    return charucoDictionaryFromName(charucoDictionaryNameFromEnv());
}

static QString formatCvMatrix(const cv::Mat& matrix) {
    QString out;
    for (int r = 0; r < matrix.rows; ++r) {
        QStringList row;
        for (int c = 0; c < matrix.cols; ++c) {
            row << QString::number(matrix.at<double>(r, c), 'f', 6);
        }
        out += QStringLiteral("    [") + row.join(QStringLiteral(", ")) + QStringLiteral("]\n");
    }
    return out.trimmed();
}

static QString formatArrayMatrix4(const double matrix[16]) {
    QString out;
    for (int r = 0; r < 4; ++r) {
        QStringList row;
        for (int c = 0; c < 4; ++c) {
            row << QString::number(matrix[r * 4 + c], 'f', 6);
        }
        out += QStringLiteral("    [") + row.join(QStringLiteral(", ")) + QStringLiteral("]\n");
    }
    return out.trimmed();
}

static QString formatRoi(const SVzNLROIRect& roi) {
    return QStringLiteral("left=%1 right=%2 top=%3 bottom=%4 w=%5 h=%6")
        .arg(roi.left)
        .arg(roi.right)
        .arg(roi.top)
        .arg(roi.bottom)
        .arg(roi.right - roi.left)
        .arg(roi.bottom - roi.top);
}

static cv::Mat mat4FromArray(const double matrix[16]) {
    cv::Mat out(4, 4, CV_64F);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out.at<double>(r, c) = matrix[r * 4 + c];
        }
    }
    return out;
}

static bool leftEyeIntrinsicsFromQMatrix(VZNLHANDLE hDevice, cv::Mat& cameraMatrix,
                                         cv::Mat& distCoeffs, QString& report,
                                         double qMatrix[16]) {
    std::fill(qMatrix, qMatrix + 16, 0.0);
    const int rc = VzNL_GetImageCalibMatrix(hDevice, qMatrix);
    if (rc != 0) {
        report = QStringLiteral("VzNL_GetImageCalibMatrix: %1").arg(errString(rc));
        return false;
    }

    const double cx = -qMatrix[3];
    const double cy = -qMatrix[7];
    const double f = qMatrix[11];
    if (!std::isfinite(f) || !std::isfinite(cx) || !std::isfinite(cy) || f <= 0.0) {
        report = QStringLiteral("VzNL_GetImageCalibMatrix 返回的 Q 矩阵无法转换左目内参: f=%1 cx=%2 cy=%3")
                     .arg(f, 0, 'f', 6)
                     .arg(cx, 0, 'f', 6)
                     .arg(cy, 0, 'f', 6);
        return false;
    }

    cameraMatrix = (cv::Mat_<double>(3, 3) <<
        f, 0.0, cx,
        0.0, f, cy,
        0.0, 0.0, 1.0);
    distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    report = QStringLiteral("已由 Q 矩阵转换左目内参，矫正图畸变按 0 处理");
    return true;
}

static void drawBoardCoordinateOverlay(cv::Mat& image, const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                       const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                       double boardWidth, double boardHeight, double axisLength) {
    std::vector<cv::Point3f> points3d = {
        cv::Point3f(0.0f, 0.0f, 0.0f),
        cv::Point3f(static_cast<float>(boardWidth), 0.0f, 0.0f),
        cv::Point3f(static_cast<float>(boardWidth), static_cast<float>(boardHeight), 0.0f),
        cv::Point3f(0.0f, static_cast<float>(boardHeight), 0.0f),
        cv::Point3f(static_cast<float>(axisLength), 0.0f, 0.0f),
        cv::Point3f(0.0f, static_cast<float>(axisLength), 0.0f),
        cv::Point3f(0.0f, 0.0f, static_cast<float>(axisLength))
    };
    std::vector<cv::Point2f> points2d;
    cv::projectPoints(points3d, rvec, tvec, cameraMatrix, distCoeffs, points2d);
    if (points2d.size() != points3d.size()) {
        return;
    }

    auto toPoint = [](const cv::Point2f& p) {
        return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
    };

    const cv::Scalar frameColor(0, 255, 255);
    for (int i = 0; i < 4; ++i) {
        cv::line(image, toPoint(points2d[i]), toPoint(points2d[(i + 1) % 4]), frameColor, 3, cv::LINE_AA);
        cv::circle(image, toPoint(points2d[i]), 5, frameColor, -1, cv::LINE_AA);
    }

    const cv::Point origin = toPoint(points2d[0]);
    cv::line(image, origin, toPoint(points2d[4]), cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
    cv::line(image, origin, toPoint(points2d[5]), cv::Scalar(0, 255, 0), 4, cv::LINE_AA);
    cv::line(image, origin, toPoint(points2d[6]), cv::Scalar(255, 0, 0), 4, cv::LINE_AA);

    const int font = cv::FONT_HERSHEY_SIMPLEX;
    cv::putText(image, "O", origin + cv::Point(8, -8), font, 0.8, frameColor, 2, cv::LINE_AA);
    cv::putText(image, "+X", toPoint(points2d[4]) + cv::Point(8, 0), font, 0.8, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::putText(image, "+Y", toPoint(points2d[5]) + cv::Point(8, 0), font, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::putText(image, "+Z", toPoint(points2d[6]) + cv::Point(8, 0), font, 0.8, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
}

static bool charucoReprojectionError(const cv::Ptr<cv::aruco::CharucoBoard>& board,
                                     const cv::Mat& charucoCorners, const cv::Mat& charucoIds,
                                     const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                                     const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                                     double& meanError, double& maxError) {
    meanError = 0.0;
    maxError = 0.0;
    if (!board || charucoCorners.empty() || charucoIds.empty() ||
        charucoCorners.total() != charucoIds.total()) {
        return false;
    }

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    objectPoints.reserve(charucoIds.total());
    imagePoints.reserve(charucoIds.total());
    for (int i = 0; i < charucoIds.rows; ++i) {
        const int id = charucoIds.at<int>(i, 0);
        if (id < 0 || id >= static_cast<int>(board->chessboardCorners.size())) {
            continue;
        }
        objectPoints.push_back(board->chessboardCorners[id]);
        imagePoints.push_back(charucoCorners.at<cv::Point2f>(i, 0));
    }
    if (objectPoints.size() < 4) {
        return false;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projected);
    double sum = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const double err = cv::norm(projected[i] - imagePoints[i]);
        sum += err;
        maxError = std::max(maxError, err);
    }
    meanError = sum / static_cast<double>(projected.size());
    return true;
}

static QString grayImageStats(const QImage& source) {
    QImage image = source.convertToFormat(QImage::Format_Grayscale8);
    if (image.isNull()) {
        return {};
    }

    quint64 hist[256] = {};
    quint64 count = 0;
    double sum = 0.0;
    for (int y = 0; y < image.height(); ++y) {
        const uchar* row = image.constScanLine(y);
        for (int x = 0; x < image.width(); ++x) {
            const int v = row[x];
            ++hist[v];
            ++count;
            sum += v;
        }
    }
    if (count == 0) {
        return {};
    }

    int minValue = 0;
    while (minValue < 255 && hist[minValue] == 0) {
        ++minValue;
    }
    int maxValue = 255;
    while (maxValue > 0 && hist[maxValue] == 0) {
        --maxValue;
    }
    quint64 acc = 0;
    int p99 = 0;
    const quint64 p99Target = static_cast<quint64>(std::ceil(count * 0.99));
    for (int i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc >= p99Target) {
            p99 = i;
            break;
        }
    }
    const quint64 nonzero = count - hist[0];
    return QStringLiteral("gray min=%1 max=%2 mean=%3 p99=%4 nonzero=%5/%6")
        .arg(minValue)
        .arg(maxValue)
        .arg(sum / static_cast<double>(count), 0, 'f', 2)
        .arg(p99)
        .arg(nonzero)
        .arg(count);
}

static QVector<double> vectorFromMat4(const cv::Mat& matrix) {
    QVector<double> values;
    if (matrix.rows != 4 || matrix.cols != 4 || matrix.type() != CV_64F) {
        return values;
    }
    values.reserve(16);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            values.push_back(matrix.at<double>(r, c));
        }
    }
    return values;
}

OwnedLaserLine::OwnedLaserLine() {
    std::memset(&data, 0, sizeof(data));
}

OwnedLaserLine::~OwnedLaserLine() {
    reset();
}

OwnedLaserLine::OwnedLaserLine(OwnedLaserLine&& other) noexcept {
    data = other.data;
    other.data.p2DPoint = nullptr;
    other.data.p3DPoint = nullptr;
    other.data.nPointCount = 0;
}

OwnedLaserLine& OwnedLaserLine::operator=(OwnedLaserLine&& other) noexcept {
    if (this != &other) {
        reset();
        data = other.data;
        other.data.p2DPoint = nullptr;
        other.data.p3DPoint = nullptr;
        other.data.nPointCount = 0;
    }
    return *this;
}

void OwnedLaserLine::reset() {
    delete[] static_cast<SVzNL2DLRPoint*>(data.p2DPoint);
    delete[] static_cast<SVzNLPointXYZRGBA*>(data.p3DPoint);
    std::memset(&data, 0, sizeof(data));
}

ScanWorker::ScanWorker(QObject* parent) : QObject(parent) {}

ScanWorker::~ScanWorker() {
    // Ensure clean shutdown if user closed window while connected.
    disconnectDevice();
    destroyRgbCloudMapping();
    destroySdk();
    clearQueue();
}

// ---------------- Static SDK trampolines ----------------
void ScanWorker::s_onLaserLineCB(EVzResultDataType eType, SVzLaserLineData* pLine, void* pParam) {
    auto* self = static_cast<ScanWorker*>(pParam);
    if (self && pLine) self->onLaserLineCB(eType, pLine);
}

void ScanWorker::s_onDeviceStatusCB(EVzDeviceWorkStatus eStatus, void* pInfoParam) {
    auto* self = static_cast<ScanWorker*>(pInfoParam);
    if (self) self->onDeviceStatusCB(eStatus);
}

// ---------------- Callback handlers (run in SDK thread) ----------------
void ScanWorker::onLaserLineCB(EVzResultDataType eType, SVzLaserLineData* pLine) {
    // CRITICAL: do NOT call any blocking SDK function here, do NOT lock anything that
    // can be held by the worker thread for long. We just deep-copy + enqueue.
    if (!m_scanRunning.load()) return;
    m_cbTotalLines.fetch_add(1);
    m_cbLastType.store(static_cast<int>(eType));
    m_cbLastPointCount.store(pLine ? pLine->nPointCount : 0);
    if (eType != keResultDataType_PointXYZRGBA) return;
    if (pLine->bEndOnceScan == VzTrue) {
        m_cbEndSignals.fetch_add(1);
        m_autoStopReceived.store(true);
    }
    if (pLine->nPointCount <= 0) return;
    m_cbNonEmptyLines.fetch_add(1);

    {
        QMutexLocker lk(&m_queueMutex);
        if (m_queue.size() >= m_maxQueuedLines) {
            m_queueOverflow.store(true);
            m_autoStopReceived.store(true);
            return;
        }
    }

    OwnedLaserLine ol;
    ol.data.dStep = pLine->dStep;
    ol.data.dTotleOffset = pLine->dTotleOffset;
    ol.data.llTimeStamp = pLine->llTimeStamp;
    ol.data.llFrameIdx = pLine->llFrameIdx;
    ol.data.nEncodeNo = pLine->nEncodeNo;
    ol.data.fSwingAngle = pLine->fSwingAngle;
    ol.data.bEndOnceScan = pLine->bEndOnceScan;
    ol.data.nPointCount = pLine->nPointCount;

    if (pLine->p2DPoint) {
        auto* p2 = new SVzNL2DLRPoint[pLine->nPointCount];
        std::memcpy(p2, pLine->p2DPoint, sizeof(SVzNL2DLRPoint) * pLine->nPointCount);
        ol.data.p2DPoint = p2;
    }
    if (pLine->p3DPoint) {
        auto* p3 = new SVzNLPointXYZRGBA[pLine->nPointCount];
        std::memcpy(p3, pLine->p3DPoint, sizeof(SVzNLPointXYZRGBA) * pLine->nPointCount);
        ol.data.p3DPoint = p3;
    }

    QMutexLocker lk(&m_queueMutex);
    m_queue.push(std::move(ol));
    m_cbAcceptedLines.fetch_add(1);
}

void ScanWorker::onDeviceStatusCB(EVzDeviceWorkStatus eStatus) {
    if (!m_scanRunning.load()) {
        return;
    }
    if (eStatus == keDeviceWorkStatus_Device_Auto_Stop ||
        eStatus == keDeviceWorkStatus_Device_Swing_Finish) {
        m_autoStopReceived.store(true);
    }
}

// ---------------- Helpers ----------------
void ScanWorker::clearQueueLocked() {
    while (!m_queue.empty()) {
        m_queue.pop();
    }
}

void ScanWorker::clearQueue() {
    QMutexLocker lk(&m_queueMutex);
    clearQueueLocked();
}

bool ScanWorker::ensureClosedDetect() {
    if (m_hDevice && VzNL_IsAutoDetecting(m_hDevice, nullptr)) {
        int rc = VzNL_StopAutoDetect(m_hDevice);
        if (rc != 0) {
            emit log(QStringLiteral("VzNL_StopAutoDetect failed %1").arg(errString(rc)));
            return false;
        }
    }
    return true;
}

bool ScanWorker::configureDeviceRuntime() {
    if (!m_hDevice) {
        return false;
    }

    int rc = 0;

    // Output image format
    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);

    if (VzNL_IsSupportRGBCamera(m_hDevice, nullptr) == VzTrue) {
        rc = VzNL_EnableRGB(m_hDevice, VzTrue);
        if (rc != 0) {
            emit log(QStringLiteral("EnableRGB: %1").arg(errString(rc)));
        } else {
            emit log("RGB 相机已启用");
        }
    } else {
        emit log("当前设备不支持 RGB 相机");
    }

    VzNL_SetDeviceStatusNotify(m_hDevice, &ScanWorker::s_onDeviceStatusCB, this);

    rc = VzNL_BeginDetectLaser(m_hDevice);
    if (rc != 0) {
        emit log(QStringLiteral("BeginDetectLaser 失败: %1").arg(errString(rc)));
        VzNL_SetDeviceStatusNotify(m_hDevice, nullptr, nullptr);
        m_laserToolBegun = false;
        return false;
    }
    m_laserToolBegun = true;

    rc = VzNL_SetTriggerMode(m_hDevice, keEyeTriggerMode_Master);
    if (rc != 0) emit log(QStringLiteral("SetTriggerMode: %1").arg(errString(rc)));

    unsigned int fmin = 0, fmax = 0;
    VzNL_QueryParamRange(m_hDevice, keDeviceParamType_FrameRate, &fmin, &fmax);
    if (fmax > 0) VzNL_SetFrameRate(m_hDevice, fmax);

    m_supportMotor = (VzTrue == VzNL_IsSupportSwingMotor(m_hDevice, nullptr));
    if (m_supportMotor) {
        VzNL_EnableCalibROI(m_hDevice, VzFalse);
        rc = VzNL_EnableSwingMotor(m_hDevice, VzTrue);
        if (rc != 0) emit log(QStringLiteral("EnableSwingMotor: %1").arg(errString(rc)));

        int aSpeed = 0;
        VzNL_GetSwingAngleSpeed(m_hDevice, &aSpeed);
        if (aSpeed < 1) {
            VzNL_SetSwingAngleSpeed(m_hDevice, (int)keSwingMotorSpeed_12);
        }

        rc = VzNL_SetSwingScanMode(m_hDevice, keSwingMotorScanMode_Once);
        if (rc != 0) emit log(QStringLiteral("SetSwingScanMode: %1").arg(errString(rc)));

        emit log("摆动电机已启用，扫描模式=Once");
    } else {
        emit log("当前相机不支持摆动电机（动态相机模式）");
    }

    return true;
}

bool ScanWorker::waitForDeviceReady(int timeoutMs) {
    if (!m_hDevice) {
        return false;
    }

    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeoutMs) {
        bool ready = true;

        if (m_supportMotor || VzNL_IsSupportSwingMotor(m_hDevice, nullptr) == VzTrue) {
            int rc = 0;
            if (VzNL_FindSwing(m_hDevice, &rc) != VzTrue || rc != 0) {
                ready = false;
            }
            rc = 0;
            VzNL_GetSwingCurAngle(m_hDevice, &rc);
            if (rc != 0) {
                ready = false;
            }
        }

        if (VzNL_IsSupportCoverCamera(m_hDevice, nullptr) == VzTrue) {
            int rc = 0;
            VzNL_IsCoverCamera(m_hDevice, &rc);
            if (rc != 0) {
                ready = false;
            }
        }

        if (ready) {
            return true;
        }
        QThread::msleep(300);
    }
    return false;
}

bool ScanWorker::ensureCoverOpenForScan() {
    if (!m_hDevice || VzNL_IsSupportCoverCamera(m_hDevice, nullptr) != VzTrue) {
        return true;
    }

    int rc = 0;
    if (VzNL_IsCoverCamera(m_hDevice, &rc) == VzFalse && rc == 0) {
        return true;
    }

    QElapsedTimer timer;
    timer.start();
    bool loggedRetry = false;
    while (timer.elapsed() < 12000) {
        rc = VzNL_CoverCamera(m_hDevice, VzFalse);
        if (rc == 0) {
            QThread::msleep(300);
            int stateRc = 0;
            if (VzNL_IsCoverCamera(m_hDevice, &stateRc) == VzFalse && stateRc == 0) {
                emit log("防尘盖已开");
                return true;
            }
        } else if (!loggedRetry) {
            emit log(QStringLiteral("开盖暂未就绪，正在等待: %1").arg(errString(rc)));
            loggedRetry = true;
        }
        QThread::msleep(500);
    }

    emit log(QStringLiteral("开盖失败，设备仍未就绪: %1").arg(errString(rc)));
    return false;
}

bool ScanWorker::moveSwingToStartForScan() {
    if (!m_supportMotor) {
        return true;
    }

    int rc = VzNL_EnableSwingMotor(m_hDevice, VzTrue);
    if (rc != 0) {
        emit log(QStringLiteral("EnableSwingMotor: %1").arg(errString(rc)));
    }

    VzNL_SetSwingScanMode(m_hDevice, keSwingMotorScanModeNone);
    QThread::msleep(100);
    rc = VzNL_SetSwingScanMode(m_hDevice, keSwingMotorScanMode_Once);
    if (rc != 0) {
        emit log(QStringLiteral("SetSwingScanMode: %1").arg(errString(rc)));
    }

    SVzSwingMotorDevInfo info{};
    rc = VzNL_GetSwingMotorInfo(m_hDevice, &info);
    const bool haveStartAngle = (rc == 0);
    if (rc != 0) {
        emit log(QStringLiteral("GetSwingMotorInfo: %1").arg(errString(rc)));
    }

    rc = VzNL_RotateSwingToStartPos(m_hDevice);
    if (rc != 0) {
        emit log(QStringLiteral("RotateSwingToStartPos: %1").arg(errString(rc)));
        return false;
    }

    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < 12000) {
        int angleRc = 0;
        const float angle = VzNL_GetSwingCurAngle(m_hDevice, &angleRc);
        if (angleRc == 0) {
            if (!haveStartAngle || std::abs(angle - static_cast<float>(info.nStartAngle)) <= 1.0f) {
                m_autoStopReceived.store(false);
                return true;
            }
        }
        QThread::msleep(200);
    }

    emit log("摆动电机回起点等待超时");
    m_autoStopReceived.store(false);
    return true;
}

void ScanWorker::ensureLaserLightEnabled() {
    if (!m_hDevice || !m_supportMotor) {
        return;
    }

    const VzBool before = VzNL_IsEnableLaserLight(m_hDevice);
    int rc = VzNL_EnableLaserLight(m_hDevice, VzTrue);
    if (rc != 0) {
        emit log(QStringLiteral("EnableLaserLight: %1").arg(errString(rc)));
        return;
    }

    int lightRc = 0;
    const unsigned int light = VzNL_GetLaserLight(m_hDevice, &lightRc);
    if (before != VzTrue) {
        emit log("线激光已启用");
    }
    if (lightRc == 0) {
        emit log(QStringLiteral("线激光亮度=%1").arg(light));
    }
}

bool ScanWorker::configureFixedLineForEyeCapture() {
    if (!m_hDevice || !m_supportMotor) {
        return true;
    }

    const int rc = VzNL_EnableSwingMotor(m_hDevice, VzFalse);
    if (rc != 0) {
        emit log(QStringLiteral("固定线拍照关闭摆动电机失败: %1").arg(errString(rc)));
        return false;
    }

    emit log("固定线拍照：摆动电机已关闭");
    QThread::msleep(150);
    return true;
}

bool ScanWorker::configurePointCloudProcMode() {
    if (!m_hDevice) {
        return false;
    }

    EVzPointCloudProcMode mode = m_hasLoadedPointCloudProcMode
                                      ? m_loadedPointCloudProcMode
                                      : kePointCloudProcMode_FixedStep;
    const QString modeEnv = QProcessEnvironment::systemEnvironment()
                                .value(QStringLiteral("VIZUM_POINT_CLOUD_PROC_MODE"))
                                .trimmed();
    if (!modeEnv.isEmpty()) {
        bool ok = false;
        const int modeValue = modeEnv.toInt(&ok);
        if (ok) {
            if (modeValue == kePointCloudProcMode_Speed ||
                modeValue == kePointCloudProcMode_Encoder ||
                modeValue == kePointCloudProcMode_FixedStep) {
                mode = static_cast<EVzPointCloudProcMode>(modeValue);
            } else {
                emit log(QStringLiteral("VIZUM_POINT_CLOUD_PROC_MODE=%1 无效，使用 FixedStep(3)").arg(modeEnv));
            }
        } else if (modeEnv.compare(QStringLiteral("speed"), Qt::CaseInsensitive) == 0) {
            mode = kePointCloudProcMode_Speed;
        } else if (modeEnv.compare(QStringLiteral("encoder"), Qt::CaseInsensitive) == 0) {
            mode = kePointCloudProcMode_Encoder;
        } else if (modeEnv.compare(QStringLiteral("fixedstep"), Qt::CaseInsensitive) == 0 ||
                   modeEnv.compare(QStringLiteral("fixed_step"), Qt::CaseInsensitive) == 0) {
            mode = kePointCloudProcMode_FixedStep;
        } else {
            emit log(QStringLiteral("VIZUM_POINT_CLOUD_PROC_MODE=%1 无法识别，使用 FixedStep(3)").arg(modeEnv));
        }
    }

    int rc = VzNL_SetPointCloudProcMode(m_hDevice, mode);
    if (rc != 0) {
        emit log(QStringLiteral("SetPointCloudProcMode(%1): %2")
                     .arg(static_cast<int>(mode))
                     .arg(errString(rc)));
        return false;
    }

    if (mode == kePointCloudProcMode_FixedStep) {
        const QString stepEnv = QProcessEnvironment::systemEnvironment()
                                    .value(QStringLiteral("VIZUM_LASER_FIXED_STEP"))
                                    .trimmed();
        if (!stepEnv.isEmpty()) {
            bool ok = false;
            const double step = stepEnv.toDouble(&ok);
            if (ok && step > 0.0) {
                rc = VzNL_SetLaserLineFixedStep(m_hDevice, step);
                if (rc != 0) {
                    emit log(QStringLiteral("SetLaserLineFixedStep(%1): %2")
                                 .arg(step, 0, 'g', 12)
                                 .arg(errString(rc)));
                } else {
                    emit log(QStringLiteral("固定步长 VIZUM_LASER_FIXED_STEP=%1").arg(step, 0, 'g', 12));
                }
            } else {
                emit log(QStringLiteral("VIZUM_LASER_FIXED_STEP=%1 无效，跳过固定步长设置").arg(stepEnv));
            }
        } else if (m_loadedFixedStep > 0.0) {
            rc = VzNL_SetLaserLineFixedStep(m_hDevice, m_loadedFixedStep);
            if (rc != 0) {
                emit log(QStringLiteral("SetLaserLineFixedStep(%1): %2")
                             .arg(m_loadedFixedStep, 0, 'g', 12)
                             .arg(errString(rc)));
            } else {
                emit log(QStringLiteral("固定步长来自 Vizum 配置=%1").arg(m_loadedFixedStep, 0, 'g', 12));
            }
        }
    }

    EVzPointCloudProcMode actual = kePointCloudProcMode_Invalid;
    rc = VzNL_GetPointCloudProcMode(m_hDevice, &actual);
    if (rc == 0) {
        emit log(QStringLiteral("点云处理模式已设置: %1").arg(static_cast<int>(actual)));
    } else {
        emit log(QStringLiteral("GetPointCloudProcMode: %1").arg(errString(rc)));
    }
    return true;
}

void ScanWorker::logScanRuntimeState(QString context) {
    if (!m_hDevice) {
        return;
    }

    int rc = 0;
    const EVzEyeTriggerMode triggerMode = VzNL_GetTriggerMode(m_hDevice, &rc);
    if (rc == 0) {
        emit log(QStringLiteral("%1: TriggerMode=%2").arg(context).arg(static_cast<int>(triggerMode)));
    } else {
        emit log(QStringLiteral("%1: GetTriggerMode %2").arg(context, errString(rc)));
    }

    EVzPointCloudProcMode procMode = kePointCloudProcMode_Invalid;
    rc = VzNL_GetPointCloudProcMode(m_hDevice, &procMode);
    if (rc == 0) {
        emit log(QStringLiteral("%1: PointCloudProcMode=%2").arg(context).arg(static_cast<int>(procMode)));
    } else {
        emit log(QStringLiteral("%1: GetPointCloudProcMode %2").arg(context, errString(rc)));
    }

    if (m_supportMotor) {
        const VzBool laserOn = VzNL_IsEnableLaserLight(m_hDevice);
        rc = 0;
        const unsigned int light = VzNL_GetLaserLight(m_hDevice, &rc);
        if (rc == 0) {
            emit log(QStringLiteral("%1: LaserLight=%2 brightness=%3")
                         .arg(context)
                         .arg(laserOn == VzTrue ? 1 : 0)
                         .arg(light));
        } else {
            emit log(QStringLiteral("%1: LaserLight=%2 GetLaserLight %3")
                         .arg(context)
                         .arg(laserOn == VzTrue ? 1 : 0)
                         .arg(errString(rc)));
        }

        SVzSwingMotorDevInfo info{};
        rc = VzNL_GetSwingMotorInfo(m_hDevice, &info);
        int angleRc = 0;
        const float curAngle = VzNL_GetSwingCurAngle(m_hDevice, &angleRc);
        if (rc == 0 && angleRc == 0) {
            emit log(QStringLiteral("%1: Swing angle=%2 start=%3 end=%4 speed=%5")
                         .arg(context)
                         .arg(curAngle, 0, 'f', 2)
                         .arg(info.nStartAngle)
                         .arg(info.nEndAngle)
                         .arg(info.nSpeed));
        } else {
            emit log(QStringLiteral("%1: Swing state GetInfo=%2 GetAngle=%3")
                         .arg(context)
                         .arg(rc)
                         .arg(angleRc));
        }
    }
}

bool ScanWorker::drainQueueToFile(VZNLFILE hFile, int& totalLines, int& totalPts) {
    bool ok = true;
    for (;;) {
        OwnedLaserLine ol;
        {
            QMutexLocker lk(&m_queueMutex);
            if (m_queue.empty()) {
                break;
            }
            ol = std::move(m_queue.front());
            m_queue.pop();
        }

        if (ol.data.nPointCount <= 0) {
            continue;
        }

        if (m_rgbCloudToolBuilding && m_rgbCloudTool) {
            int mapRc = VzNL_PushPointToRGBCloudPointTool(m_rgbCloudTool, &ol.data);
            if (mapRc != 0 && !m_rgbCloudPushFailed) {
                emit log(QStringLiteral("RGB 2D->3D 映射推点失败: %1").arg(errString(mapRc)));
                m_rgbCloudPushFailed = true;
            }
        }

        int rc = VzNL_WriteLaserFile(hFile, &ol.data);
        if (rc != 0) {
            emit log(QStringLiteral("WriteLaserFile: %1").arg(errString(rc)));
            ok = false;
        } else {
            totalLines++;
            totalPts += ol.data.nPointCount;
        }
    }
    return ok;
}

bool ScanWorker::beginRgbCloudMapping() {
    destroyRgbCloudMapping();
    if (!m_hDevice) {
        return false;
    }

    m_rgbCloudTool = VzNL_CreateRGBCloudPointTool(m_hDevice);
    if (!m_rgbCloudTool) {
        emit log("RGB 2D->3D 映射工具创建失败，本次扫描仍会保存 PLY");
        return false;
    }

    int rc = VzNL_BeginPushPointToRGBCloudPointTool(m_rgbCloudTool, keResultDataType_PointXYZRGBA);
    if (rc != 0) {
        emit log(QStringLiteral("RGB 2D->3D 映射工具初始化失败: %1").arg(errString(rc)));
        VzNL_DestroyRGBCloudPointTool(m_rgbCloudTool);
        m_rgbCloudTool = nullptr;
        return false;
    }

    m_rgbCloudToolBuilding = true;
    m_rgbCloudToolReady = false;
    m_rgbCloudPushFailed = false;
    emit log("RGB 2D->3D 映射缓存开始建立");
    return true;
}

void ScanWorker::finishRgbCloudMapping() {
    if (!m_rgbCloudTool || !m_rgbCloudToolBuilding) {
        return;
    }

    int rc = VzNL_EndPushPointToRGBCloudPointTool(m_rgbCloudTool);
    m_rgbCloudToolBuilding = false;
    m_rgbCloudToolReady = (rc == 0 && !m_rgbCloudPushFailed);
    if (rc != 0) {
        emit log(QStringLiteral("RGB 2D->3D 映射缓存结束失败: %1").arg(errString(rc)));
    } else if (m_rgbCloudToolReady) {
        emit log("RGB 2D->3D 映射缓存已就绪，可在 RGB 图上画线映射到点云");
    } else {
        emit log("RGB 2D->3D 映射缓存存在推点错误，本次 RGB 画线映射不可用");
    }
}

void ScanWorker::destroyRgbCloudMapping() {
    if (!m_rgbCloudTool) {
        m_rgbCloudToolBuilding = false;
        m_rgbCloudToolReady = false;
        return;
    }
    if (m_rgbCloudToolBuilding) {
        VzNL_EndPushPointToRGBCloudPointTool(m_rgbCloudTool);
    }
    VzNL_DestroyRGBCloudPointTool(m_rgbCloudTool);
    m_rgbCloudTool = nullptr;
    m_rgbCloudToolBuilding = false;
    m_rgbCloudToolReady = false;
    m_rgbCloudPushFailed = false;
}

bool ScanWorker::findMapped3DNearPixel(const QPoint& pixel, int searchRadius, QVector3D& point) const {
    if (!m_rgbCloudTool || !m_rgbCloudToolReady) {
        return false;
    }

    for (int radius = 0; radius <= searchRadius; ++radius) {
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (radius > 0 && std::abs(dx) != radius && std::abs(dy) != radius) {
                    continue;
                }
                SVzNL2DPoint p2d{};
                p2d.x = pixel.x() + dx;
                p2d.y = pixel.y() + dy;
                SVzNL3DPoint p3d{};
                int rc = VzNL_Find3DPointFrom2DPosForRGBCloudPointTool(m_rgbCloudTool, p2d, &p3d);
                const bool validPoint = std::isfinite(p3d.x) &&
                                        std::isfinite(p3d.y) &&
                                        std::isfinite(p3d.z) &&
                                        std::abs(p3d.z) > 1e-3;
                if (rc == 0 && validPoint) {
                    point = QVector3D(static_cast<float>(p3d.x),
                                      static_cast<float>(p3d.y),
                                      static_cast<float>(p3d.z));
                    return true;
                }
            }
        }
    }
    return false;
}

// ---------------- Slots ----------------
void ScanWorker::initSdk() {
    if (m_sdkInitialized) { emit log("SDK already initialized"); return; }
    SVzNLConfigParam cfg{};
    cfg.nDeviceTimeOut = 0;
    int rc = VzNL_Init(&cfg);
    if (rc != 0) {
        emit log(QStringLiteral("VzNL_Init failed: %1").arg(errString(rc)));
        return;
    }
    VzNL_SetLogLevel(keNLLogLevel_Information, keNLLogType_File);
    m_sdkInitialized = true;
    emit log("SDK initialized");
}

void ScanWorker::loadVizumConfig(QString filePath) {
    if (!m_hDevice) {
        emit log("请先连接设备，再加载 Vizum 配置");
        return;
    }
    if (m_scanRunning.load()) {
        emit log("正在扫描，暂不加载 Vizum 配置");
        return;
    }
    ensureClosedDetect();

    const QFileInfo info(filePath);
    if (!info.exists() || !info.isFile()) {
        emit log(QStringLiteral("Vizum 配置文件不存在: %1").arg(filePath));
        return;
    }

    QSettings cfg(filePath, QSettings::IniFormat);
    cfg.setIniCodec("UTF-8");
    int okCount = 0;
    int failCount = 0;
    int skipCount = 0;

    auto hasKey = [&](const QString& key) {
        return cfg.contains(key);
    };
    auto intValue = [&](const QString& key, int fallback = 0) {
        return cfg.value(key, fallback).toInt();
    };
    auto uintValue = [&](const QString& key, unsigned int fallback = 0) {
        return cfg.value(key, fallback).toUInt();
    };
    auto doubleValue = [&](const QString& key, double fallback = 0.0) {
        return cfg.value(key, fallback).toDouble();
    };
    auto boolValue = [&](const QString& key, bool fallback = false) {
        const QVariant value = cfg.value(key, fallback);
        if (value.type() == QVariant::String) {
            const QString text = value.toString().trimmed();
            if (text.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0) return true;
            if (text.compare(QStringLiteral("false"), Qt::CaseInsensitive) == 0) return false;
        }
        return value.toBool();
    };
    auto asVzBool = [](bool value) {
        return value ? VzTrue : VzFalse;
    };
    auto apply = [&](const QString& name, int rc) {
        if (rc == 0) {
            ++okCount;
        } else {
            ++failCount;
            emit log(QStringLiteral("%1: %2").arg(name, errString(rc)));
        }
    };
    auto skip = [&](const QString& name) {
        ++skipCount;
        emit log(QStringLiteral("跳过未映射/未公开接口参数: %1").arg(name));
    };
    auto roiFromKeys = [&](const QString& prefix, SVzNLROIRect& roi) {
        if (!hasKey(prefix + QStringLiteral("_L")) ||
            !hasKey(prefix + QStringLiteral("_R")) ||
            !hasKey(prefix + QStringLiteral("_T")) ||
            !hasKey(prefix + QStringLiteral("_B"))) {
            return false;
        }
        roi.left = intValue(prefix + QStringLiteral("_L"));
        roi.right = intValue(prefix + QStringLiteral("_R"));
        roi.top = intValue(prefix + QStringLiteral("_T"));
        roi.bottom = intValue(prefix + QStringLiteral("_B"));
        return true;
    };

    emit log(QStringLiteral("加载 Vizum 配置文件: %1").arg(info.fileName()));

    const QString basic = QStringLiteral("Device_0_BasicCfg/");
    if (hasKey(basic + QStringLiteral("nFrameRate"))) {
        const int fps = intValue(basic + QStringLiteral("nFrameRate"));
        apply(QStringLiteral("EnableFreeFrameRate"), VzNL_EnableFreeFrameRate(m_hDevice, VzTrue));
        apply(QStringLiteral("SetFrameRate(%1)").arg(fps), VzNL_SetFrameRate(m_hDevice, fps));
    }
    if (hasKey(basic + QStringLiteral("nExposeTime"))) {
        const unsigned int exposure = uintValue(basic + QStringLiteral("nExposeTime"));
        apply(QStringLiteral("ConfigEyeExpose(%1)").arg(exposure),
              VzNL_ConfigEyeExpose(m_hDevice, keVzNLExposeMode_Fix, exposure));
    }
    if (hasKey(basic + QStringLiteral("nLeftCameraGain"))) {
        const unsigned short gain = static_cast<unsigned short>(uintValue(basic + QStringLiteral("nLeftCameraGain")));
        apply(QStringLiteral("SetCameraGain(left=%1)").arg(gain),
              VzNL_SetCameraGain(m_hDevice, keEyeSensorType_Left, gain));
    }
    if (hasKey(basic + QStringLiteral("nRightCameraGain"))) {
        const unsigned short gain = static_cast<unsigned short>(uintValue(basic + QStringLiteral("nRightCameraGain")));
        apply(QStringLiteral("SetCameraGain(right=%1)").arg(gain),
              VzNL_SetCameraGain(m_hDevice, keEyeSensorType_Right, gain));
    }
    if (hasKey(basic + QStringLiteral("bIsEnableFillLaserPoint"))) {
        apply(QStringLiteral("EnableFillLaserPoint(%1)").arg(intValue(basic + QStringLiteral("bIsEnableFillLaserPoint"))),
              VzNL_EnableFillLaserPoint(m_hDevice, asVzBool(boolValue(basic + QStringLiteral("bIsEnableFillLaserPoint")))));
    }
    if (hasKey(basic + QStringLiteral("bIsDynamicROIEnabel"))) {
        apply(QStringLiteral("ConfigDynamicROI(%1)").arg(intValue(basic + QStringLiteral("bIsDynamicROIEnabel"))),
              VzNL_ConfigDynamicROI(m_hDevice, asVzBool(boolValue(basic + QStringLiteral("bIsDynamicROIEnabel")))));
    }
    if (hasKey(basic + QStringLiteral("bIsEnableCalibROI"))) {
        apply(QStringLiteral("EnableCalibROI(%1)").arg(intValue(basic + QStringLiteral("bIsEnableCalibROI"))),
              VzNL_EnableCalibROI(m_hDevice, asVzBool(boolValue(basic + QStringLiteral("bIsEnableCalibROI")))));
    }
    if (hasKey(basic + QStringLiteral("nLaserLightBrightness"))) {
        apply(QStringLiteral("SetLaserLight(%1)").arg(uintValue(basic + QStringLiteral("nLaserLightBrightness"))),
              VzNL_SetLaserLight(m_hDevice, uintValue(basic + QStringLiteral("nLaserLightBrightness"))));
    }
    if (hasKey(basic + QStringLiteral("nLaserDetectThres"))) {
        skip(QStringLiteral("Device_0_BasicCfg/nLaserDetectThres"));
    }
    if (hasKey(basic + QStringLiteral("nLaserCaptureInterval"))) {
        skip(QStringLiteral("Device_0_BasicCfg/nLaserCaptureInterval"));
    }
    if (hasKey(basic + QStringLiteral("bIsOnlyResult"))) {
        skip(QStringLiteral("Device_0_BasicCfg/bIsOnlyResult"));
    }

    SVzNLROIRect leftRoi{};
    SVzNLROIRect rightRoi{};
    if (roiFromKeys(basic + QStringLiteral("LeftROI"), leftRoi) &&
        roiFromKeys(basic + QStringLiteral("RightROI"), rightRoi)) {
        apply(QStringLiteral("ConfigDetectROI L[%1,%2,%3,%4] R[%5,%6,%7,%8]")
                  .arg(leftRoi.left).arg(leftRoi.right).arg(leftRoi.top).arg(leftRoi.bottom)
                  .arg(rightRoi.left).arg(rightRoi.right).arg(rightRoi.top).arg(rightRoi.bottom),
              VzNL_ConfigDetectROIWithFormat(m_hDevice, &leftRoi, &rightRoi));
    }

    const QString work = QStringLiteral("Device_0_WorkModeCfg/");
    if (hasKey(work + QStringLiteral("eTriggerMode"))) {
        const int mode = intValue(work + QStringLiteral("eTriggerMode"));
        apply(QStringLiteral("SetTriggerMode(%1)").arg(mode),
              VzNL_SetTriggerMode(m_hDevice, static_cast<EVzEyeTriggerMode>(mode)));
    }
    if (hasKey(work + QStringLiteral("ePointCloudProcMode"))) {
        const int mode = intValue(work + QStringLiteral("ePointCloudProcMode"));
        if (mode == kePointCloudProcMode_Speed ||
            mode == kePointCloudProcMode_Encoder ||
            mode == kePointCloudProcMode_FixedStep) {
            m_loadedPointCloudProcMode = static_cast<EVzPointCloudProcMode>(mode);
            m_hasLoadedPointCloudProcMode = true;
            apply(QStringLiteral("SetPointCloudProcMode(%1)").arg(mode),
                  VzNL_SetPointCloudProcMode(m_hDevice, m_loadedPointCloudProcMode));
        } else {
            ++failCount;
            emit log(QStringLiteral("ePointCloudProcMode=%1 无效，已忽略").arg(mode));
        }
    }
    if (hasKey(work + QStringLiteral("dFixedLaserLineStep"))) {
        m_loadedFixedStep = doubleValue(work + QStringLiteral("dFixedLaserLineStep"));
        if (m_loadedFixedStep > 0.0) {
            apply(QStringLiteral("SetLaserLineFixedStep(%1)").arg(m_loadedFixedStep, 0, 'g', 12),
                  VzNL_SetLaserLineFixedStep(m_hDevice, m_loadedFixedStep));
        }
    }
    if (hasKey(work + QStringLiteral("bIgnoreTriggerSignal"))) {
        apply(QStringLiteral("IgnoreTriggerExtSignal(%1)").arg(intValue(work + QStringLiteral("bIgnoreTriggerSignal"))),
              VzNL_IgnoreTriggerExtSignal(m_hDevice, asVzBool(boolValue(work + QStringLiteral("bIgnoreTriggerSignal")))));
    }
    if (hasKey(work + QStringLiteral("bEnableHWTrigger"))) {
        apply(QStringLiteral("EnableTriggerHwExtEn(%1)").arg(intValue(work + QStringLiteral("bEnableHWTrigger"))),
              VzNL_EnableTriggerHwExtEn(m_hDevice, asVzBool(boolValue(work + QStringLiteral("bEnableHWTrigger")))));
    }
    if (hasKey(work + QStringLiteral("eTriggerOutMode"))) {
        const int mode = intValue(work + QStringLiteral("eTriggerOutMode"));
        apply(QStringLiteral("SetStrobeTriggerOutMode(%1)").arg(mode),
              VzNL_SetStrobeTriggerOutMode(m_hDevice, static_cast<EVzStrobeTriggerOutMode>(mode)));
    }
    if (hasKey(work + QStringLiteral("nTriggerDivCoe"))) {
        const unsigned int div = uintValue(work + QStringLiteral("nTriggerDivCoe"));
        apply(QStringLiteral("SetTriggerDivCoe(%1)").arg(div), VzNL_SetTriggerDivCoe(m_hDevice, div));
    }
    if (hasKey(work + QStringLiteral("bTriggerPolor"))) {
        apply(QStringLiteral("SetTriggerPolar(%1)").arg(intValue(work + QStringLiteral("bTriggerPolor"))),
              VzNL_SetTriggerPolar(m_hDevice, asVzBool(boolValue(work + QStringLiteral("bTriggerPolor")))));
    }

    const QString swing = QStringLiteral("Device_0_SwingCfg/");
    if (hasKey(swing + QStringLiteral("bEnableSwing"))) {
        apply(QStringLiteral("EnableSwingMotor(%1)").arg(intValue(swing + QStringLiteral("bEnableSwing"))),
              VzNL_EnableSwingMotor(m_hDevice, asVzBool(boolValue(swing + QStringLiteral("bEnableSwing")))));
        m_supportMotor = (VzTrue == VzNL_IsSupportSwingMotor(m_hDevice, nullptr));
    }
    if (hasKey(swing + QStringLiteral("nLaserLightBrightness"))) {
        apply(QStringLiteral("SetLaserLight(%1)").arg(uintValue(swing + QStringLiteral("nLaserLightBrightness"))),
              VzNL_SetLaserLight(m_hDevice, uintValue(swing + QStringLiteral("nLaserLightBrightness"))));
    }
    if (hasKey(swing + QStringLiteral("fAngleSpeed"))) {
        const int speed = static_cast<int>(std::round(doubleValue(swing + QStringLiteral("fAngleSpeed"))));
        apply(QStringLiteral("SetSwingAngleSpeed(%1)").arg(speed), VzNL_SetSwingAngleSpeed(m_hDevice, speed));
    }
    if (hasKey(swing + QStringLiteral("fStartAngle")) && hasKey(swing + QStringLiteral("fStopAngle"))) {
        const int start = static_cast<int>(std::round(doubleValue(swing + QStringLiteral("fStartAngle"))));
        const int stop = static_cast<int>(std::round(doubleValue(swing + QStringLiteral("fStopAngle"))));
        apply(QStringLiteral("SetSwingMotorAngle(%1,%2)").arg(start).arg(stop),
              VzNL_SetSwingMotorAngle(m_hDevice, start, stop));
    }
    if (hasKey(swing + QStringLiteral("nSideStopTime"))) {
        const unsigned int ms = uintValue(swing + QStringLiteral("nSideStopTime"));
        apply(QStringLiteral("SetSwingStopTime(%1)").arg(ms), VzNL_SetSwingStopTime(m_hDevice, ms));
    }
    if (hasKey(swing + QStringLiteral("eScanMode"))) {
        const int mode = intValue(swing + QStringLiteral("eScanMode"));
        apply(QStringLiteral("SetSwingScanMode(%1)").arg(mode),
              VzNL_SetSwingScanMode(m_hDevice, static_cast<EVzSwingMotorScanMode>(mode)));
    }
    if (hasKey(swing + QStringLiteral("eRotateDirect"))) {
        const int direct = intValue(swing + QStringLiteral("eRotateDirect"));
        apply(QStringLiteral("SetSwingRotateDirect(%1)").arg(direct),
              VzNL_SetSwingRotateDirect(m_hDevice, static_cast<EVzSwingRotateDirect>(direct)));
    }
    if (hasKey(swing + QStringLiteral("dAngleDifTolerance"))) {
        const float angle = static_cast<float>(doubleValue(swing + QStringLiteral("dAngleDifTolerance")));
        apply(QStringLiteral("SetSwingEndAngleThres(%1)").arg(angle, 0, 'g', 8),
              VzNL_SetSwingEndAngleThres(m_hDevice, angle));
    }

    const QString rgb = QStringLiteral("Device_0_RGBCameraCfg/");
    if (hasKey(rgb + QStringLiteral("bEnableRGBD"))) {
        apply(QStringLiteral("EnableRGB(%1)").arg(intValue(rgb + QStringLiteral("bEnableRGBD"))),
              VzNL_EnableRGB(m_hDevice, asVzBool(boolValue(rgb + QStringLiteral("bEnableRGBD")))));
    }
    if (hasKey(rgb + QStringLiteral("bEnableSyncRGBD"))) {
        apply(QStringLiteral("EnableSyncRGBD(%1)").arg(intValue(rgb + QStringLiteral("bEnableSyncRGBD"))),
              VzNL_EnableSyncRGBD(m_hDevice, asVzBool(boolValue(rgb + QStringLiteral("bEnableSyncRGBD")))));
    }
    if (hasKey(rgb + QStringLiteral("bIsEnableRGBAWB"))) {
        apply(QStringLiteral("EnableRGBAWB(%1)").arg(intValue(rgb + QStringLiteral("bIsEnableRGBAWB"))),
              VzNL_EnableRGBAWB(m_hDevice, asVzBool(boolValue(rgb + QStringLiteral("bIsEnableRGBAWB")))));
    }
    if (hasKey(rgb + QStringLiteral("bEnableRGBAutoExpose"))) {
        apply(QStringLiteral("EnableRGBAutoExpose(%1)").arg(intValue(rgb + QStringLiteral("bEnableRGBAutoExpose"))),
              VzNL_EnableRGBAutoExpose(m_hDevice, asVzBool(boolValue(rgb + QStringLiteral("bEnableRGBAutoExpose")))));
    }
    if (hasKey(rgb + QStringLiteral("nFrameRate"))) {
        const unsigned int fps = uintValue(rgb + QStringLiteral("nFrameRate"));
        apply(QStringLiteral("SetRGBFrameRate(%1)").arg(fps), VzNL_SetRGBFrameRate(m_hDevice, fps));
    }
    if (hasKey(rgb + QStringLiteral("nExpose"))) {
        const unsigned int exposure = uintValue(rgb + QStringLiteral("nExpose"));
        apply(QStringLiteral("SetRGBExpose(%1)").arg(exposure), VzNL_SetRGBExpose(m_hDevice, exposure));
    }
    if (hasKey(rgb + QStringLiteral("nGain"))) {
        const unsigned int gain = uintValue(rgb + QStringLiteral("nGain"));
        apply(QStringLiteral("SetRGBGain(%1)").arg(gain), VzNL_SetRGBGain(m_hDevice, gain));
    }
    if (hasKey(rgb + QStringLiteral("fAutoExposeThres"))) {
        const float threshold = static_cast<float>(doubleValue(rgb + QStringLiteral("fAutoExposeThres")));
        apply(QStringLiteral("SetRGBAutoExposeThres(%1)").arg(threshold, 0, 'g', 8),
              VzNL_SetRGBAutoExposeThres(m_hDevice, threshold));
    }
    if (hasKey(rgb + QStringLiteral("eTriMode"))) {
        const int mode = intValue(rgb + QStringLiteral("eTriMode"));
        apply(QStringLiteral("SetRGBTriggerMode(%1)").arg(mode),
              VzNL_SetRGBTriggerMode(m_hDevice, static_cast<EVzEyeTriggerMode>(mode)));
    }
    SVzNLROIRect rgbRoi{};
    if (roiFromKeys(rgb + QStringLiteral("sROI"), rgbRoi)) {
        apply(QStringLiteral("SetRGBROI [%1,%2,%3,%4]")
                  .arg(rgbRoi.left).arg(rgbRoi.right).arg(rgbRoi.top).arg(rgbRoi.bottom),
              VzNL_SetRGBROI(m_hDevice, &rgbRoi));
    }

    const QString detect = QStringLiteral("Device_0_DetectCfg/");
    if (hasKey(detect + QStringLiteral("bIsEnableLaserLineFilterHeight"))) {
        apply(QStringLiteral("EnableLaserLineFilterHeight(%1)")
                  .arg(intValue(detect + QStringLiteral("bIsEnableLaserLineFilterHeight"))),
              VzNL_EnableLaserLineFilterHeight(
                  m_hDevice, asVzBool(boolValue(detect + QStringLiteral("bIsEnableLaserLineFilterHeight")))));
    }
    if (hasKey(detect + QStringLiteral("dFilterHeight"))) {
        const double height = doubleValue(detect + QStringLiteral("dFilterHeight"));
        apply(QStringLiteral("ConfigLaserLineFilterHeight(%1)").arg(height, 0, 'g', 12),
              VzNL_ConfigLaserLineFilterHeight(m_hDevice, height));
    }
    if (hasKey(detect + QStringLiteral("ePresetProfile"))) {
        skip(QStringLiteral("Device_0_DetectCfg/ePresetProfile"));
    }
    if (hasKey(detect + QStringLiteral("dMaxDeviation"))) {
        skip(QStringLiteral("Device_0_DetectCfg/dMaxDeviation"));
    }

    emit log(QStringLiteral("Vizum 配置加载完成: 成功 %1 项，失败 %2 项，跳过 %3 项")
                 .arg(okCount)
                 .arg(failCount)
                 .arg(skipCount));
}

void ScanWorker::connectDevice() {
    if (!m_sdkInitialized) initSdk();
    if (!m_sdkInitialized) return;
    if (m_hDevice) { emit log("Device already connected"); return; }

    emit busyChanged(true);

    // Search & bind in a loop (mirrors demo)
    std::vector<SVzNLEyeCBInfo> devices;
    bool keep = true;
    int loopGuard = 0;
    while (keep && loopGuard++ < 5) {
        keep = false;
        VzNL_ResearchDevice(keSearchDeviceFlag_EthLaserRobotEye);
        int n = 0;
        VzNL_GetEyeCBDeviceInfo(nullptr, &n);
        if (n <= 0) break;
        devices.assign(n, SVzNLEyeCBInfo{});
        VzNL_GetEyeCBDeviceInfo(devices.data(), &n);

        for (auto& d : devices) {
            if (d.bValidDevice) continue;
            SVzNLEthernetEyeConfigInfo eth{};
            int rc = VzNL_GetEthernetEyeConfigInfo(&d, &eth);
            if (rc != 0) {
                emit log(QStringLiteral("GetEthernetEyeConfigInfo: %1").arg(errString(rc)));
                continue;
            }
            if (memcmp(eth.byLocalIP, eth.sNetCardInfo.byLocalIP, 4) == 0) {
                emit log("请检查网卡/管理员权限。");
                continue;
            }
            rc = VzNL_BindEthernetEye(&d);
            if (rc != 0) {
                emit log(QStringLiteral("BindEthernetEye: %1").arg(errString(rc)));
            } else {
                keep = true;
            }
        }
    }

    if (devices.empty()) {
        emit log("未发现相机设备");
        emit busyChanged(false);
        return;
    }

    // Pick first valid device
    SVzNLEyeCBInfo* picked = nullptr;
    for (auto& d : devices) if (d.bValidDevice) { picked = &d; break; }
    if (!picked) picked = &devices[0];
    emit log(QStringLiteral("准备打开相机 %1").arg(QString::fromLatin1((const char*)picked->byServerIP)));

    SVzNLOpenDeviceParam openParam;  // ctor sets defaults
    int err = 0;
    VZNLHANDLE h = VzNL_OpenDevice(picked, &openParam, &err);
    if (err != 0 || !h) {
        emit log(QStringLiteral("打开设备失败: %1").arg(errString(err)));
        emit busyChanged(false);
        return;
    }
    m_hDevice = h;
    if (!configureDeviceRuntime()) {
        VzNL_SetDeviceStatusNotify(m_hDevice, nullptr, nullptr);
        VzNL_CloseDevice(m_hDevice);
        m_hDevice = nullptr;
        emit busyChanged(false);
        return;
    }

    emit log("设备连接成功");
    emit connectionChanged(true);
    emit busyChanged(false);
}

void ScanWorker::disconnectDevice() {
    if (!m_hDevice) { emit connectionChanged(false); return; }
    emit busyChanged(true);

    ensureClosedDetect();
    destroyRgbCloudMapping();

    if (m_laserToolBegun) {
        VzNL_EndDetectLaser(m_hDevice);
        m_laserToolBegun = false;
    }
    // Clear status notify
    VzNL_SetDeviceStatusNotify(m_hDevice, nullptr, nullptr);

    VzNL_CloseDevice(m_hDevice);
    m_hDevice = nullptr;
    m_hasLoadedPointCloudProcMode = false;
    m_loadedPointCloudProcMode = kePointCloudProcMode_Invalid;
    m_loadedFixedStep = 0.0;
    clearQueue();
    emit log("设备已断开");
    emit connectionChanged(false);
    emit busyChanged(false);
}

void ScanWorker::softResetDevice() {
    if (!m_hDevice) { emit log("未连接设备，无法软复位"); return; }
    emit busyChanged(true);
    emit log("开始软复位：停止检测流并重建线激光检测工具");

    m_scanRunning.store(false);
    m_autoStopReceived.store(false);
    ensureClosedDetect();
    clearQueue();
    destroyRgbCloudMapping();

    VzNL_SetDeviceStatusNotify(m_hDevice, nullptr, nullptr);
    if (m_laserToolBegun) {
        VzNL_EndDetectLaser(m_hDevice);
        m_laserToolBegun = false;
        QThread::msleep(150);
    }

    if (m_supportMotor || VzNL_IsSupportSwingMotor(m_hDevice, nullptr) == VzTrue) {
        int rc = VzNL_RebootSwing(m_hDevice);
        if (rc != 0) {
            emit log(QStringLiteral("RebootSwing: %1").arg(errString(rc)));
        } else {
            emit log("摆动模块已软重启");
            emit log("等待摆动模块恢复...");
            if (!waitForDeviceReady(20000)) {
                emit log("摆动模块恢复等待超时，继续尝试重建检测工具");
            }
        }
    }

    int rc = VzNL_ReConnectDevice(m_hDevice);
    if (rc != 0) {
        if (rc == -80000) {
            emit log("当前设备/SDK不支持控制器软重连，已跳过");
        } else {
            emit log(QStringLiteral("ReConnectDevice: %1").arg(errString(rc)));
        }
        QThread::msleep(300);
    } else {
        emit log("控制器重连接成功");
    }

    if (configureDeviceRuntime()) {
        if (!waitForDeviceReady(12000)) {
            emit log("软复位后设备动作接口仍在恢复中，首次扫描会继续等待");
        }
        m_autoStopReceived.store(false);
        emit log("软复位完成，可重新线扫建图");
        emit connectionChanged(true);
    } else {
        emit log("软复位未完成：线激光检测工具重建失败，请尝试关闭设备后重新连接，必要时再整机重启");
        emit connectionChanged(true);
    }
    emit busyChanged(false);
}

void ScanWorker::rebootDevice() {
    if (!m_hDevice) { emit log("未连接设备，无法重启"); return; }
    emit busyChanged(true);
    ensureClosedDetect();
    int rc = VzNL_RebootDevice(m_hDevice);
    if (rc != 0) emit log(QStringLiteral("RebootDevice: %1").arg(errString(rc)));
    else emit log("已发送重启命令");

    // After reboot we must close and let the user re-connect once camera comes back
    destroyRgbCloudMapping();
    if (m_laserToolBegun) { VzNL_EndDetectLaser(m_hDevice); m_laserToolBegun = false; }
    VzNL_CloseDevice(m_hDevice);
    m_hDevice = nullptr;
    clearQueue();
    emit connectionChanged(false);
    emit busyChanged(false);
}

void ScanWorker::destroySdk() {
    if (m_hDevice) disconnectDevice();
    if (m_sdkInitialized) {
        VzNL_Destroy();
        m_sdkInitialized = false;
        emit log("SDK 已销毁");
    }
}

void ScanWorker::getVersions() {
    if (!m_hDevice) {
        emit log("未连接设备，无法获取版本号");
        return;
    }

    SVzNLVersionInfo version{};
    int rc = VzNL_GetVersion(m_hDevice, &version);
    if (rc != 0) {
        emit log(QStringLiteral("获取版本号失败: %1").arg(errString(rc)));
        return;
    }

    emit log("版本信息：");
    auto logVersion = [this](const QString& name, long valid, const char* value) {
        if (valid) {
            emit log(QStringLiteral("  %1: %2").arg(name, QString::fromLocal8Bit(value)));
        } else {
            emit log(QStringLiteral("  %1: 不支持/不可用").arg(name));
        }
    };

    logVersion("SDK", version.sCap.sVersionCap.bValidSDKVersion, version.szSDKVersion);
    logVersion("EyeCB/App", version.sCap.sVersionCap.bValidAppVersion, version.szAppVersion);
    logVersion("Camera/Eye", version.sCap.sVersionCap.bValidCameraVersion, version.szCameraVersion);
    logVersion("Algo", version.sCap.sVersionCap.bValidAlgoVersion, version.szAlgoVersion);
    logVersion("APK", version.sCap.sVersionCap.bValidApkVersion, version.szApkVersion);
    logVersion("Device ID", version.sCap.sVersionCap.bValidDevIDVersion, version.szDevIDVersion);
    logVersion("WiFi", version.sCap.sVersionCap.bValidWifiVersion, version.szWifiVersion);
    logVersion("HDMI", version.sCap.sVersionCap.bValidHDMIVersion, version.szHDMIVersion);
    logVersion("Vizum Eye Hardware", version.sCap.sVersionCap.bValidVizumEyeVersion, version.szVizumEyeVersion);
    logVersion("Firmware", version.sCap.sVersionCap.bValidFwVersion, version.szFwVersion);

    SVzNLEyeDeviceInfoEx devInfo{};
    devInfo.sEyeCBInfo.nSize = sizeof(SVzNLEyeDeviceInfoEx);
    rc = VzNL_GetDeviceInfo(m_hDevice, &devInfo.sEyeCBInfo);
    if (rc == 0) {
        emit log(QStringLiteral("  Device Name: %1").arg(QString::fromLocal8Bit(devInfo.sEyeCBInfo.szDeviceName)));
        emit log(QStringLiteral("  Device IP: %1").arg(QString::fromLocal8Bit(devInfo.sEyeCBInfo.byServerIP)));
        emit log(QStringLiteral("  Device ID: %1").arg(QString::fromLocal8Bit(devInfo.sEyeCBInfo.szDeviceID)));
        emit log(QStringLiteral("  Device Hardware Version: %1").arg(devInfo.nHardwareVersion));
        if (devInfo.szVersion[0] != '\0') {
            emit log(QStringLiteral("  Device Version String: %1").arg(QString::fromLocal8Bit(devInfo.szVersion)));
        }
        if (devInfo.szHardwareVersion[0] != '\0') {
            emit log(QStringLiteral("  Hardware Version String: %1").arg(QString::fromLocal8Bit(devInfo.szHardwareVersion)));
        }
    } else {
        emit log(QStringLiteral("获取设备信息失败: %1").arg(errString(rc)));
    }

    if (m_supportMotor) {
        unsigned int swingVersion = VzNL_GetSwingVersionCode(m_hDevice);
        emit log(QStringLiteral("  Swing Motor Version Code: %1 (0x%2)")
                 .arg(swingVersion)
                 .arg(QString::number(swingVersion, 16).toUpper()));
    } else {
        emit log("  Swing Motor Version Code: 当前设备不支持摆动模块");
    }
}

void ScanWorker::openCover() {
    if (!m_hDevice) { emit log("未连接设备"); return; }
    emit busyChanged(true);
    if (VzNL_IsSupportCoverCamera(m_hDevice, nullptr) != VzTrue) {
        emit log("当前设备不支持防尘盖控制");
        emit busyChanged(false);
        return;
    }
    int rc = VzNL_CoverCamera(m_hDevice, VzFalse);  // false = open
    if (rc != 0) emit log(QStringLiteral("开盖失败: %1").arg(errString(rc)));
    else emit log("防尘盖已开");
    emit busyChanged(false);
}

void ScanWorker::closeCover() {
    if (!m_hDevice) { emit log("未连接设备"); return; }
    emit busyChanged(true);
    if (VzNL_IsSupportCoverCamera(m_hDevice, nullptr) != VzTrue) {
        emit log("当前设备不支持防尘盖控制");
        emit busyChanged(false);
        return;
    }
    int rc = VzNL_CoverCamera(m_hDevice, VzTrue);  // true = close
    if (rc != 0) emit log(QStringLiteral("关盖失败: %1").arg(errString(rc)));
    else emit log("防尘盖已关");
    emit busyChanged(false);
}

void ScanWorker::grabRgbImage() {
    if (!m_hDevice) {
        emit log("未连接设备，无法获取 RGB 图片");
        return;
    }
    if (m_scanRunning.load() || VzNL_IsAutoDetecting(m_hDevice, nullptr)) {
        emit log("扫描中，暂不获取 RGB 图片");
        return;
    }
    if (VzNL_IsSupportRGBCamera(m_hDevice, nullptr) != VzTrue) {
        emit log("当前设备不支持 RGB 相机");
        return;
    }

    int rc = 0;
    if (m_supportMotor && VzNL_IsEnableLaserLight(m_hDevice) == VzTrue) {
        rc = VzNL_EnableLaserLight(m_hDevice, VzFalse);
        if (rc != 0) {
            emit log(QStringLiteral("RGB 拍照前关闭线激光失败: %1").arg(errString(rc)));
        } else {
            emit log("RGB 拍照前已关闭线激光");
            QThread::msleep(300);
        }
    }

    if (VzNL_IsEnableRGB(m_hDevice, &rc) != VzTrue) {
        rc = VzNL_EnableRGB(m_hDevice, VzTrue);
        if (rc != 0) {
            emit log(QStringLiteral("启用 RGB 失败: %1").arg(errString(rc)));
            return;
        }
    }

    rc = VzNL_EnableRGBAWB(m_hDevice, VzTrue);
    if (rc != 0) {
        emit log(QStringLiteral("启用 RGB 自动白平衡失败: %1").arg(errString(rc)));
    }
    rc = VzNL_EnableRGBAutoExpose(m_hDevice, VzTrue);
    if (rc != 0) {
        emit log(QStringLiteral("启用 RGB 自动曝光失败: %1").arg(errString(rc)));
    }
    QThread::msleep(200);

    SVzNLImageData* rgbImage = nullptr;
    rc = VzNL_GetRGBImage(m_hDevice, &rgbImage);
    if (rc != 0 || !rgbImage) {
        VzNL_GenRGBSoftSignal(m_hDevice);
        QThread::msleep(150);
        rc = VzNL_GetRGBImage(m_hDevice, &rgbImage);
    }

    if (rc != 0 || !rgbImage) {
        emit log(QStringLiteral("获取 RGB 图片失败: %1").arg(errString(rc)));
        return;
    }

    QImage image = imageFromVzImage(rgbImage);
    if (image.isNull()) {
        const QString tmpPath = QDir::temp().filePath(
            QStringLiteral("vizum_rgb_%1.png").arg(QDateTime::currentMSecsSinceEpoch()));
        const QByteArray tmpLocal = tmpPath.toLocal8Bit();
        int saveRc = VzNL_SaveImage(tmpLocal.constData(), rgbImage);
        if (saveRc == 0) {
            image.load(tmpPath);
            QFile::remove(tmpPath);
        } else {
            emit log(QStringLiteral("RGB 图片格式转换失败，SaveImage: %1").arg(errString(saveRc)));
        }
    }
    const QString desc = QStringLiteral("RGB %1x%2 type=%3 channels=%4")
                             .arg(rgbImage->nWidth)
                             .arg(rgbImage->nHeight)
                             .arg(static_cast<int>(rgbImage->eImageType))
                             .arg(rgbImage->nChannels);
    VzNL_ReleaseImage(&rgbImage);

    if (image.isNull()) {
        emit log("RGB 图片格式暂不支持或数据为空");
        return;
    }

    emit log(QStringLiteral("已获取 %1").arg(desc));
    emit rgbImageReady(image, desc);
}

bool ScanWorker::configureLeftEyeImaging(int frameRate, int exposure, int gain) {
    if (!m_hDevice) {
        emit log("未连接设备，无法配置左目相机");
        return false;
    }

    bool ok = true;
    if (frameRate > 0) {
        unsigned int minFps = 0;
        unsigned int maxFps = 0;
        VzNL_QueryParamRange(m_hDevice, keDeviceParamType_FrameRate, &minFps, &maxFps);
        int freeRc = VzNL_EnableFreeFrameRate(m_hDevice, VzTrue);
        if (freeRc != 0) {
            emit log(QStringLiteral("启用自由帧率失败: %1").arg(errString(freeRc)));
        }
        const int requestedFrameRate =
            maxFps > 0 ? std::min(frameRate, static_cast<int>(maxFps)) : frameRate;
        int rc = VzNL_SetFrameRate(m_hDevice, requestedFrameRate);
        if (rc != 0) {
            emit log(QStringLiteral("设置双目帧率失败 fps=%1 range=[%2,%3]: %4")
                     .arg(requestedFrameRate).arg(minFps).arg(maxFps).arg(errString(rc)));
            ok = false;
        } else {
            int actualFps = 0;
            int getRc = VzNL_GetFrameRate(m_hDevice, &actualFps);
            emit log(getRc == 0
                     ? QStringLiteral("双目帧率=%1 fps range=[%2,%3]").arg(actualFps).arg(minFps).arg(maxFps)
                     : QStringLiteral("双目帧率已设置为 %1 fps range=[%2,%3]")
                           .arg(requestedFrameRate).arg(minFps).arg(maxFps));
        }
    }

    if (exposure >= 0) {
        const unsigned int exposeTime = static_cast<unsigned int>(std::max(0, std::min(exposure, 65535)));
        int rc = VzNL_ConfigEyeExpose(m_hDevice, keVzNLExposeMode_Fix, exposeTime);
        if (rc != 0) {
            emit log(QStringLiteral("设置左目曝光失败 exposure=%1: %2").arg(exposeTime).arg(errString(rc)));
            ok = false;
        } else {
            EVzNLExposeMode mode = keVzNLExposeMode_Fix;
            unsigned int actual = 0;
            int getRc = VzNL_GetConfigEyeExpose(m_hDevice, &mode, &actual);
            if (getRc == 0) {
                emit log(QStringLiteral("左目曝光=%1 mode=%2").arg(actual).arg(static_cast<int>(mode)));
            } else {
                emit log(QStringLiteral("左目曝光已设置为 %1").arg(exposeTime));
            }
        }
    }

    if (gain >= 0) {
        const unsigned short cameraGain = static_cast<unsigned short>(std::max(0, std::min(gain, 255)));
        int rc = VzNL_SetCameraGain(m_hDevice, keEyeSensorType_Left, cameraGain);
        if (rc != 0) {
            emit log(QStringLiteral("设置左目增益失败 gain=%1: %2").arg(cameraGain).arg(errString(rc)));
            ok = false;
        } else {
            unsigned short actual = 0;
            int getRc = VzNL_GetCameraGain(m_hDevice, keEyeSensorType_Left, &actual);
            emit log(getRc == 0
                     ? QStringLiteral("左目增益=%1").arg(actual)
                     : QStringLiteral("左目增益已设置为 %1").arg(cameraGain));
        }

        rc = VzNL_SetCameraGain(m_hDevice, keEyeSensorType_Right, cameraGain);
        if (rc != 0) {
            emit log(QStringLiteral("设置右目增益失败 gain=%1: %2").arg(cameraGain).arg(errString(rc)));
            ok = false;
        } else {
            unsigned short actual = 0;
            int getRc = VzNL_GetCameraGain(m_hDevice, keEyeSensorType_Right, &actual);
            emit log(getRc == 0
                     ? QStringLiteral("右目增益=%1").arg(actual)
                     : QStringLiteral("右目增益已设置为 %1").arg(cameraGain));
        }
    }

    if (frameRate > 0 || exposure >= 0 || gain >= 0) {
        const int settleMs = frameRate > 0
            ? std::max(250, static_cast<int>(std::ceil(2000.0 / static_cast<double>(frameRate))))
            : 250;
        emit log(QStringLiteral("等待左目曝光参数生效 %1 ms").arg(settleMs));
        QThread::msleep(static_cast<unsigned long>(settleMs));
    }
    return ok;
}

void ScanWorker::configureEyeCalibrationRuntime() {
    if (!m_hDevice) {
        return;
    }

    int rc = VzNL_SetTriggerMode(m_hDevice, keEyeTriggerMode_Master);
    if (rc != 0) {
        emit log(QStringLiteral("设置触发模式=主模式失败: %1").arg(errString(rc)));
    } else {
        int modeRc = 0;
        const EVzEyeTriggerMode mode = VzNL_GetTriggerMode(m_hDevice, &modeRc);
        emit log(modeRc == 0
                 ? QStringLiteral("触发模式=主模式(%1)").arg(static_cast<int>(mode))
                 : QStringLiteral("触发模式=主模式"));
    }

    rc = VzNL_IgnoreTriggerExtSignal(m_hDevice, VzTrue);
    if (rc != 0) {
        emit log(QStringLiteral("关闭外部使能模式失败: %1").arg(errString(rc)));
    } else {
        int stateRc = 0;
        const VzBool ignored = VzNL_IsIgnoreTriggerExtSignal(m_hDevice, &stateRc);
        emit log(stateRc == 0
                 ? QStringLiteral("外部使能模式=关(ignore=%1)").arg(static_cast<int>(ignored))
                 : QStringLiteral("外部使能模式=关"));
    }

    rc = VzNL_EnableTriggerHwExtEn(m_hDevice, VzFalse);
    if (rc != 0) {
        emit log(QStringLiteral("禁用硬件外部使能失败: %1").arg(errString(rc)));
    }

    rc = VzNL_SetStrobeTriggerOutMode(m_hDevice, keStrobeTriggerOutMode_None);
    if (rc != 0) {
        emit log(QStringLiteral("设置触发输出=无失败: %1").arg(errString(rc)));
    }

    rc = VzNL_SetAntiReflectGainType(m_hDevice, keAntiReflectGainType_Close);
    if (rc != 0) {
        emit log(QStringLiteral("关闭双目补偿失败: %1").arg(errString(rc)));
    } else {
        int antiRc = 0;
        const EVzAntiReflectGainType anti = VzNL_GetAntiReflectGainType(m_hDevice, &antiRc);
        emit log(antiRc == 0
                 ? QStringLiteral("双目补偿=关(%1)").arg(static_cast<int>(anti))
                 : QStringLiteral("双目补偿=关"));
    }
}

bool ScanWorker::configureFullEyeRoiForCalibration(bool useCalibImage) {
    if (!m_hDevice) {
        return false;
    }

    VzNL_SetOutputROIImage(useCalibImage ? VzTrue : VzFalse);

    int rc = VzNL_EnableTransPicMode(m_hDevice, VzTrue);
    if (rc != 0) {
        emit log(QStringLiteral("启用双目传图模式失败: %1").arg(errString(rc)));
    } else {
        emit log("双目传图模式已启用");
    }

    int captureRc = 0;
    if (VzNL_IsSupportCaptureMode(m_hDevice, &captureRc) == VzTrue) {
        captureRc = VzNL_SetCaptureMode(m_hDevice, keCaptureMode_LR_Image);
        if (captureRc != 0) {
            emit log(QStringLiteral("设置双目采图模式失败: %1").arg(errString(captureRc)));
        } else {
            int modeRc = 0;
            const EVzCaptureMode mode = VzNL_GetCaptureMode(m_hDevice, &modeRc);
            emit log(modeRc == 0
                     ? QStringLiteral("采图模式=LR_Image(%1)").arg(static_cast<int>(mode))
                     : QStringLiteral("采图模式已设置为 LR_Image"));
        }
    } else if (captureRc != 0) {
        emit log(QStringLiteral("当前设备不支持设置采图模式: %1").arg(errString(captureRc)));
    }

    rc = VzNL_EnableCalibROI(m_hDevice, useCalibImage ? VzTrue : VzFalse);
    if (rc != 0) {
        emit log(QStringLiteral("%1校正 ROI 图像失败: %2")
                 .arg(useCalibImage ? QStringLiteral("启用") : QStringLiteral("禁用"),
                      errString(rc)));
    } else {
        emit log(useCalibImage ? QStringLiteral("图像校正=开(矫正图)")
                               : QStringLiteral("图像校正=关"));
    }

    rc = VzNL_ConfigDynamicROI(m_hDevice, VzFalse);
    if (rc != 0) {
        emit log(QStringLiteral("禁用动态 ROI 失败: %1").arg(errString(rc)));
    }

    SVzVideoResolution resolution{};
    rc = VzNL_GetResolution(m_hDevice, &resolution);
    if (rc != 0 || resolution.nFrameWidth == 0 || resolution.nFrameHeight == 0) {
        emit log(QStringLiteral("获取左右目分辨率失败，无法设置全幅 ROI: %1").arg(errString(rc)));
        return false;
    }

    const int width = static_cast<int>(resolution.nFrameWidth);
    const int height = static_cast<int>(resolution.nFrameHeight);

    auto trySetRoi = [&](int right, int bottom, const QString& tag) -> bool {
        SVzNLROIRect leftRoi{0, right, 0, bottom};
        SVzNLROIRect rightRoi{0, right, 0, bottom};
        int setRc = VzNL_ConfigDetectROIWithFormat(m_hDevice, &leftRoi, &rightRoi);
        if (setRc != 0) {
            emit log(QStringLiteral("设置左右目全幅 ROI(%1) 失败: %2").arg(tag, errString(setRc)));
            return false;
        }

        SVzNLROIRect actualLeft{};
        SVzNLROIRect actualRight{};
        int getRc = VzNL_GetConfigDetectROI(m_hDevice, &actualLeft, &actualRight);
        if (getRc == 0) {
            emit log(QStringLiteral("左右目 ROI 已设为全幅(%1): L[%2] R[%3]")
                     .arg(tag, formatRoi(actualLeft), formatRoi(actualRight)));
        } else {
            emit log(QStringLiteral("左右目 ROI 已设置(%1)，但读回失败: %2").arg(tag, errString(getRc)));
        }
        return true;
    };

    if (trySetRoi(width, height, QStringLiteral("exclusive edge"))) {
        return true;
    }
    return trySetRoi(std::max(0, width - 1), std::max(0, height - 1), QStringLiteral("inclusive edge"));
}

bool ScanWorker::captureLeftEyeImage(QImage& image, QString& desc, int frameRate,
                                     int exposure, int gain, bool keepLaserOn,
                                     bool useCalibImage) {
    image = {};
    desc.clear();

    if (!m_hDevice) {
        emit log("未连接设备，无法获取左目灰度图");
        return false;
    }
    if (m_scanRunning.load() || VzNL_IsAutoDetecting(m_hDevice, nullptr)) {
        emit log("扫描中，暂不获取左目灰度图");
        return false;
    }

    if (m_supportMotor) {
        if (keepLaserOn) {
            int rc = VzNL_EnableLaserLight(m_hDevice, VzTrue);
            if (rc != 0) {
                emit log(QStringLiteral("左目拍照前打开线激光失败: %1").arg(errString(rc)));
            } else {
                emit log("左目拍照保持线激光开启");
                QThread::msleep(200);
            }
        } else {
            if (VzNL_IsEnableLaserLight(m_hDevice) == VzTrue) {
                int rc = VzNL_EnableLaserLight(m_hDevice, VzFalse);
                if (rc != 0) {
                    emit log(QStringLiteral("左目拍照前关闭线激光失败: %1").arg(errString(rc)));
                } else {
                    emit log("左目拍照前已关闭线激光");
                    QThread::msleep(200);
                }
            }
        }
    }

    if (!ensureCoverOpenForScan()) {
        return false;
    }

    configureFullEyeRoiForCalibration(useCalibImage);
    configureEyeCalibrationRuntime();
    configureLeftEyeImaging(frameRate, exposure, gain);

    VzNL_SetOutputImageFormat(keVzNLImageType_GRAY);
    SVzNLImageData* leftImage = nullptr;
    const unsigned int timeoutMs = static_cast<unsigned int>(
        frameRate > 0 ? std::max(2500, static_cast<int>(std::ceil(3000.0 / frameRate))) : 2000);
    int rc = 0;
    for (int i = 0; i < 2; ++i) {
        SVzNLImageData* frame = nullptr;
        rc = VzNL_GetEyeImage(m_hDevice, &frame, nullptr, timeoutMs);
        if (rc != 0 || !frame) {
            leftImage = nullptr;
            break;
        }
        if (i == 0) {
            emit log(QStringLiteral("已丢弃左目缓存帧，等待当前曝光帧"));
            VzNL_ReleaseImage(&frame);
            continue;
        }
        leftImage = frame;
    }
    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);

    if (rc != 0 || !leftImage) {
        emit log(QStringLiteral("获取左目灰度图失败: %1").arg(errString(rc)));
        return false;
    }

    image = imageFromVzImage(leftImage).convertToFormat(QImage::Format_Grayscale8);
    desc = QStringLiteral("LeftEye%1 %2x%3 type=%4 channels=%5")
               .arg(useCalibImage ? QStringLiteral("(calib)") : QString())
               .arg(leftImage->nWidth)
               .arg(leftImage->nHeight)
               .arg(static_cast<int>(leftImage->eImageType))
               .arg(leftImage->nChannels);
    VzNL_ReleaseImage(&leftImage);

    if (image.isNull()) {
        emit log("左目灰度图格式暂不支持或数据为空");
        return false;
    }

    const QString stats = grayImageStats(image);
    if (!stats.isEmpty()) {
        desc += QStringLiteral(" / %1").arg(stats);
    }
    emit log(QStringLiteral("已获取 %1").arg(desc));
    return true;
}

bool ScanWorker::captureLeftRightEyeImages(QImage& leftImage, QImage& rightImage, QString& desc,
                                           int frameRate, int exposure, int gain, bool keepLaserOn,
                                           bool useCalibImage) {
    leftImage = {};
    rightImage = {};
    desc.clear();

    if (!m_hDevice) {
        emit log("未连接设备，无法获取左右目图像");
        return false;
    }
    if (m_scanRunning.load() || VzNL_IsAutoDetecting(m_hDevice, nullptr)) {
        emit log("扫描中，暂不获取左右目图像");
        return false;
    }
    if (!configureFixedLineForEyeCapture()) {
        return false;
    }

    bool turnedLaserOnForThisCapture = false;
    if (m_supportMotor) {
        if (VzNL_IsEnableLaserLight(m_hDevice) != VzTrue) {
            int rc = VzNL_EnableLaserLight(m_hDevice, VzTrue);
            if (rc != 0) {
                emit log(QStringLiteral("左右目拍照前打开线激光失败: %1").arg(errString(rc)));
            } else {
                turnedLaserOnForThisCapture = true;
                emit log("左右目拍照已打开线激光");
                QThread::msleep(200);
            }
        } else if (keepLaserOn) {
            emit log("左右目拍照保持线激光开启");
        }
    }

    auto restoreLaserIfNeeded = [&]() {
        if (m_supportMotor && !keepLaserOn && VzNL_IsEnableLaserLight(m_hDevice) == VzTrue) {
            int rc = VzNL_EnableLaserLight(m_hDevice, VzFalse);
            if (rc != 0) {
                emit log(QStringLiteral("左右目拍照后关闭线激光失败: %1").arg(errString(rc)));
            } else if (turnedLaserOnForThisCapture) {
                emit log("左右目拍照后已关闭线激光");
            }
        }
    };

    if (!ensureCoverOpenForScan()) {
        restoreLaserIfNeeded();
        return false;
    }

    configureFullEyeRoiForCalibration(useCalibImage);
    configureEyeCalibrationRuntime();
    configureLeftEyeImaging(frameRate, exposure, gain);

    VzNL_SetOutputImageFormat(keVzNLImageType_GRAY);
    SVzNLImageData* leftFrame = nullptr;
    SVzNLImageData* rightFrame = nullptr;
    const unsigned int timeoutMs = static_cast<unsigned int>(
        frameRate > 0 ? std::max(2500, static_cast<int>(std::ceil(3000.0 / frameRate))) : 2000);

    int rc = 0;
    for (int i = 0; i < 2; ++i) {
        SVzNLImageData* l = nullptr;
        SVzNLImageData* r = nullptr;
        rc = VzNL_GetEyeImage(m_hDevice, &l, &r, timeoutMs);
        if (rc != 0 || !l || !r) {
            VzNL_ReleaseImage(&l);
            VzNL_ReleaseImage(&r);
            leftFrame = nullptr;
            rightFrame = nullptr;
            break;
        }
        if (i == 0) {
            emit log("已丢弃左右目缓存帧，等待当前曝光帧");
            VzNL_ReleaseImage(&l);
            VzNL_ReleaseImage(&r);
            continue;
        }
        leftFrame = l;
        rightFrame = r;
    }
    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);
    restoreLaserIfNeeded();

    if (rc != 0 || !leftFrame || !rightFrame) {
        emit log(QStringLiteral("获取左右目图像失败: %1").arg(errString(rc)));
        VzNL_ReleaseImage(&leftFrame);
        VzNL_ReleaseImage(&rightFrame);
        return false;
    }

    leftImage = imageFromVzImage(leftFrame).convertToFormat(QImage::Format_Grayscale8);
    rightImage = imageFromVzImage(rightFrame).convertToFormat(QImage::Format_Grayscale8);
    desc = QStringLiteral("Left %1x%2 / Right %3x%4 type=%5/%6 channels=%7/%8")
               .arg(leftFrame->nWidth)
               .arg(leftFrame->nHeight)
               .arg(rightFrame->nWidth)
               .arg(rightFrame->nHeight)
               .arg(static_cast<int>(leftFrame->eImageType))
               .arg(static_cast<int>(rightFrame->eImageType))
               .arg(leftFrame->nChannels)
               .arg(rightFrame->nChannels);
    VzNL_ReleaseImage(&leftFrame);
    VzNL_ReleaseImage(&rightFrame);

    if (leftImage.isNull() || rightImage.isNull()) {
        emit log("左右目图像格式暂不支持或数据为空");
        return false;
    }

    const QString leftStats = grayImageStats(leftImage);
    const QString rightStats = grayImageStats(rightImage);
    if (!leftStats.isEmpty() || !rightStats.isEmpty()) {
        desc += QStringLiteral(" / L:%1 / R:%2").arg(leftStats, rightStats);
    }
    emit log(QStringLiteral("已获取左右目图像: %1").arg(desc));
    return true;
}

void ScanWorker::grabLeftEyeImage() {
    emit busyChanged(true);
    QImage image;
    QString desc;
    if (captureLeftEyeImage(image, desc)) {
        emit leftEyeImageReady(image, desc);
    }
    emit busyChanged(false);
}

void ScanWorker::grabLeftEyeImageWithParams(int frameRate, int exposure, int gain, bool keepLaserOn) {
    emit busyChanged(true);
    QImage image;
    QString desc;
    if (captureLeftEyeImage(image, desc, frameRate, exposure, gain, keepLaserOn, true)) {
        emit leftEyeImageReady(image, desc);
    }
    emit busyChanged(false);
}

void ScanWorker::saveLeftRightEyeImages(int requestId, QString leftPath, QString rightPath,
                                        int frameRate, int exposure, int gain, bool keepLaserOn) {
    emit busyChanged(true);

    QImage leftImage;
    QImage rightImage;
    QString desc;
    const bool useCalibImage = vizum_capture::defaultRobotOffsetUseCalibImage();
    bool ok = captureLeftRightEyeImages(leftImage, rightImage, desc, frameRate, exposure, gain, keepLaserOn, useCalibImage);
    if (ok) {
        QFileInfo leftInfo(leftPath);
        QFileInfo rightInfo(rightPath);
        QDir().mkpath(leftInfo.absolutePath());
        QDir().mkpath(rightInfo.absolutePath());
        const bool leftOk = leftImage.save(leftPath);
        const bool rightOk = rightImage.save(rightPath);
        ok = leftOk && rightOk;
        if (ok) {
            emit log(QStringLiteral("左右目图像已保存: %1 / %2").arg(leftPath, rightPath));
        } else {
            desc += QStringLiteral(" / 保存失败 left=%1 right=%2").arg(leftOk ? 1 : 0).arg(rightOk ? 1 : 0);
            emit log(QStringLiteral("左右目图像保存失败: %1").arg(desc));
        }
    }

    emit leftRightEyeImagesSaved(requestId, ok, leftPath, rightPath, desc);
    emit busyChanged(false);
}

void ScanWorker::calibrateLeftEyeCharuco() {
    const int squaresX = envInt("VIZUM_CHARUCO_SQUARES_X", 7, 2, 40);
    const int squaresY = envInt("VIZUM_CHARUCO_SQUARES_Y", 5, 2, 40);
    const double squareMm = envDouble("VIZUM_CHARUCO_SQUARE_MM", 20.0, 0.1, 1000.0);
    const double markerMm = envDouble("VIZUM_CHARUCO_MARKER_MM", 15.0, 0.1, 1000.0);
    calibrateLeftEyeCharucoImpl(-1, -1, -1, squaresX, squaresY, squareMm, markerMm,
                                charucoDictionaryNameFromEnv(), false, false);
}

void ScanWorker::calibrateLeftEyeCharucoWithParams(int frameRate, int exposure, int gain,
                                                   int squaresX, int squaresY,
                                                   double squareMm, double markerMm, QString dictionaryName,
                                                   bool keepLaserOn) {
    calibrateLeftEyeCharucoImpl(frameRate, exposure, gain, squaresX, squaresY, squareMm, markerMm,
                                dictionaryName, true, keepLaserOn);
}

void ScanWorker::calibrateLeftEyeCharucoImpl(int frameRate, int exposure, int gain,
                                             int squaresX, int squaresY,
                                             double squareMm, double markerMm, QString dictionaryName,
                                             bool emitStructuredResult, bool keepLaserOn) {
    emit busyChanged(true);

    QImage qimage;
    QString desc;
    if (!captureLeftEyeImage(qimage, desc, frameRate, exposure, gain, keepLaserOn, true)) {
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, {}, QStringLiteral("左目取图失败"), {}, QStringLiteral("左目取图失败"));
        }
        emit busyChanged(false);
        return;
    }

    squaresX = std::max(2, std::min(squaresX, 40));
    squaresY = std::max(2, std::min(squaresY, 40));
    squareMm = std::max(0.1, std::min(squareMm, 1000.0));
    markerMm = std::max(0.1, std::min(markerMm, 1000.0));

    cv::Ptr<cv::aruco::Dictionary> dictionary = charucoDictionaryFromName(dictionaryName);
    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::aruco::CharucoBoard::create(squaresX, squaresY,
                                        static_cast<float>(squareMm),
                                        static_cast<float>(markerMm),
                                        dictionary);

    cv::Mat gray = grayMatFromQImage(qimage);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, params);

    cv::Mat annotated;
    cv::cvtColor(gray, annotated, cv::COLOR_GRAY2BGR);
    if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(annotated, markerCorners, markerIds);
    }

    if (markerIds.empty()) {
        const QString report = QStringLiteral("左目 ChArUco 外参失败：未检测到 ArUco marker。当前板: %1x%2, square=%3mm, marker=%4mm")
            .arg(squaresX).arg(squaresY).arg(squareMm).arg(markerMm);
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / no markers"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage, desc + QStringLiteral(" / no markers"), {}, report);
        }
        emit busyChanged(false);
        return;
    }

    const int maxDetectedMarkerId = *std::max_element(markerIds.begin(), markerIds.end());
    const int maxBoardMarkerId = board->ids.empty() ? -1 : *std::max_element(board->ids.begin(), board->ids.end());
    if (maxBoardMarkerId >= 0 && maxDetectedMarkerId > maxBoardMarkerId) {
        const QString report = QStringLiteral("左目 ChArUco 外参失败：当前板参数可能填错。检测到 marker id=%1，但 %2x%3 方格模型最大 id=%4；当前默认标定板请使用方格 X/Y=7/5")
            .arg(maxDetectedMarkerId)
            .arg(squaresX).arg(squaresY)
            .arg(maxBoardMarkerId);
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / board size mismatch"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage,
                                      desc + QStringLiteral(" / board size mismatch"), {}, report);
        }
        emit busyChanged(false);
        return;
    }

    cv::Mat charucoCorners;
    cv::Mat charucoIds;
    const int cornerCount = cv::aruco::interpolateCornersCharuco(
        markerCorners, markerIds, gray, board, charucoCorners, charucoIds);

    if (cornerCount > 0) {
        cv::aruco::drawDetectedCornersCharuco(annotated, charucoCorners, charucoIds);
    }

    if (cornerCount < 4) {
        const QString report = QStringLiteral("左目 ChArUco 外参失败：角点不足 markers=%1 charucoCorners=%2")
            .arg(markerIds.size()).arg(cornerCount);
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / charuco corners insufficient"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage,
                                      desc + QStringLiteral(" / charuco corners insufficient"), {}, report);
        }
        emit busyChanged(false);
        return;
    }

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    QString intrinsicsReport;
    double qMatrix[16] = {};
    if (!leftEyeIntrinsicsFromQMatrix(m_hDevice, cameraMatrix, distCoeffs, intrinsicsReport, qMatrix)) {
        const QString report = QStringLiteral("左目 ChArUco 已识别但无法解算外参：markers=%1 charucoCorners=%2；%3")
            .arg(markerIds.size()).arg(cornerCount).arg(intrinsicsReport);
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / intrinsics failed"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage,
                                      desc + QStringLiteral(" / intrinsics failed"), {}, report);
        }
        emit busyChanged(false);
        return;
    }
    emit log(QStringLiteral("左目 ChArUco 识别: markers=%1 charucoCorners=%2").arg(markerIds.size()).arg(cornerCount));
    emit log(intrinsicsReport);

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    const bool poseOk = cv::aruco::estimatePoseCharucoBoard(
        charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
    if (!poseOk) {
        const QString report = QStringLiteral("左目 ChArUco 外参失败：estimatePoseCharucoBoard 未收敛");
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / pose failed"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage, desc + QStringLiteral(" / pose failed"), {}, report);
        }
        emit busyChanged(false);
        return;
    }

    double meanReproj = 0.0;
    double maxReproj = 0.0;
    if (!charucoReprojectionError(board, charucoCorners, charucoIds, rvec, tvec,
                                  cameraMatrix, distCoeffs, meanReproj, maxReproj)) {
        const QString report = QStringLiteral("左目 ChArUco 外参失败：无法计算角点重投影误差");
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / reprojection failed"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage,
                                      desc + QStringLiteral(" / reprojection failed"), {}, report);
        }
        emit busyChanged(false);
        return;
    }
    emit log(QStringLiteral("左目 ChArUco 重投影误差: mean=%1 px max=%2 px")
             .arg(meanReproj, 0, 'f', 3)
             .arg(maxReproj, 0, 'f', 3));
    if (meanReproj > 5.0 || maxReproj > 20.0) {
        const QString report = QStringLiteral("左目 ChArUco 外参失败：重投影误差过大 mean=%1px max=%2px，板模型或内参不匹配。当前默认标定板请使用方格 X/Y=7/5")
            .arg(meanReproj, 0, 'f', 3)
            .arg(maxReproj, 0, 'f', 3);
        const QImage annotatedImage = imageFromCvBgr(annotated);
        emit log(report);
        emit leftEyeImageReady(annotatedImage, desc + QStringLiteral(" / reprojection too large"));
        if (emitStructuredResult) {
            emit leftEyeCharucoResult(false, annotatedImage,
                                      desc + QStringLiteral(" / reprojection too large"), {}, report);
        }
        emit busyChanged(false);
        return;
    }

    const double axisLength = squareMm * 2.0;
    drawBoardCoordinateOverlay(annotated, rvec, tvec, cameraMatrix, distCoeffs,
                               squaresX * squareMm, squaresY * squareMm, axisLength);
    cv::drawFrameAxes(annotated, cameraMatrix, distCoeffs, rvec, tvec, axisLength);

    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    cv::Mat tLeftBoard = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(tLeftBoard(cv::Rect(0, 0, 3, 3)));
    tLeftBoard.at<double>(0, 3) = tvec[0];
    tLeftBoard.at<double>(1, 3) = tvec[1];
    tLeftBoard.at<double>(2, 3) = tvec[2];

    emit log(QStringLiteral("左目 ChArUco 外参 OK: markers=%1 charucoCorners=%2, board=%3x%4 square=%5mm marker=%6mm")
             .arg(markerIds.size()).arg(cornerCount)
             .arg(squaresX).arg(squaresY).arg(squareMm).arg(markerMm));
    emit log("结果图已叠加棋盘外框: 黄色=OpenCV板外框/O原点，红=+X，绿=+Y，蓝=+Z");
    emit log(QStringLiteral("SDK 图像 Q 矩阵:\n%1").arg(formatArrayMatrix4(qMatrix)));
    emit log(QStringLiteral("左目内参 K:\n%1").arg(formatCvMatrix(cameraMatrix)));
    emit log(QStringLiteral("左目畸变 D: [0, 0, 0, 0, 0] (矫正图已消畸变)"));
    emit log(QStringLiteral("T_left_board (board -> left camera, mm):\n%1").arg(formatCvMatrix(tLeftBoard)));

    double leftToStereoArr[16] = {};
    int rc = VzNL_QueryLeftEye2StereoMatrix(m_hDevice, leftToStereoArr);
    if (rc == 0) {
        cv::Mat leftToStereo = mat4FromArray(leftToStereoArr);
        cv::Mat tStereoBoard = leftToStereo * tLeftBoard;
        emit log(QStringLiteral("SDK LeftEye->3D matrix:\n%1").arg(formatArrayMatrix4(leftToStereoArr)));
        emit log(QStringLiteral("T_sdk3d_board = LeftEye2Stereo * T_left_board (mm):\n%1")
                 .arg(formatCvMatrix(tStereoBoard)));
    } else {
        emit log(QStringLiteral("QueryLeftEye2StereoMatrix: %1").arg(errString(rc)));
    }

    double stereoToLeftArr[16] = {};
    rc = VzNL_QueryStereo2LeftEyeMatrix(m_hDevice, stereoToLeftArr);
    if (rc == 0) {
        emit log(QStringLiteral("SDK 3D->LeftEye matrix:\n%1").arg(formatArrayMatrix4(stereoToLeftArr)));
    } else {
        emit log(QStringLiteral("QueryStereo2LeftEyeMatrix: %1").arg(errString(rc)));
    }

    const QImage annotatedImage = imageFromCvBgr(annotated);
    const QString resultDesc = QStringLiteral("%1 / ChArUco pose OK").arg(desc);
    const QString report = QStringLiteral("markers=%1 charucoCorners=%2 board=%3x%4 square=%5mm marker=%6mm\nT_left_board (board -> left camera, mm):\n%7")
        .arg(markerIds.size()).arg(cornerCount)
        .arg(squaresX).arg(squaresY).arg(squareMm).arg(markerMm)
        .arg(formatCvMatrix(tLeftBoard));
    emit leftEyeImageReady(annotatedImage, resultDesc);
    if (emitStructuredResult) {
        emit leftEyeCharucoResult(true, annotatedImage, resultDesc, vectorFromMat4(tLeftBoard), report);
    }
    emit busyChanged(false);
}

void ScanWorker::mapRgbLineTo3D(QPoint start, QPoint end, int sampleCount) {
    if (!m_hDevice) {
        emit log("未连接设备，无法执行 RGB 线段到 3D 映射");
        emit rgbLineMapped({}, "未连接设备");
        return;
    }
    if (m_scanRunning.load() || VzNL_IsAutoDetecting(m_hDevice, nullptr)) {
        emit log("扫描中，暂不执行 RGB 线段映射");
        emit rgbLineMapped({}, "扫描中");
        return;
    }
    if (!m_rgbCloudToolReady || !m_rgbCloudTool) {
        emit log("RGB 2D->3D 映射缓存未就绪，请先完成一次线扫建图");
        emit rgbLineMapped({}, "请先线扫建图");
        return;
    }

    sampleCount = std::max(2, std::min(sampleCount, 1000));
    QVector<QVector3D> mappedPoints;
    mappedPoints.reserve(sampleCount);

    QSet<qint64> visited;
    const int searchRadius = 8;
    for (int i = 0; i < sampleCount; ++i) {
        const double t = sampleCount == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(sampleCount - 1);
        const int x = static_cast<int>(start.x() + (end.x() - start.x()) * t + 0.5);
        const int y = static_cast<int>(start.y() + (end.y() - start.y()) * t + 0.5);
        const qint64 key = (static_cast<qint64>(x) << 32) ^ static_cast<unsigned int>(y);
        if (visited.contains(key)) {
            continue;
        }
        visited.insert(key);

        QVector3D p3d;
        if (findMapped3DNearPixel(QPoint(x, y), searchRadius, p3d)) {
            mappedPoints.push_back(p3d);
        }
    }

    const QString desc = QStringLiteral("RGB线段 [%1,%2] -> [%3,%4], 采样%5，命中3D点%6")
                             .arg(start.x()).arg(start.y())
                             .arg(end.x()).arg(end.y())
                             .arg(sampleCount)
                             .arg(mappedPoints.size());
    emit log(desc);
    emit rgbLineMapped(mappedPoints, desc);
}

void ScanWorker::startScanAndSave(QString filePath) {
    if (!m_hDevice) { emit log("未连接设备"); emit scanFinished(false, filePath); return; }
    if (m_scanRunning.load()) { emit log("扫描中，请稍候"); return; }

    emit busyChanged(true);
    ensureClosedDetect();
    clearQueue();

    m_autoStopReceived.store(false);
    m_queueOverflow.store(false);
    m_cbTotalLines.store(0);
    m_cbAcceptedLines.store(0);
    m_cbNonEmptyLines.store(0);
    m_cbEndSignals.store(0);
    m_cbLastType.store(-1);
    m_cbLastPointCount.store(0);

    if (!waitForDeviceReady(8000)) {
        emit log("设备动作接口未就绪，取消本次扫描");
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    if (!ensureCoverOpenForScan()) {
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    if (!moveSwingToStartForScan()) {
        emit log("摆动电机未能回到起点，取消本次扫描");
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    ensureLaserLightEnabled();
    if (!configurePointCloudProcMode()) {
        emit log("点云处理模式设置失败，取消本次扫描");
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }
    m_autoStopReceived.store(false);
    logScanRuntimeState("扫描前状态");
    beginRgbCloudMapping();

    VZNLFILE hFile = VzNL_CreateLaserFile(keLaserFileType_Ply);
    if (!hFile) {
        emit log("创建 PLY 文件句柄失败");
        destroyRgbCloudMapping();
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    QByteArray pathLocal = filePath.toLocal8Bit();
    int rc = VzNL_OpenLaserFile(hFile, pathLocal.constData(), keFileModeWrite, keResultDataType_PointXYZRGBA);
    if (rc != 0) {
        emit log(QStringLiteral("打开输出文件失败: %1").arg(errString(rc)));
        VzNL_CloseLaserFile(hFile);
        destroyRgbCloudMapping();
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    // Reset offset; swing start position has already been verified above.
    VzNL_ConfigLaserBeginOffsetValue(m_hDevice, 0.0);

    m_scanRunning.store(true);
    m_autoStopReceived.store(false);

    rc = VzNL_StartAutoDetectEx(m_hDevice, keResultDataType_PointXYZRGBA,
                                keFlipType_None, &ScanWorker::s_onLaserLineCB, this);
    if (rc != 0) {
        m_scanRunning.store(false);
        emit log(QStringLiteral("开流失败: %1").arg(errString(rc)));
        VzNL_CloseLaserFile(hFile);
        destroyRgbCloudMapping();
        clearQueue();
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }
    emit log("开流成功，开始扫描...");

    int totalLines = 0;
    int totalPts = 0;
    bool writeOk = true;
    bool timeout = false;

    // Wait for stop signal (motor finishing) or hard timeout. While waiting, drain
    // queued callback data to disk outside of the SDK callback thread.
    QElapsedTimer t; t.start();
    const qint64 hardTimeoutMs = m_supportMotor ? 60000 : 5000;
    while (m_scanRunning.load() && !m_autoStopReceived.load() && t.elapsed() < hardTimeoutMs) {
        writeOk = drainQueueToFile(hFile, totalLines, totalPts) && writeOk;
        QThread::msleep(10);
    }
    if (!m_autoStopReceived.load() && t.elapsed() >= hardTimeoutMs) {
        timeout = true;
    }

    // Always stop, regardless of why we exited the wait loop
    rc = VzNL_StopAutoDetect(m_hDevice);
    if (rc != 0) emit log(QStringLiteral("StopAutoDetect: %1").arg(errString(rc)));
    m_scanRunning.store(false);

    // Drain any callbacks that arrived just before stop
    QThread::msleep(150);
    writeOk = drainQueueToFile(hFile, totalLines, totalPts) && writeOk;
    finishRgbCloudMapping();

    VzNL_CloseLaserFile(hFile);

    QFileInfo fi(filePath);
    bool ok = fi.exists() && fi.size() > 0 && totalPts > 0 &&
              writeOk && !timeout && !m_queueOverflow.load();
    if (timeout) {
        emit log("扫描超时，已强制停流");
    }
    if (m_queueOverflow.load()) {
        emit log("回调队列积压过多，本次扫描已主动停止，点云可能不完整");
    }
    emit log(QStringLiteral("扫描回调统计：total=%1 accepted=%2 nonEmpty=%3 endSignals=%4 lastType=%5 lastPoints=%6")
             .arg(m_cbTotalLines.load())
             .arg(m_cbAcceptedLines.load())
             .arg(m_cbNonEmptyLines.load())
             .arg(m_cbEndSignals.load())
             .arg(m_cbLastType.load())
             .arg(m_cbLastPointCount.load()));
    logScanRuntimeState("扫描后状态");
    if (!ok) {
        destroyRgbCloudMapping();
        emit log("本次扫描未生成有效 RGB 2D->3D 映射缓存");
    }
    emit log(QStringLiteral("扫描完成：%1 行 / %2 点 -> %3 (%4)")
             .arg(totalLines).arg(totalPts).arg(filePath).arg(ok ? "OK" : "EMPTY"));
    emit scanFinished(ok, filePath);
    emit busyChanged(false);
}
