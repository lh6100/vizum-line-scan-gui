#include "ScanWorker.h"
#include <QThread>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QElapsedTimer>
#include <QTextStream>
#include <QProcessEnvironment>
#include <cstring>
#include <unistd.h>

static QString errString(int code) {
    char buf[256] = {0};
    VzNL_GetErrorInfo(code, buf);
    return QStringLiteral("[%1] %2").arg(code).arg(QString::fromLocal8Bit(buf));
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
    if (eType != keResultDataType_PointXYZRGBA) return;
    if (pLine->bEndOnceScan == VzTrue) {
        m_autoStopReceived.store(true);
    }
    if (pLine->nPointCount <= 0) return;

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
}

void ScanWorker::onDeviceStatusCB(EVzDeviceWorkStatus eStatus) {
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

void ScanWorker::writeCsvHeader(QTextStream& csv) {
    csv << "line_idx,point_idx,frame_idx,timestamp,swing_angle,"
           "x_raw,y_raw,z_raw,x_m,y_m,z_m,rgb_uint,r,g,b\n";
}

void ScanWorker::writeLaserLineCsv(QTextStream& csv, const SVzLaserLineData& line,
                                   double unitScale, int lineIndex) {
    if (line.nPointCount <= 0 || !line.p3DPoint) {
        return;
    }

    const auto* pts = static_cast<const SVzNLPointXYZRGBA*>(line.p3DPoint);
    for (int i = 0; i < line.nPointCount; ++i) {
        const SVzNLPointXYZRGBA& p = pts[i];
        const unsigned int rgb = p.nRGB;
        const unsigned int r = (rgb >> 16) & 0xff;
        const unsigned int g = (rgb >> 8) & 0xff;
        const unsigned int b = rgb & 0xff;

        csv << lineIndex << ','
            << i << ','
            << line.llFrameIdx << ','
            << line.llTimeStamp << ','
            << line.fSwingAngle << ','
            << p.x << ','
            << p.y << ','
            << p.z << ','
            << (p.x * unitScale) << ','
            << (p.y * unitScale) << ','
            << (p.z * unitScale) << ','
            << rgb << ','
            << r << ','
            << g << ','
            << b << '\n';
    }
}

bool ScanWorker::drainQueueToFile(VZNLFILE hFile, QTextStream* csv, double unitScale,
                                  int& totalLines, int& totalPts) {
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

        if (csv) {
            writeLaserLineCsv(*csv, ol.data, unitScale, totalLines);
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

    // Output image format
    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);

    // Status callback for auto-stop signal
    VzNL_SetDeviceStatusNotify(m_hDevice, &ScanWorker::s_onDeviceStatusCB, this);

    // Begin laser detect tool
    int rc = VzNL_BeginDetectLaser(m_hDevice);
    if (rc != 0) {
        emit log(QStringLiteral("BeginDetectLaser 失败: %1").arg(errString(rc)));
        VzNL_CloseDevice(m_hDevice); m_hDevice = nullptr;
        emit busyChanged(false);
        return;
    }
    m_laserToolBegun = true;

    // Master trigger mode
    rc = VzNL_SetTriggerMode(m_hDevice, keEyeTriggerMode_Master);
    if (rc != 0) emit log(QStringLiteral("SetTriggerMode: %1").arg(errString(rc)));

    // Max frame rate
    unsigned int fmin = 0, fmax = 0;
    VzNL_QueryParamRange(m_hDevice, keDeviceParamType_FrameRate, &fmin, &fmax);
    if (fmax > 0) VzNL_SetFrameRate(m_hDevice, fmax);

    // Static (swing motor) camera setup
    m_supportMotor = (VzTrue == VzNL_IsSupportSwingMotor(m_hDevice, nullptr));
    if (m_supportMotor) {
        VzNL_EnableCalibROI(m_hDevice, VzFalse);
        VzNL_EnableSwingMotor(m_hDevice, VzTrue);

        // Sane default angle speed
        int aSpeed = 0;
        VzNL_GetSwingAngleSpeed(m_hDevice, &aSpeed);
        if (aSpeed < 1) {
            VzNL_SetSwingAngleSpeed(m_hDevice, (int)keSwingMotorSpeed_12);
        }

        // Once-mode so a single scan auto-stops
        VzNL_SetSwingScanMode(m_hDevice, keSwingMotorScanMode_Once);

        emit log("摆动电机已启用，扫描模式=Once");
    } else {
        emit log("当前相机不支持摆动电机（动态相机模式）");
    }

    emit log("设备连接成功");
    emit connectionChanged(true);
    emit busyChanged(false);
}

void ScanWorker::disconnectDevice() {
    if (!m_hDevice) { emit connectionChanged(false); return; }
    emit busyChanged(true);

    ensureClosedDetect();

    if (m_laserToolBegun) {
        VzNL_EndDetectLaser(m_hDevice);
        m_laserToolBegun = false;
    }
    // Clear status notify
    VzNL_SetDeviceStatusNotify(m_hDevice, nullptr, nullptr);

    VzNL_CloseDevice(m_hDevice);
    m_hDevice = nullptr;
    clearQueue();
    emit log("设备已断开");
    emit connectionChanged(false);
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
    if (VzNL_IsSupportCoverCamera(m_hDevice, nullptr) != VzTrue) {
        emit log("当前设备不支持防尘盖控制");
        return;
    }
    int rc = VzNL_CoverCamera(m_hDevice, VzFalse);  // false = open
    if (rc != 0) emit log(QStringLiteral("开盖失败: %1").arg(errString(rc)));
    else emit log("防尘盖已开");
}

void ScanWorker::closeCover() {
    if (!m_hDevice) { emit log("未连接设备"); return; }
    if (VzNL_IsSupportCoverCamera(m_hDevice, nullptr) != VzTrue) {
        emit log("当前设备不支持防尘盖控制");
        return;
    }
    int rc = VzNL_CoverCamera(m_hDevice, VzTrue);  // true = close
    if (rc != 0) emit log(QStringLiteral("关盖失败: %1").arg(errString(rc)));
    else emit log("防尘盖已关");
}

void ScanWorker::startScanAndSave(QString filePath) {
    if (!m_hDevice) { emit log("未连接设备"); emit scanFinished(false, filePath); return; }
    if (m_scanRunning.load()) { emit log("扫描中，请稍候"); return; }

    emit busyChanged(true);
    ensureClosedDetect();
    clearQueue();

    m_autoStopReceived.store(false);
    m_queueOverflow.store(false);

    VZNLFILE hFile = VzNL_CreateLaserFile(keLaserFileType_Ply);
    if (!hFile) {
        emit log("创建 PLY 文件句柄失败");
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    QByteArray pathLocal = filePath.toLocal8Bit();
    int rc = VzNL_OpenLaserFile(hFile, pathLocal.constData(), keFileModeWrite, keResultDataType_PointXYZRGBA);
    if (rc != 0) {
        emit log(QStringLiteral("打开输出文件失败: %1").arg(errString(rc)));
        VzNL_CloseLaserFile(hFile);
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    QFileInfo plyInfo(filePath);
    const QString stemPath = plyInfo.dir().filePath(plyInfo.completeBaseName());
    const QString pointsCsvPath = stemPath + QStringLiteral("_points.csv");
    const QString poseCsvPath = stemPath + QStringLiteral(".csv");

    QFile pointsCsvFile(pointsCsvPath);
    if (!pointsCsvFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        emit log(QStringLiteral("打开点云 CSV 失败: %1").arg(pointsCsvPath));
        VzNL_CloseLaserFile(hFile);
        emit scanFinished(false, filePath);
        emit busyChanged(false);
        return;
    }

    QTextStream pointsCsv(&pointsCsvFile);
    pointsCsv.setRealNumberNotation(QTextStream::FixedNotation);
    pointsCsv.setRealNumberPrecision(9);
    writeCsvHeader(pointsCsv);

    double unitScale = 0.001; // SDK raw coordinates are commonly millimetres; weld pipeline consumes metres.
    const QString scaleEnv = QProcessEnvironment::systemEnvironment().value(QStringLiteral("VIZUM_POINT_UNIT_SCALE"));
    bool scaleOk = false;
    const double envScale = scaleEnv.toDouble(&scaleOk);
    if (scaleOk && envScale > 0.0) {
        unitScale = envScale;
    }
    emit log(QStringLiteral("点云 CSV: %1").arg(pointsCsvPath));
    emit log(QStringLiteral("米制换算比例 VIZUM_POINT_UNIT_SCALE=%1").arg(unitScale, 0, 'g', 12));

    if (!QFileInfo::exists(poseCsvPath)) {
        QFile poseCsvFile(poseCsvPath);
        if (poseCsvFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
            QTextStream poseCsv(&poseCsvFile);
            poseCsv << "x,y,z,rx,ry,rz\n";
            poseCsv << "0,0,0,0,0,0\n";
            poseCsvFile.close();
            emit log(QStringLiteral("已生成机器人位姿占位 CSV，请采集后填入法奥 flange 位姿: %1").arg(poseCsvPath));
        } else {
            emit log(QStringLiteral("机器人位姿占位 CSV 创建失败，请手工创建: %1").arg(poseCsvPath));
        }
    }

    // Reset offset / motor pos (best-effort, no-op for dynamic cams)
    VzNL_ConfigLaserBeginOffsetValue(m_hDevice, 0.0);
    if (m_supportMotor) {
        VzNL_RotateSwingToStartPos(m_hDevice);
        QThread::msleep(300);
    }

    m_scanRunning.store(true);

    rc = VzNL_StartAutoDetectEx(m_hDevice, keResultDataType_PointXYZRGBA,
                                keFlipType_None, &ScanWorker::s_onLaserLineCB, this);
    if (rc != 0) {
        m_scanRunning.store(false);
        emit log(QStringLiteral("开流失败: %1").arg(errString(rc)));
        VzNL_CloseLaserFile(hFile);
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
        writeOk = drainQueueToFile(hFile, &pointsCsv, unitScale, totalLines, totalPts) && writeOk;
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
    writeOk = drainQueueToFile(hFile, &pointsCsv, unitScale, totalLines, totalPts) && writeOk;

    pointsCsv.flush();
    pointsCsvFile.close();
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
    emit log(QStringLiteral("扫描完成：%1 行 / %2 点 -> %3 (%4)")
             .arg(totalLines).arg(totalPts).arg(filePath).arg(ok ? "OK" : "EMPTY"));
    if (ok) {
        emit log(QStringLiteral("焊接输入点云 CSV 已保存: %1").arg(pointsCsvPath));
    }
    emit scanFinished(ok, filePath);
    emit busyChanged(false);
}
