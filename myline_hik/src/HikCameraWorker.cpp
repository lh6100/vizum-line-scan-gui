#include "HikCameraWorker.h"

#include <QByteArray>
#include <QMetaObject>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>

#if defined(HAVE_HIK_MVS)
#include <MvCameraControl.h>
#endif

namespace {

const int kConnectionRequestId = -1;

#if defined(HAVE_HIK_MVS)

struct MvsRuntimeState {
    std::mutex mutex;
    std::size_t referenceCount{0U};
};

MvsRuntimeState& mvsRuntimeState() {
    static MvsRuntimeState state;
    return state;
}

bool acquireMvsRuntime(bool* initializedNow,
                       std::size_t* referenceCount,
                       int* sdkError) {
    MvsRuntimeState& state = mvsRuntimeState();
    std::lock_guard<std::mutex> lock(state.mutex);
    bool didInitialize = false;
    if (state.referenceCount == 0U) {
        const int code = MV_CC_Initialize();
        if (code != MV_OK) {
            if (initializedNow) *initializedNow = false;
            if (referenceCount) *referenceCount = 0U;
            if (sdkError) *sdkError = code;
            return false;
        }
        didInitialize = true;
    }
    ++state.referenceCount;
    if (initializedNow) *initializedNow = didInitialize;
    if (referenceCount) *referenceCount = state.referenceCount;
    if (sdkError) *sdkError = MV_OK;
    return true;
}

void releaseMvsRuntime() noexcept {
    MvsRuntimeState& state = mvsRuntimeState();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.referenceCount == 0U) return;
    --state.referenceCount;
    if (state.referenceCount == 0U) {
        (void)MV_CC_Finalize();
    }
}

QString bytesToString(const unsigned char* bytes, int capacity) {
    if (!bytes || capacity <= 0) {
        return {};
    }
    int length = 0;
    while (length < capacity && bytes[length] != 0) {
        ++length;
    }
    return QString::fromLocal8Bit(reinterpret_cast<const char*>(bytes), length);
}

class FrameBufferGuard {
public:
    FrameBufferGuard(void* handle, MV_FRAME_OUT* frame)
        : m_handle(handle), m_frame(frame), m_owns(false) {}

    ~FrameBufferGuard() {
        if (m_owns && m_handle && m_frame) {
            MV_CC_FreeImageBuffer(m_handle, m_frame);
        }
    }

    void acquired() { m_owns = true; }
    int releaseNow() {
        if (m_owns && m_handle && m_frame) {
            const int code = MV_CC_FreeImageBuffer(m_handle, m_frame);
            m_owns = false;
            return code;
        }
        return MV_OK;
    }

private:
    void* m_handle;
    MV_FRAME_OUT* m_frame;
    bool m_owns;
};

#endif

} // namespace

HikCameraWorker::HikCameraWorker(QObject* parent)
    : QObject(parent),
      m_handle(nullptr),
      m_sdkRuntimeLeaseHeld(false),
      m_connected(false),
      m_grabbing(false),
      m_busy(false),
      m_deviceDisconnected(false) {}

HikCameraWorker::~HikCameraWorker() {
    // Do not emit signals from the destructor. The owner should normally call
    // disconnectCamera() in this object's thread before stopping that thread;
    // this remains a final leak-safe fallback for partial initialization.
    releaseDevice(false);
#if defined(HAVE_HIK_MVS)
    if (m_sdkRuntimeLeaseHeld) {
        releaseMvsRuntime();
        m_sdkRuntimeLeaseHeld = false;
    }
#endif
}

void HikCameraWorker::setBusy(bool busy) {
    if (m_busy == busy) {
        return;
    }
    m_busy = busy;
    emit busyChanged(busy);
}

bool HikCameraWorker::parseIpv4(const QString& text, quint32* value) {
    const QStringList parts = text.trimmed().split(QLatin1Char('.'));
    if (parts.size() != 4) {
        return false;
    }

    quint32 parsed = 0;
    for (const QString& part : parts) {
        if (part.isEmpty()) {
            return false;
        }
        bool ok = false;
        const int octet = part.toInt(&ok, 10);
        if (!ok || octet < 0 || octet > 255) {
            return false;
        }
        parsed = (parsed << 8U) | static_cast<quint32>(octet);
    }

    if (value) {
        *value = parsed;
    }
    return true;
}

QString HikCameraWorker::formatIpv4(quint32 value) {
    return QStringLiteral("%1.%2.%3.%4")
        .arg((value >> 24U) & 0xffU)
        .arg((value >> 16U) & 0xffU)
        .arg((value >> 8U) & 0xffU)
        .arg(value & 0xffU);
}

#if defined(HAVE_HIK_MVS)

bool HikCameraWorker::initializeSdk(QString* failure) {
    if (m_sdkRuntimeLeaseHeld) {
        return true;
    }

    bool initializedNow = false;
    std::size_t referenceCount = 0U;
    int code = MV_OK;
    if (!acquireMvsRuntime(&initializedNow, &referenceCount, &code)) {
        if (failure) {
            *failure = formatSdkError(QStringLiteral("MV_CC_Initialize"), code);
        }
        return false;
    }

    m_sdkRuntimeLeaseHeld = true;
    const quint32 version = MV_CC_GetSDKVersion();
    emit log(QStringLiteral("Hik MVS SDK %1: V%2.%3.%4.%5，进程级引用=%6")
                 .arg(initializedNow ? QStringLiteral("已初始化")
                                     : QStringLiteral("运行时已复用"))
                 .arg((version >> 24U) & 0xffU)
                 .arg((version >> 16U) & 0xffU)
                 .arg((version >> 8U) & 0xffU)
                 .arg(version & 0xffU)
                 .arg(static_cast<qulonglong>(referenceCount)));
    return true;
}

QString HikCameraWorker::sdkErrorName(int code) {
    switch (static_cast<quint32>(code)) {
    case MV_OK: return QStringLiteral("成功");
    case MV_E_HANDLE: return QStringLiteral("无效句柄");
    case MV_E_SUPPORT: return QStringLiteral("设备或节点不支持此功能");
    case MV_E_BUFOVER: return QStringLiteral("缓存已满");
    case MV_E_CALLORDER: return QStringLiteral("SDK 调用顺序错误");
    case MV_E_PARAMETER: return QStringLiteral("参数错误");
    case MV_E_RESOURCE: return QStringLiteral("资源申请失败");
    case MV_E_NODATA: return QStringLiteral("等待超时或没有图像数据");
    case MV_E_PRECONDITION: return QStringLiteral("前置条件或运行环境已改变");
    case MV_E_NOENOUGH_BUF: return QStringLiteral("图像缓存空间不足");
    case MV_E_ABNORMAL_IMAGE: return QStringLiteral("异常或不完整图像");
    case MV_E_NORESPONSE: return QStringLiteral("设备无响应");
    case MV_E_GC_RANGE: return QStringLiteral("参数超出相机允许范围");
    case MV_E_GC_ACCESS: return QStringLiteral("相机节点当前不可访问");
    case MV_E_GC_TIMEOUT: return QStringLiteral("GenICam 操作超时");
    case MV_E_ACCESS_DENIED: return QStringLiteral("设备无访问权限，可能被 MVS 或其他程序占用");
    case MV_E_BUSY: return QStringLiteral("设备忙或网络已断开");
    case MV_E_PACKET: return QStringLiteral("GigE 网络包错误");
    case MV_E_NETER: return QStringLiteral("GigE 网络错误");
    case MV_E_IP_CONFLICT: return QStringLiteral("相机 IP 冲突");
    default: return QStringLiteral("未识别的 MVS 错误");
    }
}

QString HikCameraWorker::formatSdkError(const QString& operation, int code) {
    const QString hex = QString::number(static_cast<quint32>(code), 16)
                            .rightJustified(8, QLatin1Char('0'))
                            .toUpper();
    return QStringLiteral("%1 失败: 0x%2 (%3)")
        .arg(operation, hex, sdkErrorName(code));
}

void HikCameraWorker::onSdkException(unsigned int messageType, void* userData) {
    HikCameraWorker* self = static_cast<HikCameraWorker*>(userData);
    if (!self) {
        return;
    }
    if (messageType == MV_EXCEPTION_DEV_DISCONNECT) {
        // SDK callbacks may run on an internal thread. Never close/destroy the
        // handle here; only publish an atomic notification to the owner thread.
        self->m_deviceDisconnected.store(true);
        (void)QMetaObject::invokeMethod(self, "handleDeviceDisconnect",
                                        Qt::QueuedConnection);
    }
}

#endif

void HikCameraWorker::handleDeviceDisconnect() {
    if (!m_deviceDisconnected.load()) return;
    if (m_continuousRunning.load()) stopContinuous();
    releaseDevice(false);
    const QString message = QStringLiteral("海康相机网络连接已断开，请重新启动程序并连接设备");
    emit error(kConnectionRequestId, message);
    emit connectionChanged(false, message);
}

void HikCameraWorker::releaseDevice(bool reportErrors) {
#if defined(HAVE_HIK_MVS)
    m_continuousRunning.store(false);
    if (!m_handle) {
        m_grabbing = false;
        m_connected = false;
        m_cameraIp.clear();
        m_deviceDisconnected.store(false);
        return;
    }

    if (m_grabbing) {
        const int stopCode = MV_CC_StopGrabbing(m_handle);
        if (reportErrors && stopCode != MV_OK) {
            emit log(formatSdkError(QStringLiteral("MV_CC_StopGrabbing"), stopCode));
        }
        m_grabbing = false;
    }

    const int closeCode = MV_CC_CloseDevice(m_handle);
    if (reportErrors && closeCode != MV_OK) {
        emit log(formatSdkError(QStringLiteral("MV_CC_CloseDevice"), closeCode));
    }

    const int destroyCode = MV_CC_DestroyHandle(m_handle);
    if (reportErrors && destroyCode != MV_OK) {
        emit log(formatSdkError(QStringLiteral("MV_CC_DestroyHandle"), destroyCode));
    }
#else
    Q_UNUSED(reportErrors)
#endif

    m_handle = nullptr;
    m_connected = false;
    m_grabbing = false;
    m_imagePool.reset();
    m_cameraIp.clear();
    m_deviceDisconnected.store(false);
}

void HikCameraWorker::connectCamera(QString ipAddress) {
    const QString requestedIp = ipAddress.trimmed();
    quint32 requestedIpValue = 0;
    if (!parseIpv4(requestedIp, &requestedIpValue)) {
        emit error(kConnectionRequestId,
                   QStringLiteral("海康相机 IP 格式无效: %1").arg(requestedIp));
        emit connectionChanged(false, QStringLiteral("相机 IP 无效"));
        return;
    }

    if (m_busy) {
        emit error(kConnectionRequestId, QStringLiteral("相机正在执行其他操作"));
        return;
    }
    if (m_connected && m_cameraIp == formatIpv4(requestedIpValue) &&
        !m_deviceDisconnected.load()) {
        emit connectionChanged(true,
                               QStringLiteral("海康相机已连接: %1").arg(m_cameraIp));
        return;
    }

    setBusy(true);

#if !defined(HAVE_HIK_MVS)
    emit error(kConnectionRequestId,
               QStringLiteral("当前程序未编译海康 MVS 支持（缺少 HAVE_HIK_MVS）"));
    emit connectionChanged(false, QStringLiteral("未编译 MVS 支持"));
#else
    releaseDevice(true);

    QString failure;
    QStringList discovered;
    MV_CC_DEVICE_INFO selected{};
    bool selectedFound = false;
    QString selectedModel;
    QString selectedSerial;
    qint64 configuredWidth = 0;
    qint64 configuredHeight = 0;

    do {
        if (!initializeSdk(&failure)) {
            break;
        }

        MV_CC_DEVICE_INFO_LIST devices{};
        const int enumCode = MV_CC_EnumDevices(MV_GIGE_DEVICE, &devices);
        if (enumCode != MV_OK) {
            failure = formatSdkError(QStringLiteral("MV_CC_EnumDevices(GigE)"), enumCode);
            break;
        }

        for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
            const MV_CC_DEVICE_INFO* device = devices.pDeviceInfo[i];
            if (!device || (device->nTLayerType != MV_GIGE_DEVICE &&
                            device->nTLayerType != MV_GENTL_GIGE_DEVICE &&
                            device->nTLayerType != MV_VIR_GIGE_DEVICE)) {
                continue;
            }

            const MV_GIGE_DEVICE_INFO& info = device->SpecialInfo.stGigEInfo;
            const QString currentIp = formatIpv4(info.nCurrentIp);
            const QString model = bytesToString(info.chModelName,
                                                 static_cast<int>(sizeof(info.chModelName)));
            const QString serial = bytesToString(info.chSerialNumber,
                                                  static_cast<int>(sizeof(info.chSerialNumber)));
            discovered << QStringLiteral("%1 (%2, SN=%3)")
                              .arg(currentIp, model, serial);
            if (info.nCurrentIp == requestedIpValue) {
                selected = *device; // SDK owns the enumeration list; keep our own copy.
                selectedFound = true;
                selectedModel = model;
                selectedSerial = serial;
            }
        }

        emit log(discovered.isEmpty()
                     ? QStringLiteral("未枚举到 GigE 相机")
                     : QStringLiteral("枚举到 GigE 相机: %1").arg(discovered.join(QStringLiteral("; "))));

        if (!selectedFound) {
            failure = QStringLiteral("未找到 IP 为 %1 的海康 GigE 相机。当前设备: %2")
                          .arg(requestedIp,
                               discovered.isEmpty() ? QStringLiteral("无")
                                                    : discovered.join(QStringLiteral("; ")));
            break;
        }

        if (!MV_CC_IsDeviceAccessible(&selected, MV_ACCESS_Exclusive)) {
            failure = QStringLiteral(
                "相机 %1 当前不能独占打开。请先在 MVS 中停止取流、关闭设备并退出 MVS，"
                "同时确认没有其他采集程序占用相机。").arg(requestedIp);
            break;
        }

        int code = MV_CC_CreateHandle(&m_handle, &selected);
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("MV_CC_CreateHandle"), code);
            break;
        }

        code = MV_CC_OpenDevice(m_handle, MV_ACCESS_Exclusive, 0);
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("MV_CC_OpenDevice"), code);
            if (static_cast<quint32>(code) == MV_E_ACCESS_DENIED ||
                static_cast<quint32>(code) == MV_E_BUSY) {
                failure += QStringLiteral("；请关闭 MVS 或其他占用该相机的程序后重试");
            }
            break;
        }

        code = MV_CC_RegisterExceptionCallBack(m_handle,
                                                &HikCameraWorker::onSdkException,
                                                this);
        if (code != MV_OK) {
            emit log(formatSdkError(QStringLiteral("MV_CC_RegisterExceptionCallBack"), code));
        }

        // Calibration uses a fixed full-sensor geometry.  Reset scaling and
        // ROI before reading any frame so a stale MVS setting cannot silently
        // change the principal point between calibration and scanning.
        const char* scalingNodes[] = {
            "BinningHorizontal", "BinningVertical",
            "DecimationHorizontal", "DecimationVertical"
        };
        for (std::size_t index = 0;
             index < sizeof(scalingNodes) / sizeof(scalingNodes[0]); ++index) {
            const int scalingCode = MV_CC_SetEnumValue(m_handle, scalingNodes[index], 1);
            if (scalingCode != MV_OK &&
                static_cast<quint32>(scalingCode) != MV_E_GC_ACCESS &&
                static_cast<quint32>(scalingCode) != MV_E_SUPPORT) {
                emit log(formatSdkError(
                    QStringLiteral("设置 %1=1").arg(QString::fromLatin1(scalingNodes[index])),
                    scalingCode));
            }
        }

        const char* offsetNodes[] = {"OffsetX", "OffsetY"};
        for (std::size_t index = 0;
             index < sizeof(offsetNodes) / sizeof(offsetNodes[0]); ++index) {
            MVCC_INTVALUE_EX value{};
            code = MV_CC_GetIntValueEx(m_handle, offsetNodes[index], &value);
            if (code != MV_OK) {
                failure = formatSdkError(
                    QStringLiteral("读取 %1").arg(QString::fromLatin1(offsetNodes[index])), code);
                break;
            }
            code = MV_CC_SetIntValueEx(m_handle, offsetNodes[index], value.nMin);
            if (code != MV_OK) {
                failure = formatSdkError(
                    QStringLiteral("设置 %1=%2")
                        .arg(QString::fromLatin1(offsetNodes[index]))
                        .arg(value.nMin), code);
                break;
            }
        }
        if (!failure.isEmpty()) {
            break;
        }
        const char* sizeNodes[] = {"Width", "Height"};
        for (std::size_t index = 0;
             index < sizeof(sizeNodes) / sizeof(sizeNodes[0]); ++index) {
            MVCC_INTVALUE_EX value{};
            code = MV_CC_GetIntValueEx(m_handle, sizeNodes[index], &value);
            if (code != MV_OK) {
                failure = formatSdkError(
                    QStringLiteral("读取 %1").arg(QString::fromLatin1(sizeNodes[index])), code);
                break;
            }
            code = MV_CC_SetIntValueEx(m_handle, sizeNodes[index], value.nMax);
            if (code != MV_OK) {
                failure = formatSdkError(
                    QStringLiteral("设置 %1=%2")
                        .arg(QString::fromLatin1(sizeNodes[index]))
                        .arg(value.nMax), code);
                break;
            }
            if (index == 0) {
                configuredWidth = value.nMax;
            } else {
                configuredHeight = value.nMax;
            }
        }
        if (!failure.isEmpty()) {
            break;
        }

        const int packetSize = MV_CC_GetOptimalPacketSize(m_handle);
        if (packetSize > 0) {
            code = MV_CC_SetIntValueEx(m_handle, "GevSCPSPacketSize", packetSize);
            if (code != MV_OK) {
                emit log(formatSdkError(QStringLiteral("设置 GigE 最佳包长"), code));
            } else {
                emit log(QStringLiteral("GigE packet size=%1").arg(packetSize));
            }
        } else {
            emit log(QStringLiteral("未能探测 GigE 最佳包长，保留相机当前设置"));
        }

        code = MV_CC_SetEnumValueByString(m_handle, "PixelFormat", "Mono8");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 PixelFormat=Mono8"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "AcquisitionMode", "Continuous");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 AcquisitionMode=Continuous"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "TriggerMode", "On");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 TriggerMode=On"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "TriggerSource", "Software");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 TriggerSource=Software"), code);
            break;
        }

        code = MV_CC_SetImageNodeNum(m_handle, 8);
        if (code != MV_OK) {
            emit log(formatSdkError(QStringLiteral("设置 SDK 图像缓存节点数=8"), code));
        }

        m_connected = true;
        m_cameraIp = formatIpv4(requestedIpValue);
        m_deviceDisconnected.store(false);
    } while (false);

    if (!failure.isEmpty()) {
        releaseDevice(false);
        emit error(kConnectionRequestId, failure);
        emit connectionChanged(false, failure);
    } else {
        const QString description = QStringLiteral(
            "海康相机已连接: %1，%2，SN=%3，Mono8，%4x%5 全幅，软件触发")
                .arg(m_cameraIp, selectedModel, selectedSerial)
                .arg(configuredWidth).arg(configuredHeight);
        emit log(description);
        emit identityChanged(selectedModel, selectedSerial, m_cameraIp);
        emit connectionChanged(true, description);
    }
#endif

    setBusy(false);
}

void HikCameraWorker::disconnectCamera() {
    if (m_busy && !m_continuousRunning.load()) {
        emit error(kConnectionRequestId, QStringLiteral("相机正在执行其他操作，暂不能断开"));
        return;
    }

    setBusy(true);
    const QString oldIp = m_cameraIp;
    releaseDevice(true);
    const QString description = oldIp.isEmpty()
        ? QStringLiteral("海康相机未连接")
        : QStringLiteral("海康相机已断开: %1").arg(oldIp);
    emit log(description);
    emit connectionChanged(false, description);
    setBusy(false);
}

void HikCameraWorker::captureSingle(int requestId,
                                    double exposureUs,
                                    double gainDb,
                                    int timeoutMs) {
    if (m_busy) {
        emit error(requestId, QStringLiteral("相机正在执行其他操作"));
        return;
    }
    if (!m_connected || !m_handle) {
        emit error(requestId, QStringLiteral("海康相机未连接"));
        return;
    }
    if (!std::isfinite(exposureUs) || !std::isfinite(gainDb) ||
        exposureUs <= 0.0 || timeoutMs < 1 || timeoutMs > 10000) {
        emit error(requestId,
                   QStringLiteral("采集参数无效: exposure=%1 us, gain=%2 dB, timeout=%3 ms")
                       .arg(exposureUs, 0, 'f', 3)
                       .arg(gainDb, 0, 'f', 3)
                       .arg(timeoutMs));
        return;
    }
    if (m_deviceDisconnected.load()) {
        releaseDevice(false);
        const QString message = QStringLiteral("海康相机网络连接已断开，请重新连接");
        emit error(requestId, message);
        emit connectionChanged(false, message);
        return;
    }

    setBusy(true);

#if !defined(HAVE_HIK_MVS)
    emit error(requestId,
               QStringLiteral("当前程序未编译海康 MVS 支持（缺少 HAVE_HIK_MVS）"));
#else
    QString failure;
    QImage image;
    quint64 frameNumber = 0;
    quint64 deviceTimestamp = 0;
    qint64 hostTimestamp = 0;
    double actualExposure = exposureUs;
    double actualGain = gainDb;
    unsigned int lostPackets = 0;

    do {
        int code = MV_CC_SetEnumValueByString(m_handle, "TriggerMode", "On");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 TriggerMode=On"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "TriggerSource", "Software");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 TriggerSource=Software"), code);
            break;
        }
        MVCC_FLOATVALUE exposureRange{};
        code = MV_CC_GetFloatValue(m_handle, "ExposureTime", &exposureRange);
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("读取 ExposureTime 范围"), code);
            break;
        }
        if (exposureUs < exposureRange.fMin || exposureUs > exposureRange.fMax) {
            failure = QStringLiteral("曝光 %1 us 超出相机范围 [%2, %3] us")
                          .arg(exposureUs, 0, 'f', 3)
                          .arg(exposureRange.fMin, 0, 'f', 3)
                          .arg(exposureRange.fMax, 0, 'f', 3);
            break;
        }

        MVCC_FLOATVALUE gainRange{};
        code = MV_CC_GetFloatValue(m_handle, "Gain", &gainRange);
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("读取 Gain 范围"), code);
            break;
        }
        if (gainDb < gainRange.fMin || gainDb > gainRange.fMax) {
            failure = QStringLiteral("增益 %1 dB 超出相机范围 [%2, %3] dB")
                          .arg(gainDb, 0, 'f', 3)
                          .arg(gainRange.fMin, 0, 'f', 3)
                          .arg(gainRange.fMax, 0, 'f', 3);
            break;
        }

        code = MV_CC_SetEnumValueByString(m_handle, "ExposureAuto", "Off");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("关闭自动曝光"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "GainAuto", "Off");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("关闭自动增益"), code);
            break;
        }
        code = MV_CC_SetFloatValue(m_handle, "ExposureTime", static_cast<float>(exposureUs));
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 ExposureTime"), code);
            break;
        }
        code = MV_CC_SetFloatValue(m_handle, "Gain", static_cast<float>(gainDb));
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 Gain"), code);
            break;
        }

        MVCC_FLOATVALUE actualValue{};
        if (MV_CC_GetFloatValue(m_handle, "ExposureTime", &actualValue) == MV_OK) {
            actualExposure = actualValue.fCurValue;
        }
        std::memset(&actualValue, 0, sizeof(actualValue));
        if (MV_CC_GetFloatValue(m_handle, "Gain", &actualValue) == MV_OK) {
            actualGain = actualValue.fCurValue;
        }

        code = MV_CC_StartGrabbing(m_handle);
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("MV_CC_StartGrabbing"), code);
            break;
        }
        m_grabbing = true;

        // Clearing after StartGrabbing is supported by MVS and prevents a
        // frame left from an earlier acquisition from satisfying this request.
        code = MV_CC_ClearImageBuffer(m_handle);
        if (code != MV_OK) {
            emit log(formatSdkError(QStringLiteral("清空旧图像缓存"), code));
        }

        code = MV_CC_SetCommandValue(m_handle, "TriggerSoftware");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("TriggerSoftware"), code);
            break;
        }

        MV_FRAME_OUT frame{};
        FrameBufferGuard frameGuard(m_handle, &frame);
        code = MV_CC_GetImageBuffer(m_handle, &frame,
                                    static_cast<unsigned int>(timeoutMs));
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("等待软件触发图像"), code);
            if (static_cast<quint32>(code) == MV_E_NODATA ||
                static_cast<quint32>(code) == MV_E_GC_TIMEOUT) {
                failure += QStringLiteral("；%1 ms 内未收到图像，请检查触发模式、网线和曝光设置")
                               .arg(timeoutMs);
            }
            break;
        }
        frameGuard.acquired();

        const MV_FRAME_OUT_INFO_EX& info = frame.stFrameInfo;
        const unsigned int width = info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
        const unsigned int height = info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;
        const quint64 frameLength = info.nFrameLenEx != 0 ? info.nFrameLenEx : info.nFrameLen;
        const quint64 requiredLength = static_cast<quint64>(width) * height;

        if (!frame.pBufAddr || width == 0 || height == 0) {
            failure = QStringLiteral("MVS 返回了空图像");
            break;
        }
        if (info.enPixelType != PixelType_Gvsp_Mono8) {
            failure = QStringLiteral("收到非 Mono8 图像，pixelType=0x%1")
                          .arg(QString::number(static_cast<quint32>(info.enPixelType), 16));
            break;
        }
        if (frameLength < requiredLength) {
            failure = QStringLiteral("Mono8 图像长度不足: got=%1, need=%2")
                          .arg(frameLength)
                          .arg(requiredLength);
            break;
        }
        if (width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
            height > static_cast<unsigned int>(std::numeric_limits<int>::max())) {
            failure = QStringLiteral("图像尺寸超出 Qt 支持范围: %1x%2")
                          .arg(width).arg(height);
            break;
        }

        image = QImage(static_cast<int>(width),
                       static_cast<int>(height),
                       QImage::Format_Grayscale8);
        if (image.isNull()) {
            failure = QStringLiteral("无法为 Mono8 图像分配内存");
            break;
        }
        for (unsigned int y = 0; y < height; ++y) {
            std::memcpy(image.scanLine(static_cast<int>(y)),
                        frame.pBufAddr + static_cast<quint64>(y) * width,
                        width);
        }

        frameNumber = info.nFrameNum;
        deviceTimestamp = (static_cast<quint64>(info.nDevTimeStampHigh) << 32U) |
                          static_cast<quint64>(info.nDevTimeStampLow);
        hostTimestamp = info.nHostTimeStamp;
        lostPackets = info.nLostPacket;

        // The QImage is already independent; return the SDK buffer before any
        // queued signal can hand the image to the GUI thread.
        const int freeCode = frameGuard.releaseNow();
        if (freeCode != MV_OK) {
            failure = formatSdkError(QStringLiteral("MV_CC_FreeImageBuffer"), freeCode);
            image = {};
            break;
        }

        if (lostPackets != 0) {
            failure = QStringLiteral("本帧检测到 %1 个丢失网络包，已拒绝用于标定/点云")
                          .arg(lostPackets);
            image = {};
            break;
        }
    } while (false);

    if (m_grabbing) {
        const int stopCode = MV_CC_StopGrabbing(m_handle);
        if (stopCode != MV_OK && failure.isEmpty()) {
            failure = formatSdkError(QStringLiteral("MV_CC_StopGrabbing"), stopCode);
            image = {};
        } else if (stopCode != MV_OK) {
            emit log(formatSdkError(QStringLiteral("MV_CC_StopGrabbing"), stopCode));
        }
        m_grabbing = false;
    }

    const bool connectionLost = m_deviceDisconnected.load();
    if (connectionLost) {
        failure = QStringLiteral("采集期间海康相机网络连接断开，请重新连接");
        image = {};
    }

    if (!failure.isEmpty()) {
        emit error(requestId, failure);
    } else {
        const QString description = QStringLiteral(
            "Mono8 %1x%2, frame=%3, exposure=%4 us, gain=%5 dB")
                .arg(image.width())
                .arg(image.height())
                .arg(frameNumber)
                .arg(actualExposure, 0, 'f', 3)
                .arg(actualGain, 0, 'f', 3);
        emit log(QStringLiteral("单帧采集成功: %1").arg(description));
        emit frameReady(requestId, image, frameNumber, deviceTimestamp,
                        hostTimestamp, actualExposure, actualGain, description);
    }

    if (connectionLost) {
        releaseDevice(false);
        emit connectionChanged(false, QStringLiteral("海康相机网络连接已断开"));
    }
#endif

    setBusy(false);
}

void HikCameraWorker::startContinuous(double exposureUs,
                                      double gainDb,
                                      double targetFps,
                                      int poolCapacity) {
    if (m_busy || m_continuousRunning.load()) {
        emit error(kConnectionRequestId, QStringLiteral("相机已在采集或执行其他操作"));
        return;
    }
    if (!m_connected || !m_handle) {
        emit error(kConnectionRequestId, QStringLiteral("海康相机未连接"));
        return;
    }
    if (!std::isfinite(exposureUs) || exposureUs <= 0.0 ||
        !std::isfinite(gainDb) || !std::isfinite(targetFps) || targetFps <= 0.0 ||
        poolCapacity < 2 || poolCapacity > 4096) {
        emit error(kConnectionRequestId, QStringLiteral(
            "连续采集参数无效: exposure=%1 us, gain=%2 dB, fps=%3, pool=%4")
            .arg(exposureUs, 0, 'f', 3).arg(gainDb, 0, 'f', 3)
            .arg(targetFps, 0, 'f', 3).arg(poolCapacity));
        return;
    }
    setBusy(true);

#if !defined(HAVE_HIK_MVS)
    emit error(kConnectionRequestId,
               QStringLiteral("当前程序未编译海康 MVS 支持（缺少 HAVE_HIK_MVS）"));
    setBusy(false);
#else
    QString failure;
    double actualExposure = exposureUs;
    double actualFps = targetFps;
    quint64 timestampFrequency = 0U;
    do {
        int code = MV_CC_SetEnumValueByString(m_handle, "ExposureAuto", "Off");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("关闭自动曝光"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "GainAuto", "Off");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("关闭自动增益"), code);
            break;
        }
        code = MV_CC_SetEnumValueByString(m_handle, "TriggerMode", "Off");
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("设置 TriggerMode=Off"), code);
            break;
        }
        MVCC_FLOATVALUE exposureRange{};
        code = MV_CC_GetFloatValue(m_handle, "ExposureTime", &exposureRange);
        if (code != MV_OK || exposureUs < exposureRange.fMin ||
            exposureUs > exposureRange.fMax) {
            failure = code == MV_OK
                ? QStringLiteral("曝光 %1 us 超出相机范围 [%2, %3] us")
                      .arg(exposureUs, 0, 'f', 3)
                      .arg(exposureRange.fMin, 0, 'f', 3)
                      .arg(exposureRange.fMax, 0, 'f', 3)
                : formatSdkError(QStringLiteral("读取 ExposureTime 范围"), code);
            break;
        }
        MVCC_FLOATVALUE gainRange{};
        code = MV_CC_GetFloatValue(m_handle, "Gain", &gainRange);
        if (code != MV_OK || gainDb < gainRange.fMin || gainDb > gainRange.fMax) {
            failure = code == MV_OK
                ? QStringLiteral("增益 %1 dB 超出相机范围 [%2, %3] dB")
                      .arg(gainDb, 0, 'f', 3)
                      .arg(gainRange.fMin, 0, 'f', 3)
                      .arg(gainRange.fMax, 0, 'f', 3)
                : formatSdkError(QStringLiteral("读取 Gain 范围"), code);
            break;
        }
        MVCC_FLOATVALUE fpsRange{};
        code = MV_CC_GetFloatValue(m_handle, "AcquisitionFrameRate", &fpsRange);
        if (code != MV_OK || targetFps < fpsRange.fMin || targetFps > fpsRange.fMax) {
            failure = code == MV_OK
                ? QStringLiteral("帧率 %1 fps 超出相机当前范围 [%2, %3] fps")
                      .arg(targetFps, 0, 'f', 3)
                      .arg(fpsRange.fMin, 0, 'f', 3)
                      .arg(fpsRange.fMax, 0, 'f', 3)
                : formatSdkError(QStringLiteral("读取 AcquisitionFrameRate 范围"), code);
            break;
        }
        code = MV_CC_SetFloatValue(m_handle, "ExposureTime", static_cast<float>(exposureUs));
        if (code == MV_OK) code = MV_CC_SetFloatValue(m_handle, "Gain", static_cast<float>(gainDb));
        if (code == MV_OK) code = MV_CC_SetBoolValue(m_handle, "AcquisitionFrameRateEnable", true);
        if (code == MV_OK) {
            code = MV_CC_SetFloatValue(m_handle, "AcquisitionFrameRate",
                                       static_cast<float>(targetFps));
        }
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("配置自由运行曝光/增益/60fps"), code);
            break;
        }
        MVCC_FLOATVALUE actualValue{};
        if (MV_CC_GetFloatValue(m_handle, "ExposureTime", &actualValue) == MV_OK) {
            actualExposure = actualValue.fCurValue;
        }
        std::memset(&actualValue, 0, sizeof(actualValue));
        if (MV_CC_GetFloatValue(m_handle, "AcquisitionFrameRate", &actualValue) == MV_OK) {
            actualFps = actualValue.fCurValue;
        }

        // GigE Vision exposes device timestamp ticks through this GenICam
        // integer node when supported. The MVS frame struct itself documents
        // only raw high/low words and does not define their unit.
        MVCC_INTVALUE_EX frequencyValue{};
        const int frequencyCode = MV_CC_GetIntValueEx(
            m_handle, "GevTimestampTickFrequency", &frequencyValue);
        if (frequencyCode == MV_OK && frequencyValue.nCurValue > 0) {
            timestampFrequency = static_cast<quint64>(frequencyValue.nCurValue);
        } else {
            emit log(QStringLiteral(
                "相机不提供可读 GevTimestampTickFrequency，设备时间戳单位无法确认；将使用 HOST_CALLBACK_FALLBACK。"));
        }

        code = MV_CC_SetImageNodeNum(m_handle, 16U);
        if (code != MV_OK) {
            emit log(formatSdkError(QStringLiteral("设置连续采集 SDK 缓存节点数=16"), code));
        }
        m_imagePool = std::make_unique<hik_sync::ImageBufferPool>(
            static_cast<std::size_t>(poolCapacity));
        m_continuousExposureUs = actualExposure;
        m_continuousFps = actualFps;
        m_deviceTimestampFrequencyHz = timestampFrequency;
        code = MV_CC_RegisterImageCallBackEx(
            m_handle, &HikCameraWorker::onContinuousImage, this);
        if (code != MV_OK) {
            failure = formatSdkError(QStringLiteral("MV_CC_RegisterImageCallBackEx"), code);
            break;
        }
        m_continuousRunning.store(true);
        code = MV_CC_StartGrabbing(m_handle);
        if (code != MV_OK) {
            m_continuousRunning.store(false);
            failure = formatSdkError(QStringLiteral("MV_CC_StartGrabbing"), code);
            break;
        }
        m_grabbing = true;
    } while (false);

    if (!failure.isEmpty()) {
        m_continuousRunning.store(false);
        m_imagePool.reset();
        emit error(kConnectionRequestId, failure);
        setBusy(false);
        return;
    }
    const QString timestampDescription = timestampFrequency > 0U
        ? QStringLiteral("DEVICE_TIMESTAMP_MAPPING, tick_frequency=%1 Hz")
              .arg(timestampFrequency)
        : QStringLiteral("HOST_CALLBACK_FALLBACK");
    emit log(QStringLiteral(
        "自由运行连续采集已启动: Mono8, exposure=%1 us, fps=%2, %3")
        .arg(actualExposure, 0, 'f', 3).arg(actualFps, 0, 'f', 3)
        .arg(timestampDescription));
    emit continuousStarted(actualExposure, actualFps, timestampFrequency,
                           timestampDescription);
#endif
}

void HikCameraWorker::stopContinuous() {
#if !defined(HAVE_HIK_MVS)
    setBusy(false);
    emit continuousStopped(
        true, QStringLiteral("未编译 MVS 支持；无活动连续取流"));
#else
    const bool wasRunning = m_continuousRunning.exchange(false);
    bool stopConfirmed = true;
    QString stopDescription =
        QStringLiteral("自由运行连续采集已停止");
    if (m_grabbing && m_handle) {
        const int code = MV_CC_StopGrabbing(m_handle);
        if (code != MV_OK) {
            stopConfirmed = false;
            stopDescription = formatSdkError(
                QStringLiteral("MV_CC_StopGrabbing"), code);
            emit log(stopDescription);
        } else {
            m_grabbing = false;
        }
    }
    m_imagePool.reset();
    if (wasRunning && stopConfirmed) emit log(stopDescription);
    setBusy(false);
    emit continuousStopped(stopConfirmed, stopDescription);
#endif
}

#if defined(HAVE_HIK_MVS)
void HikCameraWorker::onContinuousImage(unsigned char* data,
                                        MV_FRAME_OUT_INFO_EX* frameInfo,
                                        void* userData) {
    HikCameraWorker* self = static_cast<HikCameraWorker*>(userData);
    if (!self || !frameInfo) return;
    self->handleContinuousImage(data, *frameInfo);
}

void HikCameraWorker::handleContinuousImage(
        const unsigned char* data, const MV_FRAME_OUT_INFO_EX& info) {
    const int64_t callbackNs = hik_sync::getMonotonicRawNs();
    if (!m_continuousRunning.load()) return;
    const unsigned int width = info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
    const unsigned int height = info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;
    const quint64 frameLength = info.nFrameLenEx != 0 ? info.nFrameLenEx : info.nFrameLen;
    const quint64 requiredLength = static_cast<quint64>(width) * height;
    if (!data || width == 0U || height == 0U || frameLength < requiredLength ||
        info.enPixelType != PixelType_Gvsp_Mono8 || info.nLostPacket != 0U) {
        emit continuousFrameRejected(info.nFrameNum,
            info.nLostPacket != 0U
                ? QStringLiteral("帧含 %1 个丢失网络包").arg(info.nLostPacket)
                : QStringLiteral("帧为空、长度不足或不是 Mono8"));
        return;
    }
    if (width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
        height > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
        !m_imagePool) {
        emit continuousFrameRejected(info.nFrameNum,
                                     QStringLiteral("图像尺寸或内存池无效"));
        return;
    }
    std::shared_ptr<hik_sync::ImageBuffer> image = m_imagePool->acquire(
        static_cast<std::size_t>(requiredLength));
    if (!image) {
        emit imagePoolExhausted();
        return;
    }
    image->width = static_cast<int>(width);
    image->height = static_cast<int>(height);
    image->stride = static_cast<int>(width);
    image->pixelFormat = static_cast<int>(info.enPixelType);
    std::memcpy(image->bytes.data(), data, static_cast<std::size_t>(requiredLength));

    hik_sync::CameraFrame frame;
    frame.frameId = info.nFrameNum;
    frame.cameraTimestampRaw =
        (static_cast<quint64>(info.nDevTimeStampHigh) << 32U) |
        static_cast<quint64>(info.nDevTimeStampLow);
    frame.cameraTimestampFrequencyHz = m_deviceTimestampFrequencyHz;
    frame.cameraTimestampNs = hik_sync::cameraTicksToNs(
        frame.cameraTimestampRaw, frame.cameraTimestampFrequencyHz,
        &frame.timestampValid);
    frame.hostCallbackNs = callbackNs;
    frame.exposureUs = info.fExposureTime > 0.0F
        ? static_cast<double>(info.fExposureTime) : m_continuousExposureUs;
    frame.width = image->width;
    frame.height = image->height;
    frame.pixelFormat = image->pixelFormat;
    frame.image = std::move(image);
    emit continuousFrameReady(std::move(frame));
}
#endif
