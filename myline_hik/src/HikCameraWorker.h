#ifndef MYLINE_HIK_HIK_CAMERA_WORKER_H
#define MYLINE_HIK_HIK_CAMERA_WORKER_H

#include <QImage>
#include <QObject>
#include <QString>
#include <QtGlobal>

#include <atomic>

// Single-owner wrapper for one Hikrobot MVS camera.
//
// Move this object to a dedicated QThread and invoke its slots with queued
// connections.  All SDK calls for the camera handle are deliberately kept in
// that one thread.  captureSingle() blocks only the worker thread for at most
// timeoutMs; it never exposes an SDK-owned image buffer to another thread.
class HikCameraWorker final : public QObject {
    Q_OBJECT

public:
    explicit HikCameraWorker(QObject* parent = nullptr);
    ~HikCameraWorker() override;

public slots:
    // Enumerates GigE cameras and opens only the camera whose current IP is an
    // exact match.  No index/first-camera fallback is used.
    void connectCamera(QString ipAddress);
    void disconnectCamera();

    // Performs one software-triggered Mono8 acquisition. Exposure is in us,
    // gain is in dB, and timeout is in ms. The emitted QImage owns a deep copy
    // of the frame. deviceTimestamp and hostTimestamp are the raw values from
    // MV_FRAME_OUT_INFO_EX (the SDK does not document hostTimestamp's unit).
    void captureSingle(int requestId, double exposureUs, double gainDb, int timeoutMs);

signals:
    void connectionChanged(bool connected, QString description);
    void identityChanged(QString model, QString serial, QString ipAddress);
    void busyChanged(bool busy);
    void frameReady(int requestId,
                    QImage image,
                    quint64 frameNo,
                    quint64 deviceTimestamp,
                    qint64 hostTimestamp,
                    double actualExposure,
                    double actualGain,
                    QString description);
    void log(QString message);
    // requestId is -1 for connection/lifetime errors.
    void error(int requestId, QString message);

private:
    Q_DISABLE_COPY(HikCameraWorker)

    void setBusy(bool busy);
    void releaseDevice(bool reportErrors);
    static bool parseIpv4(const QString& text, quint32* value);
    static QString formatIpv4(quint32 value);

#if defined(HAVE_HIK_MVS)
    bool initializeSdk(QString* failure);
    static QString formatSdkError(const QString& operation, int code);
    static QString sdkErrorName(int code);
    static void onSdkException(unsigned int messageType, void* userData);
#endif

    void* m_handle;
    bool m_sdkInitialized;
    bool m_connected;
    bool m_grabbing;
    bool m_busy;
    QString m_cameraIp;
    std::atomic<bool> m_deviceDisconnected;
};

#endif // MYLINE_HIK_HIK_CAMERA_WORKER_H
