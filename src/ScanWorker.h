#ifndef SCANWORKER_H
#define SCANWORKER_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QImage>
#include <atomic>
#include <queue>
#include <vector>

class QTextStream;

#include "VZNL_Common.h"
#include "VZNL_EyeConfig.h"
#include "VZNL_DetectLaser.h"
#include "VZNL_DetectConfig.h"
#include "VZNL_DustCover.h"
#include "VZNL_RGBConfig.h"
#include "VZNL_SwingMotor.h"
#include "VZNL_FileUtils.h"
#include "VZNL_Graphics.h"
#include "VZNL_Utils.h"
#include "VZNL_Types.h"

// Owned copy of one laser line (deep-copied from the callback)
struct OwnedLaserLine {
    OwnedLaserLine();
    ~OwnedLaserLine();
    OwnedLaserLine(OwnedLaserLine&& other) noexcept;
    OwnedLaserLine& operator=(OwnedLaserLine&& other) noexcept;
    OwnedLaserLine(const OwnedLaserLine&) = delete;
    OwnedLaserLine& operator=(const OwnedLaserLine&) = delete;

    void reset();

    SVzLaserLineData data;   // p2DPoint / p3DPoint are owned (new[]) by us
};

class ScanWorker : public QObject {
    Q_OBJECT
public:
    explicit ScanWorker(QObject* parent = nullptr);
    ~ScanWorker() override;

public slots:
    // --- Lifetime / connection
    void initSdk();           // VzNL_Init + log level
    void connectDevice();     // research + bind + open + begin laser tool + setup motor
    void disconnectDevice();  // end laser + close
    void rebootDevice();      // reboot via SDK
    void destroySdk();        // VzNL_Destroy
    void getVersions();       // VzNL_GetVersion + device/module versions

    // --- Cover
    void openCover();         // VzNL_CoverCamera(false)
    void closeCover();        // VzNL_CoverCamera(true)

    // --- RGB
    void grabRgbImage();      // one RGB frame -> QImage

    // --- Scan
    void startScanAndSave(QString filePath);  // one full scan -> .ply

signals:
    void log(QString msg);
    void connectionChanged(bool connected);
    void scanFinished(bool ok, QString filePath);
    void busyChanged(bool busy);
    void rgbImageReady(QImage image, QString desc);

private:
    // SDK callback: must be free of SDK calls; only enqueues a deep copy.
    static void s_onLaserLineCB(EVzResultDataType eType, SVzLaserLineData* pLine, void* pParam);
    static void s_onDeviceStatusCB(EVzDeviceWorkStatus eStatus, void* pInfoParam);

    void onLaserLineCB(EVzResultDataType eType, SVzLaserLineData* pLine);
    void onDeviceStatusCB(EVzDeviceWorkStatus eStatus);

    void clearQueueLocked();   // assume m_queueMutex held
    void clearQueue();
    bool ensureClosedDetect(); // safe stop if currently detecting
    bool drainQueueToFile(VZNLFILE hFile, QTextStream* csv, double unitScale,
                          int& totalLines, int& totalPts);
    void writeCsvHeader(QTextStream& csv);
    void writeLaserLineCsv(QTextStream& csv, const SVzLaserLineData& line,
                           double unitScale, int lineIndex);

private:
    VZNLHANDLE m_hDevice = nullptr;
    bool m_sdkInitialized = false;
    bool m_laserToolBegun = false;
    bool m_supportMotor = false;

    // Scan state
    std::atomic<bool> m_scanRunning{false};
    std::atomic<bool> m_autoStopReceived{false};
    std::atomic<bool> m_queueOverflow{false};

    // Captured laser lines (deep copies)
    QMutex m_queueMutex;
    std::queue<OwnedLaserLine> m_queue;
    const unsigned int m_maxQueuedLines = 4096;
};

#endif // SCANWORKER_H
