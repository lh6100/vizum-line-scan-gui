#ifndef SCANWORKER_H
#define SCANWORKER_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QImage>
#include <QPoint>
#include <QVector>
#include <QVector3D>
#include <atomic>
#include <queue>
#include <vector>

class QTextStream;

#include "VZNL_Common.h"
#include "VZNL_EyeConfig.h"
#include "VZNL_DetectLaser.h"
#include "VZNL_DetectConfig.h"
#include "VZNL_DustCover.h"
#include "VZNL_ExtStrobeLaser.h"
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
    void softResetDevice();   // stop stream + reset swing/reconnect + rebuild laser tool
    void rebootDevice();      // reboot via SDK
    void destroySdk();        // VzNL_Destroy
    void getVersions();       // VzNL_GetVersion + device/module versions

    // --- Cover
    void openCover();         // VzNL_CoverCamera(false)
    void closeCover();        // VzNL_CoverCamera(true)

    // --- RGB
    void grabRgbImage();      // one RGB frame -> QImage
    void grabLeftEyeImage();  // one left-eye grayscale frame -> QImage
    void grabLeftEyeImageWithParams(int frameRate, int exposure, int gain, bool keepLaserOn);
    void calibrateLeftEyeCharuco(); // left-eye image + SDK intrinsics -> board pose
    void calibrateLeftEyeCharucoWithParams(int frameRate, int exposure, int gain, int squaresX, int squaresY,
                                           double squareMm, double markerMm, QString dictionaryName,
                                           bool keepLaserOn);
    void loadVizumConfig(QString filePath);
    void mapRgbLineTo3D(QPoint start, QPoint end, int sampleCount);

    // --- Scan
    void startScanAndSave(QString filePath);  // one full scan -> .ply

signals:
    void log(QString msg);
    void connectionChanged(bool connected);
    void scanFinished(bool ok, QString filePath);
    void busyChanged(bool busy);
    void rgbImageReady(QImage image, QString desc);
    void leftEyeImageReady(QImage image, QString desc);
    void leftEyeCharucoResult(bool ok, QImage image, QString desc,
                              QVector<double> tLeftBoard, QString report);
    void rgbLineMapped(QVector<QVector3D> points, QString desc);

private:
    // SDK callback: must be free of SDK calls; only enqueues a deep copy.
    static void s_onLaserLineCB(EVzResultDataType eType, SVzLaserLineData* pLine, void* pParam);
    static void s_onDeviceStatusCB(EVzDeviceWorkStatus eStatus, void* pInfoParam);

    void onLaserLineCB(EVzResultDataType eType, SVzLaserLineData* pLine);
    void onDeviceStatusCB(EVzDeviceWorkStatus eStatus);

    void clearQueueLocked();   // assume m_queueMutex held
    void clearQueue();
    bool ensureClosedDetect(); // safe stop if currently detecting
    bool configureDeviceRuntime(); // RGB/notify/laser tool/trigger/motor setup
    bool waitForDeviceReady(int timeoutMs);
    bool moveSwingToStartForScan();
    bool ensureCoverOpenForScan();
    void ensureLaserLightEnabled();
    bool configurePointCloudProcMode();
    void logScanRuntimeState(QString context);
    bool configureFullEyeRoiForCalibration(bool useCalibImage);
    void configureEyeCalibrationRuntime();
    bool drainQueueToFile(VZNLFILE hFile, QTextStream* csv, double unitScale,
                          int& totalLines, int& totalPts);
    void writeCsvHeader(QTextStream& csv);
    void writeLaserLineCsv(QTextStream& csv, const SVzLaserLineData& line,
                           double unitScale, int lineIndex);
    bool beginRgbCloudMapping();
    void finishRgbCloudMapping();
    void destroyRgbCloudMapping();
    bool findMapped3DNearPixel(const QPoint& pixel, int searchRadius, QVector3D& point) const;
    bool configureLeftEyeImaging(int frameRate, int exposure, int gain);
    bool captureLeftEyeImage(QImage& image, QString& desc, int frameRate = -1,
                             int exposure = -1, int gain = -1, bool keepLaserOn = false,
                             bool useCalibImage = false);
    void calibrateLeftEyeCharucoImpl(int frameRate, int exposure, int gain, int squaresX, int squaresY,
                                     double squareMm, double markerMm, QString dictionaryName,
                                     bool emitStructuredResult, bool keepLaserOn);

private:
    VZNLHANDLE m_hDevice = nullptr;
    bool m_sdkInitialized = false;
    bool m_laserToolBegun = false;
    bool m_supportMotor = false;
    VZNLRGBCLOUDPOINTTOOL m_rgbCloudTool = nullptr;
    bool m_rgbCloudToolBuilding = false;
    bool m_rgbCloudToolReady = false;
    bool m_rgbCloudPushFailed = false;
    bool m_hasLoadedPointCloudProcMode = false;
    EVzPointCloudProcMode m_loadedPointCloudProcMode = kePointCloudProcMode_Invalid;
    double m_loadedFixedStep = 0.0;

    // Scan state
    std::atomic<bool> m_scanRunning{false};
    std::atomic<bool> m_autoStopReceived{false};
    std::atomic<bool> m_queueOverflow{false};
    std::atomic<int> m_cbTotalLines{0};
    std::atomic<int> m_cbAcceptedLines{0};
    std::atomic<int> m_cbNonEmptyLines{0};
    std::atomic<int> m_cbEndSignals{0};
    std::atomic<int> m_cbLastType{-1};
    std::atomic<int> m_cbLastPointCount{0};

    // Captured laser lines (deep copies)
    QMutex m_queueMutex;
    std::queue<OwnedLaserLine> m_queue;
    const unsigned int m_maxQueuedLines = 4096;
};

#endif // SCANWORKER_H
