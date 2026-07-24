#ifndef MYLINE_HIK_HIK_CONSTANT_LASER_SCAN_WINDOW_H
#define MYLINE_HIK_HIK_CONSTANT_LASER_SCAN_WINDOW_H

#include "HikCalibrationCore.h"
#include "HikContinuousReconstruction.h"
#include "HikScanCore.h"
#include "HikSynchronizationCore.h"
#include "LineLaserController.h"
#include "LineLaserDeviceProfile.h"

#include <QImage>
#include <QJsonObject>
#include <QMainWindow>
#include <QMetaObject>
#include <QThread>

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

class DirectCallbackGate;
class FairinoRobotSession;
class HikCameraWorker;
class ImageView;
class QCheckBox;
class QCloseEvent;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTimer;

class HikConstantLaserScanWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit HikConstantLaserScanWindow(const LineLaserDeviceProfile& profile,
                                        LineLaserController* laserController,
                                        FairinoRobotSession* robotSession,
                                        QWidget* parent = nullptr,
                                        double scanSpeedOverrideMmS = -1.0);
    ~HikConstantLaserScanWindow() override;

    const LineLaserDeviceProfile& deviceProfile() const { return profile_; }
    void setProfileTabActive(bool active);

signals:
    void scanActivityChanged(bool active);
    void requestConnectCamera(QString ipAddress);
    void requestDisconnectCamera();
    void requestCaptureSingle(int requestId, double exposureUs, double gainDb, int timeoutMs);
    void requestStartContinuous(double exposureUs,
                                double gainDb,
                                double targetFps,
                                int poolCapacity);
    void requestStopContinuous();
    void requestConnectRobot(int clientId, QString ipAddress);
    void requestDisconnectRobot(int clientId);
    void requestReadFlangePose(int requestId);
    void requestMoveLinear(int requestId,
                           double xMm, double yMm, double zMm,
                           double rxDeg, double ryDeg, double rzDeg,
                           double velocityPercent,
                           double accelerationPercent,
                           int timeoutMs);
    void requestMoveLinearPhysical(int requestId,
                                   double xMm, double yMm, double zMm,
                                   double rxDeg, double ryDeg, double rzDeg,
                                   double speedMmS,
                                   double accelerationMmS2,
                                   int timeoutMs);
    void requestStopMotion(int requestId);

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void connectCamera();
    void disconnectCamera();
    void connectRobot();
    void disconnectRobot();
    void readCurrentPose();
    void teachStart();
    void teachEnd();
    void editStartPose();
    void editEndPose();
    void generateDryRun();
    void captureCurrentProfile();
    void startScan();
    void stopScan();
    void startContinuousScan();
    void reloadCalibration();
    void connectLaserController();
    void enableProfileLaser();
    void disableAllLasers();

    void onCameraConnectionChanged(bool connected, QString description);
    void onCameraIdentityChanged(QString model, QString serial, QString ipAddress);
    void onCameraBusyChanged(bool busy);
    void onCameraFrameReady(int requestId, QImage image, quint64 frameNo,
                            quint64 deviceTimestamp, qint64 hostTimestamp,
                            double actualExposure, double actualGain,
                            QString description);
    void onCameraLog(QString message);
    void onCameraError(int requestId, QString message);
    void onContinuousCameraStarted(double actualExposureUs,
                                   double actualFps,
                                   quint64 timestampFrequencyHz,
                                   QString timestampDescription);
    void onContinuousCameraStopped(bool confirmed, QString description);
    void onContinuousFrameRejected(quint64 frameNo, QString reason);

    void onRobotConnectionChanged(bool connected, QString description);
    void onRobotBusyChanged(bool busy);
    void onRobotFlangePoseReady(int requestId,
                                double xMm, double yMm, double zMm,
                                double rxDeg, double ryDeg, double rzDeg,
                                qint64 hostTimestampMs);
    void onRobotMotionStarted(int requestId, QString description);
    void onRobotMotionFinished(int requestId,
                               bool targetReached,
                               bool motionStoppedConfirmed,
                               QString description);
    void onRobotLog(QString message);
    void onRobotError(int requestId, QString message);
    void onRobotClientError(int clientId, QString message);
    void onLaserConnectionStateChanged(LineLaserConnectionState state,
                                       QString detail);
    void onLaserStatusChanged(LineLaserStatus status);
    void onLaserCommandFinished(QString command, bool success,
                                QString detail);
    void onLaserFault(QString detail);

private:
    enum class ScanState { Idle, Moving, Settling, ReadingBefore, Capturing, ReadingAfter };
    enum class ContinuousState { Idle, MovingToStart, StartingCamera, Scanning, Stopping };
    enum class ReadRole { None, Manual, TeachStart, TeachEnd, ScanBefore, ScanAfter };

    struct PoseReading {
        bool valid{false};
        hik_scan::Pose6D pose;
        cv::Matx44d baseFromFlange{cv::Matx44d::eye()};
        qint64 hostTimestampMs{0};
    };

    struct ProfileRow {
        int index{0};
        QString imagePath;
        PoseReading before;
        PoseReading after;
        int cameraPointCount{0};
        double minimumDepthMm{0.0};
        double maximumDepthMm{0.0};
        double lineRmsMm{0.0};
        double lineRmsLimitMm{0.0};
        bool flatTargetGate{false};
        double stripeSaturatedRatio{0.0};
        double translationDeltaMm{0.0};
        double rotationDeltaDeg{0.0};
        bool qualityExtractionPassed{false};
        int legacyPointCount{0};
        int qualityPointCount{0};
        int qualityCandidateCount{0};
        int qualityAcceptedCandidateCount{0};
        int qualityRejectedCandidateCount{0};
        int qualitySelectedPointCount{0};
        int qualitySelectedGapCount{0};
        int qualityMultiPeakScanlineCount{0};
        int qualityAmbiguousPathPointCount{0};
        int qualityRejectedLowProminenceCount{0};
        int qualityRejectedWidthCount{0};
        int qualityRejectedSaturationCount{0};
        int qualityRejectedMultiPeakCount{0};
        int qualityRejectedAsymmetryCount{0};
        int qualityRejectedFitCount{0};
        int qualityRejectedQualityCount{0};
        int qualityRejectedMaskCount{0};
        double qualityMeanSelectedQuality{0.0};
        double qualityMeanSelectedFwhmPx{0.0};
        double qualityMeanSelectedSnr{0.0};
        double qualitySelectedSaturatedRatio{0.0};
        double qualityMeanSelectedGradientAsymmetry{0.0};
        double qualityMeanSelectedFitResidual{0.0};
        double qualityMeanSelectedSecondPeakRatio{0.0};
        double qualityPathCostMarginPerPoint{0.0};
        int centerMatchedPointCount{0};
        int centerRobustMatchedPointCount{0};
        int centerGrossMismatchPointCount{0};
        double centerSignedMeanOffsetPx{0.0};
        double centerSignedMedianOffsetPx{0.0};
        double centerRobustSignedMeanOffsetPx{0.0};
        double centerRobustGatePx{0.0};
        double centerAbsoluteMedianOffsetPx{0.0};
        double centerAbsoluteP95OffsetPx{0.0};
        double centerAbsoluteMaximumOffsetPx{0.0};
        QString centerlineAlgorithmVersion;
        QString qualityExtractionError;
    };

    void buildUi();
    void setupWorkers();
    void shutdownWorkers();
    void updateUi();
    void appendLog(const QString& message);
    void showError(const QString& title, const QString& message);
    bool laserReadyForProfile(QString* error = nullptr) const;
    bool laserStatusFresh() const;
    quint64 requestLaserOff();
    void abortForLaserSafety(const QString& reason);
    void beginTerminalBarrier(bool completed,
                              const QString& reason,
                              const QString& mode,
                              const QString& sessionDirectory,
                              const QJsonObject& statistics = QJsonObject());
    void tryCompleteTerminalBarrier();
    bool writeSessionResult(QString* error = nullptr) const;
    bool writeSessionMetadata(const QString& directory,
                              QString* error) const;
    bool acquireScanActivity(QString* error);
    void releaseScanActivity();
    bool loadFormalCalibration(QString* error);
    bool formalCalibrationFilesUnchanged(QString* error) const;
    bool calibrationIdentityMatches(QString* error) const;
    bool editPoseDialog(hik_scan::Pose6D* pose,
                        const QString& title,
                        bool orientationIgnored);
    bool buildTargets(std::vector<hik_scan::Pose6D>* targets, QString* error) const;
    void issueRobotRead(ReadRole role);
    void issueMoveForCurrentTarget();
    void beginSettledCapture();
    void finishProfile(const PoseReading& after);
    void continueOrFinish();
    void abortScan(const QString& reason, bool requestStop);
    void abortContinuousScan(const QString& reason, bool requestStop);
    void finalizeContinuousScan(bool completed, const QString& reason);
    bool createSynchronizationSession(QString* error);
    bool createScanSession(QString* error);
    bool appendManifest(const ProfileRow& row,
                        quint64 frameNo,
                        quint64 deviceTimestamp,
                        qint64 cameraHostTimestampRaw,
                        double exposureUs,
                        double gainDb,
                        QString* error) const;
    bool saveCloudOutputs(QString* error);
    void refreshTable();
    QImage drawStripeOverlay(const cv::Mat& gray,
                             const hik_calibration::StaticProfileResult& profile) const;

    static cv::Mat qImageToGray(const QImage& image);
    static QImage cvMatToQImage(const cv::Mat& image);
    static std::string localPath(const QString& path);

    const LineLaserDeviceProfile profile_;
    LineLaserController* const laserController_;
    FairinoRobotSession* const robotSession_;
    int robotClientId_{0};
    QString sourceDir_;
    bool shuttingDown_{false};
    bool profileTabActive_{true};
    bool scanActivityHeld_{false};
    std::atomic<quint64> robotLeaseEpoch_{0};
    bool laserSafetyAbortIssued_{false};
    bool terminalBarrierActive_{false};
    bool terminalCompleted_{false};
    bool terminalMotionFault_{false};
    bool terminalCameraFault_{false};
    QString terminalReason_;
    QString terminalMode_;
    QString terminalSessionDirectory_;
    QString terminalEndedUtc_;
    QString terminalMotionFaultDetail_;
    QString terminalCameraFaultDetail_;
    QJsonObject terminalStatistics_;
    quint64 scanGeneration_{0};
    LineLaserConnectionState laserConnectionState_{
        LineLaserConnectionState::Disconnected};
    LineLaserStatus laserStatus_;
    qint64 laserStatusReceivedMonotonicMs_{0};
    quint64 laserStatusEventSequence_{0};
    quint64 laserTransportGeneration_{0};
    quint64 laserAcknowledgedOffCommandToken_{0};
    qint64 terminalLaserOffRequestedNs_{0};
    quint64 terminalLaserOffCommandToken_{0};
    hik_sync::SynchronizationConfig synchronizationConfig_;
    bool synchronizationConfigReady_{false};
    QString synchronizationConfigPath_;
    hik_sync::SynchronizationSession synchronizationSession_;
    hik_scan::ContinuousReconstructionPipeline continuousReconstruction_;
    ContinuousState continuousState_{ContinuousState::Idle};
    bool continuousAbortRequested_{false};
    QString synchronizationSessionDir_;

    QThread cameraThread_;
    HikCameraWorker* cameraWorker_{nullptr};
    std::shared_ptr<DirectCallbackGate> directCallbackGate_;
    QMetaObject::Connection continuousFrameConnection_;
    QMetaObject::Connection imagePoolConnection_;
    QMetaObject::Connection robotSampleConnection_;
    bool cameraConnected_{false};
    bool cameraBusy_{false};
    QString cameraModel_;
    QString cameraSerial_;
    QString cameraIp_;
    int nextCameraRequestId_{0};
    int pendingCameraRequestId_{-1};

    bool robotConnected_{false};
    bool robotBusy_{false};
    int pendingRobotRequestId_{-1};
    int pendingMotionRequestId_{-1};
    ReadRole readRole_{ReadRole::None};
    PoseReading currentPose_;

    bool calibrationReady_{false};
    hik_calibration::IntrinsicCalibrationResult intrinsics_;
    hik_calibration::IntrinsicsYamlMetadata intrinsicsMetadata_;
    hik_calibration::LaserPlaneFitResult laserPlane_;
    hik_calibration::LaserPlaneYamlMetadata laserMetadata_;
    hik_calibration::BoardSpec laserBoard_;
    hik_scan::HandEyeFile handEye_;
    QString intrinsicsPath_;
    QString laserPlanePath_;
    QString handEyePath_;
    QString intrinsicsSha256_;
    QString laserPlaneSha256_;
    QString handEyeSha256_;
    hik_calibration::SingleFrameProfileOptions profileOptions_;

    bool startTaught_{false};
    bool endTaught_{false};
    hik_scan::Pose6D startPose_;
    hik_scan::Pose6D endPose_;
    std::vector<hik_scan::Pose6D> targets_;
    int currentTargetIndex_{-1};
    bool singlePointMode_{false};
    bool stopRequested_{false};
    ScanState scanState_{ScanState::Idle};
    PoseReading beforePose_;
    hik_calibration::StaticProfileResult pendingProfile_;
    QString pendingImagePath_;
    quint64 pendingFrameNo_{0};
    quint64 pendingDeviceTimestamp_{0};
    qint64 pendingCameraHostTimestamp_{0};
    double pendingExposureUs_{0.0};
    double pendingGainDb_{0.0};
    double pendingStripeSaturatedRatio_{0.0};
    LineLaserStatus pendingLaserStatus_;
    qint64 pendingLaserStatusAgeMs_{-1};
    std::vector<hik_scan::CloudPoint> cloud_;
    std::vector<hik_scan::CloudPoint> qualityCloud_;
    hik_scan::AdjacentProfileSupportResult qualitySupportResult_;
    std::size_t qualityVoxelPointCount_{0U};
    std::vector<ProfileRow> profileRows_;

    QString scanSessionDir_;
    QString imageDir_;
    QString manifestPath_;
    QString rawPlyPath_;
    QString voxelPlyPath_;
    QString qualityOpticalPlyPath_;
    QString qualityFilteredPlyPath_;
    QString qualityRejectedPlyPath_;
    QString qualityVoxelPlyPath_;

    QLineEdit* cameraIpEdit_{nullptr};
    QDoubleSpinBox* exposureSpin_{nullptr};
    QDoubleSpinBox* gainSpin_{nullptr};
    QSpinBox* cameraTimeoutSpin_{nullptr};
    QPushButton* connectCameraButton_{nullptr};
    QPushButton* disconnectCameraButton_{nullptr};
    QLabel* cameraStatusLabel_{nullptr};
    QPushButton* connectLaserButton_{nullptr};
    QPushButton* enableProfileLaserButton_{nullptr};
    QPushButton* disableAllLasersButton_{nullptr};
    QLabel* laserStatusLabel_{nullptr};
    QTimer* laserFreshnessTimer_{nullptr};

    QLineEdit* robotIpEdit_{nullptr};
    QPushButton* connectRobotButton_{nullptr};
    QPushButton* disconnectRobotButton_{nullptr};
    QPushButton* readPoseButton_{nullptr};
    QLabel* robotStatusLabel_{nullptr};
    QLabel* currentPoseLabel_{nullptr};

    QPushButton* reloadCalibrationButton_{nullptr};
    QLabel* calibrationStatusLabel_{nullptr};
    QPushButton* teachStartButton_{nullptr};
    QPushButton* teachEndButton_{nullptr};
    QPushButton* editStartButton_{nullptr};
    QPushButton* editEndButton_{nullptr};
    QLabel* startPoseLabel_{nullptr};
    QLabel* endPoseLabel_{nullptr};
    QDoubleSpinBox* stepSpin_{nullptr};
    QDoubleSpinBox* velocitySpin_{nullptr};
    QDoubleSpinBox* scanSpeedSpin_{nullptr};
    QDoubleSpinBox* accelerationSpin_{nullptr};
    QSpinBox* settleSpin_{nullptr};
    QSpinBox* motionTimeoutSpin_{nullptr};
    QDoubleSpinBox* voxelSpin_{nullptr};
    QCheckBox* flatTargetGateCheck_{nullptr};
    QDoubleSpinBox* lineRmsLimitSpin_{nullptr};
    QCheckBox* pathLengthLimitCheck_{nullptr};
    QDoubleSpinBox* pathLengthLimitSpin_{nullptr};
    QCheckBox* targetCountLimitCheck_{nullptr};
    QSpinBox* targetCountLimitSpin_{nullptr};
    QCheckBox* dryRunCheck_{nullptr};
    QCheckBox* safetyConfirmCheck_{nullptr};
    QPushButton* dryRunButton_{nullptr};
    QPushButton* captureCurrentButton_{nullptr};
    QPushButton* startScanButton_{nullptr};
    QPushButton* startContinuousButton_{nullptr};
    QPushButton* stopButton_{nullptr};
    QLabel* scanStatusLabel_{nullptr};
    QTableWidget* profileTable_{nullptr};
    ImageView* imageView_{nullptr};
    QPlainTextEdit* logView_{nullptr};
};

#endif  // MYLINE_HIK_HIK_CONSTANT_LASER_SCAN_WINDOW_H
