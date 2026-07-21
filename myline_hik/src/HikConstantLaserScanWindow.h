#ifndef MYLINE_HIK_HIK_CONSTANT_LASER_SCAN_WINDOW_H
#define MYLINE_HIK_HIK_CONSTANT_LASER_SCAN_WINDOW_H

#include "HikCalibrationCore.h"
#include "HikScanCore.h"

#include <QImage>
#include <QMainWindow>
#include <QThread>

#include <vector>

class FairinoReadOnlyWorker;
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

class HikConstantLaserScanWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit HikConstantLaserScanWindow(QWidget* parent = nullptr);
    ~HikConstantLaserScanWindow() override;

signals:
    void requestConnectCamera(QString ipAddress);
    void requestDisconnectCamera();
    void requestCaptureSingle(int requestId, double exposureUs, double gainDb, int timeoutMs);
    void requestConnectRobot(QString ipAddress);
    void requestDisconnectRobot();
    void requestReadFlangePose(int requestId);
    void requestMoveLinear(int requestId,
                           double xMm, double yMm, double zMm,
                           double rxDeg, double ryDeg, double rzDeg,
                           double velocityPercent,
                           double accelerationPercent,
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
    void reloadCalibration();

    void onCameraConnectionChanged(bool connected, QString description);
    void onCameraIdentityChanged(QString model, QString serial, QString ipAddress);
    void onCameraBusyChanged(bool busy);
    void onCameraFrameReady(int requestId, QImage image, quint64 frameNo,
                            quint64 deviceTimestamp, qint64 hostTimestamp,
                            double actualExposure, double actualGain,
                            QString description);
    void onCameraLog(QString message);
    void onCameraError(int requestId, QString message);

    void onRobotConnectionChanged(bool connected, QString description);
    void onRobotBusyChanged(bool busy);
    void onRobotFlangePoseReady(int requestId,
                                double xMm, double yMm, double zMm,
                                double rxDeg, double ryDeg, double rzDeg,
                                qint64 hostTimestampMs);
    void onRobotMotionStarted(int requestId, QString description);
    void onRobotMotionFinished(int requestId, bool success, QString description);
    void onRobotLog(QString message);
    void onRobotError(int requestId, QString message);

private:
    enum class ScanState { Idle, Moving, Settling, ReadingBefore, Capturing, ReadingAfter };
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
    };

    void buildUi();
    void setupWorkers();
    void shutdownWorkers();
    void updateUi();
    void appendLog(const QString& message);
    void showError(const QString& title, const QString& message);
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

    QString sourceDir_;
    QString configDir_;
    bool shuttingDown_{false};

    QThread cameraThread_;
    HikCameraWorker* cameraWorker_{nullptr};
    bool cameraConnected_{false};
    bool cameraBusy_{false};
    QString cameraModel_;
    QString cameraSerial_;
    QString cameraIp_;
    int nextCameraRequestId_{0};
    int pendingCameraRequestId_{-1};

    QThread robotThread_;
    FairinoReadOnlyWorker* robotWorker_{nullptr};
    bool robotConnected_{false};
    bool robotBusy_{false};
    int nextRobotRequestId_{0};
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
    std::vector<hik_scan::CloudPoint> cloud_;
    std::vector<ProfileRow> profileRows_;

    QString scanSessionDir_;
    QString imageDir_;
    QString manifestPath_;
    QString rawPlyPath_;
    QString voxelPlyPath_;

    QLineEdit* cameraIpEdit_{nullptr};
    QDoubleSpinBox* exposureSpin_{nullptr};
    QDoubleSpinBox* gainSpin_{nullptr};
    QSpinBox* cameraTimeoutSpin_{nullptr};
    QPushButton* connectCameraButton_{nullptr};
    QPushButton* disconnectCameraButton_{nullptr};
    QLabel* cameraStatusLabel_{nullptr};

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
    QPushButton* stopButton_{nullptr};
    QLabel* scanStatusLabel_{nullptr};
    QTableWidget* profileTable_{nullptr};
    ImageView* imageView_{nullptr};
    QPlainTextEdit* logView_{nullptr};
};

#endif  // MYLINE_HIK_HIK_CONSTANT_LASER_SCAN_WINDOW_H
