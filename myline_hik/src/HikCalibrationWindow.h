#ifndef MYLINE_HIK_HIK_CALIBRATION_WINDOW_H
#define MYLINE_HIK_HIK_CALIBRATION_WINDOW_H

#include "HikCalibrationCore.h"
#include "HandEyeCalibrationCore.h"
#include "LineLaserDeviceProfile.h"

#include <QImage>
#include <QMainWindow>
#include <QThread>
#include <QString>
#include <QtGlobal>

#include <vector>

class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTabWidget;
class QTableWidget;
class HikCameraWorker;
class FairinoRobotSession;
class ImageView;

class HikCalibrationWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit HikCalibrationWindow(const LineLaserDeviceProfile& profile,
                                  FairinoRobotSession* robotSession,
                                  QWidget* parent = nullptr);
    ~HikCalibrationWindow() override;

    const LineLaserDeviceProfile& deviceProfile() const { return profile_; }
    void setProfileTabActive(bool active);

signals:
    void requestConnectCamera(QString ipAddress);
    void requestDisconnectCamera();
    void requestCaptureSingle(int requestId,
                              double exposureUs,
                              double gainDb,
                              int timeoutMs);
    void requestConnectRobot(int clientId, QString ipAddress);
    void requestDisconnectRobot(int clientId);
    void requestReadFlangePose(int requestId);

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void connectCamera();
    void disconnectCamera();
    void captureManualFrame();
    void captureIntrinsicSample();
    void captureLaserOff();
    void captureLaserOn();
    void cancelPendingLaserPair();
    void reloadProfileCalibration();
    void captureProfileOff();
    void captureProfileOn();
    void cancelPendingProfilePair();
    void importProfilePair();
    void connectRobot();
    void disconnectRobot();
    void readRobotPose();
    void captureHandEyeSample();
    void deleteSelectedHandEyeSamples();
    void clearHandEyeSamples();
    void solveHandEye();
    void saveHandEyeCandidate();
    void saveApprovedHandEye();

    void onCameraConnectionChanged(bool connected, QString description);
    void onCameraIdentityChanged(QString model, QString serial, QString ipAddress);
    void onCameraBusyChanged(bool busy);
    void onCameraFrameReady(int requestId,
                            QImage image,
                            quint64 frameNo,
                            quint64 deviceTimestamp,
                            qint64 hostTimestamp,
                            double actualExposure,
                            double actualGain,
                            QString description);
    void onCameraLog(QString message);
    void onCameraError(int requestId, QString message);
    void onRobotConnectionChanged(bool connected, QString description);
    void onRobotBusyChanged(bool busy);
    void onRobotFlangePoseReady(int requestId,
                                double xMm,
                                double yMm,
                                double zMm,
                                double rxDeg,
                                double ryDeg,
                                double rzDeg,
                                qint64 hostTimestampMs);
    void onRobotLog(QString message);
    void onRobotError(int requestId, QString message);
    void onRobotClientError(int clientId, QString message);

    void swapBoardAxes();
    void importIntrinsicImages();
    void deleteSelectedIntrinsicSamples();
    void clearIntrinsicSamples();
    void solveIntrinsics();
    void saveIntrinsicCandidate();
    void saveApprovedIntrinsics();

    void loadIntrinsicsForLaser();
    void useCurrentIntrinsicsForLaser();
    void importLaserPairs();
    void deleteSelectedLaserPairs();
    void clearLaserPairs();
    void solveLaserPlane();
    void saveLaserCandidate();
    void saveApprovedLaserPlane();

private:
    enum class CapturePurpose {
        None,
        Manual,
        Intrinsic,
        LaserOff,
        LaserOn,
        ProfileOff,
        ProfileOn,
        HandEye
    };

    enum class LaserCaptureState {
        AwaitingOff,
        AwaitingOn
    };

    enum class HandEyeCaptureState {
        Idle,
        WaitingBeforePose,
        WaitingCameraFrame,
        WaitingAfterPose
    };

    struct IntrinsicSample {
        QString sampleId;
        QString imagePath;
        bool acquisitionIdentityVerified{false};
        hik_calibration::CharucoDetectionResult detection;
        bool calibrationAccepted{false};
        double calibrationRmsPx{0.0};
        QString calibrationReason;
    };

    struct LaserPairSample {
        QString sampleId;
        QString laserOffPath;
        QString laserOnPath;
        bool acquisitionParametersVerified{false};
        hik_calibration::LaserCalibrationPairResult result;
    };

    struct PendingLaserOff {
        bool valid{false};
        QString sampleId;
        QString imagePath;
        cv::Mat image;
        double exposureUs{0.0};
        double gainDb{0.0};
        hik_calibration::CharucoDetectionResult detection;
    };

    struct PendingProfileOff {
        bool valid{false};
        QString sampleId;
        QString imagePath;
        cv::Mat image;
        double exposureUs{0.0};
        double gainDb{0.0};
    };

    struct ProfileSample {
        QString sampleId;
        QString laserOffPath;
        QString laserOnPath;
        QString plyPath;
        QString csvPath;
        bool acquisitionParametersVerified{false};
        hik_calibration::StaticProfileResult result;
    };

    struct RobotPoseReading {
        bool valid{false};
        cv::Matx44d baseFromFlange{cv::Matx44d::eye()};
        double values[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        qint64 hostTimestampMs{0};
    };

    struct HandEyeSampleEntry {
        hik_calibration::HandEyeSample sample;
        QString imagePath;
        double flangeValues[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        int cornerCount{0};
        double robotTranslationDeltaMm{0.0};
        double robotRotationDeltaDeg{0.0};
        bool solverAccepted{false};
        double translationResidualMm{0.0};
        double rotationResidualDeg{0.0};
        QString solverReason;
    };

    struct PendingHandEyeImage {
        bool valid{false};
        QString sampleId;
        QString imagePath;
        hik_calibration::CharucoDetectionResult detection;
        hik_calibration::BoardPoseResult boardPose;
        qint64 cameraHostTimestampRaw{0};
    };

    void buildUi();
    QWidget* buildCameraBar();
    QWidget* buildBoardBar();
    QWidget* buildIntrinsicPage();
    QWidget* buildLaserPage();
    QWidget* buildProfilePage();
    QWidget* buildHandEyePage();
    QWidget* buildPreviewPanel();
    void setupCameraWorker();
    void shutdownCameraWorker();
    void setupRobotSession();
    void shutdownRobotSession();
    bool createSessionDirectories(QString* error);
    bool appendCaptureManifest(CapturePurpose purpose,
                               const QString& pairId,
                               const QString& imagePath,
                               const QImage& image,
                               quint64 frameNo,
                               quint64 deviceTimestamp,
                               qint64 hostTimestamp,
                               double actualExposure,
                               double actualGain,
                               const QString& description,
                               QString* error) const;
    bool appendProfileManifest(const ProfileSample& sample,
                               QString* error) const;
    bool appendHandEyeManifest(const HandEyeSampleEntry& sample,
                               const RobotPoseReading& before,
                               const RobotPoseReading& after,
                               QString* error) const;

    void appendLog(const QString& message);
    void showError(const QString& title, const QString& message);
    void updateAllUiStates();
    void updateCameraUi();
    void updateBoardUi();
    void updateIntrinsicUi();
    void updateLaserUi();
    void updateProfileUi();
    void updateHandEyeUi();
    void resetPendingCapture();
    QString expectedCameraModelFromUi() const;
    QString expectedCameraSerialFromUi() const;
    bool currentCameraMatchesProfile(QString* error = nullptr) const;
    bool intrinsicsMetadataMatchesProfile(
        const hik_calibration::IntrinsicsYamlMetadata& metadata,
        QString* error = nullptr) const;
    bool profileIdentityCanChange() const;

    hik_calibration::BoardSpec boardSpecFromUi() const;
    bool checkedBoardSpec(hik_calibration::BoardSpec* board, QString* error) const;
    bool boardCanChange() const;
    void setBoardSpecUi(const hik_calibration::BoardSpec& board);
    bool boardMatches(const hik_calibration::BoardSpec& left,
                      const hik_calibration::BoardSpec& right) const;

    int beginCapture(CapturePurpose purpose);
    QString captureTimestamp() const;
    QString uniqueFilePath(const QString& directory,
                           const QString& stem,
                           const QString& suffix) const;
    bool saveCameraImageImmediately(const QImage& image,
                                    CapturePurpose purpose,
                                    const QString& pairId,
                                    QString* path,
                                    QString* error) const;
    bool copyImportedImageToSession(const QString& sourcePath,
                                    const QString& targetDirectory,
                                    const QString& stem,
                                    cv::Mat* grayImage,
                                    QString* targetPath,
                                    QString* error) const;

    void addIntrinsicImage(const cv::Mat& grayImage,
                           const QString& imagePath,
                           const QString& sampleId,
                           bool acquisitionIdentityVerified);
    void refreshIntrinsicTable();
    int acceptedIntrinsicSampleCount() const;
    void invalidateIntrinsicSolution();
    bool intrinsicResultHasMatrices() const;
    bool intrinsicResultPassesQuality(QString* reason = nullptr) const;
    hik_calibration::IntrinsicsYamlMetadata intrinsicMetadata() const;
    bool writeIntrinsicYaml(const QString& path, QString* error) const;

    bool setActiveLaserIntrinsics(
        const hik_calibration::IntrinsicCalibrationResult& calibration,
        const hik_calibration::IntrinsicsYamlMetadata& metadata,
        const QString& sourcePath,
        QString* error);
    void acceptLaserOffImage(const cv::Mat& grayImage,
                             const QString& imagePath,
                             const QString& sampleId,
                             double actualExposureUs,
                             double actualGainDb);
    void completeLaserPair(const cv::Mat& laserOnImage,
                           const QString& laserOnPath,
                           double actualExposureUs,
                           double actualGainDb);
    void addOfflineLaserPair(const cv::Mat& laserOffImage,
                             const QString& laserOffPath,
                             const cv::Mat& laserOnImage,
                             const QString& laserOnPath,
                             const QString& sampleId);
    void resetPendingLaserOff();
    void refreshLaserTable();
    void refreshLaserGuidance();
    int acceptedLaserPairCount() const;
    void invalidateLaserSolution();
    bool laserGuidancePasses(QString* reason = nullptr) const;
    bool laserResultPassesQuality(QString* reason = nullptr) const;
    hik_calibration::LaserPlaneYamlMetadata laserMetadata() const;
    bool writeLaserYaml(const QString& path, QString* error) const;

    bool loadFormalProfileCalibration(QString* error);
    bool profileCameraIdentityMatches(QString* error) const;
    void acceptProfileOffImage(const cv::Mat& grayImage,
                               const QString& imagePath,
                               const QString& sampleId,
                               double actualExposureUs,
                               double actualGainDb);
    void completeProfilePair(const cv::Mat& laserOnImage,
                             const QString& laserOnPath,
                             double actualExposureUs,
                             double actualGainDb,
                             bool acquisitionParametersVerified);
    void addOfflineProfilePair(const cv::Mat& laserOffImage,
                               const QString& laserOffPath,
                               const cv::Mat& laserOnImage,
                               const QString& laserOnPath,
                               const QString& sampleId);
    void resetPendingProfileOff();
    void refreshProfileTable();

    bool loadFormalHandEyeIntrinsics(QString* error);
    bool handEyeCameraIdentityMatches(QString* error) const;
    void processHandEyeImage(const cv::Mat& grayImage,
                             const QString& imagePath,
                             const QString& sampleId,
                             qint64 cameraHostTimestampRaw);
    void finishHandEyeSample(const RobotPoseReading& after);
    void resetPendingHandEye();
    void refreshHandEyeTable();
    bool handEyeResultPassesQuality(QString* reason = nullptr) const;
    hik_calibration::HandEyeYamlMetadata handEyeMetadata() const;
    bool writeHandEyeYaml(const QString& path, QString* error) const;

    bool promoteFileAtomically(const QString& source,
                               const QString& destination,
                               QString* error) const;
    QImage drawCharucoOverlay(const cv::Mat& grayImage,
                              const hik_calibration::CharucoDetectionResult& detection,
                              const QString& caption) const;
    QImage drawLaserOverlay(const cv::Mat& grayImage,
                            const hik_calibration::LaserCalibrationPairResult& pair) const;
    QImage drawProfileOverlay(const cv::Mat& grayImage,
                              const hik_calibration::StaticProfileResult& profile) const;
    static cv::Mat qImageToGrayClone(const QImage& image);
    static QImage cvMatToQImageCopy(const cv::Mat& image);
    static QString fromStdString(const std::string& text);
    static std::string toLocalPath(const QString& path);

    QThread cameraThread_;
    HikCameraWorker* cameraWorker_{nullptr};
    bool cameraConnected_{false};
    bool cameraBusy_{false};
    bool shuttingDown_{false};
    FairinoRobotSession* const robotSession_;
    int robotClientId_{0};
    bool profileTabActive_{true};
    bool robotConnected_{false};
    bool robotBusy_{false};
    int pendingRobotRequestId_{-1};
    bool manualRobotReadPending_{false};
    RobotPoseReading currentRobotPose_;
    RobotPoseReading pendingHandEyeBeforePose_;
    HandEyeCaptureState handEyeCaptureState_{HandEyeCaptureState::Idle};
    PendingHandEyeImage pendingHandEyeImage_;
    QString pendingHandEyeSampleId_;
    int nextRequestId_{0};
    int pendingRequestId_{-1};
    CapturePurpose pendingCapturePurpose_{CapturePurpose::None};
    QString pendingLaserCapturePairId_;
    QString pendingProfileCapturePairId_;
    QString currentCameraModel_;
    QString currentCameraSerial_;
    QString currentCameraIp_;
    QString intrinsicDatasetCameraModel_;
    QString intrinsicDatasetCameraSerial_;

    const LineLaserDeviceProfile profile_;
    QString sourceDir_;
    QString sessionDir_;
    QString intrinsicSessionDir_;
    QString laserSessionDir_;
    QString profileSessionDir_;
    QString handEyeSessionDir_;
    QString captureManifestPath_;
    QString profileManifestPath_;
    QString handEyeManifestPath_;
    bool sessionReady_{false};

    QLineEdit* cameraIpEdit_{nullptr};
    QDoubleSpinBox* exposureSpin_{nullptr};
    QDoubleSpinBox* gainSpin_{nullptr};
    QSpinBox* timeoutSpin_{nullptr};
    QLabel* cameraStatusLabel_{nullptr};
    QLineEdit* expectedCameraModelEdit_{nullptr};
    QLineEdit* expectedCameraSerialEdit_{nullptr};
    QPushButton* connectButton_{nullptr};
    QPushButton* disconnectButton_{nullptr};
    QPushButton* singleFrameButton_{nullptr};

    QSpinBox* squaresXSpin_{nullptr};
    QSpinBox* squaresYSpin_{nullptr};
    QDoubleSpinBox* squareLengthSpin_{nullptr};
    QDoubleSpinBox* markerLengthSpin_{nullptr};
    QComboBox* dictionaryCombo_{nullptr};
    QPushButton* swapBoardButton_{nullptr};
    QLabel* boardLockLabel_{nullptr};

    QTabWidget* calibrationTabs_{nullptr};
    QPushButton* captureIntrinsicButton_{nullptr};
    QPushButton* importIntrinsicButton_{nullptr};
    QPushButton* deleteIntrinsicButton_{nullptr};
    QPushButton* clearIntrinsicButton_{nullptr};
    QPushButton* solveIntrinsicButton_{nullptr};
    QPushButton* saveIntrinsicCandidateButton_{nullptr};
    QPushButton* saveIntrinsicApprovedButton_{nullptr};
    QLabel* intrinsicGateLabel_{nullptr};
    QLabel* intrinsicResultLabel_{nullptr};
    QTableWidget* intrinsicTable_{nullptr};

    QLineEdit* laserIntrinsicsPathEdit_{nullptr};
    QPushButton* loadLaserIntrinsicsButton_{nullptr};
    QPushButton* useCurrentIntrinsicsButton_{nullptr};
    QLabel* laserIntrinsicsStatusLabel_{nullptr};
    QPushButton* captureLaserOffButton_{nullptr};
    QPushButton* captureLaserOnButton_{nullptr};
    QPushButton* cancelLaserPairButton_{nullptr};
    QLabel* laserPairStateLabel_{nullptr};
    QPushButton* importLaserPairsButton_{nullptr};
    QPushButton* deleteLaserPairButton_{nullptr};
    QPushButton* clearLaserPairsButton_{nullptr};
    QDoubleSpinBox* ransacThresholdSpin_{nullptr};
    QPushButton* solveLaserButton_{nullptr};
    QPushButton* saveLaserCandidateButton_{nullptr};
    QPushButton* saveLaserApprovedButton_{nullptr};
    QLabel* laserGateLabel_{nullptr};
    QLabel* laserResultLabel_{nullptr};
    QLabel* laserGuidanceSummaryLabel_{nullptr};
    QTableWidget* laserGuidanceTable_{nullptr};
    QTableWidget* laserTable_{nullptr};

    QPushButton* reloadProfileCalibrationButton_{nullptr};
    QLabel* profileCalibrationStatusLabel_{nullptr};
    QPushButton* captureProfileOffButton_{nullptr};
    QPushButton* captureProfileOnButton_{nullptr};
    QPushButton* cancelProfilePairButton_{nullptr};
    QPushButton* importProfilePairButton_{nullptr};
    QLabel* profilePairStateLabel_{nullptr};
    QLabel* profileResultLabel_{nullptr};
    QTableWidget* profileTable_{nullptr};

    QLineEdit* robotIpEdit_{nullptr};
    QLabel* robotStatusLabel_{nullptr};
    QLabel* robotPoseLabel_{nullptr};
    QPushButton* connectRobotButton_{nullptr};
    QPushButton* disconnectRobotButton_{nullptr};
    QPushButton* readRobotPoseButton_{nullptr};
    QPushButton* captureHandEyeButton_{nullptr};
    QPushButton* deleteHandEyeButton_{nullptr};
    QPushButton* clearHandEyeButton_{nullptr};
    QPushButton* solveHandEyeButton_{nullptr};
    QPushButton* saveHandEyeCandidateButton_{nullptr};
    QPushButton* saveHandEyeApprovedButton_{nullptr};
    QLabel* handEyeIntrinsicsStatusLabel_{nullptr};
    QLabel* handEyeCaptureStateLabel_{nullptr};
    QLabel* handEyeGateLabel_{nullptr};
    QLabel* handEyeResultLabel_{nullptr};
    QTableWidget* handEyeTable_{nullptr};

    ImageView* imageView_{nullptr};
    QLabel* imageInfoLabel_{nullptr};
    QPlainTextEdit* logView_{nullptr};

    hik_calibration::DetectionOptions detectionOptions_;
    hik_calibration::IntrinsicCalibrationOptions intrinsicOptions_;
    hik_calibration::LaserPairOptions laserPairOptions_;
    hik_calibration::PlaneFitOptions planeOptions_;
    std::vector<IntrinsicSample> intrinsicSamples_;
    hik_calibration::IntrinsicCalibrationResult intrinsicResult_;
    bool hasIntrinsicResult_{false};
    QString lastIntrinsicCandidatePath_;

    hik_calibration::IntrinsicCalibrationResult activeLaserIntrinsics_;
    hik_calibration::IntrinsicsYamlMetadata activeLaserIntrinsicsMetadata_;
    QString activeLaserIntrinsicsPath_;
    QString activeLaserIntrinsicsSha256_;
    bool hasActiveLaserIntrinsics_{false};
    LaserCaptureState laserCaptureState_{LaserCaptureState::AwaitingOff};
    PendingLaserOff pendingLaserOff_;
    std::vector<LaserPairSample> laserPairs_;
    hik_calibration::LaserPlaneFitResult laserPlaneResult_;
    bool hasLaserPlaneResult_{false};
    QString lastLaserCandidatePath_;

    hik_calibration::IntrinsicCalibrationResult profileIntrinsics_;
    hik_calibration::IntrinsicsYamlMetadata profileIntrinsicsMetadata_;
    hik_calibration::LaserPlaneFitResult profileLaserPlane_;
    hik_calibration::LaserPlaneYamlMetadata profileLaserMetadata_;
    hik_calibration::BoardSpec profileBoard_;
    hik_calibration::StaticProfileOptions profileOptions_;
    QString profileIntrinsicsPath_;
    QString profileLaserPlanePath_;
    QString profileIntrinsicsSha256_;
    QString profileLaserPlaneSha256_;
    bool hasProfileCalibration_{false};
    LaserCaptureState profileCaptureState_{LaserCaptureState::AwaitingOff};
    PendingProfileOff pendingProfileOff_;
    std::vector<ProfileSample> profileSamples_;

    hik_calibration::IntrinsicCalibrationResult handEyeIntrinsics_;
    hik_calibration::IntrinsicsYamlMetadata handEyeIntrinsicsMetadata_;
    QString handEyeIntrinsicsPath_;
    QString handEyeIntrinsicsSha256_;
    bool hasHandEyeIntrinsics_{false};
    hik_calibration::BoardPoseOptions handEyeBoardPoseOptions_;
    hik_calibration::HandEyeOptions handEyeOptions_;
    std::vector<HandEyeSampleEntry> handEyeSamples_;
    hik_calibration::HandEyeCalibrationResult handEyeResult_;
    bool hasHandEyeResult_{false};
    QString lastHandEyeCandidatePath_;
};

#endif // MYLINE_HIK_HIK_CALIBRATION_WINDOW_H
