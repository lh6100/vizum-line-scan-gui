#ifndef MYLINE_HIK_STEREO_MAPPER_WINDOW_H
#define MYLINE_HIK_STEREO_MAPPER_WINDOW_H

#include "LineLaserController.h"
#include "LineLaserDeviceProfile.h"
#include "stereo/app/StereoCameraRig.h"
#include "stereo/app/StereoMappingWorker.h"

#include <QThread>
#include <QWidget>

class FairinoRobotSession;
class ImageView;
class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTextEdit;
class QTimer;

namespace hik_stereo {

class StereoMapperWindow final : public QWidget {
    Q_OBJECT

public:
    StereoMapperWindow(LineLaserController* laserController,
                       FairinoRobotSession* robotSession,
                       QWidget* parent = nullptr);
    ~StereoMapperWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void loadCalibration();
    void connectCameras();
    void connectRobot();
    void startMapping();
    void stopMapping();
    void resetMap();
    void exportMap();
    void onLaserStatus(LineLaserStatus status);
    void onCameraConnection(StereoCameraSide side,
                            bool connected,
                            QString description);
    void onCameraIdentity(StereoCameraSide side,
                          QString model,
                          QString serial,
                          QString ipAddress);
    void onStereoPair(StereoFramePair pair);
    void onRobotSample(int clientId,
                       quint64 leaseEpoch,
                       hik_sync::RobotSample sample);
    void onFrameProcessed(StereoUiFrame frame);
    void onCameraRigStopped(bool confirmed, QString description);

private:
    void buildUi();
    void beginMappingAfterLaserOff();
    void finishMapping(const QString& description);
    void drainSynchronizedPairs();
    void updateUi();
    void appendLog(const QString& message);
    bool cameraIdentityAccepted(StereoCameraSide side) const;
    StereoDepthOptions depthOptionsFromUi() const;
    OccupancyMapOptions mapOptionsFromUi() const;
    static cv::Matx44d eigenToCv(const Eigen::Matrix4d& transform);

    LineLaserController* laserController_{nullptr};
    FairinoRobotSession* robotSession_{nullptr};
    const LineLaserDeviceProfile* leftProfile_{nullptr};
    const LineLaserDeviceProfile* rightProfile_{nullptr};
    QString sourceDirectory_;
    StereoCameraRig* cameraRig_{nullptr};
    QThread processingThread_;
    StereoMappingWorker* processingWorker_{nullptr};
    StereoPoseSynchronizer poseSynchronizer_;
    StereoRigCalibration rigCalibration_;
    int robotClientId_{0};
    quint64 robotLeaseEpoch_{0};
    quint64 pendingLaserOffToken_{0};
    qint64 laserOffRequestedNs_{0};
    quint64 mappingGeneration_{0};
    bool calibrationLoaded_{false};
    bool leftConnected_{false};
    bool rightConnected_{false};
    bool robotConnected_{false};
    bool waitingLaserOff_{false};
    bool mappingRequested_{false};
    bool mappingActive_{false};
    bool processingBusy_{false};
    int consecutiveProcessingErrors_{0};
    quint64 processedFrames_{0};
    quint64 busyDrops_{0};
    QString leftModel_;
    QString leftSerial_;
    QString rightModel_;
    QString rightSerial_;
    LineLaserStatus latestLaserStatus_;
    QTimer* laserWatchdog_{nullptr};

    QLineEdit* leftIpEdit_{nullptr};
    QLineEdit* rightIpEdit_{nullptr};
    QLineEdit* robotIpEdit_{nullptr};
    QLabel* leftStatusLabel_{nullptr};
    QLabel* rightStatusLabel_{nullptr};
    QLabel* robotStatusLabel_{nullptr};
    QLabel* laserStatusLabel_{nullptr};
    QLabel* calibrationStatusLabel_{nullptr};
    QLabel* mappingStatusLabel_{nullptr};
    QLabel* statisticsLabel_{nullptr};
    QPushButton* connectCamerasButton_{nullptr};
    QPushButton* connectRobotButton_{nullptr};
    QPushButton* loadCalibrationButton_{nullptr};
    QPushButton* startButton_{nullptr};
    QPushButton* stopButton_{nullptr};
    QPushButton* resetButton_{nullptr};
    QPushButton* exportButton_{nullptr};
    QComboBox* resolutionCombo_{nullptr};
    QDoubleSpinBox* exposureSpin_{nullptr};
    QDoubleSpinBox* gainSpin_{nullptr};
    QDoubleSpinBox* fpsSpin_{nullptr};
    QDoubleSpinBox* pairSkewSpin_{nullptr};
    QDoubleSpinBox* minimumDepthSpin_{nullptr};
    QDoubleSpinBox* maximumDepthSpin_{nullptr};
    QSpinBox* blockSizeSpin_{nullptr};
    QCheckBox* leftRightCheck_{nullptr};
    QDoubleSpinBox* voxelSizeSpin_{nullptr};
    QSpinBox* pixelStrideSpin_{nullptr};
    QDoubleSpinBox* robotOffsetSpin_{nullptr};
    QDoubleSpinBox* robotGapSpin_{nullptr};
    QDoubleSpinBox* gridMinimumHeightSpin_{nullptr};
    QDoubleSpinBox* gridMaximumHeightSpin_{nullptr};
    ImageView* leftView_{nullptr};
    ImageView* disparityView_{nullptr};
    ImageView* depthView_{nullptr};
    QTextEdit* logView_{nullptr};
};

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_MAPPER_WINDOW_H
