#ifndef MYLINE_HIK_STEREO_CALIBRATION_WINDOW_H
#define MYLINE_HIK_STEREO_CALIBRATION_WINDOW_H

#include "LineLaserController.h"
#include "LineLaserDeviceProfile.h"
#include "stereo/app/StereoCameraRig.h"
#include "stereo/calibration/StereoCalibrationEngine.h"

#include <QWidget>

#include <vector>

class FairinoRobotSession;
class ImageView;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;
class QTimer;

namespace hik_stereo {

class StereoCalibrationWindow final : public QWidget {
    Q_OBJECT

public:
    StereoCalibrationWindow(LineLaserController* laserController,
                            FairinoRobotSession* robotSession,
                            QWidget* parent = nullptr);
    ~StereoCalibrationWindow() override;

private slots:
    void connectCameras();
    void connectRobot();
    void startPreview();
    void stopPreview();
    void capturePair();
    void importSession();
    void solveCalibration();
    void approveCalibration();
    void onPair(StereoFramePair pair);
    void onRobotSample(int clientId,
                       quint64 leaseEpoch,
                       hik_sync::RobotSample sample);
    void onLaserStatus(LineLaserStatus status);

private:
    void buildUi();
    bool loadIndependentCalibration(QString* error);
    void beginPreviewAfterLaserOff();
    void finishPreview(const QString& description);
    void updateUi();
    void updateCaptureReadiness();
    void appendLog(const QString& message);
    bool cameraIdentitiesAccepted() const;
    bool robotStationary(QString* reason) const;
    bool pairWasCapturedAfterStationary(QString* reason) const;
    bool pairToGray(const StereoFramePair& pair,
                    cv::Mat* left,
                    cv::Mat* right,
                    QString* error) const;
    static QImage bufferImage(const hik_sync::CameraFrame& frame);

    LineLaserController* laserController_{nullptr};
    FairinoRobotSession* robotSession_{nullptr};
    const LineLaserDeviceProfile* leftProfile_{nullptr};
    const LineLaserDeviceProfile* rightProfile_{nullptr};
    QString sourceDirectory_;
    StereoCameraRig* cameraRig_{nullptr};
    int robotClientId_{0};
    quint64 robotLeaseEpoch_{0};
    quint64 laserOffToken_{0};
    qint64 laserOffRequestedNs_{0};
    bool leftConnected_{false};
    bool rightConnected_{false};
    bool robotConnected_{false};
    bool waitingLaserOff_{false};
    bool previewRequested_{false};
    bool previewActive_{false};
    bool seedLoaded_{false};
    QString leftModel_;
    QString leftSerial_;
    QString rightModel_;
    QString rightSerial_;
    hik_sync::RobotSample latestRobotSample_;
    int stationaryRobotSamples_{0};
    qint64 stationaryConfirmedNs_{0};
    StereoFramePair latestPair_;
    StereoRigCalibration seedRig_;
    std::vector<StereoCalibrationSample> samples_;
    std::vector<cv::Vec3d> boardRvecs_;
    std::vector<cv::Vec3d> boardTvecs_;
    StereoCalibrationResult result_;
    QString candidatePath_;
    QString sessionDirectory_;
    LineLaserStatus laserStatus_;
    QTimer* watchdog_{nullptr};

    QLineEdit* leftIpEdit_{nullptr};
    QLineEdit* rightIpEdit_{nullptr};
    QLineEdit* robotIpEdit_{nullptr};
    QLabel* leftStatus_{nullptr};
    QLabel* rightStatus_{nullptr};
    QLabel* robotStatus_{nullptr};
    QLabel* laserStatusLabel_{nullptr};
    QLabel* captureReadinessLabel_{nullptr};
    QLabel* sampleStatus_{nullptr};
    QLabel* resultStatus_{nullptr};
    QPushButton* connectCamerasButton_{nullptr};
    QPushButton* connectRobotButton_{nullptr};
    QPushButton* startButton_{nullptr};
    QPushButton* stopButton_{nullptr};
    QPushButton* captureButton_{nullptr};
    QPushButton* importSessionButton_{nullptr};
    QPushButton* solveButton_{nullptr};
    QPushButton* approveButton_{nullptr};
    QDoubleSpinBox* exposureSpin_{nullptr};
    QDoubleSpinBox* gainSpin_{nullptr};
    QDoubleSpinBox* fpsSpin_{nullptr};
    QDoubleSpinBox* pairSkewSpin_{nullptr};
    ImageView* leftView_{nullptr};
    ImageView* rightView_{nullptr};
    QTextEdit* logView_{nullptr};
};

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_CALIBRATION_WINDOW_H
