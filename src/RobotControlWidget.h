#ifndef ROBOTCONTROLWIDGET_H
#define ROBOTCONTROLWIDGET_H

#include "geometry/TransformUtils.h"
#include "robot_capture/RobotOffsetCapturePlanner.h"
#include "robot_client/FairinoRobotClient.h"
#include "welding/WeldPathPlanner.h"

#include <QVector>
#include <QWidget>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class MotionTracePlotWidget;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;

class RobotControlWidget : public QWidget {
    Q_OBJECT

public:
    explicit RobotControlWidget(QWidget* parent = nullptr);
    ~RobotControlWidget() override;

public slots:
    void setCameraLineFromFit(double sx, double sy, double sz, double ex, double ey, double ez);
    void onLeftRightEyeCaptureSaved(int requestId, bool ok, QString leftPath, QString rightPath, QString desc);

signals:
    void asyncLog(QString msg);
    void asyncMotionFinished(int err);
    void asyncMotionTraceReady(QVector<double> time, QVector<double> x, QVector<double> y, QVector<double> z);
    void requestLeftRightEyeCapture(int requestId, QString leftPath, QString rightPath,
                                    int frameRate, int exposure, int gain, bool keepLaserOn);

private slots:
    void connectRobot();
    void disconnectRobot();
    void readCurrentPose();
    void servoOn();
    void servoOff();
    void setAutoMode();
    void resetError();
    void stopMotion();
    void pauseMotion();
    void resumeMotion();
    void calculateTargets();
    void executeMove();
    void executeLastMove();
    void executeOffsetCapture();
    void updateSpeedModeUi();
    void onMotionFinished(int err);
    void onMotionTraceReady(QVector<double> time, QVector<double> x, QVector<double> y, QVector<double> z);

private:
    void appendLog(const QString& msg);
    void setConnectedUi(bool connected);
    void setMotionUi(bool running);
    void loadConfigs();
    void syncMotionConfigFromUi();
    weld_motion::WeldLinePlan buildCurrentPlan() const;
    void logPlan(const weld_motion::WeldLinePlan& plan);
    void rememberPlan(const weld_motion::WeldLinePlan& plan, const QString& source);
    void startMoveWithPlan(const weld_motion::WeldLinePlan& plan, const QString& source);
    void updateMotionTracePlots(const QVector<double>& time,
                                const QVector<double>& x,
                                const QVector<double>& y,
                                const QVector<double>& z);
    QString saveMotionTraceCsv(const QVector<double>& time,
                               const QVector<double>& x,
                               const QVector<double>& y,
                               const QVector<double>& z,
                               const QVector<double>& dx,
                               const QVector<double>& dy,
                               const QVector<double>& dz,
                               const QVector<double>& vx,
                               const QVector<double>& vy,
                               const QVector<double>& vz,
                               const QVector<double>& ax,
                               const QVector<double>& ay,
                               const QVector<double>& az);
    bool requestCaptureBlocking(QString leftPath, QString rightPath,
                                int frameRate, int exposure, int gain, bool keepLaserOn,
                                int timeoutMs = 15000);
    bool ensureRobotReadyForRealMove(const QString& actionName);
    QString offsetCaptureDirPath() const;
    weld_geometry::Vec3 startCameraPoint() const;
    weld_geometry::Vec3 endCameraPoint() const;
    weld_geometry::Pose6D fallbackFlangePose() const;
    weld_geometry::Pose6D fallbackTcpPose() const;
    QDoubleSpinBox* makeCoordSpin(double value);
    QDoubleSpinBox* makePoseSpin(double value);
    QDoubleSpinBox* makeSpeedSpin(double value);

    QLineEdit* m_ipEdit{};
    QLabel* m_statusLabel{};
    QCheckBox* m_dryRunCheck{};
    QCheckBox* m_enableMotionCheck{};
    QCheckBox* m_physicalSpeedModeCheck{};
    QDoubleSpinBox* m_safeHeightSpin{};
    QDoubleSpinBox* m_retractHeightSpin{};
    QDoubleSpinBox* m_travelVelSpin{};
    QDoubleSpinBox* m_weldVelSpin{};
    QDoubleSpinBox* m_accSpin{};
    QDoubleSpinBox* m_ovlSpin{};
    QDoubleSpinBox* m_offsetXSpin{};
    QDoubleSpinBox* m_offsetYSpin{};
    QDoubleSpinBox* m_offsetZSpin{};

    QComboBox* m_captureAxisCombo{};
    QDoubleSpinBox* m_captureDistanceSpin{};
    QDoubleSpinBox* m_captureStepSpin{};
    QDoubleSpinBox* m_captureVelSpin{};
    QDoubleSpinBox* m_captureFrameRateSpin{};
    QDoubleSpinBox* m_captureExposureSpin{};
    QDoubleSpinBox* m_captureGainSpin{};
    QCheckBox* m_captureKeepLaserOnCheck{};

    QDoubleSpinBox* m_startX{};
    QDoubleSpinBox* m_startY{};
    QDoubleSpinBox* m_startZ{};
    QDoubleSpinBox* m_endX{};
    QDoubleSpinBox* m_endY{};
    QDoubleSpinBox* m_endZ{};

    QDoubleSpinBox* m_flangeX{};
    QDoubleSpinBox* m_flangeY{};
    QDoubleSpinBox* m_flangeZ{};
    QDoubleSpinBox* m_flangeRx{};
    QDoubleSpinBox* m_flangeRy{};
    QDoubleSpinBox* m_flangeRz{};
    QDoubleSpinBox* m_tcpX{};
    QDoubleSpinBox* m_tcpY{};
    QDoubleSpinBox* m_tcpZ{};
    QDoubleSpinBox* m_tcpRx{};
    QDoubleSpinBox* m_tcpRy{};
    QDoubleSpinBox* m_tcpRz{};

    QPushButton* m_btnConnect{};
    QPushButton* m_btnDisconnect{};
    QPushButton* m_btnReadPose{};
    QPushButton* m_btnServoOn{};
    QPushButton* m_btnServoOff{};
    QPushButton* m_btnAutoMode{};
    QPushButton* m_btnResetError{};
    QPushButton* m_btnStop{};
    QPushButton* m_btnPause{};
    QPushButton* m_btnResume{};
    QPushButton* m_btnCalculate{};
    QPushButton* m_btnExecute{};
    QPushButton* m_btnExecuteLast{};
    QPushButton* m_btnOffsetCapture{};
    QLabel* m_lastPlanLabel{};
    QTabWidget* m_pages{};
    QWidget* m_tracePage{};
    QLabel* m_traceSummaryLabel{};
    MotionTracePlotWidget* m_displacementPlot{};
    MotionTracePlotWidget* m_velocityPlot{};
    MotionTracePlotWidget* m_accelerationPlot{};
    QPlainTextEdit* m_log{};

    fairino_client::FairinoRobotClient m_robot;
    fairino_client::RobotConfig m_robotConfig;
    weld_motion::HandEyeConfig m_handEyeConfig;
    weld_motion::ToolConfig m_toolConfig;
    weld_motion::WeldMotionConfig m_motionConfig;
    weld_motion::WeldLinePlan m_lastPlan;
    bool m_hasCurrentPose{false};
    bool m_hasLastPlan{false};
    std::atomic<bool> m_motionRunning{false};
    std::atomic<bool> m_stopRequested{false};
    std::thread m_motionThread;

    std::mutex m_captureMutex;
    std::condition_variable m_captureCv;
    int m_nextCaptureRequestId{0};
    int m_captureResultRequestId{-1};
    bool m_captureResultReady{false};
    bool m_captureResultOk{false};
    QString m_captureResultDesc;
};

#endif // ROBOTCONTROLWIDGET_H
