#ifndef ROBOTCONTROLWIDGET_H
#define ROBOTCONTROLWIDGET_H

#include "geometry/TransformUtils.h"
#include "robot_client/FairinoRobotClient.h"
#include "welding/WeldPathPlanner.h"

#include <QWidget>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;

class RobotControlWidget : public QWidget {
    Q_OBJECT

public:
    explicit RobotControlWidget(QWidget* parent = nullptr);
    ~RobotControlWidget() override;

public slots:
    void setCameraLineFromFit(double sx, double sy, double sz, double ex, double ey, double ez);

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

private:
    void appendLog(const QString& msg);
    void setConnectedUi(bool connected);
    void loadConfigs();
    void syncMotionConfigFromUi();
    weld_geometry::Vec3 startCameraPoint() const;
    weld_geometry::Vec3 endCameraPoint() const;
    weld_geometry::Pose6D fallbackFlangePose() const;
    weld_geometry::Pose6D fallbackTcpPose() const;
    QDoubleSpinBox* makeCoordSpin(double value);
    QDoubleSpinBox* makePoseSpin(double value);

    QLineEdit* m_ipEdit{};
    QLabel* m_statusLabel{};
    QCheckBox* m_dryRunCheck{};
    QCheckBox* m_enableMotionCheck{};
    QDoubleSpinBox* m_safeHeightSpin{};
    QDoubleSpinBox* m_velSpin{};
    QDoubleSpinBox* m_accSpin{};
    QDoubleSpinBox* m_ovlSpin{};
    QDoubleSpinBox* m_offsetXSpin{};
    QDoubleSpinBox* m_offsetYSpin{};
    QDoubleSpinBox* m_offsetZSpin{};

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
    QPlainTextEdit* m_log{};

    fairino_client::FairinoRobotClient m_robot;
    fairino_client::RobotConfig m_robotConfig;
    weld_motion::HandEyeConfig m_handEyeConfig;
    weld_motion::ToolConfig m_toolConfig;
    weld_motion::WeldMotionConfig m_motionConfig;
    bool m_hasCurrentPose{false};
};

#endif // ROBOTCONTROLWIDGET_H
