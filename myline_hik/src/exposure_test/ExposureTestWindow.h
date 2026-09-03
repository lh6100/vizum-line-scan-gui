#ifndef MYLINE_HIK_EXPOSURE_TEST_WINDOW_H
#define MYLINE_HIK_EXPOSURE_TEST_WINDOW_H

#include "LineLaserController.h"

#include <QWidget>

#include <array>
#include <memory>

class QCheckBox;
class QCloseEvent;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QThread;
class LineLaserController;

class ExposureTestWindow final : public QWidget {
    Q_OBJECT

public:
    explicit ExposureTestWindow(LineLaserController* laserController,
                                QWidget* parent = nullptr);
    ~ExposureTestWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    struct CameraPanel;

    void buildUi();
    QWidget* buildCameraPanel(int cameraIndex);
    void setupCameraWorker(int cameraIndex);
    void shutdownCameraWorkers();
    void connectCamera(int cameraIndex);
    void disconnectCamera(int cameraIndex);
    void captureCamera(int cameraIndex);
    void connectBothCameras();
    void disconnectBothCameras();
    void captureBothCameras();
    void chooseOutputDirectory();
    void openOutputDirectory();
    void applyLaserSelection();
    void turnLasersOff();
    void updateUi();
    void appendLog(const QString& message, bool error = false);
    QString defaultOutputDirectory() const;
    QString laserStateName(LineLaserState state) const;

    LineLaserController* laserController_{nullptr};
    std::array<std::unique_ptr<CameraPanel>, 2> cameras_;
    QLineEdit* outputDirectoryEdit_{nullptr};
    QPushButton* chooseOutputButton_{nullptr};
    QPushButton* openOutputButton_{nullptr};
    QPushButton* connectBothButton_{nullptr};
    QPushButton* disconnectBothButton_{nullptr};
    QPushButton* captureBothButton_{nullptr};
    QPushButton* laserConnectButton_{nullptr};
    QPushButton* laserDisconnectButton_{nullptr};
    QCheckBox* laser450Check_{nullptr};
    QCheckBox* laser650Check_{nullptr};
    QPushButton* applyLaserButton_{nullptr};
    QPushButton* lasersOffButton_{nullptr};
    QLabel* laserConnectionLabel_{nullptr};
    QLabel* laserReadbackLabel_{nullptr};
    QPlainTextEdit* logEdit_{nullptr};
    LineLaserConnectionState laserConnectionState_{
        LineLaserConnectionState::Disconnected};
    LineLaserStatus latestLaserStatus_;
    int nextRequestId_{0};
    bool shuttingDown_{false};
};

#endif // MYLINE_HIK_EXPOSURE_TEST_WINDOW_H
