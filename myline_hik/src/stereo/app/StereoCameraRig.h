#ifndef MYLINE_HIK_STEREO_CAMERA_RIG_H
#define MYLINE_HIK_STEREO_CAMERA_RIG_H

#include "stereo/core/StereoFrameSynchronizer.h"

#include <QObject>
#include <QThread>

class HikCameraWorker;

namespace hik_stereo {

enum class StereoCameraSide { Left, Right };

class StereoCameraRig final : public QObject {
    Q_OBJECT

public:
    explicit StereoCameraRig(QObject* parent = nullptr);
    ~StereoCameraRig() override;

    bool leftConnected() const { return leftConnected_; }
    bool rightConnected() const { return rightConnected_; }
    bool running() const { return running_; }
    StereoFramePairerStatistics pairerStatistics() const;

public slots:
    void connectCameras(QString leftIp, QString rightIp);
    void disconnectCameras();
    void start(double leftExposureUs,
               double rightExposureUs,
               double gainDb,
               double framesPerSecond,
               double maximumPairSkewMs);
    void stop();

signals:
    void connectionChanged(hik_stereo::StereoCameraSide side,
                           bool connected,
                           QString description);
    void identityChanged(hik_stereo::StereoCameraSide side,
                         QString model,
                         QString serial,
                         QString ipAddress);
    void pairReady(hik_stereo::StereoFramePair pair);
    void started();
    void stopped(bool confirmed, QString description);
    void error(QString message);
    void log(QString message);

private:
    void setupCamera(HikCameraWorker** worker,
                     QThread* thread,
                     StereoCameraSide side);
    void handleFrame(StereoCameraSide side, hik_sync::CameraFrame frame);
    void handleContinuousStarted(StereoCameraSide side);
    void handleContinuousStopped(StereoCameraSide side,
                                 bool confirmed,
                                 const QString& description);
    void shutdownWorker(HikCameraWorker* worker, QThread* thread);

    QThread leftThread_;
    QThread rightThread_;
    HikCameraWorker* leftWorker_{nullptr};
    HikCameraWorker* rightWorker_{nullptr};
    StereoFramePairer pairer_;
    bool leftConnected_{false};
    bool rightConnected_{false};
    bool leftStarted_{false};
    bool rightStarted_{false};
    bool leftStoppedConfirmed_{true};
    bool rightStoppedConfirmed_{true};
    bool running_{false};
    bool stopping_{false};
};

}  // namespace hik_stereo

Q_DECLARE_METATYPE(hik_stereo::StereoCameraSide)
Q_DECLARE_METATYPE(hik_stereo::StereoFramePair)

#endif  // MYLINE_HIK_STEREO_CAMERA_RIG_H
