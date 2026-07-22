#ifndef MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H
#define MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H

#include "HikSynchronizationCore.h"

#include <QObject>
#include <QString>

Q_DECLARE_METATYPE(hik_sync::RobotSample)

class FRRobot;
class QTimer;

class FairinoReadOnlyWorker final : public QObject {
    Q_OBJECT

public:
    explicit FairinoReadOnlyWorker(QObject* parent = nullptr);
    ~FairinoReadOnlyWorker() override;

public slots:
    void connectRobot(QString ipAddress);
    void disconnectRobot();
    void readFlangePose(int requestId);
    void moveLinear(int requestId,
                    double xMm,
                    double yMm,
                    double zMm,
                    double rxDeg,
                    double ryDeg,
                    double rzDeg,
                    double velocityPercent,
                    double accelerationPercent,
                    int timeoutMs);
    void moveLinearPhysical(int requestId,
                            double xMm,
                            double yMm,
                            double zMm,
                            double rxDeg,
                            double ryDeg,
                            double rzDeg,
                            double speedMmS,
                            double accelerationMmS2,
                            int timeoutMs);
    void stopMotion(int requestId);

signals:
    void connectionChanged(bool connected, QString description);
    void busyChanged(bool busy);
    void flangePoseReady(int requestId,
                         double xMm,
                         double yMm,
                         double zMm,
                         double rxDeg,
                         double ryDeg,
                         double rzDeg,
                         qint64 hostTimestampMs);
    void robotSampleReady(hik_sync::RobotSample sample);
    void realtimePeriodConfigured(int periodMs);
    void log(QString message);
    void error(int requestId, QString message);
    void motionStarted(int requestId, QString description);
    void motionFinished(int requestId, bool success, QString description);

private slots:
    void pollMotionDone();
    void pollRealtimeState();

private:
    void moveLinearImpl(int requestId,
                        double xMm, double yMm, double zMm,
                        double rxDeg, double ryDeg, double rzDeg,
                        double speedValue, double accelerationValue,
                        int timeoutMs, bool physicalMode);

    FRRobot* robot_{nullptr};
    bool connected_{false};
    bool attempted_{false};
    bool terminal_{false};
    QTimer* motionPollTimer_{nullptr};
    QTimer* realtimePollTimer_{nullptr};
    bool motionActive_{false};
    int motionRequestId_{-1};
    qint64 motionStartMs_{0};
    int motionTimeoutMs_{0};
    bool haveRealtimeFrame_{false};
    uint8_t lastRealtimeFrame_{0U};
};

#endif  // MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H
