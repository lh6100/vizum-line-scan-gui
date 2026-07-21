#ifndef MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H
#define MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H

#include <QObject>
#include <QString>

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
    void log(QString message);
    void error(int requestId, QString message);
    void motionStarted(int requestId, QString description);
    void motionFinished(int requestId, bool success, QString description);

private slots:
    void pollMotionDone();

private:
    FRRobot* robot_{nullptr};
    bool connected_{false};
    bool attempted_{false};
    bool terminal_{false};
    QTimer* motionPollTimer_{nullptr};
    bool motionActive_{false};
    int motionRequestId_{-1};
    qint64 motionStartMs_{0};
    int motionTimeoutMs_{0};
};

#endif  // MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H
