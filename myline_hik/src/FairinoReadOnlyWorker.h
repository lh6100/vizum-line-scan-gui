#ifndef MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H
#define MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H

#include "HikSynchronizationCore.h"
#include "Fr5PathEvaluator.h"

#include <QObject>
#include <QString>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

Q_DECLARE_METATYPE(hik_sync::RobotSample)

class FRRobot;
class FairinoRealtimeReceiverState;
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
    void executeAdaptiveTrajectory(
        int requestId,
        std::vector<hik_adaptive::ScanSegment> segments,
        int timeoutMs);
    void stopMotion(int requestId);
    void evaluateKinematicPaths(
        int requestId,
        std::vector<hik_fr5::PathEvaluationRequest> requests,
        hik_fr5::PathEvaluationOptions options);

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
    void motionFinished(int requestId,
                        bool targetReached,
                        bool motionStoppedConfirmed,
                        QString description);
    void motionTimingMeasured(int requestId,
                              qint64 elapsedMs,
                              bool targetReached);
    void kinematicPathEvaluated(
        int requestId,
        int actionId,
        hik_adaptive::RobotPathEvaluation evaluation);
    void kinematicPathBatchFinished(
        int requestId,
        bool completed,
        QString description);

private slots:
    void pollMotionDone();

private:
    bool startRealtimeReceiving();
    void stopRealtimeReceiving();
    void realtimeConsumerLoop();
    void moveLinearImpl(int requestId,
                        double xMm, double yMm, double zMm,
                        double rxDeg, double ryDeg, double rzDeg,
                        double speedValue, double accelerationValue,
                        int timeoutMs, bool physicalMode);
    void beginStopConfirmation(const QString& context);
    bool confirmStoppedBlocking(int timeoutMs);
    void finishMotion(bool targetReached,
                      bool motionStoppedConfirmed,
                      const QString& description);

    FRRobot* robot_{nullptr};
    bool connected_{false};
    bool attempted_{false};
    bool terminal_{false};
    QTimer* motionPollTimer_{nullptr};
    std::atomic<bool> realtimeReceiving_{false};
    std::thread realtimeThread_;
    std::unique_ptr<FairinoRealtimeReceiverState> realtimeReceiver_;
    bool motionActive_{false};
    bool motionStopUnconfirmed_{false};
    bool stopConfirmationPending_{false};
    int motionRequestId_{-1};
    qint64 motionStartMs_{0};
    int motionTimeoutMs_{0};
    qint64 stopConfirmationStartMs_{0};
    qint64 lastStopSampleReceiveNs_{0};
    int stationaryStopSamples_{0};
    std::mutex realtimeStopSnapshotMutex_;
    qint64 latestRealtimeHostReceiveNs_{0};
    int latestRealtimeMotionDone_{0};
    double latestRealtimeLinearSpeedMmS_{0.0};
    double latestRealtimeAngularSpeedDegS_{0.0};
};

#endif  // MYLINE_HIK_FAIRINO_READ_ONLY_WORKER_H
