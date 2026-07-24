#ifndef MYLINE_HIK_FAIRINO_ROBOT_SESSION_H
#define MYLINE_HIK_FAIRINO_ROBOT_SESSION_H

#include "HikSynchronizationCore.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <QThread>

#include <atomic>
#include <mutex>

class FairinoReadOnlyWorker;

// Owns the sole FAIRINO SDK client allowed in one GUI process.  Device pages
// are clients of this session; they never create or tear down FRRobot objects.
class FairinoRobotSession final : public QObject {
    Q_OBJECT

public:
    explicit FairinoRobotSession(QObject* parent = nullptr);
    ~FairinoRobotSession() override;

    int registerClient(const QString& clientName);
    int allocateRequestId(int clientId);
    bool requestBelongsToClient(int requestId, int clientId) const;

    bool acquireExclusive(int clientId,
                          const QString& purpose,
                          QString* error = nullptr);
    void releaseExclusive(int clientId);
    bool isExclusiveOwner(int clientId) const;
    bool commandAvailableTo(int clientId) const;
    quint64 exclusiveLeaseEpochFor(int clientId) const;
    QString exclusiveOwnerDescription() const;

    bool isRunning() const;
    bool isConnected() const { return connected_; }
    bool isBusy() const { return busy_; }
    bool connectionAttempted() const { return connectionAttempted_; }
    bool isTerminal() const {
        return connectionAttempted_ && !connected_ && !connecting_;
    }

public slots:
    void connectRobot(int clientId, QString ipAddress);
    void disconnectRobot(int clientId);
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
    void shutdown();

signals:
    void connectionChanged(bool connected, QString description);
    void busyChanged(bool busy);
    void exclusiveOwnerChanged(int clientId, QString description);
    void clientError(int clientId, QString message);
    void flangePoseReady(int requestId,
                         double xMm,
                         double yMm,
                         double zMm,
                         double rxDeg,
                         double ryDeg,
                         double rzDeg,
                         qint64 hostTimestampMs);
    void robotSampleReady(int clientId,
                          quint64 leaseEpoch,
                          hik_sync::RobotSample sample);
    void realtimePeriodConfigured(int periodMs);
    void log(QString message);
    void error(int requestId, QString message);
    void motionStarted(int requestId, QString description);
    void motionFinished(int requestId,
                        bool targetReached,
                        bool motionStoppedConfirmed,
                        QString description);

signals:
    void connectWorker(QString ipAddress);
    void disconnectWorker();

private slots:
    void onWorkerConnectionChanged(bool connected, QString description);
    void onWorkerBusyChanged(bool busy);
    void onWorkerError(int requestId, QString message);

private:
    static constexpr int kRequestSequenceBits = 20;
    static constexpr int kMaximumClientId = 0x7ff;
    static constexpr int kMaximumSequence = (1 << kRequestSequenceBits) - 1;

    bool validClient(int clientId) const;
    int requestClientId(int requestId) const;
    bool prepareRequest(int requestId,
                        const QString& operation,
                        bool requireExclusive,
                        int* ownerAtQueue,
                        quint64* epochAtQueue);
    bool queuedRequestStillValid(int ownerAtQueue,
                                 quint64 epochAtQueue) const;
    void rejectStaleQueuedRequest(int requestId,
                                  const QString& operation);

    QThread workerThread_;
    FairinoReadOnlyWorker* worker_{nullptr};
    QHash<int, QString> clientNames_;
    QHash<int, int> clientSequences_;
    int nextClientId_{0};
    std::atomic<int> exclusiveClientId_{0};
    std::atomic<qint64> exclusiveLeaseStartNs_{0};
    std::atomic<quint64> exclusiveLeaseEpoch_{0};
    std::atomic<bool> workerCommandsEnabled_{true};
    // Serializes lease transitions with the final worker-side validation and
    // SDK call.  Once releaseExclusive() returns, no command admitted under
    // that lease can still enter the SDK.
    mutable std::mutex commandAdmissionMutex_;
    QString exclusivePurpose_;
    bool connected_{false};
    bool busy_{false};
    bool connecting_{false};
    bool connectionAttempted_{false};
    bool shuttingDown_{false};
    int connectionRequesterClientId_{0};
    QString requestedIpAddress_;
};

#endif  // MYLINE_HIK_FAIRINO_ROBOT_SESSION_H
