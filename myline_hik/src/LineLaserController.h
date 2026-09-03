#ifndef MYLINE_HIK_LINE_LASER_CONTROLLER_H
#define MYLINE_HIK_LINE_LASER_CONTROLLER_H

#include <QMetaType>
#include <QObject>
#include <QString>

#include <atomic>

class QThread;
class LineLaserWorker;

enum class LineLaserConnectionState {
    Disconnected,
    Connecting,
    Ready,
    CommandPending,
    Disconnecting,
    Fault
};
Q_DECLARE_METATYPE(LineLaserConnectionState)

enum class LineLaserState {
    Unknown,
    Off,
    Laser450,
    Laser650,
    Both
};
Q_DECLARE_METATYPE(LineLaserState)

struct LineLaserStatus {
    // Stamped in the controller worker when this event is produced.  Consumers
    // must use this source time rather than the later GUI delivery time when
    // deciding whether a GPIO readback is fresh.
    qint64 sourceMonotonicNs{0};
    quint64 eventSequence{0};
    quint64 transportGeneration{0};
    // Non-zero only when this exact status came from a successfully validated
    // tracked "off" reply.  A later token also satisfies an earlier off
    // request because tracked tokens are process-monotonic.
    quint64 acknowledgedOffCommandToken{0};
    bool reachable{false};
    LineLaserState state{LineLaserState::Unknown};
    bool ttl450High{false};
    bool ttl650High{false};
    bool leaseActive{false};
    int leaseRemainingMs{0};
    QString daemonGeneration;
    QString boardModel;
    int laser450PhysicalPin{0};
    int laser450Gpio{0};
    int laser450ChipOffset{-1};
    int laser650PhysicalPin{0};
    int laser650Gpio{0};
    int laser650ChipOffset{-1};
    QString fault;
};
Q_DECLARE_METATYPE(LineLaserStatus)

struct LineLaserConnectionConfig {
    QString host;
    quint16 port{22};
    QString user;
    QString privateKeyPath;
    QString knownHostsPath;
    QString sshProgram;
    int connectTimeoutMs{3000};
    int commandTimeoutMs{1000};
    int heartbeatIntervalMs{500};
    bool autoReconnect{true};

    static LineLaserConnectionConfig defaults();
};
Q_DECLARE_METATYPE(LineLaserConnectionConfig)

// Thread-safe GUI facade.  The QProcess, protocol parser, heartbeat, and
// reconnect timers live in a private worker thread; public slots may be called
// from the GUI thread without blocking it.
class LineLaserController final : public QObject {
    Q_OBJECT

public:
    explicit LineLaserController(
        const LineLaserConnectionConfig& config =
            LineLaserConnectionConfig::defaults(),
        QObject* parent = nullptr);
    ~LineLaserController() override;

    LineLaserController(const LineLaserController&) = delete;
    LineLaserController& operator=(const LineLaserController&) = delete;

    // Queues an OFF command and returns its process-local causal token.
    // Terminal safety barriers should require a status whose acknowledged
    // token is at least this value; heartbeat/status replies carry token 0.
    quint64 requestOffTracked();

public slots:
    // Configuration changes are accepted only while no SSH process is active.
    void setConnectionConfig(const LineLaserConnectionConfig& config);
    void connectController();
    void disconnectController();

    // These are desired-state commands, not claims about optical emission.
    // Success is reported only after a matching daemon ACK and GPIO readback.
    void set450();
    void set650();
    void setBoth();
    void off();
    void requestStatus();

signals:
    void connectionStateChanged(
        LineLaserConnectionState state, QString detail);
    void statusChanged(LineLaserStatus status);
    void commandFinished(QString command, bool success, QString detail);
    void faultOccurred(QString detail);
    void logMessage(QString message);

private:
    friend class LineLaserWorker;

    void postConnectionState(
        LineLaserConnectionState state, const QString& detail);
    void postStatus(const LineLaserStatus& status);
    void postCommandFinished(
        const QString& command, bool success, const QString& detail);
    void postFault(const QString& detail);
    void postLog(const QString& message);

    QThread* workerThread_{nullptr};
    LineLaserWorker* worker_{nullptr};
    std::atomic<quint64> nextOffCommandToken_{0};
};

#endif  // MYLINE_HIK_LINE_LASER_CONTROLLER_H
