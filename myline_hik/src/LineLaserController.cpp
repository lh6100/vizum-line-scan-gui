#include "LineLaserController.h"

#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QMetaObject>
#include <QPointer>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QThread>
#include <QTimer>

#include <optional>
#include <time.h>

namespace {

constexpr int kProtocolVersion = 1;
constexpr int kMaximumFrameBytes = 4096;
constexpr int kInitialReconnectMs = 500;
constexpr int kMaximumReconnectMs = 5000;

qint64 currentMonotonicRawNs() {
    timespec timestamp{};
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &timestamp) != 0) {
        return 0;
    }
    return static_cast<qint64>(timestamp.tv_sec) * 1000000000LL +
           static_cast<qint64>(timestamp.tv_nsec);
}

QString expandedPath(const QString& path) {
    const QString trimmed = path.trimmed();
    if (trimmed == QStringLiteral("~")) return QDir::homePath();
    if (trimmed.startsWith(QStringLiteral("~/"))) {
        return QDir::home().absoluteFilePath(trimmed.mid(2));
    }
    return QFileInfo(trimmed).absoluteFilePath();
}

QString protocolStateName(LineLaserState state) {
    switch (state) {
        case LineLaserState::Off: return QStringLiteral("off");
        case LineLaserState::Laser450: return QStringLiteral("laser450");
        case LineLaserState::Laser650: return QStringLiteral("laser650");
        case LineLaserState::Both: return QStringLiteral("both");
        case LineLaserState::Unknown: break;
    }
    return QStringLiteral("unknown");
}

QString commandName(LineLaserState state) {
    switch (state) {
        case LineLaserState::Off: return QStringLiteral("off");
        case LineLaserState::Laser450: return QStringLiteral("set450");
        case LineLaserState::Laser650: return QStringLiteral("set650");
        case LineLaserState::Both: return QStringLiteral("setBoth");
        case LineLaserState::Unknown: break;
    }
    return QStringLiteral("unknown");
}

struct DesiredStateRequest {
    LineLaserState state{LineLaserState::Unknown};
    quint64 commandToken{0};
};

LineLaserState parsedProtocolState(const QString& value) {
    if (value == QStringLiteral("off")) return LineLaserState::Off;
    if (value == QStringLiteral("laser450")) return LineLaserState::Laser450;
    if (value == QStringLiteral("laser650")) return LineLaserState::Laser650;
    if (value == QStringLiteral("both")) return LineLaserState::Both;
    return LineLaserState::Unknown;
}

}  // namespace

LineLaserConnectionConfig LineLaserConnectionConfig::defaults() {
    LineLaserConnectionConfig config;
    QString configBase = qEnvironmentVariable("XDG_CONFIG_HOME").trimmed();
    if (configBase.isEmpty() || !QDir::isAbsolutePath(configBase)) {
        configBase = QDir::home().absoluteFilePath(QStringLiteral(".config"));
    }
    const QString root = QDir(configBase).absoluteFilePath(
        QStringLiteral("myline_hik/laser-control"));
    config.host = QStringLiteral("192.168.1.12");
    config.port = 22;
    config.user = QStringLiteral("laserctl");
    config.privateKeyPath = QDir(root).absoluteFilePath(
        QStringLiteral("id_ed25519"));
    config.knownHostsPath = QDir(root).absoluteFilePath(
        QStringLiteral("known_hosts"));
    config.sshProgram = QStringLiteral("/usr/bin/ssh");
    return config;
}

class LineLaserWorker final : public QObject {
public:
    LineLaserWorker(
        LineLaserController* owner,
        const LineLaserConnectionConfig& config)
        : owner_(owner), config_(config) {}

    void configure(const LineLaserConnectionConfig& config) {
        ensureRuntimeObjects();
        if (process_->state() != QProcess::NotRunning ||
            connectionState_ == LineLaserConnectionState::Connecting ||
            connectionState_ == LineLaserConnectionState::Ready ||
            connectionState_ == LineLaserConnectionState::CommandPending ||
            connectionState_ == LineLaserConnectionState::Disconnecting) {
            publishCommand(
                QStringLiteral("configure"), false,
                QStringLiteral("控制连接活动时不能修改配置。"));
            return;
        }
        config_ = config;
        publishLog(QStringLiteral("激光控制连接参数已更新。"));
    }

    void connectController() {
        ensureRuntimeObjects();
        wantConnected_ = true;
        disconnectRequested_ = false;
        queuedState_.reset();
        offBarrier_ = false;
        reconnectTimer_->stop();
        if (process_->state() != QProcess::NotRunning) return;
        startSsh();
    }

    void disconnectController() {
        ensureRuntimeObjects();
        wantConnected_ = false;
        reconnectTimer_->stop();
        queuedState_.reset();
        offBarrier_ = true;
        disconnectRequested_ = true;
        heartbeatTimer_->stop();

        if (process_->state() == QProcess::NotRunning) {
            resetPending();
            disconnectRequested_ = false;
            offBarrier_ = false;
            publishUnknownStatus();
            setConnectionState(
                LineLaserConnectionState::Disconnected,
                QStringLiteral("激光控制未连接。"));
            return;
        }

        setConnectionState(
            LineLaserConnectionState::Disconnecting,
            QStringLiteral("正在关闭两路 TTL 并断开。"));
        if (pendingId_ < 0) {
            sendDesiredState(LineLaserState::Off, 0);
        }
    }

    void requestState(
            LineLaserState desired, quint64 commandToken = 0) {
        ensureRuntimeObjects();
        if (desired == LineLaserState::Unknown) return;
        if (disconnectRequested_) {
            publishCommand(
                commandName(desired), false,
                QStringLiteral("控制器正在断开。"));
            return;
        }
        if (connectionState_ != LineLaserConnectionState::Ready &&
            connectionState_ != LineLaserConnectionState::CommandPending) {
            publishCommand(
                commandName(desired), false,
                QStringLiteral("鲁班猫控制通道尚未就绪。"));
            return;
        }

        if (desired == LineLaserState::Off) {
            queuedState_ = DesiredStateRequest{
                LineLaserState::Off, commandToken};
            offBarrier_ = true;
        } else {
            if (offBarrier_) {
                publishCommand(
                    commandName(desired), false,
                    QStringLiteral("安全关光命令尚未确认，拒绝新的开光命令。"));
                return;
            }
            queuedState_ = DesiredStateRequest{desired, 0};
        }

        if (pendingId_ < 0) {
            const DesiredStateRequest next = *queuedState_;
            queuedState_.reset();
            sendDesiredState(next.state, next.commandToken);
        }
    }

    void requestStatus() {
        ensureRuntimeObjects();
        if (connectionState_ != LineLaserConnectionState::Ready ||
            pendingId_ >= 0) {
            publishCommand(
                QStringLiteral("status"), false,
                QStringLiteral("控制通道未就绪或已有命令等待回复。"));
            return;
        }
        sendRequest(QStringLiteral("status"), QJsonObject(),
                    config_.commandTimeoutMs, LineLaserState::Unknown);
    }

    void shutdown() {
        if (!process_) return;
        wantConnected_ = false;
        disconnectRequested_ = true;
        heartbeatTimer_->stop();
        reconnectTimer_->stop();
        commandTimer_->stop();
        // Closing the SSH transport is itself a fail-safe action: gateway EOF
        // revokes the daemon lease and forces both outputs LOW.
        if (process_->state() != QProcess::NotRunning) {
            process_->closeWriteChannel();
            process_->terminate();
            if (!process_->waitForFinished(300)) {
                process_->kill();
                process_->waitForFinished(300);
            }
        }
        resetPending();
    }

private:
    void ensureRuntimeObjects() {
        if (process_) return;
        process_ = new QProcess(this);
        heartbeatTimer_ = new QTimer(this);
        commandTimer_ = new QTimer(this);
        reconnectTimer_ = new QTimer(this);
        heartbeatTimer_->setTimerType(Qt::PreciseTimer);
        heartbeatTimer_->setSingleShot(false);
        commandTimer_->setSingleShot(true);
        reconnectTimer_->setSingleShot(true);

        connect(process_, &QProcess::started, this, [this]() {
            publishLog(QStringLiteral("SSH 进程已启动，等待板端安全握手。"));
            sendRequest(
                QStringLiteral("hello"),
                QJsonObject{{QStringLiteral("client"),
                             QStringLiteral("HikConstantLaserScan")}},
                config_.connectTimeoutMs, LineLaserState::Unknown);
        });
        connect(
            process_,
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this,
            [this](int exitCode, QProcess::ExitStatus exitStatus) {
                onProcessFinished(exitCode, exitStatus);
            });
        connect(process_, &QProcess::errorOccurred, this,
                [this](QProcess::ProcessError error) {
            if (error == QProcess::FailedToStart) {
                failTransport(QStringLiteral("无法启动 SSH 程序：%1")
                                  .arg(process_->errorString()));
            }
        });
        connect(process_, &QProcess::readyReadStandardOutput,
                this, [this]() { readStandardOutput(); });
        connect(process_, &QProcess::readyReadStandardError,
                this, [this]() { readStandardError(); });
        connect(heartbeatTimer_, &QTimer::timeout, this, [this]() {
            if (connectionState_ == LineLaserConnectionState::Ready &&
                pendingId_ < 0) {
                sendRequest(
                    QStringLiteral("heartbeat"), QJsonObject(),
                    config_.commandTimeoutMs, LineLaserState::Unknown);
            }
        });
        connect(commandTimer_, &QTimer::timeout, this, [this]() {
            const QString operation = pendingOperation_;
            failTransport(
                QStringLiteral("激光控制命令超时：%1").arg(operation));
        });
        connect(reconnectTimer_, &QTimer::timeout,
                this, [this]() {
            if (wantConnected_ &&
                process_->state() == QProcess::NotRunning) {
                startSsh();
            }
        });
    }

    bool validateConfig(QString* error) const {
        const QRegularExpression hostPattern(
            QStringLiteral("^[A-Za-z0-9.-]+$"));
        const QRegularExpression userPattern(
            QStringLiteral("^[A-Za-z_][A-Za-z0-9_-]*$"));
        if (!hostPattern.match(config_.host.trimmed()).hasMatch()) {
            *error = QStringLiteral("板端主机名/IP 格式无效。");
            return false;
        }
        if (!userPattern.match(config_.user.trimmed()).hasMatch()) {
            *error = QStringLiteral("SSH 用户名格式无效。");
            return false;
        }
        if (config_.port == 0) {
            *error = QStringLiteral("SSH 端口不能为 0。");
            return false;
        }
        if (!QFileInfo(expandedPath(config_.sshProgram)).isExecutable()) {
            *error = QStringLiteral("SSH 程序不可执行：%1")
                         .arg(config_.sshProgram);
            return false;
        }
        const QFileInfo keyInfo(expandedPath(config_.privateKeyPath));
        if (!keyInfo.isFile() || !keyInfo.isReadable()) {
            *error = QStringLiteral("专用 SSH 私钥不可读：%1")
                         .arg(config_.privateKeyPath);
            return false;
        }
        const QFileInfo knownHostsInfo(expandedPath(config_.knownHostsPath));
        if (!knownHostsInfo.isFile() || !knownHostsInfo.isReadable()) {
            *error = QStringLiteral("固定 known_hosts 不可读：%1")
                         .arg(config_.knownHostsPath);
            return false;
        }
        if (config_.connectTimeoutMs < 500 ||
            config_.commandTimeoutMs < 200 ||
            config_.heartbeatIntervalMs < 100 ||
            config_.heartbeatIntervalMs >= 1500) {
            *error = QStringLiteral(
                "连接/命令/心跳超时配置超出安全范围。");
            return false;
        }
        return true;
    }

    void startSsh() {
        ++transportGeneration_;
        if (transportGeneration_ == 0) ++transportGeneration_;
        QString error;
        if (!validateConfig(&error)) {
            setConnectionState(LineLaserConnectionState::Fault, error);
            publishFault(error);
            scheduleReconnect();
            return;
        }
        stdoutBuffer_.clear();
        stderrBuffer_.clear();
        resetPending();
        queuedState_.reset();
        offBarrier_ = false;
        publishUnknownStatus();
        setConnectionState(
            LineLaserConnectionState::Connecting,
            QStringLiteral("正在连接鲁班猫 %1:%2。")
                .arg(config_.host).arg(config_.port));

        QStringList arguments;
        arguments
            << QStringLiteral("-T")
            << QStringLiteral("-p") << QString::number(config_.port)
            << QStringLiteral("-i") << expandedPath(config_.privateKeyPath)
            << QStringLiteral("-o") << QStringLiteral("BatchMode=yes")
            << QStringLiteral("-o") << QStringLiteral("IdentitiesOnly=yes")
            << QStringLiteral("-o") << QStringLiteral("PasswordAuthentication=no")
            << QStringLiteral("-o") << QStringLiteral("KbdInteractiveAuthentication=no")
            << QStringLiteral("-o") << QStringLiteral("ChallengeResponseAuthentication=no")
            << QStringLiteral("-o") << QStringLiteral("StrictHostKeyChecking=yes")
            << QStringLiteral("-o")
            << QStringLiteral("UserKnownHostsFile=%1")
                   .arg(expandedPath(config_.knownHostsPath))
            << QStringLiteral("-o")
            << QStringLiteral("ConnectTimeout=%1")
                   .arg(qMax(1, config_.connectTimeoutMs / 1000))
            << QStringLiteral("-o") << QStringLiteral("ServerAliveInterval=1")
            << QStringLiteral("-o") << QStringLiteral("ServerAliveCountMax=2")
            << QStringLiteral("%1@%2")
                   .arg(config_.user.trimmed(), config_.host.trimmed());

        QProcessEnvironment environment =
            QProcessEnvironment::systemEnvironment();
        environment.insert(QStringLiteral("LC_ALL"), QStringLiteral("C"));
        process_->setProcessEnvironment(environment);
        process_->setProcessChannelMode(QProcess::SeparateChannels);
        process_->setProgram(expandedPath(config_.sshProgram));
        process_->setArguments(arguments);
        process_->start(QIODevice::ReadWrite);
    }

    void sendDesiredState(
            LineLaserState desired, quint64 commandToken) {
        if (desired == LineLaserState::Off) {
            sendRequest(
                QStringLiteral("off"), QJsonObject(),
                config_.commandTimeoutMs, desired, commandToken);
            return;
        }
        sendRequest(
            QStringLiteral("set"),
            QJsonObject{{QStringLiteral("state"),
                         protocolStateName(desired)}},
            config_.commandTimeoutMs, desired);
    }

    void sendRequest(
        const QString& operation,
        const QJsonObject& fields,
        int timeoutMs,
        LineLaserState desired,
        quint64 commandToken = 0) {
        if (pendingId_ >= 0 ||
            process_->state() != QProcess::Running) {
            failTransport(QStringLiteral(
                "协议状态错误：无法发送 %1。").arg(operation));
            return;
        }
        const qint64 requestId = ++nextRequestId_;
        QJsonObject object = fields;
        object.insert(QStringLiteral("v"), kProtocolVersion);
        object.insert(QStringLiteral("id"), requestId);
        object.insert(QStringLiteral("op"), operation);
        QByteArray payload = QJsonDocument(object).toJson(
            QJsonDocument::Compact);
        payload.append('\n');
        if (payload.size() > kMaximumFrameBytes) {
            failTransport(QStringLiteral("内部协议帧超过安全上限。"));
            return;
        }
        const qint64 written = process_->write(payload);
        if (written != payload.size()) {
            failTransport(QStringLiteral("无法完整写入 SSH 控制通道。"));
            return;
        }
        pendingId_ = requestId;
        pendingOperation_ = operation;
        pendingDesiredState_ = desired;
        pendingCommandToken_ = commandToken;
        commandTimer_->start(timeoutMs);
        if (operation != QStringLiteral("heartbeat") &&
            operation != QStringLiteral("hello")) {
            setConnectionState(
                disconnectRequested_
                    ? LineLaserConnectionState::Disconnecting
                    : LineLaserConnectionState::CommandPending,
                QStringLiteral("等待板端确认：%1").arg(operation));
        }
    }

    void readStandardOutput() {
        stdoutBuffer_.append(process_->readAllStandardOutput());
        if (stdoutBuffer_.size() > kMaximumFrameBytes * 2) {
            failTransport(QStringLiteral("板端协议缓冲区超过安全上限。"));
            return;
        }
        while (true) {
            const int newline = stdoutBuffer_.indexOf('\n');
            if (newline < 0) break;
            const QByteArray frame = stdoutBuffer_.left(newline);
            stdoutBuffer_.remove(0, newline + 1);
            if (frame.size() > kMaximumFrameBytes) {
                failTransport(QStringLiteral("板端回复帧超过安全上限。"));
                return;
            }
            handleFrame(frame);
            if (connectionState_ == LineLaserConnectionState::Fault) return;
        }
    }

    void readStandardError() {
        stderrBuffer_.append(process_->readAllStandardError());
        if (stderrBuffer_.size() > 8192) {
            stderrBuffer_ = stderrBuffer_.right(8192);
        }
        while (true) {
            const int newline = stderrBuffer_.indexOf('\n');
            if (newline < 0) break;
            QString line = QString::fromUtf8(
                stderrBuffer_.left(newline)).trimmed();
            stderrBuffer_.remove(0, newline + 1);
            if (line.size() > 512) line = line.left(512);
            if (!line.isEmpty()) {
                publishLog(QStringLiteral("SSH: %1").arg(line));
            }
        }
    }

    void handleFrame(const QByteArray& frame) {
        QJsonParseError parseError;
        const QJsonDocument document =
            QJsonDocument::fromJson(frame, &parseError);
        if (parseError.error != QJsonParseError::NoError ||
            !document.isObject()) {
            failTransport(QStringLiteral("板端返回了无效 JSON。"));
            return;
        }
        const QJsonObject reply = document.object();
        const QJsonValue idValue = reply.value(QStringLiteral("id"));
        if (!idValue.isDouble()) {
            failTransport(QStringLiteral("板端回复缺少 request id。"));
            return;
        }
        const qint64 replyId = static_cast<qint64>(idValue.toDouble());
        if (replyId != pendingId_) {
            publishLog(QStringLiteral(
                "忽略过期板端回复 id=%1，当前 id=%2。")
                .arg(replyId).arg(pendingId_));
            return;
        }

        const QString operation = pendingOperation_;
        const LineLaserState desired = pendingDesiredState_;
        const quint64 commandToken = pendingCommandToken_;
        commandTimer_->stop();
        resetPending();

        LineLaserStatus status;
        QString statusError;
        if (!parseStatus(reply.value(QStringLiteral("status")),
                         &status, &statusError)) {
            failTransport(statusError);
            return;
        }
        const bool replyOk =
            reply.value(QStringLiteral("ok")).isBool() &&
            reply.value(QStringLiteral("ok")).toBool();
        if (replyOk && operation == QStringLiteral("off") &&
            desired == LineLaserState::Off && commandToken > 0 &&
            status.state == LineLaserState::Off &&
            !status.ttl450High && !status.ttl650High &&
            status.leaseActive &&
            status.fault.trimmed().isEmpty()) {
            status.acknowledgedOffCommandToken = commandToken;
        }
        publishStatus(status);

        if (!replyOk) {
            const QJsonObject errorObject =
                reply.value(QStringLiteral("error")).toObject();
            const QString code =
                errorObject.value(QStringLiteral("code")).toString(
                    QStringLiteral("UNKNOWN"));
            const QString message =
                errorObject.value(QStringLiteral("message")).toString(
                    QStringLiteral("板端拒绝命令"));
            const QString detail =
                QStringLiteral("%1: %2").arg(code, message);
            if (operation == QStringLiteral("set") ||
                operation == QStringLiteral("off") ||
                operation == QStringLiteral("status")) {
                publishCommand(
                    desired == LineLaserState::Unknown
                        ? operation : commandName(desired),
                    false, detail);
            }
            failTransport(detail);
            return;
        }

        if (operation != QStringLiteral("goodbye") &&
            !status.leaseActive) {
            failTransport(QStringLiteral(
                "板端未确认当前 SSH 通道持有激光控制租约。"));
            return;
        }

        if (operation == QStringLiteral("hello")) {
            if (status.state != LineLaserState::Off ||
                status.ttl450High || status.ttl650High) {
                failTransport(QStringLiteral(
                    "安全握手没有确认两路 LOW。"));
                return;
            }
            if (!status.boardModel.contains(
                    QStringLiteral("LubanCat-4"),
                    Qt::CaseInsensitive) ||
                status.daemonGeneration.trimmed().isEmpty() ||
                status.laser450PhysicalPin != 11 ||
                status.laser450Gpio != 15 ||
                status.laser450ChipOffset != 15 ||
                status.laser650PhysicalPin != 7 ||
                status.laser650Gpio != 16 ||
                status.laser650ChipOffset != 16) {
                failTransport(QStringLiteral(
                    "板端型号或 V1 引脚映射不匹配："
                    "要求 450nm=Pin11/GPIO15、650nm=Pin7/GPIO16。"));
                return;
            }
            reconnectDelayMs_ = kInitialReconnectMs;
            heartbeatTimer_->setInterval(config_.heartbeatIntervalMs);
            heartbeatTimer_->start();
            setConnectionState(
                LineLaserConnectionState::Ready,
                QStringLiteral("鲁班猫已连接，两路 TTL 为 LOW。"));
        } else if (operation == QStringLiteral("set") ||
                   operation == QStringLiteral("off")) {
            if (status.state != desired) {
                failTransport(QStringLiteral(
                    "板端 ACK 状态与请求不一致。"));
                return;
            }
            publishCommand(
                commandName(desired), true,
                QStringLiteral("板端已确认 %1。")
                    .arg(protocolStateName(desired)));
            if (desired == LineLaserState::Off) offBarrier_ = false;
        } else if (operation == QStringLiteral("status")) {
            publishCommand(
                QStringLiteral("status"), true,
                QStringLiteral("状态已刷新。"));
        }

        if (disconnectRequested_) {
            if (operation == QStringLiteral("off")) {
                sendRequest(
                    QStringLiteral("goodbye"), QJsonObject(),
                    config_.commandTimeoutMs, LineLaserState::Unknown);
                process_->closeWriteChannel();
            } else if (operation != QStringLiteral("goodbye") &&
                       pendingId_ < 0) {
                sendDesiredState(LineLaserState::Off, 0);
            }
            return;
        }

        if (queuedState_ && pendingId_ < 0) {
            const DesiredStateRequest next = *queuedState_;
            queuedState_.reset();
            sendDesiredState(next.state, next.commandToken);
            return;
        }

        if (operation != QStringLiteral("goodbye") &&
            connectionState_ != LineLaserConnectionState::Ready) {
            setConnectionState(
                LineLaserConnectionState::Ready,
                QStringLiteral("鲁班猫激光控制就绪。"));
        }
    }

    bool parseStatus(
        const QJsonValue& value,
        LineLaserStatus* status,
        QString* error) const {
        if (!value.isObject()) {
            *error = QStringLiteral("板端回复缺少完整状态。");
            return false;
        }
        const QJsonObject object = value.toObject();
        const QJsonValue stateValue = object.value(QStringLiteral("state"));
        const QJsonValue high450 =
            object.value(QStringLiteral("ttl450_high"));
        const QJsonValue high650 =
            object.value(QStringLiteral("ttl650_high"));
        const QJsonValue lease =
            object.value(QStringLiteral("lease_active"));
        if (!stateValue.isString() || !high450.isBool() ||
            !high650.isBool() || !lease.isBool()) {
            *error = QStringLiteral("板端状态字段类型无效。");
            return false;
        }
        status->reachable = true;
        status->state = parsedProtocolState(stateValue.toString());
        status->ttl450High = high450.toBool();
        status->ttl650High = high650.toBool();
        status->leaseActive = lease.toBool();
        status->leaseRemainingMs =
            object.value(QStringLiteral("lease_remaining_ms")).toInt(0);
        status->daemonGeneration =
            object.value(QStringLiteral("generation")).toString();
        status->boardModel =
            object.value(QStringLiteral("board_model")).toString();
        const QJsonObject pinMap =
            object.value(QStringLiteral("pin_map")).toObject();
        const QJsonObject pin450 =
            pinMap.value(QStringLiteral("laser450")).toObject();
        const QJsonObject pin650 =
            pinMap.value(QStringLiteral("laser650")).toObject();
        status->laser450PhysicalPin =
            pin450.value(QStringLiteral("physical_pin")).toInt(0);
        status->laser450Gpio =
            pin450.value(QStringLiteral("gpio")).toInt(0);
        status->laser450ChipOffset =
            pin450.value(QStringLiteral("chip_offset")).toInt(-1);
        status->laser650PhysicalPin =
            pin650.value(QStringLiteral("physical_pin")).toInt(0);
        status->laser650Gpio =
            pin650.value(QStringLiteral("gpio")).toInt(0);
        status->laser650ChipOffset =
            pin650.value(QStringLiteral("chip_offset")).toInt(-1);
        status->fault =
            object.value(QStringLiteral("fault")).toString();
        if (status->state == LineLaserState::Unknown) {
            *error = QStringLiteral("板端返回未知激光状态。");
            return false;
        }
        if ((status->state == LineLaserState::Off &&
             (status->ttl450High || status->ttl650High)) ||
            (status->state == LineLaserState::Laser450 &&
             (!status->ttl450High || status->ttl650High)) ||
            (status->state == LineLaserState::Laser650 &&
             (status->ttl450High || !status->ttl650High)) ||
            (status->state == LineLaserState::Both &&
             (!status->ttl450High || !status->ttl650High))) {
            *error = QStringLiteral("板端状态与 GPIO 读回自相矛盾。");
            return false;
        }
        return true;
    }

    void onProcessFinished(
        int exitCode, QProcess::ExitStatus exitStatus) {
        heartbeatTimer_->stop();
        commandTimer_->stop();
        const QString pending = pendingOperation_;
        resetPending();
        queuedState_.reset();
        offBarrier_ = false;
        publishUnknownStatus();

        if (disconnectRequested_ || !wantConnected_) {
            disconnectRequested_ = false;
            setConnectionState(
                LineLaserConnectionState::Disconnected,
                QStringLiteral("激光控制已断开；板端连接租约已释放。"));
            return;
        }

        const QString detail = QStringLiteral(
            "SSH 控制通道退出：code=%1, status=%2%3")
            .arg(exitCode)
            .arg(exitStatus == QProcess::NormalExit
                     ? QStringLiteral("normal")
                     : QStringLiteral("crash"))
            .arg(pending.isEmpty()
                     ? QString()
                     : QStringLiteral(", pending=") + pending);
        setConnectionState(LineLaserConnectionState::Fault, detail);
        publishFault(detail);
        scheduleReconnect();
    }

    void failTransport(const QString& detail) {
        commandTimer_->stop();
        heartbeatTimer_->stop();
        const QString failedOperation = pendingOperation_;
        const LineLaserState failedDesired = pendingDesiredState_;
        resetPending();
        queuedState_.reset();
        offBarrier_ = false;
        publishUnknownStatus();
        setConnectionState(LineLaserConnectionState::Fault, detail);
        publishFault(detail);
        if (!failedOperation.isEmpty() &&
            failedOperation != QStringLiteral("heartbeat") &&
            failedOperation != QStringLiteral("hello")) {
            publishCommand(
                failedDesired == LineLaserState::Unknown
                    ? failedOperation : commandName(failedDesired),
                false, detail);
        }
        if (process_->state() != QProcess::NotRunning) {
            process_->kill();
        } else {
            scheduleReconnect();
        }
    }

    void scheduleReconnect() {
        if (!wantConnected_ || !config_.autoReconnect ||
            reconnectTimer_->isActive()) {
            return;
        }
        reconnectTimer_->start(reconnectDelayMs_);
        publishLog(QStringLiteral("%1 ms 后重连激光控制；不会自动恢复开光。")
                       .arg(reconnectDelayMs_));
        reconnectDelayMs_ =
            qMin(kMaximumReconnectMs, reconnectDelayMs_ * 2);
    }

    void resetPending() {
        pendingId_ = -1;
        pendingOperation_.clear();
        pendingDesiredState_ = LineLaserState::Unknown;
        pendingCommandToken_ = 0;
    }

    void setConnectionState(
        LineLaserConnectionState state, const QString& detail) {
        connectionState_ = state;
        publishConnection(state, detail);
    }

    void publishUnknownStatus() {
        LineLaserStatus status;
        status.reachable = false;
        status.state = LineLaserState::Unknown;
        publishStatus(status);
    }

    template <typename Callback>
    void postToOwner(Callback callback) {
        QPointer<LineLaserController> owner(owner_);
        if (!owner) return;
        QMetaObject::invokeMethod(
            owner,
            [owner, callback]() {
                if (owner) callback(owner.data());
            },
            Qt::QueuedConnection);
    }

    void publishConnection(
        LineLaserConnectionState state, const QString& detail) {
        postToOwner([state, detail](LineLaserController* owner) {
            owner->postConnectionState(state, detail);
        });
    }

    void publishStatus(LineLaserStatus status) {
        status.sourceMonotonicNs = currentMonotonicRawNs();
        status.eventSequence = ++statusEventSequence_;
        if (status.eventSequence == 0) {
            status.eventSequence = ++statusEventSequence_;
        }
        status.transportGeneration = transportGeneration_;
        postToOwner([status](LineLaserController* owner) {
            owner->postStatus(status);
        });
    }

    void publishCommand(
        const QString& command, bool success, const QString& detail) {
        postToOwner([command, success, detail](LineLaserController* owner) {
            owner->postCommandFinished(command, success, detail);
        });
    }

    void publishFault(const QString& detail) {
        postToOwner([detail](LineLaserController* owner) {
            owner->postFault(detail);
        });
    }

    void publishLog(const QString& message) {
        postToOwner([message](LineLaserController* owner) {
            owner->postLog(message);
        });
    }

    QPointer<LineLaserController> owner_;
    LineLaserConnectionConfig config_;
    QProcess* process_{nullptr};
    QTimer* heartbeatTimer_{nullptr};
    QTimer* commandTimer_{nullptr};
    QTimer* reconnectTimer_{nullptr};
    QByteArray stdoutBuffer_;
    QByteArray stderrBuffer_;
    LineLaserConnectionState connectionState_{
        LineLaserConnectionState::Disconnected};
    bool wantConnected_{false};
    bool disconnectRequested_{false};
    bool offBarrier_{false};
    qint64 nextRequestId_{0};
    qint64 pendingId_{-1};
    QString pendingOperation_;
    LineLaserState pendingDesiredState_{LineLaserState::Unknown};
    quint64 pendingCommandToken_{0};
    std::optional<DesiredStateRequest> queuedState_;
    int reconnectDelayMs_{kInitialReconnectMs};
    quint64 statusEventSequence_{0};
    quint64 transportGeneration_{0};
};

LineLaserController::LineLaserController(
    const LineLaserConnectionConfig& config, QObject* parent)
    : QObject(parent),
      workerThread_(new QThread(this)),
      worker_(new LineLaserWorker(this, config)) {
    qRegisterMetaType<LineLaserConnectionState>(
        "LineLaserConnectionState");
    qRegisterMetaType<LineLaserState>("LineLaserState");
    qRegisterMetaType<LineLaserStatus>("LineLaserStatus");
    qRegisterMetaType<LineLaserConnectionConfig>(
        "LineLaserConnectionConfig");
    workerThread_->setObjectName(QStringLiteral("LineLaserControlThread"));
    worker_->moveToThread(workerThread_);
    connect(workerThread_, &QThread::finished,
            worker_, &QObject::deleteLater);
    workerThread_->start();
}

LineLaserController::~LineLaserController() {
    if (worker_ && workerThread_ && workerThread_->isRunning()) {
        QMetaObject::invokeMethod(
            worker_,
            [worker = worker_]() { worker->shutdown(); },
            Qt::BlockingQueuedConnection);
        workerThread_->quit();
        workerThread_->wait();
    }
    worker_ = nullptr;
}

void LineLaserController::setConnectionConfig(
    const LineLaserConnectionConfig& config) {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_, config]() { worker->configure(config); },
        Qt::QueuedConnection);
}

void LineLaserController::connectController() {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_]() { worker->connectController(); },
        Qt::QueuedConnection);
}

void LineLaserController::disconnectController() {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_]() { worker->disconnectController(); },
        Qt::QueuedConnection);
}

void LineLaserController::set450() {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_]() {
            worker->requestState(LineLaserState::Laser450);
        },
        Qt::QueuedConnection);
}

void LineLaserController::set650() {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_]() {
            worker->requestState(LineLaserState::Laser650);
        },
        Qt::QueuedConnection);
}

void LineLaserController::setBoth() {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_]() {
            worker->requestState(LineLaserState::Both);
        },
        Qt::QueuedConnection);
}

quint64 LineLaserController::requestOffTracked() {
    if (!worker_) return 0;
    quint64 token = nextOffCommandToken_.fetch_add(
                        1, std::memory_order_acq_rel) + 1;
    if (token == 0) {
        token = nextOffCommandToken_.fetch_add(
                    1, std::memory_order_acq_rel) + 1;
    }
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_, token]() {
            worker->requestState(LineLaserState::Off, token);
        },
        Qt::QueuedConnection);
    return token;
}

void LineLaserController::off() {
    (void)requestOffTracked();
}

void LineLaserController::requestStatus() {
    if (!worker_) return;
    QMetaObject::invokeMethod(
        worker_,
        [worker = worker_]() { worker->requestStatus(); },
        Qt::QueuedConnection);
}

void LineLaserController::postConnectionState(
    LineLaserConnectionState state, const QString& detail) {
    emit connectionStateChanged(state, detail);
}

void LineLaserController::postStatus(const LineLaserStatus& status) {
    emit statusChanged(status);
}

void LineLaserController::postCommandFinished(
    const QString& command, bool success, const QString& detail) {
    emit commandFinished(command, success, detail);
}

void LineLaserController::postFault(const QString& detail) {
    emit faultOccurred(detail);
}

void LineLaserController::postLog(const QString& message) {
    emit logMessage(message);
}
