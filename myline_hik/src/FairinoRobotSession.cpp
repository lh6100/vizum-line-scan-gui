#include "FairinoRobotSession.h"

#include "FairinoReadOnlyWorker.h"

#include <QMetaObject>

#include <utility>

FairinoRobotSession::FairinoRobotSession(QObject* parent)
    : QObject(parent) {
    qRegisterMetaType<hik_sync::RobotSample>("hik_sync::RobotSample");
    qRegisterMetaType<hik_adaptive::RobotPathEvaluation>(
        "hik_adaptive::RobotPathEvaluation");

    worker_ = new FairinoReadOnlyWorker;
    worker_->moveToThread(&workerThread_);
    connect(&workerThread_, &QThread::finished,
            worker_, &QObject::deleteLater);

    connect(this, &FairinoRobotSession::connectWorker,
            worker_, &FairinoReadOnlyWorker::connectRobot,
            Qt::QueuedConnection);
    connect(this, &FairinoRobotSession::disconnectWorker,
            worker_, &FairinoReadOnlyWorker::disconnectRobot,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::connectionChanged,
            this, &FairinoRobotSession::onWorkerConnectionChanged,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::busyChanged,
            this, &FairinoRobotSession::onWorkerBusyChanged,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::flangePoseReady,
            this, &FairinoRobotSession::flangePoseReady,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::motionStarted,
            this, &FairinoRobotSession::motionStarted,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::motionFinished,
            this, &FairinoRobotSession::motionFinished,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::motionTimingMeasured,
            this, &FairinoRobotSession::motionTimingMeasured,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::kinematicPathEvaluated,
            this, &FairinoRobotSession::kinematicPathEvaluated,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::kinematicPathBatchFinished,
            this, &FairinoRobotSession::kinematicPathBatchFinished,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::log,
            this, &FairinoRobotSession::log,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::error,
            this, &FairinoRobotSession::onWorkerError,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::realtimePeriodConfigured,
            this, &FairinoRobotSession::realtimePeriodConfigured,
            Qt::QueuedConnection);
    connect(worker_, &FairinoReadOnlyWorker::robotSampleReady,
            this,
            [this](hik_sync::RobotSample sample) {
                const int clientId =
                    exclusiveClientId_.load(std::memory_order_acquire);
                if (clientId <= 0) return;
                const qint64 leaseStartNs =
                    exclusiveLeaseStartNs_.load(
                        std::memory_order_acquire);
                const quint64 leaseEpoch =
                    exclusiveLeaseEpoch_.load(
                        std::memory_order_acquire);
                if (leaseStartNs <= 0 ||
                    sample.hostReceiveNs < leaseStartNs ||
                    leaseEpoch == 0) {
                    return;
                }
                emit robotSampleReady(
                    clientId, leaseEpoch, std::move(sample));
            },
            Qt::DirectConnection);

    workerThread_.setObjectName(QStringLiteral("FairinoRobotSessionWorker"));
    workerThread_.start();
}

FairinoRobotSession::~FairinoRobotSession() {
    shutdown();
}

int FairinoRobotSession::registerClient(const QString& clientName) {
    if (nextClientId_ >= kMaximumClientId) {
        return 0;
    }
    const int clientId = ++nextClientId_;
    clientNames_.insert(
        clientId,
        clientName.trimmed().isEmpty()
            ? QStringLiteral("client_%1").arg(clientId)
            : clientName.trimmed());
    clientSequences_.insert(clientId, 0);
    return clientId;
}

int FairinoRobotSession::allocateRequestId(int clientId) {
    if (!validClient(clientId)) {
        return -1;
    }
    int sequence = clientSequences_.value(clientId, 0) + 1;
    if (sequence > kMaximumSequence) {
        sequence = 1;
    }
    clientSequences_.insert(clientId, sequence);
    return (clientId << kRequestSequenceBits) | sequence;
}

bool FairinoRobotSession::requestBelongsToClient(
        int requestId, int clientId) const {
    return validClient(clientId) && requestId > 0 &&
           requestClientId(requestId) == clientId;
}

bool FairinoRobotSession::acquireExclusive(
        int clientId, const QString& purpose, QString* error) {
    const std::lock_guard<std::mutex> admissionLock(
        commandAdmissionMutex_);
    if (error) error->clear();
    if (!validClient(clientId)) {
        if (error) *error = QStringLiteral("未知的 FR5 页面客户端。");
        return false;
    }
    int expected = 0;
    if (exclusiveClientId_.compare_exchange_strong(
            expected, -1, std::memory_order_acq_rel)) {
        exclusivePurpose_ = purpose.trimmed();
        exclusiveLeaseStartNs_.store(
            hik_sync::getMonotonicRawNs(), std::memory_order_release);
        quint64 epoch = exclusiveLeaseEpoch_.fetch_add(
            1, std::memory_order_acq_rel) + 1;
        if (epoch == 0) {
            epoch = exclusiveLeaseEpoch_.fetch_add(
                1, std::memory_order_acq_rel) + 1;
        }
        exclusiveClientId_.store(clientId, std::memory_order_release);
        emit exclusiveOwnerChanged(clientId, exclusiveOwnerDescription());
        return true;
    }
    if (expected == clientId) {
        return true;
    }
    if (error) {
        *error = QStringLiteral("FR5 正由 %1 独占，当前请求已拒绝。")
                     .arg(exclusiveOwnerDescription());
    }
    return false;
}

void FairinoRobotSession::releaseExclusive(int clientId) {
    const std::lock_guard<std::mutex> admissionLock(
        commandAdmissionMutex_);
    int expected = clientId;
    if (!exclusiveClientId_.compare_exchange_strong(
            expected, -1, std::memory_order_acq_rel)) {
        return;
    }
    exclusiveLeaseStartNs_.store(0, std::memory_order_release);
    exclusivePurpose_.clear();
    exclusiveClientId_.store(0, std::memory_order_release);
    emit exclusiveOwnerChanged(0, QString());
}

bool FairinoRobotSession::isExclusiveOwner(int clientId) const {
    return validClient(clientId) &&
           exclusiveClientId_.load(std::memory_order_acquire) == clientId;
}

bool FairinoRobotSession::commandAvailableTo(int clientId) const {
    if (!validClient(clientId)) return false;
    const int owner = exclusiveClientId_.load(std::memory_order_acquire);
    return owner == 0 || owner == clientId;
}

quint64 FairinoRobotSession::exclusiveLeaseEpochFor(
        int clientId) const {
    if (!isExclusiveOwner(clientId)) return 0;
    return exclusiveLeaseEpoch_.load(std::memory_order_acquire);
}

QString FairinoRobotSession::exclusiveOwnerDescription() const {
    const int owner = exclusiveClientId_.load(std::memory_order_acquire);
    if (owner <= 0) return QString();
    QString description = clientNames_.value(
        owner, QStringLiteral("client_%1").arg(owner));
    if (!exclusivePurpose_.isEmpty()) {
        description += QStringLiteral("（%1）").arg(exclusivePurpose_);
    }
    return description;
}

bool FairinoRobotSession::isRunning() const {
    return worker_ && workerThread_.isRunning() && !shuttingDown_;
}

void FairinoRobotSession::connectRobot(
        int clientId, QString ipAddress) {
    if (!validClient(clientId) || shuttingDown_) {
        emit clientError(clientId, QStringLiteral("共享 FR5 会话不可用。"));
        return;
    }
    ipAddress = ipAddress.trimmed();
    if (connected_) {
        emit connectionChanged(
            true,
            QStringLiteral("共享 FR5 已连接：%1").arg(requestedIpAddress_));
        return;
    }
    if (connecting_) {
        if (ipAddress != requestedIpAddress_) {
            emit clientError(
                clientId,
                QStringLiteral("共享 FR5 正在连接 %1，不能同时连接 %2。")
                    .arg(requestedIpAddress_, ipAddress));
        }
        return;
    }
    if (connectionAttempted_) {
        emit clientError(
            clientId,
            QStringLiteral(
                "本进程的共享 FAIRINO RPC 会话已经结束，"
                "为避免 SDK 残留线程竞态，请重启程序后再连接。"));
        return;
    }
    connectionAttempted_ = true;
    connecting_ = true;
    connectionRequesterClientId_ = clientId;
    requestedIpAddress_ = ipAddress;
    emit connectWorker(ipAddress);
}

void FairinoRobotSession::disconnectRobot(int clientId) {
    if (!validClient(clientId) || shuttingDown_) return;
    const int owner = exclusiveClientId_.load(std::memory_order_acquire);
    if (owner != 0) {
        emit clientError(
            clientId,
            owner == clientId
                ? QStringLiteral(
                      "当前 %1 尚未完成，不能断开共享 FR5。")
                      .arg(exclusiveOwnerDescription())
                : QStringLiteral(
                      "FR5 正由 %1 独占，不能断开共享连接。")
                      .arg(exclusiveOwnerDescription()));
        return;
    }
    if (!connected_ && !connecting_) return;
    connectionRequesterClientId_ = clientId;
    emit disconnectWorker();
}

void FairinoRobotSession::readFlangePose(int requestId) {
    const QString operation = QStringLiteral("读取法兰位姿");
    int ownerAtQueue = 0;
    quint64 epochAtQueue = 0;
    if (!prepareRequest(requestId, operation, false,
                        &ownerAtQueue, &epochAtQueue)) {
        return;
    }
    FairinoReadOnlyWorker* const worker = worker_;
    if (!QMetaObject::invokeMethod(
            worker,
            [this, worker, requestId, operation,
             ownerAtQueue, epochAtQueue]() {
                const std::lock_guard<std::mutex> admissionLock(
                    commandAdmissionMutex_);
                if (!queuedRequestStillValid(
                        ownerAtQueue, epochAtQueue)) {
                    rejectStaleQueuedRequest(requestId, operation);
                    return;
                }
                worker->readFlangePose(requestId);
            },
            Qt::QueuedConnection)) {
        emit error(requestId,
                   QStringLiteral("%1 无法进入共享 FR5 队列。")
                       .arg(operation));
    }
}

void FairinoRobotSession::moveLinear(
        int requestId,
        double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg,
        double velocityPercent, double accelerationPercent,
        int timeoutMs) {
    const QString operation = QStringLiteral("MoveL");
    int ownerAtQueue = 0;
    quint64 epochAtQueue = 0;
    if (!prepareRequest(requestId, operation, true,
                        &ownerAtQueue, &epochAtQueue)) {
        return;
    }
    FairinoReadOnlyWorker* const worker = worker_;
    if (!QMetaObject::invokeMethod(
            worker,
            [this, worker, requestId, operation,
             ownerAtQueue, epochAtQueue,
             xMm, yMm, zMm, rxDeg, ryDeg, rzDeg,
             velocityPercent, accelerationPercent, timeoutMs]() {
                const std::lock_guard<std::mutex> admissionLock(
                    commandAdmissionMutex_);
                if (!queuedRequestStillValid(
                        ownerAtQueue, epochAtQueue)) {
                    rejectStaleQueuedRequest(requestId, operation);
                    return;
                }
                worker->moveLinear(
                    requestId, xMm, yMm, zMm,
                    rxDeg, ryDeg, rzDeg,
                    velocityPercent, accelerationPercent,
                    timeoutMs);
            },
            Qt::QueuedConnection)) {
        emit error(requestId,
                   QStringLiteral("%1 无法进入共享 FR5 队列。")
                       .arg(operation));
    }
}

void FairinoRobotSession::moveLinearPhysical(
        int requestId,
        double xMm, double yMm, double zMm,
        double rxDeg, double ryDeg, double rzDeg,
        double speedMmS, double accelerationMmS2,
        int timeoutMs) {
    const QString operation = QStringLiteral("物理速度 MoveL");
    int ownerAtQueue = 0;
    quint64 epochAtQueue = 0;
    if (!prepareRequest(requestId, operation, true,
                        &ownerAtQueue, &epochAtQueue)) {
        return;
    }
    FairinoReadOnlyWorker* const worker = worker_;
    if (!QMetaObject::invokeMethod(
            worker,
            [this, worker, requestId, operation,
             ownerAtQueue, epochAtQueue,
             xMm, yMm, zMm, rxDeg, ryDeg, rzDeg,
             speedMmS, accelerationMmS2, timeoutMs]() {
                const std::lock_guard<std::mutex> admissionLock(
                    commandAdmissionMutex_);
                if (!queuedRequestStillValid(
                        ownerAtQueue, epochAtQueue)) {
                    rejectStaleQueuedRequest(requestId, operation);
                    return;
                }
                worker->moveLinearPhysical(
                    requestId, xMm, yMm, zMm,
                    rxDeg, ryDeg, rzDeg,
                    speedMmS, accelerationMmS2,
                    timeoutMs);
            },
            Qt::QueuedConnection)) {
        emit error(requestId,
                   QStringLiteral("%1 无法进入共享 FR5 队列。")
                       .arg(operation));
    }
}

void FairinoRobotSession::executeAdaptiveTrajectory(
        int requestId,
        std::vector<hik_adaptive::ScanSegment> segments,
        int timeoutMs) {
    const QString operation =
        QStringLiteral("预提交 LINE/ARC 自适应轨迹");
    int ownerAtQueue = 0;
    quint64 epochAtQueue = 0;
    if (!prepareRequest(requestId, operation, true,
                        &ownerAtQueue, &epochAtQueue)) {
        return;
    }
    FairinoReadOnlyWorker* const worker = worker_;
    if (!QMetaObject::invokeMethod(
            worker,
            [this, worker, requestId, operation,
             ownerAtQueue, epochAtQueue,
             segments = std::move(segments), timeoutMs]() mutable {
                const std::lock_guard<std::mutex> admissionLock(
                    commandAdmissionMutex_);
                if (!queuedRequestStillValid(
                        ownerAtQueue, epochAtQueue)) {
                    rejectStaleQueuedRequest(requestId, operation);
                    return;
                }
                worker->executeAdaptiveTrajectory(
                    requestId, std::move(segments), timeoutMs);
            },
            Qt::QueuedConnection)) {
        emit error(requestId,
                   QStringLiteral("%1 无法进入共享 FR5 队列。")
                       .arg(operation));
    }
}

void FairinoRobotSession::stopMotion(int requestId) {
    const QString operation = QStringLiteral("StopMotion");
    int ownerAtQueue = 0;
    quint64 epochAtQueue = 0;
    if (!prepareRequest(requestId, operation, true,
                        &ownerAtQueue, &epochAtQueue)) {
        return;
    }
    FairinoReadOnlyWorker* const worker = worker_;
    if (!QMetaObject::invokeMethod(
            worker,
            [this, worker, requestId, operation,
             ownerAtQueue, epochAtQueue]() {
                const std::lock_guard<std::mutex> admissionLock(
                    commandAdmissionMutex_);
                if (!queuedRequestStillValid(
                        ownerAtQueue, epochAtQueue)) {
                    rejectStaleQueuedRequest(requestId, operation);
                    return;
                }
                worker->stopMotion(requestId);
            },
            Qt::QueuedConnection)) {
        emit error(requestId,
                   QStringLiteral("%1 无法进入共享 FR5 队列。")
                       .arg(operation));
    }
}

void FairinoRobotSession::evaluateKinematicPaths(
        int requestId,
        std::vector<hik_fr5::PathEvaluationRequest> requests,
        hik_fr5::PathEvaluationOptions options) {
    const QString operation =
        QStringLiteral("候选路径 IK/奇异性评估");
    int ownerAtQueue = 0;
    quint64 epochAtQueue = 0;
    if (!prepareRequest(requestId, operation, true,
                        &ownerAtQueue, &epochAtQueue)) {
        return;
    }
    FairinoReadOnlyWorker* const worker = worker_;
    if (!QMetaObject::invokeMethod(
            worker,
            [this, worker, requestId, operation,
             ownerAtQueue, epochAtQueue,
             requests = std::move(requests), options]() mutable {
                const std::lock_guard<std::mutex> admissionLock(
                    commandAdmissionMutex_);
                if (!queuedRequestStillValid(
                        ownerAtQueue, epochAtQueue)) {
                    rejectStaleQueuedRequest(requestId, operation);
                    return;
                }
                worker->evaluateKinematicPaths(
                    requestId, std::move(requests), options);
            },
            Qt::QueuedConnection)) {
        emit error(requestId,
                   QStringLiteral("%1 无法进入共享 FR5 队列。")
                       .arg(operation));
    }
}

void FairinoRobotSession::shutdown() {
    if (shuttingDown_) return;
    shuttingDown_ = true;
    workerCommandsEnabled_.store(false, std::memory_order_release);
    if (worker_ && workerThread_.isRunning()) {
        QMetaObject::invokeMethod(
            worker_, "disconnectRobot", Qt::BlockingQueuedConnection);
        workerThread_.quit();
        if (!workerThread_.wait(5000)) {
            workerThread_.wait();
        }
    }
    exclusiveClientId_.store(0, std::memory_order_release);
    exclusiveLeaseStartNs_.store(0, std::memory_order_release);
    exclusivePurpose_.clear();
    worker_ = nullptr;
    connected_ = false;
    busy_ = false;
    connecting_ = false;
}

void FairinoRobotSession::onWorkerConnectionChanged(
        bool connected, QString description) {
    connected_ = connected;
    connecting_ = false;
    if (!connected) {
        busy_ = false;
    }
    emit connectionChanged(connected, std::move(description));
}

void FairinoRobotSession::onWorkerBusyChanged(bool busy) {
    busy_ = busy;
    emit busyChanged(busy);
}

void FairinoRobotSession::onWorkerError(
        int requestId, QString message) {
    if (requestId < 0 && connectionRequesterClientId_ > 0) {
        emit clientError(connectionRequesterClientId_, std::move(message));
        return;
    }
    emit error(requestId, std::move(message));
}

bool FairinoRobotSession::validClient(int clientId) const {
    return clientId > 0 && clientNames_.contains(clientId);
}

int FairinoRobotSession::requestClientId(int requestId) const {
    if (requestId <= 0) return 0;
    return requestId >> kRequestSequenceBits;
}

bool FairinoRobotSession::prepareRequest(
        int requestId,
        const QString& operation,
        bool requireExclusive,
        int* ownerAtQueue,
        quint64* epochAtQueue) {
    const std::lock_guard<std::mutex> admissionLock(
        commandAdmissionMutex_);
    const int clientId = requestClientId(requestId);
    if (!validClient(clientId)) {
        emit error(requestId,
                   QStringLiteral("%1 请求的客户端标识无效。").arg(operation));
        return false;
    }
    const int owner =
        exclusiveClientId_.load(std::memory_order_acquire);
    if (requireExclusive && owner != clientId) {
        emit error(
            requestId,
            owner > 0
                ? QStringLiteral(
                      "%1 已拒绝：FR5 正由 %2 独占。")
                      .arg(operation, exclusiveOwnerDescription())
                : QStringLiteral(
                      "%1 必须先取得本页的 FR5 独占命令租约。")
                      .arg(operation));
        return false;
    }
    if (!requireExclusive && owner != 0 && owner != clientId) {
        emit error(
            requestId,
            QStringLiteral("%1 已拒绝：FR5 正由 %2 独占。")
                .arg(operation, exclusiveOwnerDescription()));
        return false;
    }
    if (!isRunning() ||
        !workerCommandsEnabled_.load(std::memory_order_acquire)) {
        emit error(requestId, QStringLiteral("共享 FR5 会话不可用。"));
        return false;
    }
    if (ownerAtQueue) *ownerAtQueue = owner;
    if (epochAtQueue) {
        *epochAtQueue =
            exclusiveLeaseEpoch_.load(std::memory_order_acquire);
    }
    emit log(QStringLiteral("%1 请求来自 %2。")
                 .arg(operation, clientNames_.value(clientId)));
    return true;
}

bool FairinoRobotSession::queuedRequestStillValid(
        int ownerAtQueue, quint64 epochAtQueue) const {
    return workerCommandsEnabled_.load(std::memory_order_acquire) &&
           exclusiveClientId_.load(std::memory_order_acquire) ==
               ownerAtQueue &&
           exclusiveLeaseEpoch_.load(std::memory_order_acquire) ==
               epochAtQueue;
}

void FairinoRobotSession::rejectStaleQueuedRequest(
        int requestId, const QString& operation) {
    QMetaObject::invokeMethod(
        this,
        [this, requestId, operation]() {
            if (shuttingDown_) return;
            emit error(
                requestId,
                QStringLiteral(
                    "%1 在 SDK 执行前因租约代次已变化而取消。")
                    .arg(operation));
        },
        Qt::QueuedConnection);
}
