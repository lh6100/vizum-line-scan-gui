#include "HikCameraWorker.h"
#include "LineLaserController.h"
#include "LineLaserDeviceProfile.h"
#include "exposure_test/ExposureTestStorage.h"

#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QSaveFile>
#include <QTextStream>
#include <QThread>
#include <QTimer>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <limits>
#include <memory>
#include <vector>

namespace {

constexpr int kCameraCount = 2;
std::atomic<bool> gInterruptRequested{false};

void handleSignal(int) {
    gInterruptRequested.store(true, std::memory_order_release);
}

QString laserStateName(LineLaserState state) {
    switch (state) {
    case LineLaserState::Off: return QStringLiteral("off");
    case LineLaserState::Laser450: return QStringLiteral("laser450");
    case LineLaserState::Laser650: return QStringLiteral("laser650");
    case LineLaserState::Both: return QStringLiteral("both");
    case LineLaserState::Unknown: break;
    }
    return QStringLiteral("unknown");
}

bool laserBothConfirmed(const LineLaserStatus& status) {
    return status.reachable && status.state == LineLaserState::Both &&
           status.ttl450High && status.ttl650High;
}

bool laserOffConfirmed(const LineLaserStatus& status, quint64 token) {
    return status.reachable && status.state == LineLaserState::Off &&
           !status.ttl450High && !status.ttl650High && token != 0 &&
           status.acknowledgedOffCommandToken >= token;
}

QDateTime captureDateTime(qint64 hostTimestamp) {
    // The current MVS cameras report a Unix-epoch millisecond host timestamp.
    // Keep the raw value in captures.csv and fall back safely if another model
    // uses an undocumented unit.
    constexpr qint64 kEarliestPlausibleMs = 1577836800000LL; // 2020-01-01
    constexpr qint64 kLatestPlausibleMs = 4102444800000LL;   // 2100-01-01
    if (hostTimestamp >= kEarliestPlausibleMs &&
        hostTimestamp < kLatestPlausibleMs) {
        return QDateTime::fromMSecsSinceEpoch(hostTimestamp);
    }
    return QDateTime::currentDateTime();
}

bool writeJsonFile(const QString& path,
                   const QJsonObject& object,
                   QString* error = nullptr) {
    if (error) error->clear();
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly)) {
        if (error) *error = file.errorString();
        return false;
    }
    const QByteArray data = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (file.write(data) != data.size()) {
        if (error) *error = file.errorString();
        file.cancelWriting();
        return false;
    }
    if (!file.commit()) {
        if (error) *error = file.errorString();
        return false;
    }
    return true;
}

struct SweepConfig {
    int startExposureUs{1000};
    int stopExposureUs{5000};
    int stepExposureUs{100};
    double gainDb{0.0};
    int timeoutMs{3000};
    int connectTimeoutMs{15000};
    int laserSettleMs{1000};
    int interStepMs{100};
    int warmupFrames{1};
    int framesPerExposure{1};
    QString outputDirectory;
    std::array<QString, kCameraCount> cameraIps;

    std::vector<int> exposures() const {
        std::vector<int> values;
        if (stepExposureUs <= 0 || startExposureUs > stopExposureUs) {
            return values;
        }
        for (int exposure = startExposureUs;
             exposure <= stopExposureUs;
             exposure += stepExposureUs) {
            values.push_back(exposure);
            if (exposure > std::numeric_limits<int>::max() - stepExposureUs) {
                break;
            }
        }
        return values;
    }
};

struct PendingFrame {
    bool received{false};
    QImage image;
    quint64 frameNo{0};
    quint64 deviceTimestamp{0};
    qint64 hostTimestamp{0};
    double exposureUs{0.0};
    double gainDb{0.0};
};

class ExposureSweepRunner final : public QObject {
public:
    ExposureSweepRunner(const SweepConfig& config,
                        LineLaserController* laserController,
                        QObject* parent = nullptr)
        : QObject(parent), config_(config), laserController_(laserController),
          exposures_(config.exposures()) {
        const std::vector<LineLaserDeviceProfile>& profiles =
            lineLaserDeviceProfiles();
        for (int index = 0; index < kCameraCount; ++index) {
            cameras_[index].profile = &profiles.at(static_cast<std::size_t>(index));
        }

        connectionTimer_.setSingleShot(true);
        captureTimer_.setSingleShot(true);
        shutdownTimer_.setSingleShot(true);
        interruptTimer_.setInterval(100);
        connect(&interruptTimer_, &QTimer::timeout, this, [this]() {
            if (gInterruptRequested.exchange(false, std::memory_order_acq_rel)) {
                fail(QStringLiteral("收到中断信号，采集已停止"));
            }
        });
        connect(&connectionTimer_, &QTimer::timeout, this, [this]() {
            fail(QStringLiteral("连接相机或等待双激光 TTL 确认超时"));
        });
        connect(&captureTimer_, &QTimer::timeout, this, [this]() {
            fail(QStringLiteral("曝光 %1 us 的双相机采集超时")
                     .arg(currentExposureUs()));
        });
        connect(&shutdownTimer_, &QTimer::timeout, this, [this]() {
            qCritical().noquote() << QStringLiteral(
                "关闭激光未在时限内得到板端确认；请立即人工确认两路激光已熄灭");
            exitCode_ = 1;
            finalMessage_ += QStringLiteral("；关闭激光未得到板端确认");
            cleanupAndExit(false);
        });

        connect(laserController_, &LineLaserController::connectionStateChanged,
                this, [this](LineLaserConnectionState state,
                             const QString& detail) {
            qInfo().noquote() << QStringLiteral("TTL连接：%1").arg(detail);
            if (shuttingDown_) return;
            if (state == LineLaserConnectionState::Ready &&
                !laserCommandRequested_) {
                laserCommandRequested_ = true;
                laserController_->setBoth();
            } else if (state == LineLaserConnectionState::Fault) {
                fail(QStringLiteral("TTL控制进入故障状态：%1").arg(detail));
            }
        });
        connect(laserController_, &LineLaserController::statusChanged,
                this, [this](const LineLaserStatus& status) {
            latestLaserStatus_ = status;
            if (shuttingDown_) {
                if (laserOffConfirmed(status, offCommandToken_)) {
                    cleanupAndExit(true);
                }
                return;
            }
            if (laserBothConfirmed(status)) {
                laserReady_ = true;
                maybeStartSweep();
            } else if (sweepStarted_ && status.reachable &&
                       status.state != LineLaserState::Unknown) {
                fail(QStringLiteral(
                    "采集期间双激光 TTL 状态丢失：state=%1, 450=%2, 650=%3")
                         .arg(laserStateName(status.state))
                         .arg(status.ttl450High ? QStringLiteral("HIGH")
                                                : QStringLiteral("LOW"))
                         .arg(status.ttl650High ? QStringLiteral("HIGH")
                                                : QStringLiteral("LOW")));
            }
        });
        connect(laserController_, &LineLaserController::commandFinished,
                this, [this](const QString& command, bool success,
                             const QString& detail) {
            qInfo().noquote() << QStringLiteral("TTL命令 %1：%2")
                                     .arg(command, detail);
            if (!success && !shuttingDown_) {
                fail(QStringLiteral("TTL命令 %1 失败：%2")
                         .arg(command, detail));
            }
        });
        connect(laserController_, &LineLaserController::faultOccurred,
                this, [this](const QString& detail) {
            if (!shuttingDown_) {
                fail(QStringLiteral("TTL控制故障：%1").arg(detail));
            }
        });
    }

    ~ExposureSweepRunner() override {
        stopCameraThreads();
    }

    void start() {
        QString validationError;
        if (!validateConfig(&validationError)) {
            fail(validationError);
            return;
        }
        if (!QDir().mkpath(config_.outputDirectory)) {
            fail(QStringLiteral("无法创建输出目录：%1")
                     .arg(config_.outputDirectory));
            return;
        }
        writePlanFile();
        setupCameraWorkers();
        interruptTimer_.start();
        connectionTimer_.start(config_.connectTimeoutMs);

        qInfo().noquote() << QStringLiteral(
            "自动曝光扫频开始：%1..%2 us，步长=%3 us，增益=%4 dB，共%5档")
                                 .arg(config_.startExposureUs)
                                 .arg(config_.stopExposureUs)
                                 .arg(config_.stepExposureUs)
                                 .arg(config_.gainDb, 0, 'f', 3)
                                 .arg(exposures_.size());
        qInfo().noquote() << QStringLiteral("输出目录：%1")
                                 .arg(config_.outputDirectory);

        for (int index = 0; index < kCameraCount; ++index) {
            QMetaObject::invokeMethod(
                cameras_[index].worker, "connectCamera", Qt::QueuedConnection,
                Q_ARG(QString, config_.cameraIps[index]));
        }
        laserController_->connectController();
    }

private:
    struct CameraState {
        const LineLaserDeviceProfile* profile{nullptr};
        QThread thread;
        HikCameraWorker* worker{nullptr};
        bool connected{false};
        bool identityValid{false};
        QString model;
        QString serial;
        QString ip;
        int pendingRequestId{0};
        PendingFrame pendingFrame;
        QStringList savedPaths;
    };

    bool validateConfig(QString* error) const {
        if (error) error->clear();
        if (!laserController_) {
            if (error) *error = QStringLiteral("TTL控制器为空");
            return false;
        }
        if (exposures_.empty() || exposures_.back() != config_.stopExposureUs) {
            if (error) {
                *error = QStringLiteral(
                    "曝光范围必须能由步长整除并包含终点：start=%1, stop=%2, step=%3")
                             .arg(config_.startExposureUs)
                             .arg(config_.stopExposureUs)
                             .arg(config_.stepExposureUs);
            }
            return false;
        }
        if (!std::isfinite(config_.gainDb) || config_.timeoutMs < 100 ||
            config_.connectTimeoutMs < 1000 || config_.laserSettleMs < 0 ||
            config_.interStepMs < 0 || config_.warmupFrames < 0 ||
            config_.framesPerExposure < 1) {
            if (error) *error = QStringLiteral("采集参数无效");
            return false;
        }
        if (config_.outputDirectory.trimmed().isEmpty()) {
            if (error) *error = QStringLiteral("输出目录为空");
            return false;
        }
        const QDir output(config_.outputDirectory);
        if (output.exists()) {
            const bool hasManifest =
                QFileInfo(output.absoluteFilePath(
                              QStringLiteral("captures.csv"))).exists();
            const bool hasPng = !output.entryList(
                {QStringLiteral("*.png")}, QDir::Files).isEmpty();
            if (hasManifest || hasPng) {
                if (error) {
                    *error = QStringLiteral(
                        "输出目录已含 captures.csv 或 PNG，请使用新的 session 目录：%1")
                                 .arg(output.absolutePath());
                }
                return false;
            }
        }
        if (config_.cameraIps[0].trimmed().isEmpty() ||
            config_.cameraIps[1].trimmed().isEmpty() ||
            config_.cameraIps[0] == config_.cameraIps[1]) {
            if (error) *error = QStringLiteral("两台相机 IP 必须非空且不能相同");
            return false;
        }
        return true;
    }

    void setupCameraWorkers() {
        for (int index = 0; index < kCameraCount; ++index) {
            CameraState& camera = cameras_[index];
            camera.worker = new HikCameraWorker;
            camera.worker->moveToThread(&camera.thread);
            connect(&camera.thread, &QThread::finished,
                    camera.worker, &QObject::deleteLater);
            connect(camera.worker, &HikCameraWorker::identityChanged,
                    this, [this, index](const QString& model,
                                       const QString& serial,
                                       const QString& ip) {
                CameraState& camera = cameras_[index];
                camera.model = model;
                camera.serial = serial;
                camera.ip = ip;
                camera.identityValid =
                    model == camera.profile->expectedCameraModel &&
                    serial == camera.profile->expectedCameraSerial;
                if (!camera.identityValid) {
                    fail(QStringLiteral(
                        "%1 相机身份不匹配：实际 %2/%3，期望 %4/%5")
                             .arg(camera.profile->id, model, serial,
                                  camera.profile->expectedCameraModel,
                                  camera.profile->expectedCameraSerial));
                }
            });
            connect(camera.worker, &HikCameraWorker::connectionChanged,
                    this, [this, index](bool connected,
                                       const QString& description) {
                cameras_[index].connected = connected;
                qInfo().noquote() << QStringLiteral("%1：%2")
                                         .arg(cameras_[index].profile->id,
                                              description);
                if (!connected && !shuttingDown_) {
                    fail(QStringLiteral("%1 相机连接失败或断开：%2")
                             .arg(cameras_[index].profile->id, description));
                    return;
                }
                maybeStartSweep();
            });
            connect(camera.worker, &HikCameraWorker::frameReady,
                    this, [this, index](int requestId,
                                       const QImage& image,
                                       quint64 frameNo,
                                       quint64 deviceTimestamp,
                                       qint64 hostTimestamp,
                                       double exposureUs,
                                       double gainDb,
                                       const QString&) {
                handleFrame(index, requestId, image, frameNo,
                            deviceTimestamp, hostTimestamp,
                            exposureUs, gainDb);
            });
            connect(camera.worker, &HikCameraWorker::error,
                    this, [this, index](int requestId,
                                       const QString& message) {
                if (shuttingDown_) return;
                const QString request = requestId > 0
                    ? QStringLiteral("request=%1").arg(requestId)
                    : QStringLiteral("连接/设备");
                fail(QStringLiteral("%1 相机错误（%2）：%3")
                         .arg(cameras_[index].profile->id, request, message));
            });
            connect(camera.worker, &HikCameraWorker::log,
                    this, [this, index](const QString& message) {
                qInfo().noquote() << QStringLiteral("%1：%2")
                                         .arg(cameras_[index].profile->id,
                                              message);
            });
            camera.thread.setObjectName(
                QStringLiteral("ExposureSweep_%1").arg(camera.profile->id));
            camera.thread.start();
        }
    }

    void maybeStartSweep() {
        if (shuttingDown_ || sweepStarted_ || settleScheduled_ || !laserReady_) {
            return;
        }
        for (const CameraState& camera : cameras_) {
            if (!camera.connected || !camera.identityValid) return;
        }
        settleScheduled_ = true;
        connectionTimer_.stop();
        qInfo().noquote() << QStringLiteral(
            "两台相机与双激光 TTL 均已确认，等待光源稳定 %1 ms")
                                 .arg(config_.laserSettleMs);
        QTimer::singleShot(config_.laserSettleMs, this, [this]() {
            if (shuttingDown_) return;
            if (!laserBothConfirmed(latestLaserStatus_)) {
                fail(QStringLiteral("稳定等待结束时双激光 TTL 状态无效"));
                return;
            }
            sweepStarted_ = true;
            exposureIndex_ = 0;
            beginExposure();
        });
    }

    int currentExposureUs() const {
        if (exposureIndex_ >= exposures_.size()) return 0;
        return exposures_[exposureIndex_];
    }

    void beginExposure() {
        if (shuttingDown_) return;
        if (exposureIndex_ >= exposures_.size()) {
            finishSuccessfully();
            return;
        }
        warmupRemaining_ = config_.warmupFrames;
        savedRepeatIndex_ = 0;
        qInfo().noquote() << QStringLiteral("[%1/%2] 曝光 %3 us")
                                 .arg(exposureIndex_ + 1)
                                 .arg(exposures_.size())
                                 .arg(currentExposureUs());
        capturePair(warmupRemaining_ > 0);
    }

    void capturePair(bool warmup) {
        if (shuttingDown_) return;
        if (!laserBothConfirmed(latestLaserStatus_)) {
            fail(QStringLiteral("开始曝光 %1 us 前双激光 TTL 未确认")
                     .arg(currentExposureUs()));
            return;
        }
        currentPairIsWarmup_ = warmup;
        int maximumTimeout = config_.timeoutMs;
        for (int index = 0; index < kCameraCount; ++index) {
            CameraState& camera = cameras_[index];
            camera.pendingFrame = {};
            camera.pendingRequestId = ++nextRequestId_;
            QMetaObject::invokeMethod(
                camera.worker, "captureSingle", Qt::QueuedConnection,
                Q_ARG(int, camera.pendingRequestId),
                Q_ARG(double, static_cast<double>(currentExposureUs())),
                Q_ARG(double, config_.gainDb),
                Q_ARG(int, config_.timeoutMs));
        }
        captureTimer_.start(maximumTimeout + 2000);
    }

    void handleFrame(int index,
                     int requestId,
                     const QImage& image,
                     quint64 frameNo,
                     quint64 deviceTimestamp,
                     qint64 hostTimestamp,
                     double exposureUs,
                     double gainDb) {
        if (shuttingDown_) return;
        CameraState& camera = cameras_[index];
        if (requestId != camera.pendingRequestId ||
            camera.pendingFrame.received) {
            fail(QStringLiteral("%1 收到不匹配或重复帧 request=%2，期望=%3")
                     .arg(camera.profile->id)
                     .arg(requestId)
                     .arg(camera.pendingRequestId));
            return;
        }
        PendingFrame& frame = camera.pendingFrame;
        frame.received = true;
        frame.image = image;
        frame.frameNo = frameNo;
        frame.deviceTimestamp = deviceTimestamp;
        frame.hostTimestamp = hostTimestamp;
        frame.exposureUs = exposureUs;
        frame.gainDb = gainDb;

        for (const CameraState& candidate : cameras_) {
            if (!candidate.pendingFrame.received) return;
        }
        captureTimer_.stop();
        processCompletedPair();
    }

    void processCompletedPair() {
        const int exposure = currentExposureUs();
        for (const CameraState& camera : cameras_) {
            const PendingFrame& frame = camera.pendingFrame;
            if (frame.image.isNull() ||
                std::abs(frame.exposureUs - exposure) > 1.0 ||
                std::abs(frame.gainDb - config_.gainDb) > 0.01) {
                fail(QStringLiteral(
                    "%1 返回参数不一致：期望 exposure=%2/gain=%3，实际=%4/%5")
                         .arg(camera.profile->id)
                         .arg(exposure)
                         .arg(config_.gainDb, 0, 'f', 3)
                         .arg(frame.exposureUs, 0, 'f', 3)
                         .arg(frame.gainDb, 0, 'f', 3));
                return;
            }
        }

        if (currentPairIsWarmup_) {
            --warmupRemaining_;
            QTimer::singleShot(20, this, [this]() {
                capturePair(warmupRemaining_ > 0);
            });
            return;
        }

        for (int index = 0; index < kCameraCount; ++index) {
            CameraState& camera = cameras_[index];
            const PendingFrame& frame = camera.pendingFrame;
            hik_exposure_test::CaptureRecord record;
            record.capturedAt = captureDateTime(frame.hostTimestamp);
            record.profileId = camera.profile->id;
            record.wavelengthNm = camera.profile->wavelengthNm;
            record.cameraIp = camera.ip;
            record.cameraModel = camera.model;
            record.cameraSerial = camera.serial;
            record.width = frame.image.width();
            record.height = frame.image.height();
            record.pixelFormat = QStringLiteral("Mono8");
            record.exposureUs = frame.exposureUs;
            record.gainDb = frame.gainDb;
            record.frameNumber = frame.frameNo;
            record.deviceTimestamp = frame.deviceTimestamp;
            record.hostTimestamp = frame.hostTimestamp;
            record.laserState = laserStateName(latestLaserStatus_.state);
            record.laserStatusReachable = latestLaserStatus_.reachable;
            record.laserStatusSourceMonotonicNs =
                latestLaserStatus_.sourceMonotonicNs;
            record.ttl450High = latestLaserStatus_.ttl450High;
            record.ttl650High = latestLaserStatus_.ttl650High;

            QString savedPath;
            QString saveError;
            if (!hik_exposure_test::saveCapture(
                    frame.image, record, config_.outputDirectory,
                    &savedPath, &saveError)) {
                fail(QStringLiteral("%1 保存曝光 %2 us 失败：%3")
                         .arg(camera.profile->id)
                         .arg(exposure)
                         .arg(saveError));
                return;
            }
            const QFileInfo savedInfo(savedPath);
            if (!savedInfo.isFile() || savedInfo.size() <= 0) {
                fail(QStringLiteral("%1 保存后校验失败：%2")
                         .arg(camera.profile->id, savedPath));
                return;
            }
            camera.savedPaths.push_back(savedPath);
        }

        ++savedRepeatIndex_;
        if (savedRepeatIndex_ < config_.framesPerExposure) {
            QTimer::singleShot(20, this, [this]() { capturePair(false); });
            return;
        }

        completedExposures_.push_back(exposure);
        ++exposureIndex_;
        QTimer::singleShot(config_.interStepMs, this,
                           [this]() { beginExposure(); });
    }

    bool verifyOutput(QString* error) const {
        if (error) error->clear();
        const int expectedPerCamera =
            static_cast<int>(exposures_.size()) * config_.framesPerExposure;
        for (const CameraState& camera : cameras_) {
            if (camera.savedPaths.size() != expectedPerCamera) {
                if (error) {
                    *error = QStringLiteral("%1 图片数=%2，期望=%3")
                                 .arg(camera.profile->id)
                                 .arg(camera.savedPaths.size())
                                 .arg(expectedPerCamera);
                }
                return false;
            }
            for (const QString& path : camera.savedPaths) {
                const QFileInfo info(path);
                if (!info.isFile() || info.size() <= 0) {
                    if (error) *error = QStringLiteral("图片缺失或为空：%1").arg(path);
                    return false;
                }
            }
        }

        QFile manifest(QDir(config_.outputDirectory)
                           .absoluteFilePath(QStringLiteral("captures.csv")));
        if (!manifest.open(QIODevice::ReadOnly | QIODevice::Text)) {
            if (error) *error = QStringLiteral("无法读取 captures.csv");
            return false;
        }
        int nonEmptyLines = 0;
        while (!manifest.atEnd()) {
            if (!manifest.readLine().trimmed().isEmpty()) ++nonEmptyLines;
        }
        const int expectedRows = kCameraCount * expectedPerCamera;
        if (nonEmptyLines != expectedRows + 1) {
            if (error) {
                *error = QStringLiteral("captures.csv 数据行=%1，期望=%2")
                             .arg(std::max(0, nonEmptyLines - 1))
                             .arg(expectedRows);
            }
            return false;
        }
        return true;
    }

    void finishSuccessfully() {
        QString verifyError;
        if (!verifyOutput(&verifyError)) {
            fail(QStringLiteral("完整性校验失败：%1").arg(verifyError));
            return;
        }
        beginShutdown(0, QStringLiteral("曝光扫频与完整性校验完成"));
    }

    void fail(const QString& message) {
        if (shuttingDown_) return;
        beginShutdown(1, message);
    }

    void beginShutdown(int exitCode, const QString& message) {
        if (shuttingDown_) return;
        shuttingDown_ = true;
        exitCode_ = exitCode;
        finalMessage_ = message;
        connectionTimer_.stop();
        captureTimer_.stop();
        interruptTimer_.stop();
        if (exitCode == 0) {
            qInfo().noquote() << message;
        } else {
            qCritical().noquote() << message;
        }
        writeSummaryFile();
        offCommandToken_ = laserController_->requestOffTracked();
        shutdownTimer_.start(4000);
    }

    void cleanupAndExit(bool laserOffConfirmedSuccessfully) {
        if (cleanupDone_) return;
        cleanupDone_ = true;
        shutdownTimer_.stop();
        laserOffWasConfirmed_ = laserOffConfirmedSuccessfully;
        stopCameraThreads();
        writeSummaryFile();
        if (exitCode_ == 0 && laserOffWasConfirmed_) {
            qInfo().noquote() << QStringLiteral(
                "完成：每台相机 %1 张，双激光已确认关闭。目录=%2")
                                     .arg(cameras_[0].savedPaths.size())
                                     .arg(config_.outputDirectory);
        }
        QCoreApplication::exit(exitCode_ == 0 && laserOffWasConfirmed_ ? 0 : 1);
    }

    void stopCameraThreads() {
        if (cameraThreadsStopped_) return;
        cameraThreadsStopped_ = true;
        for (CameraState& camera : cameras_) {
            if (camera.worker && camera.thread.isRunning()) {
                QMetaObject::invokeMethod(camera.worker, "disconnectCamera",
                                          Qt::BlockingQueuedConnection);
                camera.thread.quit();
                camera.thread.wait();
            }
            camera.worker = nullptr;
        }
    }

    QJsonObject baseJson() const {
        QJsonObject root;
        root.insert(QStringLiteral("start_exposure_us"),
                    config_.startExposureUs);
        root.insert(QStringLiteral("stop_exposure_us"),
                    config_.stopExposureUs);
        root.insert(QStringLiteral("step_exposure_us"),
                    config_.stepExposureUs);
        root.insert(QStringLiteral("gain_db"), config_.gainDb);
        root.insert(QStringLiteral("warmup_frames_per_exposure"),
                    config_.warmupFrames);
        root.insert(QStringLiteral("saved_frames_per_exposure"),
                    config_.framesPerExposure);
        root.insert(QStringLiteral("laser_state"), QStringLiteral("both"));
        QJsonArray exposures;
        for (int value : exposures_) exposures.append(value);
        root.insert(QStringLiteral("planned_exposures_us"), exposures);
        QJsonArray cameras;
        for (int index = 0; index < kCameraCount; ++index) {
            QJsonObject camera;
            camera.insert(QStringLiteral("profile_id"),
                          cameras_[index].profile->id);
            camera.insert(QStringLiteral("ip"), config_.cameraIps[index]);
            camera.insert(QStringLiteral("expected_model"),
                          cameras_[index].profile->expectedCameraModel);
            camera.insert(QStringLiteral("expected_serial"),
                          cameras_[index].profile->expectedCameraSerial);
            cameras.append(camera);
        }
        root.insert(QStringLiteral("cameras"), cameras);
        return root;
    }

    void writePlanFile() const {
        QJsonObject root = baseJson();
        root.insert(QStringLiteral("created_at"),
                    QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
        QString error;
        const QString path = QDir(config_.outputDirectory)
                                 .absoluteFilePath(QStringLiteral("sweep_plan.json"));
        if (!writeJsonFile(path, root, &error)) {
            qWarning().noquote() << QStringLiteral("写入 sweep_plan.json 失败：%1")
                                        .arg(error);
        }
    }

    void writeSummaryFile() const {
        if (config_.outputDirectory.trimmed().isEmpty() ||
            !QDir(config_.outputDirectory).exists()) {
            return;
        }
        QJsonObject root = baseJson();
        root.insert(QStringLiteral("updated_at"),
                    QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
        root.insert(QStringLiteral("success"), exitCode_ == 0);
        root.insert(QStringLiteral("message"), finalMessage_);
        root.insert(QStringLiteral("laser_off_confirmed"),
                    laserOffWasConfirmed_);
        QJsonArray completed;
        for (int value : completedExposures_) completed.append(value);
        root.insert(QStringLiteral("completed_exposures_us"), completed);
        QJsonObject savedCounts;
        for (const CameraState& camera : cameras_) {
            savedCounts.insert(camera.profile->id, camera.savedPaths.size());
        }
        root.insert(QStringLiteral("saved_image_counts"), savedCounts);
        QString error;
        const QString path = QDir(config_.outputDirectory)
                                 .absoluteFilePath(QStringLiteral("sweep_summary.json"));
        if (!writeJsonFile(path, root, &error)) {
            qWarning().noquote() << QStringLiteral(
                "写入 sweep_summary.json 失败：%1").arg(error);
        }
    }

    SweepConfig config_;
    LineLaserController* laserController_{nullptr};
    std::vector<int> exposures_;
    std::array<CameraState, kCameraCount> cameras_;
    LineLaserStatus latestLaserStatus_;
    QTimer connectionTimer_;
    QTimer captureTimer_;
    QTimer shutdownTimer_;
    QTimer interruptTimer_;
    std::size_t exposureIndex_{0};
    std::vector<int> completedExposures_;
    int nextRequestId_{0};
    int warmupRemaining_{0};
    int savedRepeatIndex_{0};
    bool currentPairIsWarmup_{false};
    bool laserCommandRequested_{false};
    bool laserReady_{false};
    bool settleScheduled_{false};
    bool sweepStarted_{false};
    bool shuttingDown_{false};
    bool cleanupDone_{false};
    bool cameraThreadsStopped_{false};
    bool laserOffWasConfirmed_{false};
    int exitCode_{1};
    QString finalMessage_{QStringLiteral("尚未完成")};
    quint64 offCommandToken_{0};
};

bool parseIntOption(const QCommandLineParser& parser,
                    const QCommandLineOption& option,
                    int minimum,
                    int maximum,
                    int* output,
                    QString* error) {
    bool ok = false;
    const int value = parser.value(option).toInt(&ok);
    if (!ok || value < minimum || value > maximum) {
        if (error) {
            *error = QStringLiteral("参数 --%1 必须在 [%2, %3] 内")
                         .arg(option.names().front())
                         .arg(minimum)
                         .arg(maximum);
        }
        return false;
    }
    *output = value;
    return true;
}

QString defaultOutputDirectory() {
#if defined(HIK_CALIBRATION_SOURCE_DIR)
    const QDir source(QStringLiteral(HIK_CALIBRATION_SOURCE_DIR));
#else
    const QDir source(QDir::currentPath());
#endif
    return source.absoluteFilePath(
        QStringLiteral("data/exposure_tests/session_%1_auto")
            .arg(QDateTime::currentDateTime().toString(
                QStringLiteral("yyyyMMdd_HHmmss_zzz"))));
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("HikExposureSweep"));
    QCoreApplication::setOrganizationName(QStringLiteral("Vizum"));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "双相机、双激光自动曝光扫频采集（默认 1000..5000 us / 100 us / 0 dB）"));
    parser.addHelpOption();

    QCommandLineOption startOption(QStringLiteral("start-us"),
        QStringLiteral("起始曝光，单位 us"), QStringLiteral("us"),
        QStringLiteral("1000"));
    QCommandLineOption stopOption(QStringLiteral("stop-us"),
        QStringLiteral("终止曝光，单位 us（包含）"), QStringLiteral("us"),
        QStringLiteral("5000"));
    QCommandLineOption stepOption(QStringLiteral("step-us"),
        QStringLiteral("曝光步长，单位 us"), QStringLiteral("us"),
        QStringLiteral("100"));
    QCommandLineOption gainOption(QStringLiteral("gain-db"),
        QStringLiteral("固定模拟增益，单位 dB"), QStringLiteral("dB"),
        QStringLiteral("0"));
    QCommandLineOption timeoutOption(QStringLiteral("timeout-ms"),
        QStringLiteral("单帧相机超时，单位 ms"), QStringLiteral("ms"),
        QStringLiteral("3000"));
    QCommandLineOption laserSettleOption(QStringLiteral("laser-settle-ms"),
        QStringLiteral("双激光确认开启后的稳定等待，单位 ms"),
        QStringLiteral("ms"), QStringLiteral("1000"));
    QCommandLineOption interStepOption(QStringLiteral("inter-step-ms"),
        QStringLiteral("相邻曝光档之间的等待，单位 ms"),
        QStringLiteral("ms"), QStringLiteral("100"));
    QCommandLineOption warmupOption(QStringLiteral("warmup-frames"),
        QStringLiteral("每个曝光档先采集并丢弃的稳定帧数"),
        QStringLiteral("count"), QStringLiteral("1"));
    QCommandLineOption framesOption(QStringLiteral("frames-per-exposure"),
        QStringLiteral("每台相机每个曝光档保存的帧数"),
        QStringLiteral("count"), QStringLiteral("1"));
    QCommandLineOption outputOption(
        {QStringLiteral("o"), QStringLiteral("output")},
        QStringLiteral("输出 session 目录"), QStringLiteral("directory"));
    QCommandLineOption camera450Option(QStringLiteral("camera-450-ip"),
        QStringLiteral("scanner_450 相机 IP"), QStringLiteral("ip"));
    QCommandLineOption camera650Option(QStringLiteral("camera-650-ip"),
        QStringLiteral("scanner_650 相机 IP"), QStringLiteral("ip"));
    QCommandLineOption dryRunOption(QStringLiteral("dry-run"),
        QStringLiteral("只校验并打印计划，不连接硬件"));
    parser.addOptions({startOption, stopOption, stepOption, gainOption,
                       timeoutOption, laserSettleOption, interStepOption,
                       warmupOption, framesOption, outputOption,
                       camera450Option, camera650Option, dryRunOption});
    parser.process(application);

    SweepConfig config;
    QString parseError;
    if (!parseIntOption(parser, startOption, 1, 1000000,
                        &config.startExposureUs, &parseError) ||
        !parseIntOption(parser, stopOption, 1, 1000000,
                        &config.stopExposureUs, &parseError) ||
        !parseIntOption(parser, stepOption, 1, 1000000,
                        &config.stepExposureUs, &parseError) ||
        !parseIntOption(parser, timeoutOption, 100, 10000,
                        &config.timeoutMs, &parseError) ||
        !parseIntOption(parser, laserSettleOption, 0, 60000,
                        &config.laserSettleMs, &parseError) ||
        !parseIntOption(parser, interStepOption, 0, 60000,
                        &config.interStepMs, &parseError) ||
        !parseIntOption(parser, warmupOption, 0, 100,
                        &config.warmupFrames, &parseError) ||
        !parseIntOption(parser, framesOption, 1, 1000,
                        &config.framesPerExposure, &parseError)) {
        qCritical().noquote() << parseError;
        return 2;
    }
    bool gainOk = false;
    config.gainDb = parser.value(gainOption).toDouble(&gainOk);
    if (!gainOk || !std::isfinite(config.gainDb) ||
        config.gainDb < 0.0 || config.gainDb > 48.0) {
        qCritical().noquote() << QStringLiteral(
            "参数 --gain-db 必须在 [0, 48] 内");
        return 2;
    }

    const std::vector<LineLaserDeviceProfile>& profiles =
        lineLaserDeviceProfiles();
    if (profiles.size() < kCameraCount) {
        qCritical("需要两个线激光设备配置");
        return 2;
    }
    config.cameraIps[0] = parser.isSet(camera450Option)
        ? parser.value(camera450Option)
        : profiles[0].defaultCameraIp;
    config.cameraIps[1] = parser.isSet(camera650Option)
        ? parser.value(camera650Option)
        : profiles[1].defaultCameraIp;
    config.outputDirectory = parser.isSet(outputOption)
        ? QDir::cleanPath(parser.value(outputOption))
        : defaultOutputDirectory();

    const std::vector<int> exposures = config.exposures();
    if (exposures.empty() || exposures.back() != config.stopExposureUs) {
        qCritical().noquote() << QStringLiteral(
            "曝光范围必须能由步长整除并包含终点");
        return 2;
    }
    if (parser.isSet(dryRunOption)) {
        qInfo().noquote() << QStringLiteral(
            "DRY RUN：%1..%2 us，步长=%3 us，共%4档；每台保存%5张；增益=%6 dB")
                                 .arg(config.startExposureUs)
                                 .arg(config.stopExposureUs)
                                 .arg(config.stepExposureUs)
                                 .arg(exposures.size())
                                 .arg(exposures.size() *
                                      static_cast<std::size_t>(config.framesPerExposure))
                                 .arg(config.gainDb, 0, 'f', 3);
        qInfo().noquote() << QStringLiteral("scanner_450=%1, scanner_650=%2")
                                 .arg(config.cameraIps[0], config.cameraIps[1]);
        qInfo().noquote() << QStringLiteral("输出目录=%1")
                                 .arg(config.outputDirectory);
        return 0;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    LineLaserController laserController;
    ExposureSweepRunner runner(config, &laserController);
    QTimer::singleShot(0, &runner, [&runner]() { runner.start(); });
    return application.exec();
}
