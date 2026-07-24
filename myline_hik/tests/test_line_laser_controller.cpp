#include "LineLaserController.h"

#include <QCoreApplication>
#include <QFile>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

qint64 jsonId(const std::string& request) {
    const std::string marker = "\"id\":";
    const std::size_t start = request.find(marker);
    if (start == std::string::npos) return 0;
    const char* first = request.c_str() + start + marker.size();
    char* end = nullptr;
    const long long value = std::strtoll(first, &end, 10);
    return end == first ? 0 : static_cast<qint64>(value);
}

std::string operation(const std::string& request) {
    const std::string marker = "\"op\":\"";
    const std::size_t start = request.find(marker);
    if (start == std::string::npos) return std::string();
    const std::size_t valueStart = start + marker.size();
    const std::size_t end = request.find('"', valueStart);
    return end == std::string::npos
        ? std::string() : request.substr(valueStart, end - valueStart);
}

int runFakeSsh() {
    std::string state = "off";
    std::string request;
    while (std::getline(std::cin, request)) {
        const qint64 id = jsonId(request);
        const std::string op = operation(request);
        if (op == "hello") {
            state = "off";
        } else if (op == "set") {
            if (request.find("\"state\":\"laser450\"") != std::string::npos) {
                state = "laser450";
            } else if (
                    request.find("\"state\":\"laser650\"") != std::string::npos) {
                state = "laser650";
            } else {
                std::cout
                    << "{\"v\":1,\"id\":" << id
                    << ",\"ok\":false,\"error\":{\"code\":\"INVALID_STATE\","
                       "\"message\":\"bad state\"},\"status\":{"
                       "\"generation\":\"fake\",\"board_model\":\"LubanCat-4 V1\","
                       "\"state\":\"off\",\"ttl450_high\":false,"
                       "\"ttl650_high\":false,\"lease_active\":true,"
                       "\"lease_remaining_ms\":2000,\"fault\":\"\","
                       "\"pin_map\":{"
                       "\"laser450\":{\"physical_pin\":11,\"gpio\":15,"
                       "\"chip_offset\":15},"
                       "\"laser650\":{\"physical_pin\":7,\"gpio\":16,"
                       "\"chip_offset\":16}}}}\n"
                    << std::flush;
                continue;
            }
        } else if (op == "off" || op == "goodbye") {
            state = "off";
        }

        const bool high450 = state == "laser450";
        const bool high650 = state == "laser650";
        std::cout
            << "{\"v\":1,\"id\":" << id
            << ",\"ok\":true,\"status\":{"
               "\"generation\":\"fake-generation\","
               "\"board_model\":\"LubanCat-4 V1 test\","
               "\"state\":\"" << state << "\","
               "\"ttl450_high\":" << (high450 ? "true" : "false") << ","
               "\"ttl650_high\":" << (high650 ? "true" : "false") << ","
               "\"lease_active\":" << (op == "goodbye" ? "false" : "true") << ","
               "\"lease_remaining_ms\":2000,\"fault\":\"\","
               "\"pin_map\":{"
               "\"laser450\":{\"physical_pin\":11,\"gpio\":15,"
               "\"chip_offset\":15},"
               "\"laser650\":{\"physical_pin\":7,\"gpio\":16,"
               "\"chip_offset\":16}}}}\n"
            << std::flush;
        if (op == "goodbye") return 0;
    }
    return 0;
}

bool writeEmptyFile(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    return file.write("test\n") == 5;
}

}  // namespace

class LineLaserControllerTest final : public QObject {
    Q_OBJECT

private slots:
    void defaultsUseDedicatedRestrictedIdentity() {
        const LineLaserConnectionConfig config =
            LineLaserConnectionConfig::defaults();
        QCOMPARE(config.host, QStringLiteral("192.168.1.12"));
        QCOMPARE(config.user, QStringLiteral("laserctl"));
        QCOMPARE(config.port, quint16(22));
        QVERIFY(config.privateKeyPath.endsWith(
            QStringLiteral("laser-control/id_ed25519")));
        QVERIFY(config.knownHostsPath.endsWith(
            QStringLiteral("laser-control/known_hosts")));
        QVERIFY(config.heartbeatIntervalMs < 1500);
    }

    void asynchronousStateAndMutualExclusion() {
        QTemporaryDir temporary;
        QVERIFY(temporary.isValid());
        const QString key = temporary.filePath(QStringLiteral("id_ed25519"));
        const QString knownHosts =
            temporary.filePath(QStringLiteral("known_hosts"));
        QVERIFY(writeEmptyFile(key));
        QVERIFY(writeEmptyFile(knownHosts));

        LineLaserConnectionConfig config =
            LineLaserConnectionConfig::defaults();
        config.sshProgram = QCoreApplication::applicationFilePath();
        config.privateKeyPath = key;
        config.knownHostsPath = knownHosts;
        config.connectTimeoutMs = 1000;
        config.commandTimeoutMs = 500;
        config.heartbeatIntervalMs = 200;
        config.autoReconnect = false;

        LineLaserController controller(config);
        QSignalSpy connectionSpy(
            &controller, &LineLaserController::connectionStateChanged);
        QSignalSpy statusSpy(
            &controller, &LineLaserController::statusChanged);
        QSignalSpy commandSpy(
            &controller, &LineLaserController::commandFinished);

        controller.connectController();
        QTRY_VERIFY_WITH_TIMEOUT(connectionSpy.count() > 0, 2000);
        QTRY_VERIFY_WITH_TIMEOUT(
            !statusSpy.isEmpty() &&
            statusSpy.last().at(0).value<LineLaserStatus>().reachable &&
            statusSpy.last().at(0).value<LineLaserStatus>().state ==
                LineLaserState::Off,
            2000);

        controller.set450();
        QTRY_VERIFY_WITH_TIMEOUT(
            !statusSpy.isEmpty() &&
            statusSpy.last().at(0).value<LineLaserStatus>().state ==
                LineLaserState::Laser450,
            1000);
        LineLaserStatus status =
            statusSpy.last().at(0).value<LineLaserStatus>();
        QVERIFY(status.sourceMonotonicNs > 0);
        QVERIFY(status.eventSequence > 0);
        QVERIFY(status.transportGeneration > 0);
        const quint64 firstStatusSequence = status.eventSequence;
        const quint64 transportGeneration =
            status.transportGeneration;
        QVERIFY(status.ttl450High);
        QVERIFY(!status.ttl650High);
        QCOMPARE(status.laser450PhysicalPin, 11);
        QCOMPARE(status.laser450Gpio, 15);
        QCOMPARE(status.laser650PhysicalPin, 7);
        QCOMPARE(status.laser650Gpio, 16);

        controller.set650();
        QTRY_VERIFY_WITH_TIMEOUT(
            statusSpy.last().at(0).value<LineLaserStatus>().state ==
                LineLaserState::Laser650,
            1000);
        status = statusSpy.last().at(0).value<LineLaserStatus>();
        QVERIFY(status.eventSequence > firstStatusSequence);
        QCOMPARE(status.transportGeneration, transportGeneration);
        QVERIFY(!status.ttl450High);
        QVERIFY(status.ttl650High);

        const quint64 offToken = controller.requestOffTracked();
        QVERIFY(offToken > 0);
        QTRY_VERIFY_WITH_TIMEOUT(
            statusSpy.last().at(0).value<LineLaserStatus>().state ==
                LineLaserState::Off &&
            statusSpy.last().at(0).value<LineLaserStatus>()
                    .acknowledgedOffCommandToken >= offToken,
            1000);
        status = statusSpy.last().at(0).value<LineLaserStatus>();
        QVERIFY(!status.ttl450High);
        QVERIFY(!status.ttl650High);
        QVERIFY(commandSpy.count() >= 3);

        controller.disconnectController();
        QTRY_VERIFY_WITH_TIMEOUT(
            !connectionSpy.isEmpty() &&
            connectionSpy.last().at(0).value<LineLaserConnectionState>() ==
                LineLaserConnectionState::Disconnected,
            2000);
    }

    void missingIdentityFailsWithoutPasswordPrompt() {
        LineLaserConnectionConfig config =
            LineLaserConnectionConfig::defaults();
        config.privateKeyPath =
            QStringLiteral("/definitely/missing/laser-key");
        config.knownHostsPath =
            QStringLiteral("/definitely/missing/known-hosts");
        config.autoReconnect = false;

        LineLaserController controller(config);
        QSignalSpy faultSpy(
            &controller, &LineLaserController::faultOccurred);
        controller.connectController();
        QTRY_VERIFY_WITH_TIMEOUT(!faultSpy.isEmpty(), 1000);
        QVERIFY(faultSpy.last().at(0).toString().contains(
            QStringLiteral("私钥")));
    }
};

int main(int argc, char** argv) {
    for (int index = 1; index < argc; ++index) {
        if (QString::fromLocal8Bit(argv[index]) == QStringLiteral("-T")) {
            return runFakeSsh();
        }
    }
    QCoreApplication application(argc, argv);
    LineLaserControllerTest test;
    return QTest::qExec(&test, argc, argv);
}

#include "test_line_laser_controller.moc"
