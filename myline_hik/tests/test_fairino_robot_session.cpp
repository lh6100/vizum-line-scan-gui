#include "FairinoRobotSession.h"

#include <QCoreApplication>
#include <QtTest>

class FairinoRobotSessionTest final : public QObject {
    Q_OBJECT

private slots:
    void requestIdsAreUniqueAndClientScoped() {
        FairinoRobotSession session;
        const int scanner450 =
            session.registerClient(QStringLiteral("scanner_450"));
        const int scanner650 =
            session.registerClient(QStringLiteral("scanner_650"));

        QVERIFY(scanner450 > 0);
        QVERIFY(scanner650 > 0);
        QVERIFY(scanner450 != scanner650);

        const int request450First =
            session.allocateRequestId(scanner450);
        const int request450Second =
            session.allocateRequestId(scanner450);
        const int request650 =
            session.allocateRequestId(scanner650);

        QVERIFY(request450First > 0);
        QVERIFY(request450Second > 0);
        QVERIFY(request650 > 0);
        QVERIFY(request450First != request450Second);
        QVERIFY(request450First != request650);
        QVERIFY(request450Second != request650);

        QVERIFY(session.requestBelongsToClient(
            request450First, scanner450));
        QVERIFY(session.requestBelongsToClient(
            request450Second, scanner450));
        QVERIFY(session.requestBelongsToClient(
            request650, scanner650));
        QVERIFY(!session.requestBelongsToClient(
            request450First, scanner650));
        QVERIFY(!session.requestBelongsToClient(
            request650, scanner450));
    }

    void exclusiveLeaseRejectsAnotherClient() {
        FairinoRobotSession session;
        const int scanner450 =
            session.registerClient(QStringLiteral("scanner_450"));
        const int scanner650 =
            session.registerClient(QStringLiteral("scanner_650"));

        QString error;
        QVERIFY(session.acquireExclusive(
            scanner450, QStringLiteral("continuous_scan"), &error));
        QVERIFY(error.isEmpty());
        QVERIFY(session.isExclusiveOwner(scanner450));
        const quint64 firstEpoch =
            session.exclusiveLeaseEpochFor(scanner450);
        QVERIFY(firstEpoch > 0);
        QCOMPARE(session.exclusiveLeaseEpochFor(scanner650),
                 quint64(0));
        QVERIFY(!session.isExclusiveOwner(scanner650));
        QVERIFY(session.commandAvailableTo(scanner450));
        QVERIFY(!session.commandAvailableTo(scanner650));
        QVERIFY(session.exclusiveOwnerDescription().contains(
            QStringLiteral("scanner_450")));
        QVERIFY(session.exclusiveOwnerDescription().contains(
            QStringLiteral("continuous_scan")));

        QSignalSpy requestErrorSpy(
            &session, &FairinoRobotSession::error);
        const int blockedRequest =
            session.allocateRequestId(scanner650);
        session.readFlangePose(blockedRequest);
        QCOMPARE(requestErrorSpy.count(), 1);
        QCOMPARE(requestErrorSpy.takeFirst().at(0).toInt(),
                 blockedRequest);

        QSignalSpy clientErrorSpy(
            &session, &FairinoRobotSession::clientError);
        session.disconnectRobot(scanner450);
        QCOMPARE(clientErrorSpy.count(), 1);
        QCOMPARE(clientErrorSpy.takeFirst().at(0).toInt(),
                 scanner450);

        QVERIFY(!session.acquireExclusive(
            scanner650, QStringLiteral("stop_and_shoot"), &error));
        QVERIFY(!error.isEmpty());
        QVERIFY(error.contains(QStringLiteral("scanner_450")));
        QVERIFY(session.isExclusiveOwner(scanner450));
        QVERIFY(!session.isExclusiveOwner(scanner650));

        session.releaseExclusive(scanner650);
        QVERIFY(session.isExclusiveOwner(scanner450));
    }

    void releasedLeaseCanBeTakenOver() {
        FairinoRobotSession session;
        const int scanner450 =
            session.registerClient(QStringLiteral("scanner_450"));
        const int scanner650 =
            session.registerClient(QStringLiteral("scanner_650"));

        QVERIFY(session.acquireExclusive(
            scanner450, QStringLiteral("first_scan")));
        session.releaseExclusive(scanner450);

        QVERIFY(!session.isExclusiveOwner(scanner450));
        QVERIFY(session.commandAvailableTo(scanner450));
        QVERIFY(session.commandAvailableTo(scanner650));
        QVERIFY(session.exclusiveOwnerDescription().isEmpty());

        QSignalSpy motionErrorSpy(
            &session, &FairinoRobotSession::error);
        const int motionWithoutLease =
            session.allocateRequestId(scanner650);
        session.moveLinear(
            motionWithoutLease,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 1.0, 1000);
        QCOMPARE(motionErrorSpy.count(), 1);
        QCOMPARE(motionErrorSpy.takeFirst().at(0).toInt(),
                 motionWithoutLease);

        QString error;
        QVERIFY(session.acquireExclusive(
            scanner650, QStringLiteral("second_scan"), &error));
        QVERIFY(error.isEmpty());
        QVERIFY(session.isExclusiveOwner(scanner650));
        QVERIFY(session.exclusiveLeaseEpochFor(scanner650) > 0);
        QVERIFY(!session.commandAvailableTo(scanner450));
        QVERIFY(session.commandAvailableTo(scanner650));
    }

    void invalidClientIsRejected() {
        FairinoRobotSession session;
        const int validClient =
            session.registerClient(QStringLiteral("valid_client"));

        QCOMPARE(session.allocateRequestId(0), -1);
        QCOMPARE(session.allocateRequestId(-1), -1);
        QCOMPARE(session.allocateRequestId(validClient + 1000), -1);
        QVERIFY(!session.requestBelongsToClient(1, 0));
        QVERIFY(!session.requestBelongsToClient(1, validClient + 1000));
        QVERIFY(!session.isExclusiveOwner(0));
        QVERIFY(!session.commandAvailableTo(0));
        QVERIFY(!session.commandAvailableTo(validClient + 1000));

        QString error;
        QVERIFY(!session.acquireExclusive(
            0, QStringLiteral("invalid"), &error));
        QVERIFY(!error.isEmpty());
        QVERIFY(!session.isExclusiveOwner(validClient));

        session.releaseExclusive(0);
        QVERIFY(session.exclusiveOwnerDescription().isEmpty());
    }
};

int main(int argc, char** argv) {
    QCoreApplication application(argc, argv);
    FairinoRobotSessionTest test;
    return QTest::qExec(&test, argc, argv);
}

#include "test_fairino_robot_session.moc"
