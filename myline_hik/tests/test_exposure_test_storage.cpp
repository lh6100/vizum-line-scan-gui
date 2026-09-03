#include "exposure_test/ExposureTestStorage.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include <iostream>

namespace {

bool require(bool condition, const char* message) {
    if (!condition) std::cerr << message << std::endl;
    return condition;
}

} // namespace

int main(int argc, char* argv[]) {
    QCoreApplication application(argc, argv);
    hik_exposure_test::CaptureRecord record;
    record.capturedAt = QDateTime::fromString(
        QStringLiteral("2026-08-26T16:23:45.123"), Qt::ISODateWithMs);
    record.profileId = QStringLiteral("scanner_450");
    record.wavelengthNm = 450;
    record.cameraIp = QStringLiteral("192.168.1.46");
    record.cameraModel = QStringLiteral("MV-CS013-60GN");
    record.cameraSerial = QStringLiteral("DB0403208");
    record.width = 1224;
    record.height = 1024;
    record.exposureUs = 1825.25;
    record.gainDb = 1.5;
    record.frameNumber = 42;
    record.laserState = QStringLiteral("laser450");
    record.laserStatusReachable = true;
    record.ttl450High = true;

    const QString fileName = hik_exposure_test::makeCaptureFileName(record);
    if (!require(fileName.contains(QStringLiteral("1224x1024_Mono8")),
                 "file name does not contain image geometry/pixel format") ||
        !require(fileName.contains(QStringLiteral("exp1825.250us_gain1.500dB")),
                 "file name does not contain exposure/gain") ||
        !require(fileName.endsWith(QStringLiteral("20260826_162345_123.png")),
                 "file name does not contain millisecond timestamp")) {
        return 1;
    }

    QTemporaryDir directory;
    if (!require(directory.isValid(), "temporary directory unavailable")) return 1;
    QImage image(12, 8, QImage::Format_Grayscale8);
    image.fill(127);
    QString path;
    QString error;
    if (!require(hik_exposure_test::saveCapture(
                     image, record, directory.path(), &path, &error),
                 error.toLocal8Bit().constData())) {
        return 1;
    }
    if (!require(QFileInfo::exists(path), "PNG was not saved")) return 1;

    QFile manifest(directory.filePath(QStringLiteral("captures.csv")));
    if (!require(manifest.open(QIODevice::ReadOnly | QIODevice::Text),
                 "captures.csv was not saved")) {
        return 1;
    }
    const QByteArray csv = manifest.readAll();
    if (!require(csv.contains("exposure_us,gain_db"),
                 "manifest header is incomplete") ||
        !require(csv.contains("1825.250,1.500"),
                 "manifest does not contain actual parameters") ||
        !require(csv.contains("laser450"),
                 "manifest does not contain laser state")) {
        return 1;
    }
    return 0;
}
