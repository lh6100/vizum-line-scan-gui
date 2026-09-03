#ifndef MYLINE_HIK_EXPOSURE_TEST_STORAGE_H
#define MYLINE_HIK_EXPOSURE_TEST_STORAGE_H

#include <QDateTime>
#include <QImage>
#include <QString>

namespace hik_exposure_test {

struct CaptureRecord {
    QDateTime capturedAt;
    QString profileId;
    int wavelengthNm{0};
    QString cameraIp;
    QString cameraModel;
    QString cameraSerial;
    int width{0};
    int height{0};
    QString pixelFormat{QStringLiteral("Mono8")};
    double exposureUs{0.0};
    double gainDb{0.0};
    quint64 frameNumber{0};
    quint64 deviceTimestamp{0};
    qint64 hostTimestamp{0};
    QString laserState{QStringLiteral("unknown")};
    bool laserStatusReachable{false};
    qint64 laserStatusSourceMonotonicNs{0};
    bool ttl450High{false};
    bool ttl650High{false};
};

QString makeCaptureFileName(const CaptureRecord& record);

// Saves the PNG atomically and appends one UTF-8 row to captures.csv.
// The output record's width/height should describe the supplied image.
bool saveCapture(const QImage& image,
                 const CaptureRecord& record,
                 const QString& outputDirectory,
                 QString* savedImagePath,
                 QString* error);

} // namespace hik_exposure_test

#endif // MYLINE_HIK_EXPOSURE_TEST_STORAGE_H
