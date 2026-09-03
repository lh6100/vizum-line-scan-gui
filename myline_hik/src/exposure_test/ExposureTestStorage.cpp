#include "exposure_test/ExposureTestStorage.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSaveFile>
#include <QTextStream>

#include <cmath>

namespace hik_exposure_test {
namespace {

QString safeToken(QString value, const QString& fallback) {
    value = value.trimmed();
    value.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9._-]+")),
                  QStringLiteral("_"));
    while (value.startsWith(QLatin1Char('.'))) value.remove(0, 1);
    while (value.endsWith(QLatin1Char('.'))) value.chop(1);
    return value.isEmpty() ? fallback : value;
}

QString parameterToken(double value) {
    if (!std::isfinite(value)) return QStringLiteral("unknown");
    return QString::number(value, 'f', 3);
}

QString csvField(QString value) {
    value.replace(QLatin1Char('"'), QStringLiteral("\"\""));
    return QStringLiteral("\"%1\"").arg(value);
}

QString uniqueImagePath(const QDir& directory, const QString& fileName) {
    QString candidate = directory.absoluteFilePath(fileName);
    if (!QFileInfo::exists(candidate)) return candidate;

    const QFileInfo info(fileName);
    const QString stem = info.completeBaseName();
    const QString suffix = info.suffix();
    for (int sequence = 2; sequence < 10000; ++sequence) {
        candidate = directory.absoluteFilePath(
            QStringLiteral("%1_%2.%3").arg(stem).arg(sequence).arg(suffix));
        if (!QFileInfo::exists(candidate)) return candidate;
    }
    return QString();
}

bool appendManifest(const QString& manifestPath,
                    const CaptureRecord& record,
                    const QString& imageFileName,
                    QString* error) {
    QFile manifest(manifestPath);
    const bool writeHeader = !manifest.exists() || manifest.size() == 0;
    if (!manifest.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法追加参数清单 %1：%2")
                         .arg(manifestPath, manifest.errorString());
        }
        return false;
    }

    QTextStream stream(&manifest);
    stream.setCodec("UTF-8");
    if (writeHeader) {
        stream << "capture_time,profile_id,wavelength_nm,camera_ip,camera_model,"
                  "camera_serial,width,height,pixel_format,exposure_us,gain_db,"
                  "frame_no,device_timestamp,host_timestamp,laser_state,"
                  "laser_status_reachable,laser_status_source_monotonic_ns,"
                  "ttl_450_high,ttl_650_high,file_name\n";
    }
    stream << csvField(record.capturedAt.toString(Qt::ISODateWithMs)) << ','
           << csvField(record.profileId) << ','
           << record.wavelengthNm << ','
           << csvField(record.cameraIp) << ','
           << csvField(record.cameraModel) << ','
           << csvField(record.cameraSerial) << ','
           << record.width << ','
           << record.height << ','
           << csvField(record.pixelFormat) << ','
           << QString::number(record.exposureUs, 'f', 3) << ','
           << QString::number(record.gainDb, 'f', 3) << ','
           << record.frameNumber << ','
           << record.deviceTimestamp << ','
           << record.hostTimestamp << ','
           << csvField(record.laserState) << ','
           << (record.laserStatusReachable ? 1 : 0) << ','
           << record.laserStatusSourceMonotonicNs << ','
           << (record.ttl450High ? 1 : 0) << ','
           << (record.ttl650High ? 1 : 0) << ','
           << csvField(imageFileName) << '\n';
    stream.flush();
    if (stream.status() != QTextStream::Ok || manifest.error() != QFile::NoError) {
        if (error) {
            *error = QStringLiteral("写入参数清单 %1 失败：%2")
                         .arg(manifestPath, manifest.errorString());
        }
        return false;
    }
    return true;
}

} // namespace

QString makeCaptureFileName(const CaptureRecord& record) {
    const QDateTime timestamp = record.capturedAt.isValid()
        ? record.capturedAt
        : QDateTime::currentDateTime();
    return QStringLiteral(
        "%1_%2_%3_%4x%5_%6_exp%7us_gain%8dB_%9.png")
        .arg(safeToken(record.profileId, QStringLiteral("camera")),
             safeToken(record.cameraModel, QStringLiteral("unknown-model")),
             safeToken(record.cameraSerial, QStringLiteral("unknown-sn")))
        .arg(record.width)
        .arg(record.height)
        .arg(safeToken(record.pixelFormat, QStringLiteral("unknown-pixel")),
             parameterToken(record.exposureUs),
             parameterToken(record.gainDb),
             timestamp.toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")));
}

bool saveCapture(const QImage& image,
                 const CaptureRecord& inputRecord,
                 const QString& outputDirectory,
                 QString* savedImagePath,
                 QString* error) {
    if (savedImagePath) savedImagePath->clear();
    if (error) error->clear();
    if (image.isNull() || image.width() <= 0 || image.height() <= 0) {
        if (error) *error = QStringLiteral("不能保存空图像");
        return false;
    }
    if (outputDirectory.trimmed().isEmpty()) {
        if (error) *error = QStringLiteral("输出目录为空");
        return false;
    }

    QDir directory(QDir::cleanPath(outputDirectory));
    if (!directory.exists() && !QDir().mkpath(directory.absolutePath())) {
        if (error) {
            *error = QStringLiteral("无法创建输出目录：%1")
                         .arg(directory.absolutePath());
        }
        return false;
    }

    CaptureRecord record = inputRecord;
    record.width = image.width();
    record.height = image.height();
    if (!record.capturedAt.isValid()) record.capturedAt = QDateTime::currentDateTime();

    const QString imagePath = uniqueImagePath(directory, makeCaptureFileName(record));
    if (imagePath.isEmpty()) {
        if (error) *error = QStringLiteral("无法生成不重复的图片文件名");
        return false;
    }

    QSaveFile imageFile(imagePath);
    if (!imageFile.open(QIODevice::WriteOnly)) {
        if (error) {
            *error = QStringLiteral("无法创建图片 %1：%2")
                         .arg(imagePath, imageFile.errorString());
        }
        return false;
    }
    if (!image.save(&imageFile, "PNG")) {
        imageFile.cancelWriting();
        if (error) *error = QStringLiteral("PNG 编码失败：%1").arg(imagePath);
        return false;
    }
    if (!imageFile.commit()) {
        if (error) {
            *error = QStringLiteral("提交图片 %1 失败：%2")
                         .arg(imagePath, imageFile.errorString());
        }
        return false;
    }

    const QString manifestPath = directory.absoluteFilePath(
        QStringLiteral("captures.csv"));
    if (!appendManifest(manifestPath, record, QFileInfo(imagePath).fileName(), error)) {
        if (savedImagePath) *savedImagePath = imagePath;
        return false;
    }

    if (savedImagePath) *savedImagePath = imagePath;
    return true;
}

} // namespace hik_exposure_test
