#include "LineLaserDeviceProfile.h"

#include <QDir>
#include <QFileInfo>
#include <QRegularExpression>

#include <cmath>

namespace {

QString resolvedPath(const QString& sourceDirectory, const QString& relativePath) {
    return QDir::cleanPath(QDir(sourceDirectory).absoluteFilePath(relativePath));
}

bool validateRelativePath(const QString& value,
                          const QString& field,
                          QString* error) {
    if (value.trimmed().isEmpty() || QFileInfo(value).isAbsolute() ||
        value.split(QLatin1Char('/')).contains(QStringLiteral(".."))) {
        if (error) {
            *error = QStringLiteral("%1 必须是项目内的非空相对路径。").arg(field);
        }
        return false;
    }
    return true;
}

} // namespace

QString LineLaserDeviceProfile::intrinsicsConfigPath(
        const QString& sourceDirectory) const {
    return resolvedPath(sourceDirectory, intrinsicsConfigRelativePath);
}

QString LineLaserDeviceProfile::laserPlaneConfigPath(
        const QString& sourceDirectory) const {
    return resolvedPath(sourceDirectory, laserPlaneConfigRelativePath);
}

QString LineLaserDeviceProfile::handEyeConfigPath(
        const QString& sourceDirectory) const {
    return resolvedPath(sourceDirectory, handEyeConfigRelativePath);
}

QString LineLaserDeviceProfile::synchronizationConfigPath(
        const QString& sourceDirectory) const {
    return resolvedPath(sourceDirectory,
                        synchronizationConfigRelativePath);
}

QString LineLaserDeviceProfile::calibrationSessionRoot(
        const QString& sourceDirectory) const {
    return resolvedPath(sourceDirectory, calibrationSessionRootRelativePath);
}

QString LineLaserDeviceProfile::scanSessionRoot(
        const QString& sourceDirectory) const {
    return resolvedPath(sourceDirectory, scanSessionRootRelativePath);
}

QString LineLaserDeviceProfile::adaptiveScanConfigPath(
        const QString& sourceDirectory) const {
    if (adaptiveScanConfigRelativePath.trimmed().isEmpty()) {
        return QString();
    }
    return resolvedPath(sourceDirectory, adaptiveScanConfigRelativePath);
}

bool LineLaserDeviceProfile::isValid(QString* error) const {
    if (error) {
        error->clear();
    }
    static const QRegularExpression idPattern(
        QStringLiteral("^[a-z][a-z0-9_]*$"));
    if (!idPattern.match(id).hasMatch()) {
        if (error) {
            *error = QStringLiteral("设备 ID 非法: %1").arg(id);
        }
        return false;
    }
    if (displayName.trimmed().isEmpty() || wavelengthNm <= 0 ||
        ttlPhysicalPin <= 0 || cameraFrame.trimmed().isEmpty()) {
        if (error) {
            *error = QStringLiteral("设备 %1 的显示名、波长、TTL Pin 或 frame 无效。")
                .arg(id);
        }
        return false;
    }
    const bool roiFinite =
        std::isfinite(stripeRoiX) && std::isfinite(stripeRoiY) &&
        std::isfinite(stripeRoiWidth) && std::isfinite(stripeRoiHeight);
    if (!roiFinite || stripeRoiX < 0.0 || stripeRoiY < 0.0 ||
        stripeRoiWidth <= 0.0 || stripeRoiHeight <= 0.0 ||
        stripeRoiX + stripeRoiWidth > 1.0 + 1.0e-9 ||
        stripeRoiY + stripeRoiHeight > 1.0 + 1.0e-9) {
        if (error) {
            *error = QStringLiteral("设备 %1 的条纹归一化 ROI 无效。").arg(id);
        }
        return false;
    }
    return validateRelativePath(
               intrinsicsConfigRelativePath, QStringLiteral("内参路径"), error) &&
           validateRelativePath(
               laserPlaneConfigRelativePath, QStringLiteral("激光平面路径"), error) &&
           validateRelativePath(
               handEyeConfigRelativePath, QStringLiteral("手眼路径"), error) &&
           validateRelativePath(
               synchronizationConfigRelativePath,
               QStringLiteral("同步配置路径"), error) &&
           validateRelativePath(
               calibrationSessionRootRelativePath,
               QStringLiteral("标定 session 路径"), error) &&
           validateRelativePath(
               scanSessionRootRelativePath,
               QStringLiteral("扫描 session 路径"), error) &&
           (adaptiveScanConfigRelativePath.trimmed().isEmpty() ||
            validateRelativePath(
                adaptiveScanConfigRelativePath,
                QStringLiteral("自适应扫描配置路径"), error));
}

const std::vector<LineLaserDeviceProfile>& lineLaserDeviceProfiles() {
    static const std::vector<LineLaserDeviceProfile> profiles = {
        {
            QStringLiteral("scanner_450"),
            QStringLiteral("450 nm｜130万相机｜12.5 mm"),
            450,
            11,
            QStringLiteral("192.168.1.46"),
            QStringLiteral("MV-CS013-60GN"),
            QStringLiteral("DB0403208"),
            QStringLiteral("hik_450_camera_optical_frame"),
            QStringLiteral("config/devices/scanner_450/hik_intrinsics.yaml"),
            QStringLiteral("config/devices/scanner_450/hik_laser_plane.yaml"),
            QStringLiteral("config/devices/scanner_450/hik_handeye.yaml"),
            QStringLiteral("config/devices/scanner_450/synchronization.yaml"),
            QStringLiteral("data/calibration/scanner_450"),
            QStringLiteral("data/scans/scanner_450"),
            QString(),
            LineLaserStripeOrientation::Auto,
            LineLaserCenterlinePolicy::Quality,
            LineLaserCenterlinePolicy::Quality,
            0.0, 0.0, 1.0, 1.0
        },
        {
            QStringLiteral("scanner_650"),
            QStringLiteral("650 nm｜160万相机｜6 mm"),
            650,
            7,
            QStringLiteral("192.168.7.45"),
            QStringLiteral("MV-CS016-10GM"),
            QStringLiteral("DA8784601"),
            QStringLiteral("hik_camera_optical_frame"),
            QStringLiteral("config/devices/scanner_650/hik_intrinsics.yaml"),
            QStringLiteral("config/devices/scanner_650/hik_laser_plane.yaml"),
            QStringLiteral("config/devices/scanner_650/hik_handeye.yaml"),
            QStringLiteral("config/devices/scanner_650/synchronization.yaml"),
            QStringLiteral("data/calibration/scanner_650"),
            QStringLiteral("data/scans/scanner_650"),
            QStringLiteral(
                "config/devices/scanner_650/adaptive_scan_650.yaml"),
            LineLaserStripeOrientation::Horizontal,
            LineLaserCenterlinePolicy::Legacy,
            LineLaserCenterlinePolicy::Shadow,
            0.0, 0.20, 1.0, 0.58
        }
    };
    return profiles;
}

const LineLaserDeviceProfile* findLineLaserDeviceProfile(const QString& id) {
    const QString normalized = id.trimmed();
    const std::vector<LineLaserDeviceProfile>& profiles =
        lineLaserDeviceProfiles();
    for (const LineLaserDeviceProfile& profile : profiles) {
        if (profile.id == normalized) {
            return &profile;
        }
    }
    return nullptr;
}

QString lineLaserStripeOrientationName(LineLaserStripeOrientation value) {
    switch (value) {
    case LineLaserStripeOrientation::Auto:
        return QStringLiteral("auto");
    case LineLaserStripeOrientation::Horizontal:
        return QStringLiteral("horizontal");
    case LineLaserStripeOrientation::Vertical:
        return QStringLiteral("vertical");
    }
    return QStringLiteral("unknown");
}

QString lineLaserCenterlinePolicyName(LineLaserCenterlinePolicy value) {
    switch (value) {
    case LineLaserCenterlinePolicy::Legacy:
        return QStringLiteral("legacy");
    case LineLaserCenterlinePolicy::Shadow:
        return QStringLiteral("shadow");
    case LineLaserCenterlinePolicy::Quality:
        return QStringLiteral("quality");
    }
    return QStringLiteral("unknown");
}
