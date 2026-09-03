#ifndef MYLINE_HIK_LINE_LASER_DEVICE_PROFILE_H
#define MYLINE_HIK_LINE_LASER_DEVICE_PROFILE_H

#include <QString>

#include <vector>

enum class LineLaserStripeOrientation {
    Auto,
    Horizontal,
    Vertical
};

enum class LineLaserCenterlinePolicy {
    Legacy,
    Shadow,
    Quality
};

struct LineLaserDeviceProfile {
    QString id;
    QString displayName;
    int wavelengthNm{0};
    int ttlPhysicalPin{0};
    QString defaultCameraIp;
    QString expectedCameraModel;
    QString expectedCameraSerial;
    QString cameraFrame;
    QString intrinsicsConfigRelativePath;
    QString laserPlaneConfigRelativePath;
    QString handEyeConfigRelativePath;
    QString synchronizationConfigRelativePath;
    QString calibrationSessionRootRelativePath;
    QString scanSessionRootRelativePath;
    // Optional profile-specific planner configuration. An empty path means
    // that the device does not provide adaptive scan planning.
    QString adaptiveScanConfigRelativePath;
    LineLaserStripeOrientation stripeOrientation{
        LineLaserStripeOrientation::Auto};
    LineLaserCenterlinePolicy calibrationCenterlinePolicy{
        LineLaserCenterlinePolicy::Legacy};
    LineLaserCenterlinePolicy scanCenterlinePolicy{
        LineLaserCenterlinePolicy::Shadow};
    // Normalized software ROI. It preserves the calibrated image geometry and
    // is intersected with the laser-plane valid-depth corridor at runtime.
    double stripeRoiX{0.0};
    double stripeRoiY{0.0};
    double stripeRoiWidth{1.0};
    double stripeRoiHeight{1.0};

    QString intrinsicsConfigPath(const QString& sourceDirectory) const;
    QString laserPlaneConfigPath(const QString& sourceDirectory) const;
    QString handEyeConfigPath(const QString& sourceDirectory) const;
    QString synchronizationConfigPath(
        const QString& sourceDirectory) const;
    QString calibrationSessionRoot(const QString& sourceDirectory) const;
    QString scanSessionRoot(const QString& sourceDirectory) const;
    QString adaptiveScanConfigPath(const QString& sourceDirectory) const;
    bool isValid(QString* error = nullptr) const;
};

const std::vector<LineLaserDeviceProfile>& lineLaserDeviceProfiles();
const LineLaserDeviceProfile* findLineLaserDeviceProfile(const QString& id);
QString lineLaserStripeOrientationName(LineLaserStripeOrientation value);
QString lineLaserCenterlinePolicyName(LineLaserCenterlinePolicy value);

#endif // MYLINE_HIK_LINE_LASER_DEVICE_PROFILE_H
