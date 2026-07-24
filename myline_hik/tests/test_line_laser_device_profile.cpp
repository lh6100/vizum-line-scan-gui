#include "LineLaserDeviceProfile.h"

#include <QDir>

#include <cstdlib>
#include <iostream>
#include <set>

namespace {

int failures = 0;

void check(bool condition, const char* message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAILED: " << message << '\n';
    }
}

} // namespace

int main() {
    const std::vector<LineLaserDeviceProfile>& profiles =
        lineLaserDeviceProfiles();
    check(profiles.size() == 2U, "exactly two device profiles are required");

    const LineLaserDeviceProfile* scanner450 =
        findLineLaserDeviceProfile(QStringLiteral("scanner_450"));
    const LineLaserDeviceProfile* scanner650 =
        findLineLaserDeviceProfile(QStringLiteral("scanner_650"));
    check(scanner450 != nullptr, "scanner_450 must exist");
    check(scanner650 != nullptr, "scanner_650 must exist");
    check(findLineLaserDeviceProfile(QStringLiteral("unknown")) == nullptr,
          "unknown profiles must not fall back to a default");
    if (!scanner450 || !scanner650) {
        return EXIT_FAILURE;
    }

    QString error;
    check(scanner450->isValid(&error),
          "scanner_450 profile must pass validation");
    check(scanner650->isValid(&error),
          "scanner_650 profile must pass validation");
    check(scanner450->wavelengthNm == 450 &&
              scanner450->ttlPhysicalPin == 11,
          "scanner_450 wavelength/TTL mapping is wrong");
    check(scanner650->wavelengthNm == 650 &&
              scanner650->ttlPhysicalPin == 7,
          "scanner_650 wavelength/TTL mapping is wrong");
    check(scanner450->displayName.contains(QStringLiteral("130万相机")) &&
              scanner450->displayName.contains(QStringLiteral("12.5 mm")),
          "scanner_450 optical hardware label is wrong");
    check(scanner650->displayName.contains(QStringLiteral("160万相机")) &&
              scanner650->displayName.contains(QStringLiteral("6 mm")),
          "scanner_650 optical hardware label is wrong");
    check(scanner450->defaultCameraIp.isEmpty() &&
              scanner450->expectedCameraModel.isEmpty() &&
              scanner450->expectedCameraSerial.isEmpty(),
          "scanner_450 unknown camera identity must remain user-configurable");
    check(scanner650->defaultCameraIp == QStringLiteral("192.168.7.45") &&
              scanner650->expectedCameraModel == QStringLiteral("MV-CS016-10GM") &&
              scanner650->expectedCameraSerial == QStringLiteral("DA8784601"),
          "scanner_650 calibrated camera identity changed");
    check(scanner650->cameraFrame == QStringLiteral("hik_camera_optical_frame"),
          "scanner_650 legacy frame must remain hash-compatible");
    check(scanner450->cameraFrame != scanner650->cameraFrame,
          "scanner_450 must use an independent camera frame");
    check(scanner450->stripeOrientation ==
              LineLaserStripeOrientation::Auto &&
              scanner450->calibrationCenterlinePolicy ==
                  LineLaserCenterlinePolicy::Quality &&
              scanner450->scanCenterlinePolicy ==
                  LineLaserCenterlinePolicy::Quality,
          "uncalibrated scanner_450 must use the quality extractor while "
          "keeping orientation auto until calibration confirms it");
    check(scanner650->stripeOrientation ==
              LineLaserStripeOrientation::Horizontal &&
              scanner650->calibrationCenterlinePolicy ==
                  LineLaserCenterlinePolicy::Legacy &&
              scanner650->scanCenterlinePolicy ==
                  LineLaserCenterlinePolicy::Shadow,
          "calibrated scanner_650 must keep legacy calibration semantics and "
          "run the quality extractor in shadow mode");
    check(scanner650->stripeRoiY > 0.0 &&
              scanner650->stripeRoiY + scanner650->stripeRoiHeight < 1.0,
          "scanner_650 must use a fixed software ROI around its horizontal stripe");

    const QString sourceRoot = QDir::cleanPath(
        QStringLiteral("/tmp/myline_hik_profile_test_root"));
    check(scanner650->intrinsicsConfigPath(sourceRoot) ==
              QDir(sourceRoot).absoluteFilePath(
                  QStringLiteral("config/hik_intrinsics.yaml")),
          "scanner_650 must continue using the calibrated intrinsic path");
    check(scanner650->laserPlaneConfigPath(sourceRoot) ==
              QDir(sourceRoot).absoluteFilePath(
                  QStringLiteral("config/hik_laser_plane.yaml")),
          "scanner_650 must continue using the calibrated laser-plane path");
    check(scanner650->handEyeConfigPath(sourceRoot) ==
              QDir(sourceRoot).absoluteFilePath(
                  QStringLiteral("config/hik_handeye.yaml")),
          "scanner_650 must continue using the calibrated hand-eye path");
    check(scanner450->intrinsicsConfigPath(sourceRoot).contains(
              QStringLiteral("/config/devices/scanner_450/")),
          "scanner_450 intrinsics must stay inside its own config directory");
    check(scanner450->laserPlaneConfigPath(sourceRoot).contains(
              QStringLiteral("/config/devices/scanner_450/")),
          "scanner_450 laser plane must stay inside its own config directory");
    check(scanner450->handEyeConfigPath(sourceRoot).contains(
              QStringLiteral("/config/devices/scanner_450/")),
          "scanner_450 hand-eye must stay inside its own config directory");
    check(scanner450->synchronizationConfigPath(sourceRoot).endsWith(
              QStringLiteral(
                  "/config/devices/scanner_450/synchronization.yaml")) &&
              scanner650->synchronizationConfigPath(sourceRoot).endsWith(
                  QStringLiteral(
                      "/config/synchronization.yaml")) &&
              scanner450->synchronizationConfigPath(sourceRoot) !=
                  scanner650->synchronizationConfigPath(sourceRoot),
          "synchronization settings must be profile-scoped");

    std::set<QString> formalPaths;
    for (const LineLaserDeviceProfile& profile : profiles) {
        formalPaths.insert(profile.intrinsicsConfigPath(sourceRoot));
        formalPaths.insert(profile.laserPlaneConfigPath(sourceRoot));
        formalPaths.insert(profile.handEyeConfigPath(sourceRoot));
    }
    check(formalPaths.size() == profiles.size() * 3U,
          "formal calibration paths must not collide across profiles");
    check(scanner450->calibrationSessionRoot(sourceRoot) !=
              scanner650->calibrationSessionRoot(sourceRoot),
          "calibration session roots must be isolated");
    check(scanner450->calibrationSessionRoot(sourceRoot).endsWith(
              QStringLiteral("/data/calibration/scanner_450")) &&
              scanner650->calibrationSessionRoot(sourceRoot).endsWith(
                  QStringLiteral("/data/calibration/scanner_650")),
          "profile session roots use the wrong layout");
    check(scanner450->scanSessionRoot(sourceRoot).endsWith(
              QStringLiteral("/data/scans/scanner_450")) &&
              scanner650->scanSessionRoot(sourceRoot).endsWith(
                  QStringLiteral("/data/scans/scanner_650")) &&
              scanner450->scanSessionRoot(sourceRoot) !=
                  scanner650->scanSessionRoot(sourceRoot),
          "scan session roots must be profile-scoped");

    if (failures != 0) {
        std::cerr << "LineLaserDeviceProfile tests failed: "
                  << failures << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "LineLaserDeviceProfile tests passed\n";
    return EXIT_SUCCESS;
}
