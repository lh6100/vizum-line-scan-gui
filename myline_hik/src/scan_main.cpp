#include "FairinoRobotSession.h"
#include "HikConstantLaserScanWindow.h"
#include "LineLaserController.h"
#include "LineLaserDeviceProfile.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QStringList>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    application.setApplicationName(QStringLiteral("HikConstantLaserScan"));
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "FR5/Hikrobot line-laser calibration, stop-and-shoot validation, and continuous time synchronization"));
    parser.addHelpOption();
    const QCommandLineOption scanSpeedOption(
        QStringLiteral("scan-speed"),
        QStringLiteral("连续同步的实际 TCP 目标速度，范围 10–50 mm/s。"),
        QStringLiteral("mm/s"));
    const QCommandLineOption profileOption(
        QStringLiteral("profile"),
        QStringLiteral("仅打开指定设备组；省略时以标签页同时显示全部设备组。"),
        QStringLiteral("profile-id"));
    const QCommandLineOption smokeTestOption(
        QStringLiteral("smoke-test"),
        QStringLiteral("创建界面后立即退出；不连接任何硬件。"));
    parser.addOption(scanSpeedOption);
    parser.addOption(profileOption);
    parser.addOption(smokeTestOption);
    parser.process(application);
    const bool smokeTest = parser.isSet(smokeTestOption);

    double scanSpeedOverride = -1.0;
    if (parser.isSet(scanSpeedOption)) {
        bool ok = false;
        scanSpeedOverride = parser.value(scanSpeedOption).toDouble(&ok);
        if (!ok || scanSpeedOverride < 10.0 || scanSpeedOverride > 50.0) {
            QTextStream(stderr) << "错误：--scan-speed 必须是 10–50 mm/s，收到："
                                << parser.value(scanSpeedOption) << '\n';
            return 2;
        }
    }

    FairinoRobotSession robotSession;

    if (parser.isSet(profileOption)) {
        const QString requestedProfile = parser.value(profileOption).trimmed();
        const LineLaserDeviceProfile* profile =
            findLineLaserDeviceProfile(requestedProfile);
        if (!profile) {
            QStringList validIds;
            for (const LineLaserDeviceProfile& candidate :
                 lineLaserDeviceProfiles()) {
                validIds.push_back(candidate.id);
            }
            QTextStream(stderr) << "错误：未知 --profile="
                                << requestedProfile << "；可选值："
                                << validIds.join(QStringLiteral(", ")) << '\n';
            return 2;
        }
        QString profileError;
        if (!profile->isValid(&profileError)) {
            QTextStream(stderr) << "错误：设备 profile 无效："
                                << profileError << '\n';
            return 2;
        }
        LineLaserController laserController;
        HikConstantLaserScanWindow window(
            *profile, &laserController, &robotSession,
            nullptr, scanSpeedOverride);
        QObject::connect(&application, &QCoreApplication::aboutToQuit,
                         &laserController, &LineLaserController::off);
        window.show();
        if (smokeTest) {
            QTimer::singleShot(0, &application, &QCoreApplication::quit);
        } else {
            QTimer::singleShot(
                0, &laserController,
                &LineLaserController::connectController);
        }
        return application.exec();
    }

    LineLaserController laserController;
    QTabWidget tabs;
    tabs.setWindowTitle(QStringLiteral("FR5 双组线激光独立扫描"));
    tabs.resize(1640, 1000);
    tabs.setDocumentMode(true);

    HikConstantLaserScanWindow* activeScanWindow = nullptr;
    for (const LineLaserDeviceProfile& profile : lineLaserDeviceProfiles()) {
        QString profileError;
        if (!profile.isValid(&profileError)) {
            QTextStream(stderr) << "错误：设备 profile 无效："
                                << profileError << '\n';
            return 2;
        }
        auto* window = new HikConstantLaserScanWindow(
            profile, &laserController, &robotSession,
            &tabs, scanSpeedOverride);
        const int tabIndex = tabs.addTab(window, profile.displayName);
        tabs.setTabToolTip(tabIndex,
            QStringLiteral("profile=%1｜%2 nm｜TTL Pin %3")
                .arg(profile.id)
                .arg(profile.wavelengthNm)
                .arg(profile.ttlPhysicalPin));
        QObject::connect(
            window, &HikConstantLaserScanWindow::scanActivityChanged,
            &tabs,
            [&tabs, &activeScanWindow, window](bool active) {
                if (active) {
                    activeScanWindow = window;
                    for (int index = 0; index < tabs.count(); ++index) {
                        tabs.setTabEnabled(
                            index, tabs.widget(index) == window);
                    }
                } else if (activeScanWindow == window) {
                    activeScanWindow = nullptr;
                    for (int index = 0; index < tabs.count(); ++index) {
                        tabs.setTabEnabled(index, true);
                    }
                }
            });
    }
    QObject::connect(
        &tabs, &QTabWidget::currentChanged, &tabs,
        [&tabs](int currentIndex) {
            for (int index = 0; index < tabs.count(); ++index) {
                auto* window = qobject_cast<HikConstantLaserScanWindow*>(
                    tabs.widget(index));
                if (window) window->setProfileTabActive(index == currentIndex);
            }
        });
    for (int index = 0; index < tabs.count(); ++index) {
        auto* window = qobject_cast<HikConstantLaserScanWindow*>(
            tabs.widget(index));
        if (window) window->setProfileTabActive(index == tabs.currentIndex());
    }
    QObject::connect(&application, &QCoreApplication::aboutToQuit,
                     &laserController, &LineLaserController::off);
    tabs.show();
    if (smokeTest) {
        QTimer::singleShot(0, &application, &QCoreApplication::quit);
    } else {
        QTimer::singleShot(
            0, &laserController,
            &LineLaserController::connectController);
    }
    return application.exec();
}
