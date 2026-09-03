#include "FairinoRobotSession.h"
#include "HikCalibrationWindow.h"
#include "LineLaserController.h"
#include "LineLaserDeviceProfile.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QStringList>
#include <QTabWidget>
#include <QTimer>

#include <exception>
#include <vector>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("HikLineLaserCalibration"));
    QCoreApplication::setOrganizationName(QStringLiteral("Vizum"));

    const QStringList arguments = QCoreApplication::arguments();
    QString requestedProfileId;
    for (int index = 1; index < arguments.size(); ++index) {
        const QString argument = arguments.at(index);
        if (argument == QStringLiteral("--profile")) {
            if (index + 1 >= arguments.size()) {
                qCritical().noquote() << "--profile 缺少设备 ID";
                return 2;
            }
            requestedProfileId = arguments.at(++index).trimmed();
        } else if (argument.startsWith(QStringLiteral("--profile="))) {
            requestedProfileId =
                argument.mid(QStringLiteral("--profile=").size()).trimmed();
        }
    }

    std::vector<const LineLaserDeviceProfile*> selectedProfiles;
    if (!requestedProfileId.isEmpty()) {
        const LineLaserDeviceProfile* profile =
            findLineLaserDeviceProfile(requestedProfileId);
        if (!profile) {
            qCritical().noquote()
                << QStringLiteral("未知设备 profile：%1；可选 scanner_450/scanner_650")
                       .arg(requestedProfileId);
            return 2;
        }
        selectedProfiles.push_back(profile);
    } else {
        for (const LineLaserDeviceProfile& profile : lineLaserDeviceProfiles()) {
            selectedProfiles.push_back(&profile);
        }
    }

    FairinoRobotSession robotSession;
    LineLaserController laserController;
    QTabWidget deviceTabs;
    deviceTabs.setWindowTitle(QStringLiteral("海康双线激光标定"));
    deviceTabs.setDocumentMode(true);
    deviceTabs.resize(1600, 960);
    try {
        for (const LineLaserDeviceProfile* profile : selectedProfiles) {
            HikCalibrationWindow* window =
                new HikCalibrationWindow(
                    *profile, &laserController,
                    &robotSession, &deviceTabs);
            deviceTabs.addTab(
                window,
                QStringLiteral("%1｜%2").arg(profile->id, profile->displayName));
        }
    } catch (const std::exception& exception) {
        qCritical().noquote()
            << QStringLiteral("标定窗口初始化失败：%1")
                   .arg(QString::fromUtf8(exception.what()));
        return 1;
    }
    QObject::connect(
        &deviceTabs, &QTabWidget::currentChanged, &deviceTabs,
        [&deviceTabs](int currentIndex) {
            for (int index = 0; index < deviceTabs.count(); ++index) {
                auto* window = qobject_cast<HikCalibrationWindow*>(
                    deviceTabs.widget(index));
                if (window) {
                    window->setProfileTabActive(index == currentIndex);
                }
            }
        });
    QObject::connect(&application, &QCoreApplication::aboutToQuit,
                     &laserController, &LineLaserController::off);
    for (int index = 0; index < deviceTabs.count(); ++index) {
        auto* window = qobject_cast<HikCalibrationWindow*>(
            deviceTabs.widget(index));
        if (window) {
            window->setProfileTabActive(
                index == deviceTabs.currentIndex());
        }
    }

    deviceTabs.show();
    if (arguments.contains(QStringLiteral("--smoke-test"))) {
        QTimer::singleShot(250, &application, &QCoreApplication::quit);
    } else {
        QTimer::singleShot(
            0, &laserController,
            &LineLaserController::connectController);
    }
    return application.exec();
}
