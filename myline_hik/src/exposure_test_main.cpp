#include "LineLaserController.h"
#include "exposure_test/ExposureTestWindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QTimer>

#include <exception>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("HikExposureTest"));
    QCoreApplication::setOrganizationName(QStringLiteral("Vizum"));

    LineLaserController laserController;
    try {
        ExposureTestWindow window(&laserController);
        QObject::connect(&application, &QCoreApplication::aboutToQuit,
                         &laserController, &LineLaserController::off);
        window.show();
        if (QCoreApplication::arguments().contains(QStringLiteral("--smoke-test"))) {
            QTimer::singleShot(250, &application, &QCoreApplication::quit);
        } else {
            QTimer::singleShot(0, &laserController,
                               &LineLaserController::connectController);
        }
        return application.exec();
    } catch (const std::exception& exception) {
        qCritical("曝光测试窗口初始化失败: %s", exception.what());
        return 1;
    }
}
