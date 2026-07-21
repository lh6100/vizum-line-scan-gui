#include "HikCalibrationWindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QTimer>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("HikLineLaserCalibration"));
    QCoreApplication::setOrganizationName(QStringLiteral("Vizum"));

    HikCalibrationWindow window;
    window.show();
    if (QCoreApplication::arguments().contains(QStringLiteral("--smoke-test"))) {
        QTimer::singleShot(250, &application, &QCoreApplication::quit);
    }
    return application.exec();
}
