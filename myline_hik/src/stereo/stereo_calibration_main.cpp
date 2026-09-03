#include "FairinoRobotSession.h"
#include "LineLaserController.h"
#include "stereo/calibration_app/StereoCalibrationWindow.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QTimer>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    application.setApplicationName(QStringLiteral("HikStereoCalibration"));
    QCommandLineParser parser;
    parser.addHelpOption();
    const QCommandLineOption smokeTest(
        QStringLiteral("smoke-test"),
        QStringLiteral("创建双目标定界面后退出，不连接硬件。"));
    parser.addOption(smokeTest);
    parser.process(application);
    FairinoRobotSession robotSession;
    LineLaserController laserController;
    hik_stereo::StereoCalibrationWindow window(
        &laserController, &robotSession);
    QObject::connect(&application, &QCoreApplication::aboutToQuit,
                     &laserController, &LineLaserController::off);
    window.show();
    if (parser.isSet(smokeTest))
        QTimer::singleShot(250, &application, &QCoreApplication::quit);
    else
        QTimer::singleShot(0, &laserController,
                           &LineLaserController::connectController);
    return application.exec();
}
