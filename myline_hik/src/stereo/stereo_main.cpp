#include "FairinoRobotSession.h"
#include "LineLaserController.h"
#include "stereo/app/StereoMapperWindow.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QTimer>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    application.setApplicationName(QStringLiteral("HikStereoMapper"));
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "Hikrobot grayscale stereo depth and FR5 base-frame occupancy mapper"));
    parser.addHelpOption();
    const QCommandLineOption smokeTest(
        QStringLiteral("smoke-test"),
        QStringLiteral("创建双目建图界面后退出，不连接硬件。"));
    parser.addOption(smokeTest);
    parser.process(application);

    FairinoRobotSession robotSession;
    LineLaserController laserController;
    hik_stereo::StereoMapperWindow window(
        &laserController, &robotSession);
    QObject::connect(&application, &QCoreApplication::aboutToQuit,
                     &laserController, &LineLaserController::off);
    window.show();
    if (parser.isSet(smokeTest)) {
        QTimer::singleShot(250, &application, &QCoreApplication::quit);
    } else {
        QTimer::singleShot(0, &laserController,
                           &LineLaserController::connectController);
    }
    return application.exec();
}
