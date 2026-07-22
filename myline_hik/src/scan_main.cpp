#include "HikConstantLaserScanWindow.h"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QTextStream>

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
    parser.addOption(scanSpeedOption);
    parser.process(application);

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
    HikConstantLaserScanWindow window(nullptr, scanSpeedOverride);
    window.show();
    return application.exec();
}
