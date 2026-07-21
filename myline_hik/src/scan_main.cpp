#include "HikConstantLaserScanWindow.h"

#include <QApplication>

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    application.setApplicationName(QStringLiteral("HikConstantLaserScan"));
    HikConstantLaserScanWindow window;
    window.show();
    return application.exec();
}
