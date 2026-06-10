#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QImage>
#include <QPoint>
#include <QVector>
#include <QVector3D>

class QPushButton;
class QPlainTextEdit;
class QLabel;
class ScanWorker;
class WeldSeamWidget;
class RgbImageLabel;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void closeEvent(QCloseEvent* ev) override;

private slots:
    void onConnectClicked();
    void onDisconnectClicked();
    void onRebootClicked();
    void onGetVersionsClicked();
    void onOpenCoverClicked();
    void onCloseCoverClicked();
    void onScanClicked();
    void onGrabRgbClicked();

    void onLog(QString msg);
    void onConnectionChanged(bool connected);
    void onScanFinished(bool ok, QString path);
    void onBusyChanged(bool busy);
    void onRgbImageReady(QImage image, QString desc);
    void onRgbLineSelected(QPoint start, QPoint end);
    void onRgbLineMapped(QVector<QVector3D> points, QString desc);

signals:
    // Cross-thread invokers to ScanWorker
    void reqInit();
    void reqConnect();
    void reqDisconnect();
    void reqReboot();
    void reqDestroy();
    void reqGetVersions();
    void reqOpenCover();
    void reqCloseCover();
    void reqScan(QString path);
    void reqGrabRgb();
    void reqMapRgbLine(QPoint start, QPoint end, int sampleCount);

private:
    void setBusy(bool busy);
    QString defaultPlyPath();
    QString defaultRgbPath();
    QString dataDirPath();

    QPushButton* m_btnConnect{};
    QPushButton* m_btnDisconnect{};
    QPushButton* m_btnReboot{};
    QPushButton* m_btnGetVersions{};
    QPushButton* m_btnOpenCover{};
    QPushButton* m_btnCloseCover{};
    QPushButton* m_btnScan{};
    QPushButton* m_btnGrabRgb{};
    RgbImageLabel* m_rgbImageLabel{};
    QLabel* m_rgbInfoLabel{};
    QPlainTextEdit* m_logView{};
    QLabel* m_statusLabel{};
    WeldSeamWidget* m_weldWidget{};

    QThread m_workerThread;
    ScanWorker* m_worker{};

    bool m_connected{false};
    int m_scanCounter{0};
    int m_rgbCounter{0};
};

#endif // MAINWINDOW_H
