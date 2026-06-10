#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QImage>

class QPushButton;
class QPlainTextEdit;
class QLabel;
class ScanWorker;
class WeldSeamWidget;

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
    QLabel* m_rgbImageLabel{};
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
