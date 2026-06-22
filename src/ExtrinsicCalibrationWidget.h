#ifndef EXTRINSIC_CALIBRATION_WIDGET_H
#define EXTRINSIC_CALIBRATION_WIDGET_H

#include "geometry/TransformUtils.h"
#include "robot_client/FairinoRobotClient.h"

#include <QImage>
#include <QVector>
#include <QWidget>

#include <opencv2/core.hpp>

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class RgbImageLabel;
class ScanWorker;

class ExtrinsicCalibrationWidget : public QWidget {
    Q_OBJECT
public:
    explicit ExtrinsicCalibrationWidget(QWidget* parent = nullptr);
    ~ExtrinsicCalibrationWidget() override;

    void setWorker(ScanWorker* worker);

signals:
    void reqGrabLeftEye(int frameRate, int exposure, int gain, bool keepLaserOn);
    void reqDetectLeftEyeCharuco(int frameRate, int exposure, int gain, int squaresX, int squaresY,
                                 double squareMm, double markerMm, QString dictionaryName,
                                 bool keepLaserOn);

private slots:
    void onGrabLeftEyeClicked();
    void onDetectClicked();
    void onAddSampleClicked();
    void onClearSamplesClicked();
    void onSolveClicked();
    void onConnectRobotClicked();
    void onDisconnectRobotClicked();
    void onReadFlangeClicked();
    void onLeftEyeImageReady(QImage image, QString desc);
    void onCharucoResult(bool ok, QImage image, QString desc,
                         QVector<double> tLeftBoard, QString report);
    void onWorkerLog(QString msg);
    void onWorkerBusyChanged(bool busy);

private:
    struct CalibrationSample {
        weld_geometry::Pose6D flangePose;
        cv::Mat tLeftBoard;
        QString desc;
    };

    QSpinBox* makeIntSpin(int minValue, int maxValue, int value, int step = 1);
    QDoubleSpinBox* makeDoubleSpin(double minValue, double maxValue, double value,
                                   int decimals = 3, double step = 1.0);
    QDoubleSpinBox* makePoseSpin(double value);

    void appendLog(const QString& msg);
    void setRobotUi(bool connected);
    void setCameraUi(bool busy);
    bool ensureWorker() const;
    int frameRateValue() const;
    int exposureValue() const;
    int gainValue() const;
    QString dictionaryName() const;
    QImage displayImageForPreview(const QImage& image) const;
    weld_geometry::Pose6D currentFlangePose() const;
    void setFlangePose(const weld_geometry::Pose6D& pose);
    bool readCurrentFlangePose();
    void addSample(const weld_geometry::Pose6D& pose, const cv::Mat& tLeftBoard, const QString& desc);
    void refreshSampleTable();
    void updateSampleActions();
    QString saveResultYaml(const cv::Mat& tFlangeLeft, const QString& summary) const;
    QString dataDirPath() const;

    ScanWorker* m_worker{};
    RgbImageLabel* m_imageLabel{};
    QLabel* m_imageInfoLabel{};
    QLabel* m_robotStatusLabel{};
    QPlainTextEdit* m_log{};
    QTableWidget* m_sampleTable{};

    QSpinBox* m_frameRateSpin{};
    QSpinBox* m_exposureSpin{};
    QSpinBox* m_gainSpin{};
    QSpinBox* m_squaresXSpin{};
    QSpinBox* m_squaresYSpin{};
    QDoubleSpinBox* m_squareMmSpin{};
    QDoubleSpinBox* m_markerMmSpin{};
    QComboBox* m_dictionaryCombo{};
    QCheckBox* m_previewStretchCheck{};
    QCheckBox* m_keepLaserCheck{};

    QLineEdit* m_ipEdit{};
    QDoubleSpinBox* m_flangeX{};
    QDoubleSpinBox* m_flangeY{};
    QDoubleSpinBox* m_flangeZ{};
    QDoubleSpinBox* m_flangeRx{};
    QDoubleSpinBox* m_flangeRy{};
    QDoubleSpinBox* m_flangeRz{};

    QPushButton* m_btnGrabLeft{};
    QPushButton* m_btnDetect{};
    QPushButton* m_btnAddSample{};
    QPushButton* m_btnClearSamples{};
    QPushButton* m_btnSolve{};
    QPushButton* m_btnConnectRobot{};
    QPushButton* m_btnDisconnectRobot{};
    QPushButton* m_btnReadFlange{};

    fairino_client::FairinoRobotClient m_robot;
    fairino_client::RobotConfig m_robotConfig;
    QVector<CalibrationSample> m_samples;
    bool m_cameraBusy{false};
    bool m_pendingAddSample{false};
    weld_geometry::Pose6D m_pendingFlangePose{};
};

#endif // EXTRINSIC_CALIBRATION_WIDGET_H
