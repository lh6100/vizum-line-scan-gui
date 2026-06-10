#ifndef WELDSEAMWIDGET_H
#define WELDSEAMWIDGET_H

#include <QWidget>
#include <QString>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vtkSmartPointer.h>

#include <vector>

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QPlainTextEdit;
class QSpinBox;
class QVTKOpenGLNativeWidget;

class vtkActor;
class vtkCallbackCommand;
class vtkLineSource;
class vtkObject;
class vtkPoints;
class vtkPolyData;
class vtkRenderer;
class vtkSphereWidget;

class WeldSeamWidget : public QWidget {
    Q_OBJECT

public:
    explicit WeldSeamWidget(QWidget* parent = nullptr);
    ~WeldSeamWidget() override;

signals:
    void cameraLineChanged(double sx, double sy, double sz, double ex, double ey, double ez);

private slots:
    void loadPointCloud();
    void clearFit();
    void copyResult();

private:
    using PointT = pcl::PointXYZRGBA;
    using CloudT = pcl::PointCloud<PointT>;

    static void onMiddleButtonPress(vtkObject* caller, unsigned long eventId, void* clientData, void* callData);
    static void onStartHandleMoved(vtkObject* caller, unsigned long eventId, void* clientData, void* callData);
    static void onEndHandleMoved(vtkObject* caller, unsigned long eventId, void* clientData, void* callData);

    void setupVtk();
    void loadCloudFromPath(const QString& filePath);
    void updateCloudActor();
    void fitAtPointId(vtkIdType pointId);
    bool fitLineFromIndices(const std::vector<int>& indices, Eigen::Vector3f& start, Eigen::Vector3f& end,
                            std::vector<int>& inlierIndices, QString& errorText) const;
    bool fitLinePca(const std::vector<int>& indices, Eigen::Vector3f& start, Eigen::Vector3f& end) const;
    void showFitResult(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const std::vector<int>& inliers);
    void updateLineActor();
    void updateHandleWidgets();
    void updateResultText();
    void appendLog(const QString& msg);

private:
    QVTKOpenGLNativeWidget* m_vtkWidget{};
    QPushButton* m_btnLoad{};
    QPushButton* m_btnClear{};
    QPushButton* m_btnCopy{};
    QDoubleSpinBox* m_radiusSpin{};
    QDoubleSpinBox* m_ransacThresholdSpin{};
    QSpinBox* m_minPointsSpin{};
    QLabel* m_fileLabel{};
    QLabel* m_pickLabel{};
    QPlainTextEdit* m_resultText{};

    CloudT::Ptr m_cloud;
    pcl::KdTreeFLANN<PointT> m_kdtree;
    bool m_treeReady{false};
    std::vector<int> m_displayPointToCloudIndex;
    const std::size_t m_maxDisplayPoints{1200000};

    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkPolyData> m_cloudPolyData;
    vtkSmartPointer<vtkActor> m_cloudActor;
    vtkSmartPointer<vtkPolyData> m_inlierPolyData;
    vtkSmartPointer<vtkActor> m_inlierActor;
    vtkSmartPointer<vtkLineSource> m_lineSource;
    vtkSmartPointer<vtkActor> m_lineActor;
    vtkSmartPointer<vtkSphereWidget> m_startHandle;
    vtkSmartPointer<vtkSphereWidget> m_endHandle;
    vtkSmartPointer<vtkCallbackCommand> m_pickCallback;
    vtkSmartPointer<vtkCallbackCommand> m_startHandleCallback;
    vtkSmartPointer<vtkCallbackCommand> m_endHandleCallback;

    bool m_hasFit{false};
    Eigen::Vector3f m_startPoint{0.f, 0.f, 0.f};
    Eigen::Vector3f m_endPoint{0.f, 0.f, 0.f};
    QString m_loadedPath;
};

#endif // WELDSEAMWIDGET_H
