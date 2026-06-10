#include "WeldSeamWidget.h"

#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include <QVTKOpenGLNativeWidget.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCellArray.h>
#include <vtkLineSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPointPicker.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPLYReader.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkCommand.h>
#include <vtkSphereWidget.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <numeric>

class WeldViewInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
    static WeldViewInteractorStyle* New();
    vtkTypeMacro(WeldViewInteractorStyle, vtkInteractorStyleTrackballCamera);

    void OnLeftButtonDown() override {}
    void OnLeftButtonUp() override {}
    void OnMiddleButtonDown() override {}
    void OnMiddleButtonUp() override {}

    void OnRightButtonDown() override {
        this->FindPokedRenderer(this->Interactor->GetEventPosition()[0],
                                this->Interactor->GetEventPosition()[1]);
        if (!this->CurrentRenderer) {
            return;
        }
        this->GrabFocus(this->EventCallbackCommand);
        this->StartRotate();
    }

    void OnRightButtonUp() override {
        if (this->State == VTKIS_ROTATE) {
            this->EndRotate();
        }
        if (this->Interactor) {
            this->ReleaseFocus();
        }
    }
};

vtkStandardNewMacro(WeldViewInteractorStyle);

WeldSeamWidget::WeldSeamWidget(QWidget* parent)
    : QWidget(parent),
      m_cloud(new CloudT) {
    auto* root = new QVBoxLayout(this);

    auto* controls = new QGroupBox("焊缝拟合", this);
    auto* grid = new QGridLayout(controls);

    m_btnLoad = new QPushButton("加载 PLY", this);
    m_btnClear = new QPushButton("清除拟合", this);
    m_btnCopy = new QPushButton("复制结果", this);

    m_radiusSpin = new QDoubleSpinBox(this);
    m_radiusSpin->setRange(1.0, 500.0);
    m_radiusSpin->setDecimals(1);
    m_radiusSpin->setSingleStep(5.0);
    m_radiusSpin->setValue(35.0);
    m_radiusSpin->setSuffix(" mm");

    m_ransacThresholdSpin = new QDoubleSpinBox(this);
    m_ransacThresholdSpin->setRange(0.1, 50.0);
    m_ransacThresholdSpin->setDecimals(2);
    m_ransacThresholdSpin->setSingleStep(0.5);
    m_ransacThresholdSpin->setValue(3.0);
    m_ransacThresholdSpin->setSuffix(" mm");

    m_minPointsSpin = new QSpinBox(this);
    m_minPointsSpin->setRange(10, 1000000);
    m_minPointsSpin->setValue(80);

    m_fileLabel = new QLabel("未加载点云", this);
    m_pickLabel = new QLabel("中键点击焊缝附近点进行拟合；滚轮缩放，右键旋转，左键拖动端点球体。", this);

    grid->addWidget(m_btnLoad, 0, 0);
    grid->addWidget(m_btnClear, 0, 1);
    grid->addWidget(m_btnCopy, 0, 2);
    grid->addWidget(new QLabel("邻域半径", this), 1, 0);
    grid->addWidget(m_radiusSpin, 1, 1);
    grid->addWidget(new QLabel("RANSAC距离阈值", this), 1, 2);
    grid->addWidget(m_ransacThresholdSpin, 1, 3);
    grid->addWidget(new QLabel("最少邻域点", this), 1, 4);
    grid->addWidget(m_minPointsSpin, 1, 5);
    grid->addWidget(m_fileLabel, 2, 0, 1, 3);
    grid->addWidget(m_pickLabel, 2, 3, 1, 3);
    root->addWidget(controls);

    auto* body = new QHBoxLayout;
    m_vtkWidget = new QVTKOpenGLNativeWidget(this);
    m_vtkWidget->setMinimumSize(760, 520);
    body->addWidget(m_vtkWidget, 1);

    m_resultText = new QPlainTextEdit(this);
    m_resultText->setReadOnly(true);
    m_resultText->setMinimumWidth(380);
    m_resultText->setMaximumBlockCount(1000);
    body->addWidget(m_resultText);
    root->addLayout(body, 1);

    setupVtk();

    connect(m_btnLoad, &QPushButton::clicked, this, &WeldSeamWidget::loadPointCloud);
    connect(m_btnClear, &QPushButton::clicked, this, &WeldSeamWidget::clearFit);
    connect(m_btnCopy, &QPushButton::clicked, this, &WeldSeamWidget::copyResult);
}

WeldSeamWidget::~WeldSeamWidget() {
    if (m_startHandle) m_startHandle->Off();
    if (m_endHandle) m_endHandle->Off();
}

void WeldSeamWidget::setupVtk() {
    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderer->SetBackground(0.05, 0.08, 0.10);
    m_vtkWidget->renderWindow()->AddRenderer(m_renderer);

    vtkNew<WeldViewInteractorStyle> style;
    m_vtkWidget->interactor()->SetInteractorStyle(style);

    m_cloudPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkNew<vtkPolyDataMapper> cloudMapper;
    cloudMapper->SetInputData(m_cloudPolyData);
    cloudMapper->ScalarVisibilityOn();
    cloudMapper->SetScalarModeToUsePointData();
    m_cloudActor = vtkSmartPointer<vtkActor>::New();
    m_cloudActor->SetMapper(cloudMapper);
    m_cloudActor->GetProperty()->SetPointSize(2.0);
    m_renderer->AddActor(m_cloudActor);

    m_inlierPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkNew<vtkPolyDataMapper> inlierMapper;
    inlierMapper->SetInputData(m_inlierPolyData);
    m_inlierActor = vtkSmartPointer<vtkActor>::New();
    m_inlierActor->SetMapper(inlierMapper);
    m_inlierActor->GetProperty()->SetColor(1.0, 0.9, 0.05);
    m_inlierActor->GetProperty()->SetPointSize(5.0);
    m_inlierActor->SetVisibility(false);
    m_renderer->AddActor(m_inlierActor);

    m_lineSource = vtkSmartPointer<vtkLineSource>::New();
    vtkNew<vtkPolyDataMapper> lineMapper;
    lineMapper->SetInputConnection(m_lineSource->GetOutputPort());
    m_lineActor = vtkSmartPointer<vtkActor>::New();
    m_lineActor->SetMapper(lineMapper);
    m_lineActor->GetProperty()->SetColor(1.0, 0.05, 0.05);
    m_lineActor->GetProperty()->SetLineWidth(5.0);
    m_lineActor->SetVisibility(false);
    m_renderer->AddActor(m_lineActor);

    m_pickCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_pickCallback->SetClientData(this);
    m_pickCallback->SetCallback(&WeldSeamWidget::onMiddleButtonPress);
    m_vtkWidget->interactor()->AddObserver(vtkCommand::MiddleButtonPressEvent, m_pickCallback);

    m_startHandleCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_startHandleCallback->SetClientData(this);
    m_startHandleCallback->SetCallback(&WeldSeamWidget::onStartHandleMoved);
    m_endHandleCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_endHandleCallback->SetClientData(this);
    m_endHandleCallback->SetCallback(&WeldSeamWidget::onEndHandleMoved);
}

void WeldSeamWidget::loadPointCloud() {
    QString startDir = m_loadedPath.isEmpty() ? QStringLiteral("/home/lh/Desktop/vizum116/data")
                                              : QFileInfo(m_loadedPath).absolutePath();
    QString filePath = QFileDialog::getOpenFileName(this, "加载 PLY 点云", startDir, "PLY (*.ply)");
    if (filePath.isEmpty()) return;
    loadCloudFile(filePath);
}

void WeldSeamWidget::loadCloudFile(QString filePath) {
    if (filePath.isEmpty()) return;
    loadCloudFromPath(filePath);
}

void WeldSeamWidget::loadCloudFromPath(const QString& filePath) {
    vtkNew<vtkPLYReader> reader;
    reader->SetFileName(filePath.toLocal8Bit().constData());
    reader->Update();

    vtkPolyData* polyData = reader->GetOutput();
    vtkPoints* vtkPointsData = polyData ? polyData->GetPoints() : nullptr;
    if (!vtkPointsData || vtkPointsData->GetNumberOfPoints() <= 0) {
        appendLog(QStringLiteral("加载失败: %1").arg(filePath));
        return;
    }

    vtkDataArray* scalars = polyData->GetPointData() ? polyData->GetPointData()->GetScalars() : nullptr;
    CloudT::Ptr loaded(new CloudT);
    loaded->reserve(static_cast<std::size_t>(vtkPointsData->GetNumberOfPoints()));
    for (vtkIdType i = 0; i < vtkPointsData->GetNumberOfPoints(); ++i) {
        double xyz[3];
        vtkPointsData->GetPoint(i, xyz);

        PointT p;
        p.x = static_cast<float>(xyz[0]);
        p.y = static_cast<float>(xyz[1]);
        p.z = static_cast<float>(xyz[2]);
        p.a = 255;
        if (scalars && scalars->GetNumberOfComponents() >= 3) {
            double rgb[4] = {255.0, 255.0, 255.0, 255.0};
            scalars->GetTuple(i, rgb);
            p.r = static_cast<std::uint8_t>(std::max(0.0, std::min(255.0, rgb[0])));
            p.g = static_cast<std::uint8_t>(std::max(0.0, std::min(255.0, rgb[1])));
            p.b = static_cast<std::uint8_t>(std::max(0.0, std::min(255.0, rgb[2])));
        } else {
            p.r = p.g = p.b = 210;
        }
        loaded->push_back(p);
    }

    m_cloud = loaded;
    m_loadedPath = filePath;
    m_treeReady = false;
    m_kdtree.setInputCloud(m_cloud);
    m_treeReady = true;

    clearFit();
    updateCloudActor();
    m_renderer->ResetCamera();
    m_vtkWidget->renderWindow()->Render();

    m_fileLabel->setText(QStringLiteral("%1 点，显示 %2 点: %3")
                         .arg(m_cloud->size())
                         .arg(m_displayPointToCloudIndex.size())
                         .arg(QFileInfo(filePath).fileName()));
    appendLog(QStringLiteral("已加载点云: %1, 原始点数 %2, 显示点数 %3")
              .arg(filePath)
              .arg(m_cloud->size())
              .arg(m_displayPointToCloudIndex.size()));
}

void WeldSeamWidget::updateCloudActor() {
    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> vertices;
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetName("colors");
    colors->SetNumberOfComponents(3);

    m_displayPointToCloudIndex.clear();
    if (m_cloud->empty()) return;

    const std::size_t stride = std::max<std::size_t>(1, (m_cloud->size() + m_maxDisplayPoints - 1) / m_maxDisplayPoints);
    const std::size_t displayCount = (m_cloud->size() + stride - 1) / stride;
    m_displayPointToCloudIndex.reserve(displayCount);

    points->SetDataTypeToFloat();
    points->Allocate(static_cast<vtkIdType>(displayCount));
    vertices->AllocateEstimate(static_cast<vtkIdType>(displayCount), 1);
    colors->Allocate(static_cast<vtkIdType>(displayCount) * 3);

    for (std::size_t i = 0; i < m_cloud->size(); i += stride) {
        const auto& p = (*m_cloud)[i];
        vtkIdType id = points->InsertNextPoint(p.x, p.y, p.z);
        vertices->InsertNextCell(1);
        vertices->InsertCellPoint(id);
        unsigned char rgb[3] = {p.r, p.g, p.b};
        colors->InsertNextTypedTuple(rgb);
        m_displayPointToCloudIndex.push_back(static_cast<int>(i));
    }

    m_cloudPolyData->SetPoints(points);
    m_cloudPolyData->SetVerts(vertices);
    m_cloudPolyData->GetPointData()->SetScalars(colors);
    m_cloudPolyData->Modified();
}

void WeldSeamWidget::onMiddleButtonPress(vtkObject* caller, unsigned long, void* clientData, void*) {
    auto* self = static_cast<WeldSeamWidget*>(clientData);
    auto* interactor = vtkRenderWindowInteractor::SafeDownCast(caller);
    if (!self || !interactor || self->m_cloud->empty()) return;

    int* pos = interactor->GetEventPosition();
    vtkNew<vtkPointPicker> picker;
    picker->SetTolerance(0.02);
    int picked = picker->Pick(pos[0], pos[1], 0, self->m_renderer);
    vtkIdType pointId = picker->GetPointId();
    if (picked && pointId >= 0 &&
        pointId < static_cast<vtkIdType>(self->m_displayPointToCloudIndex.size())) {
        self->fitAtPointId(self->m_displayPointToCloudIndex[static_cast<std::size_t>(pointId)]);
    }
}

void WeldSeamWidget::fitAtPointId(vtkIdType pointId) {
    if (!m_treeReady || pointId < 0 || pointId >= static_cast<vtkIdType>(m_cloud->size())) return;

    const PointT& picked = (*m_cloud)[static_cast<std::size_t>(pointId)];
    std::vector<int> indices;
    std::vector<float> distances;
    const float radius = static_cast<float>(m_radiusSpin->value());
    int found = m_kdtree.radiusSearch(picked, radius, indices, distances);
    m_pickLabel->setText(QStringLiteral("选中点 #%1: [%2, %3, %4], 邻域点 %5")
                         .arg(pointId)
                         .arg(picked.x, 0, 'f', 2)
                         .arg(picked.y, 0, 'f', 2)
                         .arg(picked.z, 0, 'f', 2)
                         .arg(found));

    if (found < m_minPointsSpin->value()) {
        appendLog(QStringLiteral("邻域点不足: %1 < %2，请增大半径或降低最少点数")
                  .arg(found).arg(m_minPointsSpin->value()));
        return;
    }

    Eigen::Vector3f start;
    Eigen::Vector3f end;
    std::vector<int> inlierIndices;
    QString errorText;
    if (!fitLineFromIndices(indices, start, end, inlierIndices, errorText)) {
        appendLog(errorText);
        return;
    }

    showFitResult(start, end, inlierIndices.empty() ? indices : inlierIndices);
    appendLog(QStringLiteral("拟合成功: 邻域 %1 点, RANSAC内点 %2 点, 长度 %3 mm")
              .arg(found)
              .arg(inlierIndices.empty() ? 0 : static_cast<int>(inlierIndices.size()))
              .arg((end - start).norm(), 0, 'f', 2));
}

bool WeldSeamWidget::fitLineFromIndices(const std::vector<int>& indices, Eigen::Vector3f& start,
                                        Eigen::Vector3f& end, std::vector<int>& inlierIndices,
                                        QString& errorText) const {
    if (indices.size() < 2) {
        errorText = "拟合失败: 点数不足";
        return false;
    }

    CloudT::Ptr subset(new CloudT);
    subset->reserve(indices.size());
    for (int index : indices) {
        subset->push_back((*m_cloud)[static_cast<std::size_t>(index)]);
    }

    pcl::SampleConsensusModelLine<PointT>::Ptr model(new pcl::SampleConsensusModelLine<PointT>(subset));
    pcl::RandomSampleConsensus<PointT> ransac(model);
    ransac.setDistanceThreshold(m_ransacThresholdSpin->value());
    ransac.setMaxIterations(1000);

    std::vector<int> inliers;
    Eigen::VectorXf coeffs;
    if (ransac.computeModel()) {
        ransac.getInliers(inliers);
        ransac.getModelCoefficients(coeffs);
    }

    std::vector<int> endpointIndices;
    if (coeffs.size() >= 6 && inliers.size() >= 2) {
        endpointIndices.reserve(inliers.size());
        for (int localIndex : inliers) {
            endpointIndices.push_back(indices[static_cast<std::size_t>(localIndex)]);
        }
        inlierIndices = endpointIndices;
    } else {
        if (!fitLinePca(indices, start, end)) {
            errorText = "拟合失败: RANSAC 和 PCA 都无法得到有效直线";
            return false;
        }
        inlierIndices.clear();
        return true;
    }

    Eigen::Vector3f origin(coeffs[0], coeffs[1], coeffs[2]);
    Eigen::Vector3f direction(coeffs[3], coeffs[4], coeffs[5]);
    if (direction.norm() < std::numeric_limits<float>::epsilon()) {
        errorText = "拟合失败: 直线方向向量无效";
        return false;
    }
    direction.normalize();

    float tMin = std::numeric_limits<float>::max();
    float tMax = -std::numeric_limits<float>::max();
    for (int index : endpointIndices) {
        const auto& p = (*m_cloud)[static_cast<std::size_t>(index)];
        Eigen::Vector3f point(p.x, p.y, p.z);
        float t = (point - origin).dot(direction);
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }

    if (tMax <= tMin) {
        errorText = "拟合失败: 线段端点投影无效";
        return false;
    }

    start = origin + tMin * direction;
    end = origin + tMax * direction;
    return true;
}

bool WeldSeamWidget::fitLinePca(const std::vector<int>& indices, Eigen::Vector3f& start, Eigen::Vector3f& end) const {
    if (indices.size() < 2) return false;

    Eigen::Vector3f mean(0.f, 0.f, 0.f);
    for (int index : indices) {
        const auto& p = (*m_cloud)[static_cast<std::size_t>(index)];
        mean += Eigen::Vector3f(p.x, p.y, p.z);
    }
    mean /= static_cast<float>(indices.size());

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int index : indices) {
        const auto& p = (*m_cloud)[static_cast<std::size_t>(index)];
        Eigen::Vector3f d = Eigen::Vector3f(p.x, p.y, p.z) - mean;
        cov += d * d.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    if (solver.info() != Eigen::Success) return false;

    Eigen::Vector3f direction = solver.eigenvectors().col(2);
    if (direction.norm() < std::numeric_limits<float>::epsilon()) return false;
    direction.normalize();

    float tMin = std::numeric_limits<float>::max();
    float tMax = -std::numeric_limits<float>::max();
    for (int index : indices) {
        const auto& p = (*m_cloud)[static_cast<std::size_t>(index)];
        float t = (Eigen::Vector3f(p.x, p.y, p.z) - mean).dot(direction);
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
    }
    if (tMax <= tMin) return false;

    start = mean + tMin * direction;
    end = mean + tMax * direction;
    return true;
}

void WeldSeamWidget::showFitResult(const Eigen::Vector3f& start, const Eigen::Vector3f& end,
                                   const std::vector<int>& inliers) {
    m_startPoint = start;
    m_endPoint = end;
    m_hasFit = true;

    vtkNew<vtkPoints> points;
    vtkNew<vtkCellArray> vertices;
    points->SetDataTypeToFloat();
    points->Allocate(static_cast<vtkIdType>(inliers.size()));
    vertices->AllocateEstimate(static_cast<vtkIdType>(inliers.size()), 1);
    for (int index : inliers) {
        const auto& p = (*m_cloud)[static_cast<std::size_t>(index)];
        vtkIdType id = points->InsertNextPoint(p.x, p.y, p.z);
        vertices->InsertNextCell(1);
        vertices->InsertCellPoint(id);
    }
    m_inlierPolyData->SetPoints(points);
    m_inlierPolyData->SetVerts(vertices);
    m_inlierPolyData->Modified();
    m_inlierActor->SetVisibility(true);

    updateLineActor();
    updateHandleWidgets();
    updateResultText();
    m_vtkWidget->renderWindow()->Render();
}

void WeldSeamWidget::updateLineActor() {
    if (!m_hasFit) return;
    m_lineSource->SetPoint1(m_startPoint.x(), m_startPoint.y(), m_startPoint.z());
    m_lineSource->SetPoint2(m_endPoint.x(), m_endPoint.y(), m_endPoint.z());
    m_lineSource->Update();
    m_lineActor->SetVisibility(true);
}

void WeldSeamWidget::updateHandleWidgets() {
    if (!m_hasFit) return;
    double radius = std::max(2.0, static_cast<double>((m_endPoint - m_startPoint).norm()) * 0.025);

    if (!m_startHandle) {
        m_startHandle = vtkSmartPointer<vtkSphereWidget>::New();
        m_startHandle->SetInteractor(m_vtkWidget->interactor());
        m_startHandle->SetCurrentRenderer(m_renderer);
        m_startHandle->AddObserver(vtkCommand::InteractionEvent, m_startHandleCallback);
    }
    if (!m_endHandle) {
        m_endHandle = vtkSmartPointer<vtkSphereWidget>::New();
        m_endHandle->SetInteractor(m_vtkWidget->interactor());
        m_endHandle->SetCurrentRenderer(m_renderer);
        m_endHandle->AddObserver(vtkCommand::InteractionEvent, m_endHandleCallback);
    }

    m_startHandle->SetCenter(m_startPoint.x(), m_startPoint.y(), m_startPoint.z());
    m_startHandle->SetRadius(radius);
    m_startHandle->GetSphereProperty()->SetColor(0.0, 1.0, 0.0);
    m_startHandle->On();

    m_endHandle->SetCenter(m_endPoint.x(), m_endPoint.y(), m_endPoint.z());
    m_endHandle->SetRadius(radius);
    m_endHandle->GetSphereProperty()->SetColor(1.0, 0.25, 0.05);
    m_endHandle->On();
}

void WeldSeamWidget::onStartHandleMoved(vtkObject* caller, unsigned long, void* clientData, void*) {
    auto* self = static_cast<WeldSeamWidget*>(clientData);
    auto* widget = vtkSphereWidget::SafeDownCast(caller);
    if (!self || !widget) return;
    double center[3];
    widget->GetCenter(center);
    self->m_startPoint = Eigen::Vector3f(center[0], center[1], center[2]);
    self->updateLineActor();
    self->updateResultText();
    self->m_vtkWidget->renderWindow()->Render();
}

void WeldSeamWidget::onEndHandleMoved(vtkObject* caller, unsigned long, void* clientData, void*) {
    auto* self = static_cast<WeldSeamWidget*>(clientData);
    auto* widget = vtkSphereWidget::SafeDownCast(caller);
    if (!self || !widget) return;
    double center[3];
    widget->GetCenter(center);
    self->m_endPoint = Eigen::Vector3f(center[0], center[1], center[2]);
    self->updateLineActor();
    self->updateResultText();
    self->m_vtkWidget->renderWindow()->Render();
}

void WeldSeamWidget::updateResultText() {
    if (!m_hasFit) {
        m_resultText->setPlainText("暂无拟合结果");
        return;
    }

    const Eigen::Vector3f delta = m_endPoint - m_startPoint;
    const float length = delta.norm();
    QString text;
    text += "焊缝线段端点 Camera 坐标\n";
    text += QString("Start: %1, %2, %3\n")
                .arg(m_startPoint.x(), 0, 'f', 3)
                .arg(m_startPoint.y(), 0, 'f', 3)
                .arg(m_startPoint.z(), 0, 'f', 3);
    text += QString("End:   %1, %2, %3\n")
                .arg(m_endPoint.x(), 0, 'f', 3)
                .arg(m_endPoint.y(), 0, 'f', 3)
                .arg(m_endPoint.z(), 0, 'f', 3);
    text += QString("Length: %1 mm\n").arg(length, 0, 'f', 3);
    if (length > std::numeric_limits<float>::epsilon()) {
        Eigen::Vector3f dir = delta / length;
        text += QString("Direction: %1, %2, %3\n")
                    .arg(dir.x(), 0, 'f', 6)
                    .arg(dir.y(), 0, 'f', 6)
                    .arg(dir.z(), 0, 'f', 6);
    }
    text += "\n左键拖动绿色球体调整起点，左键拖动红色球体调整终点。";
    m_resultText->setPlainText(text);
    emit cameraLineChanged(m_startPoint.x(), m_startPoint.y(), m_startPoint.z(),
                           m_endPoint.x(), m_endPoint.y(), m_endPoint.z());
}

void WeldSeamWidget::clearFit() {
    m_hasFit = false;
    if (m_startHandle) m_startHandle->Off();
    if (m_endHandle) m_endHandle->Off();
    if (m_lineActor) m_lineActor->SetVisibility(false);
    if (m_inlierActor) m_inlierActor->SetVisibility(false);
    if (m_inlierPolyData) {
        m_inlierPolyData->Initialize();
        m_inlierPolyData->Modified();
    }
    updateResultText();
    if (m_vtkWidget) m_vtkWidget->renderWindow()->Render();
}

void WeldSeamWidget::copyResult() {
    QApplication::clipboard()->setText(m_resultText->toPlainText());
    appendLog("结果已复制到剪贴板");
}

void WeldSeamWidget::appendLog(const QString& msg) {
    m_resultText->appendPlainText(QStringLiteral("\n[%1] %2")
                                  .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
                                  .arg(msg));
}
