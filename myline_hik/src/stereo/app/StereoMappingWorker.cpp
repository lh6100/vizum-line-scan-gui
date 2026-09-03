#include "stereo/app/StereoMappingWorker.h"

#include <QDateTime>
#include <QDir>

namespace hik_stereo {

StereoMappingWorker::StereoMappingWorker(QObject* parent)
    : QObject(parent) {}

bool StereoMappingWorker::configure(
        const StereoRigCalibration& rig,
        const StereoDepthOptions& depthOptions,
        const OccupancyMapOptions& mapOptions,
        QString* error) {
    std::string detail;
    configured_ = pipeline_.configure(
        rig, depthOptions, mapOptions, &detail);
    if (!configured_ && error) *error = QString::fromStdString(detail);
    return configured_;
}

void StereoMappingWorker::process(StereoProcessingRequest request) {
    StereoUiFrame ui;
    ui.generation = request.generation;
    ui.leftFrameId = request.pair.left.frameId;
    ui.rightFrameId = request.pair.right.frameId;
    ui.pairSkewMs = request.pair.skewMs;
    ui.robotGapMs = request.robotGapMs;
    ui.robotSpeedMmS = request.robotSpeedMmS;
    if (!configured_ || !request.pair.left.image ||
        !request.pair.right.image) {
        ui.error = QStringLiteral("双目处理器未配置或图像缓冲为空。");
        emit frameProcessed(std::move(ui));
        return;
    }
    const hik_sync::ImageBuffer& leftBuffer = *request.pair.left.image;
    const hik_sync::ImageBuffer& rightBuffer = *request.pair.right.image;
    if (leftBuffer.width <= 0 || leftBuffer.height <= 0 ||
        rightBuffer.width <= 0 || rightBuffer.height <= 0 ||
        leftBuffer.stride < leftBuffer.width ||
        rightBuffer.stride < rightBuffer.width ||
        leftBuffer.bytes.size() < static_cast<std::size_t>(
            leftBuffer.stride) * leftBuffer.height ||
        rightBuffer.bytes.size() < static_cast<std::size_t>(
            rightBuffer.stride) * rightBuffer.height) {
        ui.error = QStringLiteral("双目图像缓冲尺寸/步长无效。");
        emit frameProcessed(std::move(ui));
        return;
    }
    const cv::Mat left(
        leftBuffer.height, leftBuffer.width, CV_8UC1,
        const_cast<std::uint8_t*>(leftBuffer.bytes.data()),
        static_cast<std::size_t>(leftBuffer.stride));
    const cv::Mat right(
        rightBuffer.height, rightBuffer.width, CV_8UC1,
        const_cast<std::uint8_t*>(rightBuffer.bytes.data()),
        static_cast<std::size_t>(rightBuffer.stride));
    StereoMappingResult result;
    if (!pipeline_.process(
            left, right, request.baseFromFlange, &result)) {
        ui.error = QString::fromStdString(result.error);
        emit frameProcessed(std::move(ui));
        return;
    }
    ui.rectifiedLeft = grayscaleImage(result.depth.rectifiedLeft);
    ui.disparity = bgrImage(result.depth.disparityPreviewBgr);
    ui.depth = bgrImage(result.depth.depthPreviewBgr);
    ui.depthStatistics = result.depth.statistics;
    ui.mapStatistics = result.map;
    ui.ok = true;
    emit frameProcessed(std::move(ui));
}

void StereoMappingWorker::clearMap() {
    pipeline_.clearMap();
    emit mapCleared();
}

void StereoMappingWorker::exportMap(
        QString directory,
        double minimumGridHeightMm,
        double maximumGridHeightMm) {
    if (!configured_) {
        emit mapExported(false, directory,
                         QStringLiteral("双目处理器尚未配置。"));
        return;
    }
    QDir path(directory);
    if (!path.exists() && !QDir().mkpath(path.absolutePath())) {
        emit mapExported(false, directory,
                         QStringLiteral("无法创建地图输出目录。"));
        return;
    }
    std::string detail;
    const bool ok = pipeline_.saveMap(
        path.absoluteFilePath(QStringLiteral("occupied_voxels.ply"))
            .toLocal8Bit().constData(),
        path.absoluteFilePath(QStringLiteral("occupancy_grid.pgm"))
            .toLocal8Bit().constData(),
        path.absoluteFilePath(QStringLiteral("occupancy_grid.yaml"))
            .toLocal8Bit().constData(),
        minimumGridHeightMm, maximumGridHeightMm, &detail);
    emit mapExported(
        ok, path.absolutePath(),
        ok ? QStringLiteral("已导出 occupied_voxels.ply、occupancy_grid.pgm/yaml。")
           : QString::fromStdString(detail));
}

QImage StereoMappingWorker::grayscaleImage(const cv::Mat& image) {
    if (image.empty() || image.type() != CV_8UC1) return QImage();
    return QImage(image.data, image.cols, image.rows,
                  static_cast<int>(image.step),
                  QImage::Format_Grayscale8).copy();
}

QImage StereoMappingWorker::bgrImage(const cv::Mat& image) {
    if (image.empty() || image.type() != CV_8UC3) return QImage();
    QImage result(image.data, image.cols, image.rows,
                  static_cast<int>(image.step), QImage::Format_RGB888);
    return result.rgbSwapped().copy();
}

}  // namespace hik_stereo
