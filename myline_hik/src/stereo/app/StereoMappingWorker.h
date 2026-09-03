#ifndef MYLINE_HIK_STEREO_MAPPING_WORKER_H
#define MYLINE_HIK_STEREO_MAPPING_WORKER_H

#include "stereo/core/StereoFrameSynchronizer.h"
#include "stereo/pipeline/StereoMappingPipeline.h"

#include <QImage>
#include <QObject>
#include <QString>

namespace hik_stereo {

struct StereoProcessingRequest {
    quint64 generation{0};
    StereoFramePair pair;
    cv::Matx44d baseFromFlange{cv::Matx44d::eye()};
    double robotGapMs{0.0};
    double robotSpeedMmS{0.0};
};

struct StereoUiFrame {
    bool ok{false};
    QString error;
    quint64 generation{0};
    QImage rectifiedLeft;
    QImage disparity;
    QImage depth;
    quint64 leftFrameId{0};
    quint64 rightFrameId{0};
    double pairSkewMs{0.0};
    double robotGapMs{0.0};
    double robotSpeedMmS{0.0};
    StereoDepthStatistics depthStatistics;
    OccupancyMapStatistics mapStatistics;
};

class StereoMappingWorker final : public QObject {
    Q_OBJECT

public:
    explicit StereoMappingWorker(QObject* parent = nullptr);

    bool configure(const StereoRigCalibration& rig,
                   const StereoDepthOptions& depthOptions,
                   const OccupancyMapOptions& mapOptions,
                   QString* error = nullptr);

public slots:
    void process(hik_stereo::StereoProcessingRequest request);
    void clearMap();
    void exportMap(QString directory,
                   double minimumGridHeightMm,
                   double maximumGridHeightMm);

signals:
    void frameProcessed(hik_stereo::StereoUiFrame frame);
    void mapCleared();
    void mapExported(bool ok, QString directory, QString description);

private:
    static QImage grayscaleImage(const cv::Mat& image);
    static QImage bgrImage(const cv::Mat& image);
    StereoMappingPipeline pipeline_;
    bool configured_{false};
};

}  // namespace hik_stereo

Q_DECLARE_METATYPE(hik_stereo::StereoProcessingRequest)
Q_DECLARE_METATYPE(hik_stereo::StereoUiFrame)

#endif  // MYLINE_HIK_STEREO_MAPPING_WORKER_H
