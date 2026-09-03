#ifndef MYLINE_HIK_STEREO_DEPTH_ENGINE_H
#define MYLINE_HIK_STEREO_DEPTH_ENGINE_H

#include "stereo/core/StereoRigCalibration.h"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace hik_stereo {

struct StereoDepthBand {
    double minimumDepthMm{0.0};
    double maximumDepthMm{0.0};
};

struct StereoDepthOptions {
    cv::Size processingSize{612, 512};
    double minimumDepthMm{350.0};
    double maximumDepthMm{3000.0};
    int blockSize{5};
    int uniquenessRatio{8};
    int speckleWindowSize{100};
    int speckleRange{2};
    int leftRightMaximumDifferencePx{1};
    int disparityMarginPx{16};
    int maximumNumDisparities{512};
    bool enableLeftRightCheck{true};
    bool enableClahe{false};
    double claheClipLimit{2.0};
    std::vector<StereoDepthBand> depthBands;
};

struct StereoDepthStatistics {
    std::uint64_t inputPixels{0};
    std::uint64_t validPixels{0};
    double validFraction{0.0};
    double medianDepthMm{0.0};
    double processingMs{0.0};
    int minimumDisparity{0};
    int numberOfDisparities{0};
    int bandCount{1};
    int totalBandDisparities{0};
};

struct StereoDepthResult {
    bool ok{false};
    std::string error;
    cv::Mat rectifiedLeft;
    cv::Mat rectifiedRight;
    cv::Mat disparityPx;
    cv::Mat xyzLeftMm;
    cv::Mat validMask;
    cv::Mat confidence;
    cv::Mat disparityPreviewBgr;
    cv::Mat depthPreviewBgr;
    StereoDepthStatistics statistics;
};

class StereoDepthEngine {
public:
    bool configure(const StereoRigCalibration& rig,
                   const StereoDepthOptions& options,
                   std::string* error = nullptr);

    bool compute(const cv::Mat& leftGray,
                 const cv::Mat& rightGray,
                 StereoDepthResult* result) const;

    bool configured() const { return configured_; }
    const StereoProcessingGeometry& geometry() const { return geometry_; }
    const cv::Mat& reprojectionMatrix() const { return reprojectionMatrix_; }
    int minimumDisparity() const { return minimumDisparity_; }
    int numberOfDisparities() const { return numberOfDisparities_; }

private:
    struct ConfiguredBand {
        double minimumDepthMm{0.0};
        double maximumDepthMm{0.0};
        cv::Rect validDisparityRoi;
        int minimumDisparity{0};
        int numberOfDisparities{0};
    };

    bool configured_{false};
    StereoDepthOptions options_;
    StereoProcessingGeometry geometry_;
    cv::Mat leftMapX_;
    cv::Mat leftMapY_;
    cv::Mat rightMapX_;
    cv::Mat rightMapY_;
    cv::Mat reprojectionMatrix_;
    std::vector<ConfiguredBand> bands_;
    int minimumDisparity_{0};
    int numberOfDisparities_{0};
    int totalBandDisparities_{0};
};

}  // namespace hik_stereo

#endif  // MYLINE_HIK_STEREO_DEPTH_ENGINE_H
