#ifndef MYLINE_HIK_HIK_CALIBRATION_CORE_H
#define MYLINE_HIK_HIK_CALIBRATION_CORE_H

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace hik_calibration {

// All geometric lengths in this API are millimetres unless a field explicitly
// says otherwise.  Pixel coordinates use the original (distorted) image.
struct BoardSpec {
    int squaresX;
    int squaresY;
    double squareLengthMm;
    double markerLengthMm;
    int dictionaryId;

    BoardSpec();

    int charucoCornerCount() const;
    double widthMm() const;
    double heightMm() const;
};

bool validateBoardSpec(const BoardSpec& spec, std::string* error = 0);
std::string dictionaryName(int dictionaryId);

struct ErrorMetrics {
    double mean;
    double rms;
    double p95;
    double maximum;

    ErrorMetrics();
};

struct CharucoQuality {
    bool accepted;
    int markerCount;
    int cornerCount;
    double hullAreaRatio;
    double minimumBorderDistancePx;
    double meanIntensity;
    double saturationRatio;
    double laplacianVariance;
    std::string rejectReason;

    CharucoQuality();
};

struct DetectionOptions {
    int minMarkers;
    int minCorners;
    double minHullAreaRatio;
    double minBorderDistancePx;
    int saturationThreshold;
    double maxSaturationRatio;
    double minLaplacianVariance;
    bool refineMarkerCorners;

    DetectionOptions();
};

struct CharucoObservation {
    std::string sampleId;
    cv::Size imageSize;
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    CharucoQuality quality;
};

struct CharucoDetectionResult {
    bool ok;
    std::string error;
    CharucoObservation observation;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;

    CharucoDetectionResult();
};

// cameraMatrix/distCoeffs may be empty during the first intrinsic-calibration
// pass.  When supplied, OpenCV uses them while interpolating ChArUco corners.
bool detectCharuco(const cv::Mat& image,
                   const std::string& sampleId,
                   const BoardSpec& board,
                   const DetectionOptions& options,
                   CharucoDetectionResult* result,
                   const cv::Mat& cameraMatrix = cv::Mat(),
                   const cv::Mat& distCoeffs = cv::Mat());

struct IntrinsicCalibrationOptions {
    int minViews;
    int maxOutlierRounds;
    double madScale;
    double minOutlierThresholdPx;
    double hardMaxViewRmsPx;
    bool requireAcceptedDetection;
    int calibrationFlags;
    int maxIterations;
    double epsilon;

    IntrinsicCalibrationOptions();
};

struct IntrinsicViewResult {
    std::string sampleId;
    bool accepted;
    std::string rejectReason;
    int cornerCount;
    ErrorMetrics reprojection;
    cv::Vec3d rvec;
    cv::Vec3d tvec;

    IntrinsicViewResult();
};

struct IntrinsicCalibrationResult {
    bool ok;
    std::string error;
    BoardSpec board;
    cv::Size imageSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat intrinsicStdDeviations;
    double calibrationRmsPx;
    int inputViewCount;
    int acceptedViewCount;
    int rejectedViewCount;
    std::vector<int> acceptedObservationIndices;
    std::vector<IntrinsicViewResult> views;

    IntrinsicCalibrationResult();
};

// Calibrates a standard OpenCV pinhole/plumb-bob model.  After every fit,
// per-view RMS values are evaluated and robust MAD outliers are removed.
bool calibrateIntrinsics(const std::vector<CharucoObservation>& observations,
                         const BoardSpec& board,
                         const IntrinsicCalibrationOptions& options,
                         IntrinsicCalibrationResult* result);

struct BoardPoseOptions {
    int minCorners;
    double maxRmsErrorPx;
    double maxPointErrorPx;
    bool refineLm;

    BoardPoseOptions();
};

struct BoardPoseResult {
    bool ok;
    std::string error;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    cv::Matx33d rotation;
    cv::Matx44d cameraFromBoard;
    ErrorMetrics reprojection;

    BoardPoseResult();
};

// Returns T_camera_board (board coordinates transformed into camera optical
// coordinates).  A planar IPPE solution is refined with LM when requested.
bool estimateBoardPose(const CharucoObservation& observation,
                       const BoardSpec& board,
                       const cv::Mat& cameraMatrix,
                       const cv::Mat& distCoeffs,
                       const BoardPoseOptions& options,
                       BoardPoseResult* result);

struct StripeExtractionOptions {
    int minimumDifference;
    double thresholdStddevScale;
    int halfWindow;
    int minPointCount;
    int maxGapRows;
    double maxWidthPx;
    double minConfidence;
    double continuityPenalty;

    StripeExtractionOptions();
};

struct StripePoint {
    cv::Point2d pixel;
    int row;
    int peakX;
    double peakDifference;
    double widthPx;
    double confidence;

    StripePoint();
};

// Extracts one continuous, predominantly vertical laser centerline from an
// 8-bit laser-on minus laser-off difference image. Pixel x is sub-pixel.
bool extractVerticalLaserStripe(const cv::Mat& differenceImage,
                                const StripeExtractionOptions& options,
                                std::vector<StripePoint>* points,
                                std::string* error = 0);

// Extracts one continuous, predominantly horizontal laser centerline from an
// 8-bit laser-on minus laser-off difference image. Pixel y is sub-pixel.
bool extractHorizontalLaserStripe(const cv::Mat& differenceImage,
                                  const StripeExtractionOptions& options,
                                  std::vector<StripePoint>* points,
                                  std::string* error = 0);

// Tries both image directions and returns the centerline with more accepted
// points. This keeps calibration and reconstruction independent of laser roll.
bool extractLaserStripe(const cv::Mat& differenceImage,
                        const StripeExtractionOptions& options,
                        std::vector<StripePoint>* points,
                        std::string* error = 0);

struct PairMotionMetrics {
    int commonCornerCount;
    ErrorMetrics cornerShiftPx;
    double poseTranslationDeltaMm;
    double poseRotationDeltaDeg;

    PairMotionMetrics();
};

struct LaserPairOptions {
    DetectionOptions detection;
    BoardPoseOptions pose;
    StripeExtractionOptions stripe;
    int minCommonCorners;
    double maxMeanCornerShiftPx;
    double maxP95CornerShiftPx;
    double maxPoseTranslationDeltaMm;
    double maxPoseRotationDeltaDeg;
    double boardBoundsMarginMm;
    int minIntersectionPoints;

    LaserPairOptions();
};

struct LaserCalibrationPairResult {
    bool ok;
    std::string error;
    std::string sampleId;
    CharucoDetectionResult laserOffDetection;
    CharucoDetectionResult laserOnDetection;
    BoardPoseResult laserOffPose;
    BoardPoseResult laserOnPose;
    PairMotionMetrics motion;
    cv::Mat differenceImage;
    std::vector<StripePoint> stripe;
    std::vector<cv::Point3d> cameraPointsMm;
    std::vector<cv::Point3d> boardPointsMm;

    LaserCalibrationPairResult();
};

// Validates that laser-off/on images are a static pair, extracts a vertical
// sub-pixel stripe from on-off, and intersects every accepted ray with the
// ChArUco board plane obtained from the laser-off image.
bool processLaserCalibrationPair(const cv::Mat& laserOffImage,
                                 const cv::Mat& laserOnImage,
                                 const std::string& sampleId,
                                 const BoardSpec& board,
                                 const cv::Mat& cameraMatrix,
                                 const cv::Mat& distCoeffs,
                                 const LaserPairOptions& options,
                                 LaserCalibrationPairResult* result);

struct LaserPlaneSample {
    std::string sampleId;
    std::vector<cv::Point3d> cameraPointsMm;
};

LaserPlaneSample planeSampleFromPair(const LaserCalibrationPairResult& pair);

struct PlaneFitOptions {
    int minPoseCount;
    int minPointsPerPose;
    int maxPointsPerPose;
    int ransacIterations;
    double ransacThresholdMm;
    double minInlierRatio;
    std::uint32_t randomSeed;

    PlaneFitOptions();
};

struct LaserPlane {
    cv::Vec3d normal;
    double dMm;

    LaserPlane();
};

struct PlanePoseStatistics {
    std::string sampleId;
    int pointCount;
    int inlierCount;
    ErrorMetrics distanceMm;

    PlanePoseStatistics();
};

struct LaserPlaneFitResult {
    bool ok;
    std::string error;
    LaserPlane plane;
    int poseCount;
    int pointCount;
    int inlierCount;
    double inlierRatio;
    ErrorMetrics inlierDistanceMm;
    std::vector<PlanePoseStatistics> poses;

    LaserPlaneFitResult();
};

// Balanced RANSAC samples one point from each of three distinct poses.  The
// winning inliers are refined with a per-pose-balanced weighted SVD.
bool fitLaserPlane(const std::vector<LaserPlaneSample>& samples,
                   const PlaneFitOptions& options,
                   LaserPlaneFitResult* result);

struct IntrinsicsYamlMetadata {
    std::string cameraName;
    std::string cameraModel;
    std::string cameraSerial;
    std::string frameId;
    std::string pixelFormat;
    std::string printedPatternSha256;
    std::string generatedAt;

    IntrinsicsYamlMetadata();
};

struct LaserPlaneYamlMetadata {
    std::string cameraFrame;
    std::string intrinsicsFile;
    std::string intrinsicsSha256;
    std::string printedPatternSha256;
    std::string datasetManifest;
    std::string datasetManifestSha256;
    std::string generatedAt;
    double validCameraZMinMm;
    double validCameraZMaxMm;

    LaserPlaneYamlMetadata();
};

struct StaticProfileOptions {
    StripeExtractionOptions stripe;
    DetectionOptions boardDetection;
    BoardPoseOptions boardPose;
    int minReconstructedPoints;
    double rayPlaneDenominatorEpsilon;
    double minimumDepthMm;
    double maximumDepthMm;
    double boardBoundsMarginMm;
    int minBoardValidationPoints;
    double maxLineRmsMm;
    double maxBoardPlaneRmsMm;

    StaticProfileOptions();
};

struct SingleFrameProfileOptions {
    StaticProfileOptions reconstruction;
    int backgroundKernelWidth;
    int backgroundKernelHeight;
    int minimumRawIntensity;

    SingleFrameProfileOptions();
};

struct StaticProfilePoint {
    StripePoint stripe;
    cv::Point3d cameraPointMm;
    double boardPlaneDistanceMm;
    bool insideBoard;

    StaticProfilePoint();
};

struct StaticProfileResult {
    bool ok;
    std::string error;
    std::string sampleId;
    cv::Mat differenceImage;
    std::vector<StripePoint> stripe;
    std::vector<StaticProfilePoint> points;
    int rejectedParallelRayCount;
    int rejectedBehindCameraCount;
    int rejectedDepthCount;
    double minimumDepthMm;
    double maximumDepthMm;
    double meanConfidence;
    ErrorMetrics lineDistanceMm;
    bool lineQualityPassed;
    bool boardValidationAvailable;
    bool boardValidationPassed;
    std::string boardValidationMessage;
    CharucoDetectionResult boardDetection;
    BoardPoseResult boardPose;
    int boardValidationPointCount;
    ErrorMetrics boardPlaneDistanceMm;

    StaticProfileResult();
};

// Reconstructs a static 3-D profile in the camera optical frame. A ChArUco
// board is optional: when detected in laser-off, its independently estimated
// plane supplies a true flatness/accuracy residual. Without it, lineDistanceMm
// is only 3-D contour straightness and must not be reported as plane accuracy.
bool reconstructStaticProfile(const cv::Mat& laserOffImage,
                              const cv::Mat& laserOnImage,
                              const std::string& sampleId,
                              const IntrinsicCalibrationResult& intrinsics,
                              const LaserPlaneFitResult& laserPlane,
                              const BoardSpec& validationBoard,
                              const StaticProfileOptions& options,
                              StaticProfileResult* result);

// Constant-laser validation path. A horizontal morphological opening estimates
// the slowly varying scene background, leaving a narrow predominantly vertical
// bright ridge. The resulting response is passed through the same sub-pixel,
// undistortion and ray/laser-plane reconstruction used by paired acquisition.
bool reconstructSingleFrameProfile(const cv::Mat& laserOnImage,
                                   const std::string& sampleId,
                                   const IntrinsicCalibrationResult& intrinsics,
                                   const LaserPlaneFitResult& laserPlane,
                                   const SingleFrameProfileOptions& options,
                                   StaticProfileResult* result);

bool saveStaticProfilePly(const std::string& path,
                          const StaticProfileResult& profile,
                          const std::string& frameId,
                          std::string* error = 0);

bool saveStaticProfileCsv(const std::string& path,
                          const StaticProfileResult& profile,
                          std::string* error = 0);

// These functions intentionally write ordinary YAML mappings and one-line
// numeric arrays.  They do not emit OpenCV's %YAML:1.0 or !!opencv-matrix tags.
bool saveIntrinsicsYaml(const std::string& path,
                        const IntrinsicCalibrationResult& calibration,
                        const IntrinsicsYamlMetadata& metadata,
                        std::string* error = 0);

bool loadIntrinsicsYaml(const std::string& path,
                        IntrinsicCalibrationResult* calibration,
                        IntrinsicsYamlMetadata* metadata = 0,
                        std::string* error = 0);

bool saveLaserPlaneYaml(const std::string& path,
                        const LaserPlaneFitResult& fit,
                        const BoardSpec& board,
                        const LaserPlaneYamlMetadata& metadata,
                        std::string* error = 0);

bool loadLaserPlaneYaml(const std::string& path,
                        LaserPlaneFitResult* fit,
                        BoardSpec* board = 0,
                        LaserPlaneYamlMetadata* metadata = 0,
                        std::string* error = 0);

}  // namespace hik_calibration

#endif  // MYLINE_HIK_HIK_CALIBRATION_CORE_H
