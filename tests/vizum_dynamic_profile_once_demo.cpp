#include "VZNL_Common.h"
#include "VZNL_DetectConfig.h"
#include "VZNL_DetectLaser.h"
#include "VZNL_DustCover.h"
#include "VZNL_EyeConfig.h"
#include "VZNL_Graphics.h"
#include "VZNL_RGBConfig.h"
#include "VZNL_SwingMotor.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

struct ImageCopy {
    std::vector<unsigned char> buffer;
    SVzNL2DPoint origin{};
    unsigned int width = 0;
    unsigned int height = 0;
    EVzNLImageType imageType = keVzNLImageType_UNKNOWN;
    unsigned char bitDepth = 0;
    unsigned char channels = 0;
};

struct RoiBackup {
    bool requested = false;
    bool haveOld = false;
    SVzNLROIRect oldLeft{};
    SVzNLROIRect oldRight{};
};

struct LaserPoint {
    int index = 0;
    SVzNL2DPoint left{};
    SVzNL2DPoint right{};
    SVzNL3DPoint xyz{};
    SVzNL2DPoint rgb{};
    bool hasRgb = false;
};

struct LaserLineResult {
    std::vector<LaserPoint> points;
    unsigned long long timestamp = 0;
    double totalOffset = 0.0;
    double step = 0.0;
};

struct FeaturePoint {
    std::string name;
    cv::Point2d fittedLeft{};
    int sourceIndex = -1;
    double nearestDistance = 0.0;
    LaserPoint point;
};

struct ImageCenterlinePoint {
    cv::Point2d pixel{};
    int row = 0;
    int peakX = 0;
    int intensity = 0;
    double widthPx = 0.0;
    int componentId = 0;
};

struct ImageCenterlineResult {
    bool ok = false;
    double threshold = 0.0;
    int rawCandidateRows = 0;
    int componentCount = 0;
    int keptComponentCount = 0;
    std::vector<ImageCenterlinePoint> points;
};

struct LineFitResult {
    bool ok = false;
    cv::Point2d point{};
    cv::Point2d direction{};
    std::vector<int> inlierIndices;
    std::vector<FeaturePoint> features;
};

struct CaptureContext {
    std::mutex mutex;
    std::condition_variable cv;
    LaserLineResult line;
    std::atomic<int> callbacks{0};
    std::atomic<int> nonEmptyCallbacks{0};
    std::atomic<int> lastType{0};
    std::atomic<int> lastPointCount{0};
    bool done = false;
};

std::string errString(int code) {
    char buf[256] = {0};
    VzNL_GetErrorInfo(code, buf);
    std::ostringstream out;
    out << "[" << code << "] " << buf;
    return out.str();
}

bool ensureDir(const std::string& dir) {
    if (dir.empty()) {
        return false;
    }

    std::string current;
    const bool absolute = dir[0] == '/';
    if (absolute) {
        current = "/";
    }

    std::size_t start = absolute ? 1 : 0;
    while (start <= dir.size()) {
        const std::size_t slash = dir.find('/', start);
        const std::string part = dir.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        if (!part.empty()) {
            if (!current.empty() && current[current.size() - 1] != '/') {
                current += '/';
            }
            current += part;
            if (::mkdir(current.c_str(), 0775) != 0 && errno != EEXIST) {
                std::cerr << "mkdir failed: " << current << " errno=" << errno << std::endl;
                return false;
            }
        }
        if (slash == std::string::npos) {
            break;
        }
        start = slash + 1;
    }
    return true;
}

std::string nowStamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

std::string trim(const std::string& value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool ipEquals(const char* sdkIp, const std::string& ip) {
    return ip.empty() || ip == sdkIp;
}

bool copyImage(const SVzNLImageData* src, ImageCopy* dst) {
    if (!src || !dst || !src->pBuffer || src->nWidth == 0 || src->nHeight == 0 || src->nBufferSize == 0) {
        return false;
    }
    dst->buffer.assign(src->pBuffer, src->pBuffer + src->nBufferSize);
    dst->origin = src->ptOriPos;
    dst->width = src->nWidth;
    dst->height = src->nHeight;
    dst->imageType = src->eImageType;
    dst->bitDepth = src->byBitDepth;
    dst->channels = src->nChannels;
    return true;
}

SVzNLImageData sdkImageView(ImageCopy& image) {
    SVzNLImageData view{};
    view.pBuffer = image.buffer.empty() ? nullptr : image.buffer.data();
    view.nBufferSize = static_cast<unsigned int>(image.buffer.size());
    view.ptOriPos = image.origin;
    view.nWidth = image.width;
    view.nHeight = image.height;
    view.eImageType = image.imageType;
    view.byBitDepth = image.bitDepth;
    view.nChannels = image.channels;
    return view;
}

bool saveVzImage(const std::string& path, ImageCopy image) {
    if (image.buffer.empty()) {
        return false;
    }
    SVzNLImageData view = sdkImageView(image);
    const int rc = VzNL_SaveImage(path.c_str(), &view);
    if (rc != 0) {
        std::cerr << "SaveImage failed: " << path << " " << errString(rc) << std::endl;
        return false;
    }
    return true;
}

cv::Mat bgrMatFromImage(const ImageCopy& image) {
    if (image.buffer.empty() || image.width == 0 || image.height == 0 || image.channels == 0) {
        return {};
    }

    const int width = static_cast<int>(image.width);
    const int height = static_cast<int>(image.height);
    const int channels = static_cast<int>(image.channels);
    const std::size_t expected = static_cast<std::size_t>(width) *
                                 static_cast<std::size_t>(height) *
                                 static_cast<std::size_t>(channels);
    if (image.buffer.size() < expected) {
        return {};
    }

    if (channels == 1) {
        cv::Mat gray(height, width, CV_8UC1, const_cast<unsigned char*>(image.buffer.data()));
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    if (channels == 3) {
        cv::Mat src(height, width, CV_8UC3, const_cast<unsigned char*>(image.buffer.data()));
        if (image.imageType == keVzNLImageType_RGB888) {
            cv::Mat bgr;
            cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);
            return bgr;
        }
        return src.clone();
    }

    if (channels == 4) {
        cv::Mat src(height, width, CV_8UC4, const_cast<unsigned char*>(image.buffer.data()));
        cv::Mat bgr;
        cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }

    return {};
}

ImageCopy imageCopyFromBgrMat(const cv::Mat& bgr) {
    ImageCopy image;
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        return image;
    }
    const cv::Mat contiguous = bgr.isContinuous() ? bgr : bgr.clone();
    image.width = static_cast<unsigned int>(bgr.cols);
    image.height = static_cast<unsigned int>(bgr.rows);
    image.imageType = keVzNLImageType_BGR888;
    image.bitDepth = 8;
    image.channels = 3;
    const std::size_t bytes = contiguous.total() * contiguous.elemSize();
    image.buffer.assign(contiguous.data, contiguous.data + bytes);
    return image;
}

cv::Mat grayMatFromImage(const ImageCopy& image) {
    cv::Mat bgr = bgrMatFromImage(image);
    if (bgr.empty()) {
        return {};
    }
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

double percentileGray(const cv::Mat& gray, double pct) {
    if (gray.empty() || gray.type() != CV_8UC1) {
        return 0.0;
    }
    pct = std::max(0.0, std::min(1.0, pct));
    int hist[256] = {0};
    for (int y = 0; y < gray.rows; ++y) {
        const unsigned char* row = gray.ptr<unsigned char>(y);
        for (int x = 0; x < gray.cols; ++x) {
            ++hist[row[x]];
        }
    }
    const int total = gray.rows * gray.cols;
    const int target = std::max(0, std::min(total - 1, static_cast<int>(std::lround(pct * (total - 1)))));
    int acc = 0;
    for (int i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc > target) {
            return static_cast<double>(i);
        }
    }
    return 255.0;
}

double pointLineDistance(const cv::Point2d& p, const cv::Point2d& p0, const cv::Point2d& dir);
double projectT(const cv::Point2d& p, const cv::Point2d& p0, const cv::Point2d& dir);

ImageCenterlineResult extractBrightCenterlineFromImage(const ImageCopy& image) {
    ImageCenterlineResult result;
    const cv::Mat gray = grayMatFromImage(image);
    if (gray.empty()) {
        return result;
    }

    cv::Scalar mean{};
    cv::Scalar stddev{};
    cv::meanStdDev(gray, mean, stddev);
    const double p995 = percentileGray(gray, 0.995);
    result.threshold = std::max(80.0, std::min(220.0, std::max(p995 + 55.0, mean[0] + stddev[0] * 3.5)));

    cv::Mat mask(gray.rows, gray.cols, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < gray.rows; ++y) {
        const unsigned char* row = gray.ptr<unsigned char>(y);
        unsigned char* maskRow = mask.ptr<unsigned char>(y);
        bool hasCandidate = false;
        long long rowSum = 0;
        int maxValue = 0;
        for (int x = 0; x < gray.cols; ++x) {
            const int value = row[x];
            rowSum += value;
            maxValue = std::max(maxValue, value);
        }
        const double rowMean = static_cast<double>(rowSum) / std::max(1, gray.cols);
        if (maxValue < result.threshold || maxValue - rowMean < 35.0) {
            continue;
        }
        for (int x = 0; x < gray.cols; ++x) {
            if (row[x] >= result.threshold) {
                maskRow[x] = 255;
                hasCandidate = true;
            }
        }
        if (hasCandidate) {
            ++result.rawCandidateRows;
        }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    result.componentCount = std::max(0, labelCount - 1);

    int primaryLabel = 0;
    int primaryHeight = 0;
    int primaryArea = 0;
    for (int label = 1; label < labelCount; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        if (area < 6 || height < 4) {
            continue;
        }
        const int score = height * 100 + area - width * 4;
        const int bestScore = primaryHeight * 100 + primaryArea -
                              (primaryLabel > 0 ? stats.at<int>(primaryLabel, cv::CC_STAT_WIDTH) * 4 : 0);
        if (primaryLabel == 0 || score > bestScore) {
            primaryLabel = label;
            primaryHeight = height;
            primaryArea = area;
        }
    }

    if (primaryLabel == 0) {
        return result;
    }

    const int minKeepHeight = std::max(40, static_cast<int>(std::lround(primaryHeight * 0.10)));
    std::vector<unsigned char> keep(static_cast<std::size_t>(labelCount), 0);
    for (int label = 1; label < labelCount; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        if (area < 6 || height < minKeepHeight) {
            continue;
        }
        const int maxLineLikeWidth = std::max(18, static_cast<int>(std::lround(height * 0.55)));
        if (width > maxLineLikeWidth) {
            continue;
        }
        keep[static_cast<std::size_t>(label)] = 1;
        ++result.keptComponentCount;
    }
    if (result.keptComponentCount == 0) {
        keep[static_cast<std::size_t>(primaryLabel)] = 1;
        result.keptComponentCount = 1;
    }

    std::map<int, double> lastCenterXByLabel;
    for (int y = 0; y < gray.rows; ++y) {
        const unsigned char* row = gray.ptr<unsigned char>(y);
        const int* labelRow = labels.ptr<int>(y);
        struct RowRun {
            int label = 0;
            int left = std::numeric_limits<int>::max();
            int right = -1;
            int peakX = 0;
            int peakValue = 0;
            double weightedX = 0.0;
            double weights = 0.0;
        };
        std::map<int, std::vector<RowRun>> rowRuns;
        int x = 0;
        while (x < gray.cols) {
            const int label = labelRow[x];
            if (label <= 0 ||
                label >= labelCount ||
                !keep[static_cast<std::size_t>(label)]) {
                ++x;
                continue;
            }
            RowRun run;
            run.label = label;
            while (x < gray.cols && labelRow[x] == label) {
                run.left = std::min(run.left, x);
                run.right = std::max(run.right, x);
                const int value = row[x];
                if (value > run.peakValue) {
                    run.peakValue = value;
                    run.peakX = x;
                }
                const double weight = static_cast<double>(value) - result.threshold + 1.0;
                if (weight > 0.0) {
                    run.weightedX += static_cast<double>(x) * weight;
                    run.weights += weight;
                }
                ++x;
            }
            if (run.weights > 0.0) {
                rowRuns[label].push_back(run);
            }
        }

        for (auto& item : rowRuns) {
            const int label = item.first;
            std::vector<RowRun>& runs = item.second;
            if (runs.empty()) {
                continue;
            }
            const auto previousIt = lastCenterXByLabel.find(label);
            const bool havePrevious = previousIt != lastCenterXByLabel.end();
            const double previousX = havePrevious ? previousIt->second : 0.0;
            auto bestIt = runs.begin();
            if (havePrevious) {
                bestIt = std::min_element(runs.begin(), runs.end(), [&](const RowRun& a, const RowRun& b) {
                    const double ax = a.weights > 0.0 ? a.weightedX / a.weights : (a.left + a.right) * 0.5;
                    const double bx = b.weights > 0.0 ? b.weightedX / b.weights : (b.left + b.right) * 0.5;
                    return std::fabs(ax - previousX) < std::fabs(bx - previousX);
                });
            } else {
                bestIt = std::max_element(runs.begin(), runs.end(), [](const RowRun& a, const RowRun& b) {
                    return a.peakValue < b.peakValue;
                });
            }

            RowRun selected = *bestIt;
            const int runWidth = selected.right - selected.left + 1;
            if (havePrevious && runWidth > 18) {
                const int halfWindow = 9;
                selected.left = std::max(selected.left, static_cast<int>(std::lround(previousX)) - halfWindow);
                selected.right = std::min(selected.right, static_cast<int>(std::lround(previousX)) + halfWindow);
                selected.peakX = selected.left;
                selected.peakValue = 0;
                selected.weightedX = 0.0;
                selected.weights = 0.0;
                for (int sx = selected.left; sx <= selected.right; ++sx) {
                    if (sx < 0 || sx >= gray.cols || labelRow[sx] != label) {
                        continue;
                    }
                    const int value = row[sx];
                    if (value > selected.peakValue) {
                        selected.peakValue = value;
                        selected.peakX = sx;
                    }
                    const double weight = static_cast<double>(value) - result.threshold + 1.0;
                    if (weight > 0.0) {
                        selected.weightedX += static_cast<double>(sx) * weight;
                        selected.weights += weight;
                    }
                }
                if (selected.weights <= 0.0) {
                    continue;
                }
            }

            ImageCenterlinePoint point;
            point.pixel = cv::Point2d(selected.weightedX / selected.weights, static_cast<double>(y));
            point.row = y;
            point.peakX = selected.peakX;
            point.intensity = selected.peakValue;
            point.widthPx = static_cast<double>(selected.right - selected.left + 1);
            point.componentId = label;
            result.points.push_back(point);
            lastCenterXByLabel[label] = point.pixel.x;
        }
    }

    std::sort(result.points.begin(), result.points.end(), [](const ImageCenterlinePoint& a,
                                                             const ImageCenterlinePoint& b) {
        if (a.row != b.row) {
            return a.row < b.row;
        }
        return a.pixel.x < b.pixel.x;
    });

    result.ok = result.points.size() >= 12;
    return result;
}

bool validLaserPoint(const LaserPoint& p) {
    return p.left.x >= 0 &&
           p.left.y >= 0 &&
           std::isfinite(p.xyz.x) &&
           std::isfinite(p.xyz.y) &&
           std::isfinite(p.xyz.z) &&
           std::fabs(p.xyz.z) > 1e-6;
}

double pointLineDistance(const cv::Point2d& p, const cv::Point2d& p0, const cv::Point2d& dir) {
    return std::fabs((p.x - p0.x) * dir.y - (p.y - p0.y) * dir.x);
}

double projectT(const cv::Point2d& p, const cv::Point2d& p0, const cv::Point2d& dir) {
    return (p.x - p0.x) * dir.x + (p.y - p0.y) * dir.y;
}

cv::Point2d leftPoint2d(const LaserPoint& p) {
    return cv::Point2d(static_cast<double>(p.left.x), static_cast<double>(p.left.y));
}

int nearestLaserPointIndex(const LaserLineResult& line, const std::vector<int>& candidates,
                           const cv::Point2d& fitted, double* bestDistance) {
    int best = -1;
    double bestDist = std::numeric_limits<double>::max();
    for (const int idx : candidates) {
        if (idx < 0 || idx >= static_cast<int>(line.points.size())) {
            continue;
        }
        const cv::Point2d p = leftPoint2d(line.points[static_cast<std::size_t>(idx)]);
        const double d = cv::norm(p - fitted);
        if (d < bestDist) {
            bestDist = d;
            best = idx;
        }
    }
    if (bestDistance) {
        *bestDistance = bestDist;
    }
    return best;
}

LineFitResult fitLaserLine2D(const LaserLineResult& line) {
    LineFitResult result;
    std::vector<int> validIndices;
    std::vector<cv::Point2f> samples;
    validIndices.reserve(line.points.size());
    samples.reserve(line.points.size());

    for (int i = 0; i < static_cast<int>(line.points.size()); ++i) {
        const LaserPoint& p = line.points[static_cast<std::size_t>(i)];
        if (!validLaserPoint(p)) {
            continue;
        }
        validIndices.push_back(i);
        samples.emplace_back(static_cast<float>(p.left.x), static_cast<float>(p.left.y));
    }

    if (samples.size() < 2) {
        return result;
    }

    cv::Vec4f initial{};
    cv::fitLine(samples, initial, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2d p0(initial[2], initial[3]);
    cv::Point2d dir(initial[0], initial[1]);
    const double norm = cv::norm(dir);
    if (norm <= 1e-9) {
        return result;
    }
    dir *= 1.0 / norm;

    std::vector<double> distances;
    distances.reserve(samples.size());
    for (const cv::Point2f& p : samples) {
        distances.push_back(pointLineDistance(cv::Point2d(p.x, p.y), p0, dir));
    }
    std::vector<double> sorted = distances;
    std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
    const double median = sorted[sorted.size() / 2];
    const double threshold = std::max(2.5, std::min(12.0, median * 3.0 + 1.0));

    std::vector<cv::Point2f> inlierSamples;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (distances[i] <= threshold) {
            result.inlierIndices.push_back(validIndices[i]);
            inlierSamples.push_back(samples[i]);
        }
    }
    if (inlierSamples.size() < 2) {
        result.inlierIndices = validIndices;
        inlierSamples = samples;
    }

    cv::Vec4f refined{};
    cv::fitLine(inlierSamples, refined, cv::DIST_L2, 0, 0.01, 0.01);
    result.point = cv::Point2d(refined[2], refined[3]);
    result.direction = cv::Point2d(refined[0], refined[1]);
    const double refinedNorm = cv::norm(result.direction);
    if (refinedNorm <= 1e-9) {
        return {};
    }
    result.direction *= 1.0 / refinedNorm;

    double minT = std::numeric_limits<double>::max();
    double maxT = -std::numeric_limits<double>::max();
    for (const int idx : result.inlierIndices) {
        const double t = projectT(leftPoint2d(line.points[static_cast<std::size_t>(idx)]),
                                  result.point, result.direction);
        minT = std::min(minT, t);
        maxT = std::max(maxT, t);
    }
    if (!std::isfinite(minT) || !std::isfinite(maxT) || maxT <= minT) {
        return {};
    }

    auto makeFeature = [&](const std::string& name, double t) {
        FeaturePoint feature;
        feature.name = name;
        feature.fittedLeft = result.point + result.direction * t;
        double distance = 0.0;
        const int nearest = nearestLaserPointIndex(line, result.inlierIndices, feature.fittedLeft, &distance);
        feature.sourceIndex = nearest;
        feature.nearestDistance = distance;
        if (nearest >= 0) {
            feature.point = line.points[static_cast<std::size_t>(nearest)];
        }
        return feature;
    };

    result.features.push_back(makeFeature("start", minT));
    result.features.push_back(makeFeature("mid", (minT + maxT) * 0.5));
    result.features.push_back(makeFeature("end", maxT));
    result.ok = true;
    return result;
}

cv::Point imagePointFromSdkPoint(const ImageCopy& image, const SVzNL2DPoint& p) {
    return cv::Point(p.x - image.origin.x, p.y - image.origin.y);
}

cv::Point imagePointFromFitted(const ImageCopy& image, const cv::Point2d& p) {
    return cv::Point(static_cast<int>(std::lround(p.x - image.origin.x)),
                     static_cast<int>(std::lround(p.y - image.origin.y)));
}

bool pointInside(const cv::Mat& image, const cv::Point& p) {
    return p.x >= 0 && p.y >= 0 && p.x < image.cols && p.y < image.rows;
}

cv::Point roundedPoint(const cv::Point2d& p) {
    return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
}

cv::Point clampLabelPoint(const cv::Mat& image, const cv::Point& p) {
    cv::Point out = p;
    out.x = std::max(0, std::min(image.cols - 1, out.x));
    out.y = std::max(14, std::min(image.rows - 1, out.y));
    return out;
}

bool drawBrightPointsOverlay(const std::string& path, const ImageCopy& source, const ImageCenterlineResult& centerline) {
    cv::Mat bgr = bgrMatFromImage(source);
    if (bgr.empty() || !centerline.ok) {
        return false;
    }

    const int pointStep = std::max(1, static_cast<int>(centerline.points.size() / 3500));
    for (std::size_t i = 0; i < centerline.points.size(); i += static_cast<std::size_t>(pointStep)) {
        const cv::Point p = roundedPoint(centerline.points[i].pixel);
        if (pointInside(bgr, p)) {
            cv::circle(bgr, p, 2, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
        }
    }

    return cv::imwrite(path, bgr);
}

bool writeBrightPointsCsv(const std::string& path,
                          const std::string& camera,
                          const ImageCenterlineResult& centerline) {
    std::ofstream out(path.c_str());
    if (!out) {
        return false;
    }
    out << "camera,row,x,y,peak_x,intensity,width_px,component_id,threshold,raw_candidate_rows,component_count,kept_component_count,point_count\n";
    out << std::fixed << std::setprecision(6);
    for (const ImageCenterlinePoint& point : centerline.points) {
        out << camera << ','
            << point.row << ','
            << point.pixel.x << ','
            << point.pixel.y << ','
            << point.peakX << ','
            << point.intensity << ','
            << point.widthPx << ','
            << point.componentId << ','
            << centerline.threshold << ','
            << centerline.rawCandidateRows << ','
            << centerline.componentCount << ','
            << centerline.keptComponentCount << ','
            << centerline.points.size() << '\n';
    }
    return true;
}

bool writeBrightPointsSummary(const std::string& path,
                              const ImageCopy& leftImage,
                              const ImageCopy& rightImage,
                              const ImageCenterlineResult& leftCenterline,
                              const ImageCenterlineResult& rightCenterline) {
    std::ofstream out(path.c_str());
    if (!out) {
        return false;
    }
    auto writeOne = [&](const std::string& camera, const ImageCopy& image, const ImageCenterlineResult& centerline) {
        out << camera << "_image=" << image.width << "x" << image.height << '\n';
        out << camera << "_centerline_ok=" << (centerline.ok ? 1 : 0) << '\n';
        out << camera << "_threshold=" << std::fixed << std::setprecision(3) << centerline.threshold << '\n';
        out << camera << "_raw_candidate_rows=" << centerline.rawCandidateRows << '\n';
        out << camera << "_component_count=" << centerline.componentCount << '\n';
        out << camera << "_kept_component_count=" << centerline.keptComponentCount << '\n';
        out << camera << "_point_count=" << centerline.points.size() << '\n';
        if (!centerline.points.empty()) {
            const ImageCenterlinePoint& first = centerline.points.front();
            const ImageCenterlinePoint& middle = centerline.points[centerline.points.size() / 2];
            const ImageCenterlinePoint& last = centerline.points.back();
            out << camera << "_first=(" << first.pixel.x << "," << first.pixel.y << ")\n";
            out << camera << "_middle=(" << middle.pixel.x << "," << middle.pixel.y << ")\n";
            out << camera << "_last=(" << last.pixel.x << "," << last.pixel.y << ")\n";
        }
    };
    writeOne("left", leftImage, leftCenterline);
    writeOne("right", rightImage, rightCenterline);
    return true;
}

bool runImageFitOnDirectory(const std::string& dir) {
    const std::string leftPath = dir + "/left.png";
    const std::string rightPath = dir + "/right.png";
    cv::Mat leftBgr = cv::imread(leftPath, cv::IMREAD_COLOR);
    cv::Mat rightBgr = cv::imread(rightPath, cv::IMREAD_COLOR);
    if (leftBgr.empty() || rightBgr.empty()) {
        std::cerr << "Failed to load left/right images from: " << dir << std::endl;
        return false;
    }

    ImageCopy leftImage = imageCopyFromBgrMat(leftBgr);
    ImageCopy rightImage = imageCopyFromBgrMat(rightBgr);
    const ImageCenterlineResult leftCenterline = extractBrightCenterlineFromImage(leftImage);
    const ImageCenterlineResult rightCenterline = extractBrightCenterlineFromImage(rightImage);

    const std::string leftOverlayPath = dir + "/left_bright_points.png";
    const std::string rightOverlayPath = dir + "/right_bright_points.png";
    const std::string leftCsvPath = dir + "/left_bright_points.csv";
    const std::string rightCsvPath = dir + "/right_bright_points.csv";
    const std::string summaryPath = dir + "/bright_points_summary.txt";

    const bool wroteLeft = drawBrightPointsOverlay(leftOverlayPath, leftImage, leftCenterline);
    const bool wroteRight = drawBrightPointsOverlay(rightOverlayPath, rightImage, rightCenterline);
    const bool wroteLeftCsv = writeBrightPointsCsv(leftCsvPath, "left", leftCenterline);
    const bool wroteRightCsv = writeBrightPointsCsv(rightCsvPath, "right", rightCenterline);
    const bool wroteSummary = writeBrightPointsSummary(summaryPath, leftImage, rightImage, leftCenterline, rightCenterline);

    std::cout << "Left bright point extraction: ok=" << (leftCenterline.ok ? 1 : 0)
              << " points=" << leftCenterline.points.size()
              << " threshold=" << std::fixed << std::setprecision(3) << leftCenterline.threshold << std::endl;
    std::cout << "Right bright point extraction: ok=" << (rightCenterline.ok ? 1 : 0)
              << " points=" << rightCenterline.points.size()
              << " threshold=" << std::fixed << std::setprecision(3) << rightCenterline.threshold << std::endl;
    if (wroteLeft) {
        std::cout << "Saved left bright points overlay: " << leftOverlayPath << std::endl;
    }
    if (wroteRight) {
        std::cout << "Saved right bright points overlay: " << rightOverlayPath << std::endl;
    }
    if (wroteLeftCsv) {
        std::cout << "Saved left bright points CSV: " << leftCsvPath << std::endl;
    }
    if (wroteRightCsv) {
        std::cout << "Saved right bright points CSV: " << rightCsvPath << std::endl;
    }
    if (wroteSummary) {
        std::cout << "Saved bright points summary: " << summaryPath << std::endl;
    }
    return leftCenterline.ok && rightCenterline.ok &&
           wroteLeft && wroteRight && wroteLeftCsv && wroteRightCsv && wroteSummary;
}

bool drawLeftFitOverlay(const std::string& path, const ImageCopy& leftImage,
                        const LaserLineResult& line, const LineFitResult& fit) {
    cv::Mat bgr = bgrMatFromImage(leftImage);
    if (bgr.empty() || !fit.ok) {
        return false;
    }

    const int step = std::max(1, static_cast<int>(fit.inlierIndices.size() / 2500));
    for (std::size_t i = 0; i < fit.inlierIndices.size(); i += static_cast<std::size_t>(step)) {
        const int idx = fit.inlierIndices[i];
        const cv::Point p = imagePointFromSdkPoint(leftImage, line.points[static_cast<std::size_t>(idx)].left);
        if (pointInside(bgr, p)) {
            cv::circle(bgr, p, 1, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
        }
    }

    if (fit.features.size() >= 2) {
        const cv::Point a = imagePointFromFitted(leftImage, fit.features.front().fittedLeft);
        const cv::Point b = imagePointFromFitted(leftImage, fit.features.back().fittedLeft);
        cv::line(bgr, a, b, cv::Scalar(0, 220, 0), 2, cv::LINE_AA);
    }

    for (const FeaturePoint& feature : fit.features) {
        const cv::Point fitted = imagePointFromFitted(leftImage, feature.fittedLeft);
        const cv::Point nearest = imagePointFromSdkPoint(leftImage, feature.point.left);
        if (pointInside(bgr, fitted)) {
            cv::circle(bgr, fitted, 7, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv::putText(bgr, feature.name, fitted + cv::Point(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        if (pointInside(bgr, nearest)) {
            cv::circle(bgr, nearest, 3, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
        }
    }

    return cv::imwrite(path, bgr);
}

bool drawRgbFeatureOverlay(const std::string& path, const ImageCopy& rgbImage,
                           const std::vector<FeaturePoint>& features) {
    cv::Mat bgr = bgrMatFromImage(rgbImage);
    if (bgr.empty()) {
        return false;
    }

    std::vector<cv::Point> projected;
    for (const FeaturePoint& feature : features) {
        if (!feature.point.hasRgb) {
            continue;
        }
        const cv::Point p = imagePointFromSdkPoint(rgbImage, feature.point.rgb);
        if (!pointInside(bgr, p)) {
            continue;
        }
        projected.push_back(p);
        cv::circle(bgr, p, 8, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::putText(bgr, feature.name, p + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    if (projected.size() >= 2) {
        for (std::size_t i = 1; i < projected.size(); ++i) {
            cv::line(bgr, projected[i - 1], projected[i], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
        }
    }

    return cv::imwrite(path, bgr);
}

bool writeFeatureCsv(const std::string& path, const LineFitResult& fit) {
    std::ofstream out(path.c_str());
    if (!out) {
        return false;
    }
    out << "name,fitted_left_x,fitted_left_y,nearest_idx,nearest_dist_px,left_x,left_y,right_x,right_y,x,y,z,rgb_ok,rgb_x,rgb_y\n";
    out << std::fixed << std::setprecision(6);
    for (const FeaturePoint& feature : fit.features) {
        out << feature.name << ','
            << feature.fittedLeft.x << ','
            << feature.fittedLeft.y << ','
            << feature.sourceIndex << ','
            << feature.nearestDistance << ','
            << feature.point.left.x << ','
            << feature.point.left.y << ','
            << feature.point.right.x << ','
            << feature.point.right.y << ','
            << feature.point.xyz.x << ','
            << feature.point.xyz.y << ','
            << feature.point.xyz.z << ','
            << (feature.point.hasRgb ? 1 : 0) << ','
            << feature.point.rgb.x << ','
            << feature.point.rgb.y << '\n';
    }
    return true;
}

bool runSelfTest() {
    LaserLineResult line;
    for (int i = 0; i < 80; ++i) {
        LaserPoint p;
        p.index = i;
        p.left.x = 20 + i * 5;
        p.left.y = 50 + i * 2 + ((i % 5) - 2);
        p.right.x = p.left.x - 8;
        p.right.y = p.left.y;
        p.xyz.x = static_cast<float>(i);
        p.xyz.y = static_cast<float>(i * 0.5);
        p.xyz.z = 700.0f + static_cast<float>(i);
        line.points.push_back(p);
    }
    for (int i = 0; i < 8; ++i) {
        LaserPoint p;
        p.index = 100 + i;
        p.left.x = 40 + i * 31;
        p.left.y = 400 - i * 17;
        p.right.x = p.left.x - 8;
        p.right.y = p.left.y;
        p.xyz.x = 1.0f;
        p.xyz.y = 2.0f;
        p.xyz.z = 700.0f;
        line.points.push_back(p);
    }

    const LineFitResult fit = fitLaserLine2D(line);
    if (!fit.ok || fit.features.size() != 3 || fit.inlierIndices.size() < 70) {
        std::cerr << "self-test fit failed" << std::endl;
        return false;
    }
    const double slope = std::fabs(fit.direction.x) < 1e-9
                             ? std::numeric_limits<double>::infinity()
                             : fit.direction.y / fit.direction.x;
    if (std::fabs(std::fabs(slope) - 0.4) > 0.08) {
        std::cerr << "self-test slope failed: " << slope << std::endl;
        return false;
    }

    cv::Mat synthetic(240, 320, CV_8UC3, cv::Scalar(12, 12, 12));
    for (int y = 12; y < synthetic.rows - 10; ++y) {
        const int x = static_cast<int>(std::lround(55.0 + y * 0.32));
        cv::line(synthetic, cv::Point(x - 1, y), cv::Point(x + 1, y),
                 cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    for (int y = 72; y < 104; ++y) {
        cv::line(synthetic, cv::Point(18, y), cv::Point(26, y),
                 cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    for (int y = 148; y < 166; ++y) {
        cv::circle(synthetic, cv::Point(255 - (y % 11), y), 2,
                   cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
    }
    const ImageCenterlineResult centerline = extractBrightCenterlineFromImage(imageCopyFromBgrMat(synthetic));
    if (!centerline.ok || centerline.points.size() < 180) {
        std::cerr << "self-test centerline extraction failed: points=" << centerline.points.size()
                  << " threshold=" << centerline.threshold
                  << " raw_rows=" << centerline.rawCandidateRows
                  << " components=" << centerline.componentCount
                  << " kept=" << centerline.keptComponentCount << std::endl;
        return false;
    }
    for (const ImageCenterlinePoint& point : centerline.points) {
        const double expectedX = 55.0 + point.pixel.y * 0.32;
        if (std::fabs(point.pixel.x - expectedX) > 3.5) {
            std::cerr << "self-test centerline point drifted: got x=" << point.pixel.x
                      << " expected around " << expectedX << std::endl;
            return false;
        }
    }

    std::cout << "self-test ok: sdk-fit inliers=" << fit.inlierIndices.size()
              << " centerline points=" << centerline.points.size() << std::endl;
    return true;
}

std::map<std::string, std::string> readIniFlat(const std::string& path) {
    std::ifstream in(path.c_str());
    std::map<std::string, std::string> values;
    if (!in) {
        return values;
    }

    std::string section;
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        values[section + "/" + trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    return values;
}

bool boolValue(const std::string& value) {
    const std::string v = trim(value);
    return v == "1" || v == "true" || v == "True" || v == "TRUE";
}

int intValue(const std::map<std::string, std::string>& values, const std::string& key, int fallback) {
    const auto it = values.find(key);
    if (it == values.end()) {
        return fallback;
    }
    return std::atoi(it->second.c_str());
}

bool hasKey(const std::map<std::string, std::string>& values, const std::string& key) {
    return values.find(key) != values.end();
}

bool roiFromIni(const std::map<std::string, std::string>& values,
                const std::string& prefix,
                SVzNLROIRect* roi) {
    if (!roi ||
        !hasKey(values, prefix + "_L") ||
        !hasKey(values, prefix + "_R") ||
        !hasKey(values, prefix + "_T") ||
        !hasKey(values, prefix + "_B")) {
        return false;
    }
    roi->left = intValue(values, prefix + "_L", 0);
    roi->right = intValue(values, prefix + "_R", 0);
    roi->top = intValue(values, prefix + "_T", 0);
    roi->bottom = intValue(values, prefix + "_B", 0);
    return roi->right > roi->left && roi->bottom > roi->top;
}

std::string roiString(const SVzNLROIRect& roi) {
    std::ostringstream out;
    out << "[" << roi.left << "," << roi.right << "," << roi.top << "," << roi.bottom << "]";
    return out.str();
}

RoiBackup setFullDetectRoiForTest(VZNLHANDLE h, bool enable) {
    RoiBackup backup;
    backup.requested = enable;
    if (!enable) {
        std::cout << "Full detect ROI test mode: disabled" << std::endl;
        return backup;
    }

    int rc = VzNL_GetConfigDetectROI(h, &backup.oldLeft, &backup.oldRight);
    if (rc == 0) {
        backup.haveOld = true;
        std::cout << "Saved current detect ROI before full test: L="
                  << roiString(backup.oldLeft) << " R=" << roiString(backup.oldRight) << std::endl;
    } else {
        std::cout << "GetConfigDetectROI(before full): " << errString(rc) << std::endl;
    }

    SVzVideoResolution resolution{};
    rc = VzNL_GetResolution(h, &resolution);
    if (rc != 0 || resolution.nFrameWidth == 0 || resolution.nFrameHeight == 0) {
        std::cout << "GetResolution for full ROI failed: " << errString(rc) << std::endl;
        return backup;
    }

    auto trySet = [&](int right, int bottom, const char* tag) -> bool {
        SVzNLROIRect left{0, right, 0, bottom};
        SVzNLROIRect rightRoi{0, right, 0, bottom};
        int setRc = VzNL_ConfigDetectROIWithFormat(h, &left, &rightRoi);
        std::cout << "ConfigDetectROIWithFormat(full " << tag << " L="
                  << roiString(left) << " R=" << roiString(rightRoi) << "): "
                  << errString(setRc) << std::endl;
        return setRc == 0;
    };

    const int width = static_cast<int>(resolution.nFrameWidth);
    const int height = static_cast<int>(resolution.nFrameHeight);
    if (!trySet(width, height, "exclusive-edge")) {
        trySet(std::max(0, width - 1), std::max(0, height - 1), "inclusive-edge");
    }

    SVzNLROIRect actualLeft{};
    SVzNLROIRect actualRight{};
    rc = VzNL_GetConfigDetectROI(h, &actualLeft, &actualRight);
    if (rc == 0) {
        std::cout << "Active detect ROI for this test: L="
                  << roiString(actualLeft) << " R=" << roiString(actualRight) << std::endl;
    } else {
        std::cout << "GetConfigDetectROI(after full): " << errString(rc) << std::endl;
    }
    return backup;
}

void restoreDetectRoi(VZNLHANDLE h, const RoiBackup& backup) {
    if (!backup.requested || !backup.haveOld) {
        return;
    }
    SVzNLROIRect left = backup.oldLeft;
    SVzNLROIRect right = backup.oldRight;
    const int rc = VzNL_ConfigDetectROIWithFormat(h, &left, &right);
    std::cout << "Restore detect ROI: L=" << roiString(left)
              << " R=" << roiString(right) << " " << errString(rc) << std::endl;
}

void applyVizumConfig(VZNLHANDLE h, const std::string& path) {
    if (path.empty()) {
        return;
    }
    const auto values = readIniFlat(path);
    if (values.empty()) {
        std::cout << "Config not loaded or empty: " << path << std::endl;
        return;
    }

    std::cout << "Loading Vizum config: " << path << std::endl;
    const std::string basic = "Device_0_BasicCfg/";
    const std::string work = "Device_0_WorkModeCfg/";
    const std::string swing = "Device_0_SwingCfg/";

    if (hasKey(values, basic + "bIsDynamicROIEnabel")) {
        const int rc = VzNL_ConfigDynamicROI(h, boolValue(values.at(basic + "bIsDynamicROIEnabel")) ? VzTrue : VzFalse);
        std::cout << "ConfigDynamicROI(config): " << errString(rc) << std::endl;
    }
    if (hasKey(values, basic + "bIsEnableCalibROI")) {
        const int rc = VzNL_EnableCalibROI(h, boolValue(values.at(basic + "bIsEnableCalibROI")) ? VzTrue : VzFalse);
        std::cout << "EnableCalibROI(config): " << errString(rc) << std::endl;
    }
    if (hasKey(values, basic + "nLaserLightBrightness")) {
        const int brightness = intValue(values, basic + "nLaserLightBrightness", 3);
        const int rc = VzNL_SetLaserLight(h, static_cast<unsigned int>(brightness));
        std::cout << "SetLaserLight(config basic=" << brightness << "): " << errString(rc) << std::endl;
    } else if (hasKey(values, swing + "nLaserLightBrightness")) {
        const int brightness = intValue(values, swing + "nLaserLightBrightness", 3);
        const int rc = VzNL_SetLaserLight(h, static_cast<unsigned int>(brightness));
        std::cout << "SetLaserLight(config swing=" << brightness << "): " << errString(rc) << std::endl;
    }

    SVzNLROIRect leftRoi{};
    SVzNLROIRect rightRoi{};
    if (roiFromIni(values, basic + "LeftROI", &leftRoi) &&
        roiFromIni(values, basic + "RightROI", &rightRoi)) {
        const int rc = VzNL_ConfigDetectROIWithFormat(h, &leftRoi, &rightRoi);
        std::cout << "ConfigDetectROIWithFormat(config L=["
                  << leftRoi.left << "," << leftRoi.right << "," << leftRoi.top << "," << leftRoi.bottom
                  << "] R=[" << rightRoi.left << "," << rightRoi.right << "," << rightRoi.top << "," << rightRoi.bottom
                  << "]): " << errString(rc) << std::endl;
    }

    if (hasKey(values, work + "eTriggerMode")) {
        const int mode = intValue(values, work + "eTriggerMode", keEyeTriggerMode_Master);
        const int rc = VzNL_SetTriggerMode(h, static_cast<EVzEyeTriggerMode>(mode));
        std::cout << "SetTriggerMode(config " << mode << "): " << errString(rc) << std::endl;
    }
    if (hasKey(values, work + "bEnableHWTrigger")) {
        const int rc = VzNL_EnableTriggerHwExtEn(h, boolValue(values.at(work + "bEnableHWTrigger")) ? VzTrue : VzFalse);
        std::cout << "EnableTriggerHwExtEn(config): " << errString(rc) << std::endl;
    }
    if (hasKey(values, work + "nTriggerDivCoe")) {
        const int div = intValue(values, work + "nTriggerDivCoe", 1);
        const int rc = VzNL_SetTriggerDivCoe(h, static_cast<unsigned int>(std::max(1, div)));
        std::cout << "SetTriggerDivCoe(config " << div << "): " << errString(rc) << std::endl;
    }
    if (hasKey(values, work + "bTriggerPolor")) {
        const int rc = VzNL_SetTriggerPolar(h, boolValue(values.at(work + "bTriggerPolor")) ? VzTrue : VzFalse);
        std::cout << "SetTriggerPolar(config): " << errString(rc) << std::endl;
    }
    if (hasKey(values, work + "eTriggerOutMode")) {
        const int mode = intValue(values, work + "eTriggerOutMode", keStrobeTriggerOutMode_None);
        const int rc = VzNL_SetStrobeTriggerOutMode(h, static_cast<EVzStrobeTriggerOutMode>(mode));
        std::cout << "SetStrobeTriggerOutMode(config " << mode << "): " << errString(rc) << std::endl;
    }
}

void onLaserLine(EVzResultDataType eType, SVzLaserLineData* pLine, void* pParam) {
    auto* ctx = static_cast<CaptureContext*>(pParam);
    if (!ctx || !pLine) {
        return;
    }

    ctx->callbacks.fetch_add(1);
    ctx->lastType.store(static_cast<int>(eType));
    ctx->lastPointCount.store(pLine->nPointCount);

    if (eType != keResultDataType_PointXYZRGBA ||
        pLine->nPointCount <= 0 ||
        !pLine->p2DPoint ||
        !pLine->p3DPoint) {
        return;
    }

    ctx->nonEmptyCallbacks.fetch_add(1);

    LaserLineResult line;
    line.timestamp = pLine->llTimeStamp;
    line.totalOffset = pLine->dTotleOffset;
    line.step = pLine->dStep;
    line.points.reserve(static_cast<std::size_t>(pLine->nPointCount));

    const auto* p2 = static_cast<const SVzNL2DLRPoint*>(pLine->p2DPoint);
    const auto* p3 = static_cast<const SVzNLPointXYZRGBA*>(pLine->p3DPoint);
    for (int i = 0; i < pLine->nPointCount; ++i) {
        LaserPoint point;
        point.index = i;
        point.left = p2[i].sLeft;
        point.right = p2[i].sRight;
        point.xyz.x = p3[i].x;
        point.xyz.y = p3[i].y;
        point.xyz.z = p3[i].z;
        line.points.push_back(point);
    }

    {
        std::lock_guard<std::mutex> lock(ctx->mutex);
        if (ctx->done) {
            return;
        }
        ctx->line = line;
        ctx->done = true;
    }
    ctx->cv.notify_one();
}

bool grabRgbImage(VZNLHANDLE h, ImageCopy* image) {
    if (!image) {
        return false;
    }
    if (VzNL_IsSupportRGBCamera(h, nullptr) != VzTrue) {
        std::cout << "RGB camera unsupported" << std::endl;
        return false;
    }

    int rc = 0;
    if (VzNL_IsEnableRGB(h, &rc) != VzTrue) {
        rc = VzNL_EnableRGB(h, VzTrue);
        std::cout << "EnableRGB(true): " << errString(rc) << std::endl;
        if (rc != 0) {
            return false;
        }
    }

    rc = VzNL_EnableRGBAWB(h, VzTrue);
    std::cout << "EnableRGBAWB(true): " << errString(rc) << std::endl;
    rc = VzNL_EnableRGBAutoExpose(h, VzTrue);
    std::cout << "EnableRGBAutoExpose(true): " << errString(rc) << std::endl;
    rc = VzNL_EnableOutputColor2D(h, VzTrue);
    std::cout << "EnableOutputColor2D(true): " << errString(rc) << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    SVzNLImageData* rgbImage = nullptr;
    rc = VzNL_GetRGBImage(h, &rgbImage);
    if (rc != 0 || !rgbImage) {
        const int triggerRc = VzNL_GenRGBSoftSignal(h);
        std::cout << "GenRGBSoftSignal: " << errString(triggerRc) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        rc = VzNL_GetRGBImage(h, &rgbImage);
    }
    std::cout << "GetRGBImage: " << errString(rc) << std::endl;

    const bool ok = (rc == 0 && copyImage(rgbImage, image));
    VzNL_ReleaseImage(&rgbImage);
    return ok;
}

void projectFeaturesToRgb(VZNLHANDLE h, std::vector<FeaturePoint>* features) {
    if (!features) {
        return;
    }
    for (FeaturePoint& feature : *features) {
        SVzNL3DPoint p3d = feature.point.xyz;
        SVzNL2DPoint p2d{};
        const int rc = VzNL_CalcColor2DFrom3D(h, p3d, &p2d);
        if (rc == 0 && p2d.x >= 0 && p2d.y >= 0) {
            feature.point.rgb = p2d;
            feature.point.hasRgb = true;
        } else {
            std::cout << "CalcColor2DFrom3D(" << feature.name << "): " << errString(rc) << std::endl;
        }
    }
}

bool searchDevices(std::vector<SVzNLEyeCBInfo>* devices) {
    devices->clear();
    bool retry = true;
    int loop = 0;
    while (retry && loop++ < 5) {
        retry = false;
        int rc = VzNL_ResearchDevice(keSearchDeviceFlag_EthLaserRobotEye);
        std::cout << "ResearchDevice: " << errString(rc) << std::endl;

        int count = 0;
        rc = VzNL_GetEyeCBDeviceInfo(nullptr, &count);
        std::cout << "GetEyeCBDeviceInfo count: " << errString(rc) << " count=" << count << std::endl;
        if (count <= 0) {
            continue;
        }

        devices->assign(static_cast<std::size_t>(count), SVzNLEyeCBInfo{});
        rc = VzNL_GetEyeCBDeviceInfo(devices->data(), &count);
        std::cout << "GetEyeCBDeviceInfo data: " << errString(rc) << " count=" << count << std::endl;
        devices->resize(static_cast<std::size_t>(count));

        for (auto& device : *devices) {
            std::cout << "  found ip=" << device.byServerIP
                      << " valid=" << static_cast<int>(device.bValidDevice)
                      << " name=" << device.szDeviceName << std::endl;
            if (device.bValidDevice) {
                continue;
            }

            SVzNLEthernetEyeConfigInfo eth{};
            rc = VzNL_GetEthernetEyeConfigInfo(&device, &eth);
            if (rc != 0) {
                std::cout << "  GetEthernetEyeConfigInfo: " << errString(rc) << std::endl;
                continue;
            }
            if (std::memcmp(eth.byLocalIP, eth.sNetCardInfo.byLocalIP, 4) == 0) {
                std::cout << "  skip bind: local IP matches NIC IP" << std::endl;
                continue;
            }

            rc = VzNL_BindEthernetEye(&device);
            std::cout << "  BindEthernetEye: " << errString(rc) << std::endl;
            if (rc == 0) {
                retry = true;
            }
        }
    }
    return !devices->empty();
}

SVzNLEyeCBInfo* findDevice(std::vector<SVzNLEyeCBInfo>& devices, const std::string& ip) {
    for (auto& device : devices) {
        if (device.bValidDevice && ipEquals(device.byServerIP, ip)) {
            return &device;
        }
    }
    for (auto& device : devices) {
        if (ipEquals(device.byServerIP, ip)) {
            return &device;
        }
    }
    return nullptr;
}

void configureRuntime(VZNLHANDLE h, int fps, int exposure, int gain) {
    VzNL_SetOutputROIImage(VzFalse);
    VzNL_EnableTransPicMode(h, VzTrue);
    VzNL_EnableCalibROI(h, VzFalse);
    VzNL_ConfigDynamicROI(h, VzFalse);
    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);

    int rc = VzNL_SetTriggerMode(h, keEyeTriggerMode_Master);
    std::cout << "SetTriggerMode(Master): " << errString(rc) << std::endl;
    rc = VzNL_IgnoreTriggerExtSignal(h, VzTrue);
    std::cout << "IgnoreTriggerExtSignal(true): " << errString(rc) << std::endl;
    rc = VzNL_EnableTriggerHwExtEn(h, VzFalse);
    std::cout << "EnableTriggerHwExtEn(false): " << errString(rc) << std::endl;

    unsigned int minFps = 0;
    unsigned int maxFps = 0;
    VzNL_QueryParamRange(h, keDeviceParamType_FrameRate, &minFps, &maxFps);
    if (fps <= 0 && maxFps > 0) {
        fps = static_cast<int>(maxFps);
    }
    if (fps > 0) {
        rc = VzNL_EnableFreeFrameRate(h, VzTrue);
        std::cout << "EnableFreeFrameRate(true): " << errString(rc) << std::endl;
        rc = VzNL_SetFrameRate(h, fps);
        std::cout << "SetFrameRate(" << fps << ") range=[" << minFps << "," << maxFps << "]: "
                  << errString(rc) << std::endl;
    }

    if (exposure >= 0) {
        rc = VzNL_ConfigEyeExpose(h, keVzNLExposeMode_Fix, static_cast<unsigned int>(exposure));
        std::cout << "ConfigEyeExpose(fix," << exposure << "): " << errString(rc) << std::endl;
    }
    if (gain >= 0) {
        rc = VzNL_SetCameraGain(h, keEyeSensorType_Left, static_cast<unsigned short>(gain));
        std::cout << "SetCameraGain(left," << gain << "): " << errString(rc) << std::endl;
        rc = VzNL_SetCameraGain(h, keEyeSensorType_Right, static_cast<unsigned short>(gain));
        std::cout << "SetCameraGain(right," << gain << "): " << errString(rc) << std::endl;
    }

    if (VzNL_IsSupportCaptureMode(h, &rc) == VzTrue) {
        rc = VzNL_SetCaptureMode(h, keCaptureMode_LR_Image);
        std::cout << "SetCaptureMode(LR): " << errString(rc) << std::endl;
    }

    if (VzNL_IsSupportSwingMotor(h, nullptr) == VzTrue) {
        rc = VzNL_EnableSwingMotor(h, VzFalse);
        std::cout << "EnableSwingMotor(false): " << errString(rc) << std::endl;
        rc = VzNL_EnableLaserLight(h, VzTrue);
        std::cout << "EnableLaserLight(true): " << errString(rc) << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void printUsage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [--ip 192.168.1.105] [--out data] "
              << "[--config config/VizumConfig/SavedEnvironmentConfigPara_zhongshiye_tuxiangxia_shinei5.ini] "
              << "[--fps 30] [--exposure 260] [--gain 1] [--wait-ms 5000] "
              << "[--full-roi|--no-full-roi] [--self-test]\n"
              << "       " << argv0 << " --fit-images-dir data/laser_feature_once_YYYYMMDD_HHMMSS\n"
              << "       --fps 0 uses the camera maximum frame rate.\n"
              << "       --wait-ms waits for one non-empty laser profile.\n"
              << "       --fit-images-dir extracts bright laser center points from left.png/right.png.\n"
              << "       --full-roi is enabled by default for this diagnostic demo; original ROI is restored before exit.\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string ip = "192.168.1.105";
    std::string outRoot = "data";
    std::string configPath = "config/VizumConfig/SavedEnvironmentConfigPara_zhongshiye_tuxiangxia_shinei5.ini";
    int fps = 30;
    int exposure = 260;
    int gain = 1;
    int waitMs = 5000;
    bool fullRoi = true;
    bool selfTest = false;
    std::string fitImagesDir;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            ip = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            outRoot = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = std::atoi(argv[++i]);
        } else if (arg == "--exposure" && i + 1 < argc) {
            exposure = std::atoi(argv[++i]);
        } else if (arg == "--gain" && i + 1 < argc) {
            gain = std::atoi(argv[++i]);
        } else if (arg == "--wait-ms" && i + 1 < argc) {
            waitMs = std::atoi(argv[++i]);
        } else if (arg == "--full-roi") {
            fullRoi = true;
        } else if (arg == "--no-full-roi") {
            fullRoi = false;
        } else if (arg == "--self-test") {
            selfTest = true;
        } else if (arg == "--fit-images-dir" && i + 1 < argc) {
            fitImagesDir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            printUsage(argv[0]);
            return 1;
        }
    }

    if (selfTest) {
        return runSelfTest() ? 0 : 11;
    }

    if (!fitImagesDir.empty()) {
        return runImageFitOnDirectory(fitImagesDir) ? 0 : 12;
    }

    if (!ensureDir(outRoot)) {
        return 2;
    }
    const std::string outDir = outRoot + "/laser_feature_once_" + nowStamp();
    if (!ensureDir(outDir)) {
        return 2;
    }
    std::cout << "Output dir: " << outDir << std::endl;

    SVzNLConfigParam cfg{};
    int rc = VzNL_Init(&cfg);
    std::cout << "VzNL_Init: " << errString(rc) << std::endl;
    if (rc != 0) {
        return 3;
    }
    VzNL_SetLogLevel(keNLLogLevel_Information, keNLLogType_File);

    std::vector<SVzNLEyeCBInfo> devices;
    if (!searchDevices(&devices)) {
        std::cerr << "No Vizum devices found." << std::endl;
        VzNL_Destroy();
        return 4;
    }

    SVzNLEyeCBInfo* picked = findDevice(devices, ip);
    if (!picked) {
        std::cerr << "Device IP not found: " << ip << std::endl;
        VzNL_Destroy();
        return 5;
    }

    std::cout << "Opening device ip=" << picked->byServerIP << std::endl;
    SVzNLOpenDeviceParam openParam{};
    int openRc = 0;
    VZNLHANDLE h = VzNL_OpenDevice(picked, &openParam, &openRc);
    std::cout << "OpenDevice: " << errString(openRc) << " handle=" << h << std::endl;
    if (openRc != 0 || !h) {
        VzNL_Destroy();
        return 6;
    }

    if (VzNL_IsSupportCoverCamera(h, nullptr) == VzTrue) {
        rc = VzNL_CoverCamera(h, VzFalse);
        std::cout << "CoverCamera(open): " << errString(rc) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }

    rc = VzNL_BeginDetectLaser(h);
    std::cout << "BeginDetectLaser: " << errString(rc) << std::endl;
    if (rc != 0) {
        VzNL_CloseDevice(h);
        VzNL_Destroy();
        return 7;
    }

    configureRuntime(h, fps, exposure, gain);
    applyVizumConfig(h, configPath);
    const RoiBackup roiBackup = setFullDetectRoiForTest(h, fullRoi);

    CaptureContext ctx;
    rc = VzNL_StartAutoDetectEx(h, keResultDataType_PointXYZRGBA,
                                keFlipType_None, &onLaserLine, &ctx);
    std::cout << "StartAutoDetectEx(PointXYZRGBA): " << errString(rc) << std::endl;
    if (rc != 0) {
        restoreDetectRoi(h, roiBackup);
        VzNL_EndDetectLaser(h);
        VzNL_CloseDevice(h);
        VzNL_Destroy();
        return 8;
    }

    bool gotLine = false;
    {
        std::unique_lock<std::mutex> lock(ctx.mutex);
        gotLine = ctx.cv.wait_for(lock, std::chrono::milliseconds(waitMs), [&ctx] {
            return ctx.done;
        });
    }

    rc = VzNL_StopAutoDetect(h);
    std::cout << "StopAutoDetect: " << errString(rc) << std::endl;

    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);
    SVzNLImageData* leftImage = nullptr;
    SVzNLImageData* rightImage = nullptr;
    const int imageRc = VzNL_GetEyeImage(h, &leftImage, &rightImage, 2000);
    std::cout << "GetEyeImage: " << errString(imageRc) << std::endl;
    ImageCopy leftCopy;
    ImageCopy rightCopy;
    if (imageRc == 0) {
        copyImage(leftImage, &leftCopy);
        copyImage(rightImage, &rightCopy);
    }
    VzNL_ReleaseImage(&leftImage);
    VzNL_ReleaseImage(&rightImage);

    ImageCopy rgbCopy;
    const bool gotRgb = grabRgbImage(h, &rgbCopy);

    LaserLineResult line;
    {
        std::lock_guard<std::mutex> lock(ctx.mutex);
        line = ctx.line;
    }
    const ImageCenterlineResult leftCenterline = extractBrightCenterlineFromImage(leftCopy);
    const ImageCenterlineResult rightCenterline = extractBrightCenterlineFromImage(rightCopy);

    restoreDetectRoi(h, roiBackup);
    VzNL_EndDetectLaser(h);
    VzNL_CloseDevice(h);
    VzNL_Destroy();

    const std::string leftPath = outDir + "/left.png";
    const std::string rightPath = outDir + "/right.png";
    const std::string rgbPath = outDir + "/rgb.png";
    const std::string leftBrightPointsPath = outDir + "/left_bright_points.png";
    const std::string rightBrightPointsPath = outDir + "/right_bright_points.png";
    const std::string leftBrightCsvPath = outDir + "/left_bright_points.csv";
    const std::string rightBrightCsvPath = outDir + "/right_bright_points.csv";
    const std::string brightSummaryPath = outDir + "/bright_points_summary.txt";
    const bool savedLeft = saveVzImage(leftPath, leftCopy);
    const bool savedRight = saveVzImage(rightPath, rightCopy);
    const bool savedRgb = gotRgb && saveVzImage(rgbPath, rgbCopy);
    const bool savedLeftBrightPoints = leftCenterline.ok && drawBrightPointsOverlay(leftBrightPointsPath, leftCopy, leftCenterline);
    const bool savedRightBrightPoints = rightCenterline.ok && drawBrightPointsOverlay(rightBrightPointsPath, rightCopy, rightCenterline);
    const bool savedLeftBrightCsv = writeBrightPointsCsv(leftBrightCsvPath, "left", leftCenterline);
    const bool savedRightBrightCsv = writeBrightPointsCsv(rightBrightCsvPath, "right", rightCenterline);
    const bool savedBrightSummary = writeBrightPointsSummary(brightSummaryPath, leftCopy, rightCopy, leftCenterline, rightCenterline);

    std::cout << "Saved left image: " << leftPath << std::endl;
    std::cout << "Saved right image: " << rightPath << std::endl;
    if (savedRgb) {
        std::cout << "Saved RGB image: " << rgbPath << std::endl;
    }
    if (savedLeftBrightPoints) {
        std::cout << "Saved left bright points overlay: " << leftBrightPointsPath << std::endl;
    }
    if (savedRightBrightPoints) {
        std::cout << "Saved right bright points overlay: " << rightBrightPointsPath << std::endl;
    }
    if (savedLeftBrightCsv) {
        std::cout << "Saved left bright points CSV: " << leftBrightCsvPath << std::endl;
    }
    if (savedRightBrightCsv) {
        std::cout << "Saved right bright points CSV: " << rightBrightCsvPath << std::endl;
    }
    if (savedBrightSummary) {
        std::cout << "Saved bright points summary: " << brightSummaryPath << std::endl;
    }
    std::cout << "Callback stats: total=" << ctx.callbacks.load()
              << " non_empty=" << ctx.nonEmptyCallbacks.load()
              << " last_type=" << ctx.lastType.load()
              << " last_point_count=" << ctx.lastPointCount.load() << std::endl;
    std::cout << "SDK laser point count: " << line.points.size() << std::endl;
    std::cout << "Left bright point extraction: ok=" << (leftCenterline.ok ? 1 : 0)
              << " points=" << leftCenterline.points.size()
              << " threshold=" << std::fixed << std::setprecision(3) << leftCenterline.threshold << std::endl;
    std::cout << "Right bright point extraction: ok=" << (rightCenterline.ok ? 1 : 0)
              << " points=" << rightCenterline.points.size()
              << " threshold=" << std::fixed << std::setprecision(3) << rightCenterline.threshold << std::endl;
    if (!savedLeft || !savedRight) {
        std::cerr << "Failed to save left/right images." << std::endl;
        return 9;
    }
    if (!gotLine) {
        std::cerr << "SDK laser profile was not used for bright point extraction; image extraction ran independently." << std::endl;
    }
    if (!leftCenterline.ok || !rightCenterline.ok ||
        !savedLeftBrightPoints || !savedRightBrightPoints ||
        !savedLeftBrightCsv || !savedRightBrightCsv) {
        std::cerr << "Failed to extract bright laser points on left/right images." << std::endl;
        return 10;
    }
    return 0;
}
