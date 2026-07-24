#include "HikScanCore.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace {

int gFailures = 0;

void fail(const std::string& message, int line) {
    ++gFailures;
    std::cerr << "FAIL line " << line << ": " << message << std::endl;
}

#define CHECK_TRUE(condition, message) \
    do {                                \
        if (!(condition)) {             \
            fail((message), __LINE__);  \
        }                               \
    } while (false)

bool nearlyEqual(double first, double second, double tolerance = 1.0e-12) {
    return std::isfinite(first) && std::isfinite(second) &&
           std::fabs(first - second) <= tolerance;
}

hik_scan::CloudPoint pointAt(double x,
                             double y,
                             double z,
                             int profile,
                             double confidence = 0.8) {
    hik_scan::CloudPoint point;
    point.basePointMm = cv::Point3d(x, y, z);
    point.cameraPointMm = point.basePointMm;
    point.profileIndex = profile;
    point.confidence = confidence;
    point.response = 100.0;
    point.pixelU = x;
    point.pixelV = y;
    return point;
}

void testFilterDisabledByDefault() {
    std::vector<hik_scan::CloudPoint> cloud;
    cloud.push_back(pointAt(100.0, 100.0, 100.0, 10));

    hik_scan::AdjacentProfileSupportOptions options;
    CHECK_TRUE(!options.enabled,
               "adjacent-profile filtering must be disabled by default");
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   cloud, options, &result, &error),
               std::string("disabled filter failed: ") + error);
    CHECK_TRUE(!result.applied && result.kept.size() == 1U &&
                   result.rejected.empty(),
               "disabled filter must preserve the complete input cloud");
    CHECK_TRUE(result.kept.front().qualityFlags ==
                   cloud.front().qualityFlags,
               "disabled filter must not mutate quality flags");
}

void testSparseIsolatedPointRejected() {
    std::vector<hik_scan::CloudPoint> cloud;
    cloud.push_back(pointAt(0.0, 0.0, 0.0, 9));
    cloud.push_back(pointAt(0.1, 0.0, 0.0, 10));
    cloud.push_back(pointAt(0.2, 0.0, 0.0, 11));
    cloud.push_back(pointAt(20.0, 20.0, 20.0, 10));

    hik_scan::AdjacentProfileSupportOptions options;
    options.enabled = true;
    options.radiusMm = 0.25;
    options.minimumSupportingProfiles = 1;
    options.maximumProfileGap = 1;
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   cloud, options, &result, &error),
               std::string("support filter failed: ") + error);
    CHECK_TRUE(result.kept.size() == 3U && result.rejected.size() == 1U,
               "three adjacent-profile points must remain and one isolated "
               "point must be rejected");
    CHECK_TRUE(result.statistics.insufficientSupportPointCount == 1U &&
                   result.statistics.invalidPointCount == 0U,
               "support rejection counters must distinguish isolation");
    CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                   result.rejected.front(),
                   hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT),
               "isolated point must retain an explicit rejection flag");
    for (std::size_t index = 0U; index < result.kept.size(); ++index) {
        CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                       result.kept[index],
                       hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED),
                   "kept points must record adjacent-profile support");
    }
}

void testDistinctProfilesAndMaximumGap() {
    std::vector<hik_scan::CloudPoint> oneNeighborProfile;
    oneNeighborProfile.push_back(pointAt(0.0, 0.0, 0.0, 10));
    oneNeighborProfile.push_back(pointAt(0.05, 0.0, 0.0, 9));
    oneNeighborProfile.push_back(pointAt(0.10, 0.0, 0.0, 9));

    hik_scan::AdjacentProfileSupportOptions options;
    options.enabled = true;
    options.radiusMm = 0.5;
    options.minimumSupportingProfiles = 2;
    options.maximumProfileGap = 1;
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   oneNeighborProfile, options, &result, &error),
               std::string("distinct-profile filter failed: ") + error);
    bool targetRejected = false;
    for (std::size_t index = 0U; index < result.rejected.size(); ++index) {
        if (result.rejected[index].profileIndex == 10) {
            targetRejected = true;
        }
    }
    CHECK_TRUE(targetRejected,
               "multiple points from one profile must count as one supporting "
               "profile, not multiple observations");

    std::vector<hik_scan::CloudPoint> enoughProfiles = oneNeighborProfile;
    enoughProfiles.push_back(pointAt(0.15, 0.0, 0.0, 11));
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   enoughProfiles, options, &result, &error),
               std::string("two-profile support filter failed: ") + error);
    bool targetKept = false;
    for (std::size_t index = 0U; index < result.kept.size(); ++index) {
        if (result.kept[index].profileIndex == 10) {
            targetKept = true;
        }
    }
    CHECK_TRUE(targetKept,
               "two distinct neighboring profiles must satisfy a two-profile "
               "support requirement");

    std::vector<hik_scan::CloudPoint> excessiveGap;
    excessiveGap.push_back(pointAt(0.0, 0.0, 0.0, 1));
    excessiveGap.push_back(pointAt(0.1, 0.0, 0.0, 4));
    options.minimumSupportingProfiles = 1;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   excessiveGap, options, &result, &error),
               std::string("maximum-gap filter failed: ") + error);
    CHECK_TRUE(result.kept.empty() && result.rejected.size() == 2U,
               "spatial neighbors outside maximumProfileGap must not provide "
               "support");
}

void testCoherentFalseSurfaceLimitation() {
    std::vector<hik_scan::CloudPoint> coherentFalseSurface;
    for (int profile = 20; profile <= 22; ++profile) {
        for (int sample = 0; sample < 5; ++sample) {
            coherentFalseSurface.push_back(pointAt(
                0.1 * static_cast<double>(profile - 20),
                0.1 * static_cast<double>(sample),
                500.0,
                profile));
        }
    }

    hik_scan::AdjacentProfileSupportOptions options;
    options.enabled = true;
    options.radiusMm = 0.16;
    options.minimumSupportingProfiles = 1;
    options.maximumProfileGap = 1;
    hik_scan::AdjacentProfileSupportResult result;
    std::string error;
    CHECK_TRUE(hik_scan::filterByAdjacentProfileSupport(
                   coherentFalseSurface, options, &result, &error),
               std::string("coherent-surface filter failed: ") + error);
    CHECK_TRUE(result.kept.size() == coherentFalseSurface.size() &&
                   result.rejected.empty(),
               "a coherent false surface repeated across profiles must pass "
               "this isolation-only filter; optical/temporal gates are still "
               "required");
}

void testConfidenceWeightedVoxelDownsample() {
    hik_scan::CloudPoint strong = pointAt(0.0, 0.0, 0.0, 3, 0.9);
    strong.response = 100.0;
    strong.pixelU = 10.0;
    strong.observationCount = 2U;
    strong.qualityFlags = hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED;

    hik_scan::CloudPoint weak = pointAt(0.8, 0.0, 0.0, 4, 0.1);
    weak.response = 0.0;
    weak.pixelU = 20.0;
    weak.observationCount = 3U;
    weak.qualityFlags =
        hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT;
    const std::vector<hik_scan::CloudPoint> cloud{strong, weak};

    const std::vector<hik_scan::CloudPoint> legacy =
        hik_scan::voxelDownsample(cloud, 1.0);
    CHECK_TRUE(legacy.size() == 1U &&
                   nearlyEqual(legacy.front().basePointMm.x, 0.4),
               "legacy voxel wrapper must retain equal-input averaging");

    hik_scan::VoxelDownsampleOptions options;
    options.voxelSizeMm = 1.0;
    options.confidenceWeighted = true;
    hik_scan::VoxelDownsampleStatistics statistics;
    const std::vector<hik_scan::CloudPoint> weighted =
        hik_scan::voxelDownsample(cloud, options, &statistics);
    CHECK_TRUE(weighted.size() == 1U &&
                   nearlyEqual(weighted.front().basePointMm.x, 0.08),
               "confidence weighting must move voxel geometry toward the "
               "high-confidence observation");
    CHECK_TRUE(nearlyEqual(weighted.front().response, 90.0) &&
                   nearlyEqual(weighted.front().pixelU, 11.0),
               "response and source pixel must use the same confidence weights");
    CHECK_TRUE(nearlyEqual(weighted.front().confidence, 0.5),
               "voxel confidence must remain an arithmetic source mean");
    CHECK_TRUE(weighted.front().profileIndex == 3,
               "weighted voxel must retain the strongest source profile");
    CHECK_TRUE(weighted.front().observationCount == 5U,
               "voxel must sum represented observation counts");
    CHECK_TRUE(hik_scan::cloudPointHasQualityFlag(
                   weighted.front(),
                   hik_scan::CLOUD_QUALITY_VOXEL_AGGREGATED) &&
                   hik_scan::cloudPointHasQualityFlag(
                       weighted.front(),
                       hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED) &&
                   hik_scan::cloudPointHasQualityFlag(
                       weighted.front(),
                       hik_scan::CLOUD_QUALITY_REJECTED_NO_ADJACENT_PROFILE_SUPPORT),
               "voxel must OR source quality flags and mark aggregation");
    CHECK_TRUE(statistics.inputPointCount == 2U &&
                   statistics.finitePointCount == 2U &&
                   statistics.outputPointCount == 1U &&
                   statistics.confidenceWeighted,
               "voxel statistics must report weighted reduction counts");
}

void testQualityProfilePointsCarryOpticalAcceptance() {
    hik_calibration::StaticProfilePoint source;
    source.cameraPointMm = cv::Point3d(1.0, 2.0, 500.0);
    source.stripe.pixel = cv::Point2d(20.25, 30.5);
    source.stripe.confidence = 0.8;
    source.stripe.peakDifference = 120.0;
    source.stripe.qualityExtractor = true;
    source.stripe.rejectFlags = hik_stripe::REJECT_NONE;

    std::vector<hik_scan::CloudPoint> cloud;
    std::string error;
    CHECK_TRUE(hik_scan::appendProfilePointsUsingBaseFromCamera(
                   std::vector<hik_calibration::StaticProfilePoint>{source},
                   cv::Matx44d::eye(), 7, &cloud, &error),
               std::string("quality profile append failed: ") + error);
    CHECK_TRUE(cloud.size() == 1U &&
                   hik_scan::cloudPointHasQualityFlag(
                       cloud.front(),
                       hik_scan::CLOUD_QUALITY_OPTICAL_ACCEPTED),
               "a hard-gated quality center must retain an explicit optical "
               "acceptance flag in the point cloud");
}

void testPlyQualityProperties() {
    hik_scan::CloudPoint point = pointAt(1.0, 2.0, 3.0, 5, 0.7);
    point.qualityFlags =
        hik_scan::CLOUD_QUALITY_ADJACENT_PROFILE_SUPPORTED |
        hik_scan::CLOUD_QUALITY_VOXEL_AGGREGATED;
    point.observationCount = 4U;
    const std::vector<hik_scan::CloudPoint> cloud{point};

    std::ostringstream pathStream;
    pathStream << "/tmp/myline_hik_scan_core_test_"
               << static_cast<long long>(::getpid()) << ".ply";
    const std::string path = pathStream.str();
    std::remove(path.c_str());
    std::string error;
    CHECK_TRUE(hik_scan::saveScanPly(path, cloud, "base_link", &error),
               std::string("quality PLY save failed: ") + error);
    std::ifstream input(path.c_str(), std::ios::in | std::ios::binary);
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    std::remove(path.c_str());
    CHECK_TRUE(contents.find("property float pixel_v\n"
                             "property uint quality_flags\n"
                             "property uint observation_count\n") !=
                   std::string::npos,
               "new PLY properties must be appended after all legacy fields");
    CHECK_TRUE(contents.find(" 9 4\n") != std::string::npos,
               "PLY vertex must retain quality flags and observation count");
}

}  // namespace

int main() {
    testFilterDisabledByDefault();
    testSparseIsolatedPointRejected();
    testDistinctProfilesAndMaximumGap();
    testCoherentFalseSurfaceLimitation();
    testConfidenceWeightedVoxelDownsample();
    testQualityProfilePointsCarryOpticalAcceptance();
    testPlyQualityProperties();

    if (gFailures != 0) {
        std::cerr << "HikScanCore tests failed: " << gFailures
                  << " assertion(s)" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "HikScanCore tests passed" << std::endl;
    return EXIT_SUCCESS;
}
