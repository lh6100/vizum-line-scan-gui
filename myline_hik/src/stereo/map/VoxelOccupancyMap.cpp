#include "stereo/map/VoxelOccupancyMap.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hik_stereo {
namespace {

struct VoxelKey {
    int x{0};
    int y{0};
    int z{0};
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept {
        std::size_t value = static_cast<std::size_t>(
            static_cast<std::uint32_t>(key.x) * 73856093U);
        value ^= static_cast<std::size_t>(
            static_cast<std::uint32_t>(key.y) * 19349663U);
        value ^= static_cast<std::size_t>(
            static_cast<std::uint32_t>(key.z) * 83492791U);
        return value;
    }
};

struct VoxelValue {
    double logOdds{0.0};
    std::uint32_t hits{0U};
    std::uint32_t misses{0U};
};

void setError(const std::string& value, std::string* error) {
    if (error) *error = value;
}

bool finiteTransform(const cv::Matx44d& transform) {
    for (double value : transform.val) {
        if (!std::isfinite(value)) return false;
    }
    return std::abs(transform(3, 3) - 1.0) < 1.0e-9;
}

std::string basenameOf(const std::string& path) {
    const std::string::size_type separator = path.find_last_of("/\\");
    return separator == std::string::npos ? path : path.substr(separator + 1U);
}

}  // namespace

struct VoxelOccupancyMap::Impl {
    std::unordered_map<VoxelKey, VoxelValue, VoxelKeyHash> voxels;
    OccupancyMapStatistics statistics;
};

VoxelOccupancyMap::VoxelOccupancyMap() : impl_(new Impl) {}
VoxelOccupancyMap::~VoxelOccupancyMap() { delete impl_; }

VoxelOccupancyMap::VoxelOccupancyMap(VoxelOccupancyMap&& other) noexcept
    : impl_(other.impl_), options_(other.options_) {
    other.impl_ = new Impl;
}

VoxelOccupancyMap& VoxelOccupancyMap::operator=(
        VoxelOccupancyMap&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = other.impl_;
        options_ = other.options_;
        other.impl_ = new Impl;
    }
    return *this;
}

bool VoxelOccupancyMap::configure(const OccupancyMapOptions& options,
                                  std::string* error) {
    if (!std::isfinite(options.voxelSizeMm) ||
        options.voxelSizeMm < 2.0 || options.voxelSizeMm > 500.0 ||
        options.pixelStride < 1 || options.pixelStride > 64 ||
        options.maximumRaySteps < 1 || options.maximumRaySteps > 2000 ||
        !std::isfinite(options.hitLogOdds) || options.hitLogOdds <= 0.0 ||
        !std::isfinite(options.missLogOdds) || options.missLogOdds >= 0.0 ||
        !std::isfinite(options.minimumLogOdds) ||
        !std::isfinite(options.maximumLogOdds) ||
        options.minimumLogOdds >= options.maximumLogOdds ||
        options.occupiedThresholdLogOdds <= options.minimumLogOdds ||
        options.occupiedThresholdLogOdds >= options.maximumLogOdds) {
        setError("occupancy-map options are invalid", error);
        return false;
    }
    options_ = options;
    clear();
    return true;
}

void VoxelOccupancyMap::clear() {
    impl_->voxels.clear();
    impl_->statistics = OccupancyMapStatistics();
}

bool VoxelOccupancyMap::integrate(const cv::Mat& xyzCameraMm,
                                  const cv::Mat& validMask,
                                  const cv::Matx44d& baseFromCamera,
                                  std::string* error) {
    if (xyzCameraMm.empty() || xyzCameraMm.type() != CV_32FC3 ||
        validMask.type() != CV_8UC1 ||
        xyzCameraMm.size() != validMask.size() ||
        !finiteTransform(baseFromCamera)) {
        setError("occupancy integration input is invalid", error);
        return false;
    }
    const cv::Vec3d origin(
        baseFromCamera(0, 3),
        baseFromCamera(1, 3),
        baseFromCamera(2, 3));
    auto keyFor = [this](const cv::Vec3d& point) {
        return VoxelKey{
            static_cast<int>(std::floor(point[0] / options_.voxelSizeMm)),
            static_cast<int>(std::floor(point[1] / options_.voxelSizeMm)),
            static_cast<int>(std::floor(point[2] / options_.voxelSizeMm))};
    };
    auto update = [this](const VoxelKey& key, bool occupied) {
        VoxelValue& value = impl_->voxels[key];
        value.logOdds = std::max(options_.minimumLogOdds,
            std::min(options_.maximumLogOdds,
                     value.logOdds + (occupied
                         ? options_.hitLogOdds : options_.missLogOdds)));
        if (occupied) {
            ++value.hits;
            ++impl_->statistics.occupiedUpdates;
        } else {
            ++value.misses;
            ++impl_->statistics.freeUpdates;
        }
    };
    for (int y = 0; y < xyzCameraMm.rows; y += options_.pixelStride) {
        const cv::Vec3f* xyz = xyzCameraMm.ptr<cv::Vec3f>(y);
        const uchar* valid = validMask.ptr<uchar>(y);
        for (int x = 0; x < xyzCameraMm.cols; x += options_.pixelStride) {
            ++impl_->statistics.inputDepthPoints;
            if (!valid[x]) continue;
            const cv::Vec3f cameraPoint = xyz[x];
            if (!std::isfinite(cameraPoint[0]) ||
                !std::isfinite(cameraPoint[1]) ||
                !std::isfinite(cameraPoint[2])) continue;
            cv::Vec4d homogeneous(
                cameraPoint[0], cameraPoint[1], cameraPoint[2], 1.0);
            const cv::Vec4d transformed = baseFromCamera * homogeneous;
            const cv::Vec3d endpoint(
                transformed[0], transformed[1], transformed[2]);
            const cv::Vec3d ray = endpoint - origin;
            const double distance = cv::norm(ray);
            if (!std::isfinite(distance) || distance < options_.voxelSizeMm)
                continue;
            const int naturalSteps = static_cast<int>(
                std::ceil(distance / options_.voxelSizeMm));
            const int steps = std::max(1, std::min(
                options_.maximumRaySteps, naturalSteps));
            VoxelKey previous;
            bool havePrevious = false;
            for (int step = 1; step < steps; ++step) {
                const double ratio = static_cast<double>(step) / steps;
                const VoxelKey key = keyFor(origin + ratio * ray);
                if (!havePrevious || !(key == previous)) {
                    update(key, false);
                    previous = key;
                    havePrevious = true;
                }
            }
            update(keyFor(endpoint), true);
            ++impl_->statistics.integratedEndpoints;
        }
    }
    ++impl_->statistics.integratedFrames;
    impl_->statistics.allocatedVoxels = impl_->voxels.size();
    impl_->statistics.occupiedVoxels = 0U;
    for (const auto& entry : impl_->voxels) {
        if (entry.second.logOdds >= options_.occupiedThresholdLogOdds)
            ++impl_->statistics.occupiedVoxels;
    }
    return true;
}

bool VoxelOccupancyMap::saveOccupiedPly(
        const std::string& path, std::string* error) const {
    std::ofstream output(path.c_str(), std::ios::binary);
    if (!output) {
        setError("cannot open occupancy PLY: " + path, error);
        return false;
    }
    const OccupancyMapStatistics stats = statistics();
    output << "ply\nformat ascii 1.0\n"
           << "comment occupied voxel centers in base_link, millimetres\n"
           << "element vertex " << stats.occupiedVoxels << "\n"
           << "property float x\nproperty float y\nproperty float z\n"
           << "property float probability\nend_header\n";
    for (const auto& entry : impl_->voxels) {
        if (entry.second.logOdds < options_.occupiedThresholdLogOdds) continue;
        const double probability = 1.0 /
            (1.0 + std::exp(-entry.second.logOdds));
        output << (entry.first.x + 0.5) * options_.voxelSizeMm << ' '
               << (entry.first.y + 0.5) * options_.voxelSizeMm << ' '
               << (entry.first.z + 0.5) * options_.voxelSizeMm << ' '
               << probability << '\n';
    }
    if (!output) {
        setError("failed while writing occupancy PLY: " + path, error);
        return false;
    }
    return true;
}

bool VoxelOccupancyMap::save2DGrid(
        const std::string& pgmPath,
        const std::string& yamlPath,
        double minimumHeightMm,
        double maximumHeightMm,
        std::string* error) const {
    if (!std::isfinite(minimumHeightMm) ||
        !std::isfinite(maximumHeightMm) ||
        maximumHeightMm <= minimumHeightMm) {
        setError("2-D occupancy height range is invalid", error);
        return false;
    }
    struct CellState { bool free{false}; bool occupied{false}; };
    std::map<std::pair<int, int>, CellState> cells;
    int minimumX = std::numeric_limits<int>::max();
    int maximumX = std::numeric_limits<int>::min();
    int minimumY = std::numeric_limits<int>::max();
    int maximumY = std::numeric_limits<int>::min();
    for (const auto& entry : impl_->voxels) {
        const double height = (entry.first.z + 0.5) * options_.voxelSizeMm;
        if (height < minimumHeightMm || height > maximumHeightMm) continue;
        const std::pair<int, int> key(entry.first.x, entry.first.y);
        CellState& state = cells[key];
        if (entry.second.logOdds >= options_.occupiedThresholdLogOdds)
            state.occupied = true;
        else if (entry.second.logOdds < 0.0)
            state.free = true;
        minimumX = std::min(minimumX, entry.first.x);
        maximumX = std::max(maximumX, entry.first.x);
        minimumY = std::min(minimumY, entry.first.y);
        maximumY = std::max(maximumY, entry.first.y);
    }
    if (cells.empty()) {
        setError("no occupancy voxels intersect the requested height range", error);
        return false;
    }
    const int width = maximumX - minimumX + 1;
    const int height = maximumY - minimumY + 1;
    if (width <= 0 || height <= 0 || width > 20000 || height > 20000 ||
        static_cast<std::uint64_t>(width) * height > 100000000ULL) {
        setError("2-D occupancy grid bounds are too large", error);
        return false;
    }
    std::vector<unsigned char> pixels(
        static_cast<std::size_t>(width) * height, 205U);
    for (const auto& entry : cells) {
        const int x = entry.first.first - minimumX;
        const int y = maximumY - entry.first.second;
        unsigned char value = 205U;
        if (entry.second.occupied) value = 0U;
        else if (entry.second.free) value = 254U;
        pixels[static_cast<std::size_t>(y) * width + x] = value;
    }
    std::ofstream pgm(pgmPath.c_str(), std::ios::binary);
    if (!pgm) {
        setError("cannot open occupancy PGM: " + pgmPath, error);
        return false;
    }
    pgm << "P5\n" << width << ' ' << height << "\n255\n";
    pgm.write(reinterpret_cast<const char*>(pixels.data()),
              static_cast<std::streamsize>(pixels.size()));
    if (!pgm) {
        setError("failed while writing occupancy PGM: " + pgmPath, error);
        return false;
    }
    std::ofstream yaml(yamlPath.c_str());
    if (!yaml) {
        setError("cannot open occupancy YAML: " + yamlPath, error);
        return false;
    }
    yaml << "image: \"" << basenameOf(pgmPath) << "\"\n"
         << "resolution: " << options_.voxelSizeMm / 1000.0 << "\n"
         << "origin: [" << minimumX * options_.voxelSizeMm / 1000.0
         << ", " << minimumY * options_.voxelSizeMm / 1000.0
         << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
         << "frame_id: \"base_link\"\n"
         << "height_min_m: " << minimumHeightMm / 1000.0 << "\n"
         << "height_max_m: " << maximumHeightMm / 1000.0 << "\n";
    if (!yaml) {
        setError("failed while writing occupancy YAML: " + yamlPath, error);
        return false;
    }
    return true;
}

OccupancyMapStatistics VoxelOccupancyMap::statistics() const {
    return impl_->statistics;
}

}  // namespace hik_stereo
