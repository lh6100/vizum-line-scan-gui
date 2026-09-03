#ifndef WELDING_CALIBRATION__CALIBRATION_REGISTRY_HPP_
#define WELDING_CALIBRATION__CALIBRATION_REGISTRY_HPP_

#include <filesystem>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace welding_calibration
{

struct FileDependency
{
  std::string role;
  std::filesystem::path path;
  std::string sha256;
};

struct CalibrationManifest
{
  int schema_version{0};
  std::string package_id;
  std::string status;
  std::filesystem::path manifest_path;
  std::string manifest_sha256;
  std::vector<FileDependency> files;
};

struct ActiveCalibration
{
  CalibrationManifest manifest;
  std::string approved_by;
  std::string approved_at;
  std::uint64_t activation_version{0};
};

std::string sha256_file(const std::filesystem::path & path, std::string * error = nullptr);

bool load_and_validate_manifest(
  const std::filesystem::path & path,
  CalibrationManifest * manifest,
  std::string * error = nullptr);

class CalibrationRegistry
{
public:
  explicit CalibrationRegistry(std::filesystem::path root);

  bool activate(
    const std::filesystem::path & manifest_path,
    const std::string & expected_id,
    const std::string & expected_sha256,
    const std::string & approved_by,
    ActiveCalibration * active,
    std::string * error = nullptr);

  std::optional<ActiveCalibration> active(std::string * error = nullptr) const;

  bool invalidate_active(
    const std::string & reason,
    std::string * error = nullptr);

  const std::filesystem::path & root() const {return root_;}

private:
  std::filesystem::path active_pointer_path() const;
  std::filesystem::path root_;
  mutable std::mutex mutex_;
};

}  // namespace welding_calibration

#endif  // WELDING_CALIBRATION__CALIBRATION_REGISTRY_HPP_
