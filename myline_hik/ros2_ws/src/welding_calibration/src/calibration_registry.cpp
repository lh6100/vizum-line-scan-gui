#include "welding_calibration/calibration_registry.hpp"

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <system_error>

namespace welding_calibration
{
namespace
{

const std::array<const char *, 7> kRequiredRoles = {
  "camera_650_intrinsics",
  "camera_450_intrinsics",
  "camera_650_handeye",
  "camera_450_handeye",
  "stereo_extrinsics",
  "laser_650_plane",
  "laser_450_plane"};

void set_error(const std::string & value, std::string * error)
{
  if (error != nullptr) {
    *error = value;
  }
}

std::string lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string utc_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&raw, &utc);
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

bool valid_id(const std::string & value)
{
  if (value.empty() || value.size() > 128U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '_' || c == '-' || c == '.';
  });
}

bool atomic_write(const std::filesystem::path & path, const std::string & contents, std::string * error)
{
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {
    set_error("cannot create registry directory: " + ec.message(), error);
    return false;
  }
  const std::filesystem::path temporary = path.string() + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) {
      set_error("cannot open temporary registry file: " + temporary.string(), error);
      return false;
    }
    stream << contents;
    stream.flush();
    if (!stream) {
      set_error("cannot flush temporary registry file: " + temporary.string(), error);
      return false;
    }
  }
  std::filesystem::rename(temporary, path, ec);
  if (ec) {
    std::filesystem::remove(temporary);
    set_error("cannot atomically replace registry pointer: " + ec.message(), error);
    return false;
  }
  return true;
}

}  // namespace

std::string sha256_file(const std::filesystem::path & path, std::string * error)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    set_error("cannot read file for SHA-256: " + path.string(), error);
    return {};
  }
  EVP_MD_CTX * raw_context = EVP_MD_CTX_new();
  if (raw_context == nullptr) {
    set_error("cannot allocate OpenSSL digest context", error);
    return {};
  }
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(raw_context, EVP_MD_CTX_free);
  if (EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    set_error("cannot initialize SHA-256", error);
    return {};
  }
  std::array<char, 64 * 1024> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = stream.gcount();
    if (count > 0 && EVP_DigestUpdate(context.get(), buffer.data(), static_cast<std::size_t>(count)) != 1) {
      set_error("cannot update SHA-256", error);
      return {};
    }
  }
  if (!stream.eof()) {
    set_error("failed while reading file for SHA-256: " + path.string(), error);
    return {};
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    set_error("cannot finalize SHA-256", error);
    return {};
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

bool load_and_validate_manifest(
  const std::filesystem::path & path,
  CalibrationManifest * manifest,
  std::string * error)
{
  if (manifest == nullptr) {
    set_error("manifest output is null", error);
    return false;
  }
  *manifest = CalibrationManifest{};
  std::error_code ec;
  const auto absolute_path = std::filesystem::weakly_canonical(path, ec);
  if (ec || !std::filesystem::is_regular_file(absolute_path)) {
    set_error("calibration manifest does not exist: " + path.string(), error);
    return false;
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(absolute_path.string());
    manifest->schema_version = root["schema_version"].as<int>();
    manifest->package_id = root["package_id"].as<std::string>();
    manifest->status = lower(root["status"].as<std::string>());
  } catch (const std::exception & exception) {
    set_error(std::string("invalid calibration manifest header: ") + exception.what(), error);
    return false;
  }
  if (manifest->schema_version < 2) {
    set_error("calibration manifest schema_version must be at least 2", error);
    return false;
  }
  if (!valid_id(manifest->package_id)) {
    set_error("calibration package_id is invalid", error);
    return false;
  }
  if (manifest->status != "validated" && manifest->status != "active") {
    set_error("only validated calibration packages may be activated", error);
    return false;
  }
  const YAML::Node files = root["files"];
  if (!files || !files.IsMap()) {
    set_error("calibration manifest files must be a map", error);
    return false;
  }
  std::set<std::string> seen_roles;
  for (const auto & entry : files) {
    try {
      FileDependency dependency;
      dependency.role = entry.first.as<std::string>();
      const std::filesystem::path configured_path = entry.second["path"].as<std::string>();
      dependency.path = configured_path.is_absolute() ? configured_path : absolute_path.parent_path() / configured_path;
      dependency.path = std::filesystem::weakly_canonical(dependency.path, ec);
      if (ec || !std::filesystem::is_regular_file(dependency.path)) {
        set_error("calibration dependency is missing for role " + dependency.role, error);
        return false;
      }
      dependency.sha256 = lower(entry.second["sha256"].as<std::string>());
      std::string hash_error;
      const std::string actual_hash = sha256_file(dependency.path, &hash_error);
      if (actual_hash.empty() || dependency.sha256 != actual_hash) {
        set_error("calibration dependency hash mismatch for role " + dependency.role + ": " + hash_error, error);
        return false;
      }
      if (!seen_roles.insert(dependency.role).second) {
        set_error("duplicate calibration dependency role: " + dependency.role, error);
        return false;
      }
      manifest->files.push_back(std::move(dependency));
    } catch (const std::exception & exception) {
      set_error(std::string("invalid calibration dependency: ") + exception.what(), error);
      return false;
    }
  }
  for (const char * role : kRequiredRoles) {
    if (seen_roles.count(role) == 0U) {
      set_error(std::string("calibration manifest is missing required role: ") + role, error);
      return false;
    }
  }
  manifest->manifest_path = absolute_path;
  manifest->manifest_sha256 = sha256_file(absolute_path, error);
  return !manifest->manifest_sha256.empty();
}

CalibrationRegistry::CalibrationRegistry(std::filesystem::path root)
: root_(std::move(root))
{
}

std::filesystem::path CalibrationRegistry::active_pointer_path() const
{
  return root_ / "active_calibration.yaml";
}

bool CalibrationRegistry::activate(
  const std::filesystem::path & manifest_path,
  const std::string & expected_id,
  const std::string & expected_sha256,
  const std::string & approved_by,
  ActiveCalibration * active_output,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  CalibrationManifest manifest;
  if (!load_and_validate_manifest(manifest_path, &manifest, error)) {
    return false;
  }
  if (!expected_id.empty() && expected_id != manifest.package_id) {
    set_error("calibration package_id changed before activation", error);
    return false;
  }
  if (!expected_sha256.empty() && lower(expected_sha256) != manifest.manifest_sha256) {
    set_error("calibration manifest SHA-256 changed before activation", error);
    return false;
  }
  if (!valid_id(approved_by)) {
    set_error("approved_by must be a non-empty stable operator identifier", error);
    return false;
  }
  std::uint64_t next_version = 1;
  const auto current_path = active_pointer_path();
  if (std::filesystem::is_regular_file(current_path)) {
    try {
      const YAML::Node current = YAML::LoadFile(current_path.string());
      next_version = current["activation_version"].as<std::uint64_t>() + 1U;
    } catch (const std::exception &) {
      set_error("existing active calibration pointer is corrupt", error);
      return false;
    }
  }
  const std::string approved_at = utc_now();
  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "schema_version" << YAML::Value << 2;
  output << YAML::Key << "package_id" << YAML::Value << manifest.package_id;
  output << YAML::Key << "manifest_path" << YAML::Value << manifest.manifest_path.string();
  output << YAML::Key << "manifest_sha256" << YAML::Value << manifest.manifest_sha256;
  output << YAML::Key << "status" << YAML::Value << "active";
  output << YAML::Key << "approved_by" << YAML::Value << approved_by;
  output << YAML::Key << "approved_at" << YAML::Value << approved_at;
  output << YAML::Key << "activation_version" << YAML::Value << next_version;
  output << YAML::EndMap;
  if (!atomic_write(current_path, output.c_str(), error)) {
    return false;
  }
  if (active_output != nullptr) {
    active_output->manifest = std::move(manifest);
    active_output->approved_by = approved_by;
    active_output->approved_at = approved_at;
    active_output->activation_version = next_version;
  }
  return true;
}

std::optional<ActiveCalibration> CalibrationRegistry::active(std::string * error) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto path = active_pointer_path();
  if (!std::filesystem::is_regular_file(path)) {
    set_error("no active calibration package", error);
    return std::nullopt;
  }
  ActiveCalibration result;
  std::string expected_hash;
  try {
    const YAML::Node pointer = YAML::LoadFile(path.string());
    const std::string status = lower(pointer["status"].as<std::string>());
    if (status != "active") {
      set_error("active calibration pointer is not active", error);
      return std::nullopt;
    }
    expected_hash = lower(pointer["manifest_sha256"].as<std::string>());
    result.approved_by = pointer["approved_by"].as<std::string>();
    result.approved_at = pointer["approved_at"].as<std::string>();
    result.activation_version = pointer["activation_version"].as<std::uint64_t>();
    if (!load_and_validate_manifest(pointer["manifest_path"].as<std::string>(), &result.manifest, error)) {
      return std::nullopt;
    }
  } catch (const std::exception & exception) {
    set_error(std::string("active calibration pointer is invalid: ") + exception.what(), error);
    return std::nullopt;
  }
  if (result.manifest.manifest_sha256 != expected_hash) {
    set_error("active calibration manifest changed after approval", error);
    return std::nullopt;
  }
  return result;
}

bool CalibrationRegistry::invalidate_active(const std::string & reason, std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto path = active_pointer_path();
  if (!std::filesystem::is_regular_file(path)) {
    set_error("no active calibration package to invalidate", error);
    return false;
  }
  YAML::Node pointer;
  try {
    pointer = YAML::LoadFile(path.string());
  } catch (const std::exception & exception) {
    set_error(std::string("active calibration pointer is invalid: ") + exception.what(), error);
    return false;
  }
  pointer["status"] = "stale";
  pointer["invalidated_at"] = utc_now();
  pointer["invalidation_reason"] = reason;
  YAML::Emitter output;
  output << pointer;
  const auto stale_path = root_ / ("stale_calibration_" + pointer["package_id"].as<std::string>() + ".yaml");
  if (!atomic_write(stale_path, output.c_str(), error)) {
    return false;
  }
  std::error_code ec;
  std::filesystem::remove(path, ec);
  if (ec) {
    set_error("cannot remove active calibration pointer: " + ec.message(), error);
    return false;
  }
  return true;
}

}  // namespace welding_calibration
