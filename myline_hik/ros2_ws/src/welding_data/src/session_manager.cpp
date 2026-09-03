#include "welding_data/session_manager.hpp"

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <system_error>
#include <vector>

namespace welding_data
{
namespace
{

void set_error(const std::string & value, std::string * error)
{
  if (error != nullptr) {*error = value;}
}

std::string utc_now(const char * format = "%Y-%m-%dT%H:%M:%SZ")
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&raw, &utc);
  std::ostringstream out;
  out << std::put_time(&utc, format);
  return out.str();
}

bool valid_id(const std::string & value)
{
  return !value.empty() && value.size() <= 128U &&
    std::all_of(value.begin(), value.end(), [](unsigned char c) {
      return std::isalnum(c) != 0 || c == '_' || c == '-' || c == '.';
    });
}

bool atomic_write(const std::filesystem::path & path, const std::string & text, std::string * error)
{
  const auto temporary = std::filesystem::path(path.string() + ".tmp");
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) {
      set_error("cannot open temporary file: " + temporary.string(), error);
      return false;
    }
    stream << text;
    stream.flush();
    if (!stream) {
      set_error("cannot flush temporary file: " + temporary.string(), error);
      return false;
    }
  }
  std::error_code ec;
  std::filesystem::rename(temporary, path, ec);
  if (ec) {
    std::filesystem::remove(temporary);
    set_error("cannot atomically write " + path.string() + ": " + ec.message(), error);
    return false;
  }
  return true;
}

std::string sha256_file(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {return {};}
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {return {};}
  std::array<char, 65536> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    if (count > 0 && EVP_DigestUpdate(context.get(), buffer.data(), static_cast<std::size_t>(count)) != 1) {
      return {};
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int size = 0;
  if (!stream.eof() || EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1) {return {};}
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < size; ++i) {out << std::setw(2) << static_cast<unsigned int>(digest[i]);}
  return out.str();
}

bool load_session(const std::filesystem::path & path, YAML::Node * root, std::string * error)
{
  try {
    *root = YAML::LoadFile(path.string());
    return true;
  } catch (const std::exception & exception) {
    set_error(std::string("cannot load session manifest: ") + exception.what(), error);
    return false;
  }
}

}  // namespace

SessionManager::SessionManager(std::filesystem::path root)
: root_(std::move(root))
{
}

bool SessionManager::create(
  const std::string & task_type,
  const std::string & requested_id,
  const std::string & config_snapshot_yaml,
  SessionInfo * output,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (output == nullptr || !valid_id(task_type)) {
    set_error("task_type must be a stable identifier", error);
    return false;
  }
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::string id = requested_id.empty() ?
    utc_now("%Y%m%dT%H%M%SZ") + "_" + std::to_string(ticks) : requested_id;
  if (!valid_id(id)) {
    set_error("requested session_id is invalid", error);
    return false;
  }
  std::error_code ec;
  const auto directory = root_ / id;
  if (std::filesystem::exists(directory)) {
    set_error("session already exists: " + id, error);
    return false;
  }
  const std::vector<std::filesystem::path> directories = {
    "raw/images", "raw/robot", "derived/depth", "derived/cloud",
    "artifacts/maps", "artifacts/plans", "artifacts/scans", "artifacts/welds",
    "snapshots", "approvals", "logs"};
  for (const auto & relative : directories) {
    std::filesystem::create_directories(directory / relative, ec);
    if (ec) {
      set_error("cannot create session directory: " + ec.message(), error);
      std::filesystem::remove_all(directory, ec);
      return false;
    }
  }
  if (!atomic_write(directory / "snapshots/config.yaml", config_snapshot_yaml, error)) {
    std::filesystem::remove_all(directory, ec);
    return false;
  }
  YAML::Node manifest;
  manifest["schema_version"] = 2;
  manifest["session_id"] = id;
  manifest["task_type"] = task_type;
  manifest["state"] = "open";
  manifest["created_at"] = utc_now();
  manifest["config_snapshot"] = "snapshots/config.yaml";
  YAML::Emitter emitter;
  emitter << manifest;
  if (!atomic_write(directory / "session.yaml", emitter.c_str(), error)) {
    std::filesystem::remove_all(directory, ec);
    return false;
  }
  output->id = id;
  output->task_type = task_type;
  output->directory = directory;
  output->manifest_path = directory / "session.yaml";
  std::ofstream event_stream(directory / "logs/events.log", std::ios::app);
  event_stream << utc_now() << "\tSESSION_CREATED\t" << task_type << '\n';
  return true;
}

bool SessionManager::append_event(
  const std::string & session_id,
  const std::string & event_type,
  const std::string & detail,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_id(session_id) || !valid_id(event_type)) {
    set_error("session_id or event_type is invalid", error);
    return false;
  }
  const auto session = root_ / session_id;
  YAML::Node manifest;
  if (!load_session(session / "session.yaml", &manifest, error)) {return false;}
  if (manifest["state"].as<std::string>() != "open") {
    set_error("cannot append to a finalized session", error);
    return false;
  }
  std::string safe_detail = detail;
  std::replace(safe_detail.begin(), safe_detail.end(), '\n', ' ');
  std::replace(safe_detail.begin(), safe_detail.end(), '\r', ' ');
  std::ofstream stream(session / "logs/events.log", std::ios::app);
  if (!stream) {
    set_error("cannot append session event", error);
    return false;
  }
  stream << utc_now() << '\t' << event_type << '\t' << safe_detail << '\n';
  return static_cast<bool>(stream);
}

bool SessionManager::approve_trajectory(
  const std::string & session_id,
  const std::string & plan_id,
  const std::string & trajectory_digest,
  const std::string & operator_id,
  const std::string & dependencies_yaml,
  std::string * approval_id,
  std::filesystem::path * manifest_path,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_id(session_id) || !valid_id(plan_id) || !valid_id(operator_id) ||
    trajectory_digest.size() != 64U)
  {
    set_error("approval identifiers or trajectory digest are invalid", error);
    return false;
  }
  const auto directory = root_ / session_id;
  YAML::Node session;
  if (!load_session(directory / "session.yaml", &session, error)) {return false;}
  if (session["session_id"].as<std::string>() != session_id ||
    session["state"].as<std::string>() != "open")
  {
    set_error("trajectory approval requires an open session", error);
    return false;
  }
  YAML::Node dependencies;
  try {
    dependencies = YAML::Load(dependencies_yaml);
  } catch (const std::exception & exception) {
    set_error(std::string("invalid dependency snapshot: ") + exception.what(), error);
    return false;
  }
  if (!dependencies.IsMap()) {
    set_error("dependency snapshot must be a map", error);
    return false;
  }
  const std::string id = plan_id + "_approval_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  YAML::Node approval;
  approval["schema_version"] = 2;
  approval["approval_id"] = id;
  approval["session_id"] = session_id;
  approval["plan_id"] = plan_id;
  approval["trajectory_digest"] = trajectory_digest;
  approval["operator_id"] = operator_id;
  approval["approved_at"] = utc_now();
  approval["dependencies"] = dependencies;
  YAML::Emitter emitter;
  emitter << approval;
  const auto path = directory / "approvals" / (id + ".yaml");
  if (!atomic_write(path, emitter.c_str(), error)) {return false;}
  std::ofstream event_stream(directory / "logs/events.log", std::ios::app);
  event_stream << utc_now() << "\tTRAJECTORY_APPROVED\t" << id << '\n';
  if (!event_stream) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
    set_error("cannot record approval event", error);
    return false;
  }
  if (approval_id != nullptr) {*approval_id = id;}
  if (manifest_path != nullptr) {*manifest_path = path;}
  return true;
}

bool SessionManager::finalize(
  const std::string & session_id,
  const std::string & result,
  const std::string & summary_yaml,
  std::filesystem::path * manifest_path,
  std::string * error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_id(session_id) || result.empty()) {
    set_error("session_id or result is invalid", error);
    return false;
  }
  const auto directory = root_ / session_id;
  const auto path = directory / "session.yaml";
  YAML::Node manifest;
  if (!load_session(path, &manifest, error)) {return false;}
  if (manifest["session_id"].as<std::string>() != session_id ||
    manifest["state"].as<std::string>() != "open")
  {
    set_error("session is not open", error);
    return false;
  }
  if (!atomic_write(directory / "summary.yaml", summary_yaml, error)) {return false;}
  YAML::Node artifacts(YAML::NodeType::Sequence);
  std::error_code ec;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(directory, ec)) {
    if (ec) {break;}
    if (!entry.is_regular_file() || entry.path() == path || entry.path().extension() == ".tmp") {continue;}
    YAML::Node artifact;
    artifact["path"] = std::filesystem::relative(entry.path(), directory).generic_string();
    artifact["size"] = static_cast<std::uint64_t>(entry.file_size());
    artifact["sha256"] = sha256_file(entry.path());
    artifacts.push_back(artifact);
  }
  if (ec) {
    set_error("cannot enumerate session artifacts: " + ec.message(), error);
    return false;
  }
  manifest["state"] = "finalized";
  manifest["result"] = result;
  manifest["finalized_at"] = utc_now();
  manifest["summary"] = "summary.yaml";
  manifest["artifacts"] = artifacts;
  YAML::Emitter emitter;
  emitter << manifest;
  if (!atomic_write(path, emitter.c_str(), error)) {return false;}
  if (manifest_path != nullptr) {*manifest_path = path;}
  return true;
}

}  // namespace welding_data
