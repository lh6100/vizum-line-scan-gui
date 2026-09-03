#ifndef WELDING_DATA__SESSION_MANAGER_HPP_
#define WELDING_DATA__SESSION_MANAGER_HPP_

#include <filesystem>
#include <mutex>
#include <string>

namespace welding_data
{

struct SessionInfo
{
  std::string id;
  std::string task_type;
  std::filesystem::path directory;
  std::filesystem::path manifest_path;
};

class SessionManager
{
public:
  explicit SessionManager(std::filesystem::path root);

  bool create(
    const std::string & task_type,
    const std::string & requested_id,
    const std::string & config_snapshot_yaml,
    SessionInfo * output,
    std::string * error = nullptr);

  bool append_event(
    const std::string & session_id,
    const std::string & event_type,
    const std::string & detail,
    std::string * error = nullptr);

  bool approve_trajectory(
    const std::string & session_id,
    const std::string & plan_id,
    const std::string & trajectory_digest,
    const std::string & operator_id,
    const std::string & dependencies_yaml,
    std::string * approval_id,
    std::filesystem::path * manifest_path,
    std::string * error = nullptr);

  bool finalize(
    const std::string & session_id,
    const std::string & result,
    const std::string & summary_yaml,
    std::filesystem::path * manifest_path,
    std::string * error = nullptr);

  const std::filesystem::path & root() const {return root_;}

private:
  std::filesystem::path root_;
  mutable std::mutex mutex_;
};

}  // namespace welding_data

#endif  // WELDING_DATA__SESSION_MANAGER_HPP_
