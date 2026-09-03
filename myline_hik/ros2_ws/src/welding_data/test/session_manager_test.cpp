#include "welding_data/session_manager.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>

namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path = std::filesystem::temp_directory_path() /
      ("welding_data_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path);
  }
  ~TemporaryDirectory() {std::error_code ec; std::filesystem::remove_all(path, ec);}
  std::filesystem::path path;
};

TEST(SessionManager, CreatesIsolatedSessionAndFinalizesWithHashes)
{
  TemporaryDirectory temporary;
  welding_data::SessionManager manager(temporary.path);
  welding_data::SessionInfo session;
  std::string error;
  ASSERT_TRUE(manager.create("coarse_mapping", "task_001", "speed: 0.2\n", &session, &error)) << error;
  EXPECT_TRUE(std::filesystem::is_directory(session.directory / "artifacts/maps"));
  EXPECT_FALSE(manager.create("coarse_mapping", "task_001", "", &session, &error));
  {
    std::ofstream cloud(session.directory / "artifacts/maps/coarse_map.pcd");
    cloud << "test point cloud";
  }
  ASSERT_TRUE(manager.append_event("task_001", "MAP_SAVED", "coarse_map.pcd", &error)) << error;
  std::string approval_id;
  std::filesystem::path approval_path;
  ASSERT_TRUE(manager.approve_trajectory(
    "task_001", "plan_001", std::string(64, 'a'), "operator_01", "scene: v1\n",
    &approval_id, &approval_path, &error)) << error;
  EXPECT_TRUE(std::filesystem::is_regular_file(approval_path));
  std::filesystem::path manifest;
  ASSERT_TRUE(manager.finalize("task_001", "success", "coverage: 0.98\n", &manifest, &error)) << error;
  EXPECT_TRUE(std::filesystem::is_regular_file(manifest));
  EXPECT_FALSE(manager.append_event("task_001", "LATE_WRITE", "denied", &error));
  EXPECT_FALSE(manager.finalize("task_001", "success", "", &manifest, &error));
}

}  // namespace
