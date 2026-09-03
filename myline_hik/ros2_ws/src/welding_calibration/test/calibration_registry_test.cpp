#include "welding_calibration/calibration_registry.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    path = std::filesystem::temp_directory_path() /
      ("welding_calibration_test_" + std::to_string(getpid()));
    std::filesystem::remove_all(path);
    std::filesystem::create_directories(path);
  }
  ~TemporaryDirectory() {std::filesystem::remove_all(path);}
  std::filesystem::path path;
};

TEST(CalibrationRegistry, RejectsCandidateAndActivatesImmutableValidatedPackage)
{
  TemporaryDirectory temporary;
  const char * roles[] = {
    "camera_650_intrinsics", "camera_450_intrinsics", "camera_650_handeye",
    "camera_450_handeye", "stereo_extrinsics", "laser_650_plane", "laser_450_plane"};
  std::ofstream manifest(temporary.path / "manifest.yaml");
  manifest << "schema_version: 2\npackage_id: test_package\nstatus: validated\nfiles:\n";
  for (const char * role : roles) {
    const auto file = temporary.path / (std::string(role) + ".yaml");
    std::ofstream(file) << role << "\n";
    std::string hash_error;
    const auto hash = welding_calibration::sha256_file(file, &hash_error);
    ASSERT_FALSE(hash.empty()) << hash_error;
    manifest << "  " << role << ":\n    path: " << file.string() << "\n    sha256: " << hash << "\n";
  }
  manifest.close();

  welding_calibration::CalibrationRegistry registry(temporary.path / "registry");
  welding_calibration::ActiveCalibration activated;
  std::string error;
  ASSERT_TRUE(registry.activate(
    temporary.path / "manifest.yaml", "test_package", "", "operator_1", &activated, &error)) << error;
  EXPECT_EQ(activated.activation_version, 1U);
  auto active = registry.active(&error);
  ASSERT_TRUE(active.has_value()) << error;
  EXPECT_EQ(active->manifest.package_id, "test_package");

  std::ofstream(temporary.path / "camera_650_intrinsics.yaml", std::ios::app) << "changed\n";
  active = registry.active(&error);
  EXPECT_FALSE(active.has_value());
  EXPECT_NE(error.find("hash mismatch"), std::string::npos);
}

}  // namespace
