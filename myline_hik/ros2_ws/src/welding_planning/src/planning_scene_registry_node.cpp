#include "welding_planning/plan_dependency_validator.hpp"

#include <moveit_msgs/msg/planning_scene.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <welding_interfaces/srv/create_planning_scene_version.hpp>
#include <welding_interfaces/srv/get_active_artifact.hpp>
#include <welding_interfaces/srv/validate_plan_dependencies.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{

std::string sha256_text(const std::string & text)
{
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), text.data(), text.size()) != 1)
  {
    return {};
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &size) != 1) {return {};}
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < size; ++index) {
    out << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return out.str();
}

bool atomic_write(const std::filesystem::path & path, const std::string & text, std::string * error)
{
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {*error = "cannot create scene registry: " + ec.message(); return false;}
  if (std::filesystem::exists(path)) {*error = "immutable scene manifest already exists"; return false;}
  const auto temporary = std::filesystem::path(path.string() + ".tmp");
  {std::ofstream stream(temporary, std::ios::trunc); stream << text; if (!stream) {*error = "cannot write scene manifest"; return false;}}
  std::filesystem::rename(temporary, path, ec);
  if (ec) {std::filesystem::remove(temporary); *error = "cannot commit scene manifest: " + ec.message(); return false;}
  return true;
}

std::uint64_t maximum_scene_version(const std::filesystem::path & root)
{
  std::uint64_t maximum = 0;
  std::error_code ec;
  if (!std::filesystem::is_directory(root)) {return maximum;}
  for (const auto & entry : std::filesystem::recursive_directory_iterator(root, ec)) {
    if (ec) {break;}
    if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {continue;}
    try {
      const YAML::Node manifest = YAML::LoadFile(entry.path().string());
      maximum = std::max(maximum, manifest["version"].as<std::uint64_t>());
    } catch (const std::exception &) {
      continue;
    }
  }
  return maximum;
}

class PlanningSceneRegistryNode final : public rclcpp::Node
{
public:
  PlanningSceneRegistryNode()
  : Node("planning_scene_registry"),
    registry_root_(declare_parameter<std::string>("registry_root", "data/registry/planning_scenes")),
    srdf_sha256_(declare_parameter<std::string>("srdf_sha256", "")),
    workpiece_frame_version_(declare_parameter<std::string>("workpiece_frame_version", "")),
    planner_config_sha256_(declare_parameter<std::string>("planner_config_sha256", ""))
  {
    scene_version_ = maximum_scene_version(registry_root_);
    map_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
      declare_parameter<std::string>("octomap_topic", "/welding_robot/map/octomap"),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr map) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = std::move(map);
      });
    scene_publisher_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(1).reliable().transient_local());
    calibration_client_ = create_client<welding_interfaces::srv::GetActiveArtifact>(
      "/welding_robot/calibration/get_active");
    calibration_timer_ = create_wall_timer(1s, [this]() {refresh_calibration();});
    create_service_ = create_service<welding_interfaces::srv::CreatePlanningSceneVersion>(
      "/welding_robot/planning_scene/create_version",
      [this](
        const std::shared_ptr<welding_interfaces::srv::CreatePlanningSceneVersion::Request> request,
        std::shared_ptr<welding_interfaces::srv::CreatePlanningSceneVersion::Response> response)
      {create_version(*request, response.get());});
    validate_service_ = create_service<welding_interfaces::srv::ValidatePlanDependencies>(
      "/welding_robot/planning_scene/validate_dependencies",
      [this](
        const std::shared_ptr<welding_interfaces::srv::ValidatePlanDependencies::Request> request,
        std::shared_ptr<welding_interfaces::srv::ValidatePlanDependencies::Response> response)
      {
        response->valid = validator_.validate(request->dependencies, &response->active, &response->reason);
      });
  }

private:
  void refresh_calibration()
  {
    if (!calibration_client_->service_is_ready() || calibration_request_pending_) {return;}
    calibration_request_pending_ = true;
    auto request = std::make_shared<welding_interfaces::srv::GetActiveArtifact::Request>();
    calibration_client_->async_send_request(
      request,
      [this](rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedFuture future) {
      const auto response = future.get();
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      if (response->active) {active_calibration_ = response->artifact;} else {active_calibration_.reset();}
      calibration_request_pending_ = false;
    });
  }

  void create_version(
    const welding_interfaces::srv::CreatePlanningSceneVersion::Request & request,
    welding_interfaces::srv::CreatePlanningSceneVersion::Response * response)
  {
    if (!request.self_filter_valid) {response->error = "robot self-filter is not validated"; return;}
    if (request.map_id.empty() || request.map_version == 0U || request.robot_model_sha256.empty() ||
      request.tool_model_sha256.empty())
    {response->error = "map and model dependencies must be explicit"; return;}
    octomap_msgs::msg::Octomap::ConstSharedPtr map;
    {std::lock_guard<std::mutex> lock(map_mutex_); map = latest_map_;}
    if (!map || map->data.empty()) {response->error = "no non-empty OctoMap has been received"; return;}
    std::optional<welding_interfaces::msg::ArtifactRef> calibration;
    {std::lock_guard<std::mutex> lock(calibration_mutex_); calibration = active_calibration_;}
    if (!calibration || calibration->id != request.calibration_package_id) {
      response->error = "requested calibration is not the active validated package";
      return;
    }

    const std::uint64_t version = ++scene_version_;
    const std::string scene_id = request.map_id + "_scene";
    std::ostringstream digest_input;
    digest_input << scene_id << version << request.map_id << request.map_version
                 << calibration->id << calibration->sha256 << request.robot_model_sha256
                 << request.tool_model_sha256 << map->id << map->resolution << map->binary;
    digest_input.write(reinterpret_cast<const char *>(map->data.data()),
      static_cast<std::streamsize>(map->data.size()));
    const std::string scene_sha256 = sha256_text(digest_input.str());

    welding_interfaces::msg::PlanDependencies dependencies;
    dependencies.calibration_package_id = calibration->id;
    dependencies.calibration_package_sha256 = calibration->sha256;
    dependencies.planning_scene_id = scene_id;
    dependencies.planning_scene_version = version;
    dependencies.planning_scene_sha256 = scene_sha256;
    dependencies.robot_model_sha256 = request.robot_model_sha256;
    dependencies.srdf_sha256 = srdf_sha256_;
    dependencies.tool_model_sha256 = request.tool_model_sha256;
    dependencies.workpiece_frame_version = workpiece_frame_version_;
    dependencies.planner_config_sha256 = planner_config_sha256_;
    if (!welding_planning::complete_dependencies(dependencies, &response->error)) {return;}

    YAML::Node manifest;
    manifest["schema_version"] = 2;
    manifest["scene_id"] = scene_id;
    manifest["version"] = version;
    manifest["sha256"] = scene_sha256;
    manifest["map_id"] = request.map_id;
    manifest["map_version"] = request.map_version;
    manifest["calibration_package_id"] = calibration->id;
    manifest["calibration_package_sha256"] = calibration->sha256;
    manifest["robot_model_sha256"] = request.robot_model_sha256;
    manifest["tool_model_sha256"] = request.tool_model_sha256;
    manifest["self_filter_valid"] = true;
    YAML::Emitter emitter;
    emitter << manifest;
    const auto manifest_path = registry_root_ / scene_id / (std::to_string(version) + ".yaml");
    if (!atomic_write(manifest_path, emitter.c_str(), &response->error)) {return;}
    if (!validator_.activate(dependencies, &response->error)) {return;}

    moveit_msgs::msg::PlanningScene scene;
    scene.name = scene_id + "_v" + std::to_string(version);
    scene.is_diff = true;
    scene.world.octomap.header = map->header;
    scene.world.octomap.origin.orientation.w = 1.0;
    scene.world.octomap.octomap = *map;
    scene_publisher_->publish(scene);

    response->success = true;
    response->planning_scene.id = scene_id;
    response->planning_scene.version = version;
    response->planning_scene.sha256 = scene_sha256;
    response->planning_scene.status = "active";
    response->planning_scene.manifest_path = manifest_path.string();
    RCLCPP_INFO(get_logger(), "activated planning scene %s version %lu", scene_id.c_str(), version);
  }

  std::filesystem::path registry_root_;
  std::string srdf_sha256_;
  std::string workpiece_frame_version_;
  std::string planner_config_sha256_;
  std::uint64_t scene_version_{0};
  std::mutex map_mutex_;
  octomap_msgs::msg::Octomap::ConstSharedPtr latest_map_;
  std::mutex calibration_mutex_;
  std::optional<welding_interfaces::msg::ArtifactRef> active_calibration_;
  bool calibration_request_pending_{false};
  welding_planning::PlanDependencyValidator validator_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_publisher_;
  rclcpp::Client<welding_interfaces::srv::GetActiveArtifact>::SharedPtr calibration_client_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;
  rclcpp::Service<welding_interfaces::srv::CreatePlanningSceneVersion>::SharedPtr create_service_;
  rclcpp::Service<welding_interfaces::srv::ValidatePlanDependencies>::SharedPtr validate_service_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningSceneRegistryNode>());
  rclcpp::shutdown();
  return 0;
}
