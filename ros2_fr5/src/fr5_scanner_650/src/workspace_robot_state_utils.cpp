#include "workspace_robot_state_utils.hpp"

#include <exception>

namespace fr5_scanner_650::workspace_robot_state
{

bool updatedGlobalLinkTransform(
  const moveit::core::RobotState & state, const std::string & link_name,
  Eigen::Isometry3d * transform, std::string * error)
{
  if (!transform) {
    if (error) {*error = "output transform is null";}
    return false;
  }
  if (link_name.empty()) {
    if (error) {*error = "link name is empty";}
    return false;
  }

  try {
    moveit::core::RobotState updated_state(state);
    const moveit::core::LinkModel * link = updated_state.getLinkModel(link_name);
    if (!link) {
      if (error) {*error = "robot model contains no link named " + link_name;}
      return false;
    }

    // Explicitly refresh FK before using a transform accessor.  The source
    // snapshot deliberately remains untouched.
    updated_state.update();
    const Eigen::Isometry3d updated_transform =
      updated_state.getGlobalLinkTransform(link);
    if (!updated_transform.matrix().allFinite()) {
      if (error) {*error = "updated transform for " + link_name + " is non-finite";}
      return false;
    }
    *transform = updated_transform;
    return true;
  } catch (const std::exception & exception) {
    if (error) {
      *error = "cannot update transform for " + link_name + ": " + exception.what();
    }
    return false;
  }
}

}  // namespace fr5_scanner_650::workspace_robot_state
