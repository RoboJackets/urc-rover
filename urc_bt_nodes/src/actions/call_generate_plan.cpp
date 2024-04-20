#include "urc_bt_nodes/actions/call_generate_plan.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_cpp/basic_types.h"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace behavior::actions
{

bool CallGeneratePlan::setRequest(typename Request::SharedPtr & request)
{
  auto start_pose = getInput<geometry_msgs::msg::Pose>("start_pose").value();
  auto goal_pose = getInput<geometry_msgs::msg::Pose>("goal_pose").value();

  request->start = geometry_msgs::msg::PoseStamped();
  request->start.pose = start_pose;
  request->goal.pose = goal_pose;
  return true;
}

BT::NodeStatus CallGeneratePlan::onResponseReceived(const typename Response::SharedPtr & response)
{
  if (response->error_code == 0) {
    setOutput("path", response->path);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to plan path.");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace behavior::actions

namespace BT
{

template<>
inline geometry_msgs::msg::Pose convertFromString(StringView str)
{
  auto coordinates = splitString(str, ',');
  if (coordinates.size() != 3) {
    throw std::runtime_error("Invalid input for pose. It should have three entries (x, y, theta)");
  }

  geometry_msgs::msg::Pose output;
  output.position.x = convertFromString<double>(coordinates[0]);
  output.position.y = convertFromString<double>(coordinates[1]);
  output.orientation.z = convertFromString<double>(coordinates[2]);

  return output;
}

}   // namespace BT

CreateRosNodePlugin(behavior::actions::CallGeneratePlan, "CallGeneratePlan");
