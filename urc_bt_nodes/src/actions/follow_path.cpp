#include "urc_bt_nodes/actions/follow_path.hpp"
#include "behaviortree_cpp/basic_types.h"
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>

namespace behavior::actions
{

bool FollowPath::setGoal(Goal & goal)
{
  RCLCPP_INFO(node_->get_logger(), "Setting Goal...");
  goal.path = getInput<nav_msgs::msg::Path>("path").value();
  return true;
}

void FollowPath::onHalt()
{
  RCLCPP_INFO(node_->get_logger(), "Halting...");
}

BT::NodeStatus FollowPath::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  if (feedback->distance_to_goal < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Negative distance to target, terminating following.");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    node_->get_logger(), "Following Path, %.2f m away from destination.",
    feedback->distance_to_goal);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowPath::onResultReceived(const WrappedResult & wr)
{
  RCLCPP_INFO(node_->get_logger(), "Path following is finished, code: %hu.", wr.result->error_code);
  if (wr.result->error_code == 0) {
    return BT::NodeStatus::SUCCESS;
  }

  // TODO: do some processing with the other error codes
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FollowPath::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "BT Action Node Error: %u.", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace behavior::actions

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(behavior::actions::FollowPath, "FollowPath");
