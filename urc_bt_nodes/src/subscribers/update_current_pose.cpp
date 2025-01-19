#include "urc_bt_nodes/subscribers/update_current_pose.hpp"

using namespace BT;

namespace behavior::subscribers
{
NodeStatus UpdateCurrentPose::onTick(
  const std::shared_ptr<nav_msgs::msg::Odometry> & last_msg)
{
  if (last_msg) {
    RCLCPP_INFO(node_->get_logger(), "New current pose received!");
    setOutput("pose", last_msg->pose.pose);
  }

  return NodeStatus::SUCCESS;
}
} // namespace behavior::subscribers

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(behavior::subscribers::UpdateCurrentPose, "UpdateCurrentPose");
