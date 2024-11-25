#include "search_for_aruco.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace urc_behaviors
{
SearchForAruco::SearchForAruco(const rclcpp::NodeOptions & options)
: Node("search_for_aruco", options)
{
  RCLCPP_INFO(this->get_logger(), "Search for ARUCO behavior server has been started.");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  follow_path_client_ = rclcpp_action::create_client<urc_msgs::action::FollowPath>(
    this,
    "follow_path");

  search();
}

void SearchForAruco::search()
{
  // Wait until base_link to map transform is available
  while (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
    RCLCPP_INFO(this->get_logger(), "Waiting for transform from base_link to map...");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(this->get_logger(), "Transform from base_link to map is available.");

  // Generate a search path
  auto path = generate_search_path(0.1);

  // Send the path to the follow_path action server
  auto goal = FollowPath::Goal();
  goal.path = path;

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &SearchForAruco::goal_response_callback,
    this, std::placeholders::_1);
  send_goal_options.result_callback = std::bind(
    &SearchForAruco::result_callback, this,
    std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &SearchForAruco::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  follow_path_client_->async_send_goal(goal, send_goal_options);
}

void SearchForAruco::goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void SearchForAruco::result_callback(const GoalHandleFollowPath::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}

void SearchForAruco::feedback_callback(
  GoalHandleFollowPath::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->distance_to_goal);
}

nav_msgs::msg::Path SearchForAruco::generate_search_path(double spiral_coeff)
{
  rclcpp::Time now = this->now();
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = now;

  // Lookup transform from base_link to map
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
    throw std::runtime_error("Failed to lookup transform.");
  }

  for (double t = 0; t < 12 * M_PI; t += M_PI / 4) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now;

    pose.pose.position.x = spiral_coeff * t * cos(t);
    pose.pose.position.y = spiral_coeff * t * sin(t);
    pose.pose.orientation.w = 1.0;

    // Transform pose to map frame
    tf2::doTransform(pose.pose, pose.pose, transform);

    path.poses.push_back(pose);
  }

  return path;
}
} // namespace urc_behaviors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(urc_behaviors::SearchForAruco)
