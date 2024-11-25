#include "follower_action_server.hpp"
#include "geometry_util.hpp"
#include "pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace follower_action_server
{
FollowerActionServer::FollowerActionServer(const rclcpp::NodeOptions & options)
: Node("follower_action_server", options)
{
  RCLCPP_INFO(this->get_logger(), "Follower node has been started.");

  declare_parameter("lookahead_distance", 1.0);
  declare_parameter("desired_linear_velocity", 0.5);
  declare_parameter("cmd_vel_topic", "/cmd_vel");
  declare_parameter("odom_topic", "/odom");
  declare_parameter("map_frame", "map");
  declare_parameter("goal_tolerance", 0.1);
  declare_parameter("cmd_vel_stamped", false);
  declare_parameter("lethal_cost_threshold", 50);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  stamped_ = get_parameter("cmd_vel_stamped").as_bool();

  if (stamped_) {
    cmd_vel_stamped_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(
      get_parameter(
        "cmd_vel_topic")
      .as_string(),
      10);
  } else {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter(
        "cmd_vel_topic")
      .as_string(),
      10);
  }

  carrot_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("carrot", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("lookahead_circle", 10);

  // Create the publisher
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>(
    "/path",
    rclcpp::SystemDefaultsQoS());

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(),
    10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose = msg->pose.pose;
      current_pose_ = pose;
    });

  // Setup the costmap
  costmap_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&FollowerActionServer::handleCostmap, this, std::placeholders::_1));

  // Create an action server for the follow_path action
  follow_path_server_ = rclcpp_action::create_server<urc_msgs::action::FollowPath>(
    this,
    "follow_path",
    std::bind(
      &FollowerActionServer::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&FollowerActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&FollowerActionServer::handle_accepted, this, std::placeholders::_1));
}

void FollowerActionServer::handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_costmap_ = *msg;
}

geometry_msgs::msg::TransformStamped FollowerActionServer::lookup_transform(
  std::string target_frame, std::string source_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not lookup transform: %s", ex.what());
  }
  return transform;
}

FollowerActionServer::~FollowerActionServer()
{
  RCLCPP_INFO(this->get_logger(), "Follower action server has been stopped.");
}

rclcpp_action::GoalResponse FollowerActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const urc_msgs::action::FollowPath::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");

  (void)uuid;
  (void)goal;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowerActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<urc_msgs::action::FollowPath>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowerActionServer::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<urc_msgs::action::FollowPath>> goal_handle)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&FollowerActionServer::execute, this, std::placeholders::_1),
    goal_handle}
  .detach();
}

visualization_msgs::msg::Marker FollowerActionServer::create_lookahead_circle(
  double x, double y,
  double radius,
  std::string frame_id)
{
  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = frame_id;
  circle.header.stamp = get_clock()->now();
  uint32_t shape = visualization_msgs::msg::Marker::CYLINDER;

  circle.ns = "basic_shapes";
  circle.id = 0;
  circle.type = shape;
  circle.action = visualization_msgs::msg::Marker::ADD;

  circle.pose.position.x = x;
  circle.pose.position.y = y;
  circle.pose.position.z = 0.0;
  circle.pose.orientation.x = 0.0;
  circle.pose.orientation.y = 0.0;
  circle.pose.orientation.z = 0.0;
  circle.pose.orientation.w = 1.0;

  circle.scale.x = 2 * radius;
  circle.scale.y = 2 * radius;
  circle.scale.z = 0.01;

  circle.color.r = 0.0f;
  circle.color.g = 0.0f;
  circle.color.b = 1.0f;
  circle.color.a = 0.3;

  return circle;
}

void FollowerActionServer::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = get_clock()->now();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  if (stamped_) {
    cmd_vel_stamped_pub_->publish(cmd_vel);
  } else {
    cmd_vel_pub_->publish(cmd_vel.twist);
  }
}

int FollowerActionServer::getCost(const nav_msgs::msg::OccupancyGrid & costmap, double x, double y)
{
  int map_x = (x - costmap.info.origin.position.x) / costmap.info.resolution;
  int map_y = (y - costmap.info.origin.position.y) / costmap.info.resolution;

  if (map_x < 0 || map_x >= costmap.info.width || map_y < 0 || map_y >= costmap.info.height) {
    return 0;
  }

  return costmap.data[map_y * costmap.info.width + map_x];
}

void FollowerActionServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<urc_msgs::action::FollowPath>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto feedback = std::make_shared<urc_msgs::action::FollowPath::Feedback>();
  feedback->distance_to_goal = std::numeric_limits<double>::max();

  auto result = std::make_shared<urc_msgs::action::FollowPath::Result>();
  auto & path = goal_handle->get_goal()->path;

  publishPlan(path);

  // Create a PurePursuit object
  pure_pursuit::PurePursuitParams params;
  params.lookahead_distance = get_parameter("lookahead_distance").as_double();
  params.desired_linear_velocity = get_parameter("desired_linear_velocity").as_double();
  pure_pursuit::PurePursuit pure_pursuit(params);

  pure_pursuit.setPath(path);

  pure_pursuit::PurePursuitOutput output;
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal has been canceled");
      break;
    } else if (feedback->distance_to_goal < get_parameter("goal_tolerance").as_double()) {
      result->error_code = urc_msgs::action::FollowPath::Result::SUCCESS;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal has been reached!");
      break;
    } else if (getCost(
        current_costmap_, output.lookahead_point.point.x,
        output.lookahead_point.point.y) > get_parameter("lethal_cost_threshold").as_int())
    {
      result->error_code = urc_msgs::action::FollowPath::Result::OBSTACLE_DETECTED;
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Obstacle detected!");
      break;
    }

    output =
      pure_pursuit.getCommandVelocity(
      lookup_transform(
        "base_link",
        get_parameter("map_frame").as_string()));

    auto odom_to_map_ = lookup_transform(get_parameter("map_frame").as_string(), "odom");

    if (stamped_) {
      cmd_vel_stamped_pub_->publish(output.cmd_vel);
    } else {
      cmd_vel_pub_->publish(output.cmd_vel.twist);
    }

    geometry_msgs::msg::PoseStamped current_pose_map_frame_;
    tf2::doTransform(current_pose_, current_pose_map_frame_, odom_to_map_);

    auto circle =
      create_lookahead_circle(
      current_pose_map_frame_.pose.position.x, current_pose_map_frame_.pose.position.y,
      params.lookahead_distance, get_parameter("map_frame").as_string());
    marker_pub_->publish(circle);

    // Publish the carrot point
    carrot_pub_->publish(output.lookahead_point);

    // Publish feedback
    feedback->distance_to_goal =
      geometry_util::dist2D(current_pose_map_frame_.pose.position, path.poses.back().pose.position);
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  publishZeroVelocity();
}

void FollowerActionServer::publishPlan(const nav_msgs::msg::Path & plan)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(plan);
  if (plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

} // namespace follower_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(follower_action_server::FollowerActionServer)
