#ifndef FOLLOWER_ACTION_SERVER_HPP_
#define FOLLOWER_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "urc_msgs/action/follow_path.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace follower_action_server
{
class FollowerActionServer : public rclcpp::Node
{
public:
  explicit FollowerActionServer(const rclcpp::NodeOptions & options);

  ~FollowerActionServer();

private:
  geometry_msgs::msg::TransformStamped lookup_transform(
    std::string target_frame,
    std::string source_frame);

  visualization_msgs::msg::Marker create_lookahead_circle(
    double x, double y, double radius,
    std::string frame_id);

  void publishZeroVelocity();

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const urc_msgs::action::FollowPath::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<urc_msgs::action::FollowPath>> goal_handle);

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<urc_msgs::action::FollowPath>> goal_handle);

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<urc_msgs::action::FollowPath>> goal_handle);

  /**
   * @brief Handle the costmap data
   * @param msg The costmap data
   */
  void handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Publish the plan to the /path topic for *visualization* purposes. The plan will be returned as a response to the service call.
   * @param plan The plan to be published
   */
  void publishPlan(const nav_msgs::msg::Path & plan);

  int getCost(const nav_msgs::msg::OccupancyGrid & costmap, double x, double y);

  nav_msgs::msg::OccupancyGrid current_costmap_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp_action::Server<urc_msgs::action::FollowPath>::SharedPtr follow_path_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  geometry_msgs::msg::PoseStamped current_pose_;
  bool stamped_;
};
} // namespace follower_action_server

#endif // FOLLOWER_ACTION_SERVER_HPP_
