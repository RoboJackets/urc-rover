#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "planner_server.hpp"
#include "astar.hpp"

namespace planner_server
{
PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("planner_server", options)
{
  RCLCPP_INFO(get_logger(), "Planner server started.");

  // Create the service
  plan_service_ = create_service<urc_msgs::srv::GeneratePlan>(
    "plan",
    std::bind(&PlannerServer::generatePlan, this, std::placeholders::_1, std::placeholders::_2));

  // Setup the costmap
  costmap_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&PlannerServer::handleCostmap, this, std::placeholders::_1));
}

void PlannerServer::handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "Received Costmap!");
  current_costmap_ = *msg;
}

PlannerServer::~PlannerServer() {}

void PlannerServer::generatePlan(
  const std::shared_ptr<urc_msgs::srv::GeneratePlan::Request> request,
  std::shared_ptr<urc_msgs::srv::GeneratePlan::Response> response)
{
  try {
    astar::AStar astar;
    astar.setMap(current_costmap_);

    auto start = request->start.pose;
    auto goal = request->goal.pose;

    astar.createPlan(start, goal);
    std::vector<astar::AStarNode> path = astar.getPath();

    std::vector<geometry_msgs::msg::PoseStamped> poses;

    for (auto & node : path) {
      geometry_msgs::msg::PoseStamped pose;

      pose.header.frame_id = "map";
      pose.header.stamp = get_clock()->now();
      pose.pose = node.getPose();

      poses.push_back(pose);
    }

    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    plan.header.stamp = get_clock()->now();
    plan.poses = poses;

    response->path = plan;
    response->error_code = urc_msgs::srv::GeneratePlan::Response::SUCCESS;

    RCLCPP_INFO(get_logger(), "Returning plan with %ld poses", plan.poses.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Error generating plan: %s", e.what());
    response->error_code = urc_msgs::srv::GeneratePlan::Response::FAILURE;
  }
}
} // namespace planner_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planner_server::PlannerServer)
