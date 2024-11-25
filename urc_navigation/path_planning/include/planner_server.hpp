#ifndef PLANNER_SERVER_HPP_
#define PLANNER_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "urc_msgs/srv/generate_plan.hpp"

namespace planner_server
{
class PlannerServer : public rclcpp::Node
{
public:
  explicit PlannerServer(const rclcpp::NodeOptions & options);

  ~PlannerServer();

protected:
  /**
   * @brief Call the A* algorithm to generate the plan
   */
  void generatePlan(
    const std::shared_ptr<urc_msgs::srv::GeneratePlan::Request> request,
    std::shared_ptr<urc_msgs::srv::GeneratePlan::Response> response);

  /**
   * @brief Wait for the costmap to be available
   */
  void waitForCostmap();

  /**
   * @brief Handle the costmap data
   * @param msg The costmap data
   */
  void handleCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  nav_msgs::msg::OccupancyGrid current_costmap_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Service<urc_msgs::srv::GeneratePlan>::SharedPtr plan_service_;
};
} // namespace planner_server

#endif // PLANNER_SERVER_HPP_
