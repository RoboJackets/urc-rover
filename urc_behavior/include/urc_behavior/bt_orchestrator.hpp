#ifndef BT_ORCHESTRATOR_HPP__
#define BT_ORCHESTRATOR_HPP__

#include "behaviortree_cpp/bt_factory.h"
#include "std_srvs/std_srvs/srv/detail/trigger__struct.hpp"

#include <rclcpp/rclcpp.hpp>
#include <urc_msgs/srv/detail/update_behavior_tree__struct.hpp>
#include <urc_msgs/srv/update_behavior_tree.hpp>

#include <memory>
#include <string>

namespace urc_behavior
{
  using TriggerRequest = std_srvs::srv::Trigger::Request::ConstSharedPtr;
  using TriggerResponse = std_srvs::srv::Trigger::Response::SharedPtr;
  using UpdateBTRequest = urc_msgs::srv::UpdateBehaviorTree::Request::ConstSharedPtr;
  using UpdateBTResponse = urc_msgs::srv::UpdateBehaviorTree::Response::SharedPtr;

  class BehaviorTreeOrchestrator : public rclcpp::Node
  {
  public:
    BehaviorTreeOrchestrator(const rclcpp::NodeOptions &options);
    ~BehaviorTreeOrchestrator();

  protected:
    BT::BehaviorTreeFactory tree_factory_;
    std::unique_ptr<BT::Tree> tree_;

    void initialize();
    bool renew_tree(bool use_dir, std::string dir, std::string content);
    bool start();
    bool stop();
    bool is_tree_loaded();

    bool is_running_;
    int tick_rate_ = 100;

    std::shared_ptr<std::string> tree_dir_ = nullptr;
    std::unique_ptr<rclcpp::Logger> logger_;
    std::shared_ptr<rclcpp::Node> bt_ros_nh_;
    std::shared_ptr<rclcpp::Logger> bt_ros_logger_;

    rclcpp::Service<urc_msgs::srv::UpdateBehaviorTree>::SharedPtr update_bt_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_bt_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_bt_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_bt_service_;
  };

} // namespace urc_behavior

#endif // BT_ORCHESTRATOR_HPP__
