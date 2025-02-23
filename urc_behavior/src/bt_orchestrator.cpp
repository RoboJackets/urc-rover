#include "urc_behavior/bt_orchestrator.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

#include <urc_msgs/srv/update_behavior_tree.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <thread>
#include <vector>
#include <exception>
#include <memory>

namespace urc_behavior
{

  BehaviorTreeOrchestrator::BehaviorTreeOrchestrator(const rclcpp::NodeOptions &options)
      : Node("bt_orchestrator", options)
  {
    this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    declare_parameter<std::vector<std::string>>("normal_node_lib_dirs");
    declare_parameter<std::vector<std::string>>("ros_node_lib_dirs");
    declare_parameter<std::string>("tree_file_dir");
    declare_parameter<int>("tick_rate");
    declare_parameter<bool>("start_bridge", true);

    bt_ros_nh_ = create_sub_node("ros_nh");
    bt_ros_logger_ = std::make_shared<rclcpp::Logger>(bt_ros_nh_->get_logger());

    if (has_parameter("normal_node_lib_dirs"))
    {
      std::vector<std::string> temp;
      get_parameter("normal_node_lib_dirs", temp);
      RCLCPP_INFO(this->get_logger(), "Loading %ld normal plugin%s All loaded plugins:", temp.size(), temp.size() > 1 ? "s." : ".");

      for (const auto &node_lib_dir : temp)
      {
        RCLCPP_INFO(this->get_logger(), "\t%s", node_lib_dir.c_str());
        tree_factory_.registerFromPlugin(node_lib_dir);
      }
    }

    if (has_parameter("ros_node_lib_dirs"))
    {
      std::vector<std::string> temp;
      get_parameter("ros_node_lib_dirs", temp);
      RCLCPP_INFO(this->get_logger(), "Loading %ld ROS plugin%s All loaded plugins:", temp.size(), temp.size() > 1 ? "s." : ".");

      BT::RosNodeParams params_;
      params_.nh = bt_ros_nh_;

      for (const auto &node_lib_dir : temp)
      {
        RCLCPP_INFO(this->get_logger(), "\t%s", node_lib_dir.c_str());
        RegisterRosNode(tree_factory_, node_lib_dir, params_);
      }
    }

    // Create a default tree if a file is specified, otherwise wait for the service to be called
    if (has_parameter("tree_file_dir"))
    {
      std::string temp;
      get_parameter("tree_file_dir", temp);
      tree_dir_ = std::make_shared<std::string>(temp);

      RCLCPP_INFO(this->get_logger(), "Creating tree from configuration file %s.", temp.c_str());
      tree_ = std::make_unique<BT::Tree>(tree_factory_.createTreeFromFile(temp));
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "The behavior tree file is not set! Cannot start the orchestrator until the /update_tree service is called.");
    }

    // Set the tick rate if specified, otherwise use the default rate
    if (has_parameter("tick_rate"))
    {
      tick_rate_ = get_parameter("tick_rate").as_int();
      RCLCPP_INFO(this->get_logger(), "Tick rate set to %d Hz.", tick_rate_);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No tick rate set, set to default rate of 100 Hz.");
    }

    // Start behavior tree services
    update_bt_service_ = create_service<urc_msgs::srv::UpdateBehaviorTree>(
        "~/update_tree",
        [this](const UpdateBTRequest request, UpdateBTResponse response)
        {
          response->success = renew_tree(request->use_dir, request->tree_dir, request->tree_content);
          return response;
        });

    reload_bt_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reload",
        [this](const TriggerRequest, TriggerResponse response)
        {
          if (tree_dir_ == nullptr)
          {
            RCLCPP_WARN(this->get_logger(), "Tree directory is null, cannot perform a reload.");
            response->success = false;
          }
          else
          {
            response->success = renew_tree(true, *tree_dir_, "");
          }
          return response;
        });

    start_bt_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_bt",
        [this](const TriggerRequest, TriggerResponse response)
        {
          if (!is_running_)
          {
            response->success = start();
          }
          else
          {
            response->message = "There is already a behavior tree running.";
            response->success = false;
          }
          return response;
        });

    stop_bt_service_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_bt",
        [this](TriggerRequest, TriggerResponse response)
        {
          if (is_running_)
          {
            response->success = stop();
          }
          else
          {
            response->message = "There is no behavior tree running.";
            response->success = false;
          }
          return response;
        });

    if (is_tree_loaded())
    {
      initialize();
      start();
    }
  }

  BehaviorTreeOrchestrator::~BehaviorTreeOrchestrator() { stop(); }

  bool BehaviorTreeOrchestrator::renew_tree(bool use_dir, std::string dir, std::string content)
  {
    std::unique_ptr<BT::Tree> new_tree_;

    try
    {
      if (use_dir)
      {
        new_tree_ = std::make_unique<BT::Tree>(tree_factory_.createTreeFromFile(dir));
      }
      else
      {
        new_tree_ = std::make_unique<BT::Tree>(tree_factory_.createTreeFromText(content));
      }

      stop();
      tree_ = std::move(new_tree_);
      RCLCPP_DEBUG(this->get_logger(), "Loaded new tree successfully, hotswapping...");

      initialize();
      start();
      RCLCPP_DEBUG(this->get_logger(), "Successfully hotswapped the tree!");
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load new tree: %s.", e.what());
      is_running_ = false;
      tree_.reset(nullptr);

      return false;
    }

    return true;
  }

  void BehaviorTreeOrchestrator::initialize()
  {
    if (!get_parameter("start_bridge").get_parameter_value().to_value_msg().bool_value)
    {
      return;
    }

    // Register current node to tree factory
    tree_->rootBlackboard()->set("ros_nh", bt_ros_nh_);
    tree_->rootBlackboard()->set("ros_log", bt_ros_logger_);
  }

  bool BehaviorTreeOrchestrator::start()
  {
    if (!is_tree_loaded())
    {
      RCLCPP_ERROR(this->get_logger(), "No tree is loaded! Cannot start the behavior tree orchestrator.");
      return false;
    }

    is_running_ = true;

    std::thread(
        [this]()
        {
          RCLCPP_DEBUG(this->get_logger(), "Starting behavior tree orchestrator...");
          rclcpp::Rate rate(tick_rate_);

          while (is_running_)
          {
            tree_->tickExactlyOnce();
            rate.sleep();
          }
        })
        .detach();

    return true;
  }

  bool BehaviorTreeOrchestrator::stop()
  {
    is_running_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Stopping behavior tree orchestrator...");

    return true;
  }

  bool BehaviorTreeOrchestrator::is_tree_loaded() { return tree_ != nullptr; }
} // namespace behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urc_behavior::BehaviorTreeOrchestrator);
