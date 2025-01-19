#include "sim_gps_handler.hpp"
#include <rclcpp/logging.hpp>

namespace sim_gps_handler {
SimGpsHandler::SimGpsHandler(const rclcpp::NodeOptions &options)
    : Node("sim_gps_handler", options) {
  declare_parameter("gps_in_topic", "/gps/data_raw");
  declare_parameter("gps_out_topic", "/gps/data");

  gps_publisher = create_publisher<sensor_msgs::msg::NavSatFix>(
      get_parameter("gps_out_topic").as_string(), rclcpp::SystemDefaultsQoS());

  gps_subscriber = create_subscription<sensor_msgs::msg::NavSatFix>(
      get_parameter("gps_in_topic").as_string(), rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        msg->position_covariance[0] = 0.01;
        msg->position_covariance[4] = 0.01;
        msg->position_covariance[8] = 0.01;
        gps_publisher->publish(*msg);
      });
}
} // namespace sim_gps_handler

RCLCPP_COMPONENTS_REGISTER_NODE(sim_gps_handler::SimGpsHandler)
