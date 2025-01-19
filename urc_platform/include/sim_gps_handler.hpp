#ifndef SIM_GPS_HANDLER
#define SIM_GPS_HANDLER

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace sim_gps_handler {
class SimGpsHandler : public rclcpp::Node {
public:
  explicit SimGpsHandler(const rclcpp::NodeOptions &options);

private:
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> gps_publisher;
};
} // namespace sim_gps_handler

#endif
