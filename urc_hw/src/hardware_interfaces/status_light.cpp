#include "urc_hw/hardware_interfaces/status_light.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <ostream>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <string>
#include <urc_nanopb/urc.pb.h>

PLUGINLIB_EXPORT_CLASS(
  urc_hardware::hardware_interfaces::StatusLight,
  hardware_interface::SystemInterface);

namespace urc_hardware::hardware_interfaces
{

StatusLight::StatusLight()
: hardware_interface_name("Status Light"), signals(2, 0) {}
StatusLight::~StatusLight() = default;

hardware_interface::CallbackReturn StatusLight::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("udp_address") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hardware_interface_name),
      "Error during initialization: 'udp_address' configuration not "
      "found. Expect to enter the udp server address.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.find("udp_port") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(hardware_interface_name),
      "Error during initialization: 'udp_port' configuration not "
      "found. Expect to enter the port number.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  udp_address = info_.hardware_parameters["udp_address"];
  udp_port = info_.hardware_parameters["udp_port"];

  if (info.gpios.size() == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        hardware_interface_name),
      "Should have at least one gpio to be the Status Light.");
    return CallbackReturn::ERROR;
  }

  bool find_light = false;
  for (hardware_interface::ComponentInfo component : info_.gpios) {
    if (component.name == "status_light") {
      find_light = true;
      break;
    }
  }

  if (!find_light) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        hardware_interface_name),
      "Not able to find sensor named 'status_light'.");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(hardware_interface_name),
    "Status light HW interface initialized!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StatusLight::on_configure(const rclcpp_lifecycle::State &)
{
  std::fill(signals.begin(), signals.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> StatusLight::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("status_light", "color", &this->signals[0]);
  command_interfaces.emplace_back("status_light", "state", &this->signals[1]);
  return command_interfaces;
}

std::vector<hardware_interface::StateInterface> StatusLight::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  return state_interfaces;
}

hardware_interface::CallbackReturn StatusLight::on_activate(const rclcpp_lifecycle::State &)
{
  udp_ = std::make_shared<UDPSocket<128>>(true);
  udp_->Connect(udp_address, std::stoi(udp_port));
  RCLCPP_INFO(rclcpp::get_logger(hardware_interface_name), "Status light activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StatusLight::on_deactivate(const rclcpp_lifecycle::State &)
{
  udp_->Close();
  RCLCPP_INFO(rclcpp::get_logger(hardware_interface_name), "Status light deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StatusLight::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  RCLCPP_INFO(
    rclcpp::get_logger(
      hardware_interface_name), "Red: %s, Green: %s, Blue: %s", num_to_state(
      lightStates[0]).c_str(), num_to_state(lightStates[1]).c_str(), num_to_state(
      lightStates[2]).c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StatusLight::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  TeensyMessage message = TeensyMessage_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  uint8_t selected_color = static_cast<uint8_t>(signals[0]);
  uint8_t selected_state = static_cast<uint8_t>(signals[1]);
  if (selected_color >= 0 && selected_color <= 2 && selected_state >= 0 && selected_state <= 2) {
    lightStates[selected_color] = selected_state;
  }

  // message.redEnabled = (lightStates[0] != 0);
  // message.redBlink = (lightStates[0] == 2);
  // message.greenEnabled = (lightStates[1] != 0);
  // message.greenBlink = (lightStates[1] == 2);
  // message.blueEnabled = (lightStates[2] != 0);
  // message.blueBlink = (lightStates[2] == 2);
  
  StatusLightMessage statusLight;
  statusLight.lightCommand = 0;
  statusLight.lightCommand |= (lightStates[0] != 0) << 0; // Red Enabled
  statusLight.lightCommand |= (lightStates[0] != 2) << 1; // Red Blinking
  statusLight.lightCommand |= (lightStates[1] != 0) << 2; // Greed Enabled
  statusLight.lightCommand |= (lightStates[1] != 2) << 3; // Green Blinking
  statusLight.lightCommand |= (lightStates[2] != 0) << 4; // Blue Enabled
  statusLight.lightCommand |= (lightStates[2] != 2) << 5; // Blue Blinking

  message.messageType.statusLightMessage = statusLight;

  // Fill Required Fields
  // message.m1Setpoint = 0;
  // message.m2Setpoint = 0;
  // message.m3Setpoint = 0;
  // message.m4Setpoint = 0;
  // message.m5Setpoint = 0;
  // message.m6Setpoint = 0;

  // Set message id to status light
  // message.messageID = 1;

  bool status = pb_encode(&stream, TeensyMessage_fields, &message);
  message_length = stream.bytes_written;

  if (!status) {
    return hardware_interface::return_type::ERROR;
  }
  udp_->Send((char *)buffer, sizeof(buffer));
  return hardware_interface::return_type::OK;
}

} // namespace urc_hardware::hardware_interfaces
