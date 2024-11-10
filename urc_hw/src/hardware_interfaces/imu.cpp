// #include "urc_hw/hardware_interfaces/imu.hpp"
// #include "async_sockets/udpsocket.hpp"
// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/sensor_interface.hpp"
// #include "pluginlib/class_list_macros.hpp"
// #include <async_sockets/udpserver.hpp>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <string>
// #include <unistd.h>
// #include <urc_nanopb/urc.pb.h>

// PLUGINLIB_EXPORT_CLASS(urc_hardware::hardware_interfaces::IMU, hardware_interface::SensorInterface);

// namespace urc_hardware::hardware_interfaces
// {

// IMU::IMU()
// : hardware_interface_name("IMU") {}
// IMU::~IMU() = default;

// hardware_interface::CallbackReturn IMU::on_init(const hardware_interface::HardwareInfo & info)
// {
//   if (hardware_interface::SensorInterface::on_init(info) !=
//     hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   if (info_.hardware_parameters.find("udp_address") == info_.hardware_parameters.end()) {
//     RCLCPP_ERROR(
//       rclcpp::get_logger(hardware_interface_name),
//       "Error during initialization: 'udp_address' configuration not "
//       "found. Expect to enter the udp server address.");
//     return hardware_interface::CallbackReturn::ERROR;
//   }
//   if (info_.hardware_parameters.find("udp_port") == info_.hardware_parameters.end()) {
//     RCLCPP_ERROR(
//       rclcpp::get_logger(hardware_interface_name),
//       "Error during initialization: 'udp_port' configuration not "
//       "found. Expect to enter the port number.");
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   if (info_.hardware_parameters.find("udp_self_address") == info_.hardware_parameters.end()) {
//     RCLCPP_ERROR(
//       rclcpp::get_logger(hardware_interface_name),
//       "Error during initialization: 'udp_self_address' configuration not "
//       "found. Expect to enter the udp server address.");
//     return hardware_interface::CallbackReturn::ERROR;
//   }
//   if (info_.hardware_parameters.find("udp_self_port") == info_.hardware_parameters.end()) {
//     RCLCPP_ERROR(
//       rclcpp::get_logger(hardware_interface_name),
//       "Error during initialization: 'udp_self_port' configuration not "
//       "found. Expect to enter the port number.");
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   udp_address = info_.hardware_parameters["udp_address"];
//   udp_port = info_.hardware_parameters["udp_port"];
//   udp_self_address = info_.hardware_parameters["udp_self_address"];
//   udp_self_port = info_.hardware_parameters["udp_self_port"];

//   if (info.sensors.size() == 0) {
//     RCLCPP_ERROR(
//       rclcpp::get_logger(
//         hardware_interface_name), "Should have at least one sensor to be the imu.");
//     return CallbackReturn::ERROR;
//   }

//   bool find_imu = false;
//   for (hardware_interface::ComponentInfo component : info_.sensors) {
//     if (component.name == "imu_sensor") {
//       find_imu = true;
//       break;
//     }
//   }

//   if (!find_imu) {
//     RCLCPP_ERROR(
//       rclcpp::get_logger(
//         hardware_interface_name), "Not able to find sensor named 'imu_sensor'.");
//     return CallbackReturn::ERROR;
//   }

//   quaternions.resize(4);
//   linear_accelerations.resize(3);
//   angular_velocities.resize(3);
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn IMU::on_configure(const rclcpp_lifecycle::State &)
// {
//   std::fill(quaternions.begin(), quaternions.end(), 0.0);
//   std::fill(linear_accelerations.begin(), linear_accelerations.end(), 0.0);
//   std::fill(angular_velocities.begin(), angular_velocities.end(), 0.0);
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> IMU::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   state_interfaces.emplace_back("imu_sensor", "orientation.x", &this->quaternions[0]);
//   state_interfaces.emplace_back("imu_sensor", "orientation.y", &this->quaternions[1]);
//   state_interfaces.emplace_back("imu_sensor", "orientation.z", &this->quaternions[2]);
//   state_interfaces.emplace_back("imu_sensor", "orientation.w", &this->quaternions[3]);
//   state_interfaces.emplace_back(
//     "imu_sensor", "linear_acceleration.x",
//     &this->linear_accelerations[0]);
//   state_interfaces.emplace_back(
//     "imu_sensor", "linear_acceleration.y",
//     &this->linear_accelerations[1]);
//   state_interfaces.emplace_back(
//     "imu_sensor", "linear_acceleration.z",
//     &this->linear_accelerations[2]);
//   state_interfaces.emplace_back("imu_sensor", "angular_velocity.x", &this->angular_velocities[0]);
//   state_interfaces.emplace_back("imu_sensor", "angular_velocity.y", &this->angular_velocities[1]);
//   state_interfaces.emplace_back("imu_sensor", "angular_velocity.z", &this->angular_velocities[2]);

//   return state_interfaces;
// }

// hardware_interface::CallbackReturn IMU::on_activate(const rclcpp_lifecycle::State &)
// {
//   udp_ = std::make_shared<UDPServer<128>>();
//   udp_->Bind(udp_self_address.c_str(), std::stoi(udp_self_port));
//   udp_->onRawMessageReceived = [this](const char * buf, ssize_t size, std::string, std::uint16_t) {
//       /* Allocate space for the decoded message. */
//       IMUMessage message = IMUMessage_init_zero;

//       /* Create a stream that reads from the buffer. */
//       pb_istream_t stream = pb_istream_from_buffer((unsigned char *)buf, size);

//       /* Now we are ready to decode the message. */
//       bool status = pb_decode(&stream, IMUMessage_fields, &message);

//       /* Check for errors... */
//       if (!status) {
//         printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
//         return 1;
//       }

//       this->quaternions[0] = message.quaternionX;
//       this->quaternions[1] = message.quaternionY;
//       this->quaternions[2] = message.quaternionZ;
//       this->quaternions[3] = message.quaternionW;
//       this->linear_accelerations[0] = message.linearAccelX;
//       this->linear_accelerations[1] = message.linearAccelY;
//       this->linear_accelerations[2] = message.linearAccelZ;
//       this->angular_velocities[0] = message.angularVelocityX;
//       this->angular_velocities[1] = message.angularVelocityY;
//       this->angular_velocities[2] = message.angularVelocityZ;

//       return 0;
//     };
//   RCLCPP_INFO(rclcpp::get_logger(hardware_interface_name), "IMU activated!");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn IMU::on_deactivate(const rclcpp_lifecycle::State &)
// {
//   udp_->Close();
//   RCLCPP_INFO(rclcpp::get_logger(hardware_interface_name), "IMU deactivated!");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type IMU::read(const rclcpp::Time &, const rclcpp::Duration &)
// {
//   return hardware_interface::return_type::OK;
// }

// }  // namespace urc_hardware::hardware_interfaces
