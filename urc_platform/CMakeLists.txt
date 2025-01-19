cmake_minimum_required(VERSION 3.5)
project(urc_platform)

include(../cmake/default_settings.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(usb_cam REQUIRED)
find_package(vectornav_msgs REQUIRED)

include_directories(
  include
)

# Library creation
add_library(${PROJECT_NAME} SHARED
  src/joystick_driver.cpp
  src/twist_mux.cpp
  src/sim_gps_handler.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_updater
  usb_cam
  vectornav_msgs
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Node registration
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "joystick_driver::JoystickDriver"
  EXECUTABLE ${PROJECT_NAME}_JoystickDriver
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "twist_mux::TwistMux"
  EXECUTABLE ${PROJECT_NAME}_TwistMux
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "sim_gps_handler::SimGpsHandler"
  EXECUTABLE ${PROJECT_NAME}_SimGpsHandler
)

# Install launch files.
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the copyright linker
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(msg)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
