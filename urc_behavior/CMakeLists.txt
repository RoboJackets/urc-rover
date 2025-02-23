cmake_minimum_required(VERSION 3.8)
project(urc_behavior)

include(../cmake/default_settings.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urc_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urc_bt_nodes REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  pluginlib
  rclcpp_components
  urc_msgs
  std_srvs
  urc_bt_nodes
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/bt_orchestrator.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

# Node registration
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "urc_behavior::BehaviorTreeOrchestrator"
  EXECUTABLE ${PROJECT_NAME}_BehaviorTreeOrchestrator
)

# Install launch files.
install(
  DIRECTORY
  launch
  strategies
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
