cmake_minimum_required(VERSION 3.5)
project(urc_behaviors)

include(../cmake/default_settings.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(urc_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/search_for_aruco.cpp
)

set(dependencies
  rclcpp
  rclcpp_components
  urc_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "urc_behaviors::SearchForAruco"
  EXECUTABLE ${PROJECT_NAME}_SearchForAruco
)

# Install launch files.
install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(msg)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
