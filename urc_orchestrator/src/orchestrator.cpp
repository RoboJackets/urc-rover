#include "orchestrator.hpp"

namespace orchestrator
{
Orchestrator::Orchestrator(const rclcpp::NodeOptions & options)
: rclcpp::Node("orchestrator", options)
{
  this->purePursuitEnabled = 1;
  this->maxDelta = 1.0;
  this->actualLongitude = -1;
  this->actualLatitude = -1;
  this->baseLatitude = -1;
  this->baseLongitude = -1;
  this->waypointLatitude = -1;
  this->waypointLongitude = -1;
  this->initialYaw = -1;
  this->currentYaw = -1;
  this->gpsTimestamp = this->get_clock()->now();
  current_metric_pose.position.x = -1;
  current_metric_pose.position.y = -1;
  current_metric_pose.position.z = -1;

  current_state_publisher = create_publisher<urc_msgs::msg::NavigationStatus>(
    "/current_navigation_state", rclcpp::SystemDefaultsQoS());
  cmd_vel_publisher = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/rover_drivetrain_controller/cmd_vel", rclcpp::SystemDefaultsQoS());

  metric_offset_pose_publisher = create_publisher<geometry_msgs::msg::Pose>(
    "/metric_pose_offset", rclcpp::SystemDefaultsQoS());
  costmap_offset_pose_publisher = create_publisher<geometry_msgs::msg::Pose>(
    "/costmap_pose", rclcpp::SystemDefaultsQoS());

  metric_offset_pose_publisher = create_publisher<geometry_msgs::msg::Pose>(
    "/pose/metric", rclcpp::SystemDefaultsQoS());
  costmap_offset_pose_publisher = create_publisher<geometry_msgs::msg::Pose>(
    "/pose/costmap", rclcpp::SystemDefaultsQoS());

  imu_subscriber = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::Imu msg) {IMUCallback(msg);});
  set_base_subscriber = create_subscription<std_msgs::msg::Bool>(
    "/set_base", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool msg) {SetBaseCallback(msg);});
  waypoint_subscriber = create_subscription<urc_msgs::msg::Waypoint>(
    "/waypoint", rclcpp::SystemDefaultsQoS(),
    [this](const urc_msgs::msg::Waypoint msg) {WaypointCallback(msg);});
  gps_subscriber = create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::NavSatFix msg) {GPSCallback(msg);});
}

void Orchestrator::GPSCallback(const sensor_msgs::msg::NavSatFix & msg)
{
  this->actualLatitude = msg.latitude;
  this->actualLongitude = msg.longitude;
  if (this->baseLatitude == -1) {
    this->baseLatitude = this->actualLatitude;
    this->baseLongitude = this->actualLongitude;
  }
  if (this->actualLatitude != -1 && this->actualLongitude != -1 &&
    this->waypointLatitude != -1 && this->waypointLongitude != -1)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing Poses");
    PublishMetricPose(
      this->actualLatitude - this->baseLatitude,
      this->actualLongitude - this->baseLongitude);
    PublishCostmapPose(
      this->actualLatitude - this->baseLatitude,
      this->actualLongitude - this->baseLongitude);
  }
  DetermineState();
}

void Orchestrator::IMUCallback(const sensor_msgs::msg::Imu & msg)
{
  this->current_metric_pose.orientation.x = msg.orientation.x;
  this->current_metric_pose.orientation.y = msg.orientation.y;
  this->current_metric_pose.orientation.z = msg.orientation.z;
  this->current_metric_pose.orientation.w = msg.orientation.w;
  double x = msg.orientation.x;
  double y = msg.orientation.y;
  double z = msg.orientation.z;
  double w = msg.orientation.w;
  double yaw = atan2(2 * (w * x + y * z), 1 - 2 * (pow(x, 2) + pow(y, 2)));
  if (this->initialYaw == -1) {
    this->initialYaw = yaw;
  }
  this->currentYaw = yaw;
}

void Orchestrator::SetBaseCallback(const std_msgs::msg::Bool & msg)
{
  this->baseLatitude = this->actualLatitude;
  this->baseLongitude = this->actualLongitude;
}

void Orchestrator::WaypointCallback(const urc_msgs::msg::Waypoint & msg)
{
  this->waypointLatitude = msg.latitude;
  this->waypointLongitude = msg.longitude;
  DetermineState();
}

void Orchestrator::PublishMetricPose(double gpsOffsetX, double gpsOffsetY)
{
  this->current_metric_pose.position.x = gpsOffsetX * 111139;
  this->current_metric_pose.position.y = gpsOffsetY * 111139;
  metric_offset_pose_publisher->publish(current_metric_pose);
}

void Orchestrator::PublishCostmapPose(double gpsOffsetX, double gpsOffsetY)
{
  this->current_costmap_pose.position.x = floor((gpsOffsetX * 111139) * 4 + 50);
  this->current_costmap_pose.position.y = floor((gpsOffsetY * 111139) * 4 + 50);
  costmap_offset_pose_publisher->publish(current_costmap_pose);
}

void Orchestrator::DetermineState()
{
  this->gpsTimestamp = this->get_clock()->now();

  urc_msgs::msg::NavigationStatus state_message;
  if (actualLatitude == -1 || actualLongitude == -1) {
    state_message.message = "NoGPS";
    current_state_publisher->publish(state_message);
    return;
  } else if (waypointLatitude == -1 || waypointLongitude == -1) {
    state_message.message = "NoWaypoint";
    current_state_publisher->publish(state_message);
    return;
  }

  double deltaX = waypointLongitude - actualLongitude;
  double deltaY = waypointLatitude - actualLatitude;
  double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  if (distance < 0.00001) {
    state_message.message = "Goal";
    this->waypointLatitude = -1;
    this->waypointLongitude = -1;
  } else {
    state_message.message = "Navigating";
  }
  current_state_publisher->publish(state_message);
  if (this->purePursuitEnabled &&
    state_message.message !=
    "Goal")         // IMPORTANT: only for very basic testing
  {
    PurePursuit(deltaX, deltaY);
  }
  return;
}

void Orchestrator::PurePursuit(double deltaX, double deltaY)
{
  // Publishing of Command Velocities.
  double errorDThreshold = 0.00001;
  double errorZThreshold = 0.05;

  geometry_msgs::msg::TwistStamped cmd_vel;
  double errorD = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
  // double currentAngle = currentYaw - initialYaw;
  double currentAngle = this->currentYaw - this->initialYaw;
  currentAngle *= 100;

  RCLCPP_INFO(this->get_logger(), "BASE ANGLE");
  RCLCPP_INFO(this->get_logger(), "%f", this->initialYaw);
  RCLCPP_INFO(this->get_logger(), "CURRENT ANGLE");
  RCLCPP_INFO(this->get_logger(), "%f", currentAngle);
  RCLCPP_INFO(this->get_logger(), "TARGET ANGLE");
  RCLCPP_INFO(this->get_logger(), "%f", (atan2(deltaY, deltaX) - M_PI / 2));
  double errorZ = currentAngle - (atan2(deltaY, deltaX) - M_PI / 2);
  RCLCPP_INFO(this->get_logger(), "ERRORZ");
  RCLCPP_INFO(this->get_logger(), "%f", errorZ);
  if (abs(errorZ) >= errorZThreshold) {
    RCLCPP_INFO(this->get_logger(), "CORRECTING ORIENTATION");
    double thing = 1;
    if (errorZ < 0) {
      thing = -1;
    }
    cmd_vel.twist.angular.z =
      1 * thing;   // Will probably need to multiply by some constant.
  } else if (errorD >= errorDThreshold) {
    cmd_vel.twist.linear.x =
      0.5;   // Will probably need to multiply by some constant.
  }

  cmd_vel_publisher->publish(cmd_vel);
  // sleep(15);
  // RCLCPP_INFO(this->get_logger(), "15 seconds");
  // if ((this->get_clock()->now() - this->gpsTimestamp).seconds() > 15) {
  //   RCLCPP_INFO(this->get_logger(), "True");
  // }
}

} // namespace orchestrator

RCLCPP_COMPONENTS_REGISTER_NODE(orchestrator::Orchestrator)
