#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

class TestScanToPointCloud : public ::testing::Test, public rclcpp::Node
{
public:
  TestScanToPointCloud()
  : rclcpp::Node("test_scan_to_pointcloud"),
    tfBuffer_(this->get_clock()),
    tfListener_(tfBuffer_)
  {
    get_parameter("min_dist", min_dist);
    get_parameter("neighbor_dist", neighbor_dist);

    mock_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/pc2", rclcpp::SystemDefaultsQoS());

    mock_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scan", rclcpp::SystemDefaultsQoS(), [this](sensor_msgs::msg::PointCloud2 input) {
        sub_data = input;
      });
  }

  sensor_msgs::msg::PointCloud2 createPointCloudMsg()
  {
    sensor_msgs::msg::PointCloud2 input_cloud;
    input_cloud.header.frame_id = "/lidar";
    input_cloud.data = {1, 0, 0};
    sensor_msgs::msg::PointCloud2 output_cloud;

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tfBuffer_.lookupTransform("scan", "lidar", rclcpp::Time(0));
      tf2::doTransform(input_cloud, output_cloud, transform_stamped);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Transformation Error:\n%s", ex.what());
    }

    return output_cloud;
  }

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mock_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mock_sub;
  sensor_msgs::msg::PointCloud2 sub_data;

  double min_dist;
  double neighbor_dist;

private:
  void SetUp()
  {
  }

  void TearDown()
  {
  }

  void TestBody()
  {
  }

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
};

TEST_F(TestScanToPointCloud, ParameterTest)
{
  ASSERT_TRUE(min_dist == 0.1);
  ASSERT_TRUE(neighbor_dist == 0.2);
}

TEST_F(TestScanToPointCloud, ComparisonTest)
{
  auto cloudCheck = [](std::vector<uint8_t> msg_data, std::vector<uint8_t> test_data) {
      for (size_t i = 0; i < msg_data.size(); ++i) 
      {
        EXPECT_NEAR(msg_data[i], test_data[i], 0.001);
      }
    };

  mock_pub->publish(createPointCloudMsg());

  sensor_msgs::msg::PointCloud2 published_cloud_msg;
  try {
    published_cloud_msg = sub_data;
  } catch (const std::exception &) {
    ASSERT_TRUE(false);
  }

  std::vector<uint8_t> msg_data = {published_cloud_msg.data};


  //this is obviously wrong, unsigned can't be negative
  std::vector<uint8_t> test_data = {static_cast<uint8_t>(-1.4142136), 0, 1};
  cloudCheck(msg_data, test_data);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestScanToPointCloud>());
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
