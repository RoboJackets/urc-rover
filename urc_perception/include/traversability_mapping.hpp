#ifndef TRAVERSABILITY_MAPPING_HPP_
#define TRAVERSABILITY_MAPPING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <filters/filter_chain.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace urc_perception
{

  class TraversabilityMapping : public rclcpp::Node
  {
  public:
    explicit TraversabilityMapping(const rclcpp::NodeOptions &options);
    ~TraversabilityMapping();

    bool readParameters();

  private:
    void handlePointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filterSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    grid_map::GridMap map_;
    filters::FilterChain<grid_map::GridMap> filter_chain_;

    double resolution_;
    double min_z_;
    double max_z_;
    unsigned int width_;

    std::string map_frame_;
    std::string pointcloud_topic_;
    std::string output_map_topic_;
    std::string filter_chain_parameter_name_;
  };

} // namespace urc_perception

#endif // TRAVERSABILITY_MAPPING_HPP_
