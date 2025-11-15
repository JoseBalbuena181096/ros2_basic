#include<chrono>
#include<memory>
#include<string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class VoxelGrid_filter : public rclcpp::Node
{
  public:
    VoxelGrid_filter()
    : Node("kitti_data_voxel_node"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 10);

      suscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "kitti/point_cloud", 
        10,
        std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1)
      );

    }

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);
        // Apply VoxelGrid filter
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // Set voxel size
        voxel_filter.filter(*voxel_cloud);
        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 voxel_cloud_ros2;
        pcl::toROSMsg(*voxel_cloud, voxel_cloud_ros2);
        voxel_cloud_ros2.header.frame_id = "base_link";
        voxel_cloud_ros2.header.stamp = this->now();
        std::cout << "Received point cloud with " << pcl_cloud->size() << " points." << std::endl;
        std::cout << "Publishing voxelized cloud with " << voxel_cloud->size() << " points." << std::endl;


        publisher_->publish(voxel_cloud_ros2);


      return;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr suscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGrid_filter>());
  rclcpp::shutdown();
  return 0;
}