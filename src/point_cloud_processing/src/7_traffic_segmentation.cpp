#include<chrono>
#include<memory>
#include<string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include<pcl/filters/passthrough.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class VoxelGrid_filter : public rclcpp::Node
{
  public:
    VoxelGrid_filter()
    : Node("kitti_data_voxel_node"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 10);
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bounding_boxes", 10);

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
        pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);
        // crop the cloud if needed
        pcl::PassThrough<PointT> passing_x;
        pcl::PassThrough<PointT> passing_y;
        int radius = 20;

        passing_x.setInputCloud(pcl_cloud);
        passing_x.setFilterFieldName("x");
        passing_x.setFilterLimits(-radius, radius);
        passing_x.filter(*cropped_cloud);

        // Along y axis
        passing_y.setInputCloud(cropped_cloud);
        passing_y.setFilterFieldName("y");
        passing_y.setFilterLimits(-radius, radius);
        passing_y.filter(*cropped_cloud);

        // Apply VoxelGrid filter
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud);
        voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // Set voxel size
        voxel_filter.filter(*voxel_cloud);

        // Road segmentation
        pcl::NormalEstimation<PointT, pcl::Normal> normal_extractor;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
        pcl::PointCloud<pcl::Normal>::Ptr road_normals (new pcl::PointCloud<pcl::Normal>);

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> road_seg_frm_normals;
        pcl::PointIndices::Ptr road_inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr road_coefficients (new pcl::ModelCoefficients);
        pcl::ExtractIndices<PointT> road_extract_indices;
        pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>);

        
        // Normal extractions
        normal_extractor.setSearchMethod(tree);
        normal_extractor.setInputCloud(voxel_cloud);
        normal_extractor.setKSearch(30);
        normal_extractor.compute(*road_normals);

        road_seg_frm_normals.setOptimizeCoefficients(true);
        road_seg_frm_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        road_seg_frm_normals.setMethodType(pcl::SAC_RANSAC);
        road_seg_frm_normals.setNormalDistanceWeight(0.5);
        road_seg_frm_normals.setMaxIterations(100); 
        road_seg_frm_normals.setDistanceThreshold(0.4);
        road_seg_frm_normals.setInputCloud(voxel_cloud);
        road_seg_frm_normals.setInputNormals(road_normals);
        road_seg_frm_normals.segment(*road_inliers, *road_coefficients);

        // Extraction
        road_extract_indices.setInputCloud(voxel_cloud);
        road_extract_indices.setIndices(road_inliers);
        road_extract_indices.setNegative(false);
        road_extract_indices.filter(*road_cloud);

        // ================= Traffic clustering ==================
        pcl::PointCloud<PointT>::Ptr single_segmented_cluster(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr all_clusters(new pcl::PointCloud<PointT>);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ecludian_cluster_extractor;
        size_t min_cloud_threshold = 110;
        size_t max_cloud_threshold = 5000;

        struct BBOX{
            float x_min;
            float x_max;
            float y_min;
            float y_max;
            float z_min;
            float z_max;
            double r=1.0;
            double g=0.0;
            double b=0.0;
        };

        std::vector<BBOX> bboxes;

        // Ecludian based clustering
        tree->setInputCloud(road_cloud);
        ecludian_cluster_extractor.setClusterTolerance(0.25);
        ecludian_cluster_extractor.setMinClusterSize(600);
        ecludian_cluster_extractor.setMaxClusterSize(5000);
        ecludian_cluster_extractor.setSearchMethod(tree);
        ecludian_cluster_extractor.setInputCloud(road_cloud);
        ecludian_cluster_extractor.extract(cluster_indices);

        for(size_t i = 0; i < cluster_indices.size(); ++i){
            if(cluster_indices[i].indices.size() > min_cloud_threshold && cluster_indices[i].indices.size() < max_cloud_threshold){ // Filter small clusters
                pcl::PointCloud<PointT>::Ptr reasonable_cluster(new pcl::PointCloud<PointT>);
                pcl::ExtractIndices<PointT> extract;
                pcl::IndicesPtr indices(
                  new std::vector<int>(cluster_indices[i].indices.begin(), 
                  cluster_indices[i].indices.end()));

                extract.setInputCloud(road_cloud);
                extract.setIndices(indices);
                extract.setNegative(false);
                extract.filter(*reasonable_cluster);

                // Bounding box drawing
                Eigen::Vector4f min_pt, max_pt;
                pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);
                
                pcl::PointXYZ min_point, max_point;
                pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2.0, (min_pt[1] + max_pt[1]) / 2.0, (min_pt[2] + max_pt[2]) / 2.0);
                BBOX bbox;
                bbox.x_min = min_pt[0];
                bbox.y_max = min_pt[1];
                bbox.z_min = min_pt[2];
                bbox.x_max = max_pt[0];
                bbox.y_max = max_pt[1];
                bbox.z_max = max_pt[2];
                bboxes.push_back(bbox);
            }
        }

        // =================== Draw bounding boxes ===================
        visualization_msgs::msg::MarkerArray marker_array;

        int id = 0;
        const std_msgs::msg::Header& inp_header = input_cloud->header;
        // Create a marker for each bounding box
        for (const auto& bbox : bboxes) {
            visualization_msgs::msg::Marker top_square_marker;
            top_square_marker.header = inp_header;
            top_square_marker.ns = "bounding_boxes";
            top_square_marker.id = id++;
            top_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            top_square_marker.action = visualization_msgs::msg::Marker::ADD;
            top_square_marker.pose.orientation.w = 1.0;
            top_square_marker.scale.x = 0.06; // Line width
            top_square_marker.color.r = bbox.r;
            top_square_marker.color.g = bbox.g;
            top_square_marker.color.b = bbox.b;
            top_square_marker.color.a = 1.0; // Set alpha to fully opaque

            // Add the points for the top square of the bounding box
            geometry_msgs::msg::Point p1, p2, p3, p4;
            p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
            p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
            p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
            p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
            top_square_marker.points.push_back(p1);
            top_square_marker.points.push_back(p2);
            top_square_marker.points.push_back(p3);
            top_square_marker.points.push_back(p4);
            top_square_marker.points.push_back(p1); // Close the square

            // Add the marker to the array
            marker_array.markers.push_back(top_square_marker);

            // Create the maker for the bottom square
            visualization_msgs::msg::Marker bottom_square_marker;
            bottom_square_marker.header = inp_header;
            bottom_square_marker.ns = "bounding_boxes";
            bottom_square_marker.id = id++;
            bottom_square_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            bottom_square_marker.action = visualization_msgs::msg::Marker::ADD;
            bottom_square_marker.pose.orientation.w = 1.0;
            bottom_square_marker.scale.x = 0.04; // Line width
            bottom_square_marker.color.r = bbox.r;
            bottom_square_marker.color.g = bbox.g;
            bottom_square_marker.color.b = bbox.b;
            bottom_square_marker.color.a = 1.0; // Set alpha to fully opaque

            // Add the points for the bottom square of the bounding box
            geometry_msgs::msg::Point p5, p6, p7, p8;
            p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
            p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
            p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
            p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;
            bottom_square_marker.points.push_back(p5);
            bottom_square_marker.points.push_back(p6);
            bottom_square_marker.points.push_back(p7);
            bottom_square_marker.points.push_back(p8);
            bottom_square_marker.points.push_back(p5); // Close the square
            // Add the marker to the array
            marker_array.markers.push_back(bottom_square_marker);

            // Create the marker for the lines connecting the top and bottom squares
            visualization_msgs::msg::Marker connecting_lines_marker;
            connecting_lines_marker.header = inp_header;
            connecting_lines_marker.ns = "bounding_boxes";
            connecting_lines_marker.id = id++;
            connecting_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            connecting_lines_marker.action = visualization_msgs::msg::Marker::ADD;
            connecting_lines_marker.pose.orientation.w = 1.0;
            connecting_lines_marker.scale.x = 0.04; // Line width
            connecting_lines_marker.color.r = 0.0;
            connecting_lines_marker.color.g = 1.0;
            connecting_lines_marker.color.b = 0.0;
            connecting_lines_marker.color.a = 1.0; // Set alpha to fully opaque

            // Add the points for the connecting lines
            connecting_lines_marker.points.push_back(p1);
            connecting_lines_marker.points.push_back(p5);

            connecting_lines_marker.points.push_back(p2);
            connecting_lines_marker.points.push_back(p6);

            connecting_lines_marker.points.push_back(p3);
            connecting_lines_marker.points.push_back(p7);

            connecting_lines_marker.points.push_back(p4);
            connecting_lines_marker.points.push_back(p8);

            // Add the marker to the array
            marker_array.markers.push_back(connecting_lines_marker);

            // Create a marker for the corners
            visualization_msgs::msg::Marker corner_marker;
            corner_marker.header = inp_header;
            corner_marker.ns = "bounding_boxes";
            corner_marker.id = id++;
            corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
            corner_marker.action = visualization_msgs::msg::Marker::ADD;
            corner_marker.pose.orientation.w = 1.0;
            corner_marker.scale.x = 0.4; // Sphere diameter
            corner_marker.scale.y = 0.4;
            corner_marker.scale.z = 0.4;
            corner_marker.color.r = bbox.r;
            corner_marker.color.g = 0.2;
            corner_marker.color.b = 0.5;
            corner_marker.color.a = 0.64; // Set alpha to fully opaque

            // Create a sphere for each corner and add it to the marker array
            corner_marker.pose.position = p1;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p2;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p3;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p4;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p5;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p6;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p7;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

            corner_marker.pose.position = p8;
            corner_marker.id = id++;
            marker_array.markers.push_back(corner_marker);

        }

        marker_publisher_->publish(marker_array);


        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 traffic_seg_ros2;
        pcl::toROSMsg(*road_cloud, traffic_seg_ros2);
        traffic_seg_ros2.header.frame_id = "base_link";
        traffic_seg_ros2.header.stamp = this->now();
       
        // Publish the segmented traffic cloud
        publisher_->publish(traffic_seg_ros2);
      return;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
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