#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/passthrough.h>
#include<filesystem>
#include<iostream>


int main(){
    std::string pcd_file_name = "plane_segmented.pcd";
    std::filesystem::path ros2_ws_path = std::filesystem::current_path(); 
    std::filesystem::path point_cloud_dir = ros2_ws_path/"src/point_cloud_processing/point_clouds/";
    std::string pcd_file_path = point_cloud_dir/pcd_file_name;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud);
    pcl::visualization::PCLVisualizer viewer ("Point cloud with path");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "Groudn Plane");

    float ball_radius = 0.15f;
    float ball_x = 0.0f;
    float ball_y = 0.0f;

    for(int i=0; i<5; i++){
        pcl::PointXYZ point;
        point.x = ball_x;
        point.y = ball_y;
        point.z = 0.0f;
        viewer.addSphere(point, ball_radius, 0.0, 1.0, 0.0, "Sphere" + std::to_string(i));
        ball_x = ball_radius+ 1.5f;
    }


    viewer.spin();
    return 0;
}