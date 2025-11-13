#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

int main (){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCDReader cloud_reader;

    std::string path = "/home/jose/ros2_ws/src/point_cloud_processing/point_clouds/table_scene.pcd";
    cloud_reader.read(path, *cloud);
    std::cout << "PointCloud before filtering: " << cloud->width * cloud->height 
              << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;
    return 0;
}